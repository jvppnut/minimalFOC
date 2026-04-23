#include "foc.h"
#include "core/math/foc_math.h"
#include "core/math/foc_svpwm.h"

/* -------------------------------------------------------------------------
 * Controller instances — single definitions; extern declarations in foc.h.
 * ------------------------------------------------------------------------- */
FOC_PID_t foc_pid_id;
FOC_PID_t foc_pid_iq;
FOC_PID_t foc_pid_speed;
FOC_PID_t foc_pid_pos;

/* ------------------------------------------------------------------------- */

void FOC_Init(void)
{
    FOC_Reset();
}

void FOC_Reset(void)
{
    FOC_PID_Reset(&foc_pid_id);
    FOC_PID_Reset(&foc_pid_iq);
    FOC_PID_Reset(&foc_pid_speed);
    FOC_PID_Reset(&foc_pid_pos);
}

void FOC_Calibrate(FOC_Motor_t *motor)
{
    /* TODO: drive v_d_ref to a known value in FOC_MODE_VOLTAGE, wait for
       rotor settle, read theta_mech, compute and store
       motor->hw.theta_elec_offset. */
    (void)motor;
}

/* -------------------------------------------------------------------------
 * Sub-step implementations
 * ------------------------------------------------------------------------- */

void FOC_CurrentCtrlComputation(FOC_Motor_t *motor)
{
    FOC_MotorState_t  *s = &motor->state;
    FOC_MotorRef_t    *r = &motor->ref;
    FOC_MotorOut_t    *o = &motor->out;
    FOC_MotorParams_t *p = &motor->params;

    float omega_e = s->omega_mech * (float)p->pole_pairs;
    float v_lim   = s->v_bus * FOC_ONE_OVER_SQRT3;

    /* PI feedback on each axis. */
    float vd_fb = FOC_PID_Update(&foc_pid_id, r->i_d_ref - s->i_d, 0.0f);
    float vq_fb = FOC_PID_Update(&foc_pid_iq, r->i_q_ref - s->i_q, 0.0f);

    /* Cross-coupling and back-EMF feedforward (PMSM voltage equations):
     *   v_d includes  -omega_e * Lq * i_q          (q-to-d coupling)
     *   v_q includes  +omega_e * (Ld*i_d + lpm)    (d-to-q coupling + back-EMF)
     * Decouples the axes so each PI drives a simple first-order R-L plant. */
    float vd_ff = -omega_e * p->Lq * s->i_q;
    float vq_ff =  omega_e * (p->Ld * s->i_d + p->lambda_pm);

    o->v_d = FOC_Clamp(vd_fb + vd_ff, -v_lim, v_lim);
    o->v_q = FOC_Clamp(vq_fb + vq_ff, -v_lim, v_lim);
}

void FOC_VelocityCtrlComputation(FOC_Motor_t *motor)
{
    FOC_MotorState_t *s = &motor->state;
    FOC_MotorRef_t   *r = &motor->ref;

    r->i_q_ref = FOC_PID_Update(&foc_pid_speed,
                                 r->omega_ref - s->omega_mech,
                                 0.0f /* angular accel — to be provided by estimator */);
}

void FOC_PositionCtrlComputation(FOC_Motor_t *motor)
{
    FOC_MotorState_t *s = &motor->state;
    FOC_MotorRef_t   *r = &motor->ref;

    /* Shortest-path position error, wrapped to (−π, π]. */
    float pos_err = FOC_WrapToPi(r->theta_ref - s->theta_mech);

    /* PD: output = Kp*err - Kd*omega_mech.
       FOC_PID_Update applies the negative sign on meas_dot internally. */
    r->i_q_ref = FOC_PID_Update(&foc_pid_pos, pos_err, s->omega_mech);
}

/* -------------------------------------------------------------------------
 * Top-level step
 * ------------------------------------------------------------------------- */

void FOC_Step(FOC_Motor_t *motor)
{
    FOC_MotorState_t *s  = &motor->state;
    FOC_MotorRef_t   *r  = &motor->ref;
    FOC_MotorOut_t   *o  = &motor->out;
    FOC_HWConfig_t   *hw = &motor->hw;
    FOC_MotorParams_t *p = &motor->params;

    /* --- Electrical angle ------------------------------------------------ */
    s->theta_elec = (s->theta_mech - hw->theta_mech_offset) * (float)p->pole_pairs
                    + hw->theta_elec_offset;

    float sin_th, cos_th;
    FOC_Math_SinCos(s->theta_elec, &sin_th, &cos_th);

    /* --- Current feedback ------------------------------------------------ */
    FOC_Clarke(s->i_u, s->i_v, s->i_w, &s->i_alpha, &s->i_beta);
    FOC_Park(s->i_alpha, s->i_beta, sin_th, cos_th, &s->i_d, &s->i_q);

    /* --- Control cascade ------------------------------------------------- */
    switch (r->mode) {

        case FOC_MODE_VOLTAGE:
            o->v_d = r->v_d_ref;
            o->v_q = r->v_q_ref;
            break;

        case FOC_MODE_TORQUE:
            FOC_CurrentCtrlComputation(motor);
            break;

        case FOC_MODE_VELOCITY:
            FOC_VelocityCtrlComputation(motor);
            FOC_CurrentCtrlComputation(motor);
            break;

        case FOC_MODE_POSITION:
            FOC_PositionCtrlComputation(motor);
            FOC_CurrentCtrlComputation(motor);
            break;

        default:
            o->v_d = 0.0f;
            o->v_q = 0.0f;
            break;
    }

    /* --- Output ---------------------------------------------------------- */
    FOC_InvPark(o->v_d, o->v_q, sin_th, cos_th, &o->v_alpha, &o->v_beta);

    FOC_SVPWM(o->v_alpha, o->v_beta, s->v_bus,
              hw->duty_max, hw->pwm_active_low,
              &o->duty_u, &o->duty_v, &o->duty_w);
}
