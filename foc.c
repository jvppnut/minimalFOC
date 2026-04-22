#include "foc.h"
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

void FOC_Step(FOC_Motor_t *motor)
{
    FOC_MotorState_t  *s  = &motor->state;
    FOC_MotorRef_t    *r  = &motor->ref;
    FOC_MotorOut_t    *o  = &motor->out;
    FOC_HWConfig_t    *hw = &motor->hw;
    FOC_MotorParams_t *p  = &motor->params;

    /* --- Electrical angle ------------------------------------------------ */
    s->theta_elec = (s->theta_mech - hw->theta_mech_offset) * (float)p->pole_pairs
                    + hw->theta_elec_offset;

    float sin_th, cos_th;
    FOC_Math_SinCos(s->theta_elec, &sin_th, &cos_th);

    /* --- Current feedback ------------------------------------------------ */
    FOC_Clarke(s->i_u, s->i_v, s->i_w, &s->i_alpha, &s->i_beta);
    FOC_Park(s->i_alpha, s->i_beta, sin_th, cos_th, &s->i_d, &s->i_q);

    /* --- Control cascade ------------------------------------------------- */
    float v_d, v_q;

    switch (r->mode) {

        case FOC_MODE_VOLTAGE:
            v_d = r->v_d_ref;
            v_q = r->v_q_ref;
            break;

        case FOC_MODE_TORQUE:
            v_d = FOC_PID_Update(&foc_pid_id, r->i_d_ref - s->i_d, 0.0f);
            v_q = FOC_PID_Update(&foc_pid_iq, r->i_q_ref - s->i_q, 0.0f);
            break;

        case FOC_MODE_VELOCITY:
            r->i_q_ref = FOC_PID_Update(&foc_pid_speed,
                                         r->omega_ref - s->omega_mech,
                                         0.0f /* angular accel — to be provided by estimator */);
            v_d = FOC_PID_Update(&foc_pid_id, r->i_d_ref - s->i_d, 0.0f);
            v_q = FOC_PID_Update(&foc_pid_iq, r->i_q_ref - s->i_q, 0.0f);
            break;

        case FOC_MODE_POSITION:
            r->omega_ref = FOC_PID_Update(&foc_pid_pos,
                                           r->theta_ref - s->theta_mech,
                                           s->omega_mech);
            r->i_q_ref   = FOC_PID_Update(&foc_pid_speed,
                                           r->omega_ref - s->omega_mech,
                                           0.0f /* angular accel — to be provided by estimator */);
            v_d = FOC_PID_Update(&foc_pid_id, r->i_d_ref - s->i_d, 0.0f);
            v_q = FOC_PID_Update(&foc_pid_iq, r->i_q_ref - s->i_q, 0.0f);
            break;

        default:
            v_d = 0.0f;
            v_q = 0.0f;
            break;
    }

    /* --- Output ---------------------------------------------------------- */
    o->v_d = v_d;
    o->v_q = v_q;

    FOC_InvPark(v_d, v_q, sin_th, cos_th, &o->v_alpha, &o->v_beta);

    FOC_SVPWM(o->v_alpha, o->v_beta, s->v_bus,
              hw->duty_max, hw->pwm_active_low,
              &o->duty_u, &o->duty_v, &o->duty_w);
}
