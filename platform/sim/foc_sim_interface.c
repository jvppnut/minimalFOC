#include "foc_sim_interface.h"
#include "foc.h"

static FOC_Motor_t sim_motor;

void FOC_Sim_Init(void)
{
    FOC_Math_InitTrig();
    FOC_Init();
}

void FOC_Sim_SetMotorParams(float Rs, float Ld, float Lq, float lambda_pm,
                             uint8_t pole_pairs, float J, float Dm,
                             float rated_current, float rated_speed)
{
    sim_motor.params.Rs            = Rs;
    sim_motor.params.Ld            = Ld;
    sim_motor.params.Lq            = Lq;
    sim_motor.params.lambda_pm     = lambda_pm;
    sim_motor.params.pole_pairs    = pole_pairs;
    sim_motor.params.J             = J;
    sim_motor.params.Dm            = Dm;
    sim_motor.params.rated_current = rated_current;
    sim_motor.params.rated_speed   = rated_speed;
}

void FOC_Sim_SetHWConfig(float Ts, float dead_time, float v_bus_nominal,
                          float duty_max, uint8_t pwm_active_low,
                          float theta_elec_offset)
{
    sim_motor.hw.Ts                = Ts;
    sim_motor.hw.dead_time         = dead_time;
    sim_motor.hw.v_bus_nominal     = v_bus_nominal;
    sim_motor.hw.duty_max          = duty_max;
    sim_motor.hw.pwm_active_low    = pwm_active_low;
    sim_motor.hw.theta_elec_offset = theta_elec_offset;
}

void FOC_Sim_SetPIDGains_Id(float Kp, float Ki, float Kd, float out_min, float out_max)
{
    FOC_PID_Init(&foc_pid_id, Kp, Ki, Kd, out_min, out_max);
}

void FOC_Sim_SetPIDGains_Iq(float Kp, float Ki, float Kd, float out_min, float out_max)
{
    FOC_PID_Init(&foc_pid_iq, Kp, Ki, Kd, out_min, out_max);
}

void FOC_Sim_SetPIDGains_Speed(float Kp, float Ki, float Kd, float out_min, float out_max)
{
    FOC_PID_Init(&foc_pid_speed, Kp, Ki, Kd, out_min, out_max);
}

void FOC_Sim_SetPIDGains_Pos(float Kp, float Ki, float Kd, float out_min, float out_max)
{
    FOC_PID_Init(&foc_pid_pos, Kp, Ki, Kd, out_min, out_max);
}

void FOC_Sim_SetMode(uint8_t mode)
{
    sim_motor.ref.mode = (FOC_CtrlMode_t)mode;
}

void FOC_Sim_SetRef(float v_d_ref, float v_q_ref, float i_d_ref,
                    float i_q_ref, float omega_ref, float theta_ref)
{
    sim_motor.ref.v_d_ref   = v_d_ref;
    sim_motor.ref.v_q_ref   = v_q_ref;
    sim_motor.ref.i_d_ref   = i_d_ref;
    sim_motor.ref.i_q_ref   = i_q_ref;
    sim_motor.ref.omega_ref = omega_ref;
    sim_motor.ref.theta_ref = theta_ref;
}

void FOC_Sim_Reset(void)
{
    FOC_Reset();
}

void FOC_Sim_Calibrate(float v_cal, float settle_time_s)
{
    FOC_Calibrate(&sim_motor, v_cal, settle_time_s);
}

uint8_t FOC_Sim_GetMode(void)
{
    return (uint8_t)sim_motor.ref.mode;
}

void FOC_Sim_Step(float i_u, float i_v, float i_w,
                  float theta_mech_raw, float theta_mech,
                  float omega_mech, float v_bus,
                  float *duty_u, float *duty_v, float *duty_w)
{
    sim_motor.state.i_u            = i_u;
    sim_motor.state.i_v            = i_v;
    sim_motor.state.i_w            = i_w;
    sim_motor.state.theta_mech_raw = theta_mech_raw;
    sim_motor.state.theta_mech     = theta_mech;
    sim_motor.state.omega_mech     = omega_mech;
    sim_motor.state.v_bus          = v_bus;

    FOC_Step(&sim_motor);

    *duty_u = sim_motor.out.duty_u;
    *duty_v = sim_motor.out.duty_v;
    *duty_w = sim_motor.out.duty_w;
}

void FOC_Sim_GetInternals(float *theta_elec,
                           float *i_alpha, float *i_beta,
                           float *i_d,     float *i_q,
                           float *v_d,     float *v_q,
                           float *v_alpha, float *v_beta)
{
    if (theta_elec) *theta_elec = sim_motor.state.theta_elec;
    if (i_alpha)    *i_alpha    = sim_motor.state.i_alpha;
    if (i_beta)     *i_beta     = sim_motor.state.i_beta;
    if (i_d)        *i_d        = sim_motor.state.i_d;
    if (i_q)        *i_q        = sim_motor.state.i_q;
    if (v_d)        *v_d        = sim_motor.out.v_d;
    if (v_q)        *v_q        = sim_motor.out.v_q;
    if (v_alpha)    *v_alpha    = sim_motor.out.v_alpha;
    if (v_beta)     *v_beta     = sim_motor.out.v_beta;
}
