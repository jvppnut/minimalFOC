#include "core/estimator/foc_estimator.h"
#include "core/math/foc_math.h"

static float   s_theta_mech  = 0.0f;
static float   s_theta_prev  = 0.0f;
static float   s_omega_filt  = 0.0f;
static float   s_lpf_alpha   = 0.0f;
static float   s_inv_ts      = 0.0f;
static uint8_t s_initialized = 0u;

void FOC_Estimator_Init(float lpf_alpha, float Ts)
{
    s_lpf_alpha = lpf_alpha;
    s_inv_ts    = 1.0f / Ts;
    FOC_Estimator_Reset();
}

void FOC_Estimator_Reset(void)
{
    s_theta_mech  = 0.0f;
    s_theta_prev  = 0.0f;
    s_omega_filt  = 0.0f;
    s_initialized = 0u;
}

void FOC_Estimator_Update(FOC_Motor_t *motor)
{
    float raw = motor->state.theta_mech_raw;

    /* On the first call there is no previous sample — seed state and
       output zero velocity to avoid a spurious spike. */
    if (!s_initialized) {
        s_theta_prev  = raw;
        s_initialized = 1u;
        motor->state.theta_mech = s_theta_mech;
        motor->state.omega_mech = 0.0f;
        return;
    }

    /* Unwrap: clamp delta to (-π, π] to handle single-turn wrap-around.
       Valid as long as the true per-sample displacement is less than π rad,
       which holds for any realistic speed at 20 kHz. */
    float delta = raw - s_theta_prev;
    if (delta >  FOC_PI) delta -= FOC_TWO_PI;
    if (delta < -FOC_PI) delta += FOC_TWO_PI;

    s_theta_mech += delta;
    s_theta_prev  = raw;

    /* Velocity: backward difference then first-order IIR LPF. */
    float omega_raw = delta * s_inv_ts;
    s_omega_filt    = s_lpf_alpha       * s_omega_filt
                    + (1.0f - s_lpf_alpha) * omega_raw;

    motor->state.theta_mech = s_theta_mech;
    motor->state.omega_mech = s_omega_filt;
}
