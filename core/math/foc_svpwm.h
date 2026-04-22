#ifndef FOC_SVPWM_H
#define FOC_SVPWM_H

#include "foc_transforms.h"

/*
 * Space Vector PWM: map (v_alpha, v_beta) to three duty cycles [0, 1].
 *
 * Uses the min-max zero-sequence injection method: the three-phase references
 * are first reconstructed via inverse-Clarke, then a common-mode voltage equal
 * to -(max + min)/2 is added to all three.  This centers the references
 * symmetrically within the DC bus and is mathematically equivalent to
 * sector-based SVPWM, producing the same harmonic spectrum without requiring
 * sector detection.
 *
 * Duty-to-voltage convention: v_x = (duty_x - 0.5) * v_bus
 *   duty = 0.0  → −v_bus/2  (full low)
 *   duty = 0.5  →  0 V      (midpoint)
 *   duty = 1.0  → +v_bus/2  (full high)
 *
 * Inputs must satisfy |V_ref| ≤ v_bus/√3 to stay within the linear
 * modulation range.  References beyond this are clamped.
 */
static inline void FOC_SVPWM(float v_alpha, float v_beta, float v_bus,
                               float duty_max, uint8_t pwm_active_low,
                               float *duty_u, float *duty_v, float *duty_w)
{
    /* Step 1: inverse Clarke — reconstruct three-phase voltage references. */
    float v_u, v_v, v_w;
    FOC_InvClarke(v_alpha, v_beta, &v_u, &v_v, &v_w);

    /* Step 2: zero-sequence injection — shift all three by the midpoint of
       max and min so the active vectors are centered in the PWM period. */
    float v_max = FOC_Max(FOC_Max(v_u, v_v), v_w);
    float v_min = FOC_Min(FOC_Min(v_u, v_v), v_w);
    float v_cm  = -0.5f * (v_max + v_min);

    v_u += v_cm;
    v_v += v_cm;
    v_w += v_cm;

    /* Step 3: normalise to [0, 1] and clamp against overmodulation. */
    float inv_vbus = 1.0f / v_bus;
    *duty_u = FOC_Clamp(0.5f + v_u * inv_vbus, 0.0f, duty_max);
    *duty_v = FOC_Clamp(0.5f + v_v * inv_vbus, 0.0f, duty_max);
    *duty_w = FOC_Clamp(0.5f + v_w * inv_vbus, 0.0f, duty_max);

    /* Step 4: invert if the MCU PWM output is active-low. */
    if (pwm_active_low) {
        *duty_u = 1.0f - *duty_u;
        *duty_v = 1.0f - *duty_v;
        *duty_w = 1.0f - *duty_w;
    }
}

#endif /* FOC_SVPWM_H */
