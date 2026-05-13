#ifndef FOC_MATH_H
#define FOC_MATH_H

#include <stdint.h>

/* -------------------------------------------------------------------------
 * Math constants
 * ---------------------------------------------------------------------- */
#define FOC_PI              3.14159265358979323846f
#define FOC_TWO_PI          6.28318530717958647692f
#define FOC_PI_2            1.57079632679489661923f

#define FOC_SQRT3           1.73205080756887729353f   /* √3       */
#define FOC_SQRT3_OVER_2    0.86602540378443864676f   /* √3/2     */
#define FOC_ONE_OVER_SQRT3  0.57735026918962576451f   /* 1/√3     */

/* -------------------------------------------------------------------------
 * General-purpose scalar utilities (static inline — no call overhead)
 * ---------------------------------------------------------------------- */

static inline float FOC_Min(float a, float b)
{
    return (a < b) ? a : b;
}

static inline float FOC_Max(float a, float b)
{
    return (a > b) ? a : b;
}

/* Clamp val to [lo, hi]. */
static inline float FOC_Clamp(float val, float lo, float hi)
{
    return FOC_Max(lo, FOC_Min(val, hi));
}

static inline float FOC_Abs(float val)
{
    return (val < 0.0f) ? -val : val;
}

/* Returns -1.0, 0.0, or +1.0. */
static inline float FOC_Sign(float val)
{
    if (val > 0.0f) return  1.0f;
    if (val < 0.0f) return -1.0f;
    return 0.0f;
}

/* Squaring shorthand; avoids repeated multiply expressions. */
static inline float FOC_Sq(float val)
{
    return val * val;
}

/*
 * Zero signals within ±band and shift the remainder toward zero, giving a
 * continuous (non-jumping) output.
 *   |val| <= band  → 0
 *   |val|  > band  → val - sign(val)*band
 */
static inline float FOC_Deadband(float val, float band)
{
    if (val >  band) return val - band;
    if (val < -band) return val + band;
    return 0.0f;
}

/* Wrap angle to (−π, π].  Used for shortest-path error in position control. */
static inline float FOC_WrapToPi(float angle)
{
    while (angle >  FOC_PI)  angle -= FOC_TWO_PI;
    while (angle <= -FOC_PI) angle += FOC_TWO_PI;
    return angle;
}

/* Linear interpolation: returns a + t*(b − a), t ∈ [0, 1]. */
static inline float FOC_Lerp(float a, float b, float t)
{
    return a + t * (b - a);
}

/* -------------------------------------------------------------------------
 * Trig — fast sin/cos via a 512-entry full-wave LUT + linear interpolation.
 *
 * The LUT is a const array placed in flash by the linker — no RAM cost,
 * no init call required.
 * ---------------------------------------------------------------------- */

/* Preferred form — computes both in one table walk. */
void  FOC_Math_SinCos(float angle_rad, float *sin_out, float *cos_out);

float FOC_Math_Sin(float angle_rad);
float FOC_Math_Cos(float angle_rad);

#endif /* FOC_MATH_H */
