#include "foc_math.h"
#include <math.h>
#include <stdint.h>

/* Full-wave table matching ARM CMSIS sizing.  Must be a power of 2 so that
   the index mask trick works and the cos offset (TABLE_SIZE/4) is exact. */
#define TABLE_SIZE          512u
#define TABLE_MASK          (TABLE_SIZE - 1u)
#define TABLE_QUARTER       (TABLE_SIZE / 4u)   /* index offset equivalent to π/2 rotation */

/* 1/(2*PI) — same constant ARM CMSIS uses to normalize radians to [0,1). */
#define INV_TWO_PI  0.159154943092f

static float sin_table[TABLE_SIZE + 1];

void FOC_Math_InitTrig(void)
{
    for (unsigned i = 0; i <= TABLE_SIZE; i++) {
        sin_table[i] = sinf((float)i * (FOC_TWO_PI / (float)TABLE_SIZE));
    }
}

/* Linear interpolation into sin_table at fractional index fidx ∈ [0, TABLE_SIZE). */
static float trig_lut(float fidx)
{
    uint16_t idx  = (uint16_t)fidx & TABLE_MASK;
    float    frac = fidx - (float)idx;
    return sin_table[idx] + frac * (sin_table[idx + 1] - sin_table[idx]);
}

/* Map any angle in radians to a fractional table index in [0, TABLE_SIZE).
   Mirrors the ARM CMSIS floor-toward-negative-infinity approach. */
static float angle_to_fidx(float angle_rad)
{
    float   in = angle_rad * INV_TWO_PI;
    int32_t n  = (int32_t)in;
    if (angle_rad < 0.0f) n--;             /* floor toward -infinity */
    in = in - (float)n;                    /* fractional part in [0, 1) */
    float fidx = in * (float)TABLE_SIZE;
    if (fidx >= (float)TABLE_SIZE) fidx -= (float)TABLE_SIZE;  /* fp edge case */
    return fidx;
}

void FOC_Math_SinCos(float angle_rad, float *sin_out, float *cos_out)
{
    float fidx_s = angle_to_fidx(angle_rad);
    float fidx_c = fidx_s + (float)TABLE_QUARTER;   /* cos(x) = sin(x + π/2) */
    if (fidx_c >= (float)TABLE_SIZE) fidx_c -= (float)TABLE_SIZE;

    *sin_out = trig_lut(fidx_s);
    *cos_out = trig_lut(fidx_c);
}

float FOC_Math_Sin(float angle_rad)
{
    return trig_lut(angle_to_fidx(angle_rad));
}

float FOC_Math_Cos(float angle_rad)
{
    /* Offset by a quarter rotation (π/2) so cos(x) = sin(x + π/2). */
    float fidx = angle_to_fidx(angle_rad) + (float)TABLE_QUARTER;
    if (fidx >= (float)TABLE_SIZE) fidx -= (float)TABLE_SIZE;
    return trig_lut(fidx);
}
