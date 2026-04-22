#ifndef FOC_TRANSFORMS_H
#define FOC_TRANSFORMS_H

/* Additional constants used by Clarke / inverse-Clarke. */
#define FOC_ONE_OVER_SQRT3  0.57735026919f   /* 1/√3  */
#define FOC_SQRT3_OVER_2    0.86602540378f   /* √3/2  */

/* -------------------------------------------------------------------------
 * Clarke transform: 3-phase (UVW) → stationary 2-phase (αβ)
 *
 * Amplitude-invariant form (2/3 scaling): peak αβ magnitudes equal the
 * peak phase magnitudes.
 * ---------------------------------------------------------------------- */

/* Full form — makes no assumption about the phase sum. */
static inline void FOC_Clarke(float i_u, float i_v, float i_w,
                               float *i_alpha, float *i_beta)
{
    *i_alpha = (2.0f / 3.0f) * (i_u - 0.5f * i_v - 0.5f * i_w);
    *i_beta  = FOC_ONE_OVER_SQRT3 * (i_v - i_w);
}

/* Two-measurement form — assumes i_u + i_v + i_w = 0 (Kirchhoff).
   Only two phase currents need to be sampled; i_w is reconstructed
   implicitly. */
static inline void FOC_Clarke2(float i_u, float i_v,
                                float *i_alpha, float *i_beta)
{
    *i_alpha = i_u;
    *i_beta  = FOC_ONE_OVER_SQRT3 * (i_u + 2.0f * i_v);
}

/* -------------------------------------------------------------------------
 * Inverse Clarke: stationary 2-phase (αβ) → 3-phase (UVW)
 * ---------------------------------------------------------------------- */
static inline void FOC_InvClarke(float v_alpha, float v_beta,
                                  float *v_u, float *v_v, float *v_w)
{
    *v_u =  v_alpha;
    *v_v = -0.5f * v_alpha + FOC_SQRT3_OVER_2 * v_beta;
    *v_w = -0.5f * v_alpha - FOC_SQRT3_OVER_2 * v_beta;
}

/* -------------------------------------------------------------------------
 * Park transform: stationary (αβ) → rotating (dq)
 *
 * Accepts pre-computed sin/cos so a single FOC_Math_SinCos call per
 * control step is shared across all transforms that use the same angle.
 * ---------------------------------------------------------------------- */
static inline void FOC_Park(float i_alpha, float i_beta,
                             float sin_th, float cos_th,
                             float *i_d, float *i_q)
{
    *i_d =  i_alpha * cos_th + i_beta * sin_th;
    *i_q = -i_alpha * sin_th + i_beta * cos_th;
}

/* -------------------------------------------------------------------------
 * Inverse Park transform: rotating (dq) → stationary (αβ)
 * ---------------------------------------------------------------------- */
static inline void FOC_InvPark(float v_d, float v_q,
                                float sin_th, float cos_th,
                                float *v_alpha, float *v_beta)
{
    *v_alpha = v_d * cos_th - v_q * sin_th;
    *v_beta  = v_d * sin_th + v_q * cos_th;
}

#endif /* FOC_TRANSFORMS_H */
