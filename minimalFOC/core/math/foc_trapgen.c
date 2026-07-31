#include <math.h>
#include "foc_trapgen.h"

void FOC_TrapGen_Reset(FOC_TrapGen_t *gen)
{
    gen->initialized = 0u;
}

void FOC_TrapGen_Seed(FOC_TrapGen_t *gen, float pos0)
{
    gen->pos             = pos0;
    gen->vel             = 0.0f;
    gen->p0              = pos0;
    gen->p1              = pos0;
    gen->dir             = 0.0f;
    gen->v_cruise        = 0.0f;
    gen->d_acc           = 0.0f;
    gen->t_acc           = 0.0f;
    gen->t_cruise        = 0.0f;
    gen->t_total         = 0.0f;
    gen->elapsed         = 0.0f;
    gen->last_target_raw = pos0;
    gen->initialized     = 1u;
}

void FOC_TrapGen_Start(FOC_TrapGen_t *gen, float target_raw, float v_max, float a_max)
{
    gen->last_target_raw = target_raw;

    /* Absolute (unwrapped) displacement — target_raw is a genuine target in
     * the same frame as gen->pos, not a shortest-path angle mod 2*pi. See
     * foc_trapgen.h. */
    float delta = target_raw - gen->pos;
    gen->p0  = gen->pos;
    gen->p1  = gen->pos + delta;
    gen->dir = FOC_Sign(delta);

    float L = FOC_Abs(delta);
    float d_acc_full = (v_max * v_max) / (2.0f * a_max);

    if (2.0f * d_acc_full <= L) {
        /* Full trapezoid — reaches v_max and holds it for a cruise phase. */
        gen->v_cruise = v_max;
        gen->t_acc    = v_max / a_max;
        gen->d_acc    = d_acc_full;
        gen->t_cruise = (L - 2.0f * d_acc_full) / v_max;
    } else {
        /* Move too short to reach v_max — triangular profile. */
        gen->v_cruise = sqrtf(a_max * L);
        gen->t_acc    = gen->v_cruise / a_max;
        gen->d_acc    = 0.5f * a_max * gen->t_acc * gen->t_acc;
        gen->t_cruise = 0.0f;
    }

    gen->a_max   = a_max;
    gen->t_total = 2.0f * gen->t_acc + gen->t_cruise;
    gen->elapsed = 0.0f;
}

float FOC_TrapGen_Update(FOC_TrapGen_t *gen, float Ts)
{
    gen->elapsed += Ts;
    float t = gen->elapsed;

    float pos, vel;
    if (t >= gen->t_total) {
        pos = gen->p1;
        vel = 0.0f;
    } else if (t < gen->t_acc) {
        pos = gen->p0 + gen->dir * 0.5f * gen->a_max * t * t;
        vel = gen->dir * gen->a_max * t;
    } else if (t < gen->t_acc + gen->t_cruise) {
        float t_cr = t - gen->t_acc;
        pos = gen->p0 + gen->dir * (gen->d_acc + gen->v_cruise * t_cr);
        vel = gen->dir * gen->v_cruise;
    } else {
        float t_dec = t - gen->t_acc - gen->t_cruise;
        pos = gen->p0 + gen->dir * (gen->d_acc + gen->v_cruise * gen->t_cruise
                                     + gen->v_cruise * t_dec - 0.5f * gen->a_max * t_dec * t_dec);
        vel = gen->dir * (gen->v_cruise - gen->a_max * t_dec);
    }

    gen->pos = pos;
    gen->vel = vel;
    return pos;
}
