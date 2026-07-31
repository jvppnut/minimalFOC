#ifndef FOC_TRAPGEN_H
#define FOC_TRAPGEN_H

#include <stdint.h>
#include "foc_math.h"

/*
 * Trapezoidal (triangular-fallback) position reference generator.
 *
 * Shapes a raw, instantaneously-set target position into a smooth
 * position/velocity trajectory: ramp up to a max velocity at a max
 * acceleration, cruise, ramp back down to zero velocity exactly at the
 * target. Falls back to a triangular profile (never reaches v_max) if the
 * move is too short to justify a cruise phase.
 *
 * Target changes are planned as an absolute (unwrapped) displacement from
 * the generator's current position — target_raw is a genuine target in the
 * same frame as the running position, not a shortest-path angle mod 2*pi.
 * This matters once there's a reduction ratio downstream of the motor: a
 * large target (many motor turns for one output-shaft turn) needs to be
 * tracked in full, not collapsed to its within-one-turn residue. If a
 * shortest-path angle mode is ever needed for some other application, wrap
 * target_raw with FOC_WrapToPi() (relative to the current position)
 * *before* calling FOC_TrapGen_Start(), rather than baking that assumption
 * into this module.
 *
 * Re-targeting mid-move always replans from the profile's own current
 * (pos) output, assuming zero velocity at the start of the new plan. This
 * keeps the *position* reference continuous (no jump) when the target
 * changes during a move, at the cost of a small velocity discontinuity
 * (a kink, not a jump) if retargeted while still accelerating/cruising/
 * decelerating. Good enough to avoid the large current spikes a raw
 * position step would cause; true velocity-continuous replanning would be
 * a further refinement if ever needed.
 */
typedef struct {
    /* Live output, read by the caller each tick. */
    float pos;   /* current shaped position reference (rad) */
    float vel;   /* current shaped velocity (rad/s) — for future feedforward use */

    /* Plan parameters, computed once per FOC_TrapGen_Start() call. */
    float p0, p1;        /* profile start / end position, unwrapped local frame (rad) */
    float dir;            /* +1.0 or -1.0 */
    float a_max;
    float v_cruise;        /* peak velocity actually used this move (<= v_max) */
    float d_acc;            /* distance covered during the accel phase (rad) */
    float t_acc, t_cruise, t_total;
    float elapsed;

    float   last_target_raw; /* last raw target passed to Start(), for change detection */
    uint8_t initialized;      /* 0 until FOC_TrapGen_Seed() has been called at least once */
} FOC_TrapGen_t;

/* Invalidate — marks the generator as needing re-seeding with a real
 * position before its output can be trusted. Cheap, no motor state needed;
 * call from FOC_Reset() on mode switch / fault recovery. */
void FOC_TrapGen_Reset(FOC_TrapGen_t *gen);

/* Seed the generator at a stationary point (pos = vel = 0, target = pos0).
 * Call once, lazily, the first time the generator is used after a reset —
 * typically with the motor's current actual position, for a bumpless
 * start. Safe to call repeatedly; only meaningful right after Reset(). */
void FOC_TrapGen_Seed(FOC_TrapGen_t *gen, float pos0);

/* Plan a new move from the generator's current pos to target_raw (shortest
 * rotational path). Call whenever the externally-commanded target changes;
 * safe to call mid-move (replans from the current output, see struct doc). */
void FOC_TrapGen_Start(FOC_TrapGen_t *gen, float target_raw, float v_max, float a_max);

/* Advance the profile by one control step and return the current shaped
 * position (also updates gen->pos / gen->vel). Ts in seconds. */
float FOC_TrapGen_Update(FOC_TrapGen_t *gen, float Ts);

#endif /* FOC_TRAPGEN_H */
