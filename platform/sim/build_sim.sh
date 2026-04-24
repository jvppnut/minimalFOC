#!/usr/bin/env bash
set -e

ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
OUT="$(dirname "$0")/foc_sim.so"

gcc -shared -fPIC -O2 \
    -I"$ROOT" \
    "$ROOT/core/math/foc_math.c" \
    "$ROOT/core/math/foc_pid.c" \
    "$ROOT/foc.c" \
    "$ROOT/platform/sim/foc_sim_interface.c" \
    -o "$OUT" \
    -lm

echo "Built $OUT"
