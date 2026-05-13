# minimalFOC

A field-oriented control (FOC) library targeting the STM32F7 series, designed to be maximally portable across MCU architectures.

## Project Goals
- Implement a complete FOC algorithm stack in C
- Strict separation between portable algorithm core and MCU-specific hardware drivers
- No external dependencies unless absolutely necessary (standalone)
- CAN communication is out of scope (not implemented)

## Development Workflow
- Build module by module, reviewing each together before moving to the next
- Commit to GitHub after each approved module
- Language: C (C99 or C11)

## Architecture

```
minimalFOC/
├── core/
│   ├── math/          # Pure-math transforms: Clarke, Park, SVPWM, trig tables, PID
│   ├── control/       # Current, speed, and position control loops
│   └── estimator/     # Flux observer, back-EMF estimator, sensor abstractions
├── hal/               # Hardware abstraction interface headers (no MCU code)
├── platform/
│   └── stm32f7/       # STM32F7-specific HAL implementations
├── motor/             # Motor parameter structs and limits
└── foc.h / foc.c      # Top-level state machine (init, align, run, fault)
```

## Layer Rules
- `core/` must compile with no platform headers included — zero MCU dependencies
- `hal/` defines interfaces only (function pointer tables or weak symbols); no hardware registers
- `platform/` is the only place STM32 HAL / CMSIS headers are allowed
- All floating-point math uses `float` (32-bit); `double` is avoided for MCU performance

## Module Build Order
1. `core/math/` — transforms and trig (most isolated, easiest to unit-test on host)
2. `core/math/foc_pid` — PID controller
3. `hal/` — abstract interface definitions
4. `motor/` — motor parameter struct
5. `core/control/` — current, speed, position loops
6. `core/estimator/` — observers and sensor abstractions
7. `platform/stm32f7/` — concrete HAL for STM32F7
8. `foc.h/.c` — top-level state machine

## Coding Conventions
- File prefix: `foc_` for all library files
- Struct typedef names: `FOC_Something_t`
- Public API functions: `FOC_ModuleName_Verb()` (e.g., `FOC_PID_Update()`)
- No dynamic memory allocation (`malloc`/`free` not used)
- No global mutable state — all state lives in caller-owned structs passed by pointer
