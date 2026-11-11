# v3 pre-lift support-transfer diagnostics (observational)

## Why this exists

The 14 recorded MuJoCo directional trials in PR #64 produced **zero
verified LIFT transitions** after an automatic push-triggered step request.
Most high-force trials entered SHIFT_WEIGHT and then FALL. A visible
sole-marker rise during a fall is **not** a real unloaded swing step.

This change isolates and **reuses the exact existing instantaneous
pre-lift predicates** in `src/control/supportTransferDiagnostics.ts`.
`ExperimentalStepSequence` still requires the original continuous
0.08 s emergency dwell, the >=0.25 s SHIFT_WEIGHT delay, the same
sample-gap rule and the same subsequent foot-flight, solved landing,
slip and 1 s stability requirements. No control gain, applied force,
friction coefficient, root force or pose target is changed.

## Reproduce

On the experimental v3 branch:

```sh
npm ci
npm run typecheck
npm test -- --run
HPL_RUN_DIRECTIONAL_PUSHES=1 npx vitest run tests/manual/directionalStepRecoveryBench.test.ts
```

The manual suite is deliberately opt-in and takes longer than unit
tests. Do not enable workflow jobs solely to run this matrix if CI
credits are constrained.

Each `DIRECTIONAL-STEP-PHYSICS` / `CHAINED-STEP-PHYSICS` JSON row
now includes `transferDiagnosis`, sampled every 0.02 s while the
physical FSM is in SHIFT_WEIGHT (maximum 100 trace frames per trial):

- Solved support/swing normal loads, stance share and the lowest
  swing-foot load; **not** inferred from foot geometry.
- COM world XY position, XY velocity and capture-point XY.
- COM/capture inclusion in the **currently loaded** support polygon.
- Pelvis height, root tilt and a per-sample boolean for each exact
  instantaneous pre-lift gate.
- Aggregated failed-gate frame counts, total ready frames and
  longest uninterrupted ready sequence.

A ready frame or five contiguous ready samples is **not** a verified
LIFT: the control FSM additionally enforces its own hysteresis, initial
delay and physical state transition. A LIFT is not a verified SWING,
loaded LAND or COMPLETE either. `transferDiagnosis.trace` can be
plotted offline to inspect time-to-fall and the changing foot load.

## Next physical experiment

Pick the narrowest tested forward/backward/lateral impulse range that
reliably triggers a step but is not immediately unrecoverable. Compare
the first 0.5 s of transfer against the baseline to find the dominant
failed gate, then test **one** change to load-shift/ankle/hip torques
at a time with paired trials. Keep the original v1/v2 regression matrix
and friction 0.9/1.0/1.1. Never lower acceptance gates merely to pass
tests; if no real unload occurs, record it as a negative result.

No fresh physical MuJoCo results are asserted by this diagnostic
change. The last established outcomes remain those in
`docs/DIRECTIONAL_RECOVERY_TEST_MATRIX.md`.
