# Reactive capture-point step recovery (experimental, v3 only)

## Intended behaviour

The advanced 3D force editor now defaults to **v3 – compensate, then try
a reactive step**. This experiment keeps the old whole-body momentum
response on shoulders/hips/trunk/ankles active **until** a supported,
physically measured loss of dynamic balance persists. It then considers
one leg step toward the direction in which the centre of mass is moving.

The opt-in mode is also offered in the single v3 push selector. Other
modes are unaffected: v1 and v2 are unmodified and the previous whole-body
controller without steps remains available for baseline comparison.

**A requested step is NOT a verified recovered step.** The existing
experimental v3 step FSM has so far failed to produce a fully validated
walking step under the earlier trials. This feature is an experimental
reactive trigger and a physical attempt, not validated reactive locomotion.

## How it works

- Read *solved* MuJoCo upward foot loads, loaded rearfoot/forefoot
  polygons, 3D COM/velocity and free-root orientation at 50 Hz.
- Use the linear inverted pendulum capture-point approximation:
  `capture_xy = com_xy + velocity_xy * sqrt(com_height / 9.81)`.
  This is a screening heuristic, not a true whole-body viability proof.
- Continue using shoulders, hips and trunk for ordinary recoverable
  disturbances. Require COM horizontal speed >= 0.12 m/s, capture point
  *outside* the measured loaded-support hull, body load >=60% of 75 kg
  bodyweight, and at least 0.12 s of continuous unstable observations
  before requesting a step. A missing sample resets the clock.
- Reuse `planRecovery()` to check that both feet bear load, choose an
  anatomical swing/support side, reject crossover and unreachable
  landing points, and predict a new support hull that contains the
  capture point. Planned reach is capped at 0.34 m.
- Require the proposed change in swing-foot position to point in the
  direction of COM motion. Only then start one
  `ExperimentalStepSequence` with the original physical guards.
- During active attempted stepping, transfer control to the existing
  MuJoCo hip/knee/ankle/joint PD and bounded Cartesian foot impedance.
  **No teleportation, no root force or artificial contact**.
  The actual foot must be unloaded, lifted, moved, landed, loaded and
  settled according to the existing physics gates before COMPLETE.
- If no safe step exists, keep the whole-body controller on and report
  `NO_SAFE_STEP` instead of inventing a path. Following an ABORT or FALL,
  do not try repeatedly within the same trial; reset before another
  attempt. The whole-body joint balance loop can resume after a failed
  step. The FSM may still identify a later fall.

The phase vocabulary is
`MONITORING → WARNING → STEP_ATTEMPT → STEP_COMPLETE / STEP_ABORT / FALL`,
with `NO_SAFE_STEP` used for unsupported or unreachable steps. Every
phase is an observation/planning state; only the existing MuJoCo FSM can
confirm a physical step.

## Using it

In **Humanoide – equilibrio activo**, enable
**Editor avanzado: varias fuerzas 3D** and choose the first option:
**v3 – compensar y dar un paso reactivo si es necesario**.
Set force arrows, application sites, impulse times and durations, then
click **Ejecutar secuencia**. Live output reports the planned foot,
delta X/Y in centimetres, and actual physical phase when triggered.
At completion it reports the observed outcome; a fall or abort is not
labelled as successful recovery. Compare with **v3 solo compensación
corporal**, **v3 solo postura** and the original v1 strategy on the
same input sequence.

Changing between v1 and v3 invalidates user-picked body-index
attachments (you must re-select those points) to avoid applying a force
to a different body. Force vectors and schedules remain unchanged.

## Validation and limits

Deterministic planner tests cover four motion directions, 0-step
nominal standing, transient disturbances, invalid contacts/time,
single-attempt hysteresis and abort/fall reporting. A MuJoCo test applies
real diagonal and lateral force pulses and records actual load, fall
angle, number of trigger attempts and physical step state.

**The presence of a step trigger or a green test suite does not prove
that v3 can physically complete a recovery step.** Previous v3
coordinated-gait trials terminated in ABORT or FALL. Do not enable
this experiment by default outside its deliberately selected 3D force
editor mode; don't change the existing physical acceptance gates.

Next research step: record triggered directions and COM capture-point
history over a stronger, held-out perturbation sweep. Adapt actual
support-load transfer and joint trajectories **only based on measured
outcomes**. A fully viable solution likely requires contact-aware
whole-body inverse dynamics with a foot-placement/reachability planner,
rather than simply increasing joint torques.
