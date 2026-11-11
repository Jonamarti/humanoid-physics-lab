# Recovery stepping: directional test matrix (v3 research)

## Why these tests precede more controller tuning

An out-of-support **capture point** reflects both COM position and velocity.
A static COM projection inside the feet can still be dynamically
unrecoverable if the COM is moving rapidly. Conversely, an outward torso
tilt does not by itself mean a step must occur. The controller may use
hips, trunk, arms and bilateral ankle torque until a sustained physical
support/capture problem requires foot placement.

The goal is to distinguish **planning a plausible step**, **lifting the
swing foot clear of the floor**, **moving the foot**, **landing in real
contact**, **absorbing load without locking the knee** and **recovering
dynamically stable bilateral support**. A programmed animation or green
unit tests never substitute for those observations.

## Contract and integration tests

- `tests/unit/perturbationRecoveryContracts.test.ts`: impulse
  dimensions and signs, noise latency, capture-point dynamics, bounded
  step size, directional target signs, leg-crossing constraints, knee
  shortening on lift and gradual extension on swing/landing, lateral
  ankle feedback and refusal to declare a landing before solved contact.
  These are **deterministic unit contracts**, not physical proof.
- `tests/manual/directionalStepRecoveryBench.test.ts`: opt-in MuJoCo
  fixed-timestep force pulses at the sternum. Twelve graded perturbations
  (three magnitudes per direction: front +X 120/200/300 N, back -X
  100/160/240 N, left +Y 120/200/300 N, right -Y 120/200/300 N),
  each a half-sine of duration 0.16 s beginning at t=1 s. The backward
  series deliberately starts lower. All 12 receive identical 6-second
  observation windows, with no direct pelvis actuation.
- Two 7-second **chained pulse experiments**: left then right and forward
  then forward, each pulse at t=1.0 s and t=3.4 s. The present reactive
  controller permits at most **one physical step attempt per reset**;
  thus these are *negative capability/observability tests*, not claims
  of a two-step gait.

Run the physical sweep **only when explicitly requested**:

```sh
HPL_RUN_DIRECTIONAL_PUSHES=1 npx vitest run tests/manual/directionalStepRecoveryBench.test.ts
```

The main CI skips the manual suite by default. Values recorded from
MuJoCo include impulse in N·s, capture-point exit, support loads,
pelvis minimum, root tilt, planned XY step and reach, measured swing
clearance, actual phase transitions, loaded landing, settled duration,
post-landing COM velocity and any physical fall. An `ABORT` is not a
`COMPLETE`; unconfirmed landing and sole-marker drift of zero are
**not** evidence of a firm planted foot.

## Recorded first full 14-case MuJoCo run (2026-11-11)

[Physical run 42937043421](https://github.com/Jonamarti/humanoid-physics-lab/actions/runs/42937043421)
completed twelve 6 s directional trials and two 7 s chained trials,
using solved contacts and real external half-sine pulses at the sternum.
The deterministic suite passed **159 tests**, with 9 skips and 3 explicit
todos; the two opt-in physical suites also completed as *diagnostics*.
A green software run does **not** imply successful step recovery.

| Input direction | Peak force N | Physical outcome | Real LIFT | Confirmed landing |
| --- | ---: | --- | --- | --- |
| +X forward | 120 | Upright/no step | no | no |
| +X forward | 200 | FALL | no | no |
| +X forward | 300 | FALL | no | no |
| -X backward | 100 | Upright/no step | no | no |
| -X backward | 160 | FALL | no | no |
| -X backward | 240 | FALL | no | no |
| +Y left | 120 | Upright/no step | no | no |
| +Y left | 200 | Upright/no step | no | no |
| +Y left | 300 | FALL | no | no |
| -Y right | 120 | Upright/no step | no | no |
| -Y right | 200 | Upright/no step | no | no |
| -Y right | 300 | FALL | no | no |
| left then right (160 N each) | 160 | Upright/no step | no | no |
| forward then forward (160 N each) | 160 | Upright/no step | no | no |

Every FALL case produced **one planned step**, but its physical FSM
never reached LIFT before the pelvis/tilt fell outside the acceptable
range. This is not a collision-free knee-lift accomplishment. The
measured apparent sole rise during falling is **not** proof that a
swing foot was physically raised. No case completed a planted,
loaded, settled step. The paired opposite-direction trials at
160 N were compensated without stepping, so they did not exercise
two sequential validated steps.

Observed first FALL timestamps: forward 200/300 N 3.28/2.44 s;
backward 160/240 N 2.78/2.30 s; left and right 300 N 2.54 s.
These outcomes identify the **pre-lift support transfer and timing**
as the first blocking physical stage, before the landing controller.

The one-off CI step was removed after saving this evidence. The
default branch/model gait acceptance criteria have not been changed.
Proposed next step: isolate a narrower force interval near each
automatic-step trigger, log longitudinal/lateral COM and capture-point
history during SHIFT_WEIGHT at 50 Hz, and vary only load-transfer
policy after confirming physical support. Do not loosen touchdown or
stability gates to label the run successful.

## Progressive acceptance milestones (not yet achieved)

1. **Small perturbation:** stay upright with joint balance, not an
   unnecessary footstep. Preserve v1/v2 and never use an artificial
   root force. Check both fore/aft and both lateral directions.
2. **Recoverable perturbation:** after sustained capture-point escape,
   identify the reachable landing position and support leg. Lift by
   hip flexion plus *actual* knee flexion so the foot clears the floor;
   verify absence of a significant unloaded-foot collision.
3. **First planted step:** recover a genuinely loaded new support, with
   knee still flexed enough to absorb impact, no material foot sliding
   and 1 second of continuous valid dynamic stability. Do not merely
   check COM is geometrically between feet; require capture point and
   COM velocity gates. Limit reach by articulation and contact geometry.
4. **Out-of-range perturbation:** honest `FALL` or `NO_SAFE_STEP`, not
   a fabricated arbitrarily long step or a planted foot placed by
   teleportation.
5. **Multiple recovery steps:** if a **completed** first step leaves
   velocity/capture still outside the new base, transfer load and perform
   the next real step; otherwise return to a neutral stance. The
   existing one-attempt coordinator needs redesign and new tests.
6. **Continual gait:** alternate stance and swing with rear-leg
   ground-reaction propulsion, flexed-knee impact absorption, transverse
   stability and real foot/toe contact-slip metrics. This is a separate
   locomotion controller milestone, *not* an acceptance condition of
   the current single-step prototype.

The command targets and 0.34–0.38 m experimental reach limits are
**engineering bounds**, not a claim about human anatomical reach.
The test suite cannot prove anatomical realism from a fixed nominal
humanoid alone; collision-free kinematics, torque limits and material
slip must be measured in the actual solver, then validated on varied
initial stances, friction, impulse durations and masses.

## Known gaps to keep visible

- `ReactiveStepCoordinator` stores at most one attempt; chained pushes
  cannot be expected to produce a second physical footfall.
- The current `ExperimentalStepSequence` may initiate a lift but later
  `ABORT` or `FALL`; current v3 walking remains unvalidated.
- The `left_sole` / `right_sole` sites belong to the rearfoot rigid
  body: they are not sufficient for measuring articulated-toe slipping.
- The COM/capture-point model is a heuristic for the 75 kg synthetic
  humanoid, not a full whole-body inverse-dynamics controller.

Do **not** change solver friction, step-completion criteria or inject
kinematic root motion to satisfy tests. Treat the measured limitations
as requirements for future code instead.
