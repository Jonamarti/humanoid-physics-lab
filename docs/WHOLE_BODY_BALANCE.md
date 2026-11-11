# Experimental whole-body angular-momentum response (v3 only)

Status: opt-in research controller, **not** a verified whole-body inverse-dynamics
optimizer, validated fall recovery, or a replacement for stepping.

## What is measured and controlled

MuJoCo `mj_subtreeVel` populates `data.subtree_angmom` for the free-pelvis
body (index 1), representing the whole humanoid subtree angular momentum
about its center of mass in world axes, in kg·m²/s. Free-root angular
velocity is read from `qvel[3..5]`. These signals are not the same as a
net *external* moment. The solved floor reaction is also measured and
used to inhibit body compensation when support is lost.

The new `wholeBodyBalanceCommand` uses a deadband and bounded, per-axis
mapping to internal hinge targets for shoulders, hips, and spine. There
is no direct root/pelvis actuator, rigid attachment to the ground, kinematic
teleportation or fabricated ground force. `applyStepJointTargets` checks
the hinge whitelist and applies each target through the existing limited
joint PD gains, after the standing controller. Commands update at 50 Hz;
MuJoCo still integrates every contact and articulated segment at 500 Hz.

For the present version, shoulder/hip/spine target magnitudes are bounded
by **0.33 / 0.045 / 0.05 radians** on the principal pitch response.
The roll/yaw targets are also bounded. The response signs and gains are
experimental and must be validated for specific measured perturbations,
not inferred from movement plausibility.

The controller is **disabled by default**, only accepts articulated v3
after activating standing control, and refuses simultaneous operation
with the current voluntary-step FSM. It is retained through simulation
reset (with its sampling state cleared) and disabled on model reload.
The historical v1/v2 models, baseline torques, and step acceptance
thresholds are unchanged. All published normal push comparisons still
use v1; the new optional viewer path compares two policies on v3.

## Browser workflow

In the standing scenario, choose between:

- **Referencia: control de tobillos y postura** (v3 baseline)
- **Compensación articular: hombros, caderas y tronco** (v3 experimental)

Use the v3 standing button for a 10 s nominal-stability trial, or
**Probar empujón con v3 (6 s)** with the currently selected push direction,
peak force and duration. The tool records separate v3 rearfoot/forefoot
loads and displays the controller's normalized world-axis momentum signal.
No postural success is inferred from an active target or green CI.

The original push button explicitly returns to unchanged v1 and should
not be compared directly with a v3 push when investigating gains.

## Multi-angle force editor integration

The **Editor avanzado: varias fuerzas 3D** now has its own strategy selector,
located immediately above the editor:

- **v3 — intentar compensar con brazos, caderas y tronco** (selected by
  default for this editor; **not** enabled by default for other modes).
- **v3 — solo controlador postural de referencia**, for A/B comparison on
  exactly the same articulated-foot model.
- **v1 — controlador original para comparación** (legacy behavior).

Enabling the editor prepares the requested MuJoCo model *before* the user
selects custom force anchor points. When changing strategies the model
switches immediately, the standing controller is reset and any previous
body-index-based custom anchors are invalidated: v1 and v3 body indices
may refer to different segments. Force vectors, timing and named sites
are preserved. Re-select custom body points on the new model before
running; stale anchors must never be silently applied to another limb.

The editor freezes its force sequence at run, resets the current model,
enables the opt-in whole-body controller only for that selected strategy,
and applies **every 3D force** by its authored real site/body attachment
at each MuJoCo physics step. Controllers do not alter the authored
external-force vectors, impulses or timing. Normal v1/v2 tests and the
voluntary gait-step FSM remain separate.

The resulting interface labels a **fall**, **recovery without stepping**
or **no recovery**, based on sampled minimum pelvis height, maximum
root tilt, final foot drift and final COM displacement/settling.
The judgment uses the same conservative diagnostic family as the single
push benchmark, and is explicitly approximate (render-frame sampling
is not continuous verification). The editor does not yet invent a
compensatory step.

`tests/unit/multiAngleWholeBodyMujoco.test.ts` integrates three overlapping
and non-overlapping pushes applied at different anatomical sites and
world-space axes on v3, checks that identical force samples are used for
both policies and confirms measured actuator response rather than
pretending recovery from a commanded pose.

## Physical evidence

[One-off benchmark, GitHub Actions #42927302329](https://github.com/Jonamarti/humanoid-physics-lab/actions/runs/42927302329).
`HPL_RUN_WB_BENCH=1 npx vitest run tests/manual/wholeBodyBalanceBench.test.ts`

Nominal unperturbed v3 stance with whole-body controller enabled:
10 s simulated, minimum pelvis height 0.9896 m, maximum sampled root
tilt 0.0494°. Physical force tests used the same v3 model, half-sine
sternum push at t=1 s for 0.12 s, 6 s total duration for both strategies.
No contact in these eight pairs had missing solved constraint rows.

| Direction | Peak (N) | Baseline max tilt | Whole-body max tilt | Recovered baseline / WB |
| --- | ---: | ---: | ---: | --- |
| Forward | 120 | 2.1463° | 1.8892° | yes / yes |
| Forward | 240 | 5.3714° | 5.2662° | yes / yes |
| Backward | 120 | 4.9696° | 4.6487° | yes / yes |
| Backward | 240 | 89.9533° | 89.5368° | **no / no; FALL** |
| Left | 120 | 0.2685° | 0.2948° | yes / yes |
| Left | 240 | 0.7908° | 0.8034° | yes / yes |
| Right | 120 | 0.2685° | 0.2948° | yes / yes |
| Right | 240 | 0.7908° | 0.8034° | yes / yes |

Interpretation: in 4 sagittal cases, maximum sampled tilt improved
slightly. In 4 lateral cases, it regressed by 0.0126–0.0263 degrees.
**Both policies recovered 7/8**, both fell after the 240 N backward
pulse. The torso still reaches approximately 90° tilt in that case,
so a small numerical tilt reduction is not recovery. This is an
isolated synthetic-model experiment, not evidence of humanlike
biomechanics or generalized control superiority.

The initial comparison run
[#42926953267](https://github.com/Jonamarti/humanoid-physics-lab/actions/runs/42926953267)
identified an incomplete solved-contact row during a stronger push.
The force decoder remains strict; its caller now checks that the
current `nefc` and `efc_force` have all rows before decoding.
Unsolved contacts are separately counted and **never** converted to
normal force. The final eight-pair benchmark reported zero such
unresolved floor-foot contacts. Unit tests cover invalid solver ranges.

## Boundaries and next scientific steps

1. Quantify actual shoulder/hip/spine **joint-angle responses**, root
   angular momentum trajectories, contact friction and cumulative impulse,
   not just requested targets and peak root tilt.
2. Determine whether roll-axis response should be inhibited near small
   self-correcting lateral perturbations. Do not tune to four data points
   without a holdout set; repeat diagonal and randomized directions.
3. Use contact-constrained whole-body inverse dynamics / QP with explicit
   task priorities (COM, feet, angular momentum), joint and friction limits.
   This prototype currently uses a bounded signed-momentum heuristic.
4. Add phase-aware hip/arm balance to v3 support transfer **only after**
   the standalone 6 s / 10 s perturbation benchmarks are robust.
   Preserve strict lift, landing and FALL/ABORT/COMPLETE gates.
5. Any autonomous stepping requires an independent capture-region and
   contact planner; arbitrary limb motion cannot remove total angular
   momentum without corresponding external moment from the ground.

**Do not merge this as a solved recovery controller.**
