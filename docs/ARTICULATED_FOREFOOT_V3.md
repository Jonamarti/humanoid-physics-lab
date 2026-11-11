# Gait v3: isolated, passive articulated forefoot (experimental)

## Scope

This branch adds `public/models/humanoid_gait_v3.xml`, a **new opt-in MJCF model**.
The old `humanoid_v1.xml` standing/push model and `humanoid_step_v2.xml`
step model remain byte-for-byte unchanged.

Each foot remains **1.1 kg** but is divided into a 0.8 kg rearfoot and
0.3 kg toe segment. The MTP hinge sits at local x=0.095 m. Both box soles
meet the ground at neutral zero joint angle; the toe tip lifts under
positive MTP rotation. The hinge is passive, with limited range, stiffness
and damping. It has no motor, no world anchor and no free-root actuator.

Total global mass remains 75 kg. The existing `left_sole` and
`right_sole` sites are preserved for backwards compatibility, but are
**rearfoot body sites**, not a complete measure of toe material sliding.

## Force measurement

`humanoidGroundSupportObservation()` recognizes toe geoms when present,
collects the solved floor-contact forces from both segment bodies, and
returns aggregated support per side with optional `segments.rearfoot`
and `segments.forefoot` measurements. Upward loads and horizontal
reactions are sums, not contact counts. A segment is included in the
loaded support hull only after its solved normal load crosses the existing
load threshold. v1/v2 retain their legacy observation shape.

The contact centroid is **not** a 6D center of pressure (CoP). The
current `estimateContactPatchSlip()` is a segment-rigid transform estimator:
if used for v3, feed the world pose of the *actual contacting segment*
rather than the parent foot pose. A migrating contact point by itself
is not material sliding.

## Validated benchmark outcomes (2026-11-10)

The [one-off v3 MuJoCo benchmark](https://github.com/Jonamarti/humanoid-physics-lab/actions/runs/42919217443)
executed 10 seconds of unperturbed standing and six 12 cm forward-step trials
using the unchanged v2 controller gains. The temporary workflow step was
removed afterwards, leaving the benchmark opt-in.

### V3 standing, 10 simulated seconds (2 ms integration)

| Metric | Measured |
| --- | ---: |
| Final and minimum pelvis height | 0.9896 m |
| Maximum sampled free-root tilt | 0.0494 degrees |
| Bilateral foot-loaded duration | 10 s |
| Final total upward ground reaction | 735.75 N |
| Rearfoot vertical force on each side | 328.1472 N |
| Forefoot vertical force on each side | 39.7278 N |
| Standing diagnostic candidate | true |

These observations support **unperturbed nominal standing**, not push
recovery, rolling toe-off or walking. Forefoot load at the final instant is
below the slip estimator's **40 N** loaded-sample gate; its reported zero
slip covers only 6 consecutive qualifying samples, not the entire stance.

### V3 steps, both sides and three ground-friction settings

| Friction | Left step | Right step |
| --- | --- | --- |
| 0.9 | FALL, no confirmed landing | FALL, no confirmed landing |
| 1.0 | FALL, no confirmed landing | FALL, no confirmed landing |
| 1.1 | FALL, no confirmed landing | FALL, no confirmed landing |

All **six tests failed biomechanical acceptance**: no sustained landing and
zero settled duration in every trial. Some support toes carried up to 100%
of instantaneous foot load, but this does not establish controlled toe-off.
Segment-local material slip estimates reached >10 cm in several tests.
The zero-valued sole-marker landing-slip metric on cases with *no confirmed
landing* is **not evidence of zero sliding**.

The benchmark exits with a successful software-test status because it
records failure outcomes as valid measurements, without asserting gait
success. Do **not** interpret green CI as biomechanical success.
The previous v2 controls remain unchanged and cannot be reused on
articulated feet without further closed-loop support-transfer work.

## V3 support transfer: two measured controller revisions (2026-11-11)

Experiments were run with the same 75 kg v3 model and the original six-case
friction/side sweep. The existing v1/v2 controller and all step-acceptance
gates remain unchanged.

| Controller | Left 0.9/1.0/1.1 | Right 0.9/1.0/1.1 | Stability |
| --- | --- | --- | --- |
| Original timed pre-lift sagittal + lateral shift | FALL / FALL / FALL | FALL / FALL / FALL | Loses balance after abort |
| V3 sustained load + capture gate, still forward lean | FALL / FALL / FALL | FALL / FALL / FALL | Abort at 2 s, subsequent fall at ~3.54–3.66 s |
| V3 gated **lateral-first** shift, no sagittal pre-lift lean | ABORT / ABORT / ABORT | ABORT / ABORT / ABORT | **No physical fall** in 6 s |

Source for the guarded but still forward-leaning revision:
[GitHub Actions #42924050956](https://github.com/Jonamarti/humanoid-physics-lab/actions/runs/42924050956).
Source for the lateral-first revision:
[GitHub Actions #42924297003](https://github.com/Jonamarti/humanoid-physics-lab/actions/runs/42924297003).

The lateral-first revision keeps the pelvis minimum between **0.9888 m and
0.9890 m** and samples no FALL in six cases, across friction 0.9, 1.0 and
1.1. Root tilt at abort is between **0.0597 and 0.0719 degrees**.
It aborts at t=2.00 s, deliberately without lifting the swing foot:
at this instant the stance foot carries approximately **490 N out of
736 N** (about 66.6% load), below the conservative **80% continuous**
release threshold. The system correctly refuses to invent a successful
step. No sample establishes toe-off, landing, or walking.

The v3-only release gate requires at least 80% solved load, a genuinely
unloaded opposite foot, the estimated capture point and COM projection
inside the loaded support polygon, low COM velocity, and 0.12 s of
consecutive valid 50 Hz samples. A gap or failed sample resets continuity.
The reference is bounded; **forward COM bias and spine pitching are
suppressed only during v3 SHIFT_WEIGHT**. The v2 gait logic remains
unchanged.

**Next experiment:** cautiously sweep bounded lateral COM reference
amplitude with load-feedback to reach the support threshold without
sacrificing the 6 s fall-free result. This has **not** been implemented or
validated in the current revision. Do not relax the 80% gate to manufacture
lift, and do not add propulsive toe torque before stable support transfer.

## Bounded lateral shift sweep: physically measured (2026-11-11)

The [eight-trial GitHub Actions benchmark](https://github.com/Jonamarti/humanoid-physics-lab/actions/runs/42934308595)
compared four lateral COM-reference amplitudes on both swing legs, using the
unmodified 75 kg v3 model, original strict **80% sustained support-load gate**
and the same 12 cm forward step command. Default remains 0.075 m.

| Lateral reference (m) | Max pre-lift support load | Left | Right |
| --- | ---: | --- | --- |
| 0.075 | 71.35% | ABORT, no fall | ABORT, no fall |
| 0.095 | 76.77% | ABORT, no fall | ABORT, no fall |
| 0.115 | 82.07% | LIFT -> SWING -> ABORT -> FALL | LIFT -> SWING -> ABORT -> FALL |
| 0.135 | 83.99% | LIFT -> SWING -> ABORT -> FALL | LIFT -> SWING -> LAND -> ABORT -> FALL |

For 0.115 m, lift began at ~1.50 s, swing at ~1.84 s, and the first
physical FALL occurred ~3.32 s (left) or ~3.38 s (right). For 0.135 m,
lift began at ~1.40 s. **Neither setting confirmed a stable landing**.
The 0.135 m right-foot trial reached LAND but not a confirmed loaded
touchdown with sustained equilibrium. It ultimately fell.

**Interpretation:** a larger lateral COM reference clears the original
physical takeoff-load threshold, but existing swing/landing control is
still unable to complete the maneuver. This is *not* an improved walking
controller. Raising the reference at the default v3 setting would
regress its fall-free ABORT behavior; therefore **0.075 m is retained
as default** and the wider [0.05, 0.15] m parameter range is opt-in
for diagnostics. The previous v1/v2 controller and all release and
acceptance gates are unchanged.

Run explicitly when desired:

```sh
HPL_RUN_V3_SHIFT_SWEEP=1 npx vitest run tests/manual/v3LateralShiftSweep.test.ts
```

The temporary extra CI step was removed after measurement. Next action:
diagnose actual support-foot heel/toe contact, capture-point displacement
and unloading during SWING; then independently tune the landing target and
post-ABORT torque response. Do not claim success based on the earlier
release gate or falsely count the fall as a stable landing.

## Separate optional whole-body angular-momentum controller

The v3-only, **disabled-by-default** arm/hip/spine momentum response is
documented in [WHOLE_BODY_BALANCE.md](WHOLE_BODY_BALANCE.md). The measured
eight-pair push comparison recovered 7/8 with both baseline and proposed
control: slightly lower sagittal peak tilt, slightly higher lateral peak
tilt, and an unrecovered 240 N backward fall in both policies.
This does **not** change the current gait ABORTs, nor authorize enabling
whole-body compensation simultaneously with the voluntary-step FSM.

## Browser inspection workflow (experimental)

In the existing **Humanoide — equilibrio activo** scenario, load the
nominal humanoid first. The new **Modelo físico de los pies** selector
defaults to the previously measured **v2** model. Select **v3** only to
run a voluntary step with articulated toes, or press **Probar
bipedestación con v3 (10 s)** to load the v3 model and run 10 seconds
of physical integration.

The viewer displays separate MuJoCo solved normal loads for each
rearfoot and forefoot and the fraction of each total foot load borne
by the toes. There are no added motors, toe constraints, inverse-kinematic
corrections, or hidden stabilizers in the UI.

**Interpretation:** visible toe load is only contact evidence. It does
not prove static friction, roll-over, support impulse, viable recovery,
or stable walking. The old v2 step remains the default. The push
button switches back to nominal v1 to avoid invalid comparison between
the newly articulated gait model and prior push benchmarks.

This UI route is source-level implemented but still needs an actual
browser/WebAssembly smoke test before being treated as working.

## Repeatable benchmark protocol (executed once, still opt-in)

The optional `tests/manual/articulatedForefootV3Bench.test.ts` records:

- **Standing:** 10 s at 2 ms solver timestep, pelvis minimum/end height,
  maximum root tilt, bilateral load duration, final rearfoot/forefoot loads,
  and segment-local material-point slip estimate.
- **Steps:** 12 cm forward, left and right, friction 0.9 / 1.0 / 1.1,
  existing v2 posture/Cartesian/guard gains (0.52/0.48), with physical
  final state reported verbatim: COMPLETE, ABORT or FALL.
- **Failure onset:** timestamp of first FALL, minimum pelvis height,
  maximum free-root tilt and per-phase transition snapshots, including
  the solved rearfoot/forefoot loads and support-toe fraction. Added
  after the initial six-case run; these extra columns require an opt-in rerun.
- **Contact:** peak load fraction carried by rearfoot vs toes,
  loaded centroid migration, solved tangential reaction integrated in
  N·s and the separate rigid-segment slip estimates. A missing loaded
  sample is `null`, not interpreted as zero slip.

Run only with an explicit opt-in (not in ordinary CI):

```sh
HPL_RUN_V3_BENCH=1 npx vitest run tests/manual/articulatedForefootV3Bench.test.ts
```

These diagnostics are **not** biomechanical validation. The estimated
material-point slip uses the currently loaded contact centroid and each
segment's own pose, while sole-marker displacement is a different metric.
No control forces are applied directly to the free root. Friction sweeps
are actual model changes, not threshold changes.

The benchmark was executed on GitHub Actions once (run 42919217443).
The local container still has no network access to npm dependencies, but
the physical MuJoCo outcomes are captured above. Ordinary CI does not
re-run the opt-in benchmark after removal of the temporary step.

## Next steps

1. Run `npm run typecheck && npm run lint && npm test && npm run build`
   locally, with separate long-running v3 scenarios if CPU budget allows.
2. Measure first-contact load per segment, foot pose and segment-local
   contact material velocities (not the rearfoot's site displacement).
3. Preserve the successful 10-second unperturbed standing baseline.
4. Instrument failure onset: MTP angle, solved toe load, heel-to-toe
   load-center movement and COM/capture-point position per gait phase.
5. Adapt hip/knee/ankle feedback for closed-loop support transfer before
   considering bounded toe or ankle propulsion.
6. Re-run both legs and friction sweep, keeping FALL/ABORT/COMPLETE gates.

This model is a **research prototype** and not a validated human gait.
