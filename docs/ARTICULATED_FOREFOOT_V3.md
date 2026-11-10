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

## Current acceptance status

- Model structure and segment telemetry implemented; numerical smoke tests and isolated force-aggregation tests
  added, **not yet executed** for this branch.
- 10-second standing stability with free root: **not tested**.
- Loaded heel-to-toe roll: **not tested**.
- Physically measured push-off impulse with no material sliding: **not tested**.
- Left/right COMPLETE steps across friction 0.9, 1.0 and 1.1: **not tested**.
- No changes to the baseline v2 controller or acceptance thresholds.

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

## Repeatable benchmark protocol (added, not yet executed)

The optional `tests/manual/articulatedForefootV3Bench.test.ts` records:

- **Standing:** 10 s at 2 ms solver timestep, pelvis minimum/end height,
  maximum root tilt, bilateral load duration, final rearfoot/forefoot loads,
  and segment-local material-point slip estimate.
- **Steps:** 12 cm forward, left and right, friction 0.9 / 1.0 / 1.1,
  existing v2 posture/Cartesian/guard gains (0.52/0.48), with physical
  final state reported verbatim: COMPLETE, ABORT or FALL.
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

The local execution environment used to prepare the PR did not have
MuJoCo/npm dependencies or external network access, so this run has **not
been performed**. Structural XML and mass/box geometry checks are not
substitutes for numerical MuJoCo compilation or integration.

## Next steps

1. Run `npm run typecheck && npm run lint && npm test && npm run build`
   locally, with separate long-running v3 scenarios if CPU budget allows.
2. Measure first-contact load per segment, foot pose and segment-local
   contact material velocities (not the rearfoot's site displacement).
3. Verify 10 seconds of bipedal stance before attempting any gait.
4. Tune hip/knee/ankle COM transfer for a genuinely loaded forefoot,
   then consider a bounded additional toe/ankle joint torque.
5. Compare both sides and friction sweep. Keep FALL/ABORT as actual outcomes.

This model is a **research prototype** and not a validated human gait.
