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

- Model structure and segment telemetry implemented; numerical smoke tests
  added, **not yet executed** for this branch.
- 10-second standing stability with free root: **not tested**.
- Loaded heel-to-toe roll: **not tested**.
- Physically measured push-off impulse with no material sliding: **not tested**.
- Left/right COMPLETE steps across friction 0.9, 1.0 and 1.1: **not tested**.
- No changes to the baseline v2 controller or acceptance thresholds.

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
