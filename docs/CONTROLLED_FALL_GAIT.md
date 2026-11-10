# Experimental step: controlled COM progression and anatomical knee flexion

This documents the incremental response to feedback that the previous
experimental humanoid **rose on tiptoe and barely appeared to bend the knee**,
despite some numeric lifting/contact results.

## Cause identified in the kinematic model

`humanoid_v1.xml` (the original published model) originally gave both knees the hinge axis `0 1 0`
and allowed angles `[-2.45,0]` radians. In the MuJoCo coordinate
convention (`+X` forward, `+Z` up), a **negative** rotation around
`+Y` sends the downward shank toward the **front**. Human knee flexion
should fold the shank **behind** the thigh in a forward swing.

The separate **`humanoid_step_v2.xml`** has knee axes `0 -1 0`, with the same neutral pose at 0 rad
and the same constrained negative angle range. **`humanoid_v1.xml` remains unchanged** so published standing, pushes and historical comparisons retain their previous contact dynamics. Hip negative pitch raises
the thigh **forward**; knee negative pitch now curls shank **backward**.
This changes actual physical geometry, not just the renderer.

## Coordinated opt-in gait profile

An explicit `coordinatedGaitProfile` parameter controls an experimental
posture sequence at the internal-joint level:

- `SHIFT_WEIGHT`: reference COM shifts up to **9.5 cm** toward the
  desired travel direction in addition to the existing lateral weight
  transfer. Spine's lumbar and thoracic pitches together target a
  modest **~4°** of forward lean (reverse sign for backward steps).
- `LIFT`: swing hip pitches forward to raise the **knee/thigh**, knee
  flexion ramps toward **~1.0 rad (~57°)** to shorten the leg for
  clearance; ankle follows a bounded pitch reference.
- `SWING`: keep a flexed knee while carrying the thigh forward, then
  start extension. The existing end-effector XYZ reference remains
  a bounded Jacobian-based optional servo, not a teleported foot.
- `LAND`: smoothly extend the knee before contact; suppress sudden
  snapping to the previous neutral pose.
- `RECOVER`: gradually straighten the torso and transfer the COM
  reference to the newly observed support. Contact load, body
  acceleration, pelvis height, capture-point support and 1s stability
  are **still measured** in MuJoCo.

These angles are **engineering targets**, not output animations.
`applyStepJointTargets` uses finite, clipped internal-joint torques and
does not touch the unactuated root; gravity and contact drive COM motion.
A slight torso pitch alone does not certify a walking step.

The web UI offers **Anterior** and **Experimental — torso + thigh +
knee** profiles (new profile selected by default when manually
launching a step). The push button remains completely separate:
an automatic step in response to 500 N is not yet implemented.

## References (qualitative calibration, not proof)

- Study on feedback stability of humans walking, modeling a dynamically
  controlled inverted pendulum and foot placement:
  https://pmc.ncbi.nlm.nih.gov/articles/PMC10997112/
- Review of knee kinematics during normal gait, approximately 60° of
  flexion in swing:
  https://pmc.ncbi.nlm.nih.gov/articles/PMC13418446/
- Swing-phase studies on knee flexion, followed by extension before
  landing: https://pmc.ncbi.nlm.nih.gov/articles/PMC2795578/

For human walking, saying "controlled falling" is a useful
approximation: COM position and velocity progress relative to
stance foot, while the swing leg catches/re-directs it; this does
**not** mean forcing the torso to pitch beyond recoverable support.

## Physics QA requirements

The new `forwardKneeTrunkMujoco.test.ts` records **actual** knee angle,
trunk angle, COM advance, foot clearance, contact and stability for
left/right steps, legacy/coordinated postures. Report ABORT and FALL
honestly. An actual step is not validated until support, placement,
low slip and 1s stability are demonstrated for both legs.

A second milestone will optimize COM and foot placement as a coupled
dynamic policy and then connect the existing recovery planner to
the perturbation trial. Do not claim that this release walks or
recovers pushes automatically.

## Measured progress, not yet validated walking (2026-11-10)

Real MuJoCo trials on `humanoid_step_v2.xml` established a significant
improvement after moving the COM reference forward and allowing the knee
to extend during SWING:

- Left and right, 0.12 m forward step, 20% Cartesian assistance:
  **~47.5°** achieved knee flexion, **~8.6 cm** forward COM movement,
  **~5.3 cm** maximum sole clearance, physically loaded touchdown,
  **~3.1 cm** of post-contact marker drift, about **5.6 cm** target error,
  upright pelvis. Final FSM **ABORT**, not a certified step.
- A separate 0.10 m step at 50% task-space servo reached ~3.7 cm target
  error, but still accumulated ~3.3 cm after-contact marker drift and
  **ABORT**. Other servo/step-length settings led to FALL. Parameters
  cannot be selected by passing just the numerics unit tests.
- A forward-swing **late-contact detection fix** treats a physically loaded
  toe contact after confirmed lift as end of SWING, even if the foot is
  already under the previous 2cm in-air height requirement. This prevents
  false timeouts of actual touchdown, without fabricating ground contact.

**Still missing:** verified support without >2 cm contact displacement,
<5 cm target error and 1 s of continuous dynamic stability, followed by
an actual second step. No automatic stepping after a push of 500 N.

GitHub Actions emits `GAIT-KNEE-TRUNK-MUJOCO`, `GAIT-FORWARD-TRACES`
and `GAIT-LANDING-SERVO-SWEEP` with the measured outcomes; the final
review must confirm the last CI on the branch.
