# Landing pose retention: controlled physics experiment (#49)

This trial preserves *measured* hip/knee/ankle angles immediately after
the **MuJoCo-verified first loaded touchdown**, then eases back to the
original joint reference over ~1.45 s.

## Why

The corrected-knee model lifts and places the foot, but the experimental
step relaxes the moving leg's hip/knee posture after touchdown; if the
nominal controller pulls the foot toward neutral while the new stance
is still stabilizing, contact friction may saturate and the shoe slide.

## What this branch changes (only opt-in)

- New `landingPoseHoldStrength` parameter (0 by default, 0..1 allowed)
  at the end of `MujocoSimulation.enableExperimentalStepTrial`.
- Once actual solver-backed contact is confirmed, snapshot the six
  **actual measured hinge angles** of the newly planted leg (hip pitch,
  roll, yaw; knee; ankle pitch, roll).
- Blend only internal-joint PD angle references toward that measured
  configuration, tapering with a smooth ramp. Original torque limits
  are untouched; no root actuation, no foot locking/teleportation, no
  fake traction or new external forces.
- Ten headless 75kg MuJoCo trials compare the baseline to latch strengths
  0.25/0.5/0.75/1 on 12cm forward steps with servo 0.2; 10cm steps
  with servo 0.5; and mirrored right-foot tests.

## Acceptance

Exactly the same strict physics metrics as before: contact physically
confirmed, new foot in target <=5cm, foot drift <=2cm, stance drift
<=2cm, valid COM and capture point, low COM speed, pelvis/trunk upright
and at least 1 full second of continuous loaded stable support.

**A green CI means the code is consistent, not that walking works.**
If the approach worsens slip, errors or falls, close without merging.
Do not enable this option on public steps automatically until left/right
tests succeed without regression.

The trial log marker is `LANDING-MEASURED-POSE-MUJOCO`.
