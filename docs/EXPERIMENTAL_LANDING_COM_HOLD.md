# Experimental landing stabilization: COM reference and foot placement

This branch changes **only an explicitly opt-in experimental step trial**.
Normal standing, push forces and the published step defaults are unchanged.

## Hypothesis

A correctly swung foot can still drift after touching down if the
post-contact standing controller holds the *previous* center-of-mass target
and neutral leg posture, or if the reloaded foot has no world-space horizontal
damping while friction is near saturation.

Two independently adjustable inputs are compared:

- `landingComFraction` (default **0.5**): once touchdown is **confirmed
  by MuJoCo forces**, blend the standing COM reference toward
  `original_COM + fraction*(actual_foot_landing - original_foot)`.
  Test 0.5, 0.65, 0.8 and 0.9.
- `landingHoldScale` (default **0**): after confirmed touchdown in
  `RECOVER` or `COMPLETE`, if both feet support >45 N from the
  solver, apply a bounded horizontal-only foot-site impedance through
  the hip/knee/ankle Jacobian; no Z force, no world constraint,
  no spring attached to the ground, no direct action on the free root.
  Controller is capped to ±65 N virtual task-space force per axis and
  bounded by joint torque limits. Compare 0, 0.25 and 0.5.

Do not confuse **virtual task-space force mapped into internal joint
torques** with applying additional external force to the humanoid or
altering its contact friction. MuJoCo still resolves all contacts.

## Numeric acceptance rules (unchanged)

The step FSM must report `COMPLETE` only after a genuine foot flight,
landing with measured load, both feet supporting the body, final goal
error <=5 cm, planted foot drift <=2 cm, stance-foot drift <=2 cm, and
**1 continuous second of COM/capture point within the support hull**,
with low COM speed, upright pelvis and trunk.

Any `ABORT` or `FALL` is a real engineering failure, even if
TypeScript/Vitest report green. The parameters are **not** authorized
for default/publication until a comparison with the baseline shows no
tradeoff-induced falls or worse slip.

## Benchmarks

`tests/unit/landingComHoldSweepMujoco.test.ts` executes ten trials
on the corrected-knee `humanoid_step_v2.xml` (75 kg), same initial pose,
start, force, task-space swing servo and timestep. Eight parameter cases
are sampled on the left; two mirrored baseline/candidate cases on right.
Outcomes are printed under the marker `LANDING-COM-HOLD-MUJOCO`.

If this does not improve observed stability, **close the experiment
without merge** and retain the measured negative result for issue #49.
If a variant passes physically, separately validate it on both legs,
backward and lateral steps, perturbations and contact friction changes.
