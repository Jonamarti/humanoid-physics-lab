# Contact-aware knee extension: first COMPLETE, not yet robust walking

## Mechanism

The v2 gait profile folds the free knee behind the thigh during LIFT.
Previously it extended the knee during SWING using only elapsed seconds.
MuJoCo traces showed the toe lowering after about 5-6 cm of a requested
12 cm step, creating premature contact and subsequent tangential slip.

The opt-in `swingExtensionGuard` (0..1) mixes the original time-based
extension with a ramp controlled by **measured signed progress of the
swing foot toward its goal** (sampled from actual solver world position).
This changes only internal knee-target torques in the experimental step:
no pelvis/root control, no modified friction, no fake contact.

- Guard 0 is the exact legacy temporal policy.
- Guard 1 often holds the knee bent so long that the foot cannot advance
  and the humanoid falls. It is **not** a better default.
- Moderate guards can land the foot more gently without premature
  extension. The input is a dimensionless control gain, not a muscle
  strength or probability.

## Reproducible first complete step (MuJoCo 75kg)

[Precise 16-run tuning CI](https://github.com/Jonamarti/humanoid-physics-lab/actions/runs/42890512964)
and [six mirrored/friction checks](https://github.com/Jonamarti/humanoid-physics-lab/actions/runs/42890756896).

Parameters: `humanoid_step_v2.xml`, 0.002s integration, 12cm forward,
coordinated posture 1, 52% swing Cartesian servo, 48% extension guard,
normal MuJoCo friction `1.0 0.025 0.002`. The experiment begins at
0.8s and runs for 5.2s, with no root actuation.

| Criterion | Left | Right | Acceptance |
| --- | ---: | ---: | --- |
| Swing foot planted and loaded | yes | yes | required |
| Sole-marker post-contact drift | 1.22 cm | 1.21 cm | <=2 cm |
| Support foot drift | 0.59 cm | 0.59 cm | <=2 cm |
| Final XY foot target error | 4.96 cm | 4.98 cm | <=5 cm |
| Continuous stable bilateral support | 2.74s | 2.74s | >=1s |
| Final FSM | COMPLETE | COMPLETE | COMPLETE |

This is the first condition in which **the existing strict state
machine** reports COMPLETE after measured ground contact and sustained
balance for both feet. No pass/fail thresholds were changed.

## Very important failure: lack of robustness

The exact same controller does **not** generalize to modest friction
changes in this initial sample:

| Ground friction scalar | Left outcome | Right outcome |
| --- | --- | --- |
| 0.9 | FALL | ABORT |
| 1.0 | COMPLETE | COMPLETE |
| 1.1 | ABORT | ABORT |

The friction sweep changes only the physical model's `friction`
coefficients, not controller gains or expected outcomes. Thus this is a
**calibrated demonstration** of an isolated forward step, not a reliable
gait, a broadly tuned controller or a biomechanical validation.

The web profile is explicitly labelled experimental, with default
frontal 12cm and the above gains to reproduce this one maneuver.
It remains separate from the nominal 500N push test and manual
multi-force editor, which continue using `humanoid_v1.xml`.
Automatic stepping on perturbations is still unimplemented.

## Next engineering milestones

1. Robust lateral COM/load transfer and friction-constrained planting,
   with real sensor feedback and hysteresis.
2. Validate many masses, step lengths, initial COM velocities, foot
   contacts, timestep and friction values; repeat both legs.
3. Couple a first stable step to the existing direction-sensitive
   perturbation planner, no instant 500N hard-coded magic.
4. Only then attempt alternating strides and continuous walking/running.
