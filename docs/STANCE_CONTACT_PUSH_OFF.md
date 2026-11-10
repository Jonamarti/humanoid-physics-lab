# Experiment: planted material contact, rear-foot push-off and real GRF

Status: **experimental draft; no improvement in MuJoCo walking demonstrated by this branch yet**.
Related: [#49](https://github.com/Jonamarti/humanoid-physics-lab/issues/49),
[#13](https://github.com/Jonamarti/humanoid-physics-lab/issues/13),
[#39](https://github.com/Jonamarti/humanoid-physics-lab/issues/39).

## Physical model and user's observation

In static friction, the instantaneous **material point** of the foot currently touching a stationary
floor has near-zero tangential velocity. A heel-to-toe roll can change the **location of contact** and
move a marker fixed to the rigid sole without that particular material point slipping. There is no
world-fixed foot anchor. If friction capacity is exhausted, slipping is an expected solver result.

During the propulsive part of forward stance, the rear foot attempts to drive the ground **backward**;
the solved tangential friction reaction on that foot is then **forward**. Loading and early landing
can instead **brake** the body: sign must be measured, never assumed for an entire stance phase.
Actual foot-floor normal and tangential contact forces come from MuJoCo's solved constraints,
not the magnitude of a controller request.

## New opt-in control, off by default

The viewer has a `Contacto y empuje del pie posterior` selection: 0 (default), 0.25, 0.5, or 1.
`enableExperimentalStepTrial` adds a **seventh** optional `stanceContactScale` argument.
Zero follows the original path: no extra observations and no new torque.

For enabled isolated steps, the 2 ms physics loop:
1. Reads MuJoCo's solved floor/foot normal, tangential reaction, friction utilization and the
   smaller sliding coefficient of the two contact tangents. The latter is derived from **actual**
   `contact.friction` and aggregated conservatively across loaded contacts.
2. Finds the current 3D force-weighted ground-contact centroid on the **supporting** foot,
   reconstructs its own material point in the previous measured foot pose and estimates its
   tangential velocity relative to the stationary floor. This is different from sole-site drift.
3. If contact is loaded (>=40 N), friction reserve is available and the FSM is in LIFT/SWING/LAND,
   it requests modest virtual contact-point force opposing measured slip on the **rear support foot**.
   In late **forward** SWING/LAND only, with COM beyond the contact, it also requests a modest
   **rearward** foot effort. During RECOVER it uses slip-only feedback on **both loaded feet**,
   so the newly landed foot can roll or stabilize without any programmed push-off.
   New `MaterialContactTracker` requires continuous loaded samples, engages at 60 N,
   releases below 35 N, smooths the *material-point* tangential velocity and ramps the
   requested force over ten simulation ticks; the planner also fades torque between
   40 and 100 N of normal load. This avoids full correction on first impact.
   A small load-dependent aggregate friction-reserve *proxy* clips that request; the true
   solver still determines stick, slip and ground reaction.
4. Converts the virtual contact effort to clipped **internal** hip/knee/ankle torques via `J^T F`.
   No external pelvis/root force, no repositioning the foot, no constraint welding and no
   modification of collision or surface friction.
5. The UI separates **actual signed forward GRF** from the **virtual rearward request**, and displays
   estimated contact-material sliding speed. `materialContactLocalM` additionally measures the
   force-weighted contact position in the foot reference frame, so heel-to-toe contact migration
   can be reported **separately** from true material-point sliding.

**Limits:** a finite-difference material-point speed can be noisy near impact; a centroid is not
a full pressure center/6D wrench; the aggregate friction margin is not a per-contact cone
certificate; joints may saturate and the sign of the measured GRF may differ from requested
virtual effort. All thresholds in the existing step-completion FSM remain unchanged.

## Verification and disposition

**Execution note (10 Nov 2026):** the edits have been published to an isolated GitHub branch,
with unit and opt-in MuJoCo benchmark source. A local complete TypeScript/ESLint/Vitest/MuJoCo
run has **not yet been executed** in this environment: attempting to clone the public repository
from the container failed because GitHub DNS is inaccessible. GitHub Actions remain disabled
for this experiment to avoid consuming credits. No physical performance results or stability
gains are claimed for this branch.


Run existing tests/build locally with `npm run check`. The extra real-physics A/B is **opt-in**
to avoid CI credit consumption:

```bash
RUN_LONG_BENCHMARKS=1 npx vitest run tests/unit/stanceContactMujoco.test.ts
RUN_STANCE_FRICTION_SWEEP=1 npx vitest run tests/unit/stanceFrictionSensitivityMujoco.test.ts
```

This experiment measures for **both legs**, on identical 75 kg MuJoCo
`humanoid_step_v2.xml` physics: 12 cm forward, 52% swing servo, 48% knee guard,
with stance-contact strength 0 and 0.35.

The recorded evidence includes:
- Positive and negative longitudinal **measured** GRF impulses on the rear foot (N s).
- Backward contact-force **intent** integrated separately (NOT a GRF measurement).
- Cumulative material-patch path and peak tangential material-contact speed **for both the rear support foot and the newly landed foot** at 2 ms sampling.
- Contact migration range in foot-local X (heel to toe), separate from material slip.
- Site-marker displacement, placement error, loads and final physical FSM result.
- Strict `COMPLETE` requirements: true bilateral contact, <=2 cm sole drift,
  <=5 cm target error and >=1 s of stable balance.

**Decision rule:** retain the feature only if repeated physical trials show better stick/slip
behavior and/or appropriate forward GRF **without** degraded completion/stability across friction
0.9/1.0/1.1, both feet and different step lengths. A numerically correct unit suite is
*not* proof of propulsion or walking. Until tested, keep this as a draft, retain 0 as
the safe default, and do **not** merge into `main` or wire to 500 N perturbations.

Known prior baseline (without this change): both feet reached `COMPLETE` at friction 1.0
in an isolated calibrated scenario; the same controller failed at friction 0.9 and 1.1.
See `docs/GAIT_FIRST_COMPLETE.md`.
