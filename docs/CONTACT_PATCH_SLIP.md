# Slip at a loaded foot-ground contact versus motion of a sole marker

## Why two different measurements are necessary

Issue #49 reports 11–13 cm of movement of the foot's `*_sole`
**site** after the first loaded landing. The site is a rigid marker on
the foot. It can move horizontally as the foot rocks around a planted
heel/toe, *even when the instantaneous material point in contact with
the ground is stationary*. A moving site is therefore a useful measure
of foot-placement retention, but not a clean measurement of sliding.

For a contact point `p` in world coordinates, foot pose
`(R_t, x_t)`, and previous pose `(R_(t-dt), x_(t-dt))`:

1. Find **current material coordinates** of that contact:
   `q = R_t^T (p - x_t)`.
2. Transform that same material coordinate through the **previous pose**:
   `p_prev = R_(t-dt) q + x_(t-dt)`.
3. Estimate tangential velocity of that *current contact material point*
   on a fixed floor: `v_xy ≈ (p_xy - p_prev_xy) / dt`.
4. Integrate `|v_xy| dt` across loaded samples as an **estimated
   sliding-path length**, not a certified high-frequency impact metric.

The contact point `p` comes from the **actual force-weighted 3D
MuJoCo contact centroid**; the 3D geom rotation and position are from
`MjData.geom_xmat` / `geom_xpos`. These are solver-derived contacts
and physics poses, not animation samples inferred from pixels.

## Validity and limitations

- A pure rotation around a stationary floor contact must yield
  `v_xy = 0` in the mathematical fixture even though a sole marker
  elsewhere moves.
- Actual translation across the floor must show nonzero `v_xy`.
- The contact centroid is force-weighted, not a full center of pressure:
  this estimate does not include distributed contact patch deformation,
  precise instantaneous material point, or a continuous-time force
  trajectory. A 20 ms sample misses substep stick-slip and impact
  oscillations; the true accumulated path may differ.
- The current diagnostic assumes a **stationary floor**. For moving
  surfaces, subtract floor material velocity at contact.
- Compare with site drift, foot joint angles, posture, contact forces,
  target errors and actual completed-step stability. **Do not rewrite
  pass/fail thresholds based solely on a smaller slip estimate.**
- The tool is pure numeric code with unit tests for a stationary
  rotational pivot and translation, and a headless MuJoCo comparison.

## Test protocol

`tests/unit/contactPatchSlipMujoco.test.ts` runs the same original
75 kg diagonal step with swing servo 0 % and 20 %, each for 5.2 s,
without changing any dynamics. Outputs the time of first re-contact,
measured contact-patch sliding-path estimate, peak tangential contact
speed, sole-site drift, friction and final target error. Tests *record*
actual success or failure rather than forcing passes.

## Measured results (MuJoCo 75 kg, CI 42775437755, 10 Nov 2026)

The new 3D material-contact-point estimator provides a second measure,
not a replacement for the movement of the sole marker:

| Swing servo | Outcome | Loaded re-contact time | Estimated integrated tangential contact motion | Sole-site drift | Peak patch speed |
| --- | --- | --- | --- | --- | --- |
| 0% | ABORT | 2.14 s | **16.01 cm** | 12.67 cm | 0.796 m/s |
| 20% | ABORT | 1.82 s | **17.46 cm** | 11.05 cm | 0.791 m/s |

Both runs reach friction utilization ~1.00 at some sampled contacts.
The integral uses a 20 ms fixed cadence, starts at the first *verified*
landing and accumulates speed only while the receiving foot carries >40 N.

**Interpretation:** in these scenarios, foot rotation does *not* explain
away the observed problem. The contact material velocity is nonzero and
the integrated path estimate is actually *larger* than the net drift of
the sole marker. The latter is a maximum displacement relative to the
first landing, whereas the integral sums the absolute path along the
contact and can exceed any net displacement. They are not interchangeable
metrics and should not be subtracted to infer a rolling contribution.

These measurements strengthen the evidence of unwanted tangential
motion, but they are not a ground-truth high-frequency optical slip
measurement. Therefore **leave the step-acceptance criteria and the
physical controller unchanged** until a jointly controlled step meets
contact and COM stability conditions.

CI: https://github.com/Jonamarti/humanoid-physics-lab/actions/runs/42775437755
