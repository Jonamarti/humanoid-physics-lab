# V3 reactive recovery: stance-foot loading failure (2026-11-11)

## Reproduced A/B, no improvement from retaining shoulder correction

Baseline: main at `c719ef44`. Experimental branch: `experiment/v3-arm-transfer-ab-20261111`.
[Paired 4-direction physical MuJoCo run](https://github.com/Jonamarti/humanoid-physics-lab/actions/runs/43005383353).
Same v3 MJCF, 75 kg, 160 ms half-sine external sternum force. 6 seconds per trial,
four directions, baseline and opt-in shoulder-only redistribution during SHIFT_WEIGHT.

| Push | Baseline physical result | Shoulders during transfer | Baseline first FALL | Experimental first FALL |
|---|---|---|---:|---:|
| forward 200 N | FALL, no LIFT | FALL, no LIFT | 3.28 s | 3.30 s |
| backward 160 N | ABORT then FALL, no LIFT | ABORT then FALL, no LIFT | 2.78 s | 2.78 s |
| left 300 N | FALL, no LIFT | FALL, no LIFT | 2.54 s | 2.54 s |
| right 300 N | FALL, no LIFT | FALL, no LIFT | 2.54 s | 2.54 s |

The opt-in control did not recover any step or maintain stable support,
and is therefore **not merged or enabled**. Passing observational tests
does not certify recovery.

## Confirmed physical blockade in the 14-case directional matrix

[Run 43005483025](https://github.com/Jonamarti/humanoid-physics-lab/actions/runs/43005483025)
repeated the full 12 single-push and 2 chained-pulse scenarios using the
FSM's own 50 Hz `preLiftReadiness` and `preLiftReadyDurationS`.

For **right 300 N**, the first SHIFT_WEIGHT sample at t=1.54 s shows:
- Chosen stance foot: left, **0 N solved vertical support**.
- Selected swing foot: right, **603.40 N solved vertical load**.
- COM projection still inside the *loaded-foot union*, but that region is
  on the intended **swing** foot, not the selected **stance** foot.
- Neither the support-load nor the swing-unloaded release gate passes.

For **left 300 N**, the same reversed-leg pattern occurs. Both lateral
strong cases show zero support-fraction maxima across 50 SHIFT_WEIGHT samples
per direction; no readiness dwell and no LIFT. They end in FALL at ~2.54 s.
For forward and backward strong cases, the support/swing and COM-projection
gates repeatedly fail before the later root tilt or pelvis-height failure.
A transient support fraction of 1 while both loads collapse is **not**
sustained or sufficient evidence of a recoverable transfer.

## Decision and next physics hypothesis

Do not reduce the support-load, COM, touchdown or landing gates. Do not
change friction, free-root forces or kinematic body placement. The problem
is **upstream** of foot lift: load is on the foot selected to swing while
the chosen stance foot is unsupported. Next investigate support-leg choice
and lateral center-of-mass timing, with explicit measurement at the instant
the recovery intent is created, before SHIFT_WEIGHT starts. Only test a
controller alternative when a physically loaded stance foot can be maintained.

This PR introduces a regression for the unloaded-stance/overloaded-swing
scenario and diagnostic counters in the opt-in benchmark. It does not
change the physical controller or declare a new accepted gait capability.
