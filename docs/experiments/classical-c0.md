# Classical C0 — 2026-11-27 (#114 / PR #115)

## Reproduction status
SIMBICON-inspired primitives, independent implementation from the published idea; **not** a calibrated reproduction. Original paper: https://www.microsoft.com/en-us/research/publication/simbicon-simple-biped-locomotion-control/ . GENBICON and Cartwheel sources remain indexed in #114; upstream demo and licensing audit still pending. No new ML, external root forces or changes to existing production controllers.

## C0a baseline
Commit 37eae0dbc08ab3167b516c285695bdc079d7cc53; [CI run](https://github.com/Jonamarti/humanoid-physics-lab/actions/runs/46061048878). 710 tests pass, 9 skipped, 3 todo. Identical repeat standing recordings, no root torque, limits respected. Reduced body: 75 kg, 10 controlled hinge DOFs, rigid trunk and feet, 500 Hz, 3 s. This is not a natural humanoid rig.

|Mode|First fall s|Steps|Forward displacement m|Min pelvis m|Max trunk tilt deg|Final COM speed m/s|Loaded foot center drift m|
|---|---:|---:|---:|---:|---:|---:|---:|
|passive|0.650|0|-0.499|0.104|92.76|0.000083|0.0118|
|stand|none within 3 s|0|-0.634|0.455|39.98|2.549|0.0952|
|walk|1.798|0|-0.037|0.141|100.65|0.000104|0.1497|

Standing fails physically despite no fall threshold crossing; walk times out and falls. Loaded center drift is **not material slip** (rotation changes marker position). No claim of naturalness. Full traces not uploaded by CI yet.

## C0b hypothesis
Same model, timestep, initial state and torque bounds; new balance mode only. C0a stand mode remains unchanged for paired evaluation. Add fixed initial COM reference, LIPM-derived desired CoP, bounded ankle torque only when contact is loaded, symmetric hip reaction in bilateral stance. Nominal weight distribution avoids impact-force noise. This combines changes as a single contact-aware standing policy; if it improves, ablate hip symmetry and ankle feedback separately. Not general whole-body control; gains are initial hypotheses. No parameter sweep. Report all four modes in CI. Compare final COM speed, pelvis height, tilt, drift and first fall. C1 acceptance remains 60 s plus other #114 criteria, not merely passing 3 s.

Next: inspect CI metrics, preserve negative results, generate actual replay files and video from recorded states. GitHub Pages deploys main only; draft PR stays isolated. No workflow changes or automatic merge authorized.
