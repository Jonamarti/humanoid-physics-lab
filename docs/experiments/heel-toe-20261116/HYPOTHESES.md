# Ordered hypotheses before changing gait

Baseline tag codex/protomotions-flat-foot-baseline-20261116, commit2b8d963. Preserve its recordings/videos in the preceding worktree; no overwrite or merge.

1. Foot orientation returns to zero as swing ends and MTP never flexes: this explicitly requests flat landing. Change only foot goals to heel contact, foot lowering and forefoot pivot with MTP articulation. Keep cadence0.5s, six steps, root path, gains, contact properties, seed and full initial state identical. Reject unreachable IK>2mm, loss of alternation/stability or worse penetration/work/slip. Native contact loads/COP decide whether rollover occurred, not the reference angles.
2. Quantify body motion relative to foot advance and4-15Hz head/chest accelerations. Test preparatory forward COM motion and small whole-body lean separately after the contact comparison. Policy sees25 future frames (0.833s), so a fixed timing offset alone does not guarantee body-first execution; a separate preparatory clip may be needed.
3. Only if contact/phase correction leaves high-frequency upper-body oscillation, test a dissipative bounded upper-joint damping residual based on measured inertia/velocity. Do not increase lateral hip sway or freeze the head for appearance. No motor-gain grid.

Measurements: solved heel/forefoot forces and local COP960Hz, body kinematics120Hz, contact-to-upper-body vibration, actual COM/foot onset, motor work and unchanged physical artifact gates. Replay never integrates physics. References are IK goals, never runtime pose writes. Biological walking is not reduced to bending the waist; APA/CoP precede gait initiation (https://pubmed.ncbi.nlm.nih.gov/28651215/).
