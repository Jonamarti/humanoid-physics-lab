"""Rocker, posture and damping contracts; no perceptual naturalness score."""
from pathlib import Path
import unittest
import numpy as np
import mujoco
from scipy.spatial.transform import Rotation
from heel_toe import ReferenceBuilder,roll_angle,rollover,lean_reference,gaze_reference
from upper_damping import UpperDamping
from landing_feedback import landing_target
from metrics import physical_fingerprint

ROOT=Path(__file__).resolve().parents[2]

class PhaseTests(unittest.TestCase):
    def test_rocker_profile_has_no_jumps(self):
        for phase in (-.35,.12,.40,.72,.94,1.24,1.65,2.12):
            t=1+phase*.5
            a=roll_angle(t-1e-7,0,.5,6,heel_strike=-.35)
            b=roll_angle(t+1e-7,0,.5,6,heel_strike=-.35)
            self.assertLess(abs(a-b),1e-4)
    def test_stops_rocking_after_final_step(self):
        self.assertEqual(roll_angle(8,1,.5,6),0.)
        self.assertEqual(roll_angle(0,0,.5,6),0.)
    def test_heel_contact_has_toes_up_and_toeoff_heel_up(self):
        self.assertLess(roll_angle(1+.94*.5,0,.5,6),0)
        self.assertGreater(roll_angle(1+.1*.5,0,.5,6),0)
    def test_additive_residual_respects_requested_budget(self):
        for e in np.linspace(-3,3,41):
            v=landing_target(.2,0,e,.8,.2,1/30,(-1,1),.1,"additive")
            self.assertLessEqual(abs(v-.2),.08000000001)
    def test_unloaded_target_slew_is_limited(self):
        for raw in np.linspace(-2,2,10):
            for e in (-3.,3.):
                v=landing_target(raw,0,e,1.,.1,1/30,(-.5,.5),.1,"replace")
                self.assertLessEqual(abs(v-.1),4/30+1e-12)
    def test_unloaded_target_uses_joint_range(self):
        v=landing_target(0,0,8,1,0,1.,(-.5,.5),.1,"replace")
        self.assertEqual(v,.5)

class DampingTests(unittest.TestCase):
    def setUp(self):
        self.m=mujoco.MjModel.from_xml_string('''<mujoco><worldbody><body name="chest">
          <joint name="Spine1_x" axis="0 1 0"/><geom type="box" size=".1 .1 .5" pos="0 0 .5" mass="10"/>
          </body></worldbody><actuator><general joint="Spine1_x" gaintype="fixed" gainprm="50"
          biastype="affine" biasprm="0 -50 -3" forcelimited="true" forcerange="-10 10"/></actuator></mujoco>''')
        self.d=mujoco.MjData(self.m);mujoco.mj_forward(self.m,self.d)
    def test_only_velocity_bias_changes_and_restores(self):
        m=self.m;original=physical_fingerprint(m);bias=m.actuator_biasprm.copy();gain=m.actuator_gainprm.copy()
        f=UpperDamping();notes=f(dict(upper_damping=True),m,self.d)
        np.testing.assert_array_equal(gain,m.actuator_gainprm)
        np.testing.assert_array_equal(bias[:,:2],m.actuator_biasprm[:,:2])
        self.assertGreaterEqual(-m.actuator_biasprm[0,2],3.)
        self.assertLessEqual(-m.actuator_biasprm[0,2],6.)
        self.assertEqual(notes["target_isolated_joint_damping_ratio"],.85)
        f({},m,self.d)
        self.assertEqual(original,physical_fingerprint(m))
    def test_uncapped_hip_uses_declared_inertia_ratio(self):
        # Same mechanical fixture, with a hip joint label rather than spine.
        m=mujoco.MjModel.from_xml_string('<mujoco><worldbody><body><joint name="LeftLeg_x" axis="0 1 0"/><geom type="box" size=".1 .1 .5" pos="0 0 .5" mass="10"/></body></worldbody><actuator><general joint="LeftLeg_x" gainprm="50" biastype="affine" biasprm="0 -50 -3" forcelimited="true" forcerange="-10 10"/></actuator></mujoco>')
        d=mujoco.MjData(m);mujoco.mj_forward(m,d)
        notes=UpperDamping()(dict(leg_absorption=True,inertia_matched_hip=True),m,d)
        inertia=notes["joints"][0]["locked_joint_diagonal_inertia_kg_m2"]
        self.assertAlmostEqual(-m.actuator_biasprm[0,2],2*.85*np.sqrt(50*inertia))
        self.assertEqual(m.actuator_forcerange[0,1],10.)

    def test_extra_damping_is_dissipative_inside_effort_caps(self):
        m,d=self.m,self.d;f=UpperDamping();original=m.actuator_biasprm.copy()
        f(dict(upper_damping=True),m,d);updated=m.actuator_biasprm.copy()
        for velocity in np.linspace(-6,6,21):
            for control in (-1.,0.,1.):
                d.qpos[0]=.2;d.qvel[0]=velocity;d.ctrl[0]=control
                m.actuator_biasprm[:]=original;mujoco.mj_forward(m,d);before=d.actuator_force[0]
                m.actuator_biasprm[:]=updated;mujoco.mj_forward(m,d);after=d.actuator_force[0]
                self.assertLessEqual((after-before)*velocity,1e-10)
                self.assertLessEqual(abs(after),10+1e-10)

@unittest.skipUnless((ROOT/"out/external/protomotions/protomotions/data/assets/mjcf/anatomic_soma.xml").exists(),"source model unavailable")
class ReferenceTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.b=ReferenceBuilder(ROOT)
        cls.flat,cls.meta=cls.b.walking(.5,4,lift=.18,support_center=True,forward_sync=True,settle=3.,dynamic_root=True,natural_swing=True)
        cls.rolled,cls.rm=rollover(cls.b,cls.flat,cls.meta,heel_strike=-.35)
        cls.lean,cls.lm=lean_reference(cls.b,cls.rolled,cls.rm)
    def test_contact_calibration_is_explicit_and_reversible(self):
        from heel_toe_model import HeelToeModel
        m,d=self.b.m,self.b.d;original=physical_fingerprint(m);friction=m.geom_friction.copy()
        configure=HeelToeModel();notes=configure(dict(foot_contact_tau=.01),m,d)
        np.testing.assert_array_equal(friction,m.geom_friction)
        self.assertEqual(notes["foot_contact"]["time_constant"],.01)
        configure({},m,d)
        self.assertEqual(original,physical_fingerprint(m))

    def test_reference_is_reachable(self):
        self.assertLess(self.rm["max_ik_error_m"],.002)
        self.assertLess(self.lm["max_ik_error_m"],.002)
    def test_lean_preserves_world_foot_goals(self):
        indices=[list(self.b.robot.kinematic_info.body_names).index(s+"Foot") for s in ("Left","Right")]
        self.assertLess(np.linalg.norm(self.lean["gts"][:,indices]-self.rolled["gts"][:,indices],axis=-1).max(),.002)
    def test_initial_physical_pose_is_preserved(self):
        np.testing.assert_allclose(self.flat["dps"][0],self.lean["dps"][0],atol=1e-8)
        np.testing.assert_allclose(self.flat["gts"][0],self.lean["gts"][0],atol=1e-8)
    def test_counterrotation_keeps_head_reference_level(self):
        gaze,_=gaze_reference(self.b,self.lean,self.lm)
        i=list(self.b.robot.kinematic_info.body_names).index("Head")
        pitch=Rotation.from_quat(gaze["grs"][:,i]).as_euler("xyz")[:,1]
        self.assertLess(np.abs(pitch).max(),1e-5)

if __name__=="__main__":unittest.main()
