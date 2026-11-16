"""Contract tests for physical timing, contact diagnostics and frozen inference."""
import os
from pathlib import Path
import tempfile
import unittest
import numpy as np
import mujoco
from gait import ReferenceBuilder,pulse_force,save_pack
from gait_suite import standing
from recovery import capture_error
from contact_slip import tangent_velocity
from metrics import physical_fingerprint

ROOT=Path(__file__).resolve().parents[2]

class PulseTests(unittest.TestCase):
    def test_exact_impulse_without_grid_aligned_endpoints(self):
        dt=1/960
        for duration in (.08,.18):
            for impulse in (8.,20.,36.):
                direction=np.array([0.,-1.,0.])
                total=sum((pulse_force(t,dt,.137,duration,impulse,direction)*dt
                    for t in np.arange(1000)*dt),start=np.zeros(3))
                np.testing.assert_allclose(total,direction*impulse,atol=1e-10,rtol=0)
    def test_zero_outside_pulse(self):
        np.testing.assert_array_equal(pulse_force(0.,.001,1.,.08,20.,[1,0,0]),[0,0,0])
    def test_rejects_nonunit_direction(self):
        with self.assertRaises(ValueError):pulse_force(0,.01,0,.08,20,[2,0,0])
    def test_rejects_negative_duration(self):
        with self.assertRaises(ValueError):pulse_force(0,.01,0,-.08,20,[1,0,0])

class BalanceTests(unittest.TestCase):
    def test_capture_point_accounts_for_observed_velocity(self):
        feet=np.array([[0,.1,0],[0,-.1,0]])
        cp,error=capture_error([0,0,1],[0,0,0],feet,1)
        np.testing.assert_array_equal(error,[0,0])
        cp,error=capture_error([0,0,1],[-1,0,0],feet,1)
        self.assertLess(error[0],-.2)
    def test_capture_error_translation_invariant(self):
        feet=np.array([[0,.1,0],[0,-.1,0]])
        a,e=capture_error([0,0,1],[.8,.2,0],feet,1)
        offset=np.array([200.,-300.,0.])
        b,f=capture_error(np.array([0,0,1])+offset,[.8,.2,0],feet+offset,1)
        np.testing.assert_allclose(e,f,atol=1e-12)

class ContactTests(unittest.TestCase):
    def setUp(self):
        self.m=mujoco.MjModel.from_xml_string('<mujoco><worldbody><body name="foot" pos="0 0 .1"><freejoint/><geom type="sphere" size=".1" mass="1"/></body></worldbody></mujoco>')
        self.d=mujoco.MjData(self.m)
    def velocity(self):
        mujoco.mj_forward(self.m,self.d)
        return tangent_velocity(self.m,self.d,self.m.body("foot").id,np.array([0.,0,0]),np.array([0.,0,1.]))
    def test_rolling_body_origin_moves_but_contact_material_does_not_slip(self):
        self.d.qvel[0]=.2;self.d.qvel[4]=2.
        np.testing.assert_allclose(self.velocity(),[0,0,0],atol=1e-12)
    def test_translation_is_material_slip(self):
        self.d.qvel[0]=.2
        np.testing.assert_allclose(self.velocity(),[.2,0,0],atol=1e-12)
    def test_normal_velocity_is_not_tangential_slip(self):
        self.d.qvel[2]=.2
        np.testing.assert_allclose(self.velocity(),[0,0,0],atol=1e-12)
    def test_fingerprint_includes_friction_solver(self):
        before=physical_fingerprint(self.m)
        self.m.opt.noslip_iterations=3
        self.assertNotEqual(before,physical_fingerprint(self.m))

@unittest.skipUnless((ROOT/"out/external/protomotions/protomotions/data/assets/mjcf/anatomic_soma.xml").exists(),"native ProtoMotions assets unavailable")
class ReferenceTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):cls.builder=ReferenceBuilder(ROOT)
    def test_standing_reference_has_zero_velocity(self):
        a=standing(self.builder)
        for k in ("gvs","gavs","dvs"):np.testing.assert_array_equal(a[k],0.)
        np.testing.assert_array_equal(a["gts"][0],a["gts"][-1])
    def test_real_speed_changes_reference_velocities_not_playback(self):
        slow,sm=self.builder.walking(2.4,2,dynamic_root=True,natural_swing=True)
        fast,fm=self.builder.walking(.8,2,dynamic_root=True,natural_swing=True)
        self.assertEqual(sm["steps_requested"],fm["steps_requested"])
        self.assertGreater(len(slow["gts"]),len(fast["gts"]))
        # Redundant leg IK may choose different knee positions; compare the requested root/foot endpoints.
        ids=[list(self.builder.robot.kinematic_info.body_names).index(n) for n in ("Hips","LeftFoot","RightFoot")]
        np.testing.assert_allclose(slow["gts"][-1,ids],fast["gts"][-1,ids],atol=1e-4)
        self.assertGreater(np.max(abs(fast["gvs"])),np.max(abs(slow["gvs"]))*1.7)
        self.assertLess(fm["max_ik_error_m"],.002)
    def test_loaded_policy_is_frozen_for_inference(self):
        import torch
        from proto_adapter import create_agent,reset_clip,action
        previous=Path.cwd()
        with tempfile.TemporaryDirectory(prefix="hpl-inference-test-") as tmp:
            file=Path(tmp)/"standing.pt";save_pack([standing(self.builder)],file)
            agent=create_agent(ROOT,file)
            try:
                self.assertTrue(all(not m.training for m in agent.model.modules()))
                buffers={n:v.clone() for n,v in agent.model.named_buffers() if "running_obs_norm" in n}
                self.assertTrue(buffers)
                obs=reset_clip(agent,0)
                for k in range(3):action(agent,obs,k)
                for n,v in agent.model.named_buffers():
                    if n in buffers:self.assertTrue(torch.equal(v,buffers[n]),n)
            finally:agent.env.simulator.shutdown();os.chdir(previous)

if __name__=="__main__":unittest.main()
