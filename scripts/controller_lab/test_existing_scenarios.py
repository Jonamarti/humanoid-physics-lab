"""Mechanical contracts for transferring existing attacks to the tracker."""
from pathlib import Path
from types import SimpleNamespace
import unittest
import numpy as np
from scene_forces import CoupledGrip,segment_command,two_link,wrench

class GripTests(unittest.TestCase):
    def test_waits_for_physical_reach_and_misses_without_force(self):
        g=CoupledGrip(dict(engage_s=1.,max_force_n=400.))
        zero=np.zeros(3)
        self.assertEqual(np.linalg.norm(g.force(.9,.001,np.array([.5,0,0]),zero,zero,zero)),0)
        self.assertEqual(g.status,"waiting")
        self.assertEqual(np.linalg.norm(g.force(1.4,.001,np.array([.5,0,0]),zero,zero,zero)),0)
        self.assertEqual(g.status,"missed")
        self.assertEqual(np.linalg.norm(g.force(1.5,.001,zero,zero,zero,zero)),0)
        self.assertEqual(g.status,"missed")
    def test_actual_pair_conserves_force_and_moment(self):
        g=CoupledGrip(dict(max_force_n=400.));z=np.zeros(3)
        g.force(0,.001,np.array([.08,0,0]),z,z,z)
        hand=np.array([.18,0,0]);grip=z.copy()
        force=g.force(.1,.001,hand,z,grip,z)
        self.assertGreater(np.linalg.norm(force),0)
        data=SimpleNamespace(xfrc_applied=np.zeros((3,6)),xipos=np.array([[0,0,0],[0,.1,0],[.1,-.1,.2]]))
        wrench(data,1,grip,force);wrench(data,2,hand,-force)
        np.testing.assert_allclose(data.xfrc_applied[:,:3].sum(axis=0),0,atol=1e-12)
        moments=data.xfrc_applied[:,3:]+np.cross(data.xipos,data.xfrc_applied[:,:3])
        np.testing.assert_allclose(moments.sum(axis=0),0,atol=1e-12)
        np.testing.assert_array_equal(data.xfrc_applied[0],0)
    def test_force_and_actor_power_are_bounded(self):
        g=CoupledGrip(dict(max_force_n=100.,max_power_w=50.));z=np.zeros(3)
        g.force(0,.001,np.array([.08,0,0]),z,z,z)
        force=g.force(.1,.001,np.array([.20,0,0]),np.array([6,0,0]),z,np.array([4,0,0]))
        self.assertLessEqual(np.linalg.norm(force),100+1e-12)
        self.assertLessEqual(np.dot(force,[4,0,0]),50+1e-12)
    def test_separation_breaks_grip_instead_of_pulling_remotely(self):
        g=CoupledGrip({});z=np.zeros(3)
        g.force(0,.001,np.array([.08,0,0]),z,z,z)
        force=g.force(.1,.001,np.array([.3,0,0]),z,z,z)
        self.assertEqual(g.status,"slipped");np.testing.assert_array_equal(force,0)
    def test_held_grip_releases_on_time(self):
        g=CoupledGrip(dict(release_s=.1));z=np.zeros(3)
        g.force(0,.001,np.array([.08,0,0]),z,z,z)
        g.force(.1,.001,np.array([.2,0,0]),z,z,z)
        self.assertEqual(g.status,"released")

class DriverTests(unittest.TestCase):
    def test_finite_force_and_torque_caps_under_large_tracking_error(self):
        p=dict(mass_kg=5.,max_horizontal_force_n=40.,max_vertical_force_n=50.,max_torque_nm=10.)
        ends=np.array([[0.,0,0],[0,0,1.]])
        f,t,_=segment_command(p,np.array([10.,10.,10.]),np.array([20.,20.,20.]),
            np.array([1.,0,0]),np.array([100.,100.,100.]),[.2,.01],ends,ends,.001)
        self.assertLessEqual(np.linalg.norm(f[:2]),40+1e-12)
        self.assertLessEqual(abs(f[2]),50+1e-12)
        self.assertLessEqual(np.linalg.norm(t),10+1e-12)
    def test_stationary_driver_only_supports_its_own_weight(self):
        p=dict(mass_kg=5.,max_horizontal_force_n=400.,max_vertical_force_n=500.,max_torque_nm=100.)
        ends=np.array([[0.,0,0],[0,0,1.]])
        f,t,e=segment_command(p,ends.mean(axis=0),np.zeros(3),np.array([0.,0,1.]),
            np.zeros(3),[.2,.01],ends,ends,.001)
        np.testing.assert_allclose(f,[0,0,49.05],atol=1e-12)
        np.testing.assert_array_equal(t,0);self.assertEqual(e,0)
    def test_two_link_respects_rigid_lengths_and_reach(self):
        root=np.zeros(3)
        for goal in (np.array([.4,0,.1]),np.array([2.,0,0])):
            mid,end=two_link(root,goal,.29,.30,np.array([0.,0,-1.]))
            self.assertAlmostEqual(np.linalg.norm(mid-root),.29)
            self.assertAlmostEqual(np.linalg.norm(end-mid),.30)
            self.assertLessEqual(np.linalg.norm(end-root),.59)
    def test_audit_rejects_material_gain_or_geometry_changes(self):
        from evidence_existing_scenarios import model_equivalence
        a=SimpleNamespace(actuator_biasprm=np.array([[0.,-50.,-3.,0.]]),geom_size=np.ones((1,3)))
        b=SimpleNamespace(actuator_biasprm=a.actuator_biasprm.copy(),geom_size=a.geom_size.copy())
        b.actuator_biasprm[0,2]+=1e-15
        self.assertTrue(model_equivalence(a,b)[0])
        b.actuator_biasprm[0,2]+=.001
        self.assertFalse(model_equivalence(a,b)[0])
        b.actuator_biasprm[:]=a.actuator_biasprm;b.geom_size[0,0]+=.001
        self.assertFalse(model_equivalence(a,b)[0])
        b.geom_size[:]=a.geom_size
        a.opt=SimpleNamespace(timestep=.001);b.opt=SimpleNamespace(timestep=.002)
        self.assertFalse(model_equivalence(a,b)[0])
    def test_existing_tori_is_evaluated_on_every_native_step(self):
        import json
        root=Path(__file__).resolve().parents[2]
        for name in ("outer_reap","leg_drive"):
            path=root/"out/existing-scenarios/exports-native"/(name+".json")
            if not path.exists():self.skipTest("native target export unavailable")
            data=json.loads(path.read_text(encoding="utf-8"))
            self.assertEqual(data["hz"],960)
            self.assertEqual(len(data["targets"]),int(data["source"]["duration_s"]*960)+2)
            self.assertLess(data["target_interpolation_max_error_m"],1e-12)
            original=json.loads((root/"scenarios/physical_refs"/(name+".json")).read_text(encoding="utf-8"))
            self.assertEqual(data["source"],original)
    def test_native_smoke_preserves_published_policy_body_and_motor_count(self):
        import json
        import mujoco
        root=Path(__file__).resolve().parents[2]
        path=root/"out/existing-scenarios/smoke-v2/outer_reap_tracker"
        if not (path/"summary.json").exists():self.skipTest("native smoke not run")
        s=json.loads((path/"summary.json").read_text(encoding="utf-8"))
        m=mujoco.MjModel.from_binary_path(str(path/"model.mjb"))
        self.assertEqual(m.nu,52);self.assertEqual(s["policy_actuators"],52)
        self.assertEqual(s["auxiliary_bodies_excluded_from_policy"],14)
        self.assertEqual(s["max_direct_root_wrench"],0)
        self.assertLess(abs(s["mass_kg"]-42.466827),1e-6)
        self.assertLess(abs(s["attacker"]["mass_kg"]-75),1e-4)
        self.assertTrue(all(m.body(i).name.startswith("projectile_scene_") for i in range(55,m.nbody)))

if __name__=="__main__":unittest.main()
