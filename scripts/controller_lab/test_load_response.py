"""Physical contracts for phase control, load detection and reproducible replay."""
import unittest
from types import SimpleNamespace
import numpy as np
import mujoco
from load_response import acceptance,priming_depth,LoadAcceptance
from weight_acceptance import SupportedHeelDetector,impedance_envelope,set_impedance

class PhaseTests(unittest.TestCase):
    def test_preserved_recordings_cannot_be_overwritten(self):
        import tempfile
        from pathlib import Path
        from load_response import experiment_output
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp);(root/"saved").mkdir();(root/"saved/recording.npz").write_bytes(b"saved")
            with self.assertRaises(FileExistsError):experiment_output(root,"saved")
            self.assertEqual(experiment_output(root,"saved","new"),root/"new")

    def test_pulses_have_finite_support_and_no_discontinuity(self):
        for f,end in ((acceptance,.38),(impedance_envelope,.26)):
            t=np.linspace(-.2,.8,10001)
            y=np.array([f(x,0.) for x in t])
            self.assertGreaterEqual(y.min(),-1e-12)
            self.assertLessEqual(y.max(),1+1e-12)
            np.testing.assert_allclose([f(-.01,0.),f(end+.01,0.)],[0,0],atol=1e-12)
            self.assertLess(np.max(np.abs(np.diff(y))),.01)
    def test_prepared_flexion_precedes_contact_and_returns_to_rest(self):
        from prepared_acceptance import prepared_depth
        t=np.arange(0,9,1/960);depth=prepared_depth(t,.5,6,lead=.18)
        self.assertLessEqual(depth.max(),.03+1e-12)
        self.assertGreater(depth[np.argmin(abs(t-1.48))],.029)
        self.assertEqual(depth[0],0)
        self.assertEqual(depth[-1],0)
    def test_saved_impedance_is_replayed_with_actual_gains(self):
        import sys,importlib
        original_step=mujoco.mj_step
        try:
            sys.modules.pop("render_rollover",None)
            replay=importlib.import_module("render_rollover")
        finally:mujoco.mj_step=original_step
        apply_impedance=replay.apply_impedance
        m=SimpleNamespace(actuator_gainprm=np.ones((4,3)),actuator_biasprm=np.zeros((4,3)))
        case=dict(m=m,impedance=dict(actuator_ids=[0,1,2,3],columns=["kd_left_hip"]),
            schedule=np.array([[1.,10,20,30,40,2,3,4,5,0,0]]))
        apply_impedance(case,1.)
        np.testing.assert_array_equal(m.actuator_gainprm[:,0],[10,20,30,40])
        np.testing.assert_array_equal(m.actuator_biasprm[:,1],[-10,-20,-30,-40])
        np.testing.assert_array_equal(m.actuator_biasprm[:,2],[-2,-3,-4,-5])
    def test_anticipation_does_not_accumulate_across_steps(self):
        t=np.arange(0,9,1/960)
        y=priming_depth(t,.5,6)
        self.assertLessEqual(y.max(),.003+1e-12)
        self.assertEqual(y[0],0)
        self.assertEqual(y[-1],0)
    def test_contact_response_cannot_modify_past_reference(self):
        p=LoadAcceptance.__new__(LoadAcceptance)
        p.enabled=True;p.events=[];p.index=0;p.unloaded=np.zeros(2);p.armed=[False,False]
        p.last_event=np.full(2,-10.);p.case={"qualified_lifts":True};p.depth=.008
        p.env=SimpleNamespace(reference=SimpleNamespace(times=[1.54]))
        p.rebuild=lambda:None
        tel=SimpleNamespace(contact_rows=[[1.52,100,400,100,0]],rows=[[0]*9+[1,0,0]],
            mass=42.,initial_height=1.,m=SimpleNamespace(opt=SimpleNamespace(timestep=1/960)),
            d=SimpleNamespace(time=1.54),heel_events=[dict(foot=0,contact_s=1.52,load_n=100,heel_n=100)])
        self.assertTrue(p.update(tel))
        self.assertGreaterEqual(p.events[0]["goal_start_s"],p.events[0]["contact_s"])
        np.testing.assert_array_equal(acceptance(np.array([1.4,1.52,1.54]),1.54),[0,0,0])
        self.assertFalse(p.update(tel))
        self.assertEqual(len(p.events),1)

class DetectionTests(unittest.TestCase):
    def test_requires_clearance_support_and_dwell_then_real_heel_load(self):
        d=SupportedHeelDetector()
        for k in range(20):d.update(k*.001,[0,100],[0,0],[.04,0],100,.001)
        d.update(.021,[100,0],[100,0],[0,0],100,.001)
        self.assertEqual(len(d.events),0)
        for k in range(40):d.update(.1+k*.001,[0,100],[0,0],[.04,0],100,.001)
        d.update(.15,[100,0],[0,100],[0,0],100,.001)
        self.assertEqual(len(d.events),0)
        d.update(.16,[100,0],[100,0],[0,0],100,.001)
        self.assertEqual(len(d.events),1)
        d.update(.6,[100,0],[100,0],[0,0],100,.001)
        self.assertEqual(len(d.events),1)
    def test_unloaded_but_grounded_foot_cannot_arm_reflex(self):
        d=SupportedHeelDetector()
        for k in range(100):d.update(k*.001,[0,100],[0,0],[0,0],100,.001)
        d.update(.2,[100,0],[100,0],[0,0],100,.001)
        self.assertFalse(d.events)
    def test_opposite_leg_must_support_the_swing(self):
        d=SupportedHeelDetector()
        for k in range(100):d.update(k*.001,[0,0],[0,0],[.04,.04],100,.001)
        d.update(.2,[100,0],[100,0],[0,0],100,.001)
        self.assertFalse(d.events)

class ImpedanceTests(unittest.TestCase):
    def setUp(self):
        self.m=mujoco.MjModel.from_xml_string('<mujoco><worldbody><body pos="0 0 1"><joint/><geom type="sphere" size=".1" mass="1"/></body><body pos="1 0 1"><joint/><geom type="sphere" size=".1" mass="1"/></body></worldbody><actuator><general joint="0" gainprm="50" biastype="affine" biasprm="0 -50 -3" forcelimited="true" forcerange="-10 10"/><general joint="1" gainprm="50" biastype="affine" biasprm="0 -50 -3" forcelimited="true" forcerange="-10 10"/></actuator></mujoco>'.replace('<joint/>','<joint name="j0"/>',1).replace('<joint/>','<joint name="j1"/>',1).replace('joint="0"','joint="j0"').replace('joint="1"','joint="j1"'))
        self.d=mujoco.MjData(self.m)
    def test_loading_damping_preserves_stiffness_limits_and_restores(self):
        m=self.m;g=m.actuator_gainprm.copy();b=m.actuator_biasprm.copy();caps=m.actuator_forcerange.copy()
        set_impedance(m,[[0],[1]],g,b,[1,0],damping_fraction=.5)
        np.testing.assert_array_equal(m.actuator_gainprm,g)
        np.testing.assert_array_equal(m.actuator_forcerange,caps)
        self.assertEqual(m.actuator_biasprm[0,2],-4.5)
        self.assertEqual(m.actuator_biasprm[1,2],-3)
        set_impedance(m,[[0],[1]],g,b,[0,0],damping_fraction=.5)
        np.testing.assert_array_equal(m.actuator_biasprm,b)
    def test_additional_damper_is_dissipative_even_with_saturation(self):
        m,d=self.m,self.d;g=m.actuator_gainprm.copy();b=m.actuator_biasprm.copy()
        for v in np.linspace(-6,6,25):
            for goal in (-1.,0.,1.):
                d.qvel[:]=v;d.ctrl[:]=goal
                set_impedance(m,[[0],[1]],g,b,[0,0],damping_fraction=.5)
                mujoco.mj_forward(m,d);before=d.actuator_force.copy()
                set_impedance(m,[[0],[1]],g,b,[1,1],damping_fraction=.5)
                mujoco.mj_forward(m,d);after=d.actuator_force.copy()
                self.assertTrue(np.all((after-before)*v<=1e-10))
                self.assertTrue(np.all(np.abs(after)<=10+1e-10))
    def test_invalid_active_damping_is_rejected(self):
        with self.assertRaises(ValueError):
            set_impedance(self.m,[],self.m.actuator_gainprm,self.m.actuator_biasprm,[],damping_fraction=-1)

if __name__=="__main__":unittest.main()
