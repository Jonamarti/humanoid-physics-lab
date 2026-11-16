"""Focused regression checks for experimental evidence integrity."""
import copy
import tempfile
import unittest
from pathlib import Path
import numpy as np
import mujoco
from compare import validate_pair,rescore
from snapshots import save_model
from metrics import model_fingerprint

class ComparisonIntegrityTests(unittest.TestCase):
    def setUp(self):
        self.row=dict(case="stand",model_sha256="model",reference_sha256="reference",
            initial_state_sha256="state",mujoco="3.5.0",duration_s=6.,external_impulse_ns=[25.,0,0])
    def test_rejects_changed_model_reference_state_engine_duration_and_case(self):
        for key in ("case","model_sha256","reference_sha256","initial_state_sha256","mujoco","duration_s"):
            other=copy.deepcopy(self.row);other[key]="changed"
            with self.subTest(key=key),self.assertRaises(ValueError):validate_pair(self.row,other)
    def test_rejects_changed_impulse(self):
        other=copy.deepcopy(self.row);other["external_impulse_ns"]=[24.9,0,0]
        with self.assertRaises(ValueError):validate_pair(self.row,other)
    def test_accepts_identical_conditions(self):
        validate_pair(self.row,copy.deepcopy(self.row))
    def test_falling_foot_motion_is_not_a_supported_step(self):
        rows=np.zeros((3,13));rows[:,0]=[0,.1,.2];rows[:,9]=.2;rows[:,8]=500.
        s=dict(events=[dict(t=.1,foot=0,event="clearance_3cm"),dict(t=.2,foot=0,event="loaded")],
            mass_kg=42.,initial_root_height_m=1.,foot_delta_m=[[.12,0,0],[0,0,0]],
            stable_final_second=False,artifacts=[],kind="step",outcome="incomplete")
        self.assertFalse(rescore(s,rows)["physical_step"])
    def test_supported_cycle_and_final_stability_are_required(self):
        rows=np.zeros((3,13));rows[:,0]=[0,.1,.2];rows[:,9]=1.;rows[:,8]=500.
        s=dict(events=[dict(t=.1,foot=0,event="clearance_3cm"),dict(t=.2,foot=0,event="loaded")],
            mass_kg=42.,initial_root_height_m=1.,foot_delta_m=[[.12,0,0],[0,0,0]],
            stable_final_second=True,artifacts=[],kind="step",outcome="incomplete")
        result=rescore(s,rows);self.assertTrue(result["physical_step"]);self.assertTrue(result["task_achieved"])
        s["artifacts"]=["loaded_foot_slip"]
        self.assertFalse(rescore(s,rows)["task_achieved"])

class ModelReloadTests(unittest.TestCase):
    def test_runtime_pd_bias_enum_survives_binary_and_xml(self):
        m=mujoco.MjModel.from_xml_string('<mujoco><worldbody><body name="Hips"><freejoint/><geom size=".1"/><body><joint name="hinge"/><geom size=".1" pos="0 0 .3"/></body></body></worldbody><actuator><motor joint="hinge"/></actuator></mujoco>')
        m.actuator_biastype[0]=1;m.actuator_gainprm[0,0]=50
        m.actuator_biasprm[0,:3]=[0,-50,-3]
        m.actuator_forcerange[0]=[-20,20];m.actuator_forcelimited[0]=1
        with tempfile.TemporaryDirectory() as tmp:
            save_model(m,Path(tmp))
            binary=mujoco.MjModel.from_binary_path(str(Path(tmp)/"model.mjb"))
            xml=mujoco.MjModel.from_xml_path(str(Path(tmp)/"model.xml"))
            self.assertEqual(model_fingerprint(binary),model_fingerprint(m))
            self.assertEqual(xml.actuator_biastype[0],1)
            np.testing.assert_allclose(xml.actuator_biasprm[0,:3],[0,-50,-3])
    def test_contact_compliance_changes_fingerprint(self):
        m=mujoco.MjModel.from_xml_string('<mujoco><worldbody><geom size=".1"/></worldbody></mujoco>')
        before=model_fingerprint(m);m.geom_solref[0,0]*=.5
        self.assertNotEqual(before,model_fingerprint(m))


class SoleGeometryTests(unittest.TestCase):
    def model(self,height):
        return mujoco.MjModel.from_xml_string(f'<mujoco><worldbody><geom name="floor" type="plane" size="1 1 .1"/><body name="foot" pos="0 0 {height}"><geom name="toe" type="sphere" size=".03" pos="0 0 -.07"/></body></worldbody></mujoco>')
    def test_high_ankle_with_toe_on_ground_is_not_clearance(self):
        from analyse_natural import minimum_sole_clearance
        m=self.model(.10);d=mujoco.MjData(m);mujoco.mj_forward(m,d)
        self.assertGreater(d.xpos[m.body("foot").id,2],.03)
        self.assertLess(minimum_sole_clearance(m,d,m.geom("floor").id,[m.geom("toe").id]),.005)
    def test_whole_foot_above_ground_is_measured(self):
        from analyse_natural import minimum_sole_clearance
        m=self.model(.14);d=mujoco.MjData(m);mujoco.mj_forward(m,d)
        self.assertAlmostEqual(minimum_sole_clearance(m,d,m.geom("floor").id,[m.geom("toe").id]),.04,places=6)
    def test_actuator_enum_changes_full_fingerprint(self):
        from metrics import physical_fingerprint
        m=mujoco.MjModel.from_xml_string('<mujoco><worldbody><body><joint name="j"/><geom size=".1"/></body></worldbody><actuator><motor joint="j"/></actuator></mujoco>')
        before=physical_fingerprint(m);m.actuator_biastype[0]=1
        self.assertNotEqual(before,physical_fingerprint(m))

if __name__=="__main__":unittest.main()
