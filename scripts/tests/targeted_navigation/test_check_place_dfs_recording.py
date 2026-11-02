import importlib.util
from pathlib import Path
from types import SimpleNamespace
import unittest

SCRIPT=Path(__file__).with_name("check_place_dfs_recording.py")
spec=importlib.util.spec_from_file_location("recording_checker", SCRIPT)
module=importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


def param(name,value):
    return SimpleNamespace(name=name,value=value)


class TEBRuntimeConfigTests(unittest.TestCase):
    def test_accepts_legal_forward_only_dynamic_config(self):
        message=SimpleNamespace(doubles=[
            param("max_vel_x_backwards",0.01),
            param("weight_kinematics_forward_drive",1000.0),
        ],bools=[param("allow_init_with_backwards_motion",False),
                 param("enable_homotopy_class_planning",True)],
          ints=[param("max_number_classes",5)])
        result=module.audit_teb_parameter_updates([message])
        self.assertTrue(result["passed"],result)

    def test_rejects_values_outside_installed_dynamic_bounds(self):
        message=SimpleNamespace(doubles=[
            param("max_vel_x_backwards",0.001),
            param("weight_kinematics_forward_drive",5000.0),
        ],bools=[param("allow_init_with_backwards_motion",False),
                 param("enable_homotopy_class_planning",True)],
          ints=[param("max_number_classes",5)])
        result=module.audit_teb_parameter_updates([message])
        self.assertFalse(result["passed"])
        self.assertEqual(len(result["errors"]),2)

    def test_rejects_missing_dynamic_update(self):
        result=module.audit_teb_parameter_updates([])
        self.assertFalse(result["passed"])
        self.assertIn("max_vel_x_backwards",result["missing"])

    def test_rejects_backwards_initialization(self):
        message=SimpleNamespace(doubles=[
            param("max_vel_x_backwards",0.01),
            param("weight_kinematics_forward_drive",1000.0),
        ],bools=[param("allow_init_with_backwards_motion",True),
                 param("enable_homotopy_class_planning",True)],
          ints=[param("max_number_classes",5)])
        result=module.audit_teb_parameter_updates([message])
        self.assertIn("backwards_initialization_enabled",result["errors"])

    def test_rejects_effective_planner_with_single_band_enabled(self):
        message=SimpleNamespace(doubles=[
            param("max_vel_x_backwards",0.01),
            param("weight_kinematics_forward_drive",1000.0),
        ],bools=[param("allow_init_with_backwards_motion",False),
                 param("enable_homotopy_class_planning",False)],
          ints=[param("max_number_classes",5)])
        result=module.audit_teb_parameter_updates([message])
        self.assertIn("homotopy_class_planning_disabled",result["errors"])

    def test_rejects_too_few_alternative_classes(self):
        message=SimpleNamespace(doubles=[
            param("max_vel_x_backwards",0.01),
            param("weight_kinematics_forward_drive",1000.0),
        ],bools=[param("allow_init_with_backwards_motion",False),
                 param("enable_homotopy_class_planning",True)],
          ints=[param("max_number_classes",1)])
        result=module.audit_teb_parameter_updates([message])
        self.assertIn("insufficient_homotopy_classes",result["errors"])


class RecordingContractTests(unittest.TestCase):
    def test_teb_diagnosis_topics_are_required(self):
        required=set(module.REQUIRED_TOPICS)
        self.assertTrue({
            "/move_base/NavfnROS/plan", "/move_base/current_goal",
            "/move_base/goal", "/move_base/status", "/move_base/result",
            "/move_base/recovery_status",
        }.issubset(required))

    def test_missing_current_plan_or_action_status_fails_recording_audit(self):
        counts={name:2 for name in module.REQUIRED_TOPICS}
        counts["/move_base/global_costmap/costmap_updates"]=2
        counts["/move_base/local_costmap/costmap_updates"]=2
        del counts["/move_base/NavfnROS/plan"]
        result=module.audit_counts(counts)
        self.assertFalse(result["passed"])
        self.assertIn("/move_base/NavfnROS/plan",result["missing_topics"])


if __name__=="__main__":
    unittest.main()
