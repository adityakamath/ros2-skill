#!/usr/bin/env python3
"""Profile extraction for wildcard-namespaced configs and toggle/joystick service configs."""

import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))

from ros2_robot_config import extract_robot_config

FILES = {
    "toggles.yaml": {"/**/bool_toggle_node": {"ros__parameters": {
        "toggles": ["emergency_stop", "twist_switch"],
        "emergency_stop": {"trigger_service": "emergency_stop_toggle", "target_service": "emergency_stop"},
        "twist_switch": {"trigger_service": "twist_switch_toggle", "target_service": "twist_switch"}}}},
    "base_teleop.yaml": {"/**/joy_teleop": {"ros__parameters": {
        "teleop": {"type": "topic", "interface_type": "geometry_msgs/msg/TwistStamped", "topic_name": "cmd_vel_teleop",
                   "deadman_buttons": [9],
                   "axis_mappings": {"twist-linear-x": {"scale": 0.2}, "twist-linear-y": {"scale": 0.2},
                                     "twist-angular-z": {"scale": 0.8}, "header-frame_id": {"value": "base_link"}}},
        "save_map": {"type": "service", "interface_type": "std_srvs/srv/SetBool", "service_name": "save_map",
                     "service_request": {"data": True}}}}},
    "twist_switch.yaml": {"/**/twist_switch_node": {"ros__parameters": {
        "input_topic_default": "cmd_vel_teleop", "input_topic_switched": "cmd_vel_smoothed", "output_topic": "cmd_vel_presafety"}}},
    "hardware.yaml": {"lidar": True, "gemini2": False, "note": "x"},
}


def extract(files=FILES):
    return extract_robot_config(list(files), lambda p: files[p])


class TestRobotConfig(unittest.TestCase):
    def test_idempotent_setbool_estop_from_toggle_config(self):
        e = extract()["estop_config"]
        self.assertEqual((e["service_name"], e["service_type"], e["toggle_service"]),
                         ("/emergency_stop", "std_srvs/srv/SetBool", "/emergency_stop_toggle"))

    def test_velocity_chain_and_mode_service(self):
        self.assertEqual(extract()["velocity_chain"], {
            "teleop_input": "/cmd_vel_teleop", "nav2_input": "/cmd_vel_smoothed",
            "output": "/cmd_vel_presafety", "mode_service": "/twist_switch"})

    def test_teleop_topic_and_limits_from_axis_scales(self):
        out = extract()
        self.assertEqual(out["teleop_config"]["cmd_vel_topic"], "/cmd_vel_teleop")
        self.assertEqual(out["teleop_limits"]["binding"], {"linear_x": 0.2, "linear_y": 0.2, "angular_z": 0.8})

    def test_joystick_services_and_hardware_flags(self):
        out = extract()
        self.assertEqual(out["robot_services"]["save_map"],
                         {"service": "/save_map", "type": "std_srvs/srv/SetBool", "request": {"data": True}})
        self.assertEqual(out["hardware_fitted"], {"lidar": True, "gemini2": False})

    def test_unrelated_configs_yield_nothing(self):
        self.assertEqual(extract({"nav2.yaml": {"planner_server": {"ros__parameters": {"x": 1}}}}), {})


class TestProfileWildcards(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        try:
            import ros2_profile
        except Exception as exc:  # no ROS 2 environment
            raise unittest.SkipTest(f"ros2_profile not importable: {exc}")
        cls.mod = ros2_profile

    def test_find_node_block_matches_wildcard_namespace(self):
        data = {"/**/ekf_node": {"ros__parameters": {"frequency": 30.0}}}
        self.assertEqual(self.mod._find_node_block(data, {"ekf_node"}), {"frequency": 30.0})

    def test_controller_manager_wildcard_gives_drive_type(self):
        yf = {"/**/controller_manager": {"ros__parameters": {
            "update_rate": 20, "base_controller": {"type": "omni_wheel_drive_controller/OmniWheelDriveController"}}}}
        orig = self.mod._load_yaml
        self.mod._load_yaml = lambda p: yf
        try:
            result = self.mod._extract_ros2_control_config(["control.yaml"])
        finally:
            self.mod._load_yaml = orig
        self.assertEqual(result["drive_type"], "holonomic_omni")
        self.assertEqual(result["controller_update_rate_hz"], 20)

    def test_outside_modules_drops_driver_submodule_files_unless_nothing_remains(self):
        f = self.mod._outside_modules
        self.assertEqual(f(["/ws/lekiwi/lekiwi_control/a.yaml", "/ws/lekiwi/modules/sts/b.yaml"]),
                         ["/ws/lekiwi/lekiwi_control/a.yaml"])
        self.assertEqual(f(["/ws/modules/only.yaml"]), ["/ws/modules/only.yaml"])


if __name__ == "__main__":
    unittest.main()
