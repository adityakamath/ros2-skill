#!/usr/bin/env python3
"""Unit tests for ros2_safety and safety hooks in topic/action commands."""

import copy
import json
import os
import sys
import tempfile
import types
import unittest
from unittest.mock import patch


sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "scripts"))


import ros2_safety  # noqa: E402


class TestSafetyCore(unittest.TestCase):
    def test_validate_publish_modes(self):
        msg = {"linear": {"x": 1.0, "y": 0.0, "z": 0.0}, "angular": {"x": 0.0, "y": 0.0, "z": 0.0}}

        cfg_reject = copy.deepcopy(ros2_safety.DEFAULT_CONFIG)
        cfg_reject["velocity_limits"]["on_violation"] = "reject"
        ok, action, capped, detail = ros2_safety.validate_publish("/cmd_vel", msg, "geometry_msgs/msg/Twist", config=cfg_reject)
        self.assertFalse(ok)
        self.assertEqual(action, "block")
        self.assertIsNone(capped)
        self.assertIn("violations", detail)

        cfg_warn = copy.deepcopy(ros2_safety.DEFAULT_CONFIG)
        cfg_warn["velocity_limits"]["on_violation"] = "warn"
        ok, action, capped, _ = ros2_safety.validate_publish("/cmd_vel", msg, "geometry_msgs/msg/Twist", config=cfg_warn)
        self.assertTrue(ok)
        self.assertEqual(action, "warn")
        self.assertIsNone(capped)

        cfg_cap = copy.deepcopy(ros2_safety.DEFAULT_CONFIG)
        cfg_cap["velocity_limits"]["on_violation"] = "cap"
        ok, action, capped, _ = ros2_safety.validate_publish("/cmd_vel", msg, "geometry_msgs/msg/Twist", config=cfg_cap)
        self.assertTrue(ok)
        self.assertEqual(action, "cap")
        self.assertIsInstance(capped, dict)
        self.assertLessEqual(abs(float(capped["linear"]["x"])), cfg_cap["velocity_limits"]["linear_max_norm"])

    def test_check_command_modes_and_exceptions(self):
        cfg_block = copy.deepcopy(ros2_safety.DEFAULT_CONFIG)
        cfg_block["command_filter"].update({
            "enabled": True,
            "mode": "blocklist",
            "blocklist": ["topics publish", "nav2"],
        })
        allowed, reason, _ = ros2_safety.check_command("topics", "publish", config=cfg_block)
        self.assertFalse(allowed)
        self.assertIn("blocked", reason)

        allowed, _, _ = ros2_safety.check_command("nav2", "go", config=cfg_block)
        self.assertFalse(allowed)

        # Safety and estop commands are always allowed to avoid lockout.
        self.assertTrue(ros2_safety.check_command("safety", "show", config=cfg_block)[0])
        self.assertTrue(ros2_safety.check_command("estop", None, config=cfg_block)[0])

        cfg_allow = copy.deepcopy(ros2_safety.DEFAULT_CONFIG)
        cfg_allow["command_filter"].update({
            "enabled": True,
            "mode": "allowlist",
            "allowlist": ["topics list"],
        })
        self.assertTrue(ros2_safety.check_command("topics", "list", config=cfg_allow)[0])
        self.assertFalse(ros2_safety.check_command("topics", "publish", config=cfg_allow)[0])

    def test_check_geofence_circle_and_rectangle(self):
        cfg = copy.deepcopy(ros2_safety.DEFAULT_CONFIG)
        cfg["geofence"].update({"enabled": True, "mode": "circle"})
        cfg["geofence"]["circle"].update({"center_x": 0.0, "center_y": 0.0, "radius_m": 1.0})
        self.assertTrue(ros2_safety.check_geofence(0.5, 0.5, config=cfg)[0])
        self.assertFalse(ros2_safety.check_geofence(2.0, 0.0, config=cfg)[0])

        cfg["geofence"].update({"mode": "rectangle"})
        cfg["geofence"]["rectangle"].update({"x_min": -1.0, "x_max": 1.0, "y_min": -2.0, "y_max": 2.0})
        self.assertTrue(ros2_safety.check_geofence(0.0, -1.5, config=cfg)[0])
        self.assertFalse(ros2_safety.check_geofence(3.0, 0.0, config=cfg)[0])

    def test_heartbeat_touch_creates_or_updates_sentinel(self):
        with tempfile.TemporaryDirectory() as tmp:
            sentinel = os.path.join(tmp, "heartbeat.sentinel")
            with patch.object(ros2_safety, "_ARTIFACTS_DIR", tmp), \
                 patch.object(ros2_safety, "_HEARTBEAT_SENTINEL", sentinel):
                result = ros2_safety.heartbeat_touch()
                self.assertTrue(os.path.exists(sentinel))
                self.assertEqual(result["path"], sentinel)

    def test_config_paths_are_modular_and_generic(self):
        with tempfile.TemporaryDirectory() as tmp:
            base_dir = os.path.join(tmp, "robot_runtime")
            cfg = ros2_safety.load_config(base_dir=base_dir)
            self.assertTrue(os.path.exists(os.path.join(base_dir, ".presets", "safety.json")))
            self.assertIn("velocity_limits", cfg)

            custom_cfg = copy.deepcopy(cfg)
            custom_cfg["command_filter"].update({
                "enabled": True,
                "mode": "blocklist",
                "always_allowed": ["custom_guard"],
            })
            saved = ros2_safety.save_config(custom_cfg, base_dir=base_dir)
            self.assertEqual(saved["command_filter"]["always_allowed"], ["custom_guard"])

            allowed, reason, detail = ros2_safety.check_command("custom_guard", None, config=saved)
            self.assertTrue(allowed)
            self.assertEqual(detail.get("mode"), "blocklist")


class TestSafetyHooks(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        import ros2_action
        import ros2_topic

        cls.ros2_action = ros2_action
        cls.ros2_topic = ros2_topic

    def test_publish_blocked_by_safety_before_ros_context(self):
        args = types.SimpleNamespace(
            topic="/cmd_vel",
            msg=json.dumps({"linear": {"x": 1.0}, "angular": {"z": 0.0}}),
            msg_type="geometry_msgs/msg/Twist",
            max_vel=None,
            max_ang=None,
            dry_run=False,
            rate=None,
            duration=None,
            timeout=None,
        )
        detail = {"violations": ["too fast"], "limits": {"linear_max_norm": 0.5}, "actual": {"linear_norm": 1.0}}
        with patch.object(self.ros2_topic, "validate_publish", return_value=(False, "block", None, detail)), \
             patch.object(self.ros2_topic, "ros2_context", side_effect=AssertionError("ros2_context should not be called")), \
             patch.object(self.ros2_topic, "output") as mock_output:
            self.ros2_topic.cmd_topics_publish(args)
            payload = mock_output.call_args[0][0]
            self.assertTrue(payload.get("blocked"))
            self.assertIn("safety", payload.get("error", "").lower())

    def test_action_send_blocked_by_geofence_before_action_client(self):
        goal = {
            "pose": {
                "pose": {
                    "position": {"x": 10.0, "y": 10.0, "z": 0.0},
                    "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
                }
            }
        }
        args = types.SimpleNamespace(action="/navigate_to_pose", goal=json.dumps(goal), timeout=2.0)
        cfg = copy.deepcopy(ros2_safety.DEFAULT_CONFIG)
        cfg["geofence"].update({"enabled": True, "on_violation": "reject"})

        with patch.object(self.ros2_action, "load_config", return_value=cfg), \
             patch.object(self.ros2_action, "check_geofence", return_value=(False, "outside", {"mode": "circle"})), \
             patch.object(self.ros2_action, "ros2_context", side_effect=AssertionError("ros2_context should not be called")), \
             patch.object(self.ros2_action, "output") as mock_output:
            self.ros2_action.cmd_actions_send(args)
            payload = mock_output.call_args[0][0]
            self.assertTrue(payload.get("blocked"))
            self.assertIn("geofence", payload.get("error", "").lower())


if __name__ == "__main__":
    unittest.main()
