#!/usr/bin/env python3
"""Integration tests for safety policy enforcement across commands."""

import copy
import json
import os
import sys
import tempfile
import time
import unittest
from pathlib import Path
from unittest.mock import Mock, patch, MagicMock, call

sys.path.insert(0, str(Path(__file__).parent.parent / "scripts"))

import ros2_safety
from ros2_safety import DEFAULT_CONFIG


class TestVelocityLimitEnforcement(unittest.TestCase):
    """Test velocity limit enforcement in topic publish commands."""

    def setUp(self):
        self.test_dir = tempfile.mkdtemp()
        self.config_path = os.path.join(self.test_dir, "safety.json")

    def tearDown(self):
        import shutil
        shutil.rmtree(self.test_dir, ignore_errors=True)

    def test_reject_mode_blocks_unsafe_velocity(self):
        """Test that reject mode blocks velocity commands exceeding limits."""
        config = copy.deepcopy(DEFAULT_CONFIG)
        config["velocity_limits"]["enabled"] = True
        config["velocity_limits"]["on_violation"] = "reject"
        config["velocity_limits"]["linear_max_norm"] = 0.5
        
        # Safe velocity
        safe_msg = {"linear": {"x": 0.3, "y": 0.0, "z": 0.0}, "angular": {"x": 0.0, "y": 0.0, "z": 0.0}}
        ok, action, capped, detail = ros2_safety.validate_publish(
            "/cmd_vel", safe_msg, "geometry_msgs/msg/Twist", config=config
        )
        self.assertTrue(ok)
        self.assertEqual(action, "allow")
        
        # Unsafe velocity
        unsafe_msg = {"linear": {"x": 2.0, "y": 0.0, "z": 0.0}, "angular": {"x": 0.0, "y": 0.0, "z": 0.0}}
        ok, action, capped, detail = ros2_safety.validate_publish(
            "/cmd_vel", unsafe_msg, "geometry_msgs/msg/Twist", config=config
        )
        self.assertFalse(ok)
        self.assertEqual(action, "block")
        self.assertIn("violations", detail)
        # violations is a list of strings
        violations_str = " ".join(detail["violations"])
        self.assertIn("linear", violations_str.lower())

    def test_cap_mode_scales_velocity(self):
        """Test that cap mode scales down excessive velocities."""
        config = copy.deepcopy(DEFAULT_CONFIG)
        config["velocity_limits"]["enabled"] = True
        config["velocity_limits"]["on_violation"] = "cap"
        config["velocity_limits"]["linear_max_norm"] = 1.0
        config["velocity_limits"]["angular_max_norm"] = 1.0
        
        # Velocity that needs capping (norm = 2.0)
        msg = {"linear": {"x": 2.0, "y": 0.0, "z": 0.0}, "angular": {"x": 0.0, "y": 0.0, "z": 0.0}}
        ok, action, capped, detail = ros2_safety.validate_publish(
            "/cmd_vel", msg, "geometry_msgs/msg/Twist", config=config
        )
        self.assertTrue(ok)
        self.assertEqual(action, "cap")
        self.assertIsNotNone(capped)
        
        # Capped linear x should be scaled to max
        capped_x = float(capped["linear"]["x"])
        self.assertAlmostEqual(capped_x, 1.0, places=2)

    def test_warn_mode_allows_but_reports(self):
        """Test that warn mode allows unsafe velocities but logs warning."""
        config = copy.deepcopy(DEFAULT_CONFIG)
        config["velocity_limits"]["enabled"] = True
        config["velocity_limits"]["on_violation"] = "warn"
        config["velocity_limits"]["linear_max_norm"] = 0.5
        
        msg = {"linear": {"x": 2.0, "y": 0.0, "z": 0.0}, "angular": {"x": 0.0, "y": 0.0, "z": 0.0}}
        ok, action, capped, detail = ros2_safety.validate_publish(
            "/cmd_vel", msg, "geometry_msgs/msg/Twist", config=config
        )
        self.assertTrue(ok)
        self.assertEqual(action, "warn")
        self.assertIsNone(capped)

    def test_per_axis_limits(self):
        """Test per-axis velocity limits."""
        config = copy.deepcopy(DEFAULT_CONFIG)
        config["velocity_limits"]["enabled"] = True
        config["velocity_limits"]["on_violation"] = "reject"
        config["velocity_limits"]["per_axis"]["linear_x_max"] = 0.3
        config["velocity_limits"]["per_axis"]["angular_z_max"] = 0.5
        
        # Violates linear_x limit
        msg1 = {"linear": {"x": 0.4, "y": 0.0, "z": 0.0}, "angular": {"x": 0.0, "y": 0.0, "z": 0.0}}
        ok, action, _, detail = ros2_safety.validate_publish(
            "/cmd_vel", msg1, "geometry_msgs/msg/Twist", config=config
        )
        self.assertFalse(ok)
        violations_str = " ".join(detail["violations"])
        self.assertIn("linear", violations_str.lower())
        
        # Violates angular_z limit
        msg2 = {"linear": {"x": 0.2, "y": 0.0, "z": 0.0}, "angular": {"x": 0.0, "y": 0.0, "z": 0.6}}
        ok, action, _, detail = ros2_safety.validate_publish(
            "/cmd_vel", msg2, "geometry_msgs/msg/Twist", config=config
        )
        self.assertFalse(ok)
        violations_str = " ".join(detail["violations"])
        self.assertIn("angular", violations_str.lower())

    def test_disabled_velocity_limits(self):
        """Test that disabled velocity limits allow all commands."""
        config = copy.deepcopy(DEFAULT_CONFIG)
        config["velocity_limits"]["enabled"] = False
        
        msg = {"linear": {"x": 100.0, "y": 0.0, "z": 0.0}, "angular": {"x": 0.0, "y": 0.0, "z": 0.0}}
        ok, action, _, _ = ros2_safety.validate_publish(
            "/cmd_vel", msg, "geometry_msgs/msg/Twist", config=config
        )
        self.assertTrue(ok)
        self.assertEqual(action, "allow")


class TestGeofenceEnforcement(unittest.TestCase):
    """Test geofence enforcement in navigation commands."""

    def test_circle_geofence_enforcement(self):
        """Test circular geofence boundaries."""
        config = copy.deepcopy(DEFAULT_CONFIG)
        config["geofence"]["enabled"] = True
        config["geofence"]["mode"] = "circle"
        config["geofence"]["circle"]["center_x"] = 0.0
        config["geofence"]["circle"]["center_y"] = 0.0
        config["geofence"]["circle"]["radius_m"] = 5.0
        
        # Inside geofence
        ok, reason, detail = ros2_safety.check_geofence(3.0, 4.0, config=config)
        self.assertTrue(ok)
        
        # On boundary (should be allowed)
        ok, reason, detail = ros2_safety.check_geofence(5.0, 0.0, config=config)
        self.assertTrue(ok)
        
        # Outside geofence
        ok, reason, detail = ros2_safety.check_geofence(10.0, 0.0, config=config)
        self.assertFalse(ok)
        self.assertIn("outside", reason.lower())

    def test_rectangle_geofence_enforcement(self):
        """Test rectangular geofence boundaries."""
        config = copy.deepcopy(DEFAULT_CONFIG)
        config["geofence"]["enabled"] = True
        config["geofence"]["mode"] = "rectangle"
        config["geofence"]["rectangle"]["x_min"] = -10.0
        config["geofence"]["rectangle"]["x_max"] = 10.0
        config["geofence"]["rectangle"]["y_min"] = -5.0
        config["geofence"]["rectangle"]["y_max"] = 5.0
        
        # Inside geofence
        ok, _, _ = ros2_safety.check_geofence(5.0, 3.0, config=config)
        self.assertTrue(ok)
        
        # On boundary (should be allowed)
        ok, _, _ = ros2_safety.check_geofence(10.0, 5.0, config=config)
        self.assertTrue(ok)
        
        # Outside X bounds
        ok, reason, _ = ros2_safety.check_geofence(15.0, 0.0, config=config)
        self.assertFalse(ok)
        self.assertIn("outside", reason.lower())
        
        # Outside Y bounds
        ok, reason, _ = ros2_safety.check_geofence(0.0, 10.0, config=config)
        self.assertFalse(ok)

    def test_disabled_geofence(self):
        """Test that disabled geofence allows all positions."""
        config = copy.deepcopy(DEFAULT_CONFIG)
        config["geofence"]["enabled"] = False
        config["geofence"]["mode"] = "circle"
        config["geofence"]["circle"]["radius_m"] = 1.0
        
        # Should allow even though outside radius
        ok, reason, _ = ros2_safety.check_geofence(100.0, 100.0, config=config)
        self.assertTrue(ok)

    def test_geofence_with_offset_center(self):
        """Test geofence with non-origin center."""
        config = copy.deepcopy(DEFAULT_CONFIG)
        config["geofence"]["enabled"] = True
        config["geofence"]["mode"] = "circle"
        config["geofence"]["circle"]["center_x"] = 10.0
        config["geofence"]["circle"]["center_y"] = 20.0
        config["geofence"]["circle"]["radius_m"] = 5.0
        
        # Should be inside (distance = 0)
        ok, _, _ = ros2_safety.check_geofence(10.0, 20.0, config=config)
        self.assertTrue(ok)
        
        # Should be inside (distance < 5)
        ok, _, _ = ros2_safety.check_geofence(13.0, 24.0, config=config)
        self.assertTrue(ok)
        
        # Should be outside
        ok, _, _ = ros2_safety.check_geofence(0.0, 0.0, config=config)
        self.assertFalse(ok)


class TestCommandFilter(unittest.TestCase):
    """Test command filtering (blocklist/allowlist)."""

    def test_blocklist_mode(self):
        """Test blocklist filtering mode."""
        config = copy.deepcopy(DEFAULT_CONFIG)
        config["command_filter"]["enabled"] = True
        config["command_filter"]["mode"] = "blocklist"
        config["command_filter"]["blocklist"] = ["topics publish", "services call", "nav2 go"]
        
        # Blocked commands
        ok, reason, _ = ros2_safety.check_command("topics", "publish", config=config)
        self.assertFalse(ok)
        self.assertIn("blocked", reason.lower())
        
        ok, _, _ = ros2_safety.check_command("services", "call", config=config)
        self.assertFalse(ok)
        
        ok, _, _ = ros2_safety.check_command("nav2", "go", config=config)
        self.assertFalse(ok)
        
        # Allowed commands
        ok, _, _ = ros2_safety.check_command("topics", "list", config=config)
        self.assertTrue(ok)
        
        ok, _, _ = ros2_safety.check_command("nav2", "status", config=config)
        self.assertTrue(ok)

    def test_allowlist_mode(self):
        """Test allowlist filtering mode."""
        config = copy.deepcopy(DEFAULT_CONFIG)
        config["command_filter"]["enabled"] = True
        config["command_filter"]["mode"] = "allowlist"
        config["command_filter"]["allowlist"] = ["topics list", "nodes list", "nav2 status"]
        
        # Allowed commands
        ok, _, _ = ros2_safety.check_command("topics", "list", config=config)
        self.assertTrue(ok)
        
        ok, _, _ = ros2_safety.check_command("nodes", "list", config=config)
        self.assertTrue(ok)
        
        ok, _, _ = ros2_safety.check_command("nav2", "status", config=config)
        self.assertTrue(ok)
        
        # Blocked commands (not in allowlist)
        ok, reason, _ = ros2_safety.check_command("topics", "publish", config=config)
        self.assertFalse(ok)
        self.assertIn("not in allowlist", reason.lower())
        
        ok, _, _ = ros2_safety.check_command("nav2", "go", config=config)
        self.assertFalse(ok)

    def test_safety_commands_always_allowed(self):
        """Test that safety and estop commands are never blocked."""
        config = copy.deepcopy(DEFAULT_CONFIG)
        config["command_filter"]["enabled"] = True
        config["command_filter"]["mode"] = "blocklist"
        config["command_filter"]["blocklist"] = ["safety", "estop"]
        
        # Safety and estop should still be allowed
        ok, _, _ = ros2_safety.check_command("safety", "show", config=config)
        self.assertTrue(ok)
        
        ok, _, _ = ros2_safety.check_command("safety", "disable", config=config)
        self.assertTrue(ok)
        
        ok, _, _ = ros2_safety.check_command("estop", None, config=config)
        self.assertTrue(ok)

    def test_wildcard_blocking(self):
        """Test wildcard command blocking."""
        config = copy.deepcopy(DEFAULT_CONFIG)
        config["command_filter"]["enabled"] = True
        config["command_filter"]["mode"] = "blocklist"
        config["command_filter"]["blocklist"] = ["nav2"]
        
        # All nav2 subcommands should be blocked
        ok, _, _ = ros2_safety.check_command("nav2", "go", config=config)
        self.assertFalse(ok)
        
        ok, _, _ = ros2_safety.check_command("nav2", "rotate", config=config)
        self.assertFalse(ok)
        
        ok, _, _ = ros2_safety.check_command("nav2", "cancel", config=config)
        self.assertFalse(ok)

    def test_disabled_command_filter(self):
        """Test that disabled filter allows all commands."""
        config = copy.deepcopy(DEFAULT_CONFIG)
        config["command_filter"]["enabled"] = False
        config["command_filter"]["mode"] = "blocklist"
        config["command_filter"]["blocklist"] = ["topics publish"]
        
        # Should be allowed even though in blocklist
        ok, _, _ = ros2_safety.check_command("topics", "publish", config=config)
        self.assertTrue(ok)


class TestHeartbeatSystem(unittest.TestCase):
    """Test heartbeat and watchdog functionality."""

    def setUp(self):
        self.test_dir = tempfile.mkdtemp()

    def tearDown(self):
        import shutil
        shutil.rmtree(self.test_dir, ignore_errors=True)

    def test_heartbeat_touch_creates_sentinel(self):
        """Test that heartbeat touch creates/updates sentinel file."""
        sentinel_path = os.path.join(self.test_dir, "heartbeat.sentinel")
        
        with patch.object(ros2_safety, "_HEARTBEAT_SENTINEL", sentinel_path):
            result = ros2_safety.heartbeat_touch()
        
        self.assertTrue(os.path.exists(sentinel_path))
        self.assertEqual(result["path"], sentinel_path)
        self.assertIn("timestamp", result)

    def test_heartbeat_touch_updates_mtime(self):
        """Test that repeated touches update mtime."""
        sentinel_path = os.path.join(self.test_dir, "heartbeat.sentinel")
        
        with patch.object(ros2_safety, "_HEARTBEAT_SENTINEL", sentinel_path):
            ros2_safety.heartbeat_touch()
            mtime1 = os.path.getmtime(sentinel_path)
            
            time.sleep(0.1)
            
            ros2_safety.heartbeat_touch()
            mtime2 = os.path.getmtime(sentinel_path)
        
        self.assertGreater(mtime2, mtime1)

    def test_sentinel_age_calculation(self):
        """Test that we can detect old sentinel files by mtime."""
        sentinel_path = os.path.join(self.test_dir, "heartbeat.sentinel")
        
        # Create old sentinel
        Path(sentinel_path).touch()
        old_time = time.time() - 10.0
        os.utime(sentinel_path, (old_time, old_time))
        
        # Check that age is correct
        mtime = os.path.getmtime(sentinel_path)
        age = time.time() - mtime
        self.assertGreater(age, 9.0)
        self.assertLess(age, 11.0)

    def test_fresh_sentinel_detected(self):
        """Test that fresh sentinel has recent mtime."""
        sentinel_path = os.path.join(self.test_dir, "heartbeat.sentinel")
        
        # Create fresh sentinel
        Path(sentinel_path).touch()
        
        # Check that age is recent
        mtime = os.path.getmtime(sentinel_path)
        age = time.time() - mtime
        self.assertLess(age, 1.0)


class TestConfigManagement(unittest.TestCase):
    """Test safety config loading and validation."""

    def setUp(self):
        self.test_dir = tempfile.mkdtemp()

    def tearDown(self):
        import shutil
        shutil.rmtree(self.test_dir, ignore_errors=True)

    def test_load_config_creates_default(self):
        """Test that load_config creates default config if not exists."""
        config = ros2_safety.load_config(base_dir=self.test_dir)
        
        self.assertIn("velocity_limits", config)
        self.assertIn("geofence", config)
        self.assertIn("command_filter", config)
        self.assertIn("heartbeat", config)
        
        # Check file was created
        config_path = os.path.join(self.test_dir, ".presets", "safety.json")
        self.assertTrue(os.path.exists(config_path))

    def test_save_and_load_config(self):
        """Test saving and loading config."""
        config = copy.deepcopy(DEFAULT_CONFIG)
        config["velocity_limits"]["linear_max_norm"] = 1.5
        config["geofence"]["enabled"] = True
        
        ros2_safety.save_config(config, base_dir=self.test_dir)
        
        loaded = ros2_safety.load_config(base_dir=self.test_dir)
        self.assertEqual(loaded["velocity_limits"]["linear_max_norm"], 1.5)
        self.assertTrue(loaded["geofence"]["enabled"])

    def test_config_validation_corrects_invalid_values(self):
        """Test that config validation corrects invalid values."""
        config = {
            "version": "1",
            "enabled": True,
            "velocity_limits": {
                "enabled": True,
                "on_violation": "invalid_mode",  # Should be corrected
                "linear_max_norm": -1.0,  # Should use default
                "angular_max_norm": "not_a_number",  # Should use default
            },
            "geofence": DEFAULT_CONFIG["geofence"],
            "command_filter": DEFAULT_CONFIG["command_filter"],
            "heartbeat": DEFAULT_CONFIG["heartbeat"],
        }
        
        validated, warnings = ros2_safety._validate_config(config)
        
        self.assertEqual(validated["velocity_limits"]["on_violation"], "reject")
        self.assertEqual(validated["velocity_limits"]["linear_max_norm"], 0.5)
        self.assertEqual(validated["velocity_limits"]["angular_max_norm"], 1.0)
        self.assertGreater(len(warnings), 0)


if __name__ == '__main__':
    unittest.main()
