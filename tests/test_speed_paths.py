#!/usr/bin/env python3
"""Start-up cost fixes: one rclpy init per process, no live decel sweep in Path A, cached profile reads."""

import json
import sys
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))

try:
    import ros2_utils
    import ros2_topic
    import ros2_profile
    _ERR = None
except Exception as exc:  # no ROS 2 environment
    _ERR = exc


@unittest.skipIf(_ERR, f"ROS 2 python not importable: {_ERR}")
class TestSpeedPaths(unittest.TestCase):
    def test_nested_contexts_share_one_init_and_one_shutdown(self):
        ros2_utils._ensure_rclpy()  # rclpy is imported lazily
        with patch.object(ros2_utils.rclpy, "init") as init, patch.object(ros2_utils.rclpy, "shutdown") as shutdown:
            with ros2_utils.ros2_context():
                with ros2_utils.ros2_context():
                    init.assert_called_once()
                shutdown.assert_not_called()
            shutdown.assert_called_once()

    def test_decel_params_without_a_node_use_defaults_instantly(self):
        with patch("ros2_braking.load_braking", return_value=None):  # a stored measurement would replace the default
            zone, factor, meta = ros2_topic.compute_decel_params({"linear": {"x": 0.2}}, 1.0, False, False, node=None)
        self.assertEqual(meta["params_source"], "defaults")
        self.assertAlmostEqual(zone, 0.05)
        self.assertAlmostEqual(factor, 0.5)

    def test_profile_record_is_cached_until_the_file_changes(self):
        with tempfile.TemporaryDirectory() as d:
            path = Path(d) / "bot_profile.json"
            path.write_text(json.dumps({"robot_name": "bot", "summary": {"x": 1}}))
            with patch.object(ros2_profile, "_load_profile_record_uncached", wraps=ros2_profile._load_profile_record_uncached) as slow:
                first = ros2_profile.load_profile_record(profiles_dir=d)
                second = ros2_profile.load_profile_record(profiles_dir=d)
                self.assertEqual(first, second)
                self.assertEqual(slow.call_count, 1)
                path.write_text(json.dumps({"robot_name": "bot", "summary": {"x": 2, "more": True}}))
                self.assertEqual(ros2_profile.load_profile_record(profiles_dir=d)["summary"]["x"], 2)
                self.assertEqual(slow.call_count, 2)


@unittest.skipIf(_ERR, f"ROS 2 python not importable: {_ERR}")
class TestRunCli(unittest.TestCase):
    def test_in_process_commands_get_the_same_defaults_as_main(self):
        """Without main()'s global overrides, --retries stayed None and `services call` crashed."""
        import ros2_cli
        import ros2_plan
        seen = {}

        def handler(args):
            seen["retries"] = args.retries
            print(json.dumps({"success": True}))

        with patch.dict(ros2_cli.DISPATCH, {("services", "call"): handler}):
            out, rc = ros2_plan.run_cli(["services", "call", "/s", "{}"])
        self.assertEqual((out, rc), ({"success": True}, 0))
        self.assertIsNotNone(seen["retries"])

    def test_handler_errors_come_back_as_json_not_exceptions(self):
        import ros2_cli
        import ros2_plan

        def boom(args):
            raise RuntimeError("nope")

        with patch.dict(ros2_cli.DISPATCH, {("services", "call"): boom}):
            out, rc = ros2_plan.run_cli(["services", "call", "/s", "{}"])
        self.assertEqual(rc, 1)
        self.assertIn("nope", out["error"])


if __name__ == "__main__":
    unittest.main()
