"""Pure-function tests for snapshot/look helpers, the launch-log parser and the new plan verbs."""

import math
import os
import struct
import sys
import unittest
from types import SimpleNamespace as NS

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "scripts"))

import ros2_logs  # noqa: E402
import ros2_plan  # noqa: E402
import ros2_snapshot as snap  # noqa: E402


def _scan(ranges, angle_min=-math.pi, inc=None):
    inc = inc or 2 * math.pi / len(ranges)
    return NS(ranges=ranges, angle_min=angle_min, angle_increment=inc, range_min=0.1, range_max=10.0,
              header=NS(frame_id="laser"))


class TestSnapshotHelpers(unittest.TestCase):
    def test_scan_sectors_ignore_invalid_and_find_nearest(self):
        ranges = [float("inf")] * 360
        ranges[180] = 2.0      # 0 degrees: ahead (angle_min -pi + 180 deg)
        ranges[0] = 0.5        # -180 degrees: behind
        ranges[270] = 0.05     # below range_min: ignored
        out = snap._scan_sections(_scan(ranges))
        self.assertEqual(out["sectors_m"]["front"], 2.0)
        self.assertEqual(out["sectors_m"]["back"], 0.5)
        self.assertIsNone(out["sectors_m"]["left"])
        self.assertEqual(out["nearest_m"], 0.5)

    def test_tilt_level_and_rolled(self):
        self.assertEqual(snap._tilt(NS(x=0.0, y=0.0, z=0.0, w=1.0)), {"roll_deg": 0.0, "pitch_deg": 0.0})
        half = math.radians(10) / 2
        roll = snap._tilt(NS(x=math.sin(half), y=0.0, z=0.0, w=math.cos(half)))
        self.assertAlmostEqual(roll["roll_deg"], 10.0, places=1)

    def test_pointcloud_ranges_skips_nan(self):
        pts = [(0.0, 0.0, 1.0), (float("nan"),) * 3, (0.0, 0.0, 2.0)]
        data = b"".join(struct.pack("<fff", *p) for p in pts)
        fields = [NS(name=n, offset=4 * i) for i, n in enumerate("xyz")]
        out = snap._pointcloud_ranges(NS(fields=fields, point_step=12, data=data, width=3, height=1,
                                         header=NS(frame_id="tof")))
        self.assertEqual((out["points"], out["of"], out["min_m"], out["max_m"]), (2, 3, 1.0, 2.0))

    def test_depth_centre_16uc1_and_invalid(self):
        w, h = 4, 4
        buf = bytearray(w * h * 2)
        struct.pack_into("<H", buf, (h // 2) * w * 2 + (w // 2) * 2, 3401)
        msg = NS(width=w, height=h, step=w * 2, encoding="16UC1", data=bytes(buf))
        self.assertEqual(snap._depth_centre(msg)["centre_m"], 3.401)
        msg.data = bytes(len(buf))
        self.assertIsNone(snap._depth_centre(msg)["centre_m"])


class TestLaunchLogParsing(unittest.TestCase):
    def test_launch_own_line(self):
        e = ros2_logs._parse_line("1795577191.47 [INFO] [launch.user]: Gemini 2 ready")
        self.assertEqual((e["severity"], e["node"], e["message"]), ("INFO", "launch.user", "Gemini 2 ready"))

    def test_wrapped_node_line_with_ansi(self):
        e = ros2_logs._parse_line("1795577197.6 [ros2_control_node-2] \x1b[33m[WARN] [1795577197.642] [controller_manager]: slow")
        self.assertEqual((e["severity"], e["node"], e["message"]), ("WARN", "controller_manager", "slow"))
        self.assertAlmostEqual(e["timestamp"], 1795577197.642)

    def test_plain_stdout_is_not_an_entry(self):
        self.assertIsNone(ros2_logs._parse_line("1795577191.5 [spawner-17] some plain output"))


class TestNewPlanVerbs(unittest.TestCase):
    def cmds(self, goal):
        return [s["commands"] for s in ros2_plan.compile_plan(goal, None, [])["steps"]]

    def test_save_place_and_list_in_one_goal(self):
        self.assertEqual(self.cmds("Save this spot as 'bench' and list my places"),
                         [["nav2 place save bench"], ["nav2 place list"]])

    def test_picture_status_and_map_name(self):
        self.assertEqual(self.cmds("take a picture"), [["look"]])
        self.assertEqual(self.cmds("snapshot"), [["snapshot"]])
        self.assertEqual(self.cmds("save the map as kitchen"), [["nav2 map save --name kitchen"]])

    def test_patrol_keeps_its_own_and(self):
        warnings = ros2_plan.compile_plan("patrol a and b", None, [])["warnings"]
        self.assertTrue(any("unknown place" in w for w in warnings))


if __name__ == "__main__":
    unittest.main()


class TestRelativeTimeArgs(unittest.TestCase):
    def test_negative_relative_time_is_joined_to_its_flag(self):
        import ros2_cli
        self.assertEqual(ros2_cli._join_relative_times(["logs", "query", "--after", "-30s", "--severity", "WARN"]),
                         ["logs", "query", "--after=-30s", "--severity", "WARN"])
        self.assertEqual(ros2_cli._join_relative_times(["logs", "query", "--before", "-2m"]), ["logs", "query", "--before=-2m"])

    def test_other_values_are_untouched(self):
        import ros2_cli
        argv = ["topics", "publish", "/t", "-5", "--after", "2026-01-01"]
        self.assertEqual(ros2_cli._join_relative_times(argv), argv)


class TestPlanRunRefusesPartialGoals(unittest.TestCase):
    def test_run_with_an_uncompiled_part_executes_nothing(self):
        import io
        import contextlib
        import json
        import types
        import ros2_plan
        buf = io.StringIO()
        args = types.SimpleNamespace(goal=["rotate", "90", "then", "fly"], run=True)
        with contextlib.redirect_stdout(buf):
            ros2_plan.cmd_plan(args)
        out = json.loads(buf.getvalue())
        self.assertFalse(out["executed"])
        self.assertIn("nothing ran", out["refused"])


class TestSettledSample(unittest.TestCase):
    @staticmethod
    def odom(lin, ang=0.0):
        return {"twist": {"twist": {"linear": {"x": lin, "y": 0.0, "z": 0.0}, "angular": {"x": 0.0, "y": 0.0, "z": ang}}}}

    def test_quiet_first_sample_needs_no_second_read(self):
        from ros2_preflight import settled_sample
        calls = []
        self.assertEqual(settled_sample(self.odom(0.0099), lambda: calls.append(1), 0.01), self.odom(0.0099))
        self.assertEqual(calls, [])

    def test_noise_spike_is_settled_by_the_next_sample(self):
        from ros2_preflight import settled_sample
        reads = iter([self.odom(0.002)])
        self.assertEqual(settled_sample(self.odom(0.0105), lambda: next(reads), 0.01), self.odom(0.002))

    def test_moving_robot_stays_blocked(self):
        from ros2_preflight import settled_sample
        got = settled_sample(self.odom(0.1), lambda: self.odom(0.09), 0.01)
        self.assertGreater(got["twist"]["twist"]["linear"]["x"], 0.01)

    def test_missing_next_sample_keeps_the_first(self):
        from ros2_preflight import settled_sample
        self.assertEqual(settled_sample(self.odom(0.5), lambda: None, 0.01), self.odom(0.5))


class TestSlowMove(unittest.TestCase):
    def test_normal_move_is_not_flagged(self):
        from ros2_topic import slow_move
        self.assertIsNone(slow_move(0.3, 0.1, 5.0))      # 3 s cruise + ramp
        self.assertIsNone(slow_move(0.1, 0.1, 1.9))

    def test_throttled_move_is_flagged(self):
        from ros2_topic import slow_move
        flag = slow_move(0.2, 0.2, 11.1)                  # Q21's backward leg
        self.assertEqual(flag["average_mps"], 0.018)
        self.assertIn("limiter", flag["hint"])

    def test_no_distance_or_speed_gives_nothing(self):
        from ros2_topic import slow_move
        self.assertIsNone(slow_move(0, 0.1, 9))
        self.assertIsNone(slow_move(0.2, 0, 9))


class TestBraking(unittest.TestCase):
    def test_decel_from_coast(self):
        from ros2_braking import decel_from_coast
        self.assertAlmostEqual(decel_from_coast(0.2, 0.06), 0.333, places=3)
        self.assertIsNone(decel_from_coast(0.2, 0.001))   # inside odometry noise
        self.assertIsNone(decel_from_coast(0, 0.05))

    def test_stored_value_replaces_the_default_zone(self):
        import json
        import tempfile
        from pathlib import Path
        import ros2_braking
        from ros2_topic import compute_decel_params
        original = ros2_braking.STORE
        with tempfile.TemporaryDirectory() as tmp:
            ros2_braking.STORE = Path(tmp) / "braking.json"
            try:
                ros2_braking.STORE.write_text(json.dumps({"linear_mps2": 0.25}))
                zone, _factor, meta = compute_decel_params({"linear": {"x": 0.2}}, 1.0, False, False, None)
                self.assertAlmostEqual(zone, 0.08, places=3)       # 0.2² / (2 × 0.25)
                self.assertEqual(meta["params_source"], "measured braking")
                ros2_braking.STORE.unlink()
                zone, _factor, meta = compute_decel_params({"linear": {"x": 0.2}}, 1.0, False, False, None)
                self.assertAlmostEqual(zone, 0.05, places=3)   # default 0.5 m/s² → 0.04, floor 0.05
            finally:
                ros2_braking.STORE = original


class TestNotes(unittest.TestCase):
    def setUp(self):
        import tempfile
        from pathlib import Path
        import ros2_notes
        self.mod, self.original = ros2_notes, ros2_notes.STORE
        self.tmp = tempfile.TemporaryDirectory()
        ros2_notes.STORE = Path(self.tmp.name) / "notes.json"

    def tearDown(self):
        self.mod.STORE = self.original
        self.tmp.cleanup()

    def test_remember_and_recall_newest_first(self):
        self.mod.remember("Dock is left of the door", ["dock"], now="2026-11-01T10:00:00")
        self.mod.remember("Map updated 2026-11-20", ["map"])
        self.assertEqual([n["id"] for n in self.mod.recall()], [2, 1])
        self.assertEqual([n["id"] for n in self.mod.recall("dock door")], [1])
        self.assertEqual([n["id"] for n in self.mod.recall(tag="map")], [2])
        self.assertEqual(self.mod.recall("charger"), [])

    def test_ids_are_never_reused(self):
        self.mod.remember("one")
        self.mod.remember("two")
        self.assertTrue(self.mod.forget(2))
        self.assertEqual(self.mod.remember("three")["id"], 3)
        self.assertFalse(self.mod.forget(99))

    def test_corrupt_store_reads_as_empty(self):
        self.mod.STORE.write_text("{not json")
        self.assertEqual(self.mod.recall(), [])
        self.assertEqual(self.mod.remember("fresh")["id"], 1)

    def test_notes_and_places_live_outside_the_profiles_dir(self):
        import ros2_nav2
        self.assertEqual(self.original.parent.name, ".artifacts")
        self.assertEqual(ros2_nav2._PLACE_DIR.parent.name, ".artifacts")  # a profile switch cannot touch either

    def test_plan_verbs(self):
        from ros2_plan import compile_plan
        cmd = compile_plan("remember that the dock is by the door")["steps"][0]["commands"][0]
        self.assertEqual(cmd, "notes remember 'the dock is by the door'")
        self.assertEqual(compile_plan("what do you remember about the dock")["steps"][0]["commands"][0], "notes recall dock")


class TestCancellation(unittest.TestCase):
    def test_spin_until_runs_on_cancel_then_reraises(self):
        import ros2_utils
        calls = []

        class Future:
            def done(self):
                return False

        def interrupted(node, timeout_sec):
            raise KeyboardInterrupt

        original = ros2_utils.rclpy
        ros2_utils.rclpy = types_ns(spin_once=interrupted)
        try:
            with self.assertRaises(KeyboardInterrupt):
                ros2_utils.spin_until(object(), Future(), 1e12, on_cancel=lambda: calls.append("cancelled"))
        finally:
            ros2_utils.rclpy = original
        self.assertEqual(calls, ["cancelled"])

    def test_spin_until_returns_done_state(self):
        import ros2_utils
        done = types_ns(done=lambda: True)
        self.assertTrue(ros2_utils.spin_until(object(), done, 0))

    def test_uninterruptible_restores_the_handlers(self):
        import signal
        import ros2_utils
        before = signal.getsignal(signal.SIGINT), signal.getsignal(signal.SIGTERM)
        with ros2_utils.uninterruptible():
            self.assertEqual(signal.getsignal(signal.SIGINT), signal.SIG_IGN)
            self.assertEqual(signal.getsignal(signal.SIGTERM), signal.SIG_IGN)
        self.assertEqual((signal.getsignal(signal.SIGINT), signal.getsignal(signal.SIGTERM)), before)

    def test_plan_stops_and_reports_when_a_step_is_cancelled(self):
        from ros2_plan import compile_plan, run_plan
        plan = compile_plan("move forward 0.1 m then take a picture", {
            **TestPlanSummary.SUMMARY}, [])
        seen = []

        def fake(argv):
            seen.append(argv[0])
            if argv[:2] == ["topics", "publish-until"]:
                return {"cancelled": True, "error": "interrupted"}, 130
            return ({"ready_for_motion": True, "drive_mode": "teleop"} if argv[0] == "preflight" else {"success": True}), 0

        report = run_plan(plan, fake, "/odom")
        self.assertTrue(report["cancelled"])
        self.assertFalse(report["ok"])
        self.assertEqual(seen[-1], "stop")        # velocity zeroed
        self.assertNotIn("look", seen)            # the picture step never ran
        self.assertNotIn("estop", seen)           # a cancel is not a failure: no latching e-stop

    def test_interrupt_inside_a_command_is_a_cancel(self):
        from ros2_plan import compile_plan, run_plan
        plan = compile_plan("move forward 0.1 m", TestPlanSummary.SUMMARY, [])

        def fake(argv):
            if argv[:2] == ["topics", "publish-until"]:
                raise KeyboardInterrupt
            return ({"ready_for_motion": True, "drive_mode": "teleop"} if argv[0] == "preflight" else {"success": True}), 0

        self.assertTrue(run_plan(plan, fake, "/odom")["cancelled"])


class TestPlanSummary:
    SUMMARY = {
        "cmd_vel_topic": "/cmd_vel", "velocity_topics": [{"topic": "/cmd_vel", "type": "geometry_msgs/msg/Twist"}],
        "localization_config": {"fused_sources": {"odom0": "/odom"}},
        "safety_limits": {"binding": {"linear_x": 0.2, "angular_z": 0.5}},
        "active_controllers": ["base_controller"],
        "capabilities": {"move_distance": {}, "rotate_to_heading": {}},
    }


def types_ns(**kwargs):
    import types
    return types.SimpleNamespace(**kwargs)
