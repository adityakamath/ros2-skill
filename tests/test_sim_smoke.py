#!/usr/bin/env python3
"""Smoke tests that run the real CLI against an already-running graph (e.g. `sim:=true`, started by you).

Read-only unless ROS2_SIM_MOTION=1, which also drives a 0.1 m closed-loop move. Enable with
`pytest tests/test_sim_smoke.py --sim` (or ROS2_SIM_TESTS=1). Every command's stdout must be clean JSON.
"""

import json
import os
import shlex
import subprocess
import sys
import unittest
from pathlib import Path

CLI = [sys.executable, str(Path(__file__).resolve().parents[1] / "scripts" / "ros2_cli.py")]
ENABLED = os.environ.get("ROS2_SIM_TESTS") == "1"


def run(*args, timeout=60):
    proc = subprocess.run([*CLI, *args], capture_output=True, text=True, timeout=timeout)
    try:
        return json.loads(proc.stdout), proc.returncode
    except ValueError:
        raise AssertionError(f"stdout is not clean JSON for {args}: {proc.stdout[:200]!r}")


@unittest.skipUnless(ENABLED, "needs a running graph: use --sim or ROS2_SIM_TESTS=1")
class TestSmoke(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        topics, rc = run("topics", "list", "--limit", "0")
        if rc != 0 or not topics.get("topics"):
            raise unittest.SkipTest("no ROS 2 graph visible")
        cls.topics = set(topics["topics"])

    def test_graph_has_the_robot_stack(self):
        for topic in ("/scan", "/cmd_vel_presafety"):
            self.assertIn(topic, self.topics)

    def test_preflight_reports_state_fields(self):
        data, _ = run("preflight", "motion", "--controller", "base_controller", "--odom-topic", "/base_controller/odom")
        self.assertIn("ready_for_motion", data)
        self.assertNotIn("covariance", json.dumps(data))

    def test_service_state_reader(self):
        data, rc = run("services", "state", "/emergency_stop", "/twist_switch")
        self.assertEqual(rc, 0)
        self.assertEqual(set(data["states"]), {"/emergency_stop", "/twist_switch"})

    def test_plan_uses_the_profile(self):
        data, _ = run("plan", "move forward 0.1 m")
        self.assertTrue(data["steps"])
        self.assertFalse(data["executed"])

    def test_hz_sees_best_effort_scan(self):
        data, rc = run("topics", "hz", "/scan", "--duration", "3")
        self.assertEqual(rc, 0, data)
        self.assertGreater(data["rate"], 1.0)

    def test_snapshot_reads_state_in_one_call(self):
        data, rc = run("snapshot", "--only", "scan,odom,state")
        self.assertEqual(rc, 0, data)
        self.assertIn("lidar", data)
        self.assertIn("service_state", data)

    def test_plan_run_refuses_a_goal_it_cannot_fully_compile(self):
        data, _ = run("plan", "--run", "move forward 0.1 m then fly")
        self.assertFalse(data["executed"])
        self.assertIn("nothing ran", data["refused"])

    def test_notes_round_trip(self):
        saved, _ = run("notes", "remember", "sim smoke note", "--tag", "smoke")
        try:
            found, _ = run("notes", "recall", "smoke", "note")
            self.assertTrue(any(n["id"] == saved["note"]["id"] for n in found["notes"]))
        finally:
            run("notes", "forget", str(saved["note"]["id"]))

    @unittest.skipUnless(os.environ.get("ROS2_SIM_MOTION") == "1", "set ROS2_SIM_MOTION=1 to drive the robot")
    def test_estop_interrupts_a_running_move_and_the_plan_reports_it(self):
        import time
        for _ in range(20):  # a move from the previous test may still be coasting; the pre-motion check would refuse to start
            snap, _ = run("snapshot", "--only", "odom")
            if (snap.get("pose") or {}).get("stationary"):
                break
            time.sleep(0.5)
        proc = subprocess.Popen([*CLI, "plan", "--run", "move forward 0.8 m at 0.1 m/s"], stdout=subprocess.PIPE,
                                stderr=subprocess.DEVNULL, text=True)
        try:
            time.sleep(4)
            estop, _ = run("estop")
            self.assertTrue(estop.get("success"), estop)
            out, _ = proc.communicate(timeout=30)
            report = json.loads(out)
            self.assertTrue(report.get("cancelled"), report)
        finally:
            if proc.poll() is None:
                proc.kill()
            run("services", "call", "/emergency_stop", '{"data": false}')

    @unittest.skipUnless(os.environ.get("ROS2_SIM_MOTION") == "1", "set ROS2_SIM_MOTION=1 to drive the robot")
    def test_closed_loop_move(self):
        plan, _ = run("plan", "move forward 0.1 m")
        command = next(c for step in plan["steps"] for c in step["commands"] if c.startswith("topics publish-until"))
        data, rc = run(*shlex.split(command), timeout=60)
        self.assertEqual(rc, 0, data)
        self.assertTrue(data["condition_met"])


if __name__ == "__main__":
    unittest.main()
