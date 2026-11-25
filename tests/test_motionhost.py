#!/usr/bin/env python3
"""Moves hosted by the fast daemon: stop conditions, halting on cancel / client loss / timeout, and the client contract."""

import socket
import sys
import threading
import time
import types
import unittest
from pathlib import Path
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))

import ros2_condition  # noqa: E402
import ros2_fastmotion  # noqa: E402
import ros2_motionhost  # noqa: E402


def odom(x=0.0, y=0.0, yaw=0.0):
    import math
    return {"pose": {"pose": {"position": {"x": x, "y": y, "z": 0.0},
                              "orientation": {"x": 0.0, "y": 0.0, "z": math.sin(yaw / 2), "w": math.cos(yaw / 2)}}},
            "twist": {"twist": {"linear": {"x": 0.0, "y": 0.0, "z": 0.0}, "angular": {"x": 0.0, "y": 0.0, "z": 0.0}}}}


class FakeHub:
    """Stands in for the rclpy hub: records publishes and lets a test play the odometry."""

    def __init__(self):
        self.tracker = None
        self.sent, self.zeros = [], 0
        self.node = None
        self.latest = None
        self.sub = object()
        self._to_dict = lambda m: m

    def publish(self, topic, data):
        self.sent.append(data)

    def zero(self, topic=None):
        self.zeros += 1

    def feed(self, *messages, delay=0.02):
        for msg in messages:
            time.sleep(delay)
            if self.tracker is not None:
                self.tracker.update(msg)


MOVE = {"topic": "/cmd_vel", "msg": {"linear": {"x": 0.1, "y": 0.0, "z": 0.0}, "angular": {"x": 0.0, "y": 0.0, "z": 0.0}},
        "msg_type": "geometry_msgs/msg/Twist", "monitor": "/odom", "monitor_msg_type": "nav_msgs/msg/Odometry",
        "fields": ["pose.pose.position"], "euclidean": True, "threshold": 0.3, "rotate": None, "timeout": 2.0, "rate": 50.0}


class TestConditionTracker(unittest.TestCase):
    def test_euclidean_distance_met(self):
        stop = threading.Event()
        t = ros2_condition.ConditionTracker(["pose.pose.position"], "delta", 0.3, stop, euclidean=True)
        t.update(odom(0, 0))
        t.update(odom(0.2, 0))
        self.assertFalse(stop.is_set())
        t.update(odom(0.1, 0.3))
        self.assertTrue(stop.is_set())
        self.assertIsNotNone(t.end_msg)

    def test_rotation_accumulates_and_unwraps(self):
        stop = threading.Event()
        t = ros2_condition.ConditionTracker("pose.pose.orientation", None, None, stop, rotate=1.0)
        for yaw in (0.0, 0.4, 0.8, 1.05):
            t.update(odom(yaw=yaw))
        self.assertTrue(stop.is_set())
        self.assertAlmostEqual(t.rotation_delta, 1.05, places=3)

    def test_missing_field_is_an_error_not_a_hang(self):
        stop = threading.Event()
        t = ros2_condition.ConditionTracker(["pose.pose.nothing"], "delta", 0.3, stop, euclidean=True)
        t.update(odom())
        self.assertTrue(stop.is_set())
        self.assertIn("not found", t.field_error)


@patch.object(ros2_motionhost, "_types", lambda node, topic, given: given)
@patch("ros2_safety.validate_publish", lambda topic, data, mtype: (True, "ok", None, {}))
class TestRunMove(unittest.TestCase):
    def setUp(self):
        self.hub = FakeHub()
        patcher = patch.object(ros2_motionhost, "_hub", lambda *a, **k: self.hub)
        patcher.start()
        self.addCleanup(patcher.stop)
        ros2_motionhost._CANCEL.clear()

    def run_in_thread(self, params, conn=None):
        box = {}
        thread = threading.Thread(target=lambda: box.update(r=ros2_motionhost.run_move(None, params, conn)))
        thread.start()
        return thread, box

    def test_move_finishes_on_the_condition_and_zeroes_velocity(self):
        thread, box = self.run_in_thread(dict(MOVE))
        time.sleep(0.15)
        self.hub.feed(odom(0, 0), odom(0.15, 0), odom(0.31, 0))
        thread.join(5)
        result = box["r"]
        self.assertTrue(result["success"] and result["condition_met"], result)
        self.assertGreater(len(self.hub.sent), 2)
        self.assertGreaterEqual(self.hub.zeros, 4)
        self.assertEqual(result["source"], "fast daemon")
        self.assertNotIn("covariance", str(result["end_msg"]))

    def test_cancel_stops_publishing_and_zeroes(self):
        thread, box = self.run_in_thread(dict(MOVE, timeout=30.0))
        time.sleep(0.2)
        self.assertTrue(ros2_motionhost.cancel_motion()["cancelled"])
        thread.join(3)
        self.assertFalse(thread.is_alive())
        self.assertTrue(box["r"]["cancelled"])
        self.assertGreaterEqual(self.hub.zeros, 4)

    def test_client_that_disappears_halts_the_move(self):
        server, client = socket.socketpair()
        thread, box = self.run_in_thread(dict(MOVE, timeout=30.0), conn=server)
        time.sleep(0.2)
        client.close()  # Ctrl-C, kill -9 or a crash on the client side
        thread.join(3)
        server.close()
        self.assertFalse(thread.is_alive())
        self.assertTrue(box["r"]["cancelled"])
        self.assertGreaterEqual(self.hub.zeros, 4)

    def test_timeout_reports_not_met_and_still_zeroes(self):
        with patch("ros2_profile.load_profile_summary", lambda readonly=False: None):
            thread, box = self.run_in_thread(dict(MOVE, timeout=0.3))
            thread.join(5)
        self.assertFalse(box["r"]["success"])
        self.assertIn("Timeout", box["r"]["error"])
        self.assertGreaterEqual(self.hub.zeros, 4)

    def test_slow_down_zone_scales_the_command_near_the_target(self):
        thread, box = self.run_in_thread(dict(MOVE, slow_zone=0.2, slow_factor=0.25, timeout=1.0))
        time.sleep(0.1)
        self.hub.feed(odom(0, 0), odom(0.25, 0))   # 0.05 m left, inside the 0.2 m zone
        time.sleep(0.2)
        self.hub.feed(odom(0.31, 0))
        thread.join(5)
        speeds = [m["linear"]["x"] for m in self.hub.sent]
        self.assertAlmostEqual(speeds[0], 0.1)
        self.assertLess(min(speeds), 0.1)

    def test_cancel_with_nothing_running_is_harmless(self):
        self.assertFalse(ros2_motionhost.cancel_motion()["cancelled"])

    def test_only_one_move_at_a_time(self):
        thread, _box = self.run_in_thread(dict(MOVE, timeout=30.0))
        time.sleep(0.2)
        second = ros2_motionhost.run_move(None, dict(MOVE), None)
        self.assertTrue(second["blocked"])
        ros2_motionhost.cancel_motion()
        thread.join(3)


class TestClientContract(unittest.TestCase):
    PLAN = {"dry_run": False, "euclidean": True, "rotate_angle": None, "operator": "delta", "threshold": 0.3,
            "msg_data": {}, "field_paths": ["pose.pose.position"], "use_degrees": False, "clamp_notices": [],
            "safety_action": "ok", "safety_warning": None}
    ARGS = types.SimpleNamespace(topic="/cmd_vel", msg_type=None, monitor="/odom", monitor_msg_type=None, timeout=5,
                                 rate=None, slow_last=None, slow_factor=0.25)

    def test_no_socket_means_run_locally(self):
        with patch.object(ros2_fastmotion, "_request", lambda *a: ("unavailable", None)):
            self.assertIsNone(ros2_fastmotion.try_publish_until(self.ARGS, self.PLAN))

    def test_a_request_that_may_have_been_received_is_never_retried_locally(self):
        stops = []
        with patch.object(ros2_fastmotion, "_request", lambda *a: ("lost", None)), \
                patch.object(ros2_fastmotion, "stop_robot", lambda *a, **k: stops.append(1)):
            result = ros2_fastmotion.try_publish_until(self.ARGS, self.PLAN)
        self.assertTrue(result["cancelled"])
        self.assertEqual(stops, [1])           # velocity zeroed, and no second move

    def test_daemon_declining_before_moving_falls_back(self):
        with patch.object(ros2_fastmotion, "_request", lambda *a: ("ok", {"fallback": True})):
            self.assertIsNone(ros2_fastmotion.try_publish_until(self.ARGS, self.PLAN))

    def test_dry_run_and_generic_conditions_stay_local(self):
        self.assertIsNone(ros2_fastmotion.try_publish_until(self.ARGS, dict(self.PLAN, dry_run=True)))
        self.assertIsNone(ros2_fastmotion.try_publish_until(self.ARGS, dict(self.PLAN, euclidean=False, operator="above")))

    def test_kill_switch_env_disables_the_daemon_path(self):
        with patch.dict("os.environ", {"ROS2_SKILL_NO_FASTD": "1"}):
            self.assertEqual(ros2_fastmotion._request("stop", {}, 1), ("unavailable", None))


class TestPreflightInDaemon(unittest.TestCase):
    def test_blocks_when_odometry_is_moving_and_reports_state(self):
        import ros2_preflight
        moving = odom()
        moving["twist"]["twist"]["linear"]["x"] = 0.2
        params = {"controller": "base_controller", "odom_topic": "/odom"}
        ctrls = lambda p: {"controllers": [{"name": "base_controller", "state": "active", "type": "x"}]}   # noqa: E731
        with patch.object(ros2_preflight, "_robot_state_augmenter", lambda args: (lambda data: data)), \
                patch("time.sleep", lambda s: None):
            blocked = ros2_preflight.preflight_in_daemon(params, ctrls, lambda p: {"msg": moving})
            self.assertFalse(blocked["ready_for_motion"])
            ready = ros2_preflight.preflight_in_daemon(params, ctrls, lambda p: {"msg": odom()})
            self.assertTrue(ready["ready_for_motion"])

    def test_missing_odometry_is_an_error_with_the_controller(self):
        import ros2_preflight
        out = ros2_preflight.preflight_in_daemon(
            {"controller": "c", "odom_topic": "/odom"}, lambda p: {"controllers": []}, lambda p: {"error": "Timeout"})
        self.assertEqual(out["error"], "Timeout")


if __name__ == "__main__":
    unittest.main()


class TestHubHealth(unittest.TestCase):
    class Pub:
        def __init__(self, n):
            self.n = n

        def get_subscription_count(self):
            return self.n

    class Sub:
        def __init__(self, n):
            self.n = n

        def get_publisher_count(self):
            return self.n

    def hub(self, listeners, feeders):
        return types.SimpleNamespace(pubs={"/cmd_vel": self.Pub(listeners)}, sub=self.Sub(feeders))

    def test_healthy_hub_is_kept(self):
        self.assertFalse(ros2_motionhost._hub_stale(self.hub(1, 1)))

    def test_lost_listener_or_feeder_marks_the_hub_stale(self):
        self.assertTrue(ros2_motionhost._hub_stale(self.hub(0, 1)))   # nothing subscribes to the command topic any more
        self.assertTrue(ros2_motionhost._hub_stale(self.hub(1, 0)))   # odometry publisher gone

    def test_a_broken_hub_counts_as_stale(self):
        self.assertTrue(ros2_motionhost._hub_stale(types.SimpleNamespace(pubs=object(), sub=None)))
