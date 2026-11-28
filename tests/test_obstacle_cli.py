#!/usr/bin/env python3
"""The in-process velocity commands with the obstacle stop: what actually reaches the velocity topic."""

import contextlib
import json
import sys
import types
import unittest
from pathlib import Path
from unittest.mock import MagicMock, patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
sys.path.insert(0, str(Path(__file__).resolve().parent))

import ros2_obstacle  # noqa: E402
import test_obstacle as fakes  # noqa: E402


def _import_topic():
    """``ros2_topic`` with ROS stood in for, without leaving the stand-ins behind for other tests."""
    try:
        import rclpy  # noqa: F401
        import ros2_topic
        return ros2_topic, None
    except ImportError:
        pass
    node = types.ModuleType("rclpy.node")
    node.Node = type("Node", (), {})
    stubs = {name: MagicMock() for name in ("rclpy", "rclpy.qos", "rclpy.action")}
    stubs["rclpy.node"] = node
    stubs["rclpy"].node = node
    try:
        with patch.dict(sys.modules, stubs):
            import ros2_topic
        return ros2_topic, None
    except Exception as exc:
        return None, exc


ros2_topic, IMPORT_ERROR = _import_topic()


def speed(msg):
    return (msg["linear"]["x"], msg["linear"]["y"], msg["angular"]["z"])


@unittest.skipIf(fakes.np is None or ros2_topic is None, f"needs numpy and an importable ros2_topic: {IMPORT_ERROR}")
class TestInProcessPublish(unittest.TestCase):
    def setUp(self):
        self.sent, self.results, self.scan = [], [], fakes.scan()
        self.on_publish = lambda msg: None
        case = self

        class Guard(fakes.FakeGuard):
            """Real decisions; each look sees the scan the test currently provides."""

            def _survey(self):
                if case.scan is not None:
                    self._on_data(self.slots[0], case.scan)
                return super()._survey()

        def publish(msg):
            self.sent.append(speed(msg))
            self.on_publish(msg)

        publisher = types.SimpleNamespace(pub=types.SimpleNamespace(publish=publish))

        def host(sensing, tf_buffer=None):
            return types.SimpleNamespace(guard=Guard(fakes.FakeNode(), sensing, None), node=None, close=lambda: None)

        summary = fakes.summary(cmd_vel_topic="/cmd_vel")
        for target, name, value in (
                (ros2_topic, "ros2_context", contextlib.nullcontext), (ros2_topic, "ROS2CLI", MagicMock()),
                (ros2_topic, "resolve_topic_type", lambda *a: "geometry_msgs/msg/Twist"),
                (ros2_topic, "get_msg_type", lambda _t: object), (ros2_topic, "dict_to_msg", lambda _c, d: d),
                (ros2_topic, "TopicPublisher", lambda *a: publisher), (ros2_topic, "output", self.results.append),
                (ros2_topic, "validate_publish", lambda t, d, m: (True, "ok", None, {})),
                (ros2_topic, "heartbeat_touch", lambda: None),
                (ros2_obstacle, "GuardHost", host), (ros2_obstacle, "confirm_sources", lambda *a: None),
                (ros2_obstacle, "READY_TIMEOUT_S", 0.2), (ros2_obstacle, "COLD_TIMEOUT_S", 0.2),
                (ros2_obstacle, "ONE_SHOT_S", 0.15)):
            patcher = patch.object(target, name, value)
            patcher.start()
            self.addCleanup(patcher.stop)
        patcher = patch("ros2_profile_context.guard_inputs", lambda: (summary, {}))
        patcher.start()
        self.addCleanup(patcher.stop)

    def args(self, **extra):
        base = dict(topic="/cmd_vel", msg_type=None, max_vel=None, max_ang=None, rate=20.0, duration=None, timeout=None,
                    dry_run=False)
        base.update(extra)
        return types.SimpleNamespace(**base)

    def publish(self, msg, **extra):
        ros2_topic.cmd_topics_publish(self.args(msg=json.dumps(msg), **extra))
        return self.results[-1]

    def sequence(self, messages, durations):
        ros2_topic.cmd_topics_publish_sequence(
            self.args(messages=json.dumps(messages), durations=json.dumps(durations)))
        return self.results[-1]

    def test_a_single_velocity_message_is_withdrawn(self):
        result = self.publish(fakes.twist(0.2))
        self.assertTrue(result["success"], result)
        self.assertEqual(self.sent[0], (0.2, 0.0, 0.0))
        self.assertEqual(self.sent[1:], [(0.0, 0.0, 0.0)] * 4)

    def test_a_single_message_towards_an_obstacle_is_never_sent(self):
        self.scan = fakes.scan([(0, 0.3, 20)])
        result = self.publish(fakes.twist(0.2))
        self.assertFalse(result["success"])
        self.assertEqual(set(self.sent), {(0.0, 0.0, 0.0)})

    def test_a_timed_publish_stops_when_something_appears(self):
        def appear(_msg):
            if len(self.sent) == 3:
                self.scan = fakes.scan([(0, 0.3, 20)])

        self.on_publish = appear
        result = self.publish(fakes.twist(0.2), duration=3.0)
        self.assertEqual(result["stopped_by"], "obstacle", result)
        self.assertFalse(result["success"])
        self.assertEqual(self.sent[3:], [(0.0, 0.0, 0.0)] * 4)

    def test_the_guard_closes_while_ros_is_still_up(self):
        order = []

        @contextlib.contextmanager
        def context():
            yield
            order.append("ros shut down")

        with patch.object(ros2_topic, "ros2_context", context), \
                patch.object(ros2_obstacle, "confirm_sources", lambda *a: order.append("guard closed")):
            self.publish(fakes.twist(0.2), duration=0.1)
            self.sequence([fakes.twist(0.2), fakes.twist()], [0.1, 0.1])
        self.assertEqual(order, ["guard closed", "ros shut down"] * 2)

    def test_a_sequence_may_turn_and_stop_after_driving(self):
        steps = [fakes.twist(0.2), fakes.twist(wz=0.5), fakes.twist()]
        result = self.sequence(steps, [0.15, 0.15, 0.1])
        self.assertTrue(result["success"], result)
        self.assertNotIn("stopped_at_index", result)
        self.assertIn((0.0, 0.0, 0.5), self.sent)

    def test_a_sequence_holds_and_stops_before_an_unwatched_direction(self):
        self.scan = fakes.scan(angle_min=-1.2, angle_max=1.2, beams=120)   # looks forward only
        result = self.sequence([fakes.twist(0.2), fakes.twist(-0.2)], [0.15, 0.5])
        self.assertEqual(result["stopped_at_index"], 1, result)
        self.assertFalse(result["success"])
        self.assertFalse([s for s in self.sent if s[0] < 0])
        self.assertEqual(self.sent[-1], (0.0, 0.0, 0.0))


if __name__ == "__main__":
    unittest.main()
