#!/usr/bin/env python3
"""JSON ints must not reach float message fields: rosidl aborts the whole process on that."""

import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))

try:
    from ros2_utils import dict_to_msg, get_msg_type
    _ERR = None if get_msg_type("geometry_msgs/msg/Twist") else "geometry_msgs unavailable"
except Exception as exc:
    _ERR = str(exc)


@unittest.skipIf(_ERR, f"ROS 2 message types unavailable: {_ERR}")
class TestCoercion(unittest.TestCase):
    def test_int_values_become_floats_in_nested_twist(self):
        msg = dict_to_msg(get_msg_type("geometry_msgs/msg/TwistStamped"),
                          {"header": {"stamp": {"sec": 0}, "frame_id": ""},
                           "twist": {"linear": {"x": 1, "y": 0, "z": 0}, "angular": {"x": 0, "y": 0, "z": 2}}})
        self.assertIsInstance(msg.twist.linear.y, float)
        self.assertEqual(msg.twist.angular.z, 2.0)

    def test_float_arrays_and_whole_number_ints(self):
        msg = dict_to_msg(get_msg_type("geometry_msgs/msg/PoseWithCovarianceStamped"), {"pose": {"covariance": [0] * 36}})
        self.assertEqual(len(msg.pose.covariance), 36)
        self.assertEqual(dict_to_msg(get_msg_type("std_msgs/msg/Int32"), {"data": 3.0}).data, 3)


if __name__ == "__main__":
    unittest.main()


@unittest.skipIf(_ERR, f"ROS 2 message types unavailable: {_ERR}")
class TestStampingPublisher(unittest.TestCase):
    def test_zero_stamp_is_replaced_and_real_stamp_kept(self):
        from types import SimpleNamespace
        from unittest.mock import MagicMock
        import ros2_topic
        sent = []
        node = MagicMock()
        node.get_clock.return_value.now.return_value.to_msg.return_value = "NOW"
        pub = ros2_topic._StampingPublisher(SimpleNamespace(publish=sent.append), node)
        zero = SimpleNamespace(header=SimpleNamespace(stamp=SimpleNamespace(sec=0, nanosec=0)))
        real = SimpleNamespace(header=SimpleNamespace(stamp=SimpleNamespace(sec=5, nanosec=1)))
        plain = SimpleNamespace(linear=1)
        for msg in (zero, real, plain):
            pub.publish(msg)
        self.assertEqual(zero.header.stamp, "NOW")
        self.assertEqual(real.header.stamp.sec, 5)
        self.assertEqual(len(sent), 3)
