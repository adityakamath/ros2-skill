#!/usr/bin/env python3
"""QoS auto-matching for topic subscribers."""

import sys
import unittest
from pathlib import Path
from types import SimpleNamespace

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))

try:
    from rclpy.qos import ReliabilityPolicy, qos_profile_sensor_data, qos_profile_system_default
    import ros2_topic
    _IMPORT_ERROR = None
except Exception as exc:  # no ROS 2 environment
    _IMPORT_ERROR = exc


def _node(*reliabilities, raises=False):
    def info(_topic):
        if raises:
            raise RuntimeError("graph unavailable")
        return [SimpleNamespace(qos_profile=SimpleNamespace(reliability=r)) for r in reliabilities]
    return SimpleNamespace(get_publishers_info_by_topic=info)


@unittest.skipIf(_IMPORT_ERROR, f"rclpy not importable: {_IMPORT_ERROR}")
class TestPublisherMatchedQos(unittest.TestCase):
    def test_best_effort_publisher_gets_sensor_qos(self):
        node = _node(ReliabilityPolicy.RELIABLE, ReliabilityPolicy.BEST_EFFORT)
        self.assertIs(ros2_topic._publisher_matched_qos(node, "/odom"), qos_profile_sensor_data)

    def test_reliable_or_no_publishers_get_default_qos(self):
        for node in (_node(ReliabilityPolicy.RELIABLE), _node()):
            self.assertIs(ros2_topic._publisher_matched_qos(node, "/t"), qos_profile_system_default)

    def test_graph_error_falls_back_to_default(self):
        self.assertIs(ros2_topic._publisher_matched_qos(_node(raises=True), "/t"), qos_profile_system_default)


if __name__ == "__main__":
    unittest.main()
