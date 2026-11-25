#!/usr/bin/env python3
"""SetBool state extraction from replayed service-introspection events."""

import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))

from ros2_robot_state import last_setbool_state  # pure function: no ROS needed


def ev(etype, seq, request=None, response=None, gid=(1, 2)):
    return {"info": {"event_type": etype, "client_gid": list(gid), "sequence_number": seq},
            "request": [{"data": request}] if request is not None else [],
            "response": [{"success": response}] if response is not None else []}


class TestLastSetBoolState(unittest.TestCase):
    def test_successful_call_sets_state(self):
        self.assertTrue(last_setbool_state([ev(1, 7, request=True), ev(2, 7, response=True)]))
        self.assertFalse(last_setbool_state([ev(1, 7, request=False), ev(2, 7, response=True)]))

    def test_failed_response_is_ignored(self):
        self.assertIsNone(last_setbool_state([ev(1, 7, request=True), ev(2, 7, response=False)]))

    def test_last_successful_call_wins(self):
        events = [ev(1, 1, request=True), ev(2, 1, response=True), ev(1, 2, request=False), ev(2, 2, response=True)]
        self.assertFalse(last_setbool_state(events))

    def test_unmatched_response_and_no_events_are_unknown(self):
        self.assertIsNone(last_setbool_state([ev(2, 9, response=True)]))
        self.assertIsNone(last_setbool_state([]))

    def test_requests_from_different_clients_do_not_mix(self):
        events = [ev(1, 1, request=True, gid=(1,)), ev(1, 1, request=False, gid=(2,)), ev(2, 1, response=True, gid=(1,))]
        self.assertTrue(last_setbool_state(events))


if __name__ == "__main__":
    unittest.main()


try:
    from ros2_preflight import _drop_keys, robot_state_blockers
    _PREFLIGHT_ERR = None
except Exception as exc:  # no ROS 2 environment
    _PREFLIGHT_ERR = exc


@unittest.skipIf(_PREFLIGHT_ERR, f"ros2_preflight not importable: {_PREFLIGHT_ERR}")
class TestRobotStateBlockers(unittest.TestCase):
    def test_estop_engaged_blocks(self):
        self.assertIn("emergency stop", robot_state_blockers(True, False, "teleop", "/twist_switch")[0])

    def test_mode_mismatch_blocks_with_fix_command(self):
        msg = robot_state_blockers(False, False, "nav2", "/twist_switch")[0]   # switch is teleop, command uses nav2
        self.assertIn("teleop mode", msg)
        self.assertIn('services call /twist_switch \'{"data": true}\'', msg)
        self.assertEqual(len(robot_state_blockers(False, True, "teleop", "/twist_switch")), 1)

    def test_matching_or_unknown_state_does_not_block(self):
        self.assertEqual(robot_state_blockers(False, False, "teleop", "/twist_switch"), [])
        self.assertEqual(robot_state_blockers(False, True, "nav2", "/twist_switch"), [])
        self.assertEqual(robot_state_blockers(None, None, "nav2", "/twist_switch"), [])
        self.assertEqual(robot_state_blockers(None, True, "nav2", None), [])

    def test_drop_keys_is_recursive(self):
        self.assertEqual(_drop_keys({"a": {"covariance": [0] * 36, "x": 1}, "b": [{"covariance": 1, "y": 2}]}, {"covariance"}),
                         {"a": {"x": 1}, "b": [{"y": 2}]})


@unittest.skipIf(_PREFLIGHT_ERR, f"ros2_preflight not importable: {_PREFLIGHT_ERR}")
class TestBattery(unittest.TestCase):
    def test_percent_accepts_fraction_or_percent_scales(self):
        from ros2_preflight import battery_percent
        self.assertEqual(battery_percent(0.5), 50.0)
        self.assertEqual(battery_percent(62.8), 62.8)
        self.assertIsNone(battery_percent(None))

    def test_blockers_and_warnings_use_system_battery_thresholds(self):
        from ros2_preflight import battery_blockers
        self.assertIn("critical", battery_blockers(15)[0][0])
        self.assertEqual(battery_blockers(25)[0], [])
        self.assertIn("low", battery_blockers(25)[1][0])
        self.assertEqual(battery_blockers(80), ([], []))
        self.assertEqual(battery_blockers(None), ([], []))
