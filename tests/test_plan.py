#!/usr/bin/env python3
"""`plan` compiles goals to CLI commands deterministically and never guesses."""

import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))

from ros2_plan import compile_plan  # pure module: no ROS needed

SUMMARY = {
    "cmd_vel_topic": "/base_controller/cmd_vel",
    "velocity_topics": [{"topic": "/base_controller/cmd_vel", "type": "geometry_msgs/msg/TwistStamped"}],
    "localization_config": {"fused_sources": {"odom0": "/base_controller/odom"}},
    "safety_limits": {"binding": {"linear_x": 0.2, "linear_y": 0.17, "angular_z": 0.68}},
    "active_controllers": ["arm_controller", "base_controller"],
    "capabilities": {k: {} for k in ("move_distance", "rotate_to_heading", "strafe", "navigate_to_pose", "follow_waypoints")},
}
PLACES = [{"name": "kitchen", "x": 2.0, "y": 3.0}, {"name": "dock", "x": 0.0, "y": -1.0}]


def plan(goal, summary=SUMMARY, places=PLACES, **kwargs):
    return compile_plan(goal, summary, places, **kwargs)


class TestPlan(unittest.TestCase):
    def test_move_is_closed_loop_with_ceilings_and_calculated_timeout(self):
        step = plan("move forward 3 m")["steps"][0]
        cmd = step["commands"][0]
        self.assertIn("publish-until /base_controller/cmd_vel", cmd)
        self.assertIn("--monitor /base_controller/odom", cmd)
        self.assertIn("--delta 3", cmd)
        self.assertIn("--timeout 30", cmd)           # ceil(2 * 3 / 0.2)
        self.assertIn("--max-vel 0.2 --max-ang 0.68", cmd)
        self.assertIn('"x":0.2', cmd)

    def test_backward_and_cm_units(self):
        cmd = plan("move back 50 cm")["steps"][0]["commands"][0]
        self.assertIn('"x":-0.2', cmd)
        self.assertIn("--delta 0.5", cmd)

    def test_back_up_and_reverse_phrasings(self):
        for goal in ("back up 0.3 m at 0.1 m/s", "reverse 30 cm at 0.1 m/s", "move backwards 0.3 m at 0.1 m/s"):
            cmd = plan(goal)["steps"][0]["commands"][0]
            self.assertIn('"x":-0.1', cmd, goal)
            self.assertIn("--delta 0.3", cmd, goal)

    def test_rotate_sign_and_nav2_preference(self):
        self.assertEqual(plan("turn left 90 degrees")["steps"][0]["commands"], ["nav2 rotate 90"])
        self.assertEqual(plan("turn right 45")["steps"][0]["commands"], ["nav2 rotate -- -45"])

    def test_rotate_without_nav2_uses_publish_until_with_matching_signs(self):
        no_nav = dict(SUMMARY, capabilities={"rotate_to_heading": {}})
        cmd = plan("turn right 90", no_nav)["steps"][0]["commands"][0]
        self.assertIn("--rotate -90 --degrees", cmd)
        self.assertIn('"z":-0.5', cmd)

    def test_strafe_left_is_positive_y(self):
        self.assertIn('"y":0.17', plan("strafe left 1 m")["steps"][0]["commands"][0])
        self.assertIn('"y":-0.17', plan("strafe right 1 m")["steps"][0]["commands"][0])

    def test_strafe_refused_without_capability(self):
        diff = dict(SUMMARY, capabilities={"move_distance": {}})
        result = plan("strafe left 1 m", diff)
        self.assertEqual(result["steps"], [])
        self.assertIn("strafe", result["warnings"][0])

    def test_unknown_place_is_a_warning_not_a_guess(self):
        result = plan("go to the garage")
        self.assertEqual(result["steps"], [])
        self.assertIn("unknown place 'garage'", result["warnings"][0])

    def test_known_place_and_coordinates_and_patrol(self):
        self.assertEqual(plan("go to the kitchen")["steps"][0]["commands"], ["nav2 place go kitchen"])
        self.assertEqual(plan("go to 1.5, -0.3 yaw 90")["steps"][0]["commands"], ["nav2 go 1.5 -0.3 --yaw 90"])
        self.assertEqual(plan("patrol kitchen and dock")["steps"][0]["commands"], ["nav2 go-waypoints 2,3 0,-1"])

    def test_oversized_motion_is_refused(self):
        self.assertEqual(plan("move forward 60 m")["steps"], [])
        self.assertEqual(plan("turn left 4000 degrees")["steps"], [])

    def test_multi_step_order_and_unparsed_text_warns(self):
        result = plan("move forward 1 m then turn left 90 then dance")
        self.assertEqual([s["capability"] for s in result["steps"]], ["move_distance", "rotate_to_heading"])
        self.assertEqual([s["step"] for s in result["steps"]], [1, 2])
        self.assertIn("'dance': not understood", result["warnings"][0])

    def test_preflight_picks_base_controller(self):
        self.assertIn("--controller base_controller --odom-topic /base_controller/odom", plan("move forward 1 m")["preflight"])
        self.assertIsNone(plan("go to the dock")["preflight"])

    def test_plain_twist_payload(self):
        summary = dict(SUMMARY, velocity_topics=[{"topic": "/base_controller/cmd_vel", "type": "geometry_msgs/msg/Twist"}])
        self.assertNotIn("header", plan("move forward 1 m", summary)["steps"][0]["commands"][0])

    def test_no_profile_uses_placeholders_and_warns(self):
        result = plan("move forward 1 m", None, [])
        self.assertIn("<VEL_TOPIC>", result["steps"][0]["commands"][0])
        self.assertIn("no profile loaded", result["warnings"][0])


if __name__ == "__main__":
    unittest.main()


LEKIWI = dict(
    SUMMARY,
    velocity_chain={"teleop_input": "/cmd_vel_teleop", "nav2_input": "/cmd_vel_smoothed", "mode_service": "/twist_switch"},
    toggles={"emergency_stop": "/emergency_stop", "twist_switch": "/twist_switch", "waypoint_follow": "/waypoint_follow"},
    robot_services={"record_waypoint": {"service": "/record_waypoint", "type": "std_srvs/srv/SetBool", "request": {"data": True}},
                    "reset_waypoints": {"service": "/reset_waypoints", "type": "std_srvs/srv/SetBool", "request": {"data": True}},
                    "save_map": {"service": "/save_map", "type": "std_srvs/srv/SetBool", "request": {"data": True}}},
    hardware_fitted={"gemini2": True, "lidar": True},
    tf_frames={"odom_frame": "odom", "base_frame": "base_footprint"},
)


class TestRobotSpecificPlans(unittest.TestCase):
    def test_mode_switch_inserted_only_when_the_velocity_input_changes(self):
        caps = [s["capability"] for s in plan("move forward 1 m then turn left 90 then go to the dock", LEKIWI)["steps"]]
        self.assertEqual(caps, ["set_drive_mode", "move_distance", "set_drive_mode", "rotate_to_heading", "navigate_to_pose"])
        steps = plan("move forward 1 m", LEKIWI)["steps"]
        self.assertEqual(steps[0]["commands"], ["services call /twist_switch '{\"data\": false}'"])
        nav = plan("go to the dock", LEKIWI)["steps"][0]
        self.assertEqual(nav["commands"], ["services call /twist_switch '{\"data\": true}'"])

    def test_no_mode_steps_without_a_twist_switch(self):
        self.assertNotIn("set_drive_mode", [s["capability"] for s in plan("move forward 1 m then turn left 90")["steps"]])

    def test_preflight_source_follows_the_first_motion_input(self):
        self.assertIsNone(plan("go to the dock", LEKIWI)["preflight"])
        self.assertIn("--source teleop", plan("move forward 1 m", LEKIWI)["preflight"])

    def test_patrol_and_waypoint_services(self):
        start = plan("start patrol", LEKIWI)["steps"]
        self.assertEqual(start[-1]["commands"], ["services call /waypoint_follow '{\"data\": true}'"])
        stop = plan("stop patrol", LEKIWI)["steps"][-1]
        self.assertEqual(stop["commands"], ["services call /waypoint_follow '{\"data\": false}'"])
        self.assertEqual(plan("record waypoint", LEKIWI)["steps"][0]["commands"], ["services call /record_waypoint '{\"data\": true}'"])
        self.assertEqual(plan("reset waypoints", LEKIWI)["steps"][0]["commands"], ["services call /reset_waypoints '{\"data\": true}'"])

    def test_save_map_prefers_robot_service_then_falls_back(self):
        self.assertEqual(plan("save the map", LEKIWI)["steps"][0]["commands"], ["services call /save_map '{\"data\": true}'"])
        self.assertEqual(plan("save the map")["steps"][0]["commands"], ["nav2 map save"])

    def test_robot_without_the_service_warns(self):
        result = plan("record waypoint", SUMMARY)
        self.assertEqual(result["steps"], [])
        self.assertIn("record_waypoint", result["warnings"][0])

    def test_look_around_captures_each_view_and_rotates_between(self):
        steps = plan("look around 4 views", LEKIWI, artifacts_dir="/skill/.artifacts")["steps"]
        look = steps[-1]["commands"]
        self.assertEqual(sum(c.startswith("topics capture-image --topic /gemini2/color/image_raw") for c in look), 4)
        self.assertEqual(look.count("nav2 rotate 90"), 3)
        self.assertIn("--output /skill/.artifacts/look_1of4.jpg", look[0])
        self.assertIn("tf lookup odom base_footprint", look)
        self.assertEqual(steps[0]["capability"], "set_drive_mode")

    def test_look_around_needs_nav2_and_sane_view_count(self):
        no_nav = dict(LEKIWI, capabilities={"rotate_to_heading": {}})
        self.assertEqual(plan("look around", no_nav)["steps"], [])
        self.assertEqual(plan("look around 40 views", LEKIWI)["steps"], [])


class TestSpeedPhrases(unittest.TestCase):
    def test_top_speed_uses_the_profile_ceiling(self):
        fast = dict(SUMMARY, safety_limits={"binding": {"linear_x": 0.35, "linear_y": 0.2, "angular_z": 0.68}})
        cmd = plan("move forward 2 meters at its top speed", fast)["steps"][0]["commands"][0]
        self.assertIn('"x":0.35', cmd)
        self.assertIn("--timeout 15", cmd)           # ceil(2 * 2 / 0.35) = 12, minimum 15
        self.assertIn("--max-vel 0.35", cmd)

    def test_explicit_speed_is_capped_at_the_ceiling(self):
        self.assertIn('"x":0.1', plan("move forward 1 m at 0.1 m/s")["steps"][0]["commands"][0])
        self.assertIn('"x":0.2', plan("move forward 1 m at 5 m/s")["steps"][0]["commands"][0])

    def test_default_speed_without_a_phrase_is_unchanged(self):
        self.assertIn('"x":0.2', plan("move forward 1 m")["steps"][0]["commands"][0])


def fake_cli(responses, log):
    """run_cli stand-in: *responses* maps an argv prefix (tuple) to its (dict, rc); every call is logged."""
    def run(argv):
        log.append(list(argv))
        for prefix, reply in responses.items():
            if tuple(argv[:len(prefix)]) == prefix:
                return reply
        return {}, 0
    return run


READY = ({"ready_for_motion": True, "drive_mode": "teleop", "odom": {"pose": {"pose": {"position": {"x": 1.0, "y": 0.0}}}}}, 0)
MOVED = ({"success": True, "condition_met": True, "euclidean_distance": 1.002, "duration": 6.1}, 0)
STOPPED = ({"verified_stationary": True, "final_position": {"x": 2.05, "y": 0.0}}, 0)


class TestRunPlan(unittest.TestCase):
    def setUp(self):
        from ros2_plan import run_plan
        self.run_plan = run_plan

    def test_runs_check_then_move_then_stop_and_reports_the_distance(self):
        log = []
        result = compile_plan("move forward 1 m at top speed", LEKIWI, [])
        report = self.run_plan(result, fake_cli({("preflight",): READY, ("topics", "publish-until"): MOVED, ("stop",): STOPPED}, log), "/odom")
        self.assertTrue(report["ok"])
        self.assertEqual([a[0] for a in log], ["preflight", "topics", "stop"])     # teleop mode already: no switch
        self.assertEqual(report["moved_m"], 1.05)
        self.assertTrue(report["steps"][-1]["result"]["stopped"]["verified_stationary"])

    def test_blocked_preflight_means_no_motion(self):
        log = []
        blocked = ({"ready_for_motion": False, "blocked_by": ["battery is critical"]}, 0)
        result = compile_plan("move forward 1 m", LEKIWI, [])
        report = self.run_plan(result, fake_cli({("preflight",): blocked}, log), "/odom")
        self.assertFalse(report["ok"])
        self.assertTrue(report["blocked"])
        self.assertNotIn("topics", [a[0] for a in log])

    def test_wrong_drive_mode_switches_before_moving(self):
        log = []
        nav2_mode = ({"ready_for_motion": False, "drive_mode": "nav2"}, 0)
        result = compile_plan("move forward 1 m", LEKIWI, [])
        calls = iter([nav2_mode, READY])
        run = fake_cli({("topics", "publish-until"): MOVED, ("stop",): STOPPED}, log)
        report = self.run_plan(result, lambda argv: next(calls) if argv[0] == "preflight" else run(argv), "/odom")
        self.assertTrue(report["ok"])
        self.assertIn(["services", "call", "/twist_switch", '{"data": false}'], log)

    def test_limited_move_reports_the_cause_and_does_not_latch_the_estop(self):
        log = []
        limited = ({"success": False, "condition_met": False,
                    "limited_downstream": {"commanded_mps": 0.2, "executed_mps": 0.025, "limiter_node": "collision_monitor"}}, 0)
        result = compile_plan("move forward 1 m", LEKIWI, [])
        report = self.run_plan(result, fake_cli({("preflight",): READY, ("topics", "publish-until"): limited, ("stop",): STOPPED}, log), "/odom")
        self.assertFalse(report["ok"])
        self.assertEqual(report["cause"]["limiter_node"], "collision_monitor")
        self.assertEqual(log[-1][0], "stop")
        self.assertNotIn("estop", [a[0] for a in log])

    def test_failed_motion_latches_the_estop_and_stops_the_run(self):
        log = []
        timeout = ({"success": True, "condition_met": False}, 0)
        result = compile_plan("move forward 1 m then turn left 90", LEKIWI, [])
        report = self.run_plan(result, fake_cli({("preflight",): READY, ("topics", "publish-until"): timeout}, log), "/odom")
        self.assertFalse(report["ok"])
        self.assertEqual(log[-1][0], "estop")
        self.assertNotIn(["nav2", "rotate", "90"], log)           # the second step never ran

    def test_empty_plan_is_not_executed(self):
        report = self.run_plan(compile_plan("dance", LEKIWI, []), fake_cli({}, []), "/odom")
        self.assertFalse(report["executed"])


class TestGist(unittest.TestCase):
    def test_service_reply_message_is_kept(self):
        from ros2_plan import _gist
        out = {"service": "/record_waypoint", "success": True, "result": {"success": True, "message": "Waypoint 0 recorded at (1, 2)"}}
        self.assertEqual(_gist(out)["message"], "Waypoint 0 recorded at (1, 2)")


class TestPlanFewerTurns(unittest.TestCase):
    """Verbs that fold several model turns into one plan."""

    def caps(self, goal):
        return [s["capability"] for s in plan(goal)["steps"] if s["capability"] != "set_drive_mode"]

    def test_publish_until_carries_message_types_so_it_skips_discovery(self):
        cmd = plan("move forward 1 m")["steps"][0]["commands"][0]
        self.assertIn("--msg-type geometry_msgs/msg/TwistStamped --monitor-msg-type nav_msgs/msg/Odometry", cmd)

    def test_no_types_without_a_profile_type(self):
        cmd = plan("move forward 1 m", summary={**SUMMARY, "velocity_topics": []})["steps"][0]["commands"][0]
        self.assertNotIn("--msg-type", cmd)

    def test_rotate_direction_is_optional_and_may_trail(self):
        self.assertEqual(plan("rotate 90 degrees")["steps"][-1]["inputs"]["degrees"], 90)
        self.assertEqual(plan("spin 45 degrees right")["steps"][-1]["inputs"]["degrees"], -45)
        self.assertEqual(plan("rotate clockwise 30")["steps"][-1]["inputs"]["degrees"], -30)
        self.assertEqual(plan("rotate -30")["steps"][-1]["inputs"]["degrees"], -30)

    def test_turn_around_is_180_left(self):
        self.assertEqual(plan("turn around")["steps"][-1]["inputs"]["degrees"], 180)

    def test_rotate_back_reverses_the_last_motion_not_the_picture(self):
        steps = plan("rotate 90 degrees then take a picture then rotate back")["steps"]
        self.assertEqual([s["inputs"].get("degrees") for s in steps if s["capability"] == "rotate_to_heading"], [90, -90])

    def test_undo_reverses_move_and_strafe(self):
        move = plan("move forward 0.2 m and undo that")["steps"]
        self.assertIn('"x":-0.2', move[-1]["commands"][0])
        strafe = plan("strafe left 0.2 m then come back")["steps"][-1]
        self.assertEqual(strafe["inputs"]["direction"], "right")

    def test_undo_with_nothing_before_it_is_a_warning(self):
        result = plan("go back")
        self.assertEqual(result["steps"], [])
        self.assertIn("nothing to undo", result["warnings"][0])

    def test_battery_surroundings_and_look_at(self):
        self.assertEqual(plan("report the battery")["steps"][0]["commands"], ["snapshot --only battery,state"])
        self.assertEqual(plan("is the path ahead clear")["steps"][0]["commands"], ["snapshot --only scan,tof,depth"])
        self.assertEqual(plan("what is around me")["steps"][0]["commands"], ["snapshot --only scan,tof,depth"])
        self.assertEqual(plan("look at the desk")["steps"][0]["commands"], ["look"])

    def test_move_picture_battery_in_one_goal(self):
        self.assertEqual(self.caps("move forward 0.3 m, take a picture, and report the battery"),
                         ["move_distance", "capture_image", "snapshot"])
