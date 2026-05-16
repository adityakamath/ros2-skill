#!/usr/bin/env python3
"""ROS 2 Nav2 navigation commands.

Supports only NavigateToPose — the only action server configured on lekiwi_ros2.
NavigateThroughPoses and FollowWaypoints are intentionally absent (not in the
robot's Nav2 bringup).

Action status codes (action_msgs/GoalStatus):
  0 = UNKNOWN, 1 = ACCEPTED, 2 = EXECUTING, 3 = CANCELING,
  4 = SUCCEEDED, 5 = CANCELED, 6 = ABORTED
"""

import json
import math
import threading
import time

import rclpy

from ros2_utils import (
    ROS2CLI, get_action_type, get_msg_type, msg_to_dict, dict_to_msg,
    output, ros2_context,
)

# Default action server name for NavigateToPose.
_NAV2_ACTION = "/navigate_to_pose"

# Default timeout for navigation goals (navigation can take minutes).
_NAV_TIMEOUT = 120.0


# ---------------------------------------------------------------------------
# Pure Python helpers — no ROS required
# ---------------------------------------------------------------------------

def _yaw_to_quaternion(yaw_deg: float) -> dict:
    """Convert a yaw angle (degrees, planar rotation about Z) to a quaternion dict.

    For 2D navigation yaw is the only rotation component, so:
      x = 0, y = 0, z = sin(yaw/2), w = cos(yaw/2)

    Returns dict with keys x, y, z, w (all float).
    """
    half = math.radians(yaw_deg) / 2.0
    return {"x": 0.0, "y": 0.0, "z": math.sin(half), "w": math.cos(half)}


def _parse_waypoints(waypoints: list) -> list:
    """Parse a list of 'x,y' strings into a list of (float, float) tuples.

    Raises ValueError on any malformed entry (missing component, extra
    components, non-numeric values, empty string).
    """
    result = []
    for entry in waypoints:
        if not entry:
            raise ValueError(f"Empty waypoint string")
        parts = entry.split(",")
        if len(parts) != 2:
            raise ValueError(
                f"Waypoint '{entry}' must be 'x,y' (exactly two comma-separated values)"
            )
        try:
            x, y = float(parts[0]), float(parts[1])
        except ValueError:
            raise ValueError(f"Waypoint '{entry}' contains non-numeric value")
        result.append((x, y))
    return result


def _build_nav2_goal(x: float, y: float, yaw_deg=None, frame: str = "map") -> dict:
    """Build a NavigateToPose.Goal dict from x, y, optional yaw (degrees), and frame.

    Returns a nested dict matching the NavigateToPose.Goal message structure:
      {pose: {header: {frame_id}, pose: {position: {x,y,z}, orientation: {x,y,z,w}}}}
    """
    q = _yaw_to_quaternion(yaw_deg) if yaw_deg is not None else {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0}
    return {
        "pose": {
            "header": {"frame_id": frame},
            "pose": {
                "position": {"x": x, "y": y, "z": 0.0},
                "orientation": q,
            },
        }
    }


# ---------------------------------------------------------------------------
# Internal helpers
# ---------------------------------------------------------------------------

def _send_navigate_to_pose(node, action_class, action_name, goal_dict,
                            timeout, collect_feedback=False):
    """Send one NavigateToPose goal and block until result or timeout.

    Returns (success: bool, status: int, result_dict: dict, feedback_msgs: list).
    Raises on action-client / spin errors.
    """
    from rclpy.action import ActionClient  # noqa: PLC0415

    client = ActionClient(node, action_class, action_name)
    if not client.wait_for_server(timeout_sec=timeout):
        raise RuntimeError(
            f"navigate_to_pose action server not available "
            f"(waited {timeout}s) — is the Nav2 stack running?"
        )

    goal_msg = dict_to_msg(action_class.Goal, goal_dict)

    feedback_msgs = []
    feedback_lock = threading.Lock()

    def _fb_cb(fb_msg):
        if collect_feedback:
            with feedback_lock:
                feedback_msgs.append(msg_to_dict(fb_msg.feedback))

    future = client.send_goal_async(
        goal_msg,
        feedback_callback=_fb_cb if collect_feedback else None,
    )

    end = time.time() + timeout
    while time.time() < end and not future.done():
        rclpy.spin_once(node, timeout_sec=0.1)

    if not future.done():
        future.cancel()
        raise RuntimeError(f"Timeout waiting for goal acceptance ({timeout}s)")

    goal_handle = future.result()
    if not goal_handle.accepted:
        return False, 6, {}, []  # ABORTED

    result_future = goal_handle.get_result_async()

    end = time.time() + timeout
    while time.time() < end and not result_future.done():
        rclpy.spin_once(node, timeout_sec=0.1)

    if not result_future.done():
        result_future.cancel()
        raise RuntimeError(
            f"Navigation timeout after {timeout}s — goal still active. "
            "Call 'nav2 cancel' to stop the robot."
        )

    wrapped = result_future.result()
    status = int(wrapped.status)
    success = (status == 4)  # GoalStatus.SUCCEEDED
    result_dict = msg_to_dict(wrapped.result) if wrapped.result else {}

    with feedback_lock:
        fb = list(feedback_msgs)

    return success, status, result_dict, fb


def _status_name(status_int: int) -> str:
    _MAP = {0: "UNKNOWN", 1: "ACCEPTED", 2: "EXECUTING", 3: "CANCELING",
            4: "SUCCEEDED", 5: "CANCELED", 6: "ABORTED"}
    return _MAP.get(status_int, f"STATUS_{status_int}")


def _publish_zero_burst(node):
    """Publish 3 zero-velocity messages to the first Twist topic found.

    Best-effort — errors are silently ignored so cancel still reports success.
    """
    try:
        from geometry_msgs.msg import Twist  # noqa: PLC0415
        topics = node.get_topic_names_and_types()
        vel_topic = None
        for name, types in topics:
            for t in types:
                if "Twist" in t and "cmd_vel" in name.lower():
                    vel_topic = name
                    break
            if vel_topic:
                break

        if vel_topic:
            pub = node.create_publisher(Twist, vel_topic, 10)
            zero = Twist()
            for _ in range(3):
                pub.publish(zero)
                rclpy.spin_once(node, timeout_sec=0.02)
    except Exception:
        pass


# ---------------------------------------------------------------------------
# Command implementations
# ---------------------------------------------------------------------------

def cmd_nav2_go(args):
    """Send a NavigateToPose goal and wait for completion."""
    action_name = getattr(args, "action", _NAV2_ACTION)

    action_class = get_action_type("nav2_msgs/action/NavigateToPose")
    if not action_class:
        return output({
            "error": f"Cannot load nav2_msgs/action/NavigateToPose "
                     f"(needed for {action_name}) — "
                     "is nav2_msgs installed and sourced?",
        })

    goal_dict = _build_nav2_goal(args.x, args.y, args.yaw, args.frame)
    collect_feedback = getattr(args, "feedback", False)

    try:
        with ros2_context():
            node = ROS2CLI()
            success, status, result_dict, feedback_msgs = _send_navigate_to_pose(
                node, action_class, action_name, goal_dict,
                args.timeout, collect_feedback,
            )

        out = {
            "action": action_name,
            "success": success,
            "status": status,
            "status_name": _status_name(status),
            "goal": {"x": args.x, "y": args.y, "frame": args.frame},
        }
        if args.yaw is not None:
            out["goal"]["yaw_deg"] = args.yaw
        if not success:
            out["error"] = (
                f"Navigation {_status_name(status).lower()} "
                f"(status {status})"
            )
        if result_dict:
            out["result"] = result_dict
        if collect_feedback:
            out["feedback_msgs"] = feedback_msgs

        output(out)

    except Exception as e:
        output({"error": str(e)})


def cmd_nav2_cancel(args):
    """Cancel all active NavigateToPose goals and send a zero-velocity burst."""
    action_name = getattr(args, "action", _NAV2_ACTION)
    timeout = args.timeout

    try:
        from action_msgs.srv import CancelGoal  # noqa: PLC0415

        with ros2_context():
            node = ROS2CLI()
            client = node.create_client(CancelGoal, action_name + "/_action/cancel_goal")

            if not client.wait_for_service(timeout_sec=timeout):
                return output({
                    "error": (
                        f"navigate_to_pose cancel service not available after {timeout}s "
                        "— is Nav2 running and a goal active?"
                    )
                })

            req = CancelGoal.Request()  # empty goal_id = cancel ALL goals
            future = client.call_async(req)

            end = time.time() + timeout
            while time.time() < end and not future.done():
                rclpy.spin_once(node, timeout_sec=0.1)

            if not future.done():
                return output({"error": "Timeout waiting for cancel response"})

            resp = future.result()
            goals_cancelled = len(resp.goals_canceling) if resp and resp.goals_canceling else 0

            _publish_zero_burst(node)

        output({
            "cancelled": True,
            "goals_cancelled": goals_cancelled,
            "action": action_name,
            "zero_velocity_sent": True,
            "note": "Call 'estop' as well if the robot does not decelerate.",
        })

    except Exception as e:
        output({"error": str(e)})


def cmd_nav2_status(args):
    """Report current navigation status: active goal feedback + collision monitor."""
    timeout = args.timeout

    try:
        with ros2_context():
            node = ROS2CLI()

            # Detect whether navigate_to_pose action server is present.
            topic_types = node.get_topic_names_and_types()
            nav2_active = any(
                name == _NAV2_ACTION + "/_action/feedback"
                for name, _ in topic_types
            )

            result = {
                "nav2_available": nav2_active,
                "action_server": _NAV2_ACTION,
                "active_goal": None,
                "collision_monitor": None,
            }

            if nav2_active:
                # Attempt to grab one feedback message (short timeout).
                fb_timeout = min(timeout, 2.0)
                fb_class = get_msg_type(
                    "nav2_msgs/action/NavigateToPose_FeedbackMessage"
                )
                if fb_class:
                    received = []
                    done_event = threading.Event()

                    sub = node.create_subscription(
                        fb_class,
                        _NAV2_ACTION + "/_action/feedback",
                        lambda msg: (received.append(msg), done_event.set()),
                        10,
                    )
                    end = time.time() + fb_timeout
                    while time.time() < end and not done_event.is_set():
                        rclpy.spin_once(node, timeout_sec=0.1)

                    if received:
                        result["active_goal"] = msg_to_dict(received[0].feedback)

            # Attempt to get one collision monitor state message (best-effort).
            cm_class = get_msg_type("nav2_msgs/msg/CollisionMonitorState")
            if cm_class:
                cm_received = []
                cm_done = threading.Event()

                cm_sub = node.create_subscription(
                    cm_class,
                    "/collision_monitor_state",
                    lambda msg: (cm_received.append(msg), cm_done.set()),
                    10,
                )
                end = time.time() + min(timeout, 1.0)
                while time.time() < end and not cm_done.is_set():
                    rclpy.spin_once(node, timeout_sec=0.1)

                if cm_received:
                    result["collision_monitor"] = msg_to_dict(cm_received[0])

        output(result)

    except Exception as e:
        output({"error": str(e)})


def cmd_nav2_go_waypoints(args):
    """Navigate through a sequence of waypoints by chaining NavigateToPose goals.

    NavigateThroughPoses is not configured on lekiwi_ros2, so this command
    chains repeated NavigateToPose calls instead.
    """
    waypoints_raw = getattr(args, "waypoints", [])

    if not waypoints_raw:
        return output({"error": "At least one waypoint is required (format: 'x,y')"})

    try:
        waypoints = _parse_waypoints(waypoints_raw)
    except ValueError as e:
        return output({"error": f"Invalid waypoint: {e}"})

    action_name = getattr(args, "action", _NAV2_ACTION)
    yaw = getattr(args, "yaw", None)
    frame = getattr(args, "frame", "map")
    timeout = args.timeout
    stop_on_failure = getattr(args, "stop_on_failure", True)

    action_class = get_action_type("nav2_msgs/action/NavigateToPose")
    if not action_class:
        return output({
            "error": f"Cannot load nav2_msgs/action/NavigateToPose "
                     f"(needed for {action_name}) — "
                     "is nav2_msgs installed and sourced?",
        })

    results = []
    try:
        with ros2_context():
            node = ROS2CLI()

            for i, (x, y) in enumerate(waypoints):
                goal_dict = _build_nav2_goal(x, y, yaw, frame)
                wp_out = {
                    "waypoint_index": i,
                    "x": x,
                    "y": y,
                    "frame": frame,
                }
                if yaw is not None:
                    wp_out["yaw_deg"] = yaw

                try:
                    success, status, _, _ = _send_navigate_to_pose(
                        node, action_class, action_name, goal_dict, timeout
                    )
                    wp_out.update({
                        "success": success,
                        "status": status,
                        "status_name": _status_name(status),
                    })
                    if not success:
                        wp_out["error"] = (
                            f"Navigation {_status_name(status).lower()} "
                            f"(status {status})"
                        )
                except Exception as e:
                    wp_out.update({"success": False, "error": str(e)})

                results.append(wp_out)

                if not wp_out.get("success") and stop_on_failure:
                    break

    except Exception as e:
        return output({"error": str(e), "results": results})

    succeeded = sum(1 for r in results if r.get("success"))
    failed = sum(1 for r in results if not r.get("success"))
    total = len(waypoints)
    completed = len(results)

    output({
        "action": action_name,
        "total_waypoints": total,
        "completed": completed,
        "succeeded": succeeded,
        "failed": failed,
        "stopped_early": completed < total,
        "results": results,
    })


def cmd_nav2_initial_pose(args):
    """Publish an initial pose estimate to /initialpose for AMCL localisation.

    Publishes the pose 3 times to ensure AMCL receives it even under high load.
    Only meaningful when the Nav2 stack is running in 'amcl' mode.
    """
    try:
        from geometry_msgs.msg import PoseWithCovarianceStamped  # noqa: PLC0415

        q = _yaw_to_quaternion(args.yaw)

        with ros2_context():
            node = ROS2CLI()
            pub = node.create_publisher(PoseWithCovarianceStamped, "/initialpose", 10)

            msg = PoseWithCovarianceStamped()
            msg.header.frame_id = args.frame
            msg.pose.pose.position.x = float(args.x)
            msg.pose.pose.position.y = float(args.y)
            msg.pose.pose.position.z = 0.0
            msg.pose.pose.orientation.x = q["x"]
            msg.pose.pose.orientation.y = q["y"]
            msg.pose.pose.orientation.z = q["z"]
            msg.pose.pose.orientation.w = q["w"]

            # Publish 3 times; brief spin between sends.
            for _ in range(3):
                pub.publish(msg)
                rclpy.spin_once(node, timeout_sec=0.05)

        output({
            "published": True,
            "topic": "/initialpose",
            "x": args.x,
            "y": args.y,
            "yaw_deg": args.yaw,
            "frame": args.frame,
            "quaternion": q,
            "note": "Pose published 3× to /initialpose. "
                    "AMCL will use this to re-localise. "
                    "Only valid in amcl slam_mode.",
        })

    except Exception as e:
        output({"error": str(e)})
