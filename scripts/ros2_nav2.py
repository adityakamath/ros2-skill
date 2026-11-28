#!/usr/bin/env python3
"""ROS 2 Nav2 navigation commands.

Supports NavigateToPose, waypoint navigation, map lifecycle management
(``nav map list/save/load/delete``) and navigation mode detection /
switching (``nav mode get/set``).

Action status codes (action_msgs/GoalStatus):
  0 = UNKNOWN, 1 = ACCEPTED, 2 = EXECUTING, 3 = CANCELING,
  4 = SUCCEEDED, 5 = CANCELED, 6 = ABORTED

Nav2 lifecycle transition IDs (lifecycle_msgs/msg/Transition):
  1 = CONFIGURE, 2 = CLEANUP, 3 = ACTIVATE, 4 = DEACTIVATE
  5 = UNCONFIGURED_SHUTDOWN, 6 = INACTIVE_SHUTDOWN, 7 = ACTIVE_SHUTDOWN

Nav2 lifecycle state IDs (lifecycle_msgs/msg/State):
  0 = UNKNOWN, 1 = UNCONFIGURED, 2 = INACTIVE, 3 = ACTIVE, 4 = FINALIZED
"""

import json
import math
import os
import pathlib
import signal
import subprocess
import sys
import threading
import time
import uuid
from typing import Optional

import rclpy

import ros2_obstacle
from ros2_motion_core import prepare_guard
from ros2_utils import (
    ROS2CLI, call_ros_service, get_action_type, get_msg_type, get_srv_type, msg_to_dict, dict_to_msg,
    output, ros2_context, spin_until, uninterruptible,
)

# Default action server name for NavigateToPose.
_NAV2_ACTION = "/navigate_to_pose"

# Default timeout for navigation goals (navigation can take minutes).
_NAV_TIMEOUT = 120.0

_MISSION_DIR = pathlib.Path(__file__).resolve().parent.parent / ".artifacts" / "missions"
from ros2_places import PLACE_DIR as _PLACE_DIR  # noqa: E402  (shared with `plan`, which must not import rclpy)


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

    def _late_goal():  # interrupted while the goal was being accepted: cancel it if it was
        if future.done() and future.result() is not None and future.result().accepted:
            _cancel_goal_and_halt(node, future.result())
        else:
            _publish_zero_burst(node)

    spin_until(node, future, time.time() + timeout, on_cancel=_late_goal)

    if not future.done():
        future.cancel()
        raise RuntimeError(f"Timeout waiting for goal acceptance ({timeout}s)")

    goal_handle = future.result()
    if not goal_handle.accepted:
        return False, 6, {}, []  # ABORTED

    result_future = goal_handle.get_result_async()

    spin_until(node, result_future, time.time() + timeout, on_cancel=lambda: _cancel_goal_and_halt(node, goal_handle))

    if not result_future.done():
        result_future.cancel()
        with uninterruptible():  # a timeout is a cancel path too: never leave the goal running
            _cancel_goal_and_halt(node, goal_handle)
        raise RuntimeError(
            f"Navigation timeout after {timeout}s — the goal was cancelled and velocity zeroed."
        )

    wrapped = result_future.result()
    status = int(wrapped.status)
    success = (status == 4)  # GoalStatus.SUCCEEDED
    result_dict = msg_to_dict(wrapped.result) if wrapped.result else {}

    with feedback_lock:
        fb = list(feedback_msgs)

    return success, status, result_dict, fb


def _cancel_goal_and_halt(node, goal_handle):
    """Interrupt path for a running Nav2 goal: cancel it (so it cannot resume later) and zero the base."""
    if goal_handle is not None:
        future = goal_handle.cancel_goal_async()
        end = time.time() + 1.5
        while time.time() < end and not future.done():
            rclpy.spin_once(node, timeout_sec=0.05)
    _publish_zero_burst(node)


def _status_name(status_int: int) -> str:
    _MAP = {0: "UNKNOWN", 1: "ACCEPTED", 2: "EXECUTING", 3: "CANCELING",
            4: "SUCCEEDED", 5: "CANCELED", 6: "ABORTED"}
    return _MAP.get(status_int, f"STATUS_{status_int}")


def _publish_zero_burst(node):
    """Publish 3 zero-velocity messages to every velocity topic, using each topic's own type.

    Best-effort — errors are silently ignored so cancel still reports success.
    """
    try:
        from ros2_topic import VELOCITY_TYPES, _zero_msg  # noqa: PLC0415
        from ros2_utils import get_msg_type  # noqa: PLC0415
        pubs = []
        for name, types in node.get_topic_names_and_types():
            vtype = next((t for t in types if t in VELOCITY_TYPES), None)
            msg_class = get_msg_type(vtype) if vtype else None
            if msg_class:
                pubs.append((node.create_publisher(msg_class, name, 10), _zero_msg(msg_class)))
        for _ in range(3):
            for pub, msg in pubs:
                pub.publish(msg)
            rclpy.spin_once(node, timeout_sec=0.02)
    except Exception:
        pass


def _mission_path(mission_id: str) -> pathlib.Path:
    _MISSION_DIR.mkdir(parents=True, exist_ok=True)
    safe_id = "".join(ch for ch in mission_id if ch.isalnum() or ch in ("-", "_")) or "mission"
    return _MISSION_DIR / f"{safe_id}.json"


def _load_mission_state(mission_id: str) -> Optional[dict]:
    path = _mission_path(mission_id)
    if not path.exists():
        return None
    try:
        return json.loads(path.read_text(encoding="utf-8"))
    except Exception:
        return None


def _save_mission_state(state: dict) -> pathlib.Path:
    state = dict(state)
    state["updated_at"] = time.time()
    path = _mission_path(state["mission_id"])
    path.write_text(json.dumps(state, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    return path


def _mission_summary(state: dict) -> dict:
    history = state.get("history", []) if isinstance(state.get("history"), list) else []
    return {
        "mission_id": state.get("mission_id"),
        "name": state.get("name"),
        "status": state.get("status"),
        "current_step": state.get("current_step", -1),
        "step_count": len(state.get("steps", [])) if isinstance(state.get("steps"), list) else 0,
        "history_count": len(history),
        "runner_pid": state.get("runner_pid"),
        "failure_reason": state.get("failure_reason"),
    }


def _place_path(place_name: str) -> pathlib.Path:
    _PLACE_DIR.mkdir(parents=True, exist_ok=True)
    safe_name = "".join(ch for ch in place_name if ch.isalnum() or ch in ("-", "_")) or "place"
    return _PLACE_DIR / f"{safe_name}.json"


def _load_place(place_name: str) -> Optional[dict]:
    path = _place_path(place_name)
    if not path.exists():
        return None
    try:
        return json.loads(path.read_text(encoding="utf-8"))
    except Exception:
        return None


def _save_place(place: dict) -> pathlib.Path:
    path = _place_path(place["name"])
    payload = dict(place)
    payload["updated_at"] = time.time()
    path.write_text(json.dumps(payload, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    return path


def _list_places() -> list[dict]:
    from ros2_places import list_places
    return list_places(_PLACE_DIR)


def _current_pose_from_topic(topic_name: str, timeout: float) -> Optional[dict]:
    msg_type = None
    if topic_name.endswith("/amcl_pose"):
        msg_type = "geometry_msgs/msg/PoseWithCovarianceStamped"
    elif topic_name.endswith("/odom"):
        msg_type = "nav_msgs/msg/Odometry"
    else:
        msg_type = None

    msg_class = get_msg_type(msg_type) if msg_type else None
    if not msg_class:
        return None

    try:
        with ros2_context():
            node = ROS2CLI()
            received = []
            done = threading.Event()

            def _cb(msg):
                received.append(msg_to_dict(msg))
                done.set()

            node.create_subscription(msg_class, topic_name, _cb, 10)
            end = time.time() + timeout
            while time.time() < end and not done.is_set():
                rclpy.spin_once(node, timeout_sec=0.1)

        if not received:
            return None

        msg = received[0]
        if "pose" in msg and isinstance(msg["pose"], dict):
            pose = msg["pose"]
            if "pose" in pose and isinstance(pose["pose"], dict):
                pose = pose["pose"]
        elif "pose" in msg and isinstance(msg["pose"], dict):
            pose = msg["pose"]
        else:
            return None

        position = pose.get("position", {}) if isinstance(pose, dict) else {}
        orientation = pose.get("orientation", {}) if isinstance(pose, dict) else {}
        return {
            "x": float(position.get("x", 0.0)),
            "y": float(position.get("y", 0.0)),
            "yaw_deg": _yaw_from_quaternion(orientation),
            "frame": "map" if topic_name.endswith("/amcl_pose") else "odom",
            "source_topic": topic_name,
        }
    except Exception:
        return None


def _yaw_from_quaternion(quat: dict) -> float:
    x = float(quat.get("x", 0.0))
    y = float(quat.get("y", 0.0))
    z = float(quat.get("z", 0.0))
    w = float(quat.get("w", 1.0))
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.degrees(math.atan2(siny_cosp, cosy_cosp))


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


def cmd_nav2_rotate(args):
    """Rotate in place by sending a nav2_msgs/Spin goal to Nav2.

    Unlike publish-until with angular velocity, Nav2 handles the spin as a
    full action: it monitors for obstacles and aborts if the path is blocked.
    """
    _SPIN_ACTION = "/spin"

    action_class = get_action_type("nav2_msgs/action/Spin")
    if not action_class:
        return output({
            "error": "Cannot load nav2_msgs/action/Spin — is nav2_msgs installed and sourced?",
        })

    target_yaw = math.radians(args.degrees)
    goal_dict = {"target_yaw": target_yaw}

    try:
        with ros2_context():
            from rclpy.action import ActionClient  # noqa: PLC0415

            node = ROS2CLI()
            client = ActionClient(node, action_class, _SPIN_ACTION)
            if not client.wait_for_server(timeout_sec=args.timeout):
                return output({
                    "error": f"Spin action server not available (waited {args.timeout}s) — is Nav2 running?",
                })

            goal_msg = dict_to_msg(action_class.Goal, goal_dict)
            future = client.send_goal_async(goal_msg)

            spin_until(node, future, time.time() + args.timeout, on_cancel=lambda: _publish_zero_burst(node))

            if not future.done():
                future.cancel()
                return output({"error": f"Timeout waiting for goal acceptance ({args.timeout}s)"})

            goal_handle = future.result()
            if not goal_handle.accepted:
                return output({"success": False, "status": "ABORTED",
                               "error": "Spin goal rejected by Nav2"})

            result_future = goal_handle.get_result_async()
            spin_until(node, result_future, time.time() + args.timeout,
                       on_cancel=lambda: _cancel_goal_and_halt(node, goal_handle))

            if not result_future.done():
                result_future.cancel()
                with uninterruptible():  # a timeout is a cancel path too: never leave the spin running
                    _cancel_goal_and_halt(node, goal_handle)
                return output({
                    "error": f"Spin timeout after {args.timeout}s — the goal was cancelled and velocity zeroed.",
                    "hint": "A spin completes on the EKF yaw: wheels off the ground or a stuck IMU never reach the target.",
                })

            wrapped = result_future.result()
            status = int(wrapped.status)
            success = (status == 4)  # GoalStatus.SUCCEEDED
            out = {
                "success": success,
                "status": _status_name(status),
                "degrees": args.degrees,
                "target_yaw_rad": round(target_yaw, 6),
            }
            if not success:
                out["error"] = f"Spin {_status_name(status).lower()} (status {status})"
            output(out)

    except Exception as e:
        output({"error": str(e)})


_NAV2_CANCEL_ACTIONS = ("/navigate_to_pose", "/navigate_through_poses", "/follow_waypoints",
                        "/spin", "/backup", "/drive_on_heading", "/wait")


def cmd_nav2_cancel(args):
    """Cancel active goals on every Nav2 action server (or just --action) and send a zero-velocity burst."""
    explicit = getattr(args, "action", None)
    actions = [explicit] if explicit and explicit != _NAV2_ACTION else list(_NAV2_CANCEL_ACTIONS)
    timeout = args.timeout

    try:
        from action_msgs.srv import CancelGoal  # noqa: PLC0415

        with ros2_context():
            node = ROS2CLI()
            cancelled, unavailable = {}, []
            for action_name in actions:
                client = node.create_client(CancelGoal, action_name + "/_action/cancel_goal")
                if not client.wait_for_service(timeout_sec=timeout if len(actions) == 1 else 0.5):
                    unavailable.append(action_name)
                    node.destroy_client(client)
                    continue
                future = client.call_async(CancelGoal.Request())  # empty goal_id = cancel ALL goals
                end = time.time() + timeout
                while time.time() < end and not future.done():
                    rclpy.spin_once(node, timeout_sec=0.1)
                resp = future.result() if future.done() else None
                cancelled[action_name] = len(resp.goals_canceling) if resp and resp.goals_canceling else 0
                node.destroy_client(client)

            if not cancelled:
                return output({"error": f"No Nav2 action server answered ({', '.join(unavailable)}) — is Nav2 running?"})
            _publish_zero_burst(node)

        output({
            "cancelled": True,
            "goals_cancelled": sum(cancelled.values()),
            "per_action": cancelled,
            "unavailable": unavailable,
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
            # CollisionMonitor only publishes on a state *change* (e.g.
            # NORMAL -> APPROACH), not periodically — so "no message within
            # this short window" is the expected, common case whenever
            # nothing nearby has changed recently, and looks identical to
            # "collision_monitor isn't running at all" unless distinguished.
            # A cheap graph check (is there a publisher on this topic at all?)
            # tells them apart without waiting.
            cm_class = get_msg_type("nav2_msgs/msg/CollisionMonitorState")
            if cm_class:
                cm_topic = "/collision_monitor_state"
                result["collision_monitor_publisher_present"] = any(
                    name == cm_topic for name, _ in topic_types
                )

                cm_received = []
                cm_done = threading.Event()

                cm_sub = node.create_subscription(
                    cm_class,
                    cm_topic,
                    lambda msg: (cm_received.append(msg), cm_done.set()),
                    10,
                )
                end = time.time() + min(timeout, 1.0)
                while time.time() < end and not cm_done.is_set():
                    rclpy.spin_once(node, timeout_sec=0.1)

                if cm_received:
                    result["collision_monitor"] = msg_to_dict(cm_received[0])
                elif result["collision_monitor_publisher_present"]:
                    result["collision_monitor_note"] = (
                        "collision_monitor is running but published no state-change message "
                        "in this window — normal if nothing nearby has changed recently, not "
                        "an error. Use 'lifecycle get /collision_monitor' to confirm it's active."
                    )

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


# ---------------------------------------------------------------------------
# Shared service-call helper
# ---------------------------------------------------------------------------

def _call_service(node, svc_name, srv_class, request, timeout: float = 5.0):
    """Call *svc_name* with *request* and return ``(response, error_str)``.

    Returns ``(None, error_message)`` on failure; ``(response, None)`` on
    success. Thin string-error adapter around ros2_utils.call_ros_service
    (which returns a dict-shaped error) — kept so this module's 8 existing
    call sites, which expect a plain string here, don't need to change.
    """
    result, err = call_ros_service(node, srv_class, svc_name, request, timeout)
    if err:
        return None, err["error"]
    return result, None


# ---------------------------------------------------------------------------
# Map lifecycle — nav map list / save / load / delete
# ---------------------------------------------------------------------------

# Lifecycle state IDs
_LC_ACTIVE = 3
_LC_STATE_NAMES = {0: "unknown", 1: "unconfigured", 2: "inactive", 3: "active", 4: "finalized"}

# Lifecycle transition IDs
_LC_CONFIGURE   = 1
_LC_ACTIVATE    = 3
_LC_DEACTIVATE  = 4


def _get_managed_node_state(node, node_name: str, timeout: float = 3.0) -> Optional[dict]:
    """Return ``{"state_id": N, "state": label}`` for a lifecycle node, or None."""
    from lifecycle_msgs.srv import GetState  # noqa: PLC0415
    svc = f"{node_name}/get_state"
    resp, err = _call_service(node, svc, GetState, GetState.Request(), timeout)
    if err or resp is None:
        return None
    sid = int(resp.current_state.id)
    return {"state_id": sid, "state": _LC_STATE_NAMES.get(sid, str(sid))}


def _lifecycle_transition(node, node_name: str, transition_id: int,
                           timeout: float = 10.0) -> tuple[bool, str]:
    """Trigger a lifecycle transition; return (success, message)."""
    from lifecycle_msgs.srv import ChangeState  # noqa: PLC0415
    from lifecycle_msgs.msg import Transition   # noqa: PLC0415

    req = ChangeState.Request()
    req.transition = Transition()
    req.transition.id = transition_id

    svc = f"{node_name}/change_state"
    resp, err = _call_service(node, svc, ChangeState, req, timeout)
    if err:
        return False, err
    if not resp.success:
        return False, f"Transition {transition_id} rejected by '{node_name}'"
    return True, "ok"


def cmd_nav2_map_list(args):
    """List available maps.

    Reads from the robot profile's ``summary.maps`` field (populated by
    ``profile scan``) and from ``--maps-dir`` (default ``./maps/``) on the
    filesystem.  Deduplicates by stem name so the same map only appears once
    even if listed in both sources.
    """
    import glob
    import json as _json
    import os

    maps_dir = getattr(args, "maps_dir", None) or "maps"
    maps: dict[str, dict] = {}

    # ---- Profile source (non-ROS, instant) ---------------------------------
    profile_path = os.path.expanduser("~/.ros2_skill/.profiles/profile.json")
    try:
        with open(profile_path) as f:
            profile = _json.load(f)
        for m in profile.get("summary", {}).get("maps", []):
            stem = os.path.splitext(os.path.basename(m.get("image", m.get("name", ""))))[0]
            if not stem:
                stem = m.get("name", "")
            maps[stem] = {
                "name": stem,
                "type": m.get("type", "occupancy"),
                "resolution": m.get("resolution"),
                "source": "profile",
                "yaml": m.get("yaml") or m.get("name"),
            }
    except (FileNotFoundError, KeyError, Exception):
        pass

    # ---- Filesystem scan ---------------------------------------------------
    if os.path.isdir(maps_dir):
        for yaml_path in glob.glob(os.path.join(maps_dir, "*.yaml")):
            stem = os.path.splitext(os.path.basename(yaml_path))[0]
            if stem not in maps:
                maps[stem] = {
                    "name": stem,
                    "type": "occupancy",
                    "source": "filesystem",
                    "yaml": yaml_path,
                }

    result = sorted(maps.values(), key=lambda m: m["name"])
    return output({
        "maps": result,
        "count": len(result),
        "maps_dir": maps_dir,
        **({"hint": (
            "Use 'nav map load <name>' to activate a saved map. "
            "Use 'nav map save [--name N]' to save the current slam_toolbox map."
        )} if not result else {}),
    })


def cmd_nav2_map_save(args):
    """Save the current slam_toolbox map (or map_saver fallback).

    Tries ``/slam_toolbox/save_map`` first (SLAM Toolbox), then
    ``/map_saver/save_map`` (Nav2 map_saver_server).  The saved files are
    ``<name>.yaml`` and ``<name>.pgm`` (or ``<name>.png``).

    ``--name`` is the filename stem (default: ``map``).  Relative to the
    current directory unless an absolute path is given.
    """
    map_name = getattr(args, "name", None) or "map"
    timeout  = getattr(args, "timeout", 10.0)

    try:
        with ros2_context():
            node = ROS2CLI()

            # ---- slam_toolbox/save_map (preferred) -------------------------
            try:
                from slam_toolbox.srv import SaveMap  # noqa: PLC0415
                req = SaveMap.Request()
                req.name.data = map_name
                resp, err = _call_service(node, "/slam_toolbox/save_map", SaveMap, req, timeout)
                if not err and resp is not None:
                    # slam_toolbox SaveMap: result 0 == RESULT_SUCCESS
                    result = resp.result if hasattr(resp, 'result') else 0
                    return output({
                        "success": result == 0,
                        "method": "slam_toolbox",
                        "service": "/slam_toolbox/save_map",
                        "name": map_name,
                        "files": [f"{map_name}.yaml", f"{map_name}.pgm"],
                    })
            except ImportError:
                pass

            # ---- nav2_msgs/srv/SaveMap (map_saver_server fallback) ---------
            try:
                from nav2_msgs.srv import SaveMap  # noqa: PLC0415
                req = SaveMap.Request()
                req.map_topic = "/map"
                req.map_url   = map_name
                req.image_format = "pgm"
                req.map_mode  = "trinary"
                req.free_thresh = 0.25
                req.occupied_thresh = 0.65
                resp, err = _call_service(node, "/map_saver/save_map", SaveMap, req, timeout)
                if not err and resp is not None:
                    return output({
                        "success": resp.result,
                        "method": "map_saver",
                        "service": "/map_saver/save_map",
                        "name": map_name,
                        "files": [f"{map_name}.yaml", f"{map_name}.pgm"],
                    })
                if err:
                    return output({
                        "error": err,
                        "hint": (
                            "Ensure slam_toolbox or nav2_map_saver_server is running. "
                            "Check with 'nodes list'."
                        ),
                    })
            except ImportError:
                pass

            return output({
                "error": "No map-save service available (tried slam_toolbox and map_saver)",
                "hint": (
                    "Start slam_toolbox or nav2_map_saver_server, then retry. "
                    "Or save manually: ros2 run nav2_map_server map_saver_cli -f <name>"
                ),
            })

    except Exception as e:
        return output({"error": str(e)})


def cmd_nav2_map_load(args):
    """Load a saved map via the map_server/load_map service.

    The map server must be running and in the ``active`` lifecycle state.
    ``<name>`` is a path to a map YAML file (absolute or relative to the
    current directory).  Appends ``.yaml`` if the path has no extension.
    """
    import os

    map_name = args.name
    timeout  = getattr(args, "timeout", 10.0)

    # Resolve path
    if not os.path.splitext(map_name)[1]:
        map_name = map_name + ".yaml"
    if not os.path.isabs(map_name):
        if not os.path.exists(map_name) and os.path.exists(os.path.join("maps", map_name)):
            map_name = os.path.join("maps", map_name)
    map_name = os.path.abspath(map_name)

    if not os.path.exists(map_name):
        return output({
            "error": f"Map file not found: {map_name}",
            "hint": "Use 'nav map list' to see available maps.",
        })

    try:
        from nav2_msgs.srv import LoadMap  # noqa: PLC0415
    except ImportError:
        return output({
            "error": "nav2_msgs not available — cannot load LoadMap service type",
            "hint": "Install nav2_msgs: sudo apt install ros-$ROS_DISTRO-nav2-msgs",
        })

    try:
        with ros2_context():
            node = ROS2CLI()
            req = LoadMap.Request()
            req.map_url = map_name
            resp, err = _call_service(node, "/map_server/load_map", LoadMap, req, timeout)

        if err:
            return output({
                "error": err,
                "hint": (
                    "Ensure map_server is running and active. "
                    "Check: lifecycle get /map_server"
                ),
            })
        result_code = int(resp.result)
        success = (result_code == 0)
        _LOAD_RESULTS = {
            0: "SUCCESS", 1: "MAP_DOES_NOT_EXIST", 2: "INVALID_MAP_DATA",
            3: "INVALID_MAP_METADATA", 255: "UNDEFINED_FAILURE",
        }
        return output({
            "success": success,
            "map": map_name,
            "result_code": result_code,
            "result_name": _LOAD_RESULTS.get(result_code, str(result_code)),
            "service": "/map_server/load_map",
            **({"hint": "Activate map_server if inactive: 'lifecycle set /map_server activate'"}
               if not success else {}),
        })

    except Exception as e:
        return output({"error": str(e)})


def cmd_nav2_map_delete(args):
    """Delete a map's files from the filesystem.

    Removes ``<name>.yaml`` and the associated image file (``.pgm`` / ``.png`` /
    ``.bmp``).  **Requires ``--confirm``** to prevent accidental deletion.
    """
    import os

    if not getattr(args, "confirm", False):
        return output({
            "error": "Safety interlock: --confirm flag required",
            "hint": "Re-run with --confirm to permanently delete the map files.",
        })

    map_name = args.name
    maps_dir = getattr(args, "maps_dir", None) or "maps"

    # Resolve YAML path
    if not os.path.splitext(map_name)[1]:
        candidates = [
            map_name + ".yaml",
            os.path.join(maps_dir, map_name + ".yaml"),
        ]
    else:
        candidates = [map_name, os.path.join(maps_dir, map_name)]

    yaml_path = None
    for c in candidates:
        if os.path.exists(c):
            yaml_path = os.path.abspath(c)
            break

    if yaml_path is None:
        return output({
            "error": f"Map YAML file not found for '{map_name}'",
            "hint": "Use 'nav map list' to see available maps.",
        })

    stem = os.path.splitext(yaml_path)[0]
    deleted = []

    # Delete YAML
    try:
        os.remove(yaml_path)
        deleted.append(yaml_path)
    except OSError as e:
        return output({"error": f"Cannot delete {yaml_path}: {e}"})

    # Delete associated image (pgm, png, bmp)
    for ext in (".pgm", ".png", ".bmp"):
        img = stem + ext
        if os.path.exists(img):
            try:
                os.remove(img)
                deleted.append(img)
            except OSError:
                pass

    return output({"success": True, "deleted": deleted})


# ---------------------------------------------------------------------------
# Navigation mode — nav mode get / set
# ---------------------------------------------------------------------------

def _infer_nav_mode(node, timeout: float = 3.0) -> dict:
    """Query lifecycle states of slam_toolbox and amcl to infer navigation mode.

    Returns::

        {
          "mode": "mapfree" | "mapping" | "navigation" | "unknown",
          "slam_node": "/slam_toolbox" | None,
          "slam_state": "active" | "inactive" | None,
          "amcl_node": "/amcl" | None,
          "amcl_state": "active" | "inactive" | None,
        }
    """
    slam_node, slam_state_id = None, None
    amcl_node, amcl_state_id = None, None

    # Discover managed nodes via their /get_state service
    service_info = node.get_service_names_and_types()
    managed = [
        svc[:-len('/get_state')]
        for svc, types in service_info
        if 'lifecycle_msgs/srv/GetState' in types and svc.endswith('/get_state')
    ]

    for n in managed:
        lc_name = n.lower()
        if any(c in lc_name for c in ("slam_toolbox", "slam_tool")):
            slam_node = n
        elif "amcl" in lc_name:
            amcl_node = n

    if slam_node:
        st = _get_managed_node_state(node, slam_node, timeout)
        slam_state_id = st["state_id"] if st else None

    if amcl_node:
        st = _get_managed_node_state(node, amcl_node, timeout)
        amcl_state_id = st["state_id"] if st else None

    slam_active = (slam_state_id == _LC_ACTIVE)
    amcl_active = (amcl_state_id == _LC_ACTIVE)

    if slam_active and amcl_active:
        mode = "unknown"
    elif slam_active:
        mode = "mapping"
    elif amcl_active:
        mode = "navigation"
    else:
        mode = "mapfree"

    return {
        "mode": mode,
        "slam_node":  slam_node,
        "slam_state": _LC_STATE_NAMES.get(slam_state_id) if slam_state_id is not None else None,
        "amcl_node":  amcl_node,
        "amcl_state": _LC_STATE_NAMES.get(amcl_state_id) if amcl_state_id is not None else None,
    }


def cmd_nav2_mode_get(args):
    """Report the current navigation mode by inspecting lifecycle states.

    Checks which of slam_toolbox and amcl are in the ``active`` lifecycle
    state and infers the navigation mode:

    * ``mapfree``    — neither slam_toolbox nor amcl active
    * ``mapping``    — slam_toolbox active (building a new map)
    * ``navigation`` — amcl active (localising against a saved map)
    * ``unknown``    — both active simultaneously (misconfiguration)
    """
    timeout = getattr(args, "timeout", 5.0)

    try:
        with ros2_context():
            node = ROS2CLI()
            info = _infer_nav_mode(node, timeout)

        result: dict = {"mode": info["mode"]}
        if info["slam_node"]:
            result["slam"] = {"node": info["slam_node"], "state": info["slam_state"]}
        if info["amcl_node"]:
            result["amcl"] = {"node": info["amcl_node"], "state": info["amcl_state"]}

        if not info["slam_node"] and not info["amcl_node"]:
            result["note"] = (
                "No slam_toolbox or amcl lifecycle nodes detected. "
                "Nav2 may be running without lifecycle management, or not started. "
                "Mode assumed mapfree."
            )
        elif info["mode"] == "unknown":
            result["warning"] = (
                "Both slam_toolbox and amcl are active simultaneously — "
                "this is usually a misconfiguration. Deactivate one before proceeding."
            )

        return output(result)

    except Exception as e:
        return output({"error": str(e)})


def cmd_nav2_mode_set(args):
    """Switch the navigation mode by orchestrating lifecycle transitions.

    Valid modes:

    * ``mapfree``    — deactivate slam_toolbox and amcl (if running)
    * ``mapping``    — activate slam_toolbox; deactivate amcl
    * ``navigation`` — deactivate slam_toolbox; activate amcl (requires a
      saved map to have been loaded first with ``nav map load``)

    Activation follows the full lifecycle sequence: configure (if
    unconfigured) → activate.  Deactivation: deactivate (if active).
    Nodes that are not detected are skipped with a note.
    """
    new_mode = args.mode.lower()
    if new_mode not in ("mapfree", "mapping", "navigation"):
        return output({
            "error": f"Unknown mode '{new_mode}'",
            "hint": "Valid modes: mapfree | mapping | navigation",
        })

    timeout = getattr(args, "timeout", 10.0)

    want_slam_active = (new_mode == "mapping")
    want_amcl_active = (new_mode == "navigation")

    try:
        with ros2_context():
            node = ROS2CLI()
            info = _infer_nav_mode(node, timeout=3.0)

            actions: list[dict] = []
            errors:  list[str]  = []

            def _ensure_state(lc_node: Optional[str], currently_active: bool,
                               want_active: bool, label: str) -> None:
                if lc_node is None:
                    actions.append({"node": label, "skipped": "not detected as a lifecycle node"})
                    return

                if want_active and not currently_active:
                    st = _get_managed_node_state(node, lc_node, timeout)
                    current_id = st["state_id"] if st else 0
                    if current_id == 1:   # unconfigured → configure first
                        ok, msg = _lifecycle_transition(node, lc_node, _LC_CONFIGURE, timeout)
                        actions.append({"node": lc_node, "transition": "configure",
                                        "ok": ok, "detail": msg})
                        if not ok:
                            errors.append(msg)
                            return
                    ok, msg = _lifecycle_transition(node, lc_node, _LC_ACTIVATE, timeout)
                    actions.append({"node": lc_node, "transition": "activate",
                                    "ok": ok, "detail": msg})
                    if not ok:
                        errors.append(msg)

                elif not want_active and currently_active:
                    ok, msg = _lifecycle_transition(node, lc_node, _LC_DEACTIVATE, timeout)
                    actions.append({"node": lc_node, "transition": "deactivate",
                                    "ok": ok, "detail": msg})
                    if not ok:
                        errors.append(msg)
                else:
                    actions.append({
                        "node": lc_node,
                        "skipped": f"already {'active' if currently_active else 'inactive'}",
                    })

            _ensure_state(info["slam_node"], info["slam_state"] == "active",
                          want_slam_active, "slam_toolbox")
            _ensure_state(info["amcl_node"], info["amcl_state"] == "active",
                          want_amcl_active, "amcl")

            final_info = _infer_nav_mode(node, timeout=3.0)

        result: dict = {
            "success": not errors,
            "requested_mode": new_mode,
            "actual_mode": final_info["mode"],
            "actions": actions,
        }
        if errors:
            result["errors"] = errors
        if new_mode == "navigation" and not info["amcl_node"]:
            result["note"] = (
                "amcl node not detected — ensure Nav2 is started with amcl configured "
                "and a map is loaded via 'nav map load'."
            )
        return output(result)

    except Exception as e:
        return output({"error": str(e)})


# ---------------------------------------------------------------------------
# nav localize
# ---------------------------------------------------------------------------

def cmd_nav2_localize(args):
    """Trigger global re-localisation via AMCL.

    Calls the ``/reinitialize_global_localization`` service (standard Nav2
    empty service) which tells AMCL to spread particles uniformly across the
    map and re-estimate the robot's pose.  Use this after loading a new map
    or when the robot's pose estimate has drifted badly.

    After calling this command, drive the robot around slowly to let AMCL
    converge, or provide an explicit initial pose via ``nav2 initial-pose``.

    Returns::

        {
          "localized": true,
          "service": "/reinitialize_global_localization",
          "note": "AMCL will reinitialize particle filter..."
        }

    **Profile usage:** if the profile lists a non-standard localization
    service, pass it via ``--service``.
    """
    service_name = getattr(args, "service", None) or "/reinitialize_global_localization"
    timeout      = getattr(args, "timeout", 10.0)

    try:
        # Try to load std_srvs/srv/Empty
        srv_class = get_srv_type("std_srvs/srv/Empty") or get_srv_type("std_srvs/Empty")
        if srv_class is None:
            return output({
                "error": "Cannot load std_srvs/srv/Empty",
                "hint": "Install: sudo apt install ros-$ROS_DISTRO-std-srvs",
            })

        with ros2_context():
            node = ROS2CLI()

            # Check service exists
            service_types: list[str] = []
            for svc, types in node.get_service_names_and_types():
                if svc == service_name:
                    service_types = list(types)
                    break

            if not service_types:
                return output({
                    "error": f"Service '{service_name}' not found on the graph",
                    "hint": (
                        "Ensure AMCL is running in navigation mode. "
                        "Switch mode with: nav mode set navigation"
                    ),
                    "available_localization_services": [
                        s for s, _ in node.get_service_names_and_types()
                        if "localiz" in s.lower() or "amcl" in s.lower()
                    ][:10],
                })

            req = srv_class.Request()
            resp, err = _call_service(node, service_name, srv_class, req, timeout=timeout)
            if err:
                return output({"error": err, "service": service_name})

        return output({
            "localized": True,
            "service": service_name,
            "note": (
                "AMCL will reinitialize the particle filter across the full map. "
                "Drive the robot slowly to let it converge on the correct pose, "
                "or use 'nav2 initial-pose' to provide an estimate."
            ),
        })

    except Exception as e:
        return output({"error": str(e)})


def cmd_nav2_move_timed(args):
    """Publish velocity commands for a fixed duration then stop.

        Auto-discovers a ``cmd_vel``-style topic on the graph, preferring topics
        that have active subscribers.
    Directions map to Twist fields:
      fwd/back → linear.x (±speed m/s)
      left/right → angular.z (±speed rad/s)
    """
    direction = args.direction.lower()
    if direction not in ("fwd", "back", "left", "right"):
        return output({
            "error": f"Unknown direction '{direction}'",
            "hint": "Valid: fwd | back | left | right",
        })

    duration = args.duration
    speed = args.speed

    try:
        from geometry_msgs.msg import Twist, TwistStamped  # noqa: PLC0415

        with ros2_context():
            node = ROS2CLI()

            topics = node.get_topic_names_and_types()
            requested_topic = getattr(args, "topic", None)

            # Keep preference deterministic when multiple cmd_vel-like topics exist.
            # Favour the normal ingress path first, but never pick a topic with
            # zero subscribers when a subscribed option exists.
            preference_order = [
                "/cmd_vel_raw",
                "/cmd_vel",
                "/cmd_vel_smoothed",
                "/cmd_vel_presafety",
                "/cmd_vel_teleop",
                "/base_controller/cmd_vel",
            ]
            preference_rank = {name: i for i, name in enumerate(preference_order)}

            candidates = []
            for name, types in topics:
                if "cmd_vel" not in name.lower():
                    continue
                for t in types:
                    if t in ("geometry_msgs/msg/Twist", "geometry_msgs/msg/TwistStamped"):
                        sub_count = int(node.count_subscribers(name))
                        candidates.append({
                            "topic": name,
                            "type": t,
                            "subscribers": sub_count,
                            "rank": preference_rank.get(name, len(preference_order)),
                        })

            chosen = None
            if requested_topic:
                for c in candidates:
                    if c["topic"] == requested_topic:
                        chosen = c
                        break
                if not chosen:
                    return output({
                        "error": f"Requested topic '{requested_topic}' is not a cmd_vel Twist/TwistStamped topic",
                        "available_cmd_vel_topics": [c["topic"] for c in sorted(candidates, key=lambda x: x["topic"])],
                    })
            else:
                # Prefer subscribed topics first, then preference rank.
                candidates.sort(key=lambda c: (c["subscribers"] <= 0, c["rank"], c["topic"]))
                if candidates:
                    chosen = candidates[0]

            if not chosen:
                return output({
                    "error": "No cmd_vel Twist topic found on the graph",
                    "hint": "Ensure the robot's velocity controller is running",
                })

            vel_topic = chosen["topic"]
            vel_type = chosen["type"]

            if vel_type == "geometry_msgs/msg/TwistStamped":
                pub = node.create_publisher(TwistStamped, vel_topic, 10)
                msg = TwistStamped()
                twist = msg.twist
            else:
                pub = node.create_publisher(Twist, vel_topic, 10)
                msg = Twist()
                twist = msg

            if direction == "fwd":
                twist.linear.x = speed
            elif direction == "back":
                twist.linear.x = -speed
            elif direction == "left":
                twist.angular.z = speed
            else:  # right
                twist.angular.z = -speed

            command = {"linear": {"x": float(twist.linear.x)}, "angular": {"z": float(twist.angular.z)}}
            err, guard_cfg = prepare_guard(args, command, vel_topic, vel_type)
            if err:
                return output(err)
            guard, halt, published = ros2_obstacle.CommandGuard(guard_cfg), threading.Event(), 0
            result = {"topic": vel_topic, "msg_type": vel_type, "topic_subscribers": chosen["subscribers"],
                      "direction": direction, "speed": speed, "duration_s": duration}
            try:
                try:
                    guard.arm(command, halt)
                    end = time.time() + duration
                    while time.time() < end and not halt.is_set() and not guard.poll():
                        pub.publish(msg)
                        published += 1
                        rclpy.spin_once(node, timeout_sec=0.05)
                finally:  # also on Ctrl-C / SIGTERM / estop
                    with uninterruptible():
                        _publish_zero_burst(node)
                if guard.active:
                    result["published_count"] = published
                guard.annotate(result, met=guard.tripped is None)
            finally:
                guard.close()

        return output({"moved": published > 0, **result})

    except Exception as e:
        return output({"error": str(e)})


def cmd_nav2_costmap_clear(args):
    """Clear Nav2 costmap obstacle layers to recover from false detections.

    Calls the standard Nav2 clearing services:
      local  → /local_costmap/clear_entirely_local_costmap
      global → /global_costmap/clear_entirely_global_costmap
    """
    layer = getattr(args, "layer", "both").lower()
    timeout = getattr(args, "timeout", 10.0)

    services_to_call = []
    if layer in ("local", "both"):
        services_to_call.append("/local_costmap/clear_entirely_local_costmap")
    if layer in ("global", "both"):
        services_to_call.append("/global_costmap/clear_entirely_global_costmap")

    if not services_to_call:
        return output({
            "error": f"Unknown layer '{layer}'",
            "hint": "Valid: local | global | both",
        })

    # Pre-existing bug fixed here: these services are nav2_msgs/srv/
    # ClearEntireCostmap, not std_srvs/srv/Empty — confirmed via
    # `ros2 service type /global_costmap/clear_entirely_global_costmap`.
    # A client created with the wrong type never sees the real server as
    # available (request/response type is part of the underlying DDS
    # topic name), so this always failed with "service not available"
    # despite the service genuinely running.
    srv_class = get_srv_type("nav2_msgs/srv/ClearEntireCostmap")
    if srv_class is None:
        return output({
            "error": "Cannot load nav2_msgs/srv/ClearEntireCostmap",
            "hint": "Install: sudo apt install ros-$ROS_DISTRO-nav2-msgs",
        })

    cleared = []
    errors = []

    try:
        with ros2_context():
            node = ROS2CLI()
            for svc_name in services_to_call:
                _, err = _call_service(node, svc_name, srv_class, srv_class.Request(), timeout)
                if err:
                    errors.append({"service": svc_name, "error": err})
                else:
                    cleared.append(svc_name)

        result = {
            "cleared": len(cleared) > 0,
            "layer": layer,
            "layers_cleared": cleared,
        }
        if errors:
            result["errors"] = errors
        return output(result)

    except Exception as e:
        return output({"error": str(e)})


def cmd_nav2_diagnose(args):
    """Aggregate Nav2 health: action server, localization, costmaps, goal queue."""
    timeout = getattr(args, "timeout", 5.0)  # noqa: F841 — reserved for future subscription use

    try:
        with ros2_context():
            node = ROS2CLI()
            topic_types = node.get_topic_names_and_types()
            node_names = node.get_node_names_and_namespaces()

        topic_set = {name for name, _ in topic_types}
        node_name_set = {name for name, _ in node_names}

        # 1. Nav2 action server alive — feedback topic published when BT navigator running
        nav2_ready = (_NAV2_ACTION + "/_action/feedback") in topic_set

        # 2. Localization — amcl node or amcl-related topic present
        localized = (
            any("amcl" in n for n in node_name_set)
            or any("amcl" in t for t in topic_set)
        )

        # 3. Costmaps — both local and global costmap nodes must be running
        has_local = any("local_costmap" in n for n in node_name_set)
        has_global = any("global_costmap" in n for n in node_name_set)
        costmaps_ok = has_local and has_global

        # 4. Active goals — 0 when status topic absent; None (unknown) when present
        #    (subscribing requires a live spin loop; deferred for now)
        status_topic = _NAV2_ACTION + "/_action/status"
        active_goals = None if status_topic in topic_set else 0

        output({
            "nav2_ready": nav2_ready,
            "localized": localized,
            "costmaps_ok": costmaps_ok,
            "active_goals": active_goals,
        })

    except Exception as e:
        output({"error": str(e)})


# ---------------------------------------------------------------------------
# Mission control — pause / resume / cancel / status
# ---------------------------------------------------------------------------

def _normalize_mission_steps(raw_steps):
    if isinstance(raw_steps, str):
        raw_steps = json.loads(raw_steps)
    if not isinstance(raw_steps, list) or not raw_steps:
        raise ValueError("mission steps must be a non-empty JSON list")
    steps = []
    for step in raw_steps:
        if not isinstance(step, dict):
            raise ValueError("each mission step must be a JSON object")
        step_type = str(step.get("type", "")).strip().lower()
        if not step_type:
            raise ValueError("each mission step needs a 'type'")
        steps.append(dict(step))
    return steps


def _mission_runner_script() -> pathlib.Path:
    return pathlib.Path(__file__).resolve().parent / "ros2_mission.py"


def cmd_nav2_mission_run(args):
    """Start a detached file-backed mission runner."""
    mission_name = getattr(args, "name", None) or "mission"
    mission_id = getattr(args, "mission_id", None) or f"{mission_name}-{int(time.time())}"

    raw_steps = getattr(args, "steps", None)
    steps_file = getattr(args, "file", None)
    if raw_steps is None and steps_file is None:
        return output({"error": "Provide --steps or --file"})

    try:
        if steps_file:
            raw_steps = pathlib.Path(steps_file).read_text(encoding="utf-8", errors="replace")
        steps = _normalize_mission_steps(raw_steps)
    except Exception as exc:
        return output({"error": f"Invalid mission steps: {exc}"})

    state = {
        "mission_id": mission_id,
        "name": mission_name,
        "status": "queued",
        "steps": steps,
        "current_step": -1,
        "history": [],
        "created_at": time.time(),
    }
    state_path = _save_mission_state(state)
    log_path = state_path.with_suffix(".log")

    runner_script = _mission_runner_script()
    if not runner_script.exists():
        return output({"error": f"Mission runner not found: {runner_script}"})

    with open(log_path, "a", encoding="utf-8") as log_file:
        proc = subprocess.Popen(
            [sys.executable, str(runner_script), "--mission-id", mission_id],
            stdout=log_file,
            stderr=log_file,
            start_new_session=True,
        )

    state["runner_pid"] = proc.pid
    _save_mission_state(state)

    return output({
        "started": True,
        "mission_id": mission_id,
        "mission_name": mission_name,
        "state_file": str(state_path),
        "log_file": str(log_path),
        "steps": len(steps),
        "runner_pid": proc.pid,
        "hint": "Use 'nav2 mission pause|resume|cancel|status' with the mission_id.",
    })


def _update_mission_status(mission_id: str, status: str):
    state = _load_mission_state(mission_id)
    if state is None:
        return None
    state["status"] = status
    _save_mission_state(state)
    return state


def cmd_nav2_mission_pause(args):
    mission_id = getattr(args, "mission_id", None)
    state = _update_mission_status(mission_id, "paused") if mission_id else None
    if state is None:
        return output({"error": f"Mission '{mission_id}' not found"})
    return output({"paused": True, **_mission_summary(state)})


def cmd_nav2_mission_resume(args):
    mission_id = getattr(args, "mission_id", None)
    state = _update_mission_status(mission_id, "running") if mission_id else None
    if state is None:
        return output({"error": f"Mission '{mission_id}' not found"})
    return output({"resumed": True, **_mission_summary(state)})


def cmd_nav2_mission_cancel(args):
    mission_id = getattr(args, "mission_id", None)
    state = _load_mission_state(mission_id) if mission_id else None
    if state is None:
        return output({"error": f"Mission '{mission_id}' not found"})

    state["status"] = "canceling"
    _save_mission_state(state)

    # A live runner is signalled so the step in progress is interrupted now (its cleanup cancels the Nav2 goal and
    # zeroes the base); without this the cancel would only take effect between steps.
    pid = state.get("runner_pid")
    signalled = False
    if pid:
        try:
            if b"ros2_mission" in pathlib.Path(f"/proc/{int(pid)}/cmdline").read_bytes():
                os.kill(int(pid), signal.SIGTERM)
                signalled = True
        except OSError:
            pass
    if pid and not signalled:
        try:
            os.kill(int(pid), 0)
        except OSError:
            try:
                with ros2_context():
                    node = ROS2CLI()
                    _publish_zero_burst(node)
            except Exception:
                pass

    return output({"cancel_requested": True, "runner_signalled": signalled, **_mission_summary(state)})


def cmd_nav2_mission_status(args):
    mission_id = getattr(args, "mission_id", None)
    state = _load_mission_state(mission_id) if mission_id else None
    if state is None:
        return output({"error": f"Mission '{mission_id}' not found"})
    return output({"mission": state, "summary": _mission_summary(state)})


def _profile_odom_topic() -> Optional[str]:
    """Odometry topic from the loaded profile (first fused source), or None."""
    try:
        from ros2_profile import load_profile_summary
        fused = (load_profile_summary(readonly=True) or {}).get("localization_config", {}).get("fused_sources") or {}
        return next(iter(fused.values()), None)
    except Exception:
        return None


def cmd_nav2_place_save(args):
    """Save the current robot pose as a named place."""
    place_name = getattr(args, "name", None)
    if not place_name:
        return output({"error": "Place name is required"})

    timeout = float(getattr(args, "timeout", 2.0) or 2.0)
    frame = getattr(args, "frame", None) or "map"
    topic_name = getattr(args, "topic", None) or ("/amcl_pose" if frame == "map" else "/odom")

    x = getattr(args, "x", None)
    y = getattr(args, "y", None)
    yaw_deg = getattr(args, "yaw", None)
    if x is None or y is None:
        pose = _current_pose_from_topic(topic_name, timeout)
        if pose is None and not getattr(args, "topic", None):
            odom = _profile_odom_topic()
            pose = _current_pose_from_topic(odom, timeout) if odom else None
        if pose is None:
            return output({
                "error": "Could not capture current pose",
                "hint": "Provide --x/--y/--yaw explicitly or use a running /amcl_pose or odometry topic.",
            })
        x = pose["x"]
        y = pose["y"]
        yaw_deg = pose["yaw_deg"]
        frame = pose.get("frame", frame)
        topic_name = pose.get("source_topic", topic_name)

    place = {
        "name": place_name,
        "x": float(x),
        "y": float(y),
        "yaw_deg": float(yaw_deg or 0.0),
        "frame": frame,
        "source_topic": topic_name,
        "created_at": time.time(),
    }
    if getattr(args, "snapshot", False):  # a frame to look at before deciding to go there
        from ros2_snapshot import capture_frame
        shot = capture_frame(str(_place_path(place_name).with_suffix(".jpg")), timeout=timeout + 3.0)
        place["image"] = shot.get("path") if "error" not in shot else None
        if "error" in shot:
            place["image_error"] = shot["error"]
    saved_path = _save_place(place)
    return output({"saved": True, "place": place, "file": str(saved_path)})


def cmd_nav2_place_list(args):
    """List stored named places."""
    places = _list_places()
    return output({"places": places, "count": len(places), "places_dir": str(_PLACE_DIR)})


def cmd_nav2_place_go(args):
    """Navigate to a stored place."""
    place_name = getattr(args, "name", None)
    if not place_name:
        return output({"error": "Place name is required"})
    place = _load_place(place_name)
    if place is None:
        return output({"error": f"Place '{place_name}' not found"})

    action_name = getattr(args, "action", _NAV2_ACTION)
    timeout = float(getattr(args, "timeout", _NAV_TIMEOUT) or _NAV_TIMEOUT)
    action_class = get_action_type("nav2_msgs/action/NavigateToPose")
    if not action_class:
        return output({"error": "Cannot load nav2_msgs/action/NavigateToPose — is nav2_msgs installed and sourced?"})

    goal_dict = _build_nav2_goal(place["x"], place["y"], place.get("yaw_deg"), place.get("frame", "map"))
    try:
        with ros2_context():
            node = ROS2CLI()
            success, status, result_dict, feedback_msgs = _send_navigate_to_pose(
                node, action_class, action_name, goal_dict, timeout, bool(getattr(args, "feedback", False))
            )

        out = {
            "action": action_name,
            "success": success,
            "status": status,
            "status_name": _status_name(status),
            "place": place,
        }
        if result_dict:
            out["result"] = result_dict
        if getattr(args, "feedback", False):
            out["feedback_msgs"] = feedback_msgs
        if not success:
            out["error"] = f"Navigation {_status_name(status).lower()} (status {status})"
        output(out)
    except Exception as exc:
        output({"error": str(exc)})


def cmd_nav2_place_delete(args):
    """Delete a stored place."""
    place_name = getattr(args, "name", None)
    if not place_name:
        return output({"error": "Place name is required"})
    path = _place_path(place_name)
    if not path.exists():
        return output({"error": f"Place '{place_name}' not found"})
    path.unlink()
    path.with_suffix(".jpg").unlink(missing_ok=True)
    return output({"deleted": True, "name": place_name, "file": str(path)})
