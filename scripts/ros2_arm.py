#!/usr/bin/env python3
"""Arm and manipulation commands.

Provides arm-specific shortcuts that sit above the generic ``topics`` /
``services`` / ``actions`` commands:

- ``arm state``      — single-shot snapshot of all joint positions, velocities,
                       and efforts from the JointState topic.
- ``arm joints set`` — send a joint-space goal via a configurable service or
                       action (``/arm/goto_js`` by default when using Innate;
                       ``/move_group`` FollowJointTrajectory for MoveIt2).
- ``arm home``       — move the arm to its named home joint configuration
                       (from profile or ``--joints`` override).
- ``arm pose get``   — read the end-effector pose via TF lookup.

All commands follow the two-path model: profile-first, live-discovery
fallback.  Topic and service names are resolved from the profile's
``summary.hardware_interfaces`` or ``--topic`` / ``--service`` overrides.
"""

import math
import time

import rclpy

from ros2_utils import (
    ROS2CLI, get_msg_type, get_srv_type, msg_to_dict, output, ros2_context,
)
from ros2_topic import TopicSubscriber

# Default topic for joint states (standard ROS 2 convention).
_JOINT_STATES_TOPIC = "/joint_states"

# Seconds to wait for a JointState message.
_DEFAULT_TIMEOUT = 5.0


# ---------------------------------------------------------------------------
# JointState helpers
# ---------------------------------------------------------------------------

def _resolve_joint_states_topic(node, override: str | None) -> str:
    """Return the best joint-states topic name.

    Priority: explicit override → profile hardware_interfaces → /joint_states.
    """
    if override:
        return override

    # No profile lookup in this standalone call — return default.
    # Agents that have a loaded profile can pass the resolved name via --topic.
    return _JOINT_STATES_TOPIC


def _parse_joint_state(msg_dict: dict) -> list[dict]:
    """Convert a JointState msg_dict into a list of per-joint dicts.

    Handles missing velocity / effort arrays (some publishers omit them).
    """
    names     = msg_dict.get("name", []) or []
    positions = msg_dict.get("position", []) or []
    velocities= msg_dict.get("velocity", []) or []
    efforts   = msg_dict.get("effort", []) or []

    joints = []
    for i, name in enumerate(names):
        joints.append({
            "name":     name,
            "position": positions[i]  if i < len(positions)  else None,
            "velocity": velocities[i] if i < len(velocities) else None,
            "effort":   efforts[i]    if i < len(efforts)    else None,
        })
    return joints


# ---------------------------------------------------------------------------
# arm state
# ---------------------------------------------------------------------------

def cmd_arm_state(args):
    """Snapshot of all joint positions, velocities, and efforts.

    Subscribes to ``/joint_states`` (or the topic in ``--topic``) and reads
    one message.  Returns a per-joint breakdown::

        {
          "topic": "/joint_states",
          "joints": [
            {"name": "joint1", "position": 0.0, "velocity": 0.0, "effort": 0.0},
            ...
          ],
          "count": N,
          "stamp": {"sec": ..., "nanosec": ...}
        }

    **Profile usage:** When a robot profile is loaded, pass the exact topic
    from ``summary.hardware_interfaces`` via ``--topic`` to avoid re-discovering
    it.  If the profile lists arm-specific joint names, the returned joints can
    be filtered by name.
    """
    topic_arg  = getattr(args, "topic",   None)
    timeout    = getattr(args, "timeout", _DEFAULT_TIMEOUT)
    joint_filter = getattr(args, "joints", None)  # comma-separated names to filter

    try:
        joint_class = None
        for t in ("sensor_msgs/msg/JointState", "sensor_msgs/JointState"):
            joint_class = get_msg_type(t)
            if joint_class:
                break
        if joint_class is None:
            return output({
                "error": "Could not load sensor_msgs/JointState",
                "hint": "Install: sudo apt install ros-$ROS_DISTRO-sensor-msgs",
            })

        with ros2_context():
            node = ROS2CLI()
            topic = _resolve_joint_states_topic(node, topic_arg)

            # Confirm the topic exists
            topic_exists = any(
                t == topic for t, _ in node.get_topic_names_and_types()
            )
            if not topic_exists:
                return output({
                    "error": f"Topic '{topic}' not found on the graph",
                    "hint": (
                        "Ensure the robot's joint state publisher is running. "
                        "Use '--topic /custom/joint_states' if published elsewhere."
                    ),
                })

            executor = rclpy.executors.SingleThreadedExecutor()
            sub = TopicSubscriber(topic, "sensor_msgs/msg/JointState",
                                  msg_class=joint_class)
            executor.add_node(sub)

            end = time.time() + timeout
            while time.time() < end and not sub.messages:
                executor.spin_once(timeout_sec=0.1)

            with sub.lock:
                msgs = sub.messages[:]

        if not msgs:
            return output({
                "error": f"Timeout — no message received on '{topic}' within {timeout}s",
                "hint": "Check that the joint state publisher is active.",
            })

        joints = _parse_joint_state(msgs[0])
        stamp  = msgs[0].get("header", {}).get("stamp", {})

        # Optional name filter
        if joint_filter:
            names_wanted = {n.strip() for n in joint_filter.split(",")}
            joints = [j for j in joints if j["name"] in names_wanted]

        return output({
            "topic":  topic,
            "joints": joints,
            "count":  len(joints),
            "stamp":  stamp,
        })

    except Exception as e:
        return output({"error": str(e)})


# ---------------------------------------------------------------------------
# arm joints set
# ---------------------------------------------------------------------------

def cmd_arm_joints_set(args):
    """Send a joint-space goal to the arm.

    Calls a configurable ROS 2 service (default: ``/arm/goto_js``) with
    target joint angles (radians) and an optional duration.

    The service type is detected automatically from the graph.  For **Innate**
    robots the service is ``/mars/arm/goto_js`` (``brain_messages/srv/GotoJS``).
    For **MoveIt2**-based robots, use ``--action`` mode with the
    ``FollowJointTrajectory`` action server.

    Angles are in **radians** unless ``--degrees`` is given.

    Usage examples::

        arm joints set 0 -1.57 0 1.57 0 0 --duration 2.0
        arm joints set 0 0 0 0 0 0 --service /mars/arm/goto_js
        arm joints set 90 0 0 0 0 0 --degrees

    """
    raw_angles   = args.angles        # list of float strings
    duration     = getattr(args, "duration", 2.0)
    service_name = getattr(args, "service", None) or "/arm/goto_js"
    use_degrees  = getattr(args, "degrees", False)
    timeout      = getattr(args, "timeout", 10.0)

    # Parse angles
    try:
        angles = [float(a) for a in raw_angles]
    except ValueError as e:
        return output({"error": f"Invalid angle value: {e}"})

    if not angles:
        return output({"error": "At least one joint angle is required"})

    if use_degrees:
        angles = [math.radians(a) for a in angles]

    try:
        with ros2_context():
            node = ROS2CLI()

            # Detect service type from graph
            service_types: list[str] = []
            for svc, types in node.get_service_names_and_types():
                if svc == service_name:
                    service_types = list(types)
                    break

            if not service_types:
                return output({
                    "error": f"Service '{service_name}' not found on the graph",
                    "available_services": [
                        s for s, _ in node.get_service_names_and_types()
                        if "arm" in s.lower() or "goto" in s.lower() or "joint" in s.lower()
                    ][:10],
                    "hint": (
                        "Specify the correct service with --service. "
                        "Use 'services list' to see all available services."
                    ),
                })

            srv_type_str = service_types[0]

            # Try to load the service class
            srv_class = get_srv_type(srv_type_str)
            if srv_class is None:
                return output({
                    "error": f"Cannot load service type '{srv_type_str}'",
                    "hint": "Ensure the relevant ROS package is installed and sourced.",
                })

            # Build the request — works with any service whose Request
            # has a list field containing the joint angles.
            # For Innate GotoJS: req.joint_goal.data = [float, ...]
            # For generic: req.positions = [float, ...]
            req = srv_class.Request()

            # Attempt to fill common field names
            set_ok = False
            for field_name in ("joint_goal", "positions", "joints", "goal", "angles"):
                if hasattr(req, field_name):
                    field = getattr(req, field_name)
                    if hasattr(field, "data"):
                        field.data = angles
                    else:
                        try:
                            setattr(req, field_name, angles)
                        except (TypeError, AttributeError):
                            continue
                    set_ok = True
                    break
            if hasattr(req, "duration"):
                req.duration = duration
            if not set_ok:
                return output({
                    "error": (
                        f"Cannot auto-fill request for '{srv_type_str}'. "
                        "Use 'services call' with a hand-crafted JSON request instead."
                    ),
                    "service": service_name,
                    "type": srv_type_str,
                    "angles_rad": angles,
                })

            # Call the service
            client = node.create_client(srv_class, service_name)
            try:
                if not client.wait_for_service(timeout_sec=timeout):
                    return output({
                        "error": f"Service '{service_name}' not available (waited {timeout}s)",
                        "hint": "Ensure the arm controller node is running.",
                    })
                future = client.call_async(req)
                end = time.time() + timeout
                while time.time() < end and not future.done():
                    rclpy.spin_once(node, timeout_sec=0.1)
                if not future.done():
                    future.cancel()
                    return output({
                        "error": f"Service call timed out after {timeout}s",
                        "service": service_name,
                    })
                resp = future.result()
            finally:
                client.destroy()

        # Return result
        resp_dict = {}
        try:
            resp_dict = msg_to_dict(resp)
        except Exception:
            pass

        return output({
            "success": True,
            "service": service_name,
            "angles_rad": angles,
            "duration": duration,
            "response": resp_dict,
        })

    except Exception as e:
        return output({"error": str(e)})


# ---------------------------------------------------------------------------
# arm home
# ---------------------------------------------------------------------------

def cmd_arm_home(args):
    """Move the arm to its safe home joint configuration.

    Reads the home angles from the ``--joints`` argument (comma-separated
    radians), or delegates to ``arm joints set`` with the robot's predefined
    home configuration if ``--service`` is set and the service accepts a
    named state string.

    This is a thin wrapper around ``cmd_arm_joints_set`` with a pre-filled
    angle list.  Profile-aware agents should:

    1. Read ``summary.hardware_interfaces`` to find the arm plugin.
    2. Check ``profile show`` for an ``arm_home_joints`` annotation.
    3. Pass those angles via ``--joints``.
    """
    home_joints = getattr(args, "joints", None)
    if not home_joints:
        return output({
            "error": "No home joint configuration available",
            "hint": (
                "Provide home angles via --joints '0,0,0,0,0,0' (comma-separated radians), "
                "or annotate the profile: profile annotate 'arm_home_joints=0,0,0,0,0,0'."
            ),
        })

    # Parse comma-separated angles
    try:
        angles = [float(a.strip()) for a in home_joints.split(",")]
    except ValueError as e:
        return output({"error": f"Invalid joint angle in --joints: {e}"})

    import argparse
    sub_args = argparse.Namespace(
        angles=angles,
        duration=getattr(args, "duration", 2.0),
        service=getattr(args, "service", "/arm/goto_js"),
        degrees=False,
        timeout=getattr(args, "timeout", 10.0),
    )
    # Override: angles is already floats (not strings) — reuse directly
    sub_args.angles = [str(a) for a in angles]
    return cmd_arm_joints_set(sub_args)


# ---------------------------------------------------------------------------
# arm pose get
# ---------------------------------------------------------------------------

def cmd_arm_pose_get(args):
    """Read the end-effector pose via TF lookup.

    Looks up the transform from *reference_frame* (default: ``map``) to
    *eef_frame* (default: ``tool0`` — a common end-effector frame name).

    Returns position (x, y, z), quaternion (x, y, z, w), and RPY
    (roll, pitch, yaw) in both radians and degrees.

    **Frame resolution:** the correct end-effector frame depends on the robot's
    URDF.  Common names: ``tool0``, ``ee_link``, ``gripper``, ``hand_link``,
    ``tcp``.  Use ``tf list`` or ``tf tree`` to discover available frames.
    """
    eef_frame = getattr(args, "frame", "tool0") or "tool0"
    ref_frame  = getattr(args, "reference_frame", "base_link") or "base_link"
    timeout    = getattr(args, "timeout", 5.0)

    try:
        import tf2_ros  # noqa: PLC0415

        with ros2_context():
            node = ROS2CLI()
            tf_buffer  = tf2_ros.Buffer()
            tf_listener = tf2_ros.TransformListener(tf_buffer, node)

            executor = rclpy.executors.SingleThreadedExecutor()
            executor.add_node(node)

            end = time.time() + timeout
            transform = None
            while time.time() < end:
                executor.spin_once(timeout_sec=0.1)
                try:
                    transform = tf_buffer.lookup_transform(
                        ref_frame, eef_frame, rclpy.time.Time()
                    )
                    break
                except Exception:
                    pass

        if transform is None:
            return output({
                "error": f"Could not look up TF from '{ref_frame}' to '{eef_frame}' within {timeout}s",
                "hint": (
                    f"Verify frame names with 'tf list' or 'tf tree'. "
                    f"Common EEF frames: tool0, ee_link, gripper, hand_link, tcp."
                ),
            })

        t = transform.transform.translation
        r = transform.transform.rotation

        # Convert quaternion to RPY
        sinr = 2.0 * (r.w * r.x + r.y * r.z)
        cosr = 1.0 - 2.0 * (r.x * r.x + r.y * r.y)
        roll = math.atan2(sinr, cosr)

        sinp = 2.0 * (r.w * r.y - r.z * r.x)
        pitch = math.asin(max(-1.0, min(1.0, sinp)))

        siny = 2.0 * (r.w * r.z + r.x * r.y)
        cosy = 1.0 - 2.0 * (r.y * r.y + r.z * r.z)
        yaw = math.atan2(siny, cosy)

        return output({
            "eef_frame":       eef_frame,
            "reference_frame": ref_frame,
            "position": {"x": t.x, "y": t.y, "z": t.z},
            "quaternion": {"x": r.x, "y": r.y, "z": r.z, "w": r.w},
            "rpy_rad": {"roll": roll, "pitch": pitch, "yaw": yaw},
            "rpy_deg": {
                "roll":  math.degrees(roll),
                "pitch": math.degrees(pitch),
                "yaw":   math.degrees(yaw),
            },
        })

    except Exception as e:
        return output({"error": str(e)})


# ---------------------------------------------------------------------------
# arm ik-check
# ---------------------------------------------------------------------------

def cmd_arm_ik_check(args):
    """Check if a Cartesian pose is reachable via MoveIt2 IK.

    Calls the ``/compute_ik`` service (``moveit_msgs/srv/GetPositionIK``)
    to determine whether the target position/orientation has a valid
    inverse-kinematics solution.

    Returns::

        {
          "reachable": true/false,
          "joint_solution": {"joint1": 0.0, ...},   # only when reachable
          "error_code": 1,                           # MoveIt error code
          "error_name": "SUCCESS",
          "service": "/compute_ik"
        }

    Position is in metres; RPY is in **degrees** by default (use ``--rad``
    for radians).  The end-effector group defaults to ``"arm"`` — pass
    ``--group`` to change it.

    **Frame resolution:** position is expressed in the robot's base frame
    (default ``base_link``).  Use ``--frame`` to override.
    """
    x       = args.x
    y       = args.y
    z       = args.z
    rpy_deg = getattr(args, "rpy", None) or [0.0, 0.0, 0.0]
    use_rad = getattr(args, "rad", False)
    group   = getattr(args, "group", "arm") or "arm"
    eef     = getattr(args, "eef", "tool0") or "tool0"
    frame   = getattr(args, "frame", "base_link") or "base_link"
    service_name = getattr(args, "service", None) or "/compute_ik"
    timeout = getattr(args, "timeout", 10.0)

    # Convert RPY to radians for the quaternion
    if use_rad:
        roll, pitch, yaw = float(rpy_deg[0]), float(rpy_deg[1]), float(rpy_deg[2])
    else:
        roll  = math.radians(float(rpy_deg[0]))
        pitch = math.radians(float(rpy_deg[1]))
        yaw   = math.radians(float(rpy_deg[2]))

    # RPY → quaternion (intrinsic X-Y-Z / extrinsic Z-Y-X)
    cr, sr = math.cos(roll  / 2), math.sin(roll  / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw   / 2), math.sin(yaw   / 2)
    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy
    qw = cr * cp * cy + sr * sp * sy

    try:
        srv_class = get_srv_type("moveit_msgs/srv/GetPositionIK")
        if srv_class is None:
            return output({
                "error": "Cannot load moveit_msgs/srv/GetPositionIK",
                "hint": "Install: sudo apt install ros-$ROS_DISTRO-moveit-msgs",
            })

        with ros2_context():
            node = ROS2CLI()

            # Confirm service exists
            svc_types: list[str] = []
            for svc, types in node.get_service_names_and_types():
                if svc == service_name:
                    svc_types = list(types)
                    break

            if not svc_types:
                return output({
                    "error": f"Service '{service_name}' not found on the graph",
                    "hint": "Ensure MoveIt2 is running. Use --service to specify a custom IK service.",
                    "available_ik_services": [
                        s for s, _ in node.get_service_names_and_types()
                        if "ik" in s.lower() or "compute" in s.lower() or "moveit" in s.lower()
                    ][:10],
                })

            req = srv_class.Request()

            # Fill the IK request (moveit_msgs/msg/PositionIKRequest)
            req.ik_request.group_name = group

            # Pose stamped
            req.ik_request.pose_stamped.header.frame_id = frame
            req.ik_request.pose_stamped.pose.position.x = float(x)
            req.ik_request.pose_stamped.pose.position.y = float(y)
            req.ik_request.pose_stamped.pose.position.z = float(z)
            req.ik_request.pose_stamped.pose.orientation.x = qx
            req.ik_request.pose_stamped.pose.orientation.y = qy
            req.ik_request.pose_stamped.pose.orientation.z = qz
            req.ik_request.pose_stamped.pose.orientation.w = qw

            # EEF link
            if hasattr(req.ik_request, "ik_link_name"):
                req.ik_request.ik_link_name = eef

            # Timeout
            if hasattr(req.ik_request, "timeout"):
                import builtin_interfaces.msg as _bi  # noqa: PLC0415
                t = _bi.Duration()
                t.sec = int(timeout)
                t.nanosec = int((timeout - int(timeout)) * 1e9)
                req.ik_request.timeout = t

            client = node.create_client(srv_class, service_name)
            try:
                if not client.wait_for_service(timeout_sec=timeout):
                    return output({
                        "error": f"Service '{service_name}' not available (waited {timeout:.0f}s)",
                    })
                future = client.call_async(req)
                end = time.time() + timeout
                while time.time() < end and not future.done():
                    rclpy.spin_once(node, timeout_sec=0.1)
                if not future.done():
                    future.cancel()
                    return output({"error": f"IK service timed out after {timeout:.0f}s"})
                resp = future.result()
            finally:
                client.destroy()

        # MoveIt error codes: 1 = SUCCESS, negative = failure
        error_code = getattr(resp.error_code, "val", None)
        _EC_NAMES = {
            1: "SUCCESS", -1: "FAILURE", -2: "PLANNING_FAILED",
            -5: "INVALID_MOTION_PLAN", -7: "NO_IK_SOLUTION",
            -31: "FRAME_TRANSFORM_FAILURE", -32: "COLLISION_CHECKING_UNAVAILABLE",
        }
        ec_name = _EC_NAMES.get(error_code, f"CODE_{error_code}")
        reachable = (error_code == 1)

        result: dict = {
            "reachable":   reachable,
            "error_code":  error_code,
            "error_name":  ec_name,
            "service":     service_name,
            "target": {
                "x": float(x), "y": float(y), "z": float(z),
                "roll_deg": math.degrees(roll),
                "pitch_deg": math.degrees(pitch),
                "yaw_deg": math.degrees(yaw),
            },
        }

        if reachable and hasattr(resp, "solution"):
            sol = resp.solution
            joint_state = getattr(sol, "joint_state", None)
            if joint_state:
                names = list(getattr(joint_state, "name", []) or [])
                positions = list(getattr(joint_state, "position", []) or [])
                result["joint_solution"] = {
                    n: round(p, 6) for n, p in zip(names, positions)
                }

        if not reachable:
            result["hint"] = (
                f"No IK solution found (error: {ec_name}). "
                "Check the target pose is within the arm's workspace and "
                "doesn't collide with the environment."
            )

        return output(result)

    except Exception as e:
        return output({"error": str(e)})


if __name__ == "__main__":
    import sys as _sys
    _mod = __import__("os").path.basename(__file__)
    _cli = __import__("os").path.join(
        __import__("os").path.dirname(__import__("os").path.abspath(__file__)),
        "ros2_cli.py",
    )
    print(
        f"[ros2-skill] '{_mod}' is an internal module — do not run it directly.\n"
        "Use the main entry point:\n"
        f"  python3 {_cli} <command> [subcommand] [args]\n"
        f"See all commands:  python3 {_cli} --help",
        file=_sys.stderr,
    )
    _sys.exit(1)
