#!/usr/bin/env python3
"""Combined pre-motion preflight check.

Path A motion (see RULES-MOTION.md Rule 3 Step 1) requires two mandatory
live checks before every velocity command: (1) confirm the profile-named
controller is active, and (2) confirm the robot is stationary via odom.
Doing these as two separate CLI invocations pays rclpy.init()/shutdown()
--  i.e. full middleware session/discovery overhead -- twice. This module
runs both checks inside a single rclpy context and a single node spin,
cutting that overhead in half.

Further acceleration: if the persistent fast-daemon (ros2_fastd.py) is
running, both checks are serviced over its socket instead, avoiding
rclpy.init()/shutdown() entirely for this call. This is a pure latency
optimization — if the daemon isn't running, behavior is identical to the
non-daemon path (see ros2_utils.try_fastd).
"""

import math
import time

import rclpy

from ros2_control import _call_cm_service
from ros2_topic import TopicSubscriber
from ros2_utils import ROS2CLI, msg_to_dict, output, ros2_context, resolve_topic_type, try_fastd


def _finish_stationary_check(target, controller_active, msg, velocity_threshold, ready_key):
    """Shared math for both the motion and joint-command preflight checks
    where a Twist-shaped velocity needs to be read from a message — used
    only by cmd_preflight_motion (joint-command has its own finish step,
    since it reports positions rather than a stationary bool)."""
    twist = msg.get("twist", {}).get("twist", {})
    lin = twist.get("linear", {}) or {}
    ang = twist.get("angular", {}) or {}
    linear_speed = math.sqrt(
        lin.get("x", 0.0) ** 2 + lin.get("y", 0.0) ** 2 + lin.get("z", 0.0) ** 2
    )
    angular_speed_z = abs(ang.get("z", 0.0))
    stationary = linear_speed <= velocity_threshold and angular_speed_z <= velocity_threshold
    return {
        "controller": target,
        "controller_active": controller_active,
        "odom": msg,
        "stationary": stationary,
        "linear_speed": linear_speed,
        "angular_speed_z": angular_speed_z,
        ready_key: controller_active and stationary,
    }


def cmd_preflight_motion(args):
    """Check controller-active state and odom-stationary state in one node.

    Returns a single JSON object combining both results plus a top-level
    ``ready_for_motion`` bool so callers can act on one field instead of
    correlating two separate command outputs.
    """
    controller_manager = args.controller_manager
    timeout = args.timeout
    odom_topic = args.odom_topic
    stationary_timeout = args.stationary_timeout
    velocity_threshold = args.velocity_threshold

    # --- Fast path: persistent daemon, no rclpy.init() at all for this call ---
    daemon_ctrl = try_fastd("list_controllers", {
        "controller_manager": controller_manager, "timeout": timeout,
    })
    if daemon_ctrl is not None:
        if "error" in daemon_ctrl:
            return output(daemon_ctrl)
        controllers = daemon_ctrl.get("controllers", [])
        target = next((c for c in controllers if c.get("name") == args.controller), None)
        controller_active = bool(target) and target.get("state") == "active"

        daemon_odom = try_fastd(
            "subscribe_once", {"topic": odom_topic, "timeout": stationary_timeout},
            timeout=stationary_timeout + 2.0,
        )
        if daemon_odom is not None:
            if "error" in daemon_odom:
                err = dict(daemon_odom)
                err["controller"] = target
                err["controller_active"] = controller_active
                return output(err)
            return output(_finish_stationary_check(
                target, controller_active, daemon_odom["msg"],
                velocity_threshold, "ready_for_motion",
            ))
        # Daemon served the controller check but dropped before the odom
        # subscribe (rare — e.g. daemon restarted mid-sequence). Finish just
        # the odom half locally instead of discarding the daemon result.
        return output(_local_subscribe_and_finish(
            target, controller_active, odom_topic, stationary_timeout,
            velocity_threshold, "ready_for_motion",
        ))

    # --- Local fallback: daemon not running ---
    return _local_preflight_motion(args)


def _local_subscribe_and_finish(target, controller_active, topic, sub_timeout,
                                 velocity_threshold, ready_key):
    try:
        with ros2_context():
            node = ROS2CLI("skill_preflight_partial")
            msg_type = resolve_topic_type(node, topic, None)
            if not msg_type:
                node.destroy_node()
                return {
                    "error": f"Could not detect message type for topic: {topic}",
                    "controller": target, "controller_active": controller_active,
                }
            subscriber = TopicSubscriber(topic, msg_type)
            if subscriber.sub is None:
                node.destroy_node()
                subscriber.destroy_node()
                return {
                    "error": f"Could not load message type: {msg_type}",
                    "controller": target, "controller_active": controller_active,
                }
            executor = rclpy.executors.SingleThreadedExecutor()
            executor.add_node(subscriber)
            end_time = time.time() + sub_timeout
            msg = None
            while time.time() < end_time:
                executor.spin_once(timeout_sec=0.1)
                with subscriber.lock:
                    if subscriber.messages:
                        msg = subscriber.messages[0]
                        break
            subscriber.destroy_node()
            node.destroy_node()

        if msg is None:
            return {
                "error": f"Timeout after {sub_timeout}s waiting for a message on {topic}",
                "controller": target, "controller_active": controller_active,
            }
        return _finish_stationary_check(target, controller_active, msg, velocity_threshold, ready_key)
    except Exception as e:
        return {"error": str(e), "type": type(e).__name__}


def _local_preflight_motion(args):
    """Original single-process implementation — used when the fast daemon
    isn't running. Behavior is identical to the daemon path above."""
    try:
        with ros2_context():
            node = ROS2CLI("skill_preflight")

            from controller_manager_msgs.srv import ListControllers
            result, err = _call_cm_service(
                node, ListControllers, args.controller_manager,
                "list_controllers", ListControllers.Request(),
                args.timeout, getattr(args, 'retries', 1),
            )
            if err:
                node.destroy_node()
                return output(err)

            controllers = [msg_to_dict(c) for c in result.controller]
            target = next((c for c in controllers if c.get("name") == args.controller), None)
            controller_active = bool(target) and target.get("state") == "active"

            msg_type = resolve_topic_type(node, args.odom_topic, None)
            if not msg_type:
                node.destroy_node()
                return output({
                    "error": f"Could not detect message type for topic: {args.odom_topic}",
                    "controller": target,
                    "controller_active": controller_active,
                })

            subscriber = TopicSubscriber(args.odom_topic, msg_type)
            if subscriber.sub is None:
                node.destroy_node()
                subscriber.destroy_node()
                return output({
                    "error": f"Could not load message type: {msg_type}",
                    "controller": target,
                    "controller_active": controller_active,
                })

            executor = rclpy.executors.SingleThreadedExecutor()
            executor.add_node(subscriber)

            end_time = time.time() + args.stationary_timeout
            odom_msg = None
            while time.time() < end_time:
                executor.spin_once(timeout_sec=0.1)
                with subscriber.lock:
                    if subscriber.messages:
                        odom_msg = subscriber.messages[0]
                        break

            subscriber.destroy_node()
            node.destroy_node()

        if odom_msg is None:
            return output({
                "error": f"Timeout after {args.stationary_timeout}s waiting for odom message on {args.odom_topic}",
                "controller": target,
                "controller_active": controller_active,
            })

        output(_finish_stationary_check(
            target, controller_active, odom_msg, args.velocity_threshold, "ready_for_motion",
        ))
    except Exception as e:
        output({"error": str(e), "type": type(e).__name__})


def _finish_joint_command(target, controller_active, joint_msg, requested_joints):
    names = joint_msg.get("name", []) or []
    positions = joint_msg.get("position", []) or []
    velocities = joint_msg.get("velocity", []) or []

    joint_positions = {}
    joint_velocities = {}
    for i, n in enumerate(names):
        if requested_joints and n not in requested_joints:
            continue
        if i < len(positions):
            joint_positions[n] = positions[i]
        if i < len(velocities):
            joint_velocities[n] = velocities[i]

    missing = [j for j in requested_joints if j not in joint_positions]

    return {
        "controller": target,
        "controller_active": controller_active,
        "joint_positions": joint_positions,
        "joint_velocities": joint_velocities,
        "missing_joints": missing,
        "ready_for_command": controller_active and not missing,
    }


def cmd_preflight_joint_command(args):
    """Check controller-active state and current joint positions in one node.

    Analogous to cmd_preflight_motion, but for joint-position commands
    (pan-tilt, arm, gripper — any controller in
    profile.summary.joint_command_topics) instead of base velocity.
    """
    controller_manager = args.controller_manager
    timeout = args.timeout
    joint_state_topic = args.joint_state_topic
    state_timeout = args.state_timeout
    requested_joints = [j.strip() for j in (args.joints or "").split(",") if j.strip()]

    # --- Fast path: persistent daemon, no rclpy.init() at all for this call ---
    daemon_ctrl = try_fastd("list_controllers", {
        "controller_manager": controller_manager, "timeout": timeout,
    })
    if daemon_ctrl is not None:
        if "error" in daemon_ctrl:
            return output(daemon_ctrl)
        controllers = daemon_ctrl.get("controllers", [])
        target = next((c for c in controllers if c.get("name") == args.controller), None)
        controller_active = bool(target) and target.get("state") == "active"

        daemon_joint = try_fastd(
            "subscribe_once", {"topic": joint_state_topic, "timeout": state_timeout},
            timeout=state_timeout + 2.0,
        )
        if daemon_joint is not None:
            if "error" in daemon_joint:
                err = dict(daemon_joint)
                err["controller"] = target
                err["controller_active"] = controller_active
                return output(err)
            return output(_finish_joint_command(
                target, controller_active, daemon_joint["msg"], requested_joints,
            ))
        # Daemon dropped before the joint_states subscribe — finish locally.
        return output(_local_subscribe_and_finish_joint(
            target, controller_active, joint_state_topic, state_timeout, requested_joints,
        ))

    # --- Local fallback: daemon not running ---
    return _local_preflight_joint_command(args)


def _local_subscribe_and_finish_joint(target, controller_active, topic, sub_timeout, requested_joints):
    try:
        with ros2_context():
            node = ROS2CLI("skill_preflight_joint_partial")
            msg_type = resolve_topic_type(node, topic, None)
            if not msg_type:
                node.destroy_node()
                return {
                    "error": f"Could not detect message type for topic: {topic}",
                    "controller": target, "controller_active": controller_active,
                }
            subscriber = TopicSubscriber(topic, msg_type)
            if subscriber.sub is None:
                node.destroy_node()
                subscriber.destroy_node()
                return {
                    "error": f"Could not load message type: {msg_type}",
                    "controller": target, "controller_active": controller_active,
                }
            executor = rclpy.executors.SingleThreadedExecutor()
            executor.add_node(subscriber)
            end_time = time.time() + sub_timeout
            msg = None
            while time.time() < end_time:
                executor.spin_once(timeout_sec=0.1)
                with subscriber.lock:
                    if subscriber.messages:
                        msg = subscriber.messages[0]
                        break
            subscriber.destroy_node()
            node.destroy_node()

        if msg is None:
            return {
                "error": f"Timeout after {sub_timeout}s waiting for a message on {topic}",
                "controller": target, "controller_active": controller_active,
            }
        return _finish_joint_command(target, controller_active, msg, requested_joints)
    except Exception as e:
        return {"error": str(e), "type": type(e).__name__}


def _local_preflight_joint_command(args):
    """Original single-process implementation — used when the fast daemon
    isn't running. Behavior is identical to the daemon path above."""
    try:
        with ros2_context():
            node = ROS2CLI("skill_preflight_joint")

            from controller_manager_msgs.srv import ListControllers
            result, err = _call_cm_service(
                node, ListControllers, args.controller_manager,
                "list_controllers", ListControllers.Request(),
                args.timeout, getattr(args, 'retries', 1),
            )
            if err:
                node.destroy_node()
                return output(err)

            controllers = [msg_to_dict(c) for c in result.controller]
            target = next((c for c in controllers if c.get("name") == args.controller), None)
            controller_active = bool(target) and target.get("state") == "active"

            msg_type = resolve_topic_type(node, args.joint_state_topic, None)
            if not msg_type:
                node.destroy_node()
                return output({
                    "error": f"Could not detect message type for topic: {args.joint_state_topic}",
                    "controller": target,
                    "controller_active": controller_active,
                })

            subscriber = TopicSubscriber(args.joint_state_topic, msg_type)
            if subscriber.sub is None:
                node.destroy_node()
                subscriber.destroy_node()
                return output({
                    "error": f"Could not load message type: {msg_type}",
                    "controller": target,
                    "controller_active": controller_active,
                })

            executor = rclpy.executors.SingleThreadedExecutor()
            executor.add_node(subscriber)

            end_time = time.time() + args.state_timeout
            joint_msg = None
            while time.time() < end_time:
                executor.spin_once(timeout_sec=0.1)
                with subscriber.lock:
                    if subscriber.messages:
                        joint_msg = subscriber.messages[0]
                        break

            subscriber.destroy_node()
            node.destroy_node()

        if joint_msg is None:
            return output({
                "error": f"Timeout after {args.state_timeout}s waiting for a message on {args.joint_state_topic}",
                "controller": target,
                "controller_active": controller_active,
            })

        requested_joints = [j.strip() for j in (args.joints or "").split(",") if j.strip()]
        output(_finish_joint_command(target, controller_active, joint_msg, requested_joints))
    except Exception as e:
        output({"error": str(e), "type": type(e).__name__})
