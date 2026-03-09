#!/usr/bin/env python3
"""ROS 2 action commands."""

import json
import re
import threading
import time

import rclpy

from ros2_utils import (
    ROS2CLI, get_action_type, msg_to_dict, dict_to_msg, output,
)


def cmd_actions_list(args):
    try:
        rclpy.init()
        node = ROS2CLI()

        topics = node.get_topic_names_and_types()
        actions = []
        seen = set()

        for name, types in topics:
            if '/_action/' in name:
                action_name = name.split('/_action/')[0]
                if action_name not in seen:
                    seen.add(action_name)
                    actions.append(action_name)

        rclpy.shutdown()
        output({"actions": actions, "count": len(actions)})
    except Exception as e:
        output({"error": str(e)})


def cmd_actions_details(args):
    try:
        rclpy.init()
        node = ROS2CLI()

        topics = node.get_topic_names_and_types()

        result = {"action": args.action, "action_type": "", "goal": {}, "result": {}, "feedback": {}}

        action_type = ""
        for name, types in topics:
            if name == args.action + "/_action/feedback":
                for t in types:
                    if '/action/' in t:
                        action_type = re.sub(r'_FeedbackMessage$', '', t)
                        break
                if action_type:
                    break

        result["action_type"] = action_type

        if action_type:
            action_class = get_action_type(action_type)
            if action_class:
                try:
                    result["goal"] = msg_to_dict(action_class.Goal())
                except Exception:
                    pass
                try:
                    result["result"] = msg_to_dict(action_class.Result())
                except Exception:
                    pass
                try:
                    result["feedback"] = msg_to_dict(action_class.Feedback())
                except Exception:
                    pass

        rclpy.shutdown()
        output(result)
    except Exception as e:
        output({"error": str(e)})


def cmd_actions_send(args):
    try:
        goal_data = json.loads(args.goal)
    except (json.JSONDecodeError, TypeError, ValueError) as e:
        return output({"error": f"Invalid JSON goal: {e}"})

    try:
        from rclpy.action import ActionClient
        rclpy.init()
        node = ROS2CLI()

        topics = node.get_topic_names_and_types()

        action_type = None
        for name, types in topics:
            if name == args.action + "/_action/feedback":
                for t in types:
                    if '/action/' in t:
                        action_type = re.sub(r'_FeedbackMessage$', '', t)
                        break
                if action_type:
                    break

        if not action_type:
            rclpy.shutdown()
            return output({"error": f"Action server not found: {args.action}"})

        action_class = get_action_type(action_type)
        if not action_class:
            rclpy.shutdown()
            return output({"error": f"Cannot load action type: {action_type}"})

        client = ActionClient(node, action_class, args.action)
        timeout = args.timeout
        retries = getattr(args, 'retries', 1)

        if not client.wait_for_server(timeout_sec=timeout):
            rclpy.shutdown()
            return output({"error": f"Action server not available: {args.action}"})

        goal_id = f"goal_{int(time.time() * 1000)}"
        collect_feedback = getattr(args, 'feedback', False)

        for attempt in range(retries):
            last_attempt = (attempt == retries - 1)

            feedback_msgs = []
            feedback_lock = threading.Lock()

            def _feedback_cb(fb_msg):
                if collect_feedback:
                    with feedback_lock:
                        feedback_msgs.append(msg_to_dict(fb_msg.feedback))

            goal_msg = dict_to_msg(action_class.Goal, goal_data)
            future = client.send_goal_async(
                goal_msg,
                feedback_callback=_feedback_cb if collect_feedback else None,
            )

            end_time = time.time() + timeout
            while time.time() < end_time and not future.done():
                rclpy.spin_once(node, timeout_sec=0.1)

            if not future.done():
                if not last_attempt:
                    continue
                rclpy.shutdown()
                output({"action": args.action, "success": False,
                        "error": "Timeout waiting for goal acceptance"})
                return

            goal_handle = future.result()

            if not goal_handle.accepted:
                if not last_attempt:
                    continue
                rclpy.shutdown()
                output({"action": args.action, "success": False, "error": "Goal rejected"})
                return

            result_future = goal_handle.get_result_async()

            end_time = time.time() + timeout
            while time.time() < end_time and not result_future.done():
                rclpy.spin_once(node, timeout_sec=0.1)

            if result_future.done():
                result_msg = result_future.result().result
                result_dict = msg_to_dict(result_msg)
                rclpy.shutdown()
                out = {"action": args.action, "success": True,
                       "goal_id": goal_id, "result": result_dict}
                if collect_feedback:
                    with feedback_lock:
                        out["feedback_msgs"] = list(feedback_msgs)
                output(out)
                return

            if not last_attempt:
                continue

        rclpy.shutdown()
        output({"action": args.action, "success": False,
                "error": f"Timeout after {timeout}s"})
    except Exception as e:
        output({"error": str(e)})


def cmd_actions_type(args):
    """Get the type of an action server."""
    if not args.action:
        return output({"error": "action argument is required"})

    action = args.action.rstrip('/')

    try:
        rclpy.init()
        node = ROS2CLI()
        all_topics = node.get_topic_names_and_types()
        rclpy.shutdown()

        feedback_topic = action + '/_action/feedback'
        action_type = None
        for name, types in all_topics:
            if name == feedback_topic and types:
                raw = types[0]
                action_type = re.sub(r'_FeedbackMessage$', '', raw)
                break

        if action_type is None:
            return output({"error": f"Action '{action}' not found in the ROS graph"})

        output({"action": action, "type": action_type})
    except Exception as e:
        output({"error": str(e)})


def cmd_actions_cancel(args):
    """Cancel all in-flight goals on an action server."""
    if not args.action:
        return output({"error": "action argument is required"})

    action = args.action.rstrip('/')
    timeout = args.timeout

    try:
        from action_msgs.srv import CancelGoal
        from builtin_interfaces.msg import Time as BuiltinTime
        rclpy.init()
        node = ROS2CLI()

        service_name = action + '/_action/cancel_goal'
        client = node.create_client(CancelGoal, service_name)

        if not client.wait_for_service(timeout_sec=timeout):
            rclpy.shutdown()
            return output({"error": f"Action server '{action}' not available"})

        request = CancelGoal.Request()
        request.goal_info.goal_id.uuid = [0] * 16
        request.goal_info.stamp = BuiltinTime(sec=0, nanosec=0)

        future = client.call_async(request)
        end_time = time.time() + timeout
        while time.time() < end_time and not future.done():
            rclpy.spin_once(node, timeout_sec=0.1)

        rclpy.shutdown()
        if not future.done():
            return output({"error": f"Timeout cancelling goals on '{action}'"})

        result = future.result()
        cancelled = [str(bytes(g.goal_id.uuid)) for g in (result.goals_canceling or [])]
        output({
            "action": action,
            "return_code": result.return_code,
            "cancelled_goals": len(cancelled),
        })
    except Exception as e:
        output({"error": str(e)})


def cmd_actions_find(args):
    """Find action servers of a specific action type."""
    if not args.action_type:
        return output({"error": "action_type argument is required"})

    target_raw = args.action_type

    def _norm_action(t):
        return re.sub(r'/action/', '/', t)

    target_norm = _norm_action(target_raw)

    try:
        rclpy.init()
        node = ROS2CLI()
        all_topics = node.get_topic_names_and_types()
        rclpy.shutdown()

        matched = []
        seen = set()
        for name, types in all_topics:
            if '/_action/feedback' in name:
                action_name = name.split('/_action/')[0]
                if action_name in seen:
                    continue
                for t in types:
                    resolved = re.sub(r'_FeedbackMessage$', '', t)
                    if _norm_action(resolved) == target_norm:
                        matched.append(action_name)
                        seen.add(action_name)
                        break

        output({
            "action_type": target_raw,
            "actions": matched,
            "count": len(matched),
        })
    except Exception as e:
        output({"error": str(e)})
