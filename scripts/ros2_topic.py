#!/usr/bin/env python3
"""ROS 2 topic commands and emergency stop."""

import json
import math
import os
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_system_default

from ros2_utils import (
    ROS2CLI, get_msg_type, get_msg_error, get_msg_fields,
    msg_to_dict, dict_to_msg, output, resolve_field, _get_service_event_qos,
    resolve_output_path, ros2_context, resolve_topic_type,
)

# ---------------------------------------------------------------------------
# Velocity message types used to identify velocity command topics.
VELOCITY_TYPES = {
    "geometry_msgs/msg/Twist", "geometry_msgs/msg/TwistStamped",
    "geometry_msgs/Twist", "geometry_msgs/TwistStamped",
}

# Diagnostic message types (with and without /msg/ qualifier).
DIAG_TYPES = {
    "diagnostic_msgs/msg/DiagnosticArray",
    "diagnostic_msgs/DiagnosticArray",
}

# Numeric level → human-readable name (mirrors DiagnosticStatus constants).
_DIAG_LEVEL_NAMES = {0: "OK", 1: "WARN", 2: "ERROR", 3: "STALE"}

# Battery message types (with and without /msg/ qualifier).
BATTERY_TYPES = {
    "sensor_msgs/msg/BatteryState",
    "sensor_msgs/BatteryState",
}

# Numeric code → human-readable name (mirrors BatteryState constants).
_BATTERY_POWER_SUPPLY_STATUS = {
    0: "UNKNOWN", 1: "CHARGING", 2: "DISCHARGING", 3: "NOT_CHARGING", 4: "FULL",
}
_BATTERY_POWER_SUPPLY_HEALTH = {
    0: "UNKNOWN", 1: "GOOD", 2: "OVERHEAT", 3: "DEAD", 4: "OVERVOLTAGE",
    5: "UNSPEC_FAILURE", 6: "COLD", 7: "WATCHDOG_TIMER_EXPIRE", 8: "SAFETY_TIMER_EXPIRE",
}
_BATTERY_POWER_SUPPLY_TECHNOLOGY = {
    0: "UNKNOWN", 1: "NIMH", 2: "LION", 3: "LIPO", 4: "LIFE", 5: "NICD", 6: "LIMN",
}


# ---------------------------------------------------------------------------
# estop
# ---------------------------------------------------------------------------

def cmd_estop(args):
    """Emergency stop for mobile robots - auto-detect and publish zero velocity."""

    def find_velocity_topic(node):
        topics = node.get_topic_names()
        candidates = [
            (name, next(t for t in types if t in VELOCITY_TYPES))
            for name, types in topics
            if any(t in VELOCITY_TYPES for t in types)
        ]
        if not candidates:
            return None, None
        preferred = [(name, t) for name, t in candidates if "cmd_vel" in name.lower()]
        return preferred[0] if preferred else candidates[0]

    try:
        with ros2_context():
            node = ROS2CLI("estop")

            if args.topic:
                topic = args.topic
                msg_type = None
                for name, types in node.get_topic_names():
                    if name == topic and types:
                        msg_type = types[0]
                        break
                if not msg_type:
                    return output({
                        "error": f"Could not detect message type for topic '{topic}'",
                        "hint": "Ensure the topic is active and visible in the ROS graph. Use 'topics type <topic>' to inspect it."
                    })
            else:
                topic, msg_type = find_velocity_topic(node)

            if not topic:
                return output({
                    "error": "Could not find velocity command topic",
                    "hint": "This command is for mobile robots only (not arms). Ensure the robot has a /cmd_vel topic."
                })

            msg_class = get_msg_type(msg_type)
            if not msg_class:
                if args.topic:
                    return output({
                        "error": f"Could not load message type '{msg_type}' for topic '{topic}'",
                        "hint": "Ensure the ROS workspace is built and sourced: cd ~/ros2_ws && colcon build && source install/setup.bash"
                    })
                else:
                    for t in VELOCITY_TYPES:
                        msg_class = get_msg_type(t)
                        if msg_class:
                            msg_type = t
                            break

            if not msg_class:
                return output({"error": f"Could not load message type: {msg_type}"})

            pub = node.create_publisher(msg_class, topic, 10)
            msg = msg_class()

            if hasattr(msg, "twist"):
                msg.header.stamp = node.get_clock().now().to_msg()
                msg.twist.linear.x = 0.0
                msg.twist.linear.y = 0.0
                msg.twist.linear.z = 0.0
                msg.twist.angular.x = 0.0
                msg.twist.angular.y = 0.0
                msg.twist.angular.z = 0.0
            else:
                msg.linear.x = 0.0
                msg.linear.y = 0.0
                msg.linear.z = 0.0
                msg.angular.x = 0.0
                msg.angular.y = 0.0
                msg.angular.z = 0.0

            pub.publish(msg)
            time.sleep(0.1)

        output({
            "success": True,
            "topic": topic,
            "type": msg_type,
            "message": "Emergency stop activated (mobile robot stopped)"
        })
    except Exception as e:
        output({"error": str(e)})


# ---------------------------------------------------------------------------
# topics list / type / details / message
# ---------------------------------------------------------------------------

def cmd_topics_list(args):
    try:
        with ros2_context():
            node = ROS2CLI()
            topics = node.get_topic_names()
            topic_list = []
            type_list = []
            for name, types in topics:
                topic_list.append(name)
                type_list.append(types[0] if types else "")
            result = {"topics": topic_list, "types": type_list, "count": len(topic_list)}
        output(result)
    except Exception as e:
        output({"error": str(e)})


def cmd_topics_type(args):
    try:
        with ros2_context():
            node = ROS2CLI()
            topics = node.get_topic_names()
            result = {"topic": args.topic, "type": ""}
            for name, types in topics:
                if name == args.topic:
                    result["type"] = types[0] if types else ""
                    break
        output(result)
    except Exception as e:
        output({"error": str(e)})


def cmd_topics_details(args):
    try:
        with ros2_context():
            node = ROS2CLI()
            topic_types = node.get_topic_names_and_types()
            result = {"topic": args.topic, "type": "", "publishers": [], "subscribers": []}

            for name, types in topic_types:
                if name == args.topic:
                    result["type"] = types[0] if types else ""
                    break

            try:
                pub_info = node.get_publishers_info_by_topic(args.topic)
                result["publishers"] = [
                    f"{i.node_namespace.rstrip('/')}/{i.node_name}" for i in pub_info
                ]
            except Exception:
                pass

            try:
                sub_info = node.get_subscriptions_info_by_topic(args.topic)
                result["subscribers"] = [
                    f"{i.node_namespace.rstrip('/')}/{i.node_name}" for i in sub_info
                ]
            except Exception:
                pass

        output(result)
    except Exception as e:
        output({"error": str(e)})


def cmd_topics_message(args):
    try:
        fields = get_msg_fields(args.message_type)
        output({"message_type": args.message_type, "structure": fields})
    except Exception as e:
        output({"error": str(e)})


# ---------------------------------------------------------------------------
# Subscriber node
# ---------------------------------------------------------------------------

class TopicSubscriber(Node):
    def __init__(self, topic, msg_type, msg_class=None, qos=None):
        super().__init__('subscriber')
        self.msg_type = msg_type
        self.messages = []
        self.lock = threading.Lock()
        self.sub = None

        resolved_class = msg_class if msg_class is not None else get_msg_type(msg_type)
        resolved_qos = qos if qos is not None else qos_profile_system_default
        if resolved_class:
            self.sub = self.create_subscription(
                resolved_class, topic, self.callback, resolved_qos
            )

    def callback(self, msg):
        with self.lock:
            self.messages.append(msg_to_dict(msg))


def cmd_topics_subscribe(args):
    if not args.topic:
        return output({"error": "topic argument is required"})

    try:
        with ros2_context():
            temp = ROS2CLI("temp")
            msg_type = resolve_topic_type(temp, args.topic, args.msg_type)
            temp.destroy_node()

            if not msg_type:
                return output({"error": f"Could not detect message type for topic: {args.topic}"})

            subscriber = TopicSubscriber(args.topic, msg_type)

            if subscriber.sub is None:
                return output({"error": f"Could not load message type: {msg_type}"})

            executor = rclpy.executors.SingleThreadedExecutor()
            executor.add_node(subscriber)

            if args.duration:
                end_time = time.time() + args.duration
                while time.time() < end_time and len(subscriber.messages) < (args.max_messages or 100):
                    executor.spin_once(timeout_sec=0.1)

                with subscriber.lock:
                    messages = subscriber.messages[:args.max_messages] if args.max_messages else subscriber.messages

                output({
                    "topic": args.topic,
                    "collected_count": len(messages),
                    "messages": messages
                })
            else:
                timeout_sec = args.timeout
                end_time = time.time() + timeout_sec

                while time.time() < end_time:
                    executor.spin_once(timeout_sec=0.1)
                    with subscriber.lock:
                        if subscriber.messages:
                            output({"msg": subscriber.messages[0]})
                            return

                output({"error": "Timeout waiting for message"})
    except Exception as e:
        output({"error": str(e)})


# ---------------------------------------------------------------------------
# Publisher node
# ---------------------------------------------------------------------------

class TopicPublisher(Node):
    def __init__(self, topic, msg_type):
        super().__init__('publisher')
        self.topic = topic
        self.msg_type = msg_type
        self.pub = None

        msg_class = get_msg_type(msg_type)
        if msg_class:
            self.pub = self.create_publisher(msg_class, topic, qos_profile_system_default)


# ---------------------------------------------------------------------------
# Condition monitor node (for publish-until)
# ---------------------------------------------------------------------------

def quaternion_to_yaw(q):
    """Extract yaw (rotation around z-axis) from quaternion (x, y, z, w)."""
    x, y, z, w = q
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return yaw


def normalize_angle(angle):
    """Normalize angle to [-pi, pi]."""
    while angle > math.pi:
        angle -= 2 * math.pi
    while angle < -math.pi:
        angle += 2 * math.pi
    return angle


class ConditionMonitor(Node):
    """Subscriber that evaluates a stop condition on every incoming message."""

    def __init__(self, topic, msg_type, field, operator, threshold, stop_event,
                 euclidean=False, rotate=None):
        super().__init__('condition_monitor')
        self.euclidean = euclidean
        self.rotate = rotate
        if isinstance(field, list):
            self.fields = field
            self.field = field[0]
        else:
            self.fields = [field]
            self.field = field
        self.operator = operator
        self.threshold = threshold
        self.stop_event = stop_event

        self.start_value = None
        self.current_value = None
        self.start_values = None
        self.current_values = None
        self.euclidean_distance = None

        # Rotation-specific
        self.start_yaw = None
        self.last_yaw = None
        self.accumulated_rotation = 0.0
        self.rotation_delta = None

        self.start_msg = None
        self.end_msg = None
        self.field_error = None
        self.lock = threading.Lock()
        self.sub = None

        msg_class = get_msg_type(msg_type)
        if msg_class:
            self.sub = self.create_subscription(
                msg_class, topic, self.callback, qos_profile_system_default
            )

    def callback(self, msg):
        with self.lock:
            if self.stop_event.is_set():
                return

            msg_dict = msg_to_dict(msg)

            if self.euclidean:
                values = []
                for fp in self.fields:
                    try:
                        v = resolve_field(msg_dict, fp)
                    except (KeyError, IndexError, TypeError) as e:
                        self.field_error = f"Field '{fp}' not found in monitor message: {e}"
                        self.stop_event.set()
                        return

                    if isinstance(v, dict):
                        numeric_children = [
                            (k, float(v[k]))
                            for k in sorted(v.keys())
                            if isinstance(v[k], (int, float))
                        ]
                        if not numeric_children:
                            self.field_error = (
                                f"Field '{fp}' resolves to a dict with no direct numeric "
                                f"sub-fields (keys: {sorted(v.keys())}). "
                                f"Specify a leaf field such as {fp}.x"
                            )
                            self.stop_event.set()
                            return
                        values.extend(val for _, val in numeric_children)
                    else:
                        try:
                            values.append(float(v))
                        except (TypeError, ValueError) as e:
                            self.field_error = f"Field '{fp}' not numeric in monitor message: {e}"
                            self.stop_event.set()
                            return

                if self.start_values is None:
                    self.start_values = values[:]
                    self.start_msg = msg_dict

                self.current_values = values
                dist = math.sqrt(
                    sum((c - s) ** 2 for c, s in zip(values, self.start_values))
                )
                self.euclidean_distance = dist

                if dist >= float(self.threshold):
                    self.end_msg = msg_dict
                    self.stop_event.set()

            elif self.rotate is not None:
                try:
                    quat = None
                    for path in ['pose.pose.orientation', 'orientation']:
                        try:
                            quat = resolve_field(msg_dict, path)
                            if quat and isinstance(quat, dict):
                                break
                        except (KeyError, IndexError, TypeError):
                            continue

                    if not quat or not isinstance(quat, dict):
                        self.field_error = "Could not find quaternion (pose.pose.orientation) in odometry message for rotation monitoring"
                        self.stop_event.set()
                        return

                    try:
                        qx = float(quat.get('x', 0))
                        qy = float(quat.get('y', 0))
                        qz = float(quat.get('z', 0))
                        qw = float(quat.get('w', 1))
                    except (TypeError, ValueError) as e:
                        self.field_error = f"Quaternion values must be numeric: {e}"
                        self.stop_event.set()
                        return

                    current_yaw = quaternion_to_yaw((qx, qy, qz, qw))

                    if self.start_yaw is None:
                        self.start_yaw = current_yaw
                        self.last_yaw = current_yaw
                        self.start_msg = msg_dict
                        return

                    step = normalize_angle(current_yaw - self.last_yaw)
                    self.accumulated_rotation += step
                    self.last_yaw = current_yaw
                    self.rotation_delta = self.accumulated_rotation

                    target = self.rotate
                    if target > 0 and self.accumulated_rotation >= target:
                        self.end_msg = msg_dict
                        self.stop_event.set()
                    elif target < 0 and self.accumulated_rotation <= target:
                        self.end_msg = msg_dict
                        self.stop_event.set()

                except Exception as e:
                    self.field_error = f"Rotation monitoring error: {e}"
                    self.stop_event.set()
                    return

            else:
                try:
                    value = resolve_field(msg_dict, self.field)
                except (KeyError, IndexError, TypeError, ValueError) as e:
                    self.field_error = f"Field '{self.field}' not found in monitor message: {e}"
                    self.stop_event.set()
                    return

                try:
                    numeric_value = float(value)
                except (TypeError, ValueError):
                    numeric_value = None

                if self.start_value is None:
                    self.start_value = value
                    self.start_msg = msg_dict

                self.current_value = value

                condition_met = False
                if self.operator == 'delta':
                    if numeric_value is not None and self.start_value is not None:
                        try:
                            delta = numeric_value - float(self.start_value)
                            thr = float(self.threshold)
                            condition_met = delta >= thr if thr >= 0 else delta <= thr
                        except (TypeError, ValueError):
                            pass
                elif self.operator == 'above':
                    if numeric_value is not None:
                        condition_met = numeric_value > float(self.threshold)
                elif self.operator == 'below':
                    if numeric_value is not None:
                        condition_met = numeric_value < float(self.threshold)
                elif self.operator == 'equals':
                    if numeric_value is not None:
                        try:
                            condition_met = abs(numeric_value - float(self.threshold)) < 1e-9
                        except (TypeError, ValueError):
                            condition_met = str(value) == str(self.threshold)
                    else:
                        condition_met = str(value) == str(self.threshold)

                if condition_met:
                    self.end_msg = msg_dict
                    self.stop_event.set()


# ---------------------------------------------------------------------------
# Hz monitor node
# ---------------------------------------------------------------------------

class HzMonitor(Node):
    """Subscriber that measures the publish rate of a topic."""

    def __init__(self, topic, msg_type, window, done_event):
        super().__init__('hz_monitor')
        self.window = window
        self.done_event = done_event
        self.timestamps = []
        self.lock = threading.Lock()
        self.sub = None

        msg_class = get_msg_type(msg_type)
        if msg_class:
            self.sub = self.create_subscription(
                msg_class, topic, self._callback, qos_profile_system_default
            )

    def _callback(self, msg):
        with self.lock:
            if self.done_event.is_set():
                return
            self.timestamps.append(time.time())
            if len(self.timestamps) >= self.window + 1:
                self.done_event.set()


# ---------------------------------------------------------------------------
# topics publish
# ---------------------------------------------------------------------------

def cmd_topics_publish(args):
    if not args.topic:
        return output({"error": "topic argument is required"})
    if args.msg is None:
        return output({"error": "msg argument is required"})

    try:
        msg_data = json.loads(args.msg)
    except (json.JSONDecodeError, TypeError, ValueError) as e:
        return output({"error": f"Invalid JSON message: {e}"})

    try:
        with ros2_context():
            topic = args.topic
            temp_node = ROS2CLI("temp")
            msg_type = resolve_topic_type(temp_node, topic, args.msg_type)
            temp_node.destroy_node()

            if not msg_type:
                return output({"error": f"Could not detect message type for topic: {topic}. Use --msg-type to specify."})

            msg_class = get_msg_type(msg_type)
            if not msg_class:
                return output(get_msg_error(msg_type))

            publisher = TopicPublisher(topic, msg_type)

            if publisher.pub is None:
                return output({"error": f"Failed to create publisher for {msg_type}"})

            msg = dict_to_msg(msg_class, msg_data)

            rate = getattr(args, "rate", None) or 10.0
            duration = getattr(args, "duration", None) or getattr(args, "timeout", None)
            interval = 1.0 / rate

            if duration and duration > 0:
                published = 0
                start_time = time.time()
                end_time = start_time + duration
                stopped_by = "timeout"
                try:
                    while time.time() < end_time:
                        publisher.pub.publish(msg)
                        published += 1
                        remaining = end_time - time.time()
                        if remaining > 0:
                            time.sleep(min(interval, remaining))
                except KeyboardInterrupt:
                    stopped_by = "keyboard_interrupt"
                elapsed = round(time.time() - start_time, 3)
                output({"success": True, "topic": topic, "msg_type": msg_type,
                        "duration": elapsed, "rate": rate, "published_count": published,
                        "stopped_by": stopped_by})
            else:
                publisher.pub.publish(msg)
                time.sleep(0.1)
                output({"success": True, "topic": topic, "msg_type": msg_type})
    except Exception as e:
        output({"error": str(e)})


def cmd_topics_publish_sequence(args):
    if not args.topic:
        return output({"error": "topic argument is required"})
    if args.messages is None:
        return output({"error": "messages argument is required"})
    if args.durations is None:
        return output({"error": "durations argument is required"})

    try:
        messages = json.loads(args.messages)
        durations = json.loads(args.durations)
    except (json.JSONDecodeError, TypeError, ValueError) as e:
        return output({"error": f"Invalid JSON: {e}"})
    if len(messages) != len(durations):
        return output({"error": "messages and durations must have same length"})

    try:
        with ros2_context():
            topic = args.topic
            temp_node = ROS2CLI("temp")
            msg_type = resolve_topic_type(temp_node, topic, args.msg_type)
            temp_node.destroy_node()

            if not msg_type:
                return output({"error": f"Could not detect message type for topic: {topic}. Use --msg-type to specify."})

            msg_class = get_msg_type(msg_type)
            if not msg_class:
                return output(get_msg_error(msg_type))

            publisher = TopicPublisher(topic, msg_type)

            if publisher.pub is None:
                return output({"error": f"Failed to create publisher for {msg_type}"})

            rate = getattr(args, "rate", None) or 10.0
            interval = 1.0 / rate

            total_published = 0
            for msg_data, dur in zip(messages, durations):
                msg = dict_to_msg(msg_class, msg_data)
                if dur > 0:
                    end_time = time.time() + dur
                    while time.time() < end_time:
                        publisher.pub.publish(msg)
                        total_published += 1
                        remaining = end_time - time.time()
                        if remaining > 0:
                            time.sleep(min(interval, remaining))
                else:
                    publisher.pub.publish(msg)
                    total_published += 1

            output({"success": True, "published_count": total_published, "topic": topic,
                    "rate": rate})
    except Exception as e:
        output({"error": str(e)})


def cmd_topics_publish_until(args):
    """Publish to a topic until a condition on a monitor topic is met."""
    if not args.topic:
        return output({"error": "topic argument is required"})
    if args.msg is None:
        return output({"error": "msg argument is required"})
    if not args.monitor:
        return output({"error": "--monitor argument is required"})

    # Handle --rotate flag
    rotate_angle = getattr(args, 'rotate', None)
    use_degrees = getattr(args, 'degrees', False)

    if rotate_angle is not None:
        if args.field:
            return output({"error": "--rotate cannot be used with --field"})
        if args.euclidean:
            return output({"error": "--rotate cannot be used with --euclidean"})
        if args.delta or args.above or args.below or args.equals:
            return output({"error": "--rotate cannot be used with --delta, --above, --below, or --equals"})
        try:
            rotate_angle = float(rotate_angle)
        except (TypeError, ValueError):
            return output({"error": "--rotate must be a numeric value"})
        if use_degrees:
            rotate_angle = math.radians(rotate_angle)
        if rotate_angle == 0:
            return output({"error": "--rotate angle must be non-zero"})
    else:
        if not args.field:
            return output({"error": "--field argument is required (or use --rotate for rotation)"})

    field_paths = args.field if args.field else []
    euclidean = getattr(args, 'euclidean', False)

    if rotate_angle is None and len(field_paths) > 1 and not euclidean:
        return output({"error": "Multiple --field paths require the --euclidean flag"})

    operator_map = {
        'delta': args.delta,
        'above': args.above,
        'below': args.below,
        'equals': args.equals,
    }
    active = {k: v for k, v in operator_map.items() if v is not None}
    if rotate_angle is None and len(active) != 1:
        return output({"error": "Specify exactly one of --delta, --above, --below, --equals (or --rotate for rotation)"})

    if euclidean and rotate_angle is None and next(iter(active)) != 'delta':
        return output({"error": "--euclidean requires --delta (threshold is the Euclidean distance from start)"})
    operator, threshold = next(iter(active.items())) if active else (None, None)
    if euclidean and operator == 'delta' and threshold <= 0:
        return output({"error": "--delta must be > 0 when --euclidean is used"})

    try:
        msg_data = json.loads(args.msg)
    except (json.JSONDecodeError, TypeError, ValueError) as e:
        return output({"error": f"Invalid JSON message: {e}"})

    try:
        with ros2_context():
            temp_node = ROS2CLI("temp")
            pub_msg_type = resolve_topic_type(temp_node, args.topic, args.msg_type)
            mon_msg_type = resolve_topic_type(temp_node, args.monitor, args.monitor_msg_type)
            temp_node.destroy_node()

            if not pub_msg_type:
                return output({"error": f"Could not detect message type for topic: {args.topic}. Use --msg-type."})
            if not mon_msg_type:
                return output({"error": f"Could not detect message type for monitor topic: {args.monitor}. Use --monitor-msg-type."})

            pub_msg_class = get_msg_type(pub_msg_type)
            if not pub_msg_class:
                return output(get_msg_error(pub_msg_type))

            # Re-read rotate_angle so any degree conversion is applied consistently
            rotate_angle = getattr(args, 'rotate', None)
            use_degrees = getattr(args, 'degrees', False)
            if rotate_angle is not None:
                try:
                    rotate_angle = float(rotate_angle)
                except (TypeError, ValueError):
                    return output({"error": "--rotate must be a numeric value"})
                if use_degrees:
                    rotate_angle = math.radians(rotate_angle)
                field_paths = ['pose.pose.orientation']

            stop_event = threading.Event()
            publisher = TopicPublisher(args.topic, pub_msg_type)
            monitor = ConditionMonitor(
                args.monitor, mon_msg_type,
                field_paths if euclidean else field_paths[0],
                operator, threshold, stop_event,
                euclidean=euclidean,
                rotate=rotate_angle,
            )

            if publisher.pub is None:
                return output({"error": f"Failed to create publisher for {pub_msg_type}"})
            if monitor.sub is None:
                return output({"error": f"Could not load monitor message type: {mon_msg_type}"})

            msg = dict_to_msg(pub_msg_class, msg_data)

            executor = rclpy.executors.SingleThreadedExecutor()
            executor.add_node(publisher)
            executor.add_node(monitor)

            rate = args.rate or 10.0
            timeout = args.timeout
            interval = 1.0 / rate
            start_time = time.time()
            published_count = 0

            try:
                while not stop_event.is_set():
                    if timeout and (time.time() - start_time) >= timeout:
                        break
                    publisher.pub.publish(msg)
                    published_count += 1
                    executor.spin_once(timeout_sec=interval)
            finally:
                stop_event.set()

            elapsed = round(time.time() - start_time, 3)

            with monitor.lock:
                if monitor.field_error:
                    return output({"error": monitor.field_error})

                condition_met = monitor.end_msg is not None

                if euclidean:
                    base = {
                        "topic": args.topic,
                        "monitor_topic": args.monitor,
                        "fields": field_paths,
                        "operator": "euclidean_delta",
                        "threshold": threshold,
                        "start_values": monitor.start_values,
                        "end_values": monitor.current_values,
                        "euclidean_distance": monitor.euclidean_distance,
                        "duration": elapsed,
                        "published_count": published_count,
                    }
                elif getattr(args, 'rotate', None) is not None:
                    base = {
                        "topic": args.topic,
                        "monitor_topic": args.monitor,
                        "operator": "rotate",
                        "target_rotation": rotate_angle,
                        "target_rotation_degrees": math.degrees(rotate_angle),
                        "actual_rotation": monitor.rotation_delta,
                        "actual_rotation_degrees": math.degrees(monitor.rotation_delta) if monitor.rotation_delta is not None else None,
                        "duration": elapsed,
                        "published_count": published_count,
                    }
                else:
                    base = {
                        "topic": args.topic,
                        "monitor_topic": args.monitor,
                        "field": field_paths[0],
                        "operator": operator,
                        "threshold": threshold,
                        "start_value": monitor.start_value,
                        "end_value": monitor.current_value,
                        "duration": elapsed,
                        "published_count": published_count,
                    }

                if condition_met:
                    output({**base,
                            "success": True,
                            "condition_met": True,
                            "start_msg": monitor.start_msg,
                            "end_msg": monitor.end_msg})
                else:
                    output({**base,
                            "success": False,
                            "condition_met": False,
                            "error": f"Timeout after {timeout}s: condition not met"})

    except Exception as e:
        output({"error": str(e)})


# ---------------------------------------------------------------------------
# topics hz / find / bw / delay
# ---------------------------------------------------------------------------

def cmd_topics_hz(args):
    """Measure the publish rate of a topic."""
    if not args.topic:
        return output({"error": "topic argument is required"})

    topic = args.topic
    window = args.window
    timeout = args.timeout

    try:
        with ros2_context():
            probe = ROS2CLI()
            topic_types = dict(probe.get_topic_names_and_types())
            probe.destroy_node()

            if topic not in topic_types:
                return output({"error": f"Topic '{topic}' not found in the ROS graph"})
            msg_type = topic_types[topic][0]

            done_event = threading.Event()
            monitor = HzMonitor(topic, msg_type, window, done_event)

            if monitor.sub is None:
                return output({"error": f"Could not subscribe to '{topic}' with type '{msg_type}'"})

            executor = rclpy.executors.SingleThreadedExecutor()
            executor.add_node(monitor)

            def _spin():
                while not done_event.is_set():
                    executor.spin_once(timeout_sec=0.1)

            spin_thread = threading.Thread(target=_spin, daemon=True)
            spin_thread.start()
            done_event.wait(timeout=timeout)
            spin_thread.join(timeout=1.0)

            with monitor.lock:
                timestamps = list(monitor.timestamps)

        if len(timestamps) < 2:
            return output({"error": f"Fewer than 2 messages received within {timeout}s on '{topic}'"})

        deltas = [timestamps[i + 1] - timestamps[i] for i in range(len(timestamps) - 1)]
        mean_delta = sum(deltas) / len(deltas)
        rate = 1.0 / mean_delta if mean_delta > 0 else 0.0
        min_delta = min(deltas)
        max_delta = max(deltas)
        variance = sum((d - mean_delta) ** 2 for d in deltas) / len(deltas)
        std_dev = variance ** 0.5

        output({
            "topic": topic,
            "rate": round(rate, 4),
            "min_delta": round(min_delta, 6),
            "max_delta": round(max_delta, 6),
            "std_dev": round(std_dev, 6),
            "samples": len(deltas),
        })
    except Exception as e:
        output({"error": str(e)})


def cmd_topics_find(args):
    """Find topics publishing a specific message type."""
    import re
    if not args.msg_type:
        return output({"error": "msg_type argument is required"})

    target_raw = args.msg_type

    def _norm_msg(t):
        return re.sub(r'/msg/', '/', t)

    target_norm = _norm_msg(target_raw)

    try:
        with ros2_context():
            node = ROS2CLI()
            all_topics = node.get_topic_names_and_types()

        matched = [
            name for name, types in all_topics
            if any(_norm_msg(t) == target_norm for t in types)
        ]
        output({
            "message_type": target_raw,
            "topics": matched,
            "count": len(matched),
        })
    except Exception as e:
        output({"error": str(e)})


# ---------------------------------------------------------------------------
# Bw / delay monitor nodes
# ---------------------------------------------------------------------------

class BwMonitor(Node):
    """Subscriber that measures bandwidth by serializing each received message."""

    def __init__(self, topic, msg_type, window, done_event):
        super().__init__('bw_monitor')
        self.window = window
        self.done_event = done_event
        self.samples = []
        self.lock = threading.Lock()
        self.sub = None

        msg_class = get_msg_type(msg_type)
        if msg_class:
            self.sub = self.create_subscription(
                msg_class, topic, self._callback, qos_profile_system_default
            )

    def _callback(self, msg):
        with self.lock:
            if self.done_event.is_set():
                return
            try:
                import rclpy.serialization
                size = len(rclpy.serialization.serialize_message(msg))
            except Exception:
                size = 0
            self.samples.append((time.time(), size))
            if len(self.samples) >= self.window:
                self.done_event.set()


def cmd_topics_bw(args):
    """Measure the bandwidth of a topic."""
    if not args.topic:
        return output({"error": "topic argument is required"})

    topic = args.topic
    window = args.window
    timeout = args.timeout

    try:
        with ros2_context():
            probe = ROS2CLI()
            topic_types = dict(probe.get_topic_names_and_types())
            probe.destroy_node()

            if topic not in topic_types:
                return output({"error": f"Topic '{topic}' not found in the ROS graph"})
            msg_type = topic_types[topic][0]

            done_event = threading.Event()
            monitor = BwMonitor(topic, msg_type, window, done_event)

            if monitor.sub is None:
                return output({"error": f"Could not subscribe to '{topic}' with type '{msg_type}'"})

            executor = rclpy.executors.SingleThreadedExecutor()
            executor.add_node(monitor)

            def _spin():
                while not done_event.is_set():
                    executor.spin_once(timeout_sec=0.1)

            spin_thread = threading.Thread(target=_spin, daemon=True)
            spin_thread.start()
            done_event.wait(timeout=timeout)
            spin_thread.join(timeout=1.0)

            with monitor.lock:
                samples = list(monitor.samples)

        if len(samples) < 2:
            return output({"error": f"Fewer than 2 messages received within {timeout}s on '{topic}'"})

        duration = samples[-1][0] - samples[0][0]
        total_bytes = sum(s for _, s in samples)
        bw = total_bytes / duration if duration > 0 else 0.0
        bytes_per_msg = total_bytes / len(samples)
        rate = len(samples) / duration if duration > 0 else 0.0

        output({
            "topic": topic,
            "bw": round(bw, 4),
            "bytes_per_msg": round(bytes_per_msg, 2),
            "rate": round(rate, 4),
            "samples": len(samples),
        })
    except Exception as e:
        output({"error": str(e)})


class DelayMonitor(Node):
    """Subscriber that measures header.stamp to wall-clock latency."""

    def __init__(self, topic, msg_type, window, done_event):
        super().__init__('delay_monitor')
        self.window = window
        self.done_event = done_event
        self.delays = []
        self.lock = threading.Lock()
        self.header_missing = False
        self.sub = None

        msg_class = get_msg_type(msg_type)
        if msg_class:
            self.sub = self.create_subscription(
                msg_class, topic, self._callback, qos_profile_system_default
            )

    def _callback(self, msg):
        with self.lock:
            if self.done_event.is_set():
                return
            wall = time.time()
            if not (hasattr(msg, 'header') and hasattr(msg.header, 'stamp')):
                self.header_missing = True
                self.done_event.set()
                return
            stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            self.delays.append(wall - stamp)
            if len(self.delays) >= self.window:
                self.done_event.set()


def cmd_topics_delay(args):
    """Measure header.stamp to wall-clock latency for a topic."""
    if not args.topic:
        return output({"error": "topic argument is required"})

    topic = args.topic
    window = args.window
    timeout = args.timeout

    try:
        with ros2_context():
            probe = ROS2CLI()
            topic_types = dict(probe.get_topic_names_and_types())
            probe.destroy_node()

            if topic not in topic_types:
                return output({"error": f"Topic '{topic}' not found in the ROS graph"})
            msg_type = topic_types[topic][0]

            done_event = threading.Event()
            monitor = DelayMonitor(topic, msg_type, window, done_event)

            if monitor.sub is None:
                return output({"error": f"Could not subscribe to '{topic}' with type '{msg_type}'"})

            executor = rclpy.executors.SingleThreadedExecutor()
            executor.add_node(monitor)

            def _spin():
                while not done_event.is_set():
                    executor.spin_once(timeout_sec=0.1)

            spin_thread = threading.Thread(target=_spin, daemon=True)
            spin_thread.start()
            done_event.wait(timeout=timeout)
            spin_thread.join(timeout=1.0)

            with monitor.lock:
                delays = list(monitor.delays)
                header_missing = monitor.header_missing

        if header_missing:
            return output({"error": f"Topic '{topic}' messages have no header.stamp field"})
        if not delays:
            return output({"error": f"No messages received within {timeout}s on '{topic}'"})

        mean_delay = sum(delays) / len(delays)
        variance = sum((d - mean_delay) ** 2 for d in delays) / len(delays)
        output({
            "topic": topic,
            "mean_delay": round(mean_delay, 6),
            "min_delay": round(min(delays), 6),
            "max_delay": round(max(delays), 6),
            "std_dev": round(variance ** 0.5, 6),
            "samples": len(delays),
        })
    except Exception as e:
        output({"error": str(e)})


# ---------------------------------------------------------------------------
# topics capture-image
# ---------------------------------------------------------------------------

def cmd_topics_capture_image(args):
    """Capture an image from a ROS 2 image topic and save to .artifacts/ folder."""
    try:
        from sensor_msgs.msg import CompressedImage, Image
        import cv2
        import numpy as np
    except ImportError as e:
        return output({"error": f"Missing dependency for image capture: {e}"})

    topic = args.topic
    output_filename = args.output
    timeout = args.timeout
    img_type = args.type

    out_path = resolve_output_path(output_filename)

    try:
        received = {}

        def cb(msg):
            received['msg'] = msg

        with ros2_context():
            node = rclpy.create_node('image_capture')

            if img_type == 'compressed' or (img_type == 'auto' and topic.endswith('/compressed')):
                node.create_subscription(CompressedImage, topic, cb, 10)
            else:
                node.create_subscription(Image, topic, cb, 10)

            start = time.time()
            while time.time() - start < timeout:
                rclpy.spin_once(node, timeout_sec=0.1)
                if 'msg' in received:
                    break

            node.destroy_node()

        if 'msg' not in received:
            return output({"error": f"No image received from {topic} within {timeout} seconds"})

        msg = received['msg']

        if isinstance(msg, CompressedImage):
            np_arr = np.frombuffer(msg.data, np.uint8)
            img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            cv2.imwrite(out_path, img)
        elif isinstance(msg, Image):
            arr = np.frombuffer(msg.data, dtype=np.uint8)
            arr = arr.reshape((msg.height, msg.width, -1))
            cv2.imwrite(out_path, arr)
        else:
            return output({"error": "Unknown image message type"})

        output({"success": True, "path": out_path})
    except Exception as e:
        output({"error": str(e)})


# ---------------------------------------------------------------------------
# Diagnostics
# ---------------------------------------------------------------------------

def _discover_diag_topics(node):
    """Return [{topic, type}] for every DiagnosticArray topic in the graph."""
    results = []
    for name, types in node.get_topic_names_and_types():
        for t in types:
            if t in DIAG_TYPES:
                results.append({"topic": name, "type": t})
                break
    return sorted(results, key=lambda x: x["topic"])


def _parse_diag_array(msg_dict):
    """Convert a msg_to_dict() DiagnosticArray into a clean status list."""
    status_list = []
    for s in msg_dict.get("status", []):
        level = s.get("level", 0)
        status_list.append({
            "level": level,
            "level_name": _DIAG_LEVEL_NAMES.get(level, str(level)),
            "name": s.get("name", ""),
            "message": s.get("message", ""),
            "hardware_id": s.get("hardware_id", ""),
            "values": [
                {"key": kv.get("key", ""), "value": kv.get("value", "")}
                for kv in s.get("values", [])
            ],
        })
    return status_list


def cmd_topics_diag_list(args):
    """List all topics that publish DiagnosticArray messages (discovered by type)."""
    try:
        with ros2_context():
            node = ROS2CLI()
            topics = _discover_diag_topics(node)
        output({"topics": topics, "count": len(topics)})
    except Exception as e:
        output({"error": str(e)})


def cmd_topics_diag(args):
    """Subscribe to diagnostic topics, auto-discovered by type.

    If --topic is given, use that specific topic.  Otherwise, find every topic
    whose type is DiagnosticArray and subscribe to all of them simultaneously.
    DiagnosticArray messages are decoded into a human-readable status list with
    level_name (OK / WARN / ERROR / STALE), name, message, hardware_id and
    key-value pairs.
    """
    try:
        with ros2_context():
            node = ROS2CLI("diag_temp")

            if args.topic:
                resolved_type = None
                for name, types in node.get_topic_names_and_types():
                    if name == args.topic:
                        resolved_type = types[0] if types else None
                        break
                if resolved_type is None:
                    return output({
                        "error": f"Could not detect type for topic '{args.topic}'",
                        "hint": "Ensure the topic is active. Use 'topics type <topic>' to inspect it.",
                    })
                diag_topics = [{"topic": args.topic, "type": resolved_type}]
            else:
                diag_topics = _discover_diag_topics(node)
                if not diag_topics:
                    return output({
                        "error": "No diagnostic topics found",
                        "hint": (
                            "Ensure diagnostics-publishing nodes are running. "
                            "Diagnostic topics may be named /diagnostics, "
                            "<node>/diagnostics, or any other name — they are "
                            "identified by their DiagnosticArray message type."
                        ),
                    })

            diag_class = None
            for t in ("diagnostic_msgs/msg/DiagnosticArray", "diagnostic_msgs/DiagnosticArray"):
                diag_class = get_msg_type(t)
                if diag_class:
                    break
            if diag_class is None:
                return output({
                    "error": "Could not load diagnostic_msgs/DiagnosticArray",
                    "hint": (
                        "Install the diagnostics package: "
                        "sudo apt install ros-$ROS_DISTRO-diagnostics"
                    ),
                })

            executor = rclpy.executors.SingleThreadedExecutor()
            subscribers = {}
            for t in diag_topics:
                sub = TopicSubscriber(t["topic"], t["type"], msg_class=diag_class)
                subscribers[t["topic"]] = sub
                executor.add_node(sub)

            max_msgs = args.max_messages or 1
            timeout_sec = args.timeout

            if args.duration:
                end_time = time.time() + args.duration
                while time.time() < end_time:
                    executor.spin_once(timeout_sec=0.1)
                results = []
                for topic_name, sub in subscribers.items():
                    with sub.lock:
                        msgs = sub.messages[:max_msgs]
                    for msg_dict in msgs:
                        results.append({
                            "topic": topic_name,
                            "stamp": msg_dict.get("header", {}).get("stamp", {}),
                            "status": _parse_diag_array(msg_dict),
                        })
                output({"results": results, "topic_count": len(diag_topics)})
            else:
                end_time = time.time() + timeout_sec
                while time.time() < end_time:
                    executor.spin_once(timeout_sec=0.1)
                    all_received = all(
                        len(subscribers[t["topic"]].messages) > 0
                        for t in diag_topics
                    )
                    if all_received:
                        break
                results = []
                for t in diag_topics:
                    topic_name = t["topic"]
                    sub = subscribers[topic_name]
                    with sub.lock:
                        msgs = sub.messages[:]
                    if msgs:
                        msg_dict = msgs[0]
                        results.append({
                            "topic": topic_name,
                            "stamp": msg_dict.get("header", {}).get("stamp", {}),
                            "status": _parse_diag_array(msg_dict),
                        })
                    else:
                        results.append({
                            "topic": topic_name,
                            "error": "Timeout — no message received",
                        })
                output({"results": results, "topic_count": len(diag_topics)})
    except Exception as e:
        output({"error": str(e)})


# ---------------------------------------------------------------------------
# Battery monitoring
# ---------------------------------------------------------------------------

def _discover_battery_topics(node):
    """Return [{topic, type}] for every BatteryState topic in the graph."""
    results = []
    for name, types in node.get_topic_names_and_types():
        for t in types:
            if t in BATTERY_TYPES:
                results.append({"topic": name, "type": t})
                break
    return sorted(results, key=lambda x: x["topic"])


def _nan_to_none(v):
    """Return None for float NaN (which is invalid JSON); pass other values through."""
    if isinstance(v, float) and v != v:
        return None
    return v


def _parse_battery_state(msg_dict):
    """Convert a msg_to_dict() BatteryState into a clean summary dict.

    All float fields use NaN to signal 'unmeasured' per the sensor_msgs spec;
    those are converted to null so the output is valid JSON.  percentage is
    scaled from the 0.0–1.0 ROS convention to 0–100 for readability.
    """
    pct = msg_dict.get("percentage", float("nan"))
    status_code = msg_dict.get("power_supply_status", 0)
    health_code = msg_dict.get("power_supply_health", 0)
    tech_code = msg_dict.get("power_supply_technology", 0)

    raw_cell_v = msg_dict.get("cell_voltage", []) or []
    raw_cell_t = msg_dict.get("cell_temperature", []) or []

    return {
        "percentage": (pct * 100) if pct == pct else None,
        "voltage": _nan_to_none(msg_dict.get("voltage")),
        "current": _nan_to_none(msg_dict.get("current")),
        "charge": _nan_to_none(msg_dict.get("charge")),
        "capacity": _nan_to_none(msg_dict.get("capacity")),
        "design_capacity": _nan_to_none(msg_dict.get("design_capacity")),
        "temperature": _nan_to_none(msg_dict.get("temperature")),
        "present": msg_dict.get("present"),
        "power_supply_status": status_code,
        "status_name": _BATTERY_POWER_SUPPLY_STATUS.get(status_code, str(status_code)),
        "power_supply_health": health_code,
        "health_name": _BATTERY_POWER_SUPPLY_HEALTH.get(health_code, str(health_code)),
        "power_supply_technology": tech_code,
        "technology_name": _BATTERY_POWER_SUPPLY_TECHNOLOGY.get(tech_code, str(tech_code)),
        "cell_voltage": [_nan_to_none(v) for v in raw_cell_v],
        "cell_temperature": [_nan_to_none(v) for v in raw_cell_t],
        "location": msg_dict.get("location", ""),
        "serial_number": msg_dict.get("serial_number", ""),
    }


def cmd_topics_battery_list(args):
    """List all topics that publish BatteryState messages (discovered by type)."""
    try:
        with ros2_context():
            node = ROS2CLI()
            topics = _discover_battery_topics(node)
        output({"topics": topics, "count": len(topics)})
    except Exception as e:
        output({"error": str(e)})


def cmd_topics_battery(args):
    """Subscribe to battery topics, auto-discovered by type.

    If --topic is given, use that specific topic.  Otherwise, find every topic
    whose type is BatteryState and subscribe to all of them simultaneously.
    BatteryState messages are decoded into a human-readable summary with
    percentage (0–100), voltage, current, charge, capacity, temperature,
    status_name, health_name, technology_name, location, and serial_number.
    """
    try:
        with ros2_context():
            node = ROS2CLI("battery_temp")

            if args.topic:
                resolved_type = None
                for name, types in node.get_topic_names_and_types():
                    if name == args.topic:
                        resolved_type = types[0] if types else None
                        break
                if resolved_type is None:
                    return output({
                        "error": f"Could not detect type for topic '{args.topic}'",
                        "hint": "Ensure the topic is active. Use 'topics type <topic>' to inspect it.",
                    })
                battery_topics = [{"topic": args.topic, "type": resolved_type}]
            else:
                battery_topics = _discover_battery_topics(node)
                if not battery_topics:
                    return output({
                        "error": "No battery topics found",
                        "hint": (
                            "Ensure battery-publishing nodes are running. "
                            "Battery topics may be named /battery_state, "
                            "<robot>/battery_state, or any other name — they are "
                            "identified by their BatteryState message type."
                        ),
                    })

            battery_class = None
            for t in ("sensor_msgs/msg/BatteryState", "sensor_msgs/BatteryState"):
                battery_class = get_msg_type(t)
                if battery_class:
                    break
            if battery_class is None:
                return output({
                    "error": "Could not load sensor_msgs/BatteryState",
                    "hint": (
                        "Install the sensor_msgs package: "
                        "sudo apt install ros-$ROS_DISTRO-sensor-msgs"
                    ),
                })

            executor = rclpy.executors.SingleThreadedExecutor()
            subscribers = {}
            for t in battery_topics:
                sub = TopicSubscriber(t["topic"], t["type"], msg_class=battery_class)
                subscribers[t["topic"]] = sub
                executor.add_node(sub)

            max_msgs = args.max_messages or 1
            timeout_sec = args.timeout

            if args.duration:
                end_time = time.time() + args.duration
                while time.time() < end_time:
                    executor.spin_once(timeout_sec=0.1)
                results = []
                for topic_name, sub in subscribers.items():
                    with sub.lock:
                        msgs = sub.messages[:max_msgs]
                    for msg_dict in msgs:
                        results.append({
                            "topic": topic_name,
                            "stamp": msg_dict.get("header", {}).get("stamp", {}),
                            "battery": _parse_battery_state(msg_dict),
                        })
                output({"results": results, "topic_count": len(battery_topics)})
            else:
                end_time = time.time() + timeout_sec
                while time.time() < end_time:
                    executor.spin_once(timeout_sec=0.1)
                    all_received = all(
                        len(subscribers[t["topic"]].messages) > 0
                        for t in battery_topics
                    )
                    if all_received:
                        break
                results = []
                for t in battery_topics:
                    topic_name = t["topic"]
                    sub = subscribers[topic_name]
                    with sub.lock:
                        msgs = sub.messages[:]
                    if msgs:
                        msg_dict = msgs[0]
                        results.append({
                            "topic": topic_name,
                            "stamp": msg_dict.get("header", {}).get("stamp", {}),
                            "battery": _parse_battery_state(msg_dict),
                        })
                    else:
                        results.append({
                            "topic": topic_name,
                            "error": "Timeout — no message received",
                        })
                output({"results": results, "topic_count": len(battery_topics)})
    except Exception as e:
        output({"error": str(e)})
