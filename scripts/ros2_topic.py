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
)

# ---------------------------------------------------------------------------
# Velocity message types used to identify velocity command topics.
VELOCITY_TYPES = {
    "geometry_msgs/msg/Twist", "geometry_msgs/msg/TwistStamped",
    "geometry_msgs/Twist", "geometry_msgs/TwistStamped",
}


# ---------------------------------------------------------------------------
# estop
# ---------------------------------------------------------------------------

def cmd_estop(args):
    """Emergency stop for mobile robots - auto-detect and publish zero velocity."""

    def find_velocity_topic(node):
        """Find the velocity command topic by scanning topic types.

        Returns the first topic whose type is Twist or TwistStamped.
        Prefers topics with 'cmd_vel' in the name when multiple candidates exist.
        """
        topics = node.get_topic_names()
        candidates = [
            (name, next(t for t in types if t in VELOCITY_TYPES))
            for name, types in topics
            if any(t in VELOCITY_TYPES for t in types)
        ]
        if not candidates:
            return None, None
        # Prefer cmd_vel-named topics as the most common convention
        preferred = [(name, t) for name, t in candidates if "cmd_vel" in name.lower()]
        return preferred[0] if preferred else candidates[0]

    try:
        rclpy.init()
        node = ROS2CLI("estop")

        if args.topic:
            topic = args.topic
            msg_type = None
            for name, types in node.get_topic_names():
                if name == topic and types:
                    msg_type = types[0]
                    break
            if not msg_type:
                rclpy.shutdown()
                return output({
                    "error": f"Could not detect message type for topic '{topic}'",
                    "hint": "Ensure the topic is active and visible in the ROS graph. Use 'topics type <topic>' to inspect it."
                })
        else:
            topic, msg_type = find_velocity_topic(node)

        if not topic:
            rclpy.shutdown()
            return output({
                "error": "Could not find velocity command topic",
                "hint": "This command is for mobile robots only (not arms). Ensure the robot has a /cmd_vel topic."
            })

        msg_class = get_msg_type(msg_type)
        if not msg_class:
            if args.topic:
                rclpy.shutdown()
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
            rclpy.shutdown()
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
        rclpy.shutdown()
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
        rclpy.init()
        node = ROS2CLI()
        topics = node.get_topic_names()
        topic_list = []
        type_list = []
        for name, types in topics:
            topic_list.append(name)
            type_list.append(types[0] if types else "")
        result = {"topics": topic_list, "types": type_list, "count": len(topic_list)}
        rclpy.shutdown()
        output(result)
    except Exception as e:
        output({"error": str(e)})


def cmd_topics_type(args):
    try:
        rclpy.init()
        node = ROS2CLI()
        topics = node.get_topic_names()
        result = {"topic": args.topic, "type": ""}
        for name, types in topics:
            if name == args.topic:
                result["type"] = types[0] if types else ""
                break
        rclpy.shutdown()
        output(result)
    except Exception as e:
        output({"error": str(e)})


def cmd_topics_details(args):
    try:
        rclpy.init()
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

        rclpy.shutdown()
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
        rclpy.init()
        node = ROS2CLI("temp")

        msg_type = args.msg_type
        if not msg_type:
            topics = node.get_topic_names()
            for name, types in topics:
                if name == args.topic:
                    msg_type = types[0] if types else None
                    break
            if not msg_type:
                rclpy.shutdown()
                return output({"error": f"Could not detect message type for topic: {args.topic}"})

        subscriber = TopicSubscriber(args.topic, msg_type)

        if subscriber.sub is None:
            rclpy.shutdown()
            return output({"error": f"Could not load message type: {msg_type}"})

        if args.duration:
            executor = rclpy.executors.SingleThreadedExecutor()
            executor.add_node(subscriber)
            end_time = time.time() + args.duration
            while time.time() < end_time and len(subscriber.messages) < (args.max_messages or 100):
                executor.spin_once(timeout_sec=0.1)

            with subscriber.lock:
                messages = subscriber.messages[:args.max_messages] if args.max_messages else subscriber.messages

            rclpy.shutdown()
            output({
                "topic": args.topic,
                "collected_count": len(messages),
                "messages": messages
            })
        else:
            executor = rclpy.executors.SingleThreadedExecutor()
            executor.add_node(subscriber)
            timeout_sec = args.timeout
            end_time = time.time() + timeout_sec

            while time.time() < end_time:
                executor.spin_once(timeout_sec=0.1)
                with subscriber.lock:
                    if subscriber.messages:
                        msg = subscriber.messages[0]
                        rclpy.shutdown()
                        output({"msg": msg})
                        return

            rclpy.shutdown()
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

class ConditionMonitor(Node):
    """Subscriber that evaluates a stop condition on every incoming message."""

    def __init__(self, topic, msg_type, field, operator, threshold, stop_event,
                 euclidean=False):
        super().__init__('condition_monitor')
        self.euclidean = euclidean
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
        rclpy.init()

        msg_type = args.msg_type
        topic = args.topic

        temp_node = ROS2CLI("temp")
        for name, types in temp_node.get_topic_names():
            if name == topic and types:
                msg_type = types[0]
                break

        if not msg_type:
            rclpy.shutdown()
            return output({"error": f"Could not detect message type for topic: {topic}. Use --msg-type to specify."})

        msg_class = get_msg_type(msg_type)
        if not msg_class:
            rclpy.shutdown()
            return output(get_msg_error(msg_type))

        publisher = TopicPublisher(topic, msg_type)

        if publisher.pub is None:
            rclpy.shutdown()
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
            rclpy.shutdown()
            output({"success": True, "topic": topic, "msg_type": msg_type,
                    "duration": elapsed, "rate": rate, "published_count": published,
                    "stopped_by": stopped_by})
        else:
            publisher.pub.publish(msg)
            time.sleep(0.1)
            rclpy.shutdown()
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
        rclpy.init()

        msg_type = args.msg_type
        topic = args.topic

        temp_node = ROS2CLI("temp")
        for name, types in temp_node.get_topic_names():
            if name == topic and types:
                msg_type = types[0]
                break

        if not msg_type:
            rclpy.shutdown()
            return output({"error": f"Could not detect message type for topic: {topic}. Use --msg-type to specify."})

        msg_class = get_msg_type(msg_type)
        if not msg_class:
            rclpy.shutdown()
            return output(get_msg_error(msg_type))

        publisher = TopicPublisher(topic, msg_type)

        if publisher.pub is None:
            rclpy.shutdown()
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

        rclpy.shutdown()
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
    if not args.field:
        return output({"error": "--field argument is required"})

    field_paths = args.field
    euclidean = getattr(args, 'euclidean', False)

    if len(field_paths) > 1 and not euclidean:
        return output({"error": "Multiple --field paths require the --euclidean flag"})

    operator_map = {
        'delta': args.delta,
        'above': args.above,
        'below': args.below,
        'equals': args.equals,
    }
    active = {k: v for k, v in operator_map.items() if v is not None}
    if len(active) != 1:
        return output({"error": "Specify exactly one of --delta, --above, --below, --equals"})

    if euclidean and next(iter(active)) != 'delta':
        return output({"error": "--euclidean requires --delta (threshold is the Euclidean distance from start)"})
    operator, threshold = next(iter(active.items()))
    if euclidean and operator == 'delta' and threshold <= 0:
        return output({"error": "--delta must be > 0 when --euclidean is used"})

    try:
        msg_data = json.loads(args.msg)
    except (json.JSONDecodeError, TypeError, ValueError) as e:
        return output({"error": f"Invalid JSON message: {e}"})

    try:
        rclpy.init()

        pub_msg_type = args.msg_type
        mon_msg_type = args.monitor_msg_type

        temp_node = ROS2CLI("temp")
        for name, types in temp_node.get_topic_names():
            if name == args.topic and types and not pub_msg_type:
                pub_msg_type = types[0]
            if name == args.monitor and types and not mon_msg_type:
                mon_msg_type = types[0]

        if not pub_msg_type:
            rclpy.shutdown()
            return output({"error": f"Could not detect message type for topic: {args.topic}. Use --msg-type."})
        if not mon_msg_type:
            rclpy.shutdown()
            return output({"error": f"Could not detect message type for monitor topic: {args.monitor}. Use --monitor-msg-type."})

        pub_msg_class = get_msg_type(pub_msg_type)
        if not pub_msg_class:
            rclpy.shutdown()
            return output(get_msg_error(pub_msg_type))

        stop_event = threading.Event()
        publisher = TopicPublisher(args.topic, pub_msg_type)
        monitor = ConditionMonitor(
            args.monitor, mon_msg_type,
            field_paths if euclidean else field_paths[0],
            operator, threshold, stop_event,
            euclidean=euclidean,
        )

        if publisher.pub is None:
            rclpy.shutdown()
            return output({"error": f"Failed to create publisher for {pub_msg_type}"})
        if monitor.sub is None:
            rclpy.shutdown()
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
            rclpy.shutdown()

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
        rclpy.init()
        probe = ROS2CLI()
        topic_types = dict(probe.get_topic_names_and_types())
        rclpy.shutdown()

        if topic not in topic_types:
            return output({"error": f"Topic '{topic}' not found in the ROS graph"})
        msg_type = topic_types[topic][0]

        rclpy.init()
        done_event = threading.Event()
        monitor = HzMonitor(topic, msg_type, window, done_event)

        if monitor.sub is None:
            rclpy.shutdown()
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

        rclpy.shutdown()

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
        rclpy.init()
        node = ROS2CLI()
        all_topics = node.get_topic_names_and_types()
        rclpy.shutdown()

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
        rclpy.init()
        probe = ROS2CLI()
        topic_types = dict(probe.get_topic_names_and_types())
        rclpy.shutdown()

        if topic not in topic_types:
            return output({"error": f"Topic '{topic}' not found in the ROS graph"})
        msg_type = topic_types[topic][0]

        rclpy.init()
        done_event = threading.Event()
        monitor = BwMonitor(topic, msg_type, window, done_event)

        if monitor.sub is None:
            rclpy.shutdown()
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
        rclpy.shutdown()

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
        rclpy.init()
        probe = ROS2CLI()
        topic_types = dict(probe.get_topic_names_and_types())
        rclpy.shutdown()

        if topic not in topic_types:
            return output({"error": f"Topic '{topic}' not found in the ROS graph"})
        msg_type = topic_types[topic][0]

        rclpy.init()
        done_event = threading.Event()
        monitor = DelayMonitor(topic, msg_type, window, done_event)

        if monitor.sub is None:
            rclpy.shutdown()
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
        rclpy.shutdown()

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
    """Capture an image from a ROS 2 image topic and save to artifacts/ folder."""
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

    artifacts_dir = os.path.join(os.path.dirname(__file__), '..', 'artifacts')
    artifacts_dir = os.path.abspath(artifacts_dir)
    if not os.path.exists(artifacts_dir):
        os.makedirs(artifacts_dir, exist_ok=True)

    try:
        rclpy.init()
        node = rclpy.create_node('image_capture')
        received = {}

        def cb(msg):
            received['msg'] = msg

        if img_type == 'compressed' or (img_type == 'auto' and topic.endswith('/compressed')):
            node.create_subscription(CompressedImage, topic, cb, 10)
        else:
            node.create_subscription(Image, topic, cb, 10)

        executor = rclpy.executors.SingleThreadedExecutor()
        executor.add_node(node)
        start = time.time()
        while time.time() - start < timeout:
            rclpy.spin_once(node, timeout_sec=0.1)
            if 'msg' in received:
                break

        node.destroy_node()
        rclpy.shutdown()

        if 'msg' not in received:
            return output({"error": f"No image received from {topic} within {timeout} seconds"})

        msg = received['msg']
        out_path = os.path.join(artifacts_dir, output_filename)

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
# services echo (topic-level: subscribes to _service_event topic)
# ---------------------------------------------------------------------------

def cmd_services_echo(args):
    """Echo service request/response events via service introspection."""
    if not args.service:
        return output({"error": "service argument is required"})

    service = args.service.rstrip('/')
    event_topic = service + '/_service_event'

    try:
        rclpy.init()
        node = ROS2CLI()
        all_topics = dict(node.get_topic_names_and_types())
        rclpy.shutdown()

        if event_topic not in all_topics:
            return output({
                "error": f"No service event topic found: {event_topic}",
                "hint": (
                    "Service introspection must be enabled on the server/client "
                    "via configure_introspection(clock, qos, "
                    "ServiceIntrospectionState.METADATA or CONTENTS)."
                ),
            })

        msg_type = all_topics[event_topic][0]

        rclpy.init()
        subscriber = TopicSubscriber(event_topic, msg_type,
                                     qos=_get_service_event_qos())

        if subscriber.sub is None:
            rclpy.shutdown()
            return output({"error": f"Could not load event message type: {msg_type}"})

        executor = rclpy.executors.SingleThreadedExecutor()
        executor.add_node(subscriber)

        window = args.duration if args.duration is not None else args.timeout
        max_events = args.max_messages
        end_time = time.time() + window

        try:
            while time.time() < end_time:
                if max_events is not None and len(subscriber.messages) >= max_events:
                    break
                executor.spin_once(timeout_sec=0.1)
        except KeyboardInterrupt:
            pass

        with subscriber.lock:
            events = (subscriber.messages[:max_events]
                      if max_events is not None else subscriber.messages[:])
        rclpy.shutdown()
        output({
            "service": service,
            "event_topic": event_topic,
            "collected_count": len(events),
            "events": events,
        })
    except Exception as e:
        output({"error": str(e)})


# ---------------------------------------------------------------------------
# actions echo (topic-level: subscribes to _action/feedback topic)
# ---------------------------------------------------------------------------

def cmd_actions_echo(args):
    """Echo action feedback and status messages from a live action server."""
    import re
    if not args.action:
        return output({"error": "action argument is required"})

    action = args.action.rstrip('/')
    feedback_topic = action + '/_action/feedback'
    status_topic = action + '/_action/status'

    try:
        rclpy.init()
        node = ROS2CLI()
        all_topics = dict(node.get_topic_names_and_types())
        rclpy.shutdown()

        if feedback_topic not in all_topics:
            return output({"error": f"Action server not found: {action}"})

        feedback_type = all_topics[feedback_topic][0]
        status_type = (all_topics[status_topic][0]
                       if status_topic in all_topics else None)

        from ros2_utils import get_action_type
        action_base_type = re.sub(r'_FeedbackMessage$', '', feedback_type)
        action_class = get_action_type(action_base_type)
        if action_class is None:
            return output({"error": f"Could not load action type: {action_base_type}"})
        fb_msg_class = action_class.Impl.FeedbackMessage

        rclpy.init()
        fb_sub = TopicSubscriber(feedback_topic, feedback_type, msg_class=fb_msg_class)

        if fb_sub.sub is None:
            rclpy.shutdown()
            return output({"error": f"Could not load feedback message type: {feedback_type}"})

        executor = rclpy.executors.SingleThreadedExecutor()
        executor.add_node(fb_sub)

        status_sub = None
        if status_type:
            status_sub = TopicSubscriber(status_topic, status_type)
            executor.add_node(status_sub)

        if args.duration:
            end_time = time.time() + args.duration
            while time.time() < end_time and len(fb_sub.messages) < (args.max_messages or 100):
                executor.spin_once(timeout_sec=0.1)
            with fb_sub.lock:
                feedback_msgs = (fb_sub.messages[:args.max_messages]
                                 if args.max_messages else fb_sub.messages[:])
            status_msgs = []
            if status_sub:
                with status_sub.lock:
                    status_msgs = status_sub.messages[:]
            rclpy.shutdown()
            output({
                "action": action,
                "collected_count": len(feedback_msgs),
                "feedback": feedback_msgs,
                "status": status_msgs,
            })
        else:
            end_time = time.time() + args.timeout
            while time.time() < end_time:
                executor.spin_once(timeout_sec=0.1)
                with fb_sub.lock:
                    if fb_sub.messages:
                        msg = fb_sub.messages[0]
                        rclpy.shutdown()
                        output({"action": action, "feedback": msg})
                        return
            rclpy.shutdown()
            output({"error": "Timeout waiting for action feedback"})
    except Exception as e:
        output({"error": str(e)})
