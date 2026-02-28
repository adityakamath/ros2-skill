#!/usr/bin/env python3
"""ROS 2 Skill - Standalone CLI tool for controlling ROS 2 robots directly via rclpy.

Requires: rclpy (install via ROS 2) or pip install rclpy

Commands:

  version
    Detect ROS 2 version and distro.
    $ python3 ros2_cli.py version

  estop
    Emergency stop for mobile robots. Auto-detects velocity topic and type,
    then publishes zero velocity. For mobile bases only (not arms).
    $ python3 ros2_cli.py estop

  topics list
    List all active topics with their message types.
    $ python3 ros2_cli.py topics list

  topics type <topic>
    Get the message type of a specific topic.
    $ python3 ros2_cli.py topics type /cmd_vel
    $ python3 ros2_cli.py topics type /turtle1/pose

  topics details <topic>
    Get topic details including type, publishers, and subscribers.
    $ python3 ros2_cli.py topics details /cmd_vel

  topics message <message_type>
    Get the field structure of a message type.
    $ python3 ros2_cli.py topics message geometry_msgs/Twist
    $ python3 ros2_cli.py topics message sensor_msgs/LaserScan

  topics subscribe <topic> [--duration SEC] [--max-messages N]
  topics echo      <topic> [--duration SEC] [--max-messages N]
  topics sub       <topic> [--duration SEC] [--max-messages N]
    Subscribe to a topic (echo and sub are aliases for subscribe).
    Without --duration, returns the first message and exits.
    With --duration, collects messages for the specified time.
    --duration SECONDS        Collect messages for this duration (default: single message)
    --max-messages N          Max messages to collect during duration (default: 100)
    --max-msgs N              Alias for --max-messages
    $ python3 ros2_cli.py topics subscribe /turtle1/pose
    $ python3 ros2_cli.py topics echo /odom
    $ python3 ros2_cli.py topics sub /odom --duration 10 --max-msgs 50

  topics publish <topic> <json_message> [--duration SEC] [--rate HZ]
  topics pub      <topic> <json_message> [--duration SEC] [--rate HZ]
    Publish a message to a topic (pub is an alias for publish).
    Without --duration: sends once (single-shot).
    With --duration: publishes repeatedly at --rate Hz for the specified seconds.
    --duration SECONDS   Publish repeatedly for this duration
    --rate HZ            Publish rate (default: 10 Hz)
    $ python3 ros2_cli.py topics publish /turtle1/cmd_vel \
        '{"linear":{"x":2.0,"y":0,"z":0},"angular":{"x":0,"y":0,"z":0}}'
    $ python3 ros2_cli.py topics pub /cmd_vel \
        '{"linear":{"x":1.0,"y":0,"z":0},"angular":{"x":0,"y":0,"z":0}}' --duration 3

  topics publish-sequence <topic> <json_messages> <json_durations> [--rate HZ]
  topics pub-seq          <topic> <json_messages> <json_durations> [--rate HZ]
    Publish a sequence of messages (pub-seq is an alias for publish-sequence).
    Each message is repeatedly published at --rate Hz for its corresponding duration.
    This keeps velocity commands active for the full duration.
    <json_messages>   JSON array of messages to publish in order
    <json_durations>  JSON array of durations (seconds) for each message (must match length)
    --rate HZ         Publish rate (default: 10 Hz)
    $ python3 ros2_cli.py topics publish-sequence /turtle1/cmd_vel \
        '[{"linear":{"x":1.0},"angular":{"z":0}},{"linear":{"x":0},"angular":{"z":0}}]' \
        '[3.0, 0.5]'

  topics publish-until <topic> <json_msg> --monitor <topic> --field <path> (--delta|--above|--below|--equals) <value>
    Publish a message at --rate Hz while monitoring a second topic. Stops as soon as the
    condition on the monitored field is satisfied, or after --timeout seconds (default: 60).
    --monitor TOPIC   Topic to monitor for the stop condition
    --field PATH      Dot-separated field path (e.g. pose.pose.position.x, ranges.0)
    --delta N         Stop when field changes by ±N from its value at start
    --above N         Stop when field > N (absolute threshold)
    --below N         Stop when field < N (absolute threshold)
    --equals V        Stop when field == V (numeric or string)
    --rate HZ         Publish rate (default: 10 Hz)
    --timeout SEC     Safety timeout (default: 60 s)
    $ python3 ros2_cli.py topics publish-until /cmd_vel \
        '{"linear":{"x":0.3},"angular":{"z":0}}' \
        --monitor /odom --field pose.pose.position.x --delta 1.0

  topics publish-continuous <topic> <json_msg> --timeout SEC [--rate HZ]
    Publish at --rate Hz for exactly --timeout seconds (required).
    --timeout is mandatory — never publish to a robot without a hard time limit.
    --rate HZ         Publish rate (default: 10 Hz)
    --timeout SEC     Hard stop after this many seconds (required)
    $ python3 ros2_cli.py topics publish-continuous /cmd_vel \
        '{"linear":{"x":0.3},"angular":{"z":0}}' --timeout 5 --rate 10

  topics hz <topic> [--window N] [--timeout SEC]
    Measure the publish rate of a topic. Collects --window inter-message intervals
    and reports rate, min/max delta, and standard deviation.
    --window N    Number of intervals to sample (default: 10)
    --timeout SEC Max wait time in seconds (default: 10)
    $ python3 ros2_cli.py topics hz /turtle1/pose
    $ python3 ros2_cli.py topics hz /scan --window 20 --timeout 15

  topics find <message_type>
    Find all topics publishing a specific message type.
    Accepts both /msg/ and non-/msg/ formats (normalised for comparison).
    $ python3 ros2_cli.py topics find geometry_msgs/msg/Twist
    $ python3 ros2_cli.py topics find geometry_msgs/Twist

  topics info <topic>
    Alias for topics details (ros2 topic info).
    $ python3 ros2_cli.py topics info /cmd_vel

  services list
    List all available services.
    $ python3 ros2_cli.py services list

  services type <service>
    Get the type of a specific service.
    $ python3 ros2_cli.py services type /reset

  services details <service>
    Get service details including type, request fields, and response fields.
    $ python3 ros2_cli.py services details /spawn

  services call <service> <json_request> [--service-type TYPE]
    Call a service with a JSON request payload. Service type is auto-detected
    from the ROS graph; use --service-type when the service is not yet visible.
    $ python3 ros2_cli.py services call /reset '{}'
    $ python3 ros2_cli.py services call /spawn \
        '{"x":3.0,"y":3.0,"theta":0.0,"name":"turtle2"}'
    $ python3 ros2_cli.py services call /emergency_stop \
        '{"data": true}' --service-type std_srvs/srv/SetBool

  services find <service_type>
    Find all services of a specific type.
    Accepts both /srv/ and non-/srv/ formats (normalised for comparison).
    $ python3 ros2_cli.py services find std_srvs/srv/Empty
    $ python3 ros2_cli.py services find std_srvs/Empty

  services info <service>
    Alias for services details (ros2 service info).
    $ python3 ros2_cli.py services info /spawn

  nodes list
    List all active nodes.
    $ python3 ros2_cli.py nodes list

  nodes details <node>
  nodes info     <node>
    Get node details including publishers, subscribers, services, action servers,
    and action clients. (info is an alias for details, ros2 node info)
    $ python3 ros2_cli.py nodes details /turtlesim
    $ python3 ros2_cli.py nodes info /turtlesim

  params list <node>
    List all parameters for a node.
    $ python3 ros2_cli.py params list /turtlesim

  params get <node:param_name>
  params get <node> <param_name>
    Get a parameter value. Both formats are accepted.
    $ python3 ros2_cli.py params get /turtlesim:background_r
    $ python3 ros2_cli.py params get /turtlesim background_r

  params set <node:param_name> <value>
  params set <node> <param_name> <value>
    Set a parameter value. Both formats are accepted.
    $ python3 ros2_cli.py params set /turtlesim:background_r 255
    $ python3 ros2_cli.py params set /turtlesim background_r 255

  actions list
    List all available action servers.
    $ python3 ros2_cli.py actions list

  actions details <action>
  actions info    <action>
    Get action details including goal, result, and feedback fields.
    (info is an alias for details, ros2 action info)
    $ python3 ros2_cli.py actions details /turtle1/rotate_absolute
    $ python3 ros2_cli.py actions info /turtle1/rotate_absolute

  actions type <action>
    Get the type of an action server.
    $ python3 ros2_cli.py actions type /turtle1/rotate_absolute

  actions send      <action> <json_goal>
  actions send-goal <action> <json_goal>
    Send an action goal and wait for the result.
    (send-goal is an alias for send, ros2 action send_goal)
    $ python3 ros2_cli.py actions send /turtle1/rotate_absolute '{"theta":3.14}'
    $ python3 ros2_cli.py actions send-goal /turtle1/rotate_absolute '{"theta":3.14}'

Output:
    All commands output JSON to stdout.
    Successful results contain relevant data fields.
    Errors contain an "error" field: {"error": "description"}

Examples:
    # Explore the robot system
    python3 ros2_cli.py version
    python3 ros2_cli.py topics list
    python3 ros2_cli.py nodes list
    python3 ros2_cli.py services list

    # Move turtlesim forward 2 sec then stop
    python3 ros2_cli.py topics publish-sequence /turtle1/cmd_vel \
      '[{"linear":{"x":2},"angular":{"z":0}},{"linear":{"x":0},"angular":{"z":0}}]' \
      '[2.0, 0.5]'

    # Read LiDAR data
    python3 ros2_cli.py topics subscribe /scan --duration 3

    # Change turtlesim background to red
    python3 ros2_cli.py params set /turtlesim:background_r 255
    python3 ros2_cli.py params set /turtlesim:background_g 0
    python3 ros2_cli.py params set /turtlesim:background_b 0
"""

import argparse
import importlib
import json
import os
import re
import sys
import time
import threading

try:
    import rclpy
    from rclpy.node import Node
    from rclpy.action import ActionClient
    from rclpy.qos import qos_profile_system_default
    from rcl_interfaces.msg import Parameter, ParameterValue
    from rcl_interfaces.srv import GetParameters, SetParameters, ListParameters
    from rosidl_runtime_py import import_message
except ImportError:
    print(json.dumps({"error": "rclpy not installed. Run: pip install rclpy (or source ROS 2)"}))
    sys.exit(1)


def get_msg_type(type_str):
    if not type_str:
        return None

    # Normalize to pkg, msg_name components
    if '/msg/' in type_str:
        pkg, msg_name = type_str.split('/msg/', 1)
        msg_name = msg_name.strip()
    elif '/' in type_str:
        pkg, msg_name = type_str.rsplit('/', 1)
    elif '.' in type_str:
        # Already dot-separated (e.g. "geometry_msgs.msg.Twist")
        parts = type_str.split('.')
        try:
            module = importlib.import_module('.'.join(parts[:-1]))
            return getattr(module, parts[-1])
        except Exception:
            return None
    else:
        return None

    # Primary: importlib (works whenever the ROS env is sourced)
    try:
        module = importlib.import_module(f"{pkg}.msg")
        return getattr(module, msg_name)
    except Exception:
        pass

    # Fallback: rosidl import_message with slash format
    try:
        return import_message(f"{pkg}/msg/{msg_name}")
    except Exception:
        pass

    # Last resort: rosidl import_message with dot format
    try:
        return import_message(f"{pkg}.msg.{msg_name}")
    except Exception:
        pass

    return None


def get_action_type(type_str):
    """Import a ROS 2 action type class from a type string."""
    if not type_str:
        return None

    if '/action/' in type_str:
        pkg, action_name = type_str.split('/action/', 1)
        action_name = action_name.strip()
    elif '/' in type_str:
        pkg, action_name = type_str.rsplit('/', 1)
    else:
        return None

    # Primary: importlib
    try:
        module = importlib.import_module(f"{pkg}.action")
        return getattr(module, action_name)
    except Exception:
        pass

    # Fallback: import_message with slash format
    try:
        return import_message(f"{pkg}/action/{action_name}")
    except Exception:
        pass

    return None


def get_srv_type(type_str):
    """Import a ROS 2 service type class from a type string."""
    if not type_str:
        return None

    if '/srv/' in type_str:
        pkg, srv_name = type_str.split('/srv/', 1)
        srv_name = srv_name.strip()
    elif '/' in type_str:
        pkg, srv_name = type_str.rsplit('/', 1)
    else:
        return None

    # Primary: importlib
    try:
        module = importlib.import_module(f"{pkg}.srv")
        return getattr(module, srv_name)
    except Exception:
        pass

    # Fallback: import_message with slash format
    try:
        return import_message(f"{pkg}/srv/{srv_name}")
    except Exception:
        pass

    return None


def get_msg_error(msg_type):
    """Generate helpful error message when message type cannot be loaded."""
    ros_distro = os.environ.get('ROS_DISTRO', '')
    suggestions = []
    base_type = msg_type.split('/')[-1] if '/' in msg_type else msg_type
    common_types = {
        'twist': 'geometry_msgs/msg/Twist',
        'twiststamped': 'geometry_msgs/msg/TwistStamped',
        'pose': 'turtlesim/msg/Pose',
        'odom': 'nav_msgs/msg/Odometry',
        'laserscan': 'sensor_msgs/msg/LaserScan',
        'image': 'sensor_msgs/msg/Image',
        'battery': 'sensor_msgs/msg/BatteryState',
        'jointstate': 'sensor_msgs/msg/JointState',
        'imu': 'sensor_msgs/msg/Imu',
    }
    if base_type.lower() in common_types:
        suggestions.append(common_types[base_type.lower()])
    
    hint = "ROS 2 message types use /msg/ format (e.g., geometry_msgs/msg/Twist)"
    if ros_distro:
        hint += f". Ensure ROS 2 workspace is built: cd ~/ros2_ws && colcon build && source install/setup.bash"
    else:
        hint += ". Ensure ROS 2 environment is sourced: source /opt/ros/<distro>/setup.bash"
    
    return {
        "error": f"Unknown message type: {msg_type}",
        "hint": hint,
        "suggestions": suggestions if suggestions else None,
        "ros_distro": ros_distro if ros_distro else None,
        "troubleshooting": [
            "1. Source ROS 2: source /opt/ros/<distro>/setup.bash",
            "2. If using custom messages, build workspace: cd ~/ros2_ws && colcon build",
            "3. Verify: python3 -c 'from geometry_msgs.msg import Twist; print(Twist)'"
        ]
    }


def msg_to_dict(msg):
    result = {}
    for field in msg.get_fields_and_field_types():
        value = getattr(msg, field, None)
        if value is None:
            continue
        if hasattr(value, 'get_fields_and_field_types'):
            result[field] = msg_to_dict(value)
        elif isinstance(value, (bytes, bytearray)):
            result[field] = list(value)
        elif isinstance(value, (list, tuple)):
            result[field] = [
                msg_to_dict(v) if hasattr(v, 'get_fields_and_field_types')
                else v.tolist() if hasattr(v, 'tolist')
                else v
                for v in value
            ]
        elif hasattr(value, 'tolist'):
            # numpy.ndarray (fixed-size array fields like Odometry.covariance[36]) and
            # numpy scalar types — convert to native Python list / number so json.dumps works.
            result[field] = value.tolist()
        else:
            result[field] = value
    return result


def dict_to_msg(msg_type, data):
    msg = msg_type()
    field_types = msg.get_fields_and_field_types()
    for key, value in data.items():
        if not hasattr(msg, key):
            continue
        if isinstance(value, dict):
            setattr(msg, key, dict_to_msg(getattr(msg, key).__class__, value))
        elif isinstance(value, list) and value and isinstance(value[0], dict):
            # Array of nested messages — resolve element type from field type string.
            # rclpy reports these as "sequence<pkg/msg/Type>", "sequence<pkg/msg/Type, N>"
            # (bounded sequence where N is the max size), or "pkg/msg/Type[N]".
            # Use [^,>]+ so bounded sequences don't include ", N" in the captured type.
            field_type_str = field_types.get(key, '')
            m = re.search(r'sequence<([^,>]+)(?:,\s*\d+\s*)?>', field_type_str) or re.search(r'^(.+?)\[\d*\]$', field_type_str)
            if m:
                elem_class = get_msg_type(m.group(1).strip())
                if elem_class:
                    setattr(msg, key, [dict_to_msg(elem_class, v) for v in value])
                else:
                    setattr(msg, key, value)
            else:
                setattr(msg, key, value)
        else:
            setattr(msg, key, value)
    return msg


def resolve_field(d, path):
    """Resolve a dot-separated field path into a nested dict/list structure.

    Integer path segments index into lists, so 'ranges.0' resolves to
    d['ranges'][0].  Raises KeyError, IndexError, or TypeError when any
    segment does not exist.
    """
    current = d
    for part in path.split('.'):
        if isinstance(current, list):
            current = current[int(part)]
        else:
            current = current[part]
    return current


def _json_default(obj):
    """Fallback JSON encoder for types not handled by msg_to_dict (e.g. stray numpy scalars)."""
    if hasattr(obj, 'tolist'):
        return obj.tolist()
    return str(obj)


def output(data):
    print(json.dumps(data, indent=2, ensure_ascii=False, default=_json_default))


class ROS2CLI(Node):
    def __init__(self, node_name='ros2_cli'):
        super().__init__(node_name)

    def get_topic_names(self):
        return self.get_topic_names_and_types()

    def get_service_names(self):
        return self.get_service_names_and_types()


def get_msg_fields(msg_type_str):
    try:
        msg_class = get_msg_type(msg_type_str)
        if msg_class is None:
            return {}
        msg = msg_class()
        return msg_to_dict(msg)
    except Exception as e:
        return {}


def parse_node_param(name):
    if ':' in name:
        parts = name.split(':', 1)
        return parts[0], parts[1]
    return name, None


def cmd_version(args):
    # ROS_DOMAIN_ID is a plain env var; rclpy.utilities has no get_domain_id() API.
    domain_id = int(os.environ.get('ROS_DOMAIN_ID', 0))
    distro = os.environ.get('ROS_DISTRO', 'unknown')
    output({"version": "2", "distro": distro, "domain_id": domain_id})


VELOCITY_TOPICS = ["/cmd_vel", "/cmd_vel_nav", "/cmd_vel_raw", "/mobile_base/commands/velocity"]
VELOCITY_TYPES = [
    "geometry_msgs/msg/Twist", "geometry_msgs/msg/TwistStamped",  # ROS 2 /msg/ format
    "geometry_msgs/Twist", "geometry_msgs/TwistStamped",           # legacy format fallback
]


def find_velocity_topic(node):
    """Find the velocity command topic and its type."""
    topics = node.get_topic_names()
    for topic_name, types in topics:
        if topic_name in VELOCITY_TOPICS:
            for msg_type in types:
                if msg_type in VELOCITY_TYPES:
                    return topic_name, msg_type
    for topic_name, types in topics:
        if "cmd_vel" in topic_name.lower():
            return topic_name, types[0] if types else None
    return None, None


def cmd_estop(args):
    """Emergency stop for mobile robots - auto-detect and publish zero velocity.

    This command is designed for mobile base robots (differential drive, omnidirectional, etc.)
    that use velocity commands for movement. It will NOT work for robotic arms or manipulators.

    The command:
    1. Searches for common velocity topics (/cmd_vel, /cmd_vel_nav, etc.)
    2. Auto-detects message type (Twist or TwistStamped)
    3. Publishes zero velocity to stop all movement
    4. Only works for mobile robots with velocity-based control

    Not applicable for: robot arms, grippers, or position-controlled robots.
    """
    try:
        rclpy.init()
        node = ROS2CLI("estop")

        if args.topic:
            # Custom topic specified — detect its type from the graph only; no fallback
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
                # Custom topic — do not guess a different type; report the failure clearly.
                rclpy.shutdown()
                return output({
                    "error": f"Could not load message type '{msg_type}' for topic '{topic}'",
                    "hint": "Ensure the ROS workspace is built and sourced: cd ~/ros2_ws && colcon build && source install/setup.bash"
                })
            else:
                # Auto-detected velocity topic — try known types as fallback.
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


class TopicSubscriber(Node):
    def __init__(self, topic, msg_type):
        super().__init__('subscriber')
        self.msg_type = msg_type
        self.messages = []
        self.lock = threading.Lock()
        self.sub = None

        msg_class = get_msg_type(msg_type)
        if msg_class:
            self.sub = self.create_subscription(
                msg_class, topic, self.callback, qos_profile_system_default
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


class TopicPublisher(Node):
    def __init__(self, topic, msg_type):
        super().__init__('publisher')
        self.topic = topic
        self.msg_type = msg_type
        self.pub = None

        msg_class = get_msg_type(msg_type)
        if msg_class:
            self.pub = self.create_publisher(msg_class, topic, qos_profile_system_default)


class ConditionMonitor(Node):
    """Subscriber that evaluates a stop condition on every incoming message.

    Sets *stop_event* as soon as the condition fires.  The main publish loop
    polls *stop_event* and exits when it is set.

    Attributes (read after stop_event is set):
        start_value   -- field value from the very first message
        current_value -- field value from the most recent message
        start_msg     -- full msg dict from the first message
        end_msg       -- full msg dict from the message that triggered the stop
        field_error   -- non-None string when the field path could not be resolved
    """

    def __init__(self, topic, msg_type, field, operator, threshold, stop_event):
        super().__init__('condition_monitor')
        self.field = field
        self.operator = operator      # 'delta' | 'above' | 'below' | 'equals'
        self.threshold = threshold
        self.stop_event = stop_event

        self.start_value = None
        self.current_value = None
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

            # Resolve the field path; report error and stop on first failure.
            try:
                value = resolve_field(msg_dict, self.field)
            except (KeyError, IndexError, TypeError, ValueError) as e:
                self.field_error = f"Field '{self.field}' not found in monitor message: {e}"
                self.stop_event.set()
                return

            # Try to coerce to float for numeric comparisons.
            try:
                numeric_value = float(value)
            except (TypeError, ValueError):
                numeric_value = None

            # Capture start state on the very first message.
            if self.start_value is None:
                self.start_value = value
                self.start_msg = msg_dict

            self.current_value = value

            # Evaluate condition.
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


class HzMonitor(Node):
    """Subscriber that measures the publish rate of a topic.

    Collects message arrival timestamps until *window + 1* samples have been
    received (so that *window* inter-message deltas are available), then sets
    *done_event*.  All timestamps are stored in *timestamps* (protected by
    *lock*) and are read by the main thread after *done_event* fires.
    """

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
            # window+1 timestamps → window inter-message deltas
            if len(self.timestamps) >= self.window + 1:
                self.done_event.set()


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

        # Always query the graph so we use the type the topic actually expects.
        # Falls back to --msg-type only when the topic isn't visible yet.
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
        duration = getattr(args, "duration", None)
        interval = 1.0 / rate

        if duration and duration > 0:
            published = 0
            end_time = time.time() + duration
            while time.time() < end_time:
                publisher.pub.publish(msg)
                published += 1
                remaining = end_time - time.time()
                if remaining > 0:
                    time.sleep(min(interval, remaining))
            rclpy.shutdown()
            output({"success": True, "topic": topic, "msg_type": msg_type,
                    "duration": duration, "rate": rate, "published_count": published})
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

        # Always query the graph so we use the type the topic actually expects.
        # Falls back to --msg-type only when the topic isn't visible yet.
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

    # Exactly one condition operator must be supplied.
    operator_map = {
        'delta': args.delta,
        'above': args.above,
        'below': args.below,
        'equals': args.equals,
    }
    active = {k: v for k, v in operator_map.items() if v is not None}
    if len(active) != 1:
        return output({"error": "Specify exactly one of --delta, --above, --below, --equals"})
    operator, threshold = next(iter(active.items()))

    try:
        msg_data = json.loads(args.msg)
    except (json.JSONDecodeError, TypeError, ValueError) as e:
        return output({"error": f"Invalid JSON message: {e}"})

    try:
        rclpy.init()

        pub_msg_type = args.msg_type
        mon_msg_type = args.monitor_msg_type

        # Resolve both topic types from the graph in one pass.
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
            args.monitor, mon_msg_type, args.field, operator, threshold, stop_event
        )

        if publisher.pub is None:
            rclpy.shutdown()
            return output({"error": f"Failed to create publisher for {pub_msg_type}"})
        if monitor.sub is None:
            rclpy.shutdown()
            return output({"error": f"Could not load monitor message type: {mon_msg_type}"})

        msg = dict_to_msg(pub_msg_class, msg_data)

        executor = rclpy.executors.MultiThreadedExecutor()
        executor.add_node(publisher)
        executor.add_node(monitor)

        spin_thread = threading.Thread(target=executor.spin, daemon=True)
        spin_thread.start()

        rate = args.rate or 10.0
        timeout = args.timeout  # default 60 from parser
        interval = 1.0 / rate
        start_time = time.time()
        published_count = 0

        try:
            while not stop_event.is_set():
                if timeout and (time.time() - start_time) >= timeout:
                    # Timeout reached — break WITHOUT setting stop_event so we
                    # can distinguish timeout from condition-met after the loop.
                    break
                publisher.pub.publish(msg)
                published_count += 1
                time.sleep(interval)
        finally:
            stop_event.set()  # unblock monitor if still waiting
            executor.shutdown(wait=False)
            spin_thread.join(timeout=2.0)
            rclpy.shutdown()

        elapsed = round(time.time() - start_time, 3)

        with monitor.lock:
            if monitor.field_error:
                return output({"error": monitor.field_error})

            condition_met = monitor.end_msg is not None

            base = {
                "topic": args.topic,
                "monitor_topic": args.monitor,
                "field": args.field,
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


def cmd_topics_publish_continuous(args):
    """Publish to a topic at a fixed rate until --timeout elapses (required).

    --timeout is mandatory: never publish to a robot topic without a hard
    time limit, especially when invoked from an AI agent that cannot send
    Ctrl+C.
    """
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
            return output({"error": f"Could not detect message type for topic: {topic}. Use --msg-type."})

        msg_class = get_msg_type(msg_type)
        if not msg_class:
            rclpy.shutdown()
            return output(get_msg_error(msg_type))

        publisher = TopicPublisher(topic, msg_type)
        if publisher.pub is None:
            rclpy.shutdown()
            return output({"error": f"Failed to create publisher for {msg_type}"})

        msg = dict_to_msg(msg_class, msg_data)

        rate = args.rate or 10.0
        timeout = args.timeout  # required by parser — always set
        interval = 1.0 / rate
        start_time = time.time()
        published_count = 0
        stopped_by = "timeout"

        try:
            while (time.time() - start_time) < timeout:
                publisher.pub.publish(msg)
                published_count += 1
                time.sleep(interval)
        except KeyboardInterrupt:
            stopped_by = "keyboard_interrupt"

        elapsed = round(time.time() - start_time, 3)
        rclpy.shutdown()
        output({
            "success": True,
            "topic": topic,
            "msg_type": msg_type,
            "published_count": published_count,
            "duration": elapsed,
            "rate": rate,
            "stopped_by": stopped_by,
        })

    except Exception as e:
        output({"error": str(e)})


def cmd_services_list(args):
    try:
        rclpy.init()
        node = ROS2CLI()
        services = node.get_service_names()
        service_list = []
        type_list = []
        for name, types in services:
            service_list.append(name)
            type_list.append(types[0] if types else "")
        result = {"services": service_list, "types": type_list, "count": len(service_list)}
        rclpy.shutdown()
        output(result)
    except Exception as e:
        output({"error": str(e)})


def cmd_services_type(args):
    try:
        rclpy.init()
        node = ROS2CLI()
        services = node.get_service_names()
        result = {"service": args.service, "type": ""}
        for name, types in services:
            if name == args.service:
                result["type"] = types[0] if types else ""
                break
        rclpy.shutdown()
        output(result)
    except Exception as e:
        output({"error": str(e)})


def cmd_services_details(args):
    try:
        rclpy.init()
        node = ROS2CLI()
        services = node.get_service_names()
        
        result = {"service": args.service, "type": "", "request": {}, "response": {}}
        
        for name, types in services:
            if name == args.service:
                result["type"] = types[0] if types else ""
                break
        
        if result["type"]:
            srv_class = get_srv_type(result["type"])
            if srv_class:
                try:
                    result["request"] = msg_to_dict(srv_class.Request())
                except Exception:
                    pass
                try:
                    result["response"] = msg_to_dict(srv_class.Response())
                except Exception:
                    pass
        
        rclpy.shutdown()
        output(result)
    except Exception as e:
        output({"error": str(e)})


def cmd_services_call(args):
    # Detect which of the two supported call formats was used:
    #
    #   New (flag) form:      services call /svc '{"data":true}' --service-type pkg/srv/T
    #     args.extra_request is None  → request_json = args.request
    #                                    service_type_override = None (fall through to --service-type flag)
    #
    #   Old (positional) form: services call /svc pkg/srv/T '{"data":true}'
    #     args.extra_request is set  → args.request holds the service type string
    #                                    args.extra_request holds the JSON payload
    #                                    service_type_override = args.request
    #                                    request_json = args.extra_request
    if getattr(args, 'extra_request', None) is not None:
        service_type_override = args.request
        request_json = args.extra_request
    else:
        service_type_override = None
        request_json = args.request

    try:
        request_data = json.loads(request_json)
    except (json.JSONDecodeError, TypeError) as e:
        return output({"error": f"Invalid JSON request: {e}"})

    try:
        rclpy.init()
        node = ROS2CLI()

        service_type = service_type_override or args.service_type
        if not service_type:
            services = node.get_service_names()
            for name, types in services:
                if name == args.service:
                    service_type = types[0] if types else ""
                    break

        if not service_type:
            rclpy.shutdown()
            return output({
                "error": f"Service not found: {args.service}",
                "hint": "Use --service-type to specify the type explicitly (e.g. --service-type std_srvs/srv/SetBool)"
            })
        
        srv_class = get_srv_type(service_type)
        if not srv_class:
            rclpy.shutdown()
            return output({"error": f"Cannot load service type: {service_type}"})

        client = node.create_client(srv_class, args.service)

        if not client.wait_for_service(timeout_sec=args.timeout):
            rclpy.shutdown()
            return output({"error": f"Service not available: {args.service}"})

        request = dict_to_msg(srv_class.Request, request_data)
        future = client.call_async(request)

        timeout = args.timeout
        end_time = time.time() + timeout

        while time.time() < end_time and not future.done():
            rclpy.spin_once(node, timeout_sec=0.1)

        if future.done():
            result_msg = future.result()
            result_dict = msg_to_dict(result_msg)
            rclpy.shutdown()
            output({"service": args.service, "success": True, "result": result_dict})
        else:
            rclpy.shutdown()
            output({"service": args.service, "success": False, "error": "Service call timeout"})
    except Exception as e:
        output({"error": str(e)})


def cmd_nodes_list(args):
    try:
        rclpy.init()
        node = ROS2CLI()
        node_info = node.get_node_names_and_namespaces()
        names = [f"{ns.rstrip('/')}/{n}" for n, ns in node_info]
        result = {"nodes": names, "count": len(names)}
        rclpy.shutdown()
        output(result)
    except Exception as e:
        output({"error": str(e)})


def _split_node_path(full_name):
    """Split '/namespace/node_name' into (node_name, namespace)."""
    s = full_name.lstrip('/')
    if '/' in s:
        idx = s.rindex('/')
        return s[idx + 1:], '/' + s[:idx]
    return s, '/'


def cmd_nodes_details(args):
    try:
        rclpy.init()
        node = ROS2CLI()

        node_name, namespace = _split_node_path(args.node)

        publishers = node.get_publisher_names_and_types_by_node(node_name, namespace)
        subscribers = node.get_subscriber_names_and_types_by_node(node_name, namespace)
        services = node.get_service_names_and_types_by_node(node_name, namespace)

        result = {
            "node": args.node,
            "publishers": [topic for topic, _ in publishers],
            "subscribers": [topic for topic, _ in subscribers],
            "services": [svc for svc, _ in services],
        }

        try:
            action_servers = [
                name for name, _ in
                node.get_action_server_names_and_types_by_node(node_name, namespace)
            ]
            action_clients = [
                name for name, _ in
                node.get_action_client_names_and_types_by_node(node_name, namespace)
            ]
            result["action_servers"] = action_servers
            result["action_clients"] = action_clients
        except AttributeError:
            result["action_servers"] = []
            result["action_clients"] = []

        rclpy.shutdown()
        output(result)
    except Exception as e:
        output({"error": str(e)})


def cmd_params_list(args):
    try:
        rclpy.init()
        node = ROS2CLI()
        
        node_name, param_name = parse_node_param(args.node)
        if not node_name.startswith('/'):
            node_name = '/' + node_name
        
        service_name = f"{node_name}/list_parameters"
        
        client = node.create_client(ListParameters, service_name)
        
        if not client.wait_for_service(timeout_sec=args.timeout):
            rclpy.shutdown()
            return output({"error": f"Parameter service not available for {node_name}"})

        request = ListParameters.Request()
        future = client.call_async(request)

        end_time = time.time() + args.timeout
        while time.time() < end_time and not future.done():
            rclpy.spin_once(node, timeout_sec=0.1)

        if future.done():
            result = future.result()
            names = result.result.names if result.result else []
            formatted = [f"{node_name}:{n}" for n in names]
            rclpy.shutdown()
            output({"node": node_name, "parameters": formatted, "count": len(formatted)})
        else:
            rclpy.shutdown()
            output({"error": "Timeout listing parameters"})
    except Exception as e:
        output({"error": str(e)})


def cmd_params_get(args):
    # Accept both /node:param and "/node param_name" (two space-separated args)
    if getattr(args, 'param_name', None):
        full_name = args.name.rstrip(':') + ':' + args.param_name
    else:
        full_name = args.name
    if ':' not in full_name or not full_name.split(':', 1)[1]:
        return output({"error": "Use format /node_name:param_name or /node_name param_name (e.g. /turtlesim background_r)"})

    try:
        rclpy.init()
        node = ROS2CLI()

        node_name, param_name = full_name.split(':', 1)
        
        if not node_name.startswith('/'):
            node_name = '/' + node_name
        
        service_name = f"{node_name}/get_parameters"
        
        client = node.create_client(GetParameters, service_name)
        
        if not client.wait_for_service(timeout_sec=args.timeout):
            rclpy.shutdown()
            return output({"error": f"Parameter service not available for {node_name}"})

        request = GetParameters.Request()
        request.names = [param_name] if param_name else []

        future = client.call_async(request)

        end_time = time.time() + args.timeout
        while time.time() < end_time and not future.done():
            rclpy.spin_once(node, timeout_sec=0.1)
        
        if future.done():
            result = future.result()
            values = result.values if result.values else []
            value_str = ""
            exists = False
            if values:
                v = values[0]
                if v.type == 1:
                    value_str = str(v.bool_value)
                    exists = True
                elif v.type == 2:
                    value_str = str(v.integer_value)
                    exists = True
                elif v.type == 3:
                    value_str = str(v.double_value)
                    exists = True
                elif v.type == 4:
                    value_str = v.string_value
                    exists = True
                elif v.type in [5, 6, 7, 8, 9]:
                    value_str = str(v)
                    exists = True
            rclpy.shutdown()
            output({"name": full_name, "value": value_str, "exists": exists})
        else:
            rclpy.shutdown()
            output({"error": "Timeout getting parameter"})
    except Exception as e:
        output({"error": str(e)})


def cmd_params_set(args):
    # Accept both "/node:param value" and "/node param_name value" (three space-separated args)
    if getattr(args, 'extra_value', None) is not None:
        full_name = args.name.rstrip(':') + ':' + args.value
        value_str = args.extra_value
    else:
        full_name = args.name
        value_str = args.value
    if ':' not in full_name or not full_name.split(':', 1)[1]:
        return output({"error": "Use format /node_name:param_name value or /node_name param_name value (e.g. /turtlesim background_r 255)"})

    try:
        rclpy.init()
        node = ROS2CLI()

        node_name, param_name = full_name.split(':', 1)

        if not node_name.startswith('/'):
            node_name = '/' + node_name

        service_name = f"{node_name}/set_parameters"

        client = node.create_client(SetParameters, service_name)

        if not client.wait_for_service(timeout_sec=args.timeout):
            rclpy.shutdown()
            return output({"error": f"Parameter service not available for {node_name}"})

        request = SetParameters.Request()

        param = Parameter()
        param.name = param_name

        pv = ParameterValue()
        try:
            if value_str.lower() in ('true', 'false'):
                pv.type = 1
                pv.bool_value = value_str.lower() == 'true'
            elif '.' in value_str:
                pv.type = 3
                pv.double_value = float(value_str)
            else:
                try:
                    pv.type = 2
                    pv.integer_value = int(value_str)
                except Exception:
                    pv.type = 4
                    pv.string_value = value_str
        except Exception:
            pv.type = 4
            pv.string_value = value_str

        param.value = pv
        request.parameters = [param]

        future = client.call_async(request)

        end_time = time.time() + args.timeout
        while time.time() < end_time and not future.done():
            rclpy.spin_once(node, timeout_sec=0.1)

        rclpy.shutdown()
        if future.done():
            result = future.result()
            if result.results and result.results[0].successful:
                output({"name": full_name, "value": value_str, "success": True})
            else:
                reason = result.results[0].reason if result.results else ""
                reason_lc = reason.lower()
                if re.search(r'\b(read[- ]?only|readonly)\b', reason_lc):
                    output({"name": full_name, "value": value_str, "success": False,
                            "error": "Parameter is read-only and cannot be changed at runtime",
                            "read_only": True})
                else:
                    output({"name": full_name, "value": value_str, "success": False,
                            "error": reason or "Parameter rejected by node"})
        else:
            output({"error": "Timeout setting parameter"})
    except Exception as e:
        output({"error": str(e)})


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
        
        # In ROS 2, action topics use the /_action/ prefix.
        # The feedback topic type encodes the action package and name.
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
        rclpy.init()
        node = ROS2CLI()
        
        topics = node.get_topic_names_and_types()
        
        # In ROS 2, action topics use the /_action/ prefix.
        # The feedback topic type encodes the action package and name.
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
        
        if not client.wait_for_server(timeout_sec=args.timeout):
            rclpy.shutdown()
            return output({"error": f"Action server not available: {args.action}"})
        
        goal_msg = dict_to_msg(action_class.Goal, goal_data)
        
        goal_id = f"goal_{int(time.time() * 1000)}"
        
        future = client.send_goal_async(goal_msg)

        timeout = args.timeout
        end_time = time.time() + timeout
        
        while time.time() < end_time and not future.done():
            rclpy.spin_once(node, timeout_sec=0.1)
        
        if not future.done():
            rclpy.shutdown()
            output({"action": args.action, "success": False, "error": "Timeout waiting for goal acceptance"})
            return
        
        goal_handle = future.result()
        
        if not goal_handle.accepted:
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
            output({"action": args.action, "success": True,
                    "goal_id": goal_id, "result": result_dict})
        else:
            rclpy.shutdown()
            output({"action": args.action, "success": False, "error": f"Timeout after {timeout}s"})
    except Exception as e:
        output({"error": str(e)})


def cmd_topics_hz(args):
    """Measure the publish rate of a topic."""
    if not args.topic:
        return output({"error": "topic argument is required"})

    topic = args.topic
    window = args.window
    timeout = args.timeout

    try:
        rclpy.init()
        # Auto-detect message type from the ROS graph.
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
    if not args.msg_type:
        return output({"error": "msg_type argument is required"})

    target_raw = args.msg_type

    # Normalise: geometry_msgs/Twist and geometry_msgs/msg/Twist are equivalent.
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
                # e.g. turtlesim/action/RotateAbsolute_FeedbackMessage
                action_type = re.sub(r'_FeedbackMessage$', '', raw)
                break

        if action_type is None:
            return output({"error": f"Action '{action}' not found in the ROS graph"})

        output({"action": action, "type": action_type})
    except Exception as e:
        output({"error": str(e)})


def cmd_services_find(args):
    """Find services of a specific service type."""
    if not args.service_type:
        return output({"error": "service_type argument is required"})

    target_raw = args.service_type

    # Normalise: std_srvs/Empty and std_srvs/srv/Empty are equivalent.
    def _norm_srv(t):
        return re.sub(r'/srv/', '/', t)

    target_norm = _norm_srv(target_raw)

    try:
        rclpy.init()
        node = ROS2CLI()
        all_services = node.get_service_names_and_types()
        rclpy.shutdown()

        matched = [
            name for name, types in all_services
            if any(_norm_srv(t) == target_norm for t in types)
        ]
        output({
            "service_type": target_raw,
            "services": matched,
            "count": len(matched),
        })
    except Exception as e:
        output({"error": str(e)})


def _add_subscribe_args(p):
    """Add the shared arguments for the subscribe / echo subparsers."""
    p.add_argument("topic", nargs="?")
    p.add_argument("--msg-type", dest="msg_type", default=None,
                   help="Message type (auto-detected if not provided)")
    p.add_argument("--duration", type=float, default=None,
                   help="Subscribe for duration (seconds)")
    p.add_argument("--max-messages", "--max-msgs", dest="max_messages", type=int, default=100,
                   help="Max messages to collect")
    p.add_argument("--timeout", type=float, default=5.0,
                   help="Timeout (seconds) when waiting for a single message (default: 5)")


def build_parser():
    parser = argparse.ArgumentParser(description="ROS 2 Skill CLI - Control ROS 2 robots directly via rclpy")
    sub = parser.add_subparsers(dest="command")

    sub.add_parser("version", help="Detect ROS 2 version")

    estop = sub.add_parser("estop", help="Emergency stop for mobile robots (publishes zero velocity)")
    estop.add_argument("--topic", dest="topic", default=None, help="Custom velocity topic (default: auto-detect)")

    topics = sub.add_parser("topics", help="Topic operations")
    tsub = topics.add_subparsers(dest="subcommand")
    tsub.add_parser("list", help="List all topics")
    p = tsub.add_parser("type", help="Get topic message type")
    p.add_argument("topic")
    p = tsub.add_parser("details", help="Get topic details")
    p.add_argument("topic")
    p = tsub.add_parser("message", help="Get message structure")
    p.add_argument("message_type")
    _add_subscribe_args(tsub.add_parser("subscribe", help="Subscribe to a topic"))
    _add_subscribe_args(tsub.add_parser("echo", help="Echo topic messages (alias for subscribe)"))
    _add_subscribe_args(tsub.add_parser("sub", help="Alias for subscribe"))
    p = tsub.add_parser("info", help="Alias for details (ros2 topic info)")
    p.add_argument("topic")
    p = tsub.add_parser("hz", help="Measure topic publish rate")
    p.add_argument("topic", nargs="?")
    p.add_argument("--window", type=int, default=10,
                   help="Number of inter-message intervals to sample (default: 10)")
    p.add_argument("--timeout", type=float, default=10.0,
                   help="Max wait time in seconds (default: 10)")
    p = tsub.add_parser("find", help="Find topics by message type")
    p.add_argument("msg_type", nargs="?")
    p.add_argument("--timeout", type=float, default=5.0,
                   help="Timeout in seconds (default: 5)")
    for _pub_name in ("publish", "pub"):
        p = tsub.add_parser(_pub_name, help="Publish a message" if _pub_name == "publish"
                            else "Alias for publish")
        p.add_argument("topic", nargs="?")
        p.add_argument("msg", nargs="?", help="JSON message")
        p.add_argument("--msg-type", dest="msg_type", default=None,
                       help="Message type (auto-detected if not provided)")
        p.add_argument("--duration", type=float, default=None,
                       help="Publish repeatedly for duration (seconds)")
        p.add_argument("--rate", type=float, default=10.0, help="Publish rate in Hz (default: 10)")
    for _seq_name in ("publish-sequence", "pub-seq"):
        p = tsub.add_parser(_seq_name, help="Publish message sequence with delays"
                            if _seq_name == "publish-sequence" else "Alias for publish-sequence")
        p.add_argument("topic", nargs="?")
        p.add_argument("messages", nargs="?", help="JSON array of messages")
        p.add_argument("durations", nargs="?",
                       help="JSON array of durations in seconds (message is repeated during each)")
        p.add_argument("--msg-type", dest="msg_type", default=None,
                       help="Message type (auto-detected if not provided)")
        p.add_argument("--rate", type=float, default=10.0, help="Publish rate in Hz (default: 10)")
    p = tsub.add_parser("publish-until", help="Publish until a monitor-topic condition is met")
    p.add_argument("topic", nargs="?")
    p.add_argument("msg", nargs="?", help="JSON message to publish")
    p.add_argument("--monitor", default=None, help="Topic to monitor for the stop condition (required)")
    p.add_argument("--field", default=None, help="Dot-separated field path in monitor message (e.g. pose.pose.position.x)")
    p.add_argument("--delta", type=float, default=None, help="Stop when field changes by ±N from its value at start")
    p.add_argument("--above", type=float, default=None, help="Stop when field > N (absolute threshold)")
    p.add_argument("--below", type=float, default=None, help="Stop when field < N (absolute threshold)")
    p.add_argument("--equals", default=None, help="Stop when field == value (numeric or string)")
    p.add_argument("--rate", type=float, default=10.0, help="Publish rate in Hz (default: 10)")
    p.add_argument("--timeout", type=float, default=60.0, help="Safety timeout in seconds (default: 60)")
    p.add_argument("--msg-type", dest="msg_type", default=None, help="Publish topic message type (auto-detected)")
    p.add_argument("--monitor-msg-type", dest="monitor_msg_type", default=None, help="Monitor topic message type (auto-detected)")
    p = tsub.add_parser("publish-continuous", help="Publish at a fixed rate for a required duration (--timeout)")
    p.add_argument("topic", nargs="?")
    p.add_argument("msg", nargs="?", help="JSON message to publish")
    p.add_argument("--rate", type=float, default=10.0, help="Publish rate in Hz (default: 10)")
    p.add_argument("--timeout", type=float, required=True,
                   help="Hard stop after this many seconds (required — never publish without a time limit)")
    p.add_argument("--msg-type", dest="msg_type", default=None, help="Message type (auto-detected if not provided)")

    services = sub.add_parser("services", help="Service operations")
    ssub = services.add_subparsers(dest="subcommand")
    ssub.add_parser("list", help="List all services")
    p = ssub.add_parser("type", help="Get service type")
    p.add_argument("service")
    p = ssub.add_parser("details", help="Get service details")
    p.add_argument("service")
    p = ssub.add_parser("info", help="Alias for details (ros2 service info)")
    p.add_argument("service")
    p = ssub.add_parser("find", help="Find services by service type")
    p.add_argument("service_type", nargs="?")
    p.add_argument("--timeout", type=float, default=5.0,
                   help="Timeout in seconds (default: 5)")
    p = ssub.add_parser("call", help="Call a service")
    p.add_argument("service")
    p.add_argument("request", help="JSON request, or service type when using positional service-type format")
    p.add_argument("extra_request", nargs="?", default=None,
                   help="JSON request when using /svc service_type json positional format")
    p.add_argument("--service-type", dest="service_type", default=None,
                   help="Service type (auto-detected if not provided, e.g. std_srvs/srv/SetBool)")
    p.add_argument("--timeout", type=float, default=5.0, help="Timeout in seconds (default: 5)")

    nodes = sub.add_parser("nodes", help="Node operations")
    nsub = nodes.add_subparsers(dest="subcommand")
    nsub.add_parser("list", help="List all nodes")
    p = nsub.add_parser("details", help="Get node details")
    p.add_argument("node")
    p = nsub.add_parser("info", help="Alias for details (ros2 node info)")
    p.add_argument("node")

    params = sub.add_parser("params", help="Parameter operations")
    psub = params.add_subparsers(dest="subcommand")
    p = psub.add_parser("list", help="List parameters for a node")
    p.add_argument("node")
    p.add_argument("--timeout", type=float, default=5.0, help="Timeout in seconds (default: 5)")
    p = psub.add_parser("get", help="Get parameter value")
    p.add_argument("name", help="/node_name:param_name or just /node_name")
    p.add_argument("param_name", nargs="?", default=None, help="Parameter name (alternative to colon format)")
    p.add_argument("--timeout", type=float, default=5.0, help="Timeout in seconds (default: 5)")
    p = psub.add_parser("set", help="Set parameter value")
    p.add_argument("name", help="/node_name:param_name or just /node_name")
    p.add_argument("value", help="Value to set, or parameter name when using /node param value format")
    p.add_argument("extra_value", nargs="?", default=None, help="Value when using /node param value format")
    p.add_argument("--timeout", type=float, default=5.0, help="Timeout in seconds (default: 5)")

    actions = sub.add_parser("actions", help="Action operations")
    asub = actions.add_subparsers(dest="subcommand")
    asub.add_parser("list", help="List all actions")
    p = asub.add_parser("details", help="Get action details")
    p.add_argument("action")
    p = asub.add_parser("info", help="Alias for details (ros2 action info)")
    p.add_argument("action")
    p = asub.add_parser("type", help="Get action server type")
    p.add_argument("action", nargs="?")
    p.add_argument("--timeout", type=float, default=5.0,
                   help="Timeout in seconds (default: 5)")
    p = asub.add_parser("send", help="Send action goal")
    p.add_argument("action")
    p.add_argument("goal", help="JSON goal")
    p.add_argument("--timeout", type=float, default=30.0, help="Timeout in seconds (default: 30)")
    p = asub.add_parser("send-goal", help="Alias for send (ros2 action send_goal)")
    p.add_argument("action")
    p.add_argument("goal", help="JSON goal")
    p.add_argument("--timeout", type=float, default=30.0, help="Timeout in seconds (default: 30)")

    return parser


DISPATCH = {
    ("version", None): cmd_version,
    ("estop", None): cmd_estop,
    # topics — canonical
    ("topics", "list"): cmd_topics_list,
    ("topics", "type"): cmd_topics_type,
    ("topics", "details"): cmd_topics_details,
    ("topics", "message"): cmd_topics_message,
    ("topics", "subscribe"): cmd_topics_subscribe,
    ("topics", "publish"): cmd_topics_publish,
    ("topics", "publish-sequence"): cmd_topics_publish_sequence,
    ("topics", "publish-until"): cmd_topics_publish_until,
    ("topics", "publish-continuous"): cmd_topics_publish_continuous,
    ("topics", "hz"): cmd_topics_hz,
    ("topics", "find"): cmd_topics_find,
    # topics — aliases
    ("topics", "echo"): cmd_topics_subscribe,
    ("topics", "sub"): cmd_topics_subscribe,
    ("topics", "pub"): cmd_topics_publish,
    ("topics", "pub-seq"): cmd_topics_publish_sequence,
    ("topics", "info"): cmd_topics_details,
    # services — canonical
    ("services", "list"): cmd_services_list,
    ("services", "type"): cmd_services_type,
    ("services", "details"): cmd_services_details,
    ("services", "call"): cmd_services_call,
    ("services", "find"): cmd_services_find,
    # services — aliases
    ("services", "info"): cmd_services_details,
    # nodes — canonical
    ("nodes", "list"): cmd_nodes_list,
    ("nodes", "details"): cmd_nodes_details,
    # nodes — aliases
    ("nodes", "info"): cmd_nodes_details,
    # params
    ("params", "list"): cmd_params_list,
    ("params", "get"): cmd_params_get,
    ("params", "set"): cmd_params_set,
    # actions — canonical
    ("actions", "list"): cmd_actions_list,
    ("actions", "details"): cmd_actions_details,
    ("actions", "send"): cmd_actions_send,
    ("actions", "type"): cmd_actions_type,
    # actions — aliases
    ("actions", "info"): cmd_actions_details,
    ("actions", "send-goal"): cmd_actions_send,
}


def main():
    parser = build_parser()
    args = parser.parse_args()

    if not args.command:
        parser.print_help()
        sys.exit(1)

    key = (args.command, getattr(args, "subcommand", None))
    handler = DISPATCH.get(key)
    if handler:
        try:
            handler(args)
        finally:
            # Guarantee rclpy is shut down even if an exception escapes the
            # handler's own try/except (e.g. a bug or a KeyboardInterrupt).
            try:
                rclpy.shutdown()
            except Exception:
                pass
    else:
        parser.print_help()
        sys.exit(1)


if __name__ == "__main__":
    main()
