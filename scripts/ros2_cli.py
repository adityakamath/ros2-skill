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

  topics publish           <topic> <json_message> [--duration SEC | --timeout SEC] [--rate HZ]
  topics pub               <topic> <json_message> [--duration SEC | --timeout SEC] [--rate HZ]
  topics publish-continuous <topic> <json_message> [--duration SEC | --timeout SEC] [--rate HZ]
    Publish a message to a topic.  pub and publish-continuous are aliases for publish.
    Without --duration / --timeout: sends once (single-shot).
    With --duration / --timeout: publishes repeatedly at --rate Hz for the specified seconds,
    then stops.  Reports stopped_by: "timeout" or "keyboard_interrupt".
    --duration SECONDS   Publish repeatedly for this duration (--timeout is an alias)
    --timeout SECONDS    Alias for --duration
    --rate HZ            Publish rate (default: 10 Hz)
    $ python3 ros2_cli.py topics publish /turtle1/cmd_vel \
        '{"linear":{"x":2.0,"y":0,"z":0},"angular":{"x":0,"y":0,"z":0}}'
    $ python3 ros2_cli.py topics pub /cmd_vel \
        '{"linear":{"x":1.0,"y":0,"z":0},"angular":{"x":0,"y":0,"z":0}}' --duration 3
    $ python3 ros2_cli.py topics publish /cmd_vel \
        '{"linear":{"x":0.3},"angular":{"z":0}}' --timeout 5

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

  topics publish-until <topic> <json_msg> --monitor <topic> --field <path> [<path2> ...] (--delta|--above|--below|--equals) <value> [--euclidean]
    Publish a message at --rate Hz while monitoring a second topic. Stops as soon as the
    condition on the monitored field is satisfied, or after --timeout seconds (default: 60).
    --monitor TOPIC        Topic to monitor for the stop condition
    --field PATH [PATH...] One or more dot-separated field paths (e.g. pose.pose.position.x)
    --euclidean            Compute Euclidean distance across all --field paths; requires --delta
    --delta N              Stop when field changes by ±N (or Euclidean distance >= N)
    --above N              Stop when field > N (absolute threshold; single field only)
    --below N              Stop when field < N (absolute threshold; single field only)
    --equals V             Stop when field == V (single field only)
    --rate HZ              Publish rate (default: 10 Hz)
    --timeout SEC          Safety timeout (default: 60 s)
    $ python3 ros2_cli.py topics publish-until /cmd_vel \
        '{"linear":{"x":0.3},"angular":{"z":0}}' \
        --monitor /odom --field pose.pose.position.x --delta 1.0
    $ python3 ros2_cli.py topics publish-until /cmd_vel \
        '{"linear":{"x":0.2},"angular":{"z":0.1}}' \
        --monitor /odom --field pose.pose.position.x pose.pose.position.y --euclidean --delta 2.0

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
    $ python3 ros2_cli.py services find std srvs/Empty

  services info <service>
    Alias for services details (ros2 service info).
    $ python3 ros2_cli.py services info /spawn

  nodes list
  nodes ls
    List all active nodes.  (ls is an alias for list)
    $ python3 ros2_cli.py nodes list
    $ python3 ros2_cli.py nodes ls

  nodes details <node>
  nodes info     <node>
    Get node details including publishers, subscribers, services, action servers,
    and action clients. (info is an alias for details, ros2 node info)
    $ python3 ros2_cli.py nodes details /turtlesim
    $ python3 ros2_cli.py nodes info /turtlesim

  params list <node>
  params ls   <node>
    List all parameters for a node.  (ls is an alias for list)
    $ python3 ros2_cli.py params list /turtlesim
    $ python3 ros2_cli.py params ls /turtlesim

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
  actions ls
    List all available action servers.  (ls is an alias for list)
    $ python3 ros2_cli.py actions list
    $ python3 ros2_cli.py actions ls

  actions details <action>
  actions info    <action>
    Get action details including goal, result, and feedback fields.
    (info is an alias for details, ros2 action info)
    $ python3 ros2_cli.py actions details /turtle1/rotate_absolute
    $ python3 ros2_cli.py actions info /turtle1/rotate_absolute

  actions type <action>
    Get the type of an action server.
    $ python3 ros2_cli.py actions type /turtle1/rotate_absolute

  actions send      <action> <json_goal> [--feedback]
  actions send-goal <action> <json_goal> [--feedback]
    Send an action goal and wait for the result.
    --feedback    Collect feedback messages; included in output as feedback_msgs
    (send-goal is an alias for send, ros2 action send_goal)
    $ python3 ros2_cli.py actions send /turtle1/rotate_absolute '{"theta":3.14}'
    $ python3 ros2_cli.py actions send-goal /turtle1/rotate_absolute '{"theta":3.14}'
    $ python3 ros2_cli.py actions send /turtle1/rotate_absolute '{"theta":3.14}' --feedback

  actions cancel <action> [--timeout SEC]
    Cancel all in-flight goals on an action server.
    Uses zero UUID + zero timestamp per ROS 2 CancelGoal spec to cancel all goals.
    $ python3 ros2_cli.py actions cancel /turtle1/rotate_absolute

  topics bw <topic> [--window N] [--timeout SEC]
    Measure the bandwidth of a topic in bytes/s.
    --window N    Number of message samples (default: 10)
    --timeout SEC Max wait time in seconds (default: 10)
    $ python3 ros2_cli.py topics bw /camera/image_raw
    $ python3 ros2_cli.py topics bw /scan --window 20

  topics delay <topic> [--window N] [--timeout SEC]
    Measure end-to-end latency between header.stamp and wall clock.
    Errors if the message has no header.stamp field.
    --window N    Number of latency samples (default: 10)
    --timeout SEC Max wait time in seconds (default: 10)
    $ python3 ros2_cli.py topics delay /odom
    $ python3 ros2_cli.py topics delay /scan --window 20

  lifecycle nodes
    List all managed (lifecycle) nodes in the ROS 2 graph.
    $ python3 ros2_cli.py lifecycle nodes

  lifecycle list [<node>]
  lifecycle ls   [<node>]
    List available states and transitions for a managed (lifecycle) node.
    If no node is provided, queries all managed nodes.
    (ls is an alias for list)
    $ python3 ros2_cli.py lifecycle list /my_lifecycle_node
    $ python3 ros2_cli.py lifecycle ls

  lifecycle get <node>
    Get the current lifecycle state of a managed node.
    $ python3 ros2_cli.py lifecycle get /my_lifecycle_node

  lifecycle set <node> <transition>
    Trigger a lifecycle state transition. Accepts a label (e.g. configure) or
    a numeric transition ID. Prefers label when a string is provided.
    Common labels: configure, cleanup, activate, deactivate, shutdown
    $ python3 ros2_cli.py lifecycle set /my_lifecycle_node configure
    $ python3 ros2_cli.py lifecycle set /my_lifecycle_node 3

  params describe <node:param_name>
  params describe <node> <param_name>
    Describe a parameter: type, description, read_only, dynamic_typing, constraints.
    $ python3 ros2_cli.py params describe /turtlesim:background_r

  params dump <node> [--timeout SEC]
    Export all parameters for a node as a JSON dict {name: value}.
    $ python3 ros2_cli.py params dump /turtlesim

  params load <node> <json_or_file> [--timeout SEC]
    Bulk-set parameters from a JSON string or a file path.
    JSON format: {"param_name": value, ...}
    $ python3 ros2_cli.py params load /turtlesim '{"background_r":255}'

  params delete <node> <param_name> [extra_param_names...] [--timeout SEC]
    Delete one or more parameters from a node via DeleteParameters service.
    $ python3 ros2_cli.py params delete /turtlesim background_r

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
import os
import sys

# ---------------------------------------------------------------------------
# Shared infrastructure (re-exported for backward compatibility with tests
# and callers that do: import ros2_cli; ros2_cli.output(...)
# ---------------------------------------------------------------------------
from ros2_utils import (  # noqa: F401 – intentional re-exports
    MSG_ALIASES,
    get_msg_type,
    get_action_type,
    get_srv_type,
    get_msg_error,
    msg_to_dict,
    dict_to_msg,
    output,
    parse_node_param,
    ROS2CLI,
)

# ---------------------------------------------------------------------------
# Domain modules
# ---------------------------------------------------------------------------
from ros2_topic import (
    cmd_estop,
    cmd_topics_list,
    cmd_topics_type,
    cmd_topics_details,
    cmd_topics_message,
    cmd_topics_subscribe,
    cmd_topics_publish,
    cmd_topics_publish_sequence,
    cmd_topics_publish_until,
    cmd_topics_hz,
    cmd_topics_find,
    cmd_topics_bw,
    cmd_topics_delay,
    cmd_topics_capture_image,
    cmd_services_echo,
    cmd_actions_echo,
)

from ros2_node import (
    cmd_nodes_list,
    cmd_nodes_details,
)

from ros2_param import (
    cmd_params_list,
    cmd_params_get,
    cmd_params_set,
    cmd_params_describe,
    cmd_params_dump,
    cmd_params_load,
    cmd_params_delete,
)

from ros2_service import (
    cmd_services_list,
    cmd_services_type,
    cmd_services_details,
    cmd_services_call,
    cmd_services_find,
)

from ros2_action import (
    cmd_actions_list,
    cmd_actions_details,
    cmd_actions_send,
    cmd_actions_type,
    cmd_actions_cancel,
    cmd_actions_find,
)

from ros2_lifecycle import (
    cmd_lifecycle_nodes,
    cmd_lifecycle_list,
    cmd_lifecycle_get,
    cmd_lifecycle_set,
)

# Keep rclpy available for the main() finally block
import rclpy  # noqa: E402 (imported after sys.exit guard in ros2_utils)


# ---------------------------------------------------------------------------
# Built-in commands that don't need rclpy
# ---------------------------------------------------------------------------

def cmd_version(args):
    domain_id = int(os.environ.get('ROS_DOMAIN_ID', 0))
    distro = os.environ.get('ROS_DISTRO', 'unknown')
    output({"version": "2", "distro": distro, "domain_id": domain_id})


# ---------------------------------------------------------------------------
# Parser helpers
# ---------------------------------------------------------------------------

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
    parser = argparse.ArgumentParser(
        description="ROS 2 Skill CLI - Control ROS 2 robots directly via rclpy"
    )
    sub = parser.add_subparsers(dest="command")

    sub.add_parser("version", help="Detect ROS 2 version")

    estop = sub.add_parser("estop", help="Emergency stop for mobile robots (publishes zero velocity)")
    estop.add_argument("--topic", dest="topic", default=None,
                       help="Custom velocity topic (default: auto-detect)")

    # ------------------------------------------------------------------
    # topics
    # ------------------------------------------------------------------
    topics = sub.add_parser("topics", help="Topic operations")
    tsub = topics.add_subparsers(dest="subcommand")
    tsub.add_parser("list", help="List all topics")
    tsub.add_parser("ls", help="Alias for list")
    p = tsub.add_parser("type", help="Get topic message type")
    p.add_argument("topic")
    p = tsub.add_parser("details", help="Get topic details")
    p.add_argument("topic")
    p = tsub.add_parser("message", help="Get message structure")
    p.add_argument("message_type")
    p = tsub.add_parser("message-structure", help="Alias for message")
    p.add_argument("message_type")
    p = tsub.add_parser("message-struct", help="Alias for message")
    p.add_argument("message_type")
    _add_subscribe_args(tsub.add_parser("subscribe", help="Subscribe to a topic"))
    _add_subscribe_args(tsub.add_parser("echo", help="Echo topic messages (alias for subscribe)"))
    _add_subscribe_args(tsub.add_parser("sub", help="Alias for subscribe"))
    p = tsub.add_parser("capture-image", help="Capture image from ROS 2 topic")
    p.add_argument("--topic", required=True,
                   help="ROS 2 image topic (e.g., /camera/image_raw/compressed)")
    p.add_argument("--output", required=True, help="Output filename (saved in artifacts/)")
    p.add_argument("--timeout", type=float, default=5.0, help="Seconds to wait for image")
    p.add_argument("--type", choices=["auto", "compressed", "raw"], default="auto",
                   help="Image type: compressed, raw, or auto")
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
    p = tsub.add_parser("bw", help="Measure topic bandwidth")
    p.add_argument("topic", nargs="?")
    p.add_argument("--window", type=int, default=10,
                   help="Number of messages to sample (default: 10)")
    p.add_argument("--timeout", type=float, default=10.0,
                   help="Max wait time in seconds (default: 10)")
    p = tsub.add_parser("delay", help="Measure header.stamp → wall-clock latency")
    p.add_argument("topic", nargs="?")
    p.add_argument("--window", type=int, default=10,
                   help="Number of messages to sample (default: 10)")
    p.add_argument("--timeout", type=float, default=10.0,
                   help="Max wait time in seconds (default: 10)")
    for _pub_name in ("publish", "pub"):
        p = tsub.add_parser(_pub_name,
                            help="Publish a message" if _pub_name == "publish"
                            else "Alias for publish")
        p.add_argument("topic", nargs="?")
        p.add_argument("msg", nargs="?", help="JSON message")
        p.add_argument("--msg-type", dest="msg_type", default=None,
                       help="Message type (auto-detected if not provided)")
        p.add_argument("--duration", "--timeout", dest="duration", type=float, default=None,
                       help="Publish repeatedly for this many seconds (--timeout is an alias)")
        p.add_argument("--rate", type=float, default=10.0,
                       help="Publish rate in Hz (default: 10)")
    for _seq_name in ("publish-sequence", "pub-seq"):
        p = tsub.add_parser(_seq_name,
                            help="Publish message sequence with delays"
                            if _seq_name == "publish-sequence"
                            else "Alias for publish-sequence")
        p.add_argument("topic", nargs="?")
        p.add_argument("messages", nargs="?", help="JSON array of messages")
        p.add_argument("durations", nargs="?",
                       help="JSON array of durations in seconds (message is repeated during each)")
        p.add_argument("--msg-type", dest="msg_type", default=None,
                       help="Message type (auto-detected if not provided)")
        p.add_argument("--rate", type=float, default=10.0,
                       help="Publish rate in Hz (default: 10)")
    p = tsub.add_parser("publish-until",
                        help="Publish until a monitor-topic condition is met")
    p.add_argument("topic", nargs="?")
    p.add_argument("msg", nargs="?", help="JSON message to publish")
    p.add_argument("--monitor", default=None,
                   help="Topic to monitor for the stop condition (required)")
    p.add_argument("--field", nargs="+", default=None,
                   help="One or more dot-separated field paths in the monitor message "
                        "(e.g. pose.pose.position.x). Provide multiple paths with --euclidean "
                        "for N-dimensional Euclidean distance monitoring.")
    p.add_argument("--euclidean", action="store_true", default=False,
                   help="Compute Euclidean distance across all --field paths; "
                        "requires --delta as the distance threshold.")
    p.add_argument("--delta", type=float, default=None,
                   help="Stop when field changes by ±N from its value at start "
                        "(or Euclidean distance >= N with --euclidean)")
    p.add_argument("--above", type=float, default=None,
                   help="Stop when field > N (absolute threshold)")
    p.add_argument("--below", type=float, default=None,
                   help="Stop when field < N (absolute threshold)")
    p.add_argument("--equals", default=None,
                   help="Stop when field == value (numeric or string)")
    p.add_argument("--rate", type=float, default=10.0,
                   help="Publish rate in Hz (default: 10)")
    p.add_argument("--timeout", type=float, default=60.0,
                   help="Safety timeout in seconds (default: 60)")
    p.add_argument("--msg-type", dest="msg_type", default=None,
                   help="Publish topic message type (auto-detected)")
    p.add_argument("--monitor-msg-type", dest="monitor_msg_type", default=None,
                   help="Monitor topic message type (auto-detected)")
    p = tsub.add_parser("publish-continuous",
                        help="Alias for publish "
                             "(use topics publish --duration / --timeout instead)")
    p.add_argument("topic", nargs="?")
    p.add_argument("msg", nargs="?", help="JSON message to publish")
    p.add_argument("--msg-type", dest="msg_type", default=None,
                   help="Message type (auto-detected if not provided)")
    p.add_argument("--duration", "--timeout", dest="duration", type=float, default=None,
                   help="Publish repeatedly for this many seconds")
    p.add_argument("--rate", type=float, default=10.0,
                   help="Publish rate in Hz (default: 10)")

    # ------------------------------------------------------------------
    # services
    # ------------------------------------------------------------------
    services = sub.add_parser("services", help="Service operations")
    ssub = services.add_subparsers(dest="subcommand")
    ssub.add_parser("list", help="List all services")
    ssub.add_parser("ls", help="Alias for list")
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
    p.add_argument("request",
                   help="JSON request, or service type when using positional service-type format")
    p.add_argument("extra_request", nargs="?", default=None,
                   help="JSON request when using /svc service_type json positional format")
    p.add_argument("--service-type", dest="service_type", default=None,
                   help="Service type (auto-detected if not provided, "
                        "e.g. std_srvs/srv/SetBool)")
    p.add_argument("--timeout", type=float, default=5.0,
                   help="Timeout in seconds (default: 5)")
    p = ssub.add_parser("echo",
                        help="Echo service events (requires service introspection enabled)")
    p.add_argument("service")
    p.add_argument("--duration", type=float, default=None,
                   help="Collect events for exactly this many seconds (overrides --timeout)")
    p.add_argument("--max-messages", "--max-events", dest="max_messages", type=int, default=None,
                   help="Stop after collecting this many events "
                        "(default: unlimited within window)")
    p.add_argument("--timeout", type=float, default=30.0,
                   help="Collection window in seconds when --duration is not set (default: 30)")

    # ------------------------------------------------------------------
    # nodes
    # ------------------------------------------------------------------
    nodes = sub.add_parser("nodes", help="Node operations")
    nsub = nodes.add_subparsers(dest="subcommand")
    nsub.add_parser("list", help="List all nodes")
    nsub.add_parser("ls", help="Alias for list")
    p = nsub.add_parser("details", help="Get node details")
    p.add_argument("node")
    p = nsub.add_parser("info", help="Alias for details (ros2 node info)")
    p.add_argument("node")

    # ------------------------------------------------------------------
    # lifecycle
    # ------------------------------------------------------------------
    lifecycle = sub.add_parser("lifecycle", help="Lifecycle (managed node) operations")
    lsub = lifecycle.add_subparsers(dest="subcommand")
    lsub.add_parser("nodes", help="List all managed (lifecycle) nodes")
    for _list_name in ("list", "ls"):
        p = lsub.add_parser(_list_name,
                            help="List available states and transitions"
                            if _list_name == "list" else "Alias for list")
        p.add_argument("node", nargs="?", default=None,
                       help="Node name (e.g. /my_node); if omitted, queries all managed nodes")
        p.add_argument("--timeout", type=float, default=5.0,
                       help="Timeout per node in seconds (default: 5)")
    p = lsub.add_parser("get", help="Get current lifecycle state of a managed node")
    p.add_argument("node", help="Node name (e.g. /my_lifecycle_node)")
    p.add_argument("--timeout", type=float, default=5.0,
                   help="Timeout in seconds (default: 5)")
    p = lsub.add_parser("set", help="Trigger a lifecycle state transition")
    p.add_argument("node", help="Node name (e.g. /my_lifecycle_node)")
    p.add_argument("transition",
                   help="Transition label (e.g. configure, activate) or numeric ID")
    p.add_argument("--timeout", type=float, default=5.0,
                   help="Timeout in seconds (default: 5)")

    # ------------------------------------------------------------------
    # params
    # ------------------------------------------------------------------
    params = sub.add_parser("params", help="Parameter operations")
    psub = params.add_subparsers(dest="subcommand")
    for _params_list_name in ("list", "ls"):
        p = psub.add_parser(_params_list_name,
                            help="List parameters for a node"
                            if _params_list_name == "list" else "Alias for list")
        p.add_argument("node")
        p.add_argument("--timeout", type=float, default=5.0,
                       help="Timeout in seconds (default: 5)")
    p = psub.add_parser("get", help="Get parameter value")
    p.add_argument("name", help="/node_name:param_name or just /node_name")
    p.add_argument("param_name", nargs="?", default=None,
                   help="Parameter name (alternative to colon format)")
    p.add_argument("--timeout", type=float, default=5.0,
                   help="Timeout in seconds (default: 5)")
    p = psub.add_parser("set", help="Set parameter value")
    p.add_argument("name", help="/node_name:param_name or just /node_name")
    p.add_argument("value",
                   help="Value to set, or parameter name when using /node param value format")
    p.add_argument("extra_value", nargs="?", default=None,
                   help="Value when using /node param value format")
    p.add_argument("--timeout", type=float, default=5.0,
                   help="Timeout in seconds (default: 5)")
    p = psub.add_parser("describe", help="Get parameter descriptor")
    p.add_argument("name", help="/node_name:param_name or just /node_name")
    p.add_argument("param_name", nargs="?", default=None,
                   help="Parameter name (alternative to colon format)")
    p.add_argument("--timeout", type=float, default=5.0,
                   help="Timeout in seconds (default: 5)")
    p = psub.add_parser("dump", help="Dump all parameters of a node as JSON")
    p.add_argument("node", help="Node name (e.g. /turtlesim)")
    p.add_argument("--timeout", type=float, default=5.0,
                   help="Timeout in seconds (default: 5)")
    p = psub.add_parser("load",
                        help="Load parameters onto a node from JSON string or file")
    p.add_argument("node", help="Node name (e.g. /turtlesim)")
    p.add_argument("params", help="JSON string or file path with parameters dict")
    p.add_argument("--timeout", type=float, default=5.0,
                   help="Timeout in seconds (default: 5)")
    p = psub.add_parser("delete", help="Delete a parameter from a node")
    p.add_argument("name", help="/node_name:param_name or just /node_name")
    p.add_argument("param_name", nargs="?", default=None,
                   help="Parameter name (alternative to colon format)")
    p.add_argument("extra_names", nargs="*", default=[],
                   help="Additional parameter names to delete")
    p.add_argument("--timeout", type=float, default=5.0,
                   help="Timeout in seconds (default: 5)")

    # ------------------------------------------------------------------
    # actions
    # ------------------------------------------------------------------
    actions = sub.add_parser("actions", help="Action operations")
    asub = actions.add_subparsers(dest="subcommand")
    asub.add_parser("list", help="List all actions")
    asub.add_parser("ls", help="Alias for list")
    p = asub.add_parser("details", help="Get action details")
    p.add_argument("action")
    p = asub.add_parser("info", help="Alias for details (ros2 action info)")
    p.add_argument("action")
    p = asub.add_parser("type", help="Get action server type")
    p.add_argument("action", nargs="?")
    p.add_argument("--timeout", type=float, default=5.0,
                   help="Timeout in seconds (default: 5)")
    for _send_name in ("send", "send-goal"):
        p = asub.add_parser(_send_name,
                            help="Send action goal" if _send_name == "send"
                            else "Alias for send (ros2 action send_goal)")
        p.add_argument("action")
        p.add_argument("goal", help="JSON goal")
        p.add_argument("--timeout", type=float, default=30.0,
                       help="Timeout in seconds (default: 30)")
        p.add_argument("--feedback", action="store_true", default=False,
                       help="Collect and return feedback messages alongside the result")
    p = asub.add_parser("cancel",
                        help="Cancel all in-flight goals on an action server")
    p.add_argument("action", nargs="?")
    p.add_argument("--timeout", type=float, default=5.0,
                   help="Timeout in seconds (default: 5)")
    p = asub.add_parser("echo", help="Echo action feedback and status messages")
    p.add_argument("action")
    p.add_argument("--duration", type=float, default=None,
                   help="Collect feedback for this many seconds "
                        "(default: wait for first message)")
    p.add_argument("--max-messages", "--max-msgs", dest="max_messages", type=int, default=100,
                   help="Max feedback messages to collect when using --duration (default: 100)")
    p.add_argument("--timeout", type=float, default=5.0,
                   help="Timeout waiting for first feedback in seconds (default: 5)")
    p = asub.add_parser("find", help="Find action servers by action type")
    p.add_argument("action_type", nargs="?")

    return parser


# ---------------------------------------------------------------------------
# Dispatch table
# ---------------------------------------------------------------------------

DISPATCH = {
    ("version", None): cmd_version,
    ("estop", None): cmd_estop,
    # topics — canonical
    ("topics", "list"): cmd_topics_list,
    ("topics", "type"): cmd_topics_type,
    ("topics", "details"): cmd_topics_details,
    ("topics", "message"): cmd_topics_message,
    ("topics", "message-structure"): cmd_topics_message,
    ("topics", "message-struct"): cmd_topics_message,
    ("topics", "subscribe"): cmd_topics_subscribe,
    ("topics", "publish"): cmd_topics_publish,
    ("topics", "publish-sequence"): cmd_topics_publish_sequence,
    ("topics", "publish-until"): cmd_topics_publish_until,
    ("topics", "publish-continuous"): cmd_topics_publish,
    ("topics", "hz"): cmd_topics_hz,
    ("topics", "find"): cmd_topics_find,
    ("topics", "capture-image"): cmd_topics_capture_image,
    # topics — aliases
    ("topics", "echo"): cmd_topics_subscribe,
    ("topics", "sub"): cmd_topics_subscribe,
    ("topics", "pub"): cmd_topics_publish,
    ("topics", "pub-seq"): cmd_topics_publish_sequence,
    ("topics", "ls"): cmd_topics_list,
    ("topics", "info"): cmd_topics_details,
    # topics — Phase 2
    ("topics", "bw"): cmd_topics_bw,
    ("topics", "delay"): cmd_topics_delay,
    # services — canonical
    ("services", "list"): cmd_services_list,
    ("services", "type"): cmd_services_type,
    ("services", "details"): cmd_services_details,
    ("services", "call"): cmd_services_call,
    ("services", "find"): cmd_services_find,
    # services — aliases
    ("services", "ls"): cmd_services_list,
    ("services", "info"): cmd_services_details,
    ("services", "echo"): cmd_services_echo,
    # nodes — canonical
    ("nodes", "list"): cmd_nodes_list,
    ("nodes", "details"): cmd_nodes_details,
    # nodes — aliases
    ("nodes", "info"): cmd_nodes_details,
    ("nodes", "ls"): cmd_nodes_list,
    # lifecycle — canonical
    ("lifecycle", "nodes"): cmd_lifecycle_nodes,
    ("lifecycle", "list"): cmd_lifecycle_list,
    ("lifecycle", "get"): cmd_lifecycle_get,
    ("lifecycle", "set"): cmd_lifecycle_set,
    # lifecycle — alias
    ("lifecycle", "ls"): cmd_lifecycle_list,
    # params — canonical
    ("params", "list"): cmd_params_list,
    ("params", "get"): cmd_params_get,
    ("params", "set"): cmd_params_set,
    ("params", "describe"): cmd_params_describe,
    ("params", "dump"): cmd_params_dump,
    ("params", "load"): cmd_params_load,
    ("params", "delete"): cmd_params_delete,
    # params — alias
    ("params", "ls"): cmd_params_list,
    # actions — canonical
    ("actions", "list"): cmd_actions_list,
    ("actions", "details"): cmd_actions_details,
    ("actions", "send"): cmd_actions_send,
    ("actions", "type"): cmd_actions_type,
    ("actions", "cancel"): cmd_actions_cancel,
    ("actions", "echo"): cmd_actions_echo,
    ("actions", "find"): cmd_actions_find,
    # actions — aliases
    ("actions", "info"): cmd_actions_details,
    ("actions", "send-goal"): cmd_actions_send,
    ("actions", "ls"): cmd_actions_list,
}


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

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
            # handler's own try/except (e.g. a bug or KeyboardInterrupt).
            try:
                rclpy.shutdown()
            except Exception:
                pass
    else:
        parser.print_help()
        sys.exit(1)


if __name__ == "__main__":
    main()
