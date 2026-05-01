#!/usr/bin/env python3
"""ROS 2 Skill CLI — see references/COMMANDS.md for full command reference."""

import argparse
import os
import sys

# ---------------------------------------------------------------------------
# Shared infrastructure (re-exported for backward compatibility with tests
# and callers that do: import ros2_cli; ros2_cli.output(...)
# ---------------------------------------------------------------------------
from ros2_utils import (  # noqa: F401 – intentional re-exports
    get_msg_type,
    get_action_type,
    get_srv_type,
    get_msg_error,
    msg_to_dict,
    dict_to_msg,
    output,
    parse_node_param,
    ROS2CLI,
    ros2_context,
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
    cmd_topics_diag_list,
    cmd_topics_diag,
    cmd_topics_battery_list,
    cmd_topics_battery,
    cmd_topics_qos_check,
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
    cmd_params_preset_save,
    cmd_params_preset_load,
    cmd_params_preset_list,
    cmd_params_preset_delete,
    cmd_params_find,
)

from ros2_service import (
    cmd_services_list,
    cmd_services_type,
    cmd_services_details,
    cmd_services_call,
    cmd_services_echo,
    cmd_services_find,
)

from ros2_action import (
    cmd_actions_list,
    cmd_actions_details,
    cmd_actions_send,
    cmd_actions_echo,
    cmd_actions_type,
    cmd_actions_cancel,
    cmd_actions_find,
)

from ros2_doctor import (
    cmd_doctor_check,
    cmd_doctor_hello,
)
from ros2_multicast import (
    cmd_multicast_send,
    cmd_multicast_receive,
)
from ros2_launch import (
    cmd_launch_run,
    cmd_launch_list,
    cmd_launch_kill,
    cmd_launch_restart,
    cmd_launch_foxglove,
)
from ros2_run import (
    cmd_run,
    cmd_run_list,
    cmd_run_kill,
    cmd_run_restart,
)
from ros2_tf import (
    cmd_tf_list,
    cmd_tf_lookup,
    cmd_tf_echo,
    cmd_tf_monitor,
    cmd_tf_static,
    cmd_tf_euler_from_quaternion,
    cmd_tf_quaternion_from_euler,
    cmd_tf_euler_from_quaternion_degrees,
    cmd_tf_quaternion_from_euler_degrees,
    cmd_tf_transform_point,
    cmd_tf_transform_vector,
    cmd_tf_tree,
    cmd_tf_validate,
)
from ros2_lifecycle import (
    cmd_lifecycle_nodes,
    cmd_lifecycle_list,
    cmd_lifecycle_get,
    cmd_lifecycle_set,
)
from ros2_interface import (
    cmd_interface_list,
    cmd_interface_show,
    cmd_interface_proto,
    cmd_interface_packages,
    cmd_interface_package,
)
from ros2_bag import (
    cmd_bag_info,
)
from ros2_logs import (
    cmd_logs_list_runs,
    cmd_logs_query,
    cmd_logs_tail,
    cmd_logs_node_summary,
)
from ros2_component import (
    cmd_component_types,
    cmd_component_list,
    cmd_component_load,
    cmd_component_unload,
    cmd_component_standalone,
    cmd_component_kill,
)
from ros2_pkg import (
    cmd_pkg_list,
    cmd_pkg_prefix,
    cmd_pkg_executables,
    cmd_pkg_xml,
    cmd_pkg_create,
)
from ros2_daemon import (
    cmd_daemon_status,
    cmd_daemon_start,
    cmd_daemon_stop,
)
from ros2_control import (
    cmd_control_list_controller_types,
    cmd_control_list_controllers,
    cmd_control_list_hardware_components,
    cmd_control_list_hardware_interfaces,
    cmd_control_load_controller,
    cmd_control_unload_controller,
    cmd_control_configure_controller,
    cmd_control_reload_controller_libraries,
    cmd_control_set_controller_state,
    cmd_control_set_hardware_component_state,
    cmd_control_switch_controllers,
    cmd_control_view_controller_chains,
)

# ---------------------------------------------------------------------------
# Built-in commands that don't need rclpy
# ---------------------------------------------------------------------------

def cmd_version(args):
    domain_id = int(os.environ.get('ROS_DOMAIN_ID', 0))
    distro = os.environ.get('ROS_DISTRO', 'unknown')
    try:
        import rclpy  # noqa: F401
        rclpy_available = True
    except ImportError:
        rclpy_available = False
    output({"version": "2", "distro": distro, "domain_id": domain_id, "rclpy_available": rclpy_available})


def cmd_context(args):
    """Compact session-start graph summary: topics, services, actions, nodes."""
    limit = getattr(args, "limit", 50) or 50
    try:
        with ros2_context():
            node = ROS2CLI()
            all_topics = node.get_topic_names()
            all_services = node.get_service_names()
            all_topic_types = node.get_topic_names_and_types()
            node_info = node.get_node_names_and_namespaces()

        topic_names = [n for n, _ in all_topics]
        topic_types = [t[0] if t else "" for _, t in all_topics]

        # Derive actions from topic names containing /_action/
        actions = []
        seen = set()
        for name, _ in all_topic_types:
            if '/_action/' in name:
                action_name = name.split('/_action/')[0]
                if action_name not in seen:
                    seen.add(action_name)
                    actions.append(action_name)

        service_names = [n for n, _ in all_services]
        node_names = [f"{ns.rstrip('/')}/{n}" for n, ns in node_info]

        total_topics = len(topic_names)
        truncated = limit > 0 and total_topics > limit
        if truncated:
            topic_names = topic_names[:limit]
            topic_types = topic_types[:limit]

        result = {
            "topics": topic_names,
            "topic_types": topic_types,
            "services": service_names,
            "actions": actions,
            "nodes": node_names,
            "counts": {
                "topics": total_topics,
                "services": len(service_names),
                "actions": len(actions),
                "nodes": len(node_names),
            },
        }
        if truncated:
            result["topics_truncated"] = True
            result["topics_shown"] = limit
        output(result)
    except Exception as e:
        output({"error": str(e)})


# ---------------------------------------------------------------------------
# Parser helpers
# ---------------------------------------------------------------------------

def _add_subscribe_args(p):
    """Add the shared arguments for the subscribe / echo subparsers."""
    p.add_argument("topic", nargs="?", help="Topic name to subscribe to (e.g. /cmd_vel)")
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
    parser.add_argument(
        "--timeout", type=float, default=None, dest="global_timeout",
        metavar="SECONDS",
        help="Global timeout override in seconds — overrides the per-command default "
             "for any command that supports --timeout",
    )
    parser.add_argument(
        "--retries", type=int, default=1, dest="global_retries",
        metavar="N",
        help="Number of attempts for service/action calls before giving up "
             "(default: 1, no retry)",
    )
    sub = parser.add_subparsers(dest="command")

    sub.add_parser("version", help="Detect ROS 2 version")

    p = sub.add_parser("context", help="Compact session-start summary: topics, services, actions, nodes")
    p.add_argument("--limit", type=int, default=50,
                   help="Max topics to return (default: 50; 0 = unlimited)")

    estop = sub.add_parser("estop", help="Emergency stop for mobile robots (publishes zero velocity)")
    estop.add_argument("--topic", dest="topic", default=None,
                       help="Custom velocity topic (default: auto-detect)")

    # ------------------------------------------------------------------
    # topics
    # ------------------------------------------------------------------
    topics = sub.add_parser("topics", help="Topic operations")
    tsub = topics.add_subparsers(dest="subcommand")
    p = tsub.add_parser("list", help="List all topics")
    p.add_argument("--limit", type=int, default=0,
                   help="Max topics to return (default: 0 = unlimited)")
    p = tsub.add_parser("ls", help="Alias for list")
    p.add_argument("--limit", type=int, default=0,
                   help="Max topics to return (default: 0 = unlimited)")
    p = tsub.add_parser("type", help="Get topic message type")
    p.add_argument("topic", help="Topic name (e.g. /cmd_vel)")
    p = tsub.add_parser("details", help="Get topic details")
    p.add_argument("topic", help="Topic name (e.g. /cmd_vel)")
    p = tsub.add_parser("message", help="Get message structure")
    p.add_argument("message_type", help="Message type (e.g. geometry_msgs/msg/Twist)")
    p = tsub.add_parser("message-structure", help="Alias for message")
    p.add_argument("message_type", help="Message type (e.g. geometry_msgs/msg/Twist)")
    p = tsub.add_parser("message-struct", help="Alias for message")
    p.add_argument("message_type", help="Message type (e.g. geometry_msgs/msg/Twist)")
    _add_subscribe_args(tsub.add_parser("subscribe", help="Subscribe to a topic"))
    _add_subscribe_args(tsub.add_parser("echo", help="Echo topic messages (alias for subscribe)"))
    p = tsub.add_parser("capture-image", help="Capture image from ROS 2 topic")
    p.add_argument("--topic", required=True,
                   help="ROS 2 image topic (e.g., /camera/image_raw/compressed)")
    p.add_argument("--output", required=True, help="Output filename (saved in .artifacts/)")
    p.add_argument("--timeout", type=float, default=5.0, help="Seconds to wait for image")
    p.add_argument("--type", choices=["auto", "compressed", "raw"], default="auto",
                   help="Image type: compressed, raw, or auto")
    p = tsub.add_parser("info", help="Alias for details (ros2 topic info)")
    p.add_argument("topic", help="Topic name (e.g. /cmd_vel)")
    p = tsub.add_parser("hz", help="Measure topic publish rate")
    p.add_argument("topic", nargs="?", help="Topic name (e.g. /cmd_vel)")
    p.add_argument("--window", type=int, default=10,
                   help="Number of inter-message intervals to sample (default: 10)")
    p.add_argument("--timeout", type=float, default=10.0,
                   help="Max wait time in seconds (default: 10)")
    p = tsub.add_parser("find", help="Find topics by message type")
    p.add_argument("msg_type", nargs="?", help="Message type to search for (e.g. geometry_msgs/msg/Twist)")
    p.add_argument("--timeout", type=float, default=5.0,
                   help="Timeout in seconds (default: 5)")
    p = tsub.add_parser("bw", help="Measure topic bandwidth")
    p.add_argument("topic", nargs="?", help="Topic name (e.g. /camera/image_raw)")
    p.add_argument("--window", type=int, default=10,
                   help="Number of messages to sample (default: 10)")
    p.add_argument("--timeout", type=float, default=10.0,
                   help="Max wait time in seconds (default: 10)")
    p = tsub.add_parser("delay", help="Measure header.stamp → wall-clock latency")
    p.add_argument("topic", nargs="?", help="Topic name (must publish messages with header.stamp)")
    p.add_argument("--window", type=int, default=10,
                   help="Number of messages to sample (default: 10)")
    p.add_argument("--timeout", type=float, default=10.0,
                   help="Max wait time in seconds (default: 10)")
    p = tsub.add_parser("qos-check", help="Check QoS compatibility between publishers and subscribers")
    p.add_argument("topic", help="Topic name (e.g. /odom)")
    p.add_argument("--timeout", type=float, default=5.0,
                   help="Timeout in seconds (default: 5)")
    tsub.add_parser("diag-list",
                    help="List all topics publishing DiagnosticArray messages (by type)")
    p = tsub.add_parser("diag",
                        help="Subscribe to diagnostic topics (auto-discovered by type)")
    p.add_argument("--topic", default=None,
                   help="Specific diagnostic topic (default: auto-discover all by type)")
    p.add_argument("--duration", type=float, default=None,
                   help="Collect for N seconds (default: one-shot)")
    p.add_argument("--max-messages", dest="max_messages", type=int, default=1,
                   help="Max messages per topic in --duration mode (default: 1)")
    p.add_argument("--timeout", type=float, default=10.0,
                   help="Timeout waiting for messages (default: 10s)")
    tsub.add_parser("battery-list",
                    help="List all topics publishing BatteryState messages (by type)")
    p = tsub.add_parser("battery",
                        help="Subscribe to battery topics (auto-discovered by type)")
    p.add_argument("--topic", default=None,
                   help="Specific battery topic (default: auto-discover all by type)")
    p.add_argument("--duration", type=float, default=None,
                   help="Collect for N seconds (default: one-shot)")
    p.add_argument("--max-messages", dest="max_messages", type=int, default=1,
                   help="Max messages per topic in --duration mode (default: 1)")
    p.add_argument("--timeout", type=float, default=10.0,
                   help="Timeout waiting for messages (default: 10s)")
    for _pub_name in ("publish", "pub"):
        p = tsub.add_parser(_pub_name,
                            help="Publish a message" if _pub_name == "publish"
                            else "Alias for publish")
        p.add_argument("topic", nargs="?", help="Topic name to publish to (e.g. /cmd_vel)")
        p.add_argument("msg", nargs="?", help="JSON message")
        p.add_argument("--msg-type", dest="msg_type", default=None,
                       help="Message type (auto-detected if not provided)")
        p.add_argument("--duration", "--timeout", dest="duration", type=float, default=None,
                       help="Publish repeatedly for this many seconds (--timeout is an alias)")
        p.add_argument("--rate", type=float, default=10.0,
                       help="Publish rate in Hz (default: 10)")
        p.add_argument("--retries", type=int, default=None,
                       help="Number of attempts before giving up (overrides global --retries)")
    p = tsub.add_parser("publish-sequence", help="Publish message sequence with delays")
    p.add_argument("topic", nargs="?", help="Topic name to publish to (e.g. /cmd_vel)")
    p.add_argument("messages", nargs="?", help="JSON array of messages")
    p.add_argument("durations", nargs="?",
                   help="JSON array of durations in seconds (message is repeated during each)")
    p.add_argument("--msg-type", dest="msg_type", default=None,
                   help="Message type (auto-detected if not provided)")
    p.add_argument("--rate", type=float, default=10.0,
                   help="Publish rate in Hz (default: 10)")
    p = tsub.add_parser("publish-until",
                        help="Publish until a monitor-topic condition is met")
    p.add_argument("topic", nargs="?", help="Topic name to publish to (e.g. /cmd_vel)")
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
    p.add_argument("--rotate", type=float, default=None,
                   help="Rotate by N radians (or degrees with --degrees). "
                        "SIGN DETERMINES DIRECTION: positive = CCW (left), negative = CW (right). "
                        "The sign of --rotate MUST match the sign of angular.z in the message — "
                        "mismatched signs will cause the command to never stop (monitor waits for "
                        "opposite direction). Zero is the only invalid value. "
                        "Requires --monitor with an odometry topic.")
    p.add_argument("--degrees", action="store_true", default=False,
                   help="Interpret --rotate angle in degrees instead of radians")
    p.add_argument("--slow-last", dest="slow_last", type=float, default=None,
                   help="Begin decelerating when this many units remain before the target. "
                        "Units match the movement type: metres for --field/--euclidean distance, "
                        "degrees if --degrees is set, radians otherwise for --rotate. "
                        "Velocity ramps linearly from full commanded speed at --slow-last remaining "
                        "down to --slow-factor × full speed at the target.")
    p.add_argument("--slow-factor", dest="slow_factor", type=float, default=0.25,
                   help="Minimum velocity as a fraction of the commanded velocity at the end of "
                        "the decel zone (0–1, default 0.25). E.g. 0.25 means the robot finishes "
                        "at 25%% of its commanded speed.")
    p.add_argument("--rate", type=float, default=10.0,
                   help="Publish rate in Hz (default: 10)")
    p.add_argument("--timeout", type=float, default=60.0,
                   help="Safety timeout in seconds (default: 60)")
    p.add_argument("--retries", type=int, default=None,
                   help="Number of attempts before giving up (overrides global --retries)")
    p.add_argument("--msg-type", dest="msg_type", default=None,
                   help="Publish topic message type (auto-detected)")
    p.add_argument("--monitor-msg-type", dest="monitor_msg_type", default=None,
                   help="Monitor topic message type (auto-detected)")
    # ------------------------------------------------------------------
    # services
    # ------------------------------------------------------------------
    services = sub.add_parser("services", help="Service operations")
    ssub = services.add_subparsers(dest="subcommand")
    ssub.add_parser("list", help="List all services")
    ssub.add_parser("ls", help="Alias for list")
    p = ssub.add_parser("type", help="Get service type")
    p.add_argument("service", help="Service name (e.g. /add_two_ints)")
    p = ssub.add_parser("details", help="Get service details")
    p.add_argument("service", help="Service name (e.g. /add_two_ints)")
    p = ssub.add_parser("info", help="Alias for details (ros2 service info)")
    p.add_argument("service", help="Service name (e.g. /add_two_ints)")
    p = ssub.add_parser("find", help="Find services by service type")
    p.add_argument("service_type", nargs="?", help="Service type to search for (e.g. std_srvs/srv/SetBool)")
    p.add_argument("--timeout", type=float, default=5.0,
                   help="Timeout in seconds (default: 5)")
    p = ssub.add_parser("call", help="Call a service")
    p.add_argument("service", help="Service name (e.g. /add_two_ints)")
    p.add_argument("request",
                   help="JSON request, or service type when using positional service-type format")
    p.add_argument("extra_request", nargs="?", default=None,
                   help="JSON request when using /svc service_type json positional format")
    p.add_argument("--service-type", dest="service_type", default=None,
                   help="Service type (auto-detected if not provided, "
                        "e.g. std_srvs/srv/SetBool)")
    p.add_argument("--timeout", type=float, default=5.0,
                   help="Timeout in seconds (default: 5)")
    p.add_argument("--retries", type=int, default=None,
                   help="Number of attempts before giving up (overrides global --retries)")
    p = ssub.add_parser("echo",
                        help="Echo service events (requires service introspection enabled)")
    p.add_argument("service", help="Service name (e.g. /add_two_ints)")
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
    p.add_argument("node", help="Node name (e.g. /turtlesim)")
    p = nsub.add_parser("info", help="Alias for details (ros2 node info)")
    p.add_argument("node", help="Node name (e.g. /turtlesim)")

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
    # control
    # ------------------------------------------------------------------
    ctrl = sub.add_parser("control", help="Controller manager operations (ros2 control)")
    csub = ctrl.add_subparsers(dest="subcommand")

    def _add_cm_args(p):
        p.add_argument(
            "--controller-manager", dest="controller_manager",
            default="/controller_manager",
            help="Controller manager node name (default: /controller_manager)",
        )
        p.add_argument("--timeout", type=float, default=5.0,
                       help="Service call timeout in seconds (default: 5)")

    p = csub.add_parser("list-controller-types",
                        help="List available controller types and their base classes")
    _add_cm_args(p)

    p = csub.add_parser("list-controllers",
                        help="List loaded controllers, their type and status")
    _add_cm_args(p)

    p = csub.add_parser("list-hardware-components",
                        help="List available hardware components")
    _add_cm_args(p)

    p = csub.add_parser("list-hardware-interfaces",
                        help="List available command and state interfaces")
    _add_cm_args(p)

    for _name in ("load-controller", "load"):
        p = csub.add_parser(_name,
            help="Load a controller in the controller manager"
            if _name == "load-controller" else "Alias for load-controller")
        p.add_argument("name", help="Controller name")
        _add_cm_args(p)

    for _name in ("unload-controller", "unload"):
        p = csub.add_parser(_name,
            help="Unload a controller from the controller manager"
            if _name == "unload-controller" else "Alias for unload-controller")
        p.add_argument("name", help="Controller name")
        _add_cm_args(p)

    p = csub.add_parser("configure-controller",
                        help="Configure a loaded controller (unconfigured → inactive)")
    p.add_argument("name", help="Controller name")
    _add_cm_args(p)

    p = csub.add_parser("reload-controller-libraries",
                        help="Reload controller libraries")
    p.add_argument("--force-kill", dest="force_kill", action="store_true", default=False,
                   help="Force kill controllers before reloading")
    _add_cm_args(p)

    p = csub.add_parser("set-controller-state",
                        help="Adjust the state of the controller")
    p.add_argument("name", help="Controller name")
    p.add_argument("state", choices=["active", "inactive"],
                   help="Target state: active or inactive")
    _add_cm_args(p)

    p = csub.add_parser("set-hardware-component-state",
                        help="Adjust the state of the hardware component")
    p.add_argument("name", help="Hardware component name")
    p.add_argument("state", choices=["unconfigured", "inactive", "active", "finalized"],
                   help="Target lifecycle state")
    _add_cm_args(p)

    p = csub.add_parser("switch-controllers",
                        help="Switch controllers in the controller manager")
    p.add_argument("--activate", nargs="*", default=[],
                   help="Controllers to activate")
    p.add_argument("--deactivate", nargs="*", default=[],
                   help="Controllers to deactivate")
    p.add_argument("--strictness", choices=["BEST_EFFORT", "STRICT"],
                   default="BEST_EFFORT",
                   help="Switching strictness (default: BEST_EFFORT)")
    p.add_argument("--activate-asap", dest="activate_asap", action="store_true",
                   default=False,
                   help="Activate controllers as soon as possible")
    _add_cm_args(p)

    p = csub.add_parser("view-controller-chains",
                        help="Generate a diagram of loaded chained controllers")
    p.add_argument("--output", default="controller_diagram.pdf",
                   help="Output filename saved in .artifacts/ (default: controller_diagram.pdf)")
    p.add_argument("--channel-id", dest="channel_id", default=None,
                   help="Discord channel ID; if provided, sends the PDF via discord_tools")
    p.add_argument("--config", default="~/.nanobot/config.json",
                   help="Path to nanobot config for Discord (default: ~/.nanobot/config.json)")
    _add_cm_args(p)

    # ------------------------------------------------------------------
    # params
    # ------------------------------------------------------------------
    params = sub.add_parser("params", help="Parameter operations")
    psub = params.add_subparsers(dest="subcommand")
    for _params_list_name in ("list", "ls"):
        p = psub.add_parser(_params_list_name,
                            help="List parameters for a node"
                            if _params_list_name == "list" else "Alias for list")
        p.add_argument("node", help="Node name (e.g. /turtlesim)")
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
    p = psub.add_parser("find", help="Search for parameters matching a pattern across all nodes")
    p.add_argument("pattern", help="Case-insensitive substring to match (use 'all' or '*' for everything)")
    p.add_argument("--node", default=None,
                   help="Limit search to a single node (e.g. /turtlesim)")
    p.add_argument("--timeout", type=float, default=10.0,
                   help="Timeout per node in seconds (default: 10)")
    p = psub.add_parser("preset-save", help="Save node parameters as a named preset")
    p.add_argument("node", help="Node name (e.g. /turtlesim)")
    p.add_argument("preset", help="Preset name (e.g. indoor)")
    p.add_argument("--timeout", type=float, default=5.0,
                   help="Timeout in seconds (default: 5)")
    p = psub.add_parser("preset-load", help="Restore a named preset onto a node")
    p.add_argument("node", help="Node name (e.g. /turtlesim)")
    p.add_argument("preset", help="Preset name (e.g. indoor)")
    p.add_argument("--timeout", type=float, default=5.0,
                   help="Timeout in seconds (default: 5)")
    psub.add_parser("preset-list", help="List all saved presets")
    p = psub.add_parser("preset-delete", help="Delete a saved preset")
    p.add_argument("preset", help="Preset name to delete")

    # ------------------------------------------------------------------
    # actions
    # ------------------------------------------------------------------
    actions = sub.add_parser("actions", help="Action operations")
    asub = actions.add_subparsers(dest="subcommand")
    asub.add_parser("list", help="List all actions")
    asub.add_parser("ls", help="Alias for list")
    p = asub.add_parser("details", help="Get action details")
    p.add_argument("action", help="Action server name (e.g. /turtle1/rotate_absolute)")
    p = asub.add_parser("info", help="Alias for details (ros2 action info)")
    p.add_argument("action", help="Action server name (e.g. /turtle1/rotate_absolute)")
    p = asub.add_parser("type", help="Get action server type")
    p.add_argument("action", nargs="?", help="Action server name (e.g. /turtle1/rotate_absolute)")
    p.add_argument("--timeout", type=float, default=5.0,
                   help="Timeout in seconds (default: 5)")
    p = asub.add_parser("send", help="Send action goal")
    p.add_argument("action", help="Action server name (e.g. /turtle1/rotate_absolute)")
    p.add_argument("goal", help="JSON goal")
    p.add_argument("--timeout", type=float, default=30.0,
                   help="Timeout in seconds (default: 30)")
    p.add_argument("--retries", type=int, default=None,
                   help="Number of attempts before giving up (overrides global --retries)")
    p.add_argument("--feedback", action="store_true", default=False,
                   help="Collect and return feedback messages alongside the result")
    p = asub.add_parser("cancel",
                        help="Cancel all in-flight goals on an action server")
    p.add_argument("action", nargs="?", help="Action server name (e.g. /turtle1/rotate_absolute)")
    p.add_argument("--timeout", type=float, default=5.0,
                   help="Timeout in seconds (default: 5)")
    p.add_argument("--retries", type=int, default=None,
                   help="Number of attempts before giving up (overrides global --retries)")
    p = asub.add_parser("echo", help="Echo action feedback and status messages")
    p.add_argument("action", help="Action server name (e.g. /turtle1/rotate_absolute)")
    p.add_argument("--duration", type=float, default=None,
                   help="Collect feedback for this many seconds "
                        "(default: wait for first message)")
    p.add_argument("--max-messages", "--max-msgs", dest="max_messages", type=int, default=100,
                   help="Max feedback messages to collect when using --duration (default: 100)")
    p.add_argument("--timeout", type=float, default=5.0,
                   help="Timeout waiting for first feedback in seconds (default: 5)")
    p = asub.add_parser("find", help="Find action servers by action type")
    p.add_argument("action_type", nargs="?",
                   help="Action type to search for (e.g. turtlesim/action/RotateAbsolute)")
    p.add_argument("--timeout", type=float, default=5.0,
                   help="Timeout in seconds (default: 5)")

    # ------------------------------------------------------------------
    # doctor / wtf
    # ------------------------------------------------------------------
    def _add_doctor_args(p):
        p.add_argument("--report", "-r", action="store_true", default=False,
                       help="Print all report sections in the output")
        p.add_argument("--report-failed", "-rf", dest="report_failed",
                       action="store_true", default=False,
                       help="Print report sections for failed checks only")
        p.add_argument("--exclude-packages", "-ep", dest="exclude_packages",
                       action="store_true", default=False,
                       help="Exclude package checks")
        p.add_argument("--include-warnings", "-iw", dest="include_warnings",
                       action="store_true", default=False,
                       help="Treat warnings as failures in the summary")

    def _add_doctor_subs(p):
        dsub = p.add_subparsers(dest="subcommand")
        hello = dsub.add_parser("hello",
                                help="Check network connectivity between multiple hosts")
        hello.add_argument("--topic", "-t", default="/canyouhearme",
                           help="Topic to publish/subscribe on (default: /canyouhearme)")
        hello.add_argument("--timeout", "-to", dest="timeout", type=float, default=10.0,
                           help="How long to listen for responses in seconds (default: 10.0)")

    doctor = sub.add_parser("doctor", help="Check ROS 2 system health")
    _add_doctor_args(doctor)
    _add_doctor_subs(doctor)

    wtf = sub.add_parser("wtf", help="Alias for doctor (Why The Failure)")
    _add_doctor_args(wtf)
    _add_doctor_subs(wtf)

    # ------------------------------------------------------------------
    # multicast
    # ------------------------------------------------------------------
    def _add_multicast_args(p):
        p.add_argument("--group", "-g", default="225.0.0.1",
                       help="Multicast group address (default: 225.0.0.1)")
        p.add_argument("--port", "-p", type=int, default=49150,
                       help="UDP port (default: 49150)")

    multicast = sub.add_parser("multicast", help="Send or receive UDP multicast packets")
    mcsub = multicast.add_subparsers(dest="subcommand")

    mc_send = mcsub.add_parser("send", help="Send one UDP multicast datagram")
    _add_multicast_args(mc_send)

    mc_recv = mcsub.add_parser("receive", help="Receive UDP multicast packets")
    _add_multicast_args(mc_recv)
    mc_recv.add_argument("--timeout", "-t", type=float, default=5.0,
                         help="How long to listen in seconds (default: 5.0)")

    # ------------------------------------------------------------------
    # tf
    # ------------------------------------------------------------------
    tf = sub.add_parser("tf", help="TF2 transform utilities")
    tfsub = tf.add_subparsers(dest="subcommand")

    # tf list
    tfsub.add_parser("list", help="List all coordinate frames")
    tfsub.add_parser("ls", help="Alias for list")

    # tf lookup <source> <target>
    p = tfsub.add_parser("lookup", help="Lookup transform between frames")
    p.add_argument("source", help="Source frame")
    p.add_argument("target", help="Target frame")
    p.add_argument("--timeout", "-t", type=float, default=5.0, help="Timeout in seconds")
    p = tfsub.add_parser("get", help="Alias for lookup")
    p.add_argument("source", help="Source frame")
    p.add_argument("target", help="Target frame")
    p.add_argument("--timeout", "-t", type=float, default=5.0, help="Timeout in seconds")

    # tf echo <source> <target>
    p = tfsub.add_parser("echo", help="Echo transform between frames")
    p.add_argument("source", help="Source frame")
    p.add_argument("target", help="Target frame")
    p.add_argument("--timeout", "-t", type=float, default=5.0, help="Timeout per lookup")
    p.add_argument("--count", "-n", type=int, default=5, help="Number of echos")
    p.add_argument("--once", action="store_true", help="Echo once (equivalent to --count 1)")

    # tf monitor <frame>
    p = tfsub.add_parser("monitor", help="Monitor transform updates for a frame")
    p.add_argument("frame", help="Frame to monitor")
    p.add_argument("--reference-frame", "-r", dest="reference_frame", default=None,
                   help="Reference frame to look up against (default: auto-discovered root frame via tf list)")
    p.add_argument("--timeout", "-t", type=float, default=5.0, help="Timeout per lookup")
    p.add_argument("--count", "-n", type=int, default=5, help="Number of updates")

    # tf static — supports both named (--from/--to/--xyz/--rpy) and positional (x y z roll pitch yaw from to)
    p = tfsub.add_parser("static", help="Publish static transform")
    p.add_argument("--from", dest="from_frame", default=None, help="Source frame (named form)")
    p.add_argument("--to", dest="to_frame", default=None, help="Target frame (named form)")
    p.add_argument("--xyz", nargs=3, type=float, metavar=("X", "Y", "Z"), default=None, help="Translation x y z")
    p.add_argument("--rpy", nargs=3, type=float, metavar=("R", "P", "Y"), default=None, help="Rotation roll pitch yaw (radians)")
    # positional fallback: x y z roll pitch yaw from_frame to_frame
    p.add_argument("pos_args", nargs="*", help="Positional: x y z roll pitch yaw from_frame to_frame")

    # tf euler-from-quaternion <x> <y> <z> <w>
    p = tfsub.add_parser("euler-from-quaternion", help="Convert quaternion to Euler (radians)")
    p.add_argument("x", type=float, help="Quaternion x")
    p.add_argument("y", type=float, help="Quaternion y")
    p.add_argument("z", type=float, help="Quaternion z")
    p.add_argument("w", type=float, help="Quaternion w")

    # tf quaternion-from-euler <roll> <pitch> <yaw>
    p = tfsub.add_parser("quaternion-from-euler", help="Convert Euler to quaternion (radians)")
    p.add_argument("roll", type=float, help="Euler roll (radians)")
    p.add_argument("pitch", type=float, help="Euler pitch (radians)")
    p.add_argument("yaw", type=float, help="Euler yaw (radians)")

    # tf euler-from-quaternion-deg <x> <y> <z> <w>
    p = tfsub.add_parser("euler-from-quaternion-deg", help="Convert quaternion to Euler (degrees)")
    p.add_argument("x", type=float, help="Quaternion x")
    p.add_argument("y", type=float, help="Quaternion y")
    p.add_argument("z", type=float, help="Quaternion z")
    p.add_argument("w", type=float, help="Quaternion w")

    # tf quaternion-from-euler-deg <roll> <pitch> <yaw>
    p = tfsub.add_parser("quaternion-from-euler-deg", help="Convert Euler to quaternion (degrees)")
    p.add_argument("roll", type=float, help="Euler roll (degrees)")
    p.add_argument("pitch", type=float, help="Euler pitch (degrees)")
    p.add_argument("yaw", type=float, help="Euler yaw (degrees)")

    # tf transform-point <target> <source> <x> <y> <z>
    p = tfsub.add_parser("transform-point", help="Transform a point between frames")
    p.add_argument("target", help="Target frame")
    p.add_argument("source", help="Source frame")
    p.add_argument("x", type=float, help="Point x")
    p.add_argument("y", type=float, help="Point y")
    p.add_argument("z", type=float, help="Point z")
    p.add_argument("--timeout", "-t", type=float, default=5.0, help="Timeout")

    # tf transform-vector <target> <source> <x> <y> <z>
    p = tfsub.add_parser("transform-vector", help="Transform a vector between frames")
    p.add_argument("target", help="Target frame")
    p.add_argument("source", help="Source frame")
    p.add_argument("x", type=float, help="Vector x")
    p.add_argument("y", type=float, help="Vector y")
    p.add_argument("z", type=float, help="Vector z")
    p.add_argument("--timeout", "-t", type=float, default=5.0, help="Timeout")

    # tf tree
    p = tfsub.add_parser("tree", help="Display TF frame tree as ASCII diagram")
    p.add_argument("--duration", "-d", type=float, default=2.0,
                   help="Seconds to collect TF data (default: 2.0)")

    # tf validate
    p = tfsub.add_parser("validate", help="Validate TF tree for cycles and multiple parents")
    p.add_argument("--duration", "-d", type=float, default=2.0,
                   help="Seconds to collect TF data (default: 2.0)")

    # ------------------------------------------------------------------
    # launch
    # ------------------------------------------------------------------
    launch = sub.add_parser("launch", help="Launch ROS 2 nodes and launch files")

    lsub = launch.add_subparsers(dest="subcommand")

    # launch new <package> <launch_file> [args...]
    p = lsub.add_parser("new", help="Run a ROS 2 launch file")
    p.add_argument("package", help="Package name containing the launch file")
    p.add_argument("launch_file", help="Launch file name (without path)")
    p.add_argument("args", nargs="*", help="Additional launch arguments (name:=value)")
    p.add_argument("--param", dest="params",
                   help="Inline parameters as comma-separated key:=value pairs "
                        "(e.g. --param use_sim_time:=true,robot_name:=my_robot)")
    p.add_argument("--config-path",
                   help="Path to a YAML config file or directory of YAML files; "
                        "files are forwarded to launched nodes via --ros-args --params-file. "
                        "When a directory is given, all .yaml/.yml files are loaded in "
                        "alphabetical order (last file wins on duplicate keys).")
    p.add_argument("--preset",
                   help="Parameter preset name to apply (saved via 'params preset-save'); "
                        "preset values have lower priority than explicit args and --param")
    p.add_argument("--timeout", type=float, default=30.0, help="Timeout for launch to start (default: 30)")

    p = lsub.add_parser("list", help="List running launch sessions, or search for launch files by keyword")
    p.add_argument("keyword", nargs="?", default=None,
                   help="Optional keyword to search installed packages for matching launch files "
                        "(use 'all' to list every launch file)")
    p = lsub.add_parser("ls", help="Alias for list")
    p.add_argument("keyword", nargs="?", default=None,
                   help="Optional keyword to search for launch files")

    p = lsub.add_parser("kill", help="Kill a running launch session")
    p.add_argument("session", help="Session name to kill")

    p = lsub.add_parser("restart", help="Restart a launch session (kill and re-launch)")
    p.add_argument("session", help="Session name to restart")

    # launch foxglove
    p = lsub.add_parser("foxglove", help="Launch foxglove_bridge")
    p.add_argument("port", nargs="?", type=int, default=8765, help="Foxglove bridge port (default: 8765)")

    # ------------------------------------------------------------------
    # run
    # ------------------------------------------------------------------
    run = sub.add_parser("run", help="Run ROS 2 executables in tmux sessions")

    rsub = run.add_subparsers(dest="subcommand")

    # run new <package> <executable> [args...]
    p = rsub.add_parser("new", help="Run a ROS 2 executable")
    p.add_argument("package", help="Package name containing the executable")
    p.add_argument("executable", help="Executable name")
    p.add_argument("args", nargs="*", help="Additional arguments")
    p.add_argument("--presets", help="Preset parameters to load before running (comma-separated)")
    p.add_argument("--params", help="Parameters to set (comma-separated key:=value or key:value pairs)")
    p.add_argument("--config-path", help="Path to config directory (auto-discovered if not provided)")

    rsub.add_parser("list", help="List running run sessions")
    rsub.add_parser("ls", help="Alias for list")

    p = rsub.add_parser("kill", help="Kill a running run session")
    p.add_argument("session", help="Session name to kill")

    p = rsub.add_parser("restart", help="Restart a run session")
    p.add_argument("session", help="Session name to restart")

    # ------------------------------------------------------------------
    # interface
    # ------------------------------------------------------------------
    iface = sub.add_parser("interface", help="Interface type discovery (ros2 interface)")
    ifsub = iface.add_subparsers(dest="subcommand")

    ifsub.add_parser("list", help="List all installed interface types (msgs, srvs, actions)")
    ifsub.add_parser("ls",   help="Alias for list")

    p = ifsub.add_parser("show", help="Show field structure of a message, service, or action")
    p.add_argument("type_str", metavar="type",
                   help="Interface type, e.g. std_msgs/msg/String, std_srvs/srv/SetBool, "
                        "nav2_msgs/action/NavigateToPose, or shorthand std_msgs/String")

    p = ifsub.add_parser("proto", help="Show default-value prototype of a message, service, or action")
    p.add_argument("type_str", metavar="type",
                   help="Interface type, e.g. std_msgs/msg/String, geometry_msgs/msg/Twist")

    ifsub.add_parser("packages", help="List packages that contain at least one interface")

    p = ifsub.add_parser("package", help="List all interface types for a single package")
    p.add_argument("package", help="Package name (e.g. std_msgs, geometry_msgs)")

    # ------------------------------------------------------------------
    # bag
    # ------------------------------------------------------------------
    bag = sub.add_parser("bag", help="ROS 2 bag file utilities")
    bagsub = bag.add_subparsers(dest="subcommand")

    p = bagsub.add_parser("info", help="Show metadata for a ROS 2 bag (no graph required)")
    p.add_argument("bag_path", metavar="bag_path",
                   help="Path to a bag directory, metadata.yaml, or storage file")

    # ------------------------------------------------------------------
    # component
    # ------------------------------------------------------------------
    comp = sub.add_parser("component", help="ROS 2 composable node utilities")
    compsub = comp.add_subparsers(dest="subcommand")

    compsub.add_parser("types", help="List all registered rclcpp composable node types (no graph required)")
    p = compsub.add_parser("list", help="List all running component containers and their loaded components")
    p.add_argument("--timeout", type=float, default=5.0, dest="timeout",
                   help="Seconds to wait per container service (default: 5.0)")
    p = compsub.add_parser("ls", help="Alias for list")
    p.add_argument("--timeout", type=float, default=5.0, dest="timeout",
                   help="Seconds to wait per container service (default: 5.0)")
    p = compsub.add_parser("load", help="Load a composable node into a component container")
    p.add_argument("container",    help="Container node name (e.g. /my_container)")
    p.add_argument("package_name", help="Package containing the plugin (e.g. demo_nodes_cpp)")
    p.add_argument("plugin_name",  help="Fully-qualified plugin class name (e.g. demo_nodes_cpp::Talker)")
    p.add_argument("--node-name",      dest="node_name",      default="", help="Override the loaded node's name")
    p.add_argument("--node-namespace", dest="node_namespace", default="", help="Override the loaded node's namespace")
    p.add_argument("--remap", dest="remap_rules", nargs="*", default=[], help="Remap rules (e.g. /from:=/to)")
    p.add_argument("--log-level",      dest="log_level",      type=int, default=0, help="Log level for the loaded node (uint8: 0=unset, 10=DEBUG, 20=INFO, 30=WARN, 40=ERROR, 50=FATAL)")
    p.add_argument("--timeout", type=float, default=5.0, dest="timeout",  help="Service call timeout in seconds (default: 5.0)")
    p = compsub.add_parser("unload", help="Unload a composable node from a component container")
    p.add_argument("container",  help="Container node name (e.g. /my_container)")
    p.add_argument("unique_id",  type=int, help="Unique ID of the component to unload (from component load or component list)")
    p.add_argument("--timeout",  type=float, default=5.0, dest="timeout", help="Service call timeout in seconds (default: 5.0)")
    p = compsub.add_parser("kill", help="Kill a standalone component container session (comp_* sessions)")
    p.add_argument("session", help="Session name to kill (e.g. comp_demo_nodes_cpp_standalone_talker)")
    p = compsub.add_parser(
        "standalone",
        help="Run a composable node in its own standalone container (tmux session)",
    )
    p.add_argument("package_name", help="Package containing the component plugin")
    p.add_argument("plugin_name",  help="Fully-qualified plugin class name (e.g. demo_nodes_cpp::Talker)")
    p.add_argument("--container-type", dest="container_type",
                   choices=["component_container", "component_container_mt", "component_container_isolated"],
                   default="component_container",
                   help="Container executable to use (default: component_container)")
    p.add_argument("--node-name",      dest="node_name",      default="",
                   help="Override the loaded node's name")
    p.add_argument("--node-namespace", dest="node_namespace", default="",
                   help="Override the loaded node's namespace")
    p.add_argument("--remap",          dest="remap_rules",    nargs="*", default=[],
                   help="Remap rules (e.g. /from:=/to)")
    p.add_argument("--log-level",      dest="log_level",      type=int, default=0,
                   help="Log level for the loaded node (uint8: 0=unset, 10=DEBUG, 20=INFO, 30=WARN, 40=ERROR, 50=FATAL)")
    p.add_argument("--timeout",        dest="timeout", type=float, default=10.0,
                   help="Total timeout for container start + component load (default: 10.0s)")

    # ------------------------------------------------------------------
    # daemon
    # ------------------------------------------------------------------
    daemon = sub.add_parser("daemon", help="ROS 2 daemon lifecycle management")
    daemonsub = daemon.add_subparsers(dest="subcommand")
    daemonsub.add_parser("status", help="Check whether the ROS 2 daemon is running")
    daemonsub.add_parser("start",  help="Start the ROS 2 daemon")
    daemonsub.add_parser("stop",   help="Stop the ROS 2 daemon")

    # ------------------------------------------------------------------
    # pkg
    # ------------------------------------------------------------------
    pkg = sub.add_parser("pkg", help="ROS 2 package utilities (no graph required)")
    pkgsub = pkg.add_subparsers(dest="subcommand")

    pkgsub.add_parser("list", help="List all installed packages")
    pkgsub.add_parser("ls",   help="Alias for list")

    p = pkgsub.add_parser("prefix", help="Output the prefix path of a package")
    p.add_argument("package", help="Package name (e.g. nav2_bringup)")

    p = pkgsub.add_parser("executables", help="List executables provided by a package")
    p.add_argument("package", help="Package name (e.g. turtlesim)")

    p = pkgsub.add_parser("xml", help="Output the package.xml of a package")
    p.add_argument("package", help="Package name (e.g. std_msgs)")

    p = pkgsub.add_parser("create", help="Create a new ROS 2 package scaffold")
    p.add_argument("package_name", help="Name of the new package")
    p.add_argument("--build-type",
                   choices=["ament_cmake", "ament_python", "cmake"],
                   default=None,
                   help="Build system type (default: ament_cmake)")
    p.add_argument("--dependencies", nargs="*", metavar="DEP",
                   help="Package dependencies to declare (space-separated)")
    p.add_argument("--destination-directory", metavar="DIR",
                   help="Directory to create the package in (default: current directory)")
    p.add_argument("--license", metavar="SPDX",
                   help="SPDX license identifier (e.g. Apache-2.0, MIT)")
    p.add_argument("--maintainer-name", metavar="NAME",
                   help="Maintainer name for package.xml")
    p.add_argument("--maintainer-email", metavar="EMAIL",
                   help="Maintainer email for package.xml")
    p.add_argument("--description", metavar="TEXT",
                   help="One-line package description for package.xml")
    p.add_argument("--node-name", metavar="NODE", dest="node_name", default=None,
                   help="Node name to use in generated source stubs (ament_cmake / ament_python)")
    p.add_argument("--library-name", metavar="LIB", dest="library_name", default=None,
                   help="Library name to use in generated CMakeLists.txt stubs (ament_cmake only)")

    # -------------------------------------------------------------------------
    # logs
    # -------------------------------------------------------------------------
    logs = sub.add_parser(
        "logs",
        help="ROS 2 log file introspection — no graph required",
    )
    logssub = logs.add_subparsers(dest="subcommand")

    # list-runs
    p = logssub.add_parser(
        "list-runs",
        help="List available log runs newest-first",
    )
    p.add_argument(
        "--limit", type=int, default=20, metavar="N",
        help="Max runs to return (default: 20, 0 = no limit)",
    )
    p.add_argument(
        "--log-dir", dest="log_dir", default=None, metavar="DIR",
        help="Override log directory (default: ~/.ros/log or $ROS_LOG_DIR)",
    )

    # query
    p = logssub.add_parser(
        "query",
        help="Filter log entries by severity, node, time, text",
    )
    p.add_argument(
        "--run", default=None, metavar="RUN_ID",
        help="Run ID to query (default: latest)",
    )
    p.add_argument(
        "--severity", default="DEBUG",
        choices=["DEBUG", "INFO", "WARN", "ERROR", "FATAL"],
        help="Minimum severity level (default: DEBUG = all)",
    )
    p.add_argument(
        "--node", default=None, metavar="NAME",
        help="Filter by node name (substring match)",
    )
    p.add_argument(
        "--after", default=None, metavar="TIME",
        help="Lower time bound: -30s | -5m | -1h | epoch-float | ISO datetime",
    )
    p.add_argument(
        "--before", default=None, metavar="TIME",
        help="Upper time bound: same formats as --after",
    )
    p.add_argument(
        "--text", default=None, metavar="SUBSTR",
        help="Plain-text substring to match in message",
    )
    p.add_argument(
        "--regex", default=None, metavar="PATTERN",
        help="Python regex to match in message (case-insensitive)",
    )
    p.add_argument(
        "--max", type=int, default=200, metavar="N",
        help="Max entries to return (default: 200)",
    )
    p.add_argument(
        "--log-dir", dest="log_dir", default=None, metavar="DIR",
        help="Override log directory",
    )

    # tail
    p = logssub.add_parser(
        "tail",
        help="Incremental tail — new entries since last call",
    )
    p.add_argument(
        "--run", default=None, metavar="RUN_ID",
        help="Run ID to tail (default: latest)",
    )
    p.add_argument(
        "--initial-lines", dest="initial_lines", type=int, default=50,
        metavar="N",
        help="Lines from end to return on first call (default: 50)",
    )
    p.add_argument(
        "--reset", action="store_true",
        help="Discard saved offsets and re-seed from the tail of the file",
    )
    p.add_argument(
        "--log-dir", dest="log_dir", default=None, metavar="DIR",
        help="Override log directory",
    )

    # node-summary
    p = logssub.add_parser(
        "node-summary",
        help="Per-node statistics: totals, severity breakdown, top messages",
    )
    p.add_argument(
        "--run", default=None, metavar="RUN_ID",
        help="Run ID to analyse (default: latest)",
    )
    p.add_argument(
        "--top", type=int, default=5, metavar="N",
        help="Number of top recurring message patterns to return per node "
             "(default: 5)",
    )
    p.add_argument(
        "--log-dir", dest="log_dir", default=None, metavar="DIR",
        help="Override log directory",
    )

    return parser


# ---------------------------------------------------------------------------
# Dispatch table
# ---------------------------------------------------------------------------

DISPATCH = {
    ("version", None): cmd_version,
    ("context", None): cmd_context,
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
    ("topics", "hz"): cmd_topics_hz,
    ("topics", "find"): cmd_topics_find,
    ("topics", "capture-image"): cmd_topics_capture_image,
    ("topics", "diag-list"): cmd_topics_diag_list,
    ("topics", "diag"): cmd_topics_diag,
    ("topics", "battery-list"): cmd_topics_battery_list,
    ("topics", "battery"): cmd_topics_battery,
    # topics — aliases
    ("topics", "echo"): cmd_topics_subscribe,
    ("topics", "pub"): cmd_topics_publish,
    ("topics", "ls"): cmd_topics_list,
    ("topics", "info"): cmd_topics_details,
    # topics — Phase 2
    ("topics", "bw"): cmd_topics_bw,
    ("topics", "delay"): cmd_topics_delay,
    ("topics", "qos-check"): cmd_topics_qos_check,
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
    # control — canonical
    ("control", "list-controller-types"):        cmd_control_list_controller_types,
    ("control", "list-controllers"):             cmd_control_list_controllers,
    ("control", "list-hardware-components"):     cmd_control_list_hardware_components,
    ("control", "list-hardware-interfaces"):     cmd_control_list_hardware_interfaces,
    ("control", "load-controller"):              cmd_control_load_controller,
    ("control", "unload-controller"):            cmd_control_unload_controller,
    ("control", "configure-controller"):         cmd_control_configure_controller,
    ("control", "reload-controller-libraries"):  cmd_control_reload_controller_libraries,
    ("control", "set-controller-state"):         cmd_control_set_controller_state,
    ("control", "set-hardware-component-state"): cmd_control_set_hardware_component_state,
    ("control", "switch-controllers"):           cmd_control_switch_controllers,
    ("control", "view-controller-chains"):       cmd_control_view_controller_chains,
    # control — aliases
    ("control", "load"):  cmd_control_load_controller,
    ("control", "unload"): cmd_control_unload_controller,
    # params — canonical
    ("params", "list"): cmd_params_list,
    ("params", "get"): cmd_params_get,
    ("params", "set"): cmd_params_set,
    ("params", "describe"): cmd_params_describe,
    ("params", "dump"): cmd_params_dump,
    ("params", "load"): cmd_params_load,
    ("params", "delete"): cmd_params_delete,
    ("params", "preset-save"):   cmd_params_preset_save,
    ("params", "preset-load"):   cmd_params_preset_load,
    ("params", "preset-list"):   cmd_params_preset_list,
    ("params", "preset-delete"): cmd_params_preset_delete,
    ("params", "find"): cmd_params_find,
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
    ("actions", "ls"): cmd_actions_list,
    # doctor — canonical
    ("doctor", None):    cmd_doctor_check,
    ("doctor", "hello"): cmd_doctor_hello,
    # wtf — alias for doctor
    ("wtf",    None):    cmd_doctor_check,
    ("wtf",    "hello"): cmd_doctor_hello,
    # multicast
    ("multicast", "send"):    cmd_multicast_send,
    ("multicast", "receive"): cmd_multicast_receive,
    # tf
    ("tf", "list"): cmd_tf_list,
    ("tf", "ls"): cmd_tf_list,
    ("tf", "lookup"): cmd_tf_lookup,
    ("tf", "get"): cmd_tf_lookup,
    ("tf", "echo"): cmd_tf_echo,
    ("tf", "monitor"): cmd_tf_monitor,
    ("tf", "static"): cmd_tf_static,
    ("tf", "euler-from-quaternion"): cmd_tf_euler_from_quaternion,
    ("tf", "quaternion-from-euler"): cmd_tf_quaternion_from_euler,
    ("tf", "euler-from-quaternion-deg"): cmd_tf_euler_from_quaternion_degrees,
    ("tf", "quaternion-from-euler-deg"): cmd_tf_quaternion_from_euler_degrees,
    ("tf", "transform-point"): cmd_tf_transform_point,
    ("tf", "transform-vector"): cmd_tf_transform_vector,
    ("tf", "tree"): cmd_tf_tree,
    ("tf", "validate"): cmd_tf_validate,
    # launch
    ("launch", "new"):   cmd_launch_run,
    ("launch", "list"):   cmd_launch_list,
    ("launch", "ls"):    cmd_launch_list,
    ("launch", "kill"):  cmd_launch_kill,
    ("launch", "restart"): cmd_launch_restart,
    ("launch", "foxglove"): cmd_launch_foxglove,
    # run
    ("run", "new"):    cmd_run,
    ("run", "list"):   cmd_run_list,
    ("run", "ls"):    cmd_run_list,
    ("run", "kill"):  cmd_run_kill,
    ("run", "restart"): cmd_run_restart,
    # interface
    ("interface", "list"):     cmd_interface_list,
    ("interface", "ls"):       cmd_interface_list,
    ("interface", "show"):     cmd_interface_show,
    ("interface", "proto"):    cmd_interface_proto,
    ("interface", "packages"): cmd_interface_packages,
    ("interface", "package"):  cmd_interface_package,
    # bag
    ("bag", "info"): cmd_bag_info,
    # component
    ("component", "types"): cmd_component_types,
    ("component", "list"):  cmd_component_list,
    ("component", "ls"):    cmd_component_list,
    ("component", "load"):   cmd_component_load,
    ("component", "unload"): cmd_component_unload,
    ("component", "kill"):      cmd_component_kill,
    ("component", "standalone"): cmd_component_standalone,
    # pkg
    ("pkg", "list"):        cmd_pkg_list,
    ("pkg", "ls"):          cmd_pkg_list,
    ("pkg", "prefix"):      cmd_pkg_prefix,
    ("pkg", "executables"): cmd_pkg_executables,
    ("pkg", "xml"):         cmd_pkg_xml,
    ("pkg", "create"):      cmd_pkg_create,
    # daemon
    ("daemon", "status"): cmd_daemon_status,
    ("daemon", "start"):  cmd_daemon_start,
    ("daemon", "stop"):   cmd_daemon_stop,
    # logs
    ("logs", "list-runs"):    cmd_logs_list_runs,
    ("logs", "query"):        cmd_logs_query,
    ("logs", "tail"):         cmd_logs_tail,
    ("logs", "node-summary"): cmd_logs_node_summary,
}


# ---------------------------------------------------------------------------
# Global override helpers
# ---------------------------------------------------------------------------

def _apply_global_overrides(args):
    """Apply top-level ``--timeout`` / ``--retries`` to the args namespace.

    ``--timeout`` (stored as ``args.global_timeout``) overrides whatever
    per-command ``--timeout`` default the subparser set.  This lets callers
    write ``ros2_cli.py --timeout 30 services call …`` to raise the budget
    for a single invocation without editing every subcommand.

    ``--retries``: some subparsers (services call, actions send/cancel) also
    define ``--retries`` with ``default=None`` so the flag is accepted when
    placed *after* the subcommand.  When the subparser value is None it means
    the user did not pass the flag there, so we fall back to the global value
    from the main parser (which defaults to 1).
    """
    if getattr(args, "global_timeout", None) is not None and hasattr(args, "timeout"):
        args.timeout = args.global_timeout
    # Resolve retries: subparsers that accept --retries use default=None so the
    # flag works when placed *after* the subcommand.  When it is None (not given
    # after the subcommand), fall back to the global value from the main parser
    # (global_retries, default 1).  A subparser value of non-None takes priority.
    global_retries = getattr(args, "global_retries", 1)
    if getattr(args, "retries", None) is None:
        args.retries = global_retries


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main():
    parser = build_parser()
    args = parser.parse_args()
    _apply_global_overrides(args)

    if not args.command:
        parser.print_help()
        sys.exit(1)

    command = args.command
    subcommand = getattr(args, "subcommand", None)

    key = (command, subcommand)
    handler = DISPATCH.get(key)
    if handler:
        handler(args)
    else:
        parser.print_help()
        sys.exit(1)


if __name__ == "__main__":
    main()
