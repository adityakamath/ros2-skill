#!/usr/bin/env python3
"""ROS 2 node commands."""

import rclpy

from ros2_utils import ROS2CLI, output


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
