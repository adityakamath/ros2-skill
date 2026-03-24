#!/usr/bin/env python3
"""ROS 2 component commands.

Implements component types by reading from the ament resource index.
No rclpy.init() or running ROS 2 graph required — reads from the filesystem.

Other subcommands (list, load, unload) rely on the composition_interfaces
service API, which is not reliably discoverable via rclpy on all RMW
implementations; they will be added when subprocess delegation is permitted.
"""

from ros2_utils import output, ROS2CLI, ros2_context


# ---------------------------------------------------------------------------
# Commands
# ---------------------------------------------------------------------------

def cmd_component_types(args):
    """List all registered rclcpp composable node types installed on this system.

    Reads the 'rclcpp_components' ament resource type from the ament index.
    Each package that exports composable nodes registers a resource file whose
    lines are component class names (e.g. 'my_pkg::MyNode').

    No rclpy.init() or live ROS 2 graph required.
    """
    try:
        from ament_index_python import get_resources, get_resource
    except ImportError as exc:
        output({
            "error": "ament_index_python is required: pip install ament-index-python",
            "detail": str(exc),
        })
        return

    try:
        # Returns {package_name: resource_prefix_path} for every package that
        # registers the 'rclcpp_components' resource type.
        packages = get_resources("rclcpp_components")
    except Exception as exc:
        output({"error": f"Failed to query ament index: {exc}"})
        return

    components = []
    errors = []

    for pkg_name in sorted(packages.keys()):
        try:
            content, _ = get_resource("rclcpp_components", pkg_name)
            for line in content.splitlines():
                line = line.strip()
                # Skip blank lines and comments.
                if line and not line.startswith("#"):
                    components.append({
                        "package":   pkg_name,
                        "type_name": line,
                    })
        except Exception as exc:
            # Non-fatal: log and continue so one broken package doesn't hide others.
            errors.append({"package": pkg_name, "error": str(exc)})

    result = {
        "components": components,
        "total":      len(components),
        "packages":   sorted({c["package"] for c in components}),
    }
    if errors:
        result["warnings"] = errors

    output(result)


def cmd_component_list(args):
    """List all running component containers and their loaded components.

    Discovers containers by scanning for composition_interfaces/srv/ListNodes
    services on the live graph, then calls each one.
    """
    import time as _time
    import rclpy as _rclpy

    timeout = getattr(args, 'timeout', 5.0)

    try:
        from composition_interfaces.srv import ListNodes
    except ImportError:
        output({
            "error": "composition_interfaces is not installed",
            "hint": "Install with: sudo apt install ros-$ROS_DISTRO-composition-interfaces",
        })
        return

    try:
        with ros2_context():
            node = ROS2CLI()

            # Discover all containers by finding ListNodes services
            all_services = node.get_service_names_and_types()
            containers = []
            for svc_name, svc_types in all_services:
                if "composition_interfaces/srv/ListNodes" in svc_types:
                    if svc_name.endswith("/list_nodes"):
                        containers.append(svc_name[: -len("/list_nodes")])

            if not containers:
                output({
                    "containers": [],
                    "total_containers": 0,
                    "total_components": 0,
                    "hint": "No component containers found. Start one with: ros2 run rclcpp_components component_container",
                })
                return

            results = []
            for container in sorted(containers):
                svc_name = f"{container}/list_nodes"
                client = node.create_client(ListNodes, svc_name)

                if not client.wait_for_service(timeout_sec=timeout):
                    results.append({
                        "container": container,
                        "error": f"Service {svc_name} not available within {timeout}s",
                        "components": [],
                    })
                    continue

                future = client.call_async(ListNodes.Request())
                end = _time.time() + timeout
                while _time.time() < end and not future.done():
                    _rclpy.spin_once(node, timeout_sec=0.1)

                if not future.done():
                    results.append({
                        "container": container,
                        "error": "ListNodes call timed out",
                        "components": [],
                    })
                    continue

                resp = future.result()
                components = [
                    {"unique_id": uid, "full_node_name": name}
                    for uid, name in zip(resp.unique_ids, resp.full_node_names)
                ]
                results.append({
                    "container": container,
                    "component_count": len(components),
                    "components": components,
                })

        total_components = sum(
            len(r.get("components", [])) for r in results
        )
        output({
            "containers": results,
            "total_containers": len(results),
            "total_components": total_components,
        })

    except Exception as exc:
        output({"error": str(exc)})


if __name__ == "__main__":
    import sys
    import os
    _mod = os.path.basename(__file__)
    _cli = os.path.join(os.path.dirname(os.path.abspath(__file__)), "ros2_cli.py")
    print(
        f"[ros2-skill] '{_mod}' is an internal module — do not run it directly.\n"
        "Use the main entry point:\n"
        f"  python3 {_cli} <command> [subcommand] [args]\n"
        f"See all commands:  python3 {_cli} --help",
        file=sys.stderr,
    )
    sys.exit(1)
