#!/usr/bin/env python3
"""ROS 2 component commands.

Implements component types by reading from the ament resource index.
No rclpy.init() or running ROS 2 graph required — reads from the filesystem.

Other subcommands (list, load, unload) rely on the composition_interfaces
service API, which is not reliably discoverable via rclpy on all RMW
implementations; they will be added when subprocess delegation is permitted.
"""

from ros2_utils import output


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
