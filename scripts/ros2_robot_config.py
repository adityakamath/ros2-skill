#!/usr/bin/env python3
"""Robot-specific facts from config YAML that the generic profile scan doesn't capture.

Pure functions (no ROS, no I/O besides the injected ``load_yaml``). Each key is omitted when the
robot's configs don't declare it:

- ``estop_config``: idempotent SetBool e-stop service from a toggle config (``bool_toggle_node`` style)
- ``robot_services``: services bound to joystick buttons (record_waypoint, save_map, ...)
- ``velocity_chain``: twist-switch inputs/output and the mode service
- ``hardware_fitted``: boolean switches from a ``hardware.yaml``
"""

import pathlib

_ESTOP_NAMES = ("emergency_stop", "estop", "e_stop")


def _slash(name):
    return name if name.startswith("/") else "/" + name


def _params_blocks(data):
    """Yield every ``ros__parameters`` dict in a parsed ROS params YAML."""
    if isinstance(data, dict):
        for key, value in data.items():
            if key == "ros__parameters" and isinstance(value, dict):
                yield value
            else:
                yield from _params_blocks(value)


def extract_robot_config(yaml_files, load_yaml):
    out = {}
    toggles, services, chain = {}, {}, {}
    teleop, scales = None, {}
    for path in yaml_files:
        fname = pathlib.Path(path).name
        data = load_yaml(path)
        if not isinstance(data, dict):
            continue
        if fname == "hardware.yaml":
            flags = {k: v for k, v in data.items() if isinstance(v, bool)}
            if flags:
                out["hardware_fitted"] = flags
        for block in _params_blocks(data):
            names = block.get("toggles")
            if isinstance(names, list):
                for name in names:
                    cfg = block.get(name)
                    if isinstance(cfg, dict) and cfg.get("target_service"):
                        toggles[str(name)] = {
                            "service": _slash(cfg["target_service"]),
                            "toggle_service": _slash(cfg["trigger_service"]) if cfg.get("trigger_service") else None,
                        }
            for key, value in block.items():
                if isinstance(value, dict) and value.get("type") == "service" and value.get("service_name"):
                    entry = {"service": _slash(value["service_name"]), "type": value.get("interface_type")}
                    if isinstance(value.get("service_request"), dict):
                        entry["request"] = value["service_request"]
                    services[str(key)] = entry
            for key, value in block.items():
                if isinstance(value, dict) and value.get("type") == "topic" and value.get("topic_name") \
                        and isinstance(value.get("axis_mappings"), dict) and teleop is None:
                    teleop = {"config_file": fname, "cmd_vel_topic": _slash(value["topic_name"]),
                              "msg_type": value.get("interface_type")}
                    if value.get("deadman_buttons"):
                        teleop["deadman_buttons"] = value["deadman_buttons"]
                    for axis, name in (("twist-linear-x", "linear_x"), ("twist-linear-y", "linear_y"),
                                       ("twist-angular-z", "angular_z")):
                        scale = (value["axis_mappings"].get(axis) or {}).get("scale")
                        if isinstance(scale, (int, float)) and scale:
                            scales[name] = abs(float(scale))
            if "input_topic_default" in block and "output_topic" in block:
                chain = {"teleop_input": _slash(block["input_topic_default"]),
                         "nav2_input": _slash(block.get("input_topic_switched", "")) if block.get("input_topic_switched") else None,
                         "output": _slash(block["output_topic"])}
    for name, cfg in toggles.items():
        if name in _ESTOP_NAMES:
            out["estop_config"] = {"service_name": cfg["service"], "service_type": "std_srvs/srv/SetBool",
                                   "toggle_service": cfg["toggle_service"], "source": "toggles"}
    if services:
        out["robot_services"] = services
    if chain:
        if "twist_switch" in toggles:
            chain["mode_service"] = toggles["twist_switch"]["service"]  # SetBool: true = Nav2 input, false = teleop input
        out["velocity_chain"] = {k: v for k, v in chain.items() if v}
    if teleop:
        if scales:
            teleop["scales"] = scales
            out["teleop_limits"] = {"sources": [dict(file=teleop["config_file"], **scales)], "binding": dict(scales)}
        out["teleop_config"] = teleop
    if toggles:
        out["toggles"] = {k: v["service"] for k, v in toggles.items()}
    return out
