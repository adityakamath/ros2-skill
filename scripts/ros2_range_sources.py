#!/usr/bin/env python3
"""What the robot can sense obstacles with, for the profile (``summary.obstacle_sensing``).

Built from the workspace (lidar and Nav2 YAML, launch remappings, Python publishers, footprint radius, collision
monitor), then confirmed and completed from the running graph once. Pure: no ROS.
"""

import math
import pathlib
import re

VERSION = 1
SCAN, CLOUD, DEPTH = "scan", "pointcloud", "depth"
MSG_TYPES = {SCAN: "sensor_msgs/msg/LaserScan", CLOUD: "sensor_msgs/msg/PointCloud2", DEPTH: "sensor_msgs/msg/Image"}

# Topic name tokens that mark map-like, visualisation or re-encoded data rather than a sensor.
_NOT_SENSOR = frozenset({"costmap", "map", "voxel", "voxels", "particle", "particles", "marker", "markers", "octomap",
                         "rtabmap", "global", "plan", "compressed", "compresseddepth", "debug", "mask", "goal",
                         "trajectory", "cost", "matched", "accumulated"})
_NOT_SENSOR_NAMES = frozenset({"cloud_registered", "cloud_registered_body"})   # odometry output, not a sensor view
_STEREO_DEPTH = re.compile(r"(^|/)stereo/image(_raw|_rect_raw|_rect)?$")   # e.g. depthai: /oak/stereo/image_raw
_CLOUD_NAMES = ("points", "pointcloud", "point_cloud", "cloud", "pointcloud2")
# Clouds computed from other data: never added just because the running graph publishes them.
_DERIVED_CLOUD = frozenset({"obstacle", "obstacles", "ground", "nonground", "scan", "clusters", "segmented", "removed",
                            "filtered"})

_REMAP_BLOCK = re.compile(r"remappings\s*=\s*\[(.*?)\]", re.S)
_PAIR = re.compile(r"""\(\s*['"]([^'"]+)['"]\s*,\s*f?['"]([^'"{}]+)['"]\s*\)""")
_PUBLISHER = re.compile(r"""create_publisher\(\s*(?:[\w.]+\.)?(PointCloud2|LaserScan|Image)\s*,\s*['"]([^'"{}]+)['"]""")


def normalise(topic):
    return "/" + str(topic).strip().strip("/")


def looks_like_sensor(topic):
    segments = normalise(topic).split("/")[1:]
    if not segments or any(s.startswith(("_", "~")) for s in segments) or segments[-1].lower() in _NOT_SENSOR_NAMES:
        return False
    tokens = {t for s in segments for t in s.lower().split("_")}
    return not (tokens & _NOT_SENSOR)


def _is_depth_image(topic):
    low = topic.lower()
    if _STEREO_DEPTH.search(low):
        return True
    return "depth" in low and low.rsplit("/", 1)[-1] in ("image_raw", "image", "image_rect_raw", "image_rect")


def _source(kind, topic, origin):
    source = {"type": kind, "topic": normalise(topic), "msg_type": MSG_TYPES[kind], "from": origin}
    if kind == DEPTH:
        source["info_topic"] = source["topic"].rsplit("/", 1)[0] + "/camera_info"
    return source


def _tidy(sources):
    """Unique by topic, no ``_raw`` scan or cloud when the filtered sibling is listed, LiDAR first."""
    unique = {}
    for source in sources:
        unique.setdefault(source["topic"], source)
    for topic, source in list(unique.items()):
        if source["type"] != DEPTH and topic.endswith("_raw") and topic[:-4] in unique:
            unique.pop(topic)
    order = {SCAN: 0, CLOUD: 1, DEPTH: 2}
    return sorted(unique.values(), key=lambda s: (order[s["type"]], s["topic"]))


def _param_blocks(data, name=""):
    """Yield ``(node_name, params)`` for every ``ros__parameters`` block of a parsed params file."""
    if not isinstance(data, dict):
        return
    for key, value in data.items():
        if key == "ros__parameters" and isinstance(value, dict):
            yield name, value
        elif isinstance(value, dict):
            yield from _param_blocks(value, str(key).rstrip("/").rsplit("/", 1)[-1])


def _observation_sources(params):
    """``(kind, topic)`` for every ``observation_sources`` entry at any depth (collision monitor, costmap layers)."""
    if not isinstance(params, dict):
        return
    names = params.get("observation_sources")
    if isinstance(names, str):
        names = names.split()
    for name in names or []:
        block = params.get(name)
        if not isinstance(block, dict) or not block.get("topic") or block.get("enabled") is False:
            continue
        declared = str(block.get("type") or block.get("data_type") or "scan").lower()
        if declared in ("scan", "laserscan"):
            yield SCAN, block["topic"]
        elif declared in ("pointcloud", "pointcloud2"):
            yield CLOUD, block["topic"]
    for value in params.values():
        if isinstance(value, dict):
            yield from _observation_sources(value)


def _footprint_radius(params):
    values = []
    radius = params.get("robot_radius")
    if isinstance(radius, (int, float)) and radius > 0:
        values.append(float(radius))
    footprint = params.get("footprint")
    if isinstance(footprint, str):
        pairs = re.findall(r"\[\s*(-?[\d.eE+-]+)\s*,\s*(-?[\d.eE+-]+)\s*\]", footprint)
        try:
            corners = [math.hypot(float(x), float(y)) for x, y in pairs]
        except ValueError:
            corners = []
        if len(corners) >= 3:
            values.append(max(corners))
    return values


def _collision_monitor(node, params):
    polygons = [p for p in params.get("polygons") or [] if isinstance(p, str)]
    topics = {"cmd_vel_in": params.get("cmd_vel_in_topic", "cmd_vel_smoothed"),
              "cmd_vel_out": params.get("cmd_vel_out_topic", "cmd_vel"),
              "state_topic": params.get("state_topic", "collision_monitor_state")}
    monitor = {"node": node, **{k: normalise(v) for k, v in topics.items() if v}}
    if polygons:
        monitor["polygons"] = polygons
        monitor["enabled"] = [p for p in polygons if (params.get(p) or {}).get("enabled", True) is not False]
    return monitor


def from_source_text(text):
    """Range topics declared in one Python file: publishers, and launch remappings onto point cloud names."""
    found = []
    for message, topic in _PUBLISHER.findall(text):
        if topic.startswith("~") or not looks_like_sensor(topic):
            continue
        if message == "Image":
            if _is_depth_image(normalise(topic)):
                found.append(_source(DEPTH, topic, "source"))
        else:
            found.append(_source(SCAN if message == "LaserScan" else CLOUD, topic, "source"))
    for block in _REMAP_BLOCK.findall(text):
        for original, target in _PAIR.findall(block):
            if original.rstrip("/").rsplit("/", 1)[-1] in _CLOUD_NAMES and looks_like_sensor(target) \
                    and original.strip("/") != target.strip("/"):
                found.append(_source(CLOUD, target, "launch"))
    return found


def source_from_spec(spec):
    """A source named by the user as ``<scan|pointcloud|depth>:<topic>``, or None when the text is not that."""
    kind, _, topic = str(spec).partition(":")
    if kind not in MSG_TYPES or not topic.strip("/ "):
        return None
    return _source(kind, topic, "user")


def build(yaml_files, load_yaml, source_files, lidar_config, kinematics, base_frame, urdf_links):
    """The static part of ``summary.obstacle_sensing``."""
    sources, radii, monitor = [], [], None
    for config in lidar_config or []:
        if config.get("laser_scan_topic_name"):
            sources.append(_source(SCAN, config["laser_scan_topic_name"], "lidar_config"))
    for path in yaml_files:
        for node, params in _param_blocks(load_yaml(path)):
            radii += _footprint_radius(params)
            for kind, topic in _observation_sources(params):
                if looks_like_sensor(topic):
                    sources.append(_source(kind, topic, "nav2"))
            if "collision_monitor" in node and "cmd_vel_out_topic" in params and monitor is None:
                monitor = _collision_monitor(node, params)
    for path in source_files:
        try:
            text = pathlib.Path(path).read_text(encoding="utf-8", errors="replace")
        except OSError:
            continue
        if "create_publisher" in text or "remappings" in text:
            sources += from_source_text(text)
    kinematics = kinematics or {}
    declared = kinematics.get("robot_radius")
    if isinstance(declared, (int, float)) and declared > 0:
        radii.append(float(declared))
    links = urdf_links or []
    base = "base_footprint" if "base_footprint" in links else (base_frame or "base_link")
    floor = 0.0
    if base != "base_footprint" and isinstance(kinematics.get("wheel_radius"), (int, float)):
        floor = -float(kinematics["wheel_radius"])
    result = {"version": VERSION, "base_frame": base, "sources": _tidy(sources), "live_verified": False}
    if radii:
        result["footprint_radius_m"] = round(max(radii), 4)
    if floor:
        result["floor_z_m"] = floor
    if monitor:
        result["collision_monitor"] = monitor
    return result


def _is_sensor_cloud(topic):
    """A cloud named like a driver's output (``.../points``, ``/tof/pointcloud``, ``/velodyne_points``)."""
    low = topic.lower()
    last = low.rsplit("/", 1)[-1]
    tokens = {t for s in low.split("/") for t in s.split("_")}
    return (last in _CLOUD_NAMES or last.endswith("_points")) and not tokens & _DERIVED_CLOUD


def live_sources(graph):
    """Range topics in a running graph, ``{topic: type}`` (only topics somebody publishes should be passed in)."""
    found = []
    for topic, kind in graph.items():
        if not looks_like_sensor(topic):
            continue
        if kind == MSG_TYPES[SCAN]:
            found.append(_source(SCAN, topic, "live"))
        elif kind == MSG_TYPES[CLOUD] and _is_sensor_cloud(topic):
            found.append(_source(CLOUD, topic, "live"))
        elif kind == MSG_TYPES[DEPTH] and _is_depth_image(topic):
            found.append(_source(DEPTH, topic, "live"))
    return _tidy(found)


def merge_live(sensing, graph):
    """Add what the running graph publishes to *sensing*. Returns True when it changed.

    Static entries are kept (a part of the stack may be down), except a ``_raw`` or filtered duplicate.
    """
    found = live_sources(graph)
    if not found:
        return False
    live = {s["topic"] for s in found}
    kept = [dict(s, seen_live=True) if s["topic"] in live else s for s in sensing.get("sources") or []]
    known = {s["topic"] for s in kept}
    merged = _tidy(kept + [dict(s, seen_live=True) for s in found if s["topic"] not in known])
    changed = merged != sensing.get("sources") or not sensing.get("live_verified")
    sensing["sources"] = merged
    sensing["live_verified"] = True
    return changed
