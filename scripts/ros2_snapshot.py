#!/usr/bin/env python3
"""One-process, read-only robot state (`snapshot`) and camera capture (`look`).

`snapshot` subscribes to every sensor in parallel inside one rclpy context and answers the usual "what is the
robot doing / seeing / measuring" questions in a single call. `look` captures a camera frame (or two, to
detect change). Neither publishes or moves anything.
"""

import collections
import contextlib
import io
import json
import math
import os
import struct
import sys
import time

_SECTORS = {"front": (-30, 30), "left": (60, 120), "back": (150, 210), "right": (-120, -60)}  # degrees, laser frame
_BATTERY_STATUS = {0: "UNKNOWN", 1: "CHARGING", 2: "DISCHARGING", 3: "NOT_CHARGING", 4: "FULL"}
_STALE_S = 3.0  # a reading older than this is reported as silent, not as current
_NAV2_NODES = ("bt_navigator", "planner_server", "controller_server", "behavior_server", "collision_monitor",
               "velocity_smoother")
_SLOTS = ("scan", "imu", "odom", "battery", "tof", "depth", "joints", "diagnostics")


def _round(x, n=3):
    return round(x, n) if isinstance(x, (int, float)) and math.isfinite(x) else None


def _scan_sections(msg):
    """Nearest valid return overall and per sector (degrees in the laser frame, 0 = ahead)."""
    best = {name: None for name in _SECTORS}
    nearest = None
    for i, r in enumerate(msg.ranges):
        if not (math.isfinite(r) and msg.range_min < r <= msg.range_max):
            continue
        deg = (math.degrees(msg.angle_min + i * msg.angle_increment) + 180.0) % 360.0 - 180.0
        if nearest is None or r < nearest[0]:
            nearest = (r, deg)
        for name, (lo, hi) in _SECTORS.items():
            inside = lo <= deg <= hi or (name == "back" and (deg >= 150 or deg <= -150))
            if inside and (best[name] is None or r < best[name]):
                best[name] = r
    return {"frame": msg.header.frame_id,
            "nearest_m": _round(nearest[0]) if nearest else None,
            "nearest_bearing_deg": round(nearest[1]) if nearest else None,
            "sectors_m": {k: _round(v) for k, v in best.items()}}


def _tilt(q):
    roll = math.degrees(math.atan2(2 * (q.w * q.x + q.y * q.z), 1 - 2 * (q.x * q.x + q.y * q.y)))
    pitch = math.degrees(math.asin(max(-1.0, min(1.0, 2 * (q.w * q.y - q.z * q.x)))))
    return {"roll_deg": _round(roll, 1), "pitch_deg": _round(pitch, 1)}


def _pointcloud_ranges(msg):
    """Distances of valid xyz points of a small point cloud (ToF imager)."""
    offsets = {f.name: f.offset for f in msg.fields if f.name in ("x", "y", "z")}
    if len(offsets) < 3 or msg.point_step <= 0:
        return None
    data, dists = bytes(msg.data), []
    for i in range(len(data) // msg.point_step):
        base = i * msg.point_step
        p = [struct.unpack_from("<f", data, base + offsets[k])[0] for k in ("x", "y", "z")]
        if all(math.isfinite(c) for c in p):
            dists.append(math.sqrt(sum(c * c for c in p)))
    return {"frame": msg.header.frame_id, "points": len(dists), "of": msg.width * msg.height,
            "min_m": _round(min(dists)) if dists else None, "max_m": _round(max(dists)) if dists else None}


def _depth_centre(msg):
    u, v = msg.width // 2, msg.height // 2
    data = bytes(msg.data)
    if msg.encoding == "16UC1":
        raw = struct.unpack_from("<H", data, v * msg.step + u * 2)[0]
        metres = raw / 1000.0 if raw else None
    elif msg.encoding == "32FC1":
        raw = struct.unpack_from("<f", data, v * msg.step + u * 4)[0]
        metres = raw if math.isfinite(raw) and raw > 0 else None
    else:
        return {"error": f"unsupported depth encoding {msg.encoding}"}
    return {"centre_m": _round(metres), "pixel": [u, v], "size": [msg.width, msg.height]}


def _pick_topics(node, only, summary):
    """Choose one topic per slot: the profile's preference when it is in the graph, else the first match by type."""
    types = dict(node.get_topic_names_and_types())
    by_type = collections.defaultdict(list)
    for name, tys in types.items():
        for ty in tys:
            by_type[ty].append(name)
    lidar = summary.get("lidar_scan_topic")
    fused = (summary.get("localization_config") or {}).get("fused_sources") or {}
    imus = [e["topic"] for e in summary.get("imu_topics") or [] if e.get("topic")]
    images = by_type.get("sensor_msgs/msg/Image", [])
    clouds = by_type.get("sensor_msgs/msg/PointCloud2", [])
    scans = [t for t in by_type.get("sensor_msgs/msg/LaserScan", []) if "visualization" not in t]
    picks = {
        "scan": ([lidar] if lidar in types else []) + scans,
        "imu": imus + by_type.get("sensor_msgs/msg/Imu", []),
        "odom": list(fused.values()) + by_type.get("nav_msgs/msg/Odometry", []),
        "battery": by_type.get("sensor_msgs/msg/BatteryState", []),
        "tof": [t for t in clouds if "tof" in t.lower()],
        "depth": [t for t in images if "depth" in t.lower()],
        "joints": [t for t in by_type.get("sensor_msgs/msg/JointState", []) if t.endswith("joint_states")],
        "diagnostics": [t for t in by_type.get("diagnostic_msgs/msg/DiagnosticArray", []) if t.endswith("diagnostics")],
    }
    chosen = {}
    for slot in _SLOTS:
        if only and slot not in only:
            continue
        for topic in picks[slot]:
            if topic in types:
                chosen[slot] = (topic, types[topic][0])
                break
    return chosen


def _pose(o):
    p, t = o.pose.pose, o.twist.twist
    return {"frame": o.header.frame_id, "x": _round(p.position.x), "y": _round(p.position.y),
            "yaw_deg": _round(math.degrees(2 * math.atan2(p.orientation.z, p.orientation.w)), 1),
            "linear_mps": [_round(t.linear.x, 4), _round(t.linear.y, 4)], "angular_rps": _round(t.angular.z, 4),
            "stationary": max(abs(t.linear.x), abs(t.linear.y), abs(t.angular.z)) < 0.015}


def _battery(b):
    pct = b.percentage * 100.0 if 0.0 <= b.percentage <= 1.0 else b.percentage
    return {"percent": _round(pct, 1), "voltage_v": _round(b.voltage, 2), "current_a": _round(b.current, 2),
            "status": _BATTERY_STATUS.get(b.power_supply_status, str(b.power_supply_status)),
            "charging": b.power_supply_status == 1}


def _wheels(j):
    return {n: _round(v, 3) for n, v in zip(j.name, j.velocity)}


def _diag(arr):
    return [{"name": s.name, "level": _level(s.level), "message": s.message} for s in arr.status]


# slot -> (output key, reducer applied to the latest message)
REDUCERS = {
    "scan": ("lidar", _scan_sections), "odom": ("pose", _pose), "imu": ("tilt", lambda m: _tilt(m.orientation)),
    "battery": ("battery", _battery), "tof": ("tof", _pointcloud_ranges), "depth": ("depth", _depth_centre),
    "joints": ("wheels_rps", _wheels), "diagnostics": ("diagnostics", _diag),
}


def _subscribe(node, chosen, on_msg):
    from rclpy.qos import QoSProfile, ReliabilityPolicy

    from ros2_utils import get_msg_type
    qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
    def guarded(slot):
        def callback(msg):
            try:
                on_msg(slot, msg)
            except Exception as exc:  # an exception escaping a callback would end the executor's spin loop
                print(f"[snapshot] {slot} reducer failed: {exc}", file=sys.stderr)
        return callback

    return [node.create_subscription(get_msg_type(ty), topic, guarded(slot), qos) for slot, (topic, ty) in chosen.items()]


def _service_names(summary, only):
    if only and "state" not in only:
        return []
    return sorted({s for s in [(summary.get("estop_config") or {}).get("service_name"),
                               (summary.get("velocity_chain") or {}).get("mode_service"),
                               *(summary.get("toggles") or {}).values()] if s})


def _assemble(node, chosen, sections, only, extra, controllers=None):
    """Final result: reduced sensor sections plus graph facts, shared by the local and daemon paths."""
    out = {"topics": {s: chosen[s][0] for s in chosen}}
    for slot, value in sections.items():
        out[REDUCERS[slot][0]] = value
    out.update(extra)
    if not only or "controllers" in only:
        out["controllers"] = (controllers or (lambda: _controllers(node)))()
    names = {n for n, _ns in node.get_node_names_and_namespaces()}
    out["nodes"] = {"count": len(names), "nav2": [n for n in _NAV2_NODES if n in names]}
    silent = [s for s in chosen if s not in sections]
    if silent:
        ages = extra.get("age_s", {})
        out["silent_topics"] = {s: chosen[s][0] + (f" (last message {ages[s]} s ago)" if s in ages else "") for s in silent}
    absent = [s for s in _SLOTS if (not only or s in only) and s not in chosen]
    if absent:
        out["no_topic_for"] = absent
    return out


def cmd_snapshot(args):
    from ros2_utils import ROS2CLI, output, ros2_context, try_fastd

    only = sorted({s.strip() for s in args.only.split(",")}) if args.only else []
    if not args.rates:  # the fast daemon keeps sensors subscribed: no rclpy start-up, answers from warm data
        resp = try_fastd("snapshot", {"only": only}, timeout=10.0)
        if resp is not None and "error" not in resp:
            return output(resp)
    try:
        import rclpy
        from ros2_profile import load_profile_summary
        from ros2_robot_state import StateReader

        summary = load_profile_summary(readonly=True) or {}
        with ros2_context():
            node = ROS2CLI("snapshot")
            chosen = _pick_topics(node, set(only), summary)
            services = _service_names(summary, only)
            states = StateReader(node, services) if services else None
            latest, stamps = {}, collections.defaultdict(list)

            def on_msg(slot, msg):
                latest[slot] = msg
                stamps[slot].append(time.monotonic())

            _subscribe(node, chosen, on_msg)
            begin = time.monotonic()
            while time.monotonic() - begin < args.window:
                rclpy.spin_once(node, timeout_sec=0.05)
                if not args.rates and len(latest) == len(chosen) and time.monotonic() - begin > 0.2:
                    break
            window = time.monotonic() - begin
            extra = {"window_s": round(window, 2)}
            if args.rates:
                extra["rates_hz"] = {chosen[s][0]: _round((len(v) - 1) / (v[-1] - v[0]), 1)
                                     for s, v in stamps.items() if len(v) > 1 and v[-1] > v[0]}
            if states is not None:
                extra["service_state"] = states.finish(timeout=0.5)
            out = _assemble(node, chosen, {s: REDUCERS[s][1](m) for s, m in latest.items()}, set(only), extra)
        output(out)
    except Exception as exc:  # pragma: no cover - needs a live graph
        output({"error": str(exc)})


def _start_hub(name, build):
    """A dedicated node whose subscriptions all exist before its own executor thread starts.

    Subscriptions added to the daemon's already-spinning executor stop being serviced while the daemon is idle,
    and re-creating a subscription on the same node receives nothing; a fresh node per hub avoids both.
    """
    import threading

    import rclpy
    from rclpy.node import Node

    node = Node(name)
    extras = build(node)
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(node)
    threading.Thread(target=executor.spin, daemon=True).start()
    return {"node": node, "executor": executor, **(extras or {})}


def _stop_hub(hub):
    try:
        hub["executor"].shutdown()
        hub["node"].destroy_node()
    except Exception:
        pass


def daemon_snapshot(node, params, state, wait=1.5):
    """Serve `snapshot` from the fast daemon (called with its lock held). Every sensor stays subscribed."""
    from ros2_profile import load_profile_summary

    only = set(params.get("only") or [])
    summary = load_profile_summary(readonly=True) or {}
    services = params.get("services") or _service_names(summary, set())
    hub = state.get("hub")
    if hub is None or hub["services"] != services:  # first call, a different service set, or silent topics last time
        if hub is not None:
            _stop_hub(hub)
        chosen, data = _pick_topics(node, set(), summary), {}

        def build(hub_node):
            from ros2_robot_state import StateReader
            _subscribe(hub_node, chosen, lambda slot, m: data.__setitem__(slot, (REDUCERS[slot][1](m), time.monotonic())))
            return {"reader": StateReader(hub_node, services) if services else None}

        hub = state["hub"] = _start_hub("skill_fastd_hub", build)
        hub.update(chosen=chosen, data=data, services=services)
    chosen, data = hub["chosen"], hub["data"]
    deadline = time.monotonic() + wait
    while len(data) < len(chosen) and time.monotonic() < deadline:
        time.sleep(0.02)
    if hub["reader"] is not None and not all(len(v) >= 2 for v in hub["reader"].events.values()):
        time.sleep(0.25)  # replay of the last e-stop/mode calls is still arriving
    now = time.monotonic()
    wanted = {s: c for s, c in chosen.items() if not only or s in only}
    fresh = {s: v for s, (v, t) in dict(data).items() if s in wanted and now - t < _STALE_S}
    extra = {"source": "fast daemon", "age_s": {s: round(now - t, 2) for s, (_v, t) in dict(data).items() if s in wanted}}
    if hub["reader"] is not None and (not only or "state" in only):
        from ros2_robot_state import last_setbool_state
        extra["service_state"] = {s: last_setbool_state(evs[-20:]) for s, evs in hub["reader"].events.items()}
    out = _assemble(node, wanted, fresh, only, extra, controllers=lambda: _daemon_controllers(node))
    if any(s not in data for s in wanted):
        _stop_hub(hub)
        state["hub"] = None  # a topic never reported: rebuild on the next call
    return out


def _daemon_controllers(node):
    from ros2_fastd import _handle_list_controllers
    resp = _handle_list_controllers(node, {"controller_manager": "/controller_manager", "timeout": 3.0})
    if "error" in resp:
        return {"error": resp["error"]}
    return {c["name"]: c["state"] for c in resp.get("controllers", [])}


def _level(raw):
    return int.from_bytes(raw, "little") if isinstance(raw, (bytes, bytearray)) else int(raw)


def _controllers(node):
    try:
        from controller_manager_msgs.srv import ListControllers
        from ros2_control import _call_cm_service
        result, err = _call_cm_service(node, ListControllers, "/controller_manager", "list_controllers",
                                       ListControllers.Request(), 3.0)
        if err:
            return {"error": err.get("error", "controller manager not reachable")}
        return {c.name: c.state for c in result.controller}
    except Exception as exc:
        return {"error": str(exc)}


# ---------------------------------------------------------------------------
# look
# ---------------------------------------------------------------------------

def _pick_camera(node):
    images = [t for t, tys in node.get_topic_names_and_types()
              if "sensor_msgs/msg/Image" in tys or "sensor_msgs/msg/CompressedImage" in tys]
    colour = [t for t in images if any(k in t.lower() for k in ("color", "rgb", "image_raw")) and "depth" not in t.lower()
              and not t.endswith("compressed")]
    return (colour or [t for t in images if "depth" not in t.lower()] or [None])[0]


def _capture(topic, name, timeout, no_profile):
    """Run the shared capture-image handler in-process; returns its JSON result."""
    import argparse

    from ros2_topic import cmd_topics_capture_image
    buf = io.StringIO()
    ns = argparse.Namespace(topic=topic, output=name, timeout=timeout, type="auto", no_profile=no_profile, inline=False)
    with contextlib.redirect_stdout(buf):
        try:
            cmd_topics_capture_image(ns)
        except SystemExit:
            pass
    try:
        return json.loads(buf.getvalue())
    except ValueError:
        return {"error": "capture failed", "raw": buf.getvalue()[:200]}


def _frame_difference(path_a, path_b):
    import cv2
    import numpy as np
    a, b = cv2.imread(path_a, cv2.IMREAD_GRAYSCALE), cv2.imread(path_b, cv2.IMREAD_GRAYSCALE)
    if a is None or b is None or a.shape != b.shape:
        return {"error": "frames could not be compared"}
    diff = cv2.absdiff(a, b)
    changed = float((diff > 30).mean())
    return {"mean_grey_diff": round(float(diff.mean()), 2), "changed_pixels_pct": round(changed * 100, 3),
            "changed": changed > 0.01}


def _write_frame(msg, path, topic, no_profile):
    import cv2

    from ros2_topic import camera_rotation, image_to_bgr
    deg = camera_rotation(topic, no_profile)[0]
    cv2.imwrite(path, image_to_bgr(msg, deg))


def daemon_look(node, params, state):
    """Serve `look` from the fast daemon: one raw (undecoded) camera subscription per topic, kept open."""
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    from rclpy.serialization import deserialize_message

    from ros2_utils import get_msg_type

    topic = params.get("topic") or _pick_camera(node)
    if not topic:
        return {"error": "no camera image topic found"}
    timeout = float(params.get("timeout", 5.0))
    cams = state.setdefault("cams", {})
    if topic not in cams:
        ty = dict(node.get_topic_names_and_types())[topic][0]
        cam = {"cls": get_msg_type(ty), "raw": None, "stamp": 0.0}

        def build(hub_node):
            hub_node.create_subscription(cam["cls"], topic, lambda data: cam.update(raw=data, stamp=time.monotonic()),
                                         QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT), raw=True)
        cam["hub"] = _start_hub("skill_fastd_cam", build)
        cams[topic] = cam
    cam = cams[topic]

    def next_frame(after):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            if cam["stamp"] > after:
                return deserialize_message(cam["raw"], cam["cls"])
            time.sleep(0.02)
        return None

    first = next_frame(time.monotonic() - 1.0)  # a frame from the last second is fresh enough
    if first is None:
        _stop_hub(cam["hub"])
        del cams[topic]  # rebuild on the next call
        return {"error": f"No image received from {topic} within {timeout} seconds", "topic": topic,
                "hint": _silent_camera_hint(topic)}
    no_profile = params.get("no_profile", False)
    _write_frame(first, params["output"], topic, no_profile)
    result = {"topic": topic, "path": params["output"], "source": "fast daemon"}
    if params.get("diff"):
        time.sleep(float(params["diff"]))
        second = next_frame(time.monotonic() - 0.2)
        if second is None:
            return {"error": f"No second image from {topic}", "topic": topic}
        root, ext = os.path.splitext(params["output"])
        after = f"{root}_2{ext or '.jpg'}"
        _write_frame(second, after, topic, no_profile)
        result.update(path_after=after, seconds_apart=params["diff"], **_frame_difference(params["output"], after))
    return result


def capture_frame(path, timeout=5.0):
    """Save one colour frame to *path* (daemon first, then local). Returns ``{"path": ...}`` or ``{"error": ...}``."""
    from ros2_utils import ROS2CLI, resolve_output_path, ros2_context, try_fastd

    resolved = resolve_output_path(path)
    resp = try_fastd("look", {"topic": None, "output": resolved, "diff": None, "timeout": timeout, "no_profile": False},
                     timeout=timeout * 2 + 5)
    if resp is not None and "error" not in resp:
        return {"path": resp.get("path", resolved), "topic": resp.get("topic")}
    try:
        with ros2_context():
            topic = _pick_camera(ROS2CLI("look"))
            if not topic:
                return {"error": "no camera image topic found"}
            first = _capture(topic, path, timeout, False)
            return dict(first, topic=topic)
    except Exception as exc:  # pragma: no cover - needs a live graph
        return {"error": str(exc)}


def cmd_look(args):
    from ros2_utils import ROS2CLI, output, resolve_output_path, ros2_context, try_fastd

    path = resolve_output_path(args.output)
    resp = try_fastd("look", {"topic": args.topic, "output": path, "diff": args.diff, "timeout": args.timeout,
                              "no_profile": args.no_profile}, timeout=args.timeout * 2 + (args.diff or 0) + 5)
    if resp is not None and "error" not in resp:
        return output(resp)
    try:
        with ros2_context():
            topic = args.topic or _pick_camera(ROS2CLI("look"))
            if not topic:
                return output({"error": "no camera image topic found"})
            first = _capture(topic, args.output, args.timeout, args.no_profile)
            if "error" in first:
                return output(dict(first, topic=topic, hint=_silent_camera_hint(topic)))
            result = {"topic": topic, "path": first["path"]}
            if args.diff:
                time.sleep(args.diff)
                second = _capture(topic, f"{os.path.splitext(args.output)[0]}_2{os.path.splitext(args.output)[1] or '.jpg'}",
                                  args.timeout, args.no_profile)
                if "error" in second:
                    return output(dict(second, topic=topic))
                result.update(path_after=second["path"], seconds_apart=args.diff,
                              **_frame_difference(first["path"], second["path"]))
        output(result)
    except Exception as exc:  # pragma: no cover - needs a live graph
        output({"error": str(exc)})


def _silent_camera_hint(topic):
    from ros2_topic import stream_hint
    return stream_hint(topic)
