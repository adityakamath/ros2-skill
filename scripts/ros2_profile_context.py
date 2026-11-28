#!/usr/bin/env python3
"""``profile context``: the whole robot stack in one compact answer, loaded with the skill.

The command creates the profile when there is none and brings it up to date when it is incomplete or its sources
changed, then prints everything an agent needs to know about the stack. It runs once per session (SKILL.md injects
it), so nothing else has to read the profile again. It never fails: problems are reported inside the JSON.

Also the profile's obstacle-sensing upkeep that motion commands rely on, and ``profile set-obstacle``.
"""

import argparse
import contextlib
import importlib.util
import io
import json
import os
import time
from datetime import datetime

import ros2_profile
import ros2_range_sources
from ros2_utils import output

_SOURCE_PATTERNS = ("package.xml", ".launch.py", ".launch.xml", ".launch", ".yaml", ".yml", ".urdf", ".xacro", ".srdf")
_SKIP_DIRS = {".git", "build", "install", "log", "node_modules", "__pycache__"}


# ---------------------------------------------------------------------------
# Keeping the profile current
# ---------------------------------------------------------------------------

def newest_source_mtime(workspace):
    """Newest modification time of any file the profile is built from, 0 when there is no workspace."""
    newest = 0.0
    for root, dirs, files in os.walk(os.path.join(workspace or "", "src")):
        dirs[:] = [d for d in dirs if d not in _SKIP_DIRS]
        for name in files:
            if name.endswith(_SOURCE_PATTERNS):
                try:
                    newest = max(newest, os.stat(os.path.join(root, name)).st_mtime)
                except OSError:
                    pass
    return newest


def sources_changed(profile):
    stamp = profile.get("source_stamp")
    if stamp is None:
        try:
            stamp = datetime.fromisoformat(profile["generated_at"]).timestamp()
        except (KeyError, TypeError, ValueError):
            return False
    return newest_source_mtime(profile.get("workspace")) > stamp + 1e-3


def _run_scan(command, **fields):
    """Run ``profile scan|rescan`` quietly. Returns an error string or None."""
    args = argparse.Namespace(workspace=None, name=None, packages=None, allow_live=False, robot_type=None,
                              namespace=None, transport=None, launch_file=None, skip_daemon_check=True)
    for key, value in fields.items():
        setattr(args, key, value)
    buffer = io.StringIO()
    try:
        with contextlib.redirect_stdout(buffer):
            command(args)
    except SystemExit:
        pass
    except Exception as exc:
        return f"{type(exc).__name__}: {exc}"
    try:
        result = json.loads(buffer.getvalue())
    except ValueError:
        return None
    return result.get("error") if isinstance(result, dict) else None


def _sensing_version(profile):
    return ((profile.get("summary") or {}).get("obstacle_sensing") or {}).get("version", 0)


def _ros_available():
    """True when rclpy can be imported. Asking ros2_utils instead would end the process when it cannot."""
    try:
        return importlib.util.find_spec("rclpy") is not None
    except (ImportError, ValueError):
        return False


def ensure_current(check_sources=True):
    """Return ``(profile or None, what was changed, problem)``, creating or updating the profile as needed."""
    profile = ros2_profile.load_profile_record()
    wanted = os.environ.get("ROS2_SKILL_PROFILE") or os.environ.get("ROS2_SKILL_PROFILE_NAME")
    if not profile:
        problem = _run_scan(ros2_profile.cmd_profile_scan, name=wanted, packages=None)
        profile = ros2_profile.load_profile_record()
        if not profile and ros2_profile._discover_workspace(None)[1] == "not_found":
            if not _ros_available():
                return None, None, "no ROS 2 workspace found, and ROS 2 is not sourced so the running graph cannot be read"
            problem = _run_scan(ros2_profile.cmd_profile_scan, name=wanted, allow_live=True)
            profile = ros2_profile.load_profile_record()
        return profile, "created" if profile else None, None if profile else (problem or "no profile could be built")

    name = profile.get("robot_name") or "robot"
    workspace = profile.get("workspace")
    have_workspace = bool(workspace) and os.path.isdir(workspace)
    outdated = _sensing_version(profile) < ros2_range_sources.VERSION
    if check_sources and have_workspace and sources_changed(profile):
        problem = _run_scan(ros2_profile.cmd_profile_rescan, name=name)
        fresh = ros2_profile.load_profile_record()
        return fresh or profile, None if problem else "rescanned: sources changed", problem
    if outdated:
        try:
            if have_workspace:   # add only the missing section; everything else stays as it is
                scan = ros2_profile._run_static_scan(workspace, profile.get("ros_distro") or "unknown",
                                                     pkg_filter=",".join(profile.get("pkg_filter") or []) or None)
                sensing = scan["obstacle_sensing"]
            else:
                summary = profile.get("summary") or {}
                frames = summary.get("tf_frames") or {}
                sensing = ros2_range_sources.build([], lambda _p: None, [], summary.get("lidar_config"),
                                                   summary.get("kinematics"), frames.get("base_frame"),
                                                   frames.get("urdf_links"))
            profile.setdefault("summary", {})["obstacle_sensing"] = sensing
            ros2_profile._save_profile(profile, name=name)
        except Exception as exc:
            return profile, None, f"could not add obstacle sensing: {type(exc).__name__}: {exc}"
        return profile, "added obstacle sensing", None
    return profile, None, None


def save_live_sources(graph):
    """Merge the range topics the running graph publishes into the profile. Returns True when it changed."""
    profile = ros2_profile.load_profile_record()
    if not profile or not graph:
        return False
    sensing = (profile.get("summary") or {}).get("obstacle_sensing")
    if sensing is None or not ros2_range_sources.merge_live(sensing, graph):
        return False
    ros2_profile._save_profile(profile, name=profile.get("robot_name") or "robot")
    return True


def published_topics(node):
    """``{topic: type}`` for the topics somebody publishes, as *node* sees the graph."""
    return {name: types[0] for name, types in node.get_topic_names_and_types()
            if types and node.count_publishers(name) > 0}


def _daemon_graph():
    """The published topics as the fast daemon sees them, None when it is not up."""
    from ros2_utils import try_fastd

    served = try_fastd("published_topics", {}, timeout=3.0)
    return served["topics"] if served and isinstance(served.get("topics"), dict) else None


def _live_graph(timeout=2.0):
    """The running graph's published topics: from the fast daemon if up, else a short look of our own."""
    served = _daemon_graph()
    if served is not None:
        return served
    if not _ros_available():
        return {}
    try:
        from ros2_utils import ROS2CLI, ros2_context
        with ros2_context():
            node = ROS2CLI("skill_profile_context")
            try:
                graph, since, end = {}, time.monotonic(), time.monotonic() + timeout
                while time.monotonic() < end:
                    seen = published_topics(node)
                    if seen != graph:
                        graph, since = seen, time.monotonic()
                    elif graph and time.monotonic() - since > 0.5:
                        break
                    time.sleep(0.1)
                return graph
            finally:
                node.destroy_node()
    except (Exception, SystemExit):   # a broken ROS environment, or no graph
        return {}


# ---------------------------------------------------------------------------
# The compact context
# ---------------------------------------------------------------------------

def _pick(source, *keys):
    return {k: source[k] for k in keys if source.get(k) not in (None, [], {}, "")}


def compact_context(profile):
    """Everything about the stack an agent needs for a session, without the bulk. Pure."""
    s = profile.get("summary") or {}
    fused = (s.get("localization_config") or {}).get("fused_sources") or {}
    vel_types = {v.get("topic"): v.get("type") for v in s.get("velocity_topics") or []}
    cmd = s.get("cmd_vel_topic")
    sensing = s.get("obstacle_sensing") or {}
    overrides = profile.get("obstacle_overrides") or {}
    ignored = set(overrides.get("ignore_sources") or [])
    ranges = list(sensing.get("sources") or [])
    ranges += [x for x in overrides.get("add_sources") or [] if x["topic"] not in {r["topic"] for r in ranges}]
    monitor = sensing.get("collision_monitor")
    nav2 = s.get("nav2_config") or {}
    cameras = [m["link"] for m in s.get("sensor_mounts") or [] if "camera" in str(m.get("sensor_type"))]
    launch = list(s.get("launch_files") or [])
    detail = profile.get("detail") or {}
    robot_packages = set(profile.get("pkg_filter") or [])
    if len(launch) > 15 and robot_packages:   # a workspace with driver submodules: list the robot's own launch files
        own = [f for f in launch if any(p in str((detail.get(f) or {}).get("package", "")).lower() for p in robot_packages)]
        launch = own or launch
    out = {
        "robot": profile.get("robot_name"), "profile_generated": profile.get("generated_at"),
        "ros_distro": profile.get("ros_distro"), "workspace": profile.get("workspace"),
        **_pick(profile, "namespace", "transport"),
        **_pick(s, "robot_type", "robot_features", "drive_type"),
        "velocity": {k: v for k, v in {
            "cmd_vel_topic": cmd, "msg_type": vel_types.get(cmd) or next(iter(vel_types.values()), None),
            "other_topics": {t: ty for t, ty in vel_types.items() if t != cmd},
            "chain": s.get("velocity_chain")}.items() if v},
        "limits": (s.get("safety_limits") or {}).get("binding"),
        "odom_topic": next(iter(fused.values()), None),
        "frames": _pick(s.get("tf_frames") or {}, "map_frame", "odom_frame", "base_frame"),
        "sensor_frames": [m["link"] for m in s.get("sensor_mounts") or []],
        "obstacle_stop": {k: v for k, v in {
            "sources": [{"topic": x["topic"], "type": x["type"], **({"seen_live": True} if x.get("seen_live") else {})}
                        for x in ranges if x["topic"] not in ignored],
            "declared_without_range_sensors": overrides.get("no_range_sensors"),
            "ignored_sources": sorted(ignored),
            "footprint_radius_m": overrides.get("footprint_radius_m") or sensing.get("footprint_radius_m"),
            "base_frame": sensing.get("base_frame"),
            "collision_monitor": _pick(monitor, "cmd_vel_in", "cmd_vel_out", "state_topic", "polygons", "enabled")
            if monitor else None,
            "confirmed_live": sensing.get("live_verified", False)}.items() if v not in (None, [])},
        **_pick(s, "lidar_scan_topic", "gps_topics", "joint_command_topics"),
        "cameras": cameras,
        "controllers": s.get("active_controllers"),
        "estop": s.get("estop_config"),
        "toggles": s.get("toggles"),
        "services": {k: v.get("service") for k, v in (s.get("robot_services") or {}).items()},
        "teleop": _pick(s.get("teleop_config") or {}, "cmd_vel_topic", "joy_topic", "deadman_buttons"),
        "nav2": _pick(nav2, "controller_plugins", "planner_plugins", "behavior_plugins") or bool(s.get("has_nav2")),
        "localization": (s.get("localization_config") or {}).get("method"),
        "maps": sorted({m.get("path") for m in s.get("maps") or [] if m.get("type") == "occupancy" and m.get("path")}),
        "hardware_fitted": sorted(k for k, v in (s.get("hardware_fitted") or {}).items() if v),
        "capabilities": sorted(s.get("capabilities") or {}) if isinstance(s.get("capabilities"), dict) else None,
        "launch_files": launch,
        "annotations": [a.get("note") for a in profile.get("annotations") or []],
        **_pick(profile, "safety_overrides", "camera_rotation_overrides"),
    }
    out["imu_topics"] = [t.get("topic") if isinstance(t, dict) else t for t in s.get("imu_topics") or []]
    return {k: v for k, v in out.items() if v not in (None, [], {}, "")}


_USE = ("Static facts about this robot for the whole session: do not run `profile show`, `topics find|type`, "
        "`tf list` or `services find` for anything here, and read `annotations` as instructions about the robot. "
        "Live state (controllers active, odometry, battery) is still read live. Run `profile context` again only "
        "after a rescan.")


def cmd_profile_context(args):
    try:
        profile, changed, problem = ensure_current()
        # an unconfirmed profile is worth a short look at the graph; a confirmed one only takes what the daemon knows
        graph = None if not profile else _daemon_graph() if _sensing_confirmed(profile) else _live_graph()
        if graph and save_live_sources(graph):
            profile = ros2_profile.load_profile_record() or profile
        if not profile:
            output({"robot": None, "profile_problem": problem,
                    "use": "No profile: discover live (Path B), or run `profile scan --workspace <path>`."})
            return
        result = {"use": _USE, **compact_context(profile)}
        if changed:
            result["profile_updated"] = changed
        if problem:
            result["profile_problem"] = problem
    except (Exception, SystemExit) as exc:   # the skill must load whatever happens here
        result = {"robot": None, "profile_problem": f"{type(exc).__name__}: {exc}",
                  "use": "Profile context unavailable: use `profile show`, or discover live."}
    print(json.dumps(result, separators=(",", ":"), ensure_ascii=False, default=str))


def _sensing_confirmed(profile):
    return bool(((profile.get("summary") or {}).get("obstacle_sensing") or {}).get("live_verified"))


def cmd_profile_set_obstacle(args):
    """Store what the scan cannot know about obstacle sensing; kept across rescans."""
    profile = ros2_profile.load_profile_record(profile_name=args.name) if args.name else ros2_profile.load_profile_record()
    if not profile:
        output({"error": "No profile found to update.", "hint": "Run: profile context"})
        return
    overrides = {} if args.clear else dict(profile.get("obstacle_overrides") or {})
    if args.radius is not None:
        if args.radius <= 0:
            output({"error": "--radius must be positive (metres from the base centre to the furthest part)"})
            return
        overrides["footprint_radius_m"] = args.radius
    if args.max_height is not None:
        overrides["max_height_m"] = args.max_height
    if getattr(args, "no_sensors", False):
        overrides["no_range_sensors"] = True
    if getattr(args, "add_source", None):
        added = {s["topic"]: s for s in overrides.get("add_sources") or []}
        for spec in args.add_source:
            source = ros2_range_sources.source_from_spec(spec)
            if source is None:
                output({"error": f"--add-source takes scan:TOPIC, pointcloud:TOPIC or depth:TOPIC, not '{spec}'"})
                return
            added[source["topic"]] = source
        overrides["add_sources"] = list(added.values())
    if args.ignore_source:
        ignored = set(overrides.get("ignore_sources") or [])
        ignored.update(ros2_range_sources.normalise(t) for t in args.ignore_source)
        overrides["ignore_sources"] = sorted(ignored)
    profile["obstacle_overrides"] = overrides
    ros2_profile._save_profile(profile, name=profile.get("robot_name") or "robot")
    output({"success": True, "robot_name": profile.get("robot_name"), "obstacle_overrides": overrides,
            "obstacle_stop": compact_context(profile).get("obstacle_stop")})


def guard_inputs():
    """``(summary, obstacle_overrides)`` for a motion command, building the sensing section once if it is missing.

    The normal case is a cached profile read: no scan, no ROS. A section not yet checked against a running stack is
    completed from the fast daemon's view of the graph when that is up (milliseconds), never by looking here.
    """
    from ros2_utils import try_fastd

    profile = ros2_profile.load_profile_record(readonly=True)
    if not os.environ.get("ROS2_SKILL_NO_AUTOPROFILE"):   # set by the unit tests
        if not profile or _sensing_version(profile) < ros2_range_sources.VERSION:
            profile, _changed, _problem = ensure_current(check_sources=False)
        if profile and not _sensing_confirmed(profile):
            served = try_fastd("published_topics", {}, timeout=2.0)
            if served and save_live_sources(served.get("topics")):
                profile = ros2_profile.load_profile_record(readonly=True)
    if not profile:
        return {}, {}
    return profile.get("summary") or {}, profile.get("obstacle_overrides") or {}
