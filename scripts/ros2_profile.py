#!/usr/bin/env python3
"""ROS 2 robot profile commands.

Builds and queries a persistent per-robot profile JSON that captures
packages, configurations, launch files, topics, safety limits, and
robot type — so every agent session can load the profile instead of
re-discovering from scratch.

Scan strategy (static-first):
  1. Walk the ROS 2 workspace src/ tree  →  package.xml, launch files,
     URDF/xacro, YAML param files, source code
  2. Query ament index                   →  all installed packages + prefixes
  3. Inspect /opt/ros/<distro>/          →  global packages (optional)
  4. Live graph fallback                 →  only if static gaps remain and
                                            --allow-live is passed

Output: .profiles/robot_profile.json next to this file's parent directory.
"""

import json
import os
import pathlib
import re
import subprocess
import sys
import time
import xml.etree.ElementTree as ET
from datetime import datetime, timezone

from ros2_utils import output


# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------

SCHEMA_VERSION = 1

_SCRIPT_DIR = pathlib.Path(__file__).parent
_PROFILES_DIR = (_SCRIPT_DIR / ".." / ".profiles").resolve()

# Workspace discovery candidates (in priority order).
_WS_CANDIDATES = [
    os.environ.get("ROS2_LOCAL_WS"),
    "~/ros2_ws",
    "~/ws",
    "~/colcon_ws",
    "~/dev_ws",
    "~/robot_ws",
    "~/workspace",
    "~/ros2",
]

# Heuristic: launch file name fragments → configuration name.
# The first regex that matches the stem is used as the config name.
_LAUNCH_CONFIG_PATTERNS = [
    (re.compile(r"\b(k2|kit2|kit_2)\b", re.I), "k2"),
    (re.compile(r"\bpantilt\b", re.I), "pantilt"),
    (re.compile(r"\bbase\b", re.I), "base"),
    (re.compile(r"\bfull\b", re.I), "full"),
    (re.compile(r"\bminimal\b", re.I), "minimal"),
    (re.compile(r"\bsim(?:ulation)?\b", re.I), "sim"),
    (re.compile(r"\bgazebo\b", re.I), "sim"),
    (re.compile(r"\breal\b", re.I), "real"),
    (re.compile(r"\bdemo\b", re.I), "demo"),
]

# Velocity-related YAML keys we scan for safety limits.
_VEL_KEYS = {
    "max_vel_x", "max_vel_y", "max_vel_theta",
    "max_speed_xy", "max_rotation_speed",
    "max_linear_velocity", "max_angular_velocity",
    "translational_speed_limit", "rotational_speed_limit",
    "vel_limit", "linear_vel_limit", "angular_vel_limit",
    "linear", "angular",  # under [limits] or [safety]
}

# Nav2 action server name fragments.
_NAV2_FRAGMENTS = {
    "navigate_to_pose", "navigate_through_poses",
    "follow_path", "compute_path", "spin", "backup",
}

# Sensor package / topic fragments.
_LIDAR_HINTS = {"rplidar", "sllidar", "velodyne", "hokuyo", "laser_scan",
                "scan", "lidar", "pointcloud"}
_CAMERA_HINTS = {"camera", "realsense", "zed", "oak", "image_raw",
                 "depth_image", "rgb", "color"}
_IMU_HINTS = {"imu", "ahrs", "mpu", "bno"}
_ARM_HINTS = {"arm", "gripper", "manipulator", "moveit", "joint_trajectory"}


# ---------------------------------------------------------------------------
# Workspace discovery
# ---------------------------------------------------------------------------

def _discover_workspace(user_path=None):
    """Return the absolute path of the best candidate ROS 2 workspace.

    Search order:
      1. user_path argument
      2. ROS2_LOCAL_WS env var
      3. Common paths: ~/ros2_ws, ~/ws, ~/colcon_ws, ~/dev_ws, ~/robot_ws, ~/workspace, ~/ros2

    A directory qualifies if it contains a ``src/`` sub-directory (indicating
    a colcon workspace).  ``install/`` is not required — the workspace may not
    yet be built, and static analysis only needs the sources.

    Returns:
        (path_str, status) where status is one of
          "user_provided" | "found" | "not_found"
    """
    def _is_ws(p):
        return p.is_dir() and (p / "src").is_dir()

    if user_path:
        p = pathlib.Path(user_path).expanduser().resolve()
        if _is_ws(p):
            return str(p), "user_provided"
        # Still honour it even without src/ — user knows best.
        if p.is_dir():
            return str(p), "user_provided"

    candidates = [c for c in _WS_CANDIDATES if c]
    for raw in candidates:
        p = pathlib.Path(raw).expanduser().resolve()
        if _is_ws(p):
            return str(p), "found"

    return None, "not_found"


# ---------------------------------------------------------------------------
# Static filesystem scanning
# ---------------------------------------------------------------------------

def _scan_packages_from_ament():
    """Return {name: prefix_path} for every ament-indexed package."""
    try:
        from ament_index_python.packages import get_packages_with_prefixes
        return dict(get_packages_with_prefixes())
    except ImportError:
        return {}
    except Exception:
        return {}


def _walk_workspace_src(ws_path):
    """Walk <ws>/src and collect package information.

    Returns a dict:
    {
      "packages": [{"name": str, "path": str, "version": str,
                    "launch_files": [str], "yaml_files": [str],
                    "urdf_files": [str], "src_files": [str]}],
    }
    """
    src = pathlib.Path(ws_path) / "src"
    if not src.is_dir():
        return {"packages": []}

    packages = []
    for pkg_xml in src.rglob("package.xml"):
        pkg_dir = pkg_xml.parent
        info = _parse_package_xml(pkg_xml)
        info["path"] = str(pkg_dir)

        # Collect launch files.
        launch_files = []
        for ext in ("*.launch.py", "*.launch.xml", "*.launch", "*.launch.yaml"):
            launch_files.extend(pkg_dir.rglob(ext))
        info["launch_files"] = [str(f) for f in sorted(launch_files)]

        # Collect param YAML files.
        yaml_files = []
        for pattern in ("*.yaml", "*.yml"):
            for yf in pkg_dir.rglob(pattern):
                # Skip test fixtures and package.xml siblings.
                if "test" not in yf.parts and yf.name not in ("package.xml",):
                    yaml_files.append(yf)
        info["yaml_files"] = [str(f) for f in sorted(yaml_files)]

        # Collect URDF / xacro files.
        urdf_files = []
        for pattern in ("*.urdf", "*.urdf.xacro", "*.xacro"):
            urdf_files.extend(pkg_dir.rglob(pattern))
        info["urdf_files"] = [str(f) for f in sorted(urdf_files)]

        # Collect Python / CPP source nodes (for sensor/type hints).
        src_files = []
        for pattern in ("*.py", "*.cpp", "*.hpp"):
            for sf in pkg_dir.rglob(pattern):
                if "test" not in sf.parts:
                    src_files.append(str(sf))
        info["src_files"] = src_files

        packages.append(info)

    return {"packages": packages}


def _parse_package_xml(path):
    """Extract name and version from a package.xml file."""
    try:
        tree = ET.parse(str(path))
        root = tree.getroot()
        name = (root.findtext("name") or path.parent.name).strip()
        version = (root.findtext("version") or "").strip()
        return {"name": name, "version": version}
    except Exception:
        return {"name": path.parent.name, "version": ""}


# ---------------------------------------------------------------------------
# Launch file analysis
# ---------------------------------------------------------------------------

def _detect_config_name(launch_path):
    """Guess a configuration name from a launch file path."""
    stem = pathlib.Path(launch_path).stem.lower()
    for pattern, label in _LAUNCH_CONFIG_PATTERNS:
        if pattern.search(stem):
            return label
    # Fallback: use the stem itself if it doesn't look like a generic name.
    if stem not in {"bringup", "launch", "main", "robot", "start", "run"}:
        return stem
    return "default"


def _query_launch_args(launch_file, package=None, timeout=15):
    """Run ``ros2 launch --show-args`` and return a dict of {arg: default}.

    Returns {} on failure (file not executable, xacro expansion fails, etc.).
    """
    # Build the ros2 launch command.
    if package:
        cmd = ["ros2", "launch", package, pathlib.Path(launch_file).name, "--show-args"]
    else:
        cmd = ["ros2", "launch", str(launch_file), "--show-args"]

    try:
        proc = subprocess.run(
            cmd, capture_output=True, text=True, timeout=timeout,
        )
        stdout = proc.stdout.strip()
        if not stdout or proc.returncode != 0:
            return {}
        return _parse_show_args_output(stdout)
    except (subprocess.TimeoutExpired, FileNotFoundError, Exception):
        return {}


def _parse_show_args_output(text):
    """Parse the output of ``ros2 launch --show-args`` into a dict.

    The output looks like:
        Arguments (pass arguments as '<name>:=<value>'):
          'use_sim_time':
            default: 'False'
          'robot_description':
            (default: None)
    """
    result = {}
    current_key = None
    for line in text.splitlines():
        stripped = line.strip()
        # Match argument name lines: 'arg_name':
        m = re.match(r"^'([^']+)'\s*:", stripped)
        if m:
            current_key = m.group(1)
            result[current_key] = None
            continue
        # Match default value lines.
        if current_key:
            m2 = re.match(r"^(?:default|default:)\s*[:'\"]*([^'\"]*)['\"]?$", stripped, re.I)
            if m2:
                val = m2.group(1).strip()
                if val.lower() not in ("none", ""):
                    result[current_key] = val
                continue
    return result


# ---------------------------------------------------------------------------
# YAML safety limit extraction
# ---------------------------------------------------------------------------

def _extract_limits_from_yaml(yaml_path):
    """Return (linear_max, angular_max) found in a param YAML file, or (None, None)."""
    try:
        import yaml  # PyYAML — available in ROS 2 environments
    except ImportError:
        return _extract_limits_from_yaml_regex(yaml_path)

    try:
        with open(yaml_path, encoding="utf-8", errors="replace") as fh:
            data = yaml.safe_load(fh)
        if not isinstance(data, dict):
            return None, None
        return _search_limits_in_dict(data)
    except Exception:
        return None, None


def _search_limits_in_dict(d, depth=0):
    """Recursively search a parsed YAML dict for velocity limits."""
    if depth > 8 or not isinstance(d, dict):
        return None, None
    linear, angular = None, None
    for key, val in d.items():
        key_lower = key.lower()
        if "vel_x" in key_lower or "linear" in key_lower and "max" in key_lower:
            if isinstance(val, (int, float)) and val > 0:
                linear = float(val)
        elif "theta" in key_lower or "angular" in key_lower and "max" in key_lower:
            if isinstance(val, (int, float)) and val > 0:
                angular = float(val)
        elif isinstance(val, dict):
            sub_lin, sub_ang = _search_limits_in_dict(val, depth + 1)
            if sub_lin is not None and linear is None:
                linear = sub_lin
            if sub_ang is not None and angular is None:
                angular = sub_ang
    return linear, angular


def _extract_limits_from_yaml_regex(yaml_path):
    """Fallback: regex scan when PyYAML is unavailable."""
    linear, angular = None, None
    try:
        text = pathlib.Path(yaml_path).read_text(encoding="utf-8", errors="replace")
    except Exception:
        return None, None

    for line in text.splitlines():
        line = line.strip()
        m = re.search(r"max_vel_x\s*:\s*([\d.]+)", line)
        if m:
            linear = float(m.group(1))
        m = re.search(r"max_vel_theta\s*:\s*([\d.]+)", line)
        if m:
            angular = float(m.group(1))
    return linear, angular


# ---------------------------------------------------------------------------
# URDF joint limit extraction
# ---------------------------------------------------------------------------

def _extract_joint_limits_from_urdf(urdf_path):
    """Parse a URDF file and return a dict of {joint_name: {velocity, effort}}."""
    limits = {}
    try:
        tree = ET.parse(str(urdf_path))
        root = tree.getroot()
        for joint in root.findall(".//joint"):
            name = joint.get("name", "")
            j_type = joint.get("type", "fixed")
            if j_type == "fixed":
                continue
            limit_el = joint.find("limit")
            if limit_el is not None:
                vel = limit_el.get("velocity")
                eff = limit_el.get("effort")
                limits[name] = {
                    "velocity": float(vel) if vel else None,
                    "effort": float(eff) if eff else None,
                    "type": j_type,
                }
    except Exception:
        pass
    return limits


def _extract_safety_velocity_from_urdf(urdf_path):
    """Return (linear_max, angular_max) from URDF safety_controller elements.

    These are usually on the wheel joints and encode the actual velocity ceiling.
    """
    linear, angular = None, None
    try:
        tree = ET.parse(str(urdf_path))
        root = tree.getroot()
        for sc in root.findall(".//safety_controller"):
            vel_limit = sc.get("k_velocity") or sc.get("velocity")
            if vel_limit:
                try:
                    v = float(vel_limit)
                    if linear is None:
                        linear = v
                except ValueError:
                    pass
    except Exception:
        pass
    return linear, angular


# ---------------------------------------------------------------------------
# Source code hinting (grep-based, lightweight)
# ---------------------------------------------------------------------------

def _grep_source(paths, patterns, max_files=30):
    """Return True if any of *patterns* (regex strings) appears in any of *paths*."""
    compiled = [re.compile(p, re.I) for p in patterns]
    for f in paths[:max_files]:
        try:
            text = pathlib.Path(f).read_text(encoding="utf-8", errors="replace")
            for pat in compiled:
                if pat.search(text):
                    return True
        except Exception:
            continue
    return False


# ---------------------------------------------------------------------------
# Live graph fallback
# ---------------------------------------------------------------------------

def _live_fallback_topics(timeout=8):
    """Return (topic_list, type_map) from the live ROS 2 graph."""
    try:
        import rclpy
        from ros2_utils import ROS2CLI, ros2_context
        with ros2_context():
            node = ROS2CLI()
            topics_and_types = node.get_topic_names_and_types()
        topic_names = [n for n, _ in topics_and_types]
        type_map = {n: (tl[0] if tl else "") for n, tl in topics_and_types}
        return topic_names, type_map
    except Exception:
        return [], {}


# ---------------------------------------------------------------------------
# Robot type detection
# ---------------------------------------------------------------------------

def _detect_robot_type(all_pkg_names, all_src_files, velocity_topics,
                       joint_limits, has_nav2):
    """Return one of: mobile_base | arm | mobile_manipulator | unknown."""
    has_arm = bool(joint_limits) and any(
        _ARM_HINTS.intersection(n.lower().split("_")) for n in joint_limits
    )
    if not has_arm:
        # Check source code for arm hints.
        has_arm = _grep_source(all_src_files, list(_ARM_HINTS), max_files=50)
    if not has_arm:
        has_arm = any(
            any(h in p.lower() for h in _ARM_HINTS) for p in all_pkg_names
        )

    has_mobile = bool(velocity_topics) or has_nav2

    if has_mobile and has_arm:
        return "mobile_manipulator"
    if has_arm:
        return "arm"
    if has_mobile:
        return "mobile_base"
    return "unknown"


# ---------------------------------------------------------------------------
# Configuration grouping
# ---------------------------------------------------------------------------

def _group_launch_files(launch_files):
    """Return {config_name: [launch_file_path]} from a flat list of launch files.

    Launch files that cannot be attributed to a config go under 'default'.
    """
    groups = {}
    for lf in launch_files:
        config = _detect_config_name(lf)
        groups.setdefault(config, []).append(lf)
    return groups


# ---------------------------------------------------------------------------
# Full static scan
# ---------------------------------------------------------------------------

def _run_static_scan(ws_path, distro, allow_live=False, verbose=False):
    """Execute the full static scan and return a profile dict."""
    scan_steps = []

    # --- 1. Ament index ---
    scan_steps.append("ament_index")
    ament_pkgs = _scan_packages_from_ament()
    all_ament_pkg_names = sorted(ament_pkgs.keys())

    # --- 2. Workspace src walk ---
    scan_steps.append("workspace_walk")
    ws_data = _walk_workspace_src(ws_path) if ws_path else {"packages": []}
    ws_packages = ws_data["packages"]
    ws_pkg_names = [p["name"] for p in ws_packages]

    # Aggregate all source artifacts.
    all_launch_files = []
    all_yaml_files = []
    all_urdf_files = []
    all_src_files = []
    for pkg in ws_packages:
        all_launch_files.extend(pkg.get("launch_files", []))
        all_yaml_files.extend(pkg.get("yaml_files", []))
        all_urdf_files.extend(pkg.get("urdf_files", []))
        all_src_files.extend(pkg.get("src_files", []))

    # --- 3. Safety limits from YAML ---
    scan_steps.append("yaml_limits")
    best_linear, best_angular = None, None
    for yf in all_yaml_files:
        lin, ang = _extract_limits_from_yaml(yf)
        if lin and (best_linear is None or lin < best_linear):
            best_linear = lin
        if ang and (best_angular is None or ang < best_angular):
            best_angular = ang

    # --- 4. Joint limits from URDF ---
    scan_steps.append("urdf_limits")
    joint_limits = {}
    for uf in all_urdf_files:
        jl = _extract_joint_limits_from_urdf(uf)
        joint_limits.update(jl)
        # Also try safety_controller velocity.
        lin, ang = _extract_safety_velocity_from_urdf(uf)
        if lin and (best_linear is None or lin < best_linear):
            best_linear = lin

    # --- 5. Configuration grouping ---
    scan_steps.append("config_detection")
    config_groups = _group_launch_files(all_launch_files)
    configurations = sorted(config_groups.keys())

    # --- 6. Sensor / feature detection ---
    scan_steps.append("feature_detection")
    all_pkg_names_lower = [n.lower() for n in (ws_pkg_names + all_ament_pkg_names)]

    has_lidar = any(any(h in p for h in _LIDAR_HINTS) for p in all_pkg_names_lower)
    if not has_lidar:
        has_lidar = _grep_source(all_src_files, list(_LIDAR_HINTS), max_files=40)

    has_camera = any(any(h in p for h in _CAMERA_HINTS) for p in all_pkg_names_lower)
    if not has_camera:
        has_camera = _grep_source(all_src_files, list(_CAMERA_HINTS), max_files=40)

    has_imu = any(any(h in p for h in _IMU_HINTS) for p in all_pkg_names_lower)
    if not has_imu:
        has_imu = _grep_source(all_src_files, list(_IMU_HINTS), max_files=40)

    has_nav2 = any("nav2" in p or "navigation2" in p for p in all_pkg_names_lower)

    # Velocity topic heuristic from package + source names.
    velocity_topics = []
    vel_candidates = ["/cmd_vel", "/cmd_vel_unstamped", "/robot/cmd_vel",
                      "/base/cmd_vel", "/diff_drive_controller/cmd_vel"]
    # If we find "cmd_vel" in source files, record it.
    for sf in all_src_files[:100]:
        try:
            text = pathlib.Path(sf).read_text(encoding="utf-8", errors="replace")
            for line in text.splitlines():
                m = re.search(r'["\']([/\w]+cmd_vel[/\w]*)["\']', line)
                if m:
                    t = m.group(1)
                    if not t.startswith("/"):
                        t = "/" + t
                    if t not in velocity_topics:
                        velocity_topics.append(t)
        except Exception:
            continue
    if not velocity_topics:
        velocity_topics = ["/cmd_vel"]  # safe default

    # --- 7. Live graph fallback (optional) ---
    live_topics = []
    live_type_map = {}
    if allow_live:
        scan_steps.append("live_graph")
        live_topics, live_type_map = _live_fallback_topics()
        # Refine velocity topics from live graph.
        from ros2_topic import VELOCITY_TYPES
        live_vel_topics = [t for t, typ in live_type_map.items()
                           if typ in VELOCITY_TYPES]
        if live_vel_topics:
            velocity_topics = live_vel_topics

        # Refine sensor presence from live graph.
        topic_str = " ".join(live_topics)
        if not has_lidar:
            has_lidar = any(h in topic_str for h in _LIDAR_HINTS)
        if not has_camera:
            has_camera = any(h in topic_str for h in _CAMERA_HINTS)
        if not has_imu:
            has_imu = any(h in topic_str for h in _IMU_HINTS)
        if not has_nav2:
            has_nav2 = "navigate_to_pose" in topic_str

    # --- 8. Robot type ---
    robot_type = _detect_robot_type(
        ws_pkg_names + all_ament_pkg_names,
        all_src_files,
        velocity_topics,
        joint_limits,
        has_nav2,
    )

    # --- 9. Safety limits final ---
    limits_source = "none"
    if best_linear is not None or best_angular is not None:
        limits_source = "yaml_or_urdf"

    safety_limits = {
        "linear_max": best_linear,
        "angular_max": best_angular,
        "source": limits_source,
    }

    # --- 10. Launch args (per config, best-effort) ---
    config_details = {}
    for cfg_name, lf_list in config_groups.items():
        primary_lf = lf_list[0]  # Use first launch file per config.
        launch_args = _query_launch_args(primary_lf)

        # YAML files co-located with this launch file's package.
        cfg_yaml = [
            yf for yf in all_yaml_files
            if pathlib.Path(yf).parent.parent == pathlib.Path(primary_lf).parent.parent
        ]

        # Local joint limits for this config.
        cfg_urdf = [
            uf for uf in all_urdf_files
            if pathlib.Path(uf).parent.parent == pathlib.Path(primary_lf).parent.parent
        ]
        cfg_joints = {}
        for uf in cfg_urdf:
            cfg_joints.update(_extract_joint_limits_from_urdf(uf))

        config_details[cfg_name] = {
            "launch_file": primary_lf,
            "all_launch_files": lf_list,
            "launch_args": launch_args,
            "yaml_files": cfg_yaml,
            "urdf_files": cfg_urdf,
            "joint_limits": cfg_joints,
        }

    return {
        "scan_steps": scan_steps,
        "ws_packages": ws_pkg_names,
        "ament_packages": all_ament_pkg_names,
        "all_launch_files": all_launch_files,
        "configurations": configurations,
        "velocity_topics": velocity_topics,
        "has_lidar": has_lidar,
        "has_camera": has_camera,
        "has_imu": has_imu,
        "has_nav2": has_nav2,
        "robot_type": robot_type,
        "safety_limits": safety_limits,
        "joint_limits": joint_limits,
        "config_details": config_details,
        "live_topics": live_topics,
    }


# ---------------------------------------------------------------------------
# Profile file I/O
# ---------------------------------------------------------------------------

def _profile_path(name="robot"):
    """Return the absolute path for a named robot's profile JSON."""
    _PROFILES_DIR.mkdir(parents=True, exist_ok=True)
    safe_name = re.sub(r"[^\w\-]", "_", name.lower())
    return _PROFILES_DIR / f"{safe_name}_profile.json"


def _load_profile(name="robot"):
    """Load and return the profile dict, or None if it doesn't exist."""
    p = _profile_path(name)
    if not p.exists():
        return None
    try:
        return json.loads(p.read_text(encoding="utf-8"))
    except Exception:
        return None


def _save_profile(profile, name="robot"):
    """Write the profile dict to disk as JSON."""
    p = _profile_path(name)
    p.write_text(json.dumps(profile, indent=2, ensure_ascii=False), encoding="utf-8")
    return str(p)


def _build_profile(robot_name, workspace, distro, scan_result):
    """Assemble the final tiered profile dict from a scan result."""
    sr = scan_result
    return {
        "schema_version": SCHEMA_VERSION,
        "generated_at": datetime.now(timezone.utc).isoformat(),
        "robot_name": robot_name,
        "workspace": workspace,
        "ros_distro": distro,
        "scan_steps": sr["scan_steps"],
        # ------------------------------------------------------------------ #
        # SUMMARY — always loaded; compact; one-glance robot overview.        #
        # ------------------------------------------------------------------ #
        "summary": {
            "robot_type": sr["robot_type"],
            "configurations": sr["configurations"],
            "packages": sr["ws_packages"],
            "velocity_topics": sr["velocity_topics"],
            "has_lidar": sr["has_lidar"],
            "has_camera": sr["has_camera"],
            "has_imu": sr["has_imu"],
            "has_nav2": sr["has_nav2"],
            "safety_limits": sr["safety_limits"],
            "launch_files": {
                cfg: d["launch_file"]
                for cfg, d in sr["config_details"].items()
            },
        },
        # ------------------------------------------------------------------ #
        # DETAIL — per-configuration; load on demand.                         #
        # ------------------------------------------------------------------ #
        "detail": sr["config_details"],
    }


# ---------------------------------------------------------------------------
# Commands
# ---------------------------------------------------------------------------

def cmd_profile_scan(args):
    """Scan the robot workspace and write a robot profile JSON."""
    user_ws = getattr(args, "workspace", None)
    robot_name = getattr(args, "name", None) or "robot"
    allow_live = getattr(args, "allow_live", False)

    distro = os.environ.get("ROS_DISTRO", "unknown")

    # --- Workspace discovery ---
    ws_path, ws_status = _discover_workspace(user_ws)

    if ws_status == "not_found" and not allow_live:
        output({
            "error": (
                "No ROS 2 workspace found. "
                "Pass --workspace /path/to/ws or set ROS2_LOCAL_WS, "
                "or use --allow-live to fall back to the live graph."
            ),
            "searched": [c for c in _WS_CANDIDATES if c],
        })
        return

    # --- Run scan ---
    scan_result = _run_static_scan(
        ws_path=ws_path,
        distro=distro,
        allow_live=allow_live,
    )

    # --- Derive robot name from workspace if not provided ---
    if robot_name == "robot" and ws_path:
        derived = pathlib.Path(ws_path).name
        # Strip common suffixes like _ws, _ros2, ros2_
        derived = re.sub(r"(?:_ws|_ros2|ros2_|_robot)$", "", derived, flags=re.I)
        if derived and derived != "robot":
            robot_name = derived

    # --- Build and save profile ---
    profile = _build_profile(
        robot_name=robot_name,
        workspace=ws_path or "",
        distro=distro,
        scan_result=scan_result,
    )
    profile_file = _save_profile(profile, name=robot_name)

    output({
        "success": True,
        "robot_name": robot_name,
        "profile_file": profile_file,
        "workspace": ws_path,
        "workspace_status": ws_status,
        "configurations": scan_result["configurations"],
        "packages_found": len(scan_result["ws_packages"]),
        "robot_type": scan_result["robot_type"],
        "safety_limits": scan_result["safety_limits"],
        "scan_steps": scan_result["scan_steps"],
        "summary": profile["summary"],
    })


def cmd_profile_show(args):
    """Show the current robot profile (or a specific section)."""
    robot_name = getattr(args, "name", None) or "robot"
    section = getattr(args, "section", None)

    # Try to infer robot name if 'robot' and a profile exists.
    if robot_name == "robot":
        if _PROFILES_DIR.exists():
            profiles = sorted(_PROFILES_DIR.glob("*_profile.json"))
            if len(profiles) == 1:
                stem = profiles[0].name.replace("_profile.json", "")
                robot_name = stem

    profile = _load_profile(robot_name)
    if profile is None:
        output({
            "error": f"No profile found for robot '{robot_name}'.",
            "hint": "Run: python3 ros2_cli.py profile scan [--workspace PATH]",
            "profiles_dir": str(_PROFILES_DIR),
        })
        return

    if section:
        if section == "summary":
            output({"robot_name": robot_name, "summary": profile.get("summary", {})})
        elif section == "detail":
            output({"robot_name": robot_name, "detail": profile.get("detail", {})})
        elif section in profile.get("detail", {}):
            output({"robot_name": robot_name, "config": section,
                    "detail": profile["detail"][section]})
        else:
            available = ["summary", "detail"] + list(profile.get("detail", {}).keys())
            output({
                "error": f"Section '{section}' not found.",
                "available_sections": available,
            })
    else:
        # Default: show summary + list of available detail sections.
        output({
            "robot_name": robot_name,
            "generated_at": profile.get("generated_at"),
            "workspace": profile.get("workspace"),
            "ros_distro": profile.get("ros_distro"),
            "summary": profile.get("summary", {}),
            "detail_sections": list(profile.get("detail", {}).keys()),
            "hint": "Use --section <name> to load a configuration's full detail.",
        })


def cmd_profile_rescan(args):
    """Rescan the robot workspace, updating the profile.

    With --config NAME, rescans only that configuration's launch args
    (fast partial rescan).  Without --config, performs a full rescan.
    """
    robot_name = getattr(args, "name", None) or "robot"
    config_filter = getattr(args, "config", None)
    user_ws = getattr(args, "workspace", None)
    allow_live = getattr(args, "allow_live", False)

    existing = _load_profile(robot_name)
    if existing is None:
        # Try to infer from whatever profile exists.
        if _PROFILES_DIR.exists():
            profiles = sorted(_PROFILES_DIR.glob("*_profile.json"))
            if len(profiles) == 1:
                robot_name = profiles[0].name.replace("_profile.json", "")
                existing = _load_profile(robot_name)

    if config_filter and existing:
        # Partial rescan: just re-query launch args for the requested config.
        detail = existing.get("detail", {})
        if config_filter not in detail:
            output({
                "error": f"Configuration '{config_filter}' not found in existing profile.",
                "available": list(detail.keys()),
            })
            return
        lf = detail[config_filter].get("launch_file", "")
        new_args = _query_launch_args(lf) if lf else {}
        detail[config_filter]["launch_args"] = new_args
        existing["detail"] = detail
        existing["generated_at"] = datetime.now(timezone.utc).isoformat()
        profile_file = _save_profile(existing, name=robot_name)
        output({
            "success": True,
            "mode": "partial",
            "config": config_filter,
            "profile_file": profile_file,
            "launch_args": new_args,
        })
        return

    # Full rescan — delegate to scan.
    args.name = robot_name
    if user_ws is None and existing:
        # Re-use the workspace from the existing profile.
        args.workspace = existing.get("workspace") or None
    cmd_profile_scan(args)


def cmd_profile_list(args):
    """List all robot profiles stored in .profiles/."""
    if not _PROFILES_DIR.exists():
        output({"profiles": [], "profiles_dir": str(_PROFILES_DIR)})
        return

    profiles = []
    for p in sorted(_PROFILES_DIR.glob("*_profile.json")):
        try:
            data = json.loads(p.read_text(encoding="utf-8"))
            profiles.append({
                "file": str(p),
                "robot_name": data.get("robot_name", p.stem),
                "generated_at": data.get("generated_at"),
                "ros_distro": data.get("ros_distro"),
                "robot_type": data.get("summary", {}).get("robot_type"),
                "configurations": data.get("summary", {}).get("configurations", []),
            })
        except Exception:
            profiles.append({"file": str(p), "error": "Could not parse profile"})

    output({"profiles": profiles, "total": len(profiles),
            "profiles_dir": str(_PROFILES_DIR)})


if __name__ == "__main__":
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
