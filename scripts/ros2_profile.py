#!/usr/bin/env python3
"""ROS 2 robot profile commands.

Builds and queries a persistent per-robot profile JSON that captures
packages, launch files, topics, safety limits, and
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
import math
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

# ---------------------------------------------------------------------------
# Sensor hints (package names, topic fragments, source keywords).
# These are open-ended — add new entries as new hardware appears.
# ---------------------------------------------------------------------------

_LIDAR_HINTS = {
    "rplidar", "sllidar", "urg", "velodyne", "hokuyo",
    "laser_scan", "scan", "lidar", "pointcloud", "ouster",
    "livox", "hesai", "robosense",
}
_CAMERA_HINTS = {
    "camera", "realsense", "zed", "oak", "image_raw",
    "depth_image", "rgb", "color", "basler", "flir",
    "v4l2", "usb_cam", "opencv",
}
_IMU_HINTS = {"imu", "ahrs", "mpu", "bno", "vectornav", "xsens", "microstrain"}

# ---------------------------------------------------------------------------
# Robot-type hints — each set covers package names, topic fragments, and
# source-code keywords.  They are intentionally broad; detection is additive
# so an unrecognised platform still falls through gracefully to "unknown".
# ---------------------------------------------------------------------------

# Arm / manipulator (any DOF, any make).
_ARM_HINTS = {
    "arm", "gripper", "manipulator", "moveit", "joint_trajectory",
    "ur3", "ur5", "ur10", "panda", "kuka", "fanuc", "yaskawa",
    "abb", "kinova", "xarm", "hebi", "dynamixel", "servo",
}

# Ground / wheeled mobile base.
_MOBILE_HINTS = {
    "diff_drive", "cmd_vel", "mecanum", "omni", "ackermann",
    "turtlebot", "husky", "jackal", "ridgeback", "summit",
    "roomba", "create3", "base_controller",
}

# Aerial / UAV / drone.
_AERIAL_HINTS = {
    "drone", "uav", "quadrotor", "multirotor", "hexarotor",
    "mavros", "px4", "ardupilot", "betaflight", "ardrone",
    "parrot", "dji", "rotors", "hector_quadrotor",
    "altitude", "takeoff", "land", "thrust",
}

# Legged robots (quadruped, hexapod, biped walkers that aren't full humanoids).
_LEGGED_HINTS = {
    "legged", "quadruped", "hexapod", "biped",
    "spot", "go1", "go2", "go3", "b1", "b2", "unitree", "anymal",
    "cheetah", "cassie", "hyq", "aliengo", "a1", "mini_cheetah",
    "leg_controller", "stance", "swing", "gait",
}

# Humanoid robots — specific platform names and ZMP only.
# Do NOT add generic locomotion words ("walking", "balance") — they appear in
# any robotics codebase and cause false positives on non-humanoid platforms.
_HUMANOID_HINTS = {
    "humanoid",
    "nao", "pepper", "romeo", "atlas", "valkyrie",
    "talos", "icub", "darwin", "op2", "op3", "thormang",
    "zmp",  # Zero Moment Point — specific to humanoid stability algorithms
}

# Pan-tilt / gimbal — supplementary feature, not a primary robot type.
_PANTILT_HINTS = {
    "pantilt", "pan_tilt", "ptz", "gimbal",
    "pan_controller", "tilt_controller",
}

# All valid primary robot type values.
_VALID_ROBOT_TYPES = frozenset({
    "humanoid", "legged", "aerial", "underwater", "surface_vessel",
    "mobile_manipulator", "arm", "mobile_base", "unknown",
})

# Underwater ROV / AUV.
_UNDERWATER_HINTS = {
    "underwater", "uuv", "rov", "auv", "bluerov", "bluerobotics",
    "thruster", "dvl", "depth_sensor", "pressure",
    "waterlinked", "bar30", "ping",
}

# Surface vessels (USV / ASV / boat).
_SURFACE_VESSEL_HINTS = {
    "usv", "asv", "boat", "vessel", "marine", "nauticus",
    "wam_v", "navquad", "rudder", "propeller", "watercraft",
}


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
# Launch include analysis — static parsing of sub-launch inclusion chains
# ---------------------------------------------------------------------------

def _parse_launch_includes(launch_path):
    """Return a list of sub-launch include records found in *launch_path*.

    Each record:
    {
        "source": "<str>",         # best-effort path/package description
        "package": "<str>|None",   # package name if determinable
        "file": "<str>|None",      # filename of the included launch file
        "args_forwarded": {        # arguments passed to the included file
            "<arg>": "<value>",    # literal string, or "$ref" for LaunchConfiguration('ref')
            ...
        }
    }

    Supports:
    - Python .launch.py files  (ast-based, best-effort)
    - XML    .launch.xml files (ElementTree)
    - YAML   .launch.yaml files (PyYAML, limited)
    """
    path = pathlib.Path(launch_path)
    if not path.exists():
        return []
    name = path.name.lower()
    if name.endswith(".launch.py"):
        return _parse_python_includes(path)
    if name.endswith((".launch.xml", ".launch")) or path.suffix == ".xml":
        return _parse_xml_includes(path)
    if name.endswith((".launch.yaml", ".launch.yml")):
        return _parse_yaml_includes(path)
    return []


# ---- Python (.launch.py) --------------------------------------------------

def _ast_call_name(node):
    """Return the bare function name from an ast.Call node."""
    import ast as _ast
    if isinstance(node.func, _ast.Name):
        return node.func.id
    if isinstance(node.func, _ast.Attribute):
        return node.func.attr
    return ""


def _ast_to_str(node):
    """Convert an AST node to a concise human-readable string.

    Conventions:
    - String/int constant       → the literal value
    - LaunchConfiguration('x') → "$x"
    - get_package_share_directory('p') / FindPackageShare('p') → "pkg:p"
    - PathJoinSubstitution([...]) / os.path.join(...) → joined parts
    - Variable name reference   → "$varname"
    - Anything else             → "<expr>"
    """
    import ast as _ast

    if node is None:
        return None
    if isinstance(node, _ast.Constant):
        return str(node.value)
    if isinstance(node, _ast.Name):
        return f"${node.id}"
    if isinstance(node, _ast.Call):
        fn = _ast_call_name(node)
        if fn == "LaunchConfiguration" and node.args:
            inner = _ast_to_str(node.args[0])
            return f"${inner}" if inner else "$?"
        if fn in ("get_package_share_directory", "FindPackageShare") and node.args:
            pkg = _ast_to_str(node.args[0])
            return f"pkg:{pkg}" if pkg else "pkg:?"
        if fn in ("PathJoinSubstitution", "JoinPathSegments") and node.args:
            arg0 = node.args[0]
            if isinstance(arg0, (_ast.List, _ast.Tuple)):
                parts = [_ast_to_str(e) for e in arg0.elts]
                return "/".join(p for p in parts if p)
        if fn in ("os.path.join", "join"):
            parts = [_ast_to_str(a) for a in node.args]
            return "/".join(p for p in parts if p)
        if fn == "PythonLaunchDescriptionSource" and node.args:
            return _ast_to_str(node.args[0])
        if fn == "AnyLaunchDescriptionSource" and node.args:
            return _ast_to_str(node.args[0])
        # Generic: show the function name so the reader understands context.
        return f"{fn}(...)"
    if isinstance(node, (_ast.List, _ast.Tuple)):
        parts = [_ast_to_str(e) for e in node.elts]
        return "/".join(p for p in parts if p)
    if isinstance(node, _ast.BinOp):
        import ast as _ast2
        if isinstance(node.op, _ast2.Add):
            left = _ast_to_str(node.left)
            right = _ast_to_str(node.right)
            return f"{left}{right}"
    if isinstance(node, _ast.Attribute):
        owner = _ast_to_str(node.value)
        return f"{owner}.{node.attr}"
    if isinstance(node, _ast.JoinedStr):        # f-string — too dynamic to resolve
        return "<f-string>"
    return "<expr>"


def _ast_extract_forwarded_args(node):
    """Extract {arg_name: value_str} from a launch_arguments AST node.

    Accepts all three forms used in ROS 2 Python launch files:
    - Dict literal:                  {"key": value, ...}
    - Dict.items() call:             {"key": value, ...}.items()
    - List/tuple of (name, value):   [("key", value), ...]
    """
    import ast as _ast
    # Unwrap {}.items() — very common in ROS 2 launch files.
    if (isinstance(node, _ast.Call)
            and isinstance(node.func, _ast.Attribute)
            and node.func.attr == "items"
            and isinstance(node.func.value, _ast.Dict)):
        node = node.func.value

    result = {}
    if isinstance(node, _ast.Dict):
        for k, v in zip(node.keys, node.values):
            key = _ast_to_str(k)
            val = _ast_to_str(v)
            if key and not key.startswith("$"):
                result[key] = val
    elif isinstance(node, (_ast.List, _ast.Tuple)):
        for elt in node.elts:
            if isinstance(elt, (_ast.Tuple, _ast.List)) and len(elt.elts) == 2:
                key = _ast_to_str(elt.elts[0])
                val = _ast_to_str(elt.elts[1])
                if key and not key.startswith("$"):
                    result[key] = val
    return result


def _parse_python_includes(path):
    """Parse a Python launch file and return include records (ast-based)."""
    import ast as _ast
    try:
        text = path.read_text(encoding="utf-8", errors="replace")
        tree = _ast.parse(text, filename=str(path))
    except Exception:
        return []

    includes = []
    for node in _ast.walk(tree):
        if not isinstance(node, _ast.Call):
            continue
        if _ast_call_name(node) != "IncludeLaunchDescription":
            continue

        # ── source (first positional arg or 'launch_description_source' kw) ──
        source_node = None
        if node.args:
            source_node = node.args[0]
        else:
            for kw in node.keywords:
                if kw.arg in ("launch_description_source", None):
                    source_node = kw.value
                    break
        source_str = _ast_to_str(source_node) if source_node else None

        # ── extract package and filename from source_str ──
        pkg, fname = _decompose_source(source_str)

        # ── launch_arguments ──
        forwarded = {}
        for kw in node.keywords:
            if kw.arg == "launch_arguments":
                forwarded = _ast_extract_forwarded_args(kw.value)
                break

        includes.append({
            "source": source_str,
            "package": pkg,
            "file": fname,
            "args_forwarded": forwarded,
        })

    return includes


def _decompose_source(source_str):
    """Split a source string like 'pkg:nav2_bringup/launch/bringup_launch.py'
    into (package, filename).  Returns (None, None) when not determinable."""
    if not source_str:
        return None, None
    # "pkg:nav2_bringup/launch/bringup_launch.py"
    m = re.match(r"pkg:([^/]+)/(.+)", source_str)
    if m:
        pkg = m.group(1)
        rest = m.group(2)
        fname = pathlib.Path(rest).name
        return pkg, fname
    # Plain path — try to extract the filename
    if "/" in source_str or "\\" in source_str:
        fname = pathlib.Path(source_str).name
        return None, fname if fname else None
    return None, None


# ---- XML (.launch.xml / .launch) ------------------------------------------

def _parse_xml_includes(path):
    """Parse an XML launch file and return include records."""
    try:
        tree = ET.parse(str(path))
        root = tree.getroot()
    except Exception:
        return []

    includes = []
    for inc in root.iter("include"):
        source = inc.get("file", "") or inc.get("pkg", "")
        # $(find pkg)/launch/file.py  or  $(pkg-path pkg)/...
        pkg, fname = None, None
        m = re.search(r"\$\(find\s+([^)]+)\)", source)
        if m:
            pkg = m.group(1)
            fname = pathlib.Path(source.split(")")[-1]).name or None
        else:
            fname = pathlib.Path(source).name or None

        forwarded = {}
        for arg in inc.findall("arg"):
            name = arg.get("name", "")
            value = arg.get("value") or arg.get("default")
            if name:
                # Convert ROS XML substitution $(arg x) → $x
                if value:
                    value = re.sub(r"\$\(arg\s+([^)]+)\)", r"$\1", value)
                forwarded[name] = value

        includes.append({
            "source": source or None,
            "package": pkg,
            "file": fname,
            "args_forwarded": forwarded,
        })

    return includes


# ---- YAML (.launch.yaml) --------------------------------------------------

def _parse_yaml_includes(path):
    """Parse a YAML launch file and return include records (limited support)."""
    try:
        import yaml
        with open(path, encoding="utf-8", errors="replace") as fh:
            data = yaml.safe_load(fh)
    except Exception:
        return []

    if not isinstance(data, list):
        return []

    includes = []
    for item in data:
        if not isinstance(item, dict) or "include" not in item:
            continue
        inc = item["include"]
        if not isinstance(inc, dict):
            continue
        source = inc.pop("file", None)
        pkg, fname = _decompose_source(source)
        includes.append({
            "source": source,
            "package": pkg,
            "file": fname,
            "args_forwarded": {k: str(v) for k, v in inc.items()},
        })

    return includes


# ---------------------------------------------------------------------------
# YAML safety limit extraction
# ---------------------------------------------------------------------------

def _extract_limits_from_yaml(yaml_path):
    """Return ``(linear_x, linear_y, angular_z)`` found in a param YAML file.

    Each element is ``None`` when the axis limit is not found.
    """
    try:
        import yaml  # PyYAML — available in ROS 2 environments
    except ImportError:
        return _extract_limits_from_yaml_regex(yaml_path)

    try:
        with open(yaml_path, encoding="utf-8", errors="replace") as fh:
            data = yaml.safe_load(fh)
        if not isinstance(data, dict):
            return None, None, None
        return _search_limits_in_dict(data)
    except Exception:
        return None, None, None


def _search_limits_in_dict(d, depth=0):
    """Recursively search a parsed YAML dict for per-axis velocity limits.

    Returns ``(linear_x, linear_y, angular_z)`` — any element may be ``None``
    when the corresponding limit is not found in this subtree.

    Recognised formats
    ------------------
    * **Flat keys** (Nav2 DWB / TEB / MPPI planner, teleop flat form)::

        max_vel_x: 0.5
        max_vel_y: 0.3
        max_vel_theta: 1.0
        scale_linear: 0.5        # teleop_twist_joy flat
        scale_angular: 1.0

    * **ros2_control DiffDriveController / AckermannSteering** nested::

        linear:
          x:
            max_velocity: 0.5
        angular:
          z:
            max_velocity: 1.0

    * **teleop_twist_joy v2** nested scale::

        scale_linear:
          x: 0.5
          y: 0.3
        scale_angular:
          yaw: 1.0

    * **velocity_smoother** list form::

        max_velocity: [0.5, 0.0, 1.0]   # [lin_x, lin_y, ang_z]
    """
    if depth > 8 or not isinstance(d, dict):
        return None, None, None
    lin_x, lin_y, ang_z = None, None, None

    def _upd(current, candidate):
        """Keep the most restrictive (smallest) non-None value."""
        if candidate is None or candidate <= 0:
            return current
        return float(candidate) if current is None else min(current, float(candidate))

    for key, val in d.items():
        kl = key.lower()

        # --- Direct numeric value ---
        if isinstance(val, (int, float)) and val > 0:
            if kl in _LINEAR_X_KEYS:
                lin_x = _upd(lin_x, val)
            elif kl in _LINEAR_Y_KEYS:
                lin_y = _upd(lin_y, val)
            elif kl in _ANGULAR_Z_KEYS:
                ang_z = _upd(ang_z, val)

        # --- List form: max_velocity: [lin_x, lin_y, ang_z] ---
        elif isinstance(val, list) and ("max_velocity" in kl or "max_vel" in kl):
            if len(val) >= 1:
                lin_x = _upd(lin_x, val[0] if isinstance(val[0], (int, float)) else None)
            if len(val) >= 2:
                lin_y = _upd(lin_y, val[1] if isinstance(val[1], (int, float)) else None)
            if len(val) >= 3:
                ang_z = _upd(ang_z, val[2] if isinstance(val[2], (int, float)) else None)

        # --- Nested dict blocks ---
        elif isinstance(val, dict):
            if kl == "linear":
                # ros2_control: linear: {x: {max_velocity: N}}
                x_block = val.get("x", {})
                if isinstance(x_block, dict):
                    mv = x_block.get("max_velocity") or x_block.get("max_vel")
                    lin_x = _upd(lin_x, mv)
                # short form: linear: {max_velocity: N}
                lin_x = _upd(lin_x, val.get("max_velocity") or val.get("max_vel"))

            elif kl == "angular":
                # ros2_control: angular: {z: {max_velocity: N}}
                z_block = val.get("z", {})
                if isinstance(z_block, dict):
                    mv = z_block.get("max_velocity") or z_block.get("max_vel")
                    ang_z = _upd(ang_z, mv)
                # short form: angular: {max_velocity: N}
                ang_z = _upd(ang_z, val.get("max_velocity") or val.get("max_vel"))

            elif kl == "scale_linear":
                # teleop_twist_joy v2: scale_linear: {x: N, y: N}
                if "x" in val:
                    lin_x = _upd(lin_x, val["x"] if isinstance(val["x"], (int, float)) else None)
                if "y" in val:
                    lin_y = _upd(lin_y, val["y"] if isinstance(val["y"], (int, float)) else None)

            elif kl in ("scale_angular", "scale_angular_yaw"):
                # teleop_twist_joy v2: scale_angular: {yaw: N} or scale_angular: {z: N}
                for ak in ("yaw", "z"):
                    if ak in val:
                        ang_z = _upd(ang_z, val[ak] if isinstance(val[ak], (int, float)) else None)

            else:
                sub_x, sub_y, sub_z = _search_limits_in_dict(val, depth + 1)
                lin_x = _upd(lin_x, sub_x)
                lin_y = _upd(lin_y, sub_y)
                ang_z = _upd(ang_z, sub_z)

    return lin_x, lin_y, ang_z


def _extract_limits_from_yaml_regex(yaml_path):
    """Fallback: line-by-line regex scan when PyYAML is unavailable.

    Returns ``(linear_x, linear_y, angular_z)`` — any element may be ``None``.
    """
    _LIN_X_PATS = [re.compile(r"(?:^|\s)" + re.escape(k) + r"\s*:\s*([\d.]+)")
                   for k in _LINEAR_X_KEYS]
    _LIN_Y_PATS = [re.compile(r"(?:^|\s)" + re.escape(k) + r"\s*:\s*([\d.]+)")
                   for k in _LINEAR_Y_KEYS]
    _ANG_Z_PATS = [re.compile(r"(?:^|\s)" + re.escape(k) + r"\s*:\s*([\d.]+)")
                   for k in _ANGULAR_Z_KEYS]
    lin_x, lin_y, ang_z = None, None, None
    try:
        text = pathlib.Path(yaml_path).read_text(encoding="utf-8", errors="replace")
    except Exception:
        return None, None, None

    for line in text.splitlines():
        stripped = line.strip()
        for pat in _LIN_X_PATS:
            m = pat.search(stripped)
            if m:
                v = float(m.group(1))
                if v > 0 and (lin_x is None or v < lin_x):
                    lin_x = v
        for pat in _LIN_Y_PATS:
            m = pat.search(stripped)
            if m:
                v = float(m.group(1))
                if v > 0 and (lin_y is None or v < lin_y):
                    lin_y = v
        for pat in _ANG_Z_PATS:
            m = pat.search(stripped)
            if m:
                v = float(m.group(1))
                if v > 0 and (ang_z is None or v < ang_z):
                    ang_z = v
    return lin_x, lin_y, ang_z


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
# URDF robot name helper
# ---------------------------------------------------------------------------

def _get_urdf_robot_name(urdf_path):
    """Return the ``<robot name="…">`` attribute, or the file stem as fallback.

    Used to group joint limits by the URDF model they came from so that
    test/example URDFs in the workspace do not pollute the main robot's joints.
    """
    try:
        tree = ET.parse(str(urdf_path))
        name = tree.getroot().get("name", "").strip()
        if name:
            return name
    except Exception:
        pass
    # Strip known URDF suffixes (.urdf, .urdf.xacro, .xacro) from the filename.
    stem = pathlib.Path(urdf_path).stem
    for suffix in (".urdf", ".xacro"):
        if stem.endswith(suffix):
            stem = stem[: -len(suffix)]
    return stem or "unknown"


# ---------------------------------------------------------------------------
# Velocity-limit key sets used by YAML extraction
# ---------------------------------------------------------------------------

# Keys whose value is a linear-x velocity limit (or a combined linear limit
# when linear-y is not separately specified).
_LINEAR_X_KEYS = frozenset({
    # Nav2 planner / controller params
    "max_vel_x", "max_linear_velocity", "max_speed_xy",
    "translational_speed_limit", "linear_vel_limit",
    "max_linear", "linear_max",
    # teleop_twist_joy scale params (flat form)
    "scale_linear", "scale_linear_x", "linear_scale", "linear_scale_x",
})

# Keys whose value is a linear-y velocity limit (holonomic robots).
_LINEAR_Y_KEYS = frozenset({
    "max_vel_y",
    "scale_linear_y", "linear_scale_y",
})

# Keys whose value is an angular-z velocity limit.
_ANGULAR_Z_KEYS = frozenset({
    # Nav2 planner / controller params
    "max_vel_theta", "max_angular_velocity", "max_rotation_speed",
    "rotational_speed_limit", "angular_vel_limit",
    "max_angular", "angular_max",
    # teleop_twist_joy scale params (flat form)
    "scale_angular", "scale_angular_z", "scale_angular_yaw",
    "angular_scale", "angular_scale_z",
})

# Backward-compat aliases used elsewhere in the module.
_LINEAR_KEYS = _LINEAR_X_KEYS
_ANGULAR_KEYS = _ANGULAR_Z_KEYS


# ---------------------------------------------------------------------------
# Sensor / actuator mount classification
# ---------------------------------------------------------------------------

# Ordered list of (keyword_set, sensor_type) pairs.
# First match wins; keywords are checked as substrings of the lowercased
# child link name (e.g. "camera_link" → "camera", "lidar_front" → "lidar").
_SENSOR_LINK_PATTERNS = [
    ({"camera", "cam", "rgb", "color"},                                        "camera"),
    ({"depth", "rgbd", "d435", "d415", "d455", "d457", "l515", "realsense"},  "depth_camera"),
    ({"lidar", "laser", "scan", "velodyne", "hokuyo", "rplidar", "lms",
      "vlp", "ouster", "livox", "hesai"},                                      "lidar"),
    ({"imu", "ahrs", "mpu", "bno", "vectornav", "xsens", "microstrain"},      "imu"),
    ({"sonar", "ultrasonic", "ultrasound", "ping"},                            "sonar"),
    ({"gps", "gnss", "navsat"},                                                "gps"),
    ({"gripper", "hand", "finger", "ee_link", "tool_link", "eef",
      "end_effector"},                                                          "gripper"),
]


def _classify_sensor_link(link_name):
    """Return the sensor type string for a URDF link name, or ``None``.

    Matches are substring-based on the lowercased link name, with the first
    entry in ``_SENSOR_LINK_PATTERNS`` that has any keyword hit winning.
    Returns ``None`` when the link is not recognised as a sensor or actuator.
    """
    ll = link_name.lower()
    for keywords, stype in _SENSOR_LINK_PATTERNS:
        if any(kw in ll for kw in keywords):
            return stype
    return None


def _image_rotation_from_roll(roll_rad):
    """Return image-correction degrees for a camera roll angle, or 0.

    ≈ ±π  → 180 (upside-down)
    ≈ +π/2 → 90  (on its left side)
    ≈ −π/2 → -90 (on its right side)
    """
    if abs(abs(roll_rad) - math.pi) < 0.2:
        return 180
    if abs(roll_rad - math.pi / 2.0) < 0.2:
        return 90
    if abs(roll_rad + math.pi / 2.0) < 0.2:
        return -90
    return 0


def _extract_sensor_mounts_from_urdf(urdf_path):
    """Parse a URDF and return mount pose info for every sensor and actuator.

    Inspects all joints whose **child link name** matches any entry in
    ``_SENSOR_LINK_PATTERNS`` — cameras, depth cameras, LiDARs, IMUs, sonars,
    GPS units, grippers, and so on.  For each match the joint's
    ``<origin xyz="…" rpy="…"/>`` is read and stored verbatim so that any
    downstream command can reason about the sensor's physical placement.

    For **visual** sensors (``camera``, ``depth_camera``) the field
    ``image_rotation_deg`` is also included: the suggested image correction
    derived from the roll angle so that captured frames can be straightened
    before display or delivery.

    Returns a list of dicts (empty list when the URDF cannot be read or no
    sensor/actuator joints are found):

    .. code-block:: json

        {
          "joint":       "lidar_joint",
          "link":        "lidar_front",
          "sensor_type": "lidar",
          "xyz":         [0.15, 0.0, 0.25],
          "rpy":         [0.0, 0.0, 3.14159],

          // Only for camera / depth_camera:
          "image_rotation_deg": 180
        }
    """
    try:
        tree = ET.parse(str(urdf_path))
        root = tree.getroot()
    except Exception:
        return []

    mounts = []
    for joint in root.findall(".//joint"):
        child_el = joint.find("child")
        if child_el is None:
            continue
        child_link = child_el.get("link", "")
        sensor_type = _classify_sensor_link(child_link)
        if sensor_type is None:
            continue

        origin_el = joint.find("origin")
        if origin_el is None:
            xyz = [0.0, 0.0, 0.0]
            rpy = [0.0, 0.0, 0.0]
        else:
            try:
                xyz = [float(v) for v in origin_el.get("xyz", "0 0 0").split()]
                if len(xyz) != 3:
                    xyz = [0.0, 0.0, 0.0]
            except ValueError:
                xyz = [0.0, 0.0, 0.0]
            try:
                rpy = [float(v) for v in origin_el.get("rpy", "0 0 0").split()]
                if len(rpy) != 3:
                    rpy = [0.0, 0.0, 0.0]
            except ValueError:
                rpy = [0.0, 0.0, 0.0]

        entry = {
            "joint":       joint.get("name", ""),
            "link":        child_link,
            "sensor_type": sensor_type,
            "xyz":         [round(v, 6) for v in xyz],
            "rpy":         [round(v, 6) for v in rpy],
        }
        # For visual sensors add the actionable image correction.
        if sensor_type in ("camera", "depth_camera"):
            entry["image_rotation_deg"] = _image_rotation_from_roll(rpy[0])

        mounts.append(entry)

    return mounts


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


def _pkg_match_hints(hint_set, pkg_names):
    """Match hints against package names using token-level matching.

    Splits each package name on ``_`` and ``-`` before matching, so a hint
    like ``"nao"`` does **not** match ``"autonomous"`` or ``"scenario"`` — it
    only matches packages that have ``nao`` as a distinct token (e.g.
    ``"nao_robot"`` → tokens {``nao``, ``robot``}).

    Multi-token hints (those containing ``_`` or ``-``) fall back to substring
    matching on the full package name because they are already specific enough
    (e.g. ``"diff_drive"`` unambiguously identifies a mobile-base package).

    Returns a list of ``(hint, package_name)`` pairs for up to 5 matches
    (enough for evidence without flooding the output).
    """
    matches = []
    for pkg in pkg_names:
        pkg_lower = pkg.lower()
        # Token set for exact single-word hint matching.
        tokens = set(re.split(r"[_\-]+", pkg_lower)) - {"", "ros", "ros2", "pkg"}
        for h in hint_set:
            if "_" in h or "-" in h:
                # Compound hint → substring match is safe (specific enough).
                if h in pkg_lower:
                    matches.append((h, pkg))
                    break
            else:
                # Single-word hint → exact token match to avoid "nao" ⊂ "autonomous".
                if h in tokens:
                    matches.append((h, pkg))
                    break
        if len(matches) >= 5:
            break
    return matches


def _src_match_hints(hint_set, src_files, word_boundary=False, max_files=50):
    """Match hints against source file content.

    When *word_boundary* is True the patterns are wrapped in ``\\b`` anchors so
    that short tokens like ``"nao"`` do not match inside longer identifiers.

    Returns a list of ``(hint, filename)`` pairs (capped at 5 for readability).
    """
    if word_boundary:
        compiled = [(h, re.compile(r"\b" + re.escape(h) + r"\b", re.I))
                    for h in hint_set]
    else:
        compiled = [(h, re.compile(re.escape(h), re.I)) for h in hint_set]

    matches = []
    for fpath in src_files[:max_files]:
        try:
            text = pathlib.Path(fpath).read_text(encoding="utf-8", errors="replace")
        except Exception:
            continue
        fname = pathlib.Path(fpath).name
        for h, pat in compiled:
            if pat.search(text):
                matches.append((h, fname))
                if len(matches) >= 5:
                    return matches
    return matches


# ---------------------------------------------------------------------------
# URDF-based type evidence helpers
# ---------------------------------------------------------------------------

def _urdf_humanoid_evidence(joint_limits):
    """Return evidence strings when URDF joint names suggest a humanoid robot.

    Looks for joints whose names contain torso, neck, head, shoulder, elbow,
    wrist, ankle, knee, or hip axis names (hip_pitch / hip_roll / hip_yaw).
    Requires **≥ 4** such joints to reduce false positives from robots that
    happen to have a single ``head_pan`` joint.
    """
    _HUMANOID_JOINT_PATTERNS = {
        "torso", "neck", "head", "shoulder", "elbow", "wrist",
        "ankle", "knee", "hip_pitch", "hip_roll", "hip_yaw",
    }
    found = []
    for jname in joint_limits:
        jl = jname.lower()
        for pat in _HUMANOID_JOINT_PATTERNS:
            if pat in jl:
                found.append(f"urdf-joint:{jname}")
                break
    return found if len(found) >= 4 else []


def _urdf_legged_evidence(joint_limits):
    """Return evidence strings when URDF joint names suggest a legged robot.

    Looks for FL/FR/RL/RR leg-naming prefixes or _hip_ / _knee_ / _ankle_
    substrings.  Requires **≥ 4** such joints to distinguish a true legged
    platform from, e.g., a pan-tilt mechanism with two rotational joints.
    """
    _LEG_PREFIXES = {"fl_", "fr_", "rl_", "rr_", "lf_", "rf_", "lb_", "rb_"}
    _LEG_SUBSTRINGS = {"_hip_", "_knee_", "_ankle_", "_leg_", "_thigh_", "_shin_"}
    found = []
    for jname in joint_limits:
        jl = jname.lower()
        if any(jl.startswith(p) for p in _LEG_PREFIXES) or \
                any(s in jl for s in _LEG_SUBSTRINGS):
            found.append(f"urdf-joint:{jname}")
    return found if len(found) >= 4 else []


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

def _detect_robot_type(ws_pkg_names, all_src_files, velocity_topics,
                       joint_limits, has_nav2, robot_type_override=None):
    """Return ``(robot_type, robot_features, evidence)`` from static workspace signals.

    Parameters
    ----------
    ws_pkg_names : list[str]
        Package names from the workspace ``src/`` tree **only**.
        Ament-installed infrastructure packages are deliberately excluded —
        they don't indicate the robot's own type.
    all_src_files : list[str]
        Source paths for grep-based hinting.
    velocity_topics : list[str]
        Velocity topics discovered in the workspace.
    joint_limits : dict
        Joint names keyed from URDF files.
    has_nav2 : bool
        Whether Nav2 packages are present.
    robot_type_override : str | None
        If set, skip detection and use this value directly (user-specified).

    Returns
    -------
    robot_type : str
        Primary type (one of ``_VALID_ROBOT_TYPES``).
    robot_features : list[str]
        Supplementary features detected alongside the primary type,
        e.g. ``["pantilt"]``.
    evidence : dict[str, list[str]]
        Per-label evidence strings explaining which signals matched.
        Keys are type/feature names; values are short signal descriptions.

    Detection philosophy
    --------------------
    - Package names are checked with **token-level** matching (split on ``_``/``-``)
      so ``"nao"`` does not spuriously match ``"autonomous"`` or ``"scenario"``.
    - Source-code grep uses **word-boundary** anchors for short / ambiguous tokens
      (humanoid, legged, pantilt) and plain substring match for long/compound ones.
    - Humanoid detection additionally requires URDF joint-name confirmation if the
      only signal is a source-code match (package match alone is sufficient).
    - Type detection runs on workspace packages only; installed ROS 2 infrastructure
      packages do not influence the robot type.
    - ``robot_features`` captures supplementary capabilities (pan-tilt, gimbal …)
      that enrich the primary type without changing it.
    """
    # ── User override ─────────────────────────────────────────────────────────
    if robot_type_override:
        safe = robot_type_override if robot_type_override in _VALID_ROBOT_TYPES \
            else "unknown"
        return (safe, [], {"override": [f"user-specified: {robot_type_override}"]})

    evidence: dict = {}

    # ── Helper: check one type and record evidence ────────────────────────────
    def _check(hint_set, label, word_boundary=False):
        pkg_hits = _pkg_match_hints(hint_set, ws_pkg_names)
        if pkg_hits:
            evidence[label] = [f"pkg:{p}" for _, p in pkg_hits[:5]]
            return True
        src_hits = _src_match_hints(hint_set, all_src_files,
                                    word_boundary=word_boundary)
        if src_hits:
            evidence[label] = [f"src:{f}" for _, f in src_hits[:5]]
            return True
        return False

    # ── Humanoid — strictest: pkg token match OR (URDF ≥4 joints) ─────────────
    # Source-only match is NOT accepted for humanoid because generic words in
    # comments / tutorial code cause false positives on non-humanoid robots.
    hum_pkg_hits = _pkg_match_hints(_HUMANOID_HINTS, ws_pkg_names)
    hum_urdf = _urdf_humanoid_evidence(joint_limits)
    has_humanoid = bool(hum_pkg_hits or hum_urdf)
    if has_humanoid:
        sigs = ([f"pkg:{p}" for _, p in hum_pkg_hits[:3]] + hum_urdf[:3])
        evidence["humanoid"] = sigs[:5]

    # ── Legged — pkg token OR URDF leg-joint pattern ──────────────────────────
    leg_pkg_hits = _pkg_match_hints(_LEGGED_HINTS, ws_pkg_names)
    leg_urdf = _urdf_legged_evidence(joint_limits)
    leg_src_hits = [] if (leg_pkg_hits or leg_urdf) else \
        _src_match_hints(_LEGGED_HINTS, all_src_files)
    has_legged = bool(leg_pkg_hits or leg_urdf or leg_src_hits)
    if has_legged:
        sigs = ([f"pkg:{p}" for _, p in leg_pkg_hits[:3]]
                + leg_urdf[:3]
                + [f"src:{f}" for _, f in leg_src_hits[:2]])
        evidence["legged"] = sigs[:5]

    # ── Aerial ─────────────────────────────────────────────────────────────────
    has_aerial = _check(_AERIAL_HINTS, "aerial")
    if not has_aerial and velocity_topics:
        aerial_topics = [t for t in velocity_topics
                         if "altitude" in t or "takeoff" in t or "land" in t]
        if aerial_topics:
            has_aerial = True
            evidence["aerial"] = [f"topic:{t}" for t in aerial_topics[:3]]

    # ── Underwater / surface vessel ────────────────────────────────────────────
    has_underwater = _check(_UNDERWATER_HINTS, "underwater")
    has_surface_vessel = _check(_SURFACE_VESSEL_HINTS, "surface_vessel")

    # ── Arm — pkg/src hints OR ≥3 non-wheel URDF joints ───────────────────────
    has_arm = _check(_ARM_HINTS, "arm")
    if not has_arm and joint_limits:
        non_wheel = [n for n in joint_limits if "wheel" not in n.lower()]
        if len(non_wheel) >= 3:
            has_arm = True
            evidence["arm"] = [f"urdf-joint:{j}" for j in non_wheel[:5]]

    # ── Mobile base — velocity topics or Nav2 or explicit hints ───────────────
    has_mobile = False
    mobile_sigs = []
    if velocity_topics:
        has_mobile = True
        mobile_sigs.extend(f"topic:{t}" for t in velocity_topics[:3])
    if has_nav2:
        has_mobile = True
        mobile_sigs.append("ament:nav2")
    if not has_mobile:
        has_mobile = _check(_MOBILE_HINTS, "mobile_base")
    elif mobile_sigs:
        evidence["mobile_base"] = mobile_sigs

    # ── Supplementary features (don't affect primary type) ────────────────────
    robot_features: list = []
    pt_pkg = _pkg_match_hints(_PANTILT_HINTS, ws_pkg_names)
    pt_src = [] if pt_pkg else \
        _src_match_hints(_PANTILT_HINTS, all_src_files, word_boundary=True)
    if pt_pkg or pt_src:
        robot_features.append("pantilt")
        sigs = ([f"pkg:{p}" for _, p in pt_pkg[:3]]
                + [f"src:{f}" for _, f in pt_src[:3]])
        evidence["pantilt"] = sigs[:5]

    # ── Priority resolution ────────────────────────────────────────────────────
    if has_humanoid:
        robot_type = "humanoid"
    elif has_legged:
        robot_type = "legged"
    elif has_aerial:
        robot_type = "aerial"
    elif has_underwater:
        robot_type = "underwater"
    elif has_surface_vessel:
        robot_type = "surface_vessel"
    elif has_mobile and has_arm:
        robot_type = "mobile_manipulator"
    elif has_arm:
        robot_type = "arm"
    elif has_mobile:
        robot_type = "mobile_base"
    else:
        robot_type = "unknown"

    return robot_type, robot_features, evidence


# ---------------------------------------------------------------------------
# Launch file detail keying
# ---------------------------------------------------------------------------

def _make_detail_key(launch_path, existing_keys):
    """Return a unique key for this launch file in the detail dict.

    Uses the actual filename from the workspace (e.g. ``bringup.launch.py``).
    If that basename is already taken by another launch file in a different
    package, falls back to ``<package_dir>/<basename>`` to disambiguate.
    No name derivation or stem stripping — the key is always directly readable
    from the filesystem.
    """
    path = pathlib.Path(launch_path)
    basename = path.name  # e.g. bringup.launch.py — taken verbatim
    if basename not in existing_keys:
        return basename
    # Disambiguate with the immediate parent-of-launch directory (package name).
    pkg_name = path.parent.parent.name  # heuristic: <pkg>/launch/<file>
    return f"{pkg_name}/{basename}"


# ---------------------------------------------------------------------------
# Full static scan
# ---------------------------------------------------------------------------

def _run_static_scan(ws_path, distro, allow_live=False,
                     robot_type_override=None, verbose=False):
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
    # Each YAML file that contains at least one velocity limit is stored as a
    # separate source entry so the caller can see which config drove which limit.
    # The binding (most restrictive per axis across all sources) is computed here
    # and augmented in step 4 with URDF safety_controller values.
    scan_steps.append("yaml_limits")
    limit_sources: list = []   # [{file, path, linear_x, linear_y, angular_z}]
    best_lin_x, best_lin_y, best_ang_z = None, None, None

    def _upd_best(current, candidate):
        if candidate is None or candidate <= 0:
            return current
        return float(candidate) if current is None else min(current, float(candidate))

    for yf in all_yaml_files:
        lx, ly, az = _extract_limits_from_yaml(yf)
        if any(v is not None for v in (lx, ly, az)):
            limit_sources.append({
                "file": pathlib.Path(yf).name,
                "path": yf,
                "linear_x": lx,
                "linear_y": ly,
                "angular_z": az,
            })
            best_lin_x = _upd_best(best_lin_x, lx)
            best_lin_y = _upd_best(best_lin_y, ly)
            best_ang_z = _upd_best(best_ang_z, az)

    # --- 4. Joint limits and sensor mounts from URDF ---
    # joint_limits_by_model: {model_name: {joint_name: {velocity, effort, type}}}
    # Each URDF file is stored under its own <robot name="…"> so that test or
    # example URDFs in the workspace cannot pollute the main robot's joint list.
    scan_steps.append("urdf_limits")
    joint_limits_by_model: dict = {}   # nested — stored in profile output
    _flat_joint_limits: dict = {}      # flat — used only for type detection
    sensor_mounts: list = []
    seen_sensor_links: set = set()
    for uf in all_urdf_files:
        jl = _extract_joint_limits_from_urdf(uf)
        if jl:
            model_name = _get_urdf_robot_name(uf)
            joint_limits_by_model.setdefault(model_name, {}).update(jl)
            _flat_joint_limits.update(jl)
        # Also try safety_controller velocity (linear x only from wheel joints).
        lin, _ang = _extract_safety_velocity_from_urdf(uf)
        best_lin_x = _upd_best(best_lin_x, lin)
        # Sensor / actuator mount poses — deduplicated by child link name.
        for mount in _extract_sensor_mounts_from_urdf(uf):
            link = mount["link"]
            if link not in seen_sensor_links:
                seen_sensor_links.add(link)
                sensor_mounts.append(mount)

    # --- 5. Sensor / feature detection ---
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
    # Detection uses workspace packages only — ament-installed infrastructure
    # packages (nav2, moveit, …) don't characterise the robot's own type.
    robot_type, robot_features, robot_type_evidence = _detect_robot_type(
        ws_pkg_names=ws_pkg_names,
        all_src_files=all_src_files,
        velocity_topics=velocity_topics,
        joint_limits=_flat_joint_limits,
        has_nav2=has_nav2,
        robot_type_override=robot_type_override,
    )

    # --- 9. Safety limits final ---
    # safety_limits has two sections:
    #   sources  — one entry per YAML config that contained at least one limit;
    #              lets the caller see which teleop / planner / controller file
    #              drove each value.
    #   binding  — the most restrictive (minimum) value per axis across all
    #              YAML sources and URDF safety_controller elements; this is
    #              what agents pass to --max-vel / --max-ang.
    safety_limits = {
        "sources": limit_sources,
        "binding": {
            "linear_x": best_lin_x,
            "linear_y": best_lin_y,
            "angular_z": best_ang_z,
        },
    }

    # --- 10. Launch file details (one entry per file, keyed by filename) ---
    launch_file_details = {}
    for lf in all_launch_files:
        key = _make_detail_key(lf, launch_file_details)
        pkg_dir = pathlib.Path(lf).parent.parent  # heuristic: <pkg>/launch/<file>

        # Declared arguments (from ros2 launch --show-args, best-effort).
        launch_args = _query_launch_args(lf)

        # Sub-launch includes and how arguments are forwarded through them.
        includes = _parse_launch_includes(lf)

        # YAML and URDF files co-located with this launch file's package.
        lf_yaml = [
            yf for yf in all_yaml_files
            if pathlib.Path(yf).parent.parent == pkg_dir
        ]
        lf_urdf = [
            uf for uf in all_urdf_files
            if pathlib.Path(uf).parent.parent == pkg_dir
        ]
        # Joint limits per URDF model — same nested structure as the global table.
        lf_joints: dict = {}
        for uf in lf_urdf:
            jl = _extract_joint_limits_from_urdf(uf)
            if jl:
                model_name = _get_urdf_robot_name(uf)
                lf_joints.setdefault(model_name, {}).update(jl)

        launch_file_details[key] = {
            "path": lf,
            "package": pkg_dir.name,
            "launch_args": launch_args,
            "includes": includes,
            "yaml_files": lf_yaml,
            "urdf_files": lf_urdf,
            "joint_limits": lf_joints,
        }

    return {
        "scan_steps": scan_steps,
        "ws_packages": ws_pkg_names,
        "ament_packages": all_ament_pkg_names,
        "all_launch_files": all_launch_files,
        "all_urdf_files": all_urdf_files,
        "launch_file_details": launch_file_details,
        "velocity_topics": velocity_topics,
        "has_lidar": has_lidar,
        "has_camera": has_camera,
        "has_imu": has_imu,
        "has_nav2": has_nav2,
        "robot_type": robot_type,
        "robot_features": robot_features,
        "robot_type_evidence": robot_type_evidence,
        "safety_limits": safety_limits,
        "joint_limits": joint_limits_by_model,
        "sensor_mounts": sensor_mounts,
        "live_topics": live_topics,
    }


# ---------------------------------------------------------------------------
# Profile file I/O
# ---------------------------------------------------------------------------

def load_profile_summary():
    """Return the ``summary`` dict of the best available robot profile, or ``None``.

    This is the **public entry point** for other skill modules that want to
    read profile context before executing a command.  It is intentionally
    silent — it never writes to stdout, never raises, and never blocks.

    Selection strategy (first match wins):
    1. The single profile in ``.profiles/`` when there is exactly one.
    2. The most recently *modified* profile when there are several.

    Returns ``None`` when:
    - No profile has been scanned yet (no ``.profiles/`` directory or no JSON
      files inside it).
    - The profile file cannot be read or parsed.
    """
    try:
        if not _PROFILES_DIR.exists():
            return None
        profiles = sorted(_PROFILES_DIR.glob("*_profile.json"))
        if not profiles:
            return None
        # Prefer single profile; otherwise pick the most recently written one.
        chosen = profiles[0] if len(profiles) == 1 \
            else max(profiles, key=lambda p: p.stat().st_mtime)
        data = json.loads(chosen.read_text(encoding="utf-8"))
        return data.get("summary")
    except Exception:
        return None


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
            # robot_features: supplementary capabilities alongside the primary type.
            # e.g. ["pantilt"] for a mobile_base with a pan-tilt head.
            "robot_features": sr["robot_features"],
            # robot_type_evidence: signals that drove the detected type.
            # Each key is a type/feature label; value is a list of signal strings.
            "robot_type_evidence": sr["robot_type_evidence"],
            "packages": sr["ws_packages"],
            # launch_files: the filenames as they exist in the workspace.
            # Each entry is a key into the detail section.
            "launch_files": sorted(sr["launch_file_details"].keys()),
            "urdf_files": sr["all_urdf_files"],
            "velocity_topics": sr["velocity_topics"],
            "has_lidar": sr["has_lidar"],
            "has_camera": sr["has_camera"],
            "has_imu": sr["has_imu"],
            "has_nav2": sr["has_nav2"],
            "safety_limits": sr["safety_limits"],
            # joint_limits: {model_name: {joint_name: {velocity, effort, type}}}
            # Keyed by the <robot name="…"> from each URDF so example or test
            # URDFs in the workspace are clearly separated from the main robot.
            "joint_limits": sr["joint_limits"],
            # sensor_mounts: one entry per sensor/actuator link found in URDF.
            # Stores the physical xyz position and rpy orientation of each
            # sensor relative to its parent link.  Visual sensors
            # (camera, depth_camera) also carry image_rotation_deg — the
            # suggested correction to apply when capturing images.
            "sensor_mounts": sr["sensor_mounts"],
        },
        # ------------------------------------------------------------------ #
        # DETAIL — per launch file; load on demand via --section <filename>.  #
        # ------------------------------------------------------------------ #
        "detail": sr["launch_file_details"],
    }


# ---------------------------------------------------------------------------
# Commands
# ---------------------------------------------------------------------------

def cmd_profile_scan(args):
    """Scan the robot workspace and write a robot profile JSON."""
    user_ws = getattr(args, "workspace", None)
    robot_name = getattr(args, "name", None) or "robot"
    allow_live = getattr(args, "allow_live", False)
    robot_type_override = getattr(args, "robot_type", None)

    if robot_type_override and robot_type_override not in _VALID_ROBOT_TYPES:
        output({
            "error": f"Invalid --robot-type '{robot_type_override}'.",
            "valid_types": sorted(_VALID_ROBOT_TYPES),
        })
        return

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
        robot_type_override=robot_type_override,
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
        "launch_files_found": sorted(scan_result["launch_file_details"].keys()),
        "packages_found": len(scan_result["ws_packages"]),
        "robot_type": scan_result["robot_type"],
        "robot_features": scan_result["robot_features"],
        "robot_type_evidence": scan_result["robot_type_evidence"],
        "sensor_mounts": scan_result["sensor_mounts"],
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
            output({"robot_name": robot_name, "launch_file": section,
                    "detail": profile["detail"][section]})
        else:
            available = ["summary", "detail"] + list(profile.get("detail", {}).keys())
            output({
                "error": f"Section '{section}' not found.",
                "available_sections": available,
            })
    else:
        # Default: show summary + annotations + list of available detail sections.
        output({
            "robot_name": robot_name,
            "generated_at": profile.get("generated_at"),
            "workspace": profile.get("workspace"),
            "ros_distro": profile.get("ros_distro"),
            "summary": profile.get("summary", {}),
            # annotations: free-text notes added by the user via 'profile annotate'.
            # Always included so agents see them at session-start without an extra call.
            "annotations": profile.get("annotations", []),
            "detail_sections": list(profile.get("detail", {}).keys()),
            "hint": "Use --section <launch-filename> to load a launch file's full detail.",
        })


def cmd_profile_annotate(args):
    """Append a free-text annotation to the current robot profile.

    Annotations are stored persistently alongside the profile and are
    returned by every ``profile show`` call.  They are intended for
    information that cannot be auto-detected from the workspace — e.g.
    hardware quirks, known sensor calibration issues, or operational
    constraints that the agent must know about:

        profile annotate "Left motor encoder is worn — odometry drifts right.
                         Apply a slight left correction to cmd_vel."

        profile annotate "Camera image is horizontally mirrored because it
                         faces a reflective surface."

    Agents MUST read and apply annotations when executing commands —
    they are treated as mandatory operational context, not optional hints.
    """
    text = getattr(args, "text", None)
    if not text or not text.strip():
        output({"error": "Annotation text cannot be empty."})
        return

    robot_name = getattr(args, "name", None) or "robot"

    # Auto-detect profile if name not provided.
    existing = _load_profile(robot_name)
    if existing is None:
        if _PROFILES_DIR.exists():
            profiles = sorted(_PROFILES_DIR.glob("*_profile.json"))
            if len(profiles) == 1:
                robot_name = profiles[0].name.replace("_profile.json", "")
                existing = _load_profile(robot_name)

    if existing is None:
        output({
            "error": "No profile found to annotate.",
            "hint": "Run: python3 ros2_cli.py profile scan [--workspace PATH]",
        })
        return

    annotation = {
        "added_at": datetime.now(timezone.utc).isoformat(),
        "note": text.strip(),
    }
    existing.setdefault("annotations", []).append(annotation)
    _save_profile(existing, name=robot_name)

    output({
        "success": True,
        "robot_name": robot_name,
        "annotation_index": len(existing["annotations"]) - 1,
        "annotation": annotation,
        "total_annotations": len(existing["annotations"]),
        "hint": "Run 'profile show' to see all annotations.",
    })


def cmd_profile_rescan(args):
    """Rescan the robot workspace, updating the profile.

    With --launch-file FILENAME, rescans only that launch file's args
    (fast partial rescan).  Without it, performs a full rescan.
    Pass --robot-type TYPE on a full rescan to override the detected type.
    """
    robot_name = getattr(args, "name", None) or "robot"
    launch_filter = getattr(args, "launch_file", None)
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

    if launch_filter and existing:
        # Partial rescan: re-query launch args for the specified launch file.
        detail = existing.get("detail", {})
        if launch_filter not in detail:
            output({
                "error": f"Launch file '{launch_filter}' not found in existing profile.",
                "available": list(detail.keys()),
            })
            return
        lf = detail[launch_filter].get("path", "")
        new_args = _query_launch_args(lf) if lf else {}
        new_includes = _parse_launch_includes(lf) if lf else []
        detail[launch_filter]["launch_args"] = new_args
        detail[launch_filter]["includes"] = new_includes
        existing["detail"] = detail
        existing["generated_at"] = datetime.now(timezone.utc).isoformat()
        profile_file = _save_profile(existing, name=robot_name)
        output({
            "success": True,
            "mode": "partial",
            "launch_file": launch_filter,
            "profile_file": profile_file,
            "launch_args": new_args,
            "includes": new_includes,
        })
        return

    # Full rescan — delegate to scan.
    # Preserve any user-added annotations so they survive the rescan.
    preserved_annotations = (existing or {}).get("annotations", [])

    args.name = robot_name
    if user_ws is None and existing:
        # Re-use the workspace from the existing profile.
        args.workspace = existing.get("workspace") or None
    cmd_profile_scan(args)

    # Re-inject annotations into the freshly written profile (if any exist).
    if preserved_annotations:
        refreshed = _load_profile(robot_name)
        if refreshed is not None:
            refreshed["annotations"] = preserved_annotations
            _save_profile(refreshed, name=robot_name)


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
                "launch_files": data.get("summary", {}).get("launch_files", []),
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
