#!/usr/bin/env python3
"""AG-7 Safety Validator for ros2-skill.

Provides code-level enforcement of velocity limits, geofence constraints,
command allow/blocklists, and heartbeat watchdog. All checks run in-process
before any publish or action-send call.

No rclpy import at module level — the module is importable without a live
ROS 2 environment. rclpy is only used inside the heartbeat watchdog process.
"""

import copy
import json
import math
import os
import pathlib
import subprocess
import sys
import time

# ---------------------------------------------------------------------------
# Paths
# ---------------------------------------------------------------------------

_SCRIPT_DIR = pathlib.Path(__file__).resolve().parent
_SKILL_ROOT = _SCRIPT_DIR.parent
_CONFIG_PATH = _SKILL_ROOT / ".presets" / "safety.json"
_SENTINEL_PATH = _SKILL_ROOT / ".artifacts" / "heartbeat.sentinel"
_HEARTBEAT_LOG = _SKILL_ROOT / ".artifacts" / "heartbeat.log"
_WATCHDOG_SCRIPT = _SCRIPT_DIR / "ros2_watchdog.py"
_WATCHDOG_SESSION = "safety_watchdog"

# ---------------------------------------------------------------------------
# Velocity message types (mirrors ros2_topic.py)
# ---------------------------------------------------------------------------

VELOCITY_TYPES = {
    f"geometry_msgs{q}{t}"
    for q in ("/msg/", "/")
    for t in ("Twist", "TwistStamped")
}

# ---------------------------------------------------------------------------
# Default config
# ---------------------------------------------------------------------------

DEFAULT_CONFIG: dict = {
    "version": "1",
    "enabled": True,
    "velocity_limits": {
        "enabled": True,
        "on_violation": "reject",
        "linear_max_norm": 0.5,
        "angular_max_norm": 1.0,
        "per_axis": {
            "linear_x_max": None,
            "linear_y_max": None,
            "linear_z_max": None,
            "angular_x_max": None,
            "angular_y_max": None,
            "angular_z_max": None,
        },
        "topics": [],
    },
    "geofence": {
        "enabled": False,
        "frame_id": "map",
        "mode": "circle",
        "circle": {
            "center_x": 0.0,
            "center_y": 0.0,
            "radius_m": 10.0,
        },
        "rectangle": {
            "x_min": -5.0,
            "x_max": 5.0,
            "y_min": -5.0,
            "y_max": 5.0,
        },
        "on_violation": "reject",
    },
    "command_filter": {
        "enabled": False,
        "mode": "blocklist",
        "blocklist": [],
        "allowlist": [],
    },
    "heartbeat": {
        "enabled": False,
        "timeout_s": 5,
        "poll_interval_s": 1,
        "on_timeout": "estop",
    },
}

# Commands always implicitly allowed even in allowlist mode (prevent lockout).
_ALLOWLIST_IMPLICIT = {"safety", "version", "estop", "doctor"}

# Module-level config cache (per-process, lazy-loaded once).
_config_cache: dict | None = None


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _deep_merge(base: dict, override: dict) -> dict:
    """Recursively merge *override* into *base*, returning a new dict."""
    result = copy.deepcopy(base)
    for key, val in override.items():
        if key in result and isinstance(result[key], dict) and isinstance(val, dict):
            result[key] = _deep_merge(result[key], val)
        else:
            result[key] = val
    return result


def _warn(msg: str) -> None:
    print(json.dumps({"safety_config_warning": msg}), file=sys.stderr)


def _validate_config(cfg: dict) -> dict:
    """Validate numeric constraints; substitute defaults for bad values.

    Mutates *cfg* in-place and also returns it.
    """
    vl = cfg["velocity_limits"]
    if not isinstance(vl["linear_max_norm"], (int, float)) or vl["linear_max_norm"] <= 0:
        _warn(f"velocity_limits.linear_max_norm must be > 0; using default {DEFAULT_CONFIG['velocity_limits']['linear_max_norm']}")
        vl["linear_max_norm"] = DEFAULT_CONFIG["velocity_limits"]["linear_max_norm"]
    if not isinstance(vl["angular_max_norm"], (int, float)) or vl["angular_max_norm"] <= 0:
        _warn(f"velocity_limits.angular_max_norm must be > 0; using default {DEFAULT_CONFIG['velocity_limits']['angular_max_norm']}")
        vl["angular_max_norm"] = DEFAULT_CONFIG["velocity_limits"]["angular_max_norm"]

    for k, v in vl["per_axis"].items():
        if v is not None and (not isinstance(v, (int, float)) or v <= 0):
            _warn(f"velocity_limits.per_axis.{k} must be > 0 if set; ignoring")
            vl["per_axis"][k] = None

    if vl["on_violation"] not in ("reject", "cap", "warn"):
        _warn(f"velocity_limits.on_violation '{vl['on_violation']}' unrecognised; using 'reject'")
        vl["on_violation"] = "reject"

    gf = cfg["geofence"]
    if gf["on_violation"] not in ("reject", "warn"):
        _warn(f"geofence.on_violation '{gf['on_violation']}' unrecognised; using 'reject'")
        gf["on_violation"] = "reject"
    if gf["mode"] not in ("circle", "rectangle"):
        _warn(f"geofence.mode '{gf['mode']}' unrecognised; using 'circle'")
        gf["mode"] = "circle"
    if gf["circle"]["radius_m"] <= 0:
        _warn("geofence.circle.radius_m must be > 0; using default 10.0")
        gf["circle"]["radius_m"] = 10.0
    rect = gf["rectangle"]
    if rect["x_min"] >= rect["x_max"]:
        _warn("geofence.rectangle: x_min must be < x_max; using defaults")
        rect["x_min"] = DEFAULT_CONFIG["geofence"]["rectangle"]["x_min"]
        rect["x_max"] = DEFAULT_CONFIG["geofence"]["rectangle"]["x_max"]
    if rect["y_min"] >= rect["y_max"]:
        _warn("geofence.rectangle: y_min must be < y_max; using defaults")
        rect["y_min"] = DEFAULT_CONFIG["geofence"]["rectangle"]["y_min"]
        rect["y_max"] = DEFAULT_CONFIG["geofence"]["rectangle"]["y_max"]

    cf = cfg["command_filter"]
    if cf["mode"] not in ("blocklist", "allowlist"):
        _warn(f"command_filter.mode '{cf['mode']}' unrecognised; using 'blocklist'")
        cf["mode"] = "blocklist"

    hb = cfg["heartbeat"]
    if not isinstance(hb["timeout_s"], int) or hb["timeout_s"] < 1:
        _warn("heartbeat.timeout_s must be integer >= 1; using 5")
        hb["timeout_s"] = 5
    if hb["on_timeout"] not in ("estop", "log"):
        _warn(f"heartbeat.on_timeout '{hb['on_timeout']}' unrecognised; using 'estop'")
        hb["on_timeout"] = "estop"

    return cfg


# ---------------------------------------------------------------------------
# Config I/O
# ---------------------------------------------------------------------------

def get_config_path() -> pathlib.Path:
    return _CONFIG_PATH


def load_config(force: bool = False) -> dict:
    """Load and cache the safety config for this process.

    Falls back to DEFAULT_CONFIG on any error — a broken config must never
    disable the skill entirely.
    """
    global _config_cache
    if _config_cache is not None and not force:
        return _config_cache

    if not _CONFIG_PATH.exists():
        _config_cache = copy.deepcopy(DEFAULT_CONFIG)
        return _config_cache

    try:
        raw = _CONFIG_PATH.read_text(encoding="utf-8")
        user_cfg = json.loads(raw)
    except Exception as exc:
        _warn(f"Failed to parse safety.json: {exc}; using defaults")
        _config_cache = copy.deepcopy(DEFAULT_CONFIG)
        return _config_cache

    if user_cfg.get("version") not in ("1",):
        _warn(f"Unrecognised safety.json version '{user_cfg.get('version')}'; proceeding anyway")

    merged = _deep_merge(DEFAULT_CONFIG, user_cfg)
    _validate_config(merged)
    _config_cache = merged
    return _config_cache


def save_config(config: dict) -> None:
    """Persist config to disk and invalidate the in-process cache."""
    global _config_cache
    _CONFIG_PATH.parent.mkdir(parents=True, exist_ok=True)
    _CONFIG_PATH.write_text(json.dumps(config, indent=2), encoding="utf-8")
    _config_cache = None  # force reload on next call


def _load_fresh() -> dict:
    """Load config bypassing cache (used by mutating commands)."""
    return load_config(force=True)


# ---------------------------------------------------------------------------
# Velocity helpers
# ---------------------------------------------------------------------------

def is_velocity_msg_type(msg_type: str) -> bool:
    return msg_type in VELOCITY_TYPES


def extract_velocity_fields(msg_data: dict) -> dict:
    """Return a normalised dict with linear/angular components and a stamped flag."""
    if "twist" in msg_data:
        twist = msg_data["twist"]
        stamped = True
    else:
        twist = msg_data
        stamped = False

    def _vec(d: dict | None) -> dict:
        d = d or {}
        return {
            "x": float(d.get("x", 0.0)),
            "y": float(d.get("y", 0.0)),
            "z": float(d.get("z", 0.0)),
        }

    return {
        "linear": _vec(twist.get("linear")),
        "angular": _vec(twist.get("angular")),
        "stamped": stamped,
    }


def clamp_velocity(msg_data: dict, config: dict) -> dict:
    """Return a deep copy of msg_data with velocity components proportionally clamped."""
    cfg = config["velocity_limits"]
    fields = extract_velocity_fields(msg_data)
    lin = fields["linear"]
    ang = fields["angular"]

    # --- Norm-based scale factors ---
    lin_norm = math.sqrt(lin["x"] ** 2 + lin["y"] ** 2 + lin["z"] ** 2)
    ang_norm = math.sqrt(ang["x"] ** 2 + ang["y"] ** 2 + ang["z"] ** 2)

    lin_scale = min(1.0, cfg["linear_max_norm"] / lin_norm) if lin_norm > 0 else 1.0
    ang_scale = min(1.0, cfg["angular_max_norm"] / ang_norm) if ang_norm > 0 else 1.0

    # --- Per-axis scales ---
    per = cfg["per_axis"]
    axis_map = [
        ("linear_x_max",  lin,  "x"),
        ("linear_y_max",  lin,  "y"),
        ("linear_z_max",  lin,  "z"),
        ("angular_x_max", ang,  "x"),
        ("angular_y_max", ang,  "y"),
        ("angular_z_max", ang,  "z"),
    ]
    per_axis_scales_lin = 1.0
    per_axis_scales_ang = 1.0
    for key, vec, axis in axis_map:
        limit = per.get(key)
        if limit is None:
            continue
        val = abs(vec[axis])
        if val > 0:
            s = min(1.0, limit / val)
        else:
            s = 1.0
        if vec is lin:
            per_axis_scales_lin = min(per_axis_scales_lin, s)
        else:
            per_axis_scales_ang = min(per_axis_scales_ang, s)

    lin_final = min(lin_scale, per_axis_scales_lin)
    ang_final = min(ang_scale, per_axis_scales_ang)

    # Build clamped message
    result = copy.deepcopy(msg_data)

    def _apply(twist_dict: dict) -> None:
        l_ = twist_dict.get("linear", {})
        a_ = twist_dict.get("angular", {})
        for axis in ("x", "y", "z"):
            if axis in l_:
                l_[axis] = round(l_[axis] * lin_final, 6)
            if axis in a_:
                a_[axis] = round(a_[axis] * ang_final, 6)

    if fields["stamped"]:
        _apply(result.get("twist", {}))
    else:
        _apply(result)

    return result


# ---------------------------------------------------------------------------
# Check functions
# ---------------------------------------------------------------------------

def check_velocity(
    msg_data: dict,
    msg_type: str,
    topic: str = "",
    config: dict | None = None,
) -> tuple[bool, str | None, dict]:
    """Validate velocity message against configured limits.

    Returns (ok, reason, detail).
    """
    cfg = config or load_config()
    if not cfg["enabled"] or not cfg["velocity_limits"]["enabled"]:
        return True, None, {}
    if not is_velocity_msg_type(msg_type):
        return True, None, {}

    vlim = cfg["velocity_limits"]
    # Topic scoping: if topics list is non-empty, skip topics not in it.
    if vlim["topics"] and topic and topic not in vlim["topics"]:
        return True, None, {}

    fields = extract_velocity_fields(msg_data)
    lin = fields["linear"]
    ang = fields["angular"]

    violations: list[str] = []

    lin_norm = math.sqrt(lin["x"] ** 2 + lin["y"] ** 2 + lin["z"] ** 2)
    ang_norm = math.sqrt(ang["x"] ** 2 + ang["y"] ** 2 + ang["z"] ** 2)

    if lin_norm > vlim["linear_max_norm"]:
        violations.append(
            f"linear velocity norm {lin_norm:.3f} m/s exceeds limit {vlim['linear_max_norm']} m/s"
        )
    if ang_norm > vlim["angular_max_norm"]:
        violations.append(
            f"angular velocity norm {ang_norm:.3f} rad/s exceeds limit {vlim['angular_max_norm']} rad/s"
        )

    axis_map = {
        "linear_x_max":  (lin, "x", "m/s"),
        "linear_y_max":  (lin, "y", "m/s"),
        "linear_z_max":  (lin, "z", "m/s"),
        "angular_x_max": (ang, "x", "rad/s"),
        "angular_y_max": (ang, "y", "rad/s"),
        "angular_z_max": (ang, "z", "rad/s"),
    }
    for key, (vec, axis, unit) in axis_map.items():
        limit = vlim["per_axis"].get(key)
        if limit is not None:
            val = abs(vec[axis])
            if val > limit:
                violations.append(
                    f"{('linear' if vec is lin else 'angular')}.{axis} = {val:.3f} {unit} "
                    f"exceeds per-axis limit {limit} {unit}"
                )

    if violations:
        detail = {
            "violations": violations,
            "limits": {
                "linear_max_norm": vlim["linear_max_norm"],
                "angular_max_norm": vlim["angular_max_norm"],
                "per_axis": {k: v for k, v in vlim["per_axis"].items() if v is not None},
            },
            "actual": {
                "linear_norm": round(lin_norm, 4),
                "angular_norm": round(ang_norm, 4),
            },
        }
        return False, "; ".join(violations), detail

    return True, None, {}


def check_geofence(
    x: float,
    y: float,
    config: dict | None = None,
) -> tuple[bool, str | None, dict]:
    """Check whether (x, y) lies within the configured geofence.

    Returns (ok, reason, detail).
    """
    cfg = config or load_config()
    gf = cfg["geofence"]

    if gf["mode"] == "circle":
        cx = gf["circle"]["center_x"]
        cy = gf["circle"]["center_y"]
        r = gf["circle"]["radius_m"]
        dist = math.sqrt((x - cx) ** 2 + (y - cy) ** 2)
        if dist > r:
            return False, (
                f"goal ({x:.3f}, {y:.3f}) is {dist:.3f} m from center "
                f"({cx}, {cy}), exceeds radius {r} m"
            ), {"distance_m": round(dist, 4), "radius_m": r}
    else:  # rectangle
        rect = gf["rectangle"]
        violations = []
        if x < rect["x_min"] or x > rect["x_max"]:
            violations.append(f"x={x:.3f} outside [{rect['x_min']}, {rect['x_max']}]")
        if y < rect["y_min"] or y > rect["y_max"]:
            violations.append(f"y={y:.3f} outside [{rect['y_min']}, {rect['y_max']}]")
        if violations:
            return False, "; ".join(violations), {"bounds": rect}

    return True, None, {}


def check_command(
    command: str,
    subcommand: str | None,
    config: dict | None = None,
) -> tuple[bool, str | None, dict]:
    """Check whether a command is permitted by the command filter.

    Returns (ok, reason, detail).
    """
    cfg = config or load_config()
    cf = cfg["command_filter"]

    if not cfg["enabled"] or not cf["enabled"]:
        return True, None, {}

    # Build the command key strings to test.
    exact_key = f"{command} {subcommand}".strip() if subcommand else command
    group_key = command

    mode = cf["mode"]

    if mode == "blocklist":
        for entry in cf["blocklist"]:
            if exact_key == entry or group_key == entry:
                return False, f"'{exact_key}' is in the command blocklist", {}
        return True, None, {}

    else:  # allowlist
        # Implicit grants — never lockable.
        if command in _ALLOWLIST_IMPLICIT:
            return True, None, {}
        for entry in cf["allowlist"]:
            if exact_key == entry or group_key == entry:
                return True, None, {}
        return False, (
            f"'{exact_key}' is not in the command allowlist. "
            "Run 'safety show' to see the allowlist."
        ), {}


# ---------------------------------------------------------------------------
# Combined pre-publish check
# ---------------------------------------------------------------------------

def validate_publish(
    topic: str,
    msg_data: dict,
    msg_type: str,
    config: dict | None = None,
) -> tuple[bool, str, dict | None, dict]:
    """Run all applicable checks before a publish call.

    Returns (ok, action, modified_msg, detail) where:
      action: "allow" | "warn" | "cap" | "block"
      modified_msg: set when action == "cap"
    """
    cfg = config or load_config()
    ok, reason, detail = check_velocity(msg_data, msg_type, topic, cfg)

    if ok:
        return True, "allow", None, {}

    on_violation = cfg["velocity_limits"]["on_violation"]

    if on_violation == "reject":
        return False, "block", None, detail

    if on_violation == "cap":
        capped = clamp_velocity(msg_data, cfg)
        return True, "cap", capped, detail

    # warn mode
    return True, "warn", None, detail


# ---------------------------------------------------------------------------
# Heartbeat sentinel
# ---------------------------------------------------------------------------

def safety_heartbeat_touch() -> None:
    """Update the sentinel file mtime. Called by motion commands when heartbeat is enabled."""
    cfg = load_config()
    if not cfg["enabled"] or not cfg["heartbeat"]["enabled"]:
        return
    try:
        _SENTINEL_PATH.parent.mkdir(parents=True, exist_ok=True)
        if _SENTINEL_PATH.exists():
            os.utime(_SENTINEL_PATH, None)
        else:
            _SENTINEL_PATH.write_text(
                json.dumps({
                    "started_at": time.time(),
                    "last_ping": time.time(),
                    "timeout_s": cfg["heartbeat"]["timeout_s"],
                }),
                encoding="utf-8",
            )
    except Exception:
        pass  # heartbeat must never crash motion commands


def _watchdog_running() -> bool:
    """Return True if the safety_watchdog tmux session exists."""
    try:
        result = subprocess.run(
            ["tmux", "has-session", "-t", _WATCHDOG_SESSION],
            capture_output=True,
        )
        return result.returncode == 0
    except Exception:
        return False


# ---------------------------------------------------------------------------
# output helper (mirrors ros2_utils.output without importing rclpy)
# ---------------------------------------------------------------------------

def _output(data: dict) -> None:
    print(json.dumps(data, default=str))


# ---------------------------------------------------------------------------
# Safety command handlers
# ---------------------------------------------------------------------------

def cmd_safety_show(args) -> None:
    cfg = _load_fresh()
    _output({
        "safety_config": cfg,
        "config_path": str(_CONFIG_PATH),
        "config_exists": _CONFIG_PATH.exists(),
    })


def cmd_safety_enable(args) -> None:
    cfg = _load_fresh()
    cfg["enabled"] = True
    save_config(cfg)
    _output({"success": True, "enabled": True})


def cmd_safety_disable(args) -> None:
    cfg = _load_fresh()
    cfg["enabled"] = False
    save_config(cfg)
    _output({
        "success": True,
        "enabled": False,
        "warning": "All safety checks are now disabled. Use 'safety enable' to re-enable.",
    })


def cmd_safety_set_velocity(args) -> None:
    cfg = _load_fresh()
    vl = cfg["velocity_limits"]
    changed = {}

    linear = getattr(args, "linear", None)
    angular = getattr(args, "angular", None)
    axis = getattr(args, "axis", None)
    value = getattr(args, "value", None)

    if linear is not None:
        if linear <= 0:
            return _output({"error": "--linear must be > 0"})
        vl["linear_max_norm"] = linear
        changed["linear_max_norm"] = linear

    if angular is not None:
        if angular <= 0:
            return _output({"error": "--angular must be > 0"})
        vl["angular_max_norm"] = angular
        changed["angular_max_norm"] = angular

    if axis is not None:
        valid_axes = list(DEFAULT_CONFIG["velocity_limits"]["per_axis"].keys())
        if axis not in valid_axes:
            return _output({"error": f"--axis must be one of: {', '.join(valid_axes)}"})
        if value is None:
            return _output({"error": "--value is required when --axis is used"})
        if value <= 0:
            return _output({"error": "--value must be > 0"})
        vl["per_axis"][axis] = value
        changed[f"per_axis.{axis}"] = value

    if not changed:
        return _output({"error": "Specify --linear, --angular, or --axis with --value"})

    save_config(cfg)
    _output({"success": True, "changed": changed, "velocity_limits": vl})


def cmd_safety_set_geofence(args) -> None:
    cfg = _load_fresh()
    gf = cfg["geofence"]
    mode = getattr(args, "mode", None)

    if mode == "circle":
        cx = getattr(args, "cx", None)
        cy = getattr(args, "cy", None)
        radius = getattr(args, "radius", None)
        if cx is None or cy is None or radius is None:
            return _output({"error": "circle mode requires --cx, --cy, --radius"})
        if radius <= 0:
            return _output({"error": "--radius must be > 0"})
        gf["mode"] = "circle"
        gf["circle"] = {"center_x": cx, "center_y": cy, "radius_m": radius}

    elif mode == "rectangle":
        xmin = getattr(args, "xmin", None)
        xmax = getattr(args, "xmax", None)
        ymin = getattr(args, "ymin", None)
        ymax = getattr(args, "ymax", None)
        if any(v is None for v in [xmin, xmax, ymin, ymax]):
            return _output({"error": "rectangle mode requires --xmin, --xmax, --ymin, --ymax"})
        if xmin >= xmax:
            return _output({"error": "--xmin must be < --xmax"})
        if ymin >= ymax:
            return _output({"error": "--ymin must be < --ymax"})
        gf["mode"] = "rectangle"
        gf["rectangle"] = {"x_min": xmin, "x_max": xmax, "y_min": ymin, "y_max": ymax}

    else:
        return _output({"error": "Specify --mode circle or --mode rectangle"})

    frame = getattr(args, "frame", None)
    if frame:
        gf["frame_id"] = frame
    on_violation = getattr(args, "on_violation", None)
    if on_violation:
        if on_violation not in ("reject", "warn"):
            return _output({"error": "--on-violation must be 'reject' or 'warn'"})
        gf["on_violation"] = on_violation

    save_config(cfg)
    _output({"success": True, "geofence": gf})


def cmd_safety_geofence_enable(args) -> None:
    cfg = _load_fresh()
    cfg["geofence"]["enabled"] = True
    save_config(cfg)
    _output({"success": True, "geofence_enabled": True, "geofence": cfg["geofence"]})


def cmd_safety_geofence_disable(args) -> None:
    cfg = _load_fresh()
    cfg["geofence"]["enabled"] = False
    save_config(cfg)
    _output({"success": True, "geofence_enabled": False})


def cmd_safety_block(args) -> None:
    cmd = args.block_command
    sub = getattr(args, "block_subcommand", None)
    entry = f"{cmd} {sub}".strip() if sub else cmd

    cfg = _load_fresh()
    cf = cfg["command_filter"]
    if entry in cf["blocklist"]:
        return _output({"success": True, "already_blocked": True, "entry": entry})

    cf["blocklist"].append(entry)
    if not cf["enabled"]:
        cf["enabled"] = True
    save_config(cfg)
    _output({"success": True, "blocked": entry, "blocklist": cf["blocklist"]})


def cmd_safety_unblock(args) -> None:
    cmd = args.block_command
    sub = getattr(args, "block_subcommand", None)
    entry = f"{cmd} {sub}".strip() if sub else cmd

    cfg = _load_fresh()
    cf = cfg["command_filter"]
    if entry not in cf["blocklist"]:
        return _output({"success": False, "error": f"'{entry}' is not in the blocklist"})

    cf["blocklist"].remove(entry)
    save_config(cfg)
    _output({"success": True, "unblocked": entry, "blocklist": cf["blocklist"]})


def cmd_safety_validate(args) -> None:
    """Dry-run: run all checks and report what would happen. Does not publish."""
    topic = args.topic
    msg_str = args.msg
    msg_type = getattr(args, "msg_type", None)

    try:
        msg_data = json.loads(msg_str)
    except (json.JSONDecodeError, TypeError, ValueError) as e:
        return _output({"error": f"Invalid JSON: {e}"})

    cfg = _load_fresh()

    # Velocity check
    vel_ok, vel_reason, vel_detail = check_velocity(msg_data, msg_type or "", topic, cfg)
    on_violation = cfg["velocity_limits"]["on_violation"]
    if not vel_ok:
        if on_violation == "reject":
            vel_result = "block"
        elif on_violation == "cap":
            vel_result = "cap"
        else:
            vel_result = "warn"
    else:
        vel_result = "allow"

    checks: dict = {
        "velocity": {"result": vel_result},
        "command_filter": {"result": "skipped", "reason": "validate is not a topic publish"},
        "geofence": {"result": "skipped", "reason": "not a navigate_to_pose goal"},
    }
    if vel_detail:
        checks["velocity"].update(vel_detail)

    # Determine overall action
    if vel_result == "block":
        action = "block"
    elif vel_result == "cap":
        action = "cap"
    elif vel_result == "warn":
        action = "warn"
    else:
        action = "allow"

    result = {
        "topic": topic,
        "msg_type": msg_type or "(not provided — velocity checks skipped)",
        "action": action,
        "checks": checks,
    }
    if action == "cap":
        result["capped_msg"] = clamp_velocity(msg_data, cfg)

    _output(result)


def cmd_safety_reset(args) -> None:
    yes = getattr(args, "yes", False)
    if not yes:
        _output({
            "confirm_required": True,
            "message": "This will reset safety.json to defaults. Pass --yes to confirm.",
        })
        return
    save_config(copy.deepcopy(DEFAULT_CONFIG))
    _output({"success": True, "message": "Safety config reset to defaults.", "config": DEFAULT_CONFIG})


def cmd_safety_heartbeat_start(args) -> None:
    timeout = getattr(args, "timeout", None) or DEFAULT_CONFIG["heartbeat"]["timeout_s"]
    poll = getattr(args, "poll", None) or DEFAULT_CONFIG["heartbeat"]["poll_interval_s"]

    if _watchdog_running():
        return _output({
            "success": False,
            "error": "Watchdog session 'safety_watchdog' is already running.",
            "hint": "Run 'safety heartbeat stop' first to restart.",
        })

    # Write initial sentinel
    _SENTINEL_PATH.parent.mkdir(parents=True, exist_ok=True)
    _SENTINEL_PATH.write_text(
        json.dumps({
            "started_at": time.time(),
            "last_ping": time.time(),
            "timeout_s": timeout,
        }),
        encoding="utf-8",
    )

    # Update config
    cfg = _load_fresh()
    cfg["heartbeat"]["timeout_s"] = timeout
    cfg["heartbeat"]["poll_interval_s"] = poll
    cfg["heartbeat"]["enabled"] = True
    save_config(cfg)

    # Launch watchdog in detached tmux session
    try:
        subprocess.run(
            [
                "tmux", "new-session", "-d", "-s", _WATCHDOG_SESSION,
                f"python3 {_WATCHDOG_SCRIPT}",
            ],
            check=True,
        )
        _output({
            "success": True,
            "session": _WATCHDOG_SESSION,
            "timeout_s": timeout,
            "poll_interval_s": poll,
            "sentinel": str(_SENTINEL_PATH),
        })
    except subprocess.CalledProcessError as e:
        _output({"error": f"Failed to start watchdog tmux session: {e}"})
    except FileNotFoundError:
        _output({"error": "tmux not found. Install tmux to use the heartbeat watchdog."})


def cmd_safety_heartbeat_stop(args) -> None:
    if not _watchdog_running():
        _output({"success": True, "note": "Watchdog was not running."})
    else:
        try:
            subprocess.run(
                ["tmux", "kill-session", "-t", _WATCHDOG_SESSION],
                check=True,
            )
        except subprocess.CalledProcessError:
            pass

    # Disable in config
    cfg = _load_fresh()
    cfg["heartbeat"]["enabled"] = False
    save_config(cfg)
    _output({"success": True, "session": _WATCHDOG_SESSION, "stopped": True})


def cmd_safety_heartbeat_ping(args) -> None:
    safety_heartbeat_touch()
    ts = time.strftime("%Y-%m-%dT%H:%M:%S", time.localtime())
    _output({"success": True, "sentinel_touched": ts, "sentinel": str(_SENTINEL_PATH)})


def cmd_safety_heartbeat_status(args) -> None:
    running = _watchdog_running()
    sentinel_exists = _SENTINEL_PATH.exists()
    last_ping = None
    age_s = None
    if sentinel_exists:
        try:
            mtime = _SENTINEL_PATH.stat().st_mtime
            age_s = round(time.time() - mtime, 1)
            last_ping = time.strftime("%Y-%m-%dT%H:%M:%S", time.localtime(mtime))
        except Exception:
            pass
    cfg = _load_fresh()
    _output({
        "watchdog_running": running,
        "heartbeat_enabled": cfg["heartbeat"]["enabled"],
        "timeout_s": cfg["heartbeat"]["timeout_s"],
        "sentinel_exists": sentinel_exists,
        "last_ping": last_ping,
        "sentinel_age_s": age_s,
        "session": _WATCHDOG_SESSION,
    })
