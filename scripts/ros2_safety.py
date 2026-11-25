#!/usr/bin/env python3
"""Safety policy helpers and CLI command handlers for ros2-skill.

This module is intentionally lightweight and keeps ROS imports out of module
scope so config and policy checks stay fast.
"""

import copy
import json
import math
import os
import subprocess
import time
from typing import Any, Dict, List, Optional, Tuple

from ros2_utils import check_tmux, kill_session, output, run_cmd, session_exists
from ros2_profile import load_profile_record


_DEFAULT_SKILL_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
_PRESETS_DIR = os.path.join(_DEFAULT_SKILL_ROOT, ".presets")
_ARTIFACTS_DIR = os.path.join(_DEFAULT_SKILL_ROOT, ".artifacts")
_SAFETY_CONFIG_PATH = os.path.join(_PRESETS_DIR, "safety.json")
_HEARTBEAT_SENTINEL = os.path.join(_ARTIFACTS_DIR, "heartbeat.sentinel")
_HEARTBEAT_LOG = os.path.join(_ARTIFACTS_DIR, "heartbeat.log")
_WATCHDOG_SESSION = "safety_watchdog"
_WATCHDOG_SCRIPT = os.path.join(_DEFAULT_SKILL_ROOT, "scripts", "ros2_watchdog.py")

_VELOCITY_TYPES = {"geometry_msgs/msg/Twist", "geometry_msgs/msg/TwistStamped"}


DEFAULT_CONFIG: Dict[str, Any] = {
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


def _runtime_base_dir(base_dir: Optional[str] = None) -> str:
    if base_dir:
        return base_dir
    return _DEFAULT_SKILL_ROOT


def _resolve_runtime_paths(base_dir: Optional[str] = None) -> Dict[str, str]:
    if base_dir is not None:
        root = base_dir
        return {
            "root": root,
            "presets_dir": os.path.join(root, ".presets"),
            "artifacts_dir": os.path.join(root, ".artifacts"),
            "safety_config_path": os.path.join(root, ".presets", "safety.json"),
            "heartbeat_sentinel": os.path.join(root, ".artifacts", "heartbeat.sentinel"),
            "heartbeat_log": os.path.join(root, ".artifacts", "heartbeat.log"),
            "watchdog_script": os.path.join(root, "scripts", "ros2_watchdog.py"),
        }

    root = _runtime_base_dir(None)
    config_path = globals().get("_SAFETY_CONFIG_PATH", os.path.join(root, ".presets", "safety.json"))
    heartbeat_sentinel = globals().get("_HEARTBEAT_SENTINEL", os.path.join(root, ".artifacts", "heartbeat.sentinel"))
    heartbeat_log = globals().get("_HEARTBEAT_LOG", os.path.join(root, ".artifacts", "heartbeat.log"))
    presets_dir = globals().get("_PRESETS_DIR", os.path.join(root, ".presets"))
    artifacts_dir = globals().get("_ARTIFACTS_DIR", os.path.join(root, ".artifacts"))
    watchdog = globals().get("_WATCHDOG_SCRIPT", os.path.join(root, "scripts", "ros2_watchdog.py"))

    return {
        "root": root,
        "presets_dir": presets_dir,
        "artifacts_dir": artifacts_dir,
        "safety_config_path": config_path,
        "heartbeat_sentinel": heartbeat_sentinel,
        "heartbeat_log": heartbeat_log,
        "watchdog_script": watchdog,
    }


def _ensure_dirs(base_dir: Optional[str] = None) -> Dict[str, str]:
    paths = _resolve_runtime_paths(base_dir)
    os.makedirs(paths["presets_dir"], exist_ok=True)
    os.makedirs(paths["artifacts_dir"], exist_ok=True)
    return paths


def get_config_path(base_dir: Optional[str] = None) -> str:
    return _resolve_runtime_paths(base_dir)["safety_config_path"]


def _deep_merge(base: Dict[str, Any], override: Dict[str, Any]) -> Dict[str, Any]:
    merged = copy.deepcopy(base)
    for k, v in (override or {}).items():
        if isinstance(v, dict) and isinstance(merged.get(k), dict):
            merged[k] = _deep_merge(merged[k], v)
        else:
            merged[k] = v
    return merged


def _clean_positive(value: Any, default: float, warnings: List[str], name: str) -> float:
    try:
        v = float(value)
        if v > 0:
            return v
    except Exception:
        pass
    warnings.append(f"invalid {name}; using default {default}")
    return default


def _clean_int_min(value: Any, default: int, min_v: int, warnings: List[str], name: str) -> int:
    try:
        v = int(value)
        if v >= min_v:
            return v
    except Exception:
        pass
    warnings.append(f"invalid {name}; using default {default}")
    return default


def _normalize_msg_type(msg_type: str) -> str:
    return (msg_type or "").replace("/msg/", "/")


def _normalize_command(command: Optional[str], subcommand: Optional[str]) -> str:
    cmd = (command or "").strip()
    sub = (subcommand or "").strip()
    if sub:
        return f"{cmd} {sub}".strip()
    return cmd


def _normalize_cmd_rule(entry: str) -> str:
    return " ".join((entry or "").strip().split())


def _validate_config(config: Dict[str, Any]) -> Tuple[Dict[str, Any], List[str]]:
    warnings: List[str] = []
    cfg = copy.deepcopy(config)

    if cfg.get("version") != "1":
        warnings.append("unrecognized safety config version; continuing with compatibility mode")

    vlim = cfg["velocity_limits"]
    vlim["enabled"] = bool(vlim.get("enabled", True))
    mode = vlim.get("on_violation", "reject")
    if mode not in {"reject", "cap", "warn"}:
        warnings.append("invalid velocity_limits.on_violation; using reject")
        mode = "reject"
    vlim["on_violation"] = mode
    vlim["linear_max_norm"] = _clean_positive(vlim.get("linear_max_norm"), 0.5, warnings, "velocity_limits.linear_max_norm")
    vlim["angular_max_norm"] = _clean_positive(vlim.get("angular_max_norm"), 1.0, warnings, "velocity_limits.angular_max_norm")
    if not isinstance(vlim.get("topics"), list):
        warnings.append("invalid velocity_limits.topics; using []")
        vlim["topics"] = []

    axis = vlim.get("per_axis", {})
    if not isinstance(axis, dict):
        axis = {}
    for k in [
        "linear_x_max", "linear_y_max", "linear_z_max",
        "angular_x_max", "angular_y_max", "angular_z_max",
    ]:
        value = axis.get(k)
        if value is None:
            continue
        try:
            num = float(value)
            if num <= 0:
                raise ValueError("must be > 0")
            axis[k] = num
        except Exception:
            warnings.append(f"invalid velocity_limits.per_axis.{k}; using null")
            axis[k] = None
    vlim["per_axis"] = axis

    gf = cfg["geofence"]
    gf["enabled"] = bool(gf.get("enabled", False))
    gf_mode = gf.get("mode", "circle")
    if gf_mode not in {"circle", "rectangle"}:
        warnings.append("invalid geofence.mode; using circle")
        gf_mode = "circle"
    gf["mode"] = gf_mode
    gf_viol = gf.get("on_violation", "reject")
    if gf_viol not in {"reject", "warn"}:
        warnings.append("invalid geofence.on_violation; using reject")
        gf_viol = "reject"
    gf["on_violation"] = gf_viol

    circle = gf.get("circle", {})
    if not isinstance(circle, dict):
        circle = {}
    circle["radius_m"] = _clean_positive(circle.get("radius_m"), 10.0, warnings, "geofence.circle.radius_m")
    circle["center_x"] = float(circle.get("center_x", 0.0))
    circle["center_y"] = float(circle.get("center_y", 0.0))
    gf["circle"] = circle

    rect = gf.get("rectangle", {})
    if not isinstance(rect, dict):
        rect = {}
    x_min = float(rect.get("x_min", -5.0))
    x_max = float(rect.get("x_max", 5.0))
    y_min = float(rect.get("y_min", -5.0))
    y_max = float(rect.get("y_max", 5.0))
    if not (x_min < x_max and y_min < y_max):
        warnings.append("invalid geofence.rectangle bounds; using defaults")
        x_min, x_max, y_min, y_max = -5.0, 5.0, -5.0, 5.0
    rect.update({"x_min": x_min, "x_max": x_max, "y_min": y_min, "y_max": y_max})
    gf["rectangle"] = rect

    cf = cfg["command_filter"]
    cf["enabled"] = bool(cf.get("enabled", False))
    cf_mode = cf.get("mode", "blocklist")
    if cf_mode not in {"blocklist", "allowlist"}:
        warnings.append("invalid command_filter.mode; using blocklist")
        cf_mode = "blocklist"
    cf["mode"] = cf_mode
    if not isinstance(cf.get("blocklist"), list):
        warnings.append("invalid command_filter.blocklist; using []")
        cf["blocklist"] = []
    if not isinstance(cf.get("allowlist"), list):
        warnings.append("invalid command_filter.allowlist; using []")
        cf["allowlist"] = []

    hb = cfg["heartbeat"]
    hb["enabled"] = bool(hb.get("enabled", False))
    hb["timeout_s"] = _clean_int_min(hb.get("timeout_s"), 5, 1, warnings, "heartbeat.timeout_s")
    hb["poll_interval_s"] = _clean_int_min(hb.get("poll_interval_s"), 1, 1, warnings, "heartbeat.poll_interval_s")
    hb_mode = hb.get("on_timeout", "estop")
    if hb_mode not in {"estop", "log"}:
        warnings.append("invalid heartbeat.on_timeout; using estop")
        hb_mode = "estop"
    hb["on_timeout"] = hb_mode

    cfg["enabled"] = bool(cfg.get("enabled", True))
    return cfg, warnings


def _load_profile_safety_overlay(base_dir: Optional[str] = None) -> Dict[str, Any]:
    """Return the active profile's safety overlay, or {} when none exists."""
    paths = _resolve_runtime_paths(base_dir)
    profiles_dir = os.path.join(paths["root"], ".profiles")
    try:
        if not os.path.isdir(profiles_dir):
            return {}

        profile = load_profile_record(profiles_dir=profiles_dir, readonly=True)
        if isinstance(profile, dict):
            overlay = profile.get("safety_overrides")
            return copy.deepcopy(overlay) if isinstance(overlay, dict) else {}
        return {}
    except Exception:
        return {}


def load_config(create_if_missing: bool = True, base_dir: Optional[str] = None) -> Dict[str, Any]:
    paths = _ensure_dirs(base_dir)
    config_path = paths["safety_config_path"]
    profile_overlay = _load_profile_safety_overlay(base_dir=base_dir)
    if not os.path.exists(config_path):
        if create_if_missing:
            save_config(DEFAULT_CONFIG, base_dir=base_dir)
        merged_default = _deep_merge(DEFAULT_CONFIG, profile_overlay)
        validated, _warnings = _validate_config(merged_default)
        return validated

    try:
        with open(config_path, "r", encoding="utf-8") as f:
            user_cfg = json.load(f)
    except Exception:
        return copy.deepcopy(DEFAULT_CONFIG)

    merged = _deep_merge(DEFAULT_CONFIG, profile_overlay)
    merged = _deep_merge(merged, user_cfg if isinstance(user_cfg, dict) else {})
    validated, _warnings = _validate_config(merged)
    return validated


def save_config(config: Dict[str, Any], base_dir: Optional[str] = None) -> Dict[str, Any]:
    paths = _ensure_dirs(base_dir)
    config_path = paths["safety_config_path"]
    merged = _deep_merge(DEFAULT_CONFIG, config if isinstance(config, dict) else {})
    validated, _warnings = _validate_config(merged)
    with open(config_path, "w", encoding="utf-8") as f:
        json.dump(validated, f, indent=2)
        f.write("\n")
    return validated


def _looks_like_velocity(msg_data: Dict[str, Any]) -> bool:
    """Untyped payload shaped like Twist (linear/angular) or TwistStamped (twist.linear/angular)."""
    body = msg_data.get("twist") if isinstance(msg_data.get("twist"), dict) else msg_data
    return isinstance(body, dict) and ("linear" in body or "angular" in body)


def is_velocity_msg_type(msg_type: str) -> bool:
    return _normalize_msg_type(msg_type) in {t.replace("/msg/", "/") for t in _VELOCITY_TYPES}


def extract_velocity_fields(msg_data: Dict[str, Any]) -> Optional[Dict[str, Any]]:
    if not isinstance(msg_data, dict):
        return None

    payload = msg_data
    stamped = False
    if isinstance(msg_data.get("twist"), dict):
        payload = msg_data["twist"]
        stamped = True

    linear = payload.get("linear", {}) if isinstance(payload, dict) else {}
    angular = payload.get("angular", {}) if isinstance(payload, dict) else {}
    if not isinstance(linear, dict) or not isinstance(angular, dict):
        return None

    def _num(d: Dict[str, Any], k: str) -> float:
        v = d.get(k, 0.0)
        try:
            return float(v)
        except Exception:
            return 0.0

    return {
        "stamped": stamped,
        "linear": {"x": _num(linear, "x"), "y": _num(linear, "y"), "z": _num(linear, "z")},
        "angular": {"x": _num(angular, "x"), "y": _num(angular, "y"), "z": _num(angular, "z")},
    }


def check_velocity(
    msg_data: Dict[str, Any],
    msg_type: str,
    config: Optional[Dict[str, Any]] = None,
    topic: Optional[str] = None,
) -> Tuple[bool, Optional[str], Dict[str, Any]]:
    cfg = config or load_config()
    if not cfg.get("enabled", True):
        return True, None, {}
    vlim = cfg.get("velocity_limits", {})
    if not vlim.get("enabled", True):
        return True, None, {}
    if not is_velocity_msg_type(msg_type) and not (not msg_type and _looks_like_velocity(msg_data)):
        return True, None, {}

    scoped_topics = [str(t) for t in vlim.get("topics", []) if isinstance(t, str)]
    if scoped_topics and topic and topic not in scoped_topics:
        return True, None, {"skipped": "topic_not_in_scope", "topic": topic, "scoped_topics": scoped_topics}

    fields = extract_velocity_fields(msg_data)
    if not fields:
        return False, "velocity fields missing", {"violations": ["velocity fields missing"]}

    lin = fields["linear"]
    ang = fields["angular"]
    lin_norm = math.sqrt(lin["x"] ** 2 + lin["y"] ** 2 + lin["z"] ** 2)
    ang_norm = math.sqrt(ang["x"] ** 2 + ang["y"] ** 2 + ang["z"] ** 2)

    violations: List[str] = []
    if lin_norm > float(vlim.get("linear_max_norm", 0.5)):
        violations.append(
            f"linear velocity norm {lin_norm:.3f} m/s exceeds limit {float(vlim.get('linear_max_norm', 0.5)):.3f} m/s"
        )
    if ang_norm > float(vlim.get("angular_max_norm", 1.0)):
        violations.append(
            f"angular velocity norm {ang_norm:.3f} rad/s exceeds limit {float(vlim.get('angular_max_norm', 1.0)):.3f} rad/s"
        )

    axis_limits = vlim.get("per_axis", {}) if isinstance(vlim.get("per_axis", {}), dict) else {}
    axis_map = {
        "linear_x_max": ("linear", "x", "m/s"),
        "linear_y_max": ("linear", "y", "m/s"),
        "linear_z_max": ("linear", "z", "m/s"),
        "angular_x_max": ("angular", "x", "rad/s"),
        "angular_y_max": ("angular", "y", "rad/s"),
        "angular_z_max": ("angular", "z", "rad/s"),
    }
    for key, (vec, axis, unit) in axis_map.items():
        lim = axis_limits.get(key)
        if lim is None:
            continue
        val = abs(fields[vec][axis])
        if val > float(lim):
            violations.append(
                f"{vec}.{axis} = {val:.3f} {unit} exceeds per-axis limit {float(lim):.3f} {unit}"
            )

    detail = {
        "limits": {
            "linear_max_norm": float(vlim.get("linear_max_norm", 0.5)),
            "angular_max_norm": float(vlim.get("angular_max_norm", 1.0)),
            "per_axis": {k: v for k, v in axis_limits.items() if v is not None},
        },
        "actual": {
            "linear_norm": round(lin_norm, 4),
            "angular_norm": round(ang_norm, 4),
            "linear": lin,
            "angular": ang,
        },
    }
    if violations:
        detail["violations"] = violations
        return False, "; ".join(violations), detail
    return True, None, detail


def clamp_velocity(msg_data: Dict[str, Any], config: Dict[str, Any]) -> Dict[str, Any]:
    d = copy.deepcopy(msg_data)
    fields = extract_velocity_fields(d)
    if not fields:
        return d

    vlim = (config or {}).get("velocity_limits", {})
    per_axis = vlim.get("per_axis", {}) if isinstance(vlim.get("per_axis", {}), dict) else {}

    target = d.get("twist") if isinstance(d.get("twist"), dict) else d
    if not isinstance(target, dict):
        return d

    for vec in ("linear", "angular"):
        if not isinstance(target.get(vec), dict):
            target[vec] = {"x": 0.0, "y": 0.0, "z": 0.0}

    def _apply_axis(vec: str, axis: str, key: str):
        lim = per_axis.get(key)
        if lim is None:
            return
        try:
            v = float(target[vec].get(axis, 0.0))
        except Exception:
            v = 0.0
        limf = float(lim)
        target[vec][axis] = max(-limf, min(limf, v))

    _apply_axis("linear", "x", "linear_x_max")
    _apply_axis("linear", "y", "linear_y_max")
    _apply_axis("linear", "z", "linear_z_max")
    _apply_axis("angular", "x", "angular_x_max")
    _apply_axis("angular", "y", "angular_y_max")
    _apply_axis("angular", "z", "angular_z_max")

    lin_x = float(target.get("linear", {}).get("x", 0.0))
    lin_y = float(target.get("linear", {}).get("y", 0.0))
    lin_z = float(target.get("linear", {}).get("z", 0.0))
    ang_x = float(target.get("angular", {}).get("x", 0.0))
    ang_y = float(target.get("angular", {}).get("y", 0.0))
    ang_z = float(target.get("angular", {}).get("z", 0.0))

    lin_norm = math.sqrt(lin_x ** 2 + lin_y ** 2 + lin_z ** 2)
    ang_norm = math.sqrt(ang_x ** 2 + ang_y ** 2 + ang_z ** 2)
    lin_max = float(vlim.get("linear_max_norm", 0.5))
    ang_max = float(vlim.get("angular_max_norm", 1.0))

    if lin_norm > lin_max and lin_norm > 0:
        scale = lin_max / lin_norm
        target["linear"]["x"] = lin_x * scale
        target["linear"]["y"] = lin_y * scale
        target["linear"]["z"] = lin_z * scale
    if ang_norm > ang_max and ang_norm > 0:
        scale = ang_max / ang_norm
        target["angular"]["x"] = ang_x * scale
        target["angular"]["y"] = ang_y * scale
        target["angular"]["z"] = ang_z * scale
    return d


def validate_publish(
    topic: str,
    msg_data: Dict[str, Any],
    msg_type: str,
    config: Optional[Dict[str, Any]] = None,
) -> Tuple[bool, str, Optional[Dict[str, Any]], Dict[str, Any]]:
    cfg = config or load_config()
    if not cfg.get("enabled", True):
        return True, "allow", None, {"safety_disabled": True}

    ok, reason, detail = check_velocity(msg_data, msg_type, config=cfg, topic=topic)
    if ok:
        return True, "allow", None, detail

    mode = cfg.get("velocity_limits", {}).get("on_violation", "reject")
    if mode == "warn":
        return True, "warn", None, detail
    if mode == "cap":
        capped = clamp_velocity(msg_data, cfg)
        return True, "cap", capped, detail
    return False, "block", None, detail


def check_geofence(x: float, y: float, config: Optional[Dict[str, Any]] = None) -> Tuple[bool, Optional[str], Dict[str, Any]]:
    cfg = config or load_config()
    if not cfg.get("enabled", True):
        return True, None, {}
    gf = cfg.get("geofence", {})
    if not gf.get("enabled", False):
        return True, None, {}

    mode = gf.get("mode", "circle")
    if mode == "circle":
        c = gf.get("circle", {})
        cx = float(c.get("center_x", 0.0))
        cy = float(c.get("center_y", 0.0))
        r = float(c.get("radius_m", 10.0))
        dist = math.sqrt((x - cx) ** 2 + (y - cy) ** 2)
        if dist > r:
            return False, f"goal outside geofence circle (distance={dist:.3f} > radius={r:.3f})", {
                "mode": "circle",
                "distance": dist,
                "circle": c,
            }
        return True, None, {"mode": "circle", "distance": dist, "circle": c}

    rect = gf.get("rectangle", {})
    x_min = float(rect.get("x_min", -5.0))
    x_max = float(rect.get("x_max", 5.0))
    y_min = float(rect.get("y_min", -5.0))
    y_max = float(rect.get("y_max", 5.0))
    inside = x_min <= x <= x_max and y_min <= y <= y_max
    if not inside:
        return False, "goal outside geofence rectangle", {
            "mode": "rectangle",
            "rectangle": rect,
            "goal": {"x": x, "y": y},
        }
    return True, None, {"mode": "rectangle", "rectangle": rect, "goal": {"x": x, "y": y}}


def check_command(
    command: Optional[str],
    subcommand: Optional[str],
    config: Optional[Dict[str, Any]] = None,
) -> Tuple[bool, Optional[str], Dict[str, Any]]:
    cfg = config or load_config()
    if not cfg.get("enabled", True):
        return True, None, {"safety_disabled": True}

    cf = cfg.get("command_filter", {})
    if not cf.get("enabled", False):
        return True, None, {}

    cmd = _normalize_command(command, subcommand)
    cmd_group = (command or "").strip()
    always_allowed = {_normalize_cmd_rule(x) for x in cf.get("always_allowed", []) if isinstance(x, str)}

    # Never allow self-lockout of safety controls and emergency halt.
    if cmd_group in {"safety", "estop"} or cmd == "version":
        return True, None, {"always_allowed": True}

    def _matches(rule: str) -> bool:
        if not rule:
            return False
        if " " in rule:
            return cmd == rule
        return cmd_group == rule

    for rule in always_allowed:
        if _matches(rule):
            return True, None, {"mode": cf.get("mode", "blocklist"), "rule": rule, "always_allowed": True}

    mode = cf.get("mode", "blocklist")
    blocklist = {_normalize_cmd_rule(x) for x in cf.get("blocklist", []) if isinstance(x, str)}
    allowlist = {_normalize_cmd_rule(x) for x in cf.get("allowlist", []) if isinstance(x, str)}

    if mode == "blocklist":
        for rule in blocklist:
            if _matches(rule):
                return False, f"command blocked by rule '{rule}'", {"rule": rule, "mode": mode}
        return True, None, {"mode": mode}

    allowed = False
    matched = None
    for rule in allowlist:
        if _matches(rule):
            allowed = True
            matched = rule
            break
    if allowed:
        return True, None, {"mode": mode, "rule": matched}
    return False, "command not in allowlist", {"mode": mode, "command": cmd}


def get_heartbeat_sentinel_path(base_dir: Optional[str] = None) -> str:
    return _resolve_runtime_paths(base_dir)["heartbeat_sentinel"]


def heartbeat_touch(base_dir: Optional[str] = None) -> Dict[str, Any]:
    sentinel = get_heartbeat_sentinel_path(base_dir)
    now = time.time()
    os.makedirs(os.path.dirname(sentinel), exist_ok=True)
    if not os.path.exists(sentinel):
        with open(sentinel, "w", encoding="utf-8") as f:
            f.write(json.dumps({"created_at": now}) + "\n")
    os.utime(sentinel, (now, now))
    return {"path": sentinel, "timestamp": now}


def get_heartbeat_status(base_dir: Optional[str] = None, timeout_s: Optional[int] = None) -> Dict[str, Any]:
    """Return the current heartbeat health and the fail-safe reason when stale."""
    paths = _resolve_runtime_paths(base_dir)
    sentinel = paths["heartbeat_sentinel"]
    exists = os.path.exists(sentinel)
    timeout = int(timeout_s if timeout_s is not None else load_config(base_dir=base_dir).get("heartbeat", {}).get("timeout_s", 5))
    last_mtime = os.path.getmtime(sentinel) if exists else None
    stale = (not exists) or (last_mtime is not None and (time.time() - float(last_mtime)) > timeout)
    if not exists:
        reason = "missing sentinel"
    elif stale:
        reason = "heartbeat stale"
    else:
        reason = "ok"
    return {
        "running": session_exists(_WATCHDOG_SESSION),
        "session": _WATCHDOG_SESSION,
        "heartbeat_enabled": bool(load_config(base_dir=base_dir).get("heartbeat", {}).get("enabled", False)),
        "sentinel": sentinel,
        "sentinel_exists": exists,
        "sentinel_mtime": last_mtime,
        "timeout_s": timeout,
        "stale": stale,
        "reason": reason,
    }


def trigger_fail_safe(
    reason: str,
    base_dir: Optional[str] = None,
    on_timeout: str = "estop",
    timeout_s: Optional[int] = None,
) -> Dict[str, Any]:
    """Fail closed when the watchdog or session loses contact.

    This writes the standardized fail-safe event to the heartbeat log and, when
    configured, invokes the CLI emergency stop command.
    """
    paths = _resolve_runtime_paths(base_dir)
    os.makedirs(paths["artifacts_dir"], exist_ok=True)
    sentinel = paths["heartbeat_sentinel"]
    log_path = paths["heartbeat_log"]
    heartbeat_cfg = load_config(base_dir=base_dir)
    hb = heartbeat_cfg.get("heartbeat", {})
    timeout = int(timeout_s if timeout_s is not None else hb.get("timeout_s", 5))
    if os.path.exists(sentinel):
        age = time.time() - float(os.path.getmtime(sentinel))
    else:
        age = None

    event = {
        "watchdog_fired": True,
        "timestamp": time.time(),
        "reason": str(reason),
        "fired_at": time.time(),
        "sentinel": sentinel,
        "sentinel_exists": os.path.exists(sentinel),
        "sentinel_age_s": round(age, 3) if age is not None else None,
        "timeout_s": timeout,
        "action": on_timeout,
        "estop": None,
    }

    if on_timeout == "estop":
        try:
            proc = subprocess.run(["python3", os.path.join(_DEFAULT_SKILL_ROOT, "scripts", "ros2_cli.py"), "estop"], capture_output=True, text=True)
            event["estop"] = {
                "returncode": proc.returncode,
                "stdout": proc.stdout.strip(),
                "stderr": proc.stderr.strip(),
            }
        except Exception as exc:  # pragma: no cover - environment-specific failure path
            event["estop"] = {"error": str(exc)}

    with open(log_path, "a", encoding="utf-8") as f:
        f.write(json.dumps(event, sort_keys=True) + "\n")

    event["action"] = on_timeout
    event["watchdog_fired"] = True
    return event


def _parse_validate_payload(payload: str) -> Dict[str, Any]:
    try:
        return json.loads(payload)
    except Exception:
        return {}


def cmd_safety_show(args):
    cfg = load_config(create_if_missing=True)
    output({"config_path": _SAFETY_CONFIG_PATH, "config": cfg})


def cmd_safety_enable(args):
    cfg = load_config()
    cfg["enabled"] = True
    cfg = save_config(cfg)
    output({"success": True, "enabled": True, "config": cfg})


def cmd_safety_disable(args):
    cfg = load_config()
    cfg["enabled"] = False
    cfg = save_config(cfg)
    output({"success": True, "enabled": False, "warning": "All safety checks are disabled", "config": cfg})


def cmd_safety_set_velocity(args):
    cfg = load_config()
    vlim = cfg.setdefault("velocity_limits", {})
    vlim["enabled"] = True
    if getattr(args, "linear", None) is not None:
        vlim["linear_max_norm"] = float(args.linear)
    if getattr(args, "angular", None) is not None:
        vlim["angular_max_norm"] = float(args.angular)
    axis = getattr(args, "axis", None)
    value = getattr(args, "value", None)
    if axis:
        per_axis = vlim.setdefault("per_axis", {})
        per_axis[axis] = float(value)
    if getattr(args, "topic", None):
        topics = vlim.setdefault("topics", [])
        t = args.topic
        if t not in topics:
            topics.append(t)
    cfg = save_config(cfg)
    output({"success": True, "velocity_limits": cfg["velocity_limits"]})


def cmd_safety_set_geofence(args):
    cfg = load_config()
    gf = cfg.setdefault("geofence", {})
    mode = args.mode
    gf["mode"] = mode
    gf["enabled"] = True
    gf["frame_id"] = getattr(args, "frame", None) or gf.get("frame_id", "map")

    if mode == "circle":
        gf["circle"] = {
            "center_x": float(args.cx),
            "center_y": float(args.cy),
            "radius_m": float(args.radius),
        }
    else:
        gf["rectangle"] = {
            "x_min": float(args.xmin),
            "x_max": float(args.xmax),
            "y_min": float(args.ymin),
            "y_max": float(args.ymax),
        }
    cfg = save_config(cfg)
    output({"success": True, "geofence": cfg["geofence"]})


def cmd_safety_geofence_enable(args):
    cfg = load_config()
    cfg.setdefault("geofence", {})["enabled"] = True
    cfg = save_config(cfg)
    output({"success": True, "geofence_enabled": True, "geofence": cfg["geofence"]})


def cmd_safety_geofence_disable(args):
    cfg = load_config()
    cfg.setdefault("geofence", {})["enabled"] = False
    cfg = save_config(cfg)
    output({"success": True, "geofence_enabled": False, "geofence": cfg["geofence"]})


def cmd_safety_block(args):
    cfg = load_config()
    cf = cfg.setdefault("command_filter", {})
    cf["enabled"] = True
    cf["mode"] = "blocklist"
    blocklist = cf.setdefault("blocklist", [])
    rule = _normalize_command(args.command_name, getattr(args, "command_subname", None))
    if rule and rule not in blocklist:
        blocklist.append(rule)
    cfg = save_config(cfg)
    output({"success": True, "rule": rule, "command_filter": cfg["command_filter"]})


def cmd_safety_unblock(args):
    cfg = load_config()
    cf = cfg.setdefault("command_filter", {})
    blocklist = cf.setdefault("blocklist", [])
    rule = _normalize_command(args.command_name, getattr(args, "command_subname", None))
    if rule in blocklist:
        blocklist.remove(rule)
    cfg = save_config(cfg)
    output({"success": True, "rule": rule, "command_filter": cfg["command_filter"]})


def cmd_safety_validate(args):
    topic = args.topic
    msg_data = _parse_validate_payload(args.msg)
    msg_type = getattr(args, "msg_type", None) or "geometry_msgs/msg/Twist"
    cfg = load_config()
    ok, action, modified_msg, detail = validate_publish(topic, msg_data, msg_type, config=cfg)
    result = {
        "topic": topic,
        "msg_type": msg_type,
        "action": "allow" if ok and action == "allow" else action,
        "checks": {
            "velocity": {
                "result": action,
                "detail": detail,
            },
        },
    }
    if modified_msg is not None:
        result["modified_payload"] = modified_msg
    output(result)


def cmd_safety_reset(args):
    if not getattr(args, "yes", False):
        return output({"error": "safety reset requires --yes"})
    cfg = save_config(DEFAULT_CONFIG)
    output({"success": True, "config": cfg})


def _watchdog_start(timeout_s: int, poll_s: int, on_timeout: str) -> Tuple[bool, str]:
    if not check_tmux():
        return False, "tmux is not installed"
    if session_exists(_WATCHDOG_SESSION):
        return True, "already_running"

    heartbeat_touch()
    cmd = (
        f"tmux new-session -d -s {_WATCHDOG_SESSION} "
        f"python3 {_WATCHDOG_SCRIPT} --timeout {int(timeout_s)} --poll {int(poll_s)} --on-timeout {on_timeout}"
    )
    _out, err, rc = run_cmd(cmd)
    if rc != 0:
        return False, err or "failed to start watchdog"
    return True, "started"


def cmd_safety_heartbeat_start(args):
    cfg = load_config()
    hb = cfg.setdefault("heartbeat", {})
    if getattr(args, "timeout", None) is not None:
        hb["timeout_s"] = int(args.timeout)
    if getattr(args, "poll", None) is not None:
        hb["poll_interval_s"] = int(args.poll)
    hb["enabled"] = True
    cfg = save_config(cfg)

    ok, detail = _watchdog_start(
        timeout_s=int(hb.get("timeout_s", 5)),
        poll_s=int(hb.get("poll_interval_s", 1)),
        on_timeout=str(hb.get("on_timeout", "estop")),
    )
    if not ok:
        return output({"error": "failed to start heartbeat watchdog", "detail": detail})
    output({
        "success": True,
        "status": detail,
        "session": _WATCHDOG_SESSION,
        "heartbeat": cfg["heartbeat"],
        "sentinel": _HEARTBEAT_SENTINEL,
    })


def cmd_safety_heartbeat_stop(args):
    running = session_exists(_WATCHDOG_SESSION)
    if running:
        kill_session(_WATCHDOG_SESSION)
    output({"success": True, "was_running": running, "session": _WATCHDOG_SESSION})


def cmd_safety_heartbeat_ping(args):
    touched = heartbeat_touch()
    output({"success": True, "sentinel": touched["path"], "timestamp": touched["timestamp"]})


def cmd_safety_heartbeat_status(args):
    sentinel = get_heartbeat_sentinel_path()
    exists = os.path.exists(sentinel)
    last_mtime = os.path.getmtime(sentinel) if exists else None
    cfg = load_config()
    hb = cfg.get("heartbeat", {})
    running = session_exists(_WATCHDOG_SESSION)
    stale = None
    if exists:
        stale = (time.time() - float(last_mtime)) > float(hb.get("timeout_s", 5))
    output({
        "running": running,
        "session": _WATCHDOG_SESSION,
        "heartbeat_enabled": bool(hb.get("enabled", False)),
        "sentinel": sentinel,
        "sentinel_exists": exists,
        "sentinel_mtime": last_mtime,
        "stale": stale,
        "timeout_s": hb.get("timeout_s", 5),
    })
