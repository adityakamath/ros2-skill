#!/usr/bin/env python3
"""``plan``: compile a natural-language goal into concrete CLI commands (never executes anything).

Units, signs (REP-103: left/strafe-left = +), timeouts and velocity ceilings are computed here in
code; named places come from ``nav2 place``. Unknown places or unavailable capabilities produce a
warning and no step, never a guessed pose.
"""

import json
import math
import os
import re
import shlex
import sys
import time

_NUM = r"(-?\d+(?:\.\d+)?)"
_MAX_LINEAR_M = 50.0     # RULES-MOTION Rule 22 ceilings
_MAX_ROTATION_DEG = 3600.0
_DEFAULT_SPEED = 0.2     # m/s, capped by the profile ceiling
_DEFAULT_OMEGA = 0.5     # rad/s, capped by the profile ceiling

_SPEED = re.compile(rf"\bat\s+(?:its\s+)?(?:(top|max(?:imum)?|full)\s+speed|{_NUM}\s*m/s)", re.I)
_MOVE = re.compile(rf"^(?:move|go|drive)\s+(forward|ahead|backward|back)\s+{_NUM}\s*(m|metres?|meters?|cm)?\b", re.I)
_DIR = r"(left|right|clockwise|counter-?clockwise|ccw|cw)"
_ROTATE = re.compile(rf"^(?:turn|rotate|spin)\s+(?:{_DIR}\s+)?{_NUM}\s*(deg|degrees?|rad|radians?)?\b(?:\s+(?:to\s+the\s+)?{_DIR}\b)?", re.I)
_TURN_AROUND = re.compile(r"^(?:turn\s+around|u-?turn|about\s+face)$", re.I)
_UNDO = re.compile(r"^(?:undo(?:\s+that)?|(?:go|come|move|turn|rotate|strafe)\s+back|return\s+to\s+(?:the\s+)?(?:start|where\s+you\s+started))$", re.I)
_STRAFE = re.compile(rf"^strafe\s+(left|right)\s+{_NUM}\s*(m|metres?|meters?|cm)?\b", re.I)
_COORDS = re.compile(rf"^(?:go|navigate|drive|move)\s+to\s+{_NUM}\s*,\s*{_NUM}(?:\s+yaw\s+{_NUM})?\s*$", re.I)
_PATROL = re.compile(r"^(?:patrol|visit)\s+(.+)$", re.I)
_PLACE = re.compile(r"^(?:go|navigate|drive|move)\s+to\s+(?:the\s+)?([\w-]+)\s*$", re.I)


# steps are separated by "then", ";" or "and" followed by another verb ("save this spot as a and list places")
_VERBS = "what|remember|how|list|save|reset|clear|record|delete|forget|take|move|go|drive|turn|rotate|spin|strafe|back|reverse|look|report|check|snapshot|status|undo|return|come|patrol|visit|navigate"
_SPLIT = rf"\bthen\b|;|,\s*(?=(?:and\s+)?(?:{_VERBS})\b)|\s+and\s+(?=(?:{_VERBS.replace('go|', '')})\b)"


def _fmt(v):
    return f"{v:g}"


def _twist(vtype, lx=0.0, ly=0.0, az=0.0):
    twist = {"linear": {"x": lx, "y": ly, "z": 0.0}, "angular": {"x": 0.0, "y": 0.0, "z": az}}
    if vtype.endswith("TwistStamped"):
        twist = {"header": {"stamp": {"sec": 0}, "frame_id": ""}, "twist": twist}
    return json.dumps(twist, separators=(",", ":"))


class _Ctx:
    """Profile-derived facts, with placeholders when the profile lacks them."""

    def __init__(self, summary, places):
        s = summary or {}
        self.has_profile = bool(s)
        self.caps = set((s.get("capabilities") or {}).keys())
        self.vel = s.get("cmd_vel_topic") or "<VEL_TOPIC>"
        types = {t.get("topic"): t.get("type") for t in s.get("velocity_topics", []) if isinstance(t, dict)}
        self.vtype = types.get(s.get("cmd_vel_topic")) or "geometry_msgs/msg/Twist"
        self._typed = bool(types.get(s.get("cmd_vel_topic")))
        fused = (s.get("localization_config") or {}).get("fused_sources") or {}
        self.odom = next(iter(fused.values()), None) or "<ODOM_TOPIC>"
        binding = (s.get("safety_limits") or {}).get("binding") or {}
        self.max_lin = float(binding.get("linear_x", 0.5))
        self.max_lin_y = float(binding.get("linear_y", self.max_lin))
        self.max_ang = float(binding.get("angular_z", 1.0))
        self.places = {p["name"].lower(): p for p in places}
        self.robot_services = s.get("robot_services") or {}
        self.toggles = s.get("toggles") or {}
        self.mode_service = (s.get("velocity_chain") or {}).get("mode_service")
        fitted = s.get("hardware_fitted") or {}
        # Gemini 2 publishes under /gemini2/ (camera_name in gemini2.launch.py); other robots need discovery.
        self.camera = "/gemini2/color/image_raw" if fitted.get("gemini2") else "<CAMERA_TOPIC>"
        frames = s.get("tf_frames") or {}
        self.odom_frame, self.base_frame = frames.get("odom_frame"), frames.get("base_frame")
        self.artifacts = ".artifacts"

    def has(self, cap):
        return not self.has_profile or cap in self.caps

    def types(self):
        """Message types for publish-until, which then skips its discovery node (fused odometry is always Odometry)."""
        return f" --msg-type {self.vtype} --monitor-msg-type nav_msgs/msg/Odometry" if self._typed and self.odom[0] == "/" else ""

    def limits(self):
        return f"--max-vel {_fmt(self.max_lin)} --max-ang {_fmt(self.max_ang)}"


def _speed(text, cap):
    """Requested speed from 'at top speed' / 'at 0.1 m/s', else the default; never above *cap*."""
    m = _SPEED.search(text)
    if m and m.group(1):
        return cap
    if m and m.group(2):
        return min(abs(float(m.group(2))), cap) or min(_DEFAULT_SPEED, cap)
    return min(_DEFAULT_SPEED, cap)


def _timeout(seconds):
    return max(15, math.ceil(2.0 * seconds))


def _step_move(m, ctx, warn):
    direction, dist, unit = m.group(1).lower(), float(m.group(2)), (m.group(3) or "m").lower()
    dist = dist / 100.0 if unit == "cm" else dist
    if dist <= 0 or dist > _MAX_LINEAR_M:
        return warn(f"distance {dist:g} m is outside (0, {_MAX_LINEAR_M:g}] m; confirm with the user and split it")
    if not ctx.has("move_distance"):
        return warn("this robot has no move_distance capability (needs cmd_vel and odometry)")
    v = _speed(m.string, ctx.max_lin)
    sign = -1.0 if direction.startswith("back") else 1.0
    cmd = (f"topics publish-until {ctx.vel} '{_twist(ctx.vtype, lx=sign * v)}' --monitor {ctx.odom} "
           f"--field pose.pose.position --euclidean --delta {_fmt(dist)} --timeout {_timeout(dist / v)} {ctx.limits()}{ctx.types()}")
    return {"capability": "move_distance", "inputs": {"distance_m": sign * dist, "speed_mps": v}, "commands": [cmd], "motion": True, "source": "teleop"}


def _step_strafe(m, ctx, warn):
    direction, dist, unit = m.group(1).lower(), float(m.group(2)), (m.group(3) or "m").lower()
    dist = dist / 100.0 if unit == "cm" else dist
    if dist <= 0 or dist > _MAX_LINEAR_M:
        return warn(f"distance {dist:g} m is outside (0, {_MAX_LINEAR_M:g}] m; confirm with the user and split it")
    if ctx.has_profile and "strafe" not in ctx.caps:
        return warn("strafe needs a holonomic drive; this robot has no strafe capability")
    v = _speed(m.string, ctx.max_lin_y)
    sign = 1.0 if direction == "left" else -1.0  # REP-103: +y is left
    cmd = (f"topics publish-until {ctx.vel} '{_twist(ctx.vtype, ly=sign * v)}' --monitor {ctx.odom} "
           f"--field pose.pose.position --euclidean --delta {_fmt(dist)} --timeout {_timeout(dist / v)} {ctx.limits()}{ctx.types()}")
    return {"capability": "strafe", "inputs": {"distance_m": dist, "direction": direction, "speed_mps": v}, "commands": [cmd], "motion": True, "source": "teleop"}


def _rotation_sign(word):
    """+1 = left (counter-clockwise), -1 = right; None if the word is absent."""
    if not word:
        return None
    return -1.0 if word.lower() in ("right", "clockwise", "cw") else 1.0


def _step_rotate(m, ctx, warn):
    angle, unit = float(m.group(2)), (m.group(3) or "deg").lower()
    sign = _rotation_sign(m.group(1)) or _rotation_sign(m.group(4)) or (-1.0 if angle < 0 else 1.0)
    deg = abs(math.degrees(angle) if unit.startswith("rad") else angle)
    if deg <= 0 or deg > _MAX_ROTATION_DEG:
        return warn(f"rotation {deg:g}° is outside (0, {_MAX_ROTATION_DEG:g}]°; confirm with the user")
    if not ctx.has("rotate_to_heading"):
        return warn("this robot has no rotate_to_heading capability")
    signed = sign * deg  # positive = CCW = left
    if "navigate_to_pose" in ctx.caps:  # Nav2 present: obstacle-aware spin
        cmd = f"nav2 rotate -- {_fmt(signed)}" if signed < 0 else f"nav2 rotate {_fmt(signed)}"
    else:
        w = min(_DEFAULT_OMEGA, ctx.max_ang)
        cmd = (f"topics publish-until {ctx.vel} '{_twist(ctx.vtype, az=math.copysign(w, signed))}' --monitor {ctx.odom} "
               f"--rotate {_fmt(signed)} --degrees --timeout {_timeout(math.radians(deg) / w)} {ctx.limits()}{ctx.types()}")
    return {"capability": "rotate_to_heading", "inputs": {"degrees": signed}, "commands": [cmd], "motion": True,
            "source": "nav2" if "navigate_to_pose" in ctx.caps else "teleop", **({"nav2": True} if "navigate_to_pose" in ctx.caps else {})}


def _step_coords(m, ctx, warn):
    if not ctx.has("navigate_to_pose"):
        return warn("this robot has no Nav2 navigate_to_pose capability")
    x, y, yaw = float(m.group(1)), float(m.group(2)), m.group(3)
    cmd = f"nav2 go {_fmt(x)} {_fmt(y)}" + (f" --yaw {_fmt(float(yaw))}" if yaw is not None else "")
    return {"capability": "navigate_to_pose", "inputs": {"x": x, "y": y}, "commands": [cmd], "motion": True, "nav2": True, "source": "nav2"}


def _step_place(m, ctx, warn):
    name = m.group(1)
    if name.lower() not in ctx.places:
        known = ", ".join(sorted(ctx.places)) or "none saved"
        return warn(f"unknown place '{name}' (known: {known}); save it with `nav2 place save {name}`")
    if not ctx.has("navigate_to_pose"):
        return warn("this robot has no Nav2 navigate_to_pose capability")
    return {"capability": "navigate_to_pose", "inputs": {"place": ctx.places[name.lower()]["name"]},
            "commands": [f"nav2 place go {ctx.places[name.lower()]['name']}"], "motion": True, "nav2": True, "source": "nav2"}


def _step_patrol(m, ctx, warn):
    names = [n.strip() for n in re.split(r"\s*(?:,|\band\b)\s*", m.group(1)) if n.strip()]
    if len(names) < 2:
        return warn("patrol/visit needs at least two places")
    missing = [n for n in names if n.lower() not in ctx.places]
    if missing:
        return warn(f"unknown place(s) {missing} (known: {', '.join(sorted(ctx.places)) or 'none saved'})")
    if not ctx.has("follow_waypoints"):
        return warn("this robot has no Nav2 follow_waypoints capability")
    pts = " ".join(f"{_fmt(ctx.places[n.lower()]['x'])},{_fmt(ctx.places[n.lower()]['y'])}" for n in names)
    return {"capability": "follow_waypoints", "inputs": {"places": names}, "commands": [f"nav2 go-waypoints {pts}"],
            "motion": True, "nav2": True, "source": "nav2"}


def _robot_call(ctx, key, toggle=None):
    """(service, request) for a robot service bound under *key* (joystick services) or *toggle* (SetBool toggles)."""
    if key in ctx.robot_services and ctx.robot_services[key].get("type") == "std_srvs/srv/SetBool":
        svc = ctx.robot_services[key]
        return svc["service"], svc.get("request", {"data": True})
    if toggle and toggle in ctx.toggles:
        return ctx.toggles[toggle], None
    return None, None


def _service_step(capability, text_key):
    def build(m, ctx, warn):
        key, toggle = text_key
        svc, req = _robot_call(ctx, key, toggle)
        if not svc and capability == "save_map":  # robots without a save_map service: use slam_toolbox / map_saver
            return {"capability": capability, "inputs": {}, "commands": ["nav2 map save"], "motion": False, "source": None}
        if not svc:
            return warn(f"this robot exposes no `{key or toggle}` service")
        if req is None:  # toggle: direction comes from the matched words
            req = {"data": not re.match(r"^(stop|pause|halt)", m.group(0), re.I)}
        return {"capability": capability, "inputs": {"service": svc, **req},
                "commands": [f"services call {svc} '{json.dumps(req, separators=(',', ': '))}'"],
                "motion": capability == "patrol", "nav2": capability == "patrol",
                "source": "nav2" if capability == "patrol" else None}
    return build


def _step_look_around(m, ctx, warn):
    views = int(m.group(1) or 4)
    if not 2 <= views <= 12:
        return warn("look around supports 2-12 views")
    if not ctx.has("rotate_to_heading"):
        return warn("this robot has no rotate_to_heading capability")
    if ctx.has_profile and not ctx.caps & {"navigate_to_pose"}:
        return warn("look around rotates with Nav2 (nav2 rotate), which this robot does not have")
    turn = 360.0 / views
    cmds = []
    for i in range(views):
        cmds.append(f"topics capture-image --topic {ctx.camera} --output {ctx.artifacts}/look_{i + 1}of{views}.jpg")
        if ctx.odom_frame and ctx.base_frame:
            cmds.append(f"tf lookup {ctx.odom_frame} {ctx.base_frame}")
        if i < views - 1:
            cmds.append(f"nav2 rotate {_fmt(turn)}")
    return {"capability": "look_around", "inputs": {"views": views, "turn_deg": turn}, "commands": cmds,
            "motion": True, "nav2": True, "source": "nav2"}


_NAME = r"['\"]?([\w-]+)['\"]?"


def _static_step(capability, commands, **inputs):
    return {"capability": capability, "inputs": inputs, "commands": commands, "motion": False, "source": None}


def _step_place_save(m, ctx, warn):
    return _static_step("save_place", [f"nav2 place save {m.group(1)}"], name=m.group(1))


def _step_place_list(m, ctx, warn):
    return _static_step("list_places", ["nav2 place list"])


def _step_place_delete(m, ctx, warn):
    return _static_step("delete_place", [f"nav2 place delete {m.group(1)}"], name=m.group(1))


def _step_save_map_as(m, ctx, warn):
    return _static_step("save_map", [f"nav2 map save --name {m.group(1)}"], name=m.group(1))


def _step_picture(m, ctx, warn):
    return _static_step("capture_image", ["look"])


def _step_status(m, ctx, warn):
    return _static_step("snapshot", ["snapshot"])


def _step_remember(m, ctx, warn):
    return _static_step("remember_note", [f"notes remember {shlex.quote(m.group(1).strip())}"], text=m.group(1).strip())


def _step_recall(m, ctx, warn):
    query = " ".join(w for w in (m.group(1) or m.group(2) or "").split() if w.lower() not in ("the", "a", "an", "my", "our"))
    return _static_step("recall_notes", [f"notes recall {shlex.quote(query)}" if query else "notes recall"], query=query)


def _step_battery(m, ctx, warn):
    return _static_step("snapshot", ["snapshot --only battery,state"])


def _step_surroundings(m, ctx, warn):
    return _static_step("snapshot", ["snapshot --only scan,tof,depth"])


def _step_turn_around(m, ctx, warn):
    return _step_rotate(_ROTATE.match("turn left 180"), ctx, warn)


_PATTERNS = [
    (re.compile(r"^look\s+around(?:\s+(\d+)\s+(?:times|views|directions))?$", re.I), _step_look_around),
    (re.compile(r"^(?:start|resume|begin|stop|pause|halt)\s+(?:the\s+)?(?:patrol|waypoint\s+follow(?:ing)?)$|^patrol$", re.I),
     _service_step("patrol", (None, "waypoint_follow"))),
    (re.compile(r"^(?:record|save|add)\s+(?:a\s+|this\s+|the\s+)?(?:current\s+)?waypoint(?:\s+here)?$", re.I),
     _service_step("record_waypoint", ("record_waypoint", None))),
    (re.compile(r"^(?:reset|clear)\s+(?:all\s+|the\s+)?waypoints$", re.I),
     _service_step("reset_waypoints", ("reset_waypoints", None))),
    (re.compile(r"^save\s+(?:the\s+)?map$", re.I), _service_step("save_map", ("save_map", None))),
    (re.compile(rf"^save\s+(?:the\s+)?map\s+as\s+{_NAME}$", re.I), _step_save_map_as),
    (re.compile(rf"^(?:save|remember|mark|store)\s+(?:this|the\s+current|my\s+current)?\s*(?:spot|place|location|position|pose|here)?\s*(?:here\s+)?as\s+{_NAME}$", re.I),
     _step_place_save),
    (re.compile(r"^(?:list|show|what(?:'s| are| is))\s+(?:my\s+|the\s+|all\s+)?(?:saved\s+)?places(?:\s+(?:are\s+)?saved)?$|^what\s+places\s+(?:are\s+|do\s+you\s+have\s+)?saved$", re.I),
     _step_place_list),
    (re.compile(r"^(?:remember|note|make\s+a\s+note)\s+(?:that\s+|down\s+)?(.{3,})$", re.I), _step_remember),
    (re.compile(r"^what\s+(?:do\s+you\s+|did\s+i\s+)?(?:remember|know|note)(?:d)?\s+about\s+(.+)$|^recall\s+(.+)$|^what\s+(?:do\s+you\s+)?remember$|^list\s+(?:my\s+)?notes$", re.I), _step_recall),
    (re.compile(rf"^(?:delete|forget|remove)\s+(?:the\s+)?place\s+{_NAME}$", re.I), _step_place_delete),
    (re.compile(r"^(?:take|capture|snap)\s+(?:a\s+|another\s+)?(?:picture|photo|image|shot)$|^look$", re.I), _step_picture),
    (re.compile(r"^(?:status|snapshot|health\s+check|check\s+(?:the\s+)?(?:robot|sensors))$", re.I), _step_status),
    (re.compile(r"^look\s+(?:at\s+.+|ahead|forward|in\s+front(?:\s+of\s+you)?)$|^what\s+do\s+you\s+see$|^take\s+a\s+look$", re.I), _step_picture),
    (re.compile(r"^(?:report|check|read|show|get|what(?:'s| is))\s+(?:me\s+)?(?:the\s+)?(?:battery|charge)(?:\s+(?:level|status|voltage|percent(?:age)?))?$|^battery(?:\s+(?:level|status))?$", re.I), _step_battery),
    (re.compile(r"^(?:check|is|are)\s+(?:if\s+)?(?:the\s+)?(?:path|way|area|room)(?:\s+(?:ahead|in\s+front))?(?:\s+is)?(?:\s+(?:clear|free|safe|blocked))?$"
                r"|^what(?:'s| is)\s+(?:around|near|nearby|ahead|in\s+front)(?:\s+(?:of\s+)?(?:me|you))?$|^(?:check|scan)\s+(?:the\s+)?(?:surroundings|obstacles|area|path)$", re.I), _step_surroundings),
    (_TURN_AROUND, _step_turn_around),
]
_PATTERNS += [(_MOVE, _step_move), (_ROTATE, _step_rotate), (_STRAFE, _step_strafe),
             (_COORDS, _step_coords), (_PATROL, _step_patrol), (_PLACE, _step_place)]


def _inverse_of(step):
    """Goal text that reverses a compiled move/strafe/rotate step, or None."""
    inputs = (step or {}).get("inputs") or {}
    cap = (step or {}).get("capability")
    if cap == "move_distance":
        return f"move {'back' if inputs['distance_m'] > 0 else 'forward'} {_fmt(abs(inputs['distance_m']))} m"
    if cap == "strafe":
        return f"strafe {'right' if inputs['direction'] == 'left' else 'left'} {_fmt(inputs['distance_m'])} m"
    if cap == "rotate_to_heading":
        return f"turn {'right' if inputs['degrees'] > 0 else 'left'} {_fmt(abs(inputs['degrees']))} deg"
    return None


def compile_plan(goal, summary=None, places=None, artifacts_dir=".artifacts"):
    """Return ``{goal, steps, warnings, preflight}``. Pure function: no I/O, no ROS."""
    ctx = _Ctx(summary, places or [])
    ctx.artifacts = artifacts_dir
    warnings, steps = [], []
    if not ctx.has_profile:
        warnings.append("no profile loaded: topic names are placeholders and capabilities are unchecked")
    for text in [t.strip() for t in re.split(_SPLIT, goal, flags=re.I) if t.strip()]:
        text = re.sub(r"^and\s+", "", text, flags=re.I).rstrip(".")
        text = re.sub(r"^(?:back\s*up|reverse)\b", "move backward", text, flags=re.I)
        text = re.sub(r"^(move|go|drive)\s+backwards\b", r"\1 backward", text, flags=re.I)
        if _UNDO.match(text):  # "rotate back", "go back", "undo that": the opposite of the previous move/strafe/turn
            inverse = _inverse_of(next((st for st in reversed(steps) if st.get("motion")), None))
            if not inverse:
                warnings.append(f"'{text}': nothing to undo (only an earlier move, strafe or turn in the same goal can be reversed)")
                continue
            text = inverse
        for pattern, build in _PATTERNS:
            m = pattern.match(text)
            if m:
                problems = []
                step = build(m, ctx, lambda msg, t=text: problems.append(f"'{t}': {msg}"))
                if step:
                    steps.append(dict(step, text=text))
                else:
                    warnings.extend(problems)
                break
        else:
            warnings.append(f"'{text}': not understood; supported: move/back up/strafe/turn/turn around with a number, undo/rotate back, go to <place>|<x>, <y>, patrol <a>, <b>, save <name>, list places, take a picture, look at <x>, report the battery, check the path ahead, snapshot, record/reset waypoints, save map")
    # A twist switch routes either the teleop or the Nav2 velocity input to the wheels, so a step on the
    # unselected input is silently ignored. Insert a switch whenever the required input changes.
    ordered, current = [], None
    for step in steps:
        src = step.get("source")
        if ctx.mode_service and src and src != current:
            want = "true" if src == "nav2" else "false"
            ordered.append({"capability": "set_drive_mode", "inputs": {"mode": src}, "text": f"switch the twist switch to {src} input",
                            "commands": [f"services call {ctx.mode_service} '{{\"data\": {want}}}'"],
                            "note": f"skip if `preflight motion --source {src}` already reports drive_mode {src}"})
            current = src
        ordered.append(step)
    steps = ordered
    motion = [s for s in steps if s.get("motion")]
    preflight = None
    if any(not s.get("nav2") for s in motion) and ctx.has_profile:
        ctrls = [c for c in (summary.get("active_controllers") or []) if isinstance(c, str)]
        base = next((c for c in ctrls if re.search(r"base|mobile|diff|omni|mecanum", c, re.I)), "<CONTROLLER>")
        first = next((s.get("source") for s in motion if s.get("source")), "teleop")
        preflight = f"preflight motion --controller {base} --odom-topic {ctx.odom} --source {first}"
    for i, s in enumerate(steps, 1):
        s["step"] = i
        for k in ("motion", "nav2", "source"):
            s["_" + k] = s.pop(k, None)
    return {"goal": goal, "steps": steps, "warnings": warnings, "preflight": preflight,
            "preflight_note": "Run any set_drive_mode step first, then the preflight (it checks e-stop, drive mode and battery)."
                              if any(s["capability"] == "set_drive_mode" for s in steps) else None}


_PARSER = None


def run_cli(argv):
    """Run one CLI command in this process and return ``(json_dict, exit_code)``; no new process, no new rclpy init."""
    import contextlib
    import importlib
    import io

    import ros2_cli
    from ros2_utils import trace

    global _PARSER
    _PARSER = _PARSER or getattr(ros2_cli, "_PARSER", None) or ros2_cli.build_parser()  # building it costs ~0.15 s
    try:
        args = _PARSER.parse_args(argv)
    except SystemExit as exc:
        return {"error": f"bad arguments: {' '.join(argv)}"}, exc.code or 2
    ros2_cli._apply_global_overrides(args)  # fills --timeout/--retries defaults, as main() does
    command, sub = args.command, getattr(args, "subcommand", None)
    try:
        from ros2_safety import check_command
        allowed, reason, _detail = check_command(command, sub)
        if not allowed:
            return {"error": "Command blocked by safety configuration", "reason": reason}, 1
    except ImportError:
        pass
    handler = ros2_cli.DISPATCH[(command, sub)]
    trace(f"run {' '.join(argv[:2])}")
    from ros2_fastmotion import try_serve
    served = try_serve(command, sub, args)  # the fast daemon answers before rclpy is even imported
    if served is not None:
        return served, 1 if "error" in served else 0
    if isinstance(handler, tuple):
        handler = getattr(importlib.import_module(handler[0]), handler[1])
    buf, code = io.StringIO(), 0
    with contextlib.redirect_stdout(buf):
        try:
            handler(args)
        except SystemExit as exc:
            code = exc.code or 0
        except Exception as exc:  # report like main() does
            buf.write(json.dumps({"error": str(exc), "type": type(exc).__name__}))
            code = 1
    try:
        return json.loads(buf.getvalue() or "{}"), code
    except ValueError:
        return {"error": "unparseable output", "raw": buf.getvalue()[:200]}, code or 1


def _gist(out):
    """The few fields of a command result worth reporting."""
    keep = ("success", "condition_met", "euclidean_distance", "duration", "velocity_clamped", "status", "message",
            "path", "verified_stationary", "final_position", "linear_speed", "error", "blocked_by", "translation",
            "limited_downstream", "slow", "places", "saved", "deleted", "place")
    gist = {k: out[k] for k in keep if k in out}
    reply = out.get("result")
    if isinstance(reply, dict) and reply.get("message"):  # a service reply's message ("Waypoint 0 recorded at ...")
        gist["message"] = reply["message"]
    return gist


def _position(out):
    pos = ((out.get("odom") or {}).get("pose") or {}).get("pose", {}).get("position")
    return (pos["x"], pos["y"]) if pos else None


def run_plan(plan, run_cli_fn, odom_topic):
    from ros2_utils import trace
    """Execute a compiled plan in order: mode switch, pre-motion check, command, routine stop and verify.

    Same checks as the manual sequence (RULES-MOTION): a blocked pre-motion check means no motion, and any failed
    step latches the e-stop and stops the run.
    """
    report = {"goal": plan["goal"], "executed": True, "ok": True, "steps": []}
    steps = plan["steps"]
    if not steps:
        return dict(report, executed=False, ok=False, warnings=plan["warnings"])
    start = final = pf_cache = None

    def fail(step_report, out):
        report["ok"] = False
        step_report["ok"] = False
        step_report["result"] = _gist(out)
        if out.get("limited_downstream"):  # a limiter slowed the move: the robot is controlled, so no latching e-stop
            run_cli_fn(["stop", "--verify-odom-topic", odom_topic] if odom_topic else ["stop"])
            report["cause"] = out["limited_downstream"]
            report["estop"] = "not latched: stopped normally (the move was limited downstream)"
            return
        run_cli_fn(["estop", "--verify-odom-topic", odom_topic] if odom_topic else ["estop"])
        report["estop"] = "engaged after a failed step; release it from the joystick or `services call <estop> '{\"data\": false}'`"

    for st in steps:
        trace(f"step {st['step']} {st['capability']}")
        entry = {"step": st["step"], "text": st.get("text") or st["capability"], "ok": True}
        report["steps"].append(entry)
        cmds = st["commands"]
        if st["capability"] == "set_drive_mode":
            argv = shlex.split(cmds[0])
            svc, want = argv[2], json.loads(argv[3])["data"]
            if plan.get("preflight"):  # the pre-motion check already reports the drive mode: one read instead of two
                pf_argv = shlex.split(plan["preflight"])
                if "--source" in pf_argv:
                    pf_argv[pf_argv.index("--source") + 1] = "nav2" if want else "teleop"
                pf, _rc = run_cli_fn(pf_argv)
                mode = pf.get("drive_mode")
                if any("emergency stop" in b for b in pf.get("blocked_by") or []):
                    entry.update(ok=False, result=_gist(pf))
                    report.update(ok=False, blocked=True)
                    return report
                state = None if mode is None else mode == "nav2"
                if state == want or (state is None and want is False):
                    pf_cache = pf  # right mode already: reuse this check for the motion step
            else:
                state = run_cli_fn(["services", "state", svc])[0].get("states", {}).get(svc)
            if state == want or (state is None and want is False):
                entry["result"] = {"skipped": f"already in {'nav2' if want else 'teleop'} mode"}
                continue
        if st.get("_motion") and plan.get("preflight"):
            pf, pf_cache = pf_cache, None
            if pf is None:
                pf_argv = shlex.split(plan["preflight"])
                if "--source" in pf_argv and st.get("_source"):
                    pf_argv[pf_argv.index("--source") + 1] = st["_source"]
                pf, _rc = run_cli_fn(pf_argv)
            if not pf.get("ready_for_motion"):
                entry.update(ok=False, result=_gist(pf))
                report.update(ok=False, blocked=True)
                return report
            start = start or _position(pf)
        for cmd in cmds:
            argv = shlex.split(cmd)
            try:
                out, rc = run_cli_fn(argv)
            except KeyboardInterrupt:  # Ctrl-C or SIGTERM between or inside a command
                out, rc = {"cancelled": True, "error": "interrupted"}, 130
            if out.get("cancelled"):  # estop or Ctrl-C: stop here, never carry on to the next step
                report.update(ok=False, cancelled=True)
                entry.update(ok=False, result={"cancelled": True})
                try:
                    from ros2_utils import uninterruptible
                    with uninterruptible():
                        run_cli_fn(["stop", "--verify-odom-topic", odom_topic] if odom_topic else ["stop"])
                except Exception:
                    pass
                return report
            good = rc == 0 and "error" not in out and out.get("success", True) is not False and out.get("condition_met", True)
            if not good:
                fail(entry, out)
                return report
            entry["result"] = out if argv[0] == "snapshot" else _gist(out)
            if argv[:2] == ["topics", "publish-until"] and odom_topic:
                stop, _ = run_cli_fn(["stop", "--verify-odom-topic", odom_topic])
                entry["result"]["stopped"] = {k: stop[k] for k in ("verified_stationary", "final_position") if k in stop}
                if stop.get("final_position"):
                    final = (stop["final_position"]["x"], stop["final_position"]["y"])
    if start and final:
        report["moved_m"] = round(math.hypot(final[0] - start[0], final[1] - start[1]), 3)
    return report


def cmd_plan(args):
    import pathlib

    from ros2_utils import output, ros2_context
    from ros2_profile import load_profile_summary
    from ros2_places import list_places as _list_places

    from ros2_utils import trace
    trace("plan: start")
    goal = " ".join(args.goal).strip()
    artifacts = str(pathlib.Path(__file__).resolve().parent.parent / ".artifacts")
    summary = load_profile_summary(readonly=True)
    trace("plan: profile loaded")
    result = compile_plan(goal, summary, _list_places(), artifacts)
    trace("plan: compiled")
    if getattr(args, "run", False) and result["warnings"]:  # a partial plan would skip the unmatched parts silently
        return output({"goal": goal, "executed": False, "ok": False, "refused": "part of the goal was not compiled; nothing ran",
                       "warnings": result["warnings"], "steps": [s["text"] for s in result["steps"]]})
    if getattr(args, "run", False):
        from ros2_motion_lock import motion_lock
        fused = ((summary or {}).get("localization_config") or {}).get("fused_sources") or {}
        odom = next(iter(fused.values()), None)
        from ros2_utils import _FASTD_SOCKET
        import contextlib
        # With the fast daemon up, nothing in the plan needs rclpy in this process; a command that falls back opens its own.
        shared = contextlib.nullcontext() if os.path.exists(_FASTD_SOCKET) and not os.environ.get("ROS2_SKILL_NO_FASTD") else ros2_context()
        with shared, motion_lock("plan --run"):
            trace("plan: rclpy init + lock done")
            try:
                report = run_plan(result, run_cli, odom)
            except KeyboardInterrupt:  # interrupted outside a command (preflight, mode switch): nothing is moving
                report = {"goal": goal, "executed": True, "ok": False, "cancelled": True, "steps": []}
        report["warnings"] = result["warnings"]
        from ros2_utils import trace
        trace("plan: done (rclpy shutdown follows)")
        return output(report)
    for st in result["steps"]:
        for k in [k for k in st if k.startswith("_")]:
            st.pop(k)
    result["executed"] = False
    result["note"] = ("Nothing was run. Run `preflight` first, execute steps in order, verify each (Rule 8), "
                      "and `stop` after publish-until.")
    output(result)
