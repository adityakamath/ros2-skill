#!/usr/bin/env python3
"""File-backed Nav2 mission runner for pause/resume/cancel control.

This runner executes a mission spec stored in a JSON state file. The CLI
creates the state file and spawns this process detached; pause/resume/cancel
commands update the same file. If the state file disappears or becomes
unreadable while a mission is active, the runner fails closed by calling
Nav2 cancel and sending a zero-velocity burst.
"""

import argparse
import copy
import io
import json
import os
import pathlib
import sys
import time
import uuid
from contextlib import redirect_stdout
from datetime import datetime, timezone
from types import SimpleNamespace

_SCRIPT_DIR = pathlib.Path(__file__).resolve().parent
_SKILL_ROOT = _SCRIPT_DIR.parent
_MISSION_DIR = _SKILL_ROOT / ".artifacts" / "missions"


def _now():
    return datetime.now(timezone.utc).isoformat()


def _mission_path(mission_id: str) -> pathlib.Path:
    _MISSION_DIR.mkdir(parents=True, exist_ok=True)
    safe_id = "".join(ch for ch in mission_id if ch.isalnum() or ch in ("-", "_")) or "mission"
    return _MISSION_DIR / f"{safe_id}.json"


def load_state(mission_id: str):
    path = _mission_path(mission_id)
    if not path.exists():
        return None
    try:
        return json.loads(path.read_text(encoding="utf-8"))
    except Exception:
        return None


def save_state(state):
    path = _mission_path(state["mission_id"])
    state = copy.deepcopy(state)
    state["updated_at"] = _now()
    path.write_text(json.dumps(state, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    return path


def _capture_output(func, args):
    buf = io.StringIO()
    with redirect_stdout(buf):
        func(args)
    text = buf.getvalue().strip()
    if not text:
        return {}
    last_line = text.splitlines()[-1]
    try:
        return json.loads(last_line)
    except Exception:
        return {"raw_output": text}


def _is_success(result):
    if not isinstance(result, dict):
        return False
    if "error" in result:
        return False
    if "success" in result:
        return bool(result["success"])
    if result.get("blocked"):
        return False
    return True


def _mission_fail_safe(mission_id: str, reason: str):
    from ros2_nav2 import cmd_nav2_cancel  # noqa: PLC0415

    try:
        cancel_args = SimpleNamespace(action="/navigate_to_pose", timeout=5.0)
        _capture_output(cmd_nav2_cancel, cancel_args)
    except Exception:
        pass

    state = load_state(mission_id)
    if state:
        state["status"] = "failed"
        state["failure_reason"] = reason
        save_state(state)


def _wait_for_resume(mission_id: str, poll_interval: float = 0.25):
    while True:
        state = load_state(mission_id)
        if state is None:
            _mission_fail_safe(mission_id, "mission state vanished")
            return None
        status = state.get("status", "")
        if status == "paused":
            time.sleep(poll_interval)
            continue
        if status == "canceling":
            return state
        return state


def _build_step_args(step):
    step_type = (step.get("type") or "").strip().lower()
    base = dict(step)
    base.pop("type", None)
    return step_type, SimpleNamespace(**base)


def _step_handlers():
    from ros2_nav2 import (
        cmd_nav2_costmap_clear, cmd_nav2_go, cmd_nav2_initial_pose,
        cmd_nav2_localize, cmd_nav2_map_load, cmd_nav2_mode_set,
        cmd_nav2_move_timed, cmd_nav2_rotate,
    )

    return {
        "go": cmd_nav2_go,
        "rotate": cmd_nav2_rotate,
        "move_timed": cmd_nav2_move_timed,
        "initial_pose": cmd_nav2_initial_pose,
        "localize": cmd_nav2_localize,
        "costmap_clear": cmd_nav2_costmap_clear,
        "map_load": cmd_nav2_map_load,
        "mode_set": cmd_nav2_mode_set,
    }


def run_mission(mission_id: str):
    state = load_state(mission_id)
    if state is None:
        return 1

    state["status"] = "running"
    state["runner_pid"] = os.getpid()
    state.setdefault("history", [])
    save_state(state)

    handlers = _step_handlers()
    steps = list(state.get("steps", []))

    for index, step in enumerate(steps):
        state = _wait_for_resume(mission_id)
        if state is None:
            return 1
        if state.get("status") == "canceling":
            _mission_fail_safe(mission_id, "mission canceled")
            return 2

        handler = handlers.get((step.get("type") or "").strip().lower())
        if handler is None:
            state = load_state(mission_id) or state
            state["status"] = "failed"
            state["failure_reason"] = f"unknown step type: {step.get('type')}"
            save_state(state)
            return 1

        state = load_state(mission_id) or state
        state["current_step"] = index
        state["current_step_type"] = step.get("type")
        save_state(state)

        result = _capture_output(handler, _build_step_args(step)[1])
        state = load_state(mission_id) or state
        state.setdefault("history", []).append({
            "index": index,
            "type": step.get("type"),
            "result": result,
            "ok": _is_success(result),
        })
        save_state(state)

        if not _is_success(result):
            state["status"] = "failed"
            state["failure_reason"] = result.get("error") or f"step {index} failed"
            save_state(state)
            return 1

    state = load_state(mission_id) or state
    state["status"] = "completed"
    save_state(state)
    return 0


def main(argv=None):
    parser = argparse.ArgumentParser(description="Run a file-backed Nav2 mission")
    parser.add_argument("--mission-id", required=True)
    args = parser.parse_args(argv)
    return run_mission(args.mission_id)


if __name__ == "__main__":
    raise SystemExit(main())