#!/usr/bin/env python3
"""AG-7 Safety Heartbeat Watchdog.

Standalone script launched in a detached tmux session by 'safety heartbeat start'.
Polls the sentinel file mtime; fires an estop if the sentinel is not updated
within timeout_s seconds.

Fires exactly once, then exits. Re-arming requires 'safety heartbeat start'.
"""

import json
import os
import pathlib
import subprocess
import sys
import time

_SCRIPT_DIR = pathlib.Path(__file__).resolve().parent
_SKILL_ROOT = _SCRIPT_DIR.parent
_SENTINEL_PATH = _SKILL_ROOT / ".artifacts" / "heartbeat.sentinel"
_HEARTBEAT_LOG = _SKILL_ROOT / ".artifacts" / "heartbeat.log"
_CONFIG_PATH = _SKILL_ROOT / ".presets" / "safety.json"


def _load_heartbeat_config() -> tuple[int, int, str]:
    """Return (timeout_s, poll_interval_s, on_timeout) from safety.json."""
    try:
        cfg = json.loads(_CONFIG_PATH.read_text(encoding="utf-8"))
        hb = cfg.get("heartbeat", {})
        timeout_s = int(hb.get("timeout_s", 5))
        poll_s = int(hb.get("poll_interval_s", 1))
        on_timeout = hb.get("on_timeout", "estop")
    except Exception:
        timeout_s, poll_s, on_timeout = 5, 1, "estop"
    return max(1, timeout_s), max(1, poll_s), on_timeout


def _fire_estop() -> None:
    """Publish zero-velocity burst via ros2_cli.py estop command."""
    try:
        subprocess.run(
            [sys.executable, str(_SCRIPT_DIR / "ros2_cli.py"), "estop"],
            timeout=15,
        )
    except Exception as exc:
        _log({"watchdog_estop_error": str(exc)})


def _log(entry: dict) -> None:
    """Append a JSON entry to the heartbeat log."""
    try:
        _HEARTBEAT_LOG.parent.mkdir(parents=True, exist_ok=True)
        with _HEARTBEAT_LOG.open("a", encoding="utf-8") as f:
            f.write(json.dumps(entry, default=str) + "\n")
    except Exception:
        pass


def main() -> None:
    timeout_s, poll_s, on_timeout = _load_heartbeat_config()

    _log({
        "event": "watchdog_started",
        "timeout_s": timeout_s,
        "poll_interval_s": poll_s,
        "on_timeout": on_timeout,
        "started_at": time.time(),
    })

    while True:
        time.sleep(poll_s)

        if not _SENTINEL_PATH.exists():
            _log({
                "event": "watchdog_sentinel_missing",
                "note": "Sentinel file gone; treating as timeout.",
                "fired_at": time.time(),
            })
            break

        try:
            age = time.time() - os.path.getmtime(_SENTINEL_PATH)
        except Exception:
            age = float("inf")

        if age > timeout_s:
            _log({
                "event": "watchdog_fired",
                "sentinel_age_s": round(age, 2),
                "timeout_s": timeout_s,
                "action": on_timeout,
                "fired_at": time.time(),
            })
            break

    # Fire
    if on_timeout == "estop":
        _fire_estop()
    else:
        _log({"event": "watchdog_log_only", "note": "on_timeout=log; no estop sent."})

    _log({"event": "watchdog_exited", "exited_at": time.time()})


if __name__ == "__main__":
    main()
