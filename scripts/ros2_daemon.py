#!/usr/bin/env python3
"""ROS 2 daemon management commands.

Delegates to ``ros2 daemon start/stop/status`` via subprocess so that the
commands work regardless of whether the ``ros2cli`` Python package is
importable from the current Python environment.

No live ROS 2 graph is required.  Domain ID is read from the
``ROS_DOMAIN_ID`` environment variable (default: 0).

Note: importing this module requires rclpy to be installed because the
``output`` helper is imported from ros2_utils, which validates the rclpy
environment at import time.  The daemon commands themselves make no rclpy
calls.
"""

import os
import shlex
import socket
import subprocess
import tempfile
import time

from ros2_utils import output, check_tmux, session_exists, kill_session, run_cmd, source_local_ws

_SKILL_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
_FASTD_SESSION = "ros2_skill_fastd"
_FASTD_SCRIPT = os.path.join(_SKILL_ROOT, "scripts", "ros2_fastd.py")
_FASTD_SOCKET = os.path.join(_SKILL_ROOT, ".fastd", "fastd.sock")
_FASTD_PIDFILE = os.path.join(_SKILL_ROOT, ".fastd", "fastd.pid")


# ---------------------------------------------------------------------------
# Internal helpers
# ---------------------------------------------------------------------------

def _get_domain_id() -> int:
    """Return the active ROS domain ID (ROS_DOMAIN_ID env var, default 0)."""
    try:
        return int(os.environ.get("ROS_DOMAIN_ID", "0"))
    except (ValueError, TypeError):
        return 0


def _ros2_daemon(subcmd: str, domain_id: int) -> subprocess.CompletedProcess:
    """Run ``ros2 daemon <subcmd>`` with *domain_id* set in the environment."""
    env = {**os.environ, "ROS_DOMAIN_ID": str(domain_id)}
    return subprocess.run(
        ["ros2", "daemon", subcmd],
        capture_output=True, text=True, timeout=15, env=env,
    )


def ensure_daemon_running(domain_id=None, timeout=15):
    """Check the ROS 2 daemon and start it if it's down. Best-effort, silent.

    A dead daemon doesn't fail loudly — subprocess-based package/graph
    discovery (e.g. during 'profile scan') can instead hang or silently
    return a partial result while blocked on daemon IPC. Call this at the
    top of any command that depends on the daemon being reachable.

    Returns a dict describing what happened — never raises:
      {"was_running": bool, "action": "none"|"started"|"start_failed"|"error",
       "detail": str (only on start_failed/error)}
    """
    if domain_id is None:
        domain_id = _get_domain_id()
    try:
        proc = _ros2_daemon("status", domain_id)
        text = (proc.stdout + proc.stderr).strip()
        running = proc.returncode == 0 and "not running" not in text.lower()
        if running:
            return {"was_running": True, "action": "none"}

        start_proc = _ros2_daemon("start", domain_id)
        start_text = (start_proc.stdout + start_proc.stderr).strip()
        if start_proc.returncode == 0:
            return {"was_running": False, "action": "started"}
        return {"was_running": False, "action": "start_failed", "detail": start_text}
    except Exception as exc:
        return {"was_running": None, "action": "error", "detail": str(exc)}


# ---------------------------------------------------------------------------
# Commands
# ---------------------------------------------------------------------------

def cmd_daemon_status(args):
    """Check whether the ROS 2 daemon is running.

    Delegates to ``ros2 daemon status``.  Output keys:

    * ``status``    – ``"running"`` or ``"not_running"``
    * ``domain_id`` – active ROS domain ID
    * ``output``    – raw text from the ros2 CLI
    """
    domain_id = None
    try:
        domain_id = _get_domain_id()
        proc = _ros2_daemon("status", domain_id)
        text = (proc.stdout + proc.stderr).strip()
        running = proc.returncode == 0 and "not running" not in text.lower()
        output({
            "status": "running" if running else "not_running",
            "domain_id": domain_id,
            "output": text,
        })
    except Exception as exc:
        output({"error": str(exc), "domain_id": domain_id if domain_id is not None else 0})


def cmd_daemon_start(args):
    """Start the ROS 2 daemon.

    Delegates to ``ros2 daemon start``.  Output keys:

    * ``status``    – ``"started"`` on success, ``"error"`` on failure
    * ``domain_id`` – active ROS domain ID
    * ``output``    – raw text from the ros2 CLI (success path)
    * ``detail``    – error detail from the ros2 CLI (error path)
    """
    domain_id = None
    try:
        domain_id = _get_domain_id()
        proc = _ros2_daemon("start", domain_id)
        text = (proc.stdout + proc.stderr).strip()
        if proc.returncode != 0:
            output({"status": "error", "detail": text, "domain_id": domain_id})
        else:
            output({"status": "started", "domain_id": domain_id, "output": text})
    except Exception as exc:
        output({"error": str(exc), "domain_id": domain_id if domain_id is not None else 0})


def cmd_daemon_stop(args):
    """Stop the ROS 2 daemon.

    Delegates to ``ros2 daemon stop``.  Output keys:

    * ``status``    – ``"stopped"`` on success, ``"error"`` on failure
    * ``domain_id`` – active ROS domain ID
    * ``output``    – raw text from the ros2 CLI (success path)
    * ``detail``    – error detail from the ros2 CLI (error path)
    """
    domain_id = None
    try:
        domain_id = _get_domain_id()
        proc = _ros2_daemon("stop", domain_id)
        text = (proc.stdout + proc.stderr).strip()
        if proc.returncode != 0:
            output({"status": "error", "detail": text, "domain_id": domain_id})
        else:
            output({"status": "stopped", "domain_id": domain_id, "output": text})
    except Exception as exc:
        output({"error": str(exc), "domain_id": domain_id if domain_id is not None else 0})


# ---------------------------------------------------------------------------
# Fast daemon (ros2_fastd.py) — the skill's own persistent rclpy node,
# distinct from the built-in "ros2 daemon" managed above. Purely a latency
# optimization for hot-path commands (see ros2_utils.try_fastd); every
# command that can use it also has a working fallback when it isn't running.
# ---------------------------------------------------------------------------

def _fastd_socket_alive():
    """Return True if a process is actually listening on the fastd socket."""
    if not os.path.exists(_FASTD_SOCKET):
        return False
    try:
        s = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        s.settimeout(2.0)
        s.connect(_FASTD_SOCKET)
        s.close()
        return True
    except Exception:
        return False


def cmd_daemon_fast_start(args):
    """Start the persistent fast-daemon (ros2_fastd.py) in a tmux session."""
    if not check_tmux():
        return output({"error": "tmux is not installed",
                        "suggestion": "Install with: sudo apt install tmux"})

    if session_exists(_FASTD_SESSION):
        if _fastd_socket_alive():
            return output({"status": "already_running", "session": _FASTD_SESSION})
        # Session exists but socket is dead — stale session, clean it up first.
        kill_session(_FASTD_SESSION)
        _cleanup_fastd_scripts()

    # Explicitly source both the base distro and the local workspace overlay —
    # .bashrc only sources ROS for interactive shells, so a tmux-spawned
    # non-interactive bash cannot be assumed to have it already (see
    # SKILL.md session-start notes on this same gap).
    #
    # Written to a script file rather than interpolated into nested
    # shell-quoted strings — same pattern as ros2_launch.py/ros2_run.py/
    # ros2_tf.py. Not currently exploitable here (distro/ws_path aren't
    # free-form user CLI input), but kept consistent so this file doesn't
    # become the odd one out if that ever changes.
    distro = os.environ.get("ROS_DISTRO", "")
    script_lines = ["#!/bin/bash", "set -e", "exec 2>&1"]
    if distro:
        script_lines.append(f"source {shlex.quote(f'/opt/ros/{distro}/setup.bash')}")
    ws_path, ws_status = source_local_ws()
    if ws_status == "found" and ws_path:
        script_lines.append(f"source {shlex.quote(ws_path)}")
    script_lines.append(f"exec python3 {shlex.quote(_FASTD_SCRIPT)}")

    fastd_dir = os.path.dirname(_FASTD_SOCKET)
    os.makedirs(fastd_dir, exist_ok=True)
    fd, script_path = tempfile.mkstemp(prefix="fastd_", suffix=".sh", dir=fastd_dir)
    with os.fdopen(fd, "w") as f:
        f.write("\n".join(script_lines) + "\n")
    os.chmod(script_path, 0o700)

    tmux_cmd = f"tmux new-session -d -s {_FASTD_SESSION} bash {shlex.quote(script_path)}"
    _, stderr, rc = run_cmd(tmux_cmd)
    if rc != 0:
        return output({"error": "Failed to start fastd tmux session", "detail": stderr})

    # Wait briefly for the socket to appear rather than a fixed sleep.
    deadline = time.time() + 10.0
    while time.time() < deadline:
        if _fastd_socket_alive():
            return output({"status": "started", "session": _FASTD_SESSION, "socket": _FASTD_SOCKET})
        time.sleep(0.2)

    return output({
        "error": "fastd session started but socket did not come up in time",
        "session": _FASTD_SESSION,
        "hint": f"Check logs: tmux attach -t {_FASTD_SESSION}",
    })


def _cleanup_fastd_scripts():
    """Remove generated fastd start-up script(s) — see cmd_daemon_fast_start."""
    fastd_dir = os.path.dirname(_FASTD_SOCKET)
    try:
        for name in os.listdir(fastd_dir):
            if name.startswith("fastd_") and name.endswith(".sh"):
                try:
                    os.remove(os.path.join(fastd_dir, name))
                except OSError:
                    pass
    except OSError:
        pass
    # tmux kill-session sends a signal the daemon process doesn't handle as
    # gracefully as Ctrl+C (only SIGINT is caught as KeyboardInterrupt in
    # ros2_fastd.py's main()), so its own finally-block cleanup of the
    # pidfile doesn't reliably run — remove it here instead.
    if os.path.exists(_FASTD_PIDFILE):
        try:
            os.remove(_FASTD_PIDFILE)
        except OSError:
            pass


def cmd_daemon_fast_stop(args):
    """Stop the persistent fast-daemon."""
    if not session_exists(_FASTD_SESSION):
        if os.path.exists(_FASTD_SOCKET):
            try:
                os.remove(_FASTD_SOCKET)
            except Exception:
                pass
        _cleanup_fastd_scripts()
        return output({"status": "not_running"})
    ok = kill_session(_FASTD_SESSION)
    if os.path.exists(_FASTD_SOCKET):
        try:
            os.remove(_FASTD_SOCKET)
        except Exception:
            pass
    _cleanup_fastd_scripts()
    output({"status": "stopped" if ok else "error", "session": _FASTD_SESSION})


def cmd_daemon_fast_status(args):
    """Check whether the persistent fast-daemon is running and reachable."""
    tmux_up = session_exists(_FASTD_SESSION)
    socket_alive = _fastd_socket_alive()
    output({
        "session_running": tmux_up,
        "socket_reachable": socket_alive,
        "status": "running" if (tmux_up and socket_alive) else "not_running",
        "session": _FASTD_SESSION,
        "socket": _FASTD_SOCKET,
    })


if __name__ == "__main__":
    import sys
    import os
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
