#!/usr/bin/env python3
"""Client for moves hosted by the fast daemon (rclpy-free, so a daemon-served move never imports rclpy).

The daemon keeps the velocity publisher and odometry subscription warm, so a move starts in tens of milliseconds
instead of paying rclpy start-up and graph discovery. Safety stays the same:
  * limits are applied here (``prepare_publish_until``) and again in the daemon;
  * the daemon publishes only while this client's socket is open, so Ctrl-C, ``kill`` or a dead client halts the move;
  * ``estop`` and ``stop`` cancel the running move first;
  * if the daemon itself dies, the base controller's ``cmd_vel_timeout`` ends the motion.
A request the daemon never received falls back to the normal in-process path. A request it may have received never
does (no double move): the caller gets an error and the robot is stopped.
"""

import json
import os
import socket

from ros2_utils import _FASTD_SOCKET


def _request(op, params, timeout):
    """Send one request. Returns ``(status, response)``: status is "unavailable" (never sent), "lost" or "ok"."""
    if os.environ.get("ROS2_SKILL_NO_FASTD") or not os.path.exists(_FASTD_SOCKET):
        return "unavailable", None
    sock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    try:
        sock.settimeout(2.0)
        sock.connect(_FASTD_SOCKET)
        sock.sendall((json.dumps({"op": op, "params": params}) + "\n").encode("utf-8"))
    except OSError:
        sock.close()
        return "unavailable", None
    try:
        sock.settimeout(timeout)
        data = b""
        while not data.endswith(b"\n"):
            chunk = sock.recv(65536)
            if not chunk:
                break
            data += chunk
        return ("ok", json.loads(data.decode("utf-8"))) if data else ("lost", None)
    except (OSError, ValueError):
        return "lost", None
    finally:
        sock.close()  # closing is also the cancel signal: the daemon halts a move whose client is gone


def try_publish_until(args, plan):
    """Run an eligible ``publish-until`` in the daemon. Returns its result dict, or None to use the local path."""
    if plan["dry_run"]:
        return None
    euclid_ok = plan["euclidean"] and plan["rotate_angle"] is None and plan["operator"] == "delta"
    if not (euclid_ok or plan["rotate_angle"] is not None):
        return None  # generic field conditions (above/below/equals) stay in-process
    slow_last = getattr(args, "slow_last", None)
    slow_zone = None
    if slow_last is not None:
        import math
        slow_zone = math.radians(slow_last) if (plan["rotate_angle"] is not None and plan["use_degrees"]) else float(slow_last)
    params = {
        "topic": args.topic, "msg": plan["msg_data"], "msg_type": args.msg_type, "monitor": args.monitor,
        "monitor_msg_type": args.monitor_msg_type, "fields": plan["field_paths"], "euclidean": plan["euclidean"],
        "threshold": plan["threshold"], "rotate": plan["rotate_angle"], "timeout": args.timeout,
        "rate": args.rate or 10.0, "slow_zone": slow_zone, "slow_factor": getattr(args, "slow_factor", 0.25),
        "degrees": plan["use_degrees"], "clamp_notices": plan["clamp_notices"], "safety_capped": plan["safety_action"] == "cap",
        "safety_warning": plan["safety_warning"],
    }
    status, resp = _request("move", params, (args.timeout or 30) + 20)
    if status == "unavailable":
        return None
    if status == "lost" or resp is None:
        stop_robot()
        return {"success": False, "error": "lost the fast daemon during a move; velocity was zeroed, nothing was retried",
                "cancelled": True}
    if resp.get("fallback"):  # the daemon declined before moving (no hub, unknown types): run it here
        return None
    return resp


def stop_robot(odom_topic=None, verify_timeout=5.0, threshold=0.01):
    """Routine stop in the daemon. Returns its result, or None when the daemon is not available."""
    status, resp = _request("stop", {"odom_topic": odom_topic, "verify_timeout": verify_timeout, "threshold": threshold},
                            verify_timeout + 10)
    return resp if status == "ok" and resp and not resp.get("fallback") else None


def cancel_motion():
    """Ask the daemon to halt a move it is running (used by ``estop``). True if one was running."""
    status, resp = _request("cancel_motion", {}, 3.0)
    return bool(status == "ok" and resp and resp.get("cancelled"))


def try_serve(command, sub, args):
    """Serve a motion-path command entirely from the daemon, before its (rclpy-importing) module is loaded.

    Returns the result dict, or None to run the normal handler. Covers ``preflight motion``, ``stop`` and eligible
    ``topics publish-until``.
    """
    if os.environ.get("ROS2_SKILL_NO_FASTD") or not os.path.exists(_FASTD_SOCKET):
        return None
    if (command, sub) == ("topics", "publish-until"):
        from ros2_motion_core import prepare_publish_until
        err, plan = prepare_publish_until(args)
        return err if err else try_publish_until(args, plan)
    if (command, sub) == ("stop", None):
        return None if getattr(args, "topic", None) else stop_robot(
            getattr(args, "verify_odom_topic", None), getattr(args, "verify_timeout", 5.0), getattr(args, "velocity_threshold", 0.01))
    if (command, sub) == ("preflight", "motion"):
        params = {"controller": args.controller, "odom_topic": args.odom_topic, "source": getattr(args, "source", "teleop"),
                  "controller_manager": args.controller_manager, "timeout": args.timeout,
                  "stationary_timeout": args.stationary_timeout, "velocity_threshold": args.velocity_threshold,
                  "verbose": getattr(args, "verbose", False)}
        status, resp = _request("preflight_motion", params, args.timeout + args.stationary_timeout + 10)
        return resp if status == "ok" and resp else None
    return None


_AUTOSTART = {("snapshot", None), ("look", None), ("preflight", "motion"), ("stop", None), ("topics", "publish-until")}
_AUTOSTART_IDLE_MINUTES = 30
_STARTING_WINDOW_S = 20


def _daemon_alive():
    if not os.path.exists(_FASTD_SOCKET):
        return False
    probe = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    try:
        probe.settimeout(1.0)
        probe.connect(_FASTD_SOCKET)
        return True
    except OSError:
        return False
    finally:
        probe.close()


def maybe_autostart(command, sub, args):
    """Start the fast daemon in the background the first time a command that benefits from it runs.

    This command does not wait: it takes the normal per-call path and the next one finds the daemon up. The
    daemon started here exits after 30 idle minutes. Not started: for other commands, with
    ROS2_SKILL_NO_FASTD / ROS2_SKILL_NO_AUTOSTART set, while one was started in the last 20 s, or if one is up.
    """
    import subprocess
    import sys
    import time

    if os.environ.get("ROS2_SKILL_NO_FASTD") or os.environ.get("ROS2_SKILL_NO_AUTOSTART"):
        return False
    if not ((command, sub) in _AUTOSTART or (command == "plan" and getattr(args, "run", False))):
        return False
    if _daemon_alive():
        return False
    marker = os.path.join(os.path.dirname(_FASTD_SOCKET), "starting")
    try:
        os.makedirs(os.path.dirname(marker), exist_ok=True)
        if time.time() - os.path.getmtime(marker) < _STARTING_WINDOW_S:
            return False
    except OSError:
        pass
    try:
        with open(marker, "w") as handle:
            handle.write(str(os.getpid()))
        cli = os.path.join(os.path.dirname(os.path.abspath(__file__)), "ros2_cli.py")
        subprocess.Popen([sys.executable, cli, "daemon-fast", "start", "--idle-minutes", str(_AUTOSTART_IDLE_MINUTES)],
                         stdin=subprocess.DEVNULL, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                         start_new_session=True, env=dict(os.environ, ROS2_SKILL_NO_AUTOSTART="1"))
        return True
    except OSError:
        return False
