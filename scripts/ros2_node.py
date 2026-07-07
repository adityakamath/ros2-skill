#!/usr/bin/env python3
"""ROS 2 node commands."""

import os
import signal
import subprocess
import time

from ros2_utils import ROS2CLI, output, ros2_context, try_fastd


def _split_node_path(full_name):
    """Split '/namespace/node_name' into (node_name, namespace)."""
    s = full_name.lstrip('/')
    if '/' in s:
        idx = s.rindex('/')
        return s[idx + 1:], '/' + s[:idx]
    return s, '/'


def _kill_by_name_substring(pattern, sig=signal.SIGTERM):
    """Signal processes whose cmdline contains *pattern*, matching pkill -f's
    semantics but via direct /proc scanning + os.kill() instead of the pkill
    binary — pkill -f was found to hang indefinitely in this environment
    (confirmed in isolation, even wrapped in the shell `timeout` command),
    while killing a specific PID directly works reliably. Best-effort: only
    matches nodes whose OS process cmdline contains their ROS node name
    (true for most `ros2 run`/launch-file invocations via `--ros-args -r
    __node:=NAME`, not for a node that sets its name purely in code).
    Returns the list of PIDs actually signaled.
    """
    my_pid = os.getpid()
    signaled = []
    try:
        pids = os.listdir("/proc")
    except OSError:
        return signaled
    for entry in pids:
        if not entry.isdigit():
            continue
        pid = int(entry)
        if pid == my_pid:
            continue
        try:
            with open(f"/proc/{entry}/cmdline", "rb") as f:
                cmdline = f.read().replace(b"\x00", b" ").decode(errors="replace")
        except OSError:
            continue
        if pattern in cmdline:
            try:
                os.kill(pid, sig)
                signaled.append(pid)
            except OSError:
                pass
    return signaled


def cmd_nodes_list(args):
    # Fast path: persistent daemon, no rclpy.init() for this call at all.
    daemon_resp = try_fastd("list_nodes", {})
    if daemon_resp is not None:
        return output(daemon_resp)

    try:
        with ros2_context():
            node = ROS2CLI()
            node_info = node.get_node_names_and_namespaces()
        names = [f"{ns.rstrip('/')}/{n}" for n, ns in node_info]
        output({"nodes": names, "count": len(names)})
    except Exception as e:
        output({"error": str(e)})


def cmd_nodes_details(args):
    # Fast path: persistent daemon, no rclpy.init() for this call at all.
    daemon_resp = try_fastd("node_details", {"node": args.node})
    if daemon_resp is not None:
        return output(daemon_resp)

    try:
        with ros2_context():
            node = ROS2CLI()
            node_name, namespace = _split_node_path(args.node)
            publishers = node.get_publisher_names_and_types_by_node(node_name, namespace)
            subscribers = node.get_subscriber_names_and_types_by_node(node_name, namespace)
            services = node.get_service_names_and_types_by_node(node_name, namespace)
            result = {
                "node": args.node,
                "publishers": [topic for topic, _ in publishers],
                "subscribers": [topic for topic, _ in subscribers],
                "services": [svc for svc, _ in services],
            }
            try:
                result["action_servers"] = [
                    name for name, _ in
                    node.get_action_server_names_and_types_by_node(node_name, namespace)
                ]
                result["action_clients"] = [
                    name for name, _ in
                    node.get_action_client_names_and_types_by_node(node_name, namespace)
                ]
            except AttributeError:
                result["action_servers"] = []
                result["action_clients"] = []
        output(result)
    except Exception as e:
        output({"error": str(e)})


def cmd_nodes_kill(args):
    """Terminate a running ROS 2 node.

    Attempts graceful lifecycle shutdown first; falls back to killing by
    process name match. Requires ``--confirm`` as a safety gate.
    """
    if not getattr(args, "confirm", False):
        return output({
            "error": "Safety gate: --confirm is required to kill a node",
            "hint": f"Re-run with: nodes kill {args.node} --confirm",
        })

    node_name = args.node.lstrip("/")
    full_name = args.node if args.node.startswith("/") else f"/{args.node}"
    timeout = getattr(args, "timeout", 10.0)
    # Most nodes are NOT lifecycle-managed, so this is a quick probe, not
    # a real wait. Previously this used the full --timeout (default 10s)
    # for the external `ros2 lifecycle set` subprocess, so killing a
    # plain node ate up to 10s just to discover it has no /change_state
    # service before falling through to pkill.
    probe_timeout = min(timeout, 2.0)

    method = None
    killed = False

    # 1. Graceful: lifecycle shutdown via ros2 CLI subprocess (short probe)
    try:
        r = subprocess.run(
            ["ros2", "lifecycle", "set", full_name, "shutdown"],
            capture_output=True, text=True, timeout=probe_timeout,
        )
        if r.returncode == 0:
            method = "lifecycle_shutdown"
            killed = True
    except (subprocess.TimeoutExpired, FileNotFoundError):
        pass

    # 2. Fallback: kill by node name substring (see _kill_by_name_substring
    # — avoids the pkill binary, which hangs indefinitely in this
    # environment). Try SIGTERM first, then SIGKILL for anything still
    # alive a moment later.
    if not killed:
        matched_pids = _kill_by_name_substring(node_name, signal.SIGTERM)
        if matched_pids:
            time.sleep(0.3)
            still_alive = []
            for pid in matched_pids:
                try:
                    os.kill(pid, 0)
                    still_alive.append(pid)
                except OSError:
                    pass
            for pid in still_alive:
                try:
                    os.kill(pid, signal.SIGKILL)
                except OSError:
                    pass
            killed = True
            method = "kill_by_name"
        else:
            method = "kill_by_name (no matching process found)"

    # 3. Verify node is gone from graph. Prefer the daemon (near-instant
    # per check — no rclpy.init() at all) so the polling loop's 3s budget
    # is spent on actual polling instead of repeated RMW discovery. Only
    # if the daemon isn't running do we fall back to rclpy — and even
    # then, ONE persistent node is reused across the whole polling window
    # instead of a fresh rclpy.init()/shutdown() cycle per iteration
    # (which could singlehandedly consume the entire 3s budget on 1-2
    # iterations, since each cycle pays full RMW discovery).
    node_gone = False
    deadline = time.time() + 3.0
    try:
        daemon_up = try_fastd("list_nodes", {}) is not None
        if daemon_up:
            while time.time() < deadline:
                daemon_resp = try_fastd("list_nodes", {})
                names = (daemon_resp or {}).get("nodes", [])
                if full_name not in names:
                    node_gone = True
                    break
                time.sleep(0.2)
        else:
            with ros2_context():
                cli = ROS2CLI()
                while time.time() < deadline:
                    names = [
                        f"{ns.rstrip('/')}/{n}"
                        for n, ns in cli.get_node_names_and_namespaces()
                    ]
                    if full_name not in names:
                        node_gone = True
                        break
                    time.sleep(0.3)
    except Exception:
        pass

    if not killed and not node_gone:
        return output({
            "killed": False,
            "node": full_name,
            "error": "Could not kill node — lifecycle shutdown and kill-by-name both failed",
        })

    return output({
        "killed": True,
        "node": full_name,
        "method": method,
        "node_gone": node_gone,
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
