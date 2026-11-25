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


def _is_node_process(argv, node_name, namespace, prefixes):
    """True if the process command line *argv* belongs to the node *namespace*/*node_name*.

    A ``-r __node:=NAME`` remap decides the name. Without one, the executable must
    be named exactly like the node and be recognisably ROS (``--ros-args``, or
    installed under ``<prefix>/lib/<package>/``). The ``__ns`` remap, "/" if
    absent, must equal *namespace*. Never matches on substrings.
    """
    names, namespaces = [], []
    for flag, rule in zip(argv, argv[1:]):
        if flag not in ("-r", "--remap") or ":=" not in rule:
            continue
        lhs, value = rule.split(":=", 1)
        target, _, key = lhs.rpartition(":")  # "talker:__ns" applies to one node
        if target not in ("", node_name):
            continue
        if key in ("__node", "__name"):
            names.append(value)
        elif key == "__ns":
            namespaces.append("/" + value.strip("/"))

    if (namespaces[0] if namespaces else "/") != namespace:
        return False
    if names:
        return node_name in names

    exe = argv[0] if argv else ""
    if os.path.basename(exe).startswith("python"):  # python3 [-u] <script>
        exe = next((a for a in argv[1:] if not a.startswith("-")), "")
    if os.path.basename(exe) != node_name:
        return False
    lib_dir = os.path.dirname(os.path.dirname(exe))
    return "--ros-args" in argv or any(lib_dir == os.path.join(p, "lib") for p in prefixes)


def _find_node_pids(node_name, namespace, proc_root="/proc"):
    """PIDs of the processes running the node, found by scanning /proc (the pkill
    binary was found to hang in this environment). A node that sets its name or
    namespace only in code, or runs inside a component container, is not found.
    """
    prefixes = [p.rstrip("/") for p in os.environ.get("AMENT_PREFIX_PATH", "").split(os.pathsep) if p]
    try:
        entries = os.listdir(proc_root)
    except OSError:
        return []
    pids = []
    for entry in entries:
        if not entry.isdigit() or int(entry) == os.getpid():
            continue
        try:
            with open(os.path.join(proc_root, entry, "cmdline"), "rb") as f:
                argv = f.read().decode(errors="replace").rstrip("\0").split("\0")
        except OSError:
            continue
        if _is_node_process(argv, node_name, namespace, prefixes):
            pids.append(int(entry))
    return pids


def _signal_pids(pids, sig):
    """Send *sig* to each PID; return those that accepted it."""
    signaled = []
    for pid in pids:
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

    Attempts graceful lifecycle shutdown first; falls back to signalling the
    process whose node name and namespace match exactly. Requires ``--confirm``
    as a safety gate.
    """
    if not getattr(args, "confirm", False):
        return output({
            "error": "Safety gate: --confirm is required to kill a node",
            "hint": f"Re-run with: nodes kill {args.node} --confirm",
        })

    node_name, namespace = _split_node_path(args.node)
    full_name = args.node if args.node.startswith("/") else f"/{args.node}"
    timeout = getattr(args, "timeout", 10.0)
    # Most nodes are NOT lifecycle-managed, so this is a quick probe, not
    # a real wait. Previously this used the full --timeout (default 10s)
    # for the external `ros2 lifecycle set` subprocess, so killing a
    # plain node ate up to 10s just to discover it has no /change_state
    # service before falling through to the process kill.
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

    # 2. Fallback: signal the node's own process (exact match, see
    # _is_node_process). SIGTERM first, then SIGKILL for anything still
    # alive a moment later.
    if not killed:
        matched_pids = _signal_pids(_find_node_pids(node_name, namespace), signal.SIGTERM)
        if not matched_pids:
            return output({
                "killed": False,
                "node": full_name,
                "error": "Could not kill node: lifecycle shutdown failed and no process "
                         "matches this node name and namespace exactly",
                "hint": "Stop it through its session (launch kill / run kill / component kill), "
                        "or unload it from its container (component unload)",
            })
        time.sleep(0.3)
        _signal_pids(_signal_pids(matched_pids, 0), signal.SIGKILL)
        killed = True
        method = "kill_by_name"

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

    return output({
        "killed": killed,
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
