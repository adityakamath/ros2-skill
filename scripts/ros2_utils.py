#!/usr/bin/env python3
"""Shared utilities for ROS 2 Skill CLI modules.

Provides type resolution, message serialization, output helpers, and the
base ROS2CLI node class consumed by every domain module.
"""

import importlib
import json
import os
import re
import shlex
import socket
import subprocess
import sys
import time
from contextlib import contextmanager

# rclpy itself costs ~0.9s to import (dominant per-command latency, measured
# independently of the fast-daemon). Every domain module imports this file,
# so rclpy is loaded lazily on first actual use (ros2_context, ROS2CLI(),
# call_ros_service) instead of unconditionally at module load time — that
# lets pure-Python paths (try_fastd, get_msg_type, output, session mgmt,
# argparse construction) run without ever paying the tax.
rclpy = None


def _ensure_rclpy():
    """Import rclpy on first use and cache the module globally."""
    global rclpy
    if rclpy is not None:
        return rclpy
    try:
        import rclpy as _rclpy
    except ImportError as e:
        print(json.dumps({"error": f"Missing ROS 2 dependency: {e}. Source ROS 2 setup.bash or install the missing package."}))
        sys.exit(1)
    rclpy = _rclpy
    return rclpy


# ---------------------------------------------------------------------------
# Distribution detection
# ---------------------------------------------------------------------------

# Canonical ROS 2 release ordering (oldest → newest).
# Listed alphabetically, which matches actual release chronology for ROS 2.
# Unknown distro names are ranked alphabetically relative to this list so
# future releases (e.g. anything after "lyrical") are handled gracefully
# without requiring code changes.
_DISTRO_ORDER: list = [
    "humble",   # H — May 2022
    "iron",     # I — May 2023
    "jazzy",    # J — May 2024
    "kilted",   # K — May 2025
    "lyrical",  # L — May 2026
]

_DISTRO_UNKNOWN = "unknown"


def get_ros_distro() -> str:
    """Return the active ROS 2 distribution name in lowercase.

    Reads ``$ROS_DISTRO`` (set automatically by ``source setup.bash``).
    Returns ``"unknown"`` when the variable is absent, which causes
    :func:`is_at_least` to return ``False`` for every distro check so that
    Lyrical-only code paths are never taken on unrecognised systems.
    """
    return os.environ.get("ROS_DISTRO", _DISTRO_UNKNOWN).lower().strip()


def _distro_rank(name: str) -> int:
    """Return a numeric rank for *name* suitable for ordering comparisons.

    Known distros get their index in ``_DISTRO_ORDER``.  Unknown names are
    ranked by their alphabetical position relative to known distros, so
    any future release that sorts after "lyrical" is automatically treated
    as newer than all currently known distros.
    """
    name = name.lower().strip()
    try:
        return _DISTRO_ORDER.index(name)
    except ValueError:
        # Alphabetical fallback: find where name would sit in the sorted list.
        for i, known in enumerate(_DISTRO_ORDER):
            if name < known:
                return i - 1  # ranks below this known distro
        # Alphabetically after all known distros → newer than all of them.
        return len(_DISTRO_ORDER)


def is_at_least(distro: str) -> bool:
    """Return ``True`` if the active distro is *distro* or newer.

    Uses the canonical release ordering so callers never need to hard-code
    string comparisons.  Returns ``False`` when ``$ROS_DISTRO`` is unset.

    Examples::

        # On a Lyrical system:
        is_at_least("humble")   # True  — Lyrical ≥ Humble
        is_at_least("lyrical")  # True  — exact match
        is_at_least("melodic")  # False — hypothetical future distro

        # When $ROS_DISTRO is unset:
        is_at_least("humble")   # False — always safe default
    """
    active = get_ros_distro()
    if active == _DISTRO_UNKNOWN:
        return False
    return _distro_rank(active) >= _distro_rank(distro)


# ---------------------------------------------------------------------------
# QoS helpers
# ---------------------------------------------------------------------------

_SERVICE_EVENT_QOS = None  # initialised lazily after rclpy import guard above


def _get_service_event_qos():
    """Return (and cache) the QoS profile for service event topics."""
    global _SERVICE_EVENT_QOS
    if _SERVICE_EVENT_QOS is None:
        from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
        _SERVICE_EVENT_QOS = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=100,
        )
    return _SERVICE_EVENT_QOS


# ---------------------------------------------------------------------------
# rclpy lifecycle
# ---------------------------------------------------------------------------

@contextmanager
def ros2_context():
    """Initialise rclpy on entry and shut it down on exit."""
    _ensure_rclpy()
    rclpy.init()
    try:
        yield
    finally:
        rclpy.shutdown()


def call_ros_service(node, srv_type, service_name, request, timeout=5.0, retries=1,
                      unavailable_hint=None):
    """Call a ROS 2 service and return ``(response, error_dict)``.

    Creates the client, waits for the service, fires the async call, spins
    until done or timeout, then destroys the client in a finally block.
    Returns ``(response, None)`` on success, or ``(None, {"error": "..."})``
    if the service never became available or timed out.

    ``retries`` (default 1) controls how many total attempts are made
    before giving up — each attempt pays the full ``wait_for_service``
    timeout, so total wall-clock time can be up to ``retries * timeout``.

    ``unavailable_hint``, if given, is appended as an extra sentence to the
    "service not available" error message (e.g. "Is the controller manager
    running?") — callers with domain-specific guidance can supply this
    without needing their own copy of the whole wait/call/retry loop.

    This is the single shared implementation behind ros2_control.py's
    _call_cm_service, ros2_param.py's _call_service, and ros2_nav2.py's
    _call_service — those three (plus ros2_lifecycle.py and ros2_service.py,
    which called this same sequence inline) used to carry independent
    copies of this loop.
    """
    _ensure_rclpy()
    client = node.create_client(srv_type, service_name)
    try:
        for attempt in range(retries):
            last_attempt = (attempt == retries - 1)

            if not client.wait_for_service(timeout_sec=timeout):
                if not last_attempt:
                    continue
                msg = f"Service not available: {service_name}"
                if unavailable_hint:
                    msg += f". {unavailable_hint}"
                return None, {"error": msg}

            future = client.call_async(request)
            end_time = time.time() + timeout
            while time.time() < end_time and not future.done():
                rclpy.spin_once(node, timeout_sec=0.1)

            if future.done():
                return future.result(), None

            future.cancel()
            if not last_attempt:
                continue

        return None, {"error": f"Timeout calling {service_name}"}
    finally:
        client.destroy()


# ---------------------------------------------------------------------------
# Fast daemon (persistent rclpy node) — optional acceleration
# ---------------------------------------------------------------------------

_FASTD_SOCKET = os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))), ".fastd", "fastd.sock"
)


def try_fastd(op, params, timeout=10.0):
    """Attempt to service a request via the persistent fast-daemon (ros2_fastd.py).

    Returns the daemon's response dict on success, or ``None`` if the daemon
    isn't running/reachable or anything goes wrong — this function never
    raises. Callers MUST treat ``None`` as "fall back to the normal
    rclpy.init()-per-call path", not as an error; the daemon is a pure
    optimization and its absence must never change behavior, only latency.
    """
    if not os.path.exists(_FASTD_SOCKET):
        return None
    try:
        sock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        sock.settimeout(timeout)
        sock.connect(_FASTD_SOCKET)
        try:
            request = json.dumps({"op": op, "params": params}) + "\n"
            sock.sendall(request.encode("utf-8"))
            data = b""
            while not data.endswith(b"\n"):
                chunk = sock.recv(65536)
                if not chunk:
                    break
                data += chunk
        finally:
            sock.close()
        if not data:
            return None
        return json.loads(data.decode("utf-8"))
    except Exception:
        return None


# ---------------------------------------------------------------------------
# Type resolution helpers
# ---------------------------------------------------------------------------

def get_msg_type(type_str):
    if not type_str:
        return None

    # Normalize to pkg, msg_name components
    if '/msg/' in type_str:
        pkg, msg_name = type_str.split('/msg/', 1)
        msg_name = msg_name.strip()
    elif '/srv/' in type_str:
        pkg, msg_name = type_str.split('/srv/', 1)
        msg_name = msg_name.strip()
        try:
            module = importlib.import_module(f"{pkg}.srv")
            return getattr(module, msg_name)
        except Exception:
            return None
    elif '/action/' in type_str:
        pkg, msg_name = type_str.split('/action/', 1)
        msg_name = msg_name.strip()
        try:
            module = importlib.import_module(f"{pkg}.action")
            return getattr(module, msg_name)
        except Exception:
            return None
    elif '/' in type_str:
        pkg, msg_name = type_str.rsplit('/', 1)
    elif '.' in type_str:
        parts = type_str.split('.')
        try:
            module = importlib.import_module('.'.join(parts[:-1]))
            return getattr(module, parts[-1])
        except Exception:
            return None
    else:
        return None

    try:
        module = importlib.import_module(f"{pkg}.msg")
        return getattr(module, msg_name)
    except Exception:
        pass

    return None


def get_action_type(type_str):
    """Import a ROS 2 action type class from a type string."""
    if not type_str:
        return None

    if '/action/' in type_str:
        pkg, action_name = type_str.split('/action/', 1)
        action_name = action_name.strip()
    elif '/' in type_str:
        pkg, action_name = type_str.rsplit('/', 1)
    else:
        return None

    try:
        module = importlib.import_module(f"{pkg}.action")
        return getattr(module, action_name)
    except Exception:
        pass

    return None


def get_srv_type(type_str):
    """Import a ROS 2 service type class from a type string."""
    if not type_str:
        return None

    if '/srv/' in type_str:
        pkg, srv_name = type_str.split('/srv/', 1)
        srv_name = srv_name.strip()
    elif '/' in type_str:
        pkg, srv_name = type_str.rsplit('/', 1)
    else:
        return None

    try:
        module = importlib.import_module(f"{pkg}.srv")
        return getattr(module, srv_name)
    except Exception:
        pass

    return None


def get_msg_error(msg_type):
    """Generate helpful error message when message type cannot be loaded."""
    ros_distro = get_ros_distro()
    hint = "ROS 2 message types use /msg/ format (e.g., geometry_msgs/msg/Twist)"
    if ros_distro != _DISTRO_UNKNOWN:
        hint += f". Ensure ROS 2 workspace is built: cd ~/ros2_ws && colcon build && source install/setup.bash"
    else:
        hint += ". Ensure ROS 2 environment is sourced: source /opt/ros/<distro>/setup.bash"
    return {
        "error": f"Unknown message type: {msg_type}",
        "hint": hint,
        "ros_distro": ros_distro if ros_distro != _DISTRO_UNKNOWN else None,
        "troubleshooting": [
            "1. Source ROS 2: source /opt/ros/<distro>/setup.bash",
            "2. If using custom messages, build workspace: cd ~/ros2_ws && colcon build",
            "3. Verify: python3 -c 'from geometry_msgs.msg import Twist; print(Twist)'"
        ]
    }


# ---------------------------------------------------------------------------
# Message serialization
# ---------------------------------------------------------------------------

def msg_to_dict(msg):
    result = {}
    for field in msg.get_fields_and_field_types():
        value = getattr(msg, field, None)
        if value is None:
            continue
        if hasattr(value, 'get_fields_and_field_types'):
            result[field] = msg_to_dict(value)
        elif isinstance(value, (bytes, bytearray)):
            result[field] = list(value)
        elif isinstance(value, (list, tuple)):
            result[field] = [
                msg_to_dict(v) if hasattr(v, 'get_fields_and_field_types')
                else v.tolist() if hasattr(v, 'tolist')
                else v
                for v in value
            ]
        elif hasattr(value, 'tolist'):
            result[field] = value.tolist()
        else:
            result[field] = value
    return result


def dict_to_msg(msg_type, data):
    msg = msg_type()
    field_types = msg.get_fields_and_field_types()
    for key, value in data.items():
        if not hasattr(msg, key):
            continue
        if isinstance(value, dict):
            setattr(msg, key, dict_to_msg(getattr(msg, key).__class__, value))
        elif isinstance(value, list) and value and isinstance(value[0], dict):
            field_type_str = field_types.get(key, '')
            m = re.search(r'sequence<([^,>]+)(?:,\s*\d+\s*)?>', field_type_str) or re.search(r'^(.+?)\[\d*\]$', field_type_str)
            if m:
                elem_class = get_msg_type(m.group(1).strip())
                if elem_class:
                    setattr(msg, key, [dict_to_msg(elem_class, v) for v in value])
                else:
                    setattr(msg, key, value)
            else:
                setattr(msg, key, value)
        else:
            setattr(msg, key, value)
    return msg


def resolve_field(d, path):
    """Resolve a dot-separated field path into a nested dict/list structure."""
    current = d
    for part in path.split('.'):
        if isinstance(current, list):
            current = current[int(part)]
        else:
            current = current[part]
    return current


# ---------------------------------------------------------------------------
# Output helpers
# ---------------------------------------------------------------------------

def _json_default(obj):
    """Fallback JSON encoder for types not handled by msg_to_dict."""
    if hasattr(obj, 'tolist'):
        return obj.tolist()
    return str(obj)


def output(data):
    print(json.dumps(data, indent=2, ensure_ascii=False, default=_json_default))
    if isinstance(data, dict) and "error" in data:
        sys.exit(1)


# ---------------------------------------------------------------------------
# Base ROS 2 node
# ---------------------------------------------------------------------------

# ROS2CLI is used exclusively as ROS2CLI() / ROS2CLI("name") across every
# domain module (never subclassed) — that lets it be a factory function
# instead of a class, so rclpy.node.Node is only imported the first time a
# caller actually constructs a node, not at ros2_utils import time.
_ROS2CLI_CLASS = None


def _get_ROS2CLI_class():
    global _ROS2CLI_CLASS
    if _ROS2CLI_CLASS is None:
        _ensure_rclpy()
        from rclpy.node import Node

        class _ROS2CLIImpl(Node):
            def __init__(self, node_name='ros2_cli'):
                super().__init__(node_name)

            def get_topic_names(self):
                return self.get_topic_names_and_types()

            def get_service_names(self):
                return self.get_service_names_and_types()

        _ROS2CLI_CLASS = _ROS2CLIImpl
    return _ROS2CLI_CLASS


def ROS2CLI(node_name='ros2_cli'):
    return _get_ROS2CLI_class()(node_name)


# ---------------------------------------------------------------------------
# Misc helpers
# ---------------------------------------------------------------------------

def resolve_topic_type(node, topic, provided_type=None):
    """Return the message type for *topic* from the live graph.

    Returns *provided_type* unchanged if already set.  Otherwise queries the
    topic name/type list from *node*.  Returns None if the topic is not found.
    """
    if provided_type:
        return provided_type
    for name, types in node.get_topic_names_and_types():
        if name == topic and types:
            return types[0]
    return None


def get_msg_fields(msg_type_str):
    try:
        msg_class = get_msg_type(msg_type_str)
        if msg_class is None:
            return {}
        msg = msg_class()
        return msg_to_dict(msg)
    except Exception:
        return {}


def parse_node_param(name):
    if ':' in name:
        parts = name.split(':', 1)
        return parts[0], parts[1]
    return name, None


def resolve_output_path(filename_or_path):
    """Resolve an --output argument to an absolute path.

    If *filename_or_path* contains no directory component (plain filename),
    the file is placed in the ``.artifacts/`` directory next to this package,
    creating it when necessary.  Otherwise the value is treated as an explicit
    path and returned as an absolute path (parent directories are not created).
    """
    if os.path.dirname(filename_or_path):
        # Caller supplied an explicit directory — honour it as-is.
        return os.path.abspath(filename_or_path)
    # Plain filename: resolve to .artifacts/ beside this script.
    artifacts_dir = os.path.join(os.path.dirname(__file__), '..', '.artifacts')
    artifacts_dir = os.path.abspath(artifacts_dir)
    os.makedirs(artifacts_dir, exist_ok=True)
    return os.path.join(artifacts_dir, filename_or_path)


# ---------------------------------------------------------------------------
# Local workspace helpers
# ---------------------------------------------------------------------------

def source_local_ws(user_provided_ws=None):
    """Get local ROS 2 workspace path to source before running commands.
    
    System ROS is assumed to be already sourced (via systemd or manually).
    This helper finds the local workspace to source on top of system ROS.
    
    Args:
        user_provided_ws: Optional user-provided workspace path (from ROS2_LOCAL_WS env var)
    
    Search order:
    1. ROS2_LOCAL_WS environment variable
    2. ~/ros2_ws
    3. ~/colcon_ws
    4. ~/dev_ws
    5. ~/workspace
    6. ~/ros2
    
    Returns:
        tuple: (path_to_setup_bash, status)
            - ("/path/to/local_setup.bash", "found") - workspace found and built
            - (None, "not_built") - workspace found but local_setup.bash doesn't exist
            - (None, "not_found") - no workspace found
    """
    import os
    
    # Common workspace patterns to search (in priority order)
    ws_patterns = [
        os.environ.get('ROS2_LOCAL_WS'),  # User override
        '~/ros2_ws',      # Common default
        '~/colcon_ws',    # Common default
        '~/dev_ws',       # Common default
        '~/workspace',    # Generic
        '~/ros2',        # Generic
    ]
    
    def find_setup_files(ws_path):
        """Find valid setup files in workspace, handling various build layouts."""
        if not ws_path or not os.path.exists(ws_path):
            return None
        
        # Resolve symlinks
        ws_path = os.path.realpath(ws_path)
        
        # Try different setup file locations and build types
        possible_setups = [
            ('install/local_setup.bash', 'install/setup.bash'),
            ('install/setup.bash', 'install/setup.bash'),
            ('build/local_setup.bash', 'build/setup.bash'),
            ('build/setup.bash', 'build/setup.bash'),
            ('devel/local_setup.bash', 'devel/setup.bash'),
            ('devel/setup.bash', 'devel/setup.bash'),
            ('install/local_setup.bash', 'install/local_setup.bash'),  # merge-install
            ('setup.bash', 'setup.bash'),  # root-level (merge-install)
        ]
        
        for local_setup, fallback in possible_setups:
            local_path = os.path.join(ws_path, local_setup)
            if os.path.exists(local_path):
                return local_path
        
        return None
    
    best_status = "not_found"
    best_path = None
    ros2_local_ws_error = None
    
    # Check if ROS2_LOCAL_WS is set but invalid
    ros2_local_ws = os.environ.get('ROS2_LOCAL_WS')
    if ros2_local_ws:
        expanded = os.path.expanduser(ros2_local_ws)
        if not os.path.exists(expanded):
            ros2_local_ws_error = f"ROS2_LOCAL_WS is set to '{ros2_local_ws}' but path does not exist"
    
    for ws_pattern in ws_patterns:
        if not ws_pattern:
            continue
            
        ws_path = os.path.expanduser(ws_pattern)
        
        # Skip if path doesn't exist
        if not os.path.exists(ws_path):
            continue
        
        setup_path = find_setup_files(ws_path)
        
        if setup_path:
            return setup_path, "found"
        elif best_status == "not_found":
            # Mark as found but not built - continue searching for better options
            best_path = ws_path
            best_status = "not_built"
    
    # Return best effort: found a workspace but none are built
    if best_path:
        if ros2_local_ws_error:
            return None, "invalid"
        return None, "not_built"
    
    # ROS2_LOCAL_WS was set but invalid - return error
    if ros2_local_ws_error:
        return None, "invalid"
    
    # No workspace found
    return None, "not_found"


# ---------------------------------------------------------------------------
# Session Management (shared by launch and run commands)
# ---------------------------------------------------------------------------

# Package cache for launch and run commands
_package_cache = {}
_package_cache_initialized = False


def run_cmd(cmd, timeout=10):
    """Run a shell command and return output."""
    try:
        result = subprocess.run(
            cmd, shell=True, capture_output=True, text=True, timeout=timeout
        )
        return result.stdout.strip(), result.stderr.strip(), result.returncode
    except subprocess.TimeoutExpired:
        return "", "Command timed out", 1
    except Exception as e:
        return "", str(e), 1


def refresh_package_cache():
    """Force refresh of the package cache."""
    global _package_cache, _package_cache_initialized
    _package_cache = {}
    _package_cache_initialized = False
    list_packages()


def list_packages(force_refresh=False):
    """List all ROS 2 packages (cached)."""
    global _package_cache, _package_cache_initialized
    
    if force_refresh:
        _package_cache = {}
        _package_cache_initialized = False
    
    if _package_cache_initialized:
        return _package_cache
    
    stdout, _, rc = run_cmd("ros2 pkg list")
    if rc == 0:
        packages = stdout.strip().split('\n') if stdout.strip() else []
        for pkg in packages:
            _package_cache[pkg] = True
        _package_cache_initialized = True
    
    return _package_cache


def package_exists(package, force_refresh=False):
    """Check if a package exists (uses cache, refreshes if not found or force_refresh=True)."""
    packages = list_packages(force_refresh=force_refresh)
    if package in packages:
        return True
    
    global _package_cache_initialized
    if not force_refresh:
        _package_cache_initialized = False
        list_packages()
        return package in list_packages()
    
    return False


def get_package_prefix(package):
    """Get the prefix path for a package."""
    stdout, _, rc = run_cmd(f"ros2 pkg prefix {shlex.quote(package)}")
    if rc == 0 and stdout:
        return stdout.strip()
    return None


def check_tmux():
    """Check if tmux is available."""
    stdout, _, rc = run_cmd("which tmux")
    return rc == 0 and stdout.strip() != ""


def generate_session_name(session_type, package, name):
    """Generate a tmux session name."""
    safe_name = "".join(c for c in name if c.isalnum() or c in '_-')[:20]
    return f"{session_type}_{package}_{safe_name}"[:50]


def session_exists(session_name):
    """Check if a tmux session exists."""
    check_cmd = f"tmux has-session -t {shlex.quote(session_name)} 2>/dev/null"
    stdout, stderr, rc = run_cmd(check_cmd)
    if rc == 0:
        return True
    # Double-check with list-sessions to handle edge cases
    stdout, _, rc = run_cmd("tmux list-sessions -F '#{session_name}' 2>/dev/null")
    if rc != 0:
        return False
    return session_name in stdout.strip().split('\n')


def kill_session(session_name, graceful_timeout=10.0):
    """Kill a tmux session gracefully, then fall back to a hard kill.

    `tmux kill-session` alone sends SIGHUP to the pane's process group.
    `ros2 launch` (and many other long-running ROS 2 processes) only
    registers signal handlers for SIGINT/SIGTERM to run its shutdown
    sequence — lifecycle deactivation, hardware interface cleanup (e.g.
    releasing servo torque), node destructors, etc. SIGHUP is not one of
    those, so a bare `kill-session` tears down the whole process tree
    before any of that cleanup runs, silently skipping it.

    This sends Ctrl-C (SIGINT) to the pane first — exactly what an
    operator would press if attached to the session — and gives the
    process up to `graceful_timeout` seconds to exit and run its own
    cleanup. Only if it's still alive after that does it fall back to a
    hard `tmux kill-session`, which is now just a safety net rather than
    the primary mechanism.
    """
    run_cmd(f"tmux send-keys -t {shlex.quote(session_name)} C-c")
    deadline = time.time() + graceful_timeout
    while time.time() < deadline:
        if not session_exists(session_name):
            return True
        if not check_session_alive(session_name):
            break
        time.sleep(0.3)
    kill_cmd = f"tmux kill-session -t {shlex.quote(session_name)}"
    stdout, stderr, rc = run_cmd(kill_cmd)
    return rc == 0


def check_session_alive(session_name):
    """Check if session has a running process (not just empty shell)."""
    pid_cmd = f"tmux list-panes -t {shlex.quote(session_name)} -F '#{{pane_pid}}' 2>/dev/null | head -1"
    pid_out, _, _ = run_cmd(pid_cmd)
    
    if not pid_out:
        return False
    
    proc_cmd = f"ps -p {pid_out.strip()} -o state= 2>/dev/null | tr -d ' '"
    state_out, _, _ = run_cmd(proc_cmd)
    
    if state_out.strip() in ('R', 'S', 'D'):
        return True
    
    return False


def quote_path(path):
    """Quote a path to handle spaces and special characters."""
    if not path:
        return path
    return '"' + path.replace('\\', '\\\\').replace('"', '\\"') + '"'


def get_sessions_file():
    """Get path to session metadata file."""
    return os.path.expanduser("~/.ros2_cli_sessions.json")


def load_sessions():
    """Load session metadata from file."""
    sessions_file = get_sessions_file()
    if os.path.exists(sessions_file):
        try:
            with open(sessions_file, 'r') as f:
                return json.load(f)
        except (json.JSONDecodeError, IOError):
            return {}
    return {}


def save_session(session_name, metadata):
    """Save session metadata to file."""
    sessions_file = get_sessions_file()
    sessions = load_sessions()
    sessions[session_name] = metadata
    try:
        with open(sessions_file, 'w') as f:
            json.dump(sessions, f)
    except IOError:
        pass


def get_session_metadata(session_name):
    """Get session metadata from file."""
    sessions = load_sessions()
    return sessions.get(session_name)


def delete_session_metadata(session_name):
    """Delete session metadata from file."""
    sessions_file = get_sessions_file()
    sessions = load_sessions()
    if session_name in sessions:
        del sessions[session_name]
        try:
            with open(sessions_file, 'w') as f:
                json.dump(sessions, f)
        except IOError:
            pass


# ---------------------------------------------------------------------------
# Shared session command helpers for launch and run
# ---------------------------------------------------------------------------

def list_sessions(prefix):
    """List running sessions filtered by prefix.
    
    Args:
        prefix: Session name prefix to filter (e.g., 'launch_' or 'run_')
    
    Returns:
        dict with all_sessions, sessions (filtered), and sessions_detail
    """
    if not check_tmux():
        return {
            "error": "tmux is not installed",
            "running_sessions": []
        }
    
    stdout, stderr, rc = run_cmd("tmux list-sessions -F '#{session_name}' 2>/dev/null")
    
    if rc != 0 or not stdout.strip():
        return {
            f"{prefix.strip('_')}_sessions": [],
            "running_sessions": []
        }
    
    all_sessions = stdout.strip().split('\n')
    
    # Filter by prefix
    filtered_sessions = [s for s in all_sessions if s.startswith(prefix)]
    
    # Get details for each session
    sessions_info = []
    for session in filtered_sessions:
        info = {"session": session}
        
        pane_cmd = f"tmux list-panes -t {shlex.quote(session)} -F '#{{pane_title}}' 2>/dev/null"
        pane_out, _, _ = run_cmd(pane_cmd)
        if pane_out:
            info["command"] = pane_out.strip()

        check_cmd = f"tmux has-session -t {shlex.quote(session)} 2>/dev/null && echo 'running' || echo 'stopped'"
        status, _, _ = run_cmd(check_cmd)
        info["status"] = status.strip() if status else "unknown"
        
        sessions_info.append(info)
    
    return {
        "all_sessions": all_sessions,
        f"{prefix.strip('_')}_sessions": filtered_sessions,
        f"{prefix.strip('_')}_sessions_detail": sessions_info
    }


def kill_session_cmd(session, prefix):
    """Kill a session with prefix validation.
    
    Args:
        session: Session name to kill
        prefix: Expected prefix (e.g., 'launch_' or 'run_')
    
    Returns:
        dict with success/error status
    """
    if not check_tmux():
        return {"error": "tmux is not installed"}
    
    # Validate session name starts with prefix
    if not session.startswith(prefix):
        return {
            "error": f"Session '{session}' is not a {prefix.strip('_')} session",
            "hint": f"{prefix.strip('_').capitalize()} sessions start with '{prefix}'"
        }
    
    # Check if session exists
    if not session_exists(session):
        return {
            "error": f"Session '{session}' does not exist",
            "available_sessions": []
        }
    
    # Kill the session
    if not kill_session(session):
        return {
            "error": f"Failed to kill session: {session}",
            "session": session
        }

    # Clean up the generated launch script (see ros2_launch.py cmd_launch_run —
    # the launch command is written to a script file rather than interpolated
    # into a shell string, to avoid a nested-quoting injection vulnerability;
    # each launch creates a new file, so it must be removed here or
    # .launch_scripts/ accumulates one file per launch forever).
    metadata = get_session_metadata(session)
    script_path = (metadata or {}).get("script_path")
    if script_path and os.path.exists(script_path):
        try:
            os.remove(script_path)
        except OSError:
            pass

    # Clean up session metadata
    delete_session_metadata(session)

    return {
        "success": True,
        "session": session,
        "message": f"Session '{session}' killed"
    }


def fuzzy_match(query, candidates, threshold=0.5):
    """Fuzzy match *query* against *candidates*.

    Returns a list of (candidate, score) tuples sorted by score descending.
    Scores: 1.0 exact, 0.8 substring, 0.7 prefix, 0.5 word match.
    """
    if not query or not candidates:
        return []
    query_lower = query.lower().replace('_', '').replace('-', '')
    matches = []
    for candidate in candidates:
        c = candidate.lower().replace('_', '').replace('-', '')
        if query_lower == c:
            matches.append((candidate, 1.0))
        elif query_lower in c or c in query_lower:
            matches.append((candidate, 0.8))
        elif c.startswith(query_lower):
            matches.append((candidate, 0.7))
        elif any(word in c for word in query_lower.split()):
            matches.append((candidate, 0.5))
    matches.sort(key=lambda x: x[1], reverse=True)
    return matches


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
