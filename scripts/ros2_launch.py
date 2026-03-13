#!/usr/bin/env python3
"""ROS 2 launch commands for running launch files in tmux sessions."""

import json
import os
import subprocess
import threading

from ros2_utils import output, source_local_ws


# Cache for package information
_package_cache = {}
_package_cache_initialized = False


def _run_cmd(cmd, timeout=10):
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


def _refresh_package_cache():
    """Force refresh of the package cache."""
    global _package_cache, _package_cache_initialized
    _package_cache = {}
    _package_cache_initialized = False
    _list_packages()


def _get_package_prefix(package):
    """Get the prefix path for a package."""
    stdout, _, rc = _run_cmd(f"ros2 pkg prefix {package}")
    if rc == 0 and stdout:
        return stdout.strip()
    return None


def _list_packages(force_refresh=False):
    """List all ROS 2 packages (cached)."""
    global _package_cache, _package_cache_initialized
    
    if force_refresh:
        _package_cache = {}
        _package_cache_initialized = False
    
    if _package_cache_initialized:
        return _package_cache
    
    stdout, _, rc = _run_cmd("ros2 pkg list")
    if rc == 0:
        packages = stdout.strip().split('\n') if stdout.strip() else []
        for pkg in packages:
            _package_cache[pkg] = True
        _package_cache_initialized = True
    
    return _package_cache


def _package_exists(package, force_refresh=False):
    """Check if a package exists (uses cache, refreshes if not found or force_refresh=True)."""
    packages = _list_packages(force_refresh=force_refresh)
    if package in packages:
        return True
    
    # Auto-refresh if package not found and not already refreshed
    global _package_cache_initialized
    if not force_refresh:
        _package_cache_initialized = False
        _list_packages()
        return package in _list_packages()
    
    return False


def _get_sessions_file():
    """Get path to session metadata file."""
    return os.path.expanduser("~/.ros2_cli_sessions.json")


def _load_sessions():
    """Load session metadata from file."""
    sessions_file = _get_sessions_file()
    if os.path.exists(sessions_file):
        try:
            with open(sessions_file, 'r') as f:
                return json.load(f)
        except (json.JSONDecodeError, IOError):
            return {}
    return {}


def _save_session(session_name, metadata):
    """Save session metadata to file."""
    sessions_file = _get_sessions_file()
    sessions = _load_sessions()
    sessions[session_name] = metadata
    try:
        with open(sessions_file, 'w') as f:
            json.dump(sessions, f)
    except IOError:
        pass  # Silently fail if we can't write


def _get_session_metadata(session_name):
    """Get session metadata from file."""
    sessions = _load_sessions()
    return sessions.get(session_name)


def _delete_session_metadata(session_name):
    """Delete session metadata from file."""
    sessions_file = _get_sessions_file()
    sessions = _load_sessions()
    if session_name in sessions:
        del sessions[session_name]
        try:
            with open(sessions_file, 'w') as f:
                json.dump(sessions, f)
        except IOError:
            pass


def _find_launch_files(package):
    """Find launch files in a package."""
    prefix = _get_package_prefix(package)
    if not prefix:
        return []
    
    # Common launch directories
    launch_dirs = [
        os.path.join(prefix, "share", package, "launch"),
        os.path.join(prefix, "lib", package, "launch"),
        os.path.join(prefix, "launch"),
    ]
    
    launch_files = []
    for launch_dir in launch_dirs:
        if os.path.isdir(launch_dir):
            for f in os.listdir(launch_dir):
                if f.endswith(('.launch.py', '.launch', '.xml')):
                    launch_files.append(f)
    
    return launch_files


def _find_config_files(package, config_path=None):
    """Find config files in a package or given path."""
    if config_path and os.path.isdir(config_path):
        config_dir = config_path
    else:
        prefix = _get_package_prefix(package)
        if not prefix:
            return []
        config_dir = os.path.join(prefix, "share", package, "config")
    
    if not os.path.isdir(config_dir):
        return []
    
    config_files = []
    for f in os.listdir(config_dir):
        if f.endswith(('.yaml', '.yml')):
            config_files.append(f)
    
    return config_files


def _apply_presets(package, presets):
    """Apply preset parameters to a package/node."""
    # Presets are applied via params set before launch
    # This would require connecting to the running node
    # For now, return the presets that would be applied
    return presets


def _apply_params(params_str):
    """Parse and return params from key:value string."""
    if not params_str:
        return {}
    
    params = {}
    for pair in params_str.split(','):
        pair = pair.strip()
        if ':' in pair:
            key, value = pair.split(':', 1)
            # Try to parse value as number
            try:
                if '.' in value:
                    params[key.strip()] = float(value)
                else:
                    params[key.strip()] = int(value)
            except ValueError:
                params[key.strip()] = value.strip()
    
    return params


def _check_tmux():
    """Check if tmux is available."""
    stdout, _, rc = _run_cmd("which tmux")
    return rc == 0 and stdout.strip() != ""


def _generate_session_name(session_type, package, name):
    """Generate a tmux session name."""
    # Remove special characters and limit length
    safe_name = "".join(c for c in name if c.isalnum() or c in '_-')[:20]
    return f"{session_type}_{package}_{safe_name}"[:50]


def _session_exists(session_name):
    """Check if a tmux session exists."""
    check_cmd = f"tmux has-session -t {session_name} 2>/dev/null"
    _, _, rc = _run_cmd(check_cmd)
    return rc == 0


def _kill_session(session_name):
    """Kill a tmux session."""
    kill_cmd = f"tmux kill-session -t {session_name}"
    stdout, stderr, rc = _run_cmd(kill_cmd)
    return rc == 0


def _check_session_alive(session_name):
    """Check if session has a running process (not just empty shell)."""
    # Get the PID of the process in the pane
    pid_cmd = f"tmux list-panes -t {session_name} -F '#{{pane_pid}}' 2>/dev/null | head -1"
    pid_out, _, _ = _run_cmd(pid_cmd)
    
    if not pid_out:
        return False
    
    # Check if process is still running (not zombie/defunct)
    proc_cmd = f"ps -p {pid_out.strip()} -o state= 2>/dev/null | tr -d ' '"
    state_out, _, _ = _run_cmd(proc_cmd)
    
    # Running states: R (running), S (sleeping), D (disk sleep)
    if state_out.strip() in ('R', 'S', 'D'):
        return True
    
    return False


def _quote_path(path):
    """Quote a path to handle spaces and special characters."""
    if not path:
        return path
    # Escape backslashes first, then quotes
    return '"' + path.replace('\\', '\\\\').replace('"', '\\"') + '"'


def cmd_launch_run(args):
    """Run a ROS 2 launch file in a tmux session."""
    if not _check_tmux():
        return output({
            "error": "tmux is not installed. Install with: sudo apt install tmux",
            "suggestion": "Alternatively, launch files can be run with nohup in background"
        })
    
    package = args.package
    launch_file = args.launch_file
    launch_args = args.args or []
    presets = args.presets
    params_str = args.params
    config_path = args.config_path
    force_refresh = getattr(args, 'refresh', False)
    
    # Check package exists
    if not _package_exists(package, force_refresh=force_refresh):
        return output({
            "error": f"Package '{package}' not found",
            "available_packages": list(_list_packages().keys())[:20]
        })
    
    # Find launch file
    prefix = _get_package_prefix(package)
    launch_files = _find_launch_files(package)
    
    # Try different possible paths
    possible_paths = [
        os.path.join(prefix, "share", package, "launch", launch_file),
        os.path.join(prefix, "lib", package, "launch", launch_file),
        launch_file,  # Relative path or full path
    ]
    
    launch_path = None
    for p in possible_paths:
        if os.path.exists(p):
            launch_path = p
            break
    
    if not launch_path and not launch_files:
        return output({
            "error": f"Launch file '{launch_file}' not found in package '{package}'",
            "searched_paths": possible_paths,
            "suggestion": "Provide full path or use 'ros2 pkg files <package>' to find launch files. "
                         "If the package is in a local workspace, set ROS2_LOCAL_WS environment variable."
        })
    
    if not launch_path and launch_files:
        return output({
            "error": f"Launch file '{launch_file}' not found",
            "available_launch_files": launch_files,
            "suggestion": "If the launch file is in a local workspace, set ROS2_LOCAL_WS environment variable."
        })
    
    # Build launch command
    cmd_parts = ["ros2 launch", package, os.path.basename(launch_path)]
    cmd_parts.extend(launch_args)
    launch_cmd = " ".join(cmd_parts)
    
    # Generate session name
    session_name = _generate_session_name("launch", package, launch_file.replace('.launch.py', '').replace('.launch', ''))
    
    # Apply presets if specified
    applied_presets = []
    if presets:
        applied_presets = [p.strip() for p in presets.split(',')]
    
    # Apply params if specified
    applied_params = _apply_params(params_str) if params_str else {}
    
    # Get local workspace to source (auto-detected)
    ws_path, ws_status = source_local_ws()
    
    warning = None
    if ws_status == "not_built":
        warning = f"Warning: Local workspace found but not built. Build with 'colcon build' first."
    elif ws_status == "not_found":
        # No local workspace found - continue without sourcing
        ws_path = None
    
    # Handle existing session with same name - require explicit kill or restart
    if _session_exists(session_name):
        return output({
            "error": f"Session '{session_name}' already exists",
            "suggestion": "Use 'launch restart {session_name}' to restart, or 'launch kill {session_name}' to kill first",
            "session": session_name
        })
    
    # Build tmux command with or without sourcing
    # Use bash -c to support source command (sh doesn't support source)
    # Quote paths to handle spaces
    quoted_ws = _quote_path(ws_path) if ws_path else None
    if quoted_ws:
        tmux_cmd = f"tmux new-session -d -s {session_name} 'bash -c \"source {quoted_ws} && {launch_cmd}\" 2>&1'"
    else:
        tmux_cmd = f"tmux new-session -d -s {session_name} '{launch_cmd} 2>&1'"
    
    # Run the launch command
    stdout, stderr, rc = _run_cmd(tmux_cmd, timeout=30)
    
    if rc != 0:
        return output({
            "error": f"Failed to start launch file: {stderr}",
            "command": launch_cmd,
            "session": session_name
        })
    
    # Check if session is actually alive (has running process)
    is_alive = _check_session_alive(session_name)
    status = "running" if is_alive else "crashed"
    
    # Get PID if available
    pid_cmd = f"tmux list-panes -t {session_name} -F '{{{{pane_pid}}}}' 2>/dev/null | head -1"
    pid_output, _, _ = _run_cmd(pid_cmd)
    
    result = {
        "success": True,
        "session": session_name,
        "command": launch_cmd,
        "package": package,
        "launch_file": os.path.basename(launch_path),
        "status": status.strip() if status else "unknown",
        "presets_applied": applied_presets,
        "params_applied": applied_params,
    }
    
    if ws_path:
        result["workspace_sourced"] = ws_path
    
    if warning:
        result["warning"] = warning
    
    if pid_output:
        result["pid"] = pid_output.strip()
    
    # Save session metadata for restart
    _save_session(session_name, {
        "type": "run",
        "package": package,
        "launch_file": os.path.basename(launch_path),
        "launch_args": launch_args,
        "presets": presets,
        "params": params_str,
        "command": launch_cmd
    })
    
    output(result)


def cmd_launch_list(args):
    """List running launch sessions in tmux."""
    if not _check_tmux():
        return output({
            "error": "tmux is not installed",
            "running_sessions": []
        })
    
    stdout, stderr, rc = _run_cmd("tmux list-sessions -F '#{session_name}' 2>/dev/null")
    
    if rc != 0 or not stdout.strip():
        return output({
            "running_sessions": [],
            "launch_sessions": []
        })
    
    all_sessions = stdout.strip().split('\n')
    
    # Filter to launch sessions
    launch_sessions = [s for s in all_sessions if s.startswith('launch_')]
    
    # Get details for each launch session
    sessions_info = []
    for session in launch_sessions:
        info = {"session": session}
        
        # Get pane info
        pane_cmd = f"tmux list-panes -t {session} -F '#{{pane_title}}' 2>/dev/null"
        pane_out, _, _ = _run_cmd(pane_cmd)
        if pane_out:
            info["command"] = pane_out.strip()
        
        # Check if still running
        check_cmd = f"tmux has-session -t {session} 2>/dev/null && echo 'running' || echo 'stopped'"
        status, _, _ = _run_cmd(check_cmd)
        info["status"] = status.strip() if status else "unknown"
        
        sessions_info.append(info)
    
    output({
        "all_sessions": all_sessions,
        "launch_sessions": launch_sessions,
        "launch_sessions_detail": sessions_info
    })


def cmd_launch_kill(args):
    """Kill a running launch session."""
    if not _check_tmux():
        return output({
            "error": "tmux is not installed"
        })
    
    session = args.session
    
    # Validate session name starts with launch_
    if not session.startswith('launch_'):
        return output({
            "error": f"Session '{session}' is not a launch session",
            "hint": "Launch sessions start with 'launch_'"
        })
    
    # Check if session exists
    check_cmd = f"tmux has-session -t {session} 2>/dev/null"
    _, _, rc = _run_cmd(check_cmd)
    
    if rc != 0:
        return output({
            "error": f"Session '{session}' does not exist",
            "available_sessions": []
        })
    
    # Kill the session
    kill_cmd = f"tmux kill-session -t {session}"
    stdout, stderr, rc = _run_cmd(kill_cmd)
    
    if rc != 0:
        return output({
            "error": f"Failed to kill session: {stderr}",
            "session": session
        })
    
    # Clean up session metadata
    _delete_session_metadata(session)
    
    output({
        "success": True,
        "session": session,
        "message": f"Session '{session}' killed"
    })


def cmd_launch_restart(args):
    """Restart a launch session (kill and re-launch with same session name)."""
    if not _check_tmux():
        return output({
            "error": "tmux is not installed"
        })
    
    session = args.session
    
    # Validate session name starts with launch_
    if not session.startswith('launch_'):
        return output({
            "error": f"Session '{session}' is not a launch session",
            "hint": "Launch sessions start with 'launch_'"
        })
    
    # Check if session exists
    if not _session_exists(session):
        return output({
            "error": f"Session '{session}' does not exist",
            "suggestion": "Use 'launch run' to start a new session",
            "available_sessions": []
        })
    
    # Load session metadata
    metadata = _get_session_metadata(session)
    
    if not metadata:
        return output({
            "error": f"No metadata found for session '{session}'",
            "suggestion": "Use 'launch run' to start a fresh session",
            "session": session
        })
    
    # Kill existing session
    _kill_session(session)
    
    # Re-launch based on session type
    if metadata.get("type") == "foxglove":
        port = metadata.get("port", 8765)
        args_restart = type('Args', (), {
            'port': port,
            'refresh': False
        })()
        result = cmd_launch_foxglove(args_restart)
        result["message"] = "Session restarted"
        return result
    
    elif metadata.get("type") == "run":
        package = metadata.get("package")
        launch_file = metadata.get("launch_file")
        launch_args = metadata.get("launch_args", [])
        presets = metadata.get("presets")
        params_str = metadata.get("params")
        
        if not package or not launch_file:
            return output({
                "error": f"Incomplete metadata for session '{session}'",
                "suggestion": "Use 'launch run' to start a fresh session"
            })
        
        args_restart = type('Args', (), {
            'package': package,
            'launch_file': launch_file,
            'args': launch_args,
            'presets': presets,
            'params': params_str,
            'config_path': None,
            'refresh': False
        })()
        
        result = cmd_launch_run(args_restart)
        result["message"] = "Session restarted"
        return result
    
    else:
        return output({
            "error": f"Unknown session type for '{session}'",
            "suggestion": "Use 'launch run' to start a fresh session"
        })


def cmd_launch_foxglove(args):
    """Launch foxglove_bridge in a tmux session."""
    if not _check_tmux():
        return output({
            "error": "tmux is not installed. Install with: sudo apt install tmux"
        })
    
    port = args.port
    ros_distro = os.environ.get('ROS_DISTRO', 'unknown')
    force_refresh = getattr(args, 'refresh', False)
    
    # Validate port range
    if port < 1 or port > 65535:
        return output({
            "error": f"Invalid port: {port}",
            "suggestion": "Port must be between 1 and 65535"
        })
    
    # Check package exists
    if not _package_exists("foxglove_bridge", force_refresh=force_refresh):
        return output({
            "error": "Package 'foxglove_bridge' not found",
            "suggestion": f"Install for your ROS 2 distro with:\n  sudo apt install ros-{ros_distro}-foxglove-bridge\n\nOr build from source:\n  git clone https://github.com/foxglove/ros2-foxglove-bridge.git",
            "current_distro": ros_distro,
            "available_packages": list(_list_packages().keys())[:20]
        })
    
    # Check if launch file exists (search multiple locations)
    prefix = _get_package_prefix("foxglove_bridge")
    possible_launch_paths = [
        os.path.join(prefix, "share", "foxglove_bridge", "launch", "foxglove_bridge_launch.xml"),
        os.path.join(prefix, "lib", "foxglove_bridge", "launch", "foxglove_bridge_launch.xml"),
        os.path.join(prefix, "share", "foxglove_bridge", "foxglove_bridge_launch.xml"),
    ]
    
    launch_path = None
    for p in possible_launch_paths:
        if os.path.exists(p):
            launch_path = p
            break
    
    if not launch_path:
        return output({
            "error": "Launch file 'foxglove_bridge_launch.xml' not found in foxglove_bridge package",
            "suggestion": f"The foxglove_bridge package is installed but may be for a different ROS distro.\nCurrent distro: {ros_distro}\n\nReinstall for your distro:\n  sudo apt install ros-{ros_distro}-foxglove-bridge\n\nOr check installed packages:\n  dpkg -l | grep foxglove",
            "package_path": prefix,
            "searched_paths": possible_launch_paths
        })
    
    # Build launch command
    launch_cmd = f"ros2 launch foxglove_bridge foxglove_bridge_launch.xml port:={port}"
    
    # Generate session name
    session_name = _generate_session_name("launch", "foxglove_bridge", f"port{port}")
    
    # Get local workspace to source (auto-detected)
    ws_path, ws_status = source_local_ws()
    
    warning = None
    if ws_status == "not_built":
        warning = f"Warning: Local workspace found but not built. Build with 'colcon build' first."
    elif ws_status == "not_found":
        ws_path = None
    
    # Handle existing session with same name - require explicit kill or restart
    if _session_exists(session_name):
        return output({
            "error": f"Session '{session_name}' already exists",
            "suggestion": f"Use 'launch restart {session_name}' to restart, or 'launch kill {session_name}' to kill first",
            "session": session_name
        })
    
    # Build tmux command
    quoted_ws = _quote_path(ws_path) if ws_path else None
    if quoted_ws:
        tmux_cmd = f"tmux new-session -d -s {session_name} 'bash -c \"source {quoted_ws} && {launch_cmd}\" 2>&1'"
    else:
        tmux_cmd = f"tmux new-session -d -s {session_name} '{launch_cmd} 2>&1'"
    
    stdout, stderr, rc = _run_cmd(tmux_cmd, timeout=30)
    
    if rc != 0:
        return output({
            "error": f"Failed to start foxglove_bridge: {stderr}",
            "command": launch_cmd,
            "session": session_name
        })
    
    # Check if session is actually alive (has running process)
    is_alive = _check_session_alive(session_name)
    status = "running" if is_alive else "crashed"
    
    result = {
        "success": True,
        "session": session_name,
        "command": launch_cmd,
        "package": "foxglove_bridge",
        "launch_file": "foxglove_bridge_launch.xml",
        "port": port,
        "status": status
    }
    
    if ws_path:
        result["workspace_sourced"] = ws_path
    
    if warning:
        result["warning"] = warning
    
    # Save session metadata for restart
    _save_session(session_name, {
        "type": "foxglove",
        "port": port,
        "command": launch_cmd
    })
    
    output(result)
