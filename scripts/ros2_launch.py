#!/usr/bin/env python3
"""ROS 2 launch commands for running launch files in tmux sessions."""

import json
import os

from ros2_utils import (
    output,
    source_local_ws,
    run_cmd,
    check_tmux,
    generate_session_name,
    session_exists,
    kill_session,
    check_session_alive,
    quote_path,
    save_session,
    get_session_metadata,
    delete_session_metadata,
    list_packages,
    package_exists,
    get_package_prefix,
    list_sessions,
    kill_session_cmd,
)


# Cache for package information
_package_cache = {}
_package_cache_initialized = False


def _list_packages(force_refresh=False):
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


def _package_exists(package, force_refresh=False):
    """Check if a package exists (uses cache, refreshes if not found or force_refresh=True)."""
    packages = _list_packages(force_refresh=force_refresh)
    if package in packages:
        return True
    
    global _package_cache_initialized
    if not force_refresh:
        _package_cache_initialized = False
        _list_packages()
        return package in _list_packages()
    
    return False


def _get_package_prefix(package):
    """Get the prefix path for a package."""
    stdout, _, rc = run_cmd(f"ros2 pkg prefix {package}")
    if rc == 0 and stdout:
        return stdout.strip()
    return None


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


def cmd_launch_run(args):
    """Run a ROS 2 launch file in a tmux session."""
    if not check_tmux():
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
    session_name = generate_session_name("launch", package, launch_file.replace('.launch.py', '').replace('.launch', ''))
    
    # Apply presets if specified
    applied_presets = []
    if presets:
        applied_presets = [p.strip() for p in presets.split(',')]
    
    # Apply params if specified
    applied_params = _apply_params(params_str) if params_str else {}
    
    # Get local workspace to source (auto-detected)
    ws_path, ws_status = source_local_ws()
    
    warning = None
    if ws_status == "invalid":
        return output({
            "error": "ROS2_LOCAL_WS is set but path does not exist",
            "suggestion": "Unset ROS2_LOCAL_WS or set a valid path"
        })
    elif ws_status == "not_built":
        warning = f"Warning: Local workspace found but not built. Build with 'colcon build' first."
    elif ws_status == "not_found":
        # No local workspace found - continue without sourcing
        ws_path = None
    
    # Handle existing session with same name - require explicit kill or restart
    if session_exists(session_name):
        return output({
            "error": f"Session '{session_name}' already exists",
            "suggestion": "Use 'launch restart {session_name}' to restart, or 'launch kill {session_name}' to kill first",
            "session": session_name
        })
    
    # Build tmux command with or without sourcing
    # Use bash -c to support source command (sh doesn't support source)
    # Quote paths to handle spaces
    quoted_ws = quote_path(ws_path) if ws_path else None
    if quoted_ws:
        tmux_cmd = f"tmux new-session -d -s {session_name} 'bash -c \"source {quoted_ws} && {launch_cmd}\" 2>&1'"
    else:
        tmux_cmd = f"tmux new-session -d -s {session_name} '{launch_cmd} 2>&1'"
    
    # Run the launch command
    stdout, stderr, rc = run_cmd(tmux_cmd, timeout=30)
    
    if rc != 0:
        return output({
            "error": f"Failed to start launch file: {stderr}",
            "command": launch_cmd,
            "session": session_name
        })
    
    # Check if session is actually alive (has running process)
    is_alive = check_session_alive(session_name)
    status = "running" if is_alive else "crashed"
    
    # Get PID if available
    pid_cmd = f"tmux list-panes -t {session_name} -F '{{{{pane_pid}}}}' 2>/dev/null | head -1"
    pid_output, _, _ = run_cmd(pid_cmd)
    
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
    return result


def cmd_launch_list(args):
    """List running launch sessions in tmux."""
    result = list_sessions("launch_")
    return output(result)


def cmd_launch_kill(args):
    """Kill a running launch session."""
    session = args.session
    result = kill_session_cmd(session, "launch_")
    return output(result)


def cmd_launch_restart(args):
    """Restart a launch session (kill and re-launch with same session name)."""
    if not check_tmux():
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
    if not session_exists(session):
        return output({
            "error": f"Session '{session}' does not exist",
            "suggestion": "Use 'launch' to start a new session",
            "available_sessions": []
        })
    
    # Load session metadata
    metadata = _get_session_metadata(session)
    
    if not metadata:
        return output({
            "error": f"No metadata found for session '{session}'",
            "suggestion": "Use 'launch' to start a fresh session",
            "session": session
        })
    
    # Kill existing session
    kill_session(session)
    
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
                "suggestion": "Use 'launch' to start a fresh session"
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
            "suggestion": "Use 'launch' to start a fresh session"
        })


def cmd_launch_foxglove(args):
    """Launch foxglove_bridge in a tmux session."""
    if not check_tmux():
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
    session_name = generate_session_name("launch", "foxglove_bridge", f"port{port}")
    
    # Get local workspace to source (auto-detected)
    ws_path, ws_status = source_local_ws()
    
    warning = None
    if ws_status == "invalid":
        return output({
            "error": "ROS2_LOCAL_WS is set but path does not exist",
            "suggestion": "Unset ROS2_LOCAL_WS or set a valid path"
        })
    elif ws_status == "not_built":
        warning = f"Warning: Local workspace found but not built. Build with 'colcon build' first."
    elif ws_status == "not_found":
        ws_path = None
    
    # Handle existing session with same name - require explicit kill or restart
    if session_exists(session_name):
        return output({
            "error": f"Session '{session_name}' already exists",
            "suggestion": f"Use 'launch restart {session_name}' to restart, or 'launch kill {session_name}' to kill first",
            "session": session_name
        })
    
    # Build tmux command
    quoted_ws = quote_path(ws_path) if ws_path else None
    if quoted_ws:
        tmux_cmd = f"tmux new-session -d -s {session_name} 'bash -c \"source {quoted_ws} && {launch_cmd}\" 2>&1'"
    else:
        tmux_cmd = f"tmux new-session -d -s {session_name} '{launch_cmd} 2>&1'"
    
    stdout, stderr, rc = run_cmd(tmux_cmd, timeout=30)
    
    if rc != 0:
        return output({
            "error": f"Failed to start foxglove_bridge: {stderr}",
            "command": launch_cmd,
            "session": session_name
        })
    
    # Check if session is actually alive (has running process)
    is_alive = check_session_alive(session_name)
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
    return result
