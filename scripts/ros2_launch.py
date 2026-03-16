#!/usr/bin/env python3
"""ROS 2 launch commands for running launch files in tmux sessions."""

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


# Cache for launch arguments: {(package, launch_file): [args]}
_launch_args_cache = {}


def _get_launch_arguments(package, launch_file):
    """Get available launch arguments from a launch file.
    
    Uses cache to avoid repeated calls.
    """
    cache_key = (package, launch_file)
    
    if cache_key in _launch_args_cache:
        return _launch_args_cache[cache_key]
    
    # Get the launch file path
    prefix = _get_package_prefix(package)
    if not prefix:
        return []
    
    possible_paths = [
        os.path.join(prefix, "share", package, "launch", launch_file),
        os.path.join(prefix, "lib", package, "launch", launch_file),
        launch_file,
    ]
    
    launch_path = None
    for p in possible_paths:
        if os.path.exists(p):
            launch_path = p
            break
    
    if not launch_path:
        return []
    
    # Call --show-arguments to get available args
    cmd = f"ros2 launch {package} {os.path.basename(launch_path)} --show-arguments"
    stdout, stderr, rc = run_cmd(cmd, timeout=30)
    
    args = []
    if rc == 0 and stdout:
        # Parse output - typically shows args in format "arg_name (default value)"
        for line in stdout.split('\n'):
            line = line.strip()
            if line and not line.startswith(' '):
                # Extract arg name (before space or parenthesis)
                arg = line.split(' ')[0].split('(')[0].strip()
                if arg and arg not in args:
                    args.append(arg)
    
    _launch_args_cache[cache_key] = args
    return args


def _fuzzy_match(query, candidates, threshold=0.5):
    """Fuzzy match query against candidates.
    
    Returns list of (candidate, score) tuples sorted by score.
    """
    if not query or not candidates:
        return []
    
    query_lower = query.lower().replace('_', '').replace('-', '')
    
    matches = []
    for candidate in candidates:
        candidate_lower = candidate.lower().replace('_', '').replace('-', '')
        
        # Exact substring match (highest score)
        if query_lower == candidate_lower:
            matches.append((candidate, 1.0))
        # Substring contains
        elif query_lower in candidate_lower or candidate_lower in query_lower:
            matches.append((candidate, 0.8))
        # Starts with
        elif candidate_lower.startswith(query_lower):
            matches.append((candidate, 0.7))
        # Contains words
        elif any(word in candidate_lower for word in query_lower.split()):
            matches.append((candidate, 0.5))
    
    # Sort by score descending
    matches.sort(key=lambda x: x[1], reverse=True)
    return matches


def _validate_launch_args(user_args, available_args):
    """Validate and resolve user-provided args against real available launch args.

    Rules:
    - Always check available_args (fetched from --show-args) first.
    - Exact match → use as-is.
    - No exact match → fuzzy match against available_args only.
      If a good fuzzy match is found → use the matched name, notify user.
    - No match at all → drop the argument, notify user. Never pass an invented arg.

    Returns:
        validated_args: list of args to actually pass to ros2 launch
        notices: list of human-readable messages about what was changed/dropped
    """
    notices = []
    validated = []

    for arg in user_args:
        # Split "name:=value" or "name=value" or bare "name"
        if ':=' in arg:
            arg_name, arg_value = arg.split(':=', 1)
            fmt = lambda n, v: f"{n}:={v}"
        elif '=' in arg:
            arg_name, arg_value = arg.split('=', 1)
            fmt = lambda n, v: f"{n}={v}"
        else:
            arg_name = arg
            arg_value = None
            fmt = lambda n, v: n

        # 1. Exact match
        if arg_name in available_args:
            validated.append(arg)
            continue

        # 2. Fuzzy match — only against real available_args
        matches = _fuzzy_match(arg_name, available_args)
        if matches and matches[0][1] >= 0.6:
            matched_name = matches[0][0]
            resolved = fmt(matched_name, arg_value)
            validated.append(resolved)
            notices.append(
                f"NOTICE: '{arg_name}' not found — using closest match '{matched_name}' instead. "
                f"Passed as: {resolved}"
            )
        else:
            # 3. No match — drop and notify, do NOT pass through
            available_str = ', '.join(sorted(available_args)) if available_args else 'none'
            notices.append(
                f"NOTICE: Argument '{arg_name}' does not exist in this launch file and no similar "
                f"argument was found. It was NOT passed. Available args: [{available_str}]"
            )

    return validated, notices


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
    
    # Check package exists (auto-refresh if not found)
    if not _package_exists(package, force_refresh=False):
        _list_packages(force_refresh=True)
    if not _package_exists(package, force_refresh=False):
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
    
    # Validate and resolve launch arguments against real --show-args output
    arg_notices = []
    if launch_args:
        available_args = _get_launch_arguments(package, os.path.basename(launch_path))
        if not available_args:
            # Could not fetch args — drop all user args and notify, but still launch
            arg_notices.append(
                f"NOTICE: Could not retrieve launch arguments via --show-args. "
                f"All provided arguments {launch_args} were dropped. "
                f"Launch will proceed without arguments."
            )
            launch_args = []
        else:
            launch_args, arg_notices = _validate_launch_args(launch_args, available_args)
    
    # Build launch command
    cmd_parts = ["ros2 launch", package, os.path.basename(launch_path)]
    cmd_parts.extend(launch_args)
    
    launch_cmd = " ".join(cmd_parts)
    
    # Generate session name
    session_name = generate_session_name("launch", package, launch_file.replace('.launch.py', '').replace('.launch', ''))
    
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
        "launch_args": launch_args,
    }
    
    if ws_path:
        result["workspace_sourced"] = ws_path

    if arg_notices:
        result["arg_notices"] = arg_notices

    if warning:
        result["warning"] = warning
    
    if pid_output:
        result["pid"] = pid_output.strip()
    
    # Save session metadata for restart
    save_session(session_name, {
        "type": "run",
        "package": package,
        "launch_file": os.path.basename(launch_path),
        "launch_args": launch_args,
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
    metadata = get_session_metadata(session)
    
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
        
        if not package or not launch_file:
            return output({
                "error": f"Incomplete metadata for session '{session}'",
                "suggestion": "Use 'launch' to start a fresh session"
            })
        
        args_restart = type('Args', (), {
            'package': package,
            'launch_file': launch_file,
            'args': launch_args,
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
    
    # Validate port range
    if port < 1 or port > 65535:
        return output({
            "error": "Invalid port: {port}",
            "suggestion": "Port must be between 1 and 65535"
        })
    
    # Check package exists (auto-refresh if not found)
    if not _package_exists("foxglove_bridge", force_refresh=False):
        _list_packages(force_refresh=True)
    if not _package_exists("foxglove_bridge", force_refresh=False):
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
    save_session(session_name, {
        "type": "foxglove",
        "port": port,
        "command": launch_cmd
    })
    
    output(result)
    return result
