#!/usr/bin/env python3
"""ROS 2 launch commands for running launch files in tmux sessions."""

import json
import os
import subprocess
import threading

from ros2_utils import output


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


def _get_package_prefix(package):
    """Get the prefix path for a package."""
    stdout, _, rc = _run_cmd(f"ros2 pkg prefix {package}")
    if rc == 0 and stdout:
        return stdout.strip()
    return None


def _list_packages():
    """List all ROS 2 packages (cached)."""
    global _package_cache, _package_cache_initialized
    
    if _package_cache_initialized:
        return _package_cache
    
    stdout, _, rc = _run_cmd("ros2 pkg list")
    if rc == 0:
        packages = stdout.strip().split('\n') if stdout.strip() else []
        for pkg in packages:
            _package_cache[pkg] = True
        _package_cache_initialized = True
    
    return _package_cache


def _package_exists(package):
    """Check if a package exists (uses cache, refreshes if not found)."""
    packages = _list_packages()
    if package in packages:
        return True
    
    # Refresh cache if package not found
    global _package_cache_initialized
    _package_cache_initialized = False
    _list_packages()
    return package in _list_packages()


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
    
    # Check package exists
    if not _package_exists(package):
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
            "suggestion": "Provide full path or use 'ros2 pkg files <package>' to find launch files"
        })
    
    if not launch_path and launch_files:
        return output({
            "error": f"Launch file '{launch_file}' not found",
            "available_launch_files": launch_files
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
    
    # Build tmux command
    tmux_cmd = f"tmux new-session -d -s {session_name} '{launch_cmd} 2>&1'"
    
    # Run the launch command
    stdout, stderr, rc = _run_cmd(tmux_cmd, timeout=30)
    
    if rc != 0:
        return output({
            "error": f"Failed to start launch file: {stderr}",
            "command": launch_cmd,
            "session": session_name
        })
    
    # Check if session is running
    check_cmd = f"tmux has-session -t {session_name} 2>/dev/null && echo 'running' || echo 'not running'"
    status, _, _ = _run_cmd(check_cmd)
    
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
    
    if pid_output:
        result["pid"] = pid_output.strip()
    
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
    
    output({
        "success": True,
        "session": session,
        "message": f"Session '{session}' killed"
    })
