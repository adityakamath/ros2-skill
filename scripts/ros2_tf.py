#!/usr/bin/env python3
"""ROS 2 TF2 transform commands."""

import math
import os
import shlex
import tempfile
import time

from ros2_utils import (
    output,
    ros2_context,
    run_cmd,
    check_tmux,
    generate_session_name,
    session_exists,
    source_local_ws,
    save_session,
    try_fastd,
)


def euler_from_quaternion(x, y, z, w):
    """Convert quaternion to Euler angles (roll, pitch, yaw) in radians.

    Returns:
        tuple: (roll, pitch, yaw) in radians
    """
    # Roll (x-axis rotation)
    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    # Pitch (y-axis rotation)
    sinp = 2 * (w * y - z * x)
    if abs(sinp) >= 1:
        pitch = math.copysign(math.pi / 2, sinp)
    else:
        pitch = math.asin(sinp)

    # Yaw (z-axis rotation)
    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw


def quaternion_from_euler(roll, pitch, yaw):
    """Convert Euler angles (roll, pitch, yaw) to quaternion.

    Returns:
        tuple: (x, y, z, w)
    """
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    w = cr * cp * cy + sr * sp * sy
    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy

    return x, y, z, w


def cmd_tf_list(args):
    """List all coordinate frames."""
    try:
        import rclpy
        from tf2_ros import Buffer, TransformListener
    except ImportError:
        return output({
            "error": "tf2_ros not available",
            "suggestion": "Install with: sudo apt install ros-{distro}-tf2-ros"
        })

    timeout = getattr(args, "timeout", 3.0)

    with ros2_context():
        node = rclpy.node.Node("tf_list")
        tf_buffer = Buffer()
        tf_listener = TransformListener(tf_buffer, node)

        # A single spin_once is not reliable: DDS discovery against every TF
        # broadcaster (robot_state_publisher, controllers, EKF, etc.) can take
        # longer than one short spin, especially on a busy graph (Nav2 + many
        # nodes) — frames get missed non-deterministically. Spin repeatedly
        # for up to `timeout` seconds so the buffer has a real chance to hear
        # from all active broadcasters.
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.2)

        # all_frames_as_yaml returns a YAML string, not a dict
        all_frames_yaml = tf_buffer.all_frames_as_yaml()

        frames = []
        if all_frames_yaml:
            import yaml
            try:
                frames_dict = yaml.safe_load(all_frames_yaml)
                if frames_dict:
                    frames = list(frames_dict.keys())
            except Exception:
                # Fallback: parse line by line
                for line in all_frames_yaml.split('\n'):
                    line = line.strip()
                    if line and ':' in line:
                        frame = line.split(':')[0].strip()
                        if frame and frame not in frames:
                            frames.append(frame)

    output({
        "frames": frames,
        "count": len(frames)
    })


def _available_frames(tf_buffer):
    """Return sorted list of frame names currently in the tf buffer."""
    try:
        import yaml
        raw = tf_buffer.all_frames_as_yaml()
        if not raw:
            return []
        parsed = yaml.safe_load(raw)
        return sorted(parsed.keys()) if parsed else []
    except Exception:
        return []


def cmd_tf_lookup(args):
    """Lookup transform between source and target frames."""
    source = args.source
    target = args.target
    timeout = getattr(args, 'timeout', 5.0)

    # Fast path: persistent daemon, no rclpy.init() for this call at all.
    # The daemon's TF buffer has been listening since it started (likely far
    # longer than this call's own timeout budget), so no spin-and-wait is
    # needed here — just the lookup itself, computed the same way below.
    daemon_resp = try_fastd(
        "tf_lookup", {"source": source, "target": target, "timeout": min(timeout, 2.0)},
        timeout=timeout + 2.0,
    )
    if daemon_resp is not None:
        if "error" in daemon_resp:
            return output({
                "error": f"Transform not found: {daemon_resp['error']}",
                "suggestion": "Run 'tf list' to see all frames in the tf tree.",
            })
        t = daemon_resp["translation"]
        r = daemon_resp["rotation"]
        roll, pitch, yaw = euler_from_quaternion(r["x"], r["y"], r["z"], r["w"])
        return output({
            "source_frame": source,
            "target_frame": target,
            "translation": t,
            "rotation": r,
            "euler": {"roll": roll, "pitch": pitch, "yaw": yaw},
            "euler_degrees": {
                "roll": math.degrees(roll),
                "pitch": math.degrees(pitch),
                "yaw": math.degrees(yaw),
            },
            "timestamp": daemon_resp["timestamp"],
        })

    try:
        import rclpy
        from tf2_ros import Buffer, TransformListener
        import tf2_ros
    except ImportError:
        return output({
            "error": "tf2_ros not available",
            "suggestion": "Install with: sudo apt install ros-{distro}-tf2-ros"
        })

    with ros2_context():
        node = rclpy.node.Node("tf_lookup")
        tf_buffer = Buffer()
        tf_listener = TransformListener(tf_buffer, node)

        # A single spin_once is not reliable: DDS/discovery against every TF
        # broadcaster can take longer than one short spin, especially on a
        # busy graph, so the lookup below can spuriously fail even when the
        # transform genuinely exists (see cmd_tf_list for the same fix).
        # Spin repeatedly, exiting early the moment the transform is available
        # instead of always waiting the full timeout.
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.2)
            if tf_buffer.can_transform(target, source, rclpy.time.Time()):
                break

        try:
            # rclpy.time.Time() (= t=0) requests the latest available
            # transform instead of the literal current wall-clock time,
            # avoiding extrapolation errors when the last received transform
            # is a few milliseconds behind "now" (see cmd_tf_echo).
            transform = tf_buffer.lookup_transform(
                target,
                source,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.5)
            )

            t = transform.transform.translation
            r = transform.transform.rotation

            roll, pitch, yaw = euler_from_quaternion(r.x, r.y, r.z, r.w)

            output({
                "source_frame": source,
                "target_frame": target,
                "translation": {"x": t.x, "y": t.y, "z": t.z},
                "rotation": {"x": r.x, "y": r.y, "z": r.z, "w": r.w},
                "euler": {"roll": roll, "pitch": pitch, "yaw": yaw},
                "euler_degrees": {
                    "roll": math.degrees(roll),
                    "pitch": math.degrees(pitch),
                    "yaw": math.degrees(yaw)
                },
                "timestamp": str(transform.header.stamp)
            })
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException) as e:
            frames = _available_frames(tf_buffer)
            return output({
                "error": f"Transform not found: {e}",
                "available_frames": frames,
                "suggestion": "Run 'tf list' to see all frames in the tf tree."
            })
        except tf2_ros.ExtrapolationException as e:
            return output({"error": f"Transform extrapolation failed: {e}"})
        except Exception as e:
            return output({"error": str(e)})


def cmd_tf_echo(args):
    """Echo transform between source and target frames continuously."""
    source = args.source
    target = args.target
    timeout = getattr(args, 'timeout', 5.0)
    count = 1 if getattr(args, 'once', False) else getattr(args, 'count', 5)

    try:
        import rclpy
        from tf2_ros import Buffer, TransformListener
        import tf2_ros
    except ImportError:
        return output({
            "error": "tf2_ros not available",
            "suggestion": "Install with: sudo apt install ros-{distro}-tf2-ros"
        })

    results = []

    with ros2_context():
        node = rclpy.node.Node("tf_echo")
        tf_buffer = Buffer()
        tf_listener = TransformListener(tf_buffer, node)

        for i in range(count):
            # Spin until the transform is available (bounded by `timeout`),
            # exiting as soon as it's found. A single/no pre-spin is not
            # reliable — DDS discovery against the broadcaster can take
            # longer than one short spin, so lookup_transform's own timeout
            # (which does not spin the node itself) can spuriously miss a
            # transform that genuinely exists (see cmd_tf_list for the same
            # fix). Later iterations reuse a short pace since the buffer is
            # already warm and receiving continuously by then.
            deadline = time.monotonic() + (timeout if i == 0 else 0.2)
            while time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=0.2)
                if tf_buffer.can_transform(target, source, rclpy.time.Time()):
                    break

            try:
                # Use Time(0) to request the latest available transform, avoiding
                # extrapolation errors caused by requesting exactly "now" on a
                # freshly-started buffer that hasn't caught up yet.
                transform = tf_buffer.lookup_transform(
                    target,
                    source,
                    rclpy.time.Time(),
                    timeout=rclpy.duration.Duration(seconds=0.5)
                )

                t = transform.transform.translation
                r = transform.transform.rotation
                roll, pitch, yaw = euler_from_quaternion(r.x, r.y, r.z, r.w)

                results.append({
                    "translation": {"x": round(t.x, 4), "y": round(t.y, 4), "z": round(t.z, 4)},
                    "rotation": {"x": round(r.x, 4), "y": round(r.y, 4), "z": round(r.z, 4), "w": round(r.w, 4)},
                    "euler": {"roll": round(roll, 4), "pitch": round(pitch, 4), "yaw": round(yaw, 4)}
                })
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException) as e:
                frames = _available_frames(tf_buffer)
                results.append({
                    "error": str(e),
                    "available_frames": frames,
                    "suggestion": "Run 'tf list' to see all frames in the tf tree."
                })
            except Exception as e:
                results.append({"error": str(e)})

    output({
        "source_frame": source,
        "target_frame": target,
        "count": len(results),
        "transforms": results
    })


def cmd_tf_monitor(args):
    """Monitor transform updates for a frame."""
    frame = args.frame
    timeout = getattr(args, 'timeout', 5.0)
    count = getattr(args, 'count', 5)
    reference_frame = getattr(args, 'reference_frame', None)

    try:
        import rclpy
        from tf2_ros import Buffer, TransformListener
        import tf2_ros
    except ImportError:
        return output({
            "error": "tf2_ros not available",
            "suggestion": "Install with: sudo apt install ros-{distro}-tf2-ros"
        })

    results = []

    with ros2_context():
        node = rclpy.node.Node("tf_monitor")
        tf_buffer = Buffer()
        tf_listener = TransformListener(tf_buffer, node)

        # Allow the listener to populate before querying. A blind time.sleep()
        # does nothing useful here (the node isn't spinning during it, so no
        # messages can be received), and a single spin_once is not reliable —
        # DDS discovery against the broadcaster can take longer than one short
        # spin (see cmd_tf_list for the same fix). Spin repeatedly, bounded by
        # `timeout`, until frames actually show up.
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.2)
            if tf_buffer.all_frames_as_yaml():
                break

        # Auto-discover reference frame if not specified
        if reference_frame is None:
            try:
                # all_frames_as_yaml() nests each frame's parent/broadcaster/etc.
                # as indented sub-fields — only the top-level keys are actual
                # frame names (see _available_frames, which parses this the
                # same, correct way).
                available = _available_frames(tf_buffer)
                # Prefer common root frame names; fall back to first discovered
                preferred = ['world', 'map', 'odom']
                reference_frame = next(
                    (f for f in preferred if f in available),
                    available[0] if available else None
                )
            except Exception:
                reference_frame = None

            if reference_frame is None:
                return output({
                    "error": "Could not auto-discover a reference frame. "
                             "Use --reference-frame <frame> and run 'tf list' to see available frames."
                })

        for i in range(count):
            # Reference frame already confirmed present above for i == 0;
            # for later iterations the buffer is already warm, so just pace
            # briefly instead of a blind fixed wait.
            if i > 0:
                pace_deadline = time.monotonic() + 0.2
                while time.monotonic() < pace_deadline:
                    rclpy.spin_once(node, timeout_sec=0.2)

            try:
                # rclpy.time.Time() (= t=0) requests the latest available
                # transform instead of the literal current wall-clock time,
                # avoiding extrapolation errors (see cmd_tf_echo).
                transform = tf_buffer.lookup_transform(
                    reference_frame,
                    frame,
                    rclpy.time.Time(),
                    timeout=rclpy.duration.Duration(seconds=0.5)
                )

                t = transform.transform.translation
                r = transform.transform.rotation

                results.append({
                    "timestamp": str(transform.header.stamp),
                    "translation": {"x": round(t.x, 4), "y": round(t.y, 4), "z": round(t.z, 4)},
                    "rotation": {"x": round(r.x, 4), "y": round(r.y, 4), "z": round(r.z, 4), "w": round(r.w, 4)}
                })
            except Exception as e:
                results.append({"error": str(e)})

    output({
        "reference_frame": reference_frame,
        "frame": frame,
        "count": len(results),
        "updates": results
    })


def cmd_tf_static(args):
    """Publish static transform in tmux session."""
    if not check_tmux():
        return output({
            "error": "tmux is not installed",
            "suggestion": "Install with: sudo apt install tmux"
        })

    # Resolve arguments — support both named (--from/--to/--xyz/--rpy) and positional forms
    pos = getattr(args, 'pos_args', [])
    named_from = getattr(args, 'from_frame', None)
    named_to = getattr(args, 'to_frame', None)
    xyz = getattr(args, 'xyz', None)
    rpy = getattr(args, 'rpy', None)

    if named_from or named_to or xyz is not None or rpy is not None:
        # Named form: --from --to --xyz --rpy
        if not named_from or not named_to or xyz is None or rpy is None:
            return output({
                "error": "Named form requires --from, --to, --xyz, and --rpy",
                "usage": "tf static --from base_link --to sensor --xyz 1 2 3 --rpy 0 0 0"
            })
        x, y, z = xyz
        roll, pitch, yaw = rpy
        from_frame = named_from
        to_frame = named_to
    elif len(pos) == 8:
        # Positional form: x y z roll pitch yaw from_frame to_frame
        try:
            x, y, z, roll, pitch, yaw = [float(v) for v in pos[:6]]
        except ValueError as e:
            return output({"error": f"Invalid positional arguments: {e}",
                           "usage": "tf static x y z roll pitch yaw from_frame to_frame"})
        from_frame = pos[6]
        to_frame = pos[7]
    else:
        return output({
            "error": "Invalid arguments for tf static",
            "usage_named": "tf static --from base_link --to sensor --xyz 1 2 3 --rpy 0 0 0",
            "usage_positional": "tf static x y z roll pitch yaw from_frame to_frame"
        })

    # Generate session name. Prefixed via generate_session_name("run", ...) —
    # not "tf_static_..." — so this session is actually manageable through
    # the existing 'run kill'/'run list' machinery (which validates a "run_"
    # prefix and looks up metadata by session name). Previously this session
    # was untracked: no save_session() call, a session name that didn't
    # match any kill command's expected prefix, and the error message below
    # suggesting 'run kill' which would have rejected it — there was no
    # working way to stop a tf_static session other than raw tmux commands.
    session_name = generate_session_name("run", "tf_static", f"{from_frame}_to_{to_frame}")

    if session_exists(session_name):
        return output({
            "error": f"Session '{session_name}' already exists",
            "suggestion": f"Use 'run kill {session_name}' first, or check with 'tmux list'"
        })

        # Build static_transform_publisher command. Pre-existing bug fixed here:
    # the bare executable name is not on PATH after sourcing (confirmed via
    # `which static_transform_publisher` — not found), so this never actually
    # ran; it needs the 'ros2 run tf2_ros' wrapper like any other executable.
    # from_frame/to_frame are free-text user input — shlex.join() quotes
    # every token exactly once, matching the fix in ros2_launch.py/
    # ros2_run.py for the same class of nested-shell-quoting injection (a
    # frame name containing a single quote could otherwise break out of the
    # tmux/bash -c nesting below).
    cmd = shlex.join(["ros2", "run", "tf2_ros", "static_transform_publisher",
                       str(x), str(y), str(z),
                       str(roll), str(pitch), str(yaw), from_frame, to_frame])

    # Get local workspace to source
    ws_path, ws_status = source_local_ws()

    warning = None
    if ws_status == "invalid":
        return output({
            "error": "ROS2_LOCAL_WS is set but path does not exist",
            "suggestion": "Unset ROS2_LOCAL_WS or set a valid path"
        })
    elif ws_status == "not_built":
        warning = "Warning: Local workspace found but not built"
    elif ws_status == "not_found":
        ws_path = None

    # Write to a script file rather than interpolating into nested
    # shell-quoted strings — see ros2_launch.py cmd_launch_run for the
    # confirmed injection this avoids (a frame name containing a single
    # quote could otherwise break out of the tmux/bash -c nesting).
    scripts_dir = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), ".launch_scripts")
    os.makedirs(scripts_dir, exist_ok=True)
    script_lines = ["#!/bin/bash", "set -e", "exec 2>&1"]
    if ws_path:
        script_lines.append(f"source {shlex.quote(ws_path)}")
    script_lines.append(cmd)
    fd, script_path = tempfile.mkstemp(prefix=f"{session_name}_", suffix=".sh", dir=scripts_dir)
    with os.fdopen(fd, "w") as f:
        f.write("\n".join(script_lines) + "\n")
    os.chmod(script_path, 0o700)

    tmux_cmd = f"tmux new-session -d -s {session_name} bash {shlex.quote(script_path)}"

    _, stderr, rc = run_cmd(tmux_cmd, timeout=30)

    if rc != 0:
        return output({
            "error": f"Failed to start static transform: {stderr}",
            "command": cmd,
            "session": session_name
        })

    result = {
        "success": True,
        "session": session_name,
        "command": cmd,
        "from_frame": from_frame,
        "to_frame": to_frame,
        "translation": {"x": x, "y": y, "z": z},
        "rotation_euler": {"roll": roll, "pitch": pitch, "yaw": yaw},
    }

    x_rot, y_rot, z_rot, w_rot = quaternion_from_euler(roll, pitch, yaw)
    result["rotation_quaternion"] = {"x": x_rot, "y": y_rot, "z": z_rot, "w": w_rot}

    if ws_path:
        result["workspace_sourced"] = ws_path

    if warning:
        result["warning"] = warning

    # Save metadata so 'run kill'/'run list' can manage this session like any
    # other — including cleaning up the script file via kill_session_cmd.
    save_session(session_name, {
        "type": "run",
        "package": "tf2_ros",
        "executable": "static_transform_publisher",
        "args": [str(x), str(y), str(z), str(roll), str(pitch), str(yaw), from_frame, to_frame],
        "command": cmd,
        "script_path": script_path,
    })

    output(result)
    return result


def cmd_tf_euler_from_quaternion(args):
    """Convert quaternion to Euler angles (radians)."""
    roll, pitch, yaw = euler_from_quaternion(args.x, args.y, args.z, args.w)
    output({"quaternion": {"x": args.x, "y": args.y, "z": args.z, "w": args.w},
            "euler": {"roll": roll, "pitch": pitch, "yaw": yaw}, "unit": "radians"})


def cmd_tf_quaternion_from_euler(args):
    """Convert Euler angles to quaternion (radians)."""
    x, y, z, w = quaternion_from_euler(args.roll, args.pitch, args.yaw)
    output({"euler": {"roll": args.roll, "pitch": args.pitch, "yaw": args.yaw},
            "quaternion": {"x": x, "y": y, "z": z, "w": w}, "unit": "radians"})


def cmd_tf_euler_from_quaternion_degrees(args):
    """Convert quaternion to Euler angles (degrees)."""
    roll, pitch, yaw = euler_from_quaternion(args.x, args.y, args.z, args.w)
    output({"quaternion": {"x": args.x, "y": args.y, "z": args.z, "w": args.w},
            "euler": {"roll": math.degrees(roll), "pitch": math.degrees(pitch),
                      "yaw": math.degrees(yaw)}, "unit": "degrees"})


def cmd_tf_quaternion_from_euler_degrees(args):
    """Convert Euler angles to quaternion (degrees)."""
    x, y, z, w = quaternion_from_euler(
        math.radians(args.roll), math.radians(args.pitch), math.radians(args.yaw)
    )
    output({"euler": {"roll": args.roll, "pitch": args.pitch, "yaw": args.yaw},
            "quaternion": {"x": x, "y": y, "z": z, "w": w}, "unit": "degrees"})


def _tf_transform_stamped(args, msg_class, data_attr):
    """Transform a stamped geometry message (PointStamped or Vector3Stamped).

    *msg_class* is the stamped message class; *data_attr* is the attribute name
    on both the stamped message and the transformed result ('point' or 'vector').
    """
    target, source = args.target, args.source
    x, y, z = args.x, args.y, args.z
    timeout = getattr(args, 'timeout', 5.0)

    try:
        import rclpy
        from tf2_ros import Buffer, TransformListener
        import tf2_ros
        try:
            import tf2_geometry_msgs  # registers PointStamped/Vector3Stamped type adapters
        except ImportError:
            pass
    except ImportError:
        return output({
            "error": "tf2_ros not available",
            "suggestion": "Install with: sudo apt install ros-{distro}-tf2-ros"
        })

    with ros2_context():
        node = rclpy.node.Node(f"tf_transform_{data_attr}")
        tf_buffer = Buffer()
        TransformListener(tf_buffer, node)
        rclpy.spin_once(node, timeout_sec=1.0)

        try:
            stamped = msg_class()
            stamped.header.frame_id = source
            stamped.header.stamp = rclpy.time.Time().to_msg()  # time=0 → latest available
            field = getattr(stamped, data_attr)
            field.x, field.y, field.z = x, y, z

            transformed = tf_buffer.transform(
                stamped, target, timeout=rclpy.duration.Duration(seconds=timeout)
            )
            out = getattr(transformed, data_attr)
            output({
                "source_frame": source, "target_frame": target,
                "input": {"x": x, "y": y, "z": z},
                "output": {"x": out.x, "y": out.y, "z": out.z},
            })
        except tf2_ros.LookupException as e:
            output({"error": f"Transform not found: {e}"})
        except tf2_ros.ConnectivityException as e:
            output({"error": f"No path between frames: {e}"})
        except tf2_ros.ExtrapolationException as e:
            output({"error": f"Transform extrapolation failed: {e}"})
        except Exception as e:
            output({"error": str(e)})


def cmd_tf_transform_point(args):
    """Transform a point from source to target frame."""
    from geometry_msgs.msg import PointStamped
    _tf_transform_stamped(args, PointStamped, 'point')


def cmd_tf_transform_vector(args):
    """Transform a vector from source to target frame."""
    from geometry_msgs.msg import Vector3Stamped
    _tf_transform_stamped(args, Vector3Stamped, 'vector')


def _collect_tf_pairs(duration):
    """Subscribe to /tf and /tf_static for `duration` seconds and return a set of (parent, child) pairs.

    Returns (pairs_set, import_error_msg).  On ImportError returns (None, error_msg).
    """
    try:
        import rclpy
        from tf2_msgs.msg import TFMessage
    except ImportError:
        return None, "tf2_ros or tf2_msgs not available — install with: sudo apt install ros-{distro}-tf2-ros"

    pairs = set()

    def _tf_callback(msg):
        for transform in msg.transforms:
            parent = transform.header.frame_id
            child = transform.child_frame_id
            if parent and child and parent != child:
                pairs.add((parent, child))

    import time as _time
    with ros2_context():
        node = rclpy.node.Node("tf_tree_collector")
        node.create_subscription(
            TFMessage, "/tf", _tf_callback, 10
        )
        node.create_subscription(
            TFMessage, "/tf_static", _tf_callback, 10
        )
        end = _time.time() + duration
        while _time.time() < end:
            rclpy.spin_once(node, timeout_sec=0.1)

    return pairs, None


def _build_tree_str(children_map, roots, indent="", prefix=""):
    """Recursively build ASCII tree string from a parent→children map."""
    lines = []
    root_list = sorted(roots)
    for i, root in enumerate(root_list):
        is_last = (i == len(root_list) - 1)
        connector = "└── " if is_last else "├── "
        lines.append(indent + connector + root)
        child_indent = indent + ("    " if is_last else "│   ")
        children = sorted(children_map.get(root, []))
        if children:
            sub = _build_tree_str(children_map, children, child_indent)
            if sub:
                lines.append(sub)
    return "\n".join(lines)


def _parse_tf_pairs(pairs):
    """Parse a (parent, child) pair set into (all_frames, children_map, root_frames)."""
    all_children = {child for _, child in pairs}
    all_parents = {parent for parent, _ in pairs}
    all_frames = sorted(all_parents | all_children)
    children_map = {}
    for parent, child in pairs:
        children_map.setdefault(parent, set()).add(child)
    root_frames = sorted(all_parents - all_children) or sorted(all_parents)
    return all_frames, children_map, root_frames


def cmd_tf_tree(args):
    """Collect TF transforms and display them as an ASCII tree."""
    duration = getattr(args, 'duration', 2.0)

    pairs, err = _collect_tf_pairs(duration)
    if err:
        return output({"error": err})

    if not pairs:
        return output({
            "error": f"No TF frames received within {duration}s — is a TF publisher running?"
        })

    all_frames, children_map, root_frames = _parse_tf_pairs(pairs)

    # Build ASCII representation
    root_lines = []
    for root in root_frames:
        root_lines.append(root)
        sub = _build_tree_str(children_map, children_map.get(root, set()))
        if sub:
            root_lines.append(sub)
    tree_str = "\n".join(root_lines)

    output({
        "frames": all_frames,
        "root_frames": root_frames,
        "tree": tree_str,
        "transform_count": len(pairs),
    })


def cmd_tf_validate(args):
    """Validate the TF tree for cycles, multiple parents, and other issues."""
    duration = getattr(args, 'duration', 2.0)

    pairs, err = _collect_tf_pairs(duration)
    if err:
        return output({"error": err})

    if not pairs:
        return output({
            "error": f"No TF frames received within {duration}s — is a TF publisher running?"
        })

    all_frames, children_map, _ = _parse_tf_pairs(pairs)
    parent_map = {}  # child → set of parents
    for parent, child in pairs:
        parent_map.setdefault(child, set()).add(parent)

    issues = []
    warnings = []

    # Check multiple parents
    for child, parents in parent_map.items():
        if len(parents) > 1:
            issues.append(
                f"Frame '{child}' has multiple parents: {', '.join(sorted(parents))}"
            )

    # Check for cycles using DFS
    def _has_cycle(frame, visited, stack):
        visited.add(frame)
        stack.add(frame)
        for child in children_map.get(frame, set()):
            if child not in visited:
                result = _has_cycle(child, visited, stack)
                if result:
                    return result
            elif child in stack:
                return child
        stack.discard(frame)
        return None

    visited = set()
    for frame in all_frames:
        if frame not in visited:
            cycle_node = _has_cycle(frame, visited, set())
            if cycle_node:
                issues.append(f"Cycle detected involving frame '{cycle_node}'")

    # Leaf-node warnings
    for frame in all_frames:
        if frame not in children_map:
            warnings.append(f"Frame '{frame}' has no children — leaf node")

    valid = len(issues) == 0

    result = {
        "valid": valid,
        "frames": all_frames,
        "issues": issues,
    }
    if warnings:
        result["warnings"] = warnings

    output(result)


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
