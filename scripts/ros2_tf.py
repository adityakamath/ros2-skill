#!/usr/bin/env python3
"""ROS 2 TF2 transform commands."""

import math

from ros2_utils import (
    output,
    ros2_context,
    run_cmd,
    check_tmux,
    generate_session_name,
    session_exists,
    quote_path,
    source_local_ws,
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

    with ros2_context():
        node = rclpy.node.Node("tf_list")
        tf_buffer = Buffer()
        tf_listener = TransformListener(tf_buffer, node)
        rclpy.spin_once(node, timeout_sec=0.5)

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

    try:
        import rclpy
        from tf2_ros import Buffer, TransformListener
        import tf2_ros
    except ImportError:
        return output({
            "error": "tf2_ros not available",
            "suggestion": "Install with: sudo apt install ros-{distro}-tf2-ros"
        })

    timeout = getattr(args, 'timeout', 5.0)

    with ros2_context():
        node = rclpy.node.Node("tf_lookup")
        tf_buffer = Buffer()
        tf_listener = TransformListener(tf_buffer, node)
        rclpy.spin_once(node, timeout_sec=0.5)

        try:
            now = node.get_clock().now()
            transform = tf_buffer.lookup_transform(
                target,
                source,
                now,
                timeout=rclpy.duration.Duration(seconds=timeout)
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
            try:
                # Use Time(0) to request the latest available transform, avoiding
                # extrapolation errors caused by requesting exactly "now" on a
                # freshly-started buffer that hasn't caught up yet.
                transform = tf_buffer.lookup_transform(
                    target,
                    source,
                    rclpy.time.Time(),
                    timeout=rclpy.duration.Duration(seconds=timeout)
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

            rclpy.spin_once(node, timeout_sec=0.5)

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

        for i in range(count):
            try:
                now = node.get_clock().now()
                transform = tf_buffer.lookup_transform(
                    "world",
                    frame,
                    now,
                    timeout=rclpy.duration.Duration(seconds=timeout)
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

            rclpy.spin_once(node, timeout_sec=0.5)

    output({
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

    # Generate session name
    session_name = f"tf_static_{from_frame}_to_{to_frame}"[:50]
    session_name = "".join(c for c in session_name if c.isalnum() or c in '_-')

    if session_exists(session_name):
        return output({
            "error": f"Session '{session_name}' already exists",
            "suggestion": f"Use 'run kill {session_name}' first, or check with 'tmux list'"
        })

    # Build static_transform_publisher command
    cmd = f"static_transform_publisher {x} {y} {z} {roll} {pitch} {yaw} {from_frame} {to_frame}"

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

    # Build tmux command
    quoted_ws = quote_path(ws_path) if ws_path else None
    if quoted_ws:
        tmux_cmd = f"tmux new-session -d -s {session_name} 'bash -c \"source {quoted_ws} && {cmd}\" 2>&1'"
    else:
        tmux_cmd = f"tmux new-session -d -s {session_name} '{cmd} 2>&1'"

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

    output(result)
    return result


def cmd_tf_euler_from_quaternion(args):
    """Convert quaternion to Euler angles (radians)."""
    x = args.x
    y = args.y
    z = args.z
    w = args.w

    roll, pitch, yaw = euler_from_quaternion(x, y, z, w)

    output({
        "quaternion": {"x": x, "y": y, "z": z, "w": w},
        "euler": {"roll": roll, "pitch": pitch, "yaw": yaw},
        "unit": "radians"
    })


def cmd_tf_quaternion_from_euler(args):
    """Convert Euler angles to quaternion (radians)."""
    roll = args.roll
    pitch = args.pitch
    yaw = args.yaw

    x, y, z, w = quaternion_from_euler(roll, pitch, yaw)

    output({
        "euler": {"roll": roll, "pitch": pitch, "yaw": yaw},
        "quaternion": {"x": x, "y": y, "z": z, "w": w},
        "unit": "radians"
    })


def cmd_tf_euler_from_quaternion_degrees(args):
    """Convert quaternion to Euler angles (degrees)."""
    x = args.x
    y = args.y
    z = args.z
    w = args.w

    roll, pitch, yaw = euler_from_quaternion(x, y, z, w)

    output({
        "quaternion": {"x": x, "y": y, "z": z, "w": w},
        "euler": {
            "roll": math.degrees(roll),
            "pitch": math.degrees(pitch),
            "yaw": math.degrees(yaw)
        },
        "unit": "degrees"
    })


def cmd_tf_quaternion_from_euler_degrees(args):
    """Convert Euler angles to quaternion (degrees)."""
    roll = math.radians(args.roll)
    pitch = math.radians(args.pitch)
    yaw = math.radians(args.yaw)

    x, y, z, w = quaternion_from_euler(roll, pitch, yaw)

    output({
        "euler": {"roll": args.roll, "pitch": args.pitch, "yaw": args.yaw},
        "quaternion": {"x": x, "y": y, "z": z, "w": w},
        "unit": "degrees"
    })


def cmd_tf_transform_point(args):
    """Transform a point from source to target frame."""
    target = args.target
    source = args.source
    x = args.x
    y = args.y
    z = args.z

    try:
        import rclpy
        from tf2_ros import Buffer, TransformListener
        from geometry_msgs.msg import PointStamped
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

    timeout = getattr(args, 'timeout', 5.0)

    with ros2_context():
        node = rclpy.node.Node("tf_transform_point")
        tf_buffer = Buffer()
        tf_listener = TransformListener(tf_buffer, node)
        rclpy.spin_once(node, timeout_sec=1.0)

        try:
            point = PointStamped()
            point.header.frame_id = source
            point.header.stamp = rclpy.time.Time().to_msg()  # time=0 → latest available
            point.point.x = x
            point.point.y = y
            point.point.z = z

            transformed = tf_buffer.transform(
                point,
                target,
                timeout=rclpy.duration.Duration(seconds=timeout)
            )

            output({
                "source_frame": source,
                "target_frame": target,
                "input": {"x": x, "y": y, "z": z},
                "output": {
                    "x": transformed.point.x,
                    "y": transformed.point.y,
                    "z": transformed.point.z
                }
            })
        except tf2_ros.LookupException as e:
            return output({"error": f"Transform not found: {e}"})
        except tf2_ros.ConnectivityException as e:
            return output({"error": f"No path between frames: {e}"})
        except tf2_ros.ExtrapolationException as e:
            return output({"error": f"Transform extrapolation failed: {e}"})
        except Exception as e:
            return output({"error": str(e)})


def cmd_tf_transform_vector(args):
    """Transform a vector from source to target frame."""
    target = args.target
    source = args.source
    x = args.x
    y = args.y
    z = args.z

    try:
        import rclpy
        from tf2_ros import Buffer, TransformListener
        from geometry_msgs.msg import Vector3Stamped
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

    timeout = getattr(args, 'timeout', 5.0)

    with ros2_context():
        node = rclpy.node.Node("tf_transform_vector")
        tf_buffer = Buffer()
        tf_listener = TransformListener(tf_buffer, node)
        rclpy.spin_once(node, timeout_sec=1.0)

        try:
            vector = Vector3Stamped()
            vector.header.frame_id = source
            vector.header.stamp = rclpy.time.Time().to_msg()  # time=0 → latest available
            vector.vector.x = x
            vector.vector.y = y
            vector.vector.z = z

            transformed = tf_buffer.transform(
                vector,
                target,
                timeout=rclpy.duration.Duration(seconds=timeout)
            )

            output({
                "source_frame": source,
                "target_frame": target,
                "input": {"x": x, "y": y, "z": z},
                "output": {
                    "x": transformed.vector.x,
                    "y": transformed.vector.y,
                    "z": transformed.vector.z
                }
            })
        except tf2_ros.LookupException as e:
            return output({"error": f"Transform not found: {e}"})
        except tf2_ros.ConnectivityException as e:
            return output({"error": f"No path between frames: {e}"})
        except tf2_ros.ExtrapolationException as e:
            return output({"error": f"Transform extrapolation failed: {e}"})
        except Exception as e:
            return output({"error": str(e)})


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
