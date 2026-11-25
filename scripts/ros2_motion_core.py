#!/usr/bin/env python3
"""Closed-loop move helpers shared by the CLI and the fast daemon: velocity clamping, decel-zone maths, slow-move check.

Free of rclpy (node arguments are duck-typed) so a call served by the daemon never imports it.
"""

import copy
import math
import time

_DECEL_DEFAULTS = {
    'a_max_linear':  0.5,    # m/s²   — conservative linear accel fallback
    'a_max_angular': 1.0,    # rad/s² — conservative angular accel fallback
    'v_min_x':       0.125,  # m/s    — fine-control floor, linear x
    'v_min_y':       0.100,  # m/s    — fine-control floor, linear y
    'omega_min':     0.375,  # rad/s  — fine-control floor, angular z
}

# Param keyword groups (searched case-insensitively, first numeric match wins).
_ACCEL_KEYWORDS     = ['max_accel', 'accel_limit', 'decel_limit']
_ANG_ACCEL_KEYWORDS = ['max_ang_accel', 'ang_accel_limit', 'angular_accel_limit']
_MIN_VEL_KEYWORDS   = ['min_vel_x', 'min_vel', 'min_speed', 'speed_limit_min']
_MIN_ANG_VEL_KEYWORDS = ['min_ang_vel', 'min_angular_speed', 'min_angular_vel']


def _fetch_first_param(node, nodes_to_search, keywords, deadline):
    """Scan *nodes_to_search* for any param whose name contains a keyword.

    Returns (float_value, "node_name:param_name") or (None, "defaults").
    Stops as soon as one numeric value is found, or when *deadline* (epoch s) passes.
    """
    import rclpy
    from rcl_interfaces.srv import ListParameters, GetParameters

    for node_name in nodes_to_search:
        if time.time() >= deadline:
            break
        try:
            list_client = node.create_client(ListParameters, f"{node_name}/list_parameters")
            svc_timeout = max(0.1, min(1.0, deadline - time.time()))
            if not list_client.wait_for_service(timeout_sec=svc_timeout):
                continue

            future = list_client.call_async(ListParameters.Request())
            while time.time() < deadline and not future.done():
                rclpy.spin_once(node, timeout_sec=0.05)
            if not future.done():
                future.cancel()
                continue

            result = future.result()
            names = result.result.names if result and result.result else []
            kws_lower = [k.lower() for k in keywords]
            matched = [n for n in names if any(kw in n.lower() for kw in kws_lower)]
            if not matched:
                continue

            get_client = node.create_client(GetParameters, f"{node_name}/get_parameters")
            svc_timeout = max(0.1, min(1.0, deadline - time.time()))
            if not get_client.wait_for_service(timeout_sec=svc_timeout):
                continue

            req = GetParameters.Request()
            req.names = matched[:1]
            future2 = get_client.call_async(req)
            while time.time() < deadline and not future2.done():
                rclpy.spin_once(node, timeout_sec=0.05)
            if not future2.done():
                future2.cancel()
                continue

            values = future2.result().values if future2.result() else []
            if not values:
                continue

            pval = values[0]
            for attr in ('double_value', 'integer_value'):
                raw = getattr(pval, attr, None)
                if raw is not None and float(raw) > 0:
                    return float(raw), f"{node_name}:{matched[0]}"
        except Exception:
            continue

    return None, "defaults"


def compute_decel_params(msg_data, distance, is_rotation, is_degrees, node):
    """Auto-compute the decel zone (slow_zone) and slow_factor for publish-until.

    Uses kinematics:
      linear:   slow_zone = v_cmd² / (2 × a_max);  slow_factor = v_min / v_cmd
      rotation: slow_zone = ω_cmd² / (2 × α_max);  slow_factor = ω_min / ω_cmd

    Accel and min-vel values are fetched from live node params (2 s hard timeout).
    Falls back to _DECEL_DEFAULTS on any failure.

    Args:
        msg_data:    Parsed Twist or TwistStamped dict from the publish-until message.
        distance:    Total move in condition units (metres for linear, radians for rotation).
        is_rotation: True when using --rotate.
        is_degrees:  True when --degrees is set (affects display value in meta only).
        node:        A live ROS2CLI node to use for param service calls.

    Returns:
        (slow_zone, slow_factor, meta_dict)
        slow_zone is in condition units (metres or radians).
        meta_dict: {"auto_computed": True/False, "slow_last": X, "slow_factor": Y,
                    "params_source": "node:param" | "defaults"}
    """
    deadline = time.time() + 2.0  # hard 2-second cap for all param fetches

    # Unwrap TwistStamped → Twist
    twist = msg_data.get('twist', msg_data)
    linear  = twist.get('linear',  {})
    angular = twist.get('angular', {})

    # ------------------------------------------------------------------ rotation
    if is_rotation:
        omega_cmd = abs(angular.get('z', 0.0))
        if omega_cmd < 1e-6:
            return None, 0.25, {"auto_computed": False, "reason": "zero angular velocity"}

        try:
            node_info = node.get_node_names_and_namespaces()
            nodes_list = [f"{ns.rstrip('/')}/{n}" for n, ns in node_info]
        except Exception:
            nodes_list = []

        alpha_val, alpha_src = _fetch_first_param(node, nodes_list, _ANG_ACCEL_KEYWORDS, deadline)
        alpha_max = alpha_val if alpha_val else _DECEL_DEFAULTS['a_max_angular']

        omega_val, omega_src = _fetch_first_param(node, nodes_list, _MIN_ANG_VEL_KEYWORDS, deadline)
        omega_min = omega_val if omega_val else _DECEL_DEFAULTS['omega_min']

        # Kinematic braking distance (radians)
        raw_zone   = omega_cmd ** 2 / (2.0 * alpha_max)
        slow_zone  = max(math.radians(3.0), min(raw_zone, abs(distance) * 0.4))
        slow_factor = max(0.10, min(0.50, omega_min / omega_cmd))

        display_val  = math.degrees(slow_zone) if is_degrees else round(slow_zone, 4)
        params_source = alpha_src if alpha_src != "defaults" else omega_src

        meta = {
            "auto_computed": True,
            "slow_last":   round(display_val, 4),
            "slow_factor": round(slow_factor, 4),
            "params_source": params_source,
        }
        return slow_zone, slow_factor, meta

    # ------------------------------------------------------------------ linear
    vx = abs(linear.get('x', 0.0))
    vy = abs(linear.get('y', 0.0))

    if vx >= vy and vx > 1e-6:
        v_cmd, v_min_default, axis_kws = vx, _DECEL_DEFAULTS['v_min_x'], ['min_vel_x'] + _MIN_VEL_KEYWORDS
    elif vy > 1e-6:
        v_cmd, v_min_default, axis_kws = vy, _DECEL_DEFAULTS['v_min_y'], ['min_vel_y'] + _MIN_VEL_KEYWORDS
    else:
        return None, 0.25, {"auto_computed": False, "reason": "zero linear velocity"}

    try:
        node_info = node.get_node_names_and_namespaces()
        nodes_list = [f"{ns.rstrip('/')}/{n}" for n, ns in node_info]
    except Exception:
        nodes_list = []

    try:
        from ros2_braking import load_braking
        measured = load_braking()
    except Exception:
        measured = None
    if measured:  # a measured coast beats a parameter guess
        accel_val, accel_src = measured["linear_mps2"], "measured braking"
    else:
        accel_val, accel_src = _fetch_first_param(node, nodes_list, _ACCEL_KEYWORDS, deadline)
    a_max = accel_val if accel_val else _DECEL_DEFAULTS['a_max_linear']

    vmin_val, vmin_src = _fetch_first_param(node, nodes_list, axis_kws, deadline)
    v_min = vmin_val if vmin_val else v_min_default

    raw_zone  = v_cmd ** 2 / (2.0 * a_max)
    slow_zone = max(0.05, min(raw_zone, abs(distance) * 0.4))
    slow_factor = max(0.10, min(0.50, v_min / v_cmd))

    params_source = accel_src if accel_src != "defaults" else vmin_src

    meta = {
        "auto_computed": True,
        "slow_last":   round(slow_zone, 4),
        "slow_factor": round(slow_factor, 4),
        "params_source": params_source,
    }
    return slow_zone, slow_factor, meta


def scale_twist_velocity(data, scale):
    """Return a copy of a Twist or TwistStamped dict with all velocity fields multiplied by scale."""
    d = copy.deepcopy(data)
    if 'twist' in d and isinstance(d.get('twist'), dict):
        # TwistStamped: velocity nested under 'twist' key
        for top in ('linear', 'angular'):
            if top in d['twist'] and isinstance(d['twist'][top], dict):
                for ax in ('x', 'y', 'z'):
                    if ax in d['twist'][top] and isinstance(d['twist'][top][ax], (int, float)):
                        d['twist'][top][ax] = d['twist'][top][ax] * scale
    else:
        # Twist: velocity at top level
        for top in ('linear', 'angular'):
            if top in d and isinstance(d[top], dict):
                for ax in ('x', 'y', 'z'):
                    if ax in d[top] and isinstance(d[top][ax], (int, float)):
                        d[top][ax] = d[top][ax] * scale
    return d


def _has_velocity_fields(data):
    """Return True if *data* looks like a Twist or TwistStamped message dict.

    Used to gate the auto-hold zero-velocity burst so it only fires for motion
    payloads and is silently skipped for other message types.
    """
    if not isinstance(data, dict):
        return False
    if 'linear' in data or 'angular' in data:
        return True
    nested = data.get('twist')
    return isinstance(nested, dict) and ('linear' in nested or 'angular' in nested)


def _clamp_velocity(data, max_linear=None, max_ang=None):
    """Return (clamped_data, notices).

    Clips linear x/y/z to ±max_linear and angular z to ±max_ang in a Twist
    or TwistStamped dict.  Non-velocity payloads pass through unchanged.
    Both limits are optional; passing None means "no clamp on that axis".
    """
    if max_linear is None and max_ang is None:
        return data, []
    if not _has_velocity_fields(data):
        return data, []

    d = copy.deepcopy(data)
    notices = []

    def _clamp_axis(container, key, axis, limit, label):
        if container and isinstance(container.get(key), dict):
            val = container[key].get(axis)
            if isinstance(val, (int, float)) and limit is not None:
                clamped = max(-limit, min(limit, val))
                if clamped != val:
                    container[key][axis] = clamped
                    notices.append(
                        f"{label}.{axis}: {val} → {clamped} (clamped to ±{limit})"
                    )

    if 'twist' in d and isinstance(d.get('twist'), dict):
        # TwistStamped — velocity nested under 'twist'
        t = d['twist']
        for ax in ('x', 'y', 'z'):
            _clamp_axis(t, 'linear', ax, max_linear, 'twist.linear')
        _clamp_axis(t, 'angular', 'z', max_ang, 'twist.angular')
    else:
        # Twist — velocity at top level
        for ax in ('x', 'y', 'z'):
            _clamp_axis(d, 'linear', ax, max_linear, 'linear')
        _clamp_axis(d, 'angular', 'z', max_ang, 'angular')

    return d, notices


def _twist_speed(msg_dict):
    """Linear speed norm of a Twist or TwistStamped message dict."""
    body = msg_dict.get("twist") if isinstance(msg_dict.get("twist"), dict) else msg_dict
    lin = body.get("linear", {})
    return math.sqrt(sum(float(lin.get(k, 0.0)) ** 2 for k in ("x", "y", "z")))


def slow_move(distance_m, commanded_mps, duration_s, ramp_s=3.0):
    """Report a move that finished but took over twice the cruise time plus *ramp_s*: a limiter was likely slowing it."""
    if not distance_m or commanded_mps <= 0:
        return None
    expected = distance_m / commanded_mps
    if duration_s <= 2 * expected + ramp_s:
        return None
    return {"commanded_mps": round(commanded_mps, 3), "average_mps": round(distance_m / duration_s, 3),
            "expected_s": round(expected, 1), "duration_s": round(duration_s, 1),
            "hint": "finished but slowly: a limiter (collision monitor) probably cut the speed; `snapshot --only scan` shows what is close"}


def prepare_publish_until(args):
    """Validate a ``publish-until`` request and apply the safety clamps. Returns ``(error_dict, None)`` or ``(None, plan)``.

    Pure: no ROS. Shared by the CLI and the fast daemon client so both enforce the same limits.
    """
    import json

    from ros2_safety import heartbeat_touch, validate_publish

    if not args.topic:
        return {"error": "topic argument is required"}, None
    if args.msg is None:
        return {"error": "msg argument is required"}, None
    if not args.monitor:
        return {"error": "--monitor argument is required"}, None

    rotate_angle = getattr(args, "rotate", None)
    use_degrees = getattr(args, "degrees", False)
    if rotate_angle is not None:
        if args.field:
            return {"error": "--rotate cannot be used with --field"}, None
        if args.euclidean:
            return {"error": "--rotate cannot be used with --euclidean"}, None
        if args.delta or args.above or args.below or args.equals:
            return {"error": "--rotate cannot be used with --delta, --above, --below, or --equals"}, None
        try:
            rotate_angle = float(rotate_angle)
        except (TypeError, ValueError):
            return {"error": "--rotate must be a numeric value"}, None
        if use_degrees:
            rotate_angle = math.radians(rotate_angle)
        if rotate_angle == 0:
            return {"error": "--rotate angle must be non-zero"}, None
        field_paths = ["pose.pose.orientation"]
    else:
        if not args.field:
            return {"error": "--field argument is required (or use --rotate for rotation)"}, None
        field_paths = args.field
    euclidean = getattr(args, "euclidean", False)

    if rotate_angle is None and len(field_paths) > 1 and not euclidean:
        return {"error": "Multiple --field paths require the --euclidean flag"}, None

    active = {k: v for k, v in {"delta": args.delta, "above": args.above, "below": args.below, "equals": args.equals}.items()
              if v is not None}
    if rotate_angle is None and len(active) != 1:
        return {"error": "Specify exactly one of --delta, --above, --below, --equals (or --rotate for rotation)"}, None
    if euclidean and rotate_angle is None and next(iter(active)) != "delta":
        return {"error": "--euclidean requires --delta (threshold is the Euclidean distance from start)"}, None
    operator, threshold = next(iter(active.items())) if active else (None, None)
    if euclidean and operator == "delta" and threshold <= 0:
        return {"error": "--delta must be > 0 when --euclidean is used"}, None

    try:
        msg_data = json.loads(args.msg)
    except (json.JSONDecodeError, TypeError, ValueError) as e:
        return {"error": f"Invalid JSON message: {e}"}, None

    msg_data, clamp_notices = _clamp_velocity(msg_data, getattr(args, "max_vel", None), getattr(args, "max_ang", None))
    ok, safety_action, modified_msg, safety_detail = validate_publish(args.topic, msg_data, args.msg_type or "")
    if not ok:
        violations = safety_detail.get("violations", [])
        return {
            "error": "Publish-until blocked by safety validator",
            "blocked": True,
            "reason": "; ".join(violations) if isinstance(violations, list) else violations,
            "limits": safety_detail.get("limits"),
            "actual": safety_detail.get("actual"),
        }, None
    if safety_action == "cap" and isinstance(modified_msg, dict):
        msg_data = modified_msg
    violations = safety_detail.get("violations", [])
    safety_warning = ("; ".join(violations) if isinstance(violations, list) else violations) if safety_action == "warn" else None

    try:
        heartbeat_touch()
    except Exception:
        pass

    return None, {
        "msg_data": msg_data, "field_paths": field_paths, "euclidean": euclidean, "rotate_angle": rotate_angle,
        "use_degrees": use_degrees, "operator": operator, "threshold": threshold, "clamp_notices": clamp_notices,
        "safety_action": safety_action, "safety_warning": safety_warning, "dry_run": getattr(args, "dry_run", False),
    }


def probe_throttle(node, publish, wait, upstream, samples=15):
    """While still commanding, compare the velocity entering the limiter node with what it passes on.

    *upstream* is the velocity-chain output topic (e.g. /cmd_vel_presafety). The node subscribed to it
    (collision monitor, smoother) is the limiter; its Twist publication is the executed command.
    *node* is the publisher's own node (a fresh node would not know the graph yet), *publish()* sends the command once
    and *wait(seconds)* lets messages arrive (an executor spin in the CLI, a sleep in the daemon). Best effort.
    Returns a dict when the limiter cut the command by more than half, else None.
    """
    from ros2_utils import get_msg_type, msg_to_dict

    subs = []
    try:
        limiter = next(iter(node.get_subscriptions_info_by_topic(upstream)), None)
        if limiter is None:
            return None
        twist_types = {"geometry_msgs/msg/Twist", "geometry_msgs/msg/TwistStamped"}
        downstream = [(t, ty[0]) for t, ty in node.get_publisher_names_and_types_by_node(
            limiter.node_name, limiter.node_namespace) if ty and ty[0] in twist_types and t != upstream]
        up_type = dict(node.get_topic_names_and_types()).get(upstream, [None])[0]
        if not downstream or up_type not in twist_types:
            return None
        down_topic, down_type = downstream[0]
        seen = {}
        for top, ty in ((upstream, up_type), (down_topic, down_type)):
            subs.append(node.create_subscription(get_msg_type(ty), top,
                                                 (lambda m, k=top: seen.__setitem__(k, _twist_speed(msg_to_dict(m)))), 10))
        for _ in range(samples):  # keep commanding until both subscriptions have matched and reported
            publish()
            wait(0.1)
            if len(seen) == 2:
                break
        commanded, executed = seen.get(upstream), seen.get(down_topic)
        if commanded and executed is not None and executed < 0.5 * commanded:
            return {"commanded_mps": round(commanded, 3), "executed_mps": round(executed, 3),
                    "limiter_node": limiter.node_name, "output_topic": down_topic,
                    "hint": "a node between the command and the wheels is cutting the speed "
                            "(collision monitor: obstacle close in the direction of motion)"}
    except Exception as exc:
        from ros2_utils import trace
        trace(f"throttle probe failed: {exc}")
        return None
    finally:
        for sub in subs:
            node.destroy_subscription(sub)
    return None
