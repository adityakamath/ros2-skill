#!/usr/bin/env python3
"""Combined pre-motion preflight check.

Path A motion (see RULES-MOTION.md Rule 3 Step 1) requires two mandatory
live checks before every velocity command: (1) confirm the profile-named
controller is active, and (2) confirm the robot is stationary via odom.
Doing these as two separate CLI invocations pays rclpy.init()/shutdown()
--  i.e. full middleware session/discovery overhead -- twice. This module
runs both checks inside a single rclpy context and a single node spin,
cutting that overhead in half.

Further acceleration: if the persistent fast-daemon (ros2_fastd.py) is
running, both checks are serviced over its socket instead, avoiding
rclpy.init()/shutdown() entirely for this call. This is a pure latency
optimization — if the daemon isn't running, behavior is identical to the
non-daemon path (see ros2_utils.try_fastd).
"""

import math
import time

import rclpy

from ros2_control import _call_cm_service
from ros2_topic import TopicSubscriber
from ros2_utils import ROS2CLI, msg_to_dict, output as _raw_output, ros2_context, resolve_topic_type, try_fastd

_augment = None  # set by cmd_preflight_motion; post-processes the motion result before it is printed


def output(data):
    if _augment and isinstance(data, dict) and "ready_for_motion" in data:
        data = _augment(data)
    _raw_output(data)


def _drop_keys(obj, names):
    """Return *obj* without any dict key in *names* (recursively)."""
    if isinstance(obj, dict):
        return {k: _drop_keys(v, names) for k, v in obj.items() if k not in names}
    if isinstance(obj, list):
        return [_drop_keys(v, names) for v in obj]
    return obj


def robot_state_blockers(estop_engaged, mode_nav2, source, mode_service):
    """Reasons a motion command cannot work, from the robot's e-stop and twist-switch state.

    *estop_engaged* / *mode_nav2* are True/False, or None when unknown. *source* is the velocity
    input the command will use: "teleop" (topic publish) or "nav2" (nav2 go/rotate/missions).
    """
    problems = []
    if estop_engaged:
        problems.append("the emergency stop is engaged (motors are not driven); releasing it is the operator's decision")
    if mode_service and mode_nav2 is not None and (source == "nav2") != mode_nav2:
        want = "true" if source == "nav2" else "false"
        problems.append(
            f"the twist switch is in {'nav2' if mode_nav2 else 'teleop'} mode but this command uses the {source} input, "
            f"so it would be ignored; switch with: services call {mode_service} '{{\"data\": {want}}}'")
    return problems


_BATTERY_CRITICAL_PCT = 20  # same thresholds as `system battery`
_BATTERY_WARNING_PCT = 30


def battery_percent(raw):
    """BatteryState.percentage as 0-100. REP says 0..1, but some drivers publish 0..100; values above 1 are taken as percent."""
    if raw is None:
        return None
    raw = float(raw)
    return raw if raw > 1.0 else raw * 100.0


def battery_blockers(pct):
    """(blockers, warnings) for a battery level in percent (None = unknown)."""
    if pct is None:
        return [], []
    if pct < _BATTERY_CRITICAL_PCT:
        return [f"battery is critical ({pct:.0f}% < {_BATTERY_CRITICAL_PCT}%): charge before moving"], []
    if pct < _BATTERY_WARNING_PCT:
        return [], [f"battery is low ({pct:.0f}% < {_BATTERY_WARNING_PCT}%): keep the move short"]
    return [], []


def _start_battery_read(node, topic):
    """Subscribe now; the returned function waits (up to *timeout*) for the first message and returns the percent."""
    import rclpy as _rclpy
    from sensor_msgs.msg import BatteryState
    from rclpy.qos import qos_profile_sensor_data
    got = []
    sub = node.create_subscription(BatteryState, topic, got.append, qos_profile_sensor_data)

    def finish(timeout=0.5):
        end = time.time() + timeout
        while not got and time.time() < end:
            _rclpy.spin_once(node, timeout_sec=0.05)
        node.destroy_subscription(sub)
        return battery_percent(got[0].percentage) if got and got[0].present is not False else None

    return finish


def _robot_state_augmenter(args):
    """Build the post-processor that adds e-stop/mode state and compacts the result (None if nothing to add)."""
    try:
        from ros2_profile import load_profile_summary
        summary = load_profile_summary(readonly=True) or {}
    except Exception:
        summary = {}
    estop = (summary.get("estop_config") or {})
    estop_svc = estop.get("service_name") if estop.get("service_type") == "std_srvs/srv/SetBool" else None
    mode_svc = (summary.get("velocity_chain") or {}).get("mode_service")
    has_battery = bool((summary.get("hardware_fitted") or {}).get("battery_monitor"))
    verbose = getattr(args, "verbose", False)

    pending = {}

    def begin(node):
        """Called with the core check's node: subscribe now so the reads overlap the controller query and odom wait."""
        try:
            from ros2_robot_state import StateReader
            names = [n for n in (estop_svc, mode_svc) if n]
            pending["reader"] = StateReader(node, names) if names else None
            pending["battery"] = _start_battery_read(node, "/battery_state") if has_battery else None
        except Exception as exc:
            pending["error"] = str(exc)

    def collect():
        """Drain the early subscriptions (the node is still alive)."""
        try:
            if pending.get("reader"):
                pending["states"] = pending["reader"].finish(1.0)
            if pending.get("battery"):
                pending["pct"] = pending["battery"]()
        except Exception as exc:
            pending["error"] = str(exc)

    def augment(data):
        from ros2_utils import trace
        trace('preflight: core done')
        if estop_svc or mode_svc or has_battery:
            states, pct = pending.get("states"), pending.get("pct")
            if states is None and "reader" not in pending:  # core check ran elsewhere (fast daemon): read now
                states = {}
                warm = try_fastd("snapshot", {"only": ["state", "battery"], "services": [n for n in (estop_svc, mode_svc) if n]},
                         timeout=5.0)  # warm data: no rclpy start-up
                if warm is not None and "error" not in warm:
                    states = warm.get("service_state") or {}
                    pct = (warm.get("battery") or {}).get("percent")
                else:
                    try:
                        from ros2_robot_state import read_setbool_states
                        names = [n for n in (estop_svc, mode_svc) if n]
                        with ros2_context():
                            node = ROS2CLI("skill_preflight_state")
                            finish_battery = _start_battery_read(node, "/battery_state") if has_battery else None
                            if names:
                                states = read_setbool_states(node, names, timeout=1.0)
                            if finish_battery:
                                pct = finish_battery()
                            node.destroy_node()
                    except Exception as exc:
                        data["robot_state_error"] = str(exc)
            states = states or {}
            if pending.get("error"):
                data["robot_state_error"] = pending["error"]
            if has_battery:
                data["battery_percent"] = None if pct is None else round(pct, 1)
                bat_block, bat_warn = battery_blockers(pct)
                if bat_warn:
                    data["warnings"] = bat_warn
            estop_state, mode_state = states.get(estop_svc), states.get(mode_svc)
            if estop_svc:
                data["estop_engaged"] = estop_state  # null = never toggled since the node started
            if mode_svc:
                data["drive_mode"] = None if mode_state is None else ("nav2" if mode_state else "teleop")
            blockers = robot_state_blockers(estop_state, mode_state, getattr(args, "source", "teleop"), mode_svc)
            blockers += bat_block if has_battery else []
            if blockers:
                data["ready_for_motion"] = False
                data["blocked_by"] = blockers
        trace('preflight: state reads done')
        if not verbose:
            data = _drop_keys(data, {"covariance"})
            ctrl = data.get("controller")
            if isinstance(ctrl, dict):
                data["controller"] = {k: ctrl[k] for k in ("name", "state", "type") if k in ctrl}
        return data

    augment.begin, augment.collect = begin, collect
    return augment


def _speed_score(msg, threshold):
    twist = msg.get("twist", {}).get("twist", {})
    lin, ang = twist.get("linear", {}) or {}, twist.get("angular", {}) or {}
    return max(math.sqrt(sum(lin.get(k, 0.0) ** 2 for k in "xyz")), abs(ang.get("z", 0.0))) / threshold


def settled_sample(first, fetch, threshold, extra=2):
    """Odometry noise can peak just over the threshold at rest: re-read up to *extra* times and keep the quietest sample.

    Costs nothing when the first sample is already stationary. A moving robot stays above the threshold in every sample.
    """
    best = first
    for _ in range(extra):
        if _speed_score(best, threshold) <= 1.0:
            break
        nxt = fetch()
        if nxt is None:
            break
        if _speed_score(nxt, threshold) < _speed_score(best, threshold):
            best = nxt
    return best


def _finish_stationary_check(target, controller_active, msg, velocity_threshold, ready_key):
    """Shared math for both the motion and joint-command preflight checks
    where a Twist-shaped velocity needs to be read from a message — used
    only by cmd_preflight_motion (joint-command has its own finish step,
    since it reports positions rather than a stationary bool)."""
    twist = msg.get("twist", {}).get("twist", {})
    lin = twist.get("linear", {}) or {}
    ang = twist.get("angular", {}) or {}
    linear_speed = math.sqrt(
        lin.get("x", 0.0) ** 2 + lin.get("y", 0.0) ** 2 + lin.get("z", 0.0) ** 2
    )
    angular_speed_z = abs(ang.get("z", 0.0))
    stationary = linear_speed <= velocity_threshold and angular_speed_z <= velocity_threshold
    return {
        "controller": target,
        "controller_active": controller_active,
        "odom": msg,
        "stationary": stationary,
        "linear_speed": linear_speed,
        "angular_speed_z": angular_speed_z,
        ready_key: controller_active and stationary,
    }


def cmd_preflight_motion(args):
    """Check controller-active state and odom-stationary state in one node.

    Returns a single JSON object combining both results plus a top-level
    ``ready_for_motion`` bool so callers can act on one field instead of
    correlating two separate command outputs.
    """
    global _augment
    from ros2_utils import trace
    trace('preflight: start')
    _augment = _robot_state_augmenter(args)
    controller_manager = args.controller_manager
    timeout = args.timeout
    odom_topic = args.odom_topic
    stationary_timeout = args.stationary_timeout
    velocity_threshold = args.velocity_threshold

    # --- Fast path: persistent daemon, no rclpy.init() at all for this call ---
    daemon_ctrl = try_fastd("list_controllers", {
        "controller_manager": controller_manager, "timeout": timeout,
    })
    if daemon_ctrl is not None:
        if "error" in daemon_ctrl:
            return output(daemon_ctrl)
        controllers = daemon_ctrl.get("controllers", [])
        target = next((c for c in controllers if c.get("name") == args.controller), None)
        controller_active = bool(target) and target.get("state") == "active"

        daemon_odom = try_fastd(
            "subscribe_once", {"topic": odom_topic, "timeout": stationary_timeout, "max_age": 1.0},
            timeout=stationary_timeout + 2.0,
        )
        if daemon_odom is not None:
            if "error" in daemon_odom:
                err = dict(daemon_odom)
                err["controller"] = target
                err["controller_active"] = controller_active
                return output(err)
            def _again():
                time.sleep(0.06)  # the daemon returns its latest odometry message: wait for a newer one
                again = try_fastd("subscribe_once", {"topic": odom_topic, "timeout": 1.0, "max_age": 1.0}, timeout=3.0)
                return None if again is None or "error" in again else again.get("msg")

            msg = settled_sample(daemon_odom["msg"], _again, velocity_threshold)
            return output(_finish_stationary_check(
                target, controller_active, msg,
                velocity_threshold, "ready_for_motion",
            ))
        # Daemon served the controller check but dropped before the odom
        # subscribe (rare — e.g. daemon restarted mid-sequence). Finish just
        # the odom half locally instead of discarding the daemon result.
        return output(_local_subscribe_and_finish(
            target, controller_active, odom_topic, stationary_timeout,
            velocity_threshold, "ready_for_motion",
        ))

    # --- Local fallback: daemon not running ---
    return _local_preflight_motion(args)


def preflight_in_daemon(params, list_controllers, latest_odom):
    """The whole pre-motion check, run inside the fast daemon. Returns the result dict (with ``error`` on failure).

    *list_controllers(params)* and *latest_odom(params)* are the daemon's own handlers (called with its lock held).
    """
    from types import SimpleNamespace
    global _augment
    args = SimpleNamespace(source=params.get("source", "teleop"), verbose=bool(params.get("verbose")))
    controller, odom_topic = params["controller"], params["odom_topic"]
    threshold, stationary_timeout = float(params.get("velocity_threshold", 0.01)), float(params.get("stationary_timeout", 2.0))
    augment = _robot_state_augmenter(args)
    ctrl = list_controllers({"controller_manager": params.get("controller_manager", "/controller_manager"),
                             "timeout": params.get("timeout", 5.0)})
    if "error" in ctrl:
        return ctrl
    target = next((c for c in ctrl.get("controllers", []) if c.get("name") == controller), None)
    active = bool(target) and target.get("state") == "active"
    odom = latest_odom({"topic": odom_topic, "timeout": stationary_timeout, "max_age": 1.0})
    if "error" in odom:
        return dict(odom, controller=target, controller_active=active)

    def again():
        time.sleep(0.06)
        newer = latest_odom({"topic": odom_topic, "timeout": 1.0, "max_age": 1.0})
        return None if "error" in newer else newer.get("msg")

    msg = settled_sample(odom["msg"], again, threshold)
    return augment(_finish_stationary_check(target, active, msg, threshold, "ready_for_motion"))


def _local_subscribe_and_finish(target, controller_active, topic, sub_timeout,
                                 velocity_threshold, ready_key):
    try:
        with ros2_context():
            node = ROS2CLI("skill_preflight_partial")
            msg_type = resolve_topic_type(node, topic, None)
            if not msg_type:
                node.destroy_node()
                return {
                    "error": f"Could not detect message type for topic: {topic}",
                    "controller": target, "controller_active": controller_active,
                }
            subscriber = TopicSubscriber(topic, msg_type)
            if subscriber.sub is None:
                node.destroy_node()
                subscriber.destroy_node()
                return {
                    "error": f"Could not load message type: {msg_type}",
                    "controller": target, "controller_active": controller_active,
                }
            executor = rclpy.executors.SingleThreadedExecutor()
            executor.add_node(subscriber)
            end_time = time.time() + sub_timeout
            msg = None
            while time.time() < end_time:
                executor.spin_once(timeout_sec=0.1)
                with subscriber.lock:
                    if subscriber.messages:
                        msg = subscriber.messages[0]
                        break
            subscriber.destroy_node()
            node.destroy_node()

        if msg is None:
            return {
                "error": f"Timeout after {sub_timeout}s waiting for a message on {topic}",
                "controller": target, "controller_active": controller_active,
            }
        return _finish_stationary_check(target, controller_active, msg, velocity_threshold, ready_key)
    except Exception as e:
        return {"error": str(e), "type": type(e).__name__}


def _local_preflight_motion(args):
    """Original single-process implementation — used when the fast daemon
    isn't running. Behavior is identical to the daemon path above."""
    try:
        with ros2_context():
            node = ROS2CLI("skill_preflight")
            if getattr(_augment, "begin", None):
                _augment.begin(node)

            from controller_manager_msgs.srv import ListControllers
            result, err = _call_cm_service(
                node, ListControllers, args.controller_manager,
                "list_controllers", ListControllers.Request(),
                args.timeout, getattr(args, 'retries', 1),
            )
            if err:
                node.destroy_node()
                return output(err)

            from ros2_utils import trace
            trace('preflight: controllers listed')
            controllers = [msg_to_dict(c) for c in result.controller]
            target = next((c for c in controllers if c.get("name") == args.controller), None)
            controller_active = bool(target) and target.get("state") == "active"

            msg_type = resolve_topic_type(node, args.odom_topic, None)
            if not msg_type:
                node.destroy_node()
                return output({
                    "error": f"Could not detect message type for topic: {args.odom_topic}",
                    "controller": target,
                    "controller_active": controller_active,
                })

            subscriber = TopicSubscriber(args.odom_topic, msg_type)
            if subscriber.sub is None:
                node.destroy_node()
                subscriber.destroy_node()
                return output({
                    "error": f"Could not load message type: {msg_type}",
                    "controller": target,
                    "controller_active": controller_active,
                })

            executor = rclpy.executors.SingleThreadedExecutor()
            executor.add_node(subscriber)

            end_time = time.time() + args.stationary_timeout
            odom_msg = None
            while time.time() < end_time:
                executor.spin_once(timeout_sec=0.1)
                with subscriber.lock:
                    if subscriber.messages:
                        odom_msg = subscriber.messages[0]
                        break

            def _next_odom():
                with subscriber.lock:
                    seen = len(subscriber.messages)
                stop = time.time() + 0.5
                while time.time() < stop:
                    executor.spin_once(timeout_sec=0.05)
                    with subscriber.lock:
                        if len(subscriber.messages) > seen:
                            return subscriber.messages[-1]
                return None

            if odom_msg is not None:
                odom_msg = settled_sample(odom_msg, _next_odom, args.velocity_threshold)

            from ros2_utils import trace
            trace('preflight: odom received')
            if getattr(_augment, "collect", None):
                _augment.collect()
            subscriber.destroy_node()
            node.destroy_node()

        if odom_msg is None:
            return output({
                "error": f"Timeout after {args.stationary_timeout}s waiting for odom message on {args.odom_topic}",
                "controller": target,
                "controller_active": controller_active,
            })

        output(_finish_stationary_check(
            target, controller_active, odom_msg, args.velocity_threshold, "ready_for_motion",
        ))
    except Exception as e:
        output({"error": str(e), "type": type(e).__name__})


def _finish_joint_command(target, controller_active, joint_msg, requested_joints):
    names = joint_msg.get("name", []) or []
    positions = joint_msg.get("position", []) or []
    velocities = joint_msg.get("velocity", []) or []

    joint_positions = {}
    joint_velocities = {}
    for i, n in enumerate(names):
        if requested_joints and n not in requested_joints:
            continue
        if i < len(positions):
            joint_positions[n] = positions[i]
        if i < len(velocities):
            joint_velocities[n] = velocities[i]

    missing = [j for j in requested_joints if j not in joint_positions]

    return {
        "controller": target,
        "controller_active": controller_active,
        "joint_positions": joint_positions,
        "joint_velocities": joint_velocities,
        "missing_joints": missing,
        "ready_for_command": controller_active and not missing,
    }


def cmd_preflight_joint_command(args):
    """Check controller-active state and current joint positions in one node.

    Analogous to cmd_preflight_motion, but for joint-position commands
    (pan-tilt, arm, gripper — any controller in
    profile.summary.joint_command_topics) instead of base velocity.
    """
    controller_manager = args.controller_manager
    timeout = args.timeout
    joint_state_topic = args.joint_state_topic
    state_timeout = args.state_timeout
    requested_joints = [j.strip() for j in (args.joints or "").split(",") if j.strip()]

    # --- Fast path: persistent daemon, no rclpy.init() at all for this call ---
    daemon_ctrl = try_fastd("list_controllers", {
        "controller_manager": controller_manager, "timeout": timeout,
    })
    if daemon_ctrl is not None:
        if "error" in daemon_ctrl:
            return output(daemon_ctrl)
        controllers = daemon_ctrl.get("controllers", [])
        target = next((c for c in controllers if c.get("name") == args.controller), None)
        controller_active = bool(target) and target.get("state") == "active"

        daemon_joint = try_fastd(
            "subscribe_once", {"topic": joint_state_topic, "timeout": state_timeout},
            timeout=state_timeout + 2.0,
        )
        if daemon_joint is not None:
            if "error" in daemon_joint:
                err = dict(daemon_joint)
                err["controller"] = target
                err["controller_active"] = controller_active
                return output(err)
            return output(_finish_joint_command(
                target, controller_active, daemon_joint["msg"], requested_joints,
            ))
        # Daemon dropped before the joint_states subscribe — finish locally.
        return output(_local_subscribe_and_finish_joint(
            target, controller_active, joint_state_topic, state_timeout, requested_joints,
        ))

    # --- Local fallback: daemon not running ---
    return _local_preflight_joint_command(args)


def _local_subscribe_and_finish_joint(target, controller_active, topic, sub_timeout, requested_joints):
    try:
        with ros2_context():
            node = ROS2CLI("skill_preflight_joint_partial")
            msg_type = resolve_topic_type(node, topic, None)
            if not msg_type:
                node.destroy_node()
                return {
                    "error": f"Could not detect message type for topic: {topic}",
                    "controller": target, "controller_active": controller_active,
                }
            subscriber = TopicSubscriber(topic, msg_type)
            if subscriber.sub is None:
                node.destroy_node()
                subscriber.destroy_node()
                return {
                    "error": f"Could not load message type: {msg_type}",
                    "controller": target, "controller_active": controller_active,
                }
            executor = rclpy.executors.SingleThreadedExecutor()
            executor.add_node(subscriber)
            end_time = time.time() + sub_timeout
            msg = None
            while time.time() < end_time:
                executor.spin_once(timeout_sec=0.1)
                with subscriber.lock:
                    if subscriber.messages:
                        msg = subscriber.messages[0]
                        break
            subscriber.destroy_node()
            node.destroy_node()

        if msg is None:
            return {
                "error": f"Timeout after {sub_timeout}s waiting for a message on {topic}",
                "controller": target, "controller_active": controller_active,
            }
        return _finish_joint_command(target, controller_active, msg, requested_joints)
    except Exception as e:
        return {"error": str(e), "type": type(e).__name__}


def _local_preflight_joint_command(args):
    """Original single-process implementation — used when the fast daemon
    isn't running. Behavior is identical to the daemon path above."""
    try:
        with ros2_context():
            node = ROS2CLI("skill_preflight_joint")

            from controller_manager_msgs.srv import ListControllers
            result, err = _call_cm_service(
                node, ListControllers, args.controller_manager,
                "list_controllers", ListControllers.Request(),
                args.timeout, getattr(args, 'retries', 1),
            )
            if err:
                node.destroy_node()
                return output(err)

            controllers = [msg_to_dict(c) for c in result.controller]
            target = next((c for c in controllers if c.get("name") == args.controller), None)
            controller_active = bool(target) and target.get("state") == "active"

            msg_type = resolve_topic_type(node, args.joint_state_topic, None)
            if not msg_type:
                node.destroy_node()
                return output({
                    "error": f"Could not detect message type for topic: {args.joint_state_topic}",
                    "controller": target,
                    "controller_active": controller_active,
                })

            subscriber = TopicSubscriber(args.joint_state_topic, msg_type)
            if subscriber.sub is None:
                node.destroy_node()
                subscriber.destroy_node()
                return output({
                    "error": f"Could not load message type: {msg_type}",
                    "controller": target,
                    "controller_active": controller_active,
                })

            executor = rclpy.executors.SingleThreadedExecutor()
            executor.add_node(subscriber)

            end_time = time.time() + args.state_timeout
            joint_msg = None
            while time.time() < end_time:
                executor.spin_once(timeout_sec=0.1)
                with subscriber.lock:
                    if subscriber.messages:
                        joint_msg = subscriber.messages[0]
                        break

            subscriber.destroy_node()
            node.destroy_node()

        if joint_msg is None:
            return output({
                "error": f"Timeout after {args.state_timeout}s waiting for a message on {args.joint_state_topic}",
                "controller": target,
                "controller_active": controller_active,
            })

        requested_joints = [j.strip() for j in (args.joints or "").split(",") if j.strip()]
        output(_finish_joint_command(target, controller_active, joint_msg, requested_joints))
    except Exception as e:
        output({"error": str(e), "type": type(e).__name__})
