#!/usr/bin/env python3
"""Closed-loop moves and routine stops hosted by the fast daemon (see ros2_fastmotion.py for the safety contract).

A ``_Hub`` is one node with its velocity publishers and odometry subscription created up front and its own executor
thread (the daemon's other subscriptions stop being serviced when added to a spinning executor, so each hub is
self-contained). Hubs are kept for the daemon's life, so later moves start warm.

Halting: the move loop runs only while the client socket is open, the cancel flag is clear and the timeout has not
passed. A ``finally`` always sends the zero burst. If this whole process dies, publishing stops and the base
controller's ``cmd_vel_timeout`` (0.5 s on LeKiwi) ends the motion.
"""

import select
import socket
import threading
import time

from ros2_condition import ConditionTracker
from ros2_motion_core import (_clamp_velocity, _twist_speed, compute_decel_params, probe_throttle,
                              scale_twist_velocity, slow_move)

_VELOCITY_TYPES = {f"geometry_msgs{q}{t}" for q in ("/msg/", "/") for t in ("Twist", "TwistStamped")}
_HUBS = {}
_HUBS_LOCK = threading.Lock()
_MOVE_LOCK = threading.Lock()
_CANCEL = threading.Event()
_COUNTER = [0]


def _odom_speeds(msg):
    twist = msg.get("twist", {}).get("twist", {})
    lin, ang = twist.get("linear", {}) or {}, twist.get("angular", {}) or {}
    return (sum(float(lin.get(k, 0.0)) ** 2 for k in "xyz") ** 0.5, abs(float(ang.get("z", 0.0))))


class _Hub:
    """Publishers for the given velocity topics plus one odometry subscription, spun by a private executor."""

    def __init__(self, velocity, odom_topic, odom_type):
        import rclpy
        from rclpy.node import Node
        from rclpy.qos import QoSProfile, ReliabilityPolicy
        from ros2_utils import get_msg_type, msg_to_dict

        _COUNTER[0] += 1
        self.node = Node(f"skill_fastd_motion_{_COUNTER[0]}")
        self._to_dict = msg_to_dict
        self.classes = {}
        self.pubs = {}
        for topic, mtype in velocity:
            self.classes[topic] = get_msg_type(mtype)
            self.pubs[topic] = self.node.create_publisher(self.classes[topic], topic, 10)
        self.tracker = None
        self.latest = None   # (monotonic time, raw message)
        self.sub = None
        if odom_topic:
            qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)  # matches reliable and best-effort publishers
            self.sub = self.node.create_subscription(get_msg_type(odom_type), odom_topic, self._on_odom, qos)
        self.executor = rclpy.executors.SingleThreadedExecutor()
        self.executor.add_node(self.node)
        threading.Thread(target=self.executor.spin, daemon=True).start()
        end = time.time() + 2.0   # let the publishers and the subscription match before the first command
        while time.time() < end:
            matched = all(p.get_subscription_count() > 0 for p in self.pubs.values()) if self.pubs else True
            if matched and (self.sub is None or self.sub.get_publisher_count() > 0):
                break
            time.sleep(0.02)

    def retire(self):
        """Drop a hub whose peers are gone; its node and executor are released."""
        try:
            self.executor.shutdown()
            self.node.destroy_node()
        except Exception:
            pass

    def _on_odom(self, msg):
        self.latest = (time.monotonic(), msg)
        tracker = self.tracker
        if tracker is not None:
            try:
                tracker.update(self._to_dict(msg))
            except Exception as exc:  # an exception escaping a callback would end the executor's spin
                tracker.field_error = f"monitor failed: {exc}"
                tracker.stop_event.set()

    def publish(self, topic, data):
        from ros2_utils import dict_to_msg
        self.pubs[topic].publish(dict_to_msg(self.classes[topic], data))

    def zero(self, topic=None):
        for name, pub in self.pubs.items():
            if topic is None or name == topic:
                msg = self.classes[name]()
                target = msg.twist if hasattr(msg, "twist") else msg
                for vec in (target.linear, target.angular):
                    vec.x = vec.y = vec.z = 0.0
                if hasattr(msg, "header"):
                    msg.header.stamp = self.node.get_clock().now().to_msg()
                pub.publish(msg)


def _types(node, topic, given):
    if given:
        return given
    for name, types in node.get_topic_names_and_types():
        if name == topic and types:
            return types[0]
    return None


def _hub_stale(hub):
    """True when a kept hub has lost its peers (the robot stack restarted): nothing listens to it or nothing feeds it."""
    try:
        listening = all(p.get_subscription_count() > 0 for p in hub.pubs.values()) if hub.pubs else True
        fed = hub.sub is None or hub.sub.get_publisher_count() > 0
        return not (listening and fed)
    except Exception:
        return True


def _hub(key_velocity, odom_topic, odom_type):
    key = (tuple(sorted(key_velocity)), odom_topic)
    with _HUBS_LOCK:
        hub = _HUBS.get(key)
        if hub is not None and _hub_stale(hub):
            time.sleep(0.3)  # a restarted stack may still be re-announcing itself
            if _hub_stale(hub):
                _HUBS.pop(key, None)
                hub.retire()
                hub = None
        if hub is None:
            hub = _HUBS[key] = _Hub(list(key[0]), odom_topic, odom_type)
        return hub


def _client_gone(conn):
    if conn is None:
        return False
    try:
        ready, _, _ = select.select([conn], [], [], 0)
        return bool(ready) and conn.recv(1, socket.MSG_PEEK | socket.MSG_DONTWAIT) == b""
    except (OSError, ValueError):
        return True


def cancel_motion(_node=None, _params=None):
    """Halt a running move. The move's own `finally` sends the zero burst."""
    running = _MOVE_LOCK.locked()
    if running:
        _CANCEL.set()
    return {"cancelled": running}


def run_move(node, params, conn=None):
    from ros2_safety import validate_publish

    if not _MOVE_LOCK.acquire(blocking=False):
        return {"error": "another move is already running in the fast daemon", "blocked": True}
    try:
        _CANCEL.clear()
        topic, monitor = params["topic"], params["monitor"]
        vtype, otype = _types(node, topic, params.get("msg_type")), _types(node, monitor, params.get("monitor_msg_type"))
        if not vtype or not otype:
            return {"fallback": True}
        msg_data, _notices = _clamp_velocity(params["msg"], None, None)
        ok, action, modified, detail = validate_publish(topic, msg_data, vtype)
        if not ok:
            return {"error": "Publish-until blocked by safety validator", "blocked": True,
                    "reason": detail.get("violations"), "limits": detail.get("limits"), "actual": detail.get("actual")}
        if action == "cap" and isinstance(modified, dict):
            msg_data = modified
        try:
            hub = _hub([(topic, vtype)], monitor, otype)
        except Exception:
            return {"fallback": True}

        rotate, euclidean = params.get("rotate"), bool(params.get("euclidean"))
        stop_event = threading.Event()
        tracker = ConditionTracker(params["fields"] if euclidean else params["fields"][0], "delta" if rotate is None else None,
                                   params.get("threshold"), stop_event, euclidean=euclidean, rotate=rotate)
        distance = abs(rotate) if rotate is not None else float(params["threshold"])
        slow_zone, slow_factor = params.get("slow_zone"), max(0.0, min(1.0, params.get("slow_factor") or 0.25))
        decel_meta = {"auto_computed": False}
        if slow_zone is None:
            zone, factor, decel_meta = compute_decel_params(msg_data, distance=distance, is_rotation=rotate is not None,
                                                            is_degrees=bool(params.get("degrees")), node=None)
            if zone is not None:
                slow_zone, slow_factor = zone, factor

        timeout, interval = min(float(params.get("timeout") or 60.0), 600.0), 1.0 / float(params.get("rate") or 10.0)
        base_msg_data, start, published, cancelled, throttle = msg_data, time.time(), 0, False, None
        hub.tracker = tracker
        try:
            while not stop_event.is_set():
                if _CANCEL.is_set() or _client_gone(conn):
                    cancelled = True
                    break
                if time.time() - start >= timeout:
                    break
                out = base_msg_data
                if slow_zone and slow_zone > 0:
                    with tracker.lock:
                        if rotate is not None:
                            remaining = max(0.0, abs(rotate) - abs(tracker.accumulated_rotation or 0.0))
                        else:
                            remaining = max(0.0, float(params["threshold"]) - (tracker.euclidean_distance or 0.0))
                    if remaining < slow_zone:
                        scale = max(slow_factor, remaining / slow_zone)
                        if scale < 0.999:
                            out = scale_twist_velocity(base_msg_data, scale)
                hub.publish(topic, out)
                published += 1
                stop_event.wait(interval)
            if not cancelled and tracker.end_msg is None and tracker.field_error is None:
                try:
                    from ros2_profile import load_profile_summary
                    upstream = ((load_profile_summary(readonly=True) or {}).get("velocity_chain") or {}).get("output")
                except Exception:
                    upstream = None
                if upstream and upstream != topic:
                    throttle = probe_throttle(hub.node, lambda: hub.publish(topic, base_msg_data), time.sleep, upstream)
        finally:
            for _ in range(4):  # committed: the stop must go out whatever happened above
                try:
                    hub.zero(topic)
                except Exception:
                    pass
                time.sleep(0.05)
            hub.tracker = None

        elapsed = round(time.time() - start, 3)
        if cancelled:
            return {"success": False, "cancelled": True, "error": "interrupted: velocity zeroed", "duration": elapsed}
        with tracker.lock:
            if tracker.field_error:
                return {"error": tracker.field_error}
            met = tracker.end_msg is not None
            if euclidean:
                base = {"topic": topic, "monitor_topic": monitor, "fields": params["fields"], "operator": "euclidean_delta",
                        "threshold": params["threshold"], "start_values": tracker.start_values,
                        "end_values": tracker.current_values, "euclidean_distance": tracker.euclidean_distance}
            else:
                base = {"topic": topic, "monitor_topic": monitor, "operator": "rotate", "target_rotation": rotate,
                        "target_rotation_degrees": _deg(rotate), "actual_rotation": tracker.rotation_delta,
                        "actual_rotation_degrees": None if tracker.rotation_delta is None else _deg(tracker.rotation_delta)}
            base.update(duration=elapsed, published_count=published, source="fast daemon")
            extra = {}
            if params.get("clamp_notices"):
                extra["velocity_clamped"] = params["clamp_notices"]
            if params.get("safety_capped") or action == "cap":
                extra["safety_capped"] = True
            if params.get("safety_warning"):
                extra["safety_warning"] = params["safety_warning"]
            if met:
                slow = slow_move(base.get("euclidean_distance"), _twist_speed(msg_data), elapsed) \
                    if euclidean and base.get("euclidean_distance") else None
                return {**base, **({"slow": slow} if slow else {}), "success": True, "condition_met": True,
                        "decel_zone": decel_meta, "start_msg": _slim(tracker.start_msg), "end_msg": _slim(tracker.end_msg), **extra}
            return {**base, "success": False, "condition_met": False, "decel_zone": decel_meta,
                    "error": f"Timeout after {params.get('timeout')}s: condition not met",
                    **({"limited_downstream": throttle} if throttle else {}), **extra}
    finally:
        _MOVE_LOCK.release()


def _deg(rad):
    import math
    return math.degrees(rad)


def _slim(msg):
    """Odometry without covariance matrices (they dominate the output and carry no signal)."""
    if isinstance(msg, dict):
        return {k: _slim(v) for k, v in msg.items() if k != "covariance"}
    if isinstance(msg, list):
        return [_slim(v) for v in msg]
    return msg


def run_stop(node, params):
    """Routine stop: cancel a running move, zero every velocity topic, then report whether the base is stationary."""
    from ros2_utils import get_msg_type

    cancel_motion()
    velocity = [(name, types[0]) for name, types in node.get_topic_names_and_types()
                if types and types[0] in _VELOCITY_TYPES and get_msg_type(types[0])]
    if not velocity:
        return {"fallback": True}
    odom_topic = params.get("odom_topic")
    odom_type = _types(node, odom_topic, None) if odom_topic else None
    try:
        hub = _hub(velocity, odom_topic, odom_type)
    except Exception:
        return {"fallback": True}
    for _ in range(5):
        hub.zero()
        time.sleep(0.05)
    burst_end = time.monotonic()
    result = {"success": True, "topics": [t for t, _ in velocity], "message": "Emergency stop activated (velocity zeroed only)",
              "source": "fast daemon"}
    if odom_topic and hub.sub is not None:
        threshold, deadline = float(params.get("threshold", 0.01)), time.monotonic() + float(params.get("verify_timeout", 5.0))
        reading = None
        while time.monotonic() < deadline:
            latest = hub.latest
            if latest and latest[0] >= burst_end - 0.05:  # a reading taken during or after the burst, never an old one
                reading = hub._to_dict(latest[1])
                lin, ang = _odom_speeds(reading)
                if lin <= threshold and ang <= threshold:
                    break
            time.sleep(0.03)
        if reading is None:
            result["verify_error"] = f"no fresh message on {odom_topic}"
        else:
            lin, ang = _odom_speeds(reading)
            result.update(verified_stationary=lin <= threshold and ang <= threshold, linear_speed=lin, angular_speed_z=ang)
            pos = (reading.get("pose", {}).get("pose", {}) or {}).get("position")
            if pos:
                result["final_position"] = {"x": round(pos.get("x", 0.0), 4), "y": round(pos.get("y", 0.0), 4)}
    return result
