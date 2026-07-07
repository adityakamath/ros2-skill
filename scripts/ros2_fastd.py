#!/usr/bin/env python3
"""Persistent rclpy node server for the ros2-skill CLI ("fast daemon").

Every normal CLI invocation pays rclpy.init()/shutdown() once per command —
under this environment's RMW (Zenoh), that init/shutdown cycle is where
almost all the latency lives (participant creation + peer scouting), not
the actual ROS 2 call. This server keeps ONE rclpy context alive across many
requests so that cost is paid once (at daemon startup) instead of once per
command.

Communicates over a Unix domain socket with a simple newline-delimited JSON
request/response protocol. Client-side integration (ros2_utils.try_fastd)
fails open: if this daemon isn't running or a request errors, the caller
falls back to its normal per-process rclpy path unchanged. This process is
therefore a pure optimization — killing it or leaving it stopped never
breaks anything, it just removes the speedup.

Scope is intentionally narrow: only data-fetching primitives are served
here — controller-manager ListControllers, "wait for one message on a
topic", generic service calls, GetParameters, and TF lookups. Actuation
(publish, service calls with intentional side effects on the robot) stays
on the normal per-process path so a bug in this daemon can never cause an
unintended robot action — this process can only ever read state.

Connections are handled one thread per accepted socket, but the actual
rclpy work is serialized behind a single lock (see _serve_one) — the
persistent node and its background executor are not safe to touch from
multiple threads at once. Only the surrounding socket I/O gained real
concurrency.

TF lookups use a persistent tf2 Buffer/TransformListener attached to the
long-lived node at startup (see main()), continuously fed by the
background spin thread — so by the time a lookup request arrives, the
buffer has typically already been listening far longer than any per-
request spin-and-wait loop could afford, and the lookup itself needs no
additional waiting.

Topic subscriptions (subscribe_once) follow the same "stay warm" idea:
the first request for a given topic creates a persistent subscriber (see
_PersistentSubscriber) that keeps running for the rest of the daemon's
life; every subsequent request for that same topic returns whatever it
last received instantly, with no wait at all. Unlike the TF buffer (which
accumulates all frames it's ever seen), each per-topic subscriber only
keeps the single most recent message — safe to leave running indefinitely
even on a high-rate topic.
"""

import json
import os
import socket
import sys
import threading
import time

_SKILL_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SOCKET_PATH = os.path.join(_SKILL_ROOT, ".fastd", "fastd.sock")
PIDFILE_PATH = os.path.join(_SKILL_ROOT, ".fastd", "fastd.pid")


def _ensure_dir():
    os.makedirs(os.path.dirname(SOCKET_PATH), exist_ok=True)


def _handle_list_controllers(node, params):
    from controller_manager_msgs.srv import ListControllers
    from ros2_control import _call_cm_service
    from ros2_utils import msg_to_dict

    cm_name = params.get("controller_manager", "/controller_manager")
    timeout = float(params.get("timeout", 5.0))
    result, err = _call_cm_service(
        node, ListControllers, cm_name, "list_controllers",
        ListControllers.Request(), timeout, 1,
    )
    if err:
        return err
    return {"controllers": [msg_to_dict(c) for c in result.controller]}


class _PersistentSubscriber:
    """Long-lived subscription that keeps only the most recent message.

    Unlike TopicSubscriber (ros2_topic.py), which accumulates every message
    into a growing list — correct for a short-lived, single-command
    subscription with a bounded duration, but a real memory leak if kept
    alive for a whole daemon session on a high-rate topic (e.g.
    /joint_states) — this overwrites in place, so it's safe to keep
    running indefinitely. Wraps (rather than subclasses) rclpy.node.Node so
    the module only imports rclpy inside functions, matching the rest of
    this file's lazy-import style.
    """

    def __init__(self, rclpy_node_cls, topic, msg_class, msg_type_str):
        import re
        slug = re.sub(r'[^a-zA-Z0-9]', '_', topic.lstrip('/')).strip('_') or 'topic'
        self.node = rclpy_node_cls(f"skill_fastd_sub_{slug}")
        self.msg_type_str = msg_type_str
        self.lock = threading.Lock()
        self.event = threading.Event()
        self.latest = None
        self.sub = self.node.create_subscription(msg_class, topic, self._callback, 10)

    def _callback(self, msg):
        from ros2_utils import msg_to_dict
        with self.lock:
            self.latest = msg_to_dict(msg)
        self.event.set()


# Persistent per-topic subscription cache, keyed by topic name — see
# _PersistentSubscriber. Populated lazily on first request for each topic;
# every subsequent request for the same topic returns whatever the
# background spin thread has already received, same "already warm" benefit
# as the TF buffer below. Bounded by distinct-topics-ever-queried (typically
# tens on a real robot), not by message volume.
_TOPIC_CACHE = {}
_BG_EXECUTOR = None  # set in main(); needed to attach newly cached subscribers


def _handle_subscribe_once(node, params):
    from rclpy.node import Node
    from ros2_utils import resolve_topic_type, get_msg_type

    topic = params["topic"]
    timeout = float(params.get("timeout", 5.0))
    provided_type = params.get("msg_type")
    # Opt-in: callers that know the topic may legitimately never publish
    # (e.g. an action server's /_action/status with no goals ever sent) want
    # a fast "doesn't exist" error instead of paying the full timeout to find
    # out — 'topics subscribe' deliberately does NOT set this (a topic with
    # zero current publishers but a live subscriber-only graph is normal).
    verify_exists = params.get("verify_exists", False)

    subscriber = _TOPIC_CACHE.get(topic)
    if subscriber is None:
        if verify_exists:
            all_topics = dict(node.get_topic_names_and_types())
            if topic not in all_topics:
                return {"error": f"Topic not found: {topic}", "not_found": True}
        msg_type = resolve_topic_type(node, topic, provided_type)
        if not msg_type:
            return {"error": f"Could not detect message type for topic: {topic}"}
        msg_class = get_msg_type(msg_type)
        if not msg_class:
            return {"error": f"Could not load message type: {msg_type}"}
        subscriber = _PersistentSubscriber(Node, topic, msg_class, msg_type)
        _BG_EXECUTOR.add_node(subscriber.node)
        _TOPIC_CACHE[topic] = subscriber

    # Already have a message from a prior request (or one arrived since) —
    # instant return, no wait at all.
    with subscriber.lock:
        if subscriber.latest is not None:
            return {"msg": subscriber.latest, "msg_type": subscriber.msg_type_str}

    # First-ever message on this topic hasn't arrived yet — wait for the
    # background spin thread's callback to set it (Event.wait blocks
    # efficiently; no polling, no manual spin_once needed here since
    # _BG_EXECUTOR is already spinning this subscription continuously).
    if subscriber.event.wait(timeout=timeout):
        with subscriber.lock:
            return {"msg": subscriber.latest, "msg_type": subscriber.msg_type_str}
    return {"error": f"Timeout after {timeout}s waiting for a message on {topic}"}


def _handle_call_service(node, params):
    from ros2_utils import get_srv_type, dict_to_msg, msg_to_dict, call_ros_service

    service_name = params["service_name"]
    srv_type_str = params.get("srv_type")
    request_data = params.get("request_data", {}) or {}
    timeout = float(params.get("timeout", 5.0))

    if not srv_type_str:
        # Auto-detect, same as ros2_service.py's cmd_services_call fallback.
        for name, types in node.get_service_names_and_types():
            if name == service_name:
                srv_type_str = types[0] if types else None
                break
        if not srv_type_str:
            return {"error": f"Service not found: {service_name}"}

    srv_class = get_srv_type(srv_type_str)
    if srv_class is None:
        return {"error": f"Cannot load service type: {srv_type_str}"}

    request = dict_to_msg(srv_class.Request, request_data)
    result, err = call_ros_service(node, srv_class, service_name, request, timeout)
    if err:
        return err
    return {"result": msg_to_dict(result), "service_type": srv_type_str}


def _handle_get_parameters(node, params):
    from rcl_interfaces.srv import GetParameters
    from ros2_utils import call_ros_service
    from ros2_param import _param_value_to_python

    node_name = params["node_name"]
    param_names = params.get("param_names", [])
    timeout = float(params.get("timeout", 5.0))

    request = GetParameters.Request()
    request.names = param_names
    result, err = call_ros_service(
        node, GetParameters, f"{node_name}/get_parameters", request, timeout,
    )
    if err:
        return err

    values = []
    for pval in result.values:
        values.append(_param_value_to_python(pval) if pval.type != 0 else None)
    return {"names": list(param_names), "values": values}


# Persistent tf2 buffer, set up once in main() — see module docstring.
_TF_BUFFER = None


def _handle_tf_lookup(node, params):
    import rclpy

    if _TF_BUFFER is None:
        return {"error": "TF buffer not initialized"}

    source = params["source"]
    target = params["target"]
    timeout = float(params.get("timeout", 2.0))

    try:
        transform = _TF_BUFFER.lookup_transform(
            target, source, rclpy.time.Time(),
            timeout=rclpy.duration.Duration(seconds=timeout),
        )
    except Exception as e:
        return {"error": str(e), "type": type(e).__name__}

    t = transform.transform.translation
    r = transform.transform.rotation
    return {
        "translation": {"x": t.x, "y": t.y, "z": t.z},
        "rotation": {"x": r.x, "y": r.y, "z": r.z, "w": r.w},
        "timestamp": str(transform.header.stamp),
    }


def _handle_list_nodes(node, params):
    node_info = node.get_node_names_and_namespaces()
    names = [f"{ns.rstrip('/')}/{n}" for n, ns in node_info]
    return {"nodes": names, "count": len(names)}


def _split_node_path(full_name):
    """Split '/namespace/node_name' into (node_name, namespace) — mirrors
    ros2_node.py's helper of the same name; duplicated here rather than
    imported so this module's only ros2_node.py dependency stays confined
    to the tiny handler below, not a whole-module import."""
    s = full_name.lstrip('/')
    if '/' in s:
        idx = s.rindex('/')
        return s[idx + 1:], '/' + s[:idx]
    return s, '/'


def _handle_node_details(node, params):
    full_name = params["node"]
    node_name, namespace = _split_node_path(full_name)
    publishers = node.get_publisher_names_and_types_by_node(node_name, namespace)
    subscribers = node.get_subscriber_names_and_types_by_node(node_name, namespace)
    services = node.get_service_names_and_types_by_node(node_name, namespace)
    result = {
        "node": full_name,
        "publishers": [topic for topic, _ in publishers],
        "subscribers": [topic for topic, _ in subscribers],
        "services": [svc for svc, _ in services],
    }
    try:
        result["action_servers"] = [
            name for name, _ in node.get_action_server_names_and_types_by_node(node_name, namespace)
        ]
        result["action_clients"] = [
            name for name, _ in node.get_action_client_names_and_types_by_node(node_name, namespace)
        ]
    except AttributeError:
        result["action_servers"] = []
        result["action_clients"] = []
    return result


_HANDLERS = {
    "list_controllers": _handle_list_controllers,
    "subscribe_once": _handle_subscribe_once,
    "call_service": _handle_call_service,
    "get_parameters": _handle_get_parameters,
    "tf_lookup": _handle_tf_lookup,
    "list_nodes": _handle_list_nodes,
    "node_details": _handle_node_details,
}


def _serve_one(node, conn, rclpy_lock):
    """Service one connection: read its request, run the handler, reply.

    Runs in its own thread (see main()) so a slow client no longer blocks
    the accept loop from taking the next connection, or blocks another
    in-flight connection's request bytes from being read. The actual rclpy
    work (handler(node, params)) is still serialized via rclpy_lock — the
    persistent node and its background executor are not safe to touch from
    multiple threads simultaneously, so only the socket I/O around it
    gained concurrency here, not the ROS interaction itself.
    """
    try:
        conn.settimeout(30.0)
        data = b""
        while not data.endswith(b"\n"):
            chunk = conn.recv(65536)
            if not chunk:
                break
            data += chunk
        if not data:
            return
        request = json.loads(data.decode("utf-8"))
        op = request.get("op")
        params = request.get("params", {}) or {}
        handler = _HANDLERS.get(op)
        if handler is None:
            response = {"error": f"Unknown fastd op: {op}"}
        else:
            try:
                with rclpy_lock:
                    response = handler(node, params)
            except Exception as e:
                response = {"error": str(e), "type": type(e).__name__}
        conn.sendall((json.dumps(response) + "\n").encode("utf-8"))
    except Exception:
        pass
    finally:
        try:
            conn.close()
        except Exception:
            pass


def main():
    global _TF_BUFFER, _BG_EXECUTOR

    import rclpy
    from rclpy.node import Node
    from tf2_ros import Buffer, TransformListener

    _ensure_dir()
    if os.path.exists(SOCKET_PATH):
        os.remove(SOCKET_PATH)

    rclpy.init()
    node = Node("skill_fastd")

    # Persistent TF buffer/listener — starts accumulating transforms
    # immediately and keeps listening for the daemon's whole lifetime (fed
    # by the background spin thread below), so lookups don't need their own
    # per-request wait loop the way the non-daemon 'tf lookup' CLI path does.
    _TF_BUFFER = Buffer()
    TransformListener(_TF_BUFFER, node)

    # Keep the node spinning continuously in the background so that, even
    # between requests, it stays "warm" on the ROS graph (subscriptions and
    # service clients created per-request piggyback on the same already-
    # established participant instead of each starting cold). Also spins
    # every per-topic subscriber added to _TOPIC_CACHE (see
    # _handle_subscribe_once) — one executor, one thread, for everything
    # persistent in this process.
    bg_executor = rclpy.executors.SingleThreadedExecutor()
    bg_executor.add_node(node)
    _BG_EXECUTOR = bg_executor
    spin_thread = threading.Thread(target=bg_executor.spin, daemon=True)
    spin_thread.start()

    sock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    sock.bind(SOCKET_PATH)
    sock.listen(8)

    with open(PIDFILE_PATH, "w") as f:
        f.write(str(os.getpid()))

    print(f"[ros2-skill fastd] listening on {SOCKET_PATH} (pid {os.getpid()})", file=sys.stderr)

    # One lock shared by every connection thread — see _serve_one's
    # docstring for why the rclpy work itself stays serialized even though
    # connection handling is now concurrent.
    rclpy_lock = threading.Lock()

    try:
        while True:
            conn, _ = sock.accept()
            threading.Thread(
                target=_serve_one, args=(node, conn, rclpy_lock), daemon=True,
            ).start()
    except KeyboardInterrupt:
        pass
    finally:
        try:
            sock.close()
        except Exception:
            pass
        if os.path.exists(SOCKET_PATH):
            os.remove(SOCKET_PATH)
        if os.path.exists(PIDFILE_PATH):
            os.remove(PIDFILE_PATH)
        rclpy.shutdown()


if __name__ == "__main__":
    main()
