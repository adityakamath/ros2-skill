#!/usr/bin/env python3
"""Read the current state of SetBool toggle services (e-stop, twist switch, waypoint follow, ...).

ROS 2 service introspection publishes every call on ``<service>/_service_event``. Servers that use
transient-local durability replay their last request/response pair to late joiners, so the last
*successful* ``data`` value is the current state. Works for any server that enables introspection;
``None`` means "no replayed event", not "false".
"""

import time

_REQUEST_RECEIVED = 1
_RESPONSE_SENT = 2


def last_setbool_state(events):
    """Return the ``data`` of the most recent successful SetBool call in *events* (dicts), or None.

    Events are matched request->response by (client_gid, sequence_number); a failed response is ignored.
    """
    pending, state = {}, None
    for ev in events:
        info = ev.get("info", {})
        key = (tuple(info.get("client_gid", [])), info.get("sequence_number"))
        etype = info.get("event_type")
        if etype == _REQUEST_RECEIVED and ev.get("request"):
            pending[key] = bool(ev["request"][0].get("data"))
        elif etype == _RESPONSE_SENT and key in pending and ev.get("response"):
            value = pending.pop(key)
            if ev["response"][0].get("success"):
                state = value
    return state


class StateReader:
    """Subscribe to SetBool services' introspection events now; ``finish()`` returns ``{service: bool|None}``.

    Subscribing early lets the middleware match while the caller does other work, so ``finish()`` rarely waits.
    """

    def __init__(self, node, services):
        from rclpy.duration import Duration
        from rclpy.qos import DurabilityPolicy, LivelinessPolicy, QoSProfile, ReliabilityPolicy
        from std_srvs.srv import SetBool_Event

        from ros2_utils import msg_to_dict

        qos = QoSProfile(depth=2, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL,
                         liveliness=LivelinessPolicy.AUTOMATIC, liveliness_lease_duration=Duration(seconds=1))
        self.node = node
        self.events = {s: [] for s in services}
        self.subs = [node.create_subscription(SetBool_Event, s.rstrip("/") + "/_service_event",
                                              (lambda m, s=s: self.events[s].append(msg_to_dict(m))), qos)
                     for s in services]

    def finish(self, timeout=2.0):
        import rclpy

        end = time.time() + timeout
        try:
            while time.time() < end:
                rclpy.spin_once(self.node, timeout_sec=0.05)
                if all(len(v) >= 2 for v in self.events.values()):
                    break
        finally:
            for sub in self.subs:
                self.node.destroy_subscription(sub)
        return {s: last_setbool_state(evs) for s, evs in self.events.items()}


def read_setbool_states(node, services, timeout=2.0):
    """Return ``{service: bool|None}`` for each SetBool service, reading replayed introspection events.

    *node* is a live rclpy node. A service with no introspection topic or no replayed events maps to None.
    """
    return StateReader(node, services).finish(timeout)


def cmd_services_state(args):
    from ros2_utils import ROS2CLI, output, ros2_context

    try:
        with ros2_context():
            node = ROS2CLI()
            states = read_setbool_states(node, args.services, args.timeout)
        output({"states": states, "note": "null means no replayed introspection event (unknown), not false"})
    except Exception as e:
        output({"error": str(e)})
