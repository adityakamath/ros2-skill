#!/usr/bin/env python3
"""ROS 2 lifecycle (managed node) commands."""

from ros2_utils import ROS2CLI, call_ros_service, output, ros2_context, try_fastd


def _get_managed_nodes(rclpy_node):
    """Return sorted list of managed (lifecycle) node names from the service graph."""
    service_info = rclpy_node.get_service_names_and_types()
    return sorted([
        svc[:-len('/get_state')]
        for svc, types in service_info
        if 'lifecycle_msgs/srv/GetState' in types
        and svc.endswith('/get_state')
        and len(svc) > len('/get_state')
    ])


def cmd_lifecycle_nodes(args):
    """List all managed (lifecycle) nodes by scanning for /get_state services."""
    try:
        with ros2_context():
            node = ROS2CLI()
            managed_nodes = _get_managed_nodes(node)
        output({"managed_nodes": managed_nodes, "count": len(managed_nodes)})
    except Exception as e:
        output({"error": str(e)})


def _lifecycle_query_node(rclpy_node, node_name, timeout, retries=1):
    """Query available states and transitions for one lifecycle node."""
    from lifecycle_msgs.srv import GetAvailableStates, GetAvailableTransitions

    if not node_name.startswith('/'):
        node_name = '/' + node_name

    result = {"node": node_name}

    states_result, err = call_ros_service(
        rclpy_node, GetAvailableStates, f"{node_name}/get_available_states",
        GetAvailableStates.Request(), timeout, retries,
        unavailable_hint="Is it a managed node?",
    )
    if err:
        return {"node": node_name, "error": err["error"]}
    result["available_states"] = [
        {"id": s.id, "label": s.label}
        for s in states_result.available_states
    ]

    trans_result, err = call_ros_service(
        rclpy_node, GetAvailableTransitions, f"{node_name}/get_available_transitions",
        GetAvailableTransitions.Request(), timeout, retries,
    )
    if err:
        result["available_transitions"] = []
        result["warning"] = err["error"]
        return result
    result["available_transitions"] = [
        {
            "id": td.transition.id,
            "label": td.transition.label,
            "start_state": {"id": td.start_state.id, "label": td.start_state.label},
            "goal_state": {"id": td.goal_state.id, "label": td.goal_state.label},
        }
        for td in trans_result.available_transitions
    ]

    return result


def cmd_lifecycle_list(args):
    """List available states and transitions for one or all lifecycle nodes."""
    retries = getattr(args, 'retries', 1)
    try:
        with ros2_context():
            node = ROS2CLI()
            if args.node:
                node_name = args.node if args.node.startswith('/') else '/' + args.node
                info = _lifecycle_query_node(node, node_name, args.timeout, retries)
            else:
                managed_nodes = _get_managed_nodes(node)
                info = {"nodes": [_lifecycle_query_node(node, mn, args.timeout, retries)
                                  for mn in managed_nodes]}
        output(info)
    except Exception as e:
        output({"error": str(e)})


def cmd_lifecycle_get(args):
    """Get the current lifecycle state of a managed node."""
    retries = getattr(args, 'retries', 1)
    node_name = args.node if args.node.startswith('/') else '/' + args.node

    # Fast path: persistent daemon, no rclpy.init() for this call at all.
    # GetState is a plain service call — same generic "call_service" daemon
    # op already used by ros2_service.py's cmd_services_call.
    if retries == 1:
        daemon_resp = try_fastd("call_service", {
            "service_name": f"{node_name}/get_state",
            "srv_type": "lifecycle_msgs/srv/GetState",
            "request_data": {},
            "timeout": args.timeout,
        }, timeout=args.timeout + 2.0)
        if daemon_resp is not None:
            if "error" in daemon_resp:
                return output(daemon_resp)
            state = daemon_resp["result"]["current_state"]
            return output({"node": node_name, "state_id": state["id"], "state_label": state["label"]})

    try:
        from lifecycle_msgs.srv import GetState
        with ros2_context():
            node = ROS2CLI()
            result, err = call_ros_service(
                node, GetState, f"{node_name}/get_state", GetState.Request(),
                args.timeout, retries, unavailable_hint="Is it a managed node?",
            )
        if err:
            return output(err)
        state = result.current_state
        output({"node": node_name, "state_id": state.id, "state_label": state.label})
    except Exception as e:
        output({"error": str(e)})


def _resolve_transition_label(available_transitions, label):
    """Match *label* against available (id, label) pairs using the same
    exact → suffix → prefix → substring fallback chain, regardless of
    whether the transitions came from a live message (local path, dot
    access) or a daemon-returned dict (dict access) — callers pass a list
    of (id, label) tuples either way."""
    matching = [tid for tid, lbl in available_transitions if lbl == label]
    if not matching:
        matching = [tid for tid, lbl in available_transitions if lbl.endswith('_' + label)]
    if not matching:
        matching = [tid for tid, lbl in available_transitions if lbl.startswith(label + '_')]
    if not matching:
        matching = [tid for tid, lbl in available_transitions if label in lbl]
    return matching


def _lifecycle_set_via_daemon(node_name, args):
    """Attempt the full lifecycle-set sequence via the daemon's existing
    generic "call_service" op (same one ros2_service.py's cmd_services_call
    and cmd_lifecycle_get use — no new daemon-side handler needed).

    Returns None only if the *first* daemon call is unreachable, so the
    caller can fall back to the local rclpy path unchanged. Once that first
    call succeeds, the sequence commits to the daemon for its remaining
    calls (1-2 more) rather than risking a half-daemon half-local mix.
    """
    def call(service_suffix, srv_type, request_data):
        return try_fastd("call_service", {
            "service_name": f"{node_name}/{service_suffix}",
            "srv_type": srv_type,
            "request_data": request_data,
            "timeout": args.timeout,
        }, timeout=args.timeout + 2.0)

    try:
        transition_id = int(args.transition)
    except ValueError:
        resp = call("get_available_transitions", "lifecycle_msgs/srv/GetAvailableTransitions", {})
        if resp is None:
            return None
        if "error" in resp:
            return resp
        available = resp["result"]["available_transitions"]
        pairs = [(td["transition"]["id"], td["transition"]["label"]) for td in available]
        matching = _resolve_transition_label(pairs, args.transition)
        if not matching:
            return {"error": f"Unknown transition '{args.transition}'. Available: {[lbl for _, lbl in pairs]}"}
        transition_id = matching[0]

    resp = call("change_state", "lifecycle_msgs/srv/ChangeState",
                {"transition": {"id": transition_id, "label": ""}})
    if resp is None:
        return None
    if "error" in resp:
        return resp

    result = {"node": node_name, "transition": args.transition, "success": resp["result"]["success"]}

    if getattr(args, "verify", False) and result["success"]:
        verify_resp = call("get_state", "lifecycle_msgs/srv/GetState", {})
        if verify_resp is None or "error" in verify_resp:
            result["verify_error"] = (verify_resp or {}).get("error", "daemon became unreachable during verify")
        else:
            state = verify_resp["result"]["current_state"]
            result["verified_state_id"] = state["id"]
            result["verified_state_label"] = state["label"]

    return result


def cmd_lifecycle_set(args):
    """Trigger a lifecycle state transition on a managed node."""
    retries = getattr(args, 'retries', 1)
    node_name = args.node if args.node.startswith('/') else '/' + args.node

    # Fast path: persistent daemon, no rclpy.init() for this call at all.
    if retries == 1:
        result = _lifecycle_set_via_daemon(node_name, args)
        if result is not None:
            return output(result)

    try:
        from lifecycle_msgs.srv import ChangeState, GetAvailableTransitions, GetState
        from lifecycle_msgs.msg import Transition
        with ros2_context():
            node = ROS2CLI()

            try:
                transition_id = int(args.transition)
            except ValueError:
                trans_result, err = call_ros_service(
                    node, GetAvailableTransitions, f"{node_name}/get_available_transitions",
                    GetAvailableTransitions.Request(), args.timeout, retries,
                    unavailable_hint="Is it a managed node?",
                )
                if err:
                    return output(err)
                available_transitions = trans_result.available_transitions
                pairs = [(td.transition.id, td.transition.label) for td in available_transitions]
                matching = _resolve_transition_label(pairs, args.transition)
                if not matching:
                    return output({"error": f"Unknown transition '{args.transition}'. Available: {[lbl for _, lbl in pairs]}"})
                transition_id = matching[0]

            request = ChangeState.Request()
            transition = Transition()
            transition.id = transition_id
            request.transition = transition

            change_result, err = call_ros_service(
                node, ChangeState, f"{node_name}/change_state", request,
                args.timeout, retries, unavailable_hint="Is it a managed node?",
            )
            if err:
                return output(err)

            result = {"node": node_name, "transition": args.transition,
                      "success": change_result.success}

            # Optional same-node verify: confirm the new state via GetState
            # in this node/process instead of a second 'lifecycle get' CLI
            # invocation (second rclpy.init cycle).
            if getattr(args, "verify", False) and result["success"]:
                state_result, verify_err = call_ros_service(
                    node, GetState, f"{node_name}/get_state", GetState.Request(), args.timeout, retries,
                )
                if verify_err:
                    result["verify_error"] = verify_err["error"]
                else:
                    new_state = state_result.current_state
                    result["verified_state_id"] = new_state.id
                    result["verified_state_label"] = new_state.label

        output(result)
    except Exception as e:
        output({"error": str(e)})


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
