#!/usr/bin/env python3
"""ROS 2 parameter commands."""

import json
import os
import re
import time
import types

import rclpy
from rcl_interfaces.msg import Parameter, ParameterValue

from ros2_utils import ROS2CLI, output, parse_node_param, ros2_context


def _param_value_to_python(v):
    """Convert a rcl_interfaces ParameterValue to a native Python value."""
    if v.type == 1:
        return v.bool_value
    elif v.type == 2:
        return v.integer_value
    elif v.type == 3:
        return v.double_value
    elif v.type == 4:
        return v.string_value
    elif v.type == 5:
        return list(v.byte_array_value)
    elif v.type == 6:
        return list(v.bool_array_value)
    elif v.type == 7:
        return list(v.integer_array_value)
    elif v.type == 8:
        return list(v.double_array_value)
    elif v.type == 9:
        return list(v.string_array_value)
    return None


def _infer_param_value(value):
    """Infer ParameterValue type from a native Python value."""
    pv = ParameterValue()
    if isinstance(value, bool):
        pv.type = 1
        pv.bool_value = value
    elif isinstance(value, int):
        pv.type = 2
        pv.integer_value = value
    elif isinstance(value, float):
        pv.type = 3
        pv.double_value = value
    elif isinstance(value, str):
        pv.type = 4
        pv.string_value = value
    elif isinstance(value, (list, tuple)) and value:
        first = value[0]
        if isinstance(first, bool):
            pv.type = 6
            pv.bool_array_value = list(value)
        elif isinstance(first, int):
            pv.type = 7
            pv.integer_array_value = list(value)
        elif isinstance(first, float):
            pv.type = 8
            pv.double_array_value = list(value)
        elif isinstance(first, str):
            pv.type = 9
            pv.string_array_value = list(value)
        else:
            pv.type = 4
            pv.string_value = str(value)
    else:
        pv.type = 4
        pv.string_value = str(value)
    return pv


def cmd_params_list(args):
    try:
        from rcl_interfaces.srv import ListParameters
        with ros2_context():
            node = ROS2CLI()
            node_name, _ = parse_node_param(args.node)
            if not node_name.startswith('/'):
                node_name = '/' + node_name

            service_name = f"{node_name}/list_parameters"
            client = node.create_client(ListParameters, service_name)

            retries = getattr(args, 'retries', 1)
            for attempt in range(retries):
                last_attempt = (attempt == retries - 1)

                if not client.wait_for_service(timeout_sec=args.timeout):
                    if not last_attempt:
                        continue
                    return output({"error": f"Parameter service not available for {node_name}"})

                future = client.call_async(ListParameters.Request())
                end_time = time.time() + args.timeout
                while time.time() < end_time and not future.done():
                    rclpy.spin_once(node, timeout_sec=0.1)

                if future.done():
                    result = future.result()
                    names = result.result.names if result.result else []
                    formatted = [f"{node_name}:{n}" for n in names]
                    output({"node": node_name, "parameters": formatted, "count": len(formatted)})
                    return

                future.cancel()
                if not last_attempt:
                    continue

        output({"error": "Timeout listing parameters"})
    except Exception as e:
        output({"error": str(e)})


def cmd_params_get(args):
    if getattr(args, 'param_name', None):
        full_name = args.name.rstrip(':') + ':' + args.param_name
    else:
        full_name = args.name
    if ':' not in full_name or not full_name.split(':', 1)[1]:
        return output({"error": "Use format /node_name:param_name or /node_name param_name (e.g. /turtlesim background_r)"})

    try:
        from rcl_interfaces.srv import GetParameters
        with ros2_context():
            node = ROS2CLI()
            node_name, param_name = full_name.split(':', 1)
            if not node_name.startswith('/'):
                node_name = '/' + node_name

            service_name = f"{node_name}/get_parameters"
            client = node.create_client(GetParameters, service_name)

            retries = getattr(args, 'retries', 1)
            for attempt in range(retries):
                last_attempt = (attempt == retries - 1)

                if not client.wait_for_service(timeout_sec=args.timeout):
                    if not last_attempt:
                        continue
                    return output({"error": f"Parameter service not available for {node_name}"})

                request = GetParameters.Request()
                request.names = [param_name] if param_name else []
                future = client.call_async(request)

                end_time = time.time() + args.timeout
                while time.time() < end_time and not future.done():
                    rclpy.spin_once(node, timeout_sec=0.1)

                if future.done():
                    result = future.result()
                    values = result.values if result.values else []
                    value_str = ""
                    exists = False
                    if values:
                        v = values[0]
                        if v.type == 1:
                            value_str = str(v.bool_value)
                            exists = True
                        elif v.type == 2:
                            value_str = str(v.integer_value)
                            exists = True
                        elif v.type == 3:
                            value_str = str(v.double_value)
                            exists = True
                        elif v.type == 4:
                            value_str = v.string_value
                            exists = True
                        elif v.type in [5, 6, 7, 8, 9]:
                            value_str = str(v)
                            exists = True
                    output({"name": full_name, "value": value_str, "exists": exists})
                    return

                future.cancel()
                if not last_attempt:
                    continue

        output({"error": "Timeout getting parameter"})
    except Exception as e:
        output({"error": str(e)})


def cmd_params_set(args):
    if getattr(args, 'extra_value', None) is not None:
        full_name = args.name.rstrip(':') + ':' + args.value
        value_str = args.extra_value
    else:
        full_name = args.name
        value_str = args.value
    if ':' not in full_name or not full_name.split(':', 1)[1]:
        return output({"error": "Use format /node_name:param_name value or /node_name param_name value (e.g. /turtlesim background_r 255)"})

    try:
        from rcl_interfaces.srv import SetParameters
        with ros2_context():
            node = ROS2CLI()
            node_name, param_name = full_name.split(':', 1)
            if not node_name.startswith('/'):
                node_name = '/' + node_name

            service_name = f"{node_name}/set_parameters"
            client = node.create_client(SetParameters, service_name)

            retries = getattr(args, 'retries', 1)

            pv = ParameterValue()
            try:
                if value_str.lower() in ('true', 'false'):
                    pv.type = 1
                    pv.bool_value = value_str.lower() == 'true'
                elif '.' in value_str:
                    pv.type = 3
                    pv.double_value = float(value_str)
                else:
                    try:
                        pv.type = 2
                        pv.integer_value = int(value_str)
                    except Exception:
                        pv.type = 4
                        pv.string_value = value_str
            except Exception:
                pv.type = 4
                pv.string_value = value_str

            for attempt in range(retries):
                last_attempt = (attempt == retries - 1)

                if not client.wait_for_service(timeout_sec=args.timeout):
                    if not last_attempt:
                        continue
                    return output({"error": f"Parameter service not available for {node_name}"})

                request = SetParameters.Request()
                param = Parameter()
                param.name = param_name
                param.value = pv
                request.parameters = [param]

                future = client.call_async(request)
                end_time = time.time() + args.timeout
                while time.time() < end_time and not future.done():
                    rclpy.spin_once(node, timeout_sec=0.1)

                if future.done():
                    result = future.result()
                    if result.results and result.results[0].successful:
                        output({"name": full_name, "value": value_str, "success": True})
                    else:
                        reason = result.results[0].reason if result.results else ""
                        reason_lc = reason.lower()
                        if re.search(r'\b(read[- ]?only|readonly)\b', reason_lc):
                            output({"name": full_name, "value": value_str, "success": False,
                                    "error": "Parameter is read-only and cannot be changed at runtime",
                                    "read_only": True})
                        else:
                            output({"name": full_name, "value": value_str, "success": False,
                                    "error": reason or "Parameter rejected by node"})
                    return

                future.cancel()
                if not last_attempt:
                    continue

        output({"error": "Timeout setting parameter"})
    except Exception as e:
        output({"error": str(e)})


def cmd_params_describe(args):
    """Get the descriptor of a parameter."""
    if getattr(args, 'param_name', None):
        full_name = args.name.rstrip(':') + ':' + args.param_name
    else:
        full_name = args.name
    if ':' not in full_name or not full_name.split(':', 1)[1]:
        return output({"error": "Use format /node_name:param_name or /node_name param_name"})

    try:
        from rcl_interfaces.srv import DescribeParameters
        with ros2_context():
            node = ROS2CLI()
            node_name, param_name = full_name.split(':', 1)
            if not node_name.startswith('/'):
                node_name = '/' + node_name

            service_name = f"{node_name}/describe_parameters"
            client = node.create_client(DescribeParameters, service_name)

            retries = getattr(args, 'retries', 1)
            desc_result = None
            for attempt in range(retries):
                last_attempt = (attempt == retries - 1)

                if not client.wait_for_service(timeout_sec=args.timeout):
                    if not last_attempt:
                        continue
                    return output({"error": f"Parameter service not available for {node_name}"})

                request = DescribeParameters.Request()
                request.names = [param_name]
                future = client.call_async(request)

                end_time = time.time() + args.timeout
                while time.time() < end_time and not future.done():
                    rclpy.spin_once(node, timeout_sec=0.1)

                if future.done():
                    desc_result = future.result()
                    break
                future.cancel()
                if not last_attempt:
                    continue

        if desc_result is None:
            return output({"error": "Timeout describing parameter"})

        if not desc_result.descriptors:
            return output({"error": f"Parameter '{param_name}' not found on {node_name}"})

        d = desc_result.descriptors[0]
        out = {
            "name": full_name,
            "type": d.type,
            "description": d.description,
            "read_only": d.read_only,
            "dynamic_typing": d.dynamic_typing,
            "additional_constraints": d.additional_constraints,
        }
        if d.floating_point_range:
            r = d.floating_point_range[0]
            out["floating_point_range"] = {"from_value": r.from_value,
                                           "to_value": r.to_value, "step": r.step}
        if d.integer_range:
            r = d.integer_range[0]
            out["integer_range"] = {"from_value": r.from_value,
                                    "to_value": r.to_value, "step": r.step}
        output(out)
    except Exception as e:
        output({"error": str(e)})


def _dump_params(node, node_name, timeout, retries=1):
    """Fetch all parameters for node_name using the provided rclpy node.

    Returns a {param_name: value} dict on success, or None after calling output({"error": ...}).
    """
    try:
        from rcl_interfaces.srv import ListParameters, GetParameters

        list_client = node.create_client(ListParameters, f"{node_name}/list_parameters")
        names = None
        for attempt in range(retries):
            last_attempt = (attempt == retries - 1)
            if not list_client.wait_for_service(timeout_sec=timeout):
                if not last_attempt:
                    continue
                output({"error": f"Parameter service not available for {node_name}"})
                return None
            future = list_client.call_async(ListParameters.Request())
            end_time = time.time() + timeout
            while time.time() < end_time and not future.done():
                rclpy.spin_once(node, timeout_sec=0.1)
            if future.done():
                names = future.result().result.names if future.result().result else []
                break
            future.cancel()
            if not last_attempt:
                continue
        if names is None:
            output({"error": "Timeout listing parameters"})
            return None
        if not names:
            return {}

        get_client = node.create_client(GetParameters, f"{node_name}/get_parameters")
        values = None
        for attempt in range(retries):
            last_attempt = (attempt == retries - 1)
            if not get_client.wait_for_service(timeout_sec=timeout):
                if not last_attempt:
                    continue
                output({"error": f"GetParameters service not available for {node_name}"})
                return None
            get_req = GetParameters.Request()
            get_req.names = list(names)
            future = get_client.call_async(get_req)
            end_time = time.time() + timeout
            while time.time() < end_time and not future.done():
                rclpy.spin_once(node, timeout_sec=0.1)
            if future.done():
                values = future.result().values or []
                break
            future.cancel()
            if not last_attempt:
                continue
        if values is None:
            output({"error": "Timeout getting parameters"})
            return None

        return {n: _param_value_to_python(v) for n, v in zip(names, values)}
    except Exception as e:
        output({"error": str(e)})
        return None


def cmd_params_dump(args):
    """Export all parameters of a node as a JSON dict."""
    node_name = args.node
    if not node_name.startswith('/'):
        node_name = '/' + node_name

    with ros2_context():
        node = ROS2CLI()
        result = _dump_params(node, node_name, args.timeout, getattr(args, 'retries', 1))
    if result is not None:
        output({"node": node_name, "parameters": result, "count": len(result)})


def cmd_params_load(args):
    """Load parameters onto a node from a JSON string or file."""
    node_name = args.node
    if not node_name.startswith('/'):
        node_name = '/' + node_name

    raw = args.params
    try:
        import pathlib
        if pathlib.Path(raw).exists():
            with open(raw) as f:
                data = json.load(f)
        else:
            data = json.loads(raw)
    except (json.JSONDecodeError, TypeError, ValueError, OSError) as e:
        return output({"error": f"Invalid JSON or file not found: {e}"})

    if not isinstance(data, dict):
        return output({"error": "JSON must be a flat object {param_name: value, ...}"})

    try:
        from rcl_interfaces.srv import SetParameters
        with ros2_context():
            node = ROS2CLI()
            service_name = f"{node_name}/set_parameters"
            client = node.create_client(SetParameters, service_name)
            retries = getattr(args, 'retries', 1)
            for attempt in range(retries):
                last_attempt = (attempt == retries - 1)

                if not client.wait_for_service(timeout_sec=args.timeout):
                    if not last_attempt:
                        continue
                    return output({"error": f"Parameter service not available for {node_name}"})

                request = SetParameters.Request()
                params = []
                for pname, pvalue in data.items():
                    p = Parameter()
                    p.name = pname
                    p.value = _infer_param_value(pvalue)
                    params.append(p)
                request.parameters = params

                future = client.call_async(request)
                end_time = time.time() + args.timeout
                while time.time() < end_time and not future.done():
                    rclpy.spin_once(node, timeout_sec=0.1)

                if future.done():
                    results_raw = future.result().results or []
                    results = []
                    for pname, r in zip(data.keys(), results_raw):
                        entry = {"name": pname, "success": r.successful}
                        if not r.successful and r.reason:
                            entry["reason"] = r.reason
                        results.append(entry)
                    output({"node": node_name, "results": results})
                    return

                future.cancel()
                if not last_attempt:
                    continue

        output({"error": "Timeout loading parameters"})
    except Exception as e:
        output({"error": str(e)})


def cmd_params_delete(args):
    """Delete one or more parameters from a node."""
    if getattr(args, 'param_name', None):
        full_name = args.name.rstrip(':') + ':' + args.param_name
    else:
        full_name = args.name
    if ':' not in full_name or not full_name.split(':', 1)[1]:
        return output({"error": "Use format /node_name:param_name or /node_name param_name"})

    node_name, param_name = full_name.split(':', 1)
    if not node_name.startswith('/'):
        node_name = '/' + node_name

    param_names = [param_name] + (list(args.extra_names) if getattr(args, 'extra_names', None) else [])

    try:
        from rcl_interfaces.srv import SetParameters
        from rcl_interfaces.msg import Parameter as _Param, ParameterValue as _PV
        with ros2_context():
            node = ROS2CLI()
            service_name = f"{node_name}/set_parameters"
            client = node.create_client(SetParameters, service_name)
            retries = getattr(args, 'retries', 1)
            for attempt in range(retries):
                last_attempt = (attempt == retries - 1)

                if not client.wait_for_service(timeout_sec=args.timeout):
                    if not last_attempt:
                        continue
                    return output({"error": f"Parameter service not available for {node_name}"})

                request = SetParameters.Request()
                params = []
                for pname in param_names:
                    p = _Param()
                    p.name = pname
                    p.value = _PV()  # type=0 == PARAMETER_NOT_SET
                    params.append(p)
                request.parameters = params

                future = client.call_async(request)
                end_time = time.time() + args.timeout
                while time.time() < end_time and not future.done():
                    rclpy.spin_once(node, timeout_sec=0.1)

                if future.done():
                    results_raw = future.result().results or []
                    results = []
                    for pname, r in zip(param_names, results_raw):
                        entry = {"name": pname, "success": r.successful}
                        if not r.successful:
                            entry["error"] = r.reason or "Node rejected deletion (parameter may be read-only or undeclaring is not allowed)"
                        results.append(entry)
                    output({"node": node_name, "results": results, "count": len(param_names)})
                    return

                future.cancel()
                if not last_attempt:
                    continue

        output({"error": "Timeout deleting parameters"})
    except Exception as e:
        output({"error": str(e)})


# ---------------------------------------------------------------------------
# Parameter preset commands
# Presets are stored as plain {param_name: value} JSON files under
# .presets/{preset_name}.json  (beside the scripts/ directory,
# created automatically — same pattern as the .artifacts/ folder)
# ---------------------------------------------------------------------------

def _presets_base():
    """Return the absolute path to the .presets/ directory, creating it if needed."""
    base = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '.presets'))
    os.makedirs(base, exist_ok=True)
    return base


def cmd_params_preset_save(args):
    """Save the current parameters of a node as a named preset."""
    node_name = args.node
    if not node_name.startswith('/'):
        node_name = '/' + node_name

    with ros2_context():
        node = ROS2CLI()
        params_dict = _dump_params(node, node_name, args.timeout, getattr(args, 'retries', 1))
    if params_dict is None:
        return  # _dump_params already called output({"error": ...})

    preset_path = os.path.join(_presets_base(), f"{args.preset}.json")

    with open(preset_path, 'w') as f:
        json.dump(params_dict, f, indent=2)

    output({"node": node_name, "preset": args.preset, "path": preset_path,
            "count": len(params_dict)})


def cmd_params_preset_load(args):
    """Restore a named preset onto a node."""
    node_name = args.node
    if not node_name.startswith('/'):
        node_name = '/' + node_name

    preset_path = os.path.join(_presets_base(), f"{args.preset}.json")
    if not os.path.exists(preset_path):
        return output({"error": f"Preset '{args.preset}' not found",
                       "path": preset_path})

    load_args = types.SimpleNamespace(
        node=node_name, params=preset_path, timeout=args.timeout,
        retries=getattr(args, 'retries', 1),
    )
    cmd_params_load(load_args)


def cmd_params_preset_list(args):
    """List all saved presets."""
    base_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '.presets'))

    presets = []
    if os.path.isdir(base_dir):
        for fname in sorted(os.listdir(base_dir)):
            if fname.endswith('.json'):
                presets.append({
                    "preset": fname[:-5],
                    "path": os.path.join(base_dir, fname),
                })

    output({"presets": presets, "count": len(presets)})


def cmd_params_preset_delete(args):
    """Delete a saved preset file."""
    preset_path = os.path.join(_presets_base(), f"{args.preset}.json")
    if not os.path.exists(preset_path):
        return output({"error": f"Preset '{args.preset}' not found"})

    os.remove(preset_path)
    output({"preset": args.preset, "deleted": True})
