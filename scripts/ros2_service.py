#!/usr/bin/env python3
"""ROS 2 service commands."""

import json
import time

import rclpy

from ros2_utils import (
    ROS2CLI, get_srv_type, msg_to_dict, dict_to_msg, output,
)


def cmd_services_list(args):
    try:
        rclpy.init()
        node = ROS2CLI()
        services = node.get_service_names()
        service_list = []
        type_list = []
        for name, types in services:
            service_list.append(name)
            type_list.append(types[0] if types else "")
        result = {"services": service_list, "types": type_list, "count": len(service_list)}
        rclpy.shutdown()
        output(result)
    except Exception as e:
        output({"error": str(e)})


def cmd_services_type(args):
    try:
        rclpy.init()
        node = ROS2CLI()
        services = node.get_service_names()
        result = {"service": args.service, "type": ""}
        for name, types in services:
            if name == args.service:
                result["type"] = types[0] if types else ""
                break
        rclpy.shutdown()
        output(result)
    except Exception as e:
        output({"error": str(e)})


def cmd_services_details(args):
    try:
        rclpy.init()
        node = ROS2CLI()
        services = node.get_service_names()

        result = {"service": args.service, "type": "", "request": {}, "response": {}}

        for name, types in services:
            if name == args.service:
                result["type"] = types[0] if types else ""
                break

        if result["type"]:
            srv_class = get_srv_type(result["type"])
            if srv_class:
                try:
                    result["request"] = msg_to_dict(srv_class.Request())
                except Exception:
                    pass
                try:
                    result["response"] = msg_to_dict(srv_class.Response())
                except Exception:
                    pass

        rclpy.shutdown()
        output(result)
    except Exception as e:
        output({"error": str(e)})


def cmd_services_call(args):
    if getattr(args, 'extra_request', None) is not None:
        service_type_override = args.request
        request_json = args.extra_request
    else:
        service_type_override = None
        request_json = args.request

    try:
        request_data = json.loads(request_json)
    except (json.JSONDecodeError, TypeError) as e:
        return output({"error": f"Invalid JSON request: {e}"})

    retries = getattr(args, 'retries', 1)

    try:
        rclpy.init()
        node = ROS2CLI()

        service_type = service_type_override or args.service_type
        if not service_type:
            services = node.get_service_names()
            for name, types in services:
                if name == args.service:
                    service_type = types[0] if types else ""
                    break

        if not service_type:
            rclpy.shutdown()
            return output({
                "error": f"Service not found: {args.service}",
                "hint": "Use --service-type to specify the type explicitly (e.g. --service-type std_srvs/srv/SetBool)"
            })

        srv_class = get_srv_type(service_type)
        if not srv_class:
            rclpy.shutdown()
            return output({"error": f"Cannot load service type: {service_type}"})

        client = node.create_client(srv_class, args.service)
        timeout = args.timeout

        for attempt in range(retries):
            last_attempt = (attempt == retries - 1)

            if not client.wait_for_service(timeout_sec=timeout):
                if not last_attempt:
                    continue
                rclpy.shutdown()
                return output({"error": f"Service not available: {args.service}"})

            request = dict_to_msg(srv_class.Request, request_data)
            future = client.call_async(request)

            end_time = time.time() + timeout
            while time.time() < end_time and not future.done():
                rclpy.spin_once(node, timeout_sec=0.1)

            if future.done():
                result_msg = future.result()
                result_dict = msg_to_dict(result_msg)
                rclpy.shutdown()
                output({"service": args.service, "success": True, "result": result_dict})
                return

            future.cancel()
            if not last_attempt:
                continue

        rclpy.shutdown()
        output({"service": args.service, "success": False, "error": "Service call timeout"})
    except Exception as e:
        output({"error": str(e)})


def cmd_services_find(args):
    """Find services of a specific service type."""
    import re
    if not args.service_type:
        return output({"error": "service_type argument is required"})

    target_raw = args.service_type

    def _norm_srv(t):
        return re.sub(r'/srv/', '/', t)

    target_norm = _norm_srv(target_raw)

    try:
        rclpy.init()
        node = ROS2CLI()
        all_services = node.get_service_names_and_types()
        rclpy.shutdown()

        matched = [
            name for name, types in all_services
            if any(_norm_srv(t) == target_norm for t in types)
        ]
        output({
            "service_type": target_raw,
            "services": matched,
            "count": len(matched),
        })
    except Exception as e:
        output({"error": str(e)})
