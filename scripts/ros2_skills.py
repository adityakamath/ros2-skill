#!/usr/bin/env python3
"""Skills / behaviours introspection commands.

Provides the ``skills`` command group for listing and inspecting
behaviours that are registered with the running robot stack:

- ``skills list`` — enumerate skills/behaviours visible on the ROS 2 graph,
                    filtered by ``--loaded`` (only active) or ``--all``
                    (everything discoverable).

**What counts as a "skill"?**  On Innate robots the behaviour server exposes
registered behaviours via a ``/behaviour_server/get_registered_behaviours`` service
(or similar).  On MoveIt2 stacks the "skills" are MoveGroup capabilities.  For
generic stacks we discover skills heuristically by scanning topic and service
names for common patterns (``skill``, ``behaviour``, ``capability``, ``task``,
``action``).

All commands follow the profile-first, live-discovery pattern.
"""

import time

import rclpy

from ros2_utils import (
    ROS2CLI, get_srv_type, output, ros2_context,
)

# Well-known service names for behaviour / skill servers
_BEHAVIOUR_SERVICES = [
    "/behaviour_server/get_registered_behaviours",
    "/behavior_server/get_registered_behaviors",
    "/bt_navigator/get_registered_behaviours",
    "/capability_server/get_capabilities",
    "/skill_server/list_skills",
    "/task_server/list_tasks",
]

# Heuristic keywords for graph discovery fallback
_SKILL_KEYWORDS = ("skill", "behaviour", "behavior", "capability", "task", "plugin")


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _call_empty_service(node, svc_name, srv_class, timeout: float = 5.0):
    """Call *svc_name* with an empty request; return (response, error_str)."""
    client = node.create_client(srv_class, svc_name)
    try:
        if not client.wait_for_service(timeout_sec=min(timeout, 3.0)):
            return None, f"not available"
        future = client.call_async(srv_class.Request())
        end = time.time() + timeout
        while time.time() < end and not future.done():
            rclpy.spin_once(node, timeout_sec=0.1)
        if not future.done():
            future.cancel()
            return None, "timeout"
        return future.result(), None
    finally:
        client.destroy()


def _heuristic_skills(node) -> list[dict]:
    """Discover skills from topic/service names heuristically."""
    skills: list[dict] = []
    seen: set[str] = set()

    for svc, types in node.get_service_names_and_types():
        kw = svc.lower()
        if any(k in kw for k in _SKILL_KEYWORDS):
            name = svc.split("/")[-1]
            if name not in seen:
                seen.add(name)
                skills.append({
                    "name":   name,
                    "source": "service",
                    "path":   svc,
                    "types":  list(types),
                })

    for topic, types in node.get_topic_names_and_types():
        kw = topic.lower()
        if any(k in kw for k in _SKILL_KEYWORDS):
            name = topic.split("/")[-1]
            if name not in seen:
                seen.add(name)
                skills.append({
                    "name":   name,
                    "source": "topic",
                    "path":   topic,
                    "types":  list(types),
                })

    return skills


# ---------------------------------------------------------------------------
# skills list
# ---------------------------------------------------------------------------

def cmd_skills_list(args):
    """List skills/behaviours registered with the running robot stack.

    Tries known behaviour-server service endpoints first, then falls back
    to heuristic graph discovery.

    Flags:

    - ``--loaded`` : only return skills that are currently active / loaded.
    - ``--all``    : return everything, including discovered but inactive
                     entries (default).
    - ``--service``: override the behaviour-server service name.

    Returns::

        {
          "skills": [
            {"name": "navigate_to_pose", "source": "behaviour_server", "loaded": true},
            ...
          ],
          "count": N,
          "source": "behaviour_server" | "heuristic",
          "service": "/behaviour_server/get_registered_behaviours"
        }

    **Profile usage:** if the profile lists a ``behaviour_server_service``
    annotation, pass it via ``--service`` to skip discovery.
    """
    loaded_only  = getattr(args, "loaded", False)
    service_override = getattr(args, "service", None)
    timeout      = getattr(args, "timeout", 10.0)

    try:
        with ros2_context():
            node = ROS2CLI()

            # 1. Determine which service to try
            candidates = (
                [service_override] if service_override
                else _BEHAVIOUR_SERVICES
            )

            live_services = {s for s, _ in node.get_service_names_and_types()}
            used_service: str | None = None

            for svc in candidates:
                if svc in live_services:
                    used_service = svc
                    break

            # 2. Try structured behaviour-server call
            if used_service:
                # Detect actual service type from graph
                svc_type_str: str | None = None
                for svc, types in node.get_service_names_and_types():
                    if svc == used_service:
                        svc_type_str = types[0] if types else None
                        break

                srv_class = get_srv_type(svc_type_str) if svc_type_str else None

                if srv_class is not None:
                    resp, err = _call_empty_service(node, used_service, srv_class, timeout)
                    if resp is not None:
                        # Extract list from common response field names
                        entries = []
                        for field in ("behaviours", "behaviors", "skills", "capabilities",
                                      "tasks", "plugins", "registered_behaviours"):
                            val = getattr(resp, field, None)
                            if val is not None:
                                entries = list(val)
                                break

                        skills = []
                        for entry in entries:
                            if isinstance(entry, str):
                                skills.append({"name": entry, "loaded": True,
                                               "source": "behaviour_server"})
                            elif hasattr(entry, "name"):
                                loaded = bool(getattr(entry, "is_loaded",
                                             getattr(entry, "loaded", True)))
                                skills.append({
                                    "name":   str(entry.name),
                                    "loaded": loaded,
                                    "source": "behaviour_server",
                                })

                        if loaded_only:
                            skills = [s for s in skills if s.get("loaded")]

                        return output({
                            "skills":  skills,
                            "count":   len(skills),
                            "source":  "behaviour_server",
                            "service": used_service,
                        })

            # 3. Heuristic fallback
            skills = _heuristic_skills(node)

            if loaded_only:
                # For heuristic discovery all entries are "live" (they exist on graph)
                skills = [dict(s, loaded=True) for s in skills]
            else:
                skills = [dict(s, loaded=True) for s in skills]

        hint_parts = []
        if not used_service:
            hint_parts.append(
                "No known behaviour-server service found. "
                "Results are heuristic (topic/service name scan). "
                "Start a behaviour server or pass --service to specify one."
            )

        result: dict = {
            "skills": skills,
            "count":  len(skills),
            "source": "heuristic",
        }
        if used_service:
            result["service"] = used_service
        if hint_parts:
            result["hint"] = " ".join(hint_parts)

        return output(result)

    except Exception as e:
        return output({"error": str(e)})


if __name__ == "__main__":
    import sys as _sys
    _mod = __import__("os").path.basename(__file__)
    _cli = __import__("os").path.join(
        __import__("os").path.dirname(__import__("os").path.abspath(__file__)),
        "ros2_cli.py",
    )
    print(
        f"[ros2-skill] '{_mod}' is an internal module — do not run it directly.\n"
        "Use the main entry point:\n"
        f"  python3 {_cli} <command> [subcommand] [args]\n"
        f"See all commands:  python3 {_cli} --help",
        file=_sys.stderr,
    )
    _sys.exit(1)
