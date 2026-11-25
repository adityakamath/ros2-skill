#!/usr/bin/env python3
"""``braking``: measure how far the base coasts after the command is removed, and feed it to the slow-down zone.

``measure`` drives short closed-loop moves with the decel zone off, then reads the coast distance past the
stop condition. The deceleration ``v² / (2 × coast)`` is stored in ``.presets/braking.json`` and
``publish-until`` uses it in place of the 0.5 m/s² default. Linear motion only.
"""

import json
import math
import time
from pathlib import Path

STORE = Path(__file__).resolve().parent.parent / ".presets" / "braking.json"
_MIN_COAST_M = 0.003   # below this the coast is inside odometry noise


def load_braking():
    """The stored measurement, or None."""
    try:
        data = json.loads(STORE.read_text(encoding="utf-8"))
    except (OSError, ValueError):
        return None
    return data if isinstance(data.get("linear_mps2"), (int, float)) and data["linear_mps2"] > 0 else None


def decel_from_coast(speed_mps, coast_m):
    """Constant deceleration that stops *speed_mps* in *coast_m*, or None when the coast is below odometry resolution."""
    if speed_mps <= 0 or coast_m < _MIN_COAST_M:
        return None
    return speed_mps ** 2 / (2.0 * coast_m)


def _measure_once(direction, distance, speed, summary, odom):
    from ros2_plan import compile_plan, run_cli, run_plan
    plan = compile_plan(f"move {direction} {distance} m at {speed} m/s", summary, [], str(STORE.parent.parent / ".artifacts"))
    if plan["warnings"] or not plan["steps"]:
        return {"direction": direction, "ok": False, "error": "; ".join(plan["warnings"]) or "nothing compiled"}
    for st in plan["steps"]:  # decel zone off: the coast must be the robot's own
        st["commands"] = [c + " --slow-last 0" if c.startswith("topics publish-until") else c for c in st["commands"]]
    report = run_plan(plan, run_cli, odom)
    move = next((s for s in report["steps"] if s.get("text", "").startswith("move")), {})
    reached = (move.get("result") or {}).get("euclidean_distance")
    total = report.get("moved_m")
    if not report["ok"] or reached is None or total is None:
        return {"direction": direction, "ok": False, "error": report.get("cause") or "the move did not finish; see `plan --run`"}
    coast = round(total - reached, 4)
    return {"direction": direction, "ok": True, "reached_m": round(reached, 4), "coast_m": coast,
            "decel_mps2": None if decel_from_coast(speed, coast) is None else round(decel_from_coast(speed, coast), 3)}


def cmd_braking(args):
    from ros2_utils import output
    sub = args.subcommand
    if sub == "show":
        return output({"braking": load_braking(), "note": "none stored: publish-until uses the 0.5 m/s² default"} if not load_braking()
                      else {"braking": load_braking()})
    if sub == "clear":
        STORE.unlink(missing_ok=True)
        return output({"cleared": True})
    from ros2_motion_lock import motion_lock
    from ros2_profile import load_profile_summary
    from ros2_utils import ros2_context
    summary = load_profile_summary(readonly=True)
    fused = ((summary or {}).get("localization_config") or {}).get("fused_sources") or {}
    odom = next(iter(fused.values()), None)
    if not summary or not odom:
        return output({"error": "braking measure needs a profile with a base velocity topic and fused odometry"})
    if not 0.1 <= args.distance <= 1.0 or not 0.05 <= args.speed <= 0.5:
        return output({"error": "use --distance 0.1-1.0 m and --speed 0.05-0.5 m/s"})
    runs = []
    with ros2_context(), motion_lock("braking measure"):
        for i in range(args.runs):  # alternate directions so the robot ends where it began
            runs.append(_measure_once("forward" if i % 2 == 0 else "back", args.distance, args.speed, summary, odom))
            if not runs[-1]["ok"]:
                break
    values = [r["decel_mps2"] for r in runs if r.get("decel_mps2")]
    result = {"runs": runs, "speed_mps": args.speed, "distance_m": args.distance}
    if not values:
        return output(dict(result, success=False, error="no run gave a usable coast distance; nothing stored"))
    stored = {"linear_mps2": round(sum(values) / len(values), 3), "speed_mps": args.speed, "samples": len(values),
              "measured_at": time.strftime("%Y-%m-%dT%H:%M:%S"), "robot": (summary.get("robot_type") or "")}
    STORE.parent.mkdir(parents=True, exist_ok=True)
    STORE.write_text(json.dumps(stored, indent=2) + "\n", encoding="utf-8")
    output(dict(result, success=True, stored=stored, zone_example_m=round(0.2 ** 2 / (2 * stored["linear_mps2"]), 3)))
