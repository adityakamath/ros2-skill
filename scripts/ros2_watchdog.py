#!/usr/bin/env python3
"""Simple safety heartbeat watchdog.

When the heartbeat sentinel goes stale, this script can issue estop once and
write a log entry. Intended to run in a detached tmux session.
"""

import argparse
import json
import os
import subprocess
import time


SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
SKILL_ROOT = os.path.dirname(SCRIPT_DIR)
SENTINEL = os.path.join(SKILL_ROOT, ".artifacts", "heartbeat.sentinel")
LOG_PATH = os.path.join(SKILL_ROOT, ".artifacts", "heartbeat.log")
CLI_PATH = os.path.join(SCRIPT_DIR, "ros2_cli.py")


def write_log(entry):
    os.makedirs(os.path.dirname(LOG_PATH), exist_ok=True)
    with open(LOG_PATH, "a", encoding="utf-8") as f:
        f.write(json.dumps(entry) + "\n")


def fire_estop():
    cmd = ["python3", CLI_PATH, "estop"]
    proc = subprocess.run(cmd, capture_output=True, text=True)
    return {
        "returncode": proc.returncode,
        "stdout": proc.stdout.strip(),
        "stderr": proc.stderr.strip(),
    }


def main():
    p = argparse.ArgumentParser(description="ros2-skill heartbeat watchdog")
    p.add_argument("--timeout", type=int, default=5)
    p.add_argument("--poll", type=int, default=1)
    p.add_argument("--on-timeout", choices=["estop", "log"], default="estop")
    args = p.parse_args()

    os.makedirs(os.path.dirname(SENTINEL), exist_ok=True)
    if not os.path.exists(SENTINEL):
        with open(SENTINEL, "w", encoding="utf-8") as f:
            f.write(json.dumps({"created_at": time.time()}) + "\n")

    while True:
        try:
            mtime = os.path.getmtime(SENTINEL)
        except OSError:
            mtime = 0.0

        now = time.time()
        age = now - mtime
        if age > args.timeout:
            event = {
                "watchdog_fired": True,
                "fired_at": now,
                "sentinel_age_s": round(age, 3),
                "timeout_s": int(args.timeout),
                "action": args.on_timeout,
            }
            if args.on_timeout == "estop":
                event["estop"] = fire_estop()
            write_log(event)
            break

        time.sleep(max(1, int(args.poll)))


if __name__ == "__main__":
    main()
