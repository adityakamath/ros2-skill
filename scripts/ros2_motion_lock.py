#!/usr/bin/env python3
"""One motion command at a time (RULES-MOTION Rule 23, enforced in code).

Motion commands take a lock file holding the owner's pid. A second motion command refuses to start
while the owner is alive; ``estop`` interrupts the owner (SIGINT, so its ``finally`` zero-velocity
burst runs) and clears the lock. A dead owner's lock is ignored.
"""

import json
import os
import signal
import time
from contextlib import contextmanager
from pathlib import Path

LOCK_PATH = Path(__file__).resolve().parent.parent / ".artifacts" / "motion.lock"

# (command, subcommand) pairs that drive the robot and must not overlap.
MOTION_COMMANDS = frozenset({
    ("topics", "publish-until"), ("topics", "publish-sequence"),
    ("nav2", "go"), ("nav2", "go-waypoints"), ("nav2", "rotate"), ("nav2", "move-timed"),
})


def is_motion(command, subcommand, args=None):
    """True for commands that drive the robot (``nav2 place go`` included; other place subcommands don't)."""
    if (command, subcommand) in MOTION_COMMANDS:
        return True
    return (command, subcommand) == ("nav2", "place") and getattr(args, "place_subcommand", None) == "go"


class MotionBusy(Exception):
    def __init__(self, owner):
        self.owner = owner
        super().__init__(f"another motion command is running: {owner.get('command')} (pid {owner.get('pid')})")


def _alive(pid):
    try:
        os.kill(pid, 0)
    except OSError:
        return False
    try:  # guard against pid reuse: the owner must still be a ros2 skill process
        return b"ros2_cli" in Path(f"/proc/{pid}/cmdline").read_bytes()
    except OSError:
        return True


def current_owner():
    """Return the live owner's lock info, or None."""
    try:
        info = json.loads(LOCK_PATH.read_text(encoding="utf-8"))
    except (OSError, ValueError):
        return None
    pid = info.get("pid")
    return info if isinstance(pid, int) and pid != os.getpid() and _alive(pid) else None


@contextmanager
def motion_lock(command):
    owner = current_owner()
    if owner:
        raise MotionBusy(owner)
    LOCK_PATH.parent.mkdir(parents=True, exist_ok=True)
    LOCK_PATH.write_text(json.dumps({"pid": os.getpid(), "command": command, "started": time.time()}), encoding="utf-8")
    try:
        yield
    finally:
        try:
            if json.loads(LOCK_PATH.read_text(encoding="utf-8")).get("pid") == os.getpid():
                LOCK_PATH.unlink()
        except (OSError, ValueError):
            pass


def interrupt_owner():
    """Used by estop: SIGINT the running motion command (its finally block zeroes velocity) and clear the lock.

    Returns the interrupted owner's info, or None.
    """
    owner = current_owner()
    if owner:
        try:
            os.kill(owner["pid"], signal.SIGINT)
        except OSError:
            pass
    try:
        LOCK_PATH.unlink()
    except OSError:
        pass
    return owner
