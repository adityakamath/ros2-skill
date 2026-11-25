#!/usr/bin/env python3
"""Motion lock: one motion command at a time; estop interrupts the owner."""

import json
import os
import signal
import subprocess
import sys
import tempfile
import time
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))

import ros2_motion_lock as ml


class TestMotionLock(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        patcher = patch.object(ml, "LOCK_PATH", Path(self.tmp.name) / "motion.lock")
        patcher.start()
        self.addCleanup(patcher.stop)
        self.addCleanup(self.tmp.cleanup)

    def _foreign_owner(self):
        """A live process whose cmdline looks like a skill command (the lock checks for 'ros2_cli')."""
        proc = subprocess.Popen([sys.executable, "-c", "import time; time.sleep(30)", "ros2_cli"])
        self.addCleanup(lambda: (proc.kill(), proc.wait()))
        ml.LOCK_PATH.write_text(json.dumps({"pid": proc.pid, "command": "topics publish-until"}))
        return proc

    def test_lock_is_taken_and_released(self):
        with ml.motion_lock("nav2 go"):
            self.assertEqual(json.loads(ml.LOCK_PATH.read_text())["pid"], os.getpid())
        self.assertFalse(ml.LOCK_PATH.exists())

    def test_second_motion_is_refused_while_owner_alive(self):
        proc = self._foreign_owner()
        with self.assertRaises(ml.MotionBusy) as ctx:
            with ml.motion_lock("nav2 go"):
                self.fail("must not run")
        self.assertEqual(ctx.exception.owner["pid"], proc.pid)

    def test_dead_owner_lock_is_ignored(self):
        ml.LOCK_PATH.write_text(json.dumps({"pid": 2 ** 22 + 12345, "command": "x"}))
        with ml.motion_lock("nav2 go"):
            pass

    def test_lock_released_when_command_raises(self):
        with self.assertRaises(RuntimeError):
            with ml.motion_lock("nav2 go"):
                raise RuntimeError("boom")
        self.assertFalse(ml.LOCK_PATH.exists())

    def test_estop_interrupts_owner_and_clears_lock(self):
        proc = self._foreign_owner()
        owner = ml.interrupt_owner()
        self.assertEqual(owner["pid"], proc.pid)
        self.assertFalse(ml.LOCK_PATH.exists())
        deadline = time.time() + 5
        while proc.poll() is None and time.time() < deadline:
            time.sleep(0.05)
        self.assertEqual(proc.returncode, -signal.SIGINT.value if proc.returncode < 0 else proc.returncode)
        self.assertIsNotNone(proc.returncode)

    def test_which_commands_count_as_motion(self):
        self.assertTrue(ml.is_motion("topics", "publish-until"))
        self.assertTrue(ml.is_motion("nav2", "place", SimpleNamespace(place_subcommand="go")))
        self.assertFalse(ml.is_motion("nav2", "place", SimpleNamespace(place_subcommand="list")))
        self.assertFalse(ml.is_motion("topics", "subscribe"))
        self.assertFalse(ml.is_motion("estop", None))


if __name__ == "__main__":
    unittest.main()
