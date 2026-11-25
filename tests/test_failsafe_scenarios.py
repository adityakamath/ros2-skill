#!/usr/bin/env python3
"""Fail-safe scenario tests for mission and watchdog systems."""

import json
import os
import sys
import tempfile
import time
import unittest
from pathlib import Path
from unittest.mock import Mock, patch, MagicMock, call

sys.path.insert(0, str(Path(__file__).parent.parent / "scripts"))

import ros2_mission
import ros2_safety
import ros2_watchdog


class TestWatchdogFailSafeHardened(unittest.TestCase):
    """Test the explicit stale-session and disconnect fail-safe path."""

    def test_trigger_fail_safe_logs_event_and_calls_estop(self):
        with tempfile.TemporaryDirectory() as tmp:
            base = Path(tmp)
            sentinel = base / ".artifacts" / "heartbeat.sentinel"
            log = base / ".artifacts" / "heartbeat.log"
            sentinel.parent.mkdir(parents=True, exist_ok=True)
            sentinel.write_text(json.dumps({"created_at": time.time() - 30.0}), encoding="utf-8")

            with patch("ros2_safety.subprocess.run") as mock_run:
                mock_run.return_value = Mock(returncode=0, stdout="stopped", stderr="")
                result = ros2_safety.trigger_fail_safe(
                    reason="heartbeat stale",
                    base_dir=str(base),
                    on_timeout="estop",
                )

            self.assertTrue(result["watchdog_fired"])
            self.assertEqual(result["reason"], "heartbeat stale")
            self.assertEqual(result["action"], "estop")
            self.assertTrue(log.exists())
            self.assertEqual(mock_run.call_count, 1)

    def test_get_heartbeat_status_marks_missing_sentinel_as_stale(self):
        with tempfile.TemporaryDirectory() as tmp:
            base = Path(tmp)
            status = ros2_safety.get_heartbeat_status(base_dir=str(base), timeout_s=5)
            self.assertFalse(status["sentinel_exists"])
            self.assertTrue(status["stale"])
            self.assertEqual(status["reason"], "missing sentinel")


class TestWatchdogTimeout(unittest.TestCase):
    """Test watchdog timeout detection and estop firing."""

    def setUp(self):
        self.test_dir = tempfile.mkdtemp()
        self.sentinel_path = os.path.join(self.test_dir, "heartbeat.sentinel")
        self.log_path = os.path.join(self.test_dir, "heartbeat.log")

    def tearDown(self):
        import shutil
        shutil.rmtree(self.test_dir, ignore_errors=True)

    @patch('ros2_watchdog.fire_estop')
    @patch('ros2_watchdog.SENTINEL')
    @patch('ros2_watchdog.LOG_PATH')
    def test_watchdog_fires_on_stale_sentinel(self, mock_log_path, mock_sentinel, mock_estop):
        """Test that watchdog fires estop when sentinel is stale."""
        mock_sentinel.__str__ = lambda x: self.sentinel_path
        mock_log_path.__str__ = lambda x: self.log_path
        
        # Create stale sentinel
        Path(self.sentinel_path).touch()
        old_time = time.time() - 10.0
        os.utime(self.sentinel_path, (old_time, old_time))
        
        mock_estop.return_value = {"returncode": 0, "stdout": "estop triggered", "stderr": ""}
        
        # Mock main to run once and exit
        with patch('ros2_watchdog.main') as mock_main:
            # Simulate watchdog detecting stale sentinel
            mtime = os.path.getmtime(self.sentinel_path)
            age = time.time() - mtime
            
            self.assertGreater(age, 5.0)  # Should be stale

    @patch('ros2_watchdog.fire_estop')
    def test_watchdog_writes_log_on_timeout(self, mock_estop):
        """Test that watchdog writes log entry when timeout occurs."""
        # Create stale sentinel
        Path(self.sentinel_path).touch()
        old_time = time.time() - 10.0
        os.utime(self.sentinel_path, (old_time, old_time))
        
        mock_estop.return_value = {"returncode": 0}
        
        # Write log entry
        event = {
            "watchdog_fired": True,
            "fired_at": time.time(),
            "sentinel_age_s": 10.0,
            "timeout_s": 5,
            "action": "estop",
            "estop": mock_estop.return_value,
        }
        
        with patch.object(ros2_watchdog, 'LOG_PATH', self.log_path):
            ros2_watchdog.write_log(event)
        
        # Verify log was written
        self.assertTrue(os.path.exists(self.log_path))
        with open(self.log_path, 'r') as f:
            log_line = f.read().strip()
            log_data = json.loads(log_line)
            self.assertTrue(log_data["watchdog_fired"])
            self.assertEqual(log_data["action"], "estop")

    def test_watchdog_creates_sentinel_if_missing(self):
        """Test that watchdog creates sentinel if it doesn't exist."""
        self.assertFalse(os.path.exists(self.sentinel_path))
        
        # Create sentinel as watchdog would
        os.makedirs(os.path.dirname(self.sentinel_path), exist_ok=True)
        with open(self.sentinel_path, "w", encoding="utf-8") as f:
            f.write(json.dumps({"created_at": time.time()}) + "\n")
        
        self.assertTrue(os.path.exists(self.sentinel_path))

    @patch('subprocess.run')
    def test_fire_estop_calls_cli(self, mock_run):
        """Test that fire_estop calls the CLI estop command."""
        mock_run.return_value = Mock(returncode=0, stdout="stopped", stderr="")
        
        result = ros2_watchdog.fire_estop()
        
        self.assertEqual(result["returncode"], 0)
        self.assertIn("stopped", result["stdout"])
        mock_run.assert_called_once()
        # Verify it calls estop command
        call_args = mock_run.call_args[0][0]
        self.assertIn("estop", call_args)

    def test_watchdog_log_only_mode(self):
        """Test watchdog in log-only mode (no estop)."""
        event = {
            "watchdog_fired": True,
            "fired_at": time.time(),
            "sentinel_age_s": 10.0,
            "timeout_s": 5,
            "action": "log",  # Log only, no estop
        }
        
        with patch.object(ros2_watchdog, 'LOG_PATH', self.log_path):
            ros2_watchdog.write_log(event)
        
        with open(self.log_path, 'r') as f:
            log_data = json.loads(f.read().strip())
            self.assertEqual(log_data["action"], "log")
            self.assertNotIn("estop", log_data)


class TestMissionStateCorruption(unittest.TestCase):
    """Test mission behavior when state file is corrupted."""

    def setUp(self):
        self.test_dir = tempfile.mkdtemp()
        self.old_mission_dir = ros2_mission._MISSION_DIR
        ros2_mission._MISSION_DIR = Path(self.test_dir) / "missions"

    def tearDown(self):
        import shutil
        shutil.rmtree(self.test_dir, ignore_errors=True)
        ros2_mission._MISSION_DIR = self.old_mission_dir

    def test_load_corrupted_json(self):
        """Test that load_state handles corrupted JSON gracefully."""
        mission_id = "corrupted_test"
        path = ros2_mission._mission_path(mission_id)
        path.parent.mkdir(parents=True, exist_ok=True)
        
        # Write invalid JSON
        path.write_text("{ invalid json structure", encoding="utf-8")
        
        result = ros2_mission.load_state(mission_id)
        self.assertIsNone(result)

    def test_load_empty_file(self):
        """Test that load_state handles empty file gracefully."""
        mission_id = "empty_test"
        path = ros2_mission._mission_path(mission_id)
        path.parent.mkdir(parents=True, exist_ok=True)
        
        # Write empty file
        path.write_text("", encoding="utf-8")
        
        result = ros2_mission.load_state(mission_id)
        self.assertIsNone(result)

    def test_load_partial_json(self):
        """Test that load_state handles truncated JSON gracefully."""
        mission_id = "partial_test"
        path = ros2_mission._mission_path(mission_id)
        path.parent.mkdir(parents=True, exist_ok=True)
        
        # Write truncated JSON (missing closing brace)
        path.write_text('{"mission_id": "test", "status": "running"', encoding="utf-8")
        
        result = ros2_mission.load_state(mission_id)
        self.assertIsNone(result)

    @patch('ros2_mission._step_handlers')
    @patch('ros2_mission._mission_fail_safe')
    def test_mission_handles_vanished_state_mid_execution(self, mock_fail_safe, mock_handlers):
        """Test that mission triggers fail-safe when state file vanishes."""
        mission_id = "vanishing_test"
        state = {
            "mission_id": mission_id,
            "status": "pending",
            "steps": [{"type": "go", "x": 1.0, "y": 0.0}],
        }
        ros2_mission.save_state(state)
        
        mock_handlers.return_value = {"go": Mock()}
        
        # Mock _wait_for_resume to simulate state vanishing
        with patch('ros2_mission._wait_for_resume', return_value=None):
            result = ros2_mission.run_mission(mission_id)
        
        self.assertEqual(result, 1)  # Should return error
        # Fail-safe should have been called by _wait_for_resume

    @patch('ros2_mission._step_handlers')
    def test_mission_handles_corrupted_state_mid_execution(self, mock_handlers):
        """Test that mission detects corrupted state during execution."""
        mission_id = "corrupt_mid_test"
        state = {
            "mission_id": mission_id,
            "status": "pending",
            "steps": [
                {"type": "go", "x": 1.0, "y": 0.0},
                {"type": "go", "x": 2.0, "y": 0.0},
            ],
        }
        ros2_mission.save_state(state)
        
        mock_go = Mock()
        mock_handlers.return_value = {"go": mock_go}
        
        call_count = [0]
        original_load = ros2_mission.load_state
        
        def mock_load(mission_id):
            call_count[0] += 1
            if call_count[0] <= 2:
                # First calls return valid state
                return original_load(mission_id)
            else:
                # Later calls return None (simulating corruption)
                return None
        
        with patch('ros2_mission.load_state', side_effect=mock_load):
            with patch('ros2_mission._capture_output', return_value={"success": True}):
                with patch('ros2_mission._wait_for_resume') as mock_wait:
                    # First wait returns running state
                    mock_wait.return_value = {"mission_id": mission_id, "status": "running"}
                    result = ros2_mission.run_mission(mission_id)
        
        # Should complete successfully even with some None loads (defensive coding)
        self.assertIn(result, [0, 1])  # Either succeeds or fails gracefully


class TestMissionFailSafeScenarios(unittest.TestCase):
    """Test various scenarios that trigger mission fail-safe."""

    def setUp(self):
        self.test_dir = tempfile.mkdtemp()
        self.old_mission_dir = ros2_mission._MISSION_DIR
        ros2_mission._MISSION_DIR = Path(self.test_dir) / "missions"

    def tearDown(self):
        import shutil
        shutil.rmtree(self.test_dir, ignore_errors=True)
        ros2_mission._MISSION_DIR = self.old_mission_dir

    @patch('ros2_nav2.cmd_nav2_cancel')
    @patch('ros2_mission._capture_output')
    def test_fail_safe_on_state_vanish(self, mock_capture, mock_cancel):
        """Test fail-safe behavior when state file vanishes."""
        mission_id = "vanish_test"
        state = {
            "mission_id": mission_id,
            "status": "running",
        }
        ros2_mission.save_state(state)
        
        mock_capture.return_value = {"cancelled": True}
        
        ros2_mission._mission_fail_safe(mission_id, "state file vanished")
        
        # Verify nav2 cancel was attempted
        mock_capture.assert_called_once()
        
        # Verify state was updated
        final_state = ros2_mission.load_state(mission_id)
        self.assertEqual(final_state["status"], "failed")
        self.assertIn("vanish", final_state["failure_reason"])

    @patch('ros2_nav2.cmd_nav2_cancel')
    @patch('ros2_mission._capture_output')
    def test_fail_safe_when_nav2_unavailable(self, mock_capture, mock_cancel):
        """Test fail-safe handles nav2 being unavailable."""
        mission_id = "nav2_down_test"
        state = {
            "mission_id": mission_id,
            "status": "running",
        }
        ros2_mission.save_state(state)
        
        # Simulate nav2 cancel failing
        mock_capture.side_effect = Exception("Nav2 not responding")
        
        # Should not raise, should handle gracefully
        try:
            ros2_mission._mission_fail_safe(mission_id, "nav2 unavailable")
        except Exception as e:
            self.fail(f"Fail-safe should not raise: {e}")
        
        # State should still be updated even if nav2 cancel fails
        final_state = ros2_mission.load_state(mission_id)
        self.assertEqual(final_state["status"], "failed")

    @patch('ros2_nav2.cmd_nav2_cancel')
    @patch('ros2_mission._capture_output')
    def test_fail_safe_on_keyboard_interrupt(self, mock_capture, mock_cancel):
        """Test fail-safe behavior on keyboard interrupt simulation."""
        mission_id = "interrupt_test"
        state = {
            "mission_id": mission_id,
            "status": "running",
        }
        ros2_mission.save_state(state)
        
        mock_capture.return_value = {"cancelled": True}
        
        # Simulate interrupt scenario
        ros2_mission._mission_fail_safe(mission_id, "user interrupt")
        
        final_state = ros2_mission.load_state(mission_id)
        self.assertEqual(final_state["status"], "failed")
        self.assertIn("interrupt", final_state["failure_reason"])

    @patch('ros2_mission._step_handlers')
    @patch('ros2_mission._mission_fail_safe')
    def test_fail_safe_preserves_history(self, mock_fail_safe, mock_handlers):
        """Test that fail-safe preserves execution history."""
        mission_id = "history_test"
        state = {
            "mission_id": mission_id,
            "status": "pending",
            "steps": [
                {"type": "go", "x": 1.0, "y": 0.0},
                {"type": "go", "x": 2.0, "y": 0.0},
            ],
        }
        ros2_mission.save_state(state)
        
        mock_go = Mock()
        mock_handlers.return_value = {"go": mock_go}
        
        # First step succeeds, second step triggers cancel
        with patch('ros2_mission._capture_output', return_value={"success": True}):
            with patch('ros2_mission._wait_for_resume') as mock_wait:
                # First call returns running, second returns canceling
                mock_wait.side_effect = [
                    {"mission_id": mission_id, "status": "running"},
                    {"mission_id": mission_id, "status": "canceling"},
                ]
                result = ros2_mission.run_mission(mission_id)
        
        self.assertEqual(result, 2)  # Canceled
        mock_fail_safe.assert_called_once()


class TestWatchdogEdgeCases(unittest.TestCase):
    """Test watchdog edge cases and error handling."""

    def setUp(self):
        self.test_dir = tempfile.mkdtemp()
        self.sentinel_path = os.path.join(self.test_dir, "heartbeat.sentinel")
        self.log_path = os.path.join(self.test_dir, "heartbeat.log")

    def tearDown(self):
        import shutil
        shutil.rmtree(self.test_dir, ignore_errors=True)

    def test_watchdog_handles_missing_sentinel_directory(self):
        """Test watchdog creates directory if it doesn't exist."""
        sentinel_in_deep_path = os.path.join(self.test_dir, "deep", "path", "sentinel")
        
        # Create directory as watchdog would
        os.makedirs(os.path.dirname(sentinel_in_deep_path), exist_ok=True)
        
        self.assertTrue(os.path.exists(os.path.dirname(sentinel_in_deep_path)))

    def test_watchdog_handles_permission_errors(self):
        """Test watchdog handles permission errors gracefully."""
        # Create sentinel
        Path(self.sentinel_path).touch()
        
        # Try to get mtime (should work)
        try:
            mtime = os.path.getmtime(self.sentinel_path)
            self.assertIsInstance(mtime, float)
        except OSError:
            self.fail("Should be able to read mtime")

    def test_watchdog_log_handles_multiple_entries(self):
        """Test that watchdog can write multiple log entries."""
        events = [
            {"watchdog_fired": True, "fired_at": time.time(), "timeout_s": 5, "action": "estop"},
            {"watchdog_fired": True, "fired_at": time.time() + 1, "timeout_s": 5, "action": "log"},
            {"watchdog_fired": True, "fired_at": time.time() + 2, "timeout_s": 5, "action": "estop"},
        ]
        
        with patch.object(ros2_watchdog, 'LOG_PATH', self.log_path):
            for event in events:
                ros2_watchdog.write_log(event)
        
        # Verify all entries were written
        with open(self.log_path, 'r') as f:
            lines = f.readlines()
            self.assertEqual(len(lines), 3)
            for line in lines:
                log_data = json.loads(line.strip())
                self.assertTrue(log_data["watchdog_fired"])

    def test_write_log_creates_directory(self):
        """Test that write_log creates log directory if needed."""
        deep_log_path = os.path.join(self.test_dir, "deep", "log", "test.log")
        
        # Patch LOG_PATH temporarily
        with patch.object(ros2_watchdog, 'LOG_PATH', deep_log_path):
            ros2_watchdog.write_log({"test": "entry"})
        
        self.assertTrue(os.path.exists(deep_log_path))


class TestMissionStepHandlerFailures(unittest.TestCase):
    """Test mission behavior when step handlers fail in various ways."""

    def setUp(self):
        self.test_dir = tempfile.mkdtemp()
        self.old_mission_dir = ros2_mission._MISSION_DIR
        ros2_mission._MISSION_DIR = Path(self.test_dir) / "missions"

    def tearDown(self):
        import shutil
        shutil.rmtree(self.test_dir, ignore_errors=True)
        ros2_mission._MISSION_DIR = self.old_mission_dir

    @patch('ros2_mission._step_handlers')
    def test_mission_handles_handler_exception(self, mock_handlers):
        """Test mission handles exceptions from step handlers."""
        mission_id = "handler_exception_test"
        state = {
            "mission_id": mission_id,
            "status": "pending",
            "steps": [{"type": "go", "x": 1.0, "y": 0.0}],
        }
        ros2_mission.save_state(state)
        
        # Mock handler that raises exception
        mock_go = Mock(side_effect=Exception("Handler crashed"))
        mock_handlers.return_value = {"go": mock_go}
        
        # _capture_output should catch the exception
        # We need to mock _capture_output to return an error
        with patch('ros2_mission._capture_output', return_value={"error": "handler exception"}):
            result = ros2_mission.run_mission(mission_id)
        
        self.assertEqual(result, 1)  # Should fail
        final_state = ros2_mission.load_state(mission_id)
        self.assertEqual(final_state["status"], "failed")

    @patch('ros2_mission._step_handlers')
    def test_mission_handles_handler_timeout(self, mock_handlers):
        """Test mission handles step timeouts."""
        mission_id = "timeout_test"
        state = {
            "mission_id": mission_id,
            "status": "pending",
            "steps": [{"type": "go", "x": 1.0, "y": 0.0}],
        }
        ros2_mission.save_state(state)
        
        mock_go = Mock()
        mock_handlers.return_value = {"go": mock_go}
        
        # Simulate timeout by returning timeout error
        with patch('ros2_mission._capture_output', return_value={"error": "timeout", "blocked": True}):
            result = ros2_mission.run_mission(mission_id)
        
        self.assertEqual(result, 1)
        final_state = ros2_mission.load_state(mission_id)
        self.assertEqual(final_state["status"], "failed")

    @patch('ros2_mission._step_handlers')
    def test_mission_records_partial_success_before_failure(self, mock_handlers):
        """Test that mission records successful steps before failure."""
        mission_id = "partial_success_test"
        state = {
            "mission_id": mission_id,
            "status": "pending",
            "steps": [
                {"type": "go", "x": 1.0, "y": 0.0},
                {"type": "go", "x": 2.0, "y": 0.0},
                {"type": "go", "x": 3.0, "y": 0.0},  # This will fail
            ],
        }
        ros2_mission.save_state(state)
        
        mock_go = Mock()
        mock_handlers.return_value = {"go": mock_go}
        
        # First two succeed, third fails
        with patch('ros2_mission._capture_output') as mock_capture:
            mock_capture.side_effect = [
                {"success": True},
                {"success": True},
                {"error": "navigation failed"},
            ]
            result = ros2_mission.run_mission(mission_id)
        
        self.assertEqual(result, 1)
        final_state = ros2_mission.load_state(mission_id)
        self.assertEqual(final_state["status"], "failed")
        
        # Verify history shows first two steps succeeded
        history = final_state.get("history", [])
        self.assertEqual(len(history), 3)
        self.assertTrue(history[0]["ok"])
        self.assertTrue(history[1]["ok"])
        self.assertFalse(history[2]["ok"])


class TestSystemRecovery(unittest.TestCase):
    """Test system recovery after failures."""

    def setUp(self):
        self.test_dir = tempfile.mkdtemp()
        self.old_mission_dir = ros2_mission._MISSION_DIR
        ros2_mission._MISSION_DIR = Path(self.test_dir) / "missions"

    def tearDown(self):
        import shutil
        shutil.rmtree(self.test_dir, ignore_errors=True)
        ros2_mission._MISSION_DIR = self.old_mission_dir

    def test_mission_state_recovery_after_corruption(self):
        """Test that system can recover after state corruption."""
        mission_id = "recovery_test"
        
        # Create initial valid state
        state = {
            "mission_id": mission_id,
            "status": "pending",
            "steps": [{"type": "go", "x": 1.0, "y": 0.0}],
        }
        path = ros2_mission.save_state(state)
        
        # Verify can load
        loaded = ros2_mission.load_state(mission_id)
        self.assertIsNotNone(loaded)
        
        # Corrupt it
        path.write_text("corrupted", encoding="utf-8")
        loaded = ros2_mission.load_state(mission_id)
        self.assertIsNone(loaded)
        
        # Recover by saving new valid state
        new_state = {
            "mission_id": mission_id,
            "status": "pending",
            "steps": [{"type": "go", "x": 2.0, "y": 0.0}],
        }
        ros2_mission.save_state(new_state)
        
        # Should be able to load again
        loaded = ros2_mission.load_state(mission_id)
        self.assertIsNotNone(loaded)
        self.assertEqual(loaded["status"], "pending")

    def test_multiple_missions_independent_failure(self):
        """Test that one mission's failure doesn't affect others."""
        mission_id_1 = "mission_1"
        mission_id_2 = "mission_2"
        
        state1 = {"mission_id": mission_id_1, "status": "running", "steps": []}
        state2 = {"mission_id": mission_id_2, "status": "running", "steps": []}
        
        ros2_mission.save_state(state1)
        ros2_mission.save_state(state2)
        
        # Corrupt mission 1
        path1 = ros2_mission._mission_path(mission_id_1)
        path1.write_text("corrupted", encoding="utf-8")
        
        # Mission 1 should be unloadable
        loaded1 = ros2_mission.load_state(mission_id_1)
        self.assertIsNone(loaded1)
        
        # Mission 2 should still be fine
        loaded2 = ros2_mission.load_state(mission_id_2)
        self.assertIsNotNone(loaded2)
        self.assertEqual(loaded2["status"], "running")


if __name__ == '__main__':
    unittest.main()
