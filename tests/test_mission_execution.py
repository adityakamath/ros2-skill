#!/usr/bin/env python3
"""Unit tests for mission execution - pause/resume/cancel semantics."""

import json
import os
import tempfile
import time
import unittest
from pathlib import Path
from unittest.mock import Mock, patch, MagicMock

import sys
sys.path.insert(0, str(Path(__file__).parent.parent / "scripts"))

import ros2_mission


class TestMissionState(unittest.TestCase):
    """Test mission state file management."""

    def setUp(self):
        self.test_dir = tempfile.mkdtemp()
        self.old_mission_dir = ros2_mission._MISSION_DIR
        ros2_mission._MISSION_DIR = Path(self.test_dir) / "missions"

    def tearDown(self):
        import shutil
        shutil.rmtree(self.test_dir, ignore_errors=True)
        ros2_mission._MISSION_DIR = self.old_mission_dir

    def test_mission_path_sanitization(self):
        """Test that mission IDs are properly sanitized for filenames."""
        path1 = ros2_mission._mission_path("simple_mission")
        self.assertEqual(path1.name, "simple_mission.json")
        
        path2 = ros2_mission._mission_path("mission with spaces")
        self.assertEqual(path2.name, "missionwithspaces.json")
        
        path3 = ros2_mission._mission_path("mission/../escape")
        self.assertNotIn("..", str(path3))

    def test_save_and_load_state(self):
        """Test saving and loading mission state."""
        state = {
            "mission_id": "test_mission",
            "status": "pending",
            "steps": [{"type": "go", "x": 1.0, "y": 2.0}],
        }
        
        path = ros2_mission.save_state(state)
        self.assertTrue(path.exists())
        
        loaded = ros2_mission.load_state("test_mission")
        self.assertIsNotNone(loaded)
        self.assertEqual(loaded["mission_id"], "test_mission")
        self.assertEqual(loaded["status"], "pending")
        self.assertIn("updated_at", loaded)

    def test_load_nonexistent_mission(self):
        """Test loading a mission that doesn't exist."""
        result = ros2_mission.load_state("nonexistent")
        self.assertIsNone(result)

    def test_load_corrupted_state(self):
        """Test loading a corrupted mission state file."""
        mission_id = "corrupted_mission"
        path = ros2_mission._mission_path(mission_id)
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text("{ invalid json }", encoding="utf-8")
        
        result = ros2_mission.load_state(mission_id)
        self.assertIsNone(result)


class TestMissionHelpers(unittest.TestCase):
    """Test mission helper functions."""

    def test_is_success_with_success_field(self):
        """Test _is_success with explicit success field."""
        self.assertTrue(ros2_mission._is_success({"success": True}))
        self.assertFalse(ros2_mission._is_success({"success": False}))

    def test_is_success_with_error(self):
        """Test _is_success with error field."""
        self.assertFalse(ros2_mission._is_success({"error": "something broke"}))

    def test_is_success_with_blocked(self):
        """Test _is_success with blocked field."""
        self.assertFalse(ros2_mission._is_success({"blocked": True}))
        self.assertTrue(ros2_mission._is_success({"blocked": False, "result": "ok"}))

    def test_is_success_with_empty_dict(self):
        """Test _is_success with empty or missing indicators."""
        self.assertTrue(ros2_mission._is_success({}))
        self.assertTrue(ros2_mission._is_success({"result": "completed"}))

    def test_build_step_args(self):
        """Test building step arguments from step dict."""
        step = {"type": "go", "x": 1.5, "y": -2.0, "yaw": 90}
        step_type, args = ros2_mission._build_step_args(step)
        
        self.assertEqual(step_type, "go")
        self.assertEqual(args.x, 1.5)
        self.assertEqual(args.y, -2.0)
        self.assertEqual(args.yaw, 90)
        self.assertFalse(hasattr(args, "type"))

    def test_build_step_args_type_normalization(self):
        """Test that step types are normalized to lowercase."""
        step1 = {"type": "GO", "x": 1.0}
        self.assertEqual(ros2_mission._build_step_args(step1)[0], "go")
        
        step2 = {"type": " Rotate ", "degrees": 90}
        self.assertEqual(ros2_mission._build_step_args(step2)[0], "rotate")


class TestMissionExecution(unittest.TestCase):
    """Test mission execution logic."""

    def setUp(self):
        self.test_dir = tempfile.mkdtemp()
        self.old_mission_dir = ros2_mission._MISSION_DIR
        ros2_mission._MISSION_DIR = Path(self.test_dir) / "missions"

    def tearDown(self):
        import shutil
        shutil.rmtree(self.test_dir, ignore_errors=True)
        ros2_mission._MISSION_DIR = self.old_mission_dir

    @patch('ros2_mission._step_handlers')
    def test_successful_mission_execution(self, mock_handlers):
        """Test a mission that completes successfully."""
        # Create initial state
        state = {
            "mission_id": "test_success",
            "status": "pending",
            "steps": [
                {"type": "go", "x": 1.0, "y": 0.0},
                {"type": "rotate", "degrees": 90},
            ],
        }
        ros2_mission.save_state(state)
        
        # Mock handlers to return success
        mock_go = Mock(return_value=None)
        mock_rotate = Mock(return_value=None)
        mock_handlers.return_value = {
            "go": mock_go,
            "rotate": mock_rotate,
        }
        
        # Mock _capture_output to return success results
        with patch('ros2_mission._capture_output') as mock_capture:
            mock_capture.side_effect = [
                {"success": True},
                {"success": True},
            ]
            
            result = ros2_mission.run_mission("test_success")
        
        # Verify execution
        self.assertEqual(result, 0)
        
        # Check final state
        final_state = ros2_mission.load_state("test_success")
        self.assertEqual(final_state["status"], "completed")
        self.assertEqual(len(final_state.get("history", [])), 2)
        self.assertTrue(all(h["ok"] for h in final_state["history"]))

    @patch('ros2_mission._step_handlers')
    def test_mission_fails_on_step_error(self, mock_handlers):
        """Test that a mission fails when a step returns an error."""
        state = {
            "mission_id": "test_failure",
            "status": "pending",
            "steps": [
                {"type": "go", "x": 1.0, "y": 0.0},
                {"type": "go", "x": 100.0, "y": 100.0},  # This will fail
            ],
        }
        ros2_mission.save_state(state)
        
        mock_go = Mock()
        mock_handlers.return_value = {"go": mock_go}
        
        with patch('ros2_mission._capture_output') as mock_capture:
            mock_capture.side_effect = [
                {"success": True},
                {"error": "Goal aborted"},
            ]
            
            result = ros2_mission.run_mission("test_failure")
        
        # Should return 1 for failure
        self.assertEqual(result, 1)
        
        # Check state
        final_state = ros2_mission.load_state("test_failure")
        self.assertEqual(final_state["status"], "failed")
        self.assertIn("failure_reason", final_state)
        self.assertEqual(len(final_state.get("history", [])), 2)
        self.assertTrue(final_state["history"][0]["ok"])
        self.assertFalse(final_state["history"][1]["ok"])

    @patch('ros2_mission._step_handlers')
    def test_mission_with_unknown_step_type(self, mock_handlers):
        """Test that mission fails with unknown step type."""
        state = {
            "mission_id": "test_unknown",
            "status": "pending",
            "steps": [{"type": "invalid_step"}],
        }
        ros2_mission.save_state(state)
        
        mock_handlers.return_value = {}
        
        result = ros2_mission.run_mission("test_unknown")
        
        self.assertEqual(result, 1)
        final_state = ros2_mission.load_state("test_unknown")
        self.assertEqual(final_state["status"], "failed")
        self.assertIn("unknown step type", final_state.get("failure_reason", ""))


class TestMissionPauseResume(unittest.TestCase):
    """Test mission pause/resume functionality."""

    def setUp(self):
        self.test_dir = tempfile.mkdtemp()
        self.old_mission_dir = ros2_mission._MISSION_DIR
        ros2_mission._MISSION_DIR = Path(self.test_dir) / "missions"

    def tearDown(self):
        import shutil
        shutil.rmtree(self.test_dir, ignore_errors=True)
        ros2_mission._MISSION_DIR = self.old_mission_dir

    def test_wait_for_resume_immediate_return(self):
        """Test _wait_for_resume returns immediately when not paused."""
        state = {
            "mission_id": "test_running",
            "status": "running",
        }
        ros2_mission.save_state(state)
        
        result = ros2_mission._wait_for_resume("test_running")
        self.assertIsNotNone(result)
        self.assertEqual(result["status"], "running")

    def test_wait_for_resume_on_cancel(self):
        """Test _wait_for_resume returns state when status is canceling."""
        state = {
            "mission_id": "test_cancel",
            "status": "canceling",
        }
        ros2_mission.save_state(state)
        
        result = ros2_mission._wait_for_resume("test_cancel")
        self.assertIsNotNone(result)
        self.assertEqual(result["status"], "canceling")

    def test_wait_for_resume_vanished_state(self):
        """Test _wait_for_resume handles vanished state file."""
        # Don't create a state file
        with patch('ros2_mission._mission_fail_safe') as mock_fail_safe:
            result = ros2_mission._wait_for_resume("nonexistent")
        
        self.assertIsNone(result)
        mock_fail_safe.assert_called_once()

    @patch('time.sleep')
    def test_wait_for_resume_paused_then_resumed(self, mock_sleep):
        """Test _wait_for_resume waits while paused then returns on resume."""
        state = {
            "mission_id": "test_pause_resume",
            "status": "paused",
        }
        ros2_mission.save_state(state)
        
        call_count = [0]
        original_load = ros2_mission.load_state
        
        def mock_load(mission_id):
            call_count[0] += 1
            if call_count[0] <= 2:
                # Return paused for first 2 calls
                return {"mission_id": mission_id, "status": "paused"}
            else:
                # Then return running
                return {"mission_id": mission_id, "status": "running"}
        
        with patch('ros2_mission.load_state', side_effect=mock_load):
            result = ros2_mission._wait_for_resume("test_pause_resume")
        
        self.assertIsNotNone(result)
        self.assertEqual(result["status"], "running")
        # Should have slept at least twice (once for each paused check)
        self.assertGreaterEqual(mock_sleep.call_count, 2)


class TestMissionFailSafe(unittest.TestCase):
    """Test mission fail-safe behavior."""

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
    def test_fail_safe_cancels_nav2(self, mock_capture, mock_cancel):
        """Test that fail-safe calls nav2 cancel."""
        state = {
            "mission_id": "test_failsafe",
            "status": "running",
        }
        ros2_mission.save_state(state)
        
        mock_capture.return_value = {"cancelled": True}
        
        ros2_mission._mission_fail_safe("test_failsafe", "test reason")
        
        # Verify nav2 cancel was attempted
        mock_capture.assert_called_once()

    @patch('ros2_nav2.cmd_nav2_cancel')
    @patch('ros2_mission._capture_output')
    def test_fail_safe_updates_state(self, mock_capture, mock_cancel):
        """Test that fail-safe updates mission state to failed."""
        state = {
            "mission_id": "test_failsafe_state",
            "status": "running",
        }
        ros2_mission.save_state(state)
        
        mock_capture.return_value = {}
        
        ros2_mission._mission_fail_safe("test_failsafe_state", "connection lost")
        
        final_state = ros2_mission.load_state("test_failsafe_state")
        self.assertEqual(final_state["status"], "failed")
        self.assertEqual(final_state["failure_reason"], "connection lost")

    @patch('ros2_nav2.cmd_nav2_cancel')
    @patch('ros2_mission._capture_output')
    def test_fail_safe_handles_cancel_exception(self, mock_capture, mock_cancel):
        """Test that fail-safe handles exceptions during nav2 cancel gracefully."""
        state = {
            "mission_id": "test_failsafe_exception",
            "status": "running",
        }
        ros2_mission.save_state(state)
        
        # Make cancel raise an exception
        mock_capture.side_effect = Exception("Nav2 not available")
        
        # Should not raise, just handle gracefully
        ros2_mission._mission_fail_safe("test_failsafe_exception", "test")
        
        # State should still be updated
        final_state = ros2_mission.load_state("test_failsafe_exception")
        self.assertEqual(final_state["status"], "failed")


class TestMissionCancel(unittest.TestCase):
    """Test mission cancellation during execution."""

    def setUp(self):
        self.test_dir = tempfile.mkdtemp()
        self.old_mission_dir = ros2_mission._MISSION_DIR
        ros2_mission._MISSION_DIR = Path(self.test_dir) / "missions"

    def tearDown(self):
        import shutil
        shutil.rmtree(self.test_dir, ignore_errors=True)
        ros2_mission._MISSION_DIR = self.old_mission_dir

    @patch('ros2_mission._step_handlers')
    @patch('ros2_mission._mission_fail_safe')
    def test_mission_canceled_mid_execution(self, mock_fail_safe, mock_handlers):
        """Test that mission detects cancel status and exits."""
        state = {
            "mission_id": "test_cancel_mid",
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
        original_wait = ros2_mission._wait_for_resume
        
        def mock_wait(mission_id):
            call_count[0] += 1
            if call_count[0] == 1:
                # First call: return running
                return {"mission_id": mission_id, "status": "running"}
            else:
                # Second call: return canceling
                return {"mission_id": mission_id, "status": "canceling"}
        
        with patch('ros2_mission._wait_for_resume', side_effect=mock_wait):
            with patch('ros2_mission._capture_output', return_value={"success": True}):
                result = ros2_mission.run_mission("test_cancel_mid")
        
        # Should return 2 for canceled
        self.assertEqual(result, 2)
        mock_fail_safe.assert_called_once()


if __name__ == '__main__':
    unittest.main()
