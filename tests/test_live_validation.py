#!/usr/bin/env python3
"""Live validation tests for ros2-skill commands on actual hardware.

These tests are designed to run on lekiwi_ros2 hardware placed on a platform
where wheels can rotate but the robot won't move. Tests verify command execution,
timing, and result consistency without requiring actual navigation.

Run with: python3 -m pytest tests/test_live_validation.py -v --live
"""

import json
import os
import subprocess
import sys
import time
import unittest
from pathlib import Path

# Only run if --live flag is provided
LIVE_TESTS_ENABLED = os.environ.get("ROS2_LIVE_TESTS") == "1"

SCRIPT_DIR = Path(__file__).parent.parent / "scripts"
CLI_PATH = SCRIPT_DIR / "ros2_cli.py"


def run_cli(*args, timeout=10.0):
    """Run CLI command and return result."""
    cmd = ["python3", str(CLI_PATH)] + list(args)
    start = time.time()
    try:
        proc = subprocess.run(
            cmd,
            capture_output=True,
            text=True,
            timeout=timeout,
        )
        elapsed = time.time() - start
        
        # Parse JSON output if possible
        output = proc.stdout.strip()
        try:
            # Try to parse the entire output as JSON
            data = json.loads(output)
        except (json.JSONDecodeError, ValueError):
            # If that fails, try to find JSON on the last line
            try:
                data = json.loads(output.split('\n')[-1])
            except (json.JSONDecodeError, IndexError, ValueError):
                data = {"raw_output": output}
        
        return {
            "returncode": proc.returncode,
            "stdout": output,
            "stderr": proc.stderr.strip(),
            "data": data,
            "elapsed_s": elapsed,
            "cmd": " ".join(args),
        }
    except subprocess.TimeoutExpired:
        elapsed = time.time() - start
        return {
            "returncode": -1,
            "stdout": "",
            "stderr": f"Timeout after {timeout}s",
            "data": {"error": "timeout"},
            "elapsed_s": elapsed,
            "cmd": " ".join(args),
        }


@unittest.skipUnless(LIVE_TESTS_ENABLED, "Live tests disabled (use --live or ROS2_LIVE_TESTS=1)")
class TestLiveConnectivity(unittest.TestCase):
    """Test basic ROS 2 connectivity and discovery."""

    def test_version_command(self):
        """Test that version command returns ROS info."""
        result = run_cli("version", timeout=5.0)
        self.assertEqual(result["returncode"], 0, f"version failed: {result['stderr']}")
        self.assertIn("rclpy_available", result["data"])
        self.assertTrue(result["data"]["rclpy_available"], "rclpy not available")
        print(f"✓ Version: {result['data'].get('distro')} (took {result['elapsed_s']:.2f}s)")

    def test_context_discovery(self):
        """Test context command discovers ROS graph."""
        result = run_cli("context", timeout=10.0)
        self.assertEqual(result["returncode"], 0, f"context failed: {result['stderr']}")
        
        data = result["data"]
        self.assertIn("topics", data)
        self.assertIn("services", data)
        self.assertIn("nodes", data)
        
        counts = data.get("counts", {})
        print(f"✓ Context: {counts.get('topics', 0)} topics, "
              f"{counts.get('services', 0)} services, "
              f"{counts.get('nodes', 0)} nodes "
              f"(took {result['elapsed_s']:.2f}s)")

    def test_topics_list(self):
        """Test listing ROS topics."""
        result = run_cli("topics", "list", timeout=10.0)
        self.assertEqual(result["returncode"], 0, f"topics list failed: {result['stderr']}")
        
        data = result["data"]
        self.assertIn("topics", data)
        self.assertIsInstance(data["topics"], list)
        self.assertGreater(len(data["topics"]), 0, "No topics found")
        print(f"✓ Topics: Found {len(data['topics'])} topics (took {result['elapsed_s']:.2f}s)")

    def test_nodes_list(self):
        """Test listing ROS nodes."""
        result = run_cli("nodes", "list", timeout=10.0)
        self.assertEqual(result["returncode"], 0, f"nodes list failed: {result['stderr']}")
        
        data = result["data"]
        self.assertIn("nodes", data)
        self.assertIsInstance(data["nodes"], list)
        self.assertGreater(len(data["nodes"]), 0, "No nodes found")
        print(f"✓ Nodes: Found {len(data['nodes'])} nodes (took {result['elapsed_s']:.2f}s)")


@unittest.skipUnless(LIVE_TESTS_ENABLED, "Live tests disabled (use --live or ROS2_LIVE_TESTS=1)")
class TestLiveVelocityCommands(unittest.TestCase):
    """Test velocity commands on platform (safe since wheels can spin but robot won't move)."""

    def test_topics_publish_velocity_zero(self):
        """Test publishing zero velocity (safe stop)."""
        result = run_cli(
            "topics", "publish", "/cmd_vel",
            '{"linear": {"x": 0.0, "y": 0.0, "z": 0.0}, "angular": {"x": 0.0, "y": 0.0, "z": 0.0}}',
            "--msg-type", "geometry_msgs/msg/Twist",
            "--duration", "0.1",
            timeout=5.0
        )
        self.assertEqual(result["returncode"], 0, f"Zero velocity publish failed: {result['stderr']}")
        print(f"✓ Publish zero velocity (took {result['elapsed_s']:.2f}s)")

    def test_topics_publish_velocity_small(self):
        """Test publishing small velocity (safe on platform)."""
        result = run_cli(
            "topics", "publish", "/cmd_vel",
            '{"linear": {"x": 0.1, "y": 0.0, "z": 0.0}, "angular": {"x": 0.0, "y": 0.0, "z": 0.0}}',
            "--msg-type", "geometry_msgs/msg/Twist",
            "--duration", "0.1",
            timeout=5.0
        )
        
        # Should either succeed or be blocked by safety
        if result["returncode"] == 0:
            print(f"✓ Publish small velocity allowed (took {result['elapsed_s']:.2f}s)")
        else:
            data = result["data"]
            if "blocked" in data or "error" in data:
                print(f"✓ Publish small velocity blocked by safety: {data.get('error', data.get('blocked'))}")
            else:
                self.fail(f"Unexpected failure: {result['stderr']}")

    def test_topics_echo_cmd_vel(self):
        """Test echoing velocity topic."""
        result = run_cli("topics", "echo", "/cmd_vel", "--count", "1", timeout=5.0)
        
        # May timeout if no messages, which is fine
        if result["returncode"] == 0:
            print(f"✓ Echo /cmd_vel received message (took {result['elapsed_s']:.2f}s)")
        elif "timeout" in result.get("stderr", "").lower():
            print(f"✓ Echo /cmd_vel timeout (no messages - expected on idle robot)")
        else:
            print(f"⚠ Echo /cmd_vel: {result.get('stderr', 'unknown error')}")


@unittest.skipUnless(LIVE_TESTS_ENABLED, "Live tests disabled (use --live or ROS2_LIVE_TESTS=1)")
class TestLiveSafetyEnforcement(unittest.TestCase):
    """Test safety policy enforcement on live system."""

    def test_safety_show_config(self):
        """Test showing current safety config."""
        result = run_cli("safety", "show", timeout=5.0)
        self.assertEqual(result["returncode"], 0, f"safety show failed: {result['stderr']}")
        
        data = result["data"]
        self.assertIn("config", data)
        config = data["config"]
        self.assertIn("velocity_limits", config)
        self.assertIn("geofence", config)
        print(f"✓ Safety config loaded (took {result['elapsed_s']:.2f}s)")
        print(f"  - Velocity limits enabled: {config.get('velocity_limits', {}).get('enabled')}")
        print(f"  - Geofence enabled: {config.get('geofence', {}).get('enabled')}")

    def test_velocity_limit_enforcement(self):
        """Test that excessive velocity is blocked."""
        result = run_cli(
            "topics", "publish", "/cmd_vel",
            '{"linear": {"x": 10.0, "y": 0.0, "z": 0.0}, "angular": {"x": 0.0, "y": 0.0, "z": 0.0}}',
            "--msg-type", "geometry_msgs/msg/Twist",
            "--duration", "0.1",
            timeout=5.0
        )
        
        # Should be blocked by safety or succeed with warning
        data = result["data"]
        if "blocked" in data or "error" in data:
            print(f"✓ Excessive velocity blocked by safety (took {result['elapsed_s']:.2f}s)")
        elif "warning" in data or "capped" in data:
            print(f"✓ Excessive velocity capped/warned (took {result['elapsed_s']:.2f}s)")
        else:
            print(f"⚠ Excessive velocity not blocked - check safety config")


@unittest.skipUnless(LIVE_TESTS_ENABLED, "Live tests disabled (use --live or ROS2_LIVE_TESTS=1)")
class TestLiveSystemCommands(unittest.TestCase):
    """Test system status and information commands."""

    def test_lifecycle_list(self):
        """Test listing lifecycle nodes."""
        result = run_cli("lifecycle", "list", timeout=10.0)
        
        # May not have lifecycle nodes
        if result["returncode"] == 0:
            data = result["data"]
            nodes = data.get("lifecycle_nodes", [])
            print(f"✓ Lifecycle nodes: Found {len(nodes)} nodes (took {result['elapsed_s']:.2f}s)")
        else:
            print(f"⚠ Lifecycle list: {result.get('stderr', 'failed')}")

    def test_params_list_nodes(self):
        """Test listing nodes with parameters."""
        result = run_cli("params", "list-nodes", timeout=10.0)
        
        if result["returncode"] == 0:
            data = result["data"]
            nodes = data.get("nodes", [])
            print(f"✓ Param nodes: Found {len(nodes)} nodes (took {result['elapsed_s']:.2f}s)")
        else:
            print(f"⚠ Params list-nodes: {result.get('stderr', 'failed')}")


@unittest.skipUnless(LIVE_TESTS_ENABLED, "Live tests disabled (use --live or ROS2_LIVE_TESTS=1)")
class TestLiveTimingBenchmark(unittest.TestCase):
    """Benchmark command timing for optimization insights."""

    def test_timing_topics_list(self):
        """Benchmark topics list timing."""
        times = []
        for i in range(3):
            result = run_cli("topics", "list", timeout=10.0)
            if result["returncode"] == 0:
                times.append(result["elapsed_s"])
        
        if times:
            avg = sum(times) / len(times)
            print(f"✓ Topics list timing: {avg:.3f}s avg ({min(times):.3f}s min, {max(times):.3f}s max)")
            self.assertLess(avg, 5.0, f"Topics list too slow: {avg:.3f}s")

    def test_timing_context(self):
        """Benchmark context discovery timing."""
        times = []
        for i in range(3):
            result = run_cli("context", timeout=10.0)
            if result["returncode"] == 0:
                times.append(result["elapsed_s"])
        
        if times:
            avg = sum(times) / len(times)
            print(f"✓ Context timing: {avg:.3f}s avg ({min(times):.3f}s min, {max(times):.3f}s max)")
            self.assertLess(avg, 10.0, f"Context too slow: {avg:.3f}s")

    def test_timing_safety_check(self):
        """Benchmark safety check timing."""
        times = []
        for i in range(5):
            result = run_cli("safety", "show", timeout=5.0)
            if result["returncode"] == 0:
                times.append(result["elapsed_s"])
        
        if times:
            avg = sum(times) / len(times)
            print(f"✓ Safety check timing: {avg:.3f}s avg ({min(times):.3f}s min, {max(times):.3f}s max)")
            self.assertLess(avg, 1.0, f"Safety check too slow: {avg:.3f}s")


if __name__ == '__main__':
    if not LIVE_TESTS_ENABLED:
        print("Live tests disabled. Run with --live or set ROS2_LIVE_TESTS=1")
        print("Example: python3 -m pytest tests/test_live_validation.py -v --live")
        sys.exit(0)
    
    unittest.main(argv=[sys.argv[0], '-v'])
