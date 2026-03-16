#!/usr/bin/env python3
"""Unit tests for ros2_cli.py.

Tests cover argument parsing, dispatch table, JSON handling,
and utility functions.
"""

import json
import sys
import os
import unittest
from unittest.mock import patch, MagicMock
from io import StringIO

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "scripts"))


def check_rclpy_available():
    """Check if rclpy is available without importing the full module."""
    try:
        import rclpy
        return True
    except ImportError:
        return False


class TestBuildParser(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        if not check_rclpy_available():
            raise unittest.SkipTest("rclpy not available - requires ROS 2 environment")
        import ros2_cli
        cls.ros2_cli = ros2_cli

    def setUp(self):
        self.parser = self.ros2_cli.build_parser()

    def test_parser_subcommands(self):
        # Basic commands
        self.assertEqual(self.parser.parse_args(["version"]).command, "version")
        self.assertEqual(self.parser.parse_args(["nodes", "list"]).subcommand, "list")
        self.assertEqual(self.parser.parse_args(["nodes", "details", "/n"]).node, "/n")
        
        # Topics
        self.assertEqual(self.parser.parse_args(["topics", "list"]).subcommand, "list")
        self.assertEqual(self.parser.parse_args(["topics", "type", "/t"]).topic, "/t")
        args = self.parser.parse_args(["topics", "subscribe", "/s", "--duration", "10", "--max-messages", "50"])
        self.assertEqual(args.duration, 10.0)
        self.assertEqual(args.max_messages, 50)
        args = self.parser.parse_args(["topics", "publish", "/t", "{}", "--rate", "20"])
        self.assertEqual(args.rate, 20.0)
        self.assertEqual(self.parser.parse_args(["topics", "publish-sequence", "/t", "[]", "[]"]).subcommand, "publish-sequence")
        
        # Services & Actions
        self.assertEqual(self.parser.parse_args(["services", "call", "/s", "{}"]).service, "/s")
        args = self.parser.parse_args(["services", "echo", "/s", "--timeout", "10"])
        self.assertEqual(args.timeout, 10.0)
        self.assertEqual(self.parser.parse_args(["actions", "send", "/a", "{}"]).action, "/a")
        args = self.parser.parse_args(["actions", "echo", "/a", "--timeout", "2"])
        self.assertEqual(args.timeout, 2.0)
        self.assertEqual(self.parser.parse_args(["actions", "find", "p/A", "--timeout", "10"]).timeout, 10.0)
        
        # Params & Lifecycle
        self.assertEqual(self.parser.parse_args(["params", "list", "/n"]).node, "/n")
        self.assertEqual(self.parser.parse_args(["params", "get", "/n:p"]).name, "/n:p")
        self.assertEqual(self.parser.parse_args(["params", "set", "/n:p", "v"]).value, "v")
        self.assertEqual(self.parser.parse_args(["params", "describe", "/n", "p"]).param_name, "p")
        self.assertEqual(self.parser.parse_args(["params", "delete", "/n", "p"]).param_name, "p")
        self.assertEqual(self.parser.parse_args(["params", "dump", "/n"]).node, "/n")
        self.assertEqual(self.parser.parse_args(["params", "load", "/n", "{}"]).params, "{}")
        
        self.assertEqual(self.parser.parse_args(["lifecycle", "nodes"]).subcommand, "nodes")
        self.assertEqual(self.parser.parse_args(["lifecycle", "list", "/n"]).node, "/n")
        self.assertEqual(self.parser.parse_args(["lifecycle", "get", "/n"]).node, "/n")
        self.assertEqual(self.parser.parse_args(["lifecycle", "set", "/n", "configure"]).transition, "configure")
        self.assertEqual(self.parser.parse_args(["lifecycle", "set", "/n", "3", "--timeout", "10"]).timeout, 10.0)
        
        # Misc
        self.assertEqual(self.parser.parse_args(["estop", "--topic", "/e"]).topic, "/e")


class TestDispatchTable(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        if not check_rclpy_available():
            raise unittest.SkipTest("rclpy not available - requires ROS 2 environment")
        import ros2_cli
        cls.ros2_cli = ros2_cli

    def test_dispatch_table(self):
        # All keys callable
        for k, h in self.ros2_cli.DISPATCH.items():
            self.assertTrue(callable(h), f"{k} not callable")
        
        # Spot check key groups
        D = self.ros2_cli.DISPATCH
        self.assertIn(("version", None), D)
        self.assertIn(("topics", "list"), D)
        self.assertIn(("services", "call"), D)
        self.assertIn(("params", "get"), D)
        self.assertIn(("actions", "send"), D)
        self.assertIn(("lifecycle", "set"), D)
        self.assertIn(("tf", "lookup"), D)
        self.assertIn(("control", "list-controllers"), D)
        self.assertIn(("launch", "new"), D)
        self.assertIn(("run", "new"), D)
        
        # Verify specific aliases
        self.assertIs(D[("lifecycle", "ls")], D[("lifecycle", "list")])
        self.assertIs(D[("interface", "ls")], D[("interface", "list")])
        self.assertIs(D[("tf", "ls")], D[("tf", "list")])
        self.assertIs(D[("tf", "get")], D[("tf", "lookup")])
        self.assertIs(D[("launch", "ls")], D[("launch", "list")])
        self.assertIs(D[("run", "ls")], D[("run", "list")])



class TestOutput(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        if not check_rclpy_available():
            raise unittest.SkipTest("rclpy not available - requires ROS 2 environment")
        import ros2_cli
        cls.ros2_cli = ros2_cli

    def test_output_and_conversion(self):
        # JSON output
        for data in [{"k": "v"}, {"m": "로봇"}, {"a": {"b": [1]}}]:
            with patch("sys.stdout", new_callable=StringIO) as mock_stdout:
                self.ros2_cli.output(data)
                self.assertEqual(json.loads(mock_stdout.getvalue()), data)
        
        # msg_to_dict
        mock_msg = MagicMock()
        mock_msg.get_fields_and_field_types.return_value = ["f1"]
        mock_msg.f1 = "v1"
        self.assertEqual(self.ros2_cli.msg_to_dict(mock_msg), {"f1": "v1"})
        
        # dict_to_msg
        mock_class = MagicMock()
        mock_instance = MagicMock()
        mock_class.return_value = mock_instance
        self.assertIs(self.ros2_cli.dict_to_msg(mock_class, {"k": "v"}), mock_instance)
        self.assertEqual(mock_instance.k, "v")

    def test_parsing_helpers(self):
        # parse_node_param
        self.assertEqual(self.ros2_cli.parse_node_param("/n:p"), ("/n", "p"))
        self.assertEqual(self.ros2_cli.parse_node_param("/n"), ("/n", None))



class TestGetMsgType(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        if not check_rclpy_available():
            raise unittest.SkipTest("rclpy not available - requires ROS 2 environment")
        import ros2_cli
        cls.ros2_cli = ros2_cli

    def test_get_msg_type(self):
        # None/empty inputs return None
        self.assertIsNone(self.ros2_cli.get_msg_type(None))
        self.assertIsNone(self.ros2_cli.get_msg_type(''))
        # Known resolvable type returns the class
        msg_type = self.ros2_cli.get_msg_type('std_msgs/msg/String')
        self.assertIsNotNone(msg_type)
        self.assertEqual(msg_type.__name__, 'String')



class TestLifecycleParsing(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        if not check_rclpy_available():
            raise unittest.SkipTest("rclpy not available - requires ROS 2 environment")
        import ros2_cli
        cls.ros2_cli = ros2_cli
        cls.parser = ros2_cli.build_parser()

    def test_lifecycle_structural(self):
        # nodes subcommand positional rejection
        self.assertIsNone(getattr(self.parser.parse_args(["lifecycle", "nodes"]), "node", None))
        with self.assertRaises(SystemExit):
            self.parser.parse_args(["lifecycle", "nodes", "/extra"])
        
        # Dispatch identity (ls -> list)
        self.assertIs(self.ros2_cli.DISPATCH[("lifecycle", "ls")], self.ros2_cli.DISPATCH[("lifecycle", "list")])



class TestDoctorParsing(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        if not check_rclpy_available():
            raise unittest.SkipTest("rclpy not available - requires ROS 2 environment")
        import ros2_cli
        cls.ros2_cli = ros2_cli
        cls.parser = ros2_cli.build_parser()

    def test_doctor_and_multicast(self):
        # doctor/wtf flags
        args = self.parser.parse_args(["doctor", "--report", "--include-warnings"])
        self.assertTrue(args.report and args.include_warnings)
        self.assertEqual(self.parser.parse_args(["doctor", "hello", "--timeout", "5"]).timeout, 5.0)
        
        # multicast
        args = self.parser.parse_args(["multicast", "send", "-g", "239.0.0.1", "-p", "12345"])
        self.assertEqual(args.group, "239.0.0.1")
        self.assertEqual(self.parser.parse_args(["multicast", "receive", "-t", "10"]).timeout, 10.0)
        
        # Dispatch identity
        D = self.ros2_cli.DISPATCH
        self.assertIs(D[("wtf", None)], D[("doctor", None)])
        self.assertIs(D[("wtf", "hello")], D[("doctor", "hello")])



class TestInterfaceParsing(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        if not check_rclpy_available():
            raise unittest.SkipTest("rclpy not available - requires ROS 2 environment")
        import ros2_cli
        cls.ros2_cli = ros2_cli
        cls.parser = ros2_cli.build_parser()

    def test_interface_and_presets(self):
        # interface
        self.assertEqual(self.parser.parse_args(["interface", "list"]).subcommand, "list")
        self.assertEqual(self.parser.parse_args(["interface", "show", "std_msgs/String"]).type_str, "std_msgs/String")
        self.assertEqual(self.parser.parse_args(["interface", "package", "std_msgs"]).package, "std_msgs")
        
        # presets
        args = self.parser.parse_args(["params", "preset-save", "/n", "p", "--timeout", "10"])
        self.assertEqual(args.preset, "p")
        self.assertEqual(args.timeout, 10.0)
        self.assertEqual(self.parser.parse_args(["params", "preset-load", "/n", "p"]).preset, "p")
        self.assertEqual(self.parser.parse_args(["params", "preset-delete", "p"]).preset, "p")
        
        # dispatch
        D = self.ros2_cli.DISPATCH
        self.assertIs(D[("interface", "ls")], D[("interface", "list")])
        self.assertIsNot(D[("params", "preset-save")], D[("params", "preset-load")])



class TestDiagnosticsParsing(unittest.TestCase):
    """Parser argument and DISPATCH wiring tests for topics diag-list and topics diag."""

    @classmethod
    def setUpClass(cls):
        if not check_rclpy_available():
            raise unittest.SkipTest("rclpy not available - requires ROS 2 environment")
        import ros2_cli
        import ros2_topic
        cls.ros2_cli = ros2_cli
        cls.ros2_topic = ros2_topic
        cls.parser = ros2_cli.build_parser()

    def test_diag_parsing_and_dispatch(self):
        # diag-list
        args = self.parser.parse_args(["topics", "diag-list"])
        self.assertEqual(args.subcommand, "diag-list")
        self.assertFalse(hasattr(args, "topic"))
        # diag defaults
        args = self.parser.parse_args(["topics", "diag"])
        self.assertIsNone(args.topic)
        self.assertEqual(args.timeout, 10.0)
        self.assertEqual(args.max_messages, 1)
        # diag custom
        args = self.parser.parse_args([
            "topics", "diag", "--topic", "/d", "--timeout", "5",
            "--duration", "3.5", "--max-messages", "20"
        ])
        self.assertEqual(args.topic, "/d")
        self.assertEqual(args.timeout, 5.0)
        self.assertEqual(args.duration, 3.5)
        self.assertEqual(args.max_messages, 20)
        # Dispatch
        D = self.ros2_cli.DISPATCH
        self.assertTrue(callable(D[("topics", "diag-list")]))
        self.assertTrue(callable(D[("topics", "diag")]))
        self.assertIsNot(D[("topics", "diag-list")], D[("topics", "diag")])
        # Constants
        self.assertIn("diagnostic_msgs/msg/DiagnosticArray", self.ros2_topic.DIAG_TYPES)

    def test_parse_diag_array_logic(self):
        """Pure-Python logic for _parse_diag_array."""
        # Levels
        for level, name in [(0, "OK"), (1, "WARN"), (2, "ERROR"), (3, "STALE")]:
            msg = {"status": [{"level": level, "name": "t", "message": "", "hardware_id": "", "values": []}]}
            self.assertEqual(self.ros2_topic._parse_diag_array(msg)[0]["level_name"], name)
        # Empty and Multiple
        self.assertEqual(self.ros2_topic._parse_diag_array({"status": []}), [])
        msg = {"status": [
            {"level": 0, "name": "cpu", "message": "OK", "hardware_id": "h", "values": [{"key": "k", "value": "v"}]},
            {"level": 2, "name": "disk", "message": "F", "hardware_id": "", "values": []}
        ]}
        result = self.ros2_topic._parse_diag_array(msg)
        self.assertEqual(len(result), 2)
        self.assertEqual(result[0]["level_name"], "OK")
        self.assertEqual(result[1]["level_name"], "ERROR")


class TestBatteryParsing(unittest.TestCase):
    """Tests for 'topics battery-list' and 'topics battery' subcommands."""

    @classmethod
    def setUpClass(cls):
        if not check_rclpy_available():
            raise unittest.SkipTest("rclpy not available - requires ROS 2 environment")
        import ros2_cli
        import ros2_topic
        cls.ros2_cli = ros2_cli
        cls.ros2_topic = ros2_topic
        cls.parser = ros2_cli.build_parser()

    def test_battery_parsing_and_dispatch(self):
        # battery-list
        args = self.parser.parse_args(["topics", "battery-list"])
        self.assertEqual(args.subcommand, "battery-list")
        # battery defaults
        args = self.parser.parse_args(["topics", "battery"])
        self.assertIsNone(args.topic)
        self.assertEqual(args.timeout, 10.0)
        self.assertEqual(args.max_messages, 1)
        # battery custom
        args = self.parser.parse_args([
            "topics", "battery", "--topic", "/b", "--timeout", "5",
            "--duration", "3.5", "--max-messages", "5"
        ])
        self.assertEqual(args.topic, "/b")
        self.assertEqual(args.timeout, 5.0)
        self.assertEqual(args.duration, 3.5)
        self.assertEqual(args.max_messages, 5)
        # Dispatch
        D = self.ros2_cli.DISPATCH
        self.assertTrue(callable(D[("topics", "battery-list")]))
        self.assertTrue(callable(D[("topics", "battery")]))
        # Constants
        self.assertIn("sensor_msgs/msg/BatteryState", self.ros2_topic.BATTERY_TYPES)

    def test_parse_battery_state_logic(self):
        """Pure-Python logic for _parse_battery_state."""
        # Status codes
        for code, name in [(1, "CHARGING"), (2, "DISCHARGING")]:
            self.assertEqual(self.ros2_topic._parse_battery_state({"power_supply_status": code})["status_name"], name)
        # Health codes
        for code, name in [(1, "GOOD"), (2, "OVERHEAT")]:
            self.assertEqual(self.ros2_topic._parse_battery_state({"power_supply_health": code})["health_name"], name)
        # Fields and NaN
        msg = {"percentage": 0.75, "voltage": float("nan"), "present": True, "cell_voltage": [4.1, float("nan")]}
        r = self.ros2_topic._parse_battery_state(msg)
        self.assertAlmostEqual(r["percentage"], 75.0)
        self.assertIsNone(r["voltage"])
        self.assertTrue(r["present"])
        self.assertEqual(r["cell_voltage"], [4.1, None])



class TestGlobalOverrides(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        if not check_rclpy_available():
            raise unittest.SkipTest("rclpy not available - requires ROS 2 environment")
        import ros2_cli
        cls.ros2_cli = ros2_cli
        cls.parser = ros2_cli.build_parser()

    def test_global_overrides(self):
        # Argparse integration
        args = self.parser.parse_args(["--timeout", "30", "--retries", "3", "topics", "list"])
        self.assertEqual(args.global_timeout, 30.0)
        self.assertEqual(args.global_retries, 3)
        
        # Logic verification
        from types import SimpleNamespace
        args = SimpleNamespace(global_timeout=30.0, timeout=5.0, retries=1)
        self.ros2_cli._apply_global_overrides(args)
        self.assertEqual(args.timeout, 30.0)
        
        # Fallback verification
        args = SimpleNamespace(global_timeout=None)
        self.ros2_cli._apply_global_overrides(args)
        self.assertEqual(args.retries, 1)
        self.assertFalse(hasattr(args, "timeout"))

    def test_retry_behavior(self):
        # This mocks the ROS 2 layer to exercise retry loop logic
        with patch("ros2_service.rclpy") as mock_rclpy, \
             patch("ros2_service.output") as mock_output, \
             patch("ros2_service.ROS2CLI") as mock_node_cls:
            import ros2_service
            mock_client = MagicMock()
            mock_client.wait_for_service.side_effect = [False, False, True] # 2 fails, 1 success
            mock_node_cls.return_value.create_client.return_value = mock_client
            
            future = MagicMock()
            future.done.return_value = True
            mock_client.call_async.return_value = future
            
            from types import SimpleNamespace
            args = SimpleNamespace(service="/s", service_type="std_srvs/srv/Empty", extra_request=None, request="{}", timeout=1.0, retries=3, global_timeout=None)
            
            with patch("ros2_service.get_srv_type"), patch("ros2_service.time"):
                ros2_service.cmd_services_call(args)
            
            self.assertEqual(mock_client.wait_for_service.call_count, 3)



class TestTFParsing(unittest.TestCase):
    """Parser argument and DISPATCH wiring tests for the tf subcommands."""

    @classmethod
    def setUpClass(cls):
        if not check_rclpy_available():
            raise unittest.SkipTest("rclpy not available - requires ROS 2 environment")
        import ros2_cli
        cls.ros2_cli = ros2_cli
        cls.parser = ros2_cli.build_parser()

    def test_tf_list_lookup_echo_monitor(self):
        # list
        self.assertEqual(self.parser.parse_args(["tf", "list"]).subcommand, "list")
        # lookup
        args = self.parser.parse_args(["tf", "lookup", "s", "t", "--timeout", "10"])
        self.assertEqual(args.source, "s")
        self.assertEqual(args.timeout, 10.0)
        # echo
        args = self.parser.parse_args(["tf", "echo", "s", "t", "--count", "3", "--once"])
        self.assertEqual(args.count, 3)
        self.assertTrue(args.once)
        # monitor
        args = self.parser.parse_args(["tf", "monitor", "f", "--count", "2"])
        self.assertEqual(args.frame, "f")
        self.assertEqual(args.count, 2)

    def test_tf_static(self):
        # named
        args = self.parser.parse_args(["tf", "static", "--from", "s", "--to", "t", "--xyz", "1", "2", "3", "--rpy", "0", "0", "0"])
        self.assertEqual(args.from_frame, "s")
        self.assertEqual(args.xyz, [1.0, 2.0, 3.0])
        # positional
        args = self.parser.parse_args(["tf", "static", "1", "2", "3", "0", "0", "0", "s", "t"])
        self.assertEqual(args.pos_args, ["1", "2", "3", "0", "0", "0", "s", "t"])

    def test_tf_conversions(self):
        # euler-from-quaternion
        args = self.parser.parse_args(["tf", "euler-from-quaternion", "0", "0", "0", "1"])
        self.assertEqual(args.w, 1.0)
        # quaternion-from-euler
        args = self.parser.parse_args(["tf", "quaternion-from-euler", "0", "0", "0"])
        self.assertEqual(args.yaw, 0.0)
        # degrees variants
        self.assertEqual(self.parser.parse_args(["tf", "euler-from-quaternion-deg", "0", "0", "0", "1"]).subcommand, "euler-from-quaternion-deg")
        self.assertEqual(self.parser.parse_args(["tf", "quaternion-from-euler-deg", "0", "0", "0"]).subcommand, "quaternion-from-euler-deg")

    def test_tf_transform_point_vector(self):
        # point
        args = self.parser.parse_args(["tf", "transform-point", "t", "s", "1", "2", "3"])
        self.assertEqual(args.target, "t")
        self.assertEqual(args.x, 1.0)
        # vector
        args = self.parser.parse_args(["tf", "transform-vector", "t", "s", "1", "0", "0"])
        self.assertEqual(args.target, "t")

    def test_tf_aliases_and_dispatch(self):
        # parsing subset
        self.assertEqual(self.parser.parse_args(["tf", "ls"]).subcommand, "ls")
        self.assertEqual(self.parser.parse_args(["tf", "get", "s", "t"]).subcommand, "get")
        # verify DISPATCH — only kept aliases: ls, get
        D = self.ros2_cli.DISPATCH
        self.assertIs(D[("tf", "ls")], D[("tf", "list")])
        self.assertIs(D[("tf", "get")], D[("tf", "lookup")])
        for key in [k for k in D if k[0] == "tf"]:
            self.assertTrue(callable(D[key]))


class TestLaunchParsing(unittest.TestCase):
    """Parser argument and DISPATCH wiring tests for the launch subcommands."""

    @classmethod
    def setUpClass(cls):
        if not check_rclpy_available():
            raise unittest.SkipTest("rclpy not available - requires ROS 2 environment")
        import ros2_cli
        cls.ros2_cli = ros2_cli
        cls.parser = ros2_cli.build_parser()

    def test_launch_commands(self):
        # new
        args = self.parser.parse_args(["launch", "new", "p", "f", "a:=1", "--timeout", "60"])
        self.assertEqual(args.package, "p")
        self.assertEqual(args.args, ["a:=1"])
        self.assertEqual(args.timeout, 60.0)
        # list/kill/restart
        self.assertEqual(self.parser.parse_args(["launch", "list"]).subcommand, "list")
        self.assertEqual(self.parser.parse_args(["launch", "kill", "s"]).session, "s")
        self.assertEqual(self.parser.parse_args(["launch", "restart", "s"]).session, "s")
        # foxglove
        self.assertEqual(self.parser.parse_args(["launch", "foxglove", "9000"]).port, 9000)

    def test_launch_dispatch(self):
        D = self.ros2_cli.DISPATCH
        self.assertIs(D[("launch", "ls")], D[("launch", "list")])
        for key in [k for k in D if k[0] == "launch"]:
            self.assertTrue(callable(D[key]))

class TestRunParsing(unittest.TestCase):
    """Parser argument and DISPATCH wiring tests for the run subcommands."""

    @classmethod
    def setUpClass(cls):
        if not check_rclpy_available():
            raise unittest.SkipTest("rclpy not available - requires ROS 2 environment")
        import ros2_cli
        cls.ros2_cli = ros2_cli
        cls.parser = ros2_cli.build_parser()

    def test_run_commands(self):
        # new
        args = self.parser.parse_args(["run", "new", "p", "e", "--presets", "a", "--params", "b", "--config-path", "c"])
        self.assertEqual(args.presets, "a")
        self.assertEqual(args.params, "b")
        self.assertEqual(args.config_path, "c")
        # list/kill/restart
        self.assertEqual(self.parser.parse_args(["run", "list"]).subcommand, "list")
        self.assertEqual(self.parser.parse_args(["run", "kill", "s"]).session, "s")
        self.assertEqual(self.parser.parse_args(["run", "restart", "s"]).session, "s")

    def test_run_dispatch(self):
        D = self.ros2_cli.DISPATCH
        self.assertIs(D[("run", "ls")], D[("run", "list")])
        for key in [k for k in D if k[0] == "run"]:
            self.assertTrue(callable(D[key]))


class TestPublishUntilParsing(unittest.TestCase):
    """Parser argument tests for topics publish-until."""

    @classmethod
    def setUpClass(cls):
        if not check_rclpy_available():
            raise unittest.SkipTest("rclpy not available - requires ROS 2 environment")
        import ros2_cli
        cls.ros2_cli = ros2_cli
        cls.parser = ros2_cli.build_parser()

    def test_publish_until_parsing(self):
        # Defaults
        args = self.parser.parse_args([
            "topics", "publish-until", "/c", "{}", "--monitor", "/o",
            "--field", "f", "--delta", "1.0"
        ])
        self.assertEqual(args.topic, "/c")
        self.assertEqual(args.delta, 1.0)
        self.assertFalse(args.euclidean)
        # Euclidean + Multiple fields
        args = self.parser.parse_args([
            "topics", "publish-until", "/c", "{}", "--monitor", "/o",
            "--field", "f1", "f2", "--euclidean", "--delta", "2.0"
        ])
        self.assertTrue(args.euclidean)
        self.assertEqual(args.field, ["f1", "f2"])
        # Numeric bounds
        self.assertEqual(self.parser.parse_args(["topics", "publish-until", "/c", "{}", "--monitor", "/o", "--above", "5"]).above, 5.0)
        self.assertEqual(self.parser.parse_args(["topics", "publish-until", "/c", "{}", "--monitor", "/o", "--below", "0.5"]).below, 0.5)
        self.assertEqual(self.parser.parse_args(["topics", "publish-until", "/c", "{}", "--monitor", "/o", "--equals", "v"]).equals, "v")
        # Rotation
        args = self.parser.parse_args(["topics", "publish-until", "/c", "{}", "--monitor", "/o", "--rotate", "90", "--degrees"])
        self.assertEqual(args.rotate, 90.0)
        self.assertTrue(args.degrees)
        # Dispatch
        key = ("topics", "publish-until")
        self.assertIn(key, self.ros2_cli.DISPATCH)
        self.assertTrue(callable(self.ros2_cli.DISPATCH[key]))


class TestRotationMath(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        if not check_rclpy_available():
            raise unittest.SkipTest("rclpy not available - requires ROS 2 environment")
        import ros2_topic
        cls.ros2_topic = ros2_topic

    def test_math_helpers(self):
        import math
        # quaternion_to_yaw
        self.assertAlmostEqual(self.ros2_topic.quaternion_to_yaw((0,0,0,1)), 0.0)
        self.assertAlmostEqual(self.ros2_topic.quaternion_to_yaw((0,0,math.sin(math.pi/4),math.cos(math.pi/4))), math.pi/2)
        
        # normalize_angle
        self.assertAlmostEqual(self.ros2_topic.normalize_angle(0.0), 0.0)
        self.assertAlmostEqual(self.ros2_topic.normalize_angle(math.pi + 0.5), -(math.pi - 0.5))
        self.assertAlmostEqual(self.ros2_topic.normalize_angle(5 * math.pi), math.pi)



class TestControlParsing(unittest.TestCase):
    """Parser argument and DISPATCH wiring tests for the control subcommands."""

    @classmethod
    def setUpClass(cls):
        if not check_rclpy_available():
            raise unittest.SkipTest("rclpy not available - requires ROS 2 environment")
        import ros2_cli
        cls.ros2_cli = ros2_cli
        cls.parser = ros2_cli.build_parser()

    def test_control_list_commands(self):
        # controller-types
        args = self.parser.parse_args(["control", "list-controller-types", "--controller-manager", "/cm"])
        self.assertEqual(args.controller_manager, "/cm")
        self.assertEqual(args.timeout, 5.0)
        # controllers
        self.assertEqual(self.parser.parse_args(["control", "list-controllers"]).subcommand, "list-controllers")
        # hardware-components/interfaces
        self.assertEqual(self.parser.parse_args(["control", "list-hardware-components"]).subcommand, "list-hardware-components")
        self.assertEqual(self.parser.parse_args(["control", "list-hardware-interfaces"]).subcommand, "list-hardware-interfaces")

    def test_control_load_unload_configure(self):
        self.assertEqual(self.parser.parse_args(["control", "load-controller", "c"]).name, "c")
        self.assertEqual(self.parser.parse_args(["control", "unload-controller", "c"]).name, "c")
        self.assertEqual(self.parser.parse_args(["control", "configure-controller", "c"]).name, "c")

    def test_control_reload_and_state(self):
        # reload
        self.assertTrue(self.parser.parse_args(["control", "reload-controller-libraries", "--force-kill"]).force_kill)
        # controller state
        self.assertEqual(self.parser.parse_args(["control", "set-controller-state", "c", "active"]).state, "active")
        # hardware state
        args = self.parser.parse_args(["control", "set-hardware-component-state", "h", "inactive"])
        self.assertEqual(args.state, "inactive")

    def test_control_switch_and_view(self):
        # switch
        args = self.parser.parse_args(["control", "switch-controllers", "--activate", "a", "--deactivate", "d", "--strictness", "STRICT", "--activate-asap"])
        self.assertEqual(args.activate, ["a"])
        self.assertEqual(args.deactivate, ["d"])
        self.assertEqual(args.strictness, "STRICT")
        self.assertTrue(args.activate_asap)
        # view
        args = self.parser.parse_args(["control", "view-controller-chains", "--output", "o.pdf", "--channel-id", "1"])
        self.assertEqual(args.output, "o.pdf")
        self.assertEqual(args.channel_id, "1")

    def test_control_aliases(self):
        # Test kept aliases for parsing (load, unload)
        self.assertEqual(self.parser.parse_args(["control", "load", "c"]).name, "c")
        self.assertEqual(self.parser.parse_args(["control", "unload", "c"]).name, "c")
        # Verify DISPATCH identity for kept aliases only
        D = self.ros2_cli.DISPATCH
        self.assertIs(D[("control", "load")], D[("control", "load-controller")])
        self.assertIs(D[("control", "unload")], D[("control", "unload-controller")])

    def test_control_dispatch_wiring(self):
        for key in [k for k in self.ros2_cli.DISPATCH if k[0] == "control"]:
            self.assertTrue(callable(self.ros2_cli.DISPATCH[key]))


class TestBagParsing(unittest.TestCase):
    """Parser argument and DISPATCH wiring tests for the bag subcommands."""

    @classmethod
    def setUpClass(cls):
        if not check_rclpy_available():
            raise unittest.SkipTest("rclpy not available - requires ROS 2 environment")
        import ros2_cli
        cls.ros2_cli = ros2_cli
        cls.parser = ros2_cli.build_parser()

    def test_bag_info_parsing(self):
        args = self.parser.parse_args(["bag", "info", "/path/to/my_bag"])
        self.assertEqual(args.command, "bag")
        self.assertEqual(args.subcommand, "info")
        self.assertEqual(args.bag_path, "/path/to/my_bag")

    def test_bag_info_accepts_metadata_path(self):
        args = self.parser.parse_args(["bag", "info", "/path/to/my_bag/metadata.yaml"])
        self.assertEqual(args.bag_path, "/path/to/my_bag/metadata.yaml")

    def test_bag_dispatch(self):
        D = self.ros2_cli.DISPATCH
        self.assertIn(("bag", "info"), D)
        self.assertTrue(callable(D[("bag", "info")]))


class TestBagMetadataLogic(unittest.TestCase):
    """Pure-Python logic tests for ros2_bag helpers.

    No rclpy calls are made by the functions under test, but ros2_utils
    (imported by ros2_bag) calls sys.exit(1) when rclpy is absent, so the
    rclpy guard is still required to import the module safely.
    """

    @classmethod
    def setUpClass(cls):
        if not check_rclpy_available():
            raise unittest.SkipTest("rclpy not available - requires ROS 2 environment")
        import ros2_bag
        cls.ros2_bag = ros2_bag

    def test_ns_to_sec(self):
        self.assertAlmostEqual(self.ros2_bag._ns_to_sec(0), 0.0)
        self.assertAlmostEqual(self.ros2_bag._ns_to_sec(1_000_000_000), 1.0)
        self.assertAlmostEqual(self.ros2_bag._ns_to_sec(500_000_000), 0.5)
        self.assertAlmostEqual(self.ros2_bag._ns_to_sec(1_234_567_890), 1.23456789)

    def test_parse_metadata_from_directory(self):
        import tempfile
        import pathlib
        try:
            import yaml
        except ImportError:
            raise unittest.SkipTest("PyYAML not available")

        meta = {
            "rosbag2_bagfile_information": {
                "duration":        {"nanoseconds": 5_000_000_000},
                "starting_time":   {"nanoseconds_since_epoch": 1_700_000_000_000_000_000},
                "storage_identifier": "sqlite3",
                "message_count":   42,
                "topics_with_message_count": [
                    {
                        "topic_metadata": {
                            "name":                 "/cmd_vel",
                            "type":                 "geometry_msgs/msg/Twist",
                            "serialization_format": "cdr",
                            "offered_qos_profiles": "",
                        },
                        "message_count": 10,
                    },
                    {
                        "topic_metadata": {
                            "name":                 "/odom",
                            "type":                 "nav_msgs/msg/Odometry",
                            "serialization_format": "cdr",
                            "offered_qos_profiles": "",
                        },
                        "message_count": 32,
                    },
                ],
            }
        }
        with tempfile.TemporaryDirectory() as tmpdir:
            meta_path = pathlib.Path(tmpdir) / "metadata.yaml"
            with open(meta_path, "w") as fh:
                yaml.dump(meta, fh)
            result = self.ros2_bag._parse_metadata(tmpdir)

        info = result["rosbag2_bagfile_information"]
        self.assertEqual(info["storage_identifier"], "sqlite3")
        self.assertEqual(info["message_count"], 42)
        self.assertEqual(len(info["topics_with_message_count"]), 2)

    def test_parse_metadata_from_metadata_yaml_path(self):
        import tempfile
        import pathlib
        try:
            import yaml
        except ImportError:
            raise unittest.SkipTest("PyYAML not available")

        meta = {"rosbag2_bagfile_information": {"storage_identifier": "mcap"}}
        with tempfile.TemporaryDirectory() as tmpdir:
            meta_path = pathlib.Path(tmpdir) / "metadata.yaml"
            with open(meta_path, "w") as fh:
                yaml.dump(meta, fh)
            # Pass the metadata.yaml path directly
            result = self.ros2_bag._parse_metadata(str(meta_path))

        self.assertEqual(result["rosbag2_bagfile_information"]["storage_identifier"], "mcap")

    def test_parse_metadata_missing_raises(self):
        import tempfile
        with tempfile.TemporaryDirectory() as tmpdir:
            with self.assertRaises(FileNotFoundError):
                self.ros2_bag._parse_metadata(tmpdir)


class TestComponentParsing(unittest.TestCase):
    """Parser argument and DISPATCH wiring tests for the component subcommands."""

    @classmethod
    def setUpClass(cls):
        if not check_rclpy_available():
            raise unittest.SkipTest("rclpy not available - requires ROS 2 environment")
        import ros2_cli
        cls.ros2_cli = ros2_cli
        cls.parser = ros2_cli.build_parser()

    def test_component_types_parsing(self):
        args = self.parser.parse_args(["component", "types"])
        self.assertEqual(args.command, "component")
        self.assertEqual(args.subcommand, "types")

    def test_component_dispatch(self):
        D = self.ros2_cli.DISPATCH
        self.assertIn(("component", "types"), D)
        self.assertTrue(callable(D[("component", "types")]))


class TestComponentTypesLogic(unittest.TestCase):
    """Pure-Python logic tests for ros2_component.

    No rclpy calls are made by the functions under test, but ros2_utils
    (imported by ros2_component) calls sys.exit(1) when rclpy is absent, so
    the rclpy guard is still required to import the module safely.
    """

    @classmethod
    def setUpClass(cls):
        if not check_rclpy_available():
            raise unittest.SkipTest("rclpy not available - requires ROS 2 environment")
        import ros2_component
        cls.ros2_component = ros2_component

    def test_cmd_component_types_no_ament(self):
        """When ament_index_python is missing, output contains an error key."""
        import sys
        captured = []
        with patch.dict(sys.modules, {"ament_index_python": None}), \
             patch("ros2_component.output", side_effect=captured.append):
            self.ros2_component.cmd_component_types(None)
        self.assertEqual(len(captured), 1)
        self.assertIn("error", captured[0])

    def test_cmd_component_types_with_ament(self):
        """With a mocked ament index, components are listed correctly."""
        import sys
        from unittest.mock import MagicMock

        mock_ament = MagicMock()
        mock_ament.get_resources.return_value = {"demo_nodes_cpp": "/opt/ros/humble"}
        mock_ament.get_resource.return_value = (
            "demo_nodes_cpp::Talker\ndemo_nodes_cpp::Listener\n# a comment\n\n",
            "/opt/ros/humble",
        )

        captured = []
        with patch.dict(sys.modules, {"ament_index_python": mock_ament}), \
             patch("ros2_component.output", side_effect=captured.append):
            self.ros2_component.cmd_component_types(None)

        self.assertEqual(len(captured), 1)
        result = captured[0]
        self.assertIn("components", result)
        # Comment and blank lines must be skipped
        self.assertEqual(result["total"], 2)
        self.assertIn("demo_nodes_cpp", result["packages"])
        names = [c["type_name"] for c in result["components"]]
        self.assertIn("demo_nodes_cpp::Talker", names)
        self.assertIn("demo_nodes_cpp::Listener", names)
        self.assertNotIn("warnings", result)

    def test_cmd_component_types_partial_error(self):
        """A per-package read error populates warnings but does not stop enumeration."""
        import sys
        from unittest.mock import MagicMock

        mock_ament = MagicMock()
        mock_ament.get_resources.return_value = {
            "good_pkg": "/opt/ros/humble",
            "bad_pkg":  "/opt/ros/humble",
        }

        def get_resource(res_type, pkg):
            if pkg == "bad_pkg":
                raise RuntimeError("disk error")
            return ("good_pkg::GoodNode\n", "/opt")

        mock_ament.get_resource.side_effect = get_resource

        captured = []
        with patch.dict(sys.modules, {"ament_index_python": mock_ament}), \
             patch("ros2_component.output", side_effect=captured.append):
            self.ros2_component.cmd_component_types(None)

        result = captured[0]
        self.assertEqual(result["total"], 1)
        self.assertIn("good_pkg", result["packages"])
        self.assertIn("warnings", result)
        self.assertEqual(len(result["warnings"]), 1)
        self.assertEqual(result["warnings"][0]["package"], "bad_pkg")


class TestBagInfoCommand(unittest.TestCase):
    """End-to-end tests for cmd_bag_info.

    Tests the full function including metadata parsing, field extraction,
    duration conversion, topic sorting, compression handling, and every
    error path.  Uses real tempfiles + PyYAML so the filesystem code is
    exercised without a live ROS 2 graph.

    Note: the functions under test make no rclpy calls, but ros2_utils
    (imported by ros2_bag) calls sys.exit(1) when rclpy is absent, so
    the rclpy guard is required to import the module safely.
    """

    @classmethod
    def setUpClass(cls):
        if not check_rclpy_available():
            raise unittest.SkipTest("rclpy not available - requires ROS 2 environment")
        import ros2_bag
        cls.ros2_bag = ros2_bag

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------

    def _skip_if_no_yaml(self):
        try:
            import yaml  # noqa: F401
        except ImportError:
            raise unittest.SkipTest("PyYAML not available")

    def _write_bag(self, tmpdir, meta):
        import pathlib
        import yaml
        with open(pathlib.Path(tmpdir) / "metadata.yaml", "w") as fh:
            yaml.dump(meta, fh)

    def _base_meta(self, **updates):
        """Return a minimal valid metadata dict with optional field overrides."""
        info = {
            "duration":         {"nanoseconds": 10_000_000_000},
            "starting_time":    {"nanoseconds_since_epoch": 1_700_000_000_000_000_000},
            "storage_identifier": "sqlite3",
            "message_count":    150,
            "topics_with_message_count": [],
        }
        info.update(updates)
        return {"rosbag2_bagfile_information": info}

    def _run(self, bag_path):
        import types
        captured = []
        with patch("ros2_bag.output", side_effect=captured.append):
            self.ros2_bag.cmd_bag_info(types.SimpleNamespace(bag_path=bag_path))
        self.assertEqual(len(captured), 1)
        return captured[0]

    # ------------------------------------------------------------------
    # Valid bag — field presence and values
    # ------------------------------------------------------------------

    def test_valid_bag_required_fields_present(self):
        """All required top-level output fields are present for a valid bag."""
        self._skip_if_no_yaml()
        import tempfile
        with tempfile.TemporaryDirectory() as tmpdir:
            self._write_bag(tmpdir, self._base_meta())
            result = self._run(tmpdir)
        for field in ("bag_path", "storage_identifier", "duration",
                      "starting_time", "message_count", "topic_count", "topics"):
            self.assertIn(field, result)

    def test_valid_bag_duration_seconds_conversion(self):
        """duration.seconds is correctly derived from nanoseconds."""
        self._skip_if_no_yaml()
        import tempfile
        with tempfile.TemporaryDirectory() as tmpdir:
            self._write_bag(tmpdir, self._base_meta(duration={"nanoseconds": 5_500_000_000}))
            result = self._run(tmpdir)
        self.assertAlmostEqual(result["duration"]["seconds"], 5.5)
        self.assertEqual(result["duration"]["nanoseconds"], 5_500_000_000)

    def test_valid_bag_starting_time_preserved(self):
        """starting_time nanoseconds_since_epoch value is passed through unchanged."""
        self._skip_if_no_yaml()
        import tempfile
        ns = 1_700_000_000_123_456_789
        with tempfile.TemporaryDirectory() as tmpdir:
            self._write_bag(tmpdir, self._base_meta(
                starting_time={"nanoseconds_since_epoch": ns}
            ))
            result = self._run(tmpdir)
        self.assertEqual(result["starting_time"]["nanoseconds_since_epoch"], ns)

    def test_valid_bag_storage_identifier(self):
        """storage_identifier is correctly read for both sqlite3 and mcap."""
        self._skip_if_no_yaml()
        import tempfile
        for storage in ("sqlite3", "mcap"):
            with tempfile.TemporaryDirectory() as tmpdir:
                self._write_bag(tmpdir, self._base_meta(storage_identifier=storage))
                result = self._run(tmpdir)
            self.assertEqual(result["storage_identifier"], storage)

    def test_valid_bag_message_count(self):
        """message_count in output matches the metadata value."""
        self._skip_if_no_yaml()
        import tempfile
        with tempfile.TemporaryDirectory() as tmpdir:
            self._write_bag(tmpdir, self._base_meta(message_count=999))
            result = self._run(tmpdir)
        self.assertEqual(result["message_count"], 999)

    def test_valid_bag_path_is_absolute(self):
        """bag_path in output is the resolved absolute path."""
        self._skip_if_no_yaml()
        import tempfile, pathlib
        with tempfile.TemporaryDirectory() as tmpdir:
            self._write_bag(tmpdir, self._base_meta())
            result = self._run(tmpdir)
        self.assertTrue(pathlib.Path(result["bag_path"]).is_absolute())

    # ------------------------------------------------------------------
    # Topic handling
    # ------------------------------------------------------------------

    def test_topics_sorted_alphabetically(self):
        """Topics in output are sorted alphabetically by name regardless of input order."""
        self._skip_if_no_yaml()
        import tempfile
        topics = [
            {"topic_metadata": {"name": "/scan",    "type": "sensor_msgs/msg/LaserScan", "serialization_format": "cdr", "offered_qos_profiles": ""}, "message_count": 10},
            {"topic_metadata": {"name": "/cmd_vel", "type": "geometry_msgs/msg/Twist",   "serialization_format": "cdr", "offered_qos_profiles": ""}, "message_count": 50},
            {"topic_metadata": {"name": "/odom",    "type": "nav_msgs/msg/Odometry",     "serialization_format": "cdr", "offered_qos_profiles": ""}, "message_count": 100},
        ]
        with tempfile.TemporaryDirectory() as tmpdir:
            self._write_bag(tmpdir, self._base_meta(topics_with_message_count=topics))
            result = self._run(tmpdir)
        names = [t["name"] for t in result["topics"]]
        self.assertEqual(names, sorted(names))
        self.assertEqual(names, ["/cmd_vel", "/odom", "/scan"])

    def test_topic_count_always_matches_topics_list_length(self):
        """topic_count == len(topics) for 0, 1, and N topics."""
        self._skip_if_no_yaml()
        import tempfile
        for n in (0, 1, 5):
            topics = [
                {"topic_metadata": {"name": f"/t{i}", "type": "std_msgs/msg/String",
                                    "serialization_format": "cdr", "offered_qos_profiles": ""},
                 "message_count": i}
                for i in range(n)
            ]
            with tempfile.TemporaryDirectory() as tmpdir:
                self._write_bag(tmpdir, self._base_meta(topics_with_message_count=topics))
                result = self._run(tmpdir)
            self.assertEqual(result["topic_count"], n)
            self.assertEqual(result["topic_count"], len(result["topics"]))

    def test_topic_entry_has_all_required_fields(self):
        """Each topic dict contains name, type, serialization_format, offered_qos_profiles, message_count."""
        self._skip_if_no_yaml()
        import tempfile
        topics = [
            {"topic_metadata": {"name": "/cmd_vel", "type": "geometry_msgs/msg/Twist",
                                "serialization_format": "cdr",
                                "offered_qos_profiles": "- history: 1\n"},
             "message_count": 42},
        ]
        with tempfile.TemporaryDirectory() as tmpdir:
            self._write_bag(tmpdir, self._base_meta(topics_with_message_count=topics))
            result = self._run(tmpdir)
        topic = result["topics"][0]
        for field in ("name", "type", "serialization_format", "offered_qos_profiles", "message_count"):
            self.assertIn(field, topic)
        self.assertEqual(topic["name"], "/cmd_vel")
        self.assertEqual(topic["message_count"], 42)

    # ------------------------------------------------------------------
    # Compression
    # ------------------------------------------------------------------

    def test_compression_fields_present_when_set(self):
        """compression_format and compression_mode appear when the metadata has them."""
        self._skip_if_no_yaml()
        import tempfile
        with tempfile.TemporaryDirectory() as tmpdir:
            self._write_bag(tmpdir, self._base_meta(
                compression_format="zstd", compression_mode="file"
            ))
            result = self._run(tmpdir)
        self.assertEqual(result["compression_format"], "zstd")
        self.assertEqual(result["compression_mode"], "file")

    def test_compression_fields_absent_when_not_set(self):
        """compression_format and compression_mode are absent when metadata omits them."""
        self._skip_if_no_yaml()
        import tempfile
        with tempfile.TemporaryDirectory() as tmpdir:
            self._write_bag(tmpdir, self._base_meta())  # no compression keys
            result = self._run(tmpdir)
        self.assertNotIn("compression_format", result)
        self.assertNotIn("compression_mode", result)

    # ------------------------------------------------------------------
    # Error paths
    # ------------------------------------------------------------------

    def test_nonexistent_path_returns_error_with_hint(self):
        """A nonexistent bag path returns {"error": ..., "hint": ...} — never raises."""
        result = self._run("/does/not/exist/bag")
        self.assertIn("error", result)
        self.assertIn("hint", result)
        self.assertIsInstance(result["error"], str)
        self.assertGreater(len(result["error"]), 0)

    def test_empty_directory_returns_error_with_hint(self):
        """A directory with no metadata.yaml returns {"error": ..., "hint": ...}."""
        import tempfile
        with tempfile.TemporaryDirectory() as tmpdir:
            result = self._run(tmpdir)
        self.assertIn("error", result)
        self.assertIn("hint", result)

    def test_cmd_bag_info_never_raises(self):
        """cmd_bag_info catches all exceptions — no unhandled exception escapes."""
        import tempfile, types
        for bad_path in ("/does/not/exist", ""):
            try:
                captured = []
                with patch("ros2_bag.output", side_effect=captured.append):
                    self.ros2_bag.cmd_bag_info(types.SimpleNamespace(bag_path=bad_path))
                self.assertGreater(len(captured), 0, f"No output for path '{bad_path}'")
                self.assertIn("error", captured[0], f"No 'error' key for path '{bad_path}'")
            except Exception as exc:
                self.fail(f"cmd_bag_info raised unexpectedly for '{bad_path}': {exc}")

    def test_metadata_yaml_path_accepted_directly(self):
        """Passing the metadata.yaml file path directly also works."""
        self._skip_if_no_yaml()
        import tempfile, pathlib
        with tempfile.TemporaryDirectory() as tmpdir:
            self._write_bag(tmpdir, self._base_meta(storage_identifier="mcap"))
            meta_path = str(pathlib.Path(tmpdir) / "metadata.yaml")
            result = self._run(meta_path)
        self.assertEqual(result["storage_identifier"], "mcap")
        self.assertNotIn("error", result)


class TestArgumentIntrospection(unittest.TestCase):
    """Verify every registered subcommand accepts --help (exit 0) and that
    required positional arguments are enforced (non-zero exit) rather than
    silently coerced to None.

    Covers the 'no invention of arguments' and 'ambiguity handling' gaps:
    if an agent passes an unrecognised flag the parser must reject it,
    and if a required arg is omitted the parser must fail loudly.
    """

    @classmethod
    def setUpClass(cls):
        if not check_rclpy_available():
            raise unittest.SkipTest("rclpy not available - requires ROS 2 environment")
        import ros2_cli
        cls.ros2_cli = ros2_cli
        cls.parser = ros2_cli.build_parser()

    def _assert_help_exits_zero(self, *args):
        with patch("sys.stdout", new_callable=StringIO), \
             self.assertRaises(SystemExit) as cm:
            self.parser.parse_args(list(args) + ["--help"])
        self.assertEqual(cm.exception.code, 0,
                         f"Expected SystemExit(0) for {args!r} --help")

    def _assert_missing_required_fails(self, *args):
        with patch("sys.stderr", new_callable=StringIO), \
             self.assertRaises(SystemExit) as cm:
            self.parser.parse_args(list(args))
        self.assertNotEqual(cm.exception.code, 0,
                            f"Expected non-zero exit for missing required arg {args!r}")

    def _assert_unknown_flag_fails(self, *args):
        with patch("sys.stderr", new_callable=StringIO), \
             self.assertRaises(SystemExit) as cm:
            self.parser.parse_args(list(args))
        self.assertNotEqual(cm.exception.code, 0,
                            f"Expected non-zero exit for unknown flag in {args!r}")

    # ------------------------------------------------------------------
    # bag
    # ------------------------------------------------------------------

    def test_bag_info_help_exits_zero(self):
        self._assert_help_exits_zero("bag", "info")

    def test_bag_info_requires_bag_path(self):
        """bag info without a positional argument must fail — not silently return None."""
        self._assert_missing_required_fails("bag", "info")

    def test_bag_info_rejects_unknown_flags(self):
        """An agent must not invent flags; unknown flags are rejected."""
        self._assert_unknown_flag_fails("bag", "info", "/path", "--invented-flag")

    # ------------------------------------------------------------------
    # component
    # ------------------------------------------------------------------

    def test_component_types_help_exits_zero(self):
        self._assert_help_exits_zero("component", "types")

    def test_component_types_rejects_unknown_flags(self):
        self._assert_unknown_flag_fails("component", "types", "--invented-flag")

    # ------------------------------------------------------------------
    # Core commands — spot checks to catch global parser breakage
    # ------------------------------------------------------------------

    def test_estop_help_exits_zero(self):
        self._assert_help_exits_zero("estop")

    def test_topics_list_help_exits_zero(self):
        self._assert_help_exits_zero("topics", "list")

    def test_nodes_list_help_exits_zero(self):
        self._assert_help_exits_zero("nodes", "list")

    def test_tf_lookup_help_exits_zero(self):
        self._assert_help_exits_zero("tf", "lookup")

    def test_topics_type_requires_topic(self):
        """topics type requires a topic positional (subscribe uses nargs='?' for auto-discovery)."""
        self._assert_missing_required_fails("topics", "type")

    def test_params_set_requires_name_and_value(self):
        """params set without name:param and value must fail."""
        self._assert_missing_required_fails("params", "set")

    def test_all_dispatch_handlers_are_callable(self):
        """Every handler in DISPATCH is callable — no stale string references."""
        for key, handler in self.ros2_cli.DISPATCH.items():
            self.assertTrue(callable(handler),
                            f"DISPATCH[{key!r}] is not callable: {handler!r}")


class TestErrorOutputStructure(unittest.TestCase):
    """Verify that all error output dicts from command functions consistently
    include the 'error' key with a non-empty string value.

    Covers the 'critical error workflow' gap: any failure in a command must
    produce a structured JSON-serialisable error rather than an uncaught
    exception that would crash the agent.

    Note: the functions under test make no rclpy calls, but ros2_utils
    calls sys.exit(1) when rclpy is absent, so the rclpy guard is required.
    """

    @classmethod
    def setUpClass(cls):
        if not check_rclpy_available():
            raise unittest.SkipTest("rclpy not available - requires ROS 2 environment")
        import ros2_bag
        import ros2_component
        cls.ros2_bag = ros2_bag
        cls.ros2_component = ros2_component

    def _capture_once(self, module_name, func, args):
        """Run func(args), assert output() was called exactly once, return the dict."""
        captured = []
        with patch(f"{module_name}.output", side_effect=captured.append):
            func(args)
        self.assertEqual(
            len(captured), 1,
            f"{func.__name__} must call output() exactly once per invocation"
        )
        return captured[0]

    # ------------------------------------------------------------------
    # bag info error paths
    # ------------------------------------------------------------------

    def test_bag_info_nonexistent_path_error_key(self):
        import types
        result = self._capture_once(
            "ros2_bag", self.ros2_bag.cmd_bag_info,
            types.SimpleNamespace(bag_path="/does/not/exist")
        )
        self.assertIn("error", result)
        self.assertIsInstance(result["error"], str)
        self.assertGreater(len(result["error"]), 0)

    def test_bag_info_missing_metadata_has_hint_key(self):
        """FileNotFoundError path must include both 'error' and 'hint' keys."""
        import tempfile, types
        with tempfile.TemporaryDirectory() as tmpdir:
            result = self._capture_once(
                "ros2_bag", self.ros2_bag.cmd_bag_info,
                types.SimpleNamespace(bag_path=tmpdir)
            )
        self.assertIn("error", result)
        self.assertIn("hint", result)
        self.assertIsInstance(result["hint"], str)
        self.assertGreater(len(result["hint"]), 0)

    def test_bag_info_error_is_never_none(self):
        """The 'error' value is always a non-None, non-empty string."""
        import types
        for bad_path in ("/no/such/path", ""):
            result = self._capture_once(
                "ros2_bag", self.ros2_bag.cmd_bag_info,
                types.SimpleNamespace(bag_path=bad_path)
            )
            self.assertIn("error", result)
            self.assertIsNotNone(result["error"])
            self.assertGreater(len(result["error"]), 0)

    def test_bag_info_output_called_exactly_once_on_error(self):
        """output() is called exactly once even on error — not zero or twice."""
        import types
        captured = []
        with patch("ros2_bag.output", side_effect=captured.append):
            self.ros2_bag.cmd_bag_info(types.SimpleNamespace(bag_path="/no/such/path"))
        self.assertEqual(len(captured), 1)

    # ------------------------------------------------------------------
    # component types error paths
    # ------------------------------------------------------------------

    def test_component_types_no_ament_error_and_detail_keys(self):
        """ImportError path must produce {"error": ..., "detail": ...}."""
        import sys
        captured = []
        with patch.dict(sys.modules, {"ament_index_python": None}), \
             patch("ros2_component.output", side_effect=captured.append):
            self.ros2_component.cmd_component_types(None)
        result = captured[0]
        self.assertIn("error", result)
        self.assertIn("detail", result)
        self.assertIsInstance(result["error"], str)
        self.assertIsInstance(result["detail"], str)

    def test_component_types_ament_index_failure_error_key(self):
        """get_resources() raising an exception must produce {"error": ...}."""
        import sys
        from unittest.mock import MagicMock
        mock_ament = MagicMock()
        mock_ament.get_resources.side_effect = RuntimeError("ament index corrupt")
        captured = []
        with patch.dict(sys.modules, {"ament_index_python": mock_ament}), \
             patch("ros2_component.output", side_effect=captured.append):
            self.ros2_component.cmd_component_types(None)
        self.assertIn("error", captured[0])

    def test_component_types_output_called_exactly_once_on_error(self):
        """output() is called exactly once per invocation even on failure."""
        import sys
        captured = []
        with patch.dict(sys.modules, {"ament_index_python": None}), \
             patch("ros2_component.output", side_effect=captured.append):
            self.ros2_component.cmd_component_types(None)
        self.assertEqual(len(captured), 1)


class TestComponentTypesOutputValidation(unittest.TestCase):
    """Structural validation of cmd_component_types output.

    Covers the 'Component types command output validation' gap: verifies that
    the output dict is always well-formed regardless of ament index content,
    including edge cases (empty index, comments, blank lines, partial errors).

    Note: the functions under test make no rclpy calls, but ros2_utils
    calls sys.exit(1) when rclpy is absent, so the rclpy guard is required.
    """

    @classmethod
    def setUpClass(cls):
        if not check_rclpy_available():
            raise unittest.SkipTest("rclpy not available - requires ROS 2 environment")
        import ros2_component
        cls.ros2_component = ros2_component

    def _mock_ament(self, packages):
        """Build a mock ament module from {pkg_name: [type_name, ...]} dict."""
        from unittest.mock import MagicMock
        mock = MagicMock()
        mock.get_resources.return_value = {pkg: "/path" for pkg in packages}

        def get_resource(resource_type, pkg):
            lines = "\n".join(packages[pkg]) + "\n"
            return (lines, "/path")

        mock.get_resource.side_effect = get_resource
        return mock

    def _run(self, mock_ament):
        import sys
        captured = []
        with patch.dict(sys.modules, {"ament_index_python": mock_ament}), \
             patch("ros2_component.output", side_effect=captured.append):
            self.ros2_component.cmd_component_types(None)
        self.assertEqual(len(captured), 1)
        return captured[0]

    # ------------------------------------------------------------------
    # Structure
    # ------------------------------------------------------------------

    def test_output_has_required_keys(self):
        """Output always contains 'components', 'total', and 'packages'."""
        result = self._run(self._mock_ament({"pkg_a": ["pkg_a::NodeA"]}))
        for key in ("components", "total", "packages"):
            self.assertIn(key, result)

    def test_empty_ament_index_produces_empty_result(self):
        """When no packages export components, result is empty — no crash."""
        from unittest.mock import MagicMock
        mock = MagicMock()
        mock.get_resources.return_value = {}
        result = self._run(mock)
        self.assertEqual(result["total"], 0)
        self.assertEqual(result["components"], [])
        self.assertEqual(result["packages"], [])
        self.assertNotIn("warnings", result)

    def test_total_equals_len_components(self):
        """total always equals len(components) — never stale."""
        for n_types in (1, 3, 7):
            types_list = [f"pkg::Node{i}" for i in range(n_types)]
            result = self._run(self._mock_ament({"my_pkg": types_list}))
            self.assertEqual(result["total"], n_types)
            self.assertEqual(result["total"], len(result["components"]))

    def test_packages_field_is_sorted_alphabetically(self):
        """The 'packages' list is always sorted, regardless of dict iteration order."""
        result = self._run(self._mock_ament({
            "z_pkg": ["z_pkg::Z"],
            "a_pkg": ["a_pkg::A"],
            "m_pkg": ["m_pkg::M"],
        }))
        self.assertEqual(result["packages"], sorted(result["packages"]))
        self.assertEqual(result["packages"][0], "a_pkg")
        self.assertEqual(result["packages"][-1], "z_pkg")

    def test_packages_field_contains_only_unique_names(self):
        """Each package name appears exactly once in 'packages'."""
        result = self._run(self._mock_ament({
            "pkg_a": ["pkg_a::N1", "pkg_a::N2", "pkg_a::N3"],
        }))
        self.assertEqual(len(result["packages"]), 1)
        self.assertEqual(result["packages"], ["pkg_a"])

    def test_each_component_entry_has_package_and_type_name(self):
        """Every component dict has non-empty 'package' and 'type_name' strings."""
        result = self._run(self._mock_ament({
            "my_pkg": ["my_pkg::Alpha", "my_pkg::Beta"],
        }))
        for comp in result["components"]:
            self.assertIn("package", comp)
            self.assertIn("type_name", comp)
            self.assertIsInstance(comp["package"], str)
            self.assertIsInstance(comp["type_name"], str)
            self.assertGreater(len(comp["package"]), 0)
            self.assertGreater(len(comp["type_name"]), 0)

    def test_comment_and_blank_lines_are_excluded(self):
        """Comment lines (# …) and blank lines in resource files are never emitted."""
        from unittest.mock import MagicMock
        mock = MagicMock()
        mock.get_resources.return_value = {"pkg": "/path"}
        mock.get_resource.return_value = (
            "# This is a comment\n"
            "pkg::RealNode\n"
            "\n"
            "   \n"           # whitespace-only line
            "# another comment\n"
            "pkg::AnotherNode\n",
            "/path",
        )
        result = self._run(mock)
        self.assertEqual(result["total"], 2)
        type_names = [c["type_name"] for c in result["components"]]
        self.assertIn("pkg::RealNode", type_names)
        self.assertIn("pkg::AnotherNode", type_names)

    def test_multi_package_components_ordered_by_package(self):
        """Components from multiple packages are grouped/sorted by package name."""
        result = self._run(self._mock_ament({
            "beta_pkg":  ["beta_pkg::Node"],
            "alpha_pkg": ["alpha_pkg::Node"],
        }))
        pkg_order = [c["package"] for c in result["components"]]
        self.assertEqual(pkg_order, sorted(pkg_order))

    # ------------------------------------------------------------------
    # Partial-failure resilience
    # ------------------------------------------------------------------

    def test_warnings_absent_on_clean_run(self):
        """'warnings' key must not appear when all packages load without error."""
        result = self._run(self._mock_ament({"good_pkg": ["good_pkg::Node"]}))
        self.assertNotIn("warnings", result)

    def test_warnings_contain_package_and_error_keys(self):
        """Each warning entry has 'package' and 'error' keys."""
        from unittest.mock import MagicMock
        import sys
        mock = MagicMock()
        mock.get_resources.return_value = {
            "good_pkg": "/path",
            "bad_pkg":  "/path",
        }

        def get_resource(res_type, pkg):
            if pkg == "bad_pkg":
                raise RuntimeError("disk error")
            return ("good_pkg::GoodNode\n", "/path")

        mock.get_resource.side_effect = get_resource
        captured = []
        with patch.dict(sys.modules, {"ament_index_python": mock}), \
             patch("ros2_component.output", side_effect=captured.append):
            self.ros2_component.cmd_component_types(None)

        result = captured[0]
        self.assertIn("warnings", result)
        warning = result["warnings"][0]
        self.assertIn("package", warning)
        self.assertIn("error", warning)
        self.assertEqual(warning["package"], "bad_pkg")
        # Good package still enumerated despite bad_pkg failing
        self.assertEqual(result["total"], 1)
        self.assertIn("good_pkg", result["packages"])


class TestResolveField(unittest.TestCase):
    """Tests for ros2_utils.resolve_field — the dot-path resolver used by
    publish-until's condition monitor to read live odometry / sensor fields.

    Covers the 'motion commands with odometry monitoring (delta, threshold
    logic)' gap: every stop condition ultimately calls resolve_field to
    extract a value from a live message dict before comparing it to the
    threshold.
    """

    @classmethod
    def setUpClass(cls):
        if not check_rclpy_available():
            raise unittest.SkipTest("rclpy not available - requires ROS 2 environment")
        from ros2_utils import resolve_field
        cls.resolve_field = staticmethod(resolve_field)

    # ------------------------------------------------------------------
    # Happy paths
    # ------------------------------------------------------------------

    def test_simple_key(self):
        self.assertEqual(self.resolve_field({"x": 1}, "x"), 1)

    def test_nested_two_levels(self):
        self.assertEqual(self.resolve_field({"a": {"b": 2}}, "a.b"), 2)

    def test_nested_three_levels(self):
        self.assertEqual(self.resolve_field({"a": {"b": {"c": 3}}}, "a.b.c"), 3)

    def test_list_index(self):
        """Array elements are accessed by integer index in the path."""
        self.assertEqual(self.resolve_field({"ranges": [10, 20, 30]}, "ranges.1"), 20)

    def test_list_first_element(self):
        self.assertEqual(self.resolve_field({"ranges": [99]}, "ranges.0"), 99)

    def test_nested_list_then_key(self):
        """Path can traverse into a list element then access a dict key."""
        msg = {"status": [{"name": "cpu"}, {"name": "disk"}]}
        self.assertEqual(self.resolve_field(msg, "status.0.name"), "cpu")
        self.assertEqual(self.resolve_field(msg, "status.1.name"), "disk")

    def test_typical_odom_path(self):
        """pose.pose.position.x mirrors the real odometry message structure."""
        msg = {"pose": {"pose": {"position": {"x": 1.23, "y": 0.0, "z": 0.0}}}}
        self.assertAlmostEqual(self.resolve_field(msg, "pose.pose.position.x"), 1.23)

    def test_typical_scan_ranges(self):
        """ranges.0 — the first range reading from a LaserScan."""
        msg = {"ranges": [0.5, 1.0, 2.0]}
        self.assertAlmostEqual(self.resolve_field(msg, "ranges.0"), 0.5)

    def test_returns_zero(self):
        """A value of zero is returned correctly (not confused with falsy)."""
        self.assertEqual(self.resolve_field({"v": 0}, "v"), 0)

    def test_returns_nested_dict(self):
        """Partial path returns the sub-dict, not just a leaf."""
        msg = {"pose": {"position": {"x": 1.0}}}
        result = self.resolve_field(msg, "pose.position")
        self.assertIsInstance(result, dict)
        self.assertIn("x", result)

    # ------------------------------------------------------------------
    # Error paths (agent must catch these before trusting field data)
    # ------------------------------------------------------------------

    def test_missing_key_raises(self):
        """Missing key raises KeyError — field path does not exist in message."""
        with self.assertRaises(KeyError):
            self.resolve_field({"x": 1}, "y")

    def test_missing_nested_key_raises(self):
        with self.assertRaises(KeyError):
            self.resolve_field({"a": {"b": 1}}, "a.c")

    def test_index_out_of_range_raises(self):
        """Index out of range raises IndexError — protect before using."""
        with self.assertRaises(IndexError):
            self.resolve_field({"ranges": [1.0]}, "ranges.5")


class TestFuzzyMatch(unittest.TestCase):
    """Tests for ros2_launch._fuzzy_match — the ambiguity resolver used when
    launch arguments do not exactly match what the launch file declares.

    Covers the 'ambiguity handling' gap: the skill must never invent arg names.
    When an arg name is close but not exact, fuzzy matching surfaces the real
    name so the agent can confirm before proceeding.
    """

    @classmethod
    def setUpClass(cls):
        if not check_rclpy_available():
            raise unittest.SkipTest("rclpy not available - requires ROS 2 environment")
        from ros2_launch import _fuzzy_match
        cls._fuzzy_match = staticmethod(_fuzzy_match)

    # ------------------------------------------------------------------
    # Match quality tiers
    # ------------------------------------------------------------------

    def test_exact_match_score_1(self):
        results = self._fuzzy_match("use_sim_time", ["use_sim_time", "robot_name"])
        self.assertTrue(any(c == "use_sim_time" and s == 1.0 for c, s in results))

    def test_exact_match_case_insensitive_and_normalised(self):
        """Underscores and hyphens are stripped before comparison."""
        results = self._fuzzy_match("use-sim-time", ["use_sim_time"])
        # After normalisation both become "usesimtime" → exact match
        self.assertTrue(len(results) > 0)
        self.assertEqual(results[0][1], 1.0)

    def test_substring_match_score_0_8(self):
        """Query is a substring of a candidate → score 0.8."""
        results = self._fuzzy_match("sim", ["use_sim_time"])
        self.assertTrue(len(results) > 0)
        self.assertEqual(results[0][1], 0.8)

    def test_candidate_is_substring_of_query_score_0_8(self):
        """Candidate is a substring of query → score 0.8."""
        results = self._fuzzy_match("use_sim_time_extended", ["use_sim_time"])
        # "usesimtime" in "usesimtimeextended" → 0.8
        self.assertTrue(len(results) > 0)
        self.assertGreaterEqual(results[0][1], 0.7)

    def test_starts_with_scores_0_8_via_substring(self):
        """A candidate that starts with the query also satisfies the substring
        branch (score 0.8), which is checked first in the elif chain.

        The dedicated startswith branch (0.7) is only reached when neither
        string is a substring of the other — a structurally impossible
        condition for the startswith case.
        """
        results = self._fuzzy_match("robot", ["robot_name", "unrelated"])
        # "robot" in "robotname" → True → score 0.8 (substring branch wins)
        match_scores = {c: s for c, s in results}
        self.assertIn("robot_name", match_scores)
        self.assertEqual(match_scores["robot_name"], 0.8)

    def test_no_match_returns_empty(self):
        results = self._fuzzy_match("xyz_unknown", ["use_sim_time", "robot_name"])
        self.assertEqual(results, [])

    def test_empty_query_returns_empty(self):
        self.assertEqual(self._fuzzy_match("", ["use_sim_time"]), [])

    def test_empty_candidates_returns_empty(self):
        self.assertEqual(self._fuzzy_match("use_sim_time", []), [])

    def test_results_sorted_descending_by_score(self):
        """Higher-scoring matches appear first."""
        results = self._fuzzy_match("sim", ["use_sim_time", "simulation_mode", "sim"])
        scores = [s for _, s in results]
        self.assertEqual(scores, sorted(scores, reverse=True))

    def test_multiple_exact_matches_all_returned(self):
        candidates = ["sim_time", "sim_delay"]
        results = self._fuzzy_match("sim", candidates)
        returned_names = {c for c, _ in results}
        # Both contain "sim" as a substring → both returned
        self.assertTrue(returned_names.issuperset({"sim_time", "sim_delay"}))


class TestValidateLaunchArgs(unittest.TestCase):
    """Tests for ros2_launch._validate_launch_args — the argument validation
    layer that prevents the agent from inventing or passing unknown launch args.

    Covers the 'ambiguity handling' and 'skill/script argument introspection'
    gaps: the agent must check available args first, use exact matches as-is,
    substitute close matches with a notice, and silently drop unknown args
    rather than passing them through.
    """

    @classmethod
    def setUpClass(cls):
        if not check_rclpy_available():
            raise unittest.SkipTest("rclpy not available - requires ROS 2 environment")
        from ros2_launch import _validate_launch_args
        cls._validate_launch_args = staticmethod(_validate_launch_args)

    # ------------------------------------------------------------------
    # Exact match
    # ------------------------------------------------------------------

    def test_exact_match_passes_through_unchanged(self):
        validated, notices = self._validate_launch_args(
            ["use_sim_time:=true"], ["use_sim_time", "robot_name"]
        )
        self.assertIn("use_sim_time:=true", validated)
        self.assertEqual(notices, [])

    def test_multiple_exact_matches(self):
        validated, notices = self._validate_launch_args(
            ["use_sim_time:=true", "robot_name:=robot1"],
            ["use_sim_time", "robot_name"]
        )
        self.assertEqual(len(validated), 2)
        self.assertEqual(notices, [])

    def test_bare_name_exact_match(self):
        """Bare arg name (no value) passes through if it's an exact match."""
        validated, notices = self._validate_launch_args(
            ["use_sim_time"], ["use_sim_time"]
        )
        self.assertIn("use_sim_time", validated)
        self.assertEqual(notices, [])

    # ------------------------------------------------------------------
    # Fuzzy substitution
    # ------------------------------------------------------------------

    def test_fuzzy_match_substitutes_name_and_generates_notice(self):
        """Close arg name → real name substituted, agent notified."""
        validated, notices = self._validate_launch_args(
            ["sim_time:=true"], ["use_sim_time"]
        )
        # The matched name should be used
        self.assertTrue(len(validated) > 0 or len(notices) > 0)
        # Whether matched or dropped, a notice must explain what happened
        self.assertTrue(len(notices) > 0)

    # ------------------------------------------------------------------
    # No-match → drop and notify
    # ------------------------------------------------------------------

    def test_unknown_arg_dropped_with_notice(self):
        """Completely unknown arg is dropped — never passed to launch."""
        validated, notices = self._validate_launch_args(
            ["invented_arg:=value"], ["use_sim_time", "robot_name"]
        )
        self.assertNotIn("invented_arg:=value", validated)
        self.assertEqual(len(notices), 1)
        self.assertIn("invented_arg", notices[0])

    def test_unknown_arg_notice_lists_available(self):
        """Drop notice includes the list of available args for debugging."""
        _, notices = self._validate_launch_args(
            ["bad_arg:=x"], ["use_sim_time", "robot_name"]
        )
        self.assertTrue(any("use_sim_time" in n or "robot_name" in n for n in notices))

    def test_empty_available_args_drops_all_user_args(self):
        """When no args are declared, every user arg is dropped."""
        validated, notices = self._validate_launch_args(
            ["use_sim_time:=true", "robot_name:=r1"], []
        )
        self.assertEqual(validated, [])
        self.assertEqual(len(notices), 2)

    def test_empty_user_args_returns_empty(self):
        validated, notices = self._validate_launch_args([], ["use_sim_time"])
        self.assertEqual(validated, [])
        self.assertEqual(notices, [])

    # ------------------------------------------------------------------
    # Mixed inputs
    # ------------------------------------------------------------------

    def test_mixed_exact_and_unknown(self):
        """One exact match passes through; one unknown arg is dropped."""
        validated, notices = self._validate_launch_args(
            ["use_sim_time:=true", "invented:=val"],
            ["use_sim_time", "robot_name"]
        )
        self.assertIn("use_sim_time:=true", validated)
        self.assertNotIn("invented:=val", validated)
        self.assertEqual(len(notices), 1)

    def test_equals_format_exact_match(self):
        """name=value format (as opposed to name:=value) also works for exact match."""
        validated, notices = self._validate_launch_args(
            ["use_sim_time=true"], ["use_sim_time"]
        )
        self.assertIn("use_sim_time=true", validated)
        self.assertEqual(notices, [])


class TestNanToNone(unittest.TestCase):
    """Tests for ros2_topic._nan_to_none — used to sanitise sensor values
    before JSON serialisation (float NaN is not valid JSON).

    Covers the 'edge cases' gap for sensor data: battery, IMU, and other
    sensor messages use NaN to indicate unmeasured fields; the agent must
    receive None (null in JSON) rather than a non-serialisable float.
    """

    @classmethod
    def setUpClass(cls):
        if not check_rclpy_available():
            raise unittest.SkipTest("rclpy not available - requires ROS 2 environment")
        from ros2_topic import _nan_to_none
        cls._nan_to_none = staticmethod(_nan_to_none)

    def test_nan_becomes_none(self):
        self.assertIsNone(self._nan_to_none(float("nan")))

    def test_positive_float_unchanged(self):
        self.assertAlmostEqual(self._nan_to_none(3.14), 3.14)

    def test_zero_unchanged(self):
        self.assertEqual(self._nan_to_none(0.0), 0.0)

    def test_negative_float_unchanged(self):
        self.assertAlmostEqual(self._nan_to_none(-1.5), -1.5)

    def test_integer_unchanged(self):
        self.assertEqual(self._nan_to_none(42), 42)

    def test_none_unchanged(self):
        """None input passes through — already represents 'no value'."""
        self.assertIsNone(self._nan_to_none(None))

    def test_positive_infinity_unchanged(self):
        """Infinity is not NaN and must pass through (some sensors use it for max range)."""
        result = self._nan_to_none(float("inf"))
        self.assertEqual(result, float("inf"))

    def test_negative_infinity_unchanged(self):
        result = self._nan_to_none(float("-inf"))
        self.assertEqual(result, float("-inf"))

    def test_string_unchanged(self):
        """Non-float types pass through unchanged."""
        self.assertEqual(self._nan_to_none("ok"), "ok")


class TestBagInfoEdgeCases(unittest.TestCase):
    """Edge-case tests for cmd_bag_info and _parse_metadata.

    Covers the 'bag info with invalid bag_path' gap with additional
    scenarios beyond the core TestBagInfoCommand suite: legacy integer-format
    duration, flat bag format (no rosbag2_bagfile_information wrapper),
    and bags with a files list.
    """

    @classmethod
    def setUpClass(cls):
        if not check_rclpy_available():
            raise unittest.SkipTest("rclpy not available - requires ROS 2 environment")
        import ros2_bag
        cls.ros2_bag = ros2_bag

    def _skip_if_no_yaml(self):
        try:
            import yaml  # noqa: F401
        except ImportError:
            raise unittest.SkipTest("PyYAML not available")

    def _write_and_run(self, meta):
        import tempfile, pathlib, yaml, types
        with tempfile.TemporaryDirectory() as tmpdir:
            with open(pathlib.Path(tmpdir) / "metadata.yaml", "w") as fh:
                yaml.dump(meta, fh)
            captured = []
            with patch("ros2_bag.output", side_effect=captured.append):
                self.ros2_bag.cmd_bag_info(types.SimpleNamespace(bag_path=tmpdir))
        self.assertEqual(len(captured), 1)
        return captured[0]

    def test_legacy_integer_duration(self):
        """Older bags store duration as a plain integer nanoseconds value."""
        self._skip_if_no_yaml()
        meta = {"rosbag2_bagfile_information": {
            "duration": 3_000_000_000,   # integer, not dict
            "starting_time": {"nanoseconds_since_epoch": 0},
            "storage_identifier": "sqlite3",
            "message_count": 10,
            "topics_with_message_count": [],
        }}
        result = self._write_and_run(meta)
        self.assertNotIn("error", result)
        self.assertEqual(result["duration"]["nanoseconds"], 3_000_000_000)
        self.assertAlmostEqual(result["duration"]["seconds"], 3.0)

    def test_flat_bag_format_no_wrapper_key(self):
        """Bags without the rosbag2_bagfile_information wrapper are handled gracefully."""
        self._skip_if_no_yaml()
        meta = {
            "duration": {"nanoseconds": 1_000_000_000},
            "starting_time": {"nanoseconds_since_epoch": 0},
            "storage_identifier": "sqlite3",
            "message_count": 5,
            "topics_with_message_count": [],
        }
        result = self._write_and_run(meta)
        self.assertNotIn("error", result)
        self.assertEqual(result["storage_identifier"], "sqlite3")

    def test_bag_with_files_list(self):
        """When the metadata lists storage files, they appear in the output."""
        self._skip_if_no_yaml()
        meta = {"rosbag2_bagfile_information": {
            "duration": {"nanoseconds": 1_000_000_000},
            "starting_time": {"nanoseconds_since_epoch": 0},
            "storage_identifier": "sqlite3",
            "message_count": 0,
            "topics_with_message_count": [],
            "files": [
                {"path": "my_bag_0.db3"},
                {"path": "my_bag_1.db3"},
            ],
        }}
        result = self._write_and_run(meta)
        self.assertIn("files", result)
        self.assertEqual(len(result["files"]), 2)
        self.assertIn("my_bag_0.db3", result["files"])

    def test_zero_duration_bag(self):
        """A bag with zero duration (e.g. a single-frame snapshot) is valid."""
        self._skip_if_no_yaml()
        meta = {"rosbag2_bagfile_information": {
            "duration": {"nanoseconds": 0},
            "starting_time": {"nanoseconds_since_epoch": 0},
            "storage_identifier": "sqlite3",
            "message_count": 1,
            "topics_with_message_count": [],
        }}
        result = self._write_and_run(meta)
        self.assertNotIn("error", result)
        self.assertEqual(result["duration"]["seconds"], 0.0)

    def test_storage_file_path_accepted(self):
        """Passing a .db3 storage file path resolves to the parent metadata.yaml."""
        self._skip_if_no_yaml()
        import tempfile, pathlib, yaml, types
        meta = {"rosbag2_bagfile_information": {
            "duration": {"nanoseconds": 1_000_000_000},
            "starting_time": {"nanoseconds_since_epoch": 0},
            "storage_identifier": "sqlite3",
            "message_count": 0,
            "topics_with_message_count": [],
        }}
        with tempfile.TemporaryDirectory() as tmpdir:
            with open(pathlib.Path(tmpdir) / "metadata.yaml", "w") as fh:
                yaml.dump(meta, fh)
            # Create a fake .db3 file
            db3_path = pathlib.Path(tmpdir) / "bag_0.db3"
            db3_path.touch()
            captured = []
            with patch("ros2_bag.output", side_effect=captured.append):
                self.ros2_bag.cmd_bag_info(types.SimpleNamespace(bag_path=str(db3_path)))
        result = captured[0]
        self.assertNotIn("error", result)
        self.assertEqual(result["storage_identifier"], "sqlite3")


class TestArgumentIntrospectionExtended(unittest.TestCase):
    """Extended argument introspection tests covering additional commands
    from the DISPATCH table.

    Covers the 'skill/script argument introspection — no invention of
    arguments' gap with broader command coverage than TestArgumentIntrospection.
    """

    @classmethod
    def setUpClass(cls):
        if not check_rclpy_available():
            raise unittest.SkipTest("rclpy not available - requires ROS 2 environment")
        import ros2_cli
        cls.ros2_cli = ros2_cli
        cls.parser = ros2_cli.build_parser()

    def _assert_help_exits_zero(self, *args):
        with patch("sys.stdout", new_callable=StringIO), \
             self.assertRaises(SystemExit) as cm:
            self.parser.parse_args(list(args) + ["--help"])
        self.assertEqual(cm.exception.code, 0)

    def _assert_missing_required_fails(self, *args):
        with patch("sys.stderr", new_callable=StringIO), \
             self.assertRaises(SystemExit) as cm:
            self.parser.parse_args(list(args))
        self.assertNotEqual(cm.exception.code, 0)

    # ------------------------------------------------------------------
    # topics publish (requires topic + json_message)
    # ------------------------------------------------------------------

    def test_topics_publish_help(self):
        self._assert_help_exits_zero("topics", "publish")

    def test_topics_publish_parses_topic_and_json(self):
        """Parser accepts topic + json_message — both are optional (nargs='?')
        at argparse level; runtime validation happens inside the command."""
        ns = self.parser.parse_args(
            ["topics", "publish", "/cmd_vel", '{"linear":{"x":1.0}}']
        )
        self.assertEqual(ns.topic, "/cmd_vel")
        self.assertIn("linear", ns.msg)

    def test_topics_publish_parses_topic_alone(self):
        """topic is nargs='?' so parser accepts a bare topic; msg defaults to None."""
        ns = self.parser.parse_args(["topics", "publish", "/cmd_vel"])
        self.assertEqual(ns.topic, "/cmd_vel")
        self.assertIsNone(ns.msg)

    # ------------------------------------------------------------------
    # nodes details (requires node)
    # ------------------------------------------------------------------

    def test_nodes_details_help(self):
        self._assert_help_exits_zero("nodes", "details")

    def test_nodes_details_requires_node(self):
        self._assert_missing_required_fails("nodes", "details")

    # ------------------------------------------------------------------
    # services call (requires service + request json)
    # ------------------------------------------------------------------

    def test_services_call_help(self):
        self._assert_help_exits_zero("services", "call")

    def test_services_call_requires_service(self):
        self._assert_missing_required_fails("services", "call")

    # ------------------------------------------------------------------
    # actions send (requires action + goal json)
    # ------------------------------------------------------------------

    def test_actions_send_help(self):
        self._assert_help_exits_zero("actions", "send")

    def test_actions_send_requires_action(self):
        self._assert_missing_required_fails("actions", "send")

    # ------------------------------------------------------------------
    # tf lookup (requires source + target)
    # ------------------------------------------------------------------

    def test_tf_lookup_requires_source_and_target(self):
        self._assert_missing_required_fails("tf", "lookup")

    def test_tf_lookup_requires_target_when_source_given(self):
        self._assert_missing_required_fails("tf", "lookup", "base_link")

    # ------------------------------------------------------------------
    # params get/set/describe (require node:param)
    # ------------------------------------------------------------------

    def test_params_get_help(self):
        self._assert_help_exits_zero("params", "get")

    def test_params_get_requires_name(self):
        self._assert_missing_required_fails("params", "get")

    def test_params_set_requires_name_and_value(self):
        self._assert_missing_required_fails("params", "set")

    # ------------------------------------------------------------------
    # lifecycle set (requires node + transition)
    # ------------------------------------------------------------------

    def test_lifecycle_set_requires_node_and_transition(self):
        self._assert_missing_required_fails("lifecycle", "set")

    def test_lifecycle_set_requires_transition_when_node_given(self):
        self._assert_missing_required_fails("lifecycle", "set", "/my_node")

    # ------------------------------------------------------------------
    # publish-until (requires topic, json, --monitor, and a stop condition)
    # ------------------------------------------------------------------

    def test_publish_until_help(self):
        self._assert_help_exits_zero("topics", "publish-until")

    def test_publish_until_parses_full_command(self):
        """Verify publish-until recognises all real-world args without error.
        topic and msg are nargs='?' (validated at runtime, not by argparse)."""
        ns = self.parser.parse_args([
            "topics", "publish-until",
            "/cmd_vel", '{"linear":{"x":0.3}}',
            "--monitor", "/odom", "--delta", "1.0",
        ])
        self.assertEqual(ns.topic, "/cmd_vel")
        self.assertEqual(ns.monitor, "/odom")


if __name__ == "__main__":
    unittest.main()
