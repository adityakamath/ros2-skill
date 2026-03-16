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
    """Pure-Python tests for ros2_bag helpers — no rclpy required."""

    @classmethod
    def setUpClass(cls):
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
    """Pure-Python tests for ros2_component — no rclpy required."""

    @classmethod
    def setUpClass(cls):
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


if __name__ == "__main__":
    unittest.main()
