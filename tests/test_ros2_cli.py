#!/usr/bin/env python3
"""Unit tests for ros2_cli.py.

Tests cover argument parsing, dispatch table, JSON handling,
and utility functions.
"""

import argparse
import json
import pathlib
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


# ---------------------------------------------------------------------------
# Gap 1 — rclpy mock infrastructure
#
# Captured BEFORE mocks are injected so tests that need a real ROS 2
# environment can still guard against running without it.
# ---------------------------------------------------------------------------

_ROS_AVAILABLE = check_rclpy_available()


def _setup_ros_mocks():
    """Inject MagicMock stubs for rclpy and all ROS 2 message packages.

    Called once at module level when rclpy is not installed.  After this
    runs, ``check_rclpy_available()`` returns True (rclpy is in sys.modules)
    so every ``if not check_rclpy_available(): raise SkipTest`` guard in
    existing test classes is bypassed automatically — no per-class changes
    needed.  Classes that require a *live* ROS 2 environment should guard
    against ``_ROS_AVAILABLE`` instead.
    """
    mocked = [
        'rclpy', 'rclpy.node', 'rclpy.qos', 'rclpy.action',
        'rclpy.duration', 'rclpy.clock', 'rclpy.time',
        'geometry_msgs', 'geometry_msgs.msg',
        'std_msgs', 'std_msgs.msg',
        'sensor_msgs', 'sensor_msgs.msg',
        'nav_msgs', 'nav_msgs.msg',
        'rcl_interfaces', 'rcl_interfaces.msg', 'rcl_interfaces.srv',
        'builtin_interfaces', 'builtin_interfaces.msg',
        'lifecycle_msgs', 'lifecycle_msgs.msg', 'lifecycle_msgs.srv',
        'action_msgs', 'action_msgs.msg',
        'controller_manager_msgs', 'controller_manager_msgs.srv',
        'tf2_ros', 'tf2_geometry_msgs',
        'cv_bridge', 'cv2',
    ]
    for mod in mocked:
        sys.modules.setdefault(mod, MagicMock())


if not _ROS_AVAILABLE:
    _setup_ros_mocks()


# ---------------------------------------------------------------------------
# Gap 4 — MINIMAL_ARGS: exhaustive (cmd, sub, cli_args) table
#
# Every entry in DISPATCH must appear here.  Used by TestExhaustiveParser
# (parse sweep) and TestDispatchTable (symmetry canary).
# ---------------------------------------------------------------------------

MINIMAL_ARGS = [
    # version / estop
    ("version",  None,               ["version"]),
    ("estop",    None,               ["estop"]),
    # topics — canonical
    ("topics",   "list",             ["topics", "list"]),
    ("topics",   "type",             ["topics", "type", "/t"]),
    ("topics",   "details",          ["topics", "details", "/t"]),
    ("topics",   "message",          ["topics", "message", "geometry_msgs/Twist"]),
    ("topics",   "message-structure",["topics", "message-structure", "geometry_msgs/Twist"]),
    ("topics",   "message-struct",   ["topics", "message-struct", "geometry_msgs/Twist"]),
    ("topics",   "subscribe",        ["topics", "subscribe", "/t"]),
    ("topics",   "publish",          ["topics", "publish", "/t", "{}"]),
    ("topics",   "publish-sequence", ["topics", "publish-sequence", "/t", "[]", "[]"]),
    ("topics",   "publish-until",    ["topics", "publish-until", "/t", "{}", "--monitor", "/m", "--field", "x", "--delta", "1"]),
    ("topics",   "publish-continuous",["topics", "publish-continuous", "/t", "{}"]),
    ("topics",   "hz",               ["topics", "hz", "/t"]),
    ("topics",   "find",             ["topics", "find", "std_msgs/msg/String"]),
    ("topics",   "capture-image",    ["topics", "capture-image", "--topic", "/camera/image_raw", "--output", "img.jpg"]),
    ("topics",   "diag-list",        ["topics", "diag-list"]),
    ("topics",   "diag",             ["topics", "diag"]),
    ("topics",   "battery-list",     ["topics", "battery-list"]),
    ("topics",   "battery",          ["topics", "battery"]),
    ("topics",   "bw",               ["topics", "bw", "/t"]),
    ("topics",   "delay",            ["topics", "delay", "/t"]),
    ("topics",   "qos-check",        ["topics", "qos-check", "/t"]),
    # topics — aliases
    ("topics",   "echo",             ["topics", "echo", "/t"]),
    ("topics",   "pub",              ["topics", "pub", "/t", "{}"]),
    ("topics",   "ls",               ["topics", "ls"]),
    ("topics",   "info",             ["topics", "info", "/t"]),
    # services
    ("services", "list",             ["services", "list"]),
    ("services", "type",             ["services", "type", "/s"]),
    ("services", "details",          ["services", "details", "/s"]),
    ("services", "call",             ["services", "call", "/s", "{}"]),
    ("services", "find",             ["services", "find", "std_srvs/srv/Empty"]),
    ("services", "echo",             ["services", "echo", "/s"]),
    ("services", "ls",               ["services", "ls"]),
    ("services", "info",             ["services", "info", "/s"]),
    # nodes
    ("nodes",    "list",             ["nodes", "list"]),
    ("nodes",    "details",          ["nodes", "details", "/n"]),
    ("nodes",    "info",             ["nodes", "info", "/n"]),
    ("nodes",    "ls",               ["nodes", "ls"]),
    # lifecycle
    ("lifecycle","nodes",            ["lifecycle", "nodes"]),
    ("lifecycle","list",             ["lifecycle", "list"]),
    ("lifecycle","get",              ["lifecycle", "get", "/n"]),
    ("lifecycle","set",              ["lifecycle", "set", "/n", "configure"]),
    ("lifecycle","ls",               ["lifecycle", "ls"]),
    # control
    ("control",  "list-controller-types",       ["control", "list-controller-types"]),
    ("control",  "list-controllers",            ["control", "list-controllers"]),
    ("control",  "list-hardware-components",    ["control", "list-hardware-components"]),
    ("control",  "list-hardware-interfaces",    ["control", "list-hardware-interfaces"]),
    ("control",  "load-controller",             ["control", "load-controller", "c"]),
    ("control",  "unload-controller",           ["control", "unload-controller", "c"]),
    ("control",  "configure-controller",        ["control", "configure-controller", "c"]),
    ("control",  "reload-controller-libraries", ["control", "reload-controller-libraries"]),
    ("control",  "set-controller-state",        ["control", "set-controller-state", "c", "active"]),
    ("control",  "set-hardware-component-state",["control", "set-hardware-component-state", "h", "active"]),
    ("control",  "switch-controllers",          ["control", "switch-controllers", "--activate", "a"]),
    ("control",  "view-controller-chains",      ["control", "view-controller-chains"]),
    ("control",  "load",                        ["control", "load", "c"]),
    ("control",  "unload",                      ["control", "unload", "c"]),
    # params
    ("params",   "list",             ["params", "list", "/n"]),
    ("params",   "get",              ["params", "get", "/n:p"]),
    ("params",   "set",              ["params", "set", "/n:p", "v"]),
    ("params",   "describe",         ["params", "describe", "/n", "p"]),
    ("params",   "dump",             ["params", "dump", "/n"]),
    ("params",   "load",             ["params", "load", "/n", "{}"]),
    ("params",   "delete",           ["params", "delete", "/n", "p"]),
    ("params",   "preset-save",      ["params", "preset-save", "/n", "p"]),
    ("params",   "preset-load",      ["params", "preset-load", "/n", "p"]),
    ("params",   "preset-list",      ["params", "preset-list"]),
    ("params",   "preset-delete",    ["params", "preset-delete", "p"]),
    ("params",   "find",             ["params", "find", "velocity"]),
    ("params",   "ls",               ["params", "ls", "/n"]),
    # actions
    ("actions",  "list",             ["actions", "list"]),
    ("actions",  "details",          ["actions", "details", "/a"]),
    ("actions",  "send",             ["actions", "send", "/a", "{}"]),
    ("actions",  "type",             ["actions", "type", "/a"]),
    ("actions",  "cancel",           ["actions", "cancel", "/a"]),
    ("actions",  "echo",             ["actions", "echo", "/a"]),
    ("actions",  "find",             ["actions", "find", "pkg/action/Type"]),
    ("actions",  "info",             ["actions", "info", "/a"]),
    ("actions",  "ls",               ["actions", "ls"]),
    # doctor / wtf
    ("doctor",   None,               ["doctor"]),
    ("doctor",   "hello",            ["doctor", "hello"]),
    ("wtf",      None,               ["wtf"]),
    ("wtf",      "hello",            ["wtf", "hello"]),
    # multicast
    ("multicast","send",             ["multicast", "send"]),
    ("multicast","receive",          ["multicast", "receive"]),
    # tf
    ("tf",       "list",             ["tf", "list"]),
    ("tf",       "ls",               ["tf", "ls"]),
    ("tf",       "lookup",           ["tf", "lookup", "s", "t"]),
    ("tf",       "get",              ["tf", "get", "s", "t"]),
    ("tf",       "echo",             ["tf", "echo", "s", "t"]),
    ("tf",       "monitor",          ["tf", "monitor", "f"]),
    ("tf",       "static",           ["tf", "static", "--from", "s", "--to", "t", "--xyz", "0", "0", "0", "--rpy", "0", "0", "0"]),
    ("tf",       "euler-from-quaternion",     ["tf", "euler-from-quaternion", "0", "0", "0", "1"]),
    ("tf",       "euler-from-quaternion-deg", ["tf", "euler-from-quaternion-deg", "0", "0", "0", "1"]),
    ("tf",       "quaternion-from-euler",     ["tf", "quaternion-from-euler", "0", "0", "0"]),
    ("tf",       "quaternion-from-euler-deg", ["tf", "quaternion-from-euler-deg", "0", "0", "0"]),
    ("tf",       "transform-point",  ["tf", "transform-point", "t", "s", "0", "0", "0"]),
    ("tf",       "transform-vector", ["tf", "transform-vector", "t", "s", "0", "0", "0"]),
    ("tf",       "tree",             ["tf", "tree"]),
    ("tf",       "validate",         ["tf", "validate"]),
    # launch
    ("launch",   "new",      ["launch", "new", "p", "f.launch.py"]),
    ("launch",   "list",     ["launch", "list"]),
    ("launch",   "ls",       ["launch", "ls"]),
    ("launch",   "kill",     ["launch", "kill", "s"]),
    ("launch",   "restart",  ["launch", "restart", "s"]),
    ("launch",   "foxglove", ["launch", "foxglove"]),
    # run
    ("run",      "new",      ["run", "new", "p", "e"]),
    ("run",      "list",     ["run", "list"]),
    ("run",      "ls",       ["run", "ls"]),
    ("run",      "kill",     ["run", "kill", "s"]),
    ("run",      "restart",  ["run", "restart", "s"]),
    # interface
    ("interface","list",     ["interface", "list"]),
    ("interface","ls",       ["interface", "ls"]),
    ("interface","show",     ["interface", "show", "std_msgs/String"]),
    ("interface","proto",    ["interface", "proto", "std_msgs/String"]),
    ("interface","packages", ["interface", "packages"]),
    ("interface","package",  ["interface", "package", "std_msgs"]),
    # bag
    ("bag",      "info",     ["bag", "info", "/path/to/bag"]),
    # component
    ("component","types",    ["component", "types"]),
    ("component","list",     ["component", "list"]),
    ("component","ls",       ["component", "ls"]),
    ("component","load",     ["component", "load", "/ComponentManager", "demo_nodes_cpp", "demo_nodes_cpp::Talker"]),
    ("component","unload",   ["component", "unload", "/ComponentManager", "1"]),
    ("component","standalone",["component", "standalone", "demo_nodes_cpp", "demo_nodes_cpp::Talker"]),
    # pkg
    ("pkg",       "list",        ["pkg", "list"]),
    ("pkg",       "ls",          ["pkg", "ls"]),
    ("pkg",       "prefix",      ["pkg", "prefix", "std_msgs"]),
    ("pkg",       "executables", ["pkg", "executables", "turtlesim"]),
    ("pkg",       "xml",         ["pkg", "xml", "std_msgs"]),
    # daemon
    ("daemon",   "status",   ["daemon", "status"]),
    ("daemon",   "start",    ["daemon", "start"]),
    ("daemon",   "stop",     ["daemon", "stop"]),
]


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
        # Requires *live* ROS 2: resolves real message packages by name.
        if not _ROS_AVAILABLE:
            raise unittest.SkipTest("live rclpy required — real message packages must be importable")
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

    def test_component_list_parsing(self):
        args = self.parser.parse_args(["component", "list"])
        self.assertEqual(args.command, "component")
        self.assertEqual(args.subcommand, "list")

    def test_component_ls_alias_parsing(self):
        args = self.parser.parse_args(["component", "ls"])
        self.assertEqual(args.subcommand, "ls")

    def test_component_list_timeout_default(self):
        args = self.parser.parse_args(["component", "list"])
        self.assertEqual(args.timeout, 5.0)

    def test_component_load_parsing(self):
        args = self.parser.parse_args([
            "component", "load",
            "/my_container", "demo_nodes_cpp", "demo_nodes_cpp::Talker"
        ])
        self.assertEqual(args.subcommand, "load")
        self.assertEqual(args.container, "/my_container")
        self.assertEqual(args.package_name, "demo_nodes_cpp")
        self.assertEqual(args.plugin_name, "demo_nodes_cpp::Talker")

    def test_component_load_optional_flags(self):
        args = self.parser.parse_args([
            "component", "load",
            "/c", "pkg", "pkg::Node",
            "--node-name", "my_node",
            "--node-namespace", "/ns",
            "--log-level", "10",
        ])
        self.assertEqual(args.node_name, "my_node")
        self.assertEqual(args.node_namespace, "/ns")
        self.assertEqual(args.log_level, 10)

    def test_component_load_remap_dest(self):
        args = self.parser.parse_args([
            "component", "load", "/c", "pkg", "pkg::Node",
            "--remap", "/from:=/to",
        ])
        self.assertEqual(args.remap_rules, ["/from:=/to"])

    def test_component_unload_parsing(self):
        args = self.parser.parse_args(["component", "unload", "/my_container", "42"])
        self.assertEqual(args.subcommand, "unload")
        self.assertEqual(args.container, "/my_container")
        self.assertEqual(args.unique_id, 42)

    def test_component_unload_unique_id_is_int(self):
        args = self.parser.parse_args(["component", "unload", "/c", "7"])
        self.assertIsInstance(args.unique_id, int)

    def test_component_load_log_level_default_is_zero_int(self):
        # Regression test: omitting --log-level must produce int 0, not string "".
        # LoadNode.Request.log_level is uint8; passing a string causes a
        # PyLong_Check assertion failure (exit code 134) in the C bindings.
        args = self.parser.parse_args([
            "component", "load", "/c", "pkg", "pkg::Node"
        ])
        self.assertIsInstance(args.log_level, int)
        self.assertEqual(args.log_level, 0)

    def test_component_standalone_parsing(self):
        args = self.parser.parse_args([
            "component", "standalone",
            "demo_nodes_cpp", "demo_nodes_cpp::Talker"
        ])
        self.assertEqual(args.subcommand, "standalone")
        self.assertEqual(args.package_name, "demo_nodes_cpp")
        self.assertEqual(args.plugin_name, "demo_nodes_cpp::Talker")

    def test_component_standalone_default_container_type(self):
        args = self.parser.parse_args(["component", "standalone", "pkg", "pkg::Node"])
        self.assertEqual(args.container_type, "component_container")

    def test_component_standalone_container_type_mt(self):
        args = self.parser.parse_args([
            "component", "standalone", "pkg", "pkg::Node",
            "--container-type", "component_container_mt"
        ])
        self.assertEqual(args.container_type, "component_container_mt")

    def test_component_standalone_log_level_default_is_zero_int(self):
        # Regression guard: must be int 0, not string "" — same crash risk as component load
        args = self.parser.parse_args(["component", "standalone", "pkg", "pkg::Node"])
        self.assertIsInstance(args.log_level, int)
        self.assertEqual(args.log_level, 0)

    def test_component_standalone_timeout_default(self):
        args = self.parser.parse_args(["component", "standalone", "pkg", "pkg::Node"])
        self.assertEqual(args.timeout, 10.0)

    def test_component_dispatch_new_entries(self):
        D = self.ros2_cli.DISPATCH
        for key in [
            ("component", "list"),
            ("component", "ls"),
            ("component", "load"),
            ("component", "unload"),
            ("component", "standalone"),
        ]:
            self.assertIn(key, D, f"Missing dispatch entry: {key}")
            self.assertTrue(callable(D[key]))


class TestPkgParsing(unittest.TestCase):
    """Parser argument and DISPATCH wiring tests for the pkg subcommands."""

    @classmethod
    def setUpClass(cls):
        if not check_rclpy_available():
            raise unittest.SkipTest("rclpy not available - requires ROS 2 environment")
        import ros2_cli
        cls.ros2_cli = ros2_cli
        cls.parser = ros2_cli.build_parser()

    def test_pkg_list_parsing(self):
        args = self.parser.parse_args(["pkg", "list"])
        self.assertEqual(args.command, "pkg")
        self.assertEqual(args.subcommand, "list")

    def test_pkg_ls_alias(self):
        args = self.parser.parse_args(["pkg", "ls"])
        self.assertEqual(args.command, "pkg")
        self.assertEqual(args.subcommand, "ls")

    def test_pkg_prefix_parsing(self):
        args = self.parser.parse_args(["pkg", "prefix", "nav2_bringup"])
        self.assertEqual(args.command, "pkg")
        self.assertEqual(args.subcommand, "prefix")
        self.assertEqual(args.package, "nav2_bringup")

    def test_pkg_executables_parsing(self):
        args = self.parser.parse_args(["pkg", "executables", "turtlesim"])
        self.assertEqual(args.command, "pkg")
        self.assertEqual(args.subcommand, "executables")
        self.assertEqual(args.package, "turtlesim")

    def test_pkg_xml_parsing(self):
        args = self.parser.parse_args(["pkg", "xml", "std_msgs"])
        self.assertEqual(args.command, "pkg")
        self.assertEqual(args.subcommand, "xml")
        self.assertEqual(args.package, "std_msgs")

    def test_pkg_dispatch(self):
        D = self.ros2_cli.DISPATCH
        for subcommand in ("list", "ls", "prefix", "executables", "xml"):
            with self.subTest(subcommand=subcommand):
                self.assertIn(("pkg", subcommand), D)
                self.assertTrue(callable(D[("pkg", subcommand)]))


class TestPkgLogic(unittest.TestCase):
    """Pure-Python logic tests for ros2_pkg with mocked ament_index_python.

    No rclpy calls are made by the functions under test, but ros2_utils
    (imported by ros2_pkg) calls sys.exit(1) when rclpy is absent, so the
    rclpy guard is still required to import the module safely.
    """

    @classmethod
    def setUpClass(cls):
        if not check_rclpy_available():
            raise unittest.SkipTest("rclpy not available - requires ROS 2 environment")
        import ros2_pkg
        cls.ros2_pkg = ros2_pkg

    # ------------------------------------------------------------------
    # pkg list
    # ------------------------------------------------------------------

    def test_pkg_list_no_ament(self):
        """When ament_index_python is missing, output contains an error key."""
        captured = []
        with patch.dict(sys.modules, {"ament_index_python": None,
                                       "ament_index_python.packages": None}), \
             patch("ros2_pkg.output", side_effect=captured.append):
            self.ros2_pkg.cmd_pkg_list(None)
        self.assertEqual(len(captured), 1)
        self.assertIn("error", captured[0])

    def test_pkg_list_returns_sorted_packages(self):
        """Packages are sorted alphabetically and total matches count."""
        mock_pkgs = MagicMock()
        mock_pkgs.get_packages_with_prefixes.return_value = {
            "zebra_pkg": "/opt/ros/humble",
            "alpha_pkg": "/opt/ros/humble",
            "middle_pkg": "/opt/ros/humble",
        }
        captured = []
        with patch.dict(sys.modules, {
                "ament_index_python.packages": mock_pkgs}), \
             patch("ros2_pkg.output", side_effect=captured.append):
            self.ros2_pkg.cmd_pkg_list(None)
        result = captured[0]
        self.assertIn("packages", result)
        self.assertEqual(result["packages"], ["alpha_pkg", "middle_pkg", "zebra_pkg"])
        self.assertEqual(result["total"], 3)

    # ------------------------------------------------------------------
    # pkg prefix
    # ------------------------------------------------------------------

    def test_pkg_prefix_found(self):
        """Returns package name and prefix path."""
        mock_pkgs = MagicMock()
        mock_pkgs.get_package_prefix.return_value = "/opt/ros/humble"
        args = argparse.Namespace(package="turtlesim")
        captured = []
        with patch.dict(sys.modules, {"ament_index_python.packages": mock_pkgs}), \
             patch("ros2_pkg.output", side_effect=captured.append):
            self.ros2_pkg.cmd_pkg_prefix(args)
        result = captured[0]
        self.assertEqual(result["package"], "turtlesim")
        self.assertEqual(result["prefix"], "/opt/ros/humble")

    def test_pkg_prefix_not_found(self):
        """KeyError from ament_index produces an error with the package name."""
        mock_pkgs = MagicMock()
        mock_pkgs.get_package_prefix.side_effect = KeyError("nonexistent_pkg")
        args = argparse.Namespace(package="nonexistent_pkg")
        captured = []
        with patch.dict(sys.modules, {"ament_index_python.packages": mock_pkgs}), \
             patch("ros2_pkg.output", side_effect=captured.append):
            self.ros2_pkg.cmd_pkg_prefix(args)
        self.assertIn("error", captured[0])
        self.assertIn("nonexistent_pkg", captured[0]["error"])

    # ------------------------------------------------------------------
    # pkg executables
    # ------------------------------------------------------------------

    def test_pkg_executables_found(self):
        """Executable files in lib/<pkg>/ are listed; non-executables are excluded."""
        import tempfile
        mock_pkgs = MagicMock()
        with tempfile.TemporaryDirectory() as tmpdir:
            lib_dir = pathlib.Path(tmpdir) / "lib" / "mypkg"
            lib_dir.mkdir(parents=True)
            exe = lib_dir / "my_node"
            exe.write_text("#!/bin/sh")
            exe.chmod(0o755)
            non_exe = lib_dir / "readme.txt"
            non_exe.write_text("not a binary")
            non_exe.chmod(0o644)
            mock_pkgs.get_package_prefix.return_value = tmpdir
            args = argparse.Namespace(package="mypkg")
            captured = []
            with patch.dict(sys.modules, {"ament_index_python.packages": mock_pkgs}), \
                 patch("ros2_pkg.output", side_effect=captured.append):
                self.ros2_pkg.cmd_pkg_executables(args)
        result = captured[0]
        self.assertEqual(result["package"], "mypkg")
        self.assertIn("my_node", result["executables"])
        self.assertNotIn("readme.txt", result["executables"])
        self.assertEqual(result["total"], 1)

    def test_pkg_executables_no_lib_dir(self):
        """When lib/<pkg>/ does not exist, returns empty list without error."""
        import tempfile
        mock_pkgs = MagicMock()
        with tempfile.TemporaryDirectory() as tmpdir:
            mock_pkgs.get_package_prefix.return_value = tmpdir
            args = argparse.Namespace(package="mypkg")
            captured = []
            with patch.dict(sys.modules, {"ament_index_python.packages": mock_pkgs}), \
                 patch("ros2_pkg.output", side_effect=captured.append):
                self.ros2_pkg.cmd_pkg_executables(args)
        result = captured[0]
        self.assertEqual(result["executables"], [])
        self.assertEqual(result["total"], 0)

    def test_pkg_executables_not_found(self):
        """Unknown package produces an error."""
        mock_pkgs = MagicMock()
        mock_pkgs.get_package_prefix.side_effect = KeyError("ghost_pkg")
        args = argparse.Namespace(package="ghost_pkg")
        captured = []
        with patch.dict(sys.modules, {"ament_index_python.packages": mock_pkgs}), \
             patch("ros2_pkg.output", side_effect=captured.append):
            self.ros2_pkg.cmd_pkg_executables(args)
        self.assertIn("error", captured[0])

    # ------------------------------------------------------------------
    # pkg xml
    # ------------------------------------------------------------------

    def test_pkg_xml_found(self):
        """Returns package name, path, and xml content."""
        import tempfile
        mock_pkgs = MagicMock()
        xml_content = '<?xml version="1.0"?>\n<package format="3"><name>mypkg</name></package>\n'
        with tempfile.TemporaryDirectory() as tmpdir:
            share_dir = pathlib.Path(tmpdir) / "share" / "mypkg"
            share_dir.mkdir(parents=True)
            (share_dir / "package.xml").write_text(xml_content, encoding="utf-8")
            mock_pkgs.get_package_share_directory.return_value = str(share_dir)
            args = argparse.Namespace(package="mypkg")
            captured = []
            with patch.dict(sys.modules, {"ament_index_python.packages": mock_pkgs}), \
                 patch("ros2_pkg.output", side_effect=captured.append):
                self.ros2_pkg.cmd_pkg_xml(args)
        result = captured[0]
        self.assertEqual(result["package"], "mypkg")
        self.assertIn("<name>mypkg</name>", result["xml"])
        self.assertIn("path", result)

    def test_pkg_xml_package_not_found(self):
        """Unknown package produces an error."""
        mock_pkgs = MagicMock()
        mock_pkgs.get_package_share_directory.side_effect = KeyError("ghost")
        args = argparse.Namespace(package="ghost")
        captured = []
        with patch.dict(sys.modules, {"ament_index_python.packages": mock_pkgs}), \
             patch("ros2_pkg.output", side_effect=captured.append):
            self.ros2_pkg.cmd_pkg_xml(args)
        self.assertIn("error", captured[0])

    def test_pkg_xml_missing_file(self):
        """Package in ament index but package.xml absent on disk → error."""
        import tempfile
        mock_pkgs = MagicMock()
        with tempfile.TemporaryDirectory() as tmpdir:
            share_dir = pathlib.Path(tmpdir) / "share" / "mypkg"
            share_dir.mkdir(parents=True)
            # No package.xml written
            mock_pkgs.get_package_share_directory.return_value = str(share_dir)
            args = argparse.Namespace(package="mypkg")
            captured = []
            with patch.dict(sys.modules, {"ament_index_python.packages": mock_pkgs}), \
                 patch("ros2_pkg.output", side_effect=captured.append):
                self.ros2_pkg.cmd_pkg_xml(args)
        self.assertIn("error", captured[0])


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


class TestComponentListLogic(unittest.TestCase):
    """Pure-logic tests for cmd_component_list (mocked ROS 2)."""

    @classmethod
    def setUpClass(cls):
        if not check_rclpy_available():
            raise unittest.SkipTest("rclpy not available - requires ROS 2 environment")
        import ros2_component
        cls.ros2_component = ros2_component

    def _run_no_containers(self):
        from unittest.mock import MagicMock, patch
        mock_node = MagicMock()
        mock_node.get_service_names_and_types.return_value = []
        captured = []
        mock_srv = MagicMock()
        mock_srv.ListNodes = MagicMock()
        mock_srv.ListNodes.Request = MagicMock(return_value=MagicMock())
        with patch("ros2_component.ros2_context") as ctx, \
             patch("ros2_component.ROS2CLI", return_value=mock_node), \
             patch("ros2_component.output", side_effect=captured.append), \
             patch.dict("sys.modules", {"composition_interfaces.srv": mock_srv}):
            ctx.return_value.__enter__ = MagicMock(return_value=None)
            ctx.return_value.__exit__ = MagicMock(return_value=False)
            args = MagicMock(timeout=5.0)
            self.ros2_component.cmd_component_list(args)
        return captured[0]

    def test_no_containers_returns_empty_list(self):
        result = self._run_no_containers()
        self.assertEqual(result["containers"], [])
        self.assertEqual(result["total_containers"], 0)
        self.assertEqual(result["total_components"], 0)

    def test_no_containers_has_hint(self):
        result = self._run_no_containers()
        self.assertIn("hint", result)

    def test_output_has_required_keys(self):
        result = self._run_no_containers()
        for key in ("containers", "total_containers", "total_components"):
            self.assertIn(key, result)

    def test_composition_interfaces_import_error(self):
        from unittest.mock import patch
        captured = []
        with patch.dict("sys.modules", {"composition_interfaces.srv": None}), \
             patch("ros2_component.output", side_effect=captured.append):
            args = MagicMock(timeout=5.0)
            self.ros2_component.cmd_component_list(args)
        self.assertEqual(len(captured), 1)
        self.assertIn("error", captured[0])
        self.assertIn("hint", captured[0])


class TestComponentLoadLogic(unittest.TestCase):
    """Pure-logic tests for cmd_component_load (mocked ROS 2)."""

    @classmethod
    def setUpClass(cls):
        if not check_rclpy_available():
            raise unittest.SkipTest("rclpy not available - requires ROS 2 environment")
        import ros2_component
        cls.ros2_component = ros2_component

    def _make_args(self, container="/c", package="pkg", plugin="pkg::Node",
                   node_name="", node_namespace="", remap_rules=None,
                   log_level=0, timeout=5.0):
        from unittest.mock import MagicMock
        args = MagicMock()
        args.container = container
        args.package_name = package
        args.plugin_name = plugin
        args.node_name = node_name
        args.node_namespace = node_namespace
        args.remap_rules = remap_rules or []
        args.log_level = log_level
        args.timeout = timeout
        return args

    def _run(self, resp_success, full_node_name="", unique_id=1, error_message="",
             svc_available=True, future_done=True):
        from unittest.mock import MagicMock, patch
        mock_resp = MagicMock()
        mock_resp.success = resp_success
        mock_resp.full_node_name = full_node_name
        mock_resp.unique_id = unique_id
        mock_resp.error_message = error_message

        mock_future = MagicMock()
        mock_future.done.return_value = future_done
        mock_future.result.return_value = mock_resp

        mock_client = MagicMock()
        mock_client.wait_for_service.return_value = svc_available
        mock_client.call_async.return_value = mock_future

        mock_node = MagicMock()
        mock_node.create_client.return_value = mock_client

        mock_srv = MagicMock()
        mock_srv.LoadNode = MagicMock()
        mock_srv.LoadNode.Request = MagicMock(return_value=MagicMock())

        captured = []
        with patch("ros2_component.ros2_context") as ctx, \
             patch("ros2_component.ROS2CLI", return_value=mock_node), \
             patch("ros2_component.output", side_effect=captured.append), \
             patch.dict("sys.modules", {"composition_interfaces.srv": mock_srv}):
            ctx.return_value.__enter__ = MagicMock(return_value=None)
            ctx.return_value.__exit__ = MagicMock(return_value=False)
            self.ros2_component.cmd_component_load(self._make_args())
        return captured[0]

    def test_successful_load_has_required_keys(self):
        result = self._run(True, full_node_name="/c/talker", unique_id=1)
        self.assertTrue(result["success"])
        for key in ("container", "package_name", "plugin_name", "full_node_name", "unique_id"):
            self.assertIn(key, result)

    def test_failed_load_returns_success_false_with_error_message(self):
        result = self._run(False, error_message="Plugin not found")
        self.assertFalse(result["success"])
        self.assertIn("error_message", result)
        self.assertEqual(result["error_message"], "Plugin not found")

    def test_service_unavailable_returns_error_with_hint(self):
        result = self._run(True, svc_available=False)
        self.assertIn("error", result)
        self.assertIn("hint", result)

    def test_import_error_returns_error_and_hint(self):
        from unittest.mock import patch
        captured = []
        with patch.dict("sys.modules", {"composition_interfaces.srv": None}), \
             patch("ros2_component.output", side_effect=captured.append):
            self.ros2_component.cmd_component_load(self._make_args())
        self.assertIn("error", captured[0])
        self.assertIn("hint", captured[0])


class TestComponentUnloadLogic(unittest.TestCase):
    """Pure-logic tests for cmd_component_unload (mocked ROS 2)."""

    @classmethod
    def setUpClass(cls):
        if not check_rclpy_available():
            raise unittest.SkipTest("rclpy not available - requires ROS 2 environment")
        import ros2_component
        cls.ros2_component = ros2_component

    def _make_args(self, container="/c", unique_id=1, timeout=5.0):
        from unittest.mock import MagicMock
        args = MagicMock()
        args.container = container
        args.unique_id = unique_id
        args.timeout = timeout
        return args

    def _run(self, resp_success, error_message="", svc_available=True, future_done=True):
        from unittest.mock import MagicMock, patch
        mock_resp = MagicMock()
        mock_resp.success = resp_success
        mock_resp.error_message = error_message

        mock_future = MagicMock()
        mock_future.done.return_value = future_done
        mock_future.result.return_value = mock_resp

        mock_client = MagicMock()
        mock_client.wait_for_service.return_value = svc_available
        mock_client.call_async.return_value = mock_future

        mock_node = MagicMock()
        mock_node.create_client.return_value = mock_client

        mock_srv = MagicMock()
        mock_srv.UnloadNode = MagicMock()
        mock_srv.UnloadNode.Request = MagicMock(return_value=MagicMock())

        captured = []
        with patch("ros2_component.ros2_context") as ctx, \
             patch("ros2_component.ROS2CLI", return_value=mock_node), \
             patch("ros2_component.output", side_effect=captured.append), \
             patch.dict("sys.modules", {"composition_interfaces.srv": mock_srv}):
            ctx.return_value.__enter__ = MagicMock(return_value=None)
            ctx.return_value.__exit__ = MagicMock(return_value=False)
            self.ros2_component.cmd_component_unload(self._make_args())
        return captured[0]

    def test_successful_unload_has_required_keys(self):
        result = self._run(True)
        self.assertTrue(result["success"])
        for key in ("container", "unique_id"):
            self.assertIn(key, result)

    def test_failed_unload_returns_success_false_with_error_message(self):
        result = self._run(False, error_message="Component not found")
        self.assertFalse(result["success"])
        self.assertEqual(result["error_message"], "Component not found")

    def test_unique_id_preserved_in_output(self):
        result = self._run(True)
        self.assertEqual(result["unique_id"], 1)

    def test_service_unavailable_returns_error_with_hint(self):
        result = self._run(True, svc_available=False)
        self.assertIn("error", result)
        self.assertIn("hint", result)

    def test_import_error_returns_error_and_hint(self):
        from unittest.mock import patch
        captured = []
        with patch.dict("sys.modules", {"composition_interfaces.srv": None}), \
             patch("ros2_component.output", side_effect=captured.append):
            self.ros2_component.cmd_component_unload(self._make_args())
        self.assertIn("error", captured[0])
        self.assertIn("hint", captured[0])


class TestComponentStandaloneLogic(unittest.TestCase):
    """Pure-logic tests for cmd_component_standalone (mocked ROS 2 + tmux)."""

    @classmethod
    def setUpClass(cls):
        if not check_rclpy_available():
            raise unittest.SkipTest("rclpy not available - requires ROS 2 environment")
        import ros2_component
        cls.ros2_component = ros2_component

    def _make_args(self, package="demo_nodes_cpp", plugin="demo_nodes_cpp::Talker",
                   container_type="component_container", node_name="",
                   node_namespace="", remap_rules=None, log_level=0, timeout=10.0):
        from unittest.mock import MagicMock
        args = MagicMock()
        args.package_name   = package
        args.plugin_name    = plugin
        args.container_type = container_type
        args.node_name      = node_name
        args.node_namespace = node_namespace
        args.remap_rules    = remap_rules or []
        args.log_level      = log_level
        args.timeout        = timeout
        return args

    def _run(self, resp_success=True, full_node_name="/talker",
             unique_id=1, error_message="",
             tmux_ok=True, session_alive=True, container_ready=True,
             container_node_alive=False, alt_svc_path=None,
             args_override=None):
        from unittest.mock import MagicMock, patch

        mock_resp = MagicMock()
        mock_resp.success        = resp_success
        mock_resp.full_node_name = full_node_name
        mock_resp.unique_id      = unique_id
        mock_resp.error_message  = error_message

        mock_future = MagicMock()
        mock_future.done.return_value   = True
        mock_future.result.return_value = mock_resp

        mock_client = MagicMock()
        mock_client.wait_for_service.return_value = True
        mock_client.call_async.return_value       = mock_future

        list_svc = "/standalone_talker/list_nodes"
        mock_node = MagicMock()
        mock_node.create_client.return_value = mock_client

        # When container_ready, the primary poll finds the service immediately.
        # When not ready, the primary poll returns nothing; subsequent re-scan
        # may return an alt_svc_path (e.g. for component_container_isolated).
        if container_ready:
            mock_node.get_service_names_and_types.return_value = [
                (list_svc, ["composition_interfaces/srv/ListNodes"])
            ]
        elif alt_svc_path:
            # First call (poll loop) returns empty; second call (re-scan) returns alt path
            mock_node.get_service_names_and_types.side_effect = [
                [],
                [(alt_svc_path + "/list_nodes",
                  ["composition_interfaces/srv/ListNodes"])],
            ]
        else:
            mock_node.get_service_names_and_types.return_value = []

        # node name visibility for "is the container process alive?" check
        mock_node.get_node_names_and_namespaces.return_value = (
            [("standalone_talker", "/")] if container_node_alive else []
        )

        mock_srv = MagicMock()
        mock_srv.LoadNode         = MagicMock()
        mock_srv.LoadNode.Request = MagicMock(return_value=MagicMock())
        mock_srv.ListNodes        = MagicMock()

        args = args_override if args_override is not None else self._make_args()
        captured = []
        with patch("ros2_component.check_tmux",         return_value=True), \
             patch("ros2_component.session_exists",      return_value=False), \
             patch("ros2_component.source_local_ws",     return_value=(None, "not_found")), \
             patch("ros2_component.run_cmd",             return_value=("", "", 0 if tmux_ok else 1)), \
             patch("ros2_component.check_session_alive", return_value=session_alive), \
             patch("ros2_component.save_session"), \
             patch("ros2_component.ros2_context") as ctx, \
             patch("ros2_component.ROS2CLI",             return_value=mock_node), \
             patch("rclpy.spin_once"), \
             patch("ros2_component.output",              side_effect=captured.append), \
             patch.dict("sys.modules", {"composition_interfaces.srv": mock_srv}):
            ctx.return_value.__enter__ = MagicMock(return_value=None)
            ctx.return_value.__exit__  = MagicMock(return_value=False)
            self.ros2_component.cmd_component_standalone(args)
        return captured[0]

    def test_successful_standalone_has_required_keys(self):
        result = self._run(True)
        self.assertTrue(result["success"])
        for key in ("session", "container", "container_type",
                    "package_name", "plugin_name", "full_node_name", "unique_id"):
            self.assertIn(key, result)

    def test_container_path_derived_from_plugin_name(self):
        result = self._run(True)
        self.assertEqual(result["container"], "/standalone_talker")

    def test_tmux_unavailable_returns_error(self):
        from unittest.mock import patch
        captured = []
        with patch("ros2_component.check_tmux", return_value=False), \
             patch("ros2_component.output", side_effect=captured.append):
            self.ros2_component.cmd_component_standalone(self._make_args())
        self.assertIn("error", captured[0])

    def test_session_already_exists_returns_error(self):
        from unittest.mock import MagicMock, patch
        captured = []
        mock_srv = MagicMock()
        with patch("ros2_component.check_tmux",    return_value=True), \
             patch("ros2_component.session_exists", return_value=True), \
             patch("ros2_component.output",         side_effect=captured.append), \
             patch.dict("sys.modules", {"composition_interfaces.srv": mock_srv}):
            self.ros2_component.cmd_component_standalone(self._make_args())
        self.assertIn("error", captured[0])
        self.assertIn("already exists", captured[0]["error"])

    def test_tmux_start_failure_returns_error(self):
        result = self._run(True, tmux_ok=False)
        self.assertIn("error", result)

    def test_container_not_ready_node_alive_reports_container_started(self):
        """Timeout with container node alive → container_started: True in error."""
        result = self._run(container_ready=False, container_node_alive=True)
        self.assertIn("error", result)
        self.assertTrue(result.get("container_started"))

    def test_container_not_ready_node_dead_reports_container_not_started(self):
        """Timeout with container node absent → container_started: False in error."""
        result = self._run(container_ready=False, container_node_alive=False)
        self.assertIn("error", result)
        self.assertFalse(result.get("container_started"))

    def test_container_not_ready_alt_path_detected(self):
        """Timeout with service at alternate path → container_found_at reported.

        Uses timeout=0 so the poll loop exits immediately, then the re-scan
        finds the service at the _container sub-path (component_container_isolated
        layout) and reports it via container_found_at.
        """
        from unittest.mock import MagicMock, patch

        alt_container = "/standalone_talker/_container"
        alt_list_svc  = f"{alt_container}/list_nodes"

        mock_node = MagicMock()
        mock_node.create_client.return_value = MagicMock()
        # Poll loop (timeout=0) doesn't call get_service_names_and_types at all;
        # the re-scan call returns the alt service path.
        mock_node.get_service_names_and_types.return_value = [
            (alt_list_svc, ["composition_interfaces/srv/ListNodes"])
        ]
        mock_node.get_node_names_and_namespaces.return_value = []

        mock_srv = MagicMock()
        mock_srv.LoadNode         = MagicMock()
        mock_srv.LoadNode.Request = MagicMock(return_value=MagicMock())
        mock_srv.ListNodes        = MagicMock()

        # timeout=0 guarantees the poll while-loop exits before the first
        # get_service_names_and_types call in the loop body, so the only
        # call to that method comes from the re-scan block.
        args = self._make_args(timeout=0)
        captured = []
        with patch("ros2_component.check_tmux",         return_value=True), \
             patch("ros2_component.session_exists",      return_value=False), \
             patch("ros2_component.source_local_ws",     return_value=(None, "not_found")), \
             patch("ros2_component.run_cmd",             return_value=("", "", 0)), \
             patch("ros2_component.check_session_alive", return_value=True), \
             patch("ros2_component.save_session"), \
             patch("ros2_component.ros2_context") as ctx, \
             patch("ros2_component.ROS2CLI",             return_value=mock_node), \
             patch("rclpy.spin_once"), \
             patch("ros2_component.output",              side_effect=captured.append), \
             patch.dict("sys.modules", {"composition_interfaces.srv": mock_srv}):
            ctx.return_value.__enter__ = MagicMock(return_value=None)
            ctx.return_value.__exit__  = MagicMock(return_value=False)
            self.ros2_component.cmd_component_standalone(args)
        result = captured[0]
        self.assertIn("error", result)
        self.assertIn("container_found_at", result)
        self.assertEqual(result["container_found_at"], alt_container)

    def test_isolated_container_path_uses_container_suffix(self):
        """component_container_isolated → container field ends with /_container."""
        args = self._make_args(container_type="component_container_isolated")
        # For isolated type the list_svc is at /standalone_talker/_container/list_nodes
        from unittest.mock import MagicMock, patch
        isolated_svc = "/standalone_talker/_container/list_nodes"
        mock_resp = MagicMock()
        mock_resp.success        = True
        mock_resp.full_node_name = "/talker"
        mock_resp.unique_id      = 1
        mock_future = MagicMock()
        mock_future.done.return_value   = True
        mock_future.result.return_value = mock_resp
        mock_client = MagicMock()
        mock_client.wait_for_service.return_value = True
        mock_client.call_async.return_value       = mock_future
        mock_node = MagicMock()
        mock_node.create_client.return_value = mock_client
        mock_node.get_service_names_and_types.return_value = [
            (isolated_svc, ["composition_interfaces/srv/ListNodes"])
        ]
        mock_node.get_node_names_and_namespaces.return_value = []
        mock_srv = MagicMock()
        mock_srv.LoadNode         = MagicMock()
        mock_srv.LoadNode.Request = MagicMock(return_value=MagicMock())
        mock_srv.ListNodes        = MagicMock()
        captured = []
        with patch("ros2_component.check_tmux",         return_value=True), \
             patch("ros2_component.session_exists",      return_value=False), \
             patch("ros2_component.source_local_ws",     return_value=(None, "not_found")), \
             patch("ros2_component.run_cmd",             return_value=("", "", 0)), \
             patch("ros2_component.check_session_alive", return_value=True), \
             patch("ros2_component.save_session"), \
             patch("ros2_component.ros2_context") as ctx, \
             patch("ros2_component.ROS2CLI",             return_value=mock_node), \
             patch("rclpy.spin_once"), \
             patch("ros2_component.output",              side_effect=captured.append), \
             patch.dict("sys.modules", {"composition_interfaces.srv": mock_srv}):
            ctx.return_value.__enter__ = MagicMock(return_value=None)
            ctx.return_value.__exit__  = MagicMock(return_value=False)
            self.ros2_component.cmd_component_standalone(args)
        result = captured[0]
        self.assertTrue(result["success"])
        self.assertEqual(result["container"], "/standalone_talker/_container")

    def test_failed_load_returns_success_false(self):
        result = self._run(False, error_message="Plugin not found")
        self.assertFalse(result["success"])
        self.assertIn("error_message", result)

    def test_import_error_returns_error_and_hint(self):
        from unittest.mock import patch
        captured = []
        with patch.dict("sys.modules", {"composition_interfaces.srv": None}), \
             patch("ros2_component.check_tmux",     return_value=True), \
             patch("ros2_component.session_exists",  return_value=False), \
             patch("ros2_component.output",          side_effect=captured.append):
            self.ros2_component.cmd_component_standalone(self._make_args())
        self.assertIn("error", captured[0])
        self.assertIn("hint", captured[0])


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


class TestDaemonHelpers(unittest.TestCase):
    """Tests for ros2_daemon pure-Python helpers.

    No rclpy calls are made by the functions under test, but ros2_utils
    (imported by ros2_daemon) calls sys.exit(1) when rclpy is absent, so
    the rclpy guard is still required to import the module safely.
    """

    @classmethod
    def setUpClass(cls):
        if not check_rclpy_available():
            raise unittest.SkipTest("rclpy not available - requires ROS 2 environment")
        import ros2_daemon
        cls.ros2_daemon = ros2_daemon

    def test_get_domain_id_default_zero(self):
        """ROS_DOMAIN_ID absent → 0."""
        env_backup = os.environ.pop("ROS_DOMAIN_ID", None)
        try:
            self.assertEqual(self.ros2_daemon._get_domain_id(), 0)
        finally:
            if env_backup is not None:
                os.environ["ROS_DOMAIN_ID"] = env_backup

    def test_get_domain_id_from_env(self):
        """ROS_DOMAIN_ID=7 → 7."""
        with patch.dict(os.environ, {"ROS_DOMAIN_ID": "7"}):
            self.assertEqual(self.ros2_daemon._get_domain_id(), 7)


class TestDaemonCommands(unittest.TestCase):
    """End-to-end tests for cmd_daemon_status, cmd_daemon_start, cmd_daemon_stop.

    All three commands delegate to ``ros2 daemon <subcmd>`` via subprocess.
    Tests mock ``ros2_daemon.subprocess.run`` — no real daemon is touched.
    """

    @classmethod
    def setUpClass(cls):
        if not check_rclpy_available():
            raise unittest.SkipTest("rclpy not available - requires ROS 2 environment")
        import subprocess as sp
        import ros2_daemon
        cls.ros2_daemon = ros2_daemon
        cls.CP = sp.CompletedProcess  # convenience alias

    def _run_cmd(self, func, proc):
        """Run *func* with subprocess.run mocked to return *proc*."""
        import types
        captured = []
        with patch("ros2_daemon.subprocess.run", return_value=proc), \
             patch("ros2_daemon.output", side_effect=captured.append):
            func(types.SimpleNamespace())
        self.assertEqual(len(captured), 1)
        return captured[0]

    # ------------------------------------------------------------------
    # daemon status
    # ------------------------------------------------------------------

    def test_status_reports_running(self):
        """ros2 daemon status says running → status='running', domain_id present."""
        proc = self.CP(args=[], returncode=0,
                       stdout="The daemon is running\n", stderr="")
        result = self._run_cmd(self.ros2_daemon.cmd_daemon_status, proc)
        self.assertEqual(result["status"], "running")
        self.assertIn("domain_id", result)

    def test_status_reports_not_running(self):
        """ros2 daemon status says not running → status='not_running'."""
        proc = self.CP(args=[], returncode=0,
                       stdout="The daemon is not running\n", stderr="")
        result = self._run_cmd(self.ros2_daemon.cmd_daemon_status, proc)
        self.assertEqual(result["status"], "not_running")

    def test_status_exception_returns_error(self):
        """subprocess raises (e.g. ros2 not on PATH) → 'error' key in output."""
        import types
        captured = []
        with patch("ros2_daemon.subprocess.run",
                   side_effect=FileNotFoundError("ros2 not found")), \
             patch("ros2_daemon.output", side_effect=captured.append):
            self.ros2_daemon.cmd_daemon_status(types.SimpleNamespace())
        self.assertIn("error", captured[0])

    # ------------------------------------------------------------------
    # daemon start
    # ------------------------------------------------------------------

    def test_start_success(self):
        """ros2 daemon start exits 0 → status='started', domain_id present."""
        proc = self.CP(args=[], returncode=0,
                       stdout="Starting daemon\n", stderr="")
        result = self._run_cmd(self.ros2_daemon.cmd_daemon_start, proc)
        self.assertEqual(result["status"], "started")
        self.assertIn("domain_id", result)

    def test_start_failure(self):
        """ros2 daemon start exits non-zero → status='error', detail present."""
        proc = self.CP(args=[], returncode=1,
                       stdout="", stderr="Failed to start")
        result = self._run_cmd(self.ros2_daemon.cmd_daemon_start, proc)
        self.assertEqual(result["status"], "error")
        self.assertIn("detail", result)

    def test_start_domain_id_present(self):
        """domain_id key always present in start output."""
        proc = self.CP(args=[], returncode=0, stdout="ok", stderr="")
        with patch.dict(os.environ, {"ROS_DOMAIN_ID": "3"}):
            result = self._run_cmd(self.ros2_daemon.cmd_daemon_start, proc)
        self.assertEqual(result["domain_id"], 3)

    def test_start_exception_returns_error(self):
        """Unexpected exception (e.g. _get_domain_id raises) → 'error' key."""
        import types
        captured = []
        with patch.object(self.ros2_daemon, "_get_domain_id",
                          side_effect=RuntimeError("boom")), \
             patch("ros2_daemon.output", side_effect=captured.append):
            self.ros2_daemon.cmd_daemon_start(types.SimpleNamespace())
        self.assertIn("error", captured[0])

    # ------------------------------------------------------------------
    # daemon stop
    # ------------------------------------------------------------------

    def test_stop_success(self):
        """ros2 daemon stop exits 0 → status='stopped'."""
        proc = self.CP(args=[], returncode=0,
                       stdout="Stopping daemon\n", stderr="")
        result = self._run_cmd(self.ros2_daemon.cmd_daemon_stop, proc)
        self.assertEqual(result["status"], "stopped")

    def test_stop_failure(self):
        """ros2 daemon stop exits non-zero → status='error'."""
        proc = self.CP(args=[], returncode=1, stdout="", stderr="stop failed")
        result = self._run_cmd(self.ros2_daemon.cmd_daemon_stop, proc)
        self.assertEqual(result["status"], "error")

    def test_stop_domain_id_present(self):
        """domain_id key always present in stop output."""
        proc = self.CP(args=[], returncode=0, stdout="ok", stderr="")
        with patch.dict(os.environ, {"ROS_DOMAIN_ID": "5"}):
            result = self._run_cmd(self.ros2_daemon.cmd_daemon_stop, proc)
        self.assertEqual(result["domain_id"], 5)


class TestDaemonArgIntrospection(unittest.TestCase):
    """Parser registration tests for the daemon subcommands."""

    @classmethod
    def setUpClass(cls):
        if not check_rclpy_available():
            raise unittest.SkipTest("rclpy not available - requires ROS 2 environment")
        import ros2_cli
        cls.parser = ros2_cli.build_parser()

    def test_daemon_status_help(self):
        with patch("sys.stdout", new_callable=StringIO), \
             self.assertRaises(SystemExit) as cm:
            self.parser.parse_args(["daemon", "status", "--help"])
        self.assertEqual(cm.exception.code, 0)

    def test_daemon_start_help(self):
        with patch("sys.stdout", new_callable=StringIO), \
             self.assertRaises(SystemExit) as cm:
            self.parser.parse_args(["daemon", "start", "--help"])
        self.assertEqual(cm.exception.code, 0)

    def test_daemon_stop_help(self):
        with patch("sys.stdout", new_callable=StringIO), \
             self.assertRaises(SystemExit) as cm:
            self.parser.parse_args(["daemon", "stop", "--help"])
        self.assertEqual(cm.exception.code, 0)


# ---------------------------------------------------------------------------
# Q5 — Parser tests for --slow-last / --slow-factor
# ---------------------------------------------------------------------------

class TestSlowLastFlagParsing(unittest.TestCase):
    """Parser argument tests for the decel zone flags on publish-until."""

    @classmethod
    def setUpClass(cls):
        if not check_rclpy_available():
            raise unittest.SkipTest("rclpy not available - requires ROS 2 environment")
        import ros2_cli
        cls.ros2_cli = ros2_cli
        cls.parser = ros2_cli.build_parser()

    # Minimal valid publish-until invocation (--delta condition).
    BASE = ["topics", "publish-until", "/cmd_vel", "{}",
            "--monitor", "/odom", "--delta", "1.0"]

    def test_slow_last_default_is_none(self):
        args = self.parser.parse_args(self.BASE)
        self.assertIsNone(args.slow_last)

    def test_slow_factor_default_is_0_25(self):
        args = self.parser.parse_args(self.BASE)
        self.assertAlmostEqual(args.slow_factor, 0.25)

    def test_slow_last_accepts_float(self):
        args = self.parser.parse_args(self.BASE + ["--slow-last", "0.5"])
        self.assertAlmostEqual(args.slow_last, 0.5)

    def test_slow_last_accepts_integer_string(self):
        args = self.parser.parse_args(self.BASE + ["--slow-last", "2"])
        self.assertAlmostEqual(args.slow_last, 2.0)

    def test_slow_factor_accepts_custom_value(self):
        args = self.parser.parse_args(self.BASE + ["--slow-last", "1.0", "--slow-factor", "0.1"])
        self.assertAlmostEqual(args.slow_factor, 0.1)

    def test_slow_factor_accepts_zero(self):
        args = self.parser.parse_args(self.BASE + ["--slow-last", "1.0", "--slow-factor", "0.0"])
        self.assertAlmostEqual(args.slow_factor, 0.0)


# ---------------------------------------------------------------------------
# H1 — Unit tests for scale_twist_velocity
# ---------------------------------------------------------------------------

class TestScaleTwistVelocity(unittest.TestCase):
    """Unit tests for scale_twist_velocity — scales Twist / TwistStamped dicts."""

    @classmethod
    def setUpClass(cls):
        if not check_rclpy_available():
            raise unittest.SkipTest("rclpy not available - requires ROS 2 environment")
        import ros2_topic
        # staticmethod prevents Python binding self as the first argument when
        # the function is stored as a class attribute and called via self.fn().
        cls.fn = staticmethod(ros2_topic.scale_twist_velocity)

    def test_plain_twist_scales_all_axes(self):
        data = {"linear": {"x": 1.0, "y": 0.5, "z": 0.0},
                "angular": {"x": 0.0, "y": 0.0, "z": 0.5}}
        result = self.fn(data, 0.5)
        self.assertAlmostEqual(result["linear"]["x"], 0.5)
        self.assertAlmostEqual(result["linear"]["y"], 0.25)
        self.assertAlmostEqual(result["angular"]["z"], 0.25)

    def test_scale_zero_zeroes_all_velocities(self):
        data = {"linear": {"x": 2.0, "y": 1.0, "z": 0.0},
                "angular": {"x": 0.0, "y": 0.0, "z": -1.0}}
        result = self.fn(data, 0.0)
        self.assertAlmostEqual(result["linear"]["x"], 0.0)
        self.assertAlmostEqual(result["linear"]["y"], 0.0)
        self.assertAlmostEqual(result["angular"]["z"], 0.0)

    def test_scale_one_is_identity(self):
        data = {"linear": {"x": 1.5, "y": -0.3, "z": 0.0},
                "angular": {"x": 0.0, "y": 0.0, "z": 0.7}}
        result = self.fn(data, 1.0)
        self.assertAlmostEqual(result["linear"]["x"], 1.5)
        self.assertAlmostEqual(result["linear"]["y"], -0.3)
        self.assertAlmostEqual(result["angular"]["z"], 0.7)

    def test_twist_stamped_scales_nested_twist(self):
        data = {"header": {"frame_id": "base_link"},
                "twist": {"linear": {"x": 1.0, "y": 0.0, "z": 0.0},
                          "angular": {"x": 0.0, "y": 0.0, "z": 0.5}}}
        result = self.fn(data, 0.5)
        self.assertAlmostEqual(result["twist"]["linear"]["x"], 0.5)
        self.assertAlmostEqual(result["twist"]["angular"]["z"], 0.25)

    def test_twist_stamped_no_top_level_linear_key(self):
        """A TwistStamped dict has no top-level linear/angular — only under twist."""
        data = {"header": {"frame_id": "base_link"},
                "twist": {"linear": {"x": 1.0, "y": 0.0, "z": 0.0},
                          "angular": {"x": 0.0, "y": 0.0, "z": 0.5}}}
        result = self.fn(data, 0.0)
        # top-level key absent before and after
        self.assertNotIn("linear", result)
        self.assertNotIn("angular", result)
        # nested values are zeroed
        self.assertAlmostEqual(result["twist"]["linear"]["x"], 0.0)

    def test_partial_payload_skips_absent_axes(self):
        """Missing axes in a sub-dict should not raise."""
        data = {"linear": {"x": 1.0}, "angular": {}}
        result = self.fn(data, 0.5)
        self.assertAlmostEqual(result["linear"]["x"], 0.5)
        self.assertEqual(result["angular"], {})

    def test_sign_preserved_for_negative_velocity(self):
        data = {"linear": {"x": -1.0, "y": 0.0, "z": 0.0},
                "angular": {"x": 0.0, "y": 0.0, "z": 0.0}}
        result = self.fn(data, 0.5)
        self.assertAlmostEqual(result["linear"]["x"], -0.5)

    def test_does_not_mutate_input(self):
        data = {"linear": {"x": 1.0, "y": 0.0, "z": 0.0},
                "angular": {"x": 0.0, "y": 0.0, "z": 0.5}}
        _ = self.fn(data, 0.0)
        # Original dict must be unchanged after the call.
        self.assertAlmostEqual(data["linear"]["x"], 1.0)
        self.assertAlmostEqual(data["angular"]["z"], 0.5)


# ---------------------------------------------------------------------------
# H1 — Pure-arithmetic decel zone formula tests (no rclpy required)
# ---------------------------------------------------------------------------

class TestDecelZoneMath(unittest.TestCase):
    """Pure-arithmetic tests for the decel zone scale formula.

    Formula from publish-until:
        scale = max(slow_factor, remaining / slow_zone)
    Applied only when remaining < slow_zone.
    """

    @staticmethod
    def _scale(remaining, slow_zone, slow_factor):
        return max(slow_factor, remaining / slow_zone)

    def test_at_zone_boundary_scale_is_one(self):
        # remaining == slow_zone → scale = max(sf, 1.0) = 1.0
        self.assertAlmostEqual(self._scale(1.0, 1.0, 0.25), 1.0)

    def test_linear_ramp_at_midpoint(self):
        # remaining = half slow_zone → scale = 0.5
        self.assertAlmostEqual(self._scale(0.5, 1.0, 0.25), 0.5)

    def test_floor_clamps_near_zero(self):
        # remaining << slow_zone — floor kicks in
        self.assertAlmostEqual(self._scale(0.01, 1.0, 0.25), 0.25)

    def test_zero_remaining_returns_slow_factor(self):
        self.assertAlmostEqual(self._scale(0.0, 1.0, 0.25), 0.25)

    def test_slow_factor_zero_allows_ramp_to_zero(self):
        self.assertAlmostEqual(self._scale(0.5, 1.0, 0.0), 0.5)
        self.assertAlmostEqual(self._scale(0.0, 1.0, 0.0), 0.0)

    def test_larger_slow_zone(self):
        # slow_zone = 2.0, remaining = 1.0 → 0.5; floor = 0.25 → result = 0.5
        self.assertAlmostEqual(self._scale(1.0, 2.0, 0.25), 0.5)

    def test_scale_never_below_slow_factor(self):
        for remaining in [0.0, 0.01, 0.1, 0.2, 0.5, 1.0]:
            with self.subTest(remaining=remaining):
                scale = self._scale(remaining, 1.0, 0.25)
                self.assertGreaterEqual(scale, 0.25)


# ---------------------------------------------------------------------------
# H2 — Preset file I/O (ros2_param preset list / delete)
# ---------------------------------------------------------------------------

class TestParamPresetIO(unittest.TestCase):
    """Unit tests for preset list / delete using mocked filesystem calls."""

    @classmethod
    def setUpClass(cls):
        if not check_rclpy_available():
            raise unittest.SkipTest("rclpy not available - requires ROS 2 environment")
        import ros2_param
        cls.mod = ros2_param

    def _stdout(self):
        return patch("sys.stdout", new_callable=StringIO)

    def test_list_empty_when_directory_absent(self):
        with patch.object(self.mod.os.path, "isdir", return_value=False), \
             self._stdout() as buf:
            import types
            self.mod.cmd_params_preset_list(types.SimpleNamespace())
            res = json.loads(buf.getvalue())
        self.assertEqual(res["presets"], [])
        self.assertEqual(res["count"], 0)

    def test_list_filters_json_only(self):
        files = ["alpha.json", "beta.json", "readme.txt", "config.yaml"]
        with patch.object(self.mod.os.path, "isdir", return_value=True), \
             patch.object(self.mod.os, "listdir", return_value=files), \
             self._stdout() as buf:
            import types
            self.mod.cmd_params_preset_list(types.SimpleNamespace())
            res = json.loads(buf.getvalue())
        self.assertEqual(res["count"], 2)
        names = {p["preset"] for p in res["presets"]}
        self.assertEqual(names, {"alpha", "beta"})

    def test_delete_missing_preset_returns_error(self):
        with patch.object(self.mod, "_presets_base", return_value="/fake/presets"), \
             patch.object(self.mod.os.path, "exists", return_value=False), \
             self._stdout() as buf:
            import types
            self.mod.cmd_params_preset_delete(types.SimpleNamespace(preset="ghost"))
            res = json.loads(buf.getvalue())
        self.assertIn("error", res)
        self.assertIn("ghost", res["error"])

    def test_delete_existing_preset_succeeds(self):
        with patch.object(self.mod, "_presets_base", return_value="/fake/presets"), \
             patch.object(self.mod.os.path, "exists", return_value=True), \
             patch.object(self.mod.os, "remove") as mock_rm, \
             self._stdout() as buf:
            import types
            self.mod.cmd_params_preset_delete(types.SimpleNamespace(preset="cam_calib"))
            res = json.loads(buf.getvalue())
        self.assertTrue(res.get("deleted"))
        self.assertEqual(res.get("preset"), "cam_calib")
        mock_rm.assert_called_once()

    def test_delete_path_ends_with_preset_dot_json(self):
        removed = []
        with patch.object(self.mod, "_presets_base", return_value="/fake/presets"), \
             patch.object(self.mod.os.path, "exists", return_value=True), \
             patch.object(self.mod.os, "remove", side_effect=lambda p: removed.append(p)), \
             self._stdout():
            import types
            self.mod.cmd_params_preset_delete(types.SimpleNamespace(preset="robot_base"))
        self.assertEqual(len(removed), 1)
        self.assertTrue(removed[0].endswith("robot_base.json"))


# ---------------------------------------------------------------------------
# H2 — Tmux path tests (ros2_launch / ros2_run no-tmux and empty session list)
# ---------------------------------------------------------------------------

class TestLaunchTmuxPath(unittest.TestCase):
    """Unit tests for the tmux-absent error path and empty session list
    in ros2_launch and ros2_run commands."""

    @classmethod
    def setUpClass(cls):
        if not check_rclpy_available():
            raise unittest.SkipTest("rclpy not available - requires ROS 2 environment")
        import ros2_launch
        import ros2_run
        cls.launch = ros2_launch
        # Stored as ros2_run_mod (not cls.run) to avoid shadowing
        # unittest.TestCase.run(), which the test runner calls on every test.
        cls.ros2_run_mod = ros2_run

    def _stdout(self):
        return patch("sys.stdout", new_callable=StringIO)

    # --- ros2_launch ---

    def test_launch_run_no_tmux_returns_error(self):
        with patch.object(self.launch, "check_tmux", return_value=False), \
             self._stdout() as buf:
            import types
            args = types.SimpleNamespace(package="pkg", launch_file="nav.launch.py",
                                         args=[], session=None)
            self.launch.cmd_launch_run(args)
            res = json.loads(buf.getvalue())
        self.assertIn("error", res)
        self.assertIn("tmux", res["error"].lower())

    def test_launch_list_no_tmux_returns_error(self):
        with patch.object(self.launch, "list_sessions",
                          return_value={"error": "tmux is not installed",
                                        "running_sessions": []}), \
             self._stdout() as buf:
            import types
            self.launch.cmd_launch_list(types.SimpleNamespace())
            res = json.loads(buf.getvalue())
        self.assertIn("error", res)

    def test_launch_list_empty_sessions(self):
        with patch.object(self.launch, "list_sessions",
                          return_value={"launch_sessions": [], "running_sessions": []}), \
             self._stdout() as buf:
            import types
            self.launch.cmd_launch_list(types.SimpleNamespace())
            res = json.loads(buf.getvalue())
        self.assertEqual(res.get("launch_sessions"), [])

    # --- ros2_run ---

    def test_run_no_tmux_returns_error(self):
        with patch.object(self.ros2_run_mod, "check_tmux", return_value=False), \
             self._stdout() as buf:
            import types
            args = types.SimpleNamespace(package="pkg", executable="node",
                                         args=[], session=None,
                                         presets=None, params=None, config_path=None)
            self.ros2_run_mod.cmd_run(args)
            res = json.loads(buf.getvalue())
        self.assertIn("error", res)
        self.assertIn("tmux", res["error"].lower())

    def test_run_list_empty_sessions(self):
        with patch.object(self.ros2_run_mod, "list_sessions",
                          return_value={"run_sessions": [], "running_sessions": []}), \
             self._stdout() as buf:
            import types
            self.ros2_run_mod.cmd_run_list(types.SimpleNamespace())
            res = json.loads(buf.getvalue())
        self.assertEqual(res.get("run_sessions"), [])


# ---------------------------------------------------------------------------
# H1 — ConditionMonitor.callback unit tests
# ---------------------------------------------------------------------------

class TestConditionMonitorCallback(unittest.TestCase):
    """Unit tests for ConditionMonitor.callback — bypasses Node.__init__.

    Three branches tested:
      - Standard field operators (delta, above, below, equals)
      - Euclidean distance mode
      - Rotation accumulation mode
    """

    @classmethod
    def setUpClass(cls):
        # Requires *live* rclpy: uses object.__new__(ConditionMonitor) which
        # needs ConditionMonitor to be a real class (not a MagicMock stub).
        # With mocked rclpy, Node is a MagicMock so ConditionMonitor would be
        # a dynamic subclass of MagicMock, which object.__new__ cannot allocate.
        if not _ROS_AVAILABLE:
            raise unittest.SkipTest("live rclpy required — ConditionMonitor must be a real class")
        import ros2_topic
        cls.ros2_topic = ros2_topic

    def _make_monitor(self, *, euclidean=False, rotate=None,
                      field='x', fields=None, operator='delta', threshold=1.0):
        """Build a ConditionMonitor instance bypassing Node.__init__."""
        import threading
        m = object.__new__(self.ros2_topic.ConditionMonitor)
        m.lock = threading.Lock()
        m.stop_event = threading.Event()
        m.euclidean = euclidean
        m.rotate = rotate
        m.field = field
        m.fields = fields if fields is not None else [field]
        m.operator = operator
        m.threshold = threshold
        m.start_value = None
        m.current_value = None
        m.start_values = None
        m.current_values = None
        m.euclidean_distance = None
        m.start_yaw = None
        m.last_yaw = None
        m.accumulated_rotation = 0.0
        m.rotation_delta = None
        m.start_msg = None
        m.end_msg = None
        m.field_error = None
        return m

    def _call(self, monitor, msg_dict):
        """Invoke callback with a MagicMock msg whose msg_to_dict returns msg_dict."""
        with patch.object(self.ros2_topic, 'msg_to_dict', return_value=msg_dict):
            monitor.callback(MagicMock())

    # ------------------------------------------------------------------ operators

    def test_first_call_sets_start_value(self):
        m = self._make_monitor(operator='delta', threshold=1.0, field='x')
        self._call(m, {'x': 0.0})
        self.assertAlmostEqual(float(m.start_value), 0.0)
        self.assertFalse(m.stop_event.is_set())

    def test_delta_positive_triggers_stop(self):
        m = self._make_monitor(operator='delta', threshold=1.0, field='x')
        self._call(m, {'x': 0.0})
        self._call(m, {'x': 1.5})
        self.assertTrue(m.stop_event.is_set())

    def test_delta_below_threshold_no_stop(self):
        m = self._make_monitor(operator='delta', threshold=1.0, field='x')
        self._call(m, {'x': 0.0})
        self._call(m, {'x': 0.5})
        self.assertFalse(m.stop_event.is_set())

    def test_delta_negative_threshold_triggers_stop(self):
        m = self._make_monitor(operator='delta', threshold=-1.0, field='x')
        self._call(m, {'x': 0.0})
        self._call(m, {'x': -1.5})
        self.assertTrue(m.stop_event.is_set())

    def test_above_triggers_stop(self):
        m = self._make_monitor(operator='above', threshold=5.0, field='x')
        self._call(m, {'x': 3.0})
        self.assertFalse(m.stop_event.is_set())
        self._call(m, {'x': 6.0})
        self.assertTrue(m.stop_event.is_set())

    def test_below_triggers_stop(self):
        m = self._make_monitor(operator='below', threshold=5.0, field='x')
        self._call(m, {'x': 7.0})
        self.assertFalse(m.stop_event.is_set())
        self._call(m, {'x': 3.0})
        self.assertTrue(m.stop_event.is_set())

    def test_equals_triggers_stop(self):
        m = self._make_monitor(operator='equals', threshold=42.0, field='x')
        self._call(m, {'x': 41.0})
        self.assertFalse(m.stop_event.is_set())
        self._call(m, {'x': 42.0})
        self.assertTrue(m.stop_event.is_set())

    def test_stop_event_already_set_short_circuits(self):
        m = self._make_monitor(operator='above', threshold=1.0, field='x')
        m.stop_event.set()
        self._call(m, {'x': 999.0})
        # start_value must still be None — callback returned before modifying state
        self.assertIsNone(m.start_value)

    # ------------------------------------------------------------------ euclidean

    def test_euclidean_first_call_sets_start_values(self):
        m = self._make_monitor(euclidean=True, fields=['x', 'y'], threshold=1.0)
        self._call(m, {'x': 0.0, 'y': 0.0})
        self.assertEqual(m.start_values, [0.0, 0.0])
        self.assertFalse(m.stop_event.is_set())

    def test_euclidean_triggers_stop_when_distance_met(self):
        m = self._make_monitor(euclidean=True, fields=['x', 'y'], threshold=1.0)
        self._call(m, {'x': 0.0, 'y': 0.0})
        self._call(m, {'x': 1.0, 'y': 0.0})
        self.assertTrue(m.stop_event.is_set())
        self.assertAlmostEqual(m.euclidean_distance, 1.0)

    def test_euclidean_no_stop_below_threshold(self):
        m = self._make_monitor(euclidean=True, fields=['x', 'y'], threshold=2.0)
        self._call(m, {'x': 0.0, 'y': 0.0})
        self._call(m, {'x': 1.0, 'y': 0.0})
        self.assertFalse(m.stop_event.is_set())

    def test_euclidean_diagonal_distance(self):
        import math as _math
        m = self._make_monitor(euclidean=True, fields=['x', 'y'], threshold=1.4)
        self._call(m, {'x': 0.0, 'y': 0.0})
        self._call(m, {'x': 1.0, 'y': 1.0})
        self.assertTrue(m.stop_event.is_set())
        self.assertAlmostEqual(m.euclidean_distance, _math.sqrt(2), places=5)

    # ------------------------------------------------------------------ rotation

    def test_rotation_first_call_sets_start_yaw(self):
        m = self._make_monitor(rotate=1.571)
        self._call(m, {'orientation': {'x': 0.0, 'y': 0.0, 'z': 0.0, 'w': 1.0}})
        self.assertIsNotNone(m.start_yaw)
        self.assertAlmostEqual(m.start_yaw, 0.0)
        self.assertFalse(m.stop_event.is_set())

    def test_rotation_positive_triggers_stop(self):
        import math as _math
        m = self._make_monitor(rotate=_math.pi / 2)
        self._call(m, {'orientation': {'x': 0.0, 'y': 0.0, 'z': 0.0, 'w': 1.0}})
        # Use 100° (> 90° target) to avoid floating-point boundary on ARM vs x86:
        # sin/cos of exactly pi/4 can round to a yaw just below pi/2 on some platforms.
        angle = _math.radians(100)
        sz = _math.sin(angle / 2)
        cw = _math.cos(angle / 2)
        self._call(m, {'orientation': {'x': 0.0, 'y': 0.0, 'z': sz, 'w': cw}})
        self.assertTrue(m.stop_event.is_set())

    def test_rotation_negative_triggers_stop(self):
        import math as _math
        m = self._make_monitor(rotate=-_math.pi / 2)
        self._call(m, {'orientation': {'x': 0.0, 'y': 0.0, 'z': 0.0, 'w': 1.0}})
        # Use -100° (< -90° target) to avoid floating-point boundary on ARM vs x86.
        angle = _math.radians(100)
        sz = -_math.sin(angle / 2)
        cw = _math.cos(angle / 2)
        self._call(m, {'orientation': {'x': 0.0, 'y': 0.0, 'z': sz, 'w': cw}})
        self.assertTrue(m.stop_event.is_set())

    def test_rotation_missing_quaternion_sets_field_error(self):
        import math as _math
        m = self._make_monitor(rotate=_math.pi / 2)
        self._call(m, {})
        self.assertIsNotNone(m.field_error)
        self.assertTrue(m.stop_event.is_set())

    def test_rotation_accumulates_over_multiple_steps(self):
        import math as _math
        m = self._make_monitor(rotate=_math.pi)
        # First call: identity quat → sets start_yaw=0
        self._call(m, {'orientation': {'x': 0.0, 'y': 0.0, 'z': 0.0, 'w': 1.0}})
        # Second call: 90° CCW — accumulated=pi/2, target=pi → no stop
        s45 = _math.sin(_math.pi / 4)
        c45 = _math.cos(_math.pi / 4)
        self._call(m, {'orientation': {'x': 0.0, 'y': 0.0, 'z': s45, 'w': c45}})
        self.assertFalse(m.stop_event.is_set())
        # Third call: 180° CCW — accumulated=pi, target=pi → stop
        s90 = _math.sin(_math.pi / 2)
        c90 = _math.cos(_math.pi / 2)
        self._call(m, {'orientation': {'x': 0.0, 'y': 0.0, 'z': s90, 'w': c90}})
        self.assertTrue(m.stop_event.is_set())


# ---------------------------------------------------------------------------
# H2 — _get_managed_nodes filtering logic (ros2_lifecycle)
# ---------------------------------------------------------------------------

class TestGetManagedNodes(unittest.TestCase):
    """Unit tests for _get_managed_nodes — pure filtering on service graph."""

    @classmethod
    def setUpClass(cls):
        if not check_rclpy_available():
            raise unittest.SkipTest("rclpy not available - requires ROS 2 environment")
        import ros2_lifecycle
        cls.mod = ros2_lifecycle

    def test_returns_sorted_node_names(self):
        mock_node = MagicMock()
        mock_node.get_service_names_and_types.return_value = [
            ('/cam_node/get_state', ['lifecycle_msgs/srv/GetState']),
            ('/arm_node/get_state', ['lifecycle_msgs/srv/GetState']),
        ]
        result = self.mod._get_managed_nodes(mock_node)
        self.assertEqual(result, ['/arm_node', '/cam_node'])

    def test_excludes_non_lifecycle_services(self):
        mock_node = MagicMock()
        mock_node.get_service_names_and_types.return_value = [
            ('/my_node/get_state', ['lifecycle_msgs/srv/GetState']),
            ('/other/some_service', ['std_srvs/srv/Empty']),
        ]
        result = self.mod._get_managed_nodes(mock_node)
        self.assertEqual(result, ['/my_node'])

    def test_empty_graph_returns_empty(self):
        mock_node = MagicMock()
        mock_node.get_service_names_and_types.return_value = []
        result = self.mod._get_managed_nodes(mock_node)
        self.assertEqual(result, [])

    def test_excludes_bare_get_state_service(self):
        """/get_state alone has len == len('/get_state') and must be excluded."""
        mock_node = MagicMock()
        mock_node.get_service_names_and_types.return_value = [
            ('/get_state', ['lifecycle_msgs/srv/GetState']),
        ]
        result = self.mod._get_managed_nodes(mock_node)
        self.assertEqual(result, [])


# ---------------------------------------------------------------------------
# H2 — Control command error / success paths (ros2_control)
# ---------------------------------------------------------------------------

class TestControlCommandPaths(unittest.TestCase):
    """Unit tests for ros2_control commands with mocked service calls."""

    @classmethod
    def setUpClass(cls):
        if not check_rclpy_available():
            raise unittest.SkipTest("rclpy not available - requires ROS 2 environment")
        import ros2_control
        cls.mod = ros2_control

    def _stdout(self):
        return patch("sys.stdout", new_callable=StringIO)

    def _args(self, **kwargs):
        import types
        base = dict(controller_manager='/controller_manager', timeout=5.0, retries=1)
        base.update(kwargs)
        return types.SimpleNamespace(**base)

    def test_list_controllers_error_path(self):
        with patch.object(self.mod, 'ros2_context', MagicMock()), \
             patch.object(self.mod, 'ROS2CLI', MagicMock()), \
             patch.object(self.mod, '_call_cm_service',
                          return_value=(None, {"error": "service not available"})), \
             self._stdout() as buf:
            self.mod.cmd_control_list_controllers(self._args())
            res = json.loads(buf.getvalue())
        self.assertIn("error", res)

    def test_list_controllers_success_formats_output(self):
        mock_result = MagicMock()
        mock_result.controller = []
        with patch.object(self.mod, 'ros2_context', MagicMock()), \
             patch.object(self.mod, 'ROS2CLI', MagicMock()), \
             patch.object(self.mod, '_call_cm_service',
                          return_value=(mock_result, None)), \
             self._stdout() as buf:
            self.mod.cmd_control_list_controllers(self._args())
            res = json.loads(buf.getvalue())
        self.assertIn("controllers", res)
        self.assertEqual(res["count"], 0)

    def test_load_controller_success_returns_ok(self):
        mock_result = MagicMock()
        mock_result.ok = True
        with patch.object(self.mod, 'ros2_context', MagicMock()), \
             patch.object(self.mod, 'ROS2CLI', MagicMock()), \
             patch.object(self.mod, '_call_cm_service',
                          return_value=(mock_result, None)), \
             self._stdout() as buf:
            self.mod.cmd_control_load_controller(self._args(name='joint_controller'))
            res = json.loads(buf.getvalue())
        self.assertEqual(res.get("controller"), "joint_controller")
        self.assertTrue(res.get("ok"))

    def test_load_controller_error_path(self):
        with patch.object(self.mod, 'ros2_context', MagicMock()), \
             patch.object(self.mod, 'ROS2CLI', MagicMock()), \
             patch.object(self.mod, '_call_cm_service',
                          return_value=(None, {"error": "timeout"})), \
             self._stdout() as buf:
            self.mod.cmd_control_load_controller(self._args(name='joint_controller'))
            res = json.loads(buf.getvalue())
        self.assertIn("error", res)

    def test_call_cm_service_service_unavailable_returns_error(self):
        """_call_cm_service returns error dict when service never appears."""
        mock_node = MagicMock()
        mock_client = MagicMock()
        mock_node.create_client.return_value = mock_client
        mock_client.wait_for_service.return_value = False
        result, err = self.mod._call_cm_service(
            mock_node, MagicMock(), '/cm', 'list_controllers', MagicMock(), 0.1, retries=1
        )
        self.assertIsNone(result)
        self.assertIn("error", err)
        mock_client.destroy.assert_called_once()

    def test_call_cm_service_retries_before_giving_up(self):
        """_call_cm_service calls wait_for_service once per retry attempt."""
        mock_node = MagicMock()
        mock_client = MagicMock()
        mock_node.create_client.return_value = mock_client
        mock_client.wait_for_service.return_value = False
        self.mod._call_cm_service(
            mock_node, MagicMock(), '/cm', 'list_controllers', MagicMock(), 0.1, retries=3
        )
        self.assertEqual(mock_client.wait_for_service.call_count, 3)


# ---------------------------------------------------------------------------
# H2 — Lifecycle command paths (ros2_lifecycle)
# ---------------------------------------------------------------------------

class TestLifecycleCommandPaths(unittest.TestCase):
    """Unit tests for ros2_lifecycle commands with mocked context and node."""

    @classmethod
    def setUpClass(cls):
        if not check_rclpy_available():
            raise unittest.SkipTest("rclpy not available - requires ROS 2 environment")
        import ros2_lifecycle
        cls.mod = ros2_lifecycle

    def _stdout(self):
        return patch("sys.stdout", new_callable=StringIO)

    def test_cmd_lifecycle_nodes_returns_sorted_list(self):
        import types
        mock_node_inst = MagicMock()
        mock_node_inst.get_service_names_and_types.return_value = [
            ('/cam_node/get_state', ['lifecycle_msgs/srv/GetState']),
            ('/arm_node/get_state', ['lifecycle_msgs/srv/GetState']),
        ]
        with patch.object(self.mod, 'ros2_context', MagicMock()), \
             patch.object(self.mod, 'ROS2CLI', return_value=mock_node_inst), \
             self._stdout() as buf:
            self.mod.cmd_lifecycle_nodes(types.SimpleNamespace())
            res = json.loads(buf.getvalue())
        self.assertEqual(res["managed_nodes"], ['/arm_node', '/cam_node'])
        self.assertEqual(res["count"], 2)

    def test_cmd_lifecycle_nodes_empty_graph(self):
        import types
        mock_node_inst = MagicMock()
        mock_node_inst.get_service_names_and_types.return_value = []
        with patch.object(self.mod, 'ros2_context', MagicMock()), \
             patch.object(self.mod, 'ROS2CLI', return_value=mock_node_inst), \
             self._stdout() as buf:
            self.mod.cmd_lifecycle_nodes(types.SimpleNamespace())
            res = json.loads(buf.getvalue())
        self.assertEqual(res["managed_nodes"], [])
        self.assertEqual(res["count"], 0)

    def test_cmd_lifecycle_get_service_unavailable(self):
        import types, sys
        args = types.SimpleNamespace(node='/my_node', timeout=0.1, retries=1)
        mock_node_inst = MagicMock()
        mock_client = MagicMock()
        mock_node_inst.create_client.return_value = mock_client
        mock_client.wait_for_service.return_value = False

        mock_lc = MagicMock()
        mock_lc.srv.GetState = MagicMock()
        with patch.dict(sys.modules, {'lifecycle_msgs': mock_lc,
                                       'lifecycle_msgs.srv': mock_lc.srv}), \
             patch.object(self.mod, 'ros2_context', MagicMock()), \
             patch.object(self.mod, 'ROS2CLI', return_value=mock_node_inst), \
             self._stdout() as buf:
            self.mod.cmd_lifecycle_get(args)
            res = json.loads(buf.getvalue())
        self.assertIn("error", res)
        self.assertIn("my_node", res["error"])


# ---------------------------------------------------------------------------
# Wave 8 — Gap 4: cmd_estop — topic auto-detection and Twist/TwistStamped path
# ---------------------------------------------------------------------------

class TestEstopBranching(unittest.TestCase):
    """cmd_estop: velocity topic auto-detection and Twist vs TwistStamped branching."""

    @classmethod
    def setUpClass(cls):
        if not check_rclpy_available():
            raise unittest.SkipTest("rclpy not available - requires ROS 2 environment")
        import ros2_topic
        cls.mod = ros2_topic

    def _run_estop(self, topics, msg_has_twist, explicit_topic=None):
        """Run cmd_estop with a mocked ROS graph. Return (output_dict, msg_instance)."""
        import types
        # Use spec to control hasattr(msg, 'twist') — MagicMock only exposes listed attrs.
        if msg_has_twist:
            mock_msg = MagicMock(spec=['header', 'twist'])
        else:
            mock_msg = MagicMock(spec=['linear', 'angular'])
        mock_msg_class = MagicMock(return_value=mock_msg)

        mock_node = MagicMock()
        mock_node.get_topic_names.return_value = topics
        args = types.SimpleNamespace(topic=explicit_topic)

        with patch.object(self.mod, 'ros2_context', MagicMock()), \
             patch.object(self.mod, 'ROS2CLI', return_value=mock_node), \
             patch.object(self.mod, 'get_msg_type', return_value=mock_msg_class), \
             patch.object(self.mod.time, 'sleep'), \
             patch('sys.stdout', new_callable=StringIO) as buf:
            self.mod.cmd_estop(args)
        return json.loads(buf.getvalue()), mock_msg

    def test_autodetect_prefers_topic_with_cmd_vel_in_name(self):
        topics = [
            ('/base/vel', ['geometry_msgs/msg/Twist']),
            ('/cmd_vel',  ['geometry_msgs/msg/Twist']),
        ]
        result, _ = self._run_estop(topics, msg_has_twist=False)
        self.assertEqual(result['topic'], '/cmd_vel')

    def test_autodetect_falls_back_to_first_when_no_cmd_vel(self):
        topics = [
            ('/base/velocity', ['geometry_msgs/msg/Twist']),
            ('/robot/vel',     ['geometry_msgs/msg/Twist']),
        ]
        result, _ = self._run_estop(topics, msg_has_twist=False)
        self.assertEqual(result['topic'], '/base/velocity')

    def test_twist_path_zeroes_top_level_linear_and_angular(self):
        topics = [('/cmd_vel', ['geometry_msgs/msg/Twist'])]
        _, msg = self._run_estop(topics, msg_has_twist=False)
        self.assertEqual(msg.linear.x, 0.0)
        self.assertEqual(msg.angular.z, 0.0)

    def test_twist_stamped_path_zeroes_nested_twist(self):
        topics = [('/cmd_vel', ['geometry_msgs/msg/TwistStamped'])]
        _, msg = self._run_estop(topics, msg_has_twist=True)
        self.assertEqual(msg.twist.linear.x, 0.0)
        self.assertEqual(msg.twist.angular.z, 0.0)

    def test_no_velocity_topic_returns_error(self):
        import types
        topics = [('/scan', ['sensor_msgs/msg/LaserScan'])]
        mock_node = MagicMock()
        mock_node.get_topic_names.return_value = topics
        args = types.SimpleNamespace(topic=None)
        with patch.object(self.mod, 'ros2_context', MagicMock()), \
             patch.object(self.mod, 'ROS2CLI', return_value=mock_node), \
             patch('sys.stdout', new_callable=StringIO) as buf:
            self.mod.cmd_estop(args)
        self.assertIn('error', json.loads(buf.getvalue()))


# ---------------------------------------------------------------------------
# Wave 8 — Gap 6: ros2_context always shuts down rclpy, even on exception
# ---------------------------------------------------------------------------

class TestRos2ContextCleanup(unittest.TestCase):
    """ros2_context calls rclpy.shutdown() in the finally block — always."""

    @classmethod
    def setUpClass(cls):
        if not check_rclpy_available():
            raise unittest.SkipTest("rclpy not available - requires ROS 2 environment")
        import ros2_utils
        cls.mod = ros2_utils

    def test_shutdown_called_on_normal_exit(self):
        with patch.object(self.mod.rclpy, 'init'), \
             patch.object(self.mod.rclpy, 'shutdown') as mock_shutdown:
            with self.mod.ros2_context():
                pass
            mock_shutdown.assert_called_once()

    def test_shutdown_called_when_exception_raised_inside_context(self):
        with patch.object(self.mod.rclpy, 'init'), \
             patch.object(self.mod.rclpy, 'shutdown') as mock_shutdown:
            with self.assertRaises(RuntimeError):
                with self.mod.ros2_context():
                    raise RuntimeError("boom")
            mock_shutdown.assert_called_once()

    def test_exception_propagates_after_cleanup(self):
        with patch.object(self.mod.rclpy, 'init'), \
             patch.object(self.mod.rclpy, 'shutdown'):
            with self.assertRaises(ValueError):
                with self.mod.ros2_context():
                    raise ValueError("propagated")


# ---------------------------------------------------------------------------
# Wave 8 — Gap 7: ConditionMonitor picks QoS to match publisher reliability
# ---------------------------------------------------------------------------

class TestConditionMonitorQoS(unittest.TestCase):
    """ConditionMonitor uses BEST_EFFORT QoS when any publisher does; system default otherwise."""

    @classmethod
    def setUpClass(cls):
        # Requires *live* rclpy: asserts against real ReliabilityPolicy enum values and
        # qos_profile_sensor_data / qos_profile_system_default identity.
        if not _ROS_AVAILABLE:
            raise unittest.SkipTest("live rclpy required — real QoS enum values needed")
        import ros2_topic
        cls.mod = ros2_topic

    def _captured_qos(self, pub_infos):
        """Instantiate ConditionMonitor with mocked Node; return QoS passed to create_subscription."""
        import threading
        captured = {}

        def mock_node_init(self, name):
            self.get_publishers_info_by_topic = MagicMock(return_value=pub_infos)
            self.create_subscription = MagicMock(
                side_effect=lambda cls, t, cb, qos: captured.update({'qos': qos})
            )

        stop_event = threading.Event()
        with patch.object(self.mod.Node, '__init__', mock_node_init), \
             patch.object(self.mod, 'get_msg_type', return_value=MagicMock()):
            self.mod.ConditionMonitor(
                '/odom', 'nav_msgs/msg/Odometry', 'x', 'delta', 1.0, stop_event
            )
        return captured.get('qos')

    def test_best_effort_publisher_selects_sensor_data_qos(self):
        from rclpy.qos import ReliabilityPolicy
        pub = MagicMock()
        pub.qos_profile.reliability = ReliabilityPolicy.BEST_EFFORT
        self.assertIs(self._captured_qos([pub]), self.mod.qos_profile_sensor_data)

    def test_reliable_publisher_keeps_system_default_qos(self):
        from rclpy.qos import ReliabilityPolicy
        pub = MagicMock()
        pub.qos_profile.reliability = ReliabilityPolicy.RELIABLE
        self.assertIs(self._captured_qos([pub]), self.mod.qos_profile_system_default)

    def test_empty_publisher_list_keeps_system_default_qos(self):
        self.assertIs(self._captured_qos([]), self.mod.qos_profile_system_default)

    def test_get_publishers_info_exception_falls_back_to_system_default(self):
        import threading
        captured = {}

        def mock_node_init(self, name):
            self.get_publishers_info_by_topic = MagicMock(side_effect=Exception("rpc error"))
            self.create_subscription = MagicMock(
                side_effect=lambda cls, t, cb, qos: captured.update({'qos': qos})
            )

        stop_event = threading.Event()
        with patch.object(self.mod.Node, '__init__', mock_node_init), \
             patch.object(self.mod, 'get_msg_type', return_value=MagicMock()):
            self.mod.ConditionMonitor(
                '/odom', 'nav_msgs/msg/Odometry', 'x', 'delta', 1.0, stop_event
            )
        self.assertIs(captured.get('qos'), self.mod.qos_profile_system_default)


# ---------------------------------------------------------------------------
# Wave 8 — Gap 8: tf monitor returns clear error when no frame can be discovered
# ---------------------------------------------------------------------------

class TestTFMonitorMissingFrame(unittest.TestCase):
    """cmd_tf_monitor returns a clear error when reference frame auto-discovery fails."""

    @classmethod
    def setUpClass(cls):
        if not check_rclpy_available():
            raise unittest.SkipTest("rclpy not available - requires ROS 2 environment")
        import ros2_tf
        cls.mod = ros2_tf

    def _run(self, reference_frame=None, frames_yaml='', frames_raise=None):
        import types, sys
        args = types.SimpleNamespace(
            frame='base_link', reference_frame=reference_frame,
            timeout=1.0, count=1,
        )
        mock_buffer = MagicMock()
        if frames_raise:
            mock_buffer.all_frames_as_yaml.side_effect = frames_raise
        else:
            mock_buffer.all_frames_as_yaml.return_value = frames_yaml

        mock_tf2_ros = MagicMock()
        mock_tf2_ros.Buffer.return_value = mock_buffer

        with patch.object(self.mod, 'ros2_context', MagicMock()), \
             patch.dict(sys.modules, {'tf2_ros': mock_tf2_ros}), \
             patch('rclpy.node.Node', return_value=MagicMock()), \
             patch('rclpy.spin_once'), \
             patch('time.sleep'), \
             patch('sys.stdout', new_callable=StringIO) as buf:
            self.mod.cmd_tf_monitor(args)
        return json.loads(buf.getvalue())

    def test_empty_tf_tree_returns_autodiscovery_error(self):
        result = self._run(frames_yaml='')
        self.assertIn('error', result)

    def test_all_frames_exception_returns_autodiscovery_error(self):
        result = self._run(frames_raise=Exception('tf2 unavailable'))
        self.assertIn('error', result)


# ---------------------------------------------------------------------------
# Gap 2 — v1.0.5 new commands: parser tests
# ---------------------------------------------------------------------------

class TestParamsFindParsing(unittest.TestCase):
    """Parser tests for the params find subcommand added in v1.0.5."""

    @classmethod
    def setUpClass(cls):
        if not check_rclpy_available():
            raise unittest.SkipTest("rclpy not available - requires ROS 2 environment")
        import ros2_cli
        cls.ros2_cli = ros2_cli
        cls.parser = ros2_cli.build_parser()

    def test_pattern_positional_required(self):
        args = self.parser.parse_args(["params", "find", "velocity"])
        self.assertEqual(args.pattern, "velocity")
        self.assertEqual(args.subcommand, "find")

    def test_node_optional_default_none(self):
        args = self.parser.parse_args(["params", "find", "velocity"])
        self.assertIsNone(args.node)

    def test_node_flag_accepted(self):
        args = self.parser.parse_args(["params", "find", "velocity", "--node", "/controller"])
        self.assertEqual(args.node, "/controller")

    def test_wildcard_pattern(self):
        args = self.parser.parse_args(["params", "find", "all"])
        self.assertEqual(args.pattern, "all")

    def test_dispatch_wired(self):
        D = self.ros2_cli.DISPATCH
        self.assertIn(("params", "find"), D)
        self.assertTrue(callable(D[("params", "find")]))


class TestTFNewCommandsParsing(unittest.TestCase):
    """Parser tests for tf tree and tf validate added in v1.0.5."""

    @classmethod
    def setUpClass(cls):
        if not check_rclpy_available():
            raise unittest.SkipTest("rclpy not available - requires ROS 2 environment")
        import ros2_cli
        cls.ros2_cli = ros2_cli
        cls.parser = ros2_cli.build_parser()

    def test_tf_tree_parses(self):
        args = self.parser.parse_args(["tf", "tree"])
        self.assertEqual(args.subcommand, "tree")

    def test_tf_tree_duration_default(self):
        args = self.parser.parse_args(["tf", "tree"])
        self.assertAlmostEqual(args.duration, 2.0)

    def test_tf_tree_duration_custom(self):
        args = self.parser.parse_args(["tf", "tree", "--duration", "5"])
        self.assertAlmostEqual(args.duration, 5.0)

    def test_tf_validate_parses(self):
        args = self.parser.parse_args(["tf", "validate"])
        self.assertEqual(args.subcommand, "validate")

    def test_tf_validate_duration_default(self):
        args = self.parser.parse_args(["tf", "validate"])
        self.assertAlmostEqual(args.duration, 2.0)

    def test_tf_validate_duration_short_flag(self):
        args = self.parser.parse_args(["tf", "validate", "-d", "3"])
        self.assertAlmostEqual(args.duration, 3.0)

    def test_dispatch_wired(self):
        D = self.ros2_cli.DISPATCH
        self.assertIn(("tf", "tree"), D)
        self.assertIn(("tf", "validate"), D)
        self.assertTrue(callable(D[("tf", "tree")]))
        self.assertTrue(callable(D[("tf", "validate")]))


class TestQosCheckParsing(unittest.TestCase):
    """Parser tests for topics qos-check added in v1.0.5."""

    @classmethod
    def setUpClass(cls):
        if not check_rclpy_available():
            raise unittest.SkipTest("rclpy not available - requires ROS 2 environment")
        import ros2_cli
        cls.ros2_cli = ros2_cli
        cls.parser = ros2_cli.build_parser()

    def test_topic_positional_required(self):
        args = self.parser.parse_args(["topics", "qos-check", "/odom"])
        self.assertEqual(args.topic, "/odom")
        self.assertEqual(args.subcommand, "qos-check")

    def test_timeout_default(self):
        args = self.parser.parse_args(["topics", "qos-check", "/odom"])
        self.assertAlmostEqual(args.timeout, 5.0)

    def test_timeout_custom(self):
        args = self.parser.parse_args(["topics", "qos-check", "/odom", "--timeout", "10"])
        self.assertAlmostEqual(args.timeout, 10.0)

    def test_dispatch_wired(self):
        D = self.ros2_cli.DISPATCH
        self.assertIn(("topics", "qos-check"), D)
        self.assertTrue(callable(D[("topics", "qos-check")]))


# ---------------------------------------------------------------------------
# Gap 3 — _apply_params unit tests
# ---------------------------------------------------------------------------

class TestApplyParams(unittest.TestCase):
    """Unit tests for ros2_run._apply_params — inline param string parser."""

    @classmethod
    def setUpClass(cls):
        if not check_rclpy_available():
            raise unittest.SkipTest("rclpy not available - requires ROS 2 environment")
        from ros2_run import _apply_params
        cls.fn = staticmethod(_apply_params)

    def test_none_returns_empty(self):
        self.assertEqual(self.fn(None), {})

    def test_empty_string_returns_empty(self):
        self.assertEqual(self.fn(""), {})

    def test_walrus_float(self):
        result = self.fn("key:=1.5")
        self.assertAlmostEqual(result["key"], 1.5)

    def test_walrus_int(self):
        result = self.fn("key:=42")
        self.assertEqual(result["key"], 42)
        self.assertIsInstance(result["key"], int)

    def test_walrus_string_value(self):
        result = self.fn("key:=hello")
        self.assertEqual(result["key"], "hello")

    def test_colon_form_string(self):
        result = self.fn("key:value")
        self.assertEqual(result["key"], "value")

    def test_multiple_entries(self):
        result = self.fn("k1:=a,k2:=b")
        self.assertEqual(result["k1"], "a")
        self.assertEqual(result["k2"], "b")

    def test_mixed_types(self):
        result = self.fn("speed:=0.5,label:=robot,count:=3")
        self.assertAlmostEqual(result["speed"], 0.5)
        self.assertEqual(result["label"], "robot")
        self.assertEqual(result["count"], 3)

    def test_keys_stripped_of_whitespace(self):
        result = self.fn(" key :=1")
        self.assertIn("key", result)

    def test_pair_without_separator_skipped(self):
        # A token with no ':' or ':=' is silently ignored
        result = self.fn("badtoken")
        self.assertEqual(result, {})


# ---------------------------------------------------------------------------
# Gap 4 — Exhaustive parser sweep + DISPATCH/parser symmetry canary
# ---------------------------------------------------------------------------

class TestExhaustiveParser(unittest.TestCase):
    """Every (cmd, sub) in MINIMAL_ARGS parses correctly through build_parser()."""

    @classmethod
    def setUpClass(cls):
        if not check_rclpy_available():
            raise unittest.SkipTest("rclpy not available - requires ROS 2 environment")
        import ros2_cli
        cls.ros2_cli = ros2_cli
        cls.parser = ros2_cli.build_parser()

    def test_all_dispatch_keys_parse(self):
        """Every MINIMAL_ARGS entry must parse without error and resolve to the right command."""
        D = self.ros2_cli.DISPATCH
        for cmd, sub, cli_args in MINIMAL_ARGS:
            with self.subTest(cmd=cmd, sub=sub):
                args = self.parser.parse_args(cli_args)
                self.assertEqual(args.command, cmd,
                                 f"Expected command={cmd!r}, got {args.command!r}")
                if sub is not None:
                    self.assertEqual(args.subcommand, sub,
                                     f"Expected subcommand={sub!r}, got {args.subcommand!r}")
                self.assertIn((cmd, sub), D,
                              f"DISPATCH missing ({cmd!r}, {sub!r})")
                self.assertTrue(callable(D[(cmd, sub)]),
                                f"DISPATCH[({cmd!r}, {sub!r})] is not callable")

    def test_dispatch_covers_all_minimal_args_entries(self):
        """Every MINIMAL_ARGS entry is present in DISPATCH — canary for future drift."""
        D = self.ros2_cli.DISPATCH
        for cmd, sub, _ in MINIMAL_ARGS:
            with self.subTest(cmd=cmd, sub=sub):
                self.assertIn((cmd, sub), D,
                              f"DISPATCH missing ({cmd!r}, {sub!r}) — add it or update MINIMAL_ARGS")

    def test_all_dispatch_entries_covered_by_minimal_args(self):
        """Every DISPATCH entry has a corresponding MINIMAL_ARGS row — canary for new commands."""
        D = self.ros2_cli.DISPATCH
        covered = {(cmd, sub) for cmd, sub, _ in MINIMAL_ARGS}
        for key in D:
            with self.subTest(key=key):
                self.assertIn(key, covered,
                              f"DISPATCH key {key!r} has no MINIMAL_ARGS entry — add a parse test row")


if __name__ == "__main__":
    unittest.main()
