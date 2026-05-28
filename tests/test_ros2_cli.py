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
    # version / context / estop
    ("version",  None,               ["version"]),
    ("context",  None,               ["context"]),
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
    ("topics",   "echo-once",        ["topics", "echo-once", "/t"]),
    ("topics",   "depth-point",      ["topics", "depth-point", "--topic", "/depth", "--u", "320", "--v", "240"]),
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
    ("topics",   "classify",         ["topics", "classify"]),
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
    ("params",   "exists",           ["params", "exists", "/n:p"]),
    ("params",   "get-all-nodes",    ["params", "get-all-nodes", "use_sim_time"]),
    ("params",   "ls",               ["params", "ls", "/n"]),
    # actions
    ("actions",  "list",             ["actions", "list"]),
    ("actions",  "details",          ["actions", "details", "/a"]),
    ("actions",  "send",             ["actions", "send", "/a", "{}"]),
    ("actions",  "type",             ["actions", "type", "/a"]),
    ("actions",  "cancel",           ["actions", "cancel", "/a"]),
    ("actions",  "echo",             ["actions", "echo", "/a"]),
    ("actions",  "find",             ["actions", "find", "pkg/action/Type"]),
    ("actions",  "status",           ["actions", "status", "/a"]),
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
    # logs
    ("logs",     "list-runs",    ["logs", "list-runs"]),
    ("logs",     "query",        ["logs", "query"]),
    ("logs",     "tail",         ["logs", "tail"]),
    ("logs",     "node-summary", ["logs", "node-summary"]),
    # component
    ("component","types",    ["component", "types"]),
    ("component","list",     ["component", "list"]),
    ("component","ls",       ["component", "ls"]),
    ("component","load",     ["component", "load", "/ComponentManager", "demo_nodes_cpp", "demo_nodes_cpp::Talker"]),
    ("component","unload",   ["component", "unload", "/ComponentManager", "1"]),
    ("component","kill",      ["component", "kill", "comp_demo_nodes_cpp_standalone_talker"]),
    ("component","standalone",["component", "standalone", "demo_nodes_cpp", "demo_nodes_cpp::Talker"]),
    # pkg
    ("pkg",       "list",        ["pkg", "list"]),
    ("pkg",       "ls",          ["pkg", "ls"]),
    ("pkg",       "prefix",      ["pkg", "prefix", "std_msgs"]),
    ("pkg",       "executables", ["pkg", "executables", "turtlesim"]),
    ("pkg",       "xml",         ["pkg", "xml", "std_msgs"]),
    ("pkg",       "create",      ["pkg", "create", "my_pkg"]),
    # daemon
    ("daemon",   "status",   ["daemon", "status"]),
    ("daemon",   "start",    ["daemon", "start"]),
    ("daemon",   "stop",     ["daemon", "stop"]),
    # profile
    ("profile",  "scan",     ["profile", "scan"]),
    ("profile",  "show",     ["profile", "show"]),
    ("profile",  "rescan",   ["profile", "rescan"]),
    ("profile",  "list",     ["profile", "list"]),
    ("profile",  "ls",       ["profile", "ls"]),
    ("profile",  "annotate", ["profile", "annotate", "test annotation"]),
    # nav2
    ("nav2", "go",           ["nav2", "go", "1.0", "2.0"]),
    ("nav2", "cancel",       ["nav2", "cancel"]),
    ("nav2", "status",       ["nav2", "status"]),
    ("nav2", "go-waypoints", ["nav2", "go-waypoints", "1.0,2.0", "3.0,4.0"]),
    ("nav2", "initial-pose", ["nav2", "initial-pose", "1.0", "2.0"]),
    # foxglove
    ("foxglove", "start",    ["foxglove", "start"]),
    ("foxglove", "stop",     ["foxglove", "stop"]),
    ("foxglove", "status",   ["foxglove", "status"]),
    # system
    ("system",   "battery",  ["system", "battery"]),
    ("system",   "shutdown", ["system", "shutdown"]),
    ("system",   "reboot",   ["system", "reboot"]),
    # nav2 map / mode (three-level commands; subcommand is "map" / "mode")
    ("nav2",     "map",      ["nav2", "map", "list"]),
    ("nav2",     "mode",     ["nav2", "mode", "get"]),
    # arm
    ("nav2",     "localize", ["nav2", "localize"]),
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

    def test_output_conversion_and_parsing_helpers(self):
        """JSON output (unicode/nested); msg_to_dict; dict_to_msg; parse_node_param."""
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

    def test_diag_parsing_and_logic(self):
        """diag-list/diag parse+dispatch+constants; _parse_diag_array levels/empty/multiple."""
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
        # _parse_diag_array levels
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

    def test_battery_parsing_and_logic(self):
        """battery-list/battery parse and dispatched; _parse_battery_state status/health/NaN fields."""
        args = self.parser.parse_args(["topics", "battery-list"])
        self.assertEqual(args.subcommand, "battery-list")
        args = self.parser.parse_args(["topics", "battery"])
        self.assertIsNone(args.topic)
        self.assertEqual(args.timeout, 10.0)
        self.assertEqual(args.max_messages, 1)
        args = self.parser.parse_args([
            "topics", "battery", "--topic", "/b", "--timeout", "5",
            "--duration", "3.5", "--max-messages", "5"
        ])
        self.assertEqual(args.topic, "/b")
        self.assertEqual(args.timeout, 5.0)
        self.assertEqual(args.duration, 3.5)
        self.assertEqual(args.max_messages, 5)
        D = self.ros2_cli.DISPATCH
        self.assertTrue(callable(D[("topics", "battery-list")]))
        self.assertTrue(callable(D[("topics", "battery")]))
        self.assertIn("sensor_msgs/msg/BatteryState", self.ros2_topic.BATTERY_TYPES)
        for code, name in [(1, "CHARGING"), (2, "DISCHARGING")]:
            self.assertEqual(self.ros2_topic._parse_battery_state({"power_supply_status": code})["status_name"], name)
        for code, name in [(1, "GOOD"), (2, "OVERHEAT")]:
            self.assertEqual(self.ros2_topic._parse_battery_state({"power_supply_health": code})["health_name"], name)
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

    def test_global_overrides_and_retry(self):
        """Global timeout/retries parsed and applied; fallback correct; retry loop calls wait_for_service N times."""
        args = self.parser.parse_args(["--timeout", "30", "--retries", "3", "topics", "list"])
        self.assertEqual(args.global_timeout, 30.0)
        self.assertEqual(args.global_retries, 3)
        from types import SimpleNamespace
        args2 = SimpleNamespace(global_timeout=30.0, timeout=5.0, retries=1)
        self.ros2_cli._apply_global_overrides(args2)
        self.assertEqual(args2.timeout, 30.0)
        args3 = SimpleNamespace(global_timeout=None)
        self.ros2_cli._apply_global_overrides(args3)
        self.assertEqual(args3.retries, 1)
        self.assertFalse(hasattr(args3, "timeout"))
        # Retry loop
        with patch("ros2_service.rclpy"), \
             patch("ros2_service.output"), \
             patch("ros2_service.ROS2CLI") as mock_node_cls:
            import ros2_service
            mock_client = MagicMock()
            mock_client.wait_for_service.side_effect = [False, False, True]
            mock_node_cls.return_value.create_client.return_value = mock_client
            future = MagicMock()
            future.done.return_value = True
            mock_client.call_async.return_value = future
            svc_args = SimpleNamespace(service="/s", service_type="std_srvs/srv/Empty",
                                       extra_request=None, request="{}", timeout=1.0,
                                       retries=3, global_timeout=None)
            with patch("ros2_service.get_srv_type"), patch("ros2_service.time"):
                ros2_service.cmd_services_call(svc_args)
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

    def test_tf_parsing(self):
        """list/lookup/echo/monitor/static; euler↔quaternion/deg/transform; ls/get aliases; all callable."""
        p = self.parser
        self.assertEqual(p.parse_args(["tf", "list"]).subcommand, "list")
        args = p.parse_args(["tf", "lookup", "s", "t", "--timeout", "10"])
        self.assertEqual(args.source, "s")
        self.assertEqual(args.timeout, 10.0)
        args = p.parse_args(["tf", "echo", "s", "t", "--count", "3", "--once"])
        self.assertEqual(args.count, 3)
        self.assertTrue(args.once)
        args = p.parse_args(["tf", "monitor", "f", "--count", "2"])
        self.assertEqual(args.frame, "f")
        self.assertEqual(args.count, 2)
        args = p.parse_args(["tf", "static", "--from", "s", "--to", "t", "--xyz", "1", "2", "3", "--rpy", "0", "0", "0"])
        self.assertEqual(args.from_frame, "s")
        self.assertEqual(args.xyz, [1.0, 2.0, 3.0])
        self.assertEqual(p.parse_args(["tf", "static", "1", "2", "3", "0", "0", "0", "s", "t"]).pos_args,
                         ["1", "2", "3", "0", "0", "0", "s", "t"])
        self.assertEqual(p.parse_args(["tf", "euler-from-quaternion", "0", "0", "0", "1"]).w, 1.0)
        self.assertEqual(p.parse_args(["tf", "quaternion-from-euler", "0", "0", "0"]).yaw, 0.0)
        self.assertEqual(p.parse_args(["tf", "euler-from-quaternion-deg", "0", "0", "0", "1"]).subcommand, "euler-from-quaternion-deg")
        self.assertEqual(p.parse_args(["tf", "quaternion-from-euler-deg", "0", "0", "0"]).subcommand, "quaternion-from-euler-deg")
        pt = p.parse_args(["tf", "transform-point", "t", "s", "1", "2", "3"])
        self.assertEqual(pt.target, "t")
        self.assertEqual(pt.x, 1.0)
        self.assertEqual(p.parse_args(["tf", "transform-vector", "t", "s", "1", "0", "0"]).target, "t")
        self.assertEqual(p.parse_args(["tf", "ls"]).subcommand, "ls")
        self.assertEqual(p.parse_args(["tf", "get", "s", "t"]).subcommand, "get")
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

    def test_launch_commands_and_dispatch(self):
        """launch subcommands parse correctly; ls alias wired; all handlers callable."""
        p = self.parser
        args = p.parse_args(["launch", "new", "p", "f", "a:=1", "--timeout", "60"])
        self.assertEqual(args.package, "p")
        self.assertEqual(args.args, ["a:=1"])
        self.assertEqual(args.timeout, 60.0)
        self.assertEqual(p.parse_args(["launch", "list"]).subcommand, "list")
        self.assertEqual(p.parse_args(["launch", "kill", "s"]).session, "s")
        self.assertEqual(p.parse_args(["launch", "restart", "s"]).session, "s")
        self.assertEqual(p.parse_args(["launch", "foxglove", "9000"]).port, 9000)
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

    def test_run_commands_and_dispatch(self):
        """run subcommands parse correctly; ls alias wired; all handlers callable."""
        p = self.parser
        args = p.parse_args(["run", "new", "p", "e", "--presets", "a", "--params", "b", "--config-path", "c"])
        self.assertEqual(args.presets, "a")
        self.assertEqual(args.params, "b")
        self.assertEqual(args.config_path, "c")
        self.assertEqual(p.parse_args(["run", "list"]).subcommand, "list")
        self.assertEqual(p.parse_args(["run", "kill", "s"]).session, "s")
        self.assertEqual(p.parse_args(["run", "restart", "s"]).session, "s")
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

    def test_control_parsing(self):
        """list-controller-types/controllers/hw; load/unload/configure/aliases; state/switch/view; all DISPATCH callable."""
        args = self.parser.parse_args(["control", "list-controller-types", "--controller-manager", "/cm"])
        self.assertEqual(args.controller_manager, "/cm")
        self.assertEqual(args.timeout, 5.0)
        self.assertEqual(self.parser.parse_args(["control", "list-controllers"]).subcommand, "list-controllers")
        self.assertEqual(self.parser.parse_args(["control", "list-hardware-components"]).subcommand, "list-hardware-components")
        self.assertEqual(self.parser.parse_args(["control", "list-hardware-interfaces"]).subcommand, "list-hardware-interfaces")
        p = self.parser
        self.assertEqual(p.parse_args(["control", "load-controller", "c"]).name, "c")
        self.assertEqual(p.parse_args(["control", "unload-controller", "c"]).name, "c")
        self.assertEqual(p.parse_args(["control", "configure-controller", "c"]).name, "c")
        self.assertEqual(p.parse_args(["control", "load", "c"]).name, "c")
        self.assertEqual(p.parse_args(["control", "unload", "c"]).name, "c")
        D = self.ros2_cli.DISPATCH
        self.assertIs(D[("control", "load")], D[("control", "load-controller")])
        self.assertIs(D[("control", "unload")], D[("control", "unload-controller")])
        self.assertTrue(p.parse_args(["control", "reload-controller-libraries", "--force-kill"]).force_kill)
        self.assertEqual(p.parse_args(["control", "set-controller-state", "c", "active"]).state, "active")
        self.assertEqual(
            p.parse_args(["control", "set-hardware-component-state", "h", "inactive"]).state,
            "inactive")
        args2 = p.parse_args(["control", "switch-controllers", "--activate", "a", "--deactivate", "d", "--strictness", "STRICT", "--activate-asap"])
        self.assertEqual(args2.activate, ["a"])
        self.assertEqual(args2.deactivate, ["d"])
        self.assertEqual(args2.strictness, "STRICT")
        self.assertTrue(args2.activate_asap)
        args3 = p.parse_args(["control", "view-controller-chains", "--output", "o.pdf", "--channel-id", "1"])
        self.assertEqual(args3.output, "o.pdf")
        self.assertEqual(args3.channel_id, "1")
        for key in [k for k in D if k[0] == "control"]:
            self.assertTrue(callable(D[key]))


class TestBagParsing(unittest.TestCase):
    """Parser argument and DISPATCH wiring tests for the bag subcommands."""

    @classmethod
    def setUpClass(cls):
        if not check_rclpy_available():
            raise unittest.SkipTest("rclpy not available - requires ROS 2 environment")
        import ros2_cli
        cls.ros2_cli = ros2_cli
        cls.parser = ros2_cli.build_parser()

    def test_bag_info_parsing_and_dispatch(self):
        """Directory path and metadata.yaml path both accepted; DISPATCH wired."""
        args = self.parser.parse_args(["bag", "info", "/path/to/my_bag"])
        self.assertEqual(args.command, "bag")
        self.assertEqual(args.subcommand, "info")
        self.assertEqual(args.bag_path, "/path/to/my_bag")
        self.assertEqual(
            self.parser.parse_args(["bag", "info", "/path/to/my_bag/metadata.yaml"]).bag_path,
            "/path/to/my_bag/metadata.yaml",
        )
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

    def test_bag_metadata_logic(self):
        """_ns_to_sec conversions; _parse_metadata from dir, yaml path, and empty dir."""
        self.assertAlmostEqual(self.ros2_bag._ns_to_sec(0), 0.0)
        self.assertAlmostEqual(self.ros2_bag._ns_to_sec(1_000_000_000), 1.0)
        self.assertAlmostEqual(self.ros2_bag._ns_to_sec(500_000_000), 0.5)
        self.assertAlmostEqual(self.ros2_bag._ns_to_sec(1_234_567_890), 1.23456789)
        try:
            import yaml
        except ImportError:
            raise unittest.SkipTest("PyYAML not available")
        import tempfile, pathlib
        meta = {
            "rosbag2_bagfile_information": {
                "duration":        {"nanoseconds": 5_000_000_000},
                "starting_time":   {"nanoseconds_since_epoch": 1_700_000_000_000_000_000},
                "storage_identifier": "sqlite3",
                "message_count":   42,
                "topics_with_message_count": [
                    {"topic_metadata": {"name": "/cmd_vel", "type": "geometry_msgs/msg/Twist",
                                        "serialization_format": "cdr", "offered_qos_profiles": ""},
                     "message_count": 10},
                    {"topic_metadata": {"name": "/odom", "type": "nav_msgs/msg/Odometry",
                                        "serialization_format": "cdr", "offered_qos_profiles": ""},
                     "message_count": 32},
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
        # metadata.yaml path accepted directly
        meta2 = {"rosbag2_bagfile_information": {"storage_identifier": "mcap"}}
        with tempfile.TemporaryDirectory() as tmpdir2:
            p2 = pathlib.Path(tmpdir2) / "metadata.yaml"
            with open(p2, "w") as fh:
                yaml.dump(meta2, fh)
            r2 = self.ros2_bag._parse_metadata(str(p2))
        self.assertEqual(r2["rosbag2_bagfile_information"]["storage_identifier"], "mcap")
        # Empty dir → FileNotFoundError
        with tempfile.TemporaryDirectory() as tmpdir3:
            with self.assertRaises(FileNotFoundError):
                self.ros2_bag._parse_metadata(tmpdir3)


class TestComponentParsing(unittest.TestCase):
    """Parser argument and DISPATCH wiring tests for the component subcommands."""

    @classmethod
    def setUpClass(cls):
        if not check_rclpy_available():
            raise unittest.SkipTest("rclpy not available - requires ROS 2 environment")
        import ros2_cli
        cls.ros2_cli = ros2_cli
        cls.parser = ros2_cli.build_parser()

    def test_component_parsing(self):
        """types/list/ls dispatch; load positionals+flags+log_level-int; unload+kill; standalone args."""
        self.assertEqual(self.parser.parse_args(["component", "types"]).subcommand, "types")
        args = self.parser.parse_args(["component", "list"])
        self.assertEqual(args.subcommand, "list")
        self.assertEqual(args.timeout, 5.0)
        self.assertEqual(self.parser.parse_args(["component", "ls"]).subcommand, "ls")
        D = self.ros2_cli.DISPATCH
        for key in [("component", "types"), ("component", "list"), ("component", "ls"),
                    ("component", "load"), ("component", "unload"),
                    ("component", "kill"), ("component", "standalone")]:
            self.assertIn(key, D, f"Missing dispatch entry: {key}")
            self.assertTrue(callable(D[key]))
        args = self.parser.parse_args([
            "component", "load", "/my_container", "demo_nodes_cpp", "demo_nodes_cpp::Talker"
        ])
        self.assertEqual(args.container, "/my_container")
        self.assertEqual(args.package_name, "demo_nodes_cpp")
        self.assertEqual(args.plugin_name, "demo_nodes_cpp::Talker")
        args2 = self.parser.parse_args([
            "component", "load", "/c", "pkg", "pkg::Node",
            "--node-name", "my_node", "--node-namespace", "/ns", "--log-level", "10",
            "--remap", "/from:=/to",
        ])
        self.assertEqual(args2.node_name, "my_node")
        self.assertEqual(args2.node_namespace, "/ns")
        self.assertEqual(args2.log_level, 10)
        self.assertEqual(args2.remap_rules, ["/from:=/to"])
        default_args = self.parser.parse_args(["component", "load", "/c", "pkg", "pkg::Node"])
        self.assertIsInstance(default_args.log_level, int)
        self.assertEqual(default_args.log_level, 0)
        uargs = self.parser.parse_args(["component", "unload", "/my_container", "42"])
        self.assertEqual(uargs.container, "/my_container")
        self.assertEqual(uargs.unique_id, 42)
        self.assertIsInstance(uargs.unique_id, int)
        kill = self.parser.parse_args(["component", "kill", "comp_pkg_standalone_talker"])
        self.assertEqual(kill.subcommand, "kill")
        self.assertEqual(kill.session, "comp_pkg_standalone_talker")
        sa = self.parser.parse_args(["component", "standalone", "demo_nodes_cpp", "demo_nodes_cpp::Talker"])
        self.assertEqual(sa.package_name, "demo_nodes_cpp")
        self.assertEqual(sa.container_type, "component_container")
        self.assertIsInstance(sa.log_level, int)
        self.assertEqual(sa.log_level, 0)
        self.assertEqual(sa.timeout, 10.0)
        self.assertEqual(
            self.parser.parse_args([
                "component", "standalone", "pkg", "pkg::Node",
                "--container-type", "component_container_mt",
            ]).container_type,
            "component_container_mt",
        )


class TestPkgParsing(unittest.TestCase):
    """Parser argument and DISPATCH wiring tests for the pkg subcommands."""

    @classmethod
    def setUpClass(cls):
        if not check_rclpy_available():
            raise unittest.SkipTest("rclpy not available - requires ROS 2 environment")
        import ros2_cli
        cls.ros2_cli = ros2_cli
        cls.parser = ros2_cli.build_parser()

    def test_pkg_parsing_and_dispatch(self):
        """list, ls, prefix, executables, xml all parse correctly and are in DISPATCH."""
        p = self.parser
        self.assertEqual(p.parse_args(["pkg", "list"]).subcommand, "list")
        self.assertEqual(p.parse_args(["pkg", "ls"]).subcommand, "ls")
        self.assertEqual(p.parse_args(["pkg", "prefix", "nav2_bringup"]).package, "nav2_bringup")
        self.assertEqual(p.parse_args(["pkg", "executables", "turtlesim"]).package, "turtlesim")
        self.assertEqual(p.parse_args(["pkg", "xml", "std_msgs"]).package, "std_msgs")
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

    def _mock_pkg_modules(self, packages=None, prefix_return=None, prefix_error=None,
                          share_return=None, share_error=None):
        """Build a (mock_pkgs, packages_mock) for patching ament_index_python.packages."""
        mock = MagicMock()
        if packages is not None:
            mock.get_packages_with_prefixes.return_value = {p: "/opt/ros" for p in packages}
        if prefix_error:
            mock.get_package_prefix.side_effect = prefix_error
        elif prefix_return is not None:
            mock.get_package_prefix.return_value = prefix_return
        if share_error:
            mock.get_package_share_directory.side_effect = share_error
        elif share_return is not None:
            mock.get_package_share_directory.return_value = share_return
        return mock

    def _run_pkg(self, func, args=None, mock_pkgs=None, extra_patches=None):
        captured = []
        patches = {"ament_index_python.packages": mock_pkgs} if mock_pkgs else {}
        with patch.dict(sys.modules, patches), \
             patch("ros2_pkg.output", side_effect=captured.append):
            func(args)
        return captured[0] if captured else None

    def test_pkg_logic(self):
        """List: sorted+total, missing ament→error; prefix: found/missing; executables; xml."""
        import tempfile
        mock = self._mock_pkg_modules(packages=["zebra_pkg", "alpha_pkg", "middle_pkg"])
        result = self._run_pkg(self.ros2_pkg.cmd_pkg_list, mock_pkgs=mock)
        self.assertEqual(result["packages"], ["alpha_pkg", "middle_pkg", "zebra_pkg"])
        self.assertEqual(result["total"], 3)
        captured = []
        with patch.dict(sys.modules, {"ament_index_python": None,
                                       "ament_index_python.packages": None}), \
             patch("ros2_pkg.output", side_effect=captured.append):
            self.ros2_pkg.cmd_pkg_list(None)
        self.assertIn("error", captured[0])
        r_pf = self._run_pkg(self.ros2_pkg.cmd_pkg_prefix,
                              argparse.Namespace(package="turtlesim"),
                              mock_pkgs=self._mock_pkg_modules(prefix_return="/opt/ros/humble"))
        self.assertEqual(r_pf["package"], "turtlesim")
        self.assertEqual(r_pf["prefix"], "/opt/ros/humble")
        r_pf2 = self._run_pkg(self.ros2_pkg.cmd_pkg_prefix,
                               argparse.Namespace(package="nonexistent_pkg"),
                               mock_pkgs=self._mock_pkg_modules(
                                   prefix_error=KeyError("nonexistent_pkg")))
        self.assertIn("error", r_pf2)
        self.assertIn("nonexistent_pkg", r_pf2["error"])
        mock_ex = MagicMock()
        with tempfile.TemporaryDirectory() as tmpdir:
            lib_dir = pathlib.Path(tmpdir) / "lib" / "mypkg"
            lib_dir.mkdir(parents=True)
            exe = lib_dir / "my_node"
            exe.write_text("#!/bin/sh")
            exe.chmod(0o755)
            (lib_dir / "readme.txt").write_text("not a binary")
            mock_ex.get_package_prefix.return_value = tmpdir
            r_exe = self._run_pkg(self.ros2_pkg.cmd_pkg_executables,
                                  argparse.Namespace(package="mypkg"), mock_pkgs=mock_ex)
            self.assertIn("my_node", r_exe["executables"])
            self.assertNotIn("readme.txt", r_exe["executables"])
            self.assertEqual(r_exe["total"], 1)
        with tempfile.TemporaryDirectory() as tmpdir2:
            mock2 = MagicMock()
            mock2.get_package_prefix.return_value = tmpdir2
            r2 = self._run_pkg(self.ros2_pkg.cmd_pkg_executables,
                               argparse.Namespace(package="mypkg"), mock_pkgs=mock2)
            self.assertEqual(r2["executables"], [])
        mock3 = MagicMock()
        mock3.get_package_prefix.side_effect = KeyError("ghost_pkg")
        self.assertIn("error", self._run_pkg(self.ros2_pkg.cmd_pkg_executables,
                                             argparse.Namespace(package="ghost_pkg"),
                                             mock_pkgs=mock3))
        xml_content = '<?xml version="1.0"?>\n<package format="3"><name>mypkg</name></package>\n'
        with tempfile.TemporaryDirectory() as tmpdir:
            share_dir = pathlib.Path(tmpdir) / "share" / "mypkg"
            share_dir.mkdir(parents=True)
            (share_dir / "package.xml").write_text(xml_content, encoding="utf-8")
            r_xml = self._run_pkg(self.ros2_pkg.cmd_pkg_xml,
                                  argparse.Namespace(package="mypkg"),
                                  mock_pkgs=self._mock_pkg_modules(share_return=str(share_dir)))
            self.assertEqual(r_xml["package"], "mypkg")
            self.assertIn("<name>mypkg</name>", r_xml["xml"])
            self.assertIn("path", r_xml)
        self.assertIn("error", self._run_pkg(self.ros2_pkg.cmd_pkg_xml,
                                             argparse.Namespace(package="ghost"),
                                             mock_pkgs=self._mock_pkg_modules(
                                                 share_error=KeyError("ghost"))))
        with tempfile.TemporaryDirectory() as tmpdir3:
            share3 = pathlib.Path(tmpdir3) / "share" / "mypkg"
            share3.mkdir(parents=True)  # no package.xml
            r_missing = self._run_pkg(self.ros2_pkg.cmd_pkg_xml,
                                      argparse.Namespace(package="mypkg"),
                                      mock_pkgs=self._mock_pkg_modules(share_return=str(share3)))
            self.assertIn("error", r_missing)


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

    def test_cmd_component_types(self):
        """Missing ament→error; mocked ament→correct listing; per-package error→warnings."""
        import sys
        from unittest.mock import MagicMock
        captured_no_ament = []
        with patch.dict(sys.modules, {"ament_index_python": None}), \
             patch("ros2_component.output", side_effect=captured_no_ament.append):
            self.ros2_component.cmd_component_types(None)
        self.assertEqual(len(captured_no_ament), 1)
        self.assertIn("error", captured_no_ament[0])
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
        result = captured[0]
        self.assertIn("components", result)
        self.assertEqual(result["total"], 2)
        self.assertIn("demo_nodes_cpp", result["packages"])
        names = [c["type_name"] for c in result["components"]]
        self.assertIn("demo_nodes_cpp::Talker", names)
        self.assertIn("demo_nodes_cpp::Listener", names)
        self.assertNotIn("warnings", result)
        # Partial failure: bad pkg → warning, good pkg still enumerated
        mock2 = MagicMock()
        mock2.get_resources.return_value = {"good_pkg": "/opt/ros/humble", "bad_pkg": "/opt/ros/humble"}
        def get_resource(res_type, pkg):
            if pkg == "bad_pkg":
                raise RuntimeError("disk error")
            return ("good_pkg::GoodNode\n", "/opt")
        mock2.get_resource.side_effect = get_resource
        captured2 = []
        with patch.dict(sys.modules, {"ament_index_python": mock2}), \
             patch("ros2_component.output", side_effect=captured2.append):
            self.ros2_component.cmd_component_types(None)
        result2 = captured2[0]
        self.assertEqual(result2["total"], 1)
        self.assertIn("good_pkg", result2["packages"])
        self.assertIn("warnings", result2)
        self.assertEqual(len(result2["warnings"]), 1)
        self.assertEqual(result2["warnings"][0]["package"], "bad_pkg")


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

    def test_bag_info_command(self):
        """Required fields; topics sorted+fields; topic_count for 0/1/5; compression; error paths; never raises."""
        self._skip_if_no_yaml()
        import tempfile, pathlib, types
        ns = 1_700_000_000_123_456_789
        with tempfile.TemporaryDirectory() as tmpdir:
            self._write_bag(tmpdir, self._base_meta(
                duration={"nanoseconds": 5_500_000_000},
                starting_time={"nanoseconds_since_epoch": ns},
                message_count=999, storage_identifier="mcap",
            ))
            result = self._run(tmpdir)
        for field in ("bag_path", "storage_identifier", "duration",
                      "starting_time", "message_count", "topic_count", "topics"):
            self.assertIn(field, result)
        self.assertAlmostEqual(result["duration"]["seconds"], 5.5)
        self.assertEqual(result["duration"]["nanoseconds"], 5_500_000_000)
        self.assertEqual(result["starting_time"]["nanoseconds_since_epoch"], ns)
        self.assertEqual(result["message_count"], 999)
        self.assertEqual(result["storage_identifier"], "mcap")
        self.assertTrue(pathlib.Path(result["bag_path"]).is_absolute())
        with tempfile.TemporaryDirectory() as tmpdir:
            self._write_bag(tmpdir, self._base_meta(topics_with_message_count=[
                {"topic_metadata": {"name": "/scan",    "type": "sensor_msgs/msg/LaserScan",
                                    "serialization_format": "cdr", "offered_qos_profiles": ""},
                 "message_count": 10},
                {"topic_metadata": {"name": "/cmd_vel", "type": "geometry_msgs/msg/Twist",
                                    "serialization_format": "cdr", "offered_qos_profiles": ""},
                 "message_count": 50},
                {"topic_metadata": {"name": "/odom",    "type": "nav_msgs/msg/Odometry",
                                    "serialization_format": "cdr", "offered_qos_profiles": ""},
                 "message_count": 100},
            ]))
            r2 = self._run(tmpdir)
        self.assertEqual([t["name"] for t in r2["topics"]], ["/cmd_vel", "/odom", "/scan"])
        self.assertEqual(r2["topic_count"], len(r2["topics"]))
        for field in ("name", "type", "serialization_format", "offered_qos_profiles", "message_count"):
            self.assertIn(field, r2["topics"][0])
        for n in (0, 1, 5):
            t_n = [{"topic_metadata": {"name": f"/t{i}", "type": "std_msgs/msg/String",
                                       "serialization_format": "cdr", "offered_qos_profiles": ""},
                    "message_count": i} for i in range(n)]
            with tempfile.TemporaryDirectory() as tmpdir2:
                self._write_bag(tmpdir2, self._base_meta(topics_with_message_count=t_n))
                r3 = self._run(tmpdir2)
            with self.subTest(n=n):
                self.assertEqual(r3["topic_count"], n)
        # Compression fields
        with tempfile.TemporaryDirectory() as tmpdir:
            self._write_bag(tmpdir, self._base_meta(
                compression_format="zstd", compression_mode="file"))
            result = self._run(tmpdir)
            self.assertEqual(result["compression_format"], "zstd")
            self.assertEqual(result["compression_mode"], "file")
            self._write_bag(tmpdir, self._base_meta())
            self.assertNotIn("compression_format", self._run(tmpdir))
            self._write_bag(tmpdir, self._base_meta(storage_identifier="mcap"))
            meta_path = str(pathlib.Path(tmpdir) / "metadata.yaml")
            self.assertNotIn("error", self._run(meta_path))
        # Error paths
        for bad_path in ("/does/not/exist/bag", ""):
            with self.subTest(path=bad_path):
                r = self._run(bad_path)
                self.assertIn("error", r)
                self.assertIn("hint", r)
                try:
                    captured = []
                    with patch("ros2_bag.output", side_effect=captured.append):
                        self.ros2_bag.cmd_bag_info(types.SimpleNamespace(bag_path=bad_path))
                except Exception as exc:
                    self.fail(f"cmd_bag_info raised for {bad_path!r}: {exc}")


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

    def test_required_positionals_and_unknown_flags(self):
        """Required positionals enforced; unknown flags rejected; all handlers callable."""
        # bag info: positional required; unknown flags rejected
        self._assert_missing_required_fails("bag", "info")
        self._assert_unknown_flag_fails("bag", "info", "/path", "--invented-flag")
        # component types: unknown flags rejected
        self._assert_unknown_flag_fails("component", "types", "--invented-flag")
        # core command spot-checks
        self._assert_missing_required_fails("topics", "type")
        self._assert_missing_required_fails("params", "set")
        # all dispatch handlers must be callable
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

    def test_error_output_structure(self):
        """bag_info bad paths→error str; empty dir→error+hint; component_types ImportError→error+detail; RuntimeError→error."""
        import types, tempfile, sys
        from unittest.mock import MagicMock
        for bad_path in ("/does/not/exist", "/no/such/path", ""):
            with self.subTest(bag_path=bad_path):
                result = self._capture_once(
                    "ros2_bag", self.ros2_bag.cmd_bag_info,
                    types.SimpleNamespace(bag_path=bad_path)
                )
                self.assertIn("error", result)
                self.assertIsInstance(result["error"], str)
                self.assertGreater(len(result["error"]), 0)
        with tempfile.TemporaryDirectory() as tmpdir:
            result = self._capture_once(
                "ros2_bag", self.ros2_bag.cmd_bag_info,
                types.SimpleNamespace(bag_path=tmpdir)
            )
        self.assertIn("error", result)
        self.assertIn("hint", result)
        self.assertIsInstance(result["hint"], str)
        self.assertGreater(len(result["hint"]), 0)
        captured = []
        with patch.dict(sys.modules, {"ament_index_python": None}), \
             patch("ros2_component.output", side_effect=captured.append):
            self.ros2_component.cmd_component_types(None)
        self.assertEqual(len(captured), 1)
        self.assertIn("error", captured[0])
        self.assertIn("detail", captured[0])
        self.assertIsInstance(captured[0]["error"], str)
        mock_ament = MagicMock()
        mock_ament.get_resources.side_effect = RuntimeError("ament index corrupt")
        captured2 = []
        with patch.dict(sys.modules, {"ament_index_python": mock_ament}), \
             patch("ros2_component.output", side_effect=captured2.append):
            self.ros2_component.cmd_component_types(None)
        self.assertIn("error", captured2[0])


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

    def test_component_types_output_validation(self):
        """Required keys; total==len; sorted packages; empty/comments filtered; partial failure→warning."""
        import sys
        result = self._run(self._mock_ament({"pkg_a": ["pkg_a::NodeA"]}))
        for key in ("components", "total", "packages"):
            self.assertIn(key, result)
        for n in (1, 3, 7):
            r = self._run(self._mock_ament({"my_pkg": [f"pkg::N{i}" for i in range(n)]}))
            self.assertEqual(r["total"], n)
            self.assertEqual(r["total"], len(r["components"]))
        r2 = self._run(self._mock_ament({"z_pkg": ["z_pkg::Z"], "a_pkg": ["a_pkg::A"],
                                         "m_pkg": ["m_pkg::M"]}))
        self.assertEqual(r2["packages"], sorted(r2["packages"]))
        r3 = self._run(self._mock_ament({"pkg_a": ["pkg_a::N1", "pkg_a::N2"]}))
        self.assertEqual(r3["packages"], ["pkg_a"])
        for comp in r2["components"]:
            self.assertIn("package", comp)
            self.assertIn("type_name", comp)
            self.assertGreater(len(comp["package"]), 0)
        from unittest.mock import MagicMock
        mock_empty = MagicMock()
        mock_empty.get_resources.return_value = {}
        r_empty = self._run(mock_empty)
        self.assertEqual(r_empty["total"], 0)
        self.assertEqual(r_empty["components"], [])
        self.assertNotIn("warnings", r_empty)
        mock_comments = MagicMock()
        mock_comments.get_resources.return_value = {"pkg": "/path"}
        mock_comments.get_resource.return_value = (
            "# comment\npkg::RealNode\n\n   \npkg::AnotherNode\n", "/path")
        r_c = self._run(mock_comments)
        self.assertEqual(r_c["total"], 2)
        type_names = [c["type_name"] for c in r_c["components"]]
        self.assertIn("pkg::RealNode", type_names)
        self.assertIn("pkg::AnotherNode", type_names)
        # Multi-pkg: sorted; partial failure → warning
        r_beta = self._run(self._mock_ament({"beta_pkg": ["beta_pkg::Node"],
                                             "alpha_pkg": ["alpha_pkg::Node"]}))
        self.assertEqual([c["package"] for c in r_beta["components"]],
                         sorted([c["package"] for c in r_beta["components"]]))
        self.assertNotIn("warnings", r_beta)
        mock = MagicMock()
        mock.get_resources.return_value = {"good_pkg": "/p", "bad_pkg": "/p"}
        def _get(res_type, pkg):
            if pkg == "bad_pkg":
                raise RuntimeError("disk error")
            return ("good_pkg::GoodNode\n", "/p")
        mock.get_resource.side_effect = _get
        captured = []
        with patch.dict(sys.modules, {"ament_index_python": mock}), \
             patch("ros2_component.output", side_effect=captured.append):
            self.ros2_component.cmd_component_types(None)
        w = captured[0]["warnings"][0]
        self.assertIn("package", w)
        self.assertIn("error", w)
        self.assertEqual(w["package"], "bad_pkg")
        self.assertEqual(captured[0]["total"], 1)


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

    def test_no_containers_output(self):
        """No containers: required keys present, empty lists, totals zero, hint; import error → error+hint."""
        result = self._run_no_containers()
        for key in ("containers", "total_containers", "total_components"):
            self.assertIn(key, result)
        self.assertEqual(result["containers"], [])
        self.assertEqual(result["total_containers"], 0)
        self.assertEqual(result["total_components"], 0)
        self.assertIn("hint", result)
        # composition_interfaces.srv import error → error+hint
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

    def test_load_all_paths(self):
        """Success: required keys present; failure: success_false+error_message; svc unavail+import error→error+hint."""
        result = self._run(True, full_node_name="/c/talker", unique_id=1)
        self.assertTrue(result["success"])
        for key in ("container", "package_name", "plugin_name", "full_node_name", "unique_id"):
            self.assertIn(key, result)
        result2 = self._run(False, error_message="Plugin not found")
        self.assertFalse(result2["success"])
        self.assertIn("error_message", result2)
        self.assertEqual(result2["error_message"], "Plugin not found")
        r = self._run(True, svc_available=False)
        self.assertIn("error", r)
        self.assertIn("hint", r)
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

    def test_unload_all_paths(self):
        """Success: required keys+unique_id; failure: success_false+error_message; svc unavail+import error→error+hint."""
        result = self._run(True)
        self.assertTrue(result["success"])
        for key in ("container", "unique_id"):
            self.assertIn(key, result)
        self.assertEqual(result["unique_id"], 1)
        result2 = self._run(False, error_message="Component not found")
        self.assertFalse(result2["success"])
        self.assertEqual(result2["error_message"], "Component not found")
        self.assertIn("error", self._run(True, svc_available=False))
        self.assertIn("hint",  self._run(True, svc_available=False))
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

    def test_component_standalone_logic(self):
        """Success keys; tmux/session/tmux-fail errors; container not ready; isolated suffix; import error."""
        result = self._run(True)
        self.assertTrue(result["success"])
        for key in ("session", "container", "container_type",
                    "package_name", "plugin_name", "full_node_name", "unique_id"):
            self.assertIn(key, result)
        self.assertEqual(result["container"], "/standalone_talker")
        from unittest.mock import MagicMock, patch
        captured = []
        with patch("ros2_component.check_tmux", return_value=False), \
             patch("ros2_component.output", side_effect=captured.append):
            self.ros2_component.cmd_component_standalone(self._make_args())
        self.assertIn("error", captured[0])
        mock_srv = MagicMock()
        captured2 = []
        with patch("ros2_component.check_tmux",    return_value=True), \
             patch("ros2_component.session_exists", return_value=True), \
             patch("ros2_component.output",         side_effect=captured2.append), \
             patch.dict("sys.modules", {"composition_interfaces.srv": mock_srv}):
            self.ros2_component.cmd_component_standalone(self._make_args())
        self.assertIn("error", captured2[0])
        self.assertIn("already exists", captured2[0]["error"])
        self.assertIn("error", self._run(True, tmux_ok=False))
        # Container not ready
        r_alive = self._run(container_ready=False, container_node_alive=True)
        self.assertIn("error", r_alive)
        self.assertTrue(r_alive.get("container_started"))
        r_dead = self._run(container_ready=False, container_node_alive=False)
        self.assertIn("error", r_dead)
        self.assertFalse(r_dead.get("container_started"))
        # Alt container path discovered
        alt_container = "/standalone_talker/_container"
        mock_node = MagicMock()
        mock_node.create_client.return_value = MagicMock()
        mock_node.get_service_names_and_types.return_value = [
            (f"{alt_container}/list_nodes", ["composition_interfaces/srv/ListNodes"])
        ]
        mock_node.get_node_names_and_namespaces.return_value = []
        mock_srv2 = MagicMock()
        mock_srv2.LoadNode         = MagicMock()
        mock_srv2.LoadNode.Request = MagicMock(return_value=MagicMock())
        mock_srv2.ListNodes        = MagicMock()
        captured3 = []
        with patch("ros2_component.check_tmux",         return_value=True), \
             patch("ros2_component.session_exists",      return_value=False), \
             patch("ros2_component.source_local_ws",     return_value=(None, "not_found")), \
             patch("ros2_component.run_cmd",             return_value=("", "", 0)), \
             patch("ros2_component.check_session_alive", return_value=True), \
             patch("ros2_component.save_session"), \
             patch("ros2_component.ros2_context") as ctx, \
             patch("ros2_component.ROS2CLI",             return_value=mock_node), \
             patch("rclpy.spin_once"), \
             patch("ros2_component.output",              side_effect=captured3.append), \
             patch.dict("sys.modules", {"composition_interfaces.srv": mock_srv2}):
            ctx.return_value.__enter__ = MagicMock(return_value=None)
            ctx.return_value.__exit__  = MagicMock(return_value=False)
            self.ros2_component.cmd_component_standalone(self._make_args(timeout=0))
        self.assertIn("error", captured3[0])
        self.assertIn("container_found_at", captured3[0])
        self.assertEqual(captured3[0]["container_found_at"], alt_container)

        # isolated container suffix + failed load + import error
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
        captured_iso = []
        with patch("ros2_component.check_tmux",         return_value=True), \
             patch("ros2_component.session_exists",      return_value=False), \
             patch("ros2_component.source_local_ws",     return_value=(None, "not_found")), \
             patch("ros2_component.run_cmd",             return_value=("", "", 0)), \
             patch("ros2_component.check_session_alive", return_value=True), \
             patch("ros2_component.save_session"), \
             patch("ros2_component.ros2_context") as ctx, \
             patch("ros2_component.ROS2CLI",             return_value=mock_node), \
             patch("rclpy.spin_once"), \
             patch("ros2_component.output",              side_effect=captured_iso.append), \
             patch.dict("sys.modules", {"composition_interfaces.srv": mock_srv}):
            ctx.return_value.__enter__ = MagicMock(return_value=None)
            ctx.return_value.__exit__  = MagicMock(return_value=False)
            self.ros2_component.cmd_component_standalone(self._make_args(container_type="component_container_isolated"))
        self.assertTrue(captured_iso[0]["success"])
        self.assertEqual(captured_iso[0]["container"], "/standalone_talker/_container")
        r_fail = self._run(False, error_message="Plugin not found")
        self.assertFalse(r_fail["success"])
        self.assertIn("error_message", r_fail)
        captured_imp = []
        with patch.dict("sys.modules", {"composition_interfaces.srv": None}), \
             patch("ros2_component.check_tmux",     return_value=True), \
             patch("ros2_component.session_exists",  return_value=False), \
             patch("ros2_component.output",          side_effect=captured_imp.append):
            self.ros2_component.cmd_component_standalone(self._make_args())
        self.assertIn("error", captured_imp[0])
        self.assertIn("hint", captured_imp[0])


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

    def test_resolve_field(self):
        """Simple/nested paths, zero value, sub-dict; list indexing, ROS paths; missing key/OOB errors."""
        self.assertEqual(self.resolve_field({"x": 1}, "x"), 1)
        self.assertEqual(self.resolve_field({"a": {"b": 2}}, "a.b"), 2)
        self.assertEqual(self.resolve_field({"a": {"b": {"c": 3}}}, "a.b.c"), 3)
        self.assertEqual(self.resolve_field({"v": 0}, "v"), 0)
        result = self.resolve_field({"pose": {"position": {"x": 1.0}}}, "pose.position")
        self.assertIsInstance(result, dict)
        self.assertIn("x", result)
        self.assertEqual(self.resolve_field({"ranges": [10, 20, 30]}, "ranges.1"), 20)
        msg = {"status": [{"name": "cpu"}, {"name": "disk"}]}
        self.assertEqual(self.resolve_field(msg, "status.0.name"), "cpu")
        self.assertEqual(self.resolve_field(msg, "status.1.name"), "disk")
        odom = {"pose": {"pose": {"position": {"x": 1.23, "y": 0.0, "z": 0.0}}}}
        self.assertAlmostEqual(self.resolve_field(odom, "pose.pose.position.x"), 1.23)
        self.assertAlmostEqual(self.resolve_field({"ranges": [0.5, 1.0, 2.0]}, "ranges.0"), 0.5)
        with self.assertRaises(KeyError):
            self.resolve_field({"x": 1}, "y")
        with self.assertRaises(KeyError):
            self.resolve_field({"a": {"b": 1}}, "a.c")
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
        # fuzzy_match lives in ros2_utils (public name, no underscore).
        # ros2_launch re-exports it, but the canonical import is from ros2_utils.
        from ros2_utils import fuzzy_match
        cls._fuzzy_match = staticmethod(fuzzy_match)

    def test_fuzzy_match(self):
        """Exact/hyphen=1.0; substring=0.8; empty→[]; sorted desc; all matches returned."""
        r = self._fuzzy_match("use_sim_time", ["use_sim_time", "robot_name"])
        self.assertTrue(any(c == "use_sim_time" and s == 1.0 for c, s in r))
        r2 = self._fuzzy_match("use-sim-time", ["use_sim_time"])
        self.assertTrue(len(r2) > 0 and r2[0][1] == 1.0)
        self.assertEqual(self._fuzzy_match("sim", ["use_sim_time"])[0][1], 0.8)
        self.assertGreaterEqual(
            self._fuzzy_match("use_sim_time_extended", ["use_sim_time"])[0][1], 0.7)
        match_scores = dict(self._fuzzy_match("robot", ["robot_name", "unrelated"]))
        self.assertEqual(match_scores.get("robot_name"), 0.8)
        self.assertEqual(self._fuzzy_match("", ["use_sim_time"]), [])
        self.assertEqual(self._fuzzy_match("use_sim_time", []), [])
        self.assertEqual(self._fuzzy_match("xyz_unknown", ["use_sim_time"]), [])
        results = self._fuzzy_match("sim", ["use_sim_time", "simulation_mode", "sim"])
        scores = [s for _, s in results]
        self.assertEqual(scores, sorted(scores, reverse=True))
        returned = {c for c, _ in self._fuzzy_match("sim", ["sim_time", "sim_delay"])}
        self.assertTrue(returned.issuperset({"sim_time", "sim_delay"}))


class TestValidateLaunchArgs(unittest.TestCase):
    """Tests for ros2_launch._validate_launch_args — the argument validation
    layer that warns about unrecognised launch args but always passes them through.

    The function is warn-only: it never drops or renames arguments.  The ROS 2
    launch system is the authoritative validator; we must not silently discard
    what the user asked for.
    """

    @classmethod
    def setUpClass(cls):
        if not check_rclpy_available():
            raise unittest.SkipTest("rclpy not available - requires ROS 2 environment")
        from ros2_launch import _validate_launch_args
        cls._validate_launch_args = staticmethod(_validate_launch_args)

    def test_validate_launch_args(self):
        """Exact matches → no notices; unknown/fuzzy → one notice each; empty/mixed handled correctly."""
        # Exact matches produce no notices
        for user_args, avail in [
            (["use_sim_time:=true"], ["use_sim_time", "robot_name"]),
            (["use_sim_time:=true", "robot_name:=robot1"], ["use_sim_time", "robot_name"]),
            (["use_sim_time"], ["use_sim_time"]),
            (["use_sim_time=true"], ["use_sim_time"]),
        ]:
            with self.subTest(user_args=user_args):
                validated, notices = self._validate_launch_args(user_args, avail)
                for arg in user_args:
                    self.assertIn(arg, validated)
                self.assertEqual(notices, [])
        # Unknown arg → passes through with one notice
        validated, notices = self._validate_launch_args(
            ["invented_arg:=value"], ["use_sim_time", "robot_name"])
        self.assertIn("invented_arg:=value", validated)
        self.assertEqual(len(notices), 1)
        self.assertIn("invented_arg", notices[0])
        self.assertTrue(any("use_sim_time" in n or "robot_name" in n for n in notices))
        # Fuzzy (close but not exact) → passes through with one notice
        v2, n2 = self._validate_launch_args(["sim_time:=true"], ["use_sim_time"])
        self.assertIn("sim_time:=true", v2)
        self.assertEqual(len(n2), 1)
        # Empty user_args → []; empty available → pass through; mixed → one notice
        v, n = self._validate_launch_args([], ["use_sim_time"])
        self.assertEqual(v, [])
        self.assertEqual(n, [])
        v3, n3 = self._validate_launch_args(["use_sim_time:=true", "robot_name:=r1"], [])
        self.assertEqual(v3, ["use_sim_time:=true", "robot_name:=r1"])
        self.assertEqual(n3, [])
        v4, n4 = self._validate_launch_args(
            ["use_sim_time:=true", "invented:=val"], ["use_sim_time", "robot_name"])
        self.assertIn("use_sim_time:=true", v4)
        self.assertIn("invented:=val", v4)
        self.assertEqual(len(n4), 1)


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

    def test_nan_to_none(self):
        """NaN → None; None/strings/regular numerics/inf pass through unchanged."""
        self.assertIsNone(self._nan_to_none(float("nan")))
        self.assertIsNone(self._nan_to_none(None))
        self.assertEqual(self._nan_to_none("ok"), "ok")
        for val in [3.14, 0.0, -1.5, 42, float("inf"), float("-inf")]:
            with self.subTest(val=val):
                self.assertEqual(self._nan_to_none(val), val)


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

    def test_bag_info_edge_cases(self):
        """Duration variants (int, flat, zero); files list; .db3 path resolves to metadata."""
        self._skip_if_no_yaml()
        # Legacy integer nanoseconds
        r = self._write_and_run({"rosbag2_bagfile_information": {
            "duration": 3_000_000_000,
            "starting_time": {"nanoseconds_since_epoch": 0},
            "storage_identifier": "sqlite3",
            "message_count": 10,
            "topics_with_message_count": [],
        }})
        self.assertNotIn("error", r)
        self.assertAlmostEqual(r["duration"]["seconds"], 3.0)
        # Flat format (no wrapper key)
        r2 = self._write_and_run({
            "duration": {"nanoseconds": 1_000_000_000},
            "starting_time": {"nanoseconds_since_epoch": 0},
            "storage_identifier": "sqlite3",
            "message_count": 5,
            "topics_with_message_count": [],
        })
        self.assertNotIn("error", r2)
        self.assertEqual(r2["storage_identifier"], "sqlite3")
        # Zero duration
        r3 = self._write_and_run({"rosbag2_bagfile_information": {
            "duration": {"nanoseconds": 0},
            "starting_time": {"nanoseconds_since_epoch": 0},
            "storage_identifier": "sqlite3",
            "message_count": 1,
            "topics_with_message_count": [],
        }})
        self.assertNotIn("error", r3)
        self.assertEqual(r3["duration"]["seconds"], 0.0)
        # Files list appears in output
        r4 = self._write_and_run({"rosbag2_bagfile_information": {
            "duration": {"nanoseconds": 1_000_000_000},
            "starting_time": {"nanoseconds_since_epoch": 0},
            "storage_identifier": "sqlite3",
            "message_count": 0,
            "topics_with_message_count": [],
            "files": [{"path": "my_bag_0.db3"}, {"path": "my_bag_1.db3"}],
        }})
        self.assertIn("files", r4)
        self.assertEqual(len(r4["files"]), 2)
        self.assertIn("my_bag_0.db3", r4["files"])
        # .db3 path resolves to parent metadata.yaml
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
            db3_path = pathlib.Path(tmpdir) / "bag_0.db3"
            db3_path.touch()
            captured = []
            with patch("ros2_bag.output", side_effect=captured.append):
                self.ros2_bag.cmd_bag_info(types.SimpleNamespace(bag_path=str(db3_path)))
        r5 = captured[0]
        self.assertNotIn("error", r5)
        self.assertEqual(r5["storage_identifier"], "sqlite3")


class TestArgumentIntrospectionExtended(unittest.TestCase):
    """Extended argument introspection: required positional enforcement and
    real-world parse acceptance for additional DISPATCH commands.
    (Help-exits-zero is an argparse guarantee and is not re-tested here.)
    """

    @classmethod
    def setUpClass(cls):
        if not check_rclpy_available():
            raise unittest.SkipTest("rclpy not available - requires ROS 2 environment")
        import ros2_cli
        cls.ros2_cli = ros2_cli
        cls.parser = ros2_cli.build_parser()

    def _fails(self, *args):
        """Assert that parse_args exits non-zero (missing required positional)."""
        with patch("sys.stderr", new_callable=StringIO), \
             self.assertRaises(SystemExit) as cm:
            self.parser.parse_args(list(args))
        self.assertNotEqual(cm.exception.code, 0)

    def test_argument_introspection_extended(self):
        """publish/publish-until parse; msg defaults None; required positionals enforced."""
        p = self.parser
        ns = p.parse_args(["topics", "publish", "/cmd_vel", '{"linear":{"x":1.0}}'])
        self.assertEqual(ns.topic, "/cmd_vel")
        self.assertIn("linear", ns.msg)
        self.assertIsNone(p.parse_args(["topics", "publish", "/cmd_vel"]).msg)
        ns2 = p.parse_args([
            "topics", "publish-until",
            "/cmd_vel", '{"linear":{"x":0.3}}',
            "--monitor", "/odom", "--delta", "1.0",
        ])
        self.assertEqual(ns2.topic, "/cmd_vel")
        self.assertEqual(ns2.monitor, "/odom")
        for cmd in [
            ("nodes",     "details"),
            ("services",  "call"),
            ("actions",   "send"),
            ("params",    "get"),
            ("params",    "set"),
            ("lifecycle", "set"),
            ("tf",        "lookup"),
            ("tf",        "lookup", "base_link"),
            ("lifecycle", "set", "/my_node"),
        ]:
            with self.subTest(cmd=cmd):
                self._fails(*cmd)


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

    def test_domain_id_default_and_env(self):
        """ROS_DOMAIN_ID absent → 0; explicitly set → integer value."""
        env_backup = os.environ.pop("ROS_DOMAIN_ID", None)
        try:
            self.assertEqual(self.ros2_daemon._get_domain_id(), 0)
        finally:
            if env_backup is not None:
                os.environ["ROS_DOMAIN_ID"] = env_backup
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

    def test_daemon_commands(self):
        """status running/not_running/exception; start/stop success+domain_id+error+exception."""
        import types
        result = self._run_cmd(
            self.ros2_daemon.cmd_daemon_status,
            self.CP(args=[], returncode=0, stdout="The daemon is running\n", stderr=""))
        self.assertEqual(result["status"], "running")
        self.assertIn("domain_id", result)
        result2 = self._run_cmd(
            self.ros2_daemon.cmd_daemon_status,
            self.CP(args=[], returncode=0, stdout="The daemon is not running\n", stderr=""))
        self.assertEqual(result2["status"], "not_running")
        captured = []
        with patch("ros2_daemon.subprocess.run",
                   side_effect=FileNotFoundError("ros2 not found")), \
             patch("ros2_daemon.output", side_effect=captured.append):
            self.ros2_daemon.cmd_daemon_status(types.SimpleNamespace())
        self.assertIn("error", captured[0])
        r = self._run_cmd(self.ros2_daemon.cmd_daemon_start,
                          self.CP(args=[], returncode=0, stdout="Starting\n", stderr=""))
        self.assertEqual(r["status"], "started")
        self.assertIn("domain_id", r)
        r2 = self._run_cmd(self.ros2_daemon.cmd_daemon_start,
                           self.CP(args=[], returncode=1, stdout="", stderr="Failed to start"))
        self.assertEqual(r2["status"], "error")
        self.assertIn("detail", r2)
        with patch.dict(os.environ, {"ROS_DOMAIN_ID": "3"}):
            r3 = self._run_cmd(self.ros2_daemon.cmd_daemon_start,
                               self.CP(args=[], returncode=0, stdout="ok", stderr=""))
        self.assertEqual(r3["domain_id"], 3)
        captured2 = []
        with patch.object(self.ros2_daemon, "_get_domain_id",
                          side_effect=RuntimeError("boom")), \
             patch("ros2_daemon.output", side_effect=captured2.append):
            self.ros2_daemon.cmd_daemon_start(types.SimpleNamespace())
        self.assertIn("error", captured2[0])
        r4 = self._run_cmd(self.ros2_daemon.cmd_daemon_stop,
                           self.CP(args=[], returncode=0, stdout="Stopping\n", stderr=""))
        self.assertEqual(r4["status"], "stopped")
        r5 = self._run_cmd(self.ros2_daemon.cmd_daemon_stop,
                           self.CP(args=[], returncode=1, stdout="", stderr="stop failed"))
        self.assertEqual(r5["status"], "error")
        with patch.dict(os.environ, {"ROS_DOMAIN_ID": "5"}):
            r6 = self._run_cmd(self.ros2_daemon.cmd_daemon_stop,
                               self.CP(args=[], returncode=0, stdout="ok", stderr=""))
        self.assertEqual(r6["domain_id"], 5)


class TestDaemonArgIntrospection(unittest.TestCase):
    """Parser registration tests for the daemon subcommands."""

    @classmethod
    def setUpClass(cls):
        if not check_rclpy_available():
            raise unittest.SkipTest("rclpy not available - requires ROS 2 environment")
        import ros2_cli
        cls.parser = ros2_cli.build_parser()
        cls.ros2_cli = ros2_cli

    def test_daemon_subcommands_registered(self):
        """status/start/stop all parse to correct subcommand and are in DISPATCH."""
        D = self.ros2_cli.DISPATCH
        for sub in ("status", "start", "stop"):
            with self.subTest(sub=sub):
                args = self.parser.parse_args(["daemon", sub])
                self.assertEqual(args.subcommand, sub)
                self.assertIn(("daemon", sub), D)


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

    def test_slow_last_and_factor_flags(self):
        """Defaults: slow_last=None, slow_factor=0.25; --slow-last/--slow-factor accept floats, ints, and zero."""
        args = self.parser.parse_args(self.BASE)
        self.assertIsNone(args.slow_last)
        self.assertAlmostEqual(args.slow_factor, 0.25)
        p = self.parser
        self.assertAlmostEqual(p.parse_args(self.BASE + ["--slow-last", "0.5"]).slow_last, 0.5)
        self.assertAlmostEqual(p.parse_args(self.BASE + ["--slow-last", "2"]).slow_last, 2.0)
        self.assertAlmostEqual(
            p.parse_args(self.BASE + ["--slow-last", "1.0", "--slow-factor", "0.1"]).slow_factor, 0.1)
        self.assertAlmostEqual(
            p.parse_args(self.BASE + ["--slow-last", "1.0", "--slow-factor", "0.0"]).slow_factor, 0.0)


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

    def test_scale_twist_velocity(self):
        """Plain Twist: scale halves/zeroes/preserves-negatives, no mutation; TwistStamped: nested scaled, no top-level leak."""
        base = {"linear": {"x": 1.0, "y": 0.5, "z": 0.0}, "angular": {"x": 0.0, "y": 0.0, "z": 0.5}}
        r = self.fn(base, 0.5)
        self.assertAlmostEqual(r["linear"]["x"], 0.5)
        self.assertAlmostEqual(r["linear"]["y"], 0.25)
        self.assertAlmostEqual(r["angular"]["z"], 0.25)
        r0 = self.fn({"linear": {"x": 2.0, "y": 1.0, "z": 0.0}, "angular": {"x": 0.0, "y": 0.0, "z": -1.0}}, 0.0)
        self.assertAlmostEqual(r0["linear"]["x"], 0.0)
        self.assertAlmostEqual(r0["angular"]["z"], 0.0)
        rn = self.fn({"linear": {"x": -1.0, "y": 0.0, "z": 0.0}, "angular": {"x": 0.0, "y": 0.0, "z": 0.0}}, 0.5)
        self.assertAlmostEqual(rn["linear"]["x"], -0.5)
        orig = {"linear": {"x": 1.0, "y": 0.0, "z": 0.0}, "angular": {"x": 0.0, "y": 0.0, "z": 0.5}}
        _ = self.fn(orig, 0.0)
        self.assertAlmostEqual(orig["linear"]["x"], 1.0)
        self.assertAlmostEqual(orig["angular"]["z"], 0.5)
        # TwistStamped: nested twist scaled; no top-level linear/angular leaked; partial payload OK
        data = {"header": {"frame_id": "base_link"},
                "twist": {"linear": {"x": 1.0, "y": 0.0, "z": 0.0},
                          "angular": {"x": 0.0, "y": 0.0, "z": 0.5}}}
        r2 = self.fn(data, 0.5)
        self.assertAlmostEqual(r2["twist"]["linear"]["x"], 0.5)
        self.assertAlmostEqual(r2["twist"]["angular"]["z"], 0.25)
        r2_0 = self.fn(data, 0.0)
        self.assertNotIn("linear", r2_0)
        self.assertNotIn("angular", r2_0)
        self.assertAlmostEqual(r2_0["twist"]["linear"]["x"], 0.0)
        partial = {"linear": {"x": 1.0}, "angular": {}}
        pr = self.fn(partial, 0.5)
        self.assertAlmostEqual(pr["linear"]["x"], 0.5)
        self.assertEqual(pr["angular"], {})


# ---------------------------------------------------------------------------
# H1b — Unit tests for _has_velocity_fields (no rclpy required)
# ---------------------------------------------------------------------------

class TestHasVelocityFields(unittest.TestCase):
    """Unit tests for _has_velocity_fields — gates zero-burst in publish-sequence."""

    @classmethod
    def setUpClass(cls):
        if not check_rclpy_available():
            raise unittest.SkipTest("rclpy not available - requires ROS 2 environment")
        import ros2_topic
        cls.fn = staticmethod(ros2_topic._has_velocity_fields)

    def test_velocity_detection(self):
        """Twist/angular-only/TwistStamped → True; non-velocity dicts, non-dicts → False."""
        self.assertTrue(self.fn({"linear": {"x": 0.5, "y": 0.0, "z": 0.0},
                                 "angular": {"x": 0.0, "y": 0.0, "z": 0.3}}))
        self.assertTrue(self.fn({"linear": {"x": 1.0}}))
        self.assertTrue(self.fn({"angular": {"z": 0.5}}))
        self.assertTrue(self.fn({"header": {"frame_id": "base_link"},
                                 "twist": {"linear": {"x": 0.5}, "angular": {"z": 0.3}}}))
        for bad in [{"data": "hello"}, {"position": {"x": 1.0}, "orientation": {}}, {},
                    {"twist": {"frame_id": "base_link"}}, None, "twist", 42]:
            with self.subTest(val=bad):
                self.assertFalse(self.fn(bad))


# ---------------------------------------------------------------------------
# RH-6 — _clamp_velocity helper tests (no rclpy required)
# ---------------------------------------------------------------------------

class TestClampVelocity(unittest.TestCase):
    """Unit tests for _clamp_velocity — velocity limit clamping for publish commands."""

    @classmethod
    def setUpClass(cls):
        import sys, os
        sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'scripts'))
        import ros2_topic
        cls.fn = staticmethod(ros2_topic._clamp_velocity)

    # ------------------------------------------------------------------
    # Pass-through cases
    # ------------------------------------------------------------------

    def test_clamp_velocity(self):
        """No limits/non-vel/within-limits→passthrough; linear/angular/combined clamped; TwistStamped nested; notice format."""
        # Passthrough cases
        d = {"linear": {"x": 2.0, "y": 0.0, "z": 0.0}, "angular": {"x": 0.0, "y": 0.0, "z": 5.0}}
        out, notices = self.fn(d, None, None)
        self.assertEqual(out, d)
        self.assertEqual(notices, [])
        out2, n2 = self.fn({"data": "hello"}, max_linear=1.0, max_ang=1.0)
        self.assertEqual(out2, {"data": "hello"})
        self.assertEqual(n2, [])
        wd = {"linear": {"x": 0.3, "y": 0.0, "z": 0.0}, "angular": {"x": 0.0, "y": 0.0, "z": 0.2}}
        out3, n3 = self.fn(wd, max_linear=0.5, max_ang=1.0)
        self.assertEqual(out3["linear"]["x"], 0.3)
        self.assertEqual(n3, [])
        big = {"linear": {"x": 2.0, "y": 0.0, "z": 0.0}, "angular": {"x": 0.0, "y": 0.0, "z": 0.0}}
        self.fn(big, max_linear=0.5, max_ang=None)
        self.assertEqual(big["linear"]["x"], 2.0)
        # Linear clamped symmetrically; notice format
        for raw, expected in [(2.0, 0.5), (-2.0, -0.5)]:
            with self.subTest(raw=raw):
                data = {"linear": {"x": raw, "y": 0.0, "z": 0.0},
                        "angular": {"x": 0.0, "y": 0.0, "z": 0.0}}
                out, notices = self.fn(data, max_linear=0.5, max_ang=None)
                self.assertAlmostEqual(out["linear"]["x"], expected)
                self.assertTrue(any("linear.x" in n for n in notices))
        _, notices_nf = self.fn({"linear": {"x": 2.5, "y": 0.0, "z": 0.0},
                                 "angular": {"x": 0.0, "y": 0.0, "z": 0.0}},
                                max_linear=1.0, max_ang=None)
        self.assertEqual(len(notices_nf), 1)
        self.assertIn("2.5", notices_nf[0])
        self.assertIn("1.0", notices_nf[0])
        self.assertIn("±1.0", notices_nf[0])
        # Angular clamped; combined
        out_a, notices_a = self.fn({"linear": {"x": 0.0, "y": 0.0, "z": 0.0},
                                    "angular": {"x": 0.0, "y": 0.0, "z": 3.0}},
                                   max_linear=None, max_ang=1.0)
        self.assertAlmostEqual(out_a["angular"]["z"], 1.0)
        self.assertTrue(any("angular.z" in n for n in notices_a))
        out2c, n2c = self.fn({"linear": {"x": 2.0, "y": 0.0, "z": 0.0},
                               "angular": {"x": 0.0, "y": 0.0, "z": 4.0}},
                              max_linear=0.5, max_ang=1.0)
        self.assertAlmostEqual(out2c["linear"]["x"],  0.5)
        self.assertAlmostEqual(out2c["angular"]["z"], 1.0)
        self.assertEqual(len(n2c), 2)
        # TwistStamped: nested clamped, header preserved
        ts = {"header": {"frame_id": "odom", "stamp": {"sec": 1, "nanosec": 0}},
              "twist": {"linear": {"x": 3.0, "y": 0.0, "z": 0.0},
                        "angular": {"x": 0.0, "y": 0.0, "z": 5.0}}}
        out_ts, n_ts = self.fn(ts, max_linear=1.0, max_ang=0.5)
        self.assertAlmostEqual(out_ts["twist"]["linear"]["x"],  1.0)
        self.assertAlmostEqual(out_ts["twist"]["angular"]["z"], 0.5)
        self.assertEqual(out_ts["header"]["frame_id"], "odom")
        self.assertEqual(out_ts["header"]["stamp"]["sec"], 1)
        self.assertTrue(any("twist.linear.x"  in n for n in n_ts))
        self.assertTrue(any("twist.angular.z" in n for n in n_ts))


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

    def test_ramp_boundary_floor_and_larger_zone(self):
        """Boundary=1.0, midpoint=0.5, floor clamps, zero remaining, larger zone; never below slow_factor."""
        self.assertAlmostEqual(self._scale(1.0, 1.0, 0.25), 1.0)   # boundary → 1.0
        self.assertAlmostEqual(self._scale(0.5, 1.0, 0.25), 0.5)   # midpoint → 0.5
        self.assertAlmostEqual(self._scale(0.01, 1.0, 0.25), 0.25)  # near zero → floor
        self.assertAlmostEqual(self._scale(0.0, 1.0, 0.25), 0.25)   # zero → slow_factor
        self.assertAlmostEqual(self._scale(0.5, 1.0, 0.0), 0.5)     # sf=0 ramp at mid
        self.assertAlmostEqual(self._scale(0.0, 1.0, 0.0), 0.0)     # sf=0 ramp to zero
        self.assertAlmostEqual(self._scale(1.0, 2.0, 0.25), 0.5)    # larger zone
        for remaining in [0.0, 0.01, 0.1, 0.2, 0.5, 1.0]:
            with self.subTest(remaining=remaining):
                self.assertGreaterEqual(self._scale(remaining, 1.0, 0.25), 0.25)


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

    def test_preset_list_and_delete(self):
        """List: absent dir → empty; present → JSON only. Delete: missing → error; existing → deleted."""
        import types
        with patch.object(self.mod.os.path, "isdir", return_value=False), self._stdout() as buf:
            self.mod.cmd_params_preset_list(types.SimpleNamespace())
            r = json.loads(buf.getvalue())
        self.assertEqual(r["presets"], [])
        self.assertEqual(r["count"], 0)
        with patch.object(self.mod.os.path, "isdir", return_value=True), \
             patch.object(self.mod.os, "listdir",
                          return_value=["alpha.json", "beta.json", "readme.txt", "config.yaml"]), \
             self._stdout() as buf:
            self.mod.cmd_params_preset_list(types.SimpleNamespace())
            r2 = json.loads(buf.getvalue())
        self.assertEqual(r2["count"], 2)
        self.assertEqual({p["preset"] for p in r2["presets"]}, {"alpha", "beta"})
        # Delete: missing preset → error; existing → deleted, path ends with .json
        with patch.object(self.mod, "_presets_base", return_value="/fake/presets"), \
             patch.object(self.mod.os.path, "exists", return_value=False), self._stdout() as buf:
            self.mod.cmd_params_preset_delete(types.SimpleNamespace(preset="ghost"))
            r3 = json.loads(buf.getvalue())
        self.assertIn("error", r3)
        self.assertIn("ghost", r3["error"])
        removed = []
        with patch.object(self.mod, "_presets_base", return_value="/fake/presets"), \
             patch.object(self.mod.os.path, "exists", return_value=True), \
             patch.object(self.mod.os, "remove", side_effect=lambda p: removed.append(p)), \
             self._stdout() as buf:
            self.mod.cmd_params_preset_delete(types.SimpleNamespace(preset="cam_calib"))
            r4 = json.loads(buf.getvalue())
        self.assertTrue(r4.get("deleted"))
        self.assertEqual(r4.get("preset"), "cam_calib")
        self.assertEqual(len(removed), 1)
        self.assertTrue(removed[0].endswith("cam_calib.json"))


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

    def test_tmux_error_and_empty_sessions(self):
        """No tmux → error from launch and run; empty sessions → empty lists from both."""
        import types
        with patch.object(self.launch, "check_tmux", return_value=False), self._stdout() as buf:
            self.launch.cmd_launch_run(types.SimpleNamespace(
                package="pkg", launch_file="nav.launch.py", args=[], session=None))
            r = json.loads(buf.getvalue())
        self.assertIn("error", r)
        self.assertIn("tmux", r["error"].lower())
        with patch.object(self.launch, "list_sessions",
                          return_value={"error": "tmux is not installed", "running_sessions": []}), \
             self._stdout() as buf:
            self.launch.cmd_launch_list(types.SimpleNamespace())
            self.assertIn("error", json.loads(buf.getvalue()))
        with patch.object(self.ros2_run_mod, "check_tmux", return_value=False), self._stdout() as buf:
            self.ros2_run_mod.cmd_run(types.SimpleNamespace(
                package="pkg", executable="node", args=[], session=None,
                presets=None, params=None, config_path=None))
            r2 = json.loads(buf.getvalue())
        self.assertIn("error", r2)
        self.assertIn("tmux", r2["error"].lower())
        # Empty sessions → empty lists
        with patch.object(self.launch, "list_sessions",
                          return_value={"launch_sessions": [], "running_sessions": []}), \
             self._stdout() as buf:
            self.launch.cmd_launch_list(types.SimpleNamespace())
            self.assertEqual(json.loads(buf.getvalue()).get("launch_sessions"), [])
        with patch.object(self.ros2_run_mod, "list_sessions",
                          return_value={"run_sessions": [], "running_sessions": []}), \
             self._stdout() as buf:
            self.ros2_run_mod.cmd_run_list(types.SimpleNamespace())
            self.assertEqual(json.loads(buf.getvalue()).get("run_sessions"), [])


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

    def test_condition_monitor_callback(self):
        """delta/above/below/equals operators; euclidean distance; rotation accumulation/missing quat."""
        import math as _math
        m = self._make_monitor(operator='delta', threshold=1.0, field='x')
        self._call(m, {'x': 0.0})
        self.assertAlmostEqual(float(m.start_value), 0.0)
        self._call(m, {'x': 0.5})
        self.assertFalse(m.stop_event.is_set())
        self._call(m, {'x': 1.5})
        self.assertTrue(m.stop_event.is_set())
        m2 = self._make_monitor(operator='delta', threshold=-1.0, field='x')
        self._call(m2, {'x': 0.0})
        self._call(m2, {'x': -1.5})
        self.assertTrue(m2.stop_event.is_set())
        m3 = self._make_monitor(operator='above', threshold=1.0, field='x')
        m3.stop_event.set()
        self._call(m3, {'x': 999.0})
        self.assertIsNone(m3.start_value)
        for op, below_val, trigger_val, threshold in [
            ('above', 3.0, 6.0, 5.0),
            ('below', 7.0, 3.0, 5.0),
            ('equals', 41.0, 42.0, 42.0),
        ]:
            with self.subTest(op=op):
                m = self._make_monitor(operator=op, threshold=threshold, field='x')
                self._call(m, {'x': below_val})
                self.assertFalse(m.stop_event.is_set())
                self._call(m, {'x': trigger_val})
                self.assertTrue(m.stop_event.is_set())
        # Euclidean mode
        me = self._make_monitor(euclidean=True, fields=['x', 'y'], threshold=1.0)
        self._call(me, {'x': 0.0, 'y': 0.0})
        self.assertEqual(me.start_values, [0.0, 0.0])
        self.assertFalse(me.stop_event.is_set())
        self._call(me, {'x': 1.0, 'y': 0.0})
        self.assertTrue(me.stop_event.is_set())
        self.assertAlmostEqual(me.euclidean_distance, 1.0)
        me2 = self._make_monitor(euclidean=True, fields=['x', 'y'], threshold=2.0)
        self._call(me2, {'x': 0.0, 'y': 0.0})
        self._call(me2, {'x': 1.0, 'y': 0.0})
        self.assertFalse(me2.stop_event.is_set())
        me3 = self._make_monitor(euclidean=True, fields=['x', 'y'], threshold=1.4)
        self._call(me3, {'x': 0.0, 'y': 0.0})
        self._call(me3, {'x': 1.0, 'y': 1.0})
        self.assertTrue(me3.stop_event.is_set())
        self.assertAlmostEqual(me3.euclidean_distance, _math.sqrt(2), places=5)
        # Rotation mode
        identity = {'orientation': {'x': 0.0, 'y': 0.0, 'z': 0.0, 'w': 1.0}}
        def _quat(deg):
            a = _math.radians(deg)
            return {'orientation': {'x': 0.0, 'y': 0.0, 'z': _math.sin(a/2), 'w': _math.cos(a/2)}}
        mr = self._make_monitor(rotate=1.571)
        self._call(mr, identity)
        self.assertAlmostEqual(mr.start_yaw, 0.0)
        self.assertFalse(mr.stop_event.is_set())
        for sign, target in [(1, _math.pi/2), (-1, -_math.pi/2)]:
            ms = self._make_monitor(rotate=target)
            self._call(ms, identity)
            self._call(ms, _quat(sign * 100))
            self.assertTrue(ms.stop_event.is_set(), f"sign={sign}")
        m6 = self._make_monitor(rotate=_math.pi)
        self._call(m6, identity)
        s45, c45 = _math.sin(_math.pi / 4), _math.cos(_math.pi / 4)
        self._call(m6, {'orientation': {'x': 0.0, 'y': 0.0, 'z': s45, 'w': c45}})
        self.assertFalse(m6.stop_event.is_set())
        s90, c90 = _math.sin(_math.pi / 2), _math.cos(_math.pi / 2)
        self._call(m6, {'orientation': {'x': 0.0, 'y': 0.0, 'z': s90, 'w': c90}})
        self.assertTrue(m6.stop_event.is_set())
        m7 = self._make_monitor(rotate=_math.pi/2)
        self._call(m7, {})
        self.assertIsNotNone(m7.field_error)
        self.assertTrue(m7.stop_event.is_set())


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

    def test_sorted_and_excludes_non_lifecycle(self):
        """Results are sorted; non-lifecycle excluded; empty graph → []; bare /get_state excluded."""
        mn = MagicMock()
        mn.get_service_names_and_types.return_value = [
            ('/cam_node/get_state', ['lifecycle_msgs/srv/GetState']),
            ('/arm_node/get_state', ['lifecycle_msgs/srv/GetState']),
            ('/other/some_service', ['std_srvs/srv/Empty']),
        ]
        self.assertEqual(self.mod._get_managed_nodes(mn), ['/arm_node', '/cam_node'])
        mn.get_service_names_and_types.return_value = []
        self.assertEqual(self.mod._get_managed_nodes(mn), [])
        mn.get_service_names_and_types.return_value = [
            ('/get_state', ['lifecycle_msgs/srv/GetState'])]
        self.assertEqual(self.mod._get_managed_nodes(mn), [])


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

    def test_control_command_paths(self):
        """list/load success+failure; _call_cm_service unavailable→error+destroy; retries honoured."""
        with patch.object(self.mod, 'ros2_context', MagicMock()), \
             patch.object(self.mod, 'ROS2CLI', MagicMock()), \
             patch.object(self.mod, '_call_cm_service',
                          return_value=(None, {"error": "service not available"})), \
             self._stdout() as buf:
            self.mod.cmd_control_list_controllers(self._args())
            self.assertIn("error", json.loads(buf.getvalue()))
        mock_result = MagicMock()
        mock_result.controller = []
        with patch.object(self.mod, 'ros2_context', MagicMock()), \
             patch.object(self.mod, 'ROS2CLI', MagicMock()), \
             patch.object(self.mod, '_call_cm_service', return_value=(mock_result, None)), \
             self._stdout() as buf:
            self.mod.cmd_control_list_controllers(self._args())
            r = json.loads(buf.getvalue())
        self.assertIn("controllers", r)
        self.assertEqual(r["count"], 0)
        mock_ok = MagicMock()
        mock_ok.ok = True
        with patch.object(self.mod, 'ros2_context', MagicMock()), \
             patch.object(self.mod, 'ROS2CLI', MagicMock()), \
             patch.object(self.mod, '_call_cm_service', return_value=(mock_ok, None)), \
             self._stdout() as buf:
            self.mod.cmd_control_load_controller(self._args(name='joint_controller'))
            r2 = json.loads(buf.getvalue())
        self.assertEqual(r2.get("controller"), "joint_controller")
        self.assertTrue(r2.get("ok"))
        with patch.object(self.mod, 'ros2_context', MagicMock()), \
             patch.object(self.mod, 'ROS2CLI', MagicMock()), \
             patch.object(self.mod, '_call_cm_service',
                          return_value=(None, {"error": "timeout"})), \
             self._stdout() as buf:
            self.mod.cmd_control_load_controller(self._args(name='joint_controller'))
            self.assertIn("error", json.loads(buf.getvalue()))
        # _call_cm_service: unavailable → error+destroy; retries=3 → 3 wait calls
        mn = MagicMock()
        mc = MagicMock()
        mn.create_client.return_value = mc
        mc.wait_for_service.return_value = False
        result, err = self.mod._call_cm_service(
            mn, MagicMock(), '/cm', 'list_controllers', MagicMock(), 0.1, retries=1)
        self.assertIsNone(result)
        self.assertIn("error", err)
        mc.destroy.assert_called_once()
        mn2 = MagicMock()
        mc2 = MagicMock()
        mn2.create_client.return_value = mc2
        mc2.wait_for_service.return_value = False
        self.mod._call_cm_service(
            mn2, MagicMock(), '/cm', 'list_controllers', MagicMock(), 0.1, retries=3)
        self.assertEqual(mc2.wait_for_service.call_count, 3)


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

    def test_cmd_lifecycle_nodes_and_get(self):
        """Sorted nodes returned; empty graph→count=0; get with unavailable service→error+node name."""
        import types, sys
        mn = MagicMock()
        mn.get_service_names_and_types.return_value = [
            ('/cam_node/get_state', ['lifecycle_msgs/srv/GetState']),
            ('/arm_node/get_state', ['lifecycle_msgs/srv/GetState']),
        ]
        with patch.object(self.mod, 'ros2_context', MagicMock()), \
             patch.object(self.mod, 'ROS2CLI', return_value=mn), self._stdout() as buf:
            self.mod.cmd_lifecycle_nodes(types.SimpleNamespace())
            r = json.loads(buf.getvalue())
        self.assertEqual(r["managed_nodes"], ['/arm_node', '/cam_node'])
        self.assertEqual(r["count"], 2)
        mn.get_service_names_and_types.return_value = []
        with patch.object(self.mod, 'ros2_context', MagicMock()), \
             patch.object(self.mod, 'ROS2CLI', return_value=mn), self._stdout() as buf:
            self.mod.cmd_lifecycle_nodes(types.SimpleNamespace())
            r2 = json.loads(buf.getvalue())
        self.assertEqual(r2["managed_nodes"], [])
        self.assertEqual(r2["count"], 0)
        # get: service unavailable → error containing node name
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

    def test_estop_autodetect_twist_and_error(self):
        """cmd_vel preferred; fallback to first; Twist/TwistStamped both zeroed; no-vel-topic → error."""
        r1, _ = self._run_estop([('/base/vel', ['geometry_msgs/msg/Twist']),
                                  ('/cmd_vel', ['geometry_msgs/msg/Twist'])], msg_has_twist=False)
        self.assertEqual(r1['topic'], '/cmd_vel')
        r2, _ = self._run_estop([('/base/velocity', ['geometry_msgs/msg/Twist']),
                                  ('/robot/vel', ['geometry_msgs/msg/Twist'])], msg_has_twist=False)
        self.assertEqual(r2['topic'], '/base/velocity')
        # Twist: zeroes top-level; TwistStamped: zeroes nested
        _, msg_plain = self._run_estop([('/cmd_vel', ['geometry_msgs/msg/Twist'])],
                                       msg_has_twist=False)
        self.assertEqual(msg_plain.linear.x, 0.0)
        self.assertEqual(msg_plain.angular.z, 0.0)
        _, msg_stamped = self._run_estop([('/cmd_vel', ['geometry_msgs/msg/TwistStamped'])],
                                         msg_has_twist=True)
        self.assertEqual(msg_stamped.twist.linear.x, 0.0)
        self.assertEqual(msg_stamped.twist.angular.z, 0.0)
        # No velocity topic → error
        import types
        mock_node = MagicMock()
        mock_node.get_topic_names.return_value = [('/scan', ['sensor_msgs/msg/LaserScan'])]
        with patch.object(self.mod, 'ros2_context', MagicMock()), \
             patch.object(self.mod, 'ROS2CLI', return_value=mock_node), \
             patch('sys.stdout', new_callable=StringIO) as buf:
            self.mod.cmd_estop(types.SimpleNamespace(topic=None))
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

    def test_shutdown_always_called(self):
        """shutdown() called on normal exit, on RuntimeError, and exception propagates after cleanup."""
        with patch.object(self.mod.rclpy, 'init'), \
             patch.object(self.mod.rclpy, 'shutdown') as ms:
            with self.mod.ros2_context():
                pass
            ms.assert_called_once()
        with patch.object(self.mod.rclpy, 'init'), \
             patch.object(self.mod.rclpy, 'shutdown') as ms2:
            with self.assertRaises(RuntimeError):
                with self.mod.ros2_context():
                    raise RuntimeError("boom")
            ms2.assert_called_once()
        # Exception propagates after cleanup
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

    def test_qos_selection(self):
        """BEST_EFFORT pub → sensor_data QoS; RELIABLE or empty or exception → system_default QoS."""
        from rclpy.qos import ReliabilityPolicy
        be_pub = MagicMock()
        be_pub.qos_profile.reliability = ReliabilityPolicy.BEST_EFFORT
        self.assertIs(self._captured_qos([be_pub]), self.mod.qos_profile_sensor_data)
        rel_pub = MagicMock()
        rel_pub.qos_profile.reliability = ReliabilityPolicy.RELIABLE
        self.assertIs(self._captured_qos([rel_pub]), self.mod.qos_profile_system_default)
        self.assertIs(self._captured_qos([]), self.mod.qos_profile_system_default)
        # get_publishers_info_by_topic exception → fall back to system_default
        import threading
        captured = {}

        def mock_node_init_exc(self, name):
            self.get_publishers_info_by_topic = MagicMock(side_effect=Exception("rpc error"))
            self.create_subscription = MagicMock(
                side_effect=lambda cls, t, cb, qos: captured.update({'qos': qos})
            )

        stop_event = threading.Event()
        with patch.object(self.mod.Node, '__init__', mock_node_init_exc), \
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

    def test_autodiscovery_error(self):
        """Empty TF tree and all_frames exception both return an error key."""
        self.assertIn('error', self._run(frames_yaml=''))
        self.assertIn('error', self._run(frames_raise=Exception('tf2 unavailable')))


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

    def test_positional_and_node_flag(self):
        """Pattern positional required; node optional (default None); --node accepted; dispatch wired."""
        p = self.parser
        a = p.parse_args(["params", "find", "velocity"])
        self.assertEqual(a.pattern, "velocity")
        self.assertEqual(a.subcommand, "find")
        self.assertIsNone(a.node)
        self.assertEqual(
            p.parse_args(["params", "find", "velocity", "--node", "/controller"]).node,
            "/controller")
        self.assertEqual(p.parse_args(["params", "find", "all"]).pattern, "all")
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

    def test_tf_tree_and_validate_parsing(self):
        """tree and validate subcommands parse; duration defaults to 2.0, custom accepted; dispatch wired."""
        p = self.parser
        self.assertEqual(p.parse_args(["tf", "tree"]).subcommand, "tree")
        self.assertAlmostEqual(p.parse_args(["tf", "tree"]).duration, 2.0)
        self.assertAlmostEqual(p.parse_args(["tf", "tree", "--duration", "5"]).duration, 5.0)
        self.assertEqual(p.parse_args(["tf", "validate"]).subcommand, "validate")
        self.assertAlmostEqual(p.parse_args(["tf", "validate"]).duration, 2.0)
        self.assertAlmostEqual(p.parse_args(["tf", "validate", "-d", "3"]).duration, 3.0)
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

    def test_positional_and_timeout(self):
        """Topic positional required; timeout defaults to 5.0; custom accepted; dispatch wired."""
        a = self.parser.parse_args(["topics", "qos-check", "/odom"])
        self.assertEqual(a.topic, "/odom")
        self.assertEqual(a.subcommand, "qos-check")
        self.assertAlmostEqual(a.timeout, 5.0)
        self.assertAlmostEqual(
            self.parser.parse_args(["topics", "qos-check", "/odom", "--timeout", "10"]).timeout, 10.0)
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

    def test_apply_params(self):
        """None/empty/no-separator→{}; walrus→typed; colon→string; whitespace stripped; multi-entry."""
        self.assertEqual(self.fn(None), {})
        self.assertEqual(self.fn(""), {})
        self.assertEqual(self.fn("badtoken"), {})
        self.assertAlmostEqual(self.fn("key:=1.5")["key"], 1.5)
        r = self.fn("key:=42")
        self.assertEqual(r["key"], 42)
        self.assertIsInstance(r["key"], int)
        self.assertEqual(self.fn("key:=hello")["key"], "hello")
        self.assertEqual(self.fn("key:value")["key"], "value")
        self.assertIn("key", self.fn(" key :=1"))
        r2 = self.fn("k1:=a,k2:=b")
        self.assertEqual(r2["k1"], "a")
        self.assertEqual(r2["k2"], "b")
        r3 = self.fn("speed:=0.5,label:=robot,count:=3")
        self.assertAlmostEqual(r3["speed"], 0.5)
        self.assertEqual(r3["label"], "robot")
        self.assertEqual(r3["count"], 3)


# ---------------------------------------------------------------------------
# Robot type detection unit tests (ros2_profile helpers)
# ---------------------------------------------------------------------------

class TestSensorMountExtraction(unittest.TestCase):
    """_extract_sensor_mounts_from_urdf parses URDF for all sensor/actuator origins."""

    @classmethod
    def setUpClass(cls):
        import sys, pathlib, types
        scripts = str(pathlib.Path(__file__).parent.parent / "scripts")
        if scripts not in sys.path:
            sys.path.insert(0, scripts)
        if "ros2_utils" not in sys.modules:
            stub = types.ModuleType("ros2_utils")
            stub.output = lambda d: None
            sys.modules["ros2_utils"] = stub
        import ros2_profile
        cls.fn = staticmethod(ros2_profile._extract_sensor_mounts_from_urdf)

    def _write_urdf(self, tmpdir, content):
        import pathlib
        p = pathlib.Path(tmpdir) / "test.urdf"
        p.write_text(content)
        return str(p)

    def test_sensor_mount_extraction(self):
        """Camera roll→image_rotation_deg; lidar sensor_type; wheel/empty/missing→[]; two joints→2 results."""
        import tempfile
        for rpy_roll, expected_deg, joint_name, link_name in [
            ("3.14159", 180, "camera_joint", "camera_link"),
            ("0", 0, "camera_joint", "camera_link"),
            ("1.5708", 90, "cam_joint", "cam_link"),
        ]:
            with self.subTest(roll=rpy_roll):
                urdf = f"""<?xml version="1.0"?>
<robot name="test">
  <link name="base_link"/>
  <link name="{link_name}"/>
  <joint name="{joint_name}" type="fixed">
    <parent link="base_link"/>
    <child link="{link_name}"/>
    <origin xyz="0 0 0" rpy="{rpy_roll} 0 0"/>
  </joint>
</robot>"""
                with tempfile.TemporaryDirectory() as d:
                    mounts = self.fn(self._write_urdf(d, urdf))
                self.assertEqual(len(mounts), 1)
                self.assertEqual(mounts[0]["image_rotation_deg"], expected_deg)
        with tempfile.TemporaryDirectory() as d:
            mounts = self.fn(self._write_urdf(d, """<?xml version="1.0"?>
<robot name="test">
  <link name="base_link"/>
  <link name="lidar_link"/>
  <joint name="lidar_joint" type="fixed">
    <parent link="base_link"/>
    <child link="lidar_link"/>
    <origin xyz="0 0 0.3" rpy="3.14159 0 0"/>
  </joint>
</robot>"""))
        self.assertEqual(len(mounts), 1)
        self.assertEqual(mounts[0]["sensor_type"], "lidar")
        self.assertNotIn("image_rotation_deg", mounts[0])
        with tempfile.TemporaryDirectory() as d:
            self.assertEqual(self.fn(self._write_urdf(d, """<?xml version="1.0"?>
<robot name="test">
  <link name="base_link"/>
  <link name="wheel_left_link"/>
  <joint name="wheel_left_joint" type="continuous">
    <parent link="base_link"/>
    <child link="wheel_left_link"/>
    <origin xyz="-0.2 0.15 0" rpy="0 0 0"/>
  </joint>
</robot>""")), [], "Wheel joint should not be detected as a sensor")
            self.assertEqual(
                self.fn(self._write_urdf(d, '<?xml version="1.0"?><robot name="test"></robot>')),
                [])
        self.assertEqual(self.fn("/nonexistent/path/robot.urdf"), [])
        with tempfile.TemporaryDirectory() as d:
            mounts = self.fn(self._write_urdf(d, """<?xml version="1.0"?>
<robot name="test">
  <link name="base_link"/>
  <link name="camera_link"/>
  <joint name="cam_j1" type="fixed">
    <parent link="base_link"/>
    <child link="camera_link"/>
    <origin xyz="0 0 0" rpy="3.14159 0 0"/>
  </joint>
  <joint name="cam_j2" type="fixed">
    <parent link="base_link"/>
    <child link="camera_link"/>
    <origin xyz="0 0 0" rpy="0 0 0"/>
  </joint>
</robot>"""))
        self.assertEqual(len(mounts), 2)


class TestLoadProfileSummary(unittest.TestCase):
    """load_profile_summary() silently returns summary dict or None."""

    @classmethod
    def setUpClass(cls):
        import sys, pathlib, types
        scripts = str(pathlib.Path(__file__).parent.parent / "scripts")
        if scripts not in sys.path:
            sys.path.insert(0, scripts)
        if "ros2_utils" not in sys.modules:
            stub = types.ModuleType("ros2_utils")
            stub.output = lambda d: None
            sys.modules["ros2_utils"] = stub
        import ros2_profile
        cls.mod = ros2_profile

    def test_load_profile_summary(self):
        """Non-existent/corrupt → None; valid profile returns summary dict with robot_type."""
        import pathlib, tempfile, json
        orig = self.mod._PROFILES_DIR
        try:
            self.mod._PROFILES_DIR = pathlib.Path("/nonexistent/__profiles_test__")
            self.assertIsNone(self.mod.load_profile_summary())
            with tempfile.TemporaryDirectory() as d:
                self.mod._PROFILES_DIR = pathlib.Path(d)
                (pathlib.Path(d) / "bad_profile.json").write_text("{not valid json")
                self.assertIsNone(self.mod.load_profile_summary())
                # Valid profile
                profile = {
                    "schema_version": 1,
                    "robot_name": "test_robot",
                    "summary": {"robot_type": "mobile_base", "sensor_mounts": []},
                    "detail": {},
                }
                (pathlib.Path(d) / "test_robot_profile.json").write_text(json.dumps(profile))
                result = self.mod.load_profile_summary()
            self.assertIsNotNone(result)
            self.assertEqual(result["robot_type"], "mobile_base")
        finally:
            self.mod._PROFILES_DIR = orig


class TestProfileAnnotate(unittest.TestCase):
    """cmd_profile_annotate appends free-text notes to the robot profile."""

    @classmethod
    def setUpClass(cls):
        import sys, pathlib, types
        scripts = str(pathlib.Path(__file__).parent.parent / "scripts")
        if scripts not in sys.path:
            sys.path.insert(0, scripts)
        if "ros2_utils" not in sys.modules:
            stub = types.ModuleType("ros2_utils")
            stub.output = lambda d: None
            sys.modules["ros2_utils"] = stub
        import ros2_profile
        cls.mod = ros2_profile

    def _make_profile(self, tmp_dir, robot_name="mybot"):
        """Write a minimal profile JSON to tmp_dir and return the path."""
        import json, pathlib
        profile = {
            "schema_version": 1,
            "robot_name": robot_name,
            "generated_at": "2026-01-01T00:00:00+00:00",
            "workspace": "",
            "ros_distro": "humble",
            "summary": {"robot_type": "mobile_base", "sensor_mounts": []},
            "detail": {},
        }
        p = pathlib.Path(tmp_dir) / f"{robot_name}_profile.json"
        p.write_text(json.dumps(profile), encoding="utf-8")
        return p

    def _run_annotate(self, text, robot_name="mybot"):
        """Run cmd_profile_annotate with the given text and return collected output."""
        import argparse
        results = []
        orig_output = self.mod.output
        self.mod.output = results.append
        try:
            ns = argparse.Namespace(text=text, name=robot_name)
            self.mod.cmd_profile_annotate(ns)
        finally:
            self.mod.output = orig_output
        return results

    def test_profile_annotate(self):
        """Single note stored+reported; accumulation; empty/whitespace→error; missing profile→error; notes survive rescan."""
        import tempfile, pathlib, json
        orig_dir = self.mod._PROFILES_DIR
        try:
            with tempfile.TemporaryDirectory() as d:
                self.mod._PROFILES_DIR = pathlib.Path(d)
                self._make_profile(d)
                results = self._run_annotate("Left encoder is worn.", robot_name="mybot")
                self.assertEqual(len(results), 1)
                r = results[0]
                self.assertTrue(r.get("success"))
                self.assertEqual(r["total_annotations"], 1)
                self.assertEqual(r["annotation"]["note"], "Left encoder is worn.")
                self.assertIn("added_at", r["annotation"])
            with tempfile.TemporaryDirectory() as d:
                self.mod._PROFILES_DIR = pathlib.Path(d)
                self._make_profile(d)
                self._run_annotate("Note one.", robot_name="mybot")
                self._run_annotate("Note two.", robot_name="mybot")
                results3 = self._run_annotate("Note three.", robot_name="mybot")
                self.assertEqual(results3[0]["total_annotations"], 3)
                self.assertEqual(results3[0]["annotation_index"], 2)
                loaded = self.mod._load_profile("mybot")
                self.assertEqual([a["note"] for a in loaded["annotations"]],
                                 ["Note one.", "Note two.", "Note three."])
            with tempfile.TemporaryDirectory() as d:
                self.mod._PROFILES_DIR = pathlib.Path(d)
                self._make_profile(d)
                self.assertIn("error", self._run_annotate("   ", robot_name="mybot")[0])
            with tempfile.TemporaryDirectory() as d2:
                self.mod._PROFILES_DIR = pathlib.Path(d2)
                self.assertIn("error", self._run_annotate("Some note.", robot_name="ghost")[0])
            with tempfile.TemporaryDirectory() as d3:
                self.mod._PROFILES_DIR = pathlib.Path(d3)
                self._make_profile(d3)
                self._run_annotate("Sensor offset calibration needed.", robot_name="mybot")
                preserved = self.mod._load_profile("mybot").get("annotations", [])
                self.assertEqual(len(preserved), 1)
                fresh = {"schema_version": 1, "robot_name": "mybot",
                         "generated_at": "2026-05-12T00:00:00+00:00", "workspace": "",
                         "ros_distro": "humble",
                         "summary": {"robot_type": "mobile_base", "sensor_mounts": []},
                         "detail": {}}
                (pathlib.Path(d3) / "mybot_profile.json").write_text(
                    json.dumps(fresh), encoding="utf-8")
                refreshed = self.mod._load_profile("mybot")
                refreshed["annotations"] = preserved
                self.mod._save_profile(refreshed, name="mybot")
                final = self.mod._load_profile("mybot")
                self.assertEqual(len(final.get("annotations", [])), 1)
                self.assertEqual(final["annotations"][0]["note"],
                                 "Sensor offset calibration needed.")
        finally:
            self.mod._PROFILES_DIR = orig_dir


class TestNewProfileExtractors(unittest.TestCase):
    """Tests for the 14 new profile-extraction functions."""

    @classmethod
    def setUpClass(cls):
        import importlib
        cls.mod = importlib.import_module("ros2_profile")

    # ------------------------------------------------------------------ helpers

    def _write_yaml(self, d, content):
        """Write content to a temp file under directory d, return path string."""
        import tempfile, pathlib
        f = tempfile.NamedTemporaryFile(
            dir=d, suffix=".yaml", delete=False, mode="w", encoding="utf-8"
        )
        f.write(content)
        f.close()
        return f.name

    def _write_urdf(self, d, content):
        import tempfile
        f = tempfile.NamedTemporaryFile(
            dir=d, suffix=".urdf", delete=False, mode="w", encoding="utf-8"
        )
        f.write(content)
        f.close()
        return f.name

    def _write_launch(self, d, content, name="test.launch.py"):
        import tempfile, pathlib
        p = pathlib.Path(d) / name
        p.write_text(content, encoding="utf-8")
        return str(p)

    def test_ros2_control_hardware_sensors_and_navigation(self):
        """ros2_control drive_type/kinematics/odom; URDF hardware interfaces; LiDAR/camera; EKF/Nav2; absent→None."""
        import tempfile
        with tempfile.TemporaryDirectory() as d:
            self.assertEqual(
                self.mod._extract_ros2_control_config([self._write_yaml(d, """\
controller_manager:
  ros__parameters:
    update_rate: 50
    base_controller:
      type: omni_wheel_drive_controller/OmniWheelDriveController
""")])["drive_type"], "holonomic_omni")
        with tempfile.TemporaryDirectory() as d:
            r = self.mod._extract_ros2_control_config([self._write_yaml(d, """\
controller_manager:
  ros__parameters:
    update_rate: 30
    base_ctrl:
      type: diff_drive_controller/DiffDriveController
""")])
            self.assertEqual(r["drive_type"], "differential")
            self.assertEqual(r["controller_update_rate_hz"], 30)
        with tempfile.TemporaryDirectory() as d:
            r2 = self.mod._extract_ros2_control_config([self._write_yaml(d, """\
base_controller:
  ros__parameters:
    wheel_radius: 0.051
    odom_frame_id: odom
    base_frame_id: base_footprint
""")])
            self.assertAlmostEqual(r2["kinematics"]["wheel_radius"], 0.051, places=4)
            self.assertEqual(r2["odom_frame_ids"]["odom_frame_id"], "odom")
        with tempfile.TemporaryDirectory() as d:
            r3 = self.mod._extract_ros2_control_config(
                [self._write_yaml(d, "some_key: 42\n")])
            self.assertIsNone(r3["drive_type"])
            self.assertIsNone(r3["kinematics"])
        # Hardware interfaces from URDF
        urdf = """\
<?xml version="1.0"?>
<robot name="test_robot">
  <ros2_control name="my_base" type="system">
    <hardware>
      <plugin>my_pkg/MyHW</plugin>
      <param name="serial_port">/dev/ttyUSB0</param>
    </hardware>
    <joint name="left_wheel_joint">
      <command_interface name="velocity"/>
      <state_interface name="position"/>
    </joint>
    <joint name="right_wheel_joint">
      <command_interface name="velocity"/>
    </joint>
  </ros2_control>
</robot>
"""
        with tempfile.TemporaryDirectory() as d:
            hw = self.mod._extract_hardware_interfaces_from_urdf(self._write_urdf(d, urdf))
            self.assertEqual(len(hw), 1)
            self.assertEqual(hw[0]["name"], "my_base")
            self.assertEqual(hw[0]["plugin"], "my_pkg/MyHW")
            self.assertIn("left_wheel_joint", hw[0]["joints"])
            self.assertIn("velocity", hw[0]["command_interfaces"])
            self.assertEqual(hw[0]["hardware_params"]["serial_port"], "/dev/ttyUSB0")
            empty = self.mod._extract_hardware_interfaces_from_urdf(
                self._write_urdf(d, '<robot name="r"><link name="base_link"/></robot>'))
            self.assertEqual(empty, [])

        import pathlib
        with tempfile.TemporaryDirectory() as d:
            # LiDAR by key presence
            yf = self._write_yaml(d, """\
laser_node:
  ros__parameters:
    product_name: LDLiDAR_LD19
    range_min: 0.02
    range_max: 12.0
""")
            result = self.mod._extract_lidar_config([yf])
            self.assertEqual(len(result), 1)
            self.assertEqual(result[0]["product_name"], "LDLiDAR_LD19")
            # LiDAR by filename
            p = pathlib.Path(d) / "laser_filter_k2.yaml"
            p.write_text("some_key: value\n", encoding="utf-8")
            result2 = self.mod._extract_lidar_config([str(p)])
            self.assertEqual(result2[0]["config_file"], "laser_filter_k2.yaml")
            # Non-lidar YAML → []
            self.assertEqual(self.mod._extract_lidar_config(
                [self._write_yaml(d, "robot_name: mybot\nversion: 1\n")]), [])
            # Camera: calibration YAML
            cal = pathlib.Path(d) / "camera_calibration.yaml"
            cal.write_text("image_width: 640\nimage_height: 480\n"
                           "distortion_model: rational_polynomial\ncamera_name: webcam\n",
                           encoding="utf-8")
            r1 = self.mod._extract_camera_configs([str(cal)])
            self.assertEqual(r1[0]["image_width"], 640)
            self.assertEqual(r1[0]["distortion_model"], "rational_polynomial")
            # Camera: OAK-D style
            yf3 = self._write_yaml(d, "oak:\n  ros__parameters:\n"
                                   "    i_pipeline_type: RGBD\n    i_enable_vio: true\n")
            r2 = self.mod._extract_camera_configs([yf3])
            self.assertEqual(r2[0]["i_pipeline_type"], "RGBD")
            self.assertTrue(r2[0]["i_enable_vio"])
            # EKF localization
            yf_ekf = self._write_yaml(d, """\
ekf_filter_node:
  ros__parameters:
    frequency: 50.0
    odom_frame: odom
    base_link_frame: base_footprint
    world_frame: odom
    odom0: /base_controller/odometry
    imu0: /imu/data
""")
            r_ekf = self.mod._extract_localization_config([yf_ekf])
            self.assertEqual(r_ekf["method"], "ekf")
            self.assertAlmostEqual(r_ekf["frequency_hz"], 50.0, places=1)
            self.assertIn("odom0", r_ekf["fused_sources"])
            self.assertIsNone(
                self.mod._extract_localization_config(
                    [self._write_yaml(d, "not_ekf: true\n")]))
            # Nav2: plugins and tolerances extracted
            yf_nav = self._write_yaml(d, """\
controller_server:
  ros__parameters:
    controller_plugins: [FollowPath]
    FollowPath:
      plugin: nav2_mppi_controller::MPPIController
      xy_goal_tolerance: 0.25
      yaw_goal_tolerance: 0.25
planner_server:
  ros__parameters:
    planner_plugins: [GridBased]
    GridBased:
      plugin: nav2_navfn_planner::NavfnPlanner
behavior_server:
  ros__parameters:
    behavior_plugins: [spin, backup, drive_on_heading, wait]
""")
            r_nav = self.mod._extract_nav2_config([yf_nav])
            self.assertIsNotNone(r_nav)
            self.assertIn("FollowPath", r_nav["controller_plugins"])
            self.assertIn("GridBased", r_nav["planner_plugins"])
            self.assertIn("spin", r_nav["behavior_plugins"])
            self.assertAlmostEqual(r_nav["xy_goal_tolerance"], 0.25, places=3)
            self.assertIsNone(
                self.mod._extract_nav2_config([self._write_yaml(d, "foo: bar\n")]))

        # --- teleop, estop, TF, launch args, controllers ---
        with tempfile.TemporaryDirectory() as d:
            teleop, estop = self.mod._extract_teleop_and_estop([self._write_yaml(d, """\
teleop_twist_joy_node:
  ros__parameters:
    topic_name: /base_controller/cmd_vel
    enable_button: 9
    scale_linear: 0.13
    scale_angular: 0.44
""")])
            self.assertIsNotNone(teleop)
            self.assertEqual(teleop["cmd_vel_topic"], "/base_controller/cmd_vel")
            self.assertEqual(teleop["enable_button"], 9)
            self.assertAlmostEqual(teleop["scales"]["scale_linear"], 0.13, places=4)
            self.assertIsNone(estop)
            _, estop2 = self.mod._extract_teleop_and_estop([self._write_yaml(d, """\
teleop_twist_joy_node:
  ros__parameters:
    topic_name: /cmd_vel
    emergency_stop: /robot/emergency_stop
""")])
            self.assertEqual(estop2["service_name"], "/robot/emergency_stop")
            t3, e3 = self.mod._extract_teleop_and_estop(
                [self._write_yaml(d, "nav2_params: true\n")])
            self.assertIsNone(t3)
            self.assertIsNone(e3)
            # TF frames from URDF links
            uf = self._write_urdf(d, """\
<?xml version="1.0"?><robot name="mybot">
  <link name="base_link"/><link name="laser_frame"/>
</robot>""")
            r_urdf = self.mod._extract_tf_frames([uf], [])
            self.assertIn("base_link", r_urdf["urdf_links"])
            self.assertIn("laser_frame", r_urdf["urdf_links"])
            # TF frames from EKF YAML
            r_yaml = self.mod._extract_tf_frames([], [self._write_yaml(d, """\
ekf_filter_node:
  ros__parameters:
    odom_frame: odom
    base_link_frame: base_footprint
    world_frame: odom
""")])
        self.assertEqual(r_yaml["odom_frame"], "odom")
        self.assertEqual(r_yaml["base_frame"], "base_footprint")

        launch_content = """\
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('config', default_value='base',
                              choices=['base', 'pantilt', 'k2'],
                              description='Robot hardware configuration'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
    ])
"""
        with tempfile.TemporaryDirectory() as d:
            result = self.mod._extract_launch_arg_choices(self._write_launch(d, launch_content))
            self.assertEqual(result["config"]["choices"], ["base", "pantilt", "k2"])
            self.assertEqual(result["config"]["default"], "base")
            self.assertEqual(result["use_sim_time"]["default"], "false")
            self.assertNotIn("choices", result["use_sim_time"])
            p = pathlib.Path(d) / "bringup.launch.xml"
            p.write_text("<launch/>\n", encoding="utf-8")
            self.assertEqual(self.mod._extract_launch_arg_choices(str(p)), {})
        # merge_launch_args precedence
        r = self.mod._merge_launch_args(
            {"config": {"default": "base", "choices": ["base", "k2"]}, "sim": {"default": "false"}}, {})
        self.assertEqual(r["config"]["choices"], ["base", "k2"])
        self.assertEqual(r["sim"]["default"], "false")
        self.assertEqual(
            self.mod._merge_launch_args({"port": {"default": "/dev/ttySERVO"}},
                                        {"port": "/dev/ttyUSB0"})["port"]["default"],
            "/dev/ttyUSB0")
        self.assertEqual(
            self.mod._merge_launch_args({"config": {"default": "base"}},
                                        {"config": None})["config"]["default"],
            "base")
        self.assertNotIn("mystery", self.mod._merge_launch_args({}, {"mystery": None}))
        self.assertEqual(
            self.mod._merge_launch_args({}, {"extra_arg": "somevalue"})["extra_arg"]["default"],
            "somevalue")
        rsort = self.mod._merge_launch_args({"z_arg": {"default": "1"}, "a_arg": {"default": "2"}}, {})
        self.assertEqual(list(rsort.keys()), ["a_arg", "z_arg"])
        # Active controllers: extracted from spawner nodes; cross-file deduplication
        spawner_launch = """\
from launch import LaunchDescription
from launch_ros.actions import Node
def generate_launch_description():
    return LaunchDescription([
        Node(package='controller_manager', executable='spawner',
             arguments=['joint_state_broadcaster']),
        Node(package='controller_manager', executable='spawner',
             arguments=['base_controller', '--controller-manager', '/controller_manager']),
    ])
"""
        dedup_launch = """\
from launch import LaunchDescription
from launch_ros.actions import Node
def generate_launch_description():
    return LaunchDescription([
        Node(package='controller_manager', executable='spawner',
             arguments=['joint_state_broadcaster']),
    ])
"""
        with tempfile.TemporaryDirectory() as d:
            ctrls = self.mod._extract_active_controllers(
                [self._write_launch(d, spawner_launch)])
            self.assertIn("joint_state_broadcaster", ctrls)
            self.assertIn("base_controller", ctrls)
            self.assertEqual(len(ctrls), 2)
            lf1 = self._write_launch(d, dedup_launch, "a.launch.py")
            lf2 = self._write_launch(d, dedup_launch, "b.launch.py")
            self.assertEqual(
                self.mod._extract_active_controllers([lf1, lf2]).count("joint_state_broadcaster"),
                1)


# ---------------------------------------------------------------------------
# _strip_nulls — absent-field pruning
# ---------------------------------------------------------------------------

class TestStripNulls(unittest.TestCase):
    """_strip_nulls removes None/[]/{}; keeps False and 0."""

    @classmethod
    def setUpClass(cls):
        import sys, pathlib, types
        scripts = str(pathlib.Path(__file__).parent.parent / "scripts")
        if scripts not in sys.path:
            sys.path.insert(0, scripts)
        if "ros2_utils" not in sys.modules:
            stub = types.ModuleType("ros2_utils")
            stub.output = lambda d: None
            sys.modules["ros2_utils"] = stub
        import ros2_profile
        cls.fn = staticmethod(ros2_profile._strip_nulls)

    def test_strip_nulls(self):
        """None/[]/{}  stripped; nested cascade; False/0 kept; lists/scalars pass through; safety_limits collapse."""
        # Null sentinels
        self.assertEqual(self.fn({"a": None, "b": 1}), {"b": 1})
        self.assertEqual(self.fn({"a": [], "b": 2}), {"b": 2})
        self.assertEqual(self.fn({"a": {}, "b": 3}), {"b": 3})
        self.assertEqual(self.fn({"outer": {"inner": None, "keep": 1}}), {"outer": {"keep": 1}})
        self.assertNotIn("outer", self.fn({"outer": {"inner": None}}))
        # Falsy but non-null values survive
        self.assertIs(self.fn({"flag": False, "other": None})["flag"], False)
        self.assertEqual(self.fn({"rate": 0, "missing": None})["rate"], 0)
        # Lists and non-dict scalars
        self.assertEqual(self.fn([{"a": None, "b": 1}, {"c": 2}]), [{"b": 1}, {"c": 2}])
        self.assertEqual(self.fn("hello"), "hello")
        self.assertEqual(self.fn(42), 42)
        # All-null binding collapses safety_limits; non-null binding kept
        self.assertEqual(
            self.fn({"safety_limits": {"sources": [],
                                       "binding": {"linear_x": None, "angular_z": None}}}),
            {},
        )
        result = self.fn({"safety_limits": {"sources": [{"file": "x"}],
                                            "binding": {"linear_x": 0.4}}})
        self.assertEqual(result["safety_limits"]["binding"]["linear_x"], 0.4)


# ---------------------------------------------------------------------------
# New profile field tests (Wave 3): maps, sensor_filter_pipeline, imu_config,
# controller_plugins, mock_hardware_available, package_dependencies
# ---------------------------------------------------------------------------

class TestExtractMaps(unittest.TestCase):
    """_extract_maps detects nav2 map-server YAML files."""

    @classmethod
    def setUpClass(cls):
        import sys, pathlib, types
        scripts = str(pathlib.Path(__file__).parent.parent / "scripts")
        if scripts not in sys.path:
            sys.path.insert(0, scripts)
        if "ros2_utils" not in sys.modules:
            stub = types.ModuleType("ros2_utils")
            stub.output = lambda d: None
            sys.modules["ros2_utils"] = stub
        import ros2_profile
        cls.mod = ros2_profile

    def _write_yaml(self, tmpdir, name, content):
        import tempfile, pathlib
        p = pathlib.Path(tmpdir) / name
        p.write_text(content)
        return str(p)

    def test_map_types_detected_and_multiple(self):
        """Occupancy (generic), keepout, speed maps all detected; multiple files supported."""
        import tempfile
        with tempfile.TemporaryDirectory() as d:
            # Occupancy map
            yf = self._write_yaml(d, "map.yaml",
                "image: map.pgm\nresolution: 0.05\norigin: [-10.0, -10.0, 0.0]\n"
                "occupied_thresh: 0.65\nfree_thresh: 0.196\n")
            r = self.mod._extract_maps([yf])
            self.assertEqual(len(r), 1)
            self.assertEqual(r[0]["type"], "occupancy")
            self.assertAlmostEqual(r[0]["resolution"], 0.05)
            self.assertEqual(r[0]["image"], "map.pgm")
            # Keepout detected by filename
            yf2 = self._write_yaml(d, "keepout_zones.yaml",
                "image: keepout.pgm\nresolution: 0.05\norigin: [0.0, 0.0, 0.0]\n")
            self.assertEqual(self.mod._extract_maps([yf2])[0]["type"], "keepout")
            # Speed detected by filename
            yf3 = self._write_yaml(d, "speed_limit_map.yaml",
                "image: speed.pgm\nresolution: 0.1\norigin: [0.0, 0.0, 0.0]\n")
            self.assertEqual(self.mod._extract_maps([yf3])[0]["type"], "speed")
            # Multiple maps returned
            self.assertEqual(len(self.mod._extract_maps([yf, yf2])), 2)
            # Non-map and image-without-resolution both return []
            self.assertEqual(self.mod._extract_maps([self._write_yaml(
                d, "params.yaml", "controller_manager:\n  ros__parameters:\n    update_rate: 100\n")]), [])
            self.assertEqual(self.mod._extract_maps([self._write_yaml(
                d, "camera_cal.yaml", "image_width: 640\nimage_height: 480\nimage: camera.png\norigin: [0,0,0]\n")]), [])


class TestExtractSensorFilterPipeline(unittest.TestCase):
    """_extract_sensor_filter_pipeline extracts filter chain entries."""

    @classmethod
    def setUpClass(cls):
        import sys, pathlib, types
        scripts = str(pathlib.Path(__file__).parent.parent / "scripts")
        if scripts not in sys.path:
            sys.path.insert(0, scripts)
        if "ros2_utils" not in sys.modules:
            stub = types.ModuleType("ros2_utils")
            stub.output = lambda d: None
            sys.modules["ros2_utils"] = stub
        import ros2_profile
        cls.mod = ros2_profile

    def _write_yaml(self, tmpdir, name, content):
        import pathlib
        p = pathlib.Path(tmpdir) / name
        p.write_text(content)
        return str(p)

    def test_filter_chain_extracted_and_source_file_recorded(self):
        """laser_filter_node and laser_scan_filter_chain keys both recognized; source_file set."""
        import tempfile
        with tempfile.TemporaryDirectory() as d:
            # Full filter chain under laser_filter_node
            yf = self._write_yaml(d, "laser_filter.yaml",
                "laser_filter_node:\n  ros__parameters:\n    filter_chain:\n"
                "      - name: range_filter\n        type: laser_filters/LaserScanRangeFilter\n"
                "        params:\n          lower_threshold: 0.1\n          upper_threshold: 10.0\n"
                "      - name: angular_filter\n        type: laser_filters/LaserScanAngularBoundsFilter\n")
            r = self.mod._extract_sensor_filter_pipeline([yf])
            self.assertIsNotNone(r)
            self.assertEqual(len(r), 2)
            self.assertEqual(r[0]["name"], "range_filter")
            self.assertEqual(r[0]["type"], "laser_filters/LaserScanRangeFilter")
            self.assertIn("params", r[0])
            # laser_scan_filter_chain key recognized
            yf2 = self._write_yaml(d, "shadow.yaml",
                "laser_scan_filter_chain:\n  - name: shadow_filter\n    type: laser_filters/LaserScanShadowFilter\n")
            r2 = self.mod._extract_sensor_filter_pipeline([yf2])
            self.assertEqual(r2[0]["type"], "laser_filters/LaserScanShadowFilter")
            # source_file recorded
            yf3 = self._write_yaml(d, "myfilter.yaml",
                "filter_chain:\n  - name: f1\n    type: laser_filters/LaserScanRangeFilter\n")
            r3 = self.mod._extract_sensor_filter_pipeline([yf3])
            self.assertEqual(r3[0]["source_file"], "myfilter.yaml")
            # YAML without filter_chain key → None
            yf4 = self._write_yaml(d, "params.yaml", "controller_manager:\n  update_rate: 100\n")
            self.assertIsNone(self.mod._extract_sensor_filter_pipeline([yf4]))


class TestExtractImuConfig(unittest.TestCase):
    """_extract_imu_config extracts IMU hardware + broadcaster config."""

    @classmethod
    def setUpClass(cls):
        import sys, pathlib, types
        scripts = str(pathlib.Path(__file__).parent.parent / "scripts")
        if scripts not in sys.path:
            sys.path.insert(0, scripts)
        if "ros2_utils" not in sys.modules:
            stub = types.ModuleType("ros2_utils")
            stub.output = lambda d: None
            sys.modules["ros2_utils"] = stub
        import ros2_profile
        cls.mod = ros2_profile

    def _write_yaml(self, tmpdir, name, content):
        import pathlib
        p = pathlib.Path(tmpdir) / name
        p.write_text(content)
        return str(p)

    def test_imu_plugin_broadcaster_and_combined(self):
        """IMU plugin from hw_ifaces; broadcaster from YAML; combined has both; absent → None."""
        import tempfile
        # Plugin from hardware interfaces
        hw = [{"name": "BNO055", "type": "sensor",
               "plugin": "bno055_hardware_interface/BNO055HardwareInterface",
               "joints": [], "command_interfaces": [],
               "state_interfaces": ["imu/orientation/x", "imu/orientation/y"],
               "hardware_params": {"i2c_bus": 1, "i2c_address": 40}}]
        r1 = self.mod._extract_imu_config(hw, [])
        self.assertIsNotNone(r1)
        self.assertIn("bno055", r1["plugin"].lower())
        self.assertEqual(r1["state_interfaces"], ["imu/orientation/x", "imu/orientation/y"])
        self.assertEqual(r1["hardware_params"]["i2c_bus"], 1)
        # Broadcaster from YAML
        with tempfile.TemporaryDirectory() as d:
            yf = self._write_yaml(d, "imu_broadcaster.yaml",
                                  "imu_sensor_broadcaster:\n  ros__parameters:\n"
                                  "    frame_id: imu_link\n    publish_rate: 100.0\n")
            r2 = self.mod._extract_imu_config([], [yf])
            self.assertIsNotNone(r2)
            self.assertEqual(r2["broadcaster"]["frame_id"], "imu_link")
            self.assertAlmostEqual(r2["broadcaster"]["publish_rate"], 100.0)
            # Combined: both plugin and broadcaster present
            hw2 = [{"name": "IMU", "type": "sensor", "plugin": "ros2_control_bno055/BNO055",
                    "joints": [], "command_interfaces": [],
                    "state_interfaces": ["imu/angular_velocity/z"],
                    "hardware_params": {"sensor_mode": 12}}]
            yf2 = self._write_yaml(d, "imu_bc.yaml",
                                   "imu_sensor_broadcaster:\n  ros__parameters:\n    frame_id: base_imu_link\n")
            r3 = self.mod._extract_imu_config(hw2, [yf2])
            self.assertIn("plugin", r3)
            self.assertIn("broadcaster", r3)
            self.assertEqual(r3["broadcaster"]["frame_id"], "base_imu_link")
            # Non-IMU hardware + unrelated YAML → None
            hw_non_imu = [{"name": "wheels", "type": "system",
                           "plugin": "diff_drive_controller/DiffDriveController",
                           "joints": ["left_wheel", "right_wheel"],
                           "command_interfaces": ["velocity"],
                           "state_interfaces": ["position", "velocity"],
                           "hardware_params": None}]
            yf_non = self._write_yaml(d, "params.yaml", "controller_manager:\n  update_rate: 50\n")
            self.assertIsNone(self.mod._extract_imu_config(hw_non_imu, [yf_non]))


class TestExtractPackageDependencies(unittest.TestCase):
    """_extract_package_dependencies returns exec_depend lists for primary packages."""

    @classmethod
    def setUpClass(cls):
        import sys, pathlib, types
        scripts = str(pathlib.Path(__file__).parent.parent / "scripts")
        if scripts not in sys.path:
            sys.path.insert(0, scripts)
        if "ros2_utils" not in sys.modules:
            stub = types.ModuleType("ros2_utils")
            stub.output = lambda d: None
            sys.modules["ros2_utils"] = stub
        import ros2_profile
        cls.mod = ros2_profile

    def test_package_dependencies(self):
        """Primary packages included with deps; missing role defaults; dependency-role excluded; empty/no-primary → None."""
        pkgs = [
            {"name": "my_robot_bringup", "role": "primary",
             "deps": ["rclpy", "nav2_bringup", "robot_state_publisher"]},
            {"name": "my_robot_description", "role": "primary", "deps": ["urdf", "xacro"]},
        ]
        r = self.mod._extract_package_dependencies(pkgs)
        self.assertIsNotNone(r)
        self.assertIn("my_robot_bringup", r)
        self.assertIn("nav2_bringup", r["my_robot_bringup"])
        self.assertIn("my_robot_description", r)
        # No 'role' key → defaults to primary
        r2 = self.mod._extract_package_dependencies([{"name": "my_robot", "deps": ["rclpy"]}])
        self.assertIsNotNone(r2)
        self.assertIn("my_robot", r2)
        # Dependency role excluded from result
        r3 = self.mod._extract_package_dependencies([
            {"name": "my_robot_bringup", "role": "primary", "deps": ["rclpy"]},
            {"name": "some_driver", "role": "dependency", "deps": ["libusb"]},
        ])
        self.assertNotIn("some_driver", r3)
        self.assertIn("my_robot_bringup", r3)
        # Empty deps → None; no primary packages → None
        self.assertIsNone(self.mod._extract_package_dependencies(
            [{"name": "my_robot_bringup", "role": "primary", "deps": []}]))
        self.assertIsNone(self.mod._extract_package_dependencies(
            [{"name": "driver", "role": "dependency", "deps": ["libusb"]}]))


class TestMockHardwareAvailable(unittest.TestCase):
    """mock_hardware_available derivation logic."""

    @classmethod
    def setUpClass(cls):
        import sys, pathlib, types
        scripts = str(pathlib.Path(__file__).parent.parent / "scripts")
        if scripts not in sys.path:
            sys.path.insert(0, scripts)
        if "ros2_utils" not in sys.modules:
            stub = types.ModuleType("ros2_utils")
            stub.output = lambda d: None
            sys.modules["ros2_utils"] = stub
        import ros2_profile
        cls.mod = ros2_profile

    def _mock_hw_available(self, hardware_interfaces, launch_configurations):
        """Reproduce the mock_hardware_available derivation from _run_static_scan."""
        return bool(
            any(
                str((iface.get("hardware_params") or {}).get("enable_mock_mode", "")).lower()
                in ("true", "1", "yes")
                for iface in hardware_interfaces
            )
            or any(
                "mock" in arg.lower() or "fake" in arg.lower()
                for arg in launch_configurations
            )
        )

    def test_mock_signals(self):
        """enable_mock_mode true/1 or mock/fake args → True; false/unrelated args → False."""
        for hw, cfg in [
            ([{"hardware_params": {"enable_mock_mode": "true"}}], {}),
            ([{"hardware_params": {"enable_mock_mode": 1}}], {}),
            ([], {"use_mock_hardware": {}}),
            ([], {"fake_sensor": {}}),
        ]:
            with self.subTest(expected=True, hw=hw, cfg=cfg):
                self.assertTrue(self._mock_hw_available(hw, cfg))
        for hw, cfg in [
            ([{"hardware_params": {"enable_mock_mode": "false"}}], {}),
            ([{"hardware_params": {"baud_rate": 115200}}], {"use_sim_time": {}}),
            ([{"hardware_params": None}], {}),
        ]:
            with self.subTest(expected=False, hw=hw, cfg=cfg):
                self.assertFalse(self._mock_hw_available(hw, cfg))


class TestControllerPlugins(unittest.TestCase):
    """controller_plugins is surfaced from ros2_control YAML at the top level."""

    @classmethod
    def setUpClass(cls):
        import sys, pathlib, types
        scripts = str(pathlib.Path(__file__).parent.parent / "scripts")
        if scripts not in sys.path:
            sys.path.insert(0, scripts)
        if "ros2_utils" not in sys.modules:
            stub = types.ModuleType("ros2_utils")
            stub.output = lambda d: None
            sys.modules["ros2_utils"] = stub
        import ros2_profile
        cls.mod = ros2_profile

    def _write_yaml(self, tmpdir, name, content):
        import pathlib
        p = pathlib.Path(tmpdir) / name
        p.write_text(content)
        return str(p)

    def test_controller_plugins_returned(self):
        """Plugins extracted from controller_manager YAML; unrelated YAML returns empty list."""
        import tempfile
        yaml_content = (
            "controller_manager:\n"
            "  ros__parameters:\n"
            "    update_rate: 100\n"
            "    base_controller:\n"
            "      type: diff_drive_controller/DiffDriveController\n"
            "    joint_state_broadcaster:\n"
            "      type: joint_state_broadcaster/JointStateBroadcaster\n"
        )
        with tempfile.TemporaryDirectory() as d:
            yf = self._write_yaml(d, "ctrl.yaml", yaml_content)
            result = self.mod._extract_ros2_control_config([yf])
            plugins = result["controller_plugins"]
            self.assertIn("diff_drive_controller/DiffDriveController", plugins)
            self.assertIn("joint_state_broadcaster/JointStateBroadcaster", plugins)
            yf2 = self._write_yaml(d, "empty.yaml", "robot_name: testbot\n")
            self.assertEqual(self.mod._extract_ros2_control_config([yf2])["controller_plugins"], [])


class TestPkgMatchHints(unittest.TestCase):
    """_pkg_match_hints uses token-level matching to avoid short-token false positives."""

    @classmethod
    def setUpClass(cls):
        import sys, pathlib
        scripts = str(pathlib.Path(__file__).parent.parent / "scripts")
        if scripts not in sys.path:
            sys.path.insert(0, scripts)
        # Import only the pure helper — no ROS 2 dependency.
        import importlib, types
        # Provide a stub for ros2_utils so the import doesn't fail without ROS.
        if "ros2_utils" not in sys.modules:
            stub = types.ModuleType("ros2_utils")
            stub.output = lambda d: None
            sys.modules["ros2_utils"] = stub
        import ros2_profile
        cls.mod = ros2_profile

    def _match(self, hints, pkgs):
        return self.mod._pkg_match_hints(hints, pkgs)

    def test_token_isolation_compound_and_cap(self):
        """Token: 'nao' matches nao_robot not autonomous_navigation; compound uses substring; capped at 5."""
        self.assertEqual(self._match({"nao"}, ["autonomous_navigation", "my_scenario_pkg"]), [])
        self.assertTrue(self._match({"nao"}, ["nao_robot"]))
        self.assertTrue(self._match({"atlas"}, ["atlas_description"]))
        self.assertTrue(self._match({"diff_drive"}, ["diff_drive_controller"]))
        self.assertEqual(self._match({"humanoid", "zmp", "valkyrie"}, ["my_bringup", "pantilt_driver"]), [])
        self.assertLessEqual(len(self._match({"nao"}, [f"nao_pkg_{i}" for i in range(10)])), 5)


class TestUrdfHumanoidEvidence(unittest.TestCase):
    """_urdf_humanoid_evidence requires ≥4 humanoid-pattern joints."""

    @classmethod
    def setUpClass(cls):
        import sys, pathlib, types
        scripts = str(pathlib.Path(__file__).parent.parent / "scripts")
        if scripts not in sys.path:
            sys.path.insert(0, scripts)
        if "ros2_utils" not in sys.modules:
            stub = types.ModuleType("ros2_utils")
            stub.output = lambda d: None
            sys.modules["ros2_utils"] = stub
        import ros2_profile
        cls.fn = staticmethod(ros2_profile._urdf_humanoid_evidence)

    def test_humanoid_evidence_thresholds(self):
        """≥4 humanoid joints → evidence with 'urdf-joint:' prefix; empty/<4/non-humanoid → []."""
        ev = self.fn({"head_pan": {}, "neck_tilt": {}, "torso_yaw": {}, "shoulder_pitch": {}})
        self.assertEqual(len(ev), 4)
        self.assertTrue(all(e.startswith("urdf-joint:") for e in ev))
        self.assertEqual(self.fn({}), [])
        self.assertEqual(self.fn({"head_pan": {}, "neck_tilt": {}, "torso_yaw": {}}), [])
        self.assertEqual(self.fn({"wheel_left": {}, "wheel_right": {}, "caster": {}, "axle": {}}), [])
        self.assertEqual(self.fn({"head_pan": {}, "head_tilt": {}}), [])


class TestUrdfLeggedEvidence(unittest.TestCase):
    """_urdf_legged_evidence requires ≥4 leg-pattern joints."""

    @classmethod
    def setUpClass(cls):
        import sys, pathlib, types
        scripts = str(pathlib.Path(__file__).parent.parent / "scripts")
        if scripts not in sys.path:
            sys.path.insert(0, scripts)
        if "ros2_utils" not in sys.modules:
            stub = types.ModuleType("ros2_utils")
            stub.output = lambda d: None
            sys.modules["ros2_utils"] = stub
        import ros2_profile
        cls.fn = staticmethod(ros2_profile._urdf_legged_evidence)

    def test_evidence_threshold_and_hip_knee_matches(self):
        """≥4 leg-pattern joints → evidence; <4 → empty; hip/knee substrings match."""
        self.assertEqual(len(self.fn({"fl_hip": {}, "fr_hip": {}, "rl_hip": {}, "rr_hip": {}})), 4)
        self.assertEqual(self.fn({"fl_hip": {}, "fr_hip": {}, "rl_hip": {}}), [])
        self.assertEqual(len(self.fn(
            {"j_hip_pitch": {}, "j_knee_pitch": {}, "j_ankle_roll": {}, "j_hip_roll": {}})), 4)


class TestDetectRobotType(unittest.TestCase):
    """_detect_robot_type integration: correct primary types and evidence."""

    @classmethod
    def setUpClass(cls):
        import sys, pathlib, types
        scripts = str(pathlib.Path(__file__).parent.parent / "scripts")
        if scripts not in sys.path:
            sys.path.insert(0, scripts)
        if "ros2_utils" not in sys.modules:
            stub = types.ModuleType("ros2_utils")
            stub.output = lambda d: None
            sys.modules["ros2_utils"] = stub
        import ros2_profile
        cls.fn = staticmethod(ros2_profile._detect_robot_type)

    def _detect(self, pkg_names=None, src_files=None, vel_topics=None,
                joints=None, nav2=False, override=None):
        return self.fn(
            ws_pkg_names=pkg_names or [],
            all_src_files=src_files or [],
            velocity_topics=vel_topics or [],
            joint_limits=joints or {},
            has_nav2=nav2,
            robot_type_override=override,
        )

    def test_detect_robot_type(self):
        """Override/mobile_base/humanoid/legged/manipulator/unknown detection; evidence structure validated."""
        import tempfile, pathlib
        rtype, _, ev = self._detect(pkg_names=["nao_description", "nao_robot"], override="mobile_base")
        self.assertEqual(rtype, "mobile_base")
        self.assertIn("override", ev)
        self.assertEqual(self._detect(override="flying_car")[0], "unknown")
        self.assertEqual(self._detect(vel_topics=["/cmd_vel"])[0], "mobile_base")
        self.assertEqual(self._detect(nav2=True)[0], "mobile_base")
        rtype2, features, ev2 = self._detect(pkg_names=["my_bringup", "pantilt_driver"],
                                             vel_topics=["/cmd_vel"])
        self.assertEqual(rtype2, "mobile_base")
        self.assertIn("pantilt", features)
        self.assertIn("pantilt", ev2)
        rtype3, _, ev3 = self._detect(vel_topics=["/cmd_vel"])
        self.assertIn(rtype3, ev3)
        _, _, ev4 = self._detect(pkg_names=["nao_robot"], vel_topics=["/cmd_vel"])
        for label, signals in ev4.items():
            for s in signals:
                self.assertIsInstance(s, str, f"Signal in {label!r} is not a string: {s!r}")
        rtype_h, _, ev_h = self._detect(pkg_names=["nao_robot"])
        self.assertEqual(rtype_h, "humanoid")
        self.assertTrue(any("pkg:" in s for s in ev_h["humanoid"]))
        for keyword, suffix in [("walking and balance control\ncmd_vel_publisher = ...", ".py"),
                                 ("// PID balance controller for wheeled platform", ".cpp")]:
            with tempfile.NamedTemporaryFile(suffix=suffix, mode="w", delete=False) as f:
                f.write(f"# {keyword}\n")
                fname = f.name
            try:
                self.assertNotEqual(
                    self._detect(src_files=[fname], vel_topics=["/cmd_vel"])[0], "humanoid",
                    f"{keyword!r} in src caused false humanoid detection",
                )
            finally:
                pathlib.Path(fname).unlink(missing_ok=True)
        joints = {"fl_hip": {}, "fr_hip": {}, "rl_hip": {}, "rr_hip": {},
                  "fl_knee": {}, "fr_knee": {}}
        self.assertEqual(self._detect(joints=joints)[0], "legged")
        self.assertEqual(self._detect(pkg_names=["arm_description", "moveit_config"],
                                      vel_topics=["/cmd_vel"])[0], "mobile_manipulator")
        rtype_u, features_u, _ = self._detect()
        self.assertEqual(rtype_u, "unknown")
        self.assertEqual(features_u, [])


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

    def test_all_dispatch_keys_parse_and_symmetry(self):
        """Every MINIMAL_ARGS entry parses correctly; MINIMAL_ARGS ↔ DISPATCH are in sync."""
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
        # Bidirectional symmetry: every MINIMAL_ARGS key is in DISPATCH and vice-versa
        covered = {(cmd, sub) for cmd, sub, _ in MINIMAL_ARGS}
        for cmd, sub, _ in MINIMAL_ARGS:
            with self.subTest(direction="minimal→dispatch", cmd=cmd, sub=sub):
                self.assertIn((cmd, sub), D,
                              f"DISPATCH missing ({cmd!r}, {sub!r}) — add it or update MINIMAL_ARGS")
        for key in D:
            with self.subTest(direction="dispatch→minimal", key=key):
                self.assertIn(key, covered,
                              f"DISPATCH key {key!r} has no MINIMAL_ARGS entry — add a parse test row")


# ---------------------------------------------------------------------------
# Path A guard tests – declarative table behaviour
# ---------------------------------------------------------------------------

class TestGetVelocityTypes(unittest.TestCase):
    """Tests for _get_velocity_types dynamic type extractor."""

    def setUp(self):
        _setup_ros_mocks()
        import ros2_profile
        self.mod = ros2_profile

    def test_baseline_and_custom_types(self):
        """Baseline always present; custom velocity_topics types added with /msg/ stripped; missing type key safe."""
        baseline = {"geometry_msgs/Twist", "geometry_msgs/TwistStamped"}
        # Baseline with empty/None input
        self.assertEqual(self.mod._get_velocity_types({}), baseline)
        self.assertEqual(self.mod._get_velocity_types({"velocity_topics": []}), baseline)
        self.assertEqual(self.mod._get_velocity_types({"velocity_topics": None}), baseline)
        # Single custom type added; /msg/ stripped
        types = self.mod._get_velocity_types({"velocity_topics": [
            {"topic": "/cmd_vel", "type": "geometry_msgs/msg/TwistWithCovarianceStamped"},
        ]})
        self.assertIn("geometry_msgs/TwistWithCovarianceStamped", types)
        self.assertIn("geometry_msgs/Twist", types)
        # Multiple entries all collected; /msg/ stripped; raw form absent
        types2 = self.mod._get_velocity_types({"velocity_topics": [
            {"topic": "/cmd_vel", "type": "custom_msgs/msg/DriveCmd"},
            {"topic": "/cmd_vel_aux", "type": "geometry_msgs/msg/Twist"},
        ]})
        self.assertIn("custom_msgs/DriveCmd", types2)
        self.assertIn("geometry_msgs/Twist", types2)
        self.assertNotIn("geometry_msgs/msg/Twist", types2)
        # Entry without a 'type' key must not crash
        t3 = self.mod._get_velocity_types({"velocity_topics": [{"topic": "/cmd_vel"}]})
        self.assertIn("geometry_msgs/Twist", t3)


class TestGetEstopTypes(unittest.TestCase):
    """Tests for _get_estop_types dynamic type extractor."""

    def setUp(self):
        _setup_ros_mocks()
        import ros2_profile
        self.mod = ros2_profile

    def test_baseline_and_custom_type(self):
        """std_srvs/SetBool always present; custom service_type added with /srv/ stripped; empty config returns baseline only."""
        # Baseline always present; empty/absent estop_config → baseline only
        self.assertEqual(self.mod._get_estop_types({}), {"std_srvs/SetBool"})
        self.assertEqual(self.mod._get_estop_types({"estop_config": {}}), {"std_srvs/SetBool"})
        # Custom type added; /srv/ segment stripped
        types = self.mod._get_estop_types({
            "estop_config": {"service_name": "/estop",
                             "service_type": "custom_srvs/srv/EmergencyStop"}})
        self.assertIn("custom_srvs/EmergencyStop", types)
        self.assertIn("std_srvs/SetBool", types)
        # /srv/ stripping: std_srvs/srv/SetBool → std_srvs/SetBool
        t2 = self.mod._get_estop_types({"estop_config": {"service_type": "std_srvs/srv/SetBool"}})
        self.assertIn("std_srvs/SetBool", t2)
        self.assertNotIn("std_srvs/srv/SetBool", t2)


class TestCheckTopicsFindPathA(unittest.TestCase):
    """Tests for check_topics_find_path_a declarative guard."""

    def setUp(self):
        _setup_ros_mocks()
        import ros2_profile
        self.fn = ros2_profile.check_topics_find_path_a

    def test_blocked_types_and_safe_cases(self):
        """Velocity and odometry types blocked with correct violation structure; safe cases → None."""
        cmd_vel_summary = {"cmd_vel_topic": "/cmd_vel", "velocity_topics": []}
        # Velocity types blocked
        for msg_type, summary in [
            ("geometry_msgs/Twist", cmd_vel_summary),
            ("geometry_msgs/TwistStamped", {"cmd_vel_topic": "/cmd_vel_stamped"}),
            ("geometry_msgs/msg/Twist", {"cmd_vel_topic": "/cmd_vel"}),
            ("geometry_msgs/TwistWithCovarianceStamped", {
                "cmd_vel_topic": "/cmd_vel",
                "velocity_topics": [{"topic": "/cmd_vel",
                                     "type": "geometry_msgs/msg/TwistWithCovarianceStamped"}],
            }),
        ]:
            with self.subTest(msg_type=msg_type):
                self.assertIsNotNone(self.fn(msg_type, summary))
        # Violation structure on canonical velocity case
        result = self.fn("geometry_msgs/Twist", cmd_vel_summary)
        self.assertEqual(result["error"], "path_a_violation")
        self.assertIn("cmd_vel_topic", result["profile_field"])
        self.assertIn("topics find geometry_msgs/Twist", result["message"])
        self.assertIn("--ignore-profile", result["override"])
        # Odometry types blocked
        fused = {"localization_config": {"fused_sources": {"odom": "/wheel/odom"}}}
        for msg_type in ("nav_msgs/Odometry", "nav_msgs/msg/Odometry",
                         "nav_msgs/OdometryWithCovarianceStamped"):
            with self.subTest(msg_type=msg_type):
                self.assertIsNotNone(self.fn(msg_type, fused))
        result2 = self.fn("nav_msgs/Odometry",
                          {"localization_config": {"fused_sources": {"odom": "/odom",
                                                                      "imu": "/imu/data"}}})
        self.assertIn("fused_sources", result2["profile_field"])
        # Safe cases → None
        for msg_type, summary in [
            ("geometry_msgs/Twist", {}),
            ("nav_msgs/Odometry", {"localization_config": {}}),
            ("sensor_msgs/LaserScan",
             {"cmd_vel_topic": "/cmd_vel", "localization_config": {"fused_sources": {}}}),
            ("geometry_msgs/Twist", None),
            ("geometry_msgs/Twist", "bad"),
        ]:
            with self.subTest(safe_type=msg_type):
                self.assertIsNone(self.fn(msg_type, summary))


class TestCheckServicesFindPathA(unittest.TestCase):
    """Tests for check_services_find_path_a declarative guard."""

    def setUp(self):
        _setup_ros_mocks()
        import ros2_profile
        self.fn = ros2_profile.check_services_find_path_a

    def test_guard_fires_and_safe_cases(self):
        """Guard fires for estop types (violation structure correct); does not fire when safe."""
        fire_cases = [
            ("std_srvs/SetBool",
             {"estop_config": {"service_name": "/estop", "service_type": "std_srvs/SetBool"}}),
            ("std_srvs/srv/SetBool",
             {"estop_config": {"service_name": "/estop"}}),
            ("custom_srvs/EmergencyStop",
             {"estop_config": {"service_name": "/emergency_stop",
                               "service_type": "custom_srvs/EmergencyStop"}}),
            ("custom_srvs/srv/EmergencyStop",
             {"estop_config": {"service_name": "/emergency_stop",
                               "service_type": "custom_srvs/EmergencyStop"}}),
        ]
        for svc_type, summary in fire_cases:
            with self.subTest(svc_type=svc_type):
                self.assertIsNotNone(self.fn(svc_type, summary))
        # Violation structure on the canonical case
        result = self.fn("std_srvs/SetBool",
                         {"estop_config": {"service_name": "/estop",
                                           "service_type": "std_srvs/SetBool"}})
        self.assertEqual(result["error"], "path_a_violation")
        self.assertIn("estop_config", result["profile_field"])
        self.assertIn("services find std_srvs/SetBool", result["message"])
        self.assertIn("--ignore-profile", result["override"])
        # No-fire: no profile, no service_name, unrelated type, empty/non-dict summary
        summary_estop = {"estop_config": {"service_name": "/estop"}}
        safe_cases = [
            ("std_srvs/SetBool", {}),
            ("std_srvs/SetBool", {"estop_config": {"service_type": "std_srvs/SetBool"}}),
            ("rcl_interfaces/srv/GetParameters", summary_estop),
            ("std_srvs/SetBool", None),
        ]
        for svc_type, summary in safe_cases:
            with self.subTest(safe_svc_type=svc_type, summary=summary):
                self.assertIsNone(self.fn(svc_type, summary))


# ---------------------------------------------------------------------------
# Nav2 command tests
# ---------------------------------------------------------------------------

class TestNav2ArgParsing(unittest.TestCase):
    """Parser tests for all nav2 subcommands."""

    @classmethod
    def setUpClass(cls):
        _setup_ros_mocks()
        import ros2_cli
        cls.parser = ros2_cli.build_parser()

    def test_nav2_all_args(self):
        """go/initial-pose/cancel/status/go-waypoints all parse correctly with correct defaults and flags."""
        p = self.parser
        go = p.parse_args(["nav2", "go", "1.5", "-2.3"])
        self.assertAlmostEqual(go.x, 1.5)
        self.assertAlmostEqual(go.y, -2.3)
        self.assertEqual(go.frame, "map")
        self.assertIsNone(go.yaw)
        self.assertGreater(go.timeout, 0)
        go2 = p.parse_args(["nav2", "go", "1.0", "2.0", "--frame", "odom", "--yaw", "90.0", "--feedback"])
        self.assertEqual(go2.frame, "odom")
        self.assertAlmostEqual(go2.yaw, 90.0)
        self.assertTrue(go2.feedback)
        ip = p.parse_args(["nav2", "initial-pose", "1.5", "2.5"])
        self.assertEqual(ip.frame, "map")
        self.assertAlmostEqual(ip.yaw, 0.0)
        self.assertAlmostEqual(
            p.parse_args(["nav2", "initial-pose", "1.0", "2.0", "--yaw", "45.0"]).yaw, 45.0)
        cancel = p.parse_args(["nav2", "cancel"])
        self.assertEqual(cancel.command, "nav2")
        self.assertEqual(cancel.subcommand, "cancel")
        self.assertEqual(
            p.parse_args(["nav2", "cancel", "--action", "/navigate_to_pose"]).action,
            "/navigate_to_pose")
        self.assertEqual(p.parse_args(["nav2", "status"]).subcommand, "status")
        wp = p.parse_args(["nav2", "go-waypoints", "1.0,2.0", "3.0,4.0", "5.0,6.0"])
        self.assertEqual(len(wp.waypoints), 3)
        self.assertTrue(wp.stop_on_failure)
        self.assertFalse(
            p.parse_args(["nav2", "go-waypoints", "1.0,2.0", "--no-stop-on-failure"]).stop_on_failure)


class TestNav2YawToQuaternion(unittest.TestCase):
    """Unit tests for _yaw_to_quaternion — pure Python, no ROS needed."""

    def setUp(self):
        _setup_ros_mocks()
        import ros2_nav2
        self.fn = ros2_nav2._yaw_to_quaternion

    def test_yaw_to_quaternion(self):
        """Specific yaw values produce correct z/w; x/y always 0; ||q||==1; 360° wraps like 0°."""
        import math
        cases = [
            (0.0,   0.0,                   1.0),
            (90.0,  math.sin(math.pi / 4), math.cos(math.pi / 4)),
            (180.0, math.sin(math.pi / 2), math.cos(math.pi / 2)),
            (-90.0, -math.sin(math.pi / 4), math.cos(math.pi / 4)),
        ]
        for yaw, exp_z, exp_w in cases:
            with self.subTest(yaw=yaw):
                q = self.fn(yaw)
                self.assertAlmostEqual(q["x"], 0.0, places=5)
                self.assertAlmostEqual(q["y"], 0.0, places=5)
                self.assertAlmostEqual(q["z"], exp_z, places=5)
                self.assertAlmostEqual(q["w"], exp_w, places=5)
        # Unit norm for all test angles; 360° → same |w| as 0°
        for yaw in [0, 30, 45, 90, 135, 180, -90, -45, 270, 360]:
            with self.subTest(norm_yaw=yaw):
                q = self.fn(float(yaw))
                norm = math.sqrt(sum(q[k] ** 2 for k in "xyzw"))
                self.assertAlmostEqual(norm, 1.0, places=6)
        self.assertAlmostEqual(abs(self.fn(0.0)["w"]), abs(self.fn(360.0)["w"]), places=5)


class TestNav2ParseWaypoints(unittest.TestCase):
    """Unit tests for _parse_waypoints — pure Python, no ROS needed."""

    def setUp(self):
        _setup_ros_mocks()
        import ros2_nav2
        self.fn = ros2_nav2._parse_waypoints

    def test_waypoints_valid_and_invalid(self):
        """Valid: single/multi/negative/int → list of 2-tuples; invalid formats raise ValueError."""
        r = self.fn(["1.0,2.0"])
        self.assertIsInstance(r, list)
        self.assertIsInstance(r[0], tuple)
        self.assertEqual(len(r[0]), 2)
        self.assertAlmostEqual(r[0][0], 1.0)
        self.assertAlmostEqual(r[0][1], 2.0)
        r3 = self.fn(["1.0,2.0", "3.0,4.0", "5.0,6.0"])
        self.assertEqual(len(r3), 3)
        self.assertAlmostEqual(r3[1][0], 3.0)
        self.assertAlmostEqual(self.fn(["-1.5,-2.5"])[0][0], -1.5)
        self.assertAlmostEqual(self.fn(["1,2"])[0][0], 1.0)
        for bad in [["1.0"], ["1.0,2.0,3.0"], ["notanumber"], [""]]:
            with self.subTest(val=bad), self.assertRaises(ValueError):
                self.fn(bad)


class TestNav2BuildGoal(unittest.TestCase):
    """Unit tests for _build_nav2_goal — pure Python, no ROS needed."""

    def setUp(self):
        _setup_ros_mocks()
        import ros2_nav2
        self.fn = ros2_nav2._build_nav2_goal

    def test_build_goal(self):
        """position/frame_id correct; no yaw → identity quaternion; yaw 90° → z≈sin(π/4)."""
        import math
        goal = self.fn(1.5, -2.3, None, "map")
        pos = goal["pose"]["pose"]["position"]
        self.assertAlmostEqual(pos["x"], 1.5)
        self.assertAlmostEqual(pos["y"], -2.3)
        self.assertAlmostEqual(pos["z"], 0.0)
        self.assertEqual(goal["pose"]["header"]["frame_id"], "map")
        self.assertEqual(self.fn(0.0, 0.0, None, "odom")["pose"]["header"]["frame_id"], "odom")
        orient = goal["pose"]["pose"]["orientation"]
        self.assertAlmostEqual(orient["w"], 1.0, places=6)
        self.assertAlmostEqual(orient["z"], 0.0, places=6)
        orient90 = self.fn(0.0, 0.0, 90.0, "map")["pose"]["pose"]["orientation"]
        self.assertAlmostEqual(orient90["z"], math.sin(math.pi / 4), places=5)
        self.assertAlmostEqual(orient90["w"], math.cos(math.pi / 4), places=5)


class TestNav2CmdValidation(unittest.TestCase):
    """Error-path and output-structure tests for all nav2 command functions.

    Uses mocked rclpy so no live ROS graph is needed — commands should fail
    gracefully with structured JSON error output.
    """

    def setUp(self):
        _setup_ros_mocks()
        import ros2_nav2
        self.mod = ros2_nav2

    def _capture(self, fn, **kw):
        import io, argparse
        buf = io.StringIO()
        with unittest.mock.patch("sys.stdout", buf):
            fn(argparse.Namespace(**kw))
        return json.loads(buf.getvalue())

    def test_nav2_command_error_paths(self):
        """go error names server; cancel→error; status/initial_pose→dict; go_waypoints bad input→error."""
        # go: error contains action server name
        result = self._capture(
            self.mod.cmd_nav2_go,
            x=1.0, y=2.0, yaw=None, frame="map",
            timeout=0.1, feedback=False, action="/navigate_to_pose",
        )
        self.assertIn("error", result)
        self.assertIn("navigate_to_pose", str(result["error"]).lower())
        # cancel returns error; status and initial_pose always return dicts
        self.assertIn("error", self._capture(
            self.mod.cmd_nav2_cancel, action="/navigate_to_pose", timeout=0.1))
        self.assertIsInstance(self._capture(self.mod.cmd_nav2_status, timeout=0.1), dict)
        r_pose = self._capture(
            self.mod.cmd_nav2_initial_pose, x=3.0, y=4.0, yaw=0.0, frame="map", timeout=0.1)
        self.assertIsInstance(r_pose, dict)
        self.assertTrue("x" in r_pose or "error" in r_pose)
        # go_waypoints: empty, invalid format, too-many-values all return error
        base = dict(yaw=None, frame="map", timeout=0.1, stop_on_failure=True,
                    action="/navigate_to_pose")
        for bad_wp in [[], ["notanumber"], ["1.0,2.0,3.0"]]:
            with self.subTest(waypoints=bad_wp):
                self.assertIn("error", self._capture(
                    self.mod.cmd_nav2_go_waypoints, waypoints=bad_wp, **base))


# ===========================================================================
# v1.0.8 feature parsing — consolidated
# ===========================================================================

class TestRecentFeatureParsing(unittest.TestCase):
    """Parser tests for v1.0.8 additions: throttle-rate-ms, params exists,
    actions status, capture-image --inline, actions cancel --goal-id."""

    @classmethod
    def setUpClass(cls):
        _setup_ros_mocks()
        import ros2_cli
        cls.parser = ros2_cli.build_parser()

    def test_recent_feature_parsing(self):
        """throttle-rate-ms/inline flags; params exists/actions status/cancel --goal-id parsed and dispatched."""
        from ros2_cli import DISPATCH
        p = self.parser
        self.assertEqual(
            p.parse_args(["topics", "subscribe", "/imu", "--throttle-rate-ms", "100"]).throttle_rate_ms, 100)
        self.assertIsNone(p.parse_args(["topics", "subscribe", "/t"]).throttle_rate_ms)
        self.assertEqual(
            p.parse_args(["topics", "echo", "/t", "--throttle-rate-ms", "50"]).throttle_rate_ms, 50)
        base = ["topics", "capture-image", "--topic", "/camera/image_raw", "--output", "out.jpg"]
        self.assertTrue(p.parse_args(base + ["--inline"]).inline)
        self.assertFalse(p.parse_args(base).inline)
        a_pe = p.parse_args(["params", "exists", "/node:my_param"])
        self.assertEqual(a_pe.subcommand, "exists")
        self.assertEqual(a_pe.name, "/node:my_param")
        self.assertEqual(p.parse_args(["params", "exists", "/n:p", "--timeout", "3"]).timeout, 3.0)
        self.assertIn(("params", "exists"), DISPATCH)
        a_as = p.parse_args(["actions", "status", "/navigate_to_pose"])
        self.assertEqual(a_as.subcommand, "status")
        self.assertEqual(a_as.action, "/navigate_to_pose")
        self.assertIn(("actions", "status"), DISPATCH)
        self.assertEqual(
            p.parse_args(["actions", "cancel", "/a", "--goal-id", "abc-123"]).goal_id, "abc-123")
        self.assertIsNone(p.parse_args(["actions", "cancel", "/a"]).goal_id)


class TestRecentFeatureOutput(unittest.TestCase):
    """Output-structure tests for v1.0.8 command functions."""

    def setUp(self):
        _setup_ros_mocks()

    def _capture(self, fn, args_ns):
        import io
        buf = io.StringIO()
        with unittest.mock.patch("sys.stdout", buf):
            fn(args_ns)
        return json.loads(buf.getvalue())

    def _ns(self, **kw):
        import argparse
        return argparse.Namespace(**kw)

    def test_recent_feature_output(self):
        """Subscribe→capped/dropped; actions list→has_active_goals; params_exists valid/bad; actions_status None→error."""
        import ros2_topic, ros2_action, ros2_param, argparse
        r_sub = self._capture(ros2_topic.cmd_topics_subscribe,
                              self._ns(topic="/t", msg_type=None, duration=0.05,
                                       max_messages=3, timeout=5.0, throttle_rate_ms=None))
        self.assertIsInstance(r_sub, dict)
        if "error" not in r_sub:
            self.assertIn("capped", r_sub)
            self.assertIn("messages_dropped", r_sub)
        r_list = self._capture(ros2_action.cmd_actions_list, self._ns(timeout=0.1))
        self.assertIsInstance(r_list, dict)
        if "error" not in r_list:
            self.assertIn("has_active_goals", r_list)
            self.assertIsInstance(r_list["has_active_goals"], dict)
        r = self._capture(ros2_param.cmd_params_exists, self._ns(name="/n:p", timeout=0.1))
        self.assertIsInstance(r, dict)
        self.assertTrue("exists" in r or "error" in r)
        self.assertIn("error", self._capture(ros2_param.cmd_params_exists,
                                             self._ns(name="/nocolon", timeout=0.1)))
        self.assertIn("error", self._capture(ros2_action.cmd_actions_status,
                                             argparse.Namespace(action=None, timeout=0.1)))
        r2 = self._capture(ros2_action.cmd_actions_status,
                           argparse.Namespace(action="/a", timeout=0.1))
        self.assertIsInstance(r2, dict)
        self.assertTrue("action" in r2 or "error" in r2)


class TestFoxglove(unittest.TestCase):
    """Unit tests for ros2_foxglove: cmd_foxglove_start/stop/status."""

    def setUp(self):
        _setup_ros_mocks()
        import ros2_foxglove
        self.mod = ros2_foxglove

    # ------------------------------------------------------------------ helpers

    def _ns(self, **kw):
        import argparse
        return argparse.Namespace(**kw)

    def _capture(self, fn, ns):
        import io
        buf = io.StringIO()
        with unittest.mock.patch("sys.stdout", buf):
            fn(ns)
        raw = buf.getvalue().strip()
        return json.loads(raw) if raw else {}

    # ------------------------------------------------------------------ start

    def test_start_no_tmux(self):
        """start → error when tmux is absent."""
        with unittest.mock.patch.object(self.mod, "check_tmux", return_value=False):
            r = self._capture(self.mod.cmd_foxglove_start, self._ns(port=8765))
        self.assertIn("error", r)
        self.assertIn("tmux", r["error"].lower())

    def test_start_invalid_port_low(self):
        """start → error for port 0."""
        with unittest.mock.patch.object(self.mod, "check_tmux", return_value=True):
            r = self._capture(self.mod.cmd_foxglove_start, self._ns(port=0))
        self.assertIn("error", r)
        self.assertIn("port", r["error"].lower())

    def test_start_invalid_port_high(self):
        """start → error for port 99999."""
        with unittest.mock.patch.object(self.mod, "check_tmux", return_value=True):
            r = self._capture(self.mod.cmd_foxglove_start, self._ns(port=99999))
        self.assertIn("error", r)

    def test_start_already_running(self):
        """start → error with session info when bridge is already running."""
        with (
            unittest.mock.patch.object(self.mod, "check_tmux", return_value=True),
            unittest.mock.patch.object(self.mod, "session_exists", return_value=True),
            unittest.mock.patch.object(self.mod, "check_session_alive", return_value=True),
            unittest.mock.patch.object(self.mod, "_is_port_bound", return_value=True),
        ):
            r = self._capture(self.mod.cmd_foxglove_start, self._ns(port=8765))
        self.assertIn("error", r)
        self.assertIn("8765", r["error"])
        self.assertIn("connect_url", r)
        self.assertEqual(r["port_bound"], True)

    def test_start_package_not_found_includes_distro_hint(self):
        """start → error with distro-specific apt hint when package absent."""
        with (
            unittest.mock.patch.object(self.mod, "check_tmux", return_value=True),
            unittest.mock.patch.object(self.mod, "session_exists", return_value=False),
            unittest.mock.patch.object(self.mod, "package_exists", return_value=False),
            unittest.mock.patch.object(self.mod, "list_packages", return_value={}),
            unittest.mock.patch.object(self.mod, "get_ros_distro", return_value="kilted"),
        ):
            r = self._capture(self.mod.cmd_foxglove_start, self._ns(port=8765))
        self.assertIn("error", r)
        self.assertIn("kilted", r.get("install", ""), msg="install hint must name the active distro")
        self.assertEqual(r.get("current_distro"), "kilted")

    def test_start_package_not_found_unknown_distro(self):
        """start → error with generic hint when $ROS_DISTRO is unset."""
        with (
            unittest.mock.patch.object(self.mod, "check_tmux", return_value=True),
            unittest.mock.patch.object(self.mod, "session_exists", return_value=False),
            unittest.mock.patch.object(self.mod, "package_exists", return_value=False),
            unittest.mock.patch.object(self.mod, "list_packages", return_value={}),
            unittest.mock.patch.object(self.mod, "get_ros_distro", return_value="unknown"),
        ):
            r = self._capture(self.mod.cmd_foxglove_start, self._ns(port=8765))
        self.assertIn("error", r)
        # Should not contain a specific distro name in the install command
        self.assertNotIn("ros-unknown-", r.get("install", ""))

    def test_start_success(self):
        """start → success dict with port, connect_url, session, distro."""
        fake_prefix = "/opt/ros/kilted"
        fake_launch = fake_prefix + "/share/foxglove_bridge/launch/foxglove_bridge_launch.xml"
        with (
            unittest.mock.patch.object(self.mod, "check_tmux", return_value=True),
            unittest.mock.patch.object(self.mod, "session_exists", return_value=False),
            unittest.mock.patch.object(self.mod, "package_exists", return_value=True),
            unittest.mock.patch.object(self.mod, "get_package_prefix", return_value=fake_prefix),
            unittest.mock.patch.object(self.mod, "_find_launch_file", return_value=fake_launch),
            unittest.mock.patch.object(self.mod, "source_local_ws", return_value=(None, "not_found")),
            unittest.mock.patch.object(self.mod, "run_cmd", return_value=("", "", 0)),
            unittest.mock.patch.object(self.mod, "check_session_alive", return_value=True),
            unittest.mock.patch.object(self.mod, "save_session", return_value=None),
            unittest.mock.patch.object(self.mod, "get_ros_distro", return_value="kilted"),
        ):
            r = self._capture(self.mod.cmd_foxglove_start, self._ns(port=8765))
        self.assertTrue(r.get("success"), r)
        self.assertEqual(r["port"], 8765)
        self.assertEqual(r["distro"], "kilted")
        self.assertIn("connect_url", r)
        self.assertIn("8765", r["connect_url"])
        self.assertIn("session", r)
        self.assertIn("foxglove_bridge", r["session"])

    def test_start_default_port(self):
        """start with no port arg defaults to 8765."""
        from ros2_cli import build_parser
        p = build_parser()
        args = p.parse_args(["foxglove", "start"])
        self.assertEqual(args.port, 8765)

    def test_start_custom_port(self):
        """start accepts an explicit port."""
        from ros2_cli import build_parser
        p = build_parser()
        args = p.parse_args(["foxglove", "start", "9000"])
        self.assertEqual(args.port, 9000)

    # ------------------------------------------------------------------ stop

    def test_stop_no_tmux(self):
        """stop → error when tmux absent."""
        with unittest.mock.patch.object(self.mod, "check_tmux", return_value=False):
            r = self._capture(self.mod.cmd_foxglove_stop, self._ns(port=None))
        self.assertIn("error", r)

    def test_stop_no_sessions(self):
        """stop → success message when nothing is running."""
        with (
            unittest.mock.patch.object(self.mod, "check_tmux", return_value=True),
            unittest.mock.patch.object(self.mod, "_get_active_foxglove_sessions", return_value=[]),
        ):
            r = self._capture(self.mod.cmd_foxglove_stop, self._ns(port=None))
        self.assertTrue(r.get("success"), r)
        self.assertEqual(r.get("stopped"), [])
        self.assertIn("message", r)

    def test_stop_single_session_no_port(self):
        """stop with one running session and no port filter kills it."""
        sname = "launch_foxglove_bridge_port8765"
        with (
            unittest.mock.patch.object(self.mod, "check_tmux", return_value=True),
            unittest.mock.patch.object(self.mod, "_get_active_foxglove_sessions",
                                       return_value=[sname]),
            unittest.mock.patch.object(self.mod, "kill_session", return_value=True),
            unittest.mock.patch.object(self.mod, "delete_session_metadata", return_value=None),
        ):
            r = self._capture(self.mod.cmd_foxglove_stop, self._ns(port=None))
        self.assertTrue(r.get("success"), r)
        self.assertIn(sname, r.get("stopped", []))

    def test_stop_multiple_sessions_no_port_returns_list(self):
        """stop with multiple sessions and no port → error listing all sessions."""
        sessions = [
            "launch_foxglove_bridge_port8765",
            "launch_foxglove_bridge_port9090",
        ]
        with (
            unittest.mock.patch.object(self.mod, "check_tmux", return_value=True),
            unittest.mock.patch.object(self.mod, "_get_active_foxglove_sessions",
                                       return_value=sessions),
        ):
            r = self._capture(self.mod.cmd_foxglove_stop, self._ns(port=None))
        self.assertIn("error", r)
        self.assertIn("running_sessions", r)
        # Each entry should expose port number
        ports = [entry["port"] for entry in r["running_sessions"]]
        self.assertIn(8765, ports)
        self.assertIn(9090, ports)

    def test_stop_port_filter_correct(self):
        """stop --port kills only the targeted session."""
        sname = "launch_foxglove_bridge_port9090"
        with (
            unittest.mock.patch.object(self.mod, "check_tmux", return_value=True),
            unittest.mock.patch.object(self.mod, "_get_active_foxglove_sessions",
                                       return_value=["launch_foxglove_bridge_port8765", sname]),
            unittest.mock.patch.object(self.mod, "kill_session", return_value=True),
            unittest.mock.patch.object(self.mod, "delete_session_metadata", return_value=None),
        ):
            r = self._capture(self.mod.cmd_foxglove_stop, self._ns(port=9090))
        self.assertTrue(r.get("success"), r)
        self.assertIn(sname, r.get("stopped", []))

    def test_stop_port_not_found(self):
        """stop --port N → error when no session on that port."""
        with (
            unittest.mock.patch.object(self.mod, "check_tmux", return_value=True),
            unittest.mock.patch.object(self.mod, "_get_active_foxglove_sessions",
                                       return_value=["launch_foxglove_bridge_port8765"]),
        ):
            r = self._capture(self.mod.cmd_foxglove_stop, self._ns(port=9999))
        self.assertIn("error", r)
        self.assertIn("running_sessions", r)

    # ------------------------------------------------------------------ status

    def test_status_no_tmux(self):
        """status → error when tmux absent."""
        with unittest.mock.patch.object(self.mod, "check_tmux", return_value=False):
            r = self._capture(self.mod.cmd_foxglove_status, self._ns())
        self.assertIn("error", r)

    def test_status_no_sessions(self):
        """status → running=False with install hint when nothing running."""
        with (
            unittest.mock.patch.object(self.mod, "check_tmux", return_value=True),
            unittest.mock.patch.object(self.mod, "_get_active_foxglove_sessions", return_value=[]),
            unittest.mock.patch.object(self.mod, "get_ros_distro", return_value="kilted"),
        ):
            r = self._capture(self.mod.cmd_foxglove_status, self._ns())
        self.assertFalse(r.get("running"), r)
        self.assertEqual(r.get("bridges"), [])
        # Install hint must name the active distro
        self.assertIn("kilted", r.get("install_hint", ""))

    def test_status_ready_session(self):
        """status shows ready when tmux alive and port bound."""
        sname = "launch_foxglove_bridge_port8765"
        with (
            unittest.mock.patch.object(self.mod, "check_tmux", return_value=True),
            unittest.mock.patch.object(self.mod, "_get_active_foxglove_sessions",
                                       return_value=[sname]),
            unittest.mock.patch.object(self.mod, "check_session_alive", return_value=True),
            unittest.mock.patch.object(self.mod, "_is_port_bound", return_value=True),
            unittest.mock.patch.object(self.mod, "get_ros_distro", return_value="kilted"),
        ):
            r = self._capture(self.mod.cmd_foxglove_status, self._ns())
        self.assertTrue(r.get("running"), r)
        self.assertEqual(r.get("count"), 1)
        bridge = r["bridges"][0]
        self.assertEqual(bridge["status"], "ready")
        self.assertEqual(bridge["port"], 8765)
        self.assertIn("connect_url", bridge)

    def test_status_starting_session(self):
        """status shows starting when tmux alive but port not yet bound."""
        sname = "launch_foxglove_bridge_port8765"
        with (
            unittest.mock.patch.object(self.mod, "check_tmux", return_value=True),
            unittest.mock.patch.object(self.mod, "_get_active_foxglove_sessions",
                                       return_value=[sname]),
            unittest.mock.patch.object(self.mod, "check_session_alive", return_value=True),
            unittest.mock.patch.object(self.mod, "_is_port_bound", return_value=False),
            unittest.mock.patch.object(self.mod, "get_ros_distro", return_value="kilted"),
        ):
            r = self._capture(self.mod.cmd_foxglove_status, self._ns())
        bridge = r["bridges"][0]
        self.assertEqual(bridge["status"], "starting")

    def test_status_crashed_session(self):
        """status shows crashed when tmux pane is dead."""
        sname = "launch_foxglove_bridge_port8765"
        with (
            unittest.mock.patch.object(self.mod, "check_tmux", return_value=True),
            unittest.mock.patch.object(self.mod, "_get_active_foxglove_sessions",
                                       return_value=[sname]),
            unittest.mock.patch.object(self.mod, "check_session_alive", return_value=False),
            unittest.mock.patch.object(self.mod, "_is_port_bound", return_value=False),
            unittest.mock.patch.object(self.mod, "get_ros_distro", return_value="kilted"),
        ):
            r = self._capture(self.mod.cmd_foxglove_status, self._ns())
        bridge = r["bridges"][0]
        self.assertEqual(bridge["status"], "crashed")
        self.assertIn("hint", bridge)

    # ------------------------------------------------------------------ helpers

    def test_port_from_session_valid(self):
        """_port_from_session parses standard session names."""
        self.assertEqual(self.mod._port_from_session("launch_foxglove_bridge_port8765"), 8765)
        self.assertEqual(self.mod._port_from_session("launch_foxglove_bridge_port1234"), 1234)

    def test_port_from_session_invalid(self):
        """_port_from_session returns None for unexpected formats."""
        self.assertIsNone(self.mod._port_from_session("launch_foxglove_bridge_portABC"))
        self.assertIsNone(self.mod._port_from_session("launch_foxglove_bridge_"))
        self.assertIsNone(self.mod._port_from_session("something_else"))

    def test_dispatch_foxglove_entries(self):
        """DISPATCH has all three foxglove entries; launch foxglove aliases start."""
        from ros2_cli import DISPATCH
        self.assertIn(("foxglove", "start"),  DISPATCH)
        self.assertIn(("foxglove", "stop"),   DISPATCH)
        self.assertIn(("foxglove", "status"), DISPATCH)
        # backward compat
        self.assertIn(("launch", "foxglove"), DISPATCH)
        self.assertIs(DISPATCH[("launch", "foxglove")], DISPATCH[("foxglove", "start")])

    def test_stop_parser_port_optional(self):
        """foxglove stop accepts --port or no args."""
        from ros2_cli import build_parser
        p = build_parser()
        args_no_port = p.parse_args(["foxglove", "stop"])
        self.assertIsNone(args_no_port.port)
        args_with_port = p.parse_args(["foxglove", "stop", "--port", "9090"])
        self.assertEqual(args_with_port.port, 9090)


class TestSystem(unittest.TestCase):
    """Unit tests for ros2_system: cmd_system_battery/shutdown/reboot."""

    @classmethod
    def setUpClass(cls):
        import ros2_system
        cls.mod = ros2_system

    # ---- helpers -----------------------------------------------------------

    def _capture(self, fn, args):
        import ros2_utils
        buf = StringIO()
        side = lambda d: buf.write(json.dumps(d))  # noqa: E731
        with (
            patch.object(ros2_utils, "output", side_effect=side),
            patch.object(self.mod, "output", side_effect=side),
        ):
            fn(args)
        return json.loads(buf.getvalue()) if buf.getvalue() else {}

    def _ns(self, **kwargs):
        defaults = dict(
            topic=None,
            threshold=20.0,
            warn=30.0,
            timeout=5.0,
            confirm=False,
        )
        defaults.update(kwargs)
        return argparse.Namespace(**defaults)

    # ------------------------------------------------------------------ battery

    def test_battery_no_topics_found(self):
        """battery → error when no BatteryState topics on graph."""
        with (
            patch.object(self.mod, "ros2_context"),
            patch.object(self.mod, "ROS2CLI"),
            patch.object(self.mod, "_discover_battery_topics", return_value=[]),
        ):
            r = self._capture(self.mod.cmd_system_battery, self._ns())
        self.assertIn("error", r)

    def test_battery_specific_topic_not_found(self):
        """battery --topic /missing_topic → error when topic absent from graph."""
        mock_node = MagicMock()
        mock_node.get_topic_names_and_types.return_value = []

        ctx = MagicMock()
        ctx.__enter__ = MagicMock(return_value=None)
        ctx.__exit__ = MagicMock(return_value=False)

        with (
            patch.object(self.mod, "ros2_context", return_value=ctx),
            patch.object(self.mod, "ROS2CLI", return_value=mock_node),
        ):
            r = self._capture(self.mod.cmd_system_battery, self._ns(topic="/missing_topic"))
        self.assertIn("error", r)

    def test_battery_msg_class_unavailable(self):
        """battery → error when sensor_msgs/BatteryState cannot be loaded."""
        with (
            patch.object(self.mod, "ros2_context"),
            patch.object(self.mod, "ROS2CLI"),
            patch.object(self.mod, "_discover_battery_topics",
                         return_value=[{"topic": "/battery_state", "type": "sensor_msgs/msg/BatteryState"}]),
            patch.object(self.mod, "get_msg_type", return_value=None),
        ):
            r = self._capture(self.mod.cmd_system_battery, self._ns())
        self.assertIn("error", r)

    def test_battery_timeout_no_message(self):
        """battery → error entry per topic when no message arrives within timeout."""
        fake_sub = MagicMock()
        fake_sub.messages = []
        fake_sub.lock = __import__("threading").Lock()

        executor = MagicMock()

        with (
            patch.object(self.mod, "ros2_context"),
            patch.object(self.mod, "ROS2CLI"),
            patch.object(self.mod, "_discover_battery_topics",
                         return_value=[{"topic": "/battery_state", "type": "sensor_msgs/msg/BatteryState"}]),
            patch.object(self.mod, "get_msg_type", return_value=MagicMock()),
            patch.object(self.mod, "TopicSubscriber", return_value=fake_sub),
            patch("rclpy.executors.SingleThreadedExecutor", return_value=executor),
        ):
            r = self._capture(self.mod.cmd_system_battery, self._ns(timeout=0.0))
        # Should still return a response (health unknown) with timeout error per battery
        self.assertIn("batteries", r)
        self.assertIn("error", r["batteries"][0])

    def test_battery_ok_health(self):
        """battery → health=ok when percentage is above both thresholds."""
        parsed_data = {
            "percentage": 80.0,
            "voltage": 12.5,
            "current": -0.5,
            "status_name": "DISCHARGING",
            "health_name": "GOOD",
            "present": True,
            "location": "",
        }
        fake_sub = MagicMock()
        fake_sub.messages = [{"raw": "data"}]
        fake_sub.lock = __import__("threading").Lock()
        executor = MagicMock()

        with (
            patch.object(self.mod, "ros2_context"),
            patch.object(self.mod, "ROS2CLI"),
            patch.object(self.mod, "_discover_battery_topics",
                         return_value=[{"topic": "/battery_state", "type": "sensor_msgs/msg/BatteryState"}]),
            patch.object(self.mod, "get_msg_type", return_value=MagicMock()),
            patch.object(self.mod, "TopicSubscriber", return_value=fake_sub),
            patch.object(self.mod, "_parse_battery_state", return_value=parsed_data),
            patch("rclpy.executors.SingleThreadedExecutor", return_value=executor),
        ):
            r = self._capture(self.mod.cmd_system_battery, self._ns())

        self.assertEqual(r.get("health"), "ok")
        self.assertFalse(r["batteries"][0]["critical"])
        self.assertFalse(r["batteries"][0]["warning"])
        self.assertNotIn("action", r)

    def test_battery_warning_health(self):
        """battery → health=warning when percentage is between warn and critical thresholds."""
        parsed_data = {
            "percentage": 25.0,    # < 30 (warn) but >= 20 (critical)
            "voltage": 11.0,
            "current": -0.5,
            "status_name": "DISCHARGING",
            "health_name": "GOOD",
            "present": True,
            "location": "",
        }
        fake_sub = MagicMock()
        fake_sub.messages = [{"raw": "data"}]
        fake_sub.lock = __import__("threading").Lock()
        executor = MagicMock()

        with (
            patch.object(self.mod, "ros2_context"),
            patch.object(self.mod, "ROS2CLI"),
            patch.object(self.mod, "_discover_battery_topics",
                         return_value=[{"topic": "/battery_state", "type": "sensor_msgs/msg/BatteryState"}]),
            patch.object(self.mod, "get_msg_type", return_value=MagicMock()),
            patch.object(self.mod, "TopicSubscriber", return_value=fake_sub),
            patch.object(self.mod, "_parse_battery_state", return_value=parsed_data),
            patch("rclpy.executors.SingleThreadedExecutor", return_value=executor),
        ):
            r = self._capture(self.mod.cmd_system_battery, self._ns())

        self.assertEqual(r.get("health"), "warning")
        self.assertFalse(r["batteries"][0]["critical"])
        self.assertTrue(r["batteries"][0]["warning"])
        self.assertIn("action", r)

    def test_battery_critical_health(self):
        """battery → health=critical when percentage is below critical threshold."""
        parsed_data = {
            "percentage": 10.0,    # < 20 (critical)
            "voltage": 9.5,
            "current": -0.3,
            "status_name": "DISCHARGING",
            "health_name": "GOOD",
            "present": True,
            "location": "",
        }
        fake_sub = MagicMock()
        fake_sub.messages = [{"raw": "data"}]
        fake_sub.lock = __import__("threading").Lock()
        executor = MagicMock()

        with (
            patch.object(self.mod, "ros2_context"),
            patch.object(self.mod, "ROS2CLI"),
            patch.object(self.mod, "_discover_battery_topics",
                         return_value=[{"topic": "/battery_state", "type": "sensor_msgs/msg/BatteryState"}]),
            patch.object(self.mod, "get_msg_type", return_value=MagicMock()),
            patch.object(self.mod, "TopicSubscriber", return_value=fake_sub),
            patch.object(self.mod, "_parse_battery_state", return_value=parsed_data),
            patch("rclpy.executors.SingleThreadedExecutor", return_value=executor),
        ):
            r = self._capture(self.mod.cmd_system_battery, self._ns())

        self.assertEqual(r.get("health"), "critical")
        self.assertTrue(r["batteries"][0]["critical"])
        self.assertFalse(r["batteries"][0]["warning"])
        self.assertIn("STOP", r.get("action", ""))

    def test_battery_custom_thresholds(self):
        """battery --threshold 15 --warn 25 respects custom thresholds."""
        parsed_data = {
            "percentage": 20.0,   # < 25 (warn) but >= 15 (critical)
            "voltage": 10.0,
            "current": -0.2,
            "status_name": "DISCHARGING",
            "health_name": "GOOD",
            "present": True,
            "location": "",
        }
        fake_sub = MagicMock()
        fake_sub.messages = [{"raw": "data"}]
        fake_sub.lock = __import__("threading").Lock()
        executor = MagicMock()

        with (
            patch.object(self.mod, "ros2_context"),
            patch.object(self.mod, "ROS2CLI"),
            patch.object(self.mod, "_discover_battery_topics",
                         return_value=[{"topic": "/battery_state", "type": "sensor_msgs/msg/BatteryState"}]),
            patch.object(self.mod, "get_msg_type", return_value=MagicMock()),
            patch.object(self.mod, "TopicSubscriber", return_value=fake_sub),
            patch.object(self.mod, "_parse_battery_state", return_value=parsed_data),
            patch("rclpy.executors.SingleThreadedExecutor", return_value=executor),
        ):
            r = self._capture(self.mod.cmd_system_battery,
                              self._ns(threshold=15.0, warn=25.0))

        self.assertEqual(r.get("health"), "warning")

    def test_battery_parser_defaults(self):
        """Parser sets correct defaults for system battery."""
        from ros2_cli import build_parser
        p = build_parser()
        args = p.parse_args(["system", "battery"])
        self.assertIsNone(args.topic)
        self.assertAlmostEqual(args.threshold, 20.0)
        self.assertAlmostEqual(args.warn, 30.0)
        self.assertAlmostEqual(args.timeout, 5.0)

    def test_battery_parser_custom(self):
        """Parser accepts --topic, --threshold, --warn, --timeout."""
        from ros2_cli import build_parser
        p = build_parser()
        args = p.parse_args([
            "system", "battery",
            "--topic", "/my_battery",
            "--threshold", "15",
            "--warn", "25",
            "--timeout", "10",
        ])
        self.assertEqual(args.topic, "/my_battery")
        self.assertAlmostEqual(args.threshold, 15.0)
        self.assertAlmostEqual(args.warn, 25.0)
        self.assertAlmostEqual(args.timeout, 10.0)

    # ------------------------------------------------------------------ shutdown

    def test_shutdown_requires_confirm(self):
        """shutdown without --confirm → safety interlock error."""
        r = self._capture(self.mod.cmd_system_shutdown, self._ns(confirm=False))
        self.assertIn("error", r)
        self.assertIn("--confirm", r.get("hint", ""))

    def test_shutdown_ros_service_success(self):
        """shutdown --confirm → uses ROS service when available."""
        with (
            patch.object(self.mod, "_try_ros_service", return_value=True),
            patch.object(self.mod, "run_cmd", return_value=("", "", 0)),
        ):
            r = self._capture(self.mod.cmd_system_shutdown, self._ns(confirm=True))
        self.assertTrue(r.get("success"))
        self.assertEqual(r.get("method"), "ros_service")

    def test_shutdown_fallback_to_sudo(self):
        """shutdown --confirm → falls back to sudo shutdown now when no ROS service."""
        with (
            patch.object(self.mod, "_try_ros_service", return_value=False),
            patch.object(self.mod, "run_cmd", return_value=("", "", 0)),
        ):
            r = self._capture(self.mod.cmd_system_shutdown, self._ns(confirm=True))
        self.assertTrue(r.get("success"))
        self.assertEqual(r.get("method"), "sudo_shutdown")

    def test_shutdown_failure(self):
        """shutdown → error when both service and sudo fail."""
        with (
            patch.object(self.mod, "_try_ros_service", return_value=False),
            patch.object(self.mod, "run_cmd", return_value=("", "permission denied", 1)),
        ):
            r = self._capture(self.mod.cmd_system_shutdown, self._ns(confirm=True))
        self.assertIn("error", r)

    # ------------------------------------------------------------------ reboot

    def test_reboot_requires_confirm(self):
        """reboot without --confirm → safety interlock error."""
        r = self._capture(self.mod.cmd_system_reboot, self._ns(confirm=False))
        self.assertIn("error", r)
        self.assertIn("--confirm", r.get("hint", ""))

    def test_reboot_ros_service_success(self):
        """reboot --confirm → uses ROS service when available."""
        with (
            patch.object(self.mod, "_try_ros_service", return_value=True),
            patch.object(self.mod, "run_cmd", return_value=("", "", 0)),
        ):
            r = self._capture(self.mod.cmd_system_reboot, self._ns(confirm=True))
        self.assertTrue(r.get("success"))
        self.assertEqual(r.get("method"), "ros_service")

    def test_reboot_fallback_to_sudo(self):
        """reboot --confirm → falls back to sudo reboot when no ROS service."""
        with (
            patch.object(self.mod, "_try_ros_service", return_value=False),
            patch.object(self.mod, "run_cmd", return_value=("", "", 0)),
        ):
            r = self._capture(self.mod.cmd_system_reboot, self._ns(confirm=True))
        self.assertTrue(r.get("success"))
        self.assertEqual(r.get("method"), "sudo_reboot")

    # ------------------------------------------------------------------ dispatch

    def test_dispatch_system_entries(self):
        """DISPATCH has all three system entries."""
        from ros2_cli import DISPATCH
        self.assertIn(("system", "battery"),  DISPATCH)
        self.assertIn(("system", "shutdown"), DISPATCH)
        self.assertIn(("system", "reboot"),   DISPATCH)

    def test_shutdown_parser_confirm_flag(self):
        """system shutdown --confirm sets confirm=True."""
        from ros2_cli import build_parser
        p = build_parser()
        self.assertFalse(p.parse_args(["system", "shutdown"]).confirm)
        self.assertTrue(p.parse_args(["system", "shutdown", "--confirm"]).confirm)

    def test_reboot_parser_confirm_flag(self):
        """system reboot --confirm sets confirm=True."""
        from ros2_cli import build_parser
        p = build_parser()
        self.assertFalse(p.parse_args(["system", "reboot"]).confirm)
        self.assertTrue(p.parse_args(["system", "reboot", "--confirm"]).confirm)


class TestNav2MapMode(unittest.TestCase):
    """Unit tests for nav2 map and nav2 mode commands."""

    @classmethod
    def setUpClass(cls):
        import ros2_nav2
        cls.mod = ros2_nav2

    def _capture(self, fn, args):
        import ros2_utils
        buf = StringIO()
        side = lambda d: buf.write(json.dumps(d))  # noqa: E731
        with (
            patch.object(ros2_utils, "output", side_effect=side),
            patch.object(self.mod, "output", side_effect=side),
        ):
            fn(args)
        return json.loads(buf.getvalue()) if buf.getvalue() else {}

    def _ns(self, **kwargs):
        defaults = dict(
            maps_dir="maps",
            name=None,
            confirm=False,
            timeout=5.0,
            mode=None,
        )
        defaults.update(kwargs)
        return argparse.Namespace(**defaults)

    # ------------------------------------------------------------------ nav2 map list

    def test_map_list_empty(self):
        """nav2 map list → empty list when no profile and maps dir absent."""
        with patch("os.path.isdir", return_value=False):
            r = self._capture(self.mod.cmd_nav2_map_list, self._ns())
        self.assertEqual(r.get("maps"), [])
        self.assertEqual(r.get("count"), 0)

    def test_map_list_from_filesystem(self):
        """nav2 map list reads .yaml files from the maps directory."""
        import glob as _glob
        with (
            patch("os.path.isdir", return_value=True),
            patch("glob.glob", return_value=["/ws/maps/office.yaml", "/ws/maps/lab.yaml"]),
            patch("builtins.open", side_effect=FileNotFoundError),  # no profile
        ):
            r = self._capture(self.mod.cmd_nav2_map_list, self._ns())
        names = [m["name"] for m in r.get("maps", [])]
        self.assertIn("office", names)
        self.assertIn("lab", names)

    # ------------------------------------------------------------------ nav2 map save

    def test_map_save_no_service(self):
        """nav2 map save → error when neither slam_toolbox nor map_saver available."""
        with (
            patch.object(self.mod, "ros2_context"),
            patch.object(self.mod, "ROS2CLI"),
            patch.object(self.mod, "_call_service", return_value=(None, "Service not available")),
        ):
            r = self._capture(self.mod.cmd_nav2_map_save, self._ns(name="test_map"))
        # Should return error (no slam_toolbox service imported)
        # ImportError is caught, falls through to error
        self.assertIn("error", r)

    # ------------------------------------------------------------------ nav2 map load

    def test_map_load_file_not_found(self):
        """nav2 map load → error when map file does not exist."""
        with patch("os.path.exists", return_value=False):
            r = self._capture(self.mod.cmd_nav2_map_load, self._ns(name="missing_map"))
        self.assertIn("error", r)
        self.assertIn("not found", r.get("error", "").lower())

    def test_map_load_success(self):
        """nav2 map load → success when service returns result_code=0."""
        mock_resp = MagicMock()
        mock_resp.result = 0

        with (
            patch("os.path.exists", return_value=True),
            patch("os.path.isabs", return_value=True),
            patch("os.path.abspath", return_value="/maps/office.yaml"),
            patch.object(self.mod, "ros2_context"),
            patch.object(self.mod, "ROS2CLI"),
            patch.object(self.mod, "_call_service", return_value=(mock_resp, None)),
        ):
            # nav2_msgs.srv.LoadMap must be importable (mock it)
            with patch.dict("sys.modules", {"nav2_msgs.srv": MagicMock()}):
                r = self._capture(self.mod.cmd_nav2_map_load, self._ns(name="/maps/office.yaml"))
        self.assertTrue(r.get("success"))
        self.assertEqual(r.get("result_name"), "SUCCESS")

    # ------------------------------------------------------------------ nav2 map delete

    def test_map_delete_requires_confirm(self):
        """nav2 map delete without --confirm → safety interlock."""
        r = self._capture(self.mod.cmd_nav2_map_delete, self._ns(name="office", confirm=False))
        self.assertIn("error", r)
        self.assertIn("--confirm", r.get("hint", ""))

    def test_map_delete_file_not_found(self):
        """nav2 map delete with --confirm → error when map file absent."""
        with patch("os.path.exists", return_value=False):
            r = self._capture(self.mod.cmd_nav2_map_delete,
                              self._ns(name="office", confirm=True))
        self.assertIn("error", r)

    # ------------------------------------------------------------------ nav2 mode get

    def test_mode_get_no_lifecycle_nodes(self):
        """nav2 mode get → mapfree when no managed nodes found."""
        mock_node = MagicMock()
        mock_node.get_service_names_and_types.return_value = []

        with (
            patch.object(self.mod, "ros2_context"),
            patch.object(self.mod, "ROS2CLI", return_value=mock_node),
        ):
            r = self._capture(self.mod.cmd_nav2_mode_get, self._ns())
        self.assertEqual(r.get("mode"), "mapfree")
        self.assertIn("note", r)

    def test_mode_get_mapping(self):
        """nav2 mode get → mapping when slam_toolbox is active."""
        mock_node = MagicMock()
        # Pretend slam_toolbox has a /get_state service
        mock_node.get_service_names_and_types.return_value = [
            ("/slam_toolbox/get_state", ["lifecycle_msgs/srv/GetState"]),
        ]
        mock_state = {"state_id": 3, "state": "active"}

        with (
            patch.object(self.mod, "ros2_context"),
            patch.object(self.mod, "ROS2CLI", return_value=mock_node),
            patch.object(self.mod, "_get_managed_node_state", return_value=mock_state),
        ):
            r = self._capture(self.mod.cmd_nav2_mode_get, self._ns())
        self.assertEqual(r.get("mode"), "mapping")
        self.assertIn("slam", r)

    def test_mode_get_navigation(self):
        """nav2 mode get → navigation when amcl is active."""
        mock_node = MagicMock()
        mock_node.get_service_names_and_types.return_value = [
            ("/amcl/get_state", ["lifecycle_msgs/srv/GetState"]),
        ]
        mock_state = {"state_id": 3, "state": "active"}

        with (
            patch.object(self.mod, "ros2_context"),
            patch.object(self.mod, "ROS2CLI", return_value=mock_node),
            patch.object(self.mod, "_get_managed_node_state", return_value=mock_state),
        ):
            r = self._capture(self.mod.cmd_nav2_mode_get, self._ns())
        self.assertEqual(r.get("mode"), "navigation")

    # ------------------------------------------------------------------ nav2 mode set

    def test_mode_set_invalid(self):
        """nav2 mode set with invalid mode → error."""
        r = self._capture(self.mod.cmd_nav2_mode_set, self._ns(mode="teleop"))
        self.assertIn("error", r)

    def test_mode_set_mapping_activates_slam(self):
        """nav2 mode set mapping → activates slam_toolbox, notes amcl not detected."""
        mock_node = MagicMock()
        mock_node.get_service_names_and_types.return_value = [
            ("/slam_toolbox/get_state", ["lifecycle_msgs/srv/GetState"]),
        ]
        # Initial: slam inactive; after set: slam active
        inactive = {"state_id": 2, "state": "inactive"}
        active   = {"state_id": 3, "state": "active"}

        with (
            patch.object(self.mod, "ros2_context"),
            patch.object(self.mod, "ROS2CLI", return_value=mock_node),
            patch.object(self.mod, "_get_managed_node_state",
                         side_effect=[inactive, inactive, active, None]),
            patch.object(self.mod, "_lifecycle_transition", return_value=(True, "ok")),
        ):
            r = self._capture(self.mod.cmd_nav2_mode_set, self._ns(mode="mapping"))
        self.assertTrue(r.get("success"))
        self.assertEqual(r.get("requested_mode"), "mapping")

    # ------------------------------------------------------------------ parsers

    def test_nav2_map_list_parser(self):
        """nav2 map list parses correctly with default maps-dir."""
        from ros2_cli import build_parser
        p = build_parser()
        args = p.parse_args(["nav2", "map", "list"])
        self.assertEqual(args.subcommand, "map")
        self.assertEqual(args.map_subcommand, "list")
        self.assertEqual(args.maps_dir, "maps")

    def test_nav2_map_load_parser(self):
        """nav2 map load <name> parses name correctly."""
        from ros2_cli import build_parser
        p = build_parser()
        args = p.parse_args(["nav2", "map", "load", "office"])
        self.assertEqual(args.map_subcommand, "load")
        self.assertEqual(args.name, "office")

    def test_nav2_map_delete_parser(self):
        """nav2 map delete --confirm parses correctly."""
        from ros2_cli import build_parser
        p = build_parser()
        args = p.parse_args(["nav2", "map", "delete", "office", "--confirm"])
        self.assertEqual(args.map_subcommand, "delete")
        self.assertEqual(args.name, "office")
        self.assertTrue(args.confirm)

    def test_nav2_mode_get_parser(self):
        """nav2 mode get parses correctly."""
        from ros2_cli import build_parser
        p = build_parser()
        args = p.parse_args(["nav2", "mode", "get"])
        self.assertEqual(args.subcommand, "mode")
        self.assertEqual(args.mode_subcommand, "get")

    def test_nav2_mode_set_parser(self):
        """nav2 mode set mapping parses correctly."""
        from ros2_cli import build_parser
        p = build_parser()
        args = p.parse_args(["nav2", "mode", "set", "mapping"])
        self.assertEqual(args.mode_subcommand, "set")
        self.assertEqual(args.mode, "mapping")

    def test_dispatch_nav2_map_mode(self):
        """DISPATCH has nav2 map and nav2 mode entries."""
        from ros2_cli import DISPATCH
        self.assertIn(("nav2", "map"),  DISPATCH)
        self.assertIn(("nav2", "mode"), DISPATCH)


class TestNav2Localize(unittest.TestCase):
    """Unit tests for nav2 localize command."""

    @classmethod
    def setUpClass(cls):
        import ros2_nav2
        cls.mod = ros2_nav2

    def _capture(self, fn, args):
        import ros2_utils
        buf = StringIO()
        side = lambda d: buf.write(json.dumps(d))  # noqa: E731
        with (
            patch.object(ros2_utils, "output", side_effect=side),
            patch.object(self.mod, "output", side_effect=side),
        ):
            fn(args)
        return json.loads(buf.getvalue()) if buf.getvalue() else {}

    def _ns(self, **kwargs):
        defaults = dict(service=None, timeout=10.0)
        defaults.update(kwargs)
        return argparse.Namespace(**defaults)

    def test_localize_service_not_found(self):
        """nav2 localize → error when service not on graph."""
        mock_node = MagicMock()
        mock_node.get_service_names_and_types.return_value = []

        with (
            patch.object(self.mod, "get_srv_type", return_value=MagicMock()),
            patch.object(self.mod, "ros2_context"),
            patch.object(self.mod, "ROS2CLI", return_value=mock_node),
        ):
            r = self._capture(self.mod.cmd_nav2_localize, self._ns())
        self.assertIn("error", r)
        self.assertIn("available_localization_services", r)

    def test_localize_std_srvs_unavailable(self):
        """nav2 localize → error when std_srvs/Empty cannot be loaded."""
        with patch.object(self.mod, "get_srv_type", return_value=None):
            r = self._capture(self.mod.cmd_nav2_localize, self._ns())
        self.assertIn("error", r)

    def test_localize_parser(self):
        """nav2 localize parses --service and --timeout."""
        from ros2_cli import build_parser
        p = build_parser()
        args = p.parse_args(["nav2", "localize", "--timeout", "15.0"])
        self.assertEqual(args.subcommand, "localize")
        self.assertAlmostEqual(args.timeout, 15.0)
        self.assertIsNone(args.service)

    def test_dispatch_nav2_localize(self):
        """DISPATCH has nav2 localize entry."""
        from ros2_cli import DISPATCH
        self.assertIn(("nav2", "localize"), DISPATCH)


class TestTopicsClassify(unittest.TestCase):
    """Pure-logic tests for _classify_topics and the topics classify command.

    _classify_topics is a pure Python function (no rclpy calls), so these
    tests run without a live ROS 2 environment — the rclpy guard is only
    needed to import ros2_topic safely.
    """

    @classmethod
    def setUpClass(cls):
        if not check_rclpy_available():
            raise unittest.SkipTest("rclpy not available - requires ROS 2 environment")
        import ros2_topic
        cls.mod = ros2_topic

    def _classify(self, topics_and_types):
        return self.mod._classify_topics(topics_and_types)

    # ------------------------------------------------------------------
    # Role classification by type
    # ------------------------------------------------------------------

    def test_motion_twist_by_type(self):
        """Topics with Twist type are classified as motion."""
        result = self._classify([("/cmd_vel", ["geometry_msgs/msg/Twist"])])
        self.assertEqual(result["topics"][0]["role"], "motion")

    def test_motion_twist_stamped_by_type(self):
        """Topics with TwistStamped type are classified as motion."""
        result = self._classify([("/cmd_vel_stamped", ["geometry_msgs/msg/TwistStamped"])])
        self.assertEqual(result["topics"][0]["role"], "motion")

    def test_motion_odometry_by_type(self):
        """Topics with Odometry type are classified as motion."""
        result = self._classify([("/odom", ["nav_msgs/msg/Odometry"])])
        self.assertEqual(result["topics"][0]["role"], "motion")

    def test_sensors_laser_scan_by_type(self):
        """Topics with LaserScan type are classified as sensors."""
        result = self._classify([("/scan", ["sensor_msgs/msg/LaserScan"])])
        self.assertEqual(result["topics"][0]["role"], "sensors")

    def test_sensors_imu_by_type(self):
        """Topics with Imu type are classified as sensors."""
        result = self._classify([("/imu/data", ["sensor_msgs/msg/Imu"])])
        self.assertEqual(result["topics"][0]["role"], "sensors")

    def test_sensors_battery_by_type(self):
        """Battery topics (BatteryState) are classified as sensors."""
        result = self._classify([("/battery_state", ["sensor_msgs/msg/BatteryState"])])
        self.assertEqual(result["topics"][0]["role"], "sensors")

    def test_camera_image_by_type_and_name(self):
        """Topics with Image type and camera-like name are classified as camera."""
        result = self._classify([("/camera/image_raw", ["sensor_msgs/msg/Image"])])
        self.assertEqual(result["topics"][0]["role"], "camera")

    def test_camera_info_by_type(self):
        """Topics with CameraInfo type are classified as camera."""
        result = self._classify([("/camera/camera_info", ["sensor_msgs/msg/CameraInfo"])])
        self.assertEqual(result["topics"][0]["role"], "camera")

    def test_depth_pointcloud_by_type(self):
        """Topics with PointCloud2 type are classified as depth."""
        result = self._classify([("/points", ["sensor_msgs/msg/PointCloud2"])])
        self.assertEqual(result["topics"][0]["role"], "depth")

    def test_depth_image_by_name_and_type(self):
        """Topics with 'depth' in the name and Image type are classified as depth."""
        result = self._classify([("/camera/depth/image_raw", ["sensor_msgs/msg/Image"])])
        self.assertEqual(result["topics"][0]["role"], "depth")

    def test_tf_topic_by_name(self):
        """/tf is classified as tf regardless of type."""
        result = self._classify([("/tf", ["tf2_msgs/msg/TFMessage"])])
        self.assertEqual(result["topics"][0]["role"], "tf")

    def test_tf_static_topic_by_name(self):
        """/tf_static is classified as tf."""
        result = self._classify([("/tf_static", ["tf2_msgs/msg/TFMessage"])])
        self.assertEqual(result["topics"][0]["role"], "tf")

    def test_diagnostics_by_type(self):
        """Topics with DiagnosticArray type are classified as diagnostics."""
        result = self._classify([("/diagnostics", ["diagnostic_msgs/msg/DiagnosticArray"])])
        self.assertEqual(result["topics"][0]["role"], "diagnostics")

    def test_other_unrecognised_type(self):
        """Unrecognised type with unrecognised name falls into other."""
        result = self._classify([("/custom_topic", ["my_pkg/msg/Custom"])])
        self.assertEqual(result["topics"][0]["role"], "other")

    # ------------------------------------------------------------------
    # by_role grouping
    # ------------------------------------------------------------------

    def test_by_role_groups_multiple_topics(self):
        """by_role groups topic names correctly across multiple roles."""
        topics = [
            ("/cmd_vel", ["geometry_msgs/msg/Twist"]),
            ("/scan", ["sensor_msgs/msg/LaserScan"]),
            ("/odom", ["nav_msgs/msg/Odometry"]),
        ]
        result = self._classify(topics)
        self.assertIn("/cmd_vel", result["by_role"]["motion"])
        self.assertIn("/odom", result["by_role"]["motion"])
        self.assertIn("/scan", result["by_role"]["sensors"])

    def test_by_role_empty_roles_present(self):
        """by_role always contains all seven role keys, even if empty."""
        result = self._classify([("/cmd_vel", ["geometry_msgs/msg/Twist"])])
        for role in ("motion", "sensors", "camera", "depth", "diagnostics", "tf", "other"):
            self.assertIn(role, result["by_role"])

    # ------------------------------------------------------------------
    # Capability flags
    # ------------------------------------------------------------------

    def test_capability_can_move_true(self):
        """can_move is True when a Twist (cmd_vel) topic exists."""
        result = self._classify([("/cmd_vel", ["geometry_msgs/msg/Twist"])])
        self.assertTrue(result["capabilities"]["can_move"])

    def test_capability_has_lidar_true(self):
        """has_lidar is True when a LaserScan topic exists."""
        result = self._classify([("/scan", ["sensor_msgs/msg/LaserScan"])])
        self.assertTrue(result["capabilities"]["has_lidar"])

    def test_capability_has_camera_true(self):
        """has_camera is True when a camera Image topic exists."""
        result = self._classify([("/camera/image_raw", ["sensor_msgs/msg/Image"])])
        self.assertTrue(result["capabilities"]["has_camera"])

    def test_capability_has_depth_true(self):
        """has_depth is True when a PointCloud2 topic exists."""
        result = self._classify([("/points", ["sensor_msgs/msg/PointCloud2"])])
        self.assertTrue(result["capabilities"]["has_depth"])

    def test_capability_has_odom_true(self):
        """has_odom is True when an Odometry topic exists."""
        result = self._classify([("/odom", ["nav_msgs/msg/Odometry"])])
        self.assertTrue(result["capabilities"]["has_odom"])

    def test_capability_has_battery_true(self):
        """has_battery is True when a BatteryState topic exists."""
        result = self._classify([("/battery_state", ["sensor_msgs/msg/BatteryState"])])
        self.assertTrue(result["capabilities"]["has_battery"])

    def test_capability_has_nav2_true(self):
        """has_nav2 is True when a navigate_to_pose action topic exists."""
        result = self._classify([
            ("/navigate_to_pose/_action/status", ["action_msgs/msg/GoalStatusArray"])
        ])
        self.assertTrue(result["capabilities"]["has_nav2"])

    def test_capability_has_arm_true(self):
        """has_arm is True when a JointState topic exists."""
        result = self._classify([("/joint_states", ["sensor_msgs/msg/JointState"])])
        self.assertTrue(result["capabilities"]["has_arm"])

    def test_capabilities_all_false_on_empty(self):
        """All capability flags are False when no topics are provided."""
        result = self._classify([])
        for key in ("can_move", "has_lidar", "has_camera", "has_depth",
                    "has_odom", "has_battery", "has_arm", "has_nav2"):
            self.assertFalse(result["capabilities"][key],
                             f"Expected {key}=False for empty input")

    def test_capabilities_false_for_unrelated_topics(self):
        """Capability flags stay False for unrelated topics."""
        result = self._classify([("/custom", ["my_pkg/msg/Custom"])])
        self.assertFalse(result["capabilities"]["can_move"])
        self.assertFalse(result["capabilities"]["has_lidar"])

    # ------------------------------------------------------------------
    # Edge cases
    # ------------------------------------------------------------------

    def test_empty_type_list_does_not_crash(self):
        """Topics with an empty type list are classified as other without error."""
        result = self._classify([("/unknown_topic", [])])
        self.assertEqual(result["topics"][0]["role"], "other")

    def test_result_has_required_keys(self):
        """Result always has topics, by_role, and capabilities keys."""
        result = self._classify([])
        self.assertIn("topics", result)
        self.assertIn("by_role", result)
        self.assertIn("capabilities", result)

    def test_each_topic_entry_has_topic_type_role(self):
        """Each entry in result['topics'] has topic, type, and role keys."""
        result = self._classify([("/cmd_vel", ["geometry_msgs/msg/Twist"])])
        entry = result["topics"][0]
        self.assertIn("topic", entry)
        self.assertIn("type", entry)
        self.assertIn("role", entry)

    def test_topic_type_is_first_type_or_empty(self):
        """Each entry's 'type' field is the first element of the type list, or ''."""
        result = self._classify([("/cmd_vel", ["geometry_msgs/msg/Twist", "other/Type"])])
        self.assertEqual(result["topics"][0]["type"], "geometry_msgs/msg/Twist")

    def test_empty_input_returns_empty_lists(self):
        """Empty input returns empty topics list and empty by_role lists."""
        result = self._classify([])
        self.assertEqual(result["topics"], [])
        for role_list in result["by_role"].values():
            self.assertEqual(role_list, [])

    # ------------------------------------------------------------------
    # Parser and dispatch
    # ------------------------------------------------------------------

    def test_parser_topics_classify(self):
        """'topics classify' parses without error."""
        from ros2_cli import build_parser
        p = build_parser()
        args = p.parse_args(["topics", "classify"])
        self.assertEqual(args.subcommand, "classify")

    def test_dispatch_topics_classify(self):
        """DISPATCH has a ('topics', 'classify') entry."""
        from ros2_cli import DISPATCH
        self.assertIn(("topics", "classify"), DISPATCH)


if __name__ == "__main__":
    unittest.main()
