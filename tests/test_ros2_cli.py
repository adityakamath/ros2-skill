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

    def test_version_command(self):
        args = self.parser.parse_args(["version"])
        self.assertEqual(args.command, "version")

    def test_topics_list(self):
        args = self.parser.parse_args(["topics", "list"])
        self.assertEqual(args.command, "topics")
        self.assertEqual(args.subcommand, "list")

    def test_topics_type(self):
        args = self.parser.parse_args(["topics", "type", "/cmd_vel"])
        self.assertEqual(args.subcommand, "type")
        self.assertEqual(args.topic, "/cmd_vel")

    def test_topics_subscribe_defaults(self):
        args = self.parser.parse_args(["topics", "subscribe", "/scan"])
        self.assertEqual(args.topic, "/scan")
        self.assertIsNone(args.duration)
        self.assertEqual(args.max_messages, 100)

    def test_topics_subscribe_with_duration(self):
        args = self.parser.parse_args([
            "topics", "subscribe", "/odom",
            "--duration", "10", "--max-messages", "50"
        ])
        self.assertEqual(args.duration, 10.0)
        self.assertEqual(args.max_messages, 50)

    def test_topics_publish(self):
        msg = '{"linear":{"x":1.0}}'
        args = self.parser.parse_args(["topics", "publish", "/cmd_vel", msg])
        self.assertEqual(args.topic, "/cmd_vel")
        self.assertEqual(args.msg, msg)
        self.assertIsNone(args.duration)
        self.assertEqual(args.rate, 10.0)
        # with explicit duration and rate
        args2 = self.parser.parse_args([
            "topics", "publish", "/cmd_vel", '{}', "--duration", "3", "--rate", "20"
        ])
        self.assertEqual(args2.duration, 3.0)
        self.assertEqual(args2.rate, 20.0)

    def test_topics_publish_sequence(self):
        msgs = '[{"linear":{"x":1}},{"linear":{"x":0}}]'
        durs = '[2.0, 0.5]'
        args = self.parser.parse_args([
            "topics", "publish-sequence", "/cmd_vel", msgs, durs
        ])
        self.assertEqual(args.subcommand, "publish-sequence")
        self.assertEqual(args.messages, msgs)
        self.assertEqual(args.durations, durs)

    def test_services_call(self):
        args = self.parser.parse_args([
            "services", "call", "/spawn",
            '{"x":3.0,"y":3.0}'
        ])
        self.assertEqual(args.command, "services")
        self.assertEqual(args.subcommand, "call")
        self.assertEqual(args.service, "/spawn")

    def test_nodes_details(self):
        args = self.parser.parse_args(["nodes", "details", "/turtlesim"])
        self.assertEqual(args.subcommand, "details")
        self.assertEqual(args.node, "/turtlesim")

    def test_params_list(self):
        args = self.parser.parse_args(["params", "list", "/turtlesim"])
        self.assertEqual(args.command, "params")
        self.assertEqual(args.node, "/turtlesim")

    def test_params_get(self):
        args = self.parser.parse_args(["params", "get", "/turtlesim:background_r"])
        self.assertEqual(args.name, "/turtlesim:background_r")

    def test_params_set(self):
        args = self.parser.parse_args(["params", "set", "/turtlesim:background_r", "255"])
        self.assertEqual(args.name, "/turtlesim:background_r")
        self.assertEqual(args.value, "255")

    def test_actions_send(self):
        args = self.parser.parse_args([
            "actions", "send", "/turtle1/rotate_absolute",
            '{"theta":3.14}'
        ])
        self.assertEqual(args.action, "/turtle1/rotate_absolute")
        self.assertEqual(args.goal, '{"theta":3.14}')

    def test_lifecycle_nodes(self):
        args = self.parser.parse_args(["lifecycle", "nodes"])
        self.assertEqual(args.command, "lifecycle")
        self.assertEqual(args.subcommand, "nodes")

    def test_lifecycle_list_with_node(self):
        args = self.parser.parse_args(["lifecycle", "list", "/my_lifecycle_node"])
        self.assertEqual(args.command, "lifecycle")
        self.assertEqual(args.subcommand, "list")
        self.assertEqual(args.node, "/my_lifecycle_node")

    def test_lifecycle_list_no_node(self):
        args = self.parser.parse_args(["lifecycle", "list"])
        self.assertEqual(args.subcommand, "list")
        self.assertIsNone(args.node)

    def test_lifecycle_ls_alias(self):
        args = self.parser.parse_args(["lifecycle", "ls", "/my_lifecycle_node"])
        self.assertEqual(args.subcommand, "ls")
        self.assertEqual(args.node, "/my_lifecycle_node")

    def test_lifecycle_get(self):
        args = self.parser.parse_args(["lifecycle", "get", "/my_lifecycle_node"])
        self.assertEqual(args.command, "lifecycle")
        self.assertEqual(args.subcommand, "get")
        self.assertEqual(args.node, "/my_lifecycle_node")

    def test_lifecycle_set_by_label(self):
        args = self.parser.parse_args(["lifecycle", "set", "/my_lifecycle_node", "configure"])
        self.assertEqual(args.command, "lifecycle")
        self.assertEqual(args.subcommand, "set")
        self.assertEqual(args.node, "/my_lifecycle_node")
        self.assertEqual(args.transition, "configure")

    def test_lifecycle_set_by_numeric_id(self):
        args = self.parser.parse_args(["lifecycle", "set", "/my_lifecycle_node", "3"])
        self.assertEqual(args.node, "/my_lifecycle_node")
        self.assertEqual(args.transition, "3")

    def test_lifecycle_default_timeout(self):
        """All lifecycle subcommands default to timeout=5.0."""
        for cmd in [
            ["lifecycle", "list", "/my_lifecycle_node"],
            ["lifecycle", "get", "/my_lifecycle_node"],
            ["lifecycle", "set", "/my_lifecycle_node", "activate"],
        ]:
            self.assertEqual(self.parser.parse_args(cmd).timeout, 5.0)

    def test_lifecycle_list_custom_timeout(self):
        args = self.parser.parse_args(["lifecycle", "list", "/my_lifecycle_node", "--timeout", "10"])
        self.assertEqual(args.timeout, 10.0)

    def test_lifecycle_namespaced_nodes(self):
        args = self.parser.parse_args(["lifecycle", "get", "/robot/camera_driver"])
        self.assertEqual(args.node, "/robot/camera_driver")
        args2 = self.parser.parse_args(["lifecycle", "set", "/robot/sensor_node", "activate"])
        self.assertEqual(args2.node, "/robot/sensor_node")
        self.assertEqual(args2.transition, "activate")

    def test_estop_no_topic(self):
        args = self.parser.parse_args(["estop"])
        self.assertEqual(args.command, "estop")
        self.assertIsNone(args.topic)

    def test_estop_custom_topic(self):
        args = self.parser.parse_args(["estop", "--topic", "/mobile_base/cmd_vel"])
        self.assertEqual(args.topic, "/mobile_base/cmd_vel")

    def test_services_echo_defaults(self):
        args = self.parser.parse_args(["services", "echo", "/add_two_ints"])
        self.assertEqual(args.command, "services")
        self.assertEqual(args.subcommand, "echo")
        self.assertEqual(args.service, "/add_two_ints")
        self.assertIsNone(args.duration)
        self.assertIsNone(args.max_messages)
        self.assertEqual(args.timeout, 30.0)

    def test_services_echo_with_options(self):
        args = self.parser.parse_args([
            "services", "echo", "/my_svc",
            "--duration", "5", "--max-messages", "10", "--timeout", "20"
        ])
        self.assertEqual(args.duration, 5.0)
        self.assertEqual(args.max_messages, 10)
        self.assertEqual(args.timeout, 20.0)

    def test_actions_echo_defaults(self):
        args = self.parser.parse_args(["actions", "echo", "/turtle1/rotate_absolute"])
        self.assertEqual(args.command, "actions")
        self.assertEqual(args.subcommand, "echo")
        self.assertEqual(args.action, "/turtle1/rotate_absolute")
        self.assertIsNone(args.duration)
        self.assertEqual(args.max_messages, 100)
        self.assertEqual(args.timeout, 5.0)

    def test_actions_echo_with_options(self):
        args = self.parser.parse_args([
            "actions", "echo", "/navigate",
            "--duration", "10", "--max-messages", "50", "--timeout", "15"
        ])
        self.assertEqual(args.duration, 10.0)
        self.assertEqual(args.max_messages, 50)
        self.assertEqual(args.timeout, 15.0)

    def test_actions_find_defaults(self):
        args = self.parser.parse_args(["actions", "find", "turtlesim/action/RotateAbsolute"])
        self.assertEqual(args.command, "actions")
        self.assertEqual(args.subcommand, "find")
        self.assertEqual(args.action_type, "turtlesim/action/RotateAbsolute")
        self.assertEqual(args.timeout, 5.0)

    def test_actions_find_custom_timeout(self):
        args = self.parser.parse_args(["actions", "find", "nav2_msgs/action/NavigateToPose", "--timeout", "10"])
        self.assertEqual(args.timeout, 10.0)

    def test_params_describe_colon_format(self):
        args = self.parser.parse_args(["params", "describe", "/turtlesim:background_r"])
        self.assertEqual(args.command, "params")
        self.assertEqual(args.subcommand, "describe")
        self.assertEqual(args.name, "/turtlesim:background_r")
        self.assertIsNone(args.param_name)
        self.assertEqual(args.timeout, 5.0)

    def test_params_describe_two_arg_format(self):
        args = self.parser.parse_args(["params", "describe", "/turtlesim", "background_r"])
        self.assertEqual(args.name, "/turtlesim")
        self.assertEqual(args.param_name, "background_r")

    def test_params_dump(self):
        args = self.parser.parse_args(["params", "dump", "/turtlesim"])
        self.assertEqual(args.command, "params")
        self.assertEqual(args.subcommand, "dump")
        self.assertEqual(args.node, "/turtlesim")
        self.assertEqual(args.timeout, 5.0)

    def test_params_load_json_string(self):
        args = self.parser.parse_args(["params", "load", "/turtlesim", '{"background_r":255}'])
        self.assertEqual(args.command, "params")
        self.assertEqual(args.subcommand, "load")
        self.assertEqual(args.node, "/turtlesim")
        self.assertEqual(args.params, '{"background_r":255}')

    def test_params_delete_single(self):
        args = self.parser.parse_args(["params", "delete", "/turtlesim:background_r"])
        self.assertEqual(args.command, "params")
        self.assertEqual(args.subcommand, "delete")
        self.assertEqual(args.name, "/turtlesim:background_r")
        self.assertEqual(args.extra_names, [])

    def test_params_delete_two_arg_format(self):
        args = self.parser.parse_args(["params", "delete", "/turtlesim", "background_r"])
        self.assertEqual(args.name, "/turtlesim")
        self.assertEqual(args.param_name, "background_r")


class TestDispatchTable(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        if not check_rclpy_available():
            raise unittest.SkipTest("rclpy not available - requires ROS 2 environment")
        import ros2_cli
        cls.ros2_cli = ros2_cli

    def test_all_keys_have_callable_handlers(self):
        for key, handler in self.ros2_cli.DISPATCH.items():
            self.assertTrue(callable(handler), f"{key} handler is not callable")

    def test_expected_keys_exist(self):
        expected_keys = [
            ("version", None),
            ("topics", "list"), ("topics", "type"), ("topics", "details"),
            ("topics", "message"), ("topics", "subscribe"), ("topics", "publish"),
            ("topics", "publish-sequence"), ("topics", "publish-until"),
            ("topics", "bw"), ("topics", "delay"),
            ("services", "list"), ("services", "type"), ("services", "details"),
            ("services", "call"), ("services", "echo"),
            ("nodes", "list"), ("nodes", "details"),
            ("params", "list"), ("params", "get"), ("params", "set"),
            ("params", "describe"), ("params", "dump"), ("params", "load"),
            ("params", "delete"),
            ("actions", "list"), ("actions", "details"), ("actions", "send"),
            ("actions", "echo"), ("actions", "find"),
            # lifecycle
            ("lifecycle", "nodes"), ("lifecycle", "list"), ("lifecycle", "ls"),
            ("lifecycle", "get"), ("lifecycle", "set"),
            # interface
            ("interface", "list"), ("interface", "ls"), ("interface", "show"),
            ("interface", "proto"),
            ("interface", "packages"), ("interface", "package"),
            # params presets
            ("params", "preset-save"), ("params", "preset-load"),
            ("params", "preset-list"), ("params", "preset-delete"),
            # tf
            ("tf", "list"), ("tf", "ls"), ("tf", "lookup"), ("tf", "get"),
            ("tf", "echo"), ("tf", "monitor"), ("tf", "static"),
            ("tf", "euler-from-quaternion"), ("tf", "e2q"), ("tf", "quat2euler"),
            ("tf", "quaternion-from-euler"), ("tf", "q2e"), ("tf", "euler2quat"),
            ("tf", "euler-from-quaternion-deg"), ("tf", "e2qdeg"),
            ("tf", "quaternion-from-euler-deg"), ("tf", "q2edeg"),
            ("tf", "transform-point"), ("tf", "tp"), ("tf", "point"),
            ("tf", "transform-vector"), ("tf", "tv"), ("tf", "vector"),
            # launch
            ("launch", "new"), ("launch", "list"), ("launch", "ls"),
            ("launch", "kill"), ("launch", "restart"), ("launch", "foxglove"),
            # run
            ("run", "new"), ("run", "list"), ("run", "ls"),
            ("run", "kill"), ("run", "restart"),
            # control — canonical
            ("control", "list-controller-types"), ("control", "list-controllers"),
            ("control", "list-hardware-components"), ("control", "list-hardware-interfaces"),
            ("control", "load-controller"), ("control", "unload-controller"),
            ("control", "configure-controller"), ("control", "reload-controller-libraries"),
            ("control", "set-controller-state"), ("control", "set-hardware-component-state"),
            ("control", "switch-controllers"), ("control", "view-controller-chains"),
            # control — aliases
            ("control", "lct"), ("control", "lc"), ("control", "lhc"), ("control", "lhi"),
            ("control", "load"), ("control", "unload"), ("control", "cc"), ("control", "rcl"),
            ("control", "scs"), ("control", "shcs"), ("control", "sc"), ("control", "swc"),
            ("control", "vcc"),
        ]
        for key in expected_keys:
            self.assertIn(key, self.ros2_cli.DISPATCH, f"Missing dispatch key: {key}")

    def test_dispatch_count(self):
        # Updated count to reflect all canonical + alias entries + Phase 2 commands
        self.assertGreater(len(self.ros2_cli.DISPATCH), 50)


class TestOutput(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        if not check_rclpy_available():
            raise unittest.SkipTest("rclpy not available - requires ROS 2 environment")
        import ros2_cli
        cls.ros2_cli = ros2_cli

    def test_output_prints_json(self):
        with patch("sys.stdout", new_callable=StringIO) as mock_stdout:
            self.ros2_cli.output({"key": "value"})
            result = json.loads(mock_stdout.getvalue())
            self.assertEqual(result, {"key": "value"})

    def test_output_unicode(self):
        with patch("sys.stdout", new_callable=StringIO) as mock_stdout:
            self.ros2_cli.output({"msg": "로봇"})
            result = json.loads(mock_stdout.getvalue())
            self.assertEqual(result["msg"], "로봇")

    def test_output_nested(self):
        data = {"a": {"b": [1, 2, 3]}}
        with patch("sys.stdout", new_callable=StringIO) as mock_stdout:
            self.ros2_cli.output(data)
            result = json.loads(mock_stdout.getvalue())
            self.assertEqual(result, data)


class TestMsgConversion(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        if not check_rclpy_available():
            raise unittest.SkipTest("rclpy not available - requires ROS 2 environment")
        import ros2_cli
        cls.ros2_cli = ros2_cli

    def test_msg_to_dict(self):
        mock_msg = MagicMock()
        mock_msg.get_fields_and_field_types.return_value = ["field1", "field2"]
        mock_msg.field1 = "value1"
        mock_msg.field2 = 42
        
        result = self.ros2_cli.msg_to_dict(mock_msg)
        self.assertEqual(result, {"field1": "value1", "field2": 42})

    def test_dict_to_msg(self):
        mock_class = MagicMock()
        mock_instance = MagicMock()
        mock_class.return_value = mock_instance
        
        result = self.ros2_cli.dict_to_msg(mock_class, {"key": "value"})
        self.assertEqual(result, mock_instance)
        mock_instance.key = "value"


class TestParseNodeParam(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        if not check_rclpy_available():
            raise unittest.SkipTest("rclpy not available - requires ROS 2 environment")
        import ros2_cli
        cls.ros2_cli = ros2_cli

    def test_with_colon(self):
        node, param = self.ros2_cli.parse_node_param("/turtlesim:background_r")
        self.assertEqual(node, "/turtlesim")
        self.assertEqual(param, "background_r")

    def test_without_colon(self):
        node, param = self.ros2_cli.parse_node_param("/turtlesim")
        self.assertEqual(node, "/turtlesim")
        self.assertIsNone(param)


class TestMessageTypeAliases(unittest.TestCase):
    """Test message type aliases functionality.
    
    These tests verify the MSG_ALIASES dictionary and get_msg_type() function.
    Tests are divided into two categories:
    
    1. Pure Python tests (no ROS packages required):
       - Dictionary structure and content validation
       - Alias format and naming conventions
       - Basic logic tests with None/empty inputs
    
    2. Integration tests (require ROS message packages):
       - Actual message type imports via get_msg_type()
       - These handle missing packages gracefully (return None)
       - Entire test class skips if rclpy is not available
    """
    
    @classmethod
    def setUpClass(cls):
        if not check_rclpy_available():
            raise unittest.SkipTest("rclpy not available - requires ROS 2 environment")
        import ros2_cli
        cls.ros2_cli = ros2_cli

    def test_alias_exists_in_dict(self):
        """Test that MSG_ALIASES dictionary exists and has expected aliases."""
        self.assertIn('twist', self.ros2_cli.MSG_ALIASES)
        self.assertIn('odom', self.ros2_cli.MSG_ALIASES)
        self.assertIn('laserscan', self.ros2_cli.MSG_ALIASES)
        self.assertIn('image', self.ros2_cli.MSG_ALIASES)
        self.assertEqual(self.ros2_cli.MSG_ALIASES['twist'], 'geometry_msgs/Twist')
        self.assertEqual(self.ros2_cli.MSG_ALIASES['odom'], 'nav_msgs/Odometry')

    def test_get_msg_type_with_alias(self):
        """Test that aliases resolve to correct message types."""
        # Test with a common alias that should be available in most ROS 2 installs
        result = self.ros2_cli.get_msg_type('twist')
        # If geometry_msgs is installed, should return a class; otherwise None
        # Either way, should not raise an exception
        self.assertTrue(result is None or callable(getattr(result, '__init__', None)),
                       "Result should be None or a message class")

    def test_get_msg_type_case_insensitive(self):
        """Test that aliases are case-insensitive."""
        # The function should handle TWIST, Twist, twist, etc.
        lower = self.ros2_cli.get_msg_type('twist')
        upper = self.ros2_cli.get_msg_type('TWIST')
        mixed = self.ros2_cli.get_msg_type('Twist')
        # All should return the same result (either all None or all the same class)
        self.assertEqual(type(lower), type(upper),
                        "Case variations should return same type")
        self.assertEqual(type(lower), type(mixed),
                        "Case variations should return same type")
        # If one is not None, all should be identical
        if lower is not None:
            self.assertIs(lower, upper, "Should return identical class object")
            self.assertIs(lower, mixed, "Should return identical class object")

    def test_get_msg_type_non_alias_formats(self):
        """Full type names and /msg/ format return None or valid class without crashing."""
        for type_str in ['geometry_msgs/Twist', 'geometry_msgs/msg/Twist',
                         'std_msgs/msg/String', 'some_random_package/RandomMsg']:
            result = self.ros2_cli.get_msg_type(type_str)
            self.assertTrue(result is None or callable(getattr(result, '__init__', None)),
                            f"Expected None or callable for: {type_str}")

    def test_get_msg_type_edge_cases(self):
        """None, empty string, and unknown aliases all return None."""
        for input_val in [None, '', 'nonexistent_alias_xyz']:
            self.assertIsNone(self.ros2_cli.get_msg_type(input_val),
                              f"Expected None for input: {input_val!r}")

    def test_alias_count(self):
        """Test that we have the expected number of aliases."""
        # We defined 50 aliases
        self.assertEqual(len(self.ros2_cli.MSG_ALIASES), 50)

    def test_alias_format_constraints(self):
        """All aliases: lowercase keys, no '/', map to package/Name without /msg/."""
        for alias, full_type in self.ros2_cli.MSG_ALIASES.items():
            self.assertEqual(alias, alias.lower(),
                             f"Alias key should be lowercase: '{alias}'")
            self.assertNotIn('/', alias,
                             f"Alias key should not contain '/': '{alias}'")
            self.assertIn('/', full_type,
                          f"Alias '{alias}' maps to invalid format: '{full_type}'")
            self.assertNotIn('/msg/', full_type,
                             f"Alias '{alias}' should not include /msg/: '{full_type}'")

    def test_all_alias_categories_present(self):
        """Test that we have aliases from all expected ROS 2 message packages."""
        expected_packages = [
            'std_msgs', 'geometry_msgs', 'sensor_msgs', 'nav_msgs',
            'visualization_msgs', 'action_msgs', 'trajectory_msgs'
        ]
        found_packages = set()
        for full_type in self.ros2_cli.MSG_ALIASES.values():
            package = full_type.split('/')[0]
            found_packages.add(package)
        
        for expected_pkg in expected_packages:
            self.assertIn(expected_pkg, found_packages,
                         f"No aliases found for package: {expected_pkg}")

    def test_duplicate_aliases_for_same_type(self):
        """Test that duplicate aliases (like odom/odometry) point to same type."""
        # Both 'odom' and 'odometry' should point to nav_msgs/Odometry
        self.assertEqual(self.ros2_cli.MSG_ALIASES['odom'],
                        self.ros2_cli.MSG_ALIASES['odometry'])
        self.assertEqual(self.ros2_cli.MSG_ALIASES['odom'], 'nav_msgs/Odometry')

    def test_specific_package_aliases(self):
        """Test that we have expected aliases for each package category."""
        # std_msgs
        std_msgs_aliases = ['string', 'int32', 'int64', 'uint8', 'float32', 'float64', 
                           'bool', 'header', 'empty', 'colorrgba']
        for alias in std_msgs_aliases:
            self.assertIn(alias, self.ros2_cli.MSG_ALIASES)
            self.assertTrue(self.ros2_cli.MSG_ALIASES[alias].startswith('std_msgs/'))
        
        # geometry_msgs
        geom_aliases = ['twist', 'pose', 'posearray', 'point', 'pointstamped', 'quaternion', 'vector3',
                       'posestamped', 'twiststamped', 'transform', 'transformstamped',
                       'polygon', 'polygonstamped']
        for alias in geom_aliases:
            self.assertIn(alias, self.ros2_cli.MSG_ALIASES)
            self.assertTrue(self.ros2_cli.MSG_ALIASES[alias].startswith('geometry_msgs/'))
        
        # sensor_msgs
        sensor_aliases = ['laserscan', 'image', 'compressedimage', 'pointcloud2',
                         'imu', 'camerainfo', 'jointstate', 'navsatfix', 
                         'fluidpressure', 'magneticfield']
        for alias in sensor_aliases:
            self.assertIn(alias, self.ros2_cli.MSG_ALIASES)
            self.assertTrue(self.ros2_cli.MSG_ALIASES[alias].startswith('sensor_msgs/'))
        
        # nav_msgs
        nav_aliases = ['odom', 'odometry', 'path', 'occupancygrid', 'gridcells']
        for alias in nav_aliases:
            self.assertIn(alias, self.ros2_cli.MSG_ALIASES)
            self.assertTrue(self.ros2_cli.MSG_ALIASES[alias].startswith('nav_msgs/'))


class TestLifecycleParsing(unittest.TestCase):
    """Lifecycle-specific structural and dispatch-wiring tests.

    Parser argument tests live in TestBuildParser (the canonical location for
    all parser coverage).  This class holds the remaining tests that are unique
    to lifecycle: structural edge cases and DISPATCH table wiring.

    All tests gracefully skip if ROS 2 / rclpy is not available, matching
    the pattern used throughout this test suite.
    """

    @classmethod
    def setUpClass(cls):
        if not check_rclpy_available():
            raise unittest.SkipTest("rclpy not available - requires ROS 2 environment")
        import ros2_cli
        cls.ros2_cli = ros2_cli
        cls.parser = ros2_cli.build_parser()

    # ------------------------------------------------------------------
    # Structural edge cases
    # ------------------------------------------------------------------

    def test_lifecycle_nodes_no_extra_args(self):
        """nodes subcommand exposes no 'node' attribute (or None) on its namespace."""
        args = self.parser.parse_args(["lifecycle", "nodes"])
        self.assertIsNone(getattr(args, "node", None))

    def test_lifecycle_nodes_rejects_extra_positional(self):
        """nodes subcommand must reject extra positional arguments."""
        with self.assertRaises(SystemExit):
            self.parser.parse_args(["lifecycle", "nodes", "/extra"])

    # ------------------------------------------------------------------
    # Dispatch table wiring
    # ------------------------------------------------------------------

    def test_lifecycle_dispatch_keys_present(self):
        """All five lifecycle (category, subcommand) keys must be in DISPATCH."""
        expected = [
            ("lifecycle", "nodes"),
            ("lifecycle", "list"),
            ("lifecycle", "ls"),
            ("lifecycle", "get"),
            ("lifecycle", "set"),
        ]
        for key in expected:
            self.assertIn(key, self.ros2_cli.DISPATCH,
                          f"Missing DISPATCH key: {key}")

    def test_lifecycle_ls_maps_to_same_handler_as_list(self):
        """ls alias must route to the same handler as list."""
        self.assertIs(
            self.ros2_cli.DISPATCH[("lifecycle", "ls")],
            self.ros2_cli.DISPATCH[("lifecycle", "list")],
        )

    def test_lifecycle_handlers_are_callable(self):
        lifecycle_keys = [k for k in self.ros2_cli.DISPATCH if k[0] == "lifecycle"]
        self.assertTrue(len(lifecycle_keys) > 0, "No lifecycle keys found in DISPATCH")
        for key in lifecycle_keys:
            self.assertTrue(callable(self.ros2_cli.DISPATCH[key]),
                            f"Handler for {key} is not callable")


class TestDoctorParsing(unittest.TestCase):
    """Parser argument and DISPATCH wiring tests for the doctor / wtf commands.

    All tests gracefully skip if ROS 2 / rclpy is not available, matching
    the pattern used throughout this test suite.
    """

    @classmethod
    def setUpClass(cls):
        if not check_rclpy_available():
            raise unittest.SkipTest("rclpy not available - requires ROS 2 environment")
        import ros2_cli
        cls.ros2_cli = ros2_cli
        cls.parser = ros2_cli.build_parser()

    # ------------------------------------------------------------------
    # doctor — top-level flags default values
    # ------------------------------------------------------------------

    def test_doctor_no_subcommand(self):
        args = self.parser.parse_args(["doctor"])
        self.assertEqual(args.command, "doctor")
        self.assertIsNone(args.subcommand)

    def test_doctor_report_flags(self):
        self.assertTrue(self.parser.parse_args(["doctor", "--report"]).report)
        self.assertTrue(self.parser.parse_args(["doctor", "-r"]).report)

    def test_doctor_report_failed_flags(self):
        self.assertTrue(self.parser.parse_args(["doctor", "--report-failed"]).report_failed)
        self.assertTrue(self.parser.parse_args(["doctor", "-rf"]).report_failed)

    def test_doctor_exclude_packages_flags(self):
        self.assertTrue(self.parser.parse_args(["doctor", "--exclude-packages"]).exclude_packages)
        self.assertTrue(self.parser.parse_args(["doctor", "-ep"]).exclude_packages)

    def test_doctor_include_warnings_flags(self):
        self.assertTrue(self.parser.parse_args(["doctor", "--include-warnings"]).include_warnings)
        self.assertTrue(self.parser.parse_args(["doctor", "-iw"]).include_warnings)

    def test_doctor_default_flags_are_false(self):
        args = self.parser.parse_args(["doctor"])
        self.assertFalse(args.report)
        self.assertFalse(args.report_failed)
        self.assertFalse(args.exclude_packages)
        self.assertFalse(args.include_warnings)

    # ------------------------------------------------------------------
    # doctor hello subcommand
    # ------------------------------------------------------------------

    def test_doctor_hello(self):
        """doctor/wtf hello subcommand is wired up for both aliases."""
        for cmd in ["doctor", "wtf"]:
            args = self.parser.parse_args([cmd, "hello"])
            self.assertEqual(args.command, cmd)
            self.assertEqual(args.subcommand, "hello")

    def test_doctor_hello_defaults(self):
        args = self.parser.parse_args(["doctor", "hello"])
        self.assertEqual(args.topic, "/canyouhearme")
        self.assertEqual(args.timeout, 10.0)

    def test_doctor_hello_options(self):
        self.assertEqual(
            self.parser.parse_args(["doctor", "hello", "--topic", "/my_hello"]).topic,
            "/my_hello")
        self.assertEqual(
            self.parser.parse_args(["doctor", "hello", "-t", "/other"]).topic,
            "/other")
        self.assertEqual(
            self.parser.parse_args(["doctor", "hello", "--timeout", "5"]).timeout,
            5.0)

    # ------------------------------------------------------------------
    # wtf — mirrors doctor exactly
    # ------------------------------------------------------------------

    def test_wtf_no_subcommand(self):
        args = self.parser.parse_args(["wtf"])
        self.assertEqual(args.command, "wtf")
        self.assertIsNone(args.subcommand)

    # ------------------------------------------------------------------
    # DISPATCH table wiring
    # ------------------------------------------------------------------

    def test_doctor_dispatch_keys_present(self):
        expected = [
            ("doctor", None),
            ("doctor", "hello"),
            ("wtf",    None),
            ("wtf",    "hello"),
        ]
        for key in expected:
            self.assertIn(key, self.ros2_cli.DISPATCH,
                          f"Missing DISPATCH key: {key}")

    def test_wtf_aliases_same_handlers(self):
        """wtf and wtf hello must route to the same handlers as doctor equivalents."""
        self.assertIs(self.ros2_cli.DISPATCH[("wtf", None)],
                      self.ros2_cli.DISPATCH[("doctor", None)])
        self.assertIs(self.ros2_cli.DISPATCH[("wtf", "hello")],
                      self.ros2_cli.DISPATCH[("doctor", "hello")])

    def test_doctor_handlers_are_callable(self):
        for key in [("doctor", None), ("doctor", "hello"),
                    ("wtf", None),    ("wtf", "hello")]:
            self.assertTrue(callable(self.ros2_cli.DISPATCH[key]),
                            f"Handler for {key} is not callable")


class TestMulticastParsing(unittest.TestCase):
    """Parser argument and DISPATCH wiring tests for the multicast commands.

    All tests gracefully skip if ROS 2 / rclpy is not available, matching
    the pattern used throughout this test suite.
    """

    @classmethod
    def setUpClass(cls):
        if not check_rclpy_available():
            raise unittest.SkipTest("rclpy not available - requires ROS 2 environment")
        import ros2_cli
        cls.ros2_cli = ros2_cli
        cls.parser = ros2_cli.build_parser()

    def test_multicast_send_command_and_defaults(self):
        args = self.parser.parse_args(["multicast", "send"])
        self.assertEqual(args.command, "multicast")
        self.assertEqual(args.subcommand, "send")
        self.assertEqual(args.group, "225.0.0.1")
        self.assertEqual(args.port, 49150)

    def test_multicast_send_custom_group(self):
        self.assertEqual(
            self.parser.parse_args(["multicast", "send", "--group", "239.0.0.1"]).group,
            "239.0.0.1")
        self.assertEqual(
            self.parser.parse_args(["multicast", "send", "-g", "239.0.0.1"]).group,
            "239.0.0.1")

    def test_multicast_send_custom_port(self):
        self.assertEqual(
            self.parser.parse_args(["multicast", "send", "--port", "12345"]).port, 12345)
        self.assertEqual(
            self.parser.parse_args(["multicast", "send", "-p", "12345"]).port, 12345)

    def test_multicast_receive_command_and_defaults(self):
        args = self.parser.parse_args(["multicast", "receive"])
        self.assertEqual(args.command, "multicast")
        self.assertEqual(args.subcommand, "receive")
        self.assertEqual(args.group, "225.0.0.1")
        self.assertEqual(args.port, 49150)
        self.assertEqual(args.timeout, 5.0)

    def test_multicast_receive_custom_timeout(self):
        self.assertEqual(
            self.parser.parse_args(["multicast", "receive", "--timeout", "10"]).timeout, 10.0)
        self.assertEqual(
            self.parser.parse_args(["multicast", "receive", "-t", "3.5"]).timeout, 3.5)

    def test_multicast_receive_custom_group_and_port(self):
        self.assertEqual(
            self.parser.parse_args(["multicast", "receive", "--group", "239.0.0.1"]).group,
            "239.0.0.1")
        self.assertEqual(
            self.parser.parse_args(["multicast", "receive", "--port", "9999"]).port, 9999)

    def test_multicast_dispatch_wiring(self):
        """Both keys present, callable, and mapped to distinct handlers."""
        for key in [("multicast", "send"), ("multicast", "receive")]:
            self.assertIn(key, self.ros2_cli.DISPATCH)
            self.assertTrue(callable(self.ros2_cli.DISPATCH[key]))
        self.assertIsNot(
            self.ros2_cli.DISPATCH[("multicast", "send")],
            self.ros2_cli.DISPATCH[("multicast", "receive")])


class TestInterfaceParsing(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        if not check_rclpy_available():
            raise unittest.SkipTest("rclpy not available - requires ROS 2 environment")
        import ros2_cli
        cls.ros2_cli = ros2_cli

    def setUp(self):
        self.parser = self.ros2_cli.build_parser()

    def test_interface_list(self):
        args = self.parser.parse_args(["interface", "list"])
        self.assertEqual(args.command, "interface")
        self.assertEqual(args.subcommand, "list")

    def test_interface_show_message(self):
        args = self.parser.parse_args(["interface", "show", "std_msgs/msg/String"])
        self.assertEqual(args.command, "interface")
        self.assertEqual(args.subcommand, "show")
        self.assertEqual(args.type_str, "std_msgs/msg/String")

    def test_interface_show_service(self):
        args = self.parser.parse_args(["interface", "show", "std_srvs/srv/SetBool"])
        self.assertEqual(args.subcommand, "show")
        self.assertEqual(args.type_str, "std_srvs/srv/SetBool")

    def test_interface_show_action(self):
        args = self.parser.parse_args(["interface", "show", "nav2_msgs/action/NavigateToPose"])
        self.assertEqual(args.subcommand, "show")
        self.assertEqual(args.type_str, "nav2_msgs/action/NavigateToPose")

    def test_interface_show_shorthand(self):
        args = self.parser.parse_args(["interface", "show", "std_msgs/String"])
        self.assertEqual(args.type_str, "std_msgs/String")

    def test_interface_proto(self):
        args = self.parser.parse_args(["interface", "proto", "std_msgs/msg/String"])
        self.assertEqual(args.command, "interface")
        self.assertEqual(args.subcommand, "proto")
        self.assertEqual(args.type_str, "std_msgs/msg/String")

    def test_interface_packages(self):
        args = self.parser.parse_args(["interface", "packages"])
        self.assertEqual(args.command, "interface")
        self.assertEqual(args.subcommand, "packages")

    def test_interface_package(self):
        args = self.parser.parse_args(["interface", "package", "std_msgs"])
        self.assertEqual(args.command, "interface")
        self.assertEqual(args.subcommand, "package")
        self.assertEqual(args.package, "std_msgs")


class TestPresetParsing(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        if not check_rclpy_available():
            raise unittest.SkipTest("rclpy not available - requires ROS 2 environment")
        import ros2_cli
        cls.ros2_cli = ros2_cli

    def setUp(self):
        self.parser = self.ros2_cli.build_parser()

    def test_preset_save_args(self):
        args = self.parser.parse_args(["params", "preset-save", "/turtlesim", "indoor"])
        self.assertEqual(args.command, "params")
        self.assertEqual(args.subcommand, "preset-save")
        self.assertEqual(args.node, "/turtlesim")
        self.assertEqual(args.preset, "indoor")
        self.assertEqual(args.timeout, 5.0)

    def test_preset_save_custom_timeout(self):
        args = self.parser.parse_args(
            ["params", "preset-save", "/turtlesim", "indoor", "--timeout", "10"]
        )
        self.assertEqual(args.timeout, 10.0)

    def test_preset_load_args(self):
        args = self.parser.parse_args(["params", "preset-load", "/turtlesim", "indoor"])
        self.assertEqual(args.command, "params")
        self.assertEqual(args.subcommand, "preset-load")
        self.assertEqual(args.node, "/turtlesim")
        self.assertEqual(args.preset, "indoor")

    def test_preset_list_no_args(self):
        args = self.parser.parse_args(["params", "preset-list"])
        self.assertEqual(args.command, "params")
        self.assertEqual(args.subcommand, "preset-list")
        # preset-list takes no arguments (flat storage: no node filter)
        self.assertFalse(hasattr(args, "node"))

    def test_preset_delete_args(self):
        # Flat storage: preset-delete only needs the preset name, no node arg
        args = self.parser.parse_args(["params", "preset-delete", "indoor"])
        self.assertEqual(args.command, "params")
        self.assertEqual(args.subcommand, "preset-delete")
        self.assertEqual(args.preset, "indoor")
        self.assertFalse(hasattr(args, "node"))

    def test_preset_save_and_load_are_different_handlers(self):
        self.assertIsNot(
            self.ros2_cli.DISPATCH[("params", "preset-save")],
            self.ros2_cli.DISPATCH[("params", "preset-load")],
        )


class TestDiagnosticsParsing(unittest.TestCase):
    """Parser argument and DISPATCH wiring tests for topics diag-list and topics diag."""

    @classmethod
    def setUpClass(cls):
        if not check_rclpy_available():
            raise unittest.SkipTest("rclpy not available - requires ROS 2 environment")
        import ros2_cli
        cls.ros2_cli = ros2_cli
        cls.parser = ros2_cli.build_parser()

    # ------------------------------------------------------------------
    # diag-list
    # ------------------------------------------------------------------

    def test_diag_list_command(self):
        args = self.parser.parse_args(["topics", "diag-list"])
        self.assertEqual(args.command, "topics")
        self.assertEqual(args.subcommand, "diag-list")
        self.assertFalse(hasattr(args, "topic"))  # diag-list takes no topic arg

    # ------------------------------------------------------------------
    # diag
    # ------------------------------------------------------------------

    def test_diag_defaults(self):
        args = self.parser.parse_args(["topics", "diag"])
        self.assertEqual(args.command, "topics")
        self.assertEqual(args.subcommand, "diag")
        self.assertIsNone(args.topic)
        self.assertEqual(args.timeout, 10.0)
        self.assertIsNone(args.duration)
        self.assertEqual(args.max_messages, 1)

    def test_diag_with_topic(self):
        self.assertEqual(
            self.parser.parse_args(["topics", "diag", "--topic", "/diagnostics"]).topic,
            "/diagnostics")
        self.assertEqual(
            self.parser.parse_args(
                ["topics", "diag", "--topic", "/robot/camera/diagnostics"]).topic,
            "/robot/camera/diagnostics")

    def test_diag_custom_options(self):
        self.assertEqual(
            self.parser.parse_args(["topics", "diag", "--timeout", "5"]).timeout, 5.0)
        self.assertEqual(
            self.parser.parse_args(["topics", "diag", "--duration", "3.5"]).duration, 3.5)
        self.assertEqual(
            self.parser.parse_args(["topics", "diag", "--max-messages", "5"]).max_messages, 5)

    def test_diag_combined_options(self):
        args = self.parser.parse_args([
            "topics", "diag",
            "--topic", "/my_node/diagnostics",
            "--duration", "10",
            "--max-messages", "20",
        ])
        self.assertEqual(args.topic, "/my_node/diagnostics")
        self.assertEqual(args.duration, 10.0)
        self.assertEqual(args.max_messages, 20)

    # ------------------------------------------------------------------
    # DISPATCH wiring + DIAG_TYPES constant
    # ------------------------------------------------------------------

    def test_diag_dispatch_wiring(self):
        """Both keys present, callable, and mapped to distinct handlers."""
        for key in [("topics", "diag-list"), ("topics", "diag")]:
            self.assertIn(key, self.ros2_cli.DISPATCH,
                          f"Missing DISPATCH key: {key}")
            self.assertTrue(callable(self.ros2_cli.DISPATCH[key]),
                            f"Handler for {key} is not callable")
        self.assertIsNot(
            self.ros2_cli.DISPATCH[("topics", "diag-list")],
            self.ros2_cli.DISPATCH[("topics", "diag")])

    def test_diag_types_constant(self):
        import ros2_topic
        self.assertTrue(hasattr(ros2_topic, "DIAG_TYPES"))
        self.assertIn("diagnostic_msgs/msg/DiagnosticArray", ros2_topic.DIAG_TYPES)
        self.assertIn("diagnostic_msgs/DiagnosticArray", ros2_topic.DIAG_TYPES)

    # ------------------------------------------------------------------
    # _parse_diag_array helper (pure Python — no rclpy needed for logic)
    # ------------------------------------------------------------------

    def test_parse_diag_array_levels(self):
        """All four diagnostic levels (OK/WARN/ERROR/STALE) map to correct names."""
        import ros2_topic
        for level, expected_name in [(0, "OK"), (1, "WARN"), (2, "ERROR"), (3, "STALE")]:
            msg = {"status": [{"level": level, "name": "test", "message": "",
                                "hardware_id": "", "values": []}]}
            result = ros2_topic._parse_diag_array(msg)
            self.assertEqual(result[0]["level_name"], expected_name)

    def test_parse_diag_array_empty(self):
        import ros2_topic
        self.assertEqual(ros2_topic._parse_diag_array({"status": []}), [])

    def test_parse_diag_array_multiple_statuses(self):
        import ros2_topic
        msg = {
            "status": [
                {"level": 0, "name": "cpu", "message": "OK",
                 "hardware_id": "host", "values": [{"key": "load", "value": "0.1"}]},
                {"level": 2, "name": "disk", "message": "Full", "hardware_id": "", "values": []},
            ]
        }
        result = ros2_topic._parse_diag_array(msg)
        self.assertEqual(len(result), 2)
        self.assertEqual(result[0]["level_name"], "OK")
        self.assertEqual(result[0]["values"][0]["key"], "load")
        self.assertEqual(result[1]["level_name"], "ERROR")


class TestBatteryParsing(unittest.TestCase):
    """Tests for 'topics battery-list' and 'topics battery' subcommands."""

    @classmethod
    def setUpClass(cls):
        if not check_rclpy_available():
            raise unittest.SkipTest("rclpy not available - requires ROS 2 environment")
        import ros2_cli
        cls.ros2_cli = ros2_cli
        cls.parser = ros2_cli.build_parser()

    # ------------------------------------------------------------------
    # battery-list
    # ------------------------------------------------------------------

    def test_battery_list_command(self):
        args = self.parser.parse_args(["topics", "battery-list"])
        self.assertEqual(args.command, "topics")
        self.assertEqual(args.subcommand, "battery-list")
        self.assertFalse(hasattr(args, "topic"))  # battery-list takes no topic arg

    # ------------------------------------------------------------------
    # battery
    # ------------------------------------------------------------------

    def test_battery_defaults(self):
        args = self.parser.parse_args(["topics", "battery"])
        self.assertEqual(args.command, "topics")
        self.assertEqual(args.subcommand, "battery")
        self.assertIsNone(args.topic)
        self.assertEqual(args.timeout, 10.0)
        self.assertIsNone(args.duration)
        self.assertEqual(args.max_messages, 1)

    def test_battery_with_topic(self):
        self.assertEqual(
            self.parser.parse_args(["topics", "battery", "--topic", "/battery_state"]).topic,
            "/battery_state")
        self.assertEqual(
            self.parser.parse_args(
                ["topics", "battery", "--topic", "/robot/battery_state"]).topic,
            "/robot/battery_state")

    def test_battery_custom_options(self):
        self.assertEqual(
            self.parser.parse_args(["topics", "battery", "--timeout", "5"]).timeout, 5.0)
        self.assertEqual(
            self.parser.parse_args(["topics", "battery", "--duration", "3.5"]).duration, 3.5)
        self.assertEqual(
            self.parser.parse_args(["topics", "battery", "--max-messages", "5"]).max_messages, 5)

    def test_battery_combined_options(self):
        args = self.parser.parse_args([
            "topics", "battery",
            "--topic", "/my_robot/battery_state",
            "--duration", "10",
            "--max-messages", "20",
        ])
        self.assertEqual(args.topic, "/my_robot/battery_state")
        self.assertEqual(args.duration, 10.0)
        self.assertEqual(args.max_messages, 20)

    # ------------------------------------------------------------------
    # DISPATCH wiring + BATTERY_TYPES constant
    # ------------------------------------------------------------------

    def test_battery_dispatch_wiring(self):
        """Both keys present, callable, and mapped to distinct handlers."""
        for key in [("topics", "battery-list"), ("topics", "battery")]:
            self.assertIn(key, self.ros2_cli.DISPATCH,
                          f"Missing DISPATCH key: {key}")
            self.assertTrue(callable(self.ros2_cli.DISPATCH[key]),
                            f"Handler for {key} is not callable")
        self.assertIsNot(
            self.ros2_cli.DISPATCH[("topics", "battery-list")],
            self.ros2_cli.DISPATCH[("topics", "battery")])

    def test_battery_types_constant(self):
        import ros2_topic
        self.assertTrue(hasattr(ros2_topic, "BATTERY_TYPES"))
        self.assertIn("sensor_msgs/msg/BatteryState", ros2_topic.BATTERY_TYPES)
        self.assertIn("sensor_msgs/BatteryState", ros2_topic.BATTERY_TYPES)

    # ------------------------------------------------------------------
    # _parse_battery_state helper (pure Python — no rclpy needed for logic)
    # ------------------------------------------------------------------

    def test_parse_battery_state_codes(self):
        """Status, health, and technology numeric codes map to correct names."""
        import ros2_topic
        # power_supply_status
        for code, name in [(0, "UNKNOWN"), (1, "CHARGING"), (2, "DISCHARGING"),
                           (3, "NOT_CHARGING"), (4, "FULL")]:
            r = ros2_topic._parse_battery_state({"power_supply_status": code})
            self.assertEqual(r["status_name"], name)
        # power_supply_health
        for code, name in [(0, "UNKNOWN"), (1, "GOOD"), (2, "OVERHEAT"), (3, "DEAD"),
                           (4, "OVERVOLTAGE"), (5, "UNSPEC_FAILURE"), (6, "COLD"),
                           (7, "WATCHDOG_TIMER_EXPIRE"), (8, "SAFETY_TIMER_EXPIRE")]:
            r = ros2_topic._parse_battery_state({"power_supply_health": code})
            self.assertEqual(r["health_name"], name)
        # power_supply_technology
        for code, name in [(0, "UNKNOWN"), (1, "NIMH"), (2, "LION"), (3, "LIPO"),
                           (4, "LIFE"), (5, "NICD"), (6, "LIMN")]:
            r = ros2_topic._parse_battery_state({"power_supply_technology": code})
            self.assertEqual(r["technology_name"], name)

    def test_parse_battery_state_fields(self):
        """All standard BatteryState fields are extracted and percentage is scaled."""
        import ros2_topic
        msg = {
            "percentage": 0.75,
            "voltage": 12.4,
            "current": -2.1,
            "charge": 3.5,
            "capacity": 5.0,
            "design_capacity": 5.2,
            "temperature": 25.0,
            "present": True,
            "power_supply_status": 2,
            "power_supply_health": 1,
            "power_supply_technology": 3,
            "cell_voltage": [4.1, 4.0, float("nan")],
            "cell_temperature": [24.5, 25.0],
            "location": "slot_0",
            "serial_number": "SN-001",
        }
        r = ros2_topic._parse_battery_state(msg)
        self.assertAlmostEqual(r["percentage"], 75.0)
        self.assertEqual(r["voltage"], 12.4)
        self.assertEqual(r["current"], -2.1)
        self.assertEqual(r["charge"], 3.5)
        self.assertEqual(r["capacity"], 5.0)
        self.assertEqual(r["design_capacity"], 5.2)
        self.assertEqual(r["temperature"], 25.0)
        self.assertTrue(r["present"])
        self.assertEqual(r["status_name"], "DISCHARGING")
        self.assertEqual(r["health_name"], "GOOD")
        self.assertEqual(r["technology_name"], "LIPO")
        # cell arrays: NaN entries become None
        self.assertEqual(r["cell_voltage"], [4.1, 4.0, None])
        self.assertEqual(r["cell_temperature"], [24.5, 25.0])
        self.assertEqual(r["location"], "slot_0")
        self.assertEqual(r["serial_number"], "SN-001")

    def test_parse_battery_state_nan_fields(self):
        """Unmeasured float fields (NaN per spec) are converted to null."""
        import ros2_topic
        nan = float("nan")
        r = ros2_topic._parse_battery_state({
            "percentage": nan,
            "voltage": nan,
            "current": nan,
            "charge": nan,
            "capacity": nan,
            "design_capacity": nan,
            "temperature": nan,
        })
        self.assertIsNone(r["percentage"])
        self.assertIsNone(r["voltage"])
        self.assertIsNone(r["current"])
        self.assertIsNone(r["charge"])
        self.assertIsNone(r["capacity"])
        self.assertIsNone(r["design_capacity"])
        self.assertIsNone(r["temperature"])


class TestGlobalOverrides(unittest.TestCase):
    """Tests for global --timeout / --retries args and _apply_global_overrides().

    These cover:
      1. Argparse integration — global args land in the right namespace attributes.
      2. _apply_global_overrides() logic — correct propagation and fallback behaviour.

    All tests skip if rclpy is not available, matching the pattern used throughout
    this test suite.
    """

    @classmethod
    def setUpClass(cls):
        if not check_rclpy_available():
            raise unittest.SkipTest("rclpy not available - requires ROS 2 environment")
        import ros2_cli
        cls.ros2_cli = ros2_cli

    def setUp(self):
        self.parser = self.ros2_cli.build_parser()

    # ------------------------------------------------------------------
    # argparse integration
    # ------------------------------------------------------------------

    def test_global_timeout_arg(self):
        """--timeout N (before subcommand) stores value in global_timeout."""
        args = self.parser.parse_args(["--timeout", "30", "topics", "list"])
        self.assertEqual(args.global_timeout, 30.0)

    def test_global_timeout_float(self):
        """--timeout accepts fractional seconds."""
        args = self.parser.parse_args(["--timeout", "2.5", "params", "list", "/turtlesim"])
        self.assertEqual(args.global_timeout, 2.5)

    def test_global_timeout_default_none(self):
        """global_timeout defaults to None (no override) when not supplied."""
        args = self.parser.parse_args(["topics", "list"])
        self.assertIsNone(args.global_timeout)

    def test_retries_arg(self):
        """--retries N stores value in retries attribute."""
        args = self.parser.parse_args(["--retries", "3", "topics", "list"])
        self.assertEqual(args.retries, 3)

    def test_retries_default_1(self):
        """retries defaults to 1 (no retry) when not supplied."""
        args = self.parser.parse_args(["topics", "list"])
        self.assertEqual(args.retries, 1)

    def test_global_timeout_does_not_shadow_per_command_timeout(self):
        """When global --timeout is set, per-command --timeout arg also exists."""
        # The per-command timeout is still present before the override is applied
        args = self.parser.parse_args(["--timeout", "30", "lifecycle", "get", "/node"])
        self.assertEqual(args.global_timeout, 30.0)
        # Per-command default (5.0 for lifecycle get) survives parse_args unchanged
        self.assertEqual(args.timeout, 5.0)

    # ------------------------------------------------------------------
    # _apply_global_overrides() logic
    # ------------------------------------------------------------------

    def test_apply_global_overrides_sets_timeout(self):
        """When global_timeout is set, _apply_global_overrides propagates it to timeout."""
        from types import SimpleNamespace
        args = SimpleNamespace(global_timeout=30.0, timeout=5.0, retries=1)
        self.ros2_cli._apply_global_overrides(args)
        self.assertEqual(args.timeout, 30.0)

    def test_apply_global_overrides_no_op_when_not_set(self):
        """When global_timeout is None, per-command timeout is left unchanged."""
        from types import SimpleNamespace
        args = SimpleNamespace(global_timeout=None, timeout=5.0, retries=1)
        self.ros2_cli._apply_global_overrides(args)
        self.assertEqual(args.timeout, 5.0)

    def test_apply_global_overrides_retries_fallback(self):
        """When retries is absent, _apply_global_overrides sets it to 1."""
        from types import SimpleNamespace
        args = SimpleNamespace(global_timeout=None, timeout=5.0)
        # No retries attribute on purpose
        self.ros2_cli._apply_global_overrides(args)
        self.assertEqual(args.retries, 1)

    def test_apply_global_overrides_retries_preserved(self):
        """When retries is already set, _apply_global_overrides does not change it."""
        from types import SimpleNamespace
        args = SimpleNamespace(global_timeout=None, timeout=5.0, retries=3)
        self.ros2_cli._apply_global_overrides(args)
        self.assertEqual(args.retries, 3)

    def test_global_timeout_end_to_end(self):
        """Full parse + override: global --timeout replaces the per-command default."""
        args = self.parser.parse_args(["--timeout", "30", "lifecycle", "get", "/node"])
        self.ros2_cli._apply_global_overrides(args)
        self.assertEqual(args.timeout, 30.0)

    def test_global_timeout_end_to_end_no_override(self):
        """Full parse + override: without global --timeout, per-command default survives."""
        args = self.parser.parse_args(["lifecycle", "get", "/node"])
        self.ros2_cli._apply_global_overrides(args)
        # lifecycle get defaults to timeout=5.0
        self.assertEqual(args.timeout, 5.0)


class TestRetryBehavior(unittest.TestCase):
    """Behavioral tests for retry loops in service/action/param handlers.

    These tests mock the ROS 2 layer so they run without a live ROS 2
    environment, exercising only the retry and timeout logic.
    """

    @classmethod
    def setUpClass(cls):
        if not check_rclpy_available():
            raise unittest.SkipTest("rclpy not available - requires ROS 2 environment")

    def _make_future(self, result_value=None, done=True):
        """Return a MagicMock that behaves like a rclpy Future."""
        f = MagicMock()
        f.done.return_value = done
        f.result.return_value = result_value
        return f

    # ------------------------------------------------------------------
    # ros2_service.py  – cmd_services_call
    # ------------------------------------------------------------------

    @patch("ros2_service.rclpy")
    @patch("ros2_service.output")
    def test_service_call_retries_on_server_unavailable(self, mock_output, mock_rclpy):
        """cmd_services_call should retry when wait_for_service times out."""
        import sys
        sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "scripts"))
        import ros2_service

        mock_node = MagicMock()
        mock_rclpy.init = MagicMock()
        mock_rclpy.shutdown = MagicMock()
        mock_rclpy.spin_once = MagicMock()

        mock_client = MagicMock()
        # Fail on first attempt, succeed on second
        mock_client.wait_for_service.side_effect = [False, True]
        future = self._make_future(result_value=MagicMock())
        future.done.return_value = True
        mock_client.call_async.return_value = future
        mock_node.create_client.return_value = mock_client
        mock_rclpy.init.return_value = None

        with patch("ros2_service.ROS2CLI", return_value=mock_node), \
             patch("ros2_service.get_message", return_value=MagicMock()), \
             patch("ros2_service.time") as mock_time:
            mock_time.time.return_value = 0  # immediately past end_time so spin exits

            from types import SimpleNamespace
            args = SimpleNamespace(
                service="/test_svc",
                srv_type="std_srvs/srv/Empty",
                request="{}",
                timeout=1.0,
                retries=2,
                global_timeout=None,
            )
            ros2_service.cmd_services_call(args)

        # wait_for_service called twice (once per attempt)
        self.assertEqual(mock_client.wait_for_service.call_count, 2)

    @patch("ros2_service.rclpy")
    @patch("ros2_service.output")
    def test_service_call_gives_up_after_all_retries(self, mock_output, mock_rclpy):
        """cmd_services_call emits error after exhausting all retries."""
        import ros2_service

        mock_node = MagicMock()
        mock_rclpy.init = MagicMock()
        mock_rclpy.shutdown = MagicMock()

        mock_client = MagicMock()
        mock_client.wait_for_service.return_value = False  # always fails
        mock_node.create_client.return_value = mock_client

        with patch("ros2_service.ROS2CLI", return_value=mock_node), \
             patch("ros2_service.get_message", return_value=MagicMock()):
            from types import SimpleNamespace
            args = SimpleNamespace(
                service="/test_svc",
                srv_type="std_srvs/srv/Empty",
                request="{}",
                timeout=1.0,
                retries=3,
                global_timeout=None,
            )
            ros2_service.cmd_services_call(args)

        # All 3 attempts exhausted
        self.assertEqual(mock_client.wait_for_service.call_count, 3)
        # Error emitted
        mock_output.assert_called_once()
        call_args = mock_output.call_args[0][0]
        self.assertIn("error", call_args)

    # ------------------------------------------------------------------
    # ros2_action.py  – cmd_actions_cancel
    # ------------------------------------------------------------------

    @patch("ros2_action.rclpy")
    @patch("ros2_action.output")
    def test_action_cancel_retries_on_server_unavailable(self, mock_output, mock_rclpy):
        """cmd_actions_cancel should retry when wait_for_service times out."""
        import ros2_action

        mock_node = MagicMock()
        mock_rclpy.init = MagicMock()
        mock_rclpy.shutdown = MagicMock()
        mock_rclpy.spin_once = MagicMock()

        mock_client = MagicMock()
        mock_client.wait_for_service.side_effect = [False, True]

        cancel_response = MagicMock()
        cancel_response.goals_canceling = []
        future = self._make_future(result_value=cancel_response)
        future.done.return_value = True
        mock_client.call_async.return_value = future
        mock_node.create_client.return_value = mock_client

        with patch("ros2_action.ROS2CLI", return_value=mock_node), \
             patch("ros2_action.time") as mock_time:
            mock_time.time.return_value = 0

            from types import SimpleNamespace
            args = SimpleNamespace(
                action="/test_action",
                goal_id=None,
                timeout=1.0,
                retries=2,
                global_timeout=None,
            )
            ros2_action.cmd_actions_cancel(args)

        self.assertEqual(mock_client.wait_for_service.call_count, 2)

    @patch("ros2_action.rclpy")
    @patch("ros2_action.output")
    def test_action_cancel_gives_up_after_all_retries(self, mock_output, mock_rclpy):
        """cmd_actions_cancel emits error after exhausting all retries."""
        import ros2_action

        mock_node = MagicMock()
        mock_rclpy.init = MagicMock()
        mock_rclpy.shutdown = MagicMock()

        mock_client = MagicMock()
        mock_client.wait_for_service.return_value = False
        mock_node.create_client.return_value = mock_client

        with patch("ros2_action.ROS2CLI", return_value=mock_node):
            from types import SimpleNamespace
            args = SimpleNamespace(
                action="/test_action",
                goal_id=None,
                timeout=1.0,
                retries=3,
                global_timeout=None,
            )
            ros2_action.cmd_actions_cancel(args)

        self.assertEqual(mock_client.wait_for_service.call_count, 3)
        mock_output.assert_called_once()
        call_args = mock_output.call_args[0][0]
        self.assertIn("error", call_args)

    # ------------------------------------------------------------------
    # _apply_global_overrides – guard for missing timeout attr
    # ------------------------------------------------------------------

    def test_apply_global_overrides_no_timeout_attr(self):
        """Global timeout must NOT inject args.timeout when attr is absent."""
        import ros2_cli
        from types import SimpleNamespace
        # Simulate a command (e.g. topics list) that has no --timeout arg
        args = SimpleNamespace(global_timeout=99.0, retries=1)
        ros2_cli._apply_global_overrides(args)
        # timeout must NOT have been injected
        self.assertFalse(hasattr(args, "timeout"),
                         "global_timeout should not inject args.timeout when attr is absent")


class TestTFParsing(unittest.TestCase):
    """Parser argument and DISPATCH wiring tests for the tf subcommands."""

    @classmethod
    def setUpClass(cls):
        if not check_rclpy_available():
            raise unittest.SkipTest("rclpy not available - requires ROS 2 environment")
        import ros2_cli
        cls.ros2_cli = ros2_cli
        cls.parser = ros2_cli.build_parser()

    def test_tf_list(self):
        args = self.parser.parse_args(["tf", "list"])
        self.assertEqual(args.command, "tf")
        self.assertEqual(args.subcommand, "list")

    def test_tf_ls_alias(self):
        args = self.parser.parse_args(["tf", "ls"])
        self.assertEqual(args.subcommand, "ls")

    def test_tf_lookup_defaults(self):
        args = self.parser.parse_args(["tf", "lookup", "world", "base_link"])
        self.assertEqual(args.command, "tf")
        self.assertEqual(args.subcommand, "lookup")
        self.assertEqual(args.source, "world")
        self.assertEqual(args.target, "base_link")
        self.assertEqual(args.timeout, 5.0)

    def test_tf_lookup_custom_timeout(self):
        args = self.parser.parse_args(["tf", "lookup", "map", "odom", "--timeout", "10"])
        self.assertEqual(args.timeout, 10.0)

    def test_tf_get_alias(self):
        args = self.parser.parse_args(["tf", "get", "world", "base_link"])
        self.assertEqual(args.subcommand, "get")
        self.assertEqual(args.source, "world")
        self.assertEqual(args.target, "base_link")

    def test_tf_echo_defaults(self):
        args = self.parser.parse_args(["tf", "echo", "world", "base_link"])
        self.assertEqual(args.subcommand, "echo")
        self.assertEqual(args.source, "world")
        self.assertEqual(args.target, "base_link")
        self.assertEqual(args.timeout, 5.0)
        self.assertEqual(args.count, 5)
        self.assertFalse(args.once)

    def test_tf_echo_with_options(self):
        args = self.parser.parse_args([
            "tf", "echo", "map", "odom",
            "--count", "3", "--once", "--timeout", "2"
        ])
        self.assertEqual(args.count, 3)
        self.assertTrue(args.once)
        self.assertEqual(args.timeout, 2.0)

    def test_tf_monitor_defaults(self):
        args = self.parser.parse_args(["tf", "monitor", "base_link"])
        self.assertEqual(args.subcommand, "monitor")
        self.assertEqual(args.frame, "base_link")
        self.assertEqual(args.timeout, 5.0)
        self.assertEqual(args.count, 5)

    def test_tf_monitor_custom(self):
        args = self.parser.parse_args(["tf", "monitor", "odom", "--count", "10", "--timeout", "3"])
        self.assertEqual(args.count, 10)
        self.assertEqual(args.timeout, 3.0)

    def test_tf_static_named_form(self):
        args = self.parser.parse_args([
            "tf", "static",
            "--from", "world", "--to", "base_link",
            "--xyz", "1.0", "2.0", "3.0",
            "--rpy", "0.0", "0.0", "1.57"
        ])
        self.assertEqual(args.subcommand, "static")
        self.assertEqual(args.from_frame, "world")
        self.assertEqual(args.to_frame, "base_link")
        self.assertEqual(args.xyz, [1.0, 2.0, 3.0])
        self.assertEqual(args.rpy, [0.0, 0.0, 1.57])

    def test_tf_static_positional_form(self):
        args = self.parser.parse_args([
            "tf", "static", "0", "0", "1", "0", "0", "0", "world", "base_link"
        ])
        self.assertEqual(args.subcommand, "static")
        self.assertEqual(args.pos_args, ["0", "0", "1", "0", "0", "0", "world", "base_link"])

    def test_tf_euler_from_quaternion(self):
        args = self.parser.parse_args(["tf", "euler-from-quaternion", "0", "0", "0", "1"])
        self.assertEqual(args.subcommand, "euler-from-quaternion")
        self.assertEqual(args.x, 0.0)
        self.assertEqual(args.y, 0.0)
        self.assertEqual(args.z, 0.0)
        self.assertEqual(args.w, 1.0)

    def test_tf_e2q_alias(self):
        args = self.parser.parse_args(["tf", "e2q", "0", "0", "0.707", "0.707"])
        self.assertEqual(args.subcommand, "e2q")
        self.assertAlmostEqual(args.z, 0.707)

    def test_tf_quat2euler_alias(self):
        args = self.parser.parse_args(["tf", "quat2euler", "0", "0", "0", "1"])
        self.assertEqual(args.subcommand, "quat2euler")

    def test_tf_quaternion_from_euler(self):
        args = self.parser.parse_args(["tf", "quaternion-from-euler", "0", "0", "1.57"])
        self.assertEqual(args.subcommand, "quaternion-from-euler")
        self.assertEqual(args.roll, 0.0)
        self.assertEqual(args.pitch, 0.0)
        self.assertAlmostEqual(args.yaw, 1.57)

    def test_tf_q2e_alias(self):
        args = self.parser.parse_args(["tf", "q2e", "0", "0", "0"])
        self.assertEqual(args.subcommand, "q2e")

    def test_tf_euler2quat_alias(self):
        args = self.parser.parse_args(["tf", "euler2quat", "0", "0", "0"])
        self.assertEqual(args.subcommand, "euler2quat")

    def test_tf_euler_from_quaternion_deg(self):
        args = self.parser.parse_args(["tf", "euler-from-quaternion-deg", "0", "0", "0", "1"])
        self.assertEqual(args.subcommand, "euler-from-quaternion-deg")

    def test_tf_e2qdeg_alias(self):
        args = self.parser.parse_args(["tf", "e2qdeg", "0", "0", "0", "1"])
        self.assertEqual(args.subcommand, "e2qdeg")

    def test_tf_quaternion_from_euler_deg(self):
        args = self.parser.parse_args(["tf", "quaternion-from-euler-deg", "0", "0", "90"])
        self.assertEqual(args.subcommand, "quaternion-from-euler-deg")
        self.assertEqual(args.yaw, 90.0)

    def test_tf_q2edeg_alias(self):
        args = self.parser.parse_args(["tf", "q2edeg", "0", "0", "90"])
        self.assertEqual(args.subcommand, "q2edeg")

    def test_tf_transform_point(self):
        args = self.parser.parse_args(["tf", "transform-point", "base_link", "world", "1", "2", "3"])
        self.assertEqual(args.subcommand, "transform-point")
        self.assertEqual(args.target, "base_link")
        self.assertEqual(args.source, "world")
        self.assertEqual(args.x, 1.0)
        self.assertEqual(args.y, 2.0)
        self.assertEqual(args.z, 3.0)
        self.assertEqual(args.timeout, 5.0)

    def test_tf_tp_alias(self):
        args = self.parser.parse_args(["tf", "tp", "base_link", "world", "1", "2", "3"])
        self.assertEqual(args.subcommand, "tp")

    def test_tf_point_alias(self):
        args = self.parser.parse_args(["tf", "point", "base_link", "world", "0", "0", "0"])
        self.assertEqual(args.subcommand, "point")

    def test_tf_transform_vector(self):
        args = self.parser.parse_args(["tf", "transform-vector", "base_link", "world", "1", "0", "0"])
        self.assertEqual(args.subcommand, "transform-vector")
        self.assertEqual(args.target, "base_link")

    def test_tf_tv_alias(self):
        args = self.parser.parse_args(["tf", "tv", "base_link", "world", "0", "1", "0"])
        self.assertEqual(args.subcommand, "tv")

    def test_tf_vector_alias(self):
        args = self.parser.parse_args(["tf", "vector", "base_link", "world", "0", "0", "1"])
        self.assertEqual(args.subcommand, "vector")

    def test_tf_dispatch_keys_present(self):
        expected = [
            ("tf", "list"), ("tf", "ls"),
            ("tf", "lookup"), ("tf", "get"),
            ("tf", "echo"), ("tf", "monitor"), ("tf", "static"),
            ("tf", "euler-from-quaternion"), ("tf", "e2q"), ("tf", "quat2euler"),
            ("tf", "quaternion-from-euler"), ("tf", "q2e"), ("tf", "euler2quat"),
            ("tf", "euler-from-quaternion-deg"), ("tf", "e2qdeg"),
            ("tf", "quaternion-from-euler-deg"), ("tf", "q2edeg"),
            ("tf", "transform-point"), ("tf", "tp"), ("tf", "point"),
            ("tf", "transform-vector"), ("tf", "tv"), ("tf", "vector"),
        ]
        for key in expected:
            self.assertIn(key, self.ros2_cli.DISPATCH, f"Missing DISPATCH key: {key}")

    def test_tf_alias_identity(self):
        D = self.ros2_cli.DISPATCH
        self.assertIs(D[("tf", "ls")], D[("tf", "list")])
        self.assertIs(D[("tf", "get")], D[("tf", "lookup")])
        self.assertIs(D[("tf", "e2q")], D[("tf", "euler-from-quaternion")])
        self.assertIs(D[("tf", "quat2euler")], D[("tf", "euler-from-quaternion")])
        self.assertIs(D[("tf", "q2e")], D[("tf", "quaternion-from-euler")])
        self.assertIs(D[("tf", "euler2quat")], D[("tf", "quaternion-from-euler")])
        self.assertIs(D[("tf", "e2qdeg")], D[("tf", "euler-from-quaternion-deg")])
        self.assertIs(D[("tf", "q2edeg")], D[("tf", "quaternion-from-euler-deg")])
        self.assertIs(D[("tf", "tp")], D[("tf", "transform-point")])
        self.assertIs(D[("tf", "point")], D[("tf", "transform-point")])
        self.assertIs(D[("tf", "tv")], D[("tf", "transform-vector")])
        self.assertIs(D[("tf", "vector")], D[("tf", "transform-vector")])

    def test_tf_handlers_callable(self):
        for key in [k for k in self.ros2_cli.DISPATCH if k[0] == "tf"]:
            self.assertTrue(callable(self.ros2_cli.DISPATCH[key]),
                            f"Handler for {key} is not callable")


class TestLaunchParsing(unittest.TestCase):
    """Parser argument and DISPATCH wiring tests for the launch subcommands."""

    @classmethod
    def setUpClass(cls):
        if not check_rclpy_available():
            raise unittest.SkipTest("rclpy not available - requires ROS 2 environment")
        import ros2_cli
        cls.ros2_cli = ros2_cli
        cls.parser = ros2_cli.build_parser()

    def test_launch_new_defaults(self):
        args = self.parser.parse_args(["launch", "new", "turtlesim", "turtlesim.launch.py"])
        self.assertEqual(args.command, "launch")
        self.assertEqual(args.subcommand, "new")
        self.assertEqual(args.package, "turtlesim")
        self.assertEqual(args.launch_file, "turtlesim.launch.py")
        self.assertEqual(args.args, [])
        self.assertEqual(args.timeout, 30.0)

    def test_launch_new_with_args(self):
        args = self.parser.parse_args([
            "launch", "new", "nav2_bringup", "bringup.launch.py",
            "use_sim_time:=true", "map:=/maps/arena.yaml"
        ])
        self.assertEqual(args.args, ["use_sim_time:=true", "map:=/maps/arena.yaml"])

    def test_launch_new_custom_timeout(self):
        args = self.parser.parse_args([
            "launch", "new", "my_pkg", "my.launch.py", "--timeout", "60"
        ])
        self.assertEqual(args.timeout, 60.0)

    def test_launch_list(self):
        args = self.parser.parse_args(["launch", "list"])
        self.assertEqual(args.command, "launch")
        self.assertEqual(args.subcommand, "list")

    def test_launch_ls_alias(self):
        args = self.parser.parse_args(["launch", "ls"])
        self.assertEqual(args.subcommand, "ls")

    def test_launch_kill(self):
        args = self.parser.parse_args(["launch", "kill", "my_session"])
        self.assertEqual(args.subcommand, "kill")
        self.assertEqual(args.session, "my_session")

    def test_launch_restart(self):
        args = self.parser.parse_args(["launch", "restart", "my_session"])
        self.assertEqual(args.subcommand, "restart")
        self.assertEqual(args.session, "my_session")

    def test_launch_foxglove_default_port(self):
        args = self.parser.parse_args(["launch", "foxglove"])
        self.assertEqual(args.subcommand, "foxglove")
        self.assertEqual(args.port, 8765)

    def test_launch_foxglove_custom_port(self):
        args = self.parser.parse_args(["launch", "foxglove", "9000"])
        self.assertEqual(args.port, 9000)

    def test_launch_dispatch_keys(self):
        expected = [
            ("launch", "new"), ("launch", "list"), ("launch", "ls"),
            ("launch", "kill"), ("launch", "restart"), ("launch", "foxglove"),
        ]
        for key in expected:
            self.assertIn(key, self.ros2_cli.DISPATCH, f"Missing DISPATCH key: {key}")
            self.assertTrue(callable(self.ros2_cli.DISPATCH[key]))

    def test_launch_ls_same_handler_as_list(self):
        D = self.ros2_cli.DISPATCH
        self.assertIs(D[("launch", "ls")], D[("launch", "list")])


class TestRunParsing(unittest.TestCase):
    """Parser argument and DISPATCH wiring tests for the run subcommands."""

    @classmethod
    def setUpClass(cls):
        if not check_rclpy_available():
            raise unittest.SkipTest("rclpy not available - requires ROS 2 environment")
        import ros2_cli
        cls.ros2_cli = ros2_cli
        cls.parser = ros2_cli.build_parser()

    def test_run_new_minimal(self):
        args = self.parser.parse_args(["run", "new", "turtlesim", "turtlesim_node"])
        self.assertEqual(args.command, "run")
        self.assertEqual(args.subcommand, "new")
        self.assertEqual(args.package, "turtlesim")
        self.assertEqual(args.executable, "turtlesim_node")
        self.assertEqual(args.args, [])
        self.assertIsNone(args.presets)
        self.assertIsNone(args.params)
        self.assertIsNone(args.config_path)

    def test_run_new_with_presets_and_params(self):
        args = self.parser.parse_args([
            "run", "new", "my_pkg", "my_node",
            "--presets", "indoor,slow",
            "--params", "speed:=0.5"
        ])
        self.assertEqual(args.presets, "indoor,slow")
        self.assertEqual(args.params, "speed:=0.5")

    def test_run_new_with_config_path(self):
        args = self.parser.parse_args([
            "run", "new", "my_pkg", "my_node",
            "--config-path", "/robot/config"
        ])
        self.assertEqual(args.config_path, "/robot/config")

    def test_run_list(self):
        args = self.parser.parse_args(["run", "list"])
        self.assertEqual(args.command, "run")
        self.assertEqual(args.subcommand, "list")

    def test_run_ls_alias(self):
        args = self.parser.parse_args(["run", "ls"])
        self.assertEqual(args.subcommand, "ls")

    def test_run_kill(self):
        args = self.parser.parse_args(["run", "kill", "turtlesim_node"])
        self.assertEqual(args.subcommand, "kill")
        self.assertEqual(args.session, "turtlesim_node")

    def test_run_restart(self):
        args = self.parser.parse_args(["run", "restart", "turtlesim_node"])
        self.assertEqual(args.subcommand, "restart")
        self.assertEqual(args.session, "turtlesim_node")

    def test_run_dispatch_keys(self):
        expected = [
            ("run", "new"), ("run", "list"), ("run", "ls"),
            ("run", "kill"), ("run", "restart"),
        ]
        for key in expected:
            self.assertIn(key, self.ros2_cli.DISPATCH, f"Missing DISPATCH key: {key}")
            self.assertTrue(callable(self.ros2_cli.DISPATCH[key]))

    def test_run_ls_same_handler_as_list(self):
        D = self.ros2_cli.DISPATCH
        self.assertIs(D[("run", "ls")], D[("run", "list")])


class TestPublishUntilParsing(unittest.TestCase):
    """Parser argument tests for topics publish-until."""

    @classmethod
    def setUpClass(cls):
        if not check_rclpy_available():
            raise unittest.SkipTest("rclpy not available - requires ROS 2 environment")
        import ros2_cli
        cls.ros2_cli = ros2_cli
        cls.parser = ros2_cli.build_parser()

    def test_publish_until_defaults(self):
        args = self.parser.parse_args([
            "topics", "publish-until",
            "/cmd_vel", '{"linear":{"x":0.3}}',
            "--monitor", "/odom",
            "--field", "pose.pose.position.x",
            "--delta", "1.0"
        ])
        self.assertEqual(args.command, "topics")
        self.assertEqual(args.subcommand, "publish-until")
        self.assertEqual(args.topic, "/cmd_vel")
        self.assertEqual(args.monitor, "/odom")
        self.assertEqual(args.field, ["pose.pose.position.x"])
        self.assertEqual(args.delta, 1.0)
        self.assertFalse(args.euclidean)
        self.assertIsNone(args.above)
        self.assertIsNone(args.below)
        self.assertIsNone(args.equals)
        self.assertIsNone(args.rotate)
        self.assertFalse(args.degrees)
        self.assertEqual(args.rate, 10.0)
        self.assertEqual(args.timeout, 60.0)

    def test_publish_until_euclidean(self):
        args = self.parser.parse_args([
            "topics", "publish-until",
            "/cmd_vel", "{}",
            "--monitor", "/odom",
            "--field", "pose.pose.position.x", "pose.pose.position.y",
            "--euclidean", "--delta", "2.0"
        ])
        self.assertTrue(args.euclidean)
        self.assertEqual(args.field, ["pose.pose.position.x", "pose.pose.position.y"])
        self.assertEqual(args.delta, 2.0)

    def test_publish_until_above(self):
        args = self.parser.parse_args([
            "topics", "publish-until", "/cmd_vel", "{}",
            "--monitor", "/odom", "--field", "pose.pose.position.x",
            "--above", "5.0"
        ])
        self.assertEqual(args.above, 5.0)
        self.assertIsNone(args.delta)

    def test_publish_until_below(self):
        args = self.parser.parse_args([
            "topics", "publish-until", "/cmd_vel", "{}",
            "--monitor", "/odom", "--field", "pose.pose.position.x",
            "--below", "0.5"
        ])
        self.assertEqual(args.below, 0.5)

    def test_publish_until_equals(self):
        args = self.parser.parse_args([
            "topics", "publish-until", "/cmd_vel", "{}",
            "--monitor", "/my_topic", "--field", "data",
            "--equals", "done"
        ])
        self.assertEqual(args.equals, "done")

    def test_publish_until_rotate_radians(self):
        args = self.parser.parse_args([
            "topics", "publish-until", "/cmd_vel", "{}",
            "--monitor", "/odom",
            "--rotate", "1.57"
        ])
        self.assertAlmostEqual(args.rotate, 1.57)
        self.assertFalse(args.degrees)

    def test_publish_until_rotate_degrees(self):
        args = self.parser.parse_args([
            "topics", "publish-until", "/cmd_vel", "{}",
            "--monitor", "/odom",
            "--rotate", "90", "--degrees"
        ])
        self.assertEqual(args.rotate, 90.0)
        self.assertTrue(args.degrees)

    def test_publish_until_rotate_negative(self):
        args = self.parser.parse_args([
            "topics", "publish-until", "/cmd_vel", "{}",
            "--monitor", "/odom",
            "--rotate", "-1.57"
        ])
        self.assertAlmostEqual(args.rotate, -1.57)

    def test_publish_until_custom_rate_and_timeout(self):
        args = self.parser.parse_args([
            "topics", "publish-until", "/cmd_vel", "{}",
            "--monitor", "/odom", "--field", "x", "--delta", "1",
            "--rate", "20", "--timeout", "30"
        ])
        self.assertEqual(args.rate, 20.0)
        self.assertEqual(args.timeout, 30.0)

    def test_publish_until_dispatch_wiring(self):
        key = ("topics", "publish-until")
        self.assertIn(key, self.ros2_cli.DISPATCH)
        self.assertTrue(callable(self.ros2_cli.DISPATCH[key]))


class TestRotationMath(unittest.TestCase):
    """Pure-Python tests for quaternion_to_yaw() and normalize_angle()."""

    @classmethod
    def setUpClass(cls):
        if not check_rclpy_available():
            raise unittest.SkipTest("rclpy not available - requires ROS 2 environment")
        import ros2_topic
        cls.ros2_topic = ros2_topic

    def test_quaternion_to_yaw_identity(self):
        """Identity quaternion (x=0,y=0,z=0,w=1) gives yaw=0."""
        yaw = self.ros2_topic.quaternion_to_yaw((0.0, 0.0, 0.0, 1.0))
        self.assertAlmostEqual(yaw, 0.0, places=9)

    def test_quaternion_to_yaw_90_ccw(self):
        """90 deg CCW about z: yaw approx pi/2."""
        import math
        z = math.sin(math.pi / 4)
        w = math.cos(math.pi / 4)
        yaw = self.ros2_topic.quaternion_to_yaw((0.0, 0.0, z, w))
        self.assertAlmostEqual(yaw, math.pi / 2, places=9)

    def test_quaternion_to_yaw_90_cw(self):
        """90 deg CW about z: yaw approx -pi/2."""
        import math
        z = -math.sin(math.pi / 4)
        w = math.cos(math.pi / 4)
        yaw = self.ros2_topic.quaternion_to_yaw((0.0, 0.0, z, w))
        self.assertAlmostEqual(yaw, -math.pi / 2, places=9)

    def test_quaternion_to_yaw_180(self):
        """180 deg about z: yaw approx pi."""
        import math
        yaw = self.ros2_topic.quaternion_to_yaw((0.0, 0.0, 1.0, 0.0))
        self.assertAlmostEqual(abs(yaw), math.pi, places=9)

    def test_normalize_angle_zero(self):
        self.assertAlmostEqual(self.ros2_topic.normalize_angle(0.0), 0.0)

    def test_normalize_angle_within_range(self):
        import math
        for a in [1.0, -1.0, math.pi - 0.001, -(math.pi - 0.001)]:
            self.assertAlmostEqual(self.ros2_topic.normalize_angle(a), a, places=9)

    def test_normalize_angle_above_pi(self):
        import math
        result = self.ros2_topic.normalize_angle(math.pi + 0.5)
        self.assertGreater(result, -math.pi)
        self.assertLessEqual(result, math.pi)
        self.assertAlmostEqual(result, -(math.pi - 0.5), places=9)

    def test_normalize_angle_below_minus_pi(self):
        import math
        result = self.ros2_topic.normalize_angle(-(math.pi + 0.5))
        self.assertGreater(result, -math.pi)
        self.assertLessEqual(result, math.pi)
        self.assertAlmostEqual(result, math.pi - 0.5, places=9)

    def test_normalize_angle_multiple_wraps(self):
        import math
        result = self.ros2_topic.normalize_angle(5 * math.pi)
        self.assertAlmostEqual(result, math.pi, places=9)
        result2 = self.ros2_topic.normalize_angle(-5 * math.pi)
        self.assertAlmostEqual(abs(result2), math.pi, places=9)


class TestControlParsing(unittest.TestCase):
    """Parser argument and DISPATCH wiring tests for the control subcommands."""

    @classmethod
    def setUpClass(cls):
        if not check_rclpy_available():
            raise unittest.SkipTest("rclpy not available - requires ROS 2 environment")
        import ros2_cli
        cls.ros2_cli = ros2_cli
        cls.parser = ros2_cli.build_parser()

    def test_list_controller_types_defaults(self):
        args = self.parser.parse_args(["control", "list-controller-types"])
        self.assertEqual(args.command, "control")
        self.assertEqual(args.subcommand, "list-controller-types")
        self.assertEqual(args.controller_manager, "/controller_manager")
        self.assertEqual(args.timeout, 5.0)

    def test_list_controller_types_custom_cm(self):
        args = self.parser.parse_args([
            "control", "list-controller-types",
            "--controller-manager", "/robot/controller_manager"
        ])
        self.assertEqual(args.controller_manager, "/robot/controller_manager")

    def test_list_controllers_defaults(self):
        args = self.parser.parse_args(["control", "list-controllers"])
        self.assertEqual(args.subcommand, "list-controllers")
        self.assertEqual(args.controller_manager, "/controller_manager")

    def test_list_hardware_components(self):
        args = self.parser.parse_args(["control", "list-hardware-components"])
        self.assertEqual(args.subcommand, "list-hardware-components")

    def test_list_hardware_interfaces(self):
        args = self.parser.parse_args(["control", "list-hardware-interfaces"])
        self.assertEqual(args.subcommand, "list-hardware-interfaces")

    def test_load_controller(self):
        args = self.parser.parse_args(["control", "load-controller", "joint_trajectory_controller"])
        self.assertEqual(args.subcommand, "load-controller")
        self.assertEqual(args.name, "joint_trajectory_controller")

    def test_unload_controller(self):
        args = self.parser.parse_args(["control", "unload-controller", "joint_trajectory_controller"])
        self.assertEqual(args.subcommand, "unload-controller")
        self.assertEqual(args.name, "joint_trajectory_controller")

    def test_configure_controller(self):
        args = self.parser.parse_args(["control", "configure-controller", "my_ctrl"])
        self.assertEqual(args.subcommand, "configure-controller")
        self.assertEqual(args.name, "my_ctrl")

    def test_reload_controller_libraries_default(self):
        args = self.parser.parse_args(["control", "reload-controller-libraries"])
        self.assertEqual(args.subcommand, "reload-controller-libraries")
        self.assertFalse(args.force_kill)

    def test_reload_controller_libraries_force_kill(self):
        args = self.parser.parse_args(["control", "reload-controller-libraries", "--force-kill"])
        self.assertTrue(args.force_kill)

    def test_set_controller_state_active(self):
        args = self.parser.parse_args([
            "control", "set-controller-state", "joint_trajectory_controller", "active"
        ])
        self.assertEqual(args.subcommand, "set-controller-state")
        self.assertEqual(args.name, "joint_trajectory_controller")
        self.assertEqual(args.state, "active")

    def test_set_controller_state_inactive(self):
        args = self.parser.parse_args([
            "control", "set-controller-state", "my_ctrl", "inactive"
        ])
        self.assertEqual(args.state, "inactive")

    def test_set_hardware_component_state(self):
        args = self.parser.parse_args([
            "control", "set-hardware-component-state", "my_robot", "active"
        ])
        self.assertEqual(args.subcommand, "set-hardware-component-state")
        self.assertEqual(args.name, "my_robot")
        self.assertEqual(args.state, "active")

    def test_set_hardware_component_state_choices(self):
        for state in ["unconfigured", "inactive", "active", "finalized"]:
            args = self.parser.parse_args([
                "control", "set-hardware-component-state", "my_hw", state
            ])
            self.assertEqual(args.state, state)

    def test_switch_controllers_defaults(self):
        args = self.parser.parse_args(["control", "switch-controllers"])
        self.assertEqual(args.subcommand, "switch-controllers")
        self.assertEqual(args.activate, [])
        self.assertEqual(args.deactivate, [])
        self.assertEqual(args.strictness, "BEST_EFFORT")
        self.assertFalse(args.activate_asap)

    def test_switch_controllers_activate_deactivate(self):
        args = self.parser.parse_args([
            "control", "switch-controllers",
            "--activate", "joint_trajectory_controller",
            "--deactivate", "cartesian_controller"
        ])
        self.assertIn("joint_trajectory_controller", args.activate)
        self.assertIn("cartesian_controller", args.deactivate)

    def test_switch_controllers_strict(self):
        args = self.parser.parse_args([
            "control", "switch-controllers", "--strictness", "STRICT"
        ])
        self.assertEqual(args.strictness, "STRICT")

    def test_switch_controllers_activate_asap(self):
        args = self.parser.parse_args([
            "control", "switch-controllers", "--activate-asap"
        ])
        self.assertTrue(args.activate_asap)

    def test_view_controller_chains_defaults(self):
        args = self.parser.parse_args(["control", "view-controller-chains"])
        self.assertEqual(args.subcommand, "view-controller-chains")
        self.assertEqual(args.output, "controller_diagram.pdf")
        self.assertIsNone(args.channel_id)

    def test_view_controller_chains_custom(self):
        args = self.parser.parse_args([
            "control", "view-controller-chains",
            "--output", "my.pdf",
            "--channel-id", "12345"
        ])
        self.assertEqual(args.output, "my.pdf")
        self.assertEqual(args.channel_id, "12345")

    def test_lct_alias(self):
        args = self.parser.parse_args(["control", "lct"])
        self.assertEqual(args.subcommand, "lct")

    def test_lc_alias(self):
        args = self.parser.parse_args(["control", "lc"])
        self.assertEqual(args.subcommand, "lc")

    def test_lhc_alias(self):
        args = self.parser.parse_args(["control", "lhc"])
        self.assertEqual(args.subcommand, "lhc")

    def test_lhi_alias(self):
        args = self.parser.parse_args(["control", "lhi"])
        self.assertEqual(args.subcommand, "lhi")

    def test_load_alias(self):
        args = self.parser.parse_args(["control", "load", "my_ctrl"])
        self.assertEqual(args.subcommand, "load")
        self.assertEqual(args.name, "my_ctrl")

    def test_unload_alias(self):
        args = self.parser.parse_args(["control", "unload", "my_ctrl"])
        self.assertEqual(args.subcommand, "unload")

    def test_cc_alias(self):
        args = self.parser.parse_args(["control", "cc", "my_ctrl"])
        self.assertEqual(args.subcommand, "cc")

    def test_rcl_alias(self):
        args = self.parser.parse_args(["control", "rcl"])
        self.assertEqual(args.subcommand, "rcl")

    def test_scs_alias(self):
        args = self.parser.parse_args(["control", "scs", "my_ctrl", "active"])
        self.assertEqual(args.subcommand, "scs")

    def test_shcs_alias(self):
        args = self.parser.parse_args(["control", "shcs", "my_hw", "active"])
        self.assertEqual(args.subcommand, "shcs")

    def test_sc_alias(self):
        args = self.parser.parse_args(["control", "sc"])
        self.assertEqual(args.subcommand, "sc")

    def test_swc_alias(self):
        args = self.parser.parse_args(["control", "swc"])
        self.assertEqual(args.subcommand, "swc")

    def test_vcc_alias(self):
        args = self.parser.parse_args(["control", "vcc"])
        self.assertEqual(args.subcommand, "vcc")

    def test_control_dispatch_keys_present(self):
        canonical = [
            ("control", "list-controller-types"),
            ("control", "list-controllers"),
            ("control", "list-hardware-components"),
            ("control", "list-hardware-interfaces"),
            ("control", "load-controller"),
            ("control", "unload-controller"),
            ("control", "configure-controller"),
            ("control", "reload-controller-libraries"),
            ("control", "set-controller-state"),
            ("control", "set-hardware-component-state"),
            ("control", "switch-controllers"),
            ("control", "view-controller-chains"),
        ]
        aliases = [
            ("control", "lct"), ("control", "lc"), ("control", "lhc"), ("control", "lhi"),
            ("control", "load"), ("control", "unload"), ("control", "cc"), ("control", "rcl"),
            ("control", "scs"), ("control", "shcs"), ("control", "sc"), ("control", "swc"),
            ("control", "vcc"),
        ]
        for key in canonical + aliases:
            self.assertIn(key, self.ros2_cli.DISPATCH, f"Missing DISPATCH key: {key}")

    def test_control_alias_identity(self):
        D = self.ros2_cli.DISPATCH
        self.assertIs(D[("control", "lct")], D[("control", "list-controller-types")])
        self.assertIs(D[("control", "lc")], D[("control", "list-controllers")])
        self.assertIs(D[("control", "lhc")], D[("control", "list-hardware-components")])
        self.assertIs(D[("control", "lhi")], D[("control", "list-hardware-interfaces")])
        self.assertIs(D[("control", "load")], D[("control", "load-controller")])
        self.assertIs(D[("control", "unload")], D[("control", "unload-controller")])
        self.assertIs(D[("control", "cc")], D[("control", "configure-controller")])
        self.assertIs(D[("control", "rcl")], D[("control", "reload-controller-libraries")])
        self.assertIs(D[("control", "scs")], D[("control", "set-controller-state")])
        self.assertIs(D[("control", "shcs")], D[("control", "set-hardware-component-state")])
        self.assertIs(D[("control", "sc")], D[("control", "switch-controllers")])
        self.assertIs(D[("control", "swc")], D[("control", "switch-controllers")])
        self.assertIs(D[("control", "vcc")], D[("control", "view-controller-chains")])

    def test_control_handlers_callable(self):
        for key in [k for k in self.ros2_cli.DISPATCH if k[0] == "control"]:
            self.assertTrue(callable(self.ros2_cli.DISPATCH[key]),
                            f"Handler for {key} is not callable")


if __name__ == "__main__":
    unittest.main()
