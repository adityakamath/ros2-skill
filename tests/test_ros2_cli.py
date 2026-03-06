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

    def test_topics_publish_with_duration_and_rate(self):
        args = self.parser.parse_args([
            "topics", "publish", "/cmd_vel", '{}',
            "--duration", "3", "--rate", "20"
        ])
        self.assertEqual(args.duration, 3.0)
        self.assertEqual(args.rate, 20.0)

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

    def test_lifecycle_ls_no_node(self):
        args = self.parser.parse_args(["lifecycle", "ls"])
        self.assertEqual(args.subcommand, "ls")
        self.assertIsNone(args.node)

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

    def test_lifecycle_list_default_timeout(self):
        args = self.parser.parse_args(["lifecycle", "list", "/my_lifecycle_node"])
        self.assertEqual(args.timeout, 5.0)

    def test_lifecycle_list_custom_timeout(self):
        args = self.parser.parse_args(["lifecycle", "list", "/my_lifecycle_node", "--timeout", "10"])
        self.assertEqual(args.timeout, 10.0)

    def test_lifecycle_get_default_timeout(self):
        args = self.parser.parse_args(["lifecycle", "get", "/my_lifecycle_node"])
        self.assertEqual(args.timeout, 5.0)

    def test_lifecycle_set_default_timeout(self):
        args = self.parser.parse_args(["lifecycle", "set", "/my_lifecycle_node", "activate"])
        self.assertEqual(args.timeout, 5.0)

    def test_lifecycle_set_custom_timeout(self):
        args = self.parser.parse_args([
            "lifecycle", "set", "/my_lifecycle_node", "activate", "--timeout", "10"
        ])
        self.assertEqual(args.timeout, 10.0)

    def test_lifecycle_get_custom_timeout(self):
        args = self.parser.parse_args(["lifecycle", "get", "/my_node", "--timeout", "15"])
        self.assertEqual(args.timeout, 15.0)

    def test_lifecycle_get_namespaced_node(self):
        args = self.parser.parse_args(["lifecycle", "get", "/robot/camera_driver"])
        self.assertEqual(args.node, "/robot/camera_driver")

    def test_lifecycle_set_namespaced_node(self):
        args = self.parser.parse_args(["lifecycle", "set", "/robot/sensor_node", "activate"])
        self.assertEqual(args.node, "/robot/sensor_node")
        self.assertEqual(args.transition, "activate")


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
            ("topics", "publish-sequence"),
            ("services", "list"), ("services", "type"), ("services", "details"),
            ("services", "call"),
            ("nodes", "list"), ("nodes", "details"),
            ("params", "list"), ("params", "get"), ("params", "set"),
            ("actions", "list"), ("actions", "details"), ("actions", "send"),
            # lifecycle
            ("lifecycle", "nodes"), ("lifecycle", "list"), ("lifecycle", "ls"),
            ("lifecycle", "get"), ("lifecycle", "set"),
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

    def test_get_msg_type_full_name_still_works(self):
        """Test that full type names (non-aliases) still work."""
        # Full type name should work as before (backward compatibility)
        result = self.ros2_cli.get_msg_type('geometry_msgs/Twist')
        # Should return None or a valid message class, but not crash
        self.assertTrue(result is None or callable(getattr(result, '__init__', None)),
                       "Full type name should still work")

    def test_get_msg_type_none_input(self):
        """Test that None input returns None."""
        result = self.ros2_cli.get_msg_type(None)
        self.assertIsNone(result)

    def test_get_msg_type_empty_string(self):
        """Test that empty string returns None."""
        result = self.ros2_cli.get_msg_type('')
        self.assertIsNone(result)

    def test_get_msg_type_unknown_alias(self):
        """Test that unknown aliases return None."""
        result = self.ros2_cli.get_msg_type('nonexistent_alias_xyz')
        self.assertIsNone(result)

    def test_get_msg_type_with_msg_format(self):
        """Test that /msg/ format still works."""
        result = self.ros2_cli.get_msg_type('geometry_msgs/msg/Twist')
        # Should return None or a valid message class
        self.assertTrue(result is None or callable(getattr(result, '__init__', None)),
                       "/msg/ format should still work")

    def test_alias_count(self):
        """Test that we have the expected number of aliases."""
        # We defined 50 aliases
        self.assertEqual(len(self.ros2_cli.MSG_ALIASES), 50)

    def test_common_aliases_exist(self):
        """Test that all common aliases are present."""
        common_aliases = [
            'twist', 'pose', 'point', 'quaternion',  # geometry_msgs
            'odom', 'odometry', 'path',  # nav_msgs
            'laserscan', 'image', 'imu', 'pointcloud2',  # sensor_msgs
            'string', 'int32', 'float32', 'bool',  # std_msgs
            'marker', 'markerarray',  # visualization_msgs
        ]
        for alias in common_aliases:
            self.assertIn(alias, self.ros2_cli.MSG_ALIASES, 
                         f"Missing expected alias: {alias}")

    def test_aliases_map_to_valid_format(self):
        """Test that all aliases map to properly formatted type names."""
        for alias, full_type in self.ros2_cli.MSG_ALIASES.items():
            # All should be in format package/MessageName
            self.assertIn('/', full_type, 
                         f"Alias '{alias}' maps to invalid format: '{full_type}'")
            self.assertNotIn('/msg/', full_type,
                           f"Alias '{alias}' should not include /msg/: '{full_type}'")
            # Package name should be lowercase with underscores
            package = full_type.split('/')[0]
            self.assertTrue(package.islower() or '_' in package,
                          f"Package name should be lowercase: '{package}'")

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

    def test_aliases_are_lowercase(self):
        """Test that all alias keys are lowercase (for case-insensitive lookup)."""
        for alias in self.ros2_cli.MSG_ALIASES.keys():
            self.assertEqual(alias, alias.lower(),
                           f"Alias '{alias}' should be lowercase")

    def test_no_alias_conflicts_with_slash(self):
        """Test that no aliases contain '/' (which would conflict with type names)."""
        for alias in self.ros2_cli.MSG_ALIASES.keys():
            self.assertNotIn('/', alias,
                           f"Alias '{alias}' should not contain '/'")

    def test_alias_transformation_logic(self):
        """Test that alias lookup transforms input correctly (no imports)."""
        # Test that the alias exists and maps correctly
        self.assertEqual(self.ros2_cli.MSG_ALIASES.get('twist'),
                        'geometry_msgs/Twist')
        self.assertEqual(self.ros2_cli.MSG_ALIASES.get('odom'),
                        'nav_msgs/Odometry')
        self.assertEqual(self.ros2_cli.MSG_ALIASES.get('laserscan'),
                        'sensor_msgs/LaserScan')
        
        # Test case-insensitive lookup (dict keys are lowercase)
        self.assertIn('twist', self.ros2_cli.MSG_ALIASES)
        self.assertIn('posestamped', self.ros2_cli.MSG_ALIASES)
        self.assertNotIn('Twist', self.ros2_cli.MSG_ALIASES)  # Keys are lowercase
        self.assertNotIn('PoseStamped', self.ros2_cli.MSG_ALIASES)  # Keys are lowercase

    def test_get_msg_type_preserves_non_alias_input(self):
        """Test that non-alias inputs are not transformed."""
        # These should return None (not found) but not transform the input
        result = self.ros2_cli.get_msg_type('some_random_package/RandomMsg')
        self.assertIsNone(result, "Unknown package should return None")
        
        result = self.ros2_cli.get_msg_type('std_msgs/msg/String')
        # This might work if std_msgs is installed, but shouldn't crash
        self.assertTrue(result is None or callable(getattr(result, '__init__', None)))

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


if __name__ == "__main__":
    unittest.main()
