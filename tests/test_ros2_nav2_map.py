#!/usr/bin/env python3
"""Unit tests for nav2 map commands."""

import json
import os
import tempfile
import unittest
from pathlib import Path
from unittest.mock import Mock, patch, mock_open

import sys
sys.path.insert(0, str(Path(__file__).parent.parent / "scripts"))

from ros2_nav2 import cmd_nav2_map_list, cmd_nav2_map_save, cmd_nav2_map_load, cmd_nav2_map_delete


class TestNav2MapList(unittest.TestCase):
    """Test nav2 map list command."""

    def setUp(self):
        self.test_dir = tempfile.mkdtemp()
        self.profile_dir = tempfile.mkdtemp()

    def tearDown(self):
        import shutil
        shutil.rmtree(self.test_dir, ignore_errors=True)
        shutil.rmtree(self.profile_dir, ignore_errors=True)

    @patch('ros2_nav2.output')
    def test_list_empty_directory(self, mock_output):
        """Test listing maps from an empty directory."""
        args = Mock(maps_dir=self.test_dir)
        cmd_nav2_map_list(args)
        
        call_args = mock_output.call_args[0][0]
        self.assertEqual(call_args['count'], 0)
        self.assertEqual(call_args['maps'], [])
        self.assertIn('hint', call_args)

    @patch('ros2_nav2.output')
    def test_list_with_yaml_files(self, mock_output):
        """Test listing maps with YAML files in directory."""
        # Create test map files
        (Path(self.test_dir) / "map1.yaml").write_text("image: map1.pgm")
        (Path(self.test_dir) / "map2.yaml").write_text("image: map2.pgm")
        
        args = Mock(maps_dir=self.test_dir)
        cmd_nav2_map_list(args)
        
        call_args = mock_output.call_args[0][0]
        self.assertEqual(call_args['count'], 2)
        map_names = [m['name'] for m in call_args['maps']]
        self.assertIn('map1', map_names)
        self.assertIn('map2', map_names)

    @patch('ros2_nav2.output')
    def test_list_deduplicates_profile_and_filesystem(self, mock_output):
        """Test that maps appearing in both profile and filesystem are deduplicated."""
        # Create filesystem map
        (Path(self.test_dir) / "office.yaml").write_text("image: office.pgm")
        
        # Mock profile with same map
        profile_data = {
            "summary": {
                "maps": [
                    {"name": "office.yaml", "type": "occupancy", "resolution": 0.05}
                ]
            }
        }
        
        with patch('builtins.open', mock_open(read_data=json.dumps(profile_data))):
            args = Mock(maps_dir=self.test_dir)
            cmd_nav2_map_list(args)
        
        call_args = mock_output.call_args[0][0]
        # Should only appear once
        self.assertEqual(call_args['count'], 1)
        self.assertEqual(call_args['maps'][0]['name'], 'office')


class TestNav2MapSave(unittest.TestCase):
    """Test nav2 map save command."""

    @patch('ros2_nav2.ros2_context')
    @patch('ros2_nav2.ROS2CLI')
    @patch('ros2_nav2._call_service')
    @patch('ros2_nav2.output')
    def test_save_with_slam_toolbox(self, mock_output, mock_call_service, mock_cli, mock_context):
        """Test saving map via slam_toolbox."""
        mock_resp = Mock(result=0)  # slam_toolbox RESULT_SUCCESS == 0
        mock_call_service.return_value = (mock_resp, None)
        
        args = Mock()
        args.name = "test_map"
        args.timeout = 10.0
        
        with patch.dict('sys.modules', {'slam_toolbox': Mock(), 'slam_toolbox.srv': Mock()}):
            cmd_nav2_map_save(args)
        
        call_args = mock_output.call_args[0][0]
        self.assertTrue(call_args['success'])
        self.assertEqual(call_args['method'], 'slam_toolbox')
        self.assertEqual(call_args['name'], 'test_map')
        self.assertIn('test_map.yaml', call_args['files'])
        self.assertIn('test_map.pgm', call_args['files'])

    @patch('ros2_nav2.ros2_context')
    @patch('ros2_nav2.ROS2CLI')
    @patch('ros2_nav2._call_service')
    @patch('ros2_nav2.output')
    def test_slam_toolbox_nonzero_result_is_failure(self, mock_output, mock_call_service, mock_cli, mock_context):
        mock_call_service.return_value = (Mock(result=3), None)
        args = Mock()
        args.name = "test_map"
        args.timeout = 10.0
        with patch.dict('sys.modules', {'slam_toolbox': Mock(), 'slam_toolbox.srv': Mock()}):
            cmd_nav2_map_save(args)
        self.assertFalse(mock_output.call_args[0][0]['success'])

    @patch('ros2_nav2.ros2_context')
    @patch('ros2_nav2.ROS2CLI')
    @patch('ros2_nav2.output')
    def test_save_without_service(self, mock_output, mock_cli, mock_context):
        """Test save command when no map service is available."""
        args = Mock()
        args.name = "test_map"
        args.timeout = 10.0
        
        # Make the import fail to simulate missing service
        with patch.dict('sys.modules', {'slam_toolbox': None, 'slam_toolbox.srv': None, 'nav2_msgs': None, 'nav2_msgs.srv': None}, clear=False):
            with patch('builtins.__import__', side_effect=ImportError("No module named 'slam_toolbox'")):
                cmd_nav2_map_save(args)
        
        call_args = mock_output.call_args[0][0]
        self.assertIn('error', call_args)
        self.assertIn('slam_toolbox', call_args['error'])


def _has_module(name):
    import importlib.util
    try:
        return importlib.util.find_spec(name) is not None
    except (ImportError, ValueError):
        return False


@unittest.skipUnless(_has_module("nav2_msgs"), "needs the nav2_msgs package (a ROS 2 install)")
class TestNav2MapLoad(unittest.TestCase):
    """Test nav2 map load command."""

    @patch('ros2_nav2.ros2_context')
    @patch('ros2_nav2.ROS2CLI')
    @patch('ros2_nav2._call_service')
    @patch('ros2_nav2.output')
    @patch('os.path.exists')
    @patch('os.path.isabs')
    def test_load_map_success(self, mock_isabs, mock_exists, mock_output, mock_call_service, mock_cli, mock_context):
        """Test successfully loading a map."""
        mock_exists.return_value = True
        mock_isabs.return_value = False
        mock_resp = Mock(result=0)  # SUCCESS = 0
        mock_call_service.return_value = (mock_resp, None)
        
        args = Mock()
        args.name = "test_map"
        args.timeout = 10.0
        cmd_nav2_map_load(args)
        
        call_args = mock_output.call_args[0][0]
        self.assertTrue(call_args['success'])
        self.assertEqual(call_args['service'], '/map_server/load_map')

    @patch('ros2_nav2.output')
    @patch('os.path.exists')
    def test_load_map_file_not_found(self, mock_exists, mock_output):
        """Test loading a non-existent map."""
        mock_exists.return_value = False
        
        args = Mock()
        args.name = "nonexistent_map"
        args.timeout = 10.0
        cmd_nav2_map_load(args)
        
        call_args = mock_output.call_args[0][0]
        self.assertIn('error', call_args)
        self.assertIn('not found', call_args['error'].lower())


class TestNav2MapDelete(unittest.TestCase):
    """Test nav2 map delete command."""

    def setUp(self):
        self.test_dir = tempfile.mkdtemp()

    def tearDown(self):
        import shutil
        shutil.rmtree(self.test_dir, ignore_errors=True)

    @patch('ros2_nav2.output')
    def test_delete_without_confirm(self, mock_output):
        """Test that delete requires --confirm flag."""
        args = Mock()
        args.name = "test_map"
        args.confirm = False
        args.maps_dir = self.test_dir
        cmd_nav2_map_delete(args)
        
        call_args = mock_output.call_args[0][0]
        self.assertIn('error', call_args)
        self.assertIn('confirm', call_args['error'].lower())

    @patch('ros2_nav2.output')
    def test_delete_nonexistent_map(self, mock_output):
        """Test deleting a map that doesn't exist."""
        args = Mock()
        args.name = "nonexistent"
        args.confirm = True
        args.maps_dir = self.test_dir
        cmd_nav2_map_delete(args)
        
        call_args = mock_output.call_args[0][0]
        self.assertIn('error', call_args)
        self.assertIn('not found', call_args['error'].lower())

    @patch('ros2_nav2.output')
    def test_delete_map_success(self, mock_output):
        """Test successfully deleting a map."""
        # Create test files
        yaml_path = Path(self.test_dir) / "test_map.yaml"
        pgm_path = Path(self.test_dir) / "test_map.pgm"
        yaml_path.write_text("image: test_map.pgm")
        pgm_path.write_bytes(b"fake pgm data")
        
        args = Mock()
        args.name = "test_map"
        args.confirm = True
        args.maps_dir = self.test_dir
        cmd_nav2_map_delete(args)
        
        call_args = mock_output.call_args[0][0]
        self.assertTrue(call_args['success'])
        self.assertEqual(len(call_args['deleted']), 2)
        
        # Verify files are actually deleted
        self.assertFalse(yaml_path.exists())
        self.assertFalse(pgm_path.exists())


if __name__ == '__main__':
    unittest.main()
