"""`nodes kill` must only ever signal the process of the exact node asked for. No ROS needed."""

import argparse
import contextlib
import io
import json
import os
import sys
import tempfile
import unittest
from pathlib import Path
from unittest import mock

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "scripts"))
import ros2_node  # noqa: E402

PREFIXES = ["/opt/ros/jazzy"]


def remap(*rules):
    argv = ["/ws/lib/pkg/driver", "--ros-args"]
    for rule in rules:
        argv += ["-r", rule]
    return argv


def matches(argv, name="camera", namespace="/"):
    return ros2_node._is_node_process(argv, name, namespace, PREFIXES)


class TestIsNodeProcess(unittest.TestCase):
    def test_exact_node_remap(self):
        self.assertTrue(matches(remap("__node:=camera")))
        self.assertTrue(matches(remap("__name:=camera")))

    def test_substrings_do_not_match(self):
        for other in ("camera_info", "front_camera", "cam"):
            self.assertFalse(matches(remap("__node:=" + other)), other)
        self.assertFalse(matches(["vim", "/home/u/camera.py"]))
        self.assertFalse(matches(["bash", "-c", "ros2 run p e --ros-args -r __node:=camera"]))

    def test_parameter_is_not_a_remap(self):
        self.assertFalse(matches(["/ws/lib/pkg/driver", "--ros-args", "-p", "__node:=camera"]))

    def test_namespace_must_match(self):
        argv = remap("__node:=camera", "__ns:=/front")
        self.assertTrue(matches(argv, namespace="/front"))
        self.assertFalse(matches(argv))
        self.assertFalse(matches(argv, namespace="/rear"))
        self.assertFalse(matches(remap("__node:=camera"), namespace="/front"))

    def test_executable_name_without_remap(self):
        self.assertTrue(matches(["/opt/ros/jazzy/lib/pkg/camera"]))
        self.assertTrue(matches(["/usr/bin/python3", "-u", "/opt/ros/jazzy/lib/pkg/camera"]))
        self.assertTrue(matches(["/ws/install/pkg/lib/pkg/camera", "--ros-args"]))
        self.assertFalse(matches(["/usr/local/bin/camera"]))

    def test_renamed_executable_does_not_match(self):
        self.assertFalse(matches(["/opt/ros/jazzy/lib/pkg/camera", "--ros-args", "-r", "__node:=rear"]))


class TestFindNodePids(unittest.TestCase):
    def test_scans_proc(self):
        with tempfile.TemporaryDirectory() as root:
            for pid, name in (("101", "camera"), ("102", "camera_info")):
                os.mkdir(os.path.join(root, pid))
                Path(root, pid, "cmdline").write_bytes(("\0".join(remap("__node:=" + name)) + "\0").encode())
            self.assertEqual(ros2_node._find_node_pids("camera", "/", proc_root=root), [101])


class TestCmdNodesKill(unittest.TestCase):
    def test_no_match_reports_not_killed_and_signals_nothing(self):
        args = argparse.Namespace(node="/camera", confirm=True, timeout=1.0)
        out = io.StringIO()
        with mock.patch.object(ros2_node.subprocess, "run", side_effect=FileNotFoundError), \
                mock.patch.object(ros2_node, "_find_node_pids", return_value=[]), \
                mock.patch.object(ros2_node.os, "kill") as kill, \
                contextlib.redirect_stdout(out), self.assertRaises(SystemExit):
            ros2_node.cmd_nodes_kill(args)
        self.assertFalse(json.loads(out.getvalue())["killed"])
        kill.assert_not_called()


if __name__ == "__main__":
    unittest.main()
