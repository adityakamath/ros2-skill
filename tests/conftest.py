"""Shared pytest config: `--live` enables tests that drive the real robot (same as ROS2_LIVE_TESTS=1)."""

import os

os.environ.setdefault("ROS2_SKILL_NO_AUTOSTART", "1")  # unit tests must not spawn a background daemon
os.environ.setdefault("ROS2_SKILL_NO_AUTOPROFILE", "1")  # nor scan a workspace to build a profile


def pytest_addoption(parser):
    parser.addoption("--live", action="store_true", default=False,
                     help="run live-robot tests (sets ROS2_LIVE_TESTS=1)")


    parser.addoption("--sim", action="store_true", default=False,
                     help="run smoke tests against an already-running graph (sim:=true or the robot; sets ROS2_SIM_TESTS=1)")


def pytest_configure(config):
    if config.getoption("--live"):
        os.environ["ROS2_LIVE_TESTS"] = "1"
    if config.getoption("--sim"):
        os.environ["ROS2_SIM_TESTS"] = "1"
