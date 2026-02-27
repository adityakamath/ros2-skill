# Changelog

All notable changes to ros2-skill will be documented in this file.

## [1.0.0] - 2026-02-27

Initial release of ros2-skill - a fork of [ros-skill](https://github.com/lpigeon/ros-skill) - redesigned for direct local ROS 2 communication via rclpy instead of rosbridge.

### Features
- Direct rclpy integration for local ROS 2 communication
- Simplified command syntax (auto-detects message/service/action types)
- Supports topics, services, nodes, parameters, and actions

### Breaking Changes from ros-skill
- Renamed CLI from `ros_cli.py` to `ros2_cli.py`
- Removed ROS 1 support - now ROS 2 only
- Replaced rosbridge WebSocket communication with direct rclpy
- Removed `--ip`, `--port`, `--timeout` global options
- Removed `connect` command

### Dependencies
- `rclpy`, `rosidl-runtime-py`

### Architecture
`Agent → ros2_cli.py → rclpy → ROS 2`

---

For ros-skill (ROS 1 + ROS 2 via rosbridge), see: [ros-skill](https://github.com/lpigeon/ros-skill)
