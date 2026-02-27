# Changelog

All notable changes to ros2-skill will be documented in this file.

## [1.0.2] - 2026-02-27

### Fixed
- Fixed `get_msg_type()` failing for all message type formats (`geometry_msgs/msg/Twist`, `geometry_msgs/Twist`, etc.)
- Root cause: `import_message` was called with dot-separated format but expects slash format; the `__import__` fallback only loaded the top-level package, making subpackage attribute lookups always fail
- Replaced broken `__import__` fallback with `importlib.import_module`, which correctly loads subpackages and works for any ROS 2 message package whenever the environment is sourced

---

## [1.0.1] - 2026-02-27

### Added
- Emergency stop command (`estop`) - auto-detects velocity topic and message type for mobile robots
- Better error messages with hints and suggestions for unknown message types

### Changed
- Topics subscribe/publish/publish-sequence now auto-detect message type from topic if not provided
- Updated documentation with new estop command

### Fixed
- Fixed infinite recursion bug in ROS2CLI class
- Fixed misplaced imports and cleaned up duplicate imports
- Fixed null pointer checks for publisher creation
- Replaced bare except clauses with proper exception handling
- Removed unused code and variables

---

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
