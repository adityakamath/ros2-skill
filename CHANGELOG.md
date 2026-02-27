# Changelog

All notable changes to ros2-skill will be documented in this file.

## [1.0.6] - 2026-02-28

### Fixed
- **`cmd_estop` no `try/except`**: unhandled exceptions left `rclpy` initialized and printed raw tracebacks instead of clean JSON; wrapped entire function body in `try/except` to match all other commands
- **`estop --topic` silent wrong-type fallback**: when a custom topic was specified but not visible in the graph, `msg_type` was `None` and the code silently fell back to trying all `VELOCITY_TYPES`, potentially publishing the wrong message type to the user's topic; now returns a clear error instead
- **`msg_to_dict` crashes on binary fields**: `bytes`/`bytearray` fields (e.g. `sensor_msgs/Image.data`, `sensor_msgs/PointCloud2.data`) fell through to the `else` branch, causing `json.dumps` to raise `TypeError: Object of type bytes is not JSON serializable`; added explicit `isinstance(value, (bytes, bytearray))` branch that converts to a list of ints
- **`cmd_params_get` reports `exists: False` for empty-string parameters**: used `bool(value_str)` to set `exists`, which returns `False` for an empty string even when the parameter is set to `""`; replaced with an explicit `exists = True` flag set whenever the parameter type matches a known type (1–9)

---

## [1.0.5] - 2026-02-28

### Fixed
- **`VELOCITY_TYPES`**: list only contained old-format strings (`geometry_msgs/Twist`) but ROS 2 `get_topic_names_and_types()` returns `/msg/`-format strings; the first match loop in `find_velocity_topic` never fired; added `/msg/` variants
- **`estop --topic`**: parser defined `--topic` but `cmd_estop` always called `find_velocity_topic()` and ignored it; now detects type from graph for the specified topic
- **`actions list`**: extracted action name by stripping `/goal`, `/cancel`, etc. from topic name, but ROS 2 action topics use the `/_action/` infix (e.g. `/_action/feedback`); replaced with `name.split('/_action/')[0]`
- **`actions details`**: searched for `action_name + "/goal"` (ROS 1 style); now uses `action_name + "/_action/feedback"` and strips `_FeedbackMessage` suffix to recover action type; loads Goal/Result/Feedback via action class instead of broken `get_msg_fields` calls
- **`actions send`**: same bad topic name search as above; `import_message(f"{action_type}/action/{action_type.split('/')[-1]}")` built a doubled path (e.g. `pkg/action/Name/action/Name`); replaced with new `get_action_type()` using `importlib.import_module(f"{pkg}.action")`
- **Added `get_action_type()`**: mirrors `get_msg_type()` / `get_srv_type()` for action classes

---

## [1.0.4] - 2026-02-28

### Fixed
- Fixed `services call` failing with "module object is not callable": `rsplit('/', 1)` on `std_srvs/srv/SetBool` produced `pkg="std_srvs/srv"` (wrong), causing a garbage import path; replaced with `get_srv_type()` using `importlib.import_module(f"{pkg}.srv")` + `getattr`
- Fixed `services details` using the same broken path logic and trying to pass `"SetBoolRequest"` to `get_msg_fields`; now loads the service class via `get_srv_type()` and introspects `.Request()` / `.Response()` directly

---

## [1.0.3] - 2026-02-28

### Changed
- `topics publish` and `topics publish-sequence` now always query the ROS graph for the topic's actual message type before publishing, overriding any `--msg-type` argument when the topic is visible (e.g. correctly uses `TwistStamped` instead of `Twist` for controllers that require it)

---

## [1.0.2] - 2026-02-27

### Fixed
- Fixed `get_msg_type()` failing for all message type formats (`geometry_msgs/msg/Twist`, `geometry_msgs/Twist`, etc.)
- Root cause: `import_message` was called with dot-separated format but expects slash format; the `__import__` fallback only loaded the top-level package, making subpackage attribute lookups always fail
- Replaced broken `__import__` fallback with `importlib.import_module`, which correctly loads subpackages and works for any ROS 2 message package whenever the environment is sourced
- Fixed four rclpy API bugs: `get_node_names_and_types()` → `get_node_names_and_namespaces()`; `get_*_by_node()` wrong arg count and return parsing; `topics details` used undefined `args.node`; `executor.wait_once()` → `executor.spin_once()`

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
