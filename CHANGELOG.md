# Changelog

All notable changes to ros2-skill will be documented in this file.

## [1.0.2] - 2026-02-28

### Fixed
- **`get_msg_type()` failing for all type formats**: `import_message` was called with dot-separated format but expects slash format; `__import__` fallback only loaded the top-level package making subpackage lookups always fail; replaced with `importlib.import_module`
- **Four rclpy API bugs**: `get_node_names_and_types()` → `get_node_names_and_namespaces()`; `get_*_by_node()` wrong arg count and return parsing; `topics details` used undefined `args.node`; `executor.wait_once()` → `executor.spin_once()`
- **`topics publish` / `topics publish-sequence` wrong message type**: now always query the ROS graph for the topic's actual type before publishing, overriding `--msg-type` when the topic is visible (e.g. correctly uses `TwistStamped` instead of `Twist`)
- **`services call` "module object is not callable"**: `rsplit('/', 1)` on `std_srvs/srv/SetBool` produced `pkg="std_srvs/srv"`, causing a garbage import path; added `get_srv_type()` using `importlib.import_module(f"{pkg}.srv")`
- **`services details` broken request/response fields**: tried `get_msg_fields("pkg/srv/NameRequest")`; now loads service class via `get_srv_type()` and introspects `.Request()` / `.Response()` directly
- **`VELOCITY_TYPES` never matching**: list lacked `/msg/`-format strings; ROS 2 graph returns `geometry_msgs/msg/Twist` not `geometry_msgs/Twist`; added `/msg/` variants
- **`estop --topic` ignored**: `cmd_estop` always called `find_velocity_topic()` regardless of `--topic`; now detects type from graph for the specified topic
- **`actions list` wrong topic name extraction**: used ROS 1-style `/goal`, `/cancel` suffixes; ROS 2 action topics use `/_action/` infix; replaced with `name.split('/_action/')[0]`
- **`actions details` wrong topic search and type extraction**: searched `action_name + "/goal"` (ROS 1); now uses `action_name + "/_action/feedback"`, strips `_FeedbackMessage` suffix to recover action type, and loads Goal/Result/Feedback via action class
- **`actions send` doubled import path**: `import_message(f"{action_type}/action/{...}")` built `pkg/action/Name/action/Name`; replaced with new `get_action_type()` using `importlib.import_module(f"{pkg}.action")`
- **`cmd_estop` no `try/except`**: unhandled exceptions left `rclpy` initialized and printed raw tracebacks; wrapped in `try/except` to match all other commands
- **`estop --topic` silent wrong-type fallback**: when a custom topic was not visible in the graph, code silently fell back to trying all `VELOCITY_TYPES` and published the wrong type; now returns a clear error instead
- **`msg_to_dict` crashes on binary fields**: `bytes`/`bytearray` fields (e.g. `sensor_msgs/Image.data`) caused `json.dumps` to raise `TypeError`; added explicit branch to convert to list of ints
- **`cmd_params_get` reports `exists: False` for empty-string parameters**: `bool(value_str)` returns `False` for `""`; replaced with explicit `exists = True` flag set whenever the parameter type is known (1–9)
- **`json.loads(None)` crashes in `topics publish`, `topics publish-sequence`, `actions send`**: positional args are `nargs="?"` so they can be `None`; `json.loads(None)` raises `TypeError` which is not a `JSONDecodeError` subclass and escapes both `try` blocks, crashing with a raw traceback; broadened exception catches to `(json.JSONDecodeError, TypeError, ValueError)`
- **`TopicPublisher.pub` / `TopicSubscriber.sub` not initialized to `None`**: both `__init__` methods only set `self.pub`/`self.sub` inside `if msg_class:`; if `get_msg_type` returned `None`, accessing `publisher.pub` raised `AttributeError` instead of evaluating to `None`; added `self.pub = None` / `self.sub = None` before the conditional
- **`topics subscribe` silent timeout on bad message type**: after creating `TopicSubscriber`, there was no check that the subscription was actually created; a wrong `--msg-type` would spin to timeout with "Timeout waiting for message" instead of a type error; added `if subscriber.sub is None:` guard
- **`params set` sets boolean values as strings**: `"true"`/`"false"` fell through int parsing and were stored as type 4 (string); nodes owning a boolean parameter reject this; added explicit `args.value.lower() in ('true', 'false')` check that sets type 1 with `bool_value`
- **`services call` and `actions send` hardcoded 5s timeout with no CLI override**: code already read `args.timeout` but `--timeout` was never added to either parser; long-running services or actions always failed after 5s; added `--timeout` to both (default 5s for services, 30s for actions)
- **`dict_to_msg` silently sets list-of-nested-message fields as plain Python dicts**: if a JSON field value was a list of dicts (e.g. `MarkerArray.markers`), `setattr` stored Python dicts instead of ROS message instances, causing publish to fail; now parses the field type string (`sequence<pkg/msg/Type>` or `pkg/msg/Type[N]`) and recursively converts each element via `dict_to_msg`

### Added
- `get_srv_type()`: loads ROS 2 service classes via `importlib`, mirrors `get_msg_type()`
- `get_action_type()`: loads ROS 2 action classes via `importlib`, mirrors `get_msg_type()`

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
