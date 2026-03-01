# Changelog

All notable changes to ros2-skill will be documented in this file.

## [1.0.3] - 2026-02-28

### Added
- **`topics publish-until --euclidean`**: N-dimensional Euclidean distance monitoring; `--field` now accepts one or more dot-separated paths; when `--euclidean` is set, `ConditionMonitor` resolves all field paths on each message, computes `sqrt(Σ(current_i − start_i)²)`, and stops when that distance ≥ the `--delta` threshold; output gains `fields` (list), `start_values` (list), `end_values` (list), `euclidean_distance`; single-field behaviour and output format are fully backward-compatible; validated: multiple fields require `--euclidean`, `--euclidean` requires `--delta`
- **`topics ls` / `services ls`** aliases: `topics ls` → `topics list`, `services ls` → `services list` (parser + DISPATCH entries were missing)

### Fixed
- **`topics publish-until` executor compatibility**: replaced `MultiThreadedExecutor` + background `spin_thread` + `executor.shutdown(wait=False)` with `SingleThreadedExecutor` + `spin_once(timeout_sec=interval)` in the main loop — eliminates `TypeError: shutdown() got an unexpected keyword argument 'wait'` on rclpy versions prior to Humble where the `wait` keyword did not exist; matches the `SingleThreadedExecutor` + `spin_once` pattern already used by every other executor in the file
- **`nodes ls` / `params ls` / `actions ls`** aliases added — all three are now registered aliases for their respective `list` subcommands (parsers + DISPATCH); `params ls <node>` passes the required node argument through as-is
- **`topics publish` / `topics pub` / `topics publish-continuous` consolidated**: `publish-continuous` is now an alias for `publish`; `cmd_topics_publish_continuous` has been removed; `cmd_topics_publish` now accepts both `--duration` and `--timeout` (they are fully equivalent — `--timeout` is registered as an alternative option string with `dest="duration"`); output for the repeated-publish path now always includes `stopped_by: "timeout" | "keyboard_interrupt"` and reports actual elapsed duration; `KeyboardInterrupt` is caught so Ctrl+C reports cleanly rather than propagating
- **`publish-continuous` parser `--timeout` no longer required**: the `required=True` constraint is removed now that `publish` and `publish-continuous` share a single handler; omitting `--duration`/`--timeout` on either command falls back to single-shot behaviour
- **`actions echo` feedback message type loading**: `Fibonacci_FeedbackMessage` and similar `_FeedbackMessage` variants are not exported from a package's `action.__init__`; fixed `cmd_actions_echo` to strip the `_FeedbackMessage` suffix, load the parent action class via `get_action_type()`, and access `action_class.Impl.FeedbackMessage` directly; `TopicSubscriber` gains optional `msg_class` parameter to accept a pre-resolved class, bypassing `get_msg_type`
- **`services echo` timeout waiting for service event**: service event publisher uses RELIABLE + VOLATILE + depth=1000 QoS; `TopicSubscriber` defaulted to `qos_profile_system_default` which caused a QoS incompatibility and silently dropped all events; added `_get_service_event_qos()` returning a matching `QoSProfile` and passed it to `TopicSubscriber` in `cmd_services_echo`; `TopicSubscriber` gains optional `qos` parameter to accept a caller-supplied QoS profile

### Added
- **`topics hz`**: measure topic publish rate; collects `--window` inter-message intervals (default 10) using an inline `HzMonitor(Node)` subscriber; reports `rate`, `min_delta`, `max_delta`, `std_dev`, `samples`; spins with `SingleThreadedExecutor` on a daemon thread; errors if fewer than 2 messages arrive within `--timeout` (default 10 s)
- **`topics find <msg_type>`**: find all topics publishing a given message type; normalises both `geometry_msgs/Twist` and `geometry_msgs/msg/Twist` forms for comparison; returns `message_type`, `topics`, `count`
- **`actions type <action>`**: look up the type of an action server by finding the `/_action/feedback` topic and stripping the `_FeedbackMessage` suffix; returns `action`, `type`
- **`services find <service_type>`**: find all services of a given type; same normalisation approach as `topics find` (strips `/srv/` for comparison); returns `service_type`, `services`, `count`
- **`nodes details` / `nodes info`** enhanced: now also queries `get_action_server_names_and_types_by_node()` and `get_action_client_names_and_types_by_node()` (wrapped in `try/except AttributeError` for rclpy versions that predate these APIs); output gains `action_servers` and `action_clients` arrays
- **`HzMonitor(Node)`**: top-level subscriber class (mirrors `ConditionMonitor`) that accumulates message timestamps in a thread-safe list and sets a `threading.Event` when `window + 1` samples are collected
- **Command aliases** — all dispatch to the same handler as the canonical command:
  - `topics sub` → `topics subscribe`
  - `topics pub` → `topics publish`
  - `topics pub-seq` → `topics publish-sequence`
  - `topics info` → `topics details`
  - `services info` → `services details`
  - `nodes info` → `nodes details`
  - `actions info` → `actions details`
  - `actions send-goal` → `actions send`
- **`topics bw`**: measure topic bandwidth; serialises each message with `rclpy.serialization.serialize_message` to count bytes; collects `--window` samples (default 10); reports `bw` (bytes/s), `bytes_per_msg`, `rate` (Hz), `samples`; uses `BwMonitor(Node)` following the same `SingleThreadedExecutor`/`threading.Event` pattern as `HzMonitor`
- **`topics delay`**: measure end-to-end latency between `header.stamp` and wall clock; errors if the message has no `header.stamp`; reports `mean_delay`, `min_delay`, `max_delay`, `std_dev`, `samples`; uses `DelayMonitor(Node)` with the same threading model
- **`params describe`**: describe a single parameter via `DescribeParameters` service; reports `name`, `type`, `description`, `read_only`, `dynamic_typing`, `additional_constraints`
- **`params dump`**: bulk-export all parameters for a node; calls `list_parameters` then `GetParameters`; returns `{"node": ..., "parameters": {name: value, ...}}`
- **`params load`**: bulk-set parameters from a JSON string or a file path; accepts `{"param": value}` flat dict; converts each value via `_infer_param_value` and calls `SetParameters`; reports per-parameter success/failure
- **`params delete`**: delete one or more parameters via `DeleteParameters` service; accepts multiple parameter names; reports `deleted`, `count`
- **`actions cancel`**: cancel all in-flight goals on an action server by sending a `CancelGoal` request with zero UUID (`[0]*16`) and zero timestamp — per ROS 2 spec this cancels all goals; reports `return_code` and `cancelled_goals`
- **`actions send --feedback`**: new `--feedback` flag on both `send` and `send-goal`; when set, passes a `feedback_callback` to `send_goal_async`; collected feedback messages are included in output as `feedback_msgs: [...]`
- **`BwMonitor(Node)`**: top-level subscriber class that accumulates `(timestamp, serialized_size)` tuples and sets `threading.Event` after `window` samples
- **`DelayMonitor(Node)`**: top-level subscriber class that accumulates header-stamp latency samples; sets `header_missing` flag if the message has no `header.stamp`
- **`_param_value_to_python(v)`**: converts a `ParameterValue` (types 1–9) to a native Python value; used by `cmd_params_dump`
- **`_infer_param_value(value)`**: infers a `ParameterValue` from a native Python value (bool→1, int→2, float→3, str→4, lists→6–9); used by `cmd_params_load`
- **Goal-Oriented Commands workflow** in `SKILL.md` (Workflow section 7): step-by-step discovery guide for constructing `publish-until` commands from natural language intent; includes `topics find` + `topics message` + `topics subscribe` introspection steps and a lookup table of common patterns (odometry position/orientation, joint states, laser scan, range, temperature, battery)
- **Discovery note** in `COMMANDS.md` `publish-until` entry cross-referencing the new SKILL.md workflow
- **Troubleshooting row** for `publish-until` hangs/no feedback
- **`topics message-structure` / `topics message-struct`** aliases: both map to `topics message` (`cmd_topics_message`); added parser entries and DISPATCH entries; all three forms are equivalent
- **`services echo <service>`**: echo service request/response event pairs; subscribes to `<service>/_service_event` topic (requires service introspection enabled via `configure_introspection`); returns clear error with hint if introspection is not active; supports `--duration` (collect mode) and `--timeout` (single-event mode); `--max-messages` / `--max-events` limit collected events
- **`actions echo <action>`**: echo live action feedback and status messages; subscribes to `<action>/_action/feedback` and `<action>/_action/status` without requiring any introspection configuration; supports `--duration`, `--max-messages` / `--max-msgs`, and `--timeout`; returns error if action server not found
- **`actions find <action_type>`**: find all action servers offering a given action type; walks `/_action/feedback` topics, strips `_FeedbackMessage` suffix, normalises `/action/` prefix — accepts both `pkg/action/Name` and `pkg/Name` forms; returns `action_type`, `actions`, `count`

### Removed
- **`interface list`**, **`interface packages`**, **`interface package`**: removed all three commands that required `rosidl_runtime_py`; `rosidl_runtime_py` is no longer a dependency of ros2-skill
- **`interface show`**, **`interface proto`**: removed the `interface` command category entirely; `topics message` remains and provides identical functionality
- **Mandatory top-level imports for Phase 2 service types**: `GetParameters`, `SetParameters`, `ListParameters`, `DescribeParameters`, `DeleteParameters`, `ActionClient`, `CancelGoal`, `BuiltinTime` are now imported locally inside each function that uses them; `GoalInfo` and `UniqueUUID` (dead imports) are removed; this prevents a boot-time crash on ROS 2 distros where any one of these symbols is unavailable

---

## [1.0.2] - 2026-02-28

### Added
- **`topics publish-until`**: publish to a topic at a fixed rate while simultaneously monitoring a second topic; stops as soon as a structured condition on the monitored field is satisfied or a safety timeout (default 60 s) elapses; supports four operators: `--delta ±N` (field changes from start), `--above N`, `--below N`, `--equals V`; field path is dot-separated and supports list indexing (`ranges.0`); output includes `start_value`, `end_value`, `start_msg`, `end_msg`, `published_count`, `duration`, and a `condition_met` flag
- **`topics publish-continuous`**: publish at a fixed rate for a mandatory `--timeout` duration; stops early on `Ctrl+C`; reports `published_count`, `duration`, and `stopped_by` (`"timeout"` or `"keyboard_interrupt"`)
- **`resolve_field(d, path)`**: shared helper that walks a nested dict/list using a dot-separated path; integer segments index into lists
- **`ConditionMonitor(Node)`**: subscriber node that evaluates the stop condition on every message and sets a `threading.Event`; runs alongside `TopicPublisher` in a `MultiThreadedExecutor` on a background thread
- `get_srv_type()`: loads ROS 2 service classes via `importlib`, mirrors `get_msg_type()`
- `get_action_type()`: loads ROS 2 action classes via `importlib`, mirrors `get_msg_type()`
- `topics echo`: alias for `topics subscribe` — same arguments, same behaviour
- `--max-msgs`: short alias for `--max-messages` on `topics subscribe` and `topics echo`
- `_json_default()`: fallback JSON encoder used by `output()` as defence-in-depth for any non-serializable type

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
- **`cmd_version` creates an unused `ROS2CLI` node**: `rclpy.utilities.get_domain_id()` only reads `ROS_DOMAIN_ID` from the environment and needs no node; removed the redundant `node = ROS2CLI()` call
- **`cmd_params_get` sends empty `names=[]` when no colon in argument**: with format `/turtlesim` (no `:param`), `param_name` was `None`, the `GetParameters` call used `names=[]` (which returns all parameters), and `values[0]` returned an arbitrary first parameter; now errors early with a format hint
- **`cmd_params_set` crashes at the ROS level when no colon in argument**: `param.name = None` raises a type error inside rclpy; now errors early with the same format hint
- **`cmd_services_call` `wait_for_service` ignores `--timeout`**: `wait_for_service(timeout_sec=5.0)` was hardcoded; now uses `args.timeout` so the full timeout applies to both service discovery and the call itself
- **`cmd_actions_send` `wait_for_server` ignores `--timeout`**: same hardcoded 5s; now uses `args.timeout`
- **`cmd_params_set` always reports success regardless of node response**: `SetParameters` returns a `results` array with `successful: bool` and `reason: str` per parameter; the code only checked `future.done()` and returned `success: True` unconditionally; now inspects `results[0].successful` and surfaces the rejection reason on failure
- **`topics subscribe/publish/publish-sequence` emit confusing `"topic: None"` error**: all three accept `topic` as `nargs="?"` so omitting it gave `"Could not detect message type for topic: None"`; added an early guard that returns `"topic argument is required"`
- **`dict_to_msg` silently sets list-of-nested-message fields as plain Python dicts**: if a JSON field value was a list of dicts (e.g. `MarkerArray.markers`), `setattr` stored Python dicts instead of ROS message instances, causing publish to fail; now parses the field type string (`sequence<pkg/msg/Type>` or `pkg/msg/Type[N]`) and recursively converts each element via `dict_to_msg`
- **`rcl_interfaces` / `rosidl_runtime_py` imports outside `try/except ImportError`**: if these secondary ROS packages were unavailable (partial install), the script crashed with a raw unhandled `ImportError` instead of the clean JSON error; moved both import groups inside the rclpy `try/except` block
- **`topics subscribe` hardcoded 5 s single-message timeout**: code already read `args.timeout` via `hasattr` but `--timeout` was never added to the `subscribe` parser so the branch always fell back to 5.0 s; added `--timeout` (default 5 s) to the subscribe subparser
- **`params get` and `params set` accept empty param name (e.g. `/turtlesim:`)**: colon present but nothing after it — `param_name` is `""` (falsy), so `GetParameters` was called with `names=[]` (returns all params) and `values[0]` silently returned the first arbitrary parameter; early-return guard now also rejects an empty param name
- **`topics publish` gives confusing error when `msg` is omitted**: `msg` is `nargs="?"` so it defaults to `None`; `json.loads(None)` raised `TypeError` → "Invalid JSON message: … NoneType" instead of a clear "msg argument is required"; added explicit `None` guard before JSON parsing
- **`topics publish-sequence` gives confusing errors when `messages` or `durations` are omitted**: same `nargs="?"` + `None` issue; added explicit guards for both arguments
- **`estop --topic` with unloadable type silently falls back to `VELOCITY_TYPES`**: when a custom `--topic` was specified and the type was visible in the graph but its Python class could not be imported, the code tried all `VELOCITY_TYPES` as fallbacks and could publish the wrong message type; the VELOCITY_TYPES fallback now only applies to the auto-detect path; custom `--topic` reports a clear error with a workspace-sourcing hint
- **`args.timeout if hasattr(args, 'timeout') else 5.0` dead-code branches**: `--timeout` was added to the `subscribe` and `actions send` parsers in a previous fix, making the `hasattr` fallback permanently unreachable; replaced with direct `args.timeout` access in both `cmd_topics_subscribe` and `cmd_actions_send`
- **`rclpy.shutdown()` never called when an exception escapes a command handler**: every command's `except Exception` block only called `output({"error": ...})` with no shutdown, leaving rclpy initialized if a bug or `KeyboardInterrupt` escaped the inner handler; added a `try/finally` around the handler dispatch in `main()` so `rclpy.shutdown()` is guaranteed on all exit paths
- **`params list/get/set` hardcoded 5 s timeout with no CLI override**: `wait_for_service(timeout_sec=5.0)` and the response-wait loop both used a hardcoded 5 s; inconsistent with `services call` and `actions send` which already had `--timeout`; added `--timeout` (default 5 s) to all three params subparsers and replaced every hardcoded `5.0` with `args.timeout`
- **`topics subscribe` crashes with `Object of type ndarray is not JSON serializable`**: rclpy stores fixed-size array fields (e.g. `nav_msgs/msg/Odometry.pose.covariance[36]`) as `numpy.ndarray`, not Python lists; `isinstance(value, (list, tuple))` does not match numpy arrays so they fell through to `else: result[field] = value`; `json.dumps` then raises `TypeError`; added `elif hasattr(value, 'tolist'):` branch to convert numpy arrays and scalars to native Python via `.tolist()`; also added `.tolist()` handling inside the list comprehension for numpy elements inside variable-length lists; added `_json_default` fallback encoder to `output()` as a final safety net
- **`cmd_version` crashes with `AttributeError: module 'rclpy.utilities' has no attribute 'get_domain_id'`**: `rclpy.utilities.get_domain_id()` does not exist in any version of rclpy; `ROS_DOMAIN_ID` is a plain environment variable; replaced the `rclpy.init()` / `rclpy.utilities.get_domain_id()` / `rclpy.shutdown()` block with a direct `int(os.environ.get('ROS_DOMAIN_ID', 0))` read; no rclpy calls needed
- **`params get` / `params set` reject space-separated node+param syntax**: parsers only accepted the colon format (`/node:param`); `params get /base_controller base_frame_id` gave "unrecognized arguments: base_frame_id"; added optional `param_name` positional to `params get` and optional `extra_value` positional to `params set`; both commands now accept either `/node:param` or `/node param_name` (and `/node param value` for set)
- **`services call` rejects old positional service-type format**: previous fix added `--service-type` flag but users still pass `services call /svc pkg/srv/Type '{"data":true}'` (3-positional old format); added optional `extra_request` positional; handler now detects which format was used and shifts service type / JSON accordingly; both old positional and new flag forms are accepted
- **`topics publish-continuous` allowed indefinite publishing**: `--timeout` was optional with no default, enabling permanent publishing; unsafe for robots controlled by AI agents where `Ctrl+C` is unavailable (e.g. Discord, Telegram); changed to `required=True`; loop changed from `while True:` with conditional break to `while (time.time() - start_time) < timeout:`; `stopped_by` defaults to `"timeout"`

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
