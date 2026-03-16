# Changelog

All notable changes to ros2-skill will be documented in this file.

## [1.0.5] - 2026-03-16

### New Commands

- `bag info <bag_path>` — show metadata for a ROS 2 bag: duration, starting time, storage format, message count, and per-topic message counts. Parses `metadata.yaml` directly — no rclpy or live ROS 2 graph required. Accepts a bag directory, a `metadata.yaml` path, or any storage file inside the bag directory.
- `component types` — list all registered `rclcpp` composable node types installed on this system. Reads from the `rclcpp_components` ament resource index — no rclpy or live ROS 2 graph required.
- `daemon status` — check whether the ROS 2 daemon is running; delegates to `ros2 daemon status` via subprocess; reads domain ID from `ROS_DOMAIN_ID` (default 0); no live ROS 2 graph required.
- `daemon start` — start the ROS 2 daemon; delegates to `ros2 daemon start` via subprocess; works regardless of whether the `ros2cli` Python package is importable from the current Python environment.
- `daemon stop` — stop the ROS 2 daemon; delegates to `ros2 daemon stop` via subprocess.

### RULES.md Hardening (8 items from ros2-engineering-skills gap analysis)

- **ros2_control hardware interface lifecycle** — Rule 0 pre-flight for controller operations now requires `control list-hardware-components` + `control list-hardware-interfaces` before any load/switch/configure; Rule 8 verification row updated to check hardware component remains `active` after controller operations
- **TF2 sensor frame validation** — Rule 0 pre-flight: before consuming any spatially-interpreted sensor data (camera, LiDAR, IMU, depth, GPS, sonar), subscribe for 1 message to read `header.frame_id`, verify it exists in `tf list`, and confirm the transform is actively updating via `tf echo`; Rule 17 Never list extended with sensor frame staleness prohibition
- **Camera pipeline perception check** — Rule 0 pre-flight: before using camera or depth image data, find the paired `camera_info` topic, subscribe to verify `K` matrix is non-zero, and confirm `header.frame_id` is present in TF; added as dedicated row in the action type table
- **Pre-escalation log level control** — Rule 7 diagnostic toolbox extended: before asking the user, escalate the relevant node's log level to DEBUG via `services call <node>/set_logger_level`; reset to INFO when done
- **Recursive nested type expansion** — Rule 0 "Publish to a topic" gains step 4: for any field whose type is not a primitive or well-known standard type, run `interface show <nested_type>` recursively until all leaf fields are primitives; Rule 1 discovery table row added
- **Parameter file pre-flight** — Rule 0 new row for `params load` / `--params-file`: compare YAML keys against `params list`, describe each key's type before loading, verify with `params get` after; vocabulary table updated with `--params-file` trigger words
- **Deployment / daemon context checks** — Rule 0.1 Step 0 added: verify `ROS_DOMAIN_ID` is not colliding, daemon is running (restart with shell if needed), and `ROS_LOCALHOST_ONLY` is not hiding cross-container topics; three vocabulary rows added for daemon, domain, and localhost-only queries
- **Testing vocabulary** — Two vocabulary rows added for `colcon test` / `colcon test-result` with Rule 2 shell-exception note

---

Internal refactor: centralized rclpy lifecycle management via `ros2_context()`, removed `MSG_ALIASES`, and eliminated dead code and duplicate helpers. No functional changes to any command.

### Internal — rclpy lifecycle

- Added `ros2_context()` context manager to `ros2_utils.py` — wraps `rclpy.init()` / `rclpy.shutdown()` in a `@contextmanager`; all rclpy-using command functions now use `with ros2_context():` instead of direct init/shutdown calls
- `rclpy.init()` and `rclpy.shutdown()` now appear **only** inside `ros2_context()` in `ros2_utils.py` — zero occurrences elsewhere in the codebase
- `ros2_cli.py` — removed safety `rclpy.shutdown()` from `main()` finally block; removed `import rclpy` (no longer needed at the dispatcher level)
- `ros2_param.py` — refactored `_dump_params()` to accept a `node` argument instead of managing its own rclpy context; callers (`cmd_params_dump`, `cmd_params_preset_save`) now create the context externally and pass the node in

### Internal — dead code and duplicates

- `ros2_launch.py` — removed 5 private session-management helpers (`_get_sessions_file`, `_load_sessions`, `_save_session`, `_get_session_metadata`, `_delete_session_metadata`) that were exact duplicates of already-imported `ros2_utils` functions; removed unused `import json`
- `ros2_run.py` — fixed `_find_executables()`: lib-dir traversal code was unreachable dead code (it appeared after a `return` statement in a different function); restored into `_find_executables()` where it belongs

### Removed

- `MSG_ALIASES` dict removed from `ros2_utils.py`; message type aliases (e.g. `twist` → `geometry_msgs/Twist`, `odom` → `nav_msgs/Odometry`) are no longer supported — use full type names

### Documentation

- `README.md` — removed "Message Type Aliases" section; updated TF2 helper command list to use full names instead of removed short aliases
- `references/COMMANDS.md` — removed stale alias references from the `topics message` command table and a broken link to the removed aliases section

---

## [1.0.4] - 2026-03-14

Added launch, run, and tf commands. Hardened movement safety rules and `--rotate` rotation monitoring.

### Launch Commands

- `launch new <package> <launch_file> [args...]` — run a ROS 2 launch file in a tmux session
- `launch list` — list running launch sessions
- `launch kill <session>` — kill a running launch session
- `launch restart <session>` — restart a launch session
- `launch foxglove [port]` — launch foxglove_bridge (default port: 8765)

### Run Commands

- `run new <package> <executable> [args...]` — run a ROS 2 executable in a tmux session
- `run new --presets <preset>` — apply preset parameters before running
- `run new --params "key:=value"` — set inline parameters
- `run new --config-path PATH` — path to config directory (auto-discovers yaml files)
- `run list` — list running run sessions
- `run kill <session>` — kill a running run session
- `run restart <session>` — restart a run session

### TF2 Commands

- `tf list` — list all coordinate frames
- `tf lookup` / `tf get <source> <target>` — query transform between frames
- `tf echo <source> <target> [--once] [--count N]` — echo transforms
- `tf monitor <frame>` — monitor transform updates for a frame
- `tf static` — publish a static transform (named or positional form)
- `tf euler-from-quaternion` / `tf e2q` / `tf quat2euler` — quaternion → Euler (radians)
- `tf quaternion-from-euler` / `tf q2e` / `tf euler2quat` — Euler → quaternion (radians)
- `tf euler-from-quaternion-deg` / `tf e2qdeg` — quaternion → Euler (degrees)
- `tf quaternion-from-euler-deg` / `tf q2edeg` — Euler → quaternion (degrees)
- `tf transform-point` / `tf tp` / `tf point` — transform a point between frames
- `tf transform-vector` / `tf tv` / `tf vector` — transform a vector between frames

### Skill

- Movement: velocity limit discovery now scans **every running node** (not just controller nodes) — `params list` on all nodes, filter by `max`/`limit`/`vel`/`speed`/`accel`, retrieve each candidate, apply minimum ceiling; conservative defaults 0.2 m/s / 0.75 rad/s if nothing found
- Movement: `--rotate` fixed for negative angles (CW), angles > 180°, and multi-turn rotations; sign of `--rotate` and `angular.z` must always match
- Movement: Case A distance now uses `--euclidean --field pose.pose.position` (frame-independent) instead of a single axis field
- Movement: pre-motion check — read odom twist before publishing; `estop` if robot is already moving
- Movement: odometry rate check before closed-loop; fall back to open-loop if rate < 5 Hz
- Rule 0: mandatory full-graph parameter introspection before every movement command
- Rule 0.1: mandatory session-start checks — `doctor`, simulated time, lifecycle node states
- Rule 0.5: never guess commands or flags; verify in COMMANDS.md then `--help` before use

---

## [1.0.3] - 2026-03-09

Added parameter preset commands, diagnostics monitoring, battery monitoring, and global timeout/retry configuration.

### Global Options

- `--timeout SECONDS` — override the per-command timeout for every ROS 2 call in the session; accepted before any subcommand (e.g. `--timeout 10 params get /node param`)
- `--retries N` — total number of attempts before giving up (default: `1`, i.e. no retry); applies to `wait_for_service`, `wait_for_server`, and async call spin loops across all command handlers
- `_apply_global_overrides(args)` propagates the global values onto per-command `timeout`/`retries` attributes after argparse; commands that have no `--timeout` arg (e.g. `topics list`) are explicitly guarded with `hasattr`

### Internal — Retry hardening

- `future.cancel()` is now called before every retry `continue` in all spin loops (18 sites across `ros2_action`, `ros2_control`, `ros2_lifecycle`, `ros2_param`, `ros2_service`) — prevents stale futures from a timed-out attempt delivering results to the next attempt
- `cmd_actions_send`: moved `wait_for_server` inside the retry loop so server unavailability is actually retried
- `cmd_actions_cancel`: added full retry loop (was missing entirely)

### Topics — Diagnostics & Battery

- `topics diag-list` — list all topics publishing `DiagnosticArray` messages, discovered by **type** (not by name); works with `/diagnostics`, `<node>/diagnostics`, `<namespace>/diagnostics`, or any other convention
- `topics diag` — subscribe to all discovered diagnostic topics simultaneously (or a specific `--topic`); returns parsed status with `level_name` (OK/WARN/ERROR/STALE), `name`, `message`, `hardware_id`, and key-value `values`; supports `--duration` + `--max-messages` for multi-message collection and `--timeout` for one-shot mode
- `topics battery-list` — list all topics publishing `BatteryState` messages, discovered by type
- `topics battery` — subscribe to battery topics; returns parsed state including percentage, voltage, current, charge, and cell details (handles NaN and numeric-to-label conversion for status/health/tech)

### Skill

- Auto-discovery: movement velocity limits, diagnostics, and battery topics are now auto-discovered by scanning the live graph's message types and parameters.

### Parameters — Presets

- `params preset-save <node> <preset>` — save the current live parameters of a node to `.presets/{preset}.json`; uses `ListParameters` + `GetParameters` and writes a plain `{param_name: value}` JSON file
- `params preset-load <node> <preset>` — restore a named preset onto a node via `SetParameters`; reports per-parameter success and failure reasons
- `params preset-list` — list all saved presets from `.presets/`; no arguments; no running ROS 2 graph required
- `params preset-delete <preset>` — remove a saved preset file by name only (no node arg needed); no running ROS 2 graph required
- Presets stored flat as `.presets/{preset}.json` beside the skill directory; use descriptive names like `turtlesim_indoor` to identify node and configuration

### Internal

- Refactored `cmd_params_dump`: extracted `_dump_params(node_name, timeout) -> dict | None` helper so preset-save can reuse the dump logic without going through `output()`
- `resolve_output_path()` now writes to `.artifacts/` (hidden) instead of `artifacts/`

---

## [1.0.2] - 2026-03-07

Added `doctor`, `wtf`, `multicast`, and `interface` commands for ROS 2 system health checking, UDP multicast diagnostics, and interface type discovery.

### Interface

- `interface list` — list all installed interface types across all packages; output groups by `messages`, `services`, `actions` with a `total` count; reads from the ament resource index, no running ROS 2 graph required
- `interface show <type>` — show the field structure of any message, service, or action type; accepts canonical formats (`pkg/msg/Name`, `pkg/srv/Name`, `pkg/action/Name`) and shorthand (`pkg/Name`); output includes `"kind"` (`message`/`service`/`action`) and the appropriate field dicts (`fields` for messages, `request`/`response` for services, `goal`/`result`/`feedback` for actions)
- `interface proto <type>` — show a default-value prototype of any message, service, or action type; unlike `show` (type strings), `proto` instantiates the type so output contains actual default values — useful as a copy-paste template for publish payloads; nested messages are recursively expanded
- `interface packages` — list all packages that define at least one interface type
- `interface package <pkg>` — list all interface types (messages, services, actions) for a specific package

### Doctor / Wtf

- `doctor` — run ROS 2 system health checks via `ros2doctor` entry-point checkers; outputs JSON summary with `passed/failed/warned` counts and per-checker `status` (`PASS`/`WARN`/`FAIL`)
- `doctor hello` — check cross-host connectivity: publishes a `std_msgs/String` on a configurable topic (default `/canyouhearme`) and sends UDP multicast packets to `225.0.0.1:49150`; reports which remote hosts replied via ROS and multicast
- `wtf` — exact alias for `doctor`; same flags (`--report`, `--report-failed`, `--exclude-packages`, `--include-warnings`) and same `hello` subcommand
- Flags: `--report` / `-r` (all reports), `--report-failed` / `-rf` (failed-checker reports only), `--exclude-packages` / `-ep` (skip package checks), `--include-warnings` / `-iw` (treat warnings as failures)

### Multicast

- `multicast send [--group GROUP] [--port PORT]` — send one UDP multicast datagram to the specified group and port (defaults: `225.0.0.1:49150`); returns JSON with `sent` details
- `multicast receive [--group GROUP] [--port PORT] [--timeout SEC]` — listen for UDP multicast packets and return all received within the timeout window (default: 5 s); returns JSON with `received` list, `total`, `group`, `port`, and `timeout`
- Flags: `--group` / `-g` (multicast group, default: `225.0.0.1`), `--port` / `-p` (UDP port, default: `49150`), `--timeout` / `-t` (receive only, default: `5.0`)

---

## [1.0.1] - 2026-03-07

Refactored the CLI into separate domain modules and added two new command domains: lifecycle (managed node introspection and control) and control (ros2_control controller manager operations).

### Architecture

- Decomposed `ros2_cli.py` into domain modules (`ros2_topic.py`, `ros2_node.py`, `ros2_param.py`, `ros2_service.py`, `ros2_action.py`, `ros2_lifecycle.py`, `ros2_control.py`) with shared utilities in `ros2_utils.py`; `ros2_cli.py` now acts as the dispatcher and argument parser only

### Topics

- `topics capture-image` — capture a single frame from a ROS 2 image topic (compressed or raw), save to `.artifacts/`; optional Discord send via `--channel-id` and `--config`

### Lifecycle

- `lifecycle nodes` — list all managed (lifecycle) nodes by scanning for `/get_state` services
- `lifecycle list` / `ls` — list available states and transitions for one or all managed nodes
- `lifecycle get` — get the current lifecycle state of a managed node
- `lifecycle set` — trigger a lifecycle state transition by label (e.g. `configure`, `activate`) or numeric ID

### Control

- `control list-controller-types` / `lct` — list controller plugin types available in the pluginlib registry
- `control list-controllers` / `lc` — list loaded controllers, their type, and current state
- `control list-hardware-components` / `lhc` — list hardware components (actuator, sensor, system) and their lifecycle state
- `control list-hardware-interfaces` / `lhi` — list all command and state interfaces
- `control load-controller` / `load` — load a controller plugin by name
- `control unload-controller` / `unload` — unload a stopped controller
- `control reload-controller-libraries` / `rcl` — reload controller plugin libraries; `--force-kill` stops running controllers first
- `control set-controller-state` / `scs` — activate or deactivate a single controller via `SwitchController`
- `control set-hardware-component-state` / `shcs` — drive a hardware component through its lifecycle (`unconfigured`, `inactive`, `active`, `finalized`)
- `control switch-controllers` / `sc` — atomically activate and/or deactivate multiple controllers in a single `SwitchController` call; `--strictness STRICT|BEST_EFFORT`
- `control view-controller-chains` / `vcc` — generate a Graphviz DOT diagram of loaded chained controllers, render to PDF in `.artifacts/`, optionally send to Discord
- `control configure-controller` / `cc` — explicitly configure a loaded controller (`unconfigured → inactive`) via the `ConfigureController` service; surfaces `on_configure()` errors that `SwitchController`'s silent auto-configure hides

### Fixes

- `control set-hardware-component-state` (`shcs`) — fixed `AttributeError`: response field is `state` (not `actual_state`) in all distros; `actual_state` key still present in JSON output
- `lifecycle set` — four-level fuzzy matching so any short form resolves to a full transition label: (1) exact, (2) suffix (`shutdown` → `unconfigured_shutdown`; `success` → `on_configure_success`), (3) prefix (`unconfigured` → `unconfigured_shutdown`; `on_configure` → `on_configure_success`), (4) substring (`configure` → `on_configure_success`); all four levels are generic for every transition, not only shutdown

### Utilities

- `resolve_output_path()` added to `ros2_utils.py` — shared helper for `--output` arguments; plain filename → `.artifacts/` (created if absent), explicit path → used as-is

---

## [1.0.0] - 2026-03-01

Initial release of ros2-skill — an adaptation of [ros-skill](https://github.com/lpigeon/ros-skill) redesigned for direct local ROS 2 communication via rclpy instead of rosbridge.

### Topics

- `topics list` / `ls` — list all active topics
- `topics type` — get the message type of a topic
- `topics details` / `info` — publishers, subscribers, and QoS for a topic
- `topics message` / `message-structure` / `message-struct` — introspect message field structure
- `topics subscribe` / `echo` / `sub` — collect messages; `--duration` + `--max-messages` for batch collection
- `topics publish` / `pub` / `publish-continuous` — single-shot or timed publish at `--rate` Hz
- `topics publish-sequence` / `pub-seq` — publish a sequence of messages with per-step durations
- `topics publish-until` — publish while monitoring a separate topic; stops when a condition is met (`--delta`, `--above`, `--below`, `--equals`); `--euclidean` for N-dimensional distance across multiple fields
- `topics hz` — measure publish rate (rate, min/max/std_dev of inter-message intervals)
- `topics bw` — measure topic bandwidth (bytes/s, bytes per message)
- `topics delay` — measure end-to-end latency via `header.stamp`
- `topics find` — find all topics publishing a given message type

### Nodes

- `nodes list` / `ls` — list all active nodes
- `nodes details` / `info` — publishers, subscribers, services, action servers, and action clients for a node

### Services

- `services list` / `ls` — list all services
- `services details` / `info` — request and response field structure for a service
- `services call` — call a service with a JSON request
- `services find` — find all services of a given type
- `services echo` — echo service request/response event pairs (requires introspection enabled on the node)

### Parameters

- `params list` / `ls` — list all parameters on a node
- `params get` — get a parameter value
- `params set` — set a parameter value
- `params describe` — describe a parameter (type, constraints, read-only flag)
- `params dump` — bulk-export all parameters for a node as JSON
- `params load` — bulk-set parameters from a JSON string or file
- `params delete` — delete one or more parameters

### Actions

- `actions list` / `ls` — list all action servers
- `actions details` / `info` — goal, result, and feedback structure for an action server
- `actions type` — get the action type of an action server
- `actions send` / `send-goal` — send a goal; `--feedback` streams feedback messages in the output
- `actions find` — find all action servers of a given action type
- `actions echo` — echo live feedback and status messages from an action server
- `actions cancel` — cancel all in-flight goals on an action server

### Utilities

- `version` — detect ROS 2 distro and domain ID
- `estop` — emergency stop; auto-detects velocity topic and publishes zero velocity

---

For the original ros-skill (ROS 1 + ROS 2 via rosbridge), see: [ros-skill](https://github.com/lpigeon/ros-skill)
