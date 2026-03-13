# Changelog

All notable changes to ros2-skill will be documented in this file.

## [1.0.6] - 2026-03-13

### Fixed — Remaining escape hatches allowing time-based movement instead of odometry feedback

Three places in SKILL.md were still routing agents toward `publish-sequence` with a fixed duration for distance/angle commands:

- **Quick Decision Card**: "Is X about controlling/moving? → Use TOPICS PUBLISH" had no distinction between distance-specified and open-ended movement. Now splits explicitly: distance/angle → `publish-until` with odometry (Rule 3); open-ended → `publish-sequence`. Odometry discovery added as a mandatory step when distance or angle is specified.
- **Agent Decision Framework table**: `"Move/drive/turn (mobile robot)"` row pointed to `publish` for all movement. Split into three distinct rows: open-ended movement (`publish-sequence`), distance commands (`publish-until --field --delta`), and angle/rotation commands (`publish-until --rotate`). Each row calls out Rule 3 for the distance/angle cases.
- **EXECUTION RULES Rule 4 "Always Stop After Movement"**: The `CORRECT` example showed `publish-sequence` with a 2-second fixed duration — exactly the pattern Rule 3 forbids. Replaced with: `publish-until` as the correct pattern for distance commands (it stops itself automatically), and `publish-sequence` with stop only for the open-ended fallback.

## [1.0.5] - 2026-03-13

### Fixed — Odometry rule enforcement

- **Merged duplicate Rule 3**: The EXECUTION RULES section had a weaker, softer copy of Rule 3 ("Movement Requires Feedback") that omitted the explicit `❌ Never` prohibitions and the mandatory user notification on fallback. Replaced with a hard pointer to the top-level Agent Behaviour Rule 3, which is authoritative.
- **Fixed "Move a Robot" Quick Reference example**: Primary example was `publish-sequence` with a fixed time, giving agents a copy-paste pattern that bypassed odometry entirely. Replaced with `publish-until` + odometry as primary; `publish-sequence` moved to a clearly-labelled fallback.
- **Fixed "Move a Robot" Quick Examples section**: Same fix — odometry + `publish-until` is now the primary example; `publish-sequence` is labelled as fallback only.
- **Fixed COMMANDS.md `publish-sequence` examples**: Added a prominent `⚠️ WARNING` block explaining `publish-sequence` is open-loop and time-based, must not be used when distance/angle is specified and odometry is available, and all existing examples relabelled as `[FALLBACK]`.

## [1.0.4] - 2026-03-13

Added launch and run commands for running ROS 2 launch files and executables in tmux sessions.

### Launch Commands

- `launch new <package> <launch_file> [args...]` — run a ROS 2 launch file in a tmux session
- `launch new --timeout SECONDS` — timeout for launch to start (default: 30)
- `launch list` — list running launch sessions in tmux
- `launch kill <session>` — kill a running launch session
- `launch restart <session>` — restart any launch session
- `launch foxglove [port]` — launch foxglove_bridge (port defaults to 8765)

### Run Commands

- `run new <package> <executable> [args...]` — run a ROS 2 executable in a tmux session
- `run new --presets <preset>` — apply preset parameters before running
- `run new --params "key:=value"` — set inline parameters (comma-separated, supports key:=value and key:value)
- `run new --config-path PATH` — path to config directory (auto-discovers yaml files and passes as --params-file)
- `run list` — list running run sessions in tmux
- `run kill <session>` — kill a running run session
- `run restart <session>` — restart a run session (preserves original parameters)

### Launch Argument Validation

- Always fetches real launch arguments via `--show-args` before passing any user-provided args
- Exact match → passed as-is
- No exact match → fuzzy-matched against real available args only (e.g. "mock" → "use_mock" only if "use_mock" exists)
- No match at all → argument is dropped, user is notified, launch still proceeds without it
- If `--show-args` fails → all user args are dropped, user is notified, launch still proceeds
- Invented or guessed argument names are never passed to the launch file

### Executable Auto-Detect

- Run command now fuzzy-matches executable names (e.g., "teleop" → "teleop_node")

### TF2 Transform Commands

- `tf list` — list all coordinate frames
- `tf lookup <source> <target>` — query transform between frames
- `tf echo <source> <target> [--once] [--count N]` — echo transforms; `--once` echoes a single transform
- `tf monitor <frame>` — monitor transform updates for a frame
- `tf static --from <f> --to <t> --xyz x y z --rpy r p y` — publish static transform (named form)
- `tf static x y z roll pitch yaw from_frame to_frame` — publish static transform (positional form)
- `tf euler-from-quaternion` / `tf e2q` / `tf quat2euler` — convert quaternion to Euler (radians)
- `tf quaternion-from-euler` / `tf q2e` / `tf euler2quat` — convert Euler to quaternion (radians)
- `tf euler-from-quaternion-deg` / `tf e2qdeg` — convert quaternion to Euler (degrees)
- `tf quaternion-from-euler-deg` / `tf q2edeg` — convert Euler to quaternion (degrees)
- `tf transform-point` / `tf tp` / `tf point` — transform a point between frames
- `tf transform-vector` / `tf tv` / `tf vector` — transform a vector between frames

### Auto-Refresh

Package cache automatically refreshes when a package is not found. No manual `--refresh` flag needed.

### Workspace Sourcing

Both `launch` and `run` commands automatically source local ROS 2 workspaces before executing. Workspaces are searched in order: `ROS2_LOCAL_WS` env var, `~/ros2_ws`, `~/colcon_ws`, `~/dev_ws`, `~/workspace`, `~/ros2`. Invalid `ROS2_LOCAL_WS` paths now return an error.

### Session Management

- Explicit session handling: fails if session exists, requires `launch/run kill` or `restart`
- Restart preserves original parameters (package, executable, args, presets, params, config-path)
- Session metadata saved to `~/.ros2_cli_sessions.json` for restart functionality
- Session alive check verifies process is actually running (not just tmux shell)

### Bug Fixes

- Fixed duplicate subparser bug in ros2_cli.py
- Fixed missing json import in ros2_launch.py
- Fixed argument parsing for launch/run commands
- Fixed parameter passing to ros2 launch command
- Fixed param parsing to accept both key:=value and key:value formats
- Improved session duplicate detection with double-verification
- Fixed TF2 commands: `tf list`, `tf lookup`, `tf echo`, `tf monitor`, `tf transform-point`, `tf transform-vector` — now properly handle time and message types
- Fixed `launch new` argument validation: fuzzy matching now only matches against real arguments fetched via `--show-args`. Unknown args are dropped and the user is notified — the launch still proceeds. Invented argument names are never passed.
- Fixed `tf echo`: added `--once` flag to echo a single transform
- Fixed `tf echo`: use `rclpy.time.Time()` (latest available) instead of `node.get_clock().now()` to avoid extrapolation errors on a freshly-started buffer
- Fixed `tf static`: added named argument form (`--from`, `--to`, `--xyz`, `--rpy`) as alternative to positional arguments
- Fixed `tf quat2euler` / `tf euler2quat` unrecognised subcommand: added as aliases for `euler-from-quaternion` and `quaternion-from-euler`
- Fixed `tf point` / `tf vector` unrecognised subcommand: added as aliases for `transform-point` and `transform-vector`
- Fixed `tf transform-point` / `tf transform-vector`: import `tf2_geometry_msgs` to register `PointStamped`/`Vector3Stamped` type adapters; use `rclpy.time.Time()` on message stamp to avoid extrapolation errors
- Fixed `tf lookup` / `tf echo`: LookupException and ConnectivityException now include `available_frames` in the error output so the user knows what frames are actually in the tf tree

### Refactoring

- Extracted common tmux/session helpers to `ros2_utils.py` to avoid duplication
- Shared functions: `run_cmd`, `check_tmux`, `session_exists`, `kill_session`, `check_session_alive`, `quote_path`, `generate_session_name`, `list_sessions`, `kill_session_cmd`, session metadata functions
- Shared package cache: `list_packages`, `package_exists`, `get_package_prefix`

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

### Topics — Diagnostics

- `topics diag-list` — list all topics publishing `DiagnosticArray` messages, discovered by **type** (not by name); works with `/diagnostics`, `<node>/diagnostics`, `<namespace>/diagnostics`, or any other convention
- `topics diag` — subscribe to all discovered diagnostic topics simultaneously (or a specific `--topic`); returns parsed status with `level_name` (OK/WARN/ERROR/STALE), `name`, `message`, `hardware_id`, and key-value `values`; supports `--duration` + `--max-messages` for multi-message collection and `--timeout` for one-shot mode

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
