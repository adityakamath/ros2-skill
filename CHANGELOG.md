# Changelog

All notable changes to ros2-skill will be documented in this file.

## [1.0.5] - 2026-03-21

Comprehensive review of RULES.md, AGENTS.md, and SKILL.md targeting agent self-reliance. Added bag, component, and daemon commands. Hardened safety rules. Closed AGENTS.md coverage gaps.

### New Commands

- `bag info <bag_path>` — show metadata for a ROS 2 bag (duration, message counts, per-topic stats); no live graph required
- `component types` — list all registered rclcpp composable node types; no live graph required
- `daemon status` — check if the ROS 2 daemon is running; no live graph required
- `daemon start` — start the ROS 2 daemon
- `daemon stop` — stop the ROS 2 daemon

### Safety Rules

- **Rules 18–21** — `estop` mandatory after every `publish-until` (Rule 18), QoS pre-flight gate (Rule 19), `--slow-last` for moves > 2 m / 180° (Rule 20), position verify before re-issue on timeout (Rule 21)
- **Rule 18 hard preemption** — estop is the first and only permitted action on any `publish-until` exit; no diagnosis before estop is verified
- **Rule 19 hard gate** — QoS and monitor field checks are pre-flight; never attempt `publish-until` if publisher count is 0 or field path cannot be resolved
- **Rule 22** — motion ceilings: > 50 m or > 3600° requires explicit user confirmation
- **Rule 23** — any new command during active motion triggers estop + verify before the new command is handled
- **Estop verification window** standardised to 5 s (10 s for heavy platforms > 20 kg)
- **Long-motion segmentation** (Rule 9) — `publish-until` with expected duration > 30 s broken into max-30 s segments with estop + odom rate check between each

### Self-Reliance Rules

- **Vague quantity defaults** (Rule 5) — "a bit" = 0.1 m / 5°, "nearby" = 0.5 m, etc.; act on defaults and note the assumption instead of asking
- **Already-at-target short-circuit** — skip motion if remaining ≤ 0.05 m or ≤ 3° (AGENTS.md Phase 2 Step 0.5)
- **Near-success tolerance** (Rule 21) — `condition_met: false` within 0.05 m / 3° of target treated as success
- **Multiple controller selection** (Rule 0) — prefer by robot part named in user request before falling back to first result
- **Discovery retry** (Rule 10) — retry once after 1 s before broadening; DDS discovery is eventually-consistent
- **Rule 9 carry-forward pose** — confirmed final position from prior move is the next baseline; velocity + hz checks still run fresh each time

### Rule Clarifications

- **Rule 0 vs Rule 5** — discovery is silent, execution is silent; no narration, no confirmation requests
- **Rule 8** — service response JSON must be inspected (`success`/`status` field); CLI exit code 0 ≠ success. Retry protocol: 1 auto + 2 fix+retry, then escalate
- **Rule 9 hard gate** — overriding a failed pre-motion check is a safety violation; escalate if recovery fails
- **Rule 9 odom rate** — `topics hz` mandatory before every motion command; cannot carry forward from prior command
- **Rule 11** — always state selection when more than one candidate existed
- **`condition_met: false` diagnostic** — `topics hz` after estop distinguishes QoS failure (0 Hz) from genuine timeout

### Code Fixes

- **M5** — `ConditionMonitor` auto-matches publisher QoS (BEST_EFFORT / RELIABLE) to prevent silent odom dropout
- **M8** — `scale_twist_velocity` restructured as if/elif to correctly handle Twist vs TwistStamped
- **M9** — 440-line module docstring removed from `ros2_cli.py`
- **H3** — `cmd_actions_details` missing None guard added
- **Q7** — `cmd_version` reports `rclpy_available`
- **Q8** — unique per-topic node names (`skill_<prefix>_<slug>`) to prevent node name collision

### AGENTS.md

- **Rules 18–23** condensed and added to Core Rules
- **Rules 24–27** added: session-start checks (← 0.1), ros2-skill exclusively (← 2), execute without asking (← 5), minimal reporting (← 6)
- **Removed** stale Safety line "Confirm before any movement — wait for acknowledgement" which contradicted Rule 5

### Tests

- 14 new unit tests: estop Twist/TwistStamped branching, `ros2_context` shutdown, `ConditionMonitor` QoS auto-matching, TF monitor missing-frame error

### New Commands

- `bag info <bag_path>` — show metadata for a ROS 2 bag (duration, message counts, per-topic stats); no live graph required
- `component types` — list all registered rclcpp composable node types; no live graph required
- `daemon status` — check if the ROS 2 daemon is running; no live graph required
- `daemon start` — start the ROS 2 daemon
- `daemon stop` — stop the ROS 2 daemon

### RULES.md Hardening

- ros2_control hardware interface lifecycle pre-flight before any controller load/switch/configure
- TF2 sensor frame validation before consuming spatially-interpreted sensor data
- Camera pipeline calibration check (camera_info K matrix + TF frame presence)
- Pre-escalation log level control via `set_logger_level` before asking the user
- Recursive nested type expansion via `interface show` before publishing
- Parameter file pre-flight for `params load` / `--params-file`
- ROS domain ID, daemon, and `ROS_LOCALHOST_ONLY` context checks (Rule 0.1)
- Testing vocabulary for `colcon test` / `colcon test-result`

### Documentation

- `AGENTS.md` — new agent guide: entry-point rules, session-start protocol, output folders (`.artifacts/`, `.presets/`, `.profiles/`), core operational rules, safety, and troubleshooting

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
