# Changelog

All notable changes to ros2-skill will be documented in this file.

## [1.0.7] - 2026-05-11

Session-start snapshot, topic list capping, launch params/config/preset, package scaffolding, security hardening, rules audit, log introspection, RULES domain split, auto-hold on motion failure, velocity clamp flags, and robot profile.

### New Commands

- `profile scan [--workspace PATH] [--name NAME] [--allow-live]` — static-first workspace scan: walks `src/`, queries ament index, parses `package.xml` / launch files / URDF / YAML configs; writes a tiered `.profiles/<robot>_profile.json` with `summary` (packages, launch files, safety limits, robot type, sensor flags) and per-launch-file `detail` sections; live graph used as fallback only when `--allow-live` is passed
- `profile show [--section S]` — load the saved robot profile; without `--section` returns `summary` + list of launch file filenames; `--section summary` / `--section detail` / `--section <launch-filename>` (e.g. `bringup.launch.py`) for progressive disclosure
- `profile rescan [--launch-file F] [--workspace PATH]` — full or partial rescan; `--launch-file FILENAME` refreshes only that file's launch args without re-walking the workspace
- `profile list` — list all robot profiles stored in `.profiles/`
- `context` — compact session-start graph snapshot: topics (capped at 50), services, actions, and nodes in one call
- `launch new --param key:=value` / `--config-path PATH` / `--preset NAME` — inline params, YAML config forwarding, and preset loading; duplicate session detection warns before launching
- `pkg create <name>` — scaffold a new ROS 2 package; supports `--build-type`, `--dependencies`, `--node-name`, and other `ros2 pkg create` flags
- `logs list-runs` / `logs query` / `logs tail` / `logs node-summary` — log file introspection without a live graph; reads `~/.ros/log/`; time filters accept relative (`-30s`, `-5m`) and absolute formats
- `topics echo-once <topic>` — subscribe and return the first message received, then exit; avoids needing `--max-messages 1` or a short `--duration` for one-shot reads
- `topics depth-point --topic T --u U --v V` — extract depth at pixel (u, v) from a depth image topic; supports 16UC1 (mm→m) and 32FC1 (native m, NaN-aware) via `struct.unpack_from`; no numpy or cv2 required; returns `{depth_m, invalid, encoding, width, height}` (implements AG-10)

### Changes

- `topics list` / `ls`: default `--limit` changed from 0 (unlimited) to 50; override with `--limit 0` for the full list; matches the `context` command cap
- `topics list` / `ls`: `--limit N` flag also added; output includes `truncated: true` and `total` when capped
- `params get`: accepts extra positional parameter names — `params get /node key1 key2 key3` issues one `GetParameters` RPC; single-key output format unchanged; multi-key returns `{node, parameters: {key: {value, exists}}, count}`
- `main()`: unhandled exceptions from any command are serialised as `{"error": "...", "type": "<ExceptionClass>"}` instead of raw Python tracebacks
- `topics publish` / `pub`, `topics publish-sequence`, `topics publish-until`: new `--max-vel N` and `--max-ang N` flags clamp linear (x/y/z) to ±N m/s and angular.z to ±N rad/s before the message is sent; Twist/TwistStamped only; other message types pass through unchanged; clamped axes reported in `velocity_clamped` in the JSON output (implements RH-6)
- `shlex.quote()` applied to all user-controlled shell inputs; `session_exists` grep pipeline replaced with Python list check; AGENTS.md jailbreak-pattern phrasing removed
- AGENTS.md: camera/depth pre-flight (camera_info + K matrix + TF frame) and nested `interface show` recursion added to Rule 2; `context` snapshot added to Session Start Step 6; robot profile load added as Session Start Step 7
- SKILL.md: Session Start Checklist extended to Step 7 (profile load); Robot Profile section added under Common Operations; profile commands added to "Commands Without a Live Graph" table
- RULES-PREFLIGHT.md Rule 0.1: Step 6 (profile load) added; Rules.md index updated to Steps 0–6
- RULES-REFERENCE.md: profile commands added to intent→command table (Step 1); Step 5 velocity-limit scan notes profile as a fast-path source if already loaded
- `.profiles/` output folder introduced (alongside `.artifacts/` and `.presets/`)
- `profile` structure: no "configurations" concept; `summary.launch_files` is a flat list of launch file filenames as they appear on disk; `detail` is keyed by filename (or `pkg/filename` on clash); no stem-stripping or name derivation
- `profile rescan --config` replaced by `profile rescan --launch-file <filename>`; `profile show --section` matches launch file filenames directly
- `profile` robot type detection: 8 values (`humanoid`, `legged`, `aerial`, `underwater`, `surface_vessel`, `mobile_manipulator`, `arm`, `mobile_base`); open-ended hint sets covering package names and source keywords; additive detection with priority ordering
- Rules: executor starvation diagnostic (Rule 7); real-time scheduling advisory (Rule 0); namespace filtering vocabulary
- `RULES.md` split into five domain files (`RULES-CORE`, `RULES-PREFLIGHT`, `RULES-MOTION`, `RULES-DIAGNOSTICS`, `RULES-REFERENCE`); `RULES.md` becomes a navigation index; AGENTS.md and SKILL.md updated to reference domain files
- SKILL.md rewritten to agentskills.io format (76 → 251 lines): `allowed-tools`, `triggers`, session start checklist, progressive disclosure table, log introspection command reference

### Fixes

- `launch new`: `--preset` was winning over `--param` due to inverted prepend order; fixed to `[preset] + [--param] + [positional]`
- `launch restart`: params and preset were re-applied on restart; fixed to pass `params=None, preset=None`
- `launch new`: config paths embedded unquoted in tmux shell string; fixed with `shlex.quote()`
- `launch new`: session-already-exists message contained literal `{session_name}` (missing `f` prefix); fixed
- `_parse_param_str`: bare args with no separator silently dropped; now passed to arg validator
- `pkg create`: `package_path` used unresolved relative path; fixed to always emit absolute path
- `topics publish-sequence`: added `try/finally` auto-hold — on any exit (completion, exception, `KeyboardInterrupt`), publishes 3 zero-velocity messages; Twist/TwistStamped only via `_has_velocity_fields`; Rule 18 updated to cover both `publish-sequence` and `publish-until`
- `launch new`: positional launch args (e.g. `config:=nav`) were corrupted to `'config'::=nav` and silently dropped; root cause was `_get_launch_arguments()` stripping lines before checking indentation (treating `default:` and `description:` as arg names), followed by a fuzzy-match rename turning `'config':` → `'config'::=nav`; arg validation is now warn-only and always passes args through unchanged
- `TestExhaustiveParser`: removed stale `publish-continuous` entry from `MINIMAL_ARGS`; added `echo-once` and `depth-point` rows; fixed `TestFuzzyMatch` import (`fuzzy_match` in `ros2_utils`, not `_fuzzy_match` in `ros2_launch`); added missing rows for `pkg create` and all four `logs` commands

---

## [1.0.6] - 2026-03-24

Completed the component command group, hardened agent self-recovery behaviour, and added vocabulary and rules for bags, camera calibration, log directories, and Nav2 goal preemption.

### New Commands

- `component list` / `component ls` — list all running component containers and their loaded components
- `component load <container> <package> <plugin>` — load a composable node into an existing container
- `component unload <container> <unique_id>` — unload a composable node by its unique ID
- `component standalone <package> <plugin>` — start a fresh container in a tmux session and load the plugin in one step; container named `standalone_<plugin_class>` (e.g. `demo_nodes_cpp::Talker` → `/standalone_talker`)
- `component kill <session>` — kill a standalone container session (`comp_*` prefix); companion to `run kill` and `launch kill`

### Changes

- Fixed `component standalone` container path for `component_container_isolated` (`/{name}/_container`); timeout error now distinguishes alive-but-slow, service at alternate path, and crashed; orphaned sessions cleaned up automatically on failure
- Fixed `--log-level` argument type to prevent `PyLong_Check` assertion failure
- Agent rules: act on CLI `hint` key immediately; autonomous tmux session error recovery table; banned "you may need to…"; background-launch narration banned; session kill routing by prefix
- Bag vocabulary added (`record a bag`, `play back bag`, `bag info`); camera calibration vocabulary row added; log directory resolution added to session-start rules; Nav2 in-flight goal preemption added as companion to Rule 9
- Documentation: `run kill` corrected to `component kill` everywhere; COMMANDS.md, EXAMPLES.md, CLI.md, README.md updated

---

## [1.0.5] - 2026-03-21

Comprehensive self-reliance review. Added introspection commands, hardened safety and motion rules, introduced AGENTS.md, and made the deceleration zone dynamic.

### New Commands

- `bag info <bag_path>` — bag metadata (duration, message counts, per-topic stats); no live graph required
- `component types` — list registered rclcpp composable node types; no live graph required
- `daemon status` / `daemon start` / `daemon stop` — ROS 2 daemon management
- `params find <pattern> [--node N]` — search all live nodes for params matching a substring
- `tf tree` — ASCII visualization of the full TF frame hierarchy
- `tf validate` — DFS cycle detection and multiple-parent checks across the TF graph
- `topics qos-check <topic>` — compare publisher/subscriber QoS profiles; suggests fix flags
- `launch list <keyword>` — find launch files across installed packages by keyword (no keyword = existing session list behaviour)
- `pkg list` / `pkg ls` — list all installed packages; no live graph required
- `pkg prefix <package>` — resolve the install prefix for a package
- `pkg executables <package>` — list executable files provided by a package
- `pkg xml <package>` — output the `package.xml` manifest for a package

### Changes

- `publish-until` deceleration zone auto-computed from kinematics; proximity sensor scan before long motions; velocity-limit scan extended to four sources; QoS auto-matching in `ConditionMonitor`
- Rules: `estop` is a hard preemption; QoS + monitor field pre-flight mandatory; motion ceilings; `publish-until` > 30 s segmented; simulated clock re-verified before every timed command; TF staleness + cycle detection pre-flight; node crash monitoring for commands > 10 s
- Fixes: `scale_twist_velocity` Twist/TwistStamped branching; `cmd_actions_details` None guard; `cmd_version` reports `rclpy_available`; unique per-topic node names
- `AGENTS.md` added — condensed operational guide covering session-start, core rules, movement, and safety

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
