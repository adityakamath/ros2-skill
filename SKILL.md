---
name: ros2-skill
description: Operate and inspect a live ROS 2 robot through a JSON CLI (ros2_cli.py). Use for topics (subscribe, publish, camera capture), services, actions, parameters, nodes, lifecycle, ros2_control controllers, closed-loop motion (publish-until), Nav2 (go, rotate, waypoints, places, missions), TF, diagnostics, battery, logs, bags, robot profiles and safety limits. Use whenever the user wants to drive, monitor or debug a robot, even if they don't say "ROS 2".
allowed-tools: ["Bash", "Read"]
---

# ROS 2 Skill

Every command is `python3 ${CLAUDE_SKILL_DIR}/scripts/ros2_cli.py <command> [subcommand] [args]` and returns JSON. Below, `CLI` means that prefix. Never call `ros2` directly or run another `scripts/ros2_*.py` file. The path is substituted when the skill loads; the base directory is also shown at the top of the loaded skill. `CLI --help` and `CLI <command> --help` list every command and flag; trust them over memory. Global options go before the command: `--timeout SECONDS`, `--retries N`.

Every task is **resolve → act → verify**:
- **Resolve:** read static facts (topic names, types, limits, frames, controllers) from the robot profile.
- **Act:** run the command.
- **Verify:** read the post-action state. A zero-error return only means the request was delivered.

## Fast path

The fast daemon starts itself in the background the first time a command that benefits runs (that command runs normally; the next ones are fast) and exits after 30 idle minutes. `CLI daemon-fast start` starts it without the idle exit, `daemon-fast stop` stops it, `ROS2_SKILL_NO_AUTOSTART=1` keeps it from starting itself. It keeps the sensors subscribed and the velocity publisher warm, so `snapshot`, `look` and the pre-motion check answer in about 0.3 s instead of 1.5-2 s, and a closed-loop move (`plan --run`, `topics publish-until`, `stop`) starts in about 0.2 s instead of 1.2 s (`ROS2_SKILL_NO_FASTD=1` turns this off).

```bash
CLI daemon-fast start
```

**State questions** ("how's the battery", "where is the robot", "what's in front", "is it level", "is the e-stop on", "anything wrong") need one call, not several:

```bash
CLI snapshot            # LiDAR sectors, pose, tilt, battery, ToF, depth at centre, wheels, e-stop/mode, controllers, Nav2 nodes, diagnostics
CLI snapshot --only battery,state      # a subset
```

**Pictures and "what do you see"**: `CLI look` saves the colour frame and prints its `path`; read that file to see it. `CLI look --diff 1` takes a second frame a second later and reports `changed` (compared in code, no need to view both).

**Plain goals** ("drive forward 1 m at top speed", "turn left 90 degrees", "go to the dock", "back up 30 cm", "save this spot as dock and list my places", "take a picture"): one call, without reading the rules or the profile first:

```bash
CLI plan "move forward 1 m at top speed" --run
```

It switches the drive mode if needed, runs the pre-motion check, runs each step with the profile's speed limits, stops and verifies. A blocked check means no motion. A failed step ends the run: it latches the e-stop (tell the user how to release it), except when the report has a `cause` (a collision monitor or smoother cut the speed; the robot is stopped normally and nothing is latched). A move that finishes but took over twice its cruise time carries `slow` in its step result (a limiter was cutting the speed). If any part of the goal is not understood, nothing runs (`refused`, with `warnings`). `estop`, Ctrl-C or `kill` during a run stops it (`cancelled: true`): velocity is zeroed and a Nav2 goal is cancelled. Report the result in one line (`moved_m`, `ok`, `cause`, `slow`). Only if it reports `blocked`, `ok: false` or warnings, or the goal is not a plain one, load the rules below.

Goals that chain in one call: "rotate 90 then take a picture then rotate back", "move forward 0.3 m, take a picture, and report the battery", "turn around", "undo that" / "go back" (reverses the previous move, strafe or turn in the same goal), "is the path ahead clear" / "what is around me" (LiDAR, ToF, depth), "look at the desk", "remember that the dock is by the door", "what do you remember about the dock".

**Memory:** `CLI notes remember <text> [--tag T]`, `notes recall [words]`, `notes forget <id>` (kept across sessions and profile switches); `CLI nav2 place save <name> --snapshot` also stores a camera frame with the place.

**Braking:** `CLI braking measure` drives two 0.4 m moves with the slow-down zone off and stores the measured deceleration; `publish-until` then uses it instead of the 0.5 m/s² default (`braking show|clear`). Run it once per robot, on the floor it works on.

## Rules to load

Before the first action that the fast path does not cover, read these from `references/`:

| File | Load |
|---|---|
| [RULES-CORE.md](references/RULES-CORE.md) | Always. Conduct, Path A discipline, reporting. |
| [RULES-PREFLIGHT.md](references/RULES-PREFLIGHT.md) | Always at session start. Profile fast-path table, session checks, QoS and lifecycle. |
| [RULES-MOTION.md](references/RULES-MOTION.md) | Before any motion (base, arm or Nav2). |
| [RULES-DIAGNOSTICS.md](references/RULES-DIAGNOSTICS.md) | When something fails, and for the verify-the-effect table. |
| [RULES-REFERENCE.md](references/RULES-REFERENCE.md) | To map an intent to a command. |
| [GUIDES.md](references/GUIDES.md) | Safety policy, missions, places, capabilities, collision-aware pipelines, camera rotation. |
| [PROFILE.md](references/PROFILE.md) | Profile field reference. |
| [COMMANDS.md](references/COMMANDS.md) | Exact flags and output. Large: use `--help` first. |
| [EXAMPLES.md](references/EXAMPLES.md) | Worked walkthroughs. |

## Hard constraints

1. **Processes are the user's.** Never run `launch new|restart|kill`, `run new` or `component standalone`. Give the user the exact command and continue when they say it is up. Reading (`launch list`, `profile show`) is fine.
2. **Motion** uses closed loop (`publish-until`) when odometry is usable. Always pass `--max-vel` / `--max-ang` from the profile's safety limits. Run `preflight motion` first, and `stop --verify-odom-topic <ODOM>` after every `publish-until` / `publish-sequence` (`estop` instead after a timeout or error). Details are in RULES-MOTION.md.
3. **Never invent a command or flag.** Check `--help`.
4. **Fix failures yourself** (introspect, correct, retry) before reporting. Report outcomes in one line.
5. **Emergency stop:** `CLI estop --verify-odom-topic <ODOM>`; `CLI stop` is the routine, non-latching version. `estop` engages the robot's SetBool e-stop service (if any), interrupts a running motion command, and zeroes **every** velocity topic. If verification shows non-zero velocity after 5 s (10 s above 20 kg), treat it as critical and escalate. Releasing a service e-stop is the operator's decision; never clear it yourself.
6. **One motion command at a time** is enforced: a second `publish-until`, `publish-sequence` or `nav2 go|rotate|move-timed|place go` is refused while one runs.

## Session start

Run once per session (full detail in RULES-PREFLIGHT.md Rule 0.1):

```bash
CLI doctor --exclude-packages          # stop and tell the user on critical failures
CLI daemon-fast start                  # speedup: snapshot, look and the pre-motion check drop to ~0.3 s; safe to skip
CLI profile show                       # decides Path A or Path B
CLI context                            # graph snapshot (only if Path B, or you need the graph)
```

If a simulated clock exists (`topics find rosgraph_msgs/msg/Clock`), confirm it advances before every timed command.

**Path A (profile loaded, `summary` non-empty):** read static facts from the profile and do not run `topics find`, `topics type`, `tf list`, `services find` or a `params list` limit sweep for anything it holds. The CLI refuses those calls and names the profile field (`--ignore-profile` overrides, and is logged). Live calls are only for runtime state:

| Need | Profile field |
|---|---|
| Velocity topic / type | `summary.cmd_vel_topic` / `summary.velocity_topics[].type` |
| Odometry topic | first of `summary.localization_config.fused_sources` |
| `--max-vel` / `--max-ang` | `summary.safety_limits.binding.linear_x` / `.angular_z` |
| TF frames, e-stop service, drive type | `summary.tf_frames`, `summary.estop_config.service_name`, `summary.drive_type` |
| Joint commands | `summary.joint_command_topics[]` (topic, controller and joint order; always `std_msgs/msg/Float64MultiArray`) |
| IMU, GPS, LiDAR | `summary.imu_topics[]`, `summary.gps_topics[]`, `summary.lidar_scan_topic` |
| Robot specifics | `summary.estop_config` (idempotent SetBool), `summary.velocity_chain` (teleop/Nav2 inputs and `mode_service`), `summary.robot_services`, `summary.hardware_fitted` |
| Always fixed | `/diagnostics`, `/joint_states` |

If a profile value disagrees with the live graph, **stop and report it** (Rule 0.0b); don't search for a replacement. A missing field means live discovery for that field only.

**Path B (no profile):** discover everything live. To build a profile, run `profile scan --name <robot>` (one robot), `profile scan --packages a,b --name label` (several), or bare `profile scan`. Scan only on user request, first bootstrap, or after a user-run launch when sources changed (Rules 0.6, 0.7).

## Core commands

```bash
# Introspection
CLI topics list|find <type>|type <t>|details <t>|hz <t>|bw <t>|diag|battery
CLI nodes list|details <n>;  CLI services list|details <s>;  CLI actions list|details <a>
CLI interface proto <type>              # payload template (always before publishing; never from memory)
CLI tf list|lookup <src> <tgt>|echo|monitor|validate

# Read and publish
CLI topics subscribe <t> --max-messages 1 --timeout 5
CLI topics publish <t> '<json>' --max-vel V --max-ang W            # --dry-run validates without sending
CLI topics publish-sequence <t> '[<move>,<zero>]' '[<secs>,0.5]' --max-vel V --max-ang W
CLI topics publish-until <t> '<json>' --monitor <ODOM> --field pose.pose.position --euclidean --delta 1.0 --timeout T --max-vel V --max-ang W
CLI topics publish-until <t> '<json>' --monitor <ODOM> --rotate 90 --degrees --timeout T --max-vel V --max-ang W
CLI stop --verify-odom-topic <ODOM>     # routine stop: zero velocity everywhere, wait until stationary, report final position; never latches the e-stop
CLI topics capture-image --topic <t> --output ${CLAUDE_SKILL_DIR}/.artifacts/<name>.jpg
CLI topics depth-point --topic <t> --u <col> --v <row>

# Pre-motion (one node spin: controller active, robot stationary, plus e-stop, drive mode and battery when the profile knows them;
# `blocked_by` says why ready_for_motion is false and how to fix it)
CLI preflight motion --controller <NAME> --odom-topic <ODOM> [--source teleop|nav2]
CLI preflight joint-command --controller <NAME> --joints j1,j2

# Services, actions, parameters, controllers, lifecycle
CLI services state <svc> [<svc> ...]   # current state of SetBool toggles (e-stop, twist switch); null = never toggled
CLI services call <s> '<json>';  CLI actions send <a> '<json>' --timeout 60;  CLI actions cancel <goal_id>
CLI params list|describe|get|set <node[:param]> [value]
CLI control list-controllers|list-hardware-components|switch-controllers --activate X --deactivate Y --strictness STRICT
CLI lifecycle nodes|get <n>|set <n> configure|activate

# Health, logs, packages, bags
CLI doctor [hello|diagnostics --level warn];  CLI system battery
CLI logs list-runs|query --severity WARN --after -30s|tail|node-summary
CLI pkg list|prefix|executables|xml <pkg>;  CLI bag info <file>
```

`CLI plan "<goal>" [--run]` turns a multi-step goal ("move forward 1 m then turn left 90", "go to the dock", "patrol a, b", "look around 4 views", "record waypoint", "save the map") into concrete commands (including twist-switch mode changes when the velocity input changes) with units, signs, timeouts and `--max-vel/--max-ang` already computed, using the profile and saved places. Without `--run` it executes nothing; unknown places or missing capabilities become warnings. With `--run` it executes the steps (see the fast path above).

Pan-tilt, arm and gripper position commands: `preflight joint-command`, publish one `{"data":[...]}` to the profile's `joint_command_topics[].topic`, then read `/joint_states` once to verify.

## Nav2

```bash
CLI nav2 status                                  # check for an active goal first; `nav2 cancel` if one is running
CLI nav2 go X Y [--yaw DEG] [--timeout 120] [--feedback]
CLI nav2 go-waypoints 1.0,0.0 2.0,1.5 [--no-stop-on-failure]
CLI nav2 rotate DEG                              # obstacle-aware; use publish-until only when Nav2 is down
CLI nav2 move-timed --duration 2 --speed 0.3 [--direction fwd|back|left|right]
CLI nav2 costmap-clear [--layer local|global|both];  CLI nav2 initial-pose X Y --yaw DEG
CLI nav2 cancel                                  # cancels goals and sends a zero-velocity burst
CLI nav2 place save|list|go|delete <name>;  CLI nav2 map list|save|load|delete;  CLI nav2 mode get|set
CLI nav2 mission run --steps '<json>'|--file F;  CLI nav2 mission status|pause|resume|cancel <id>
```

`nav2 go` returns `success`, `status` (4 = SUCCEEDED) and `goal`. Missions, places, the safety policy (`safety show|set-velocity|set-geofence|validate|heartbeat`) and collision-aware testing are in [GUIDES.md](references/GUIDES.md).

## Files the skill writes

All outputs go inside the skill directory, never `/tmp`:

| Path | Contents |
|---|---|
| `.artifacts/` | captures, mission and place state, heartbeat files |
| `.presets/` | parameter presets, `safety.json` |
| `.profiles/` | robot profiles |
| `.fastd/` | `daemon-fast` socket and pid |
