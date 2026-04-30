---
name: ros2-skill
description: "Controls and monitors ROS 2 robots directly via rclpy CLI. Use for ANY ROS 2 robot task: topics (subscribe, publish, capture images, find by type), services (list, call), actions (list, send goals), parameters (get, set, presets), nodes, lifecycle management, controllers (ros2_control), diagnostics, battery, system health checks, TF frames, bags, logs, and more. When in doubt, use this skill — it covers the full ROS 2 operation surface. Never tell the user you cannot do something ROS 2-related without checking this skill first."
version: "1.0.8"
license: Apache-2.0
compatibility: "python3, rclpy, ROS 2 environment sourced"
allowed-tools: ["Bash", "Read"]
triggers:
  - "ros2"
  - "robot.*topic|topic.*robot"
  - "publish.*topic|subscribe.*topic|listen.*topic"
  - "move.*robot|drive.*robot|robot.*move|robot.*drive"
  - "navigate|nav2|send.*goal|waypoint"
  - "ros2_control|controller.*manager|controller.*switch"
  - "lifecycle.*node|node.*lifecycle"
  - "TF.*frame|transform.*frame|frame.*transform"
  - "ros.*param|parameter.*ros"
  - "launch.*ros|ros.*launch"
  - "bag.*record|bag.*play|ros.*bag"
  - "ros.*log|node.*log"
  - "estop|emergency.*stop|stop.*robot"
  - "camera.*ros|ros.*camera|capture.*image.*robot"
  - "ros.*doctor|health.*check.*ros"
metadata:
  openclaw:
    emoji: "🤖"
    requires:
      bins: ["python3", "ros2"]
      pip: ["rclpy"]
    category: "robotics"
    tags: ["ros2", "robotics", "rclpy"]
  author: ["adityakamath", "lpigeon"]
---

# ROS 2 Skill

Provides a structured JSON interface to a live ROS 2 robot. All commands output JSON. Every skill invocation follows three mandatory phases: **introspect → act → verify**. Never skip any phase.

## Entry Point

**Always use `ros2_cli.py`. Never call `ros2` directly or run submodules directly.**

```bash
python3 {baseDir}/scripts/ros2_cli.py <command> [subcommand] [args]
python3 {baseDir}/scripts/ros2_cli.py --help           # list all commands
python3 {baseDir}/scripts/ros2_cli.py <command> --help # list subcommands
```

`{baseDir}` is the path to the skill root directory. Resolve it from the skill metadata before running any command.

---

## Session Start Checklist

Run once per session before the first task. Takes seconds. Catches the most common silent failures.

```bash
# Step 1 — health check
python3 {baseDir}/scripts/ros2_cli.py doctor

# Step 2 — daemon
python3 {baseDir}/scripts/ros2_cli.py daemon status
python3 {baseDir}/scripts/ros2_cli.py daemon start      # if not running

# Step 3 — simulated time (if applicable)
python3 {baseDir}/scripts/ros2_cli.py topics find rosgraph_msgs/msg/Clock

# Step 4 — lifecycle nodes (if present)
python3 {baseDir}/scripts/ros2_cli.py lifecycle nodes

# Step 5 — log directory (no command — note for diagnostics)
# Resolve: $ROS_LOG_DIR → $ROS_HOME/log/ → ~/.ros/log/

# Step 6 — graph snapshot
python3 {baseDir}/scripts/ros2_cli.py context          # topics (cap 50), services, actions, nodes
```

Stop and tell the user if Step 1 reports critical failures. Re-run Step 3 before every timed command in simulation.

---

## Critical Rules

Read [`references/RULES.md`](references/RULES.md) in full before the first action. These are hard constraints — not guidelines. Key principles:

1. **Discover before acting.** Never hardcode topic, node, service, action, or TF frame names. Always query the live system first. Use `topics find <type>`, `nodes list`, `services list`, `actions list`, `tf list`.

2. **Get the payload template.** Before publishing or calling: `interface proto <msg_type>`. Copy the output. Modify only the fields the task requires. For non-primitive nested fields, run `interface show <nested_type>` recursively until all leaf fields are primitives.

3. **Verify every effect.** A zero-error CLI response means the request was delivered — not that the effect occurred. Always follow up: `params get`, `control list-controllers`, subscribe to confirm, etc.

4. **Diagnose before reporting.** On any error: introspect → self-correct → retry → report. Never ask the user to interpret an error you can check with the CLI.

5. **Safety is non-negotiable.** Velocity limits, pre-motion odom checks, and post-motion verify are mandatory. Never bypass them.

6. **Context discovery is parallel.** When discovering multiple independent facts (velocity topic, odom topic, limits, controller state), issue all commands simultaneously — never sequentially.

---

## Common Operations

### Introspection

```bash
python3 {baseDir}/scripts/ros2_cli.py context                             # session-start graph snapshot
python3 {baseDir}/scripts/ros2_cli.py topics list [--limit N]             # list topics (cap at N)
python3 {baseDir}/scripts/ros2_cli.py topics find geometry_msgs/msg/Twist # find velocity topic
python3 {baseDir}/scripts/ros2_cli.py topics find nav_msgs/msg/Odometry   # find odom topic
python3 {baseDir}/scripts/ros2_cli.py topics type <topic>                 # confirm exact type
python3 {baseDir}/scripts/ros2_cli.py topics details <topic>              # publisher count + QoS
python3 {baseDir}/scripts/ros2_cli.py topics hz <topic>                   # confirm topic is live
python3 {baseDir}/scripts/ros2_cli.py nodes list
python3 {baseDir}/scripts/ros2_cli.py services list
python3 {baseDir}/scripts/ros2_cli.py actions list
python3 {baseDir}/scripts/ros2_cli.py tf list                             # discover TF frames
```

### Publishing

```bash
# Get payload template first — always
python3 {baseDir}/scripts/ros2_cli.py interface proto geometry_msgs/msg/Twist

# Single publish
python3 {baseDir}/scripts/ros2_cli.py topics publish <topic> '<json>'

# Closed-loop movement (Euclidean distance)
python3 {baseDir}/scripts/ros2_cli.py topics publish-until <vel_topic> \
  '{"linear":{"x":0.2},"angular":{"z":0}}' \
  --monitor <odom_topic> --field pose.pose.position --euclidean --delta 1.0 --timeout 60

# Closed-loop rotation (sign of --rotate MUST match sign of angular.z)
python3 {baseDir}/scripts/ros2_cli.py topics publish-until <vel_topic> \
  '{"linear":{"x":0},"angular":{"z":0.5}}' \
  --monitor <odom_topic> --rotate 90 --degrees --timeout 30

# Subscribe (read sensor data)
python3 {baseDir}/scripts/ros2_cli.py topics subscribe <topic> --max-messages 1 --timeout 5
```

### Services and Actions

```bash
python3 {baseDir}/scripts/ros2_cli.py services details <service>          # request/response fields
python3 {baseDir}/scripts/ros2_cli.py services call <service> '<json>'
python3 {baseDir}/scripts/ros2_cli.py actions details <action>            # goal/result/feedback fields
python3 {baseDir}/scripts/ros2_cli.py actions send <action> '<json>' --timeout 60
python3 {baseDir}/scripts/ros2_cli.py actions cancel <goal_id>
```

### Parameters

```bash
python3 {baseDir}/scripts/ros2_cli.py params list <node>
python3 {baseDir}/scripts/ros2_cli.py params describe <node:param>        # type, range, read-only
python3 {baseDir}/scripts/ros2_cli.py params get <node:param>
python3 {baseDir}/scripts/ros2_cli.py params set <node:param> <value>
```

### Controllers and Lifecycle

```bash
python3 {baseDir}/scripts/ros2_cli.py control list-controllers
python3 {baseDir}/scripts/ros2_cli.py control list-hardware-components
python3 {baseDir}/scripts/ros2_cli.py control switch-controllers --activate <name> --deactivate <name>
python3 {baseDir}/scripts/ros2_cli.py lifecycle nodes
python3 {baseDir}/scripts/ros2_cli.py lifecycle get <node>
python3 {baseDir}/scripts/ros2_cli.py lifecycle set <node> activate
```

### Camera and Images

```bash
# Always verify camera_info + TF before using camera data
python3 {baseDir}/scripts/ros2_cli.py topics find sensor_msgs/msg/CameraInfo
python3 {baseDir}/scripts/ros2_cli.py topics subscribe <camera_info_topic> --max-messages 1 --timeout 2
# Confirm K matrix is non-zero and frame_id is in tf list before proceeding
python3 {baseDir}/scripts/ros2_cli.py topics capture-image --topic <camera_topic> --output {baseDir}/.artifacts/<name>.jpg
```

### Safety Validator

The safety validator enforces velocity limits, geofence constraints, and command allow/blocklists
at code level — independent of the prompt-level rules in RULES.md. Config lives in
`{baseDir}/.presets/safety.json`. Defaults apply if the file doesn't exist.

```bash
# Inspect and configure
python3 {baseDir}/scripts/ros2_cli.py safety show                         # show active config + path
python3 {baseDir}/scripts/ros2_cli.py safety enable                       # re-enable globally
python3 {baseDir}/scripts/ros2_cli.py safety disable                      # disable all checks (warns)

# Velocity limits (default: linear 0.5 m/s norm, angular 1.0 rad/s norm)
python3 {baseDir}/scripts/ros2_cli.py safety set-velocity --linear 0.3 --angular 0.8
python3 {baseDir}/scripts/ros2_cli.py safety set-velocity --axis angular_z_max --value 0.5

# Geofence (off by default — must be explicitly enabled)
python3 {baseDir}/scripts/ros2_cli.py safety set-geofence --mode circle --cx 0 --cy 0 --radius 5
python3 {baseDir}/scripts/ros2_cli.py safety set-geofence --mode rectangle --xmin -3 --xmax 3 --ymin -3 --ymax 3
python3 {baseDir}/scripts/ros2_cli.py safety geofence enable
python3 {baseDir}/scripts/ros2_cli.py safety geofence disable

# Command blocklist
python3 {baseDir}/scripts/ros2_cli.py safety block topics publish         # block publish subcommand
python3 {baseDir}/scripts/ros2_cli.py safety block launch                 # block all launch subcommands
python3 {baseDir}/scripts/ros2_cli.py safety unblock topics publish

# Dry-run check (does not publish)
python3 {baseDir}/scripts/ros2_cli.py safety validate \
    --topic /cmd_vel --msg-type geometry_msgs/msg/Twist \
    --msg '{"linear":{"x":1.5},"angular":{"z":0}}'

# Heartbeat watchdog (dead man's switch — off by default)
python3 {baseDir}/scripts/ros2_cli.py safety heartbeat start --timeout 5  # arm watchdog
python3 {baseDir}/scripts/ros2_cli.py safety heartbeat ping               # refresh sentinel
python3 {baseDir}/scripts/ros2_cli.py safety heartbeat status             # check if running
python3 {baseDir}/scripts/ros2_cli.py safety heartbeat stop               # disarm

# Reset to defaults
python3 {baseDir}/scripts/ros2_cli.py safety reset --yes
```

**`on_violation` modes** (set in safety.json `velocity_limits.on_violation`):
- `"reject"` (default) — block the command; agent must stop.
- `"cap"` — clamp velocity proportionally; command executes at reduced speed.
- `"warn"` — execute unchanged; add `safety_warning` to output.

**When the agent receives `"blocked": true`**: stop immediately, report `reason` and `limits`
verbatim to the user, do not retry or work around. Only permitted next actions: ask the user
to relax limits with `safety set-velocity`, or stop the task.

### Diagnostics and Health

```bash
python3 {baseDir}/scripts/ros2_cli.py doctor                              # full DDS/graph health check
python3 {baseDir}/scripts/ros2_cli.py doctor hello                        # DDS multicast test
python3 {baseDir}/scripts/ros2_cli.py topics diag                         # subscribe /diagnostics
python3 {baseDir}/scripts/ros2_cli.py topics battery                      # battery state
```

### Launch

```bash
# Find a launch file by keyword
python3 {baseDir}/scripts/ros2_cli.py launch list <keyword>

# Start a launch file (duplicate detection runs automatically)
python3 {baseDir}/scripts/ros2_cli.py launch new <package> <launch_file>

# With inline params (higher priority than --preset, lower than positional args)
python3 {baseDir}/scripts/ros2_cli.py launch new <package> <launch_file> --param use_sim_time:=true,robot_name:=my_bot

# With a YAML config file or directory (forwarded via --ros-args --params-file)
python3 {baseDir}/scripts/ros2_cli.py launch new <package> <launch_file> --config-path /path/to/config.yaml

# With a saved parameter preset (lowest priority — positional and --param override)
python3 {baseDir}/scripts/ros2_cli.py launch new <package> <launch_file> --preset my_preset

# Manage sessions
python3 {baseDir}/scripts/ros2_cli.py launch list                         # running launch sessions
python3 {baseDir}/scripts/ros2_cli.py launch kill <session>               # stop a session
python3 {baseDir}/scripts/ros2_cli.py launch restart <session>            # restart with same args
```

**Priority order** (ROS 2 last-wins): positional args > `--param` > `--preset`. Positional args always win.
**Duplicate detection**: if the same package + launch file is already running, a warning with `existing_session` is returned instead of launching again.

### Packages, Bags, and TF

```bash
python3 {baseDir}/scripts/ros2_cli.py pkg list                            # installed packages (no graph)
python3 {baseDir}/scripts/ros2_cli.py pkg prefix <package>                # install prefix
python3 {baseDir}/scripts/ros2_cli.py pkg executables <package>           # launchable executables
python3 {baseDir}/scripts/ros2_cli.py pkg xml <package>                   # package.xml contents
python3 {baseDir}/scripts/ros2_cli.py pkg create <name> [--build-type ament_cmake|ament_python|cmake] \
    [--dependencies rclcpp std_msgs] [--node-name my_node] [--library-name my_lib] \
    [--destination-directory /path]                                        # scaffold new package (no graph)
python3 {baseDir}/scripts/ros2_cli.py bag info <bag_path>                 # bag metadata (no graph)
python3 {baseDir}/scripts/ros2_cli.py tf list                             # all TF frames
python3 {baseDir}/scripts/ros2_cli.py tf echo <source_frame> <target_frame>
```

---

## Output Folders

All outputs are written to hidden folders inside the skill directory. Never use `/tmp`.

| Folder | Contents |
|---|---|
| `{baseDir}/.artifacts/` | Captured images, logs, and all generated outputs |
| `{baseDir}/.presets/` | Saved parameter presets (`params preset-save` / `params preset-load`) |
| `{baseDir}/.profiles/` | Robot profiles |

---

## Emergency Stop

Send immediately on any unsafe motion or unexpected robot behaviour:

```bash
python3 {baseDir}/scripts/ros2_cli.py estop
```

After estop: verify velocity ≈ 0 by subscribing `<ODOM_TOPIC> --max-messages 1 --timeout 5`. If velocity is still non-zero after 5 s (10 s for heavy platforms > 20 kg), escalate as critical failure.

---

## Commands Without a Live Graph

These work without ROS running or nodes active:

| Command | Purpose |
|---|---|
| `version` | Verify skill is installed and reachable |
| `daemon status / start / stop` | Manage the ROS 2 daemon |
| `bag info <file>` | Bag metadata (duration, message counts, per-topic stats) |
| `component types` | List registered composable node types |
| `--help` on any command | Inspect flags and subcommands |
| `pkg list / prefix / executables / xml` | Package introspection |
| `pkg create <name> [flags]` | Scaffold a new ROS 2 package |

---

## Reference Files

This skill uses **progressive disclosure**. SKILL.md covers the most common operations. Load the files below when the task requires deeper detail.

| File | When to load |
|---|---|
| [`references/RULES.md`](references/RULES.md) | **Load before the first action in any session.** Contains all mandatory agent behaviour rules (Rules 0–38), safety protocols, velocity limit discovery, motion error lockout, verification protocols, and the full vocabulary trigger table. 1677 lines — use the section headers to navigate: Rule 0 (pre-flight), Rule 0.1 (session start), Rule 3 (motion), Rule 7 (failure diagnosis), Rule 8 (verification), Rule 9 (motion pre-check). |
| [`references/COMMANDS.md`](references/COMMANDS.md) | Load when you need the exact flag name, argument format, or JSON output structure for a specific command. 4535 lines — use `--help` on the specific subcommand first; only load this file if `--help` is insufficient or unavailable. |
| [`references/EXAMPLES.md`](references/EXAMPLES.md) | Load for step-by-step walkthroughs of common tasks (move N meters, capture camera image, send Nav2 goal, etc.). 699 lines. |
| [`references/CLI.md`](references/CLI.md) | Load for direct `ros2` CLI equivalents and debugging. Not needed during normal agent operation. 90 lines. |
| [`AGENTS.md`](AGENTS.md) | Load for the full agent operating protocol — condensed rules, session start detail, reporting style, subcommand inference, motion workflows, and multi-robot handling. Load this alongside RULES.md at session start. |
