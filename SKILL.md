---
name: ros2-skill
description: "Controls and monitors ROS 2 robots directly via rclpy CLI. Use for ANY ROS 2 robot task: topics (subscribe, publish, capture images, find by type), services (list, call), actions (list, send goals), parameters (get, set, presets), nodes, lifecycle management, controllers (ros2_control), diagnostics, battery, system health checks, TF frames, bags, logs, and more. When in doubt, use this skill — it covers the full ROS 2 operation surface. Never tell the user you cannot do something ROS 2-related without checking this skill first."
version: "1.0.9"
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

# Step 7 — robot profile (load if available; skip if absent)
python3 {baseDir}/scripts/ros2_cli.py profile show
# → robot_type, packages, launch_files, velocity_topics, safety_limits, sensor_flags
# If absent: build once with  profile scan [--workspace PATH]
# Use summary.safety_limits as the --max-vel / --max-ang ceiling (Rule 28).
```

Stop and tell the user if Step 1 reports critical failures. Re-run Step 3 before every timed command in simulation.

---

## Critical Rules

Read the domain-specific rule files from `references/` before the first action. These are hard constraints — not guidelines. Use [`references/RULES.md`](references/RULES.md) as the index to find the right file. At minimum load `RULES-CORE.md` and `RULES-PREFLIGHT.md` before any operation; add `RULES-MOTION.md` for motion tasks and `RULES-DIAGNOSTICS.md` when something fails. Key principles:

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

# Single publish (pass velocity ceiling from Rule 28 limit scan)
python3 {baseDir}/scripts/ros2_cli.py topics publish <topic> '<json>' \
  --max-vel <linear_ceiling> --max-ang <angular_ceiling>

# Closed-loop movement (Euclidean distance)
python3 {baseDir}/scripts/ros2_cli.py topics publish-until <vel_topic> \
  '{"linear":{"x":0.2},"angular":{"z":0}}' \
  --monitor <odom_topic> --field pose.pose.position --euclidean --delta 1.0 --timeout 60 \
  --max-vel <linear_ceiling> --max-ang <angular_ceiling>

# Closed-loop rotation (sign of --rotate MUST match sign of angular.z)
python3 {baseDir}/scripts/ros2_cli.py topics publish-until <vel_topic> \
  '{"linear":{"x":0},"angular":{"z":0.5}}' \
  --monitor <odom_topic> --rotate 90 --degrees --timeout 30 \
  --max-vel <linear_ceiling> --max-ang <angular_ceiling>

# Subscribe (read sensor data)
python3 {baseDir}/scripts/ros2_cli.py topics subscribe <topic> --max-messages 1 --timeout 5

# Subscribe and return the first message only, then exit
python3 {baseDir}/scripts/ros2_cli.py topics echo-once <topic> [--timeout 5]
```

**`--max-vel N` / `--max-ang N`** (Twist / TwistStamped only): clamp linear x/y/z to ±N m/s and angular.z to ±N rad/s inside the CLI before the message is sent. Pass the binding velocity ceiling discovered in Rule 28 here. Clamped axes are reported in `velocity_clamped` in the JSON output. Other message types pass through unchanged.

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

# Read depth at a specific pixel from a depth image topic (16UC1 or 32FC1; no numpy/cv2 needed)
python3 {baseDir}/scripts/ros2_cli.py topics depth-point --topic <depth_topic> --u <col> --v <row>
# Returns: {depth_m, invalid, encoding, width, height}
```

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

### Log Introspection

Works without a live ROS 2 graph — reads `~/.ros/log/` (or `$ROS_LOG_DIR`).

```bash
# Discover available runs (newest first)
python3 {baseDir}/scripts/ros2_cli.py logs list-runs [--limit 20]

# Query entries with filters (default: latest run, all severities)
python3 {baseDir}/scripts/ros2_cli.py logs query [--run <id>] [--severity WARN] \
  [--node <name>] [--after -30s] [--before -5m] [--text <substr>] [--regex <pat>] \
  [--max 200]

# Incremental tail — only entries since the last call (persists offsets)
python3 {baseDir}/scripts/ros2_cli.py logs tail [--run <id>] [--initial-lines 50] \
  [--reset]

# Per-node statistics: totals, severity breakdown, top recurring messages
python3 {baseDir}/scripts/ros2_cli.py logs node-summary [--run <id>] [--top 5]
```

**Time filter formats for `--after` / `--before`:** `-30s`, `-5m`, `-2h` (relative to now), epoch float, or ISO datetime (`2026-04-30T10:00:00`).

**`logs tail` workflow:** Call once to seed offsets (returns last ~50 lines); call again during the run to get only new entries. Use `--reset` to re-seed when switching to a new run.

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

### Robot Profile (no live graph required)

Build once; load every session. The profile captures robot type, packages, launch files (as filenames from the workspace), velocity topics, safety limits, and sensor flags — eliminating per-session re-discovery.

```bash
# Scan the workspace and write .profiles/<robot>_profile.json
python3 {baseDir}/scripts/ros2_cli.py profile scan [--workspace /path/to/ros2_ws] [--name my_robot]

# Add --allow-live to also query the live graph for topics static analysis missed
python3 {baseDir}/scripts/ros2_cli.py profile scan --allow-live

# Override auto-detected robot type (e.g. if detection mis-classifies)
python3 {baseDir}/scripts/ros2_cli.py profile scan --robot-type mobile_base

# Load summary (robot_type, packages, launch_files, velocity_topics, safety_limits)
python3 {baseDir}/scripts/ros2_cli.py profile show

# Load a specific section
python3 {baseDir}/scripts/ros2_cli.py profile show --section summary
python3 {baseDir}/scripts/ros2_cli.py profile show --section detail
python3 {baseDir}/scripts/ros2_cli.py profile show --section <launch-filename>   # e.g. bringup.launch.py

# Full rescan (workspace changed) — accepts --robot-type override too
python3 {baseDir}/scripts/ros2_cli.py profile rescan [--workspace PATH]

# Partial rescan — only refresh launch args for one file (fast)
python3 {baseDir}/scripts/ros2_cli.py profile rescan --launch-file <launch-filename>

# List all saved profiles
python3 {baseDir}/scripts/ros2_cli.py profile list
```

**Profile JSON shape:**
```
{
  "summary": {                          ← always load; compact
    "robot_type": "mobile_base",        ← primary type (see types below)
    "robot_features": ["pantilt"],      ← supplementary features alongside primary type
    "robot_type_evidence": {            ← signals that drove the detected type
      "mobile_base": ["topic:/cmd_vel", "ament:nav2"],
      "pantilt": ["pkg:my_pantilt_driver"]
    },
    "packages": ["my_bringup", "my_nav"],
    "launch_files": ["bringup.launch.py", "nav2_bringup.launch.py"],  ← filenames from workspace; keys into detail
    "urdf_files": ["/path/to/robot.urdf.xacro"],
    "velocity_topics": ["/cmd_vel"],
    "safety_limits": {"linear_max": 0.5, "angular_max": 1.0, "source": "yaml_or_urdf"},
    "has_lidar": true, "has_camera": true, "has_imu": true, "has_nav2": false
  },
  "detail": {                           ← load on demand per launch file
    "bringup.launch.py": {
      "path": "...", "package": "...",
      "launch_args": {"use_sim_time": "false", "robot_name": null},
      "includes": [                      ← sub-launch files included by this file
        {
          "source": "pkg:nav2_bringup/launch/bringup_launch.py",
          "package": "nav2_bringup", "file": "bringup_launch.py",
          "args_forwarded": {            ← "$ref" = pass-through; literal = hardcoded
            "use_sim_time": "$use_sim_time",
            "map_yaml_file": "$map_yaml"
          }
        }
      ],
      "yaml_files": [...], "urdf_files": [...], "joint_limits": {...}
    },
    ...
  }
}
```

**Robot type values:** `humanoid` · `legged` · `aerial` · `underwater` · `surface_vessel` · `mobile_manipulator` · `arm` · `mobile_base` · `unknown`

**Detection rules (applied strictly in this order):**
1. Humanoid — specific platform package name (NAO, Atlas, Valkyrie …) **or** ≥ 4 URDF joints with torso/neck/shoulder/elbow patterns. Generic terms like "walking"/"balance" are intentionally excluded (too broad).
2. Legged — specific quadruped/hexapod package name (Spot, Go1, ANYmal …) or ≥ 4 URDF joints with FL/FR/RL/RR leg naming.
3. Aerial, underwater, surface\_vessel — platform-specific package names or source keywords.
4. Mobile manipulator — both mobile-base *and* arm signals present.
5. Arm — arm/gripper/MoveIt package or ≥ 3 non-wheel URDF joints.
6. Mobile base — velocity topics (`/cmd_vel`) or Nav2 present, or explicit diff-drive/Ackermann packages.

**If detection is wrong:** run `profile scan --robot-type <type>` to override. The evidence field always shows what matched so you can see why a type was chosen.

Use `summary.safety_limits.linear_max` / `summary.safety_limits.angular_max` as the `--max-vel` / `--max-ang` ceiling (Rule 28). Use `summary.launch_files` to see what launch files exist in the workspace; load any one's full detail with `--section <filename>`.

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
| `logs list-runs` | List available ROS 2 log runs in the log directory |
| `logs query` | Filter log entries by severity, node, time, text |
| `logs tail` | Incremental log reading (new entries since last call) |
| `logs node-summary` | Per-node log statistics for a run |
| `component types` | List registered composable node types |
| `--help` on any command | Inspect flags and subcommands |
| `pkg list / prefix / executables / xml` | Package introspection |
| `pkg create <name> [flags]` | Scaffold a new ROS 2 package |
| `profile scan [--workspace PATH] [--robot-type TYPE]` | Build robot profile from workspace (static-first); `--robot-type` overrides detection |
| `profile show [--section S]` | Show saved robot profile or a section |
| `profile rescan [--launch-file F]` | Update existing profile (full or partial) |
| `profile list` | List all saved robot profiles |

---

## Reference Files

This skill uses **progressive disclosure**. SKILL.md covers the most common operations. Load the files below when the task requires deeper detail.

| File | When to load |
|---|---|
| [`references/RULES.md`](references/RULES.md) | **Index only** — maps each rule number to its domain file. Load first to navigate the rule set. |
| [`references/RULES-CORE.md`](references/RULES-CORE.md) | **Always load** — general agent conduct (Rules 0.5, 1, 2, 4–6, 10–13). Hard constraints that apply to every command. Includes mandatory compliance preamble and Quick Decision Card. |
| [`references/RULES-PREFLIGHT.md`](references/RULES-PREFLIGHT.md) | **Load at session start and before any action** — full pre-flight introspection protocol (Rule 0), session-start steps 0–6 (Rule 0.1), lifecycle/QoS/publisher checks (Rules 14, 15, 19). |
| [`references/RULES-MOTION.md`](references/RULES-MOTION.md) | **Load for any motion command** — movement algorithm (Rule 3), pre-motion check + Nav2 preemption (Rule 9), REP-103/105 (Rule 17), estop (Rule 18), decel zone (Rule 20), timeout recovery (Rule 21), command limits (Rules 22–23), sequencing (Rule 24), proximity scan (Rule 25). |
| [`references/RULES-DIAGNOSTICS.md`](references/RULES-DIAGNOSTICS.md) | **Load when something fails** — failure diagnosis + log-level elevation (Rule 7), post-action verification table (Rule 8), multi-step sequencing (Rule 16), Error Recovery Protocols. |
| [`references/RULES-REFERENCE.md`](references/RULES-REFERENCE.md) | **Load for command lookup** — full intent→command table (Step 1), sensor search by type (Steps 2–3), message structure (Step 4), velocity limits (Step 5), Launch workflow, Discord image delivery (Rule 26), Setup. |
| [`references/COMMANDS.md`](references/COMMANDS.md) | Load when you need the exact flag name, argument format, or JSON output structure for a specific command. 4535 lines — use `--help` on the specific subcommand first; only load this file if `--help` is insufficient or unavailable. |
| [`references/EXAMPLES.md`](references/EXAMPLES.md) | Load for step-by-step walkthroughs of common tasks (move N meters, capture camera image, send Nav2 goal, etc.). 699 lines. |
| [`references/CLI.md`](references/CLI.md) | Load for direct `ros2` CLI equivalents and debugging. Not needed during normal agent operation. 90 lines. |
| [`AGENTS.md`](AGENTS.md) | Load for the full agent operating protocol — condensed rules, session start detail, reporting style, subcommand inference, motion workflows, and multi-robot handling. Load alongside the RULES-*.md files at session start. |
