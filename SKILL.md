---
name: ros2-skill
description: "Use this skill for any ROS 2 robot task: topics (subscribe, publish, capture images, find by type), services (list, call), actions (list, send goals), parameters (get, set, presets), nodes, lifecycle management, controllers (ros2_control), Nav2 navigation (go, rotate, cancel, costmap-clear, waypoints, move-timed, initial-pose), diagnostics, battery, TF frames, bags, logs, and more. Use even if the user doesn't explicitly say 'ROS 2' — if they describe controlling, monitoring, or inspecting a robot, this skill applies. Never tell the user you cannot do something ROS 2-related without checking this skill first."
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

> **⚠️ ATTENTION — READ FIRST. This block overrides any conflicting instinct or rationalisation below.**
>
> If `profile show` returns a non-empty `summary`, the session is **Path A**. In Path A:
>
> - **Do not run `topics find`, `topics type`, `tf list` (for frame names), `services find`, or a `params list` velocity-limit sweep for any data the profile already holds.** This is a Rule 14 violation, not "extra safety".
> - The profile fields are the source of truth for: `cmd_vel_topic`, velocity message type, odometry topic, velocity safety limits, TF frame names, controller names, e-stop service, joint names.
> - The **only** live calls allowed before motion in Path A: (1) `control list-controllers` for runtime active/inactive state, (2) one odom subscribe for the stationary check, (3) `interface proto <VEL_TYPE>` once per session for the payload template, (4) optional `topics hz` if you have not yet seen the odom rate.
> - **Forbidden rationalisations** (if you find yourself thinking any of these, you are violating Rule 13 — stop and use the profile):
>   - *"Maximum safety requires live introspection."*
>   - *"This catches runtime changes the profile might miss."*
>   - *"The profile is just a reference for static info; control and safety checks must be done live."*
>   - *"Profile data alone is never enough for actuation."*
>   - *"Using the profile is justified **because it was just rescanned** / **because it is fresh**."* — The profile is the source of truth for static data **always** in Path A, not just when fresh. If you think the profile might be stale, run Rule 0.0b escalation (compare to live graph and stop on disagreement); do not bypass with live discovery.
> - The single test before any introspection: *"Is this field present in the profile?"* — yes → use it; no → fall back to live for **that one field only** (Rule 0.0a); disagrees with live graph → stop and escalate (Rule 0.0b). The path does not flip.
>
> Detailed Path A operational rules: see "Path A operational summary" section below. Authoritative rules: `references/RULES-CORE.md` Rule 13 + Rule 14, `references/RULES-MOTION.md` Rule 3 Step 1.
>
> **Structural enforcement (since v1.0.8):** the CLI itself refuses `topics find` / `services find` calls that violate Path A. The refusal JSON names the matching profile field. `profile show` always includes a `path_a_reminder` block listing forbidden commands and the profile fields that replace them. Read that block at session start — it is the contract. If the guard refuses a call you genuinely need (Path B / debug), pass `--ignore-profile`; that override is logged.

Provides a structured JSON interface to a live ROS 2 robot. All commands output JSON. Every skill invocation follows three mandatory phases: **resolve → act → verify**. **Resolve** means: read every static field (topic names, message types, limits, frame names, controller names) from the profile first (Path A) and run a live call **only** when (a) the profile is absent / does not have that field (Path B / Rule 0.0a), or (b) the value is dynamic runtime state — controller active/inactive, robot stationary, payload template, odom rate. **"Resolve" is not a synonym for "introspect everything live"** — in Path A it collapses to zero live calls for profile-covered fields. **Act** issues the command. **Verify** reads post-action state. Never skip any phase, and never expand "resolve" into full live discovery when Path A is active.

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
#
# Path classification (decided once, here):
#   profile show succeeded AND summary is non-empty  → Path A (profile-first)
#   profile show failed OR summary is empty          → Path B (live introspection for everything)
# Do NOT re-classify per task. Auto-rescan, freshness gating, and `profile scan` triggers:
# see RULES-PREFLIGHT.md Rules 0.0, 0.6, 0.7 (loaded at session start).
#
# If absent: build once, using these rules derived from the user's request:
#
#   ONE robot named  → pass --name <robot_name>  (pkg filter is auto-derived from the name)
#     python3 {baseDir}/scripts/ros2_cli.py profile scan --name lekiwi
#     python3 {baseDir}/scripts/ros2_cli.py profile scan --name depthai
#
#   MULTIPLE robots named  → pass --packages <r1>,<r2> plus --name <combined_label>
#     python3 {baseDir}/scripts/ros2_cli.py profile scan --packages lekiwi,quest_teleop --name lekiwi_quest
#
#   NO robot name given (bare "create a profile" / "scan workspace")  → omit both flags
#     python3 {baseDir}/scripts/ros2_cli.py profile scan
#
# Use summary.safety_limits as the --max-vel / --max-ang ceiling (Rule 28).
```

Stop and tell the user if Step 1 reports critical failures. Re-run Step 3 before every timed command in simulation.

---

## Path A operational summary (READ BEFORE EVERY ACTION when profile is loaded)

**When `profile show` returned a non-empty `summary` at Step 7, the session is Path A. In Path A, the profile is the source of truth for static data — running live discovery for data the profile already holds is a rule violation (Rule 14), not extra safety.**

**Use directly from the profile — zero live calls:**

| Need | Profile field | Forbidden in Path A |
|---|---|---|
| Velocity command topic | `summary.cmd_vel_topic` | `topics find geometry_msgs/msg/Twist`, `topics find geometry_msgs/msg/TwistStamped` |
| Velocity message type | `summary.velocity_topics[].type` (entry matching `cmd_vel_topic`) | `topics type <topic>` |
| Odometry topic | first value of `summary.localization_config.fused_sources` | `topics find nav_msgs/msg/Odometry` |
| Velocity safety ceiling | `summary.safety_limits.binding.{linear_x,linear_y,angular_z}` | `nodes list` + `params list` sweep for `max/limit/vel/speed/accel` |
| TF frame names | `summary.tf_frames.{odom_frame,base_frame,map_frame}` | `tf list` for frame names |
| Controller names | `summary.active_controllers` | using `control list-controllers` to *discover* controller names (still required to check runtime state — see below) |
| E-stop service | `summary.estop_config.service_name` | `services find std_srvs/srv/SetBool` |
| Joint names / order / index | `summary.hardware_interfaces[].joints`, `summary.joint_limits` | `params get /controller_manager:joints`, parsing `robot_description` |

**The only live calls still mandatory before motion in Path A** (these read runtime state, which the profile cannot know):

1. `control list-controllers` — confirm the controller named in `summary.active_controllers` is `active` right now.
2. Subscribe `<ODOM_TOPIC> --max-messages 1 --timeout 2` — confirm robot is stationary (Rule 9). This double-serves as a liveness check on the odom topic.
3. `interface proto <VEL_TYPE>` — once per session, get the payload template (the type came from the profile, but the field layout did not).
4. Optional: `topics hz <ODOM_TOPIC> --duration 2` if `publish-until` is being used and you have not confirmed odom rate this session.

**Path A violation — worked counterexample (what NOT to do):**

User request: *"drive forward 1 m"*. Profile is loaded.

```
❌ topics find geometry_msgs/msg/Twist             # Rule 14: profile has cmd_vel_topic
❌ topics find geometry_msgs/msg/TwistStamped      # Rule 14: profile has velocity_topics[].type
❌ topics find nav_msgs/msg/Odometry               # Rule 14: profile has fused_sources
❌ topics type <discovered_topic>                  # Rule 14: profile has the type
❌ nodes list && params list <each-node>           # Rule 14: profile has safety_limits.binding
```

Cost of the violation: 5–30 seconds wasted, **and** if any `topics find` returns a topic that disagrees with the profile, the agent will silently use the live answer — masking the disagreement that Rule 0.0b is specifically designed to surface. Live discovery in Path A is not "extra safety"; it actively hides safety-relevant mismatches and delays the command.

**Correct Path A motion sequence:**

```
✅ Read VEL_TOPIC, VEL_TYPE, ODOM_TOPIC, MAX_VEL, MAX_ANG from profile  (0 live calls)
✅ interface proto <VEL_TYPE>                                           (1 live call, once/session)
✅ control list-controllers                                             (1 live call — runtime state)
✅ topics subscribe <ODOM_TOPIC> --max-messages 1 --timeout 2           (1 live call — stationary + liveness)
✅ topics publish-until <VEL_TOPIC> '<payload>' --monitor <ODOM_TOPIC>   (the actual command)
✅ topics subscribe <ODOM_TOPIC> --max-messages 1 --timeout 2           (post-motion verify, Rule 8)
```

3 live calls before the command + 1 after. Not the 6–9 live discovery calls of Path B.

**If a profile field is missing** (Rule 0.0a): fall back to live discovery for **that one field only** — the path does not flip. Other fields stay on the profile.

**If a profile field's value disagrees with the live graph** (e.g., `summary.cmd_vel_topic` is not in `topics list`): stop and escalate per Rule 0.0b. Do not silently retry with live discovery.

---

## Critical Rules

Read the domain-specific rule files from `references/` before the first action. These are hard constraints — not guidelines. Use [`references/RULES.md`](references/RULES.md) as the index to find the right file. **Always load at session start: `RULES-CORE.md`, `RULES-PREFLIGHT.md`, and `RULES-MOTION.md`.** Add `RULES-DIAGNOSTICS.md` when something fails. Key principles:

1. **Two-Path Model — profile first, live fallback.** Path A (profile loaded) uses profile fields for static data and live calls only for runtime state; Path B (no profile) does full live introspection. Path is fixed for the session, decided once at Step 7. Never hardcode names, types, or limits. Full guardrails (field-presence rules, exact field names, escalation on runtime mismatch, auto-rescan gating, scan triggers) live in **RULES-PREFLIGHT.md Rules 0.0, 0.0a, 0.0b, 0.6, 0.7** and **RULES-CORE.md Rules 13, 14**.

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
python3 {baseDir}/scripts/ros2_cli.py context --include-schemas           # + schemas for all discovered topic types (opt-in; adds token cost)
python3 {baseDir}/scripts/ros2_cli.py topics list [--limit N]             # list topics (cap at N)
# Velocity topic — profile fast-path: use summary.cmd_vel_topic + summary.velocity_topics[].type
# Odom topic — profile fast-path: use summary.localization_config.fused_sources values
# Fallback (profile absent or field missing):
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

**Path A (profile loaded):** resolve `<VEL_TOPIC>` and `<VEL_TYPE>` from the profile — do NOT run `topics find` / `topics type`. See "Path A operational summary" above.

```bash
# Path A field resolution (no live calls):
#   VEL_TOPIC     = summary.cmd_vel_topic                          (e.g. /base_controller/cmd_vel)
#   VEL_TYPE      = summary.velocity_topics[].type                 (e.g. geometry_msgs/msg/TwistStamped)
#   ODOM_TOPIC    = first value of summary.localization_config.fused_sources   (e.g. /base_controller/odom)
#   MAX_VEL       = summary.safety_limits.binding.linear_x
#   MAX_ANG       = summary.safety_limits.binding.angular_z
# Path B (no profile): discover via `topics find geometry_msgs/msg/Twist` and
# `topics find geometry_msgs/msg/TwistStamped`, then `topics type` to confirm.

# Get payload template once per session — always; the type may be Twist OR TwistStamped
python3 {baseDir}/scripts/ros2_cli.py interface proto <VEL_TYPE>
# Twist:        {"linear":{"x":0,"y":0,"z":0},"angular":{"x":0,"y":0,"z":0}}
# TwistStamped: {"header":{"stamp":{"sec":0},"frame_id":""},"twist":{"linear":{...},"angular":{...}}}

# Single publish
python3 {baseDir}/scripts/ros2_cli.py topics publish <VEL_TOPIC> '<json matching VEL_TYPE>' \
  --max-vel <MAX_VEL> --max-ang <MAX_ANG>

# Closed-loop linear (Euclidean distance) — payload shape depends on VEL_TYPE
python3 {baseDir}/scripts/ros2_cli.py topics publish-until <VEL_TOPIC> \
  '<json matching VEL_TYPE>' \
  --monitor <ODOM_TOPIC> --field pose.pose.position --euclidean --delta 1.0 --timeout 60 \
  --max-vel <MAX_VEL> --max-ang <MAX_ANG>

# Closed-loop rotation (sign of --rotate MUST match sign of angular.z)
python3 {baseDir}/scripts/ros2_cli.py topics publish-until <VEL_TOPIC> \
  '<json with angular.z = +omega>' \
  --monitor <ODOM_TOPIC> --rotate 90 --degrees --timeout 30 \
  --max-vel <MAX_VEL> --max-ang <MAX_ANG>

# Subscribe (read sensor data)
python3 {baseDir}/scripts/ros2_cli.py topics subscribe <topic> --max-messages 1 --timeout 5

# Subscribe and return the first message only, then exit
python3 {baseDir}/scripts/ros2_cli.py topics echo-once <topic> [--timeout 5]
```

**`--dry-run`**: resolve topic type and validate the JSON payload without publishing anything. Returns `{dry_run: true, topic, msg_type, payload}`. Use to verify payload shape and topic availability before a real publish. Works on `publish`, `publish-sequence`, and `publish-until`.

**`--max-vel N` / `--max-ang N`** (Twist / TwistStamped only): clamp linear x/y/z to ±N m/s and angular.z to ±N rad/s inside the CLI before the message is sent. Pass `summary.safety_limits.binding.linear_x` / `.angular_z` from the profile (Path A) or the Rule 28 limit-scan result (Path B). Clamped axes are reported in `velocity_clamped` in the JSON output. Other message types pass through unchanged.

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
# Verify camera before use — K matrix must be non-zero
python3 {baseDir}/scripts/ros2_cli.py topics find sensor_msgs/msg/CameraInfo
python3 {baseDir}/scripts/ros2_cli.py topics subscribe <camera_info_topic> --max-messages 1 --timeout 2

# Capture image (auto-rotates if profile shows upside-down mount; output: {success, path, image_rotated_deg})
python3 {baseDir}/scripts/ros2_cli.py topics capture-image --topic <camera_topic> --output {baseDir}/.artifacts/<name>.jpg
python3 {baseDir}/scripts/ros2_cli.py topics capture-image --topic <camera_topic> --output {baseDir}/.artifacts/<name>.jpg --no-profile

# Depth point (16UC1 or 32FC1; returns depth_m, encoding, width, height)
python3 {baseDir}/scripts/ros2_cli.py topics depth-point --topic <depth_topic> --u <col> --v <row>
```

### Diagnostics and Health

```bash
python3 {baseDir}/scripts/ros2_cli.py doctor                              # full DDS/graph health check
python3 {baseDir}/scripts/ros2_cli.py doctor hello                        # DDS multicast test
python3 {baseDir}/scripts/ros2_cli.py doctor diagnostics                  # hardware health from /diagnostics_agg
python3 {baseDir}/scripts/ros2_cli.py doctor diagnostics --level warn     # show only WARN/ERROR/STALE
python3 {baseDir}/scripts/ros2_cli.py topics diag                         # subscribe /diagnostics
python3 {baseDir}/scripts/ros2_cli.py topics battery                      # battery state
```

### Launch

```bash
python3 {baseDir}/scripts/ros2_cli.py launch list <keyword>               # find launch files
python3 {baseDir}/scripts/ros2_cli.py launch new <package> <launch_file> use_sim_time:=true  # positional args (highest priority)
python3 {baseDir}/scripts/ros2_cli.py launch new <package> <launch_file> --param use_sim_time:=true,robot_name:=my_bot
python3 {baseDir}/scripts/ros2_cli.py launch list                         # running sessions
python3 {baseDir}/scripts/ros2_cli.py launch kill <session>
python3 {baseDir}/scripts/ros2_cli.py launch restart <session>
```

**Priority:** positional args > `--param` > `--preset`. `--config-path` adds `--ros-args --params-file` for YAML node-parameter files only — do not use it for launch args. Duplicate detection: returns `existing_session` warning if already running.

### Log Introspection

Works without a live graph — reads `~/.ros/log/` (or `$ROS_LOG_DIR`).

```bash
python3 {baseDir}/scripts/ros2_cli.py logs list-runs [--limit 20]
python3 {baseDir}/scripts/ros2_cli.py logs query [--run <id>] [--severity WARN] [--node <name>] [--after -30s] [--text <substr>] [--max 200]
python3 {baseDir}/scripts/ros2_cli.py logs tail [--run <id>] [--initial-lines 50] [--reset]   # incremental; call again for new entries only
python3 {baseDir}/scripts/ros2_cli.py logs node-summary [--run <id>] [--top 5]
```

`--after`/`--before` formats: `-30s`, `-5m`, `-2h`, epoch float, ISO datetime.

### Packages, Bags, and TF

```bash
python3 {baseDir}/scripts/ros2_cli.py pkg list|prefix|executables|xml <package>
python3 {baseDir}/scripts/ros2_cli.py pkg create <name> [--build-type ament_cmake] [--dependencies rclcpp std_msgs] [--node-name my_node]
python3 {baseDir}/scripts/ros2_cli.py bag info <bag_path>
python3 {baseDir}/scripts/ros2_cli.py tf list
python3 {baseDir}/scripts/ros2_cli.py tf echo <source_frame> <target_frame>
```

### Robot Profile (no live graph required)

Build once; load every session. Captures robot type, packages, launch files, velocity topics, safety limits, and sensor flags — eliminating per-session re-discovery.

```bash
# Build
python3 {baseDir}/scripts/ros2_cli.py profile scan --packages lekiwi [--name robot] [--workspace /path/to/ws]
python3 {baseDir}/scripts/ros2_cli.py profile scan                      # full workspace, no filter
python3 {baseDir}/scripts/ros2_cli.py profile scan --allow-live         # also query live graph
python3 {baseDir}/scripts/ros2_cli.py profile scan --robot-type mobile_base  # override detected type

# Read
python3 {baseDir}/scripts/ros2_cli.py profile show                      # full summary + annotations
python3 {baseDir}/scripts/ros2_cli.py profile show --section summary|detail|<launch-filename>

# Update
python3 {baseDir}/scripts/ros2_cli.py profile rescan [--workspace PATH] [--packages PATTERNS] [--name NAME]
python3 {baseDir}/scripts/ros2_cli.py profile rescan --launch-file <file>  # partial rescan (fast)
python3 {baseDir}/scripts/ros2_cli.py profile rescan --packages ''      # widen to full workspace

# Manage
python3 {baseDir}/scripts/ros2_cli.py profile list
python3 {baseDir}/scripts/ros2_cli.py profile annotate "Left encoder drifts right — apply 5% correction."
```

`--packages` accepts comma-separated substrings (fuzzy-match on package name or path). Omit for full workspace scan. `summary.safety_limits.binding.linear_x` → `--max-vel`; `.angular_z` → `--max-ang` (Rule 28).

Load [`references/PROFILE.md`](references/PROFILE.md) for the full JSON field reference and robot-type detection rules.

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

## Nav2 Navigation

Requires Nav2 running. Sends `NavigateToPose` / `Spin` / `FollowWaypoints` actions.

```bash
python3 {baseDir}/scripts/ros2_cli.py nav2 go 1.5 -0.3 [--yaw 90] [--timeout 120] [--feedback]
python3 {baseDir}/scripts/ros2_cli.py nav2 go-waypoints 1.0,0.0 2.0,1.5 3.0,0.0 [--no-stop-on-failure]
python3 {baseDir}/scripts/ros2_cli.py nav2 rotate 90                    # obstacle-aware (Nav2 Spin action)
python3 {baseDir}/scripts/ros2_cli.py nav2 rotate -- -45                # clockwise
python3 {baseDir}/scripts/ros2_cli.py nav2 move-timed --duration 2.0 --speed 0.3 [--direction fwd|back|left|right]
python3 {baseDir}/scripts/ros2_cli.py nav2 costmap-clear [--layer local|global|both]
python3 {baseDir}/scripts/ros2_cli.py nav2 cancel                       # cancel active goal + zero-velocity burst
python3 {baseDir}/scripts/ros2_cli.py nav2 status
python3 {baseDir}/scripts/ros2_cli.py nav2 initial-pose 1.0 2.0 --yaw 45
```

**Before `nav2 go`:** run `nav2 cancel` first if a previous goal may still be active (Rule 9). Use `nav2 status` to confirm no active goal.
**`nav2 rotate` vs `publish-until`:** prefer `nav2 rotate` when Nav2 is active — obstacle-aware; fall back to `publish-until` only when Nav2 is not running.
**Output (`nav2 go`):** `success` (bool), `status` (4=SUCCEEDED), `goal {x,y,frame}`.
**`nav2 move-timed`:** publishes `/cmd_vel` for fixed duration then auto-stops via zero-velocity burst.
**`nav2 costmap-clear`:** calls Nav2 clear-costmap services (std_srvs/Empty).

---

## Commands Without a Live Graph

| Command | Purpose |
|---|---|
| `version` | Verify skill is installed |
| `daemon status / start / stop` | Manage the ROS 2 daemon |
| `bag info <file>` | Bag metadata (duration, counts, per-topic stats) |
| `logs list-runs / query / tail / node-summary` | Log introspection |
| `component types` | List registered composable node types |
| `pkg list / prefix / executables / xml / create` | Package introspection and scaffolding |
| `profile scan / show / rescan / list / annotate` | Robot profile management |
| `--help` on any command | Inspect flags and subcommands |

---

## Reference Files

This skill uses **progressive disclosure**. SKILL.md covers the most common operations. Load the files below when the task requires deeper detail.

| File | When to load |
|---|---|
| [`references/RULES.md`](references/RULES.md) | **Index only** — maps each rule number to its domain file. Load first to navigate the rule set. |
| [`references/RULES-CORE.md`](references/RULES-CORE.md) | **Always load** — general agent conduct (Rules 0.5, 1, 2, 4–6, 10–13). Hard constraints that apply to every command. Includes mandatory compliance preamble and Quick Decision Card. |
| [`references/RULES-PREFLIGHT.md`](references/RULES-PREFLIGHT.md) | **Load at session start and before any action** — full pre-flight introspection protocol (Rule 0), session-start steps 0–6 (Rule 0.1), lifecycle/QoS/publisher checks (Rules 14, 15, 19). |
| [`references/RULES-MOTION.md`](references/RULES-MOTION.md) | **Always load at session start** (any mobile-base or arm robot may receive a motion command) — movement algorithm (Rule 3), pre-motion check + Nav2 preemption (Rule 9), REP-103/105 (Rule 17), estop (Rule 18), decel zone (Rule 20), timeout recovery (Rule 21), command limits (Rules 22–23), sequencing (Rule 24), proximity scan (Rule 25). Step 1 of Rule 3 is the authoritative source on the profile fast-path for motion. |
| [`references/RULES-DIAGNOSTICS.md`](references/RULES-DIAGNOSTICS.md) | **Load when something fails** — failure diagnosis + log-level elevation (Rule 7), post-action verification table (Rule 8), multi-step sequencing (Rule 16), Error Recovery Protocols. |
| [`references/RULES-REFERENCE.md`](references/RULES-REFERENCE.md) | **Load for command lookup** — full intent→command table (Step 1), sensor search by type (Steps 2–3), message structure (Step 4), velocity limits (Step 5), Launch workflow, Discord image delivery (Rule 26), Setup. |
| [`references/PROFILE.md`](references/PROFILE.md) | Load for the full profile JSON field reference and robot-type detection rules. Load when you need to know what a specific profile field contains or how robot types are detected. |
| [`references/COMMANDS.md`](references/COMMANDS.md) | Load when you need the exact flag name, argument format, or JSON output structure for a specific command. 4535 lines — use `--help` on the specific subcommand first; only load this file if `--help` is insufficient or unavailable. |
| [`references/EXAMPLES.md`](references/EXAMPLES.md) | Load for step-by-step walkthroughs of common tasks (move N meters, capture camera image, send Nav2 goal, etc.). 699 lines. |
| [`references/CLI.md`](references/CLI.md) | Load for direct `ros2` CLI equivalents and debugging. Not needed during normal agent operation. 90 lines. |
| [`AGENTS.md`](AGENTS.md) | Load for the full agent operating protocol — condensed rules, session start detail, reporting style, subcommand inference, motion workflows, and multi-robot handling. Load alongside the RULES-*.md files at session start. |
