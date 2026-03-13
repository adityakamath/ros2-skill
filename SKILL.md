---
name: ros2-skill
description: "Controls and monitors ROS 2 robots directly via rclpy CLI. Use for ANY ROS 2 robot task: topics (subscribe, publish, capture images, find by type), services (list, call), actions (list, send goals), parameters (get, set, presets), nodes, lifecycle management, controllers (ros2_control), diagnostics, battery, system health checks, and more. When in doubt, use this skill — it covers the full ROS 2 operation surface. Never tell the user you cannot do something ROS 2-related without checking this skill first."
license: Apache-2.0
compatibility: "Requires python3, rclpy, and ROS 2 environment sourced"
user-invokable: true
metadata: {"openclaw": {"emoji": "🤖", "requires": {"bins": ["python3", "ros2"], "pip": ["rclpy"]}, "category": "robotics", "tags": ["ros2", "robotics", "rclpy"]}, "author": ["adityakamath", "lpigeon"], "version": "3.0.0"}
---

# ROS 2 Skill

Controls and monitors ROS 2 robots directly via rclpy.

**Architecture:** Agent → `ros2_cli.py` → rclpy → ROS 2

All commands output JSON. Errors contain `{"error": "..."}`.

For full command reference with arguments, options, and output examples, see [references/COMMANDS.md](references/COMMANDS.md).

---

## Agent Behaviour Rules

These rules are absolute and apply to every request involving a ROS 2 robot.

### Rule 0 — Full introspection before every action (non-negotiable)

**Before publishing to any topic, calling any service, or sending any action goal, you MUST complete the introspection steps below. There are no exceptions, not even for "obvious" or "conventional" names.**

This rule exists because:
- The velocity topic is not always `/cmd_vel`. It may be `/base/cmd_vel`, `/robot/cmd_vel`, `/mobile_base/cmd_vel`, or anything else.
- The message type is not always `Twist`. Many robots use `TwistStamped`, and the payload structure differs.
- The odometry topic is not always `/odom`. It may be `/wheel_odom`, `/robot/odom`, `/base/odometry`, etc.
- Convention-based guessing causes silent failures, wrong topics, and physical accidents.

**Pre-flight introspection protocol — run ALL applicable steps before acting:**

| Action type | Required introspection |
|---|---|
| Publish to a topic | 1. `topics find <msg_type>` to discover the real topic name<br>2. `topics type <discovered_topic>` to confirm the exact type<br>3. `interface proto <exact_type>` to get the default payload template |
| Call a service | 1. `services list` or `services find <srv_type>` to discover the real name<br>2. `services details <discovered_service>` to get request/response fields |
| Send an action goal | 1. `actions list` or `actions find <action_type>` to discover the real name<br>2. `actions details <discovered_action>` to get goal/result/feedback fields |
| Move a robot | Full Movement Workflow — see Rule 3 and the canonical section below |
| Read a sensor | `topics find <msg_type>` to discover the topic; never subscribe to a hardcoded name |
| Any operation involving a node | `nodes list` first; never assume a node name |

**Parameter introspection is mandatory before any movement command.** Before publishing velocity:
1. Run `nodes list` to find controller nodes (names containing `controller`, `driver`, `base`, etc.)
2. Run `params list <CTRL_NODE>` on each candidate to find safety-relevant parameters
3. Look for: `max_vel_x`, `max_vel_y`, `max_vel_theta`, `max_linear_velocity`, `max_angular_velocity`, `speed_limit`, `linear.x.max_velocity`, `angular.z.max_velocity`, or any parameter with `max`, `limit`, or `vel` in the name
4. Run `params get <CTRL_NODE>:<param>` for each discovered limit
5. Cap your commanded velocity at the discovered limit — never exceed it. If no limit is found, use a conservative default (e.g. 0.1 m/s linear, 0.3 rad/s angular) and note this to the user

**Never hardcode or assume:**
- ❌ Never use `/cmd_vel` without first discovering the velocity topic with `topics find`
- ❌ Never use `Twist` payload without first confirming the type is not `TwistStamped` via `topics type`
- ❌ Never use `/odom` without first discovering the odometry topic with `topics find`
- ❌ Never use `--yaw`, `--yaw-delta`, or `--field` for rotation — the only correct flag is `--rotate N --degrees` (or `--rotate N` for radians). Use negative N for CW; `--rotate` sign and `angular.z` sign must always match.
- ❌ Never assume any topic, service, action, or node name from ROS 2 convention
- ❌ Never assume a message type from a topic name

**Introspection commands return discovered names. Use those names — not the ones you expect.**

### Rule 0.1 — Session-start checks (run once per session, before any task)

**Before executing any user command in a new session, run these checks.** They take seconds and catch the most common causes of silent failure.

**Step 1 — Run a health check:**
```bash
python3 {baseDir}/scripts/ros2_cli.py doctor
```
If the doctor reports critical failures (DDS issues, missing packages, no nodes), stop and tell the user. Do not attempt to operate a robot that fails its health check.

**Step 2 — Check for simulated time:**
```bash
python3 {baseDir}/scripts/ros2_cli.py topics find rosgraph_msgs/msg/Clock
```
If `/clock` is found, simulated time is in use. Verify it is actively publishing before issuing any timed command:
```bash
python3 {baseDir}/scripts/ros2_cli.py topics subscribe /clock --max-messages 1 --timeout 3
```
If no message arrives: the simulator is paused or crashed — do not proceed with time-sensitive operations.

**Step 3 — Check lifecycle nodes (if any):**
```bash
python3 {baseDir}/scripts/ros2_cli.py lifecycle nodes
```
If lifecycle-managed nodes exist, check their states:
```bash
python3 {baseDir}/scripts/ros2_cli.py lifecycle get <node>
```
A node in `unconfigured` or `inactive` state will silently fail when its topics or services are used. Activate required nodes before proceeding.

**These checks are session-level.** Do not re-run for every command. Re-run only if the user relaunches the robot or if nodes appear/disappear unexpectedly.

### Rule 0.5 — Never hallucinate commands, flags, or names

**If you are not certain a command, flag, topic name, or argument exists — verify it before using it. Do not guess.**

The failure mode to avoid: inventing a flag like `--yaw-delta` or `--rotate-degrees` because it sounds plausible, then failing and asking the user for help. That is the worst possible outcome — the error was self-inflicted and the user had nothing to do with it.

**The verification chain:**
1. **Check this skill first.** The full command reference is in [references/COMMANDS.md](references/COMMANDS.md). If a flag or command is not listed there, it does not exist.
2. **If still unsure, run `--help` on the exact subcommand before constructing the call.** Every subcommand supports `--help` and prints its accepted flags without requiring a live ROS 2 graph. This is mandatory, not optional.
   ```bash
   python3 {baseDir}/scripts/ros2_cli.py topics publish-until --help
   python3 {baseDir}/scripts/ros2_cli.py topics publish-sequence --help
   python3 {baseDir}/scripts/ros2_cli.py actions send --help
   # etc. — use the exact subcommand you are about to call
   ```
3. **If still stuck after checking both, ask the user.** This is the only acceptable reason to ask — not because you assumed something and it failed.

**Never:**
- Invent a flag and try it, then report failure to the user
- Assume a capability exists because it would be logical or convenient
- Ask the user to resolve an error you caused by guessing

### Rule 1 — Discover before you act, never ask

**Never ask the user for names, types, or IDs that can be discovered from the live system.** This includes topic names, service names, action names, node names, parameter names, message types, and controller names. Always query the robot first.

| What you need | How to discover it |
|---|---|
| Topic name | `topics list` or `topics find <msg_type>` |
| Topic message type | `topics type <topic>` |
| Service name | `services list` or `services find <srv_type>` |
| Service request/response fields | `services details <service>` |
| Action server name | `actions list` or `actions find <action_type>` |
| Action goal/result/feedback fields | `actions details <action>` |
| Node name | `nodes list` |
| Node's topics, services, actions | `nodes details <node>` |
| Parameter names on a node | `params list <node>` |
| Parameter value | `params get <node:param>` |
| Parameter type and constraints | `params describe <node:param>` |
| Controller names and states | `control list-controllers` |
| Hardware components | `control list-hardware-components` |
| Message / service / action type fields | `interface show <type>` or `interface proto <type>` |

**Only ask the user if**:
1. The discovery command returns an empty result or an error, **and**
2. There is genuinely no other way to determine the information from the live system.

### Rule 2 — Use ros2-skill before saying you can't

**Never tell the user you don't know how to do something with a ROS 2 robot without first checking whether ros2-skill has a command for it.** This skill covers the full range of ROS 2 operations: topics, services, actions, parameters, nodes, lifecycle, controllers, diagnostics, battery, images, interfaces, presets, and more.

When a task seems unfamiliar, look it up in the quick reference tables below before responding. Common operations that agents sometimes miss:

| Task | ros2-skill command |
|---|---|
| Capture a camera image | `topics capture-image --topic <topic> --output <file>` |
| Read laser / camera / IMU / odom data | `topics subscribe <topic>` |
| Call a ROS 2 service | `services call <service> <json>` |
| Send a navigation or manipulation goal | `actions send <action> <json>` |
| Change a node parameter at runtime | `params set <node:param> <value>` |
| Save/restore a parameter configuration | `params preset-save` / `params preset-load` |
| Activate or deactivate a controller | `control set-controller-state <name> active\|inactive` |
| Run a health check | `doctor` |
| Emergency stop | `estop` |
| Check diagnostics | `topics diag` |
| Check battery | `topics battery` |

If you genuinely cannot find a matching command after checking both the quick reference and the COMMANDS.md reference, **say so clearly and explain what you checked** — do not silently guess or use a partial solution.

### Rule 3 — Movement algorithm (always follow this sequence)

For **any** user request involving movement — regardless of whether a distance or angle is specified — follow this algorithm exactly. Do not skip steps. Do not ask the user anything that can be resolved by running a command.

**Step 1 — Discover the velocity command topic and confirm its exact type**

Run both searches in parallel:
```bash
topics find geometry_msgs/msg/Twist
topics find geometry_msgs/msg/TwistStamped
```
Record the discovered topic name — call it `VEL_TOPIC`. Then confirm the exact type:
```bash
topics type <VEL_TOPIC>
```
Use the confirmed type to choose the payload structure:
- `geometry_msgs/msg/Twist`: `{"linear":{"x":0.2,"y":0,"z":0},"angular":{"x":0,"y":0,"z":0}}`
- `geometry_msgs/msg/TwistStamped`: `{"header":{"stamp":{"sec":0},"frame_id":""},"twist":{"linear":{"x":0.2,"y":0,"z":0},"angular":{"x":0,"y":0,"z":0}}}`

If both find commands return results, run `topics type` on each and use the type returned — do not guess.

**Step 2 — Discover the odometry topic**

```bash
topics find nav_msgs/msg/Odometry
```
Record the discovered topic name — call it `ODOM_TOPIC`.

**Step 3 — Choose the execution method**

| Situation | Method |
|---|---|
| Distance or angle specified **and** odometry found | `publish-until` with `--monitor <ODOM_TOPIC>` — closed loop, stops on sensor feedback |
| Distance or angle specified **and** no odometry | `publish-sequence` with calculated duration — open loop. Tell the user: "No odometry found. Running open-loop. Distance/angle accuracy is not guaranteed." |
| No distance or angle specified (open-ended movement) | `publish-sequence` with a stop command as the final message |

**Step 4 — Execute using only discovered names**

Use `VEL_TOPIC` and `ODOM_TOPIC` from Steps 1–2. Never substitute `/cmd_vel`, `/odom`, or any other assumed name.

Distance commands:
```bash
topics publish-until <VEL_TOPIC> '<payload>' --monitor <ODOM_TOPIC> --field pose.pose.position.x --delta <N> --timeout 30
```
Angle/rotation commands — **always use `--rotate`, never `--field` or `--yaw`**:
```bash
# CCW (left): positive --rotate + positive angular.z
topics publish-until <VEL_TOPIC> '<payload>' --monitor <ODOM_TOPIC> --rotate <+N> --degrees --timeout 30
# CW (right):  negative --rotate + negative angular.z
topics publish-until <VEL_TOPIC> '<payload>' --monitor <ODOM_TOPIC> --rotate <-N> --degrees --timeout 30
```
`--rotate` sign = direction. Positive = CCW. Negative = CW. `angular.z` sign must always match `--rotate` sign — mismatched signs cause timeout. There is no `--yaw` flag. Do not attempt to monitor orientation fields manually.
Open-ended or fallback (stop is always the last message):
```bash
topics publish-sequence <VEL_TOPIC> '[<move_payload>, <zero_payload>]' '[<duration>, 0.5]'
```

See the [Movement Workflow](#movement-workflow-canonical) section below for complete worked examples covering every case.

### Rule 4 — Infer the goal, resolve the details
When a user asks to do something, **infer what they want at the goal level, then resolve all concrete details (topic names, types, field paths) from the live system**.

Examples:
- "Take a picture" → find compressed image topics (`topics find sensor_msgs/msg/CompressedImage`), capture from the first active result
- "Move the robot forward" → find velocity topic (`topics find geometry_msgs/msg/Twist` and `TwistStamped`), publish with the matching structure
- "What is the battery level?" → `topics battery` (auto-discovers `BatteryState` topics)
- "List available controllers" → `control list-controllers`
- "What parameters does the camera node have?" → `nodes list` to find the camera node name, then `params list <node>`

### Rule 5 — Execute, don't ask

**The user's message is the approval. Act on it.**

If the intent is clear, execute immediately. Do not ask for confirmation, do not summarise what you are about to do and wait for a response, do not say "I'll now run X — shall I proceed?". Just run it.

This applies without exception to:
- Running or launching nodes (`launch new`, `run new`)
- Publishing to topics
- Calling services
- Setting parameters
- Starting or stopping controllers
- Any command where the intent and target are unambiguous

**The only time to stop and ask is when there is genuine ambiguity** that cannot be resolved from the live system — for example:
- Multiple packages or launch files match and you cannot determine which one the user means
- A required argument has no match in `--show-args` and no reasonable fuzzy match exists
- The user's request contradicts itself or is physically unsafe to guess

Everything else: **just do it.**

**Explicit list of things that must never trigger a question:**
- "Should I run this command?" — No. Run it.
- "Would you like me to proceed?" — No. Proceed.
- "Do you want me to use X topic?" — No. Use it.
- "Shall I launch the file now?" — No. Launch it.
- "Do you want me to set this parameter?" — No. Set it.
- "I found Y — would you like me to use it?" — No. Use it.

### Rule 6 — Minimal reporting by default

**Keep output minimal. The user wants results, not narration.**

| Situation | What to report |
|---|---|
| Operation succeeded | One line: what was done and the key outcome. Example: "Done. Moved 1.02 m forward." |
| Movement completed | Start position, end position, actual distance/angle travelled, and any anomalies — nothing else |
| No suitable topic/source found | Clear error with what was searched and what to try next |
| Safety condition triggered | Immediate notification with what happened and what was sent (stop command) |
| Operation failed | Error message with cause and recovery suggestion |

**Never report by default:**
- The topic name selected, unless it is ambiguous or unexpected
- The message type discovered
- Intermediate introspection results
- Step-by-step narration of what you are about to do

**Report everything (verbose mode) only when the user explicitly asks** — e.g. "show me what topics you found", "give me the full details", "what type did you use?"

---

## Setup

### 1. Source ROS 2 environment

```bash
source /opt/ros/${ROS_DISTRO}/setup.bash
```

### 2. Install dependencies

```bash
pip install rclpy
```

### 3. Run on ROS 2 robot

The CLI must run on a machine with ROS 2 installed and sourced.

---

## Important: Check ROS 2 First

Before any operation, verify ROS 2 is available:

```bash
python3 {baseDir}/scripts/ros2_cli.py version
```

---

## Quick Decision Card

**Every user request follows this pattern:**

```
User: "do X"
Agent thinks:
  1. Is X about reading data? → Use TOPICS SUBSCRIBE
  2. Is X about movement (any kind)? → Follow the Movement Workflow (Rule 3 / canonical section)
  3. Is X a one-time trigger? → Use SERVICES CALL or ACTIONS SEND
  4. Is X about system info? → Use LIST commands

Agent does (for movement):
  1. Find velocity topic: topics find geometry_msgs/Twist + TwistStamped
  2. Find odometry topic: topics find nav_msgs/Odometry
  3. Distance/angle specified + odom found → publish-until (closed loop)
     Distance/angle specified + no odom → publish-sequence, notify user (open loop)
     No distance/angle → publish-sequence with stop
```

**For movement, always follow the Movement Workflow section. That section is the single source of truth.**

---

## Agent Decision Framework (MANDATORY)

**RULE: NEVER ask the user anything that can be discovered from the ROS 2 graph.**

### Step 1: Understand User Intent

| User says... | Agent interprets as... | Agent must... |
|--------------|----------------------|---------------|
| "What topics exist?" | List topics | Run `topics list` |
| "What nodes exist?" | List nodes | Run `nodes list` |
| "What services exist?" | List services | Run `services list` |
| "What actions exist?" | List actions | Run `actions list` |
| "Read the LiDAR/scan" | Subscribe to LaserScan | Find LaserScan topic → subscribe |
| "Read odometry/position" | Subscribe to Odometry | Find Odometry topic → subscribe |
| "Read camera/image" | Subscribe to Image/CompressedImage | Find Image topics → subscribe |
| "Read joint states/positions" | Subscribe to JointState | Find JointState topic → subscribe |
| "Read IMU/accelerometer" | Subscribe to Imu | Find Imu topic → subscribe |
| "Read battery/power" | Subscribe to BatteryState | Find BatteryState topic → subscribe |
| "Read joystick/gamepad" | Subscribe to Joy | Find Joy topic → subscribe |
| "Check robot diagnostics/health" | Subscribe to diagnostics | Find /diagnostics topic → subscribe |
| "Check TF/transforms" | Check TF topics | Find /tf, /tf_static topics → subscribe |
| "Move/drive/turn (mobile robot)" | Open-ended movement, no target | Find Twist/TwistStamped → **publish-sequence** with stop |
| "Move forward/back N meters" | Closed-loop distance → Movement Workflow Case A | Find odom → **publish-until** `--field pose.pose.position.x --delta N` |
| "Rotate N degrees / turn left/right / turn N radians" | Closed-loop rotation → Movement Workflow Case B | Find odom → **publish-until** `--rotate ±N --degrees` (CCW = positive, CW = negative). Sign of `--rotate` MUST match sign of `angular.z`. Never use `--field` or `--yaw` for rotation |
| "Move arm/joint (manipulator)" | Publish JointTrajectory | Find JointTrajectory topic → publish |
| "Control gripper" | Publish GripperCommand or JointTrajectory | Find gripper topic → publish |
| "Stop the robot" | Publish zero velocity | Find Twist/TwistStamped → `topics type` to confirm → publish zeros in confirmed type |
| "Emergency stop" | Publish zero velocity | Run `estop` command |
| "Call /reset" | Call service | Find service → call |
| "Navigate to..." | Send action | Find action → send goal |
| "Execute trajectory" | Send action | Find FollowJointTrajectory or ExecuteTrajectory → send |
| "Run launch file" | Launch file | Find package → find launch file → launch in tmux |
| "List running launches" | List sessions | Run `launch list` |
| "Kill launch" | Kill session | Run `launch kill <session>` |
| "What controllers?" | List controllers | Run `control list-controllers` |
| "What hardware?" | List hardware | Run `control list-hardware-components` |
| "What lifecycle nodes?" | List managed nodes | Run `lifecycle nodes` |
| "Check lifecycle state" | Get node state | Run `lifecycle get <node>` |
| "Configure/activate lifecycle node" | Set lifecycle state | Run `lifecycle set <node> <transition>` |
| "Run diagnostics/health check" | Run doctor | Run `doctor` |
| "Test connectivity" | Run multicast test | Run `doctor hello` |
| "What parameters?" | List params | Find node → `params list` |
| "What is the max speed?" | Get params | Find controller → get velocity limits |
| "Save/load parameter config" | Use presets | Run `params preset-save` / `params preset-load` |
| "Check battery level" | Subscribe to BatteryState | Run `topics battery` or find BatteryState topic |

### Step 2: Find What Exists

**ALWAYS start by exploring what's available:**

```bash
# These 4 commands tell you EVERYTHING about the system
python3 {baseDir}/scripts/ros2_cli.py topics list      # All topics
python3 {baseDir}/scripts/ros2_cli.py services list    # All services
python3 {baseDir}/scripts/ros2_cli.py actions list    # All actions
python3 {baseDir}/scripts/ros2_cli.py nodes list      # All nodes
```

### Step 3: Search by Message Type

**To find a topic/service/action, search by what you need:**

| Need to find... | Search command... |
|-----------------|------------------|
| Velocity command topic (mobile) | `topics find geometry_msgs/Twist` AND `topics find geometry_msgs/TwistStamped` → then `topics type <result>` to confirm exact type |
| Position/odom topic | `topics find nav_msgs/Odometry` |
| Joint positions | `topics find sensor_msgs/JointState` |
| Joint trajectory (arm control) | `topics find trajectory_msgs/JointTrajectory` |
| LiDAR data | `topics find sensor_msgs/LaserScan` |
| Camera feed | `topics find sensor_msgs/Image` OR `topics find sensor_msgs/CompressedImage` |
| IMU data | `topics find sensor_msgs/Imu` |
| Joystick | `topics find sensor_msgs/Joy` |
| Battery/power | `topics find sensor_msgs/BatteryState` |
| Temperature | `topics find sensor_msgs/Temperature` |
| Point clouds | `topics find sensor_msgs/PointCloud2` |
| TF transforms | Subscribe to `/tf` or `/tf_static` |
| Diagnostics | Subscribe to `/diagnostics` |
| Clock (simulated time) | Subscribe to `/clock` |
| Service by type | `services find <service_type>` |
| Action by type | `actions find <action_type>` |

### Step 4: Get Message Structure

**Before publishing or calling, always confirm the type and get the structure:**

```bash
# Confirm the exact message type of a discovered topic (critical — never skip this)
python3 {baseDir}/scripts/ros2_cli.py topics type <discovered_topic>

# Get field structure (for building payloads)
python3 {baseDir}/scripts/ros2_cli.py topics message <confirmed_message_type>

# Get default values (copy-paste template)
python3 {baseDir}/scripts/ros2_cli.py interface proto <confirmed_message_type>

# Get service/action request structure
python3 {baseDir}/scripts/ros2_cli.py services details <service_name>
python3 {baseDir}/scripts/ros2_cli.py actions details <action_name>
```

### Step 5: Get Safety Limits (for movement)

**ALWAYS check for velocity limits before publishing movement commands. Discover controller nodes first — never assume their names.**

```bash
# Step 1: List all nodes to find controller nodes
python3 {baseDir}/scripts/ros2_cli.py nodes list
# → look for nodes with names like: diff_drive_controller, base_controller,
#   velocity_controller, mobile_base_controller, etc.
# → record the actual controller node name: CTRL_NODE

# Step 2: List all parameters on the discovered controller node
python3 {baseDir}/scripts/ros2_cli.py params list <CTRL_NODE>
# → look for parameters containing: max_velocity, max_linear, max_angular, speed_limit, etc.

# Step 3: Get the actual velocity limit values (use names from Step 2)
python3 {baseDir}/scripts/ros2_cli.py params get <CTRL_NODE>:<max_velocity_param>
python3 {baseDir}/scripts/ros2_cli.py params get <CTRL_NODE>:<max_linear_param>
python3 {baseDir}/scripts/ros2_cli.py params get <CTRL_NODE>:<max_angular_param>

# Step 4: Apply limits
# velocity = min(requested_velocity, discovered_max_velocity)
# Never exceed the discovered limit — use a conservative fraction (e.g. 50%) if unsure
```

---

## Global Options

`--timeout` and `--retries` are **global** flags that apply to every command making service or action calls.

- **`--timeout` must be placed before the command name** (e.g. `--timeout 10 services call …`).
- **`--retries` can be placed before the command name OR after it** for `services call`, `actions send`, and `actions cancel` — both positions work.

| Option | Default | Description |
|--------|---------|-------------|
| `--timeout SECONDS` | per-command default | Override the per-command timeout globally |
| `--retries N` | `1` | Total attempts before giving up; `1` = single attempt with no retry |

```bash
python3 {baseDir}/scripts/ros2_cli.py --timeout 30 params list /turtlesim
python3 {baseDir}/scripts/ros2_cli.py --retries 3 services call /spawn '{}'
```

---

## EXECUTION RULES (MUST FOLLOW)

### Rule 1: Never Ask User for These

The agent MUST discover these automatically:

| User might ask... | Agent must... |
|-------------------|---------------|
| "What topic do I use?" | Use `topics find <type>` to discover |
| "What message type?" | Use `topics type <topic>` or `topics find <type>` |
| "What is the message structure?" | Use `topics message <type>` or `interface show <type>` |
| "What are the safety limits?" | Use `params list` on controller nodes |
| "Is there odometry?" | Use `topics find nav_msgs/Odometry` |

### Rule 2: Only Ask User After ALL Discovery Fails

**ONLY ask the user when:**
1. `topics find geometry_msgs/Twist` AND `topics find geometry_msgs/TwistStamped` both return empty
2. `topics find nav_msgs/Odometry` returns empty (and you need odometry for distance)
3. You've checked params on ALL controller nodes and found NO velocity limits
4. The service/action the user mentions doesn't exist in `services list` / `actions list`

### Rule 3: Movement Requires Odometry Feedback

**See Agent Behaviour Rule 3 above — it is absolute and overrides any example in this document.**

In short: for any "move N meters" or "rotate N degrees" command, you MUST find the odometry topic first (`topics find nav_msgs/Odometry`) and use `publish-until --monitor <odom_topic>`. `publish-sequence` with a fixed time is forbidden when odometry is available.

- ALWAYS apply safety limits: `velocity = min(requested, max_velocity)`

### Rule 4: Always Stop After Movement

**`publish-until` stops automatically** when the odometry condition is met — no explicit stop step is needed.

For open-ended publishes (`topics publish` or `publish-sequence` fallback), the final message MUST be all zeros.

Always use the topic name and payload type discovered in Steps 1–2. `<VEL_TOPIC>` and `<ODOM_TOPIC>` are placeholders for your discovered values.

```bash
# WRONG: open-ended publish with no stop, and hardcoded /cmd_vel — never do this
python3 {baseDir}/scripts/ros2_cli.py topics publish /cmd_vel '{"linear":{"x":1.0}}'

# CORRECT (distance specified, odometry available): use publish-until — stops itself
# VEL_TOPIC and ODOM_TOPIC come from your introspection steps
python3 {baseDir}/scripts/ros2_cli.py topics publish-until <VEL_TOPIC> \
  '<payload_matching_confirmed_type>' \
  --monitor <ODOM_TOPIC> --field pose.pose.position.x --delta 1.0 --timeout 30

# CORRECT (fallback only — no odometry, no distance specified): publish-sequence with stop at end
python3 {baseDir}/scripts/ros2_cli.py topics publish-sequence <VEL_TOPIC> \
  '[<move_payload>, <zero_payload>]' \
  '[3.0, 0.5]'
```

### Rule 5: Handle Multiple Same-Type Topics

**When multiple topics of the same type exist (e.g., 2 cameras, 3 LiDARs):**

1. **List all candidates:**
   ```bash
   python3 {baseDir}/scripts/ros2_cli.py topics find sensor_msgs/Image
   # Returns: ["/camera_front/image_raw", "/camera_rear/image_raw", ...]
   ```

2. **Select based on context or naming convention:**
   - Front camera: prefer topics with `front`, `rgb`, `color` in name
   - Rear camera: prefer topics with `rear`, `back` in name
   - Primary LiDAR: prefer topics with `front`, `base`, `main` in name
   - Default: use first topic in the list

3. **Let user know which one you're using:**
   - "Found 3 camera topics. Using /camera_front/image_raw."

---

## Error Recovery

**When commands fail, follow this recovery process:**

### Subscribe Timeouts

| Error | Recovery |
|-------|----------|
| `Timeout waiting for message` | 1. Check `topics details <topic>` to verify publisher exists<br>2. Try a different topic if multiple exist<br>3. Increase `--duration` or `--timeout` |
| No messages received | 1. Verify publisher is running: `topics details <topic>`<br>2. Check if topic requires subscription to trigger |

### Publish Failures

| Error | Recovery |
|-------|----------|
| `Could not load message type` | 1. Verify type: `topics type <topic>`<br>2. Ensure ROS workspace is built |
| `Failed to create publisher` | 1. Check topic exists: `topics list`<br>2. Verify node has permission to publish |

### Service/Action Failures

| Error | Recovery |
|-------|----------|
| Service not found | 1. Verify service exists: `services list`<br>2. Check service type: `services type <service>` |
| Action not found | 1. Verify action exists: `actions list`<br>2. Check action type: `actions type <action>` |
| Service call timeout | 1. Increase `--timeout`<br>2. Verify service server is running |
| Action goal rejected | 1. Check action details for goal requirements<br>2. Verify robot is in correct state |

### Parameter Failures

| Error | Recovery |
|-------|----------|
| Node not found | 1. Verify node exists: `nodes list`<br>2. Check namespace |
| Parameter not found | 1. List params: `params list <node>`<br>2. Parameter may not exist on this node |

### Movement / publish-until Failures

| Error | Recovery |
|-------|----------|
| `publish-until` times out without reaching target | 1. Check odometry was publishing: `topics details <ODOM_TOPIC>`<br>2. If publisher count dropped to 0 mid-motion: odometry died — send estop immediately, notify user<br>3. Increase `--timeout` if odometry is healthy but robot is slow |
| Odometry not updating during motion | 1. Immediately send zero-velocity: `estop`<br>2. Check `topics details <ODOM_TOPIC>` for publisher count and `topics hz <ODOM_TOPIC>` for rate<br>3. Do NOT continue publishing if odometry is stale — it is a runaway risk |
| Velocity topic has no subscribers | 1. Check `control list-controllers` — the controller may be inactive<br>2. Activate the controller: `control set-controller-state <name> active`<br>3. Re-verify with `topics details <VEL_TOPIC>` before retrying |
| `publish-until` hangs / no feedback | 1. Verify monitor topic: `topics details <ODOM_TOPIC>`<br>2. Verify the field path is correct: subscribe once and inspect field names<br>3. Check `--timeout` is set |



**Always retry failed operations:**
- Use `--retries 3` for unreliable services
- Use `--timeout 30` for slow operations
- Wait 1-2 seconds between retries

---

## Topic and Service Discovery

**Never guess topic names.** Any time an operation involves a topic, discover the actual topic name from the live graph first.

### Images and Camera

**Always prefer compressed topics** - use much less bandwidth:
```bash
python3 {baseDir}/scripts/ros2_cli.py topics find sensor_msgs/msg/CompressedImage
python3 {baseDir}/scripts/ros2_cli.py topics find sensor_msgs/msg/Image
```
Use `topics capture-image --topic <discovered>` - never `subscribe` for images.

### Velocity Commands (Twist vs TwistStamped)

Check both types to find the topic:
```bash
python3 {baseDir}/scripts/ros2_cli.py topics find geometry_msgs/msg/Twist
python3 {baseDir}/scripts/ros2_cli.py topics find geometry_msgs/msg/TwistStamped
```
Then confirm the exact type of the discovered topic — do not assume from the find result:
```bash
python3 {baseDir}/scripts/ros2_cli.py topics type <discovered_topic>
```
Use the confirmed type to choose the payload:
- `geometry_msgs/msg/Twist`: `{"linear": {"x": 1.0, "y": 0.0, "z": 0.0}, "angular": {"x": 0.0, "y": 0.0, "z": 0.0}}`
- `geometry_msgs/msg/TwistStamped`: `{"header": {"stamp": {"sec": 0}, "frame_id": ""}, "twist": {"linear": {"x": 1.0, "y": 0.0, "z": 0.0}, "angular": {"x": 0.0, "y": 0.0, "z": 0.0}}}`

### Quick lookup table

| If you need... | Run this... | Then... |
|----------------|-------------|---------|
| Images | `topics find sensor_msgs/msg/Image` | Subscribe to result |
| LiDAR | `topics find sensor_msgs/msg/LaserScan` | Subscribe to result |
| Odometry | `topics find nav_msgs/msg/Odometry` | Subscribe to result |
| IMU | `topics find sensor_msgs/msg/Imu` | Subscribe to result |
| Joint states | `topics find sensor_msgs/msg/JointState` | Subscribe to result |
| Move robot | `topics find geometry_msgs/msg/Twist` AND `topics find geometry_msgs/msg/TwistStamped` → then `topics type <result>` to confirm | Publish to discovered topic |
| Run launch file | `launch new <package> <file>` | Runs in tmux session |
| List running launches | `launch list` | Shows tmux sessions |
| Kill launch | `launch kill <session>` | Kills tmux session |

---

## Launch Commands

### Auto-Discovery for Launch Files

**When user says "run the bringup" or "launch navigation" (partial/ambiguous request):**

1. **Discover available packages:**
   ```bash
   ros2 pkg list  # Get all packages
   ```

2. **Find matching launch files:**
   ```bash
   ros2 pkg files <package>  # Find launch files in package
   ```

3. **Intelligent inference (use context):**
   - "bringup" → look for packages with `bringup` in name, or launch files named `bringup.launch.py`
   - "navigation" → look for `navigation2`, `nav2`, or launch files with `navigation`
   - "camera" → look for camera-related packages

4. **If exactly one clear match found:**
   - Launch it immediately — do not ask for confirmation (Rule 5)

5. **If multiple candidates found and cannot be disambiguated:**
   - Present options: "Found 3 launch files: X, Y, Z. Which one?" — this is the only case where asking is permitted

6. **If no match found:**
   - Search more broadly: check all packages for matching launch files
   - If still nothing: ask user for exact package/file name

### NEVER hallucinate:
- ❌ Never invent a package name that doesn't exist
- ❌ Never invent a launch file that doesn't exist
- ❌ Never assume a package exists without checking
- ❌ Never invent launch argument names not present in `--show-args` output
- ❌ Never fuzzy-match argument names yourself — the script does this automatically against real `--show-args` output; always pass the user's original name and let the script resolve it
- ❌ Never pass arguments that weren't explicitly provided by the user

### ALWAYS verify:
- ✅ Check `ros2 pkg list` for package existence
- ✅ Check `ros2 pkg files <package>` for launch files
- ✅ Run `ros2 launch <pkg> <file> --show-args` to get valid arguments
- ✅ Validate each user-provided argument against --show-args output

### Rule: Only ask when genuinely ambiguous

Per Rule 5, the user's message is the approval. For launch files:
- Exactly one match → launch it immediately, no confirmation needed
- Multiple matches with no way to disambiguate → list them and ask which one
- No match at all → ask for exact package/file name

### Local Workspace Sourcing

**System ROS is assumed to be already sourced** (via systemd service or manually). The skill automatically sources any local workspace on top of system ROS.

**Search order:**
1. `ROS2_LOCAL_WS` environment variable
2. `~/ros2_ws`
3. `~/colcon_ws`
4. `~/dev_ws`
5. `~/workspace`
6. `~/ros2`

**Behavior:**
| Scenario | Behavior |
|----------|----------|
| Workspace found + built | Source automatically, run silently |
| Workspace found + NOT built | Warn user, run without sourcing |
| Workspace NOT found | Continue without sourcing (system ROS only) |

**Override option:**
```bash
# Set environment variable before running
export ROS2_LOCAL_WS=~/my_robot_ws
```

### TF2 Transforms

```bash
# List all coordinate frames
python3 {baseDir}/scripts/ros2_cli.py tf list

# Lookup transform between frames
python3 {baseDir}/scripts/ros2_cli.py tf lookup base_link map

# Echo transform continuously
python3 {baseDir}/scripts/ros2_cli.py tf echo base_link map --count 10

# Echo transform once
python3 {baseDir}/scripts/ros2_cli.py tf echo base_link map --once

# Monitor a specific frame
python3 {baseDir}/scripts/ros2_cli.py tf monitor base_link --count 5

# Publish static transform — named form
python3 {baseDir}/scripts/ros2_cli.py tf static --from base_link --to sensor --xyz 1 2 3 --rpy 0 0 0

# Publish static transform — positional form
python3 {baseDir}/scripts/ros2_cli.py tf static 0 0 0 0 0 0 base_link odom

# Convert quaternion to Euler (radians) — also: e2q, quat2euler
python3 {baseDir}/scripts/ros2_cli.py tf euler-from-quaternion 0 0 0 1

# Convert Euler to quaternion (radians) — also: q2e, euler2quat
python3 {baseDir}/scripts/ros2_cli.py tf quaternion-from-euler 0 0 1.57

# Convert quaternion to Euler (degrees) — also: e2qdeg
python3 {baseDir}/scripts/ros2_cli.py tf euler-from-quaternion-deg 0 0 0 1

# Convert Euler to quaternion (degrees) — also: q2edeg
python3 {baseDir}/scripts/ros2_cli.py tf quaternion-from-euler-deg 0 0 90

# Transform a point between frames — also: tp, point
python3 {baseDir}/scripts/ros2_cli.py tf transform-point map base_link 1 0 0

# Transform a vector between frames — also: tv, vector
python3 {baseDir}/scripts/ros2_cli.py tf transform-vector map base_link 1 0 0
```

### Run a Launch File

```bash
# Basic launch
python3 {baseDir}/scripts/ros2_cli.py launch new navigation2 navigation2.launch.py

# With arguments - MUST verify arguments exist first
python3 {baseDir}/scripts/ros2_cli.py launch new navigation2 navigation2.launch.py arg1:=value arg2:=value
```

### ⚠️ STRICT: Launch Argument Validation

**This is critical for safety. Passing incorrect arguments has caused accidents.**

Rules, in order:

1. **Always fetch available arguments first** via `--show-args` before passing any args.
2. **Exact match** → pass as-is.
3. **No exact match** → fuzzy-match against the real available args only.
   - If a close match is found (e.g. "mock" → "use_mock") → use it, but **notify the user**.
   - The substitution is shown in `arg_notices` in the output.
4. **No match at all** → **drop the argument, do NOT pass it, notify the user.**
   - The launch still proceeds without that argument.
   - The user is told which argument was dropped and what the available args are.
5. **Never invent or assume argument names.** Only use names that exist in `--show-args` output.
6. **If `--show-args` fails** → drop all user-provided args, notify the user, still launch.

### Run an Executable

Run a ROS 2 executable in a tmux session. Similar to launch commands but for single executables. Auto-detects executable names (e.g., "teleop" matches "teleop_node").

```bash
# Run an executable
python3 {baseDir}/scripts/ros2_cli.py run new lekiwi_control teleop

# Run with arguments
python3 {baseDir}/scripts/ros2_cli.py run new lekiwi_control teleop --arg1 value

# Run with parameters
python3 {baseDir}/scripts/ros2_cli.py run new lekiwi_control teleop --params "speed:1.0"

# Run with presets
python3 {baseDir}/scripts/ros2_cli.py run new lekiwi_control teleop --presets indoor

# Run with config path
python3 {baseDir}/scripts/ros2_cli.py run new lekiwi_control teleop --config-path /path/to/config
```

### List Running Executables

```bash
python3 {baseDir}/scripts/ros2_cli.py run list
```

### Kill an Executable Session

```bash
python3 {baseDir}/scripts/ros2_cli.py run kill run_lekiwi_control_teleop
```

### Restart an Executable Session

```bash
python3 {baseDir}/scripts/ros2_cli.py run restart run_lekiwi_control_teleop
```

### Run Session Collision Handling

Same as launch - if a session with the same name already exists, the command will fail with an error. Use `run restart` or `run kill` first.

## Command Quick Reference

### 1. Explore a Robot System

```bash
python3 {baseDir}/scripts/ros2_cli.py version
python3 {baseDir}/scripts/ros2_cli.py topics list
python3 {baseDir}/scripts/ros2_cli.py nodes list
python3 {baseDir}/scripts/ros2_cli.py services list
python3 {baseDir}/scripts/ros2_cli.py actions list

# Find a topic by type, then inspect it
# (replace the type with whatever you're looking for)
python3 {baseDir}/scripts/ros2_cli.py topics find geometry_msgs/msg/Twist
python3 {baseDir}/scripts/ros2_cli.py topics find geometry_msgs/msg/TwistStamped
# → get the discovered topic name, then:
python3 {baseDir}/scripts/ros2_cli.py topics type <discovered_topic>
python3 {baseDir}/scripts/ros2_cli.py interface proto <confirmed_type>
```

### 2. Move a Robot

Follow the [Movement Workflow](#movement-workflow-canonical) section. It covers all cases: distance commands, rotation commands, open-ended movement, and the no-odometry fallback. Always discover topics first — never assume names.

**Emergency stop:**
```bash
python3 {baseDir}/scripts/ros2_cli.py estop
```

### 3. Read Sensor Data

**Always use auto-discovery first** to find the correct sensor topics.

```bash
# Step 1: Discover sensor topics by message type
python3 {baseDir}/scripts/ros2_cli.py topics find sensor_msgs/msg/LaserScan
python3 {baseDir}/scripts/ros2_cli.py topics find nav_msgs/msg/Odometry
python3 {baseDir}/scripts/ros2_cli.py topics find sensor_msgs/msg/JointState
# → record each discovered topic name

# Step 2: Subscribe to discovered topics (use the names from Step 1, not hardcoded names)
python3 {baseDir}/scripts/ros2_cli.py topics subscribe <LASER_TOPIC> --duration 3
python3 {baseDir}/scripts/ros2_cli.py topics subscribe <ODOM_TOPIC> --duration 10 --max-messages 50
python3 {baseDir}/scripts/ros2_cli.py topics subscribe <JOINT_STATE_TOPIC> --duration 5
```

### 4. Use Services

**Always use auto-discovery first** to find available services and their request/response structures.

```bash
# Step 1: Discover available services
python3 {baseDir}/scripts/ros2_cli.py services list

# Step 2: Find services by type (optional)
python3 {baseDir}/scripts/ros2_cli.py services find std_srvs/srv/Empty
python3 {baseDir}/scripts/ros2_cli.py services find turtlesim/srv/Spawn

# Step 3: Get service request/response structure
python3 {baseDir}/scripts/ros2_cli.py services details /spawn

# Step 4: Call the service with properly-structured payload
python3 {baseDir}/scripts/ros2_cli.py services call /spawn \
  '{"x":3.0,"y":3.0,"theta":0.0,"name":"turtle2"}'
```

### 5. Actions

**Always use auto-discovery first** to find available actions and their goal/result structures.

```bash
# Step 1: Discover available actions
python3 {baseDir}/scripts/ros2_cli.py actions list

# Step 2: Find actions by type (optional)
python3 {baseDir}/scripts/ros2_cli.py actions find turtlesim/action/RotateAbsolute
python3 {baseDir}/scripts/ros2_cli.py actions find nav2_msgs/action/NavigateToPose

# Step 3: Get action goal/result structure
python3 {baseDir}/scripts/ros2_cli.py actions details /turtle1/rotate_absolute

# Step 4: Send goal with properly-structured payload
python3 {baseDir}/scripts/ros2_cli.py actions send /turtle1/rotate_absolute \
  '{"theta":1.57}'
```

### 6. Change Parameters

**Always use auto-discovery first** to list available parameters for a node.

```bash
# Step 1: Discover nodes
python3 {baseDir}/scripts/ros2_cli.py nodes list

# Step 2: List parameters for a node
python3 {baseDir}/scripts/ros2_cli.py params list /turtlesim

# Step 3: Get current parameter value
python3 {baseDir}/scripts/ros2_cli.py params get /turtlesim:background_r

# Step 4: Set parameter value
python3 {baseDir}/scripts/ros2_cli.py params set /turtlesim:background_r 255
python3 {baseDir}/scripts/ros2_cli.py params set /turtlesim:background_g 0
python3 {baseDir}/scripts/ros2_cli.py params set /turtlesim:background_b 0
```

### 7. Goal-Oriented Commands (publish-until)

For any goal with a sensor-based stop condition — joint angles, temperature limits, proximity, battery level — use `publish-until` with the appropriate monitor topic and condition flag. **Always discover both the command topic and the monitor topic before executing** — never hardcode names.

| User intent | Discover command topic | Discover monitor topic | Condition |
|-------------|----------------------|----------------------|-----------|
| Stop near obstacle | `topics find geometry_msgs/Twist` + `TwistStamped` | `topics find sensor_msgs/LaserScan` → field `ranges.0` | `--below 0.5` |
| Stop at range | same | `topics find sensor_msgs/Range` → field `range` | `--below D` |
| Stop at temperature | — | `topics find sensor_msgs/Temperature` → field `temperature` | `--above T` |
| Stop at battery level | — | `topics find sensor_msgs/BatteryState` → field `percentage` | `--below P` |
| Joint reach angle | `topics find trajectory_msgs/JointTrajectory` or similar | `topics find sensor_msgs/JointState` → field `position.0` | `--equals A` or `--delta D` |
| Multi-joint distance | same | `topics find sensor_msgs/JointState` → fields `position.0 position.1` | `--euclidean --delta D` |

**`--euclidean`** computes `sqrt(Σ(current_i - start_i)²)` across all `--field` paths. Use it for curved or diagonal paths. **Composite field shorthand**: `--field pose.pose.position` auto-expands to `x, y, z` — equivalent to listing all three fields explicitly.

```bash
# Example: stop when front range sensor reads < 0.5 m
# Step 1: discover velocity topic
python3 {baseDir}/scripts/ros2_cli.py topics find geometry_msgs/msg/Twist
python3 {baseDir}/scripts/ros2_cli.py topics find geometry_msgs/msg/TwistStamped
# → VEL_TOPIC = (result, e.g. /base/cmd_vel)
python3 {baseDir}/scripts/ros2_cli.py topics type <VEL_TOPIC>
# → confirms type, determines payload structure

# Step 2: discover laser scan topic
python3 {baseDir}/scripts/ros2_cli.py topics find sensor_msgs/msg/LaserScan
# → SCAN_TOPIC = (result, e.g. /scan or /lidar/scan)

# Step 3: execute with discovered names
python3 {baseDir}/scripts/ros2_cli.py topics publish-until <VEL_TOPIC> \
  '<payload_matching_confirmed_type>' \
  --monitor <SCAN_TOPIC> --field ranges.0 --below 0.5 --timeout 30

# Example: move a joint until it reaches 1.5 rad
# Step 1: discover joint command topic
python3 {baseDir}/scripts/ros2_cli.py topics find trajectory_msgs/msg/JointTrajectory
# → or check nodes details for the arm controller node
# → JOINT_CMD_TOPIC = (result)

# Step 2: discover joint state topic
python3 {baseDir}/scripts/ros2_cli.py topics find sensor_msgs/msg/JointState
# → JOINT_STATE_TOPIC = (result)

# Step 3: execute with discovered names
python3 {baseDir}/scripts/ros2_cli.py topics publish-until <JOINT_CMD_TOPIC> \
  '{"data":0.5}' \
  --monitor <JOINT_STATE_TOPIC> --field position.0 --equals 1.5 --timeout 10
```

---

<a name="movement-workflow-canonical"></a>
## Movement Workflow (canonical)

This is the single authoritative workflow for all robot movement. Rule 3 in the Agent Behaviour Rules above is a summary of this section. Always follow these steps in order.

**In every example below, `<VEL_TOPIC>`, `<ODOM_TOPIC>`, and `<PAYLOAD>` are placeholders for values you discover in Steps 1–2. Never substitute `/cmd_vel`, `/odom`, or any other assumed name.**

### Step 1 — Discover the velocity command topic and confirm its exact type

Run both searches in parallel:

```bash
python3 {baseDir}/scripts/ros2_cli.py topics find geometry_msgs/msg/Twist
python3 {baseDir}/scripts/ros2_cli.py topics find geometry_msgs/msg/TwistStamped
```

Take the result — this is `VEL_TOPIC`. Then confirm the exact type:

```bash
python3 {baseDir}/scripts/ros2_cli.py topics type <VEL_TOPIC>
```

The confirmed type determines the payload:
- **`geometry_msgs/msg/Twist`**: `{"linear":{"x":0.2,"y":0,"z":0},"angular":{"x":0,"y":0,"z":0}}`
- **`geometry_msgs/msg/TwistStamped`**: `{"header":{"stamp":{"sec":0},"frame_id":""},"twist":{"linear":{"x":0.2,"y":0,"z":0},"angular":{"x":0,"y":0,"z":0}}}`
- Zero/stop — **Twist**: `{"linear":{"x":0,"y":0,"z":0},"angular":{"x":0,"y":0,"z":0}}`
- Zero/stop — **TwistStamped**: `{"header":{"stamp":{"sec":0},"frame_id":""},"twist":{"linear":{"x":0,"y":0,"z":0},"angular":{"x":0,"y":0,"z":0}}}`

If both find commands return results, run `topics type` on each to get the definitive type — do not guess from the topic name. Then choose using this priority order:
1. Prefer the topic with an active subscriber (`topics details <topic>` → `subscriber_count > 0`) — a subscribed topic means a controller is listening.
2. If both have subscribers, prefer the topic whose name contains `cmd_vel` or is at the root namespace (e.g. `/cmd_vel` over `/robot/cmd_vel_stamped`).
3. If still ambiguous, prefer `TwistStamped` over `Twist` (more modern standard).
Use the selected topic as `VEL_TOPIC` for the rest of the workflow.

### Step 2 — Discover the odometry topic

```bash
python3 {baseDir}/scripts/ros2_cli.py topics find nav_msgs/msg/Odometry
```

Take the result — this is `ODOM_TOPIC`.

### Step 2.5 — Verify topics are live, read safety limits, and capture start position

Before executing, verify both topics are active, discover velocity limits from controller parameters, and record the starting position.

**Check if the robot is already in motion** — do not command a moving robot:
```bash
python3 {baseDir}/scripts/ros2_cli.py topics subscribe <ODOM_TOPIC> --max-messages 1 --timeout 3
```
Inspect the `twist.twist.linear` and `twist.twist.angular` fields. If any value is significantly non-zero (> 0.01 m/s or rad/s), the robot is already moving. Do not publish new velocity commands — stop it first with `estop`, wait for motion to cease, then proceed.

**Verify the odometry publish rate is adequate for closed-loop control:**
```bash
python3 {baseDir}/scripts/ros2_cli.py topics hz <ODOM_TOPIC>
```
The rate must be at least 10 Hz for reliable closed-loop control. If the rate is below 5 Hz: fall back to Case D (open-loop) and warn the user that odometry is too slow for safe closed-loop operation.

**Discover velocity limits from controller parameters:**
```bash
python3 {baseDir}/scripts/ros2_cli.py nodes list
# → find controller nodes (names containing: controller, driver, base, mobile, etc.) → CTRL_NODE
python3 {baseDir}/scripts/ros2_cli.py params list <CTRL_NODE>
# → find limit parameters: max_vel_x, max_vel_theta, max_linear_velocity, max_angular_velocity,
#   speed_limit, linear.x.max_velocity, angular.z.max_velocity, or any param with max/limit/vel
python3 {baseDir}/scripts/ros2_cli.py params get <CTRL_NODE>:<discovered_limit_param>
# → repeat for each relevant limit found
```
Cap all commanded velocities at the discovered limits. If no limits are found, use conservative defaults (0.1 m/s linear, 0.3 rad/s angular) and note this.

**Verify the velocity topic has active subscribers** (something must be listening — i.e., a controller):
```bash
python3 {baseDir}/scripts/ros2_cli.py topics details <VEL_TOPIC>
```
Check the output for `subscriber_count > 0`. If no subscribers: the controller may not be running. Check `control list-controllers` and activate the correct controller before proceeding.

**Check for emergency stop / safety interlock state:**
```bash
python3 {baseDir}/scripts/ros2_cli.py control list-controllers
```
Verify that the motion controller is in `active` state. If it is `inactive`, `unconfigured`, or missing: do not publish velocity — activate it first or notify the user that the controller is not ready.

**Verify the odometry topic has an active publisher** (not stale):
```bash
python3 {baseDir}/scripts/ros2_cli.py topics details <ODOM_TOPIC>
```
Check `publisher_count > 0`. Then confirm it is actively publishing by subscribing briefly:
```bash
python3 {baseDir}/scripts/ros2_cli.py topics subscribe <ODOM_TOPIC> --max-messages 1 --timeout 3
```
If no message arrives within the timeout: the odometry source is stale or not running. Fall back to Case D (open-loop) and notify the user.

**Capture start position** (for distance reporting after motion):
```bash
python3 {baseDir}/scripts/ros2_cli.py topics subscribe <ODOM_TOPIC> --max-messages 1
```
Record `pose.pose.position.x`, `pose.pose.position.y` from the result — this is the start pose.

### Step 3 — Execute based on intent and odometry availability

#### Case A — Distance specified, odometry available → `publish-until --field` (closed loop)

```bash
# Step 1 result: VEL_TOPIC = <discovered, e.g. /base/cmd_vel or /robot/cmd_vel>
# Step 1 type check: geometry_msgs/msg/Twist → use Twist payload
# Step 2 result: ODOM_TOPIC = <discovered, e.g. /odom or /wheel_odom>

# Move forward 1 meter
python3 {baseDir}/scripts/ros2_cli.py topics publish-until <VEL_TOPIC> \
  '{"linear":{"x":0.2,"y":0,"z":0},"angular":{"x":0,"y":0,"z":0}}' \
  --monitor <ODOM_TOPIC> --field pose.pose.position.x --delta 1.0 --timeout 30

# Move forward 1 meter — TwistStamped variant (if type check returned TwistStamped)
python3 {baseDir}/scripts/ros2_cli.py topics publish-until <VEL_TOPIC> \
  '{"header":{"stamp":{"sec":0},"frame_id":""},"twist":{"linear":{"x":0.2,"y":0,"z":0},"angular":{"x":0,"y":0,"z":0}}}' \
  --monitor <ODOM_TOPIC> --field pose.pose.position.x --delta 1.0 --timeout 30

# Move backward 0.5 meters (negative delta)
python3 {baseDir}/scripts/ros2_cli.py topics publish-until <VEL_TOPIC> \
  '{"linear":{"x":-0.2,"y":0,"z":0},"angular":{"x":0,"y":0,"z":0}}' \
  --monitor <ODOM_TOPIC> --field pose.pose.position.x --delta -0.5 --timeout 30

# Move 2 meters in any direction (curved path — Euclidean distance)
python3 {baseDir}/scripts/ros2_cli.py topics publish-until <VEL_TOPIC> \
  '{"linear":{"x":0.2,"y":0,"z":0},"angular":{"x":0,"y":0,"z":0.3}}' \
  --monitor <ODOM_TOPIC> --field pose.pose.position --euclidean --delta 2.0 --timeout 60
```

#### Case B — Rotation specified, odometry available → `publish-until --rotate` (closed loop)

`--rotate` automatically extracts yaw from the odometry quaternion and handles angle wraparound. **There is no `--yaw` flag.** Do not use `--field` for rotation.

**Direction convention — `--rotate` sign and `angular.z` sign must always agree:**

| Direction | `--rotate` | `angular.z` |
|-----------|-----------|-------------|
| Left / CCW | positive (e.g. `--rotate 90 --degrees`) | positive (e.g. `0.5`) |
| Right / CW | negative (e.g. `--rotate -90 --degrees`) | negative (e.g. `-0.5`) |

`--rotate` tells the monitor *how far and in which direction* to track. `angular.z` tells the robot *which way to spin*. They must match — a positive `--rotate` with negative `angular.z` means the robot turns CW while the monitor waits for CCW accumulation and will never stop.

**Before constructing any rotation command, run `--help` to confirm accepted flags:**

```bash
python3 {baseDir}/scripts/ros2_cli.py topics publish-until --help
```

Do not proceed until you have verified the flag names from the `--help` output. Never invent or assume flags.

```bash
# Step 1 result: VEL_TOPIC = <discovered>
# Step 2 result: ODOM_TOPIC = <discovered>

# Rotate 90 degrees counter-clockwise — positive --rotate, positive angular.z (Twist)
python3 {baseDir}/scripts/ros2_cli.py topics publish-until <VEL_TOPIC> \
  '{"linear":{"x":0,"y":0,"z":0},"angular":{"x":0,"y":0,"z":0.5}}' \
  --monitor <ODOM_TOPIC> --rotate 90 --degrees --timeout 30

# Rotate 45 degrees clockwise — negative --rotate, negative angular.z (Twist)
python3 {baseDir}/scripts/ros2_cli.py topics publish-until <VEL_TOPIC> \
  '{"linear":{"x":0,"y":0,"z":0},"angular":{"x":0,"y":0,"z":-0.5}}' \
  --monitor <ODOM_TOPIC> --rotate -45 --degrees --timeout 30

# Rotate 1.57 radians CCW (TwistStamped variant)
python3 {baseDir}/scripts/ros2_cli.py topics publish-until <VEL_TOPIC> \
  '{"header":{"stamp":{"sec":0},"frame_id":""},"twist":{"linear":{"x":0,"y":0,"z":0},"angular":{"x":0,"y":0,"z":0.5}}}' \
  --monitor <ODOM_TOPIC> --rotate 1.5708 --timeout 30
```

#### Case C — Open-ended movement (no distance/angle) → `publish-sequence`

Always include a zero-velocity stop as the final message. Use the payload structure matching the confirmed type from Step 1.

```bash
# Step 1 result: VEL_TOPIC = <discovered>
# Type check: geometry_msgs/msg/Twist → Twist payload

# Drive forward for 3 seconds then stop (Twist)
python3 {baseDir}/scripts/ros2_cli.py topics publish-sequence <VEL_TOPIC> \
  '[{"linear":{"x":0.2,"y":0,"z":0},"angular":{"x":0,"y":0,"z":0}},{"linear":{"x":0,"y":0,"z":0},"angular":{"x":0,"y":0,"z":0}}]' \
  '[3.0, 0.5]'

# Drive forward for 3 seconds then stop (TwistStamped variant)
python3 {baseDir}/scripts/ros2_cli.py topics publish-sequence <VEL_TOPIC> \
  '[{"header":{"stamp":{"sec":0},"frame_id":""},"twist":{"linear":{"x":0.2,"y":0,"z":0},"angular":{"x":0,"y":0,"z":0}}},{"header":{"stamp":{"sec":0},"frame_id":""},"twist":{"linear":{"x":0,"y":0,"z":0},"angular":{"x":0,"y":0,"z":0}}}]' \
  '[3.0, 0.5]'
```

#### Case D — Distance or angle specified, but odometry NOT available → `publish-sequence` (open loop)

Tell the user before executing: *"No odometry topic found. Running open-loop — distance/angle accuracy is not guaranteed."* Then estimate `duration = distance / velocity` and use `publish-sequence` with a stop.

```bash
# Step 1 result: VEL_TOPIC = <discovered>
# Step 2 result: no odometry found

# Estimate: 1 m at 0.2 m/s ≈ 5 s (Twist)
python3 {baseDir}/scripts/ros2_cli.py topics publish-sequence <VEL_TOPIC> \
  '[{"linear":{"x":0.2,"y":0,"z":0},"angular":{"x":0,"y":0,"z":0}},{"linear":{"x":0,"y":0,"z":0},"angular":{"x":0,"y":0,"z":0}}]' \
  '[5.0, 0.5]'

# Estimate: 1 m at 0.2 m/s ≈ 5 s (TwistStamped variant)
python3 {baseDir}/scripts/ros2_cli.py topics publish-sequence <VEL_TOPIC> \
  '[{"header":{"stamp":{"sec":0},"frame_id":""},"twist":{"linear":{"x":0.2,"y":0,"z":0},"angular":{"x":0,"y":0,"z":0}}},{"header":{"stamp":{"sec":0},"frame_id":""},"twist":{"linear":{"x":0,"y":0,"z":0},"angular":{"x":0,"y":0,"z":0}}}]' \
  '[5.0, 0.5]'
```

### Step 4 — Report completion

After `publish-until` or `publish-sequence` completes, capture the end position and report to the user:

```bash
python3 {baseDir}/scripts/ros2_cli.py topics subscribe <ODOM_TOPIC> --max-messages 1
```

Compare with the start pose from Step 2.5. By default, report only: **"Done. Moved X.XX m."** (or "Done. Rotated X deg.") and any errors or anomalies. Report detailed topic selections, odometry values, and per-step data only if the user explicitly asked for verbose output.

---

## Quick Examples

### 1. Explore a Robot System
```bash
python3 {baseDir}/scripts/ros2_cli.py version
python3 {baseDir}/scripts/ros2_cli.py topics list
python3 {baseDir}/scripts/ros2_cli.py nodes list
python3 {baseDir}/scripts/ros2_cli.py services list
```

### 2. Move a Robot

See the [Movement Workflow](#movement-workflow-canonical) section — it covers all cases with complete examples.

### 3. Read Sensors
```bash
python3 {baseDir}/scripts/ros2_cli.py topics find sensor_msgs/msg/LaserScan
# → subscribe to the discovered topic name, not /scan
python3 {baseDir}/scripts/ros2_cli.py topics subscribe <LASER_TOPIC> --duration 3
```

### 4. Call a Service
```bash
# Step 1: Discover available services
python3 {baseDir}/scripts/ros2_cli.py services list

# Step 2: Get request/response structure for the service you want to call
python3 {baseDir}/scripts/ros2_cli.py services details <DISCOVERED_SERVICE>

# Step 3: Call with properly-structured payload
python3 {baseDir}/scripts/ros2_cli.py services call <DISCOVERED_SERVICE> '<json_payload>'
```

### 5. Send an Action
```bash
# Step 1: Discover available actions
python3 {baseDir}/scripts/ros2_cli.py actions list

# Step 2: Get goal/result structure for the action you want to send
python3 {baseDir}/scripts/ros2_cli.py actions details <DISCOVERED_ACTION>

# Step 3: Send goal with properly-structured payload
python3 {baseDir}/scripts/ros2_cli.py actions send <DISCOVERED_ACTION> '<json_goal>'
```

### 6. Move Forward N Meters / Rotate N Degrees

See the [Movement Workflow](#movement-workflow-canonical) section — Cases A and B cover distance and rotation with full examples.

---

## Safety Notes

**Destructive commands** (can move the robot or change state):
- `topics publish` / `topics publish-sequence` — sends movement or control commands
- `topics publish-until` — publishes continuously until a condition or timeout; always specify a conservative `--timeout`
- `topics publish-continuous` — alias for `topics publish`; `--duration` / `--timeout` is optional (single-shot without it)
- `services call` — can reset, spawn, kill, or change robot state
- `params set` — modifies runtime parameters
- `actions send` — triggers robot actions (rotation, navigation, etc.)
- `control set-controller-state` / `control switch-controllers` — activating or deactivating controllers affects what the robot can do
- `control set-hardware-component-state` — driving hardware through lifecycle states can stop actuators or sensors
- `control reload-controller-libraries` — stops all running controllers if `--force-kill` is used

**Always stop the robot after movement.** The last message in any `publish-sequence` should be all zeros, using the payload structure that matches the confirmed type (Twist or TwistStamped — from your `topics type` introspection step):
```json
// Twist stop
{"linear":{"x":0,"y":0,"z":0},"angular":{"x":0,"y":0,"z":0}}
// TwistStamped stop
{"header":{"stamp":{"sec":0},"frame_id":""},"twist":{"linear":{"x":0,"y":0,"z":0},"angular":{"x":0,"y":0,"z":0}}}
```

**Always check JSON output for errors before proceeding.**

**Use auto-discovery to avoid errors.** Before any publish/call/send operation:
1. Use `topics find`, `services find`, `actions find` to locate relevant endpoints
2. Use `topics type`, `services type`, `actions type` to get message types
3. Use `topics message`, `services details`, `actions details` to understand field structures

Never ask the user for topic names or message types — discover them from the live ROS 2 graph.

---

## Troubleshooting

### Agent Self-Correction Rules

| If Agent Asks... | Correction |
|------------------|------------|
| "What topic should I use?" | Use `topics find geometry_msgs/Twist` (and TwistStamped) |
| "What message type?" | Use `topics type <topic>` to get it from the graph |
| "How do I know the message structure?" | Use `topics message <type>` or `interface show <type>` |
| "What are the velocity limits?" | Use `params list` on controller nodes |
| "Do I need odometry?" | Always check with `topics find nav_msgs/Odometry` first |
| "Is there a service called /X?" | Use `services list` to verify it exists |
| "How do I monitor yaw / heading?" | Use `--rotate N --degrees` (or radians). There is no `--yaw` or `--yaw-delta` flag. `--rotate` handles all yaw tracking internally. |
| "Should I use --field for rotation?" | No. Never use `--field` for rotation. Use `--rotate`. |
| "How do I rotate clockwise / right?" | Use `--rotate -90 --degrees` (negative angle) with `angular.z: -0.5` (negative velocity). Sign of `--rotate` and `angular.z` must always match. |
| "Can --rotate be negative?" | Yes. Negative = CW. Positive = CCW. Zero is the only invalid value. |
| "What controller parameters exist?" | Use `params list <controller_node>` |

### Technical Troubleshooting

| Problem | Cause | Solution |
|---------|-------|----------|
| `Missing ROS 2 dependency: No module named 'X'` | A required ROS 2 package is not installed | Source ROS 2: `source /opt/ros/${ROS_DISTRO}/setup.bash`; then install: `sudo apt install ros-${ROS_DISTRO}-<package>` |
| rclpy not installed | rclpy missing or wrong Python version | Source ROS 2 setup.bash; if Python version mismatch, run with `python3.12` instead of `python3` |
| ROS 2 not sourced | Environment not set up | Run: `source /opt/ros/${ROS_DISTRO}/setup.bash` |
| No topics found | ROS nodes not running | Ensure nodes are launched and workspace is sourced |
| Service not found | Service not available | Use `services list` to see available services |
| Parameter commands fail | Node doesn't have parameters | Some nodes don't expose parameters |
| Action commands fail | Action server not available | Use `actions list` to see available actions |
| Invalid JSON error | Malformed message | Validate JSON before passing (watch for single vs double quotes) |
| Subscribe timeout | No publisher on topic | Check `topics details` to verify publishers exist |
| publish-sequence length error | Array mismatch | `messages` and `durations` arrays must have the same length |
| publish-until hangs / no feedback | Wrong monitor topic or field | Use Step 2–4 of the Goal-Oriented workflow to verify topic and field |
| Controller manager service not available | ros2_control not running or wrong namespace | Check with `nodes list`; use `--controller-manager` to set the correct namespace |
