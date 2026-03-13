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

### Rule 3 — Distance and angle commands require odometry feedback

**Whenever the user asks the robot to move a specific distance or rotate a specific angle, odometry MUST be used as the feedback source. Never use timing or dead reckoning as the primary control method.**

- "Move forward 1 metre" → find odometry topic, use `publish-until --delta 1.0`
- "Rotate 90 degrees" → find odometry topic, use `publish-until --rotate 90 --degrees`
- "Turn left by 45 degrees" → find odometry topic, use `publish-until --rotate 45 --degrees`
- "Back up 0.5 metres" → find odometry topic, use `publish-until --delta -0.5`

**Workflow:**
1. Always run `topics find nav_msgs/Odometry` first.
2. If odometry is found → use `publish-until` with `--monitor <odom_topic>` and the appropriate `--delta` or `--rotate` flag. Do not estimate with time.
3. Only if odometry is genuinely not available (topic not found, no publishers) → fall back to dead reckoning with `publish-sequence` and time. Notify the user that odometry was not found and that distance/angle accuracy is not guaranteed.

**Never**:
- ❌ Use `publish-sequence` with a fixed time to approximate distance when odometry is available
- ❌ Calculate `time = distance / velocity` as the primary method when odometry exists
- ❌ Assume the robot reached the target position without odometry confirmation

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
  2. Is X about controlling/moving? → Use TOPICS PUBLISH
  3. Is X a one-time trigger? → Use SERVICES CALL or ACTIONS SEND
  4. Is X about system info? → Use LIST commands

Agent does:
  1. EXPLORE: Run topics/services/actions list
  2. SEARCH: Run find command for relevant message type
  3. STRUCTURE: Get message structure
  4. LIMITS: Get params (for movement)
  5. EXECUTE: Run the command
```

**Never skip steps 1-4. Always explore first.**

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
| "Move/drive/turn (mobile robot)" | Publish Twist | Find Twist/TwistStamped topics → publish |
| "Move arm/joint (manipulator)" | Publish JointTrajectory | Find JointTrajectory topic → publish |
| "Control gripper" | Publish GripperCommand or JointTrajectory | Find gripper topic → publish |
| "Move forward/back N meters" | Publish with feedback | Find odom → publish-until |
| "Stop the robot" | Publish zero velocity | Find Twist → publish zeros |
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
| "Run diagnostics/health check" | Run doctor | Run `doctor check` |
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
| Velocity command topic (mobile) | `topics find geometry_msgs/Twist` AND `topics find geometry_msgs/TwistStamped` |
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

**Before publishing or calling, always get the structure:**

```bash
# Get field structure (for building payloads)
python3 {baseDir}/scripts/ros2_cli.py topics message <message_type>

# Get default values (copy-paste template)
python3 {baseDir}/scripts/ros2_cli.py interface proto <message_type>

# Get service/action request structure
python3 {baseDir}/scripts/ros2_cli.py services details <service_name>
python3 {baseDir}/scripts/ros2_cli.py actions details <action_name>
```

### Step 5: Get Safety Limits (for movement)

**ALWAYS check for velocity limits before publishing movement commands:**

```bash
# Find controller nodes first
python3 {baseDir}/scripts/ros2_cli.py nodes list

# Common controller nodes to check:
# /diff_drive_controller, /base_controller, /velocity_controller, /mobile_base

# List their parameters
python3 {baseDir}/scripts/ros2_cli.py params list /diff_drive_controller
python3 {baseDir}/scripts/ros2_cli.py params list /base_controller

# Get velocity limit parameters (if they exist)
python3 {baseDir}/scripts/ros2_cli.py params get /diff_drive_controller:max_velocity
python3 {baseDir}/scripts/ros2_cli.py params get /diff_drive_controller:max_linear_velocity
python3 {baseDir}/scripts/ros2_cli.py params get /diff_drive_controller:max_angular_velocity
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

**Every movement command MUST end with zero velocity:**

```bash
# WRONG: Move forward without stopping
topics publish /cmd_vel '{"linear":{"x":1.0}}'

# CORRECT: Include stop command
topics publish-sequence /cmd_vel \
  '[{"linear":{"x":1.0},"angular":{"z":0}},{"linear":{"x":0},"angular":{"z":0}}]' \
  '[2.0, 0.5]'
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

### Retry Strategy

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

Check both types:
```bash
python3 {baseDir}/scripts/ros2_cli.py topics find geometry_msgs/msg/Twist
python3 {baseDir}/scripts/ros2_cli.py topics find geometry_msgs/msg/TwistStamped
```
- Twist: `{"linear": {"x": 1.0}, "angular": {"z": 0.0}}`
- TwistStamped: `{"header": {"stamp": {"sec": 0}}, "twist": {"linear": {"x": 1.0}, "angular": {"z": 0.0}}}`

### Quick lookup table

| If you need... | Run this... | Then... |
|----------------|-------------|---------|
| Images | `topics find sensor_msgs/msg/Image` | Subscribe to result |
| LiDAR | `topics find sensor_msgs/msg/LaserScan` | Subscribe to result |
| Odometry | `topics find nav_msgs/msg/Odometry` | Subscribe to result |
| IMU | `topics find sensor_msgs/msg/Imu` | Subscribe to result |
| Joint states | `topics find sensor_msgs/msg/JointState` | Subscribe to result |
| Move robot | `topics find geometry_msgs/msg/Twist` | Publish to result |
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

4. **If multiple candidates found:**
   - Present options to user: "Found 3 launch files. Which one?"
   - Or ask for confirmation: "I found navigation2. Launch it?"

5. **If no match found:**
   - Search more broadly: check all packages for matching launch files
   - Ask user for exact package/file name

### NEVER hallucinate:
- ❌ Never invent a package name that doesn't exist
- ❌ Never invent a launch file that doesn't exist
- ❌ Never assume a package exists without checking
- ❌ Never guess launch argument names (e.g., don't assume "mock" → "use_mock")
- ❌ Never auto-correct or fuzzy-match argument names
- ❌ Never pass arguments that weren't explicitly provided by the user

### ALWAYS verify:
- ✅ Check `ros2 pkg list` for package existence
- ✅ Check `ros2 pkg files <package>` for launch files
- ✅ Run `ros2 launch <pkg> <file> --show-args` to get valid arguments
- ✅ Validate each user-provided argument against --show-args output
- ✅ Confirm with user if any doubt about which file/package/argument

### Rule: Confirm before executing if uncertain

If the skill finds a match but there's any doubt (multiple candidates, ambiguous request):
- "I found multiple options. Which one?"
- "I assume you mean X. Launch it?"
- Only proceed without asking if there's exactly ONE clear match

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
python3 {baseDir}/scripts/ros2_cli.py topics type /cmd_vel
python3 {baseDir}/scripts/ros2_cli.py topics message geometry_msgs/Twist
python3 {baseDir}/scripts/ros2_cli.py actions list
python3 {baseDir}/scripts/ros2_cli.py params list /robot_node
```

### 2. Move a Robot

**ALWAYS use auto-discovery first** to find the correct velocity topic and message type. Never assume topic names.

**If the user specifies a distance or angle, Rule 3 applies: you MUST use `publish-until` with odometry.**

```bash
# Step 1: Discover velocity command topics (check both Twist and TwistStamped)
python3 {baseDir}/scripts/ros2_cli.py topics find geometry_msgs/Twist
python3 {baseDir}/scripts/ros2_cli.py topics find geometry_msgs/TwistStamped

# Step 2: Discover odometry topic (REQUIRED for distance/angle commands)
python3 {baseDir}/scripts/ros2_cli.py topics find nav_msgs/Odometry

# Step 3a: Distance/angle command — use publish-until with odometry (MANDATORY)
# Move forward 1 meter
python3 {baseDir}/scripts/ros2_cli.py topics publish-until /cmd_vel \
  '{"linear":{"x":0.2},"angular":{"z":0}}' \
  --monitor /odom --field pose.pose.position.x --delta 1.0 --timeout 30

# Rotate 90 degrees
python3 {baseDir}/scripts/ros2_cli.py topics publish-until /cmd_vel \
  '{"linear":{"x":0},"angular":{"z":0.5}}' \
  --monitor /odom --rotate 90 --degrees --timeout 30

# Step 3b: FALLBACK ONLY — no distance/angle specified, or odometry genuinely not available
# Use publish-sequence with a stop command at the end.
# DO NOT use this when a distance or angle has been requested and odometry exists.
python3 {baseDir}/scripts/ros2_cli.py topics publish-sequence /cmd_vel \
  '[{"linear":{"x":1.0,"y":0,"z":0},"angular":{"x":0,"y":0,"z":0}},{"linear":{"x":0,"y":0,"z":0},"angular":{"x":0,"y":0,"z":0}}]' \
  '[2.0, 0.5]'
```

**For emergency stop:**
```bash
python3 {baseDir}/scripts/ros2_cli.py estop
```

### 3. Read Sensor Data

**Always use auto-discovery first** to find the correct sensor topics.

```bash
# Step 1: Discover sensor topics by message type
python3 {baseDir}/scripts/ros2_cli.py topics find sensor_msgs/LaserScan
python3 {baseDir}/scripts/ros2_cli.py topics find nav_msgs/Odometry
python3 {baseDir}/scripts/ros2_cli.py topics find sensor_msgs/JointState

# Step 2: Subscribe to discovered topics
python3 {baseDir}/scripts/ros2_cli.py topics subscribe /scan --duration 3
python3 {baseDir}/scripts/ros2_cli.py topics subscribe /odom --duration 10 --max-messages 50
python3 {baseDir}/scripts/ros2_cli.py topics subscribe /joint_states --duration 5
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

When the user wants the robot to **do something until a condition is met** — "drive forward 1 meter", "turn until joint reaches 1.5 rad", "stop when temperature exceeds 50°C" — use `publish-until`. Because the exact topic names, message types, and field paths vary by robot, **always introspect the live system first**.

#### Discovery workflow

**Step 1 — Find the command topic** (what to publish to)
```bash
# Find velocity/command topics
python3 {baseDir}/scripts/ros2_cli.py topics list
python3 {baseDir}/scripts/ros2_cli.py topics find geometry_msgs/msg/Twist
python3 {baseDir}/scripts/ros2_cli.py topics find geometry_msgs/msg/TwistStamped
```

**Step 2 — Find the feedback/sensor topic** (what to monitor)
```bash
# For position/distance — look for odometry
python3 {baseDir}/scripts/ros2_cli.py topics find nav_msgs/msg/Odometry

# For joint angles — look for joint states
python3 {baseDir}/scripts/ros2_cli.py topics find sensor_msgs/msg/JointState

# For temperature — look for temperature topics
python3 {baseDir}/scripts/ros2_cli.py topics find sensor_msgs/msg/Temperature

# For proximity/obstacle — look for laser/range
python3 {baseDir}/scripts/ros2_cli.py topics find sensor_msgs/msg/LaserScan
python3 {baseDir}/scripts/ros2_cli.py topics find sensor_msgs/msg/Range
```

**Step 3 — Inspect the message field structure**
```bash
python3 {baseDir}/scripts/ros2_cli.py topics message nav_msgs/msg/Odometry
python3 {baseDir}/scripts/ros2_cli.py topics message sensor_msgs/msg/JointState
python3 {baseDir}/scripts/ros2_cli.py topics message sensor_msgs/msg/Temperature
```

**Step 4 — Read the current value** to establish a baseline (needed for `--delta`)
```bash
python3 {baseDir}/scripts/ros2_cli.py topics subscribe /odom --duration 2
python3 {baseDir}/scripts/ros2_cli.py topics subscribe /joint_states --duration 2
```

**Step 5 — Construct and run publish-until**

#### Common patterns

| User intent | Monitor topic | Field(s) | Condition |
|-------------|--------------|----------|-----------|
| Move forward N m (straight) | `/odom` | `pose.pose.position.x` | `--delta N` |
| Move backward N m | `/odom` | `pose.pose.position.x` | `--delta -N` |
| Move sideways N m | `/odom` | `pose.pose.position.y` | `--delta N` |
| Move N m (any direction, 2D) | `/odom` | `pose.pose.position.x` `pose.pose.position.y` | `--euclidean --delta N` |
| Move N m (any direction, 3D) | `/odom` | `pose.pose.position.x` `pose.pose.position.y` `pose.pose.position.z` | `--euclidean --delta N` |
| Move N m (shorthand, auto-expanded) | `/odom` | `pose.pose.position` | `--euclidean --delta N` |
| **Rotate N radians** | `/odom` | *(auto-detected)* | **`--rotate N`** |
| **Rotate N degrees** | `/odom` | *(auto-detected)* | **`--rotate N --degrees`** |
| Joint reach angle | `/joint_states` | `position.0` (index of joint) | `--equals A` or `--delta D` |
| Multi-joint Euclidean distance | `/joint_states` | `position.0` `position.1` | `--euclidean --delta D` |
| Stop near obstacle | `/scan` | `ranges.0` (front index) | `--below 0.5` |
| Stop at range | `/range` | `range` | `--below D` |
| Stop at temperature | `/temperature` | `temperature` | `--above T` |
| Stop at battery level | `/battery` | `percentage` | `--below P` |

**`--rotate` flag** (NEW): Rotate the robot by a specific angle without dealing with quaternion math.
- `--rotate N` — rotate N radians (default)
- `--rotate N --degrees` — rotate N degrees
- Automatically extracts quaternion from odometry and monitors angular change
- Handles angle wraparound correctly

**`--euclidean`** takes any number of numeric fields, computes `sqrt(Σ(current_i - start_i)²)`, and stops when that distance ≥ the `--delta` threshold. Use it whenever the robot's path is not axis-aligned.

**Composite field auto-expansion**: if a `--field` path resolves to a sub-message dict (e.g. `pose.pose.position` → `{x, y, z}`), `--euclidean` mode automatically expands it to all direct numeric children (sorted alphabetically: `x, y, z`). This lets you write `--field pose.pose.position --euclidean --delta 1.0` instead of listing every sub-field explicitly. Non-numeric children (strings, nested dicts) are skipped.

**Examples:**
```bash
# Move forward 1 meter
python3 {baseDir}/scripts/ros2_cli.py topics publish-until /cmd_vel \
  '{"linear":{"x":0.2},"angular":{"z":0}}' \
  --monitor /odom --field pose.pose.position.x --delta 1.0 --timeout 30

# Move 2 meters in any direction (Euclidean)
python3 {baseDir}/scripts/ros2_cli.py topics publish-until /cmd_vel \
  '{"linear":{"x":0.2},"angular":{"z":0.3}}' \
  --monitor /odom --field pose.pose.position --euclidean --delta 2.0 --timeout 60

# Rotate 90 degrees (NEW!)
python3 {baseDir}/scripts/ros2_cli.py topics publish-until /cmd_vel \
  '{"linear":{"x":0},"angular":{"z":0.5}}' \
  --monitor /odom --rotate 1.5708 --timeout 30

# Rotate 90 degrees using --degrees flag
python3 {baseDir}/scripts/ros2_cli.py topics publish-until /cmd_vel \
  '{"linear":{"x":0},"angular":{"z":0.5}}' \
  --monitor /odom --rotate 90 --degrees --timeout 30

# Rotate -45 degrees (clockwise)
python3 {baseDir}/scripts/ros2_cli.py topics publish-until /cmd_vel \
  '{"linear":{"x":0},"angular":{"z":-0.5}}' \
  --monitor /odom --rotate 45 --degrees --timeout 30
```

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
```bash
# Discover velocity topic
python3 {baseDir}/scripts/ros2_cli.py topics find geometry_msgs/Twist
# Discover odometry topic (REQUIRED for distance/angle commands — Rule 3)
python3 {baseDir}/scripts/ros2_cli.py topics find nav_msgs/Odometry
# Move forward 1 meter using odometry feedback (MANDATORY when distance is specified)
python3 {baseDir}/scripts/ros2_cli.py topics publish-until /cmd_vel \
  '{"linear":{"x":0.2},"angular":{"z":0}}' \
  --monitor /odom --field pose.pose.position.x --delta 1.0 --timeout 30
# FALLBACK ONLY (no distance specified, or no odometry available): publish-sequence with stop
python3 {baseDir}/scripts/ros2_cli.py topics publish-sequence /cmd_vel \
  '[{"linear":{"x":1.0},"angular":{"z":0}},{"linear":{"x":0},"angular":{"z":0}}]' \
  '[2.0, 0.5]'
```

### 3. Read Sensors
```bash
python3 {baseDir}/scripts/ros2_cli.py topics find sensor_msgs/LaserScan
python3 {baseDir}/scripts/ros2_cli.py topics subscribe /scan --duration 3
```

### 4. Call a Service
```bash
python3 {baseDir}/scripts/ros2_cli.py services list
python3 {baseDir}/scripts/ros2_cli.py services call /reset '{}'
```

### 5. Send an Action
```bash
python3 {baseDir}/scripts/ros2_cli.py actions list
python3 {baseDir}/scripts/ros2_cli.py actions send /navigate_to_pose '{"pose":{"header":{"stamp":{"sec":0}},"pose":{"position":{"x":1.0,"y":0.0,"z":0.0},"orientation":{"x":0.0,"y":0.0,"z":0.0,"w":1.0}}}}'
```

### 6. Move Forward N Meters
```bash
python3 {baseDir}/scripts/ros2_cli.py topics publish-until /cmd_vel \
  '{"linear":{"x":0.2},"angular":{"z":0}}' \
  --monitor /odom --field pose.pose.position.x --delta 1.0 --timeout 30
```

### 7. Rotate N Degrees (NEW!)
```bash
# Rotate 90 degrees
python3 {baseDir}/scripts/ros2_cli.py topics publish-until /cmd_vel \
  '{"linear":{"x":0},"angular":{"z":0.5}}' \
  --monitor /odom --rotate 90 --degrees --timeout 30

# Rotate 180 degrees
python3 {baseDir}/scripts/ros2_cli.py topics publish-until /cmd_vel \
  '{"linear":{"x":0},"angular":{"z":0.5}}' \
  --monitor /odom --rotate 3.14159 --timeout 30
```

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

**Always stop the robot after movement.** The last message in any `publish-sequence` should be all zeros:
```json
{"linear":{"x":0,"y":0,"z":0},"angular":{"x":0,"y":0,"z":0}}
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
