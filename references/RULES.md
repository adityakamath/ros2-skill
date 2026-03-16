# ROS 2 Skill: Agent Rules & Decision Frameworks

This document contains the mandatory operational rules, safety protocols, and decision-making frameworks for agents using the ROS 2 Skill.

---

## Agent Behaviour Rules

These rules are absolute and apply to every request involving a ROS 2 robot.

### Rule 0 — Full introspection before every action (non-negotiable)

**Before publishing to any topic, calling any service, or sending any action goal, you MUST complete the introspection steps below. There are no exceptions, not even for "obvious" or "conventional" names.**

This rule exists because:
- The velocity topic is not always `/cmd_vel`. It may be `/base/cmd_vel`, `/robot/cmd_vel`, `/mobile_base/cmd_vel`, or anything else.
- The message type is not always `Twist`. Many robots use `TwistStamped`, and the payload structure differs.
- The odometry topic is not always `/odom`. It may be `/wheel_odom`, `/robot/odom`, `/base/odometry`, etc.
- Camera topics are not always `/camera/image_raw`. There may be multiple cameras with namespaced topics.
- TF frame names are not always `map`, `base_link`, `odom`. They vary by robot configuration.
- Controller names are not always `joint_trajectory_controller` or similar. They are defined in the robot's config.
- Convention-based guessing causes silent failures, wrong topics, and physical accidents.

**Pre-flight introspection protocol — run ALL applicable steps before acting:**

| Action type | Required introspection |
|---|---|
| Publish to a topic | 1. `topics find <msg_type>` to discover the real topic name<br>2. `topics type <discovered_topic>` to confirm the exact type<br>3. `interface proto <exact_type>` to get the default payload template |
| Call a service | 1. `services list` or `services find <srv_type>` to discover the real name<br>2. `services details <discovered_service>` to get request/response fields |
| Send an action goal | 1. `actions list` or `actions find <action_type>` to discover the real name<br>2. `actions details <discovered_action>` to get goal/result/feedback fields |
| Move a robot | Full Movement Workflow — see Rule 3 and the canonical section below |
| Read a sensor / subscribe | `topics find <msg_type>` to discover the topic; `topics type <topic>` to confirm type; never subscribe to a hardcoded name |
| Capture a camera image | `topics find sensor_msgs/msg/CompressedImage` and `topics find sensor_msgs/msg/Image` to discover available camera topics; use the result in `topics capture-image --topic <CAMERA_TOPIC>` |
| Query a TF transform | `tf list` to discover available frames; never hardcode frame names like `map`, `base_link`, `odom` |
| Switch or control a controller | `control list-controllers` to discover controller names and states; never hardcode controller names |
| Any operation involving a node | `nodes list` first; never assume a node name |
| Set / get / dump parameters | `nodes list` to discover the node name; `params list <node>` to discover parameter names |

**Parameter introspection is mandatory before any movement command.** Velocity limits can live on any node — not just nodes with "controller" in the name. Before publishing velocity:
1. Run `nodes list` to get every node currently running
2. Run `params list <NODE>` on **every single node** in the list (run in parallel batches if there are many)
3. For each node, look for any parameter whose name contains `max`, `limit`, `vel`, `speed`, or `accel` (case-insensitive). These are candidates for velocity limits.
4. Run `params get <NODE>:<param>` for every candidate found across all nodes
5. Identify the binding ceiling: the **minimum across all discovered linear limit values** and the **minimum across all discovered angular/theta limit values**
6. Cap your commanded velocity at that ceiling. If no limits are found on any node, use conservative defaults (0.2 m/s linear, 0.75 rad/s angular) and note this to the user.

**Never hardcode or assume:**
- ❌ Never use `/cmd_vel` without first discovering the velocity topic with `topics find`
- ❌ Never use `Twist` payload without first confirming the type is not `TwistStamped` via `topics type`
- ❌ Never use `/odom` without first discovering the odometry topic with `topics find`
- ❌ Never use `/camera/image_raw` or any camera topic without first discovering it with `topics find`
- ❌ Never use `map`, `base_link`, `odom`, or any TF frame name without first listing frames with `tf list`
- ❌ Never use a controller name without first listing controllers with `control list-controllers`
- ❌ Never use a node name without first listing nodes with `nodes list`
- ❌ Never use a service name without first discovering it with `services list` or `services find`
- ❌ Never use `--yaw`, `--yaw-delta`, or `--field` for rotation — the only correct flag is `--rotate N --degrees` (or `--rotate N` for radians). Use negative N for CW; `--rotate` sign and `angular.z` sign must always match.
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

**`--help` requires ROS 2 to be sourced.** Running `--help` before ROS 2 is sourced will return a JSON error instead of help text. This is not a concern in practice — ROS 2 must be sourced before any robot operation (it is a hard precondition of this skill), so `--help` will always be available during normal use. If you see a `Missing ROS 2 dependency` error from `--help`, fix the ROS 2 environment first (see Setup section and Rule 0.1).

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
| Camera topic | `topics find sensor_msgs/msg/CompressedImage` and `topics find sensor_msgs/msg/Image` |
| Odometry topic | `topics find nav_msgs/msg/Odometry` |
| Velocity command topic | `topics find geometry_msgs/msg/Twist` and `topics find geometry_msgs/msg/TwistStamped` |
| Service name | `services list` or `services find <srv_type>` |
| Service request/response fields | `services details <service>` |
| Action server name | `actions list` or `actions find <action_type>` |
| Action goal/result/feedback fields | `actions details <action>` |
| Node name | `nodes list` |
| Node's topics, services, actions | `nodes details <node>` |
| Parameter names on a node | `params list <node>` |
| Parameter value | `params get <node:param>` |
| Parameter type and constraints | `params describe <node:param>` |
| TF frame names | `tf list` |
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
- "Would you like me to use the closed-loop workflow?" — No. Use it.
- "Would you like me to auto-detect the topics and run the command?" — No. Detect and run.
- "Would you like me to fetch the --help output?" — No. Fetch it and proceed.
- "The command timed out — would you like to check odometry / retry / troubleshoot?" — No. Check odometry and report findings immediately, without asking first.

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

## Quick Decision Card

**Every user request follows this pattern:**

```
User: "do X"
Agent thinks:
  1. Is X about reading data? → Use TOPICS SUBSCRIBE
  2. Is X about movement (any kind)? → Follow the Movement Workflow (Rule 3)
  3. Is X a one-time trigger? → Use SERVICES CALL or ACTIONS SEND
  4. Is X about system info? → Use LIST commands

Agent does (for movement):
  1. Find velocity topic: topics find geometry_msgs/Twist + TwistStamped
  2. Find odometry topic: topics find nav_msgs/Odometry
  3. Distance/angle specified + odom found → publish-until (closed loop)
     Distance/angle specified + no odom → publish-sequence, notify user (open loop)
     No distance/angle → publish-sequence with stop
```

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
| "Move forward/back N meters" | Closed-loop distance → Rule 3 | Find odom → **publish-until** `--euclidean --field pose.pose.position --delta N` |
| "Rotate N degrees / turn left/right / turn N radians" | Closed-loop rotation → Rule 3 | Find odom → **publish-until** `--rotate ±N --degrees` |
| "Move arm/joint (manipulator)" | Publish JointTrajectory | Find JointTrajectory topic → publish |
| "Control gripper" | Publish GripperCommand or JointTrajectory | Find gripper topic → publish |
| "Stop the robot" | Publish zero velocity | Find Twist/TwistStamped → `topics type` to confirm → publish zeros |
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
# These commands tell you everything about the system
python3 {baseDir}/scripts/ros2_cli.py topics list             # All topics
python3 {baseDir}/scripts/ros2_cli.py services list           # All services
python3 {baseDir}/scripts/ros2_cli.py actions list            # All actions
python3 {baseDir}/scripts/ros2_cli.py nodes list              # All nodes
python3 {baseDir}/scripts/ros2_cli.py tf list                 # All TF frames
python3 {baseDir}/scripts/ros2_cli.py control list-controllers # All controllers
```

### Step 3: Search by Message Type

**To find a topic/service/action, search by what you need:**

| Need to find... | Search command... |
|-----------------|------------------|
| Velocity command topic (mobile) | `topics find geometry_msgs/msg/Twist` AND `topics find geometry_msgs/msg/TwistStamped` |
| Position/odom topic | `topics find nav_msgs/msg/Odometry` |
| Joint positions | `topics find sensor_msgs/msg/JointState` |
| Joint trajectory (arm control) | `topics find trajectory_msgs/msg/JointTrajectory` |
| LiDAR data | `topics find sensor_msgs/msg/LaserScan` |
| Camera feed | `topics find sensor_msgs/msg/Image` AND `topics find sensor_msgs/msg/CompressedImage` |
| IMU data | `topics find sensor_msgs/msg/Imu` |
| Joystick | `topics find sensor_msgs/msg/Joy` |
| Battery/power | `topics find sensor_msgs/msg/BatteryState` |
| TF frame names | `tf list` |
| TF transforms (subscribe) | Subscribe to `/tf` or `/tf_static` |
| Diagnostics | `topics diag-list` (discovers by type) |
| Clock (simulated time) | `topics find rosgraph_msgs/msg/Clock` |
| Controller names | `control list-controllers` |
| Service by type | `services find <service_type>` |
| Action by type | `actions find <action_type>` |

### Step 4: Get Message Structure

**Before publishing or calling, always confirm the type and get the structure:**

```bash
# Confirm the exact message type of a discovered topic
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

**ALWAYS check for velocity limits before publishing movement commands. Scan every node.**

```bash
# Step 1: List every running node
python3 {baseDir}/scripts/ros2_cli.py nodes list

# Step 2: Dump all parameters from every node
python3 {baseDir}/scripts/ros2_cli.py params list <NODE_1>
python3 {baseDir}/scripts/ros2_cli.py params list <NODE_2>

# Step 3: Compute binding ceiling
# linear_ceiling  = min of all discovered linear limit values
# angular_ceiling = min of all discovered angular/theta limit values
# velocity = min(requested_velocity, ceiling)
```

**If no limits are found on any node:** use conservative defaults (0.2 m/s linear, 0.75 rad/s angular) and tell the user.

---

## Error Recovery Protocols

### Subscribe Timeouts

| Error | Recovery |
|-------|----------|
| `Timeout waiting for message` | 1. Check `topics details <topic>` to verify publisher exists<br>2. Try a different topic if multiple exist<br>3. Increase `--duration` or `--timeout` |

### Publish Failures

| Error | Recovery |
|-------|----------|
| `Could not load message type` | 1. Verify type: `topics type <topic>`<br>2. Ensure ROS workspace is built |

### Service/Action Failures

| Error | Recovery |
|-------|----------|
| Service not found | 1. Verify service exists: `services list`<br>2. Check service type: `services type <service>` |
| Action goal rejected | 1. Check action details for goal requirements<br>2. Verify robot is in correct state |

### Movement / publish-until Failures

**A `publish-until` timeout is a robot or sensor issue — not a missing command.** `publish-until` exists and works; the timeout means the odometry delta was never reached. Do not conclude the command is unavailable.

| Error | Recovery |
|-------|----------|
| `publish-until` times out without reaching target | 1. **Immediately send `estop`** — do not wait, do not retry, do not ask the user first<br>2. Subscribe to `<ODOM_TOPIC>` — check if odom is publishing and values are changing<br>3. Run `topics hz <ODOM_TOPIC>` — if rate < 5 Hz, odom is stale (robot may not have moved)<br>4. Run `control list-controllers` to verify the velocity controller is active<br>5. Report to user: actual distance covered, odom status, controller state |
| Odometry not updating during motion | 1. Immediately send zero-velocity: `estop`<br>2. Check `topics details <ODOM_TOPIC>` for publisher count and `topics hz <ODOM_TOPIC>` for rate<br>3. Do NOT continue publishing if odometry is stale — it is a runaway risk |
| `Could not detect message type` for topic | The topic exists but has no publisher yet. Check `topics details <topic>` for publisher count. Wait for the publisher to connect, or pass `--msg-type` explicitly. |


---

## Action Preemption — `actions cancel` vs `estop`

Use this decision table whenever an in-flight action goal needs to be stopped:

| Situation | Action | Reason |
|-----------|--------|--------|
| Goal is running but user wants to abort gracefully | `actions cancel <action>` | Sends a cancel request to the action server; the server winds down cleanly |
| Goal is running and the robot is moving unsafely / not stopping | `estop` first, then `actions cancel <action>` | `estop` publishes zero velocity immediately; cancel cleans up the goal state |
| Goal was rejected or timed out (robot not moving) | `actions cancel <action>` | No motion risk; cancel clears the goal state |
| Action server crashed / no longer responding | `estop` | No action server to receive cancel; stop the actuators directly |
| Goal completed but robot is still drifting / coasting | `estop` | Motion is no longer governed by the goal; velocity command is needed |

**Rule:** If in doubt, send `estop` first — it is always safe. Then send `actions cancel` to clean up goal state. Never skip `estop` when the robot is or may be moving.

---

## Launch Commands & Workflow

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

### Local Workspace Sourcing

**System ROS is assumed to be already sourced** (via systemd service or manually). The skill automatically sources any local workspace on top of system ROS.

**Search order:**
1. `ROS2_LOCAL_WS` environment variable
2. `~/ros2_ws`, `~/colcon_ws`, `~/dev_ws`, `~/workspace`, `~/ros2`

**Behavior:**
| Scenario | Behavior |
|----------|----------|
| Workspace found + built | Source automatically, run silently |
| Workspace found + NOT built | Warn user, run without sourcing |
| Workspace NOT found | Continue without sourcing (system ROS only) |


---

## Handle Multiple Same-Type Topics

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

## Setup & Environment

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
