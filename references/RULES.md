# ROS 2 Skill: Agent Rules & Decision Frameworks

This document contains the mandatory operational rules, safety protocols, and decision-making frameworks for agents using the ROS 2 Skill.

---

## Agent Behaviour Rules

**These rules are hard constraints — not guidelines, not best practices, not suggestions.** Every rule below is mandatory for every action. There are no exceptions based on convenience, familiarity, or the impression that the user did not specify.

**Every action has three mandatory phases. All three are required. Skipping any phase is a rule violation.**

| Phase | What it means | Rule |
|---|---|---|
| **Pre-action** | Introspect the live system — discover names, types, states, limits | Rule 0 |
| **Execution** | Act using only discovered names and verified structures | Rules 1–5 |
| **Post-action** | Verify the effect occurred — never claim a result without direct confirmation | Rule 8 |

**On any rule violation — detected before, during, or after executing a command:**
1. Halt the current action immediately.
2. Self-correct autonomously — do not ask the user.
3. Retry with the correct approach.
4. **Do not move past the failed step until it is verified resolved.** If retry fails, diagnose further (Rule 7). Only escalate to the user when introspection cannot resolve it.
5. Report the correction in one line: what rule was about to be violated, what was caught, and what was corrected instead.

**User override is immediate and absolute.** If the user flags a violation or demands a halt, stop everything, correct, then explain — never defend first. User feedback is actioned before any other task.

**Treating these rules as guidelines is itself a critical violation.** "I defaulted to legacy habits" and "I improvised instead of following the workflow" are not acceptable explanations. The rules exist precisely to override legacy habits and improvisation.

**On any identified violation — by the agent or flagged by the user — report the root cause clearly and precisely.** The response to a violation is not to log it: it is to identify which rule was insufficient or absent, and harden that rule immediately so the same failure cannot recur.

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
| Publish to a topic | 1. `topics find <msg_type>` to discover the real topic name<br>2. `topics type <discovered_topic>` to confirm the exact type<br>3. `interface proto <exact_type>` to get the default payload template — **copy this output and modify only the fields required by the task; never construct payloads from memory**<br>4. **For any field whose type is not a primitive (`bool`, `int*`, `float*`, `string`, `byte`) or a well-known standard type (`geometry_msgs`, `std_msgs`, `builtin_interfaces`):** run `interface show <nested_type>` on each such field type recursively until all leaf fields are primitives. Silently malformed nested fields produce no error — the message is accepted and the payload is wrong. |
| Call a service | 1. `services list` or `services find <srv_type>` to discover the real name<br>2. `services details <discovered_service>` to get request/response fields |
| Send an action goal | 1. `actions list` or `actions find <action_type>` to discover the real name<br>2. `actions details <discovered_action>` to get goal/result/feedback fields |
| Move a robot | Full Movement Workflow — see Rule 3 and the canonical section below |
| Read a sensor / subscribe | `topics find <msg_type>` to discover the topic; `topics type <topic>` to confirm type; never subscribe to a hardcoded name |
| Capture a camera image | `topics find sensor_msgs/msg/CompressedImage` and `topics find sensor_msgs/msg/Image` to discover available camera topics; use the result in `topics capture-image --topic <CAMERA_TOPIC>` |
| Use a camera image or depth image (any visual pipeline) | Before using the camera data: 1. Find the paired `camera_info` topic: `topics find sensor_msgs/msg/CameraInfo` — the result will share a namespace with the image topic (e.g., `/camera/camera_info` pairs with `/camera/image_raw`). 2. Subscribe to confirm calibration is present: `topics subscribe <CAMERA_INFO_TOPIC> --max-messages 1 --timeout 2` — verify `K` (intrinsic matrix) is non-zero. 3. Read the `header.frame_id` from the `camera_info` message; confirm it is present in `tf list`. **A camera with no `camera_info` publisher is uncalibrated. A camera whose `frame_id` is absent from TF will produce wrong spatial results.** Both conditions must be satisfied before using the camera for any spatial or metric task (depth estimation, object localisation, point cloud processing). |
| Use a sensor topic whose data will be interpreted spatially (camera, LiDAR, IMU, depth, GPS, sonar) | Before consuming the data: 1. Subscribe to the topic for 1 message to read the `header.frame_id`. 2. Run `tf list` to confirm that `frame_id` is present in the TF tree. 3. Run `tf echo <SENSOR_FRAME> <BASE_FRAME> --duration 1` (where `<BASE_FRAME>` is the robot's base frame from `tf list`) to confirm the transform is actively updating and not stale. **Never use sensor data whose frame is absent from TF or has not been updated recently** — spatial computations on stale or missing transforms produce wrong results with no error. |
| Query a TF transform | `tf list` to discover available frames; never hardcode frame names like `map`, `base_link`, `odom` |
| Switch or load a controller | 1. `control list-controllers` to discover controller names and states (never hardcode).<br>2. `control list-hardware-components` to confirm the hardware component is in `active` state.<br>3. `control list-hardware-interfaces` to confirm the relevant hardware interfaces are `available/active`.<br>**Never load, switch, or configure a controller while the hardware component is `inactive` or `unavailable`** — the controller will load without error but will silently discard all commands. Always use `--strictness STRICT` for `switch-controllers` unless explicitly instructed otherwise. |
| Any operation involving a node | `nodes list` first; never assume a node name |
| Set a parameter | 1. `nodes list` to discover the node name<br>2. `params list <node>` to discover the parameter name (never assume it)<br>3. `params describe <node:param>` to confirm type (int/float/bool/string), valid range, and read-only status — **never set a parameter without this; wrong type silently truncates or is rejected**<br>4. `params set <node:param> <value>` with value matching the confirmed type; verify with `params get` after (Rule 8) |
| Get / list / dump parameters | `nodes list` to discover the node name; `params list <node>` to discover parameter names |
| Load parameters from a YAML file (`params load` or `--params-file` in launch) | 1. `nodes list` to confirm the target node is running.<br>2. `params list <node>` to get the current parameter names on that node.<br>3. Compare YAML file keys against the discovered names — **any YAML key that does not match a declared parameter is silently ignored; no error is raised.** Verify every key you intend to set is present in `params list` output before loading.<br>4. For each YAML key you will set, `params describe <node:param>` to confirm type — a YAML string loaded into an integer parameter silently fails or truncates.<br>5. After loading: `params get <node:param>` on each key to verify values were applied (Rule 8). |

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
- ❌ Never construct a message payload from memory — always use `interface proto <type>` output as the starting template and modify only the fields required by the task
- ❌ Never revert to hardcoded or legacy behaviors after a robust introspection-driven workflow is established — even if the hardcoded name "usually works" on this specific robot
- ❌ Never bypass, skip, or abbreviate safety checks even if the user explicitly requests it — safety rules are not negotiable
- ❌ Never read or report odometry for position, orientation, or yaw while the robot is moving or decelerating — wait until confirmed stationary (velocity ≈ 0 on all axes) then subscribe fresh (see Rule 8 two-phase protocol)

**Introspection commands return discovered names. Use those names — not the ones you expect.**

### Rule 0.1 — Session-start checks (run once per session, before any task)

**Before executing any user command in a new session, run these checks.** They take seconds and catch the most common causes of silent failure.

**Step 0 — Environment and domain sanity (containerised / multi-system environments):**
Before anything else, verify the ROS 2 environment is coherent:
- If `nodes list` or `topics list` returns an unexpectedly large or irrelevant set of names, the `ROS_DOMAIN_ID` may be colliding with another system. Default domain ID is `0`. In container-alongside-host deployments, both sides default to `0` and see each other's nodes — this is the most common container misconfiguration. Verify with the host before proceeding.
- If the doctor command (Step 1) reports "ROS daemon not running" or "no nodes found" despite the robot stack being up, the daemon may need restarting: run `daemon status` to confirm, then `daemon start` to restart; verify with `daemon status` after. Re-run doctor before continuing.
- In containerised environments, `ROS_LOCALHOST_ONLY` being set to `1` isolates all communication to localhost — cross-container and cross-host topics will be invisible. If discovery returns no nodes despite the stack being up, check whether this variable is set in the environment.

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

**If a command, subcommand, or flag is not present in COMMANDS.md or `--help` output, it does not exist. Treat it as a hallucination. Halt. Identify the correct command. Retry. The hallucinated command must never be sent.**

The failure mode to avoid: inventing a subcommand like `launch start` or a flag like `--yaw-delta` because it sounds plausible, sending it, failing, then asking the user for help. That is the worst possible outcome — the error was self-inflicted and the user had nothing to do with it.

**The verification chain:**
1. **Check this skill first.** The full command reference is in [references/COMMANDS.md](references/COMMANDS.md). If a subcommand, flag, or argument is not listed there, it does not exist.
2. **If still unsure, run `--help` on the top-level command to list valid subcommands, or on the exact subcommand to list valid flags.** This is mandatory before any call you are not certain about.
   ```bash
   python3 {baseDir}/scripts/ros2_cli.py launch --help         # lists: new, list, ls, kill, restart, foxglove
   python3 {baseDir}/scripts/ros2_cli.py topics publish-until --help
   python3 {baseDir}/scripts/ros2_cli.py actions send --help
   # etc. — use the exact command/subcommand you are about to call
   ```
3. **If still stuck after checking both, ask the user.** This is the only acceptable reason to ask — not because you assumed something and it failed.

**If the CLI returns "subcommand not found" or "unknown command":** you used a hallucinated subcommand. Do **not** ask the user what to do. Run `--help` on the parent command immediately, identify the correct subcommand from the output, and retry — all without reporting back to the user until you have a result.

**`--help` requires ROS 2 to be sourced.** Running `--help` before ROS 2 is sourced will return a JSON error instead of help text. This is not a concern in practice — ROS 2 must be sourced before any robot operation (it is a hard precondition of this skill), so `--help` will always be available during normal use. If you see a `Missing ROS 2 dependency` error from `--help`, fix the ROS 2 environment first (see Setup section and Rule 0.1).

**Never:**
- Invent a subcommand (e.g., `launch start`) or flag and try it, then report the failure to the user
- Assume a subcommand exists because it would be logical or convenient (`start`, `run`, `execute` are not subcommands of `launch`)
- Ask the user to resolve an error you caused by guessing
- Report "the subcommand does not exist — would you like me to retry?" — just retry with the correct subcommand

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
| Nested custom type fields within a message | `interface show <nested_type>` — run recursively on each non-primitive, non-standard field type; repeat until all leaf fields are primitives |

**Only ask the user if**:
1. The discovery command returns an empty result or an error, **and**
2. There is genuinely no other way to determine the information from the live system.

### Rule 2 — ros2-skill is the only interface; never use the `ros2` CLI directly

**All ROS 2 operations must go through `ros2_cli.py`. Never call `ros2 topic list`, `ros2 node list`, `ros2 service call`, or any other `ros2 <command>` CLI directly.**

**Never run any `ros2_*.py` file other than `ros2_cli.py` directly.** Every other file matching `ros2_*.py` in `scripts/` is a submodule — running one directly will print an error and exit without performing any ROS operation. The only valid entry point is `ros2_cli.py`:
```bash
python3 {baseDir}/scripts/ros2_cli.py <command> [subcommand] [args]
```

This rule exists because:
- The `ros2` CLI returns unstructured human-readable text that agents misparse.
- `ros2_cli.py` returns structured JSON with consistent fields — no parsing errors, no format drift.
- Using `ros2` directly bypasses retry logic, timeout handling, and safety checks built into this skill.

**The hierarchy:**
1. **ros2-skill first** — use `ros2_cli.py` for everything. It covers the full ROS 2 operation surface.
2. **ros2 CLI as last resort only** — if and only if no ros2-skill command exists for the required operation AND the operation cannot be approximated using existing skill commands. This is rare. Document why ros2-skill was insufficient.
3. **Never use `ros2` CLI for anything ros2-skill can do** — even if it seems simpler or faster.

**Never tell the user you don't know how to do something** without first checking ros2-skill for a matching command. Common operations that agents sometimes use `ros2 CLI` for unnecessarily:

| Task | ❌ Do NOT use | ✅ Use instead |
|---|---|---|
| List all topics | `ros2 topic list` | `topics list` |
| Get topic type | `ros2 topic info /topic` | `topics type <topic>` |
| Subscribe to topic | `ros2 topic echo /topic` | `topics subscribe <topic>` |
| List nodes | `ros2 node list` | `nodes list` |
| Get node info | `ros2 node info /node` | `nodes details <node>` |
| List services | `ros2 service list` | `services list` |
| Call a service | `ros2 service call /srv ...` | `services call <service> <json>` |
| List actions | `ros2 action list` | `actions list` |
| Send action goal | `ros2 action send_goal ...` | `actions send <action> <json>` |
| Get parameter | `ros2 param get /node param` | `params get <node:param>` |
| Set parameter | `ros2 param set /node param val` | `params set <node:param> <value>` |
| List parameters | `ros2 param list /node` | `params list <node>` |
| List TF frames | `ros2 run tf2_tools view_frames` | `tf list` |
| Get transform | `ros2 run tf2_ros tf2_echo ...` | `tf lookup <source> <target>` |
| List controllers | `ros2 control list_controllers` | `control list-controllers` |
| Check health | `ros2 doctor` | `doctor` |
| Capture image | custom scripts | `topics capture-image --topic <topic>` |

If you genuinely cannot find a matching command after checking both the quick reference and the COMMANDS.md reference, **say so clearly and explain what you checked** — do not silently fall back to the `ros2` CLI without logging it.

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

**If a topic is found:** record it as `ODOM_TOPIC`. Then immediately verify it is live and publishing at a usable rate:
```bash
topics hz <ODOM_TOPIC> --duration 2
```
- **Rate ≥ 5 Hz:** odom is healthy — proceed with closed-loop.
- **Rate 1–4 Hz:** odom is too slow for reliable closed-loop. Warn the user: *"Odometry is publishing at <N> Hz — too slow for accurate closed-loop control. Proceeding open-loop."* Use `publish-sequence`.
- **Rate 0 Hz / timeout:** odom topic exists but nothing is publishing on it. Treat as absent (see below).

**If no odom topic found (or rate = 0 Hz):** attempt a TF-based position fallback before declaring "no position available":
```bash
tf list    # discover actual frame names (never hardcode)
```
If `<ODOM_FRAME>` and `<BASE_LINK_FRAME>` exist in the TF tree, they can be used for position queries via `tf lookup <ODOM_FRAME> <BASE_LINK_FRAME>` — but **not** for `publish-until` closed-loop monitoring (TF lookup is one-shot, not streamed). Fall back to `publish-sequence` and note in the report: *"No odometry topic found. TF is available for position queries but cannot be used for closed-loop motion control."*

**Step 3 — Choose the execution method**

**Closed-loop motion (`publish-until` with odometry monitoring) is always the required method when odometry is available.** Open-loop (`publish-sequence`) is a fallback of last resort, used only when odometry is confirmed unavailable. Never choose open-loop when an active `<ODOM_TOPIC>` has been found — even if the task "seems simple."

| Situation | Method |
|---|---|
| Distance or angle specified **and** odometry found | `publish-until` with `--monitor <ODOM_TOPIC>` — **closed loop, mandatory**; stops precisely on sensor feedback |
| Distance or angle specified **and** no odometry | `publish-sequence` with calculated duration — **open loop, last resort only**. Notify user: *"No odometry found. Running open-loop. Distance/angle accuracy is not guaranteed."* |
| No distance or angle specified (open-ended movement) | `publish-sequence` with a stop command as the final message |

**Step 4 — Execute using only discovered names**

Use `VEL_TOPIC` and `ODOM_TOPIC` from Steps 1–2. Never substitute `/cmd_vel`, `/odom`, or any other assumed name.

Distance commands:
```bash
topics publish-until <VEL_TOPIC> '<payload>' --monitor <ODOM_TOPIC> --field pose.pose.position.x --delta <N> --timeout <TIMEOUT>
```
Angle/rotation commands — **always use `--rotate`, never `--field` or `--yaw`**:
```bash
# CCW (left): positive --rotate + positive angular.z
topics publish-until <VEL_TOPIC> '<payload>' --monitor <ODOM_TOPIC> --rotate <+N> --degrees --timeout <TIMEOUT>
# CW (right):  negative --rotate + negative angular.z
topics publish-until <VEL_TOPIC> '<payload>' --monitor <ODOM_TOPIC> --rotate <-N> --degrees --timeout <TIMEOUT>
```
`--rotate` sign = direction. Positive = CCW. Negative = CW. `angular.z` sign must always match `--rotate` sign — mismatched signs cause timeout. There is no `--yaw` flag. Do not attempt to monitor orientation fields manually.

**`<TIMEOUT>` is never hardcoded — always calculate it:**

| Command type | Formula | Minimum |
|---|---|---|
| Linear distance `N` metres at `v` m/s | `ceil((N / v) * 2.0)` seconds | 15 s |
| Rotation `θ` degrees at `ω` rad/s | `ceil(((θ * π / 180) / ω) * 2.0)` seconds | 15 s |
| Open-ended `publish-sequence` | Use explicit durations per message — no global timeout needed | — |

Example: moving 3 m at 0.2 m/s → `ceil((3 / 0.2) * 2.0) = ceil(30) = 30 s`. Moving 0.5 m at 0.2 m/s → `ceil(5) = 15 s` (minimum applies). **Never use 30 s as a default — always calculate for the actual task.**
Open-ended or fallback (stop is always the last message):
```bash
topics publish-sequence <VEL_TOPIC> '[<move_payload>, <zero_payload>]' '[<duration>, 0.5]'
```

### Rule 4 — Infer the goal, resolve the details
When a user asks to do something, **infer what they want at the goal level, then resolve all concrete details (topic names, types, field paths) from the live system**.

Examples:
- "Take a picture" → find compressed image topics (`topics find sensor_msgs/msg/CompressedImage`), capture from the first active result
- "Move the robot forward" / "drive ahead" / "go forward 1 m" → motion vocabulary detected → full Rule 3 workflow: find vel topic + find odom → if odom ≥ 5 Hz and distance given, use `publish-until`; if no odom, use `publish-sequence` with duration; if no distance, open-ended `publish-sequence`
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

**There are exactly three conditions that permit asking the user:**

1. **Genuine ambiguity that cannot be resolved by introspection:**
   - Multiple packages or launch files match and you cannot determine which one the user means
   - A required argument has no match in `--show-args` and no reasonable fuzzy match exists

2. **The action is destructive, irreversible, or safety-critical:**
   - Deleting data (preset files, logs) when the target is ambiguous
   - Stopping a controller that may be load-bearing for other systems
   - Any action the user's message did not clearly and specifically authorise

3. **The motion goal contains an ambiguous quantity that no safe default can resolve:**
   - Words like "a bit", "slightly", "a little", "nearby", "close to", "far", "some distance" specify no actionable target.
   - In such cases: ask once for the specific value needed (e.g., *"How many meters forward?"* or *"How many degrees to turn?"*). Never substitute a guessed or assumed value. Never apply a 'reasonable default distance' — motion without a verified target is a safety risk.
   - **Exception:** "Move forward" or "drive forward" with no qualifier is an open-ended command — execute with `publish-sequence` per Rule 3. The ambiguity rule applies only when a vague quantity word is present.

If none of these conditions apply: **just do it.**

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
- "The subcommand does not exist — would you like me to retry with the correct one?" — No. Run `--help`, find the correct subcommand, retry immediately.
- "I used the wrong subcommand — shall I try again?" — No. Try again.

### Rule 6 — Minimal reporting by default

**Keep output minimal. The user wants results, not narration.**

| Situation | What to report |
|---|---|
| Operation succeeded | One line: what was done and the key outcome. Example: "Done. Moved 1.02 m forward." |
| Movement completed | Start position (from pre-motion baseline, Rule 9), end position (from post-motion odom subscribe, Rule 8), actual distance/angle travelled — **all values must come from fresh `<ODOM_TOPIC>` subscribe calls, never from calculation or estimation**. Include anomalies (timeout, open-loop fallback). |
| No suitable topic/source found | Clear error with what was searched and what to try next |
| Safety condition triggered | Immediate notification with what happened and what was sent (stop command) |
| Operation failed | Error message with cause and recovery suggestion |

**Never report by default:**
- The topic name selected, unless it is ambiguous or unexpected
- The message type discovered
- Intermediate introspection results
- Step-by-step narration of what you are about to do

**Always report (even in minimal mode):**
- Self-corrections: if a rule violation was caught mid-execution, report it in one line — what was about to be violated, what was caught, and what was done instead. Example: *"Caught: was about to use `launch start` (hallucinated subcommand). Corrected to `launch new`. Retrying."*

**Report everything (verbose mode) only when the user explicitly asks** — e.g. "show me what topics you found", "give me the full details", "what type did you use?"

### Rule 7 — Diagnose failures immediately; never ask the user to diagnose

**On any CLI error — wrong subcommand, unknown flag, invalid argument, type mismatch, timeout, unexpected output — the immediate and automatic response is: introspect (run `--help` or check COMMANDS.md), correct, retry. Never report a CLI error to the user before attempting self-correction. Never ask for permission to retry.**

On any failure (command error, timeout, unexpected output, wrong result):

1. **Immediately introspect** — run CLI tools to determine the cause before reporting to the user. Do not ask the user what happened.
2. **Report succinctly** — what was tried, what the error was, what the introspection revealed. No speculation, no options list.
3. **Correct and retry if possible** — if the cause is a wrong topic name, wrong type, missing publisher, or inactive controller, fix it and retry without asking.
4. **Escalate only when genuinely stuck** — if introspection cannot resolve the issue (hardware fault, missing node stack, environment problem), report exactly what was found and suggest one specific next step.

**Never:**
- Ask the user to interpret an error message that can be checked with the CLI
- Present a menu of options ("would you like to check odometry / retry / troubleshoot?") — pick the right action and do it
- Silently ignore an error or continue as if it did not happen
- Speculate about the cause without first running the diagnostic commands
- Ask the user for permission to retry after a self-inflicted error (wrong subcommand, wrong flag) — self-correct and retry immediately

**Pre-escalation diagnostic — elevate log level before asking the user:**
If the cause of a failure is not apparent from standard introspection (wrong output, unexpected behaviour, silent failure, inconsistent results), elevate the log level for the relevant node before escalating to the user. This often reveals the root cause without user intervention:
```bash
# Step 1: find the SetLoggerLevel service for the relevant node
services find rcl_interfaces/srv/SetLoggerLevel

# Step 2: set log level to DEBUG (level 10 = DEBUG, 20 = INFO, 30 = WARN, 40 = ERROR)
services call <SET_LOGGER_LEVEL_SERVICE> '{"logger_name": "", "level": 10}'

# Step 3: retry the failing operation and observe the additional output

# Step 4: reset to INFO when done
services call <SET_LOGGER_LEVEL_SERVICE> '{"logger_name": "", "level": 20}'
```
Use `logger_name: ""` to affect the root logger, or specify the node's fully qualified logger name (e.g., `"my_controller"`) to narrow the scope. **Only escalate to the user if debug-level output still does not identify the cause.**

**Motion-error lockout — mandatory after any motion timeout or motion failure:**
After any `publish-until` or `publish-sequence` timeout, error, or unexpected stop, the robot is in an unknown physical state. The following steps are non-negotiable before any further motion command is issued:
1. Send `estop`.
2. Subscribe `<ODOM_TOPIC> --max-messages 1` — record actual pose from sensor.
3. Diagnose root cause (odom rate, controller state, etc.).
4. Report cause and actual pose to the user.

**Do not retry, re-issue, or attempt any new motion command until all four steps are complete.** Blindly retrying a timed-out motion command without diagnosis is a Rule 7 critical error.

**Example — publish-until timeout:**
- ❌ *"The command timed out. Would you like to check odometry, retry with a longer timeout, or troubleshoot the controller?"*
- ✅ Run `estop`. Run `topics hz <ODOM_TOPIC>`. Run `control list-controllers`. Report: *"Timed out after 60 s. Odom rate: 0 Hz (no publisher active). No velocity controller running. Robot did not move. Bring up the controller stack and verify odom is publishing before retrying."*

**Example — wrong subcommand:**
- ❌ *"The launch command does not support the `start` subcommand. Available subcommands are: new, list, ls, kill, restart, foxglove. Would you like me to retry using the correct subcommands?"*
- ✅ Recognise `launch start` is invalid. Run `launch new <package> <file>` immediately. Report the result only.

### Rule 8 — Verify the effect; never trust exit codes alone

**A command returning without error means the request was delivered — it does not mean the effect occurred.** Always verify the outcome with a follow-up introspection call before reporting success.

| Operation | Verification |
|---|---|
| `params set <node:param> <val>` | **Pre-set:** run `params describe <node:param>` to confirm type, range, and that the param is not read-only. Then `params set`. Then run `params get <node:param>` — confirm returned value matches what was set. |
| `control switch-controllers` | Run `control list-controllers` — confirm new controller is `active`, old is `inactive` |
| `lifecycle set <node> <transition>` | Run `lifecycle get <node>` — confirm the node reached the expected state |
| `actions send <action> <json>` | **Pass a timeout flag** (check COMMANDS.md or `actions send --help` for the flag name). On return: check `status` — `SUCCEEDED` = completed; `FAILED` or `CANCELED` = treat as failure, diagnose per Rule 7. **If the command hangs past the timeout:** immediately send `actions cancel <goal_id>` (use the goal ID returned at submission), then diagnose why the action server did not respond. Never leave an action goal orphaned. Monitor feedback messages during execution: if a navigation action sends feedback with no progress for > 10 s, treat as stuck, cancel, and diagnose. |
| `control load-controller` / `control switch-controllers` / `control configure-controller` | 1. Run `control list-controllers` — confirm controller reached the expected state (`active` or `inactive`). 2. Run `control list-hardware-components` — confirm the hardware component is still `active`. If the hardware component is `inactive` after a controller operation, no velocity commands will be executed — this is the most common silent failure in ros2_control. |
| `topics publish` (single shot, state-change intent) | Run `topics subscribe <topic> --max-messages 1 --timeout 3` or `topics hz <topic>` to confirm messages are being received |
| `estop` (emergency stop) | After sending `estop`, verify it took effect: subscribe `<ODOM_TOPIC> --max-messages 1 --timeout 3` and check `twist.twist.linear.x`, `.y`, and `twist.twist.angular.z` are all < 0.01. If velocity is still non-zero after 3 s: the estop was not received (controller may be down or topic is wrong). **Critical failure — do not proceed with any command.** Report immediately: *"Estop sent but velocity still non-zero — robot may not have stopped. Controller or velocity topic may be offline."* |
| Movement completion (position / orientation reporting) | **Three-phase protocol — all steps required:** (1) Confirm odom is still live: run `topics hz <ODOM_TOPIC> --duration 1` — if rate < 5 Hz, flag as degraded before reporting. (2) Confirm the robot is stationary: subscribe `<ODOM_TOPIC> --max-messages 1`; check `twist.twist.linear.x`, `twist.twist.linear.y`, and `twist.twist.angular.z` are all < 0.01 m/s or rad/s. If any exceed 0.01, wait 0.5 s and repeat. (3) Once confirmed stationary, subscribe `<ODOM_TOPIC> --max-messages 1` and report position, orientation, or yaw from **this** reading only. **Covariance check:** if `pose.covariance[0]` (x-variance) > 0.1 m² or `pose.covariance[35]` (yaw-variance) > 0.1 rad², qualify the report: *"Pose reported but covariance is high — estimate may be unreliable."* |
| Motion timeout (`publish-until` or `publish-sequence` did not complete) | 1. **Immediately send `estop`** — verify it took effect (see estop row above). 2. Subscribe `<ODOM_TOPIC> --max-messages 1` — record actual final pose (sensor truth, not estimated). 3. Diagnose: run `topics hz <ODOM_TOPIC>` and `control list-controllers`. 4. Report: actual final position/orientation, distance covered vs. target, diagnosed cause. 5. **Do not send any further motion commands until root cause is identified and reported.** |
| Motion error (command error, type mismatch, unexpected stop, any failure) | 1. Send `estop` and verify it took effect. 2. Subscribe `<ODOM_TOPIC> --max-messages 1` — record actual pose at time of failure. 3. Diagnose per Rule 7. 4. Report: actual pose from sensor, error description, root cause. **Do not proceed with any motion until root cause is resolved.** |

**Reading odometry while the robot is moving produces wrong results.** The robot may still be decelerating or coasting when a motion command returns. The only correct time to read odometry for position or orientation reporting is after the robot is confirmed physically stationary — `twist.twist.linear.x`, `.y`, and `twist.twist.angular.z` are all ≈ 0. Post-motion odometry is a two-step operation: first confirm stationary, then subscribe and report.

Never report yaw, position, or distance from any reading taken while motion was ongoing — including the final message delivered just before the command returned.

**Never use the words "Done", "Succeeded", "Completed", "Applied", or any equivalent without first running the verification step for that operation type.** A zero-error CLI response is not verification — it is only evidence that the request was delivered.

If verification reveals the effect did not occur (param unchanged, controller not switched, lifecycle state unchanged): diagnose immediately per Rule 7, correct, retry, and verify again. Do not move on until the verification passes.

**Exception:** For `publish-until` and `publish-sequence`, the command's own stop condition or duration is the execution criterion — do not add a separate `topics hz` check **during** an active motion sequence. This exception applies only to checking delivery-rate mid-motion. It does **not** exempt the post-motion two-phase odometry read required for position/orientation reporting — that step is always mandatory after motion completes.

### Rule 9 — Pre-motion check: confirm the robot is stationary before commanding movement

**Before issuing any motion command**, perform all three pre-motion checks. Run steps A and B in parallel, then evaluate:

**A — Odom health and pose capture** (subscribe once):
```bash
topics subscribe <ODOM_TOPIC> --max-messages 1 --timeout 2
```
From this single message:
- **Record starting pose (baseline):** save `pose.pose.position.x`, `pose.pose.position.y`, and the orientation quaternion. This is the pre-motion reference for post-motion comparison and failure reports. Never use a cached pose — it must be fresh.
- **Check velocity:** all three of `twist.twist.linear.x`, `twist.twist.linear.y`, and `twist.twist.angular.z` must be **< 0.01** (m/s or rad/s). Values below this threshold are considered stationary. Values ≥ 0.01 mean the robot is moving or drifting.

**B — Node presence check** (run in parallel with A):
```bash
nodes list
```
Confirm the velocity controller node is still present (the same node discovered during Rule 0 pre-flight). If it has disappeared since pre-flight, the robot stack has changed — halt, run Rule 0.1 health check again, and do not proceed until the stack is verified.

**C — Odom frequency check** (run after A returns):
```bash
topics hz <ODOM_TOPIC> --duration 2
```
- Rate ≥ 5 Hz → proceed with closed-loop.
- Rate 1–4 Hz → warn user, fall back to open-loop.
- Rate 0 Hz → odom is stale despite the topic existing. Treat as absent. Fall back to open-loop.

**Evaluate results:**
- **Velocity ≥ 0.01 on any axis:** send `estop` immediately, verify it took effect (Rule 8 estop row), wait 0.5 s, re-read odom, then proceed. Report: *"Robot was already moving. Stopped before issuing new command."*
- **Subscribe timeout or rate = 0 Hz:** do NOT proceed with closed-loop. Fall back to `publish-sequence`. Notify user: *"Odometry not available. Running open-loop — distance accuracy not guaranteed."* No starting pose can be recorded; note in report.
- **Controller node absent:** halt. Re-run Rule 0.1 health check. Do not proceed until the velocity controller is confirmed running.

**Never issue a new motion command on top of an existing one without stopping first.** Overlapping velocity commands cause unpredictable trajectories and runaway motion.

### Rule 10 — Empty discovery: broaden the search, never guess

When `topics find <type>` returns an empty list:

1. **Do not guess a topic name.**
2. **Do not fall back to a hardcoded name** (Rule 0).
3. **Broaden the search immediately and in parallel:**
   ```bash
   topics list    # All active topics — scan for anything plausibly relevant
   nodes list     # Are the nodes that publish this type even running?
   ```
4. **If nodes are missing:** the robot stack may not be up. Run `doctor` and report what is absent.
5. **If nodes are running but topic is absent:** the publisher may not have started yet, or the topic may be differently typed than expected. Check `nodes details <candidate_node>` for its published topics.
6. **Report exactly:** what was searched, what `topics list` and `nodes list` returned, and one specific next step. Never present a menu. Never speculate.

**Terminal case — complete stack failure:** If `topics list` returns empty **and** `nodes list` returns empty:
- The entire robot stack is not running. Do not attempt any robot operations.
- Run `doctor` to confirm and capture the failure report.
- Report: *"No nodes or topics found. The robot stack appears to be offline. Bring up the robot bringup before retrying."*
- This is the only situation where further introspection is pointless — escalate immediately with the doctor output.

**This rule applies equally to `services find`, `actions find`, and any other type-based search.** Empty results are information, not permission to guess.

### Rule 11 — Use discovered names verbatim; never mutate them

Topic names, service names, action names, node names, and TF frame names returned by discovery commands must be used **exactly as returned** — character for character, including leading slashes, namespace prefixes, and suffixes.

**Never:**
- Strip a namespace prefix (e.g., convert `/robot_1/cmd_vel` → `/cmd_vel`)
- Add a namespace that was not in the discovery result
- Normalise, shorten, or "clean up" a discovered name
- Assume that a short name and a namespaced name refer to the same entity

**When multiple topics of the same type exist**, apply this selection algorithm:

1. **Match against user context keywords first:**

   | User mentioned... | Prefer topics containing... |
   |---|---|
   | "front camera", "forward camera" | `front`, `forward`, `rgb`, `color` |
   | "rear camera", "back camera" | `rear`, `back` |
   | "arm", "manipulator" | `arm`, `manip`, `wrist`, `elbow` |
   | "robot 1", "robot_1", specific robot name | that robot's namespace prefix |
   | "primary", "main", "base" LiDAR/sensor | `front`, `base`, `main`, `primary` |
   | No context clue | use the **first result** |

2. **If context resolves to exactly one candidate:** use it, state which was selected (Rule 6).
3. **If context is still ambiguous (e.g., two `front` cameras):** use the first matching result, state which was selected. Do not ask the user (Rule 5).

**Multi-robot network exception — when namespaces suggest different robots:**
If discovery returns results from multiple distinct robot namespaces (e.g., `/robot_1/odom`, `/robot_2/odom`, `/robot_3/odom`) and no context clue in the conversation identifies which robot the user is addressing:
- This is the **one case where namespace selection requires a user question** (Rule 5, condition 1 — genuine ambiguity that introspection cannot resolve).
- Ask once, clearly: *"I found topics from multiple robots: robot_1, robot_2, robot_3. Which robot should I operate?"*
- Once confirmed, use only that robot's namespace for the entire task. Do not re-ask.

**Single namespace or same-robot duplicates:** never ask. Pick per context or first result and report.

### Rule 12 — Run independent discovery commands in parallel

When a task requires discovering multiple independent facts (velocity topic, odom topic, velocity limits, controller state, etc.), issue all discovery commands simultaneously — never sequentially.

Sequential discovery is slower and masks partial failures. If velocity discovery succeeds but odom discovery fails, sequential execution hides that failure until the motion command times out.

**Correct (parallel):**
```bash
# All three issued simultaneously
topics find geometry_msgs/msg/Twist
topics find geometry_msgs/msg/TwistStamped
topics find nav_msgs/msg/Odometry
```

**Wrong (sequential):**
```
topics find geometry_msgs/msg/Twist → wait → topics find TwistStamped → wait → ...
```

**This also applies to parameter scanning (Rule 0, Step 2):** run `params list <NODE>` for all nodes simultaneously, not one node at a time. Process all results together to find the binding velocity ceiling.

**Dependency exception:** if command B requires a result from command A (e.g., you need the topic name before you can check its type), sequential execution is correct for those two steps only. Run everything else in parallel around them.

### Rule 13 — Never reuse stale session state for a new task

Topic names, controller states, node lists, lifecycle states, and parameter values discovered earlier in the session **must not be reused as-is** for a new task. Robot state can change between tasks: nodes crash, controllers switch, topics appear or disappear.

**Per-task re-discovery is mandatory.** The only exception is within the consecutive steps of the same single task (e.g., `VEL_TOPIC` discovered in step 1 of a motion command is used in step 4 of that same command — that is valid).

**Re-discover unconditionally when:**
- The user issues a new request (any request that is not an explicit continuation of the current command)
- An error suggests the graph changed (a topic previously seen is now absent)
- A node that was present is no longer in `nodes list`

**If the graph changes unexpectedly mid-task** (a node disappears, a controller deactivates without being told to, a topic that was publishing goes silent):
1. Stop the current task.
2. Re-run the full Rule 0.1 session-start checks (`doctor`, clock check, lifecycle state).
3. Re-discover all names before continuing — the system state is unknown.
4. Report to the user: what changed, what the checks found, what the impact is.

A mid-task graph change means something crashed or was preempted. Continuing with cached state means continuing blind.

**Never say "I already discovered this earlier" as a reason to skip introspection.** Discovery takes under a second. Stale assumptions cause hard-to-debug failures.

### Rule 14 — Check lifecycle state before using any managed node's interface

If `lifecycle nodes` shows a node is lifecycle-managed, its state determines whether its topics, services, and parameters function:

| State | Behaviour |
|---|---|
| `unconfigured` | Node is up but not configured — topics and services do not exist yet |
| `inactive` | Node is configured but not active — topics exist but messages are silently dropped |
| `active` | Node is fully operational |
| `finalized` / `error` | Node has shut down or faulted — do not attempt to use it |

**Before using any topic, service, or parameter from a lifecycle-managed node:**
1. Run `lifecycle get <node>` to check current state.
2. If not `active`, apply the correct transitions:
   ```bash
   lifecycle set <node> configure   # unconfigured → inactive
   lifecycle set <node> activate    # inactive → active
   ```
3. Verify with `lifecycle get <node>` that the node reached `active` (Rule 8) before proceeding.

**Do not skip this check even if the node "usually works."** A node in `inactive` silently discards all messages — it produces no error, no warning, and no feedback. This is the most common source of inexplicable publish failures on managed-node robots.

### Rule 15 — Check publisher and subscriber counts before waiting on a topic

Before subscribing to a topic and waiting for a message, verify a publisher exists **and** that QoS is compatible:
1. Run `topics details <topic>` — check **both** `publisher_count` **and** the publisher's QoS profile.
2. **If `publisher_count == 0`:** do not subscribe (you will timeout). Report: *"No publisher on `<topic>`. Check `nodes list` to verify the publishing node is running."* Then diagnose per Rule 7.
3. **QoS compatibility check:** inspect the publisher's durability, reliability, and history settings in `topics details` output. Common mismatch: publisher uses `BEST_EFFORT` reliability but the subscriber requires `RELIABLE` — messages will never arrive despite a non-zero publisher count. If a mismatch is detected, add `--qos-reliability best_effort` (or the appropriate flag per COMMANDS.md) to match the publisher's profile.
4. **If `publisher_count > 0` and QoS is compatible but subscribe still times out:** run `topics hz <topic>` to confirm messages are actively flowing (a publisher node may be up but not sending).

**For `<ODOM_TOPIC>` specifically (used in every motion workflow):** always run step 1–3 before the first odom subscribe in Rule 9. A QoS mismatch on odom causes every motion command to hang silently at the stationary check — the most expensive possible failure point.

Before publishing to a topic intended for a subscriber:
1. Run `topics details <topic>` — check `subscriber_count`.
2. **If `subscriber_count == 0`:** there is no node listening. Still publish (the subscriber may be transient or latched), but report: *"No subscribers detected on `<topic>` — command may not reach any controller."*

**This check costs one CLI call and prevents the most common cause of subscribe timeouts.** Run it as part of any workflow that depends on receiving a message within a timeout window.

### Rule 16 — Multi-step tasks: complete and verify each step before starting the next

When a user's request involves a sequence of sub-commands (e.g., "move forward 1 m, then rotate 90°", "switch to controller A, then send a trajectory"), treat each sub-command as an independent atomic step:

1. **Execute step N.**
2. **Verify step N completed successfully (Rule 8)** — confirm the effect occurred, not just that the command returned without error.
3. **Only then start step N+1.**
4. **If step N fails:** stop the sequence immediately. Diagnose per Rule 7. Do not proceed to step N+1 with a failed or partial state from step N.

**Never pipeline or parallelise dependent steps.** Starting step N+1 before step N is confirmed complete risks compounding errors: the second command acts on a state the first command never achieved.

**Independent steps within the same phase** (e.g., discovering the velocity topic and discovering the odom topic) can and should run in parallel (Rule 12). The sequencing requirement applies only to steps where step N+1 depends on step N's outcome.

**Examples:**

| Request | Correct sequencing |
|---|---|
| "Move 1 m forward, then rotate 90° right" | 1. Discover topics → 2. publish-until forward → **verify odom delta** → 3. publish-until rotate → verify rotation |
| "Configure the arm controller, then send a trajectory" | 1. `control configure-controller` → **verify `inactive`** → 2. `control switch-controllers --activate` → **verify `active`** → 3. `actions send` |
| "Set max speed to 0.3, then move forward" | 1. `params set` → **`params get` to verify** → 2. discover topics → 3. publish-until |

**If any step in the sequence changes the robot's physical state** (position, controller state, parameter value), verify that change before building on it.

**Motion sequence pose carry-forward:** when step N is a motion command, the post-motion verified end-pose (from Rule 8 movement completion protocol) is automatically the pre-motion baseline for step N+1 — **do not issue a redundant Rule 9 odom subscribe** between consecutive motion steps in the same sequence. The verified end-pose already represents a fresh, stationary, sensor-confirmed reading. Re-run the Rule 9 node-presence check (parallel `nodes list`) before step N+1, but skip the odom subscribe — use the carry-forward pose as the new baseline instead.

### Rule 17 — Follow REP-103 and REP-105 at all times

ROS 2 has standardised units, coordinate conventions, and frame conventions. Violating them produces silent wrong results — wrong yaw, wrong direction, wrong distance — with no error from the CLI.

#### REP-103: Standard Units and Coordinate Conventions

**Units — all message values use SI units. No exceptions.**

| Quantity | Unit | Never use |
|---|---|---|
| Linear distance | metres (m) | cm, mm, inches |
| Linear velocity | m/s | cm/s, km/h |
| Angular position | radians (rad) | degrees (in payloads) |
| Angular velocity | rad/s | deg/s, RPM |
| Time | seconds (s) | ms unless in a `builtin_interfaces/Time` stamp |
| Frequency | Hz | — |

**The `--degrees` flag in `publish-until --rotate` and `--rotate --degrees` is a CLI convenience only.** The underlying `angular.z` in the Twist/TwistStamped payload is always rad/s. Never put degrees into a message field.

**Coordinate frame convention (right-hand rule):**

| Axis | Direction |
|---|---|
| x | Forward |
| y | Left |
| z | Up |

- Positive `linear.x` → robot moves **forward**
- Negative `linear.x` → robot moves **backward**
- Positive `linear.y` → robot strafes **left** *(holonomic platforms only — has no effect on differential-drive robots)*
- Negative `linear.y` → robot strafes **right** *(holonomic only)*
- Positive `angular.z` → robot rotates **CCW (left)** when viewed from above
- Negative `angular.z` → robot rotates **CW (right)**
- This matches the `--rotate` sign convention: positive = CCW, negative = CW

**Holonomic vs. differential-drive:** before commanding `linear.y`, confirm the robot is holonomic (e.g., mecanum or omni wheels). On a differential-drive robot, `linear.y` is silently ignored — use `angular.z` followed by `linear.x` to achieve lateral repositioning instead.

**Quaternion field order in all ROS 2 messages is `(x, y, z, w)` — never `(w, x, y, z)`.**

**Yaw from odometry quaternion (2D mobile robot — pure z-rotation):**
```
yaw_rad = 2 * atan2(q.z, q.w)
yaw_deg = yaw_rad * (180 / π)
```
This simplified formula is valid when `q.x ≈ 0` and `q.y ≈ 0` (flat-ground robot). For a 3D platform, use the full euler extraction formula. Verify `q.x` and `q.y` are near zero before using the simplified form.

**Never:**
- Put degree values into `angular.z` or any rotation field in a message
- Assume the quaternion order is `(w, x, y, z)` — ROS 2 uses `(x, y, z, w)`
- Use `angular.z` with opposite sign to `--rotate` — they must always match

#### REP-105: Coordinate Frames for Mobile Platforms

**The standard frame hierarchy is:**
```
map → odom → base_link (→ sensor frames)
```

| Frame | Properties | Use for |
|---|---|---|
| `base_link` | Attached to robot body | Relative transforms from the robot |
| `odom` | Continuous, no jumps — drifts over time | Closed-loop motion tracking; `publish-until` delta monitoring |
| `map` | Globally accurate — may jump when localisation corrects | Absolute position queries; navigation goals |

**For closed-loop motion (publish-until):** monitor position delta in the `odom` frame — use `pose.pose.position.x` (or `.y`) from `nav_msgs/Odometry`, which is expressed in the `odom` frame. This is correct for tracking relative displacement.

**For absolute position queries** (e.g., "where is the robot on the map?"): look up the `map` → `base_link` transform via `tf lookup`. The `odom` frame position drifts; the `map` frame is corrected by localisation (AMCL, Cartographer, etc.).

**Never:**
- Use the `map` frame position from an odometry message — odometry is expressed in `odom`, not `map`
- Use the `odom` frame for absolute global position when a localiser is running — use `map` instead
- Assume frame names are exactly `map`, `odom`, `base_link` without first running `tf list` (Rule 0) — they may be namespaced (e.g., `/robot_1/odom`)
- Confuse a jump in `map` → `odom` (localisation correction) with the robot physically moving
- Consume spatial sensor data without first verifying the sensor's `frame_id` exists in the TF tree and is not stale — run `tf list` and `tf echo <SENSOR_FRAME> <BASE_FRAME>` before using any sensor's spatial output

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

### Motion Context Cues — Vocabulary and Method Selection

**Any of the following words in a user message is a motion command.** Every motion command triggers the full Rule 3 workflow — no exceptions. Odom availability (discovered in Rule 3 Step 2) determines the execution method, not the user's phrasing.

**Motion trigger words (non-exhaustive — treat similar words the same way):**

| Category | Trigger words |
|---|---|
| General motion | move, go, drive, travel, head, proceed, advance, roll, navigate |
| Forward | forward, ahead, straight, onward, front |
| Backward | back, backward, backwards, reverse, retreat |
| Left | left, leftward |
| Right | right, rightward |
| Rotation / turning | rotate, turn, spin, yaw, pivot, swing |
| Strafing (holonomic) | strafe, slide, sidestep, lateral |

**Method selection — always determined by odom availability, not user phrasing:**

```
Motion command received
  │
  ├─ Check odom (Rule 3 Step 2: topics find nav_msgs/msg/Odometry + topics hz)
  │
  ├─ Odom available (≥ 5 Hz) AND target distance/angle specified
  │     └─ publish-until (closed-loop) — PREFERRED
  │
  ├─ Odom available (≥ 5 Hz) AND no target (open-ended)
  │     └─ publish-sequence with stop payload — odom used for pre/post health checks only
  │
  ├─ Odom unavailable (not found or rate < 5 Hz) AND target specified
  │     └─ publish-sequence with calculated duration (open-loop, last resort)
  │        Notify user: "No odometry. Running open-loop — accuracy not guaranteed."
  │
  └─ Odom unavailable AND no target
        └─ publish-sequence with stop payload
           Notify user: "No odometry available."
```

**Never interpret a motion word as "just publish a Twist message" without going through this decision tree.** The choice of `publish-until` vs. `publish-sequence` is always driven by odom availability.

### Step 1: Understand User Intent

| User says... | Agent interprets as... | Agent must... |
|--------------|----------------------|---------------|
| **— INTROSPECTION: DISCOVERY —** | | |
| "what topics / what's publishing / list topics / show all topics / show me everything on the bus / what data is available" | List all active topics | `topics list` |
| "what nodes / what's running / list nodes / what processes / active nodes / who's running / is X node running / is the robot up" | List all running nodes | `nodes list` |
| "what services / list services / what services are available / what can I call / what service endpoints exist" | List all services | `services list` |
| "what actions / list actions / what can the robot do / what action servers exist / what goals can I send" | List all actions | `actions list` |
| "what controllers / list controllers / controller status / which controller is active / is the velocity controller running / what's loaded in the controller manager" | List ros2_control controllers | `control list-controllers` |
| "what hardware / hardware components / actuators / sensors available / what physical hardware is there / what joints are registered" | List hardware components | `control list-hardware-components` |
| "what hardware interfaces / command interfaces / state interfaces / what can be commanded / what's being read from hardware" | List hardware interfaces | `control list-hardware-interfaces` |
| "what controller types / available controller types / what controllers can I load / what plugins are available" | List loadable controller types | `control list-controller-types` |
| "what lifecycle nodes / managed nodes / list lifecycle / what nodes have lifecycle / which nodes need activating" | List lifecycle-managed nodes | `lifecycle nodes` |
| "what TF frames / list frames / what coordinate frames / show TF tree / what frames are being broadcast / what links exist" | List all TF frames | `tf list` |
| "what parameters / list params / show config / what settings does X have / what does X expose / what can I tune on X" | List parameters on a node | `nodes list` → `params list <node>` |
| "is X running / is X online / is X alive / is node X up / did X start" | Check if a specific node is present | `nodes list` → check if X appears |
| "is topic X live / is X publishing / is X active / is anyone publishing X" | Check if topic has active publisher | `topics details <topic>` (publisher_count > 0) + `topics hz <topic>` |
| "does service X exist / is service X available / is X callable" | Check if service is present | `services list` → check for X |
| "does action X exist / is action server X running" | Check if action server is present | `actions list` → check for X |
| **— INTROSPECTION: DETAILS —** | | |
| "what type is topic X / what message type does X use / what does X carry" | Get topic message type | `topics type <topic>` |
| "show details of topic X / how many subscribers does X have / QoS of X / who's listening to X / who publishes X / publisher count / subscriber count" | Get topic details | `topics details <topic>` |
| "what fields does X message have / show X structure / X message layout / what's in an X message / X definition" | Get message field structure | `topics message <type>` or `interface show <type>` |
| "give me X message template / X payload template / copy-paste template for X / default values for X" | Get default message values | `interface proto <type>` |
| "what does service X do / service X request fields / service X signature / how do I call service X / what does X service take" | Get service request/response | `services details <service>` |
| "what does action X take / action X goal fields / action X signature / how do I send action X / what input does X need" | Get action goal/result/feedback | `actions details <action>` |
| "what does node X do / topics of node X / services of node X / what does X publish / what does X subscribe to / node X interfaces" | Get node details | `nodes details <node>` |
| "controller chain / how controllers connect / view controller chain / controller dependencies / controller pipeline" | View controller chain | `control view-controller-chains` |
| "what's the QoS of X / reliability of X / durability of X / is X transient local / best effort or reliable" | Check QoS profile | `topics details <topic>` (read QoS fields) |
| "what namespace is X in / what's the prefix of X / robot namespace" | Inspect topic/node namespace | `topics list` or `nodes list` → examine returned names |
| **— READING / MONITORING: GENERIC —** | | |
| "read / listen to / monitor / subscribe to / show me data from / stream / watch / look at / observe / sample / echo / print / dump / what is X publishing / what's on X" | Subscribe to a topic | `topics find <type>` → `topics subscribe <discovered_topic>` |
| "how fast is X publishing / topic rate / is X active / Hz of X / what's the frequency of X / publish rate of X / messages per second" | Check topic publish rate | `topics hz <topic>` (discover topic first if name not given) |
| "bandwidth of X / how much data is X sending / throughput of X / how many bytes per second / data rate of X" | Check topic bandwidth | `topics bw <topic>` |
| "latency of X / delay on X / how delayed is X / lag on X / transmission delay" | Check topic delay | `topics delay <topic>` |
| "is X healthy / is X working / is X OK / is X updating / is X stale" | Check liveness of a topic | `topics hz <topic>` — if rate ≥ expected → healthy; 0 Hz → stale/dead |
| **— READING / MONITORING: SENSORS —** | | |
| "read the LiDAR / scan data / laser scan / range scan / obstacle data / proximity scan / 2D scan" | Subscribe to LaserScan | Find `sensor_msgs/msg/LaserScan` → subscribe |
| "read point cloud / 3D scan / LiDAR points / 3D LiDAR / velodyne / depth points / PCL data" | Subscribe to PointCloud2 | Find `sensor_msgs/msg/PointCloud2` → subscribe |
| "read odometry / where is the robot / current position / current pose / where am I / robot location / robot coordinates / pose estimate" | Subscribe to Odometry | Find `nav_msgs/msg/Odometry` → subscribe (post-motion: use Rule 8 two-phase protocol) |
| "read camera / take a picture / capture image / take a photo / snap / grab a frame / screenshot / what does the camera see / show me the view / photograph" | Capture image from camera | Find `sensor_msgs/msg/Image` + `CompressedImage` → `topics capture-image --topic <topic>` |
| "read depth image / depth camera / RGBD / depth frame / depth data" | Subscribe to depth Image | Find `sensor_msgs/msg/Image` with `depth` in topic name → subscribe or capture-image |
| "read joint states / joint positions / joint angles / encoder values / current joint config / what are the joint positions" | Subscribe to JointState | Find `sensor_msgs/msg/JointState` → subscribe |
| "read wheel speeds / wheel velocities / wheel odometry" | Subscribe to JointState or Odometry | Find `sensor_msgs/msg/JointState` (check velocity fields) or Odometry twist fields |
| "read IMU / accelerometer / gyroscope / orientation data / angular velocity / linear acceleration / inertial data" | Subscribe to Imu | Find `sensor_msgs/msg/Imu` → subscribe |
| "read heading / compass / magnetic heading / magnetometer" | Subscribe to Imu or MagneticField | Find `sensor_msgs/msg/Imu` (orientation) or `sensor_msgs/msg/MagneticField` → subscribe |
| "read GPS / GNSS / latitude / longitude / fix / GPS coordinates" | Subscribe to NavSatFix | Find `sensor_msgs/msg/NavSatFix` → subscribe |
| "check battery / battery level / how much charge / power level / battery status / state of charge / voltage / remaining runtime" | Subscribe to BatteryState | `topics battery` (auto-discovers) or find `sensor_msgs/msg/BatteryState` |
| "read joystick / gamepad input / joystick data / controller input / joypad" | Subscribe to Joy | Find `sensor_msgs/msg/Joy` → subscribe |
| "read force / torque / wrench / FT sensor / force-torque sensor / load cell / contact force" | Subscribe to WrenchStamped | Find `geometry_msgs/msg/WrenchStamped` or `geometry_msgs/msg/Wrench` → subscribe |
| "read contact / bump sensor / collision / bumper" | Subscribe to contact/bumper topic | Find by type or keyword → subscribe |
| "read map / occupancy grid / current map / what does the map look like" | Subscribe to OccupancyGrid | Find `nav_msgs/msg/OccupancyGrid` → subscribe |
| "read costmap / local costmap / global costmap / navigation costmap" | Subscribe to costmap topic | Find `nav_msgs/msg/OccupancyGrid` with `costmap` in name → subscribe |
| "read the clock / ROS time / simulation time / what time is it in ROS" | Subscribe to /clock or get ROS time | Find `rosgraph_msgs/msg/Clock` → subscribe --max-messages 1 |
| "check diagnostics / diagnostic status / what's wrong / robot health / hardware errors / error messages / warnings / diagnostic aggregator" | Subscribe to diagnostics | `topics diag` or find `/diagnostics` topic → subscribe |
| **— TRANSFORMS (TF) —** | | |
| "where is X in Y frame / transform from X to Y / position of X relative to Y / pose of X in Y / X expressed in Y" | Look up TF transform | `tf list` (discover frames) → `tf lookup <source> <target>` |
| "stream transform / monitor TF / watch transform from X to Y / continuously get TF X to Y" | Echo TF transform continuously | `tf echo <source> <target>` |
| "is TF updating / TF health / TF alive / is the TF tree publishing / are transforms fresh" | Monitor TF update rate | `tf monitor` |
| "convert quaternion to euler / what's the roll pitch yaw / euler angles from quaternion / extract RPY / what's the yaw from this quaternion" | Convert quaternion → euler | `tf euler-from-quaternion` |
| "convert euler to quaternion / quaternion from roll pitch yaw / RPY to quaternion / degrees to quaternion" | Convert euler → quaternion | `tf quaternion-from-euler` |
| "transform point / where is point X in frame Y / reproject point to frame Y" | Transform a point between frames | `tf transform-point` |
| "transform vector / transform direction from X to Y / reproject vector" | Transform a vector between frames | `tf transform-vector` |
| "add static transform / publish fixed TF / broadcast constant TF / add fixed link / static broadcaster" | Publish a static TF | `tf static` |
| **— MOTION (mobile robot) —** | | |
| **Any motion word** + direction + **specific distance** (e.g. "move / drive / go / travel / head / proceed / advance / roll forward 1 m", "back up 0.5 m", "creep forward 10 cm", "nudge forward 0.05 m", "inch back 0.1 m") | Closed-loop distance if odom ≥ 5 Hz, else timed open-loop | **Odom ≥ 5 Hz** → `publish-until --field pose.pose.position --delta N --timeout <calc>`<br>**No odom** → `publish-sequence` duration `N/v`; notify user |
| **Any motion word** + direction + **specific angle** (e.g. "rotate / turn / spin / pivot / yaw / swing 90°", "turn left 45 degrees", "face right 180°", "turn around", "U-turn", "do a 180") | Closed-loop rotation if odom ≥ 5 Hz, else timed open-loop | **Odom ≥ 5 Hz** → `publish-until --rotate ±N --degrees --timeout <calc>`<br>**No odom** → `publish-sequence` duration `θ_rad/ω`; notify user |
| **Any motion word** + direction, **no target** (e.g. "move / drive / go / roll / head forward", "back up", "go right", "retreat", "pull back", "charge ahead") | Open-ended — run until stopped | `publish-sequence` move payload + zero payload |
| "strafe left / slide left / move sideways left / sidestep left" *(holonomic only)* | Strafe left — positive linear.y | Confirm holonomic → `publish-sequence` or `publish-until` with `linear.y > 0` |
| "strafe right / slide right / move sideways right / sidestep right" *(holonomic only)* | Strafe right — negative linear.y | Confirm holonomic → `publish-sequence` or `publish-until` with `linear.y < 0` |
| **Any motion word**, no direction, no target (e.g. bare "move", "drive", "go", "navigate") | Ambiguous — ask once (Rule 5 condition 3) | Ask: *"Which direction, and how far?"* |
| "stop / halt / freeze / stop moving / hold position / stand still / don't move" | Publish zero velocity | Find Twist/TwistStamped → publish zeros |
| "emergency stop / e-stop / STOP NOW / kill velocity / cut motors / abort motion / kill motion" | Emergency stop | `estop` (verify effect per Rule 8) |
| **— MANIPULATOR / ARM —** | | |
| "move arm / move joint / move to joint angles / move to position / send trajectory / execute trajectory / move to pose / reach X / extend arm" | Send JointTrajectory or FollowJointTrajectory action | Find `trajectory_msgs/msg/JointTrajectory` or `FollowJointTrajectory` action → send |
| "home the arm / go to home position / return to home / reset arm / go to zero / go to ready" | Send home trajectory or call home service | Find home service/action or send zero-position trajectory |
| "control gripper / open gripper / close gripper / grip / release / grab / let go / grasp / ungrasp" | Publish GripperCommand or trajectory | Find `control_msgs/msg/GripperCommand` or gripper trajectory topic → publish |
| **— SERVICES & ACTIONS —** | | |
| "call service X / trigger X / invoke X / reset X / clear X / initialize X / ping X / fire X / activate X via service" | Call a ROS 2 service | `services list` or `services find <type>` → `services call <service> <json>` |
| "reset the robot / reinitialize / factory reset / restart robot state" | Call reset service | Find reset/reinitialize service → `services call` |
| "set initial pose / tell the robot where it is / localize at X,Y / set pose estimate / AMCL initial pose" | Publish initial pose | Find `/initialpose` topic (`geometry_msgs/msg/PoseWithCovarianceStamped`) → publish |
| "clear costmap / clear the map / reset costmap / flush costmap" | Call clear_costmap service | Find `nav2_msgs/srv/ClearCostmapAroundRobot` or similar → `services call` |
| "save the map / export the map / write map to file" | Call map_saver service | Find map_saver service or action → call |
| "calibrate / run calibration / calibrate sensor X / tare X / zero the FT sensor" | Call calibration/tare service | Find calibration service → `services call` |
| "navigate to / go to pose / move to coordinates / send navigation goal / drive to X,Y / go to X,Y,Z" | Send navigation action (Nav2 / move_base) | `actions find NavigateToPose` or similar → `actions send <action> <goal_json>` |
| "dock / undock / auto-dock / return to base / go to charging station" | Send dock action or call dock service | Find dock action/service → send/call |
| "cancel navigation / abort goal / stop action / cancel goal / preempt / abort / never mind" | Cancel an action goal | `actions cancel <goal_id>` |
| "what's the navigation status / is navigation done / how far to the goal / navigation progress" | Monitor action feedback | `actions echo <action>` or check feedback from active goal |
| "watch service calls / monitor service X / echo service X / spy on service X" | Monitor service calls | `services echo <service>` |
| "watch action / monitor action X / echo action feedback / stream action feedback" | Monitor action feedback | `actions echo <action>` |
| **— LAUNCH & NODE EXECUTION —** | | |
| "start launch file / run launch file / launch X / bring up X / start bringup / start the robot / boot X / start X stack / spin up X / fire up X / initialize the robot / init bringup / start everything" | Start a launch file in tmux | `launch new <package> <launch_file>` |
| "list launches / what's launched / running sessions / what launch files are running / show active launches / what's been started / what tmux sessions exist" | List active tmux launch sessions | `launch list` |
| "stop launch / kill launch / kill session X / stop bringup / shut down X / tear down X / kill everything / stop the robot / take down bringup" | Kill a tmux launch session | `launch kill <session>` |
| "restart launch / restart bringup / restart session X / relaunch X / bounce X / reload X / cycle X" | Restart a tmux launch session | `launch restart <session>` |
| "open foxglove / start foxglove / visualize robot / foxglove bridge / open the visualizer / open foxglove studio" | Start Foxglove via launch | `launch foxglove` |
| "start node X / run node X / run executable X / execute X node / spawn X node / bring up node X only" | Run a single node | `run new <package> <executable>` |
| **— PARAMETERS —** | | |
| "what is X parameter / get X value / current value of X / what's X set to / read X param / what's the current X / show me X" | Get parameter value | `nodes list` → `params get <node:param>` |
| "what's the max speed / maximum velocity / top speed / speed limit / velocity limit / max linear / max angular" | Get velocity limit parameters | `nodes list` → `params list <node>` per node → find max/limit/vel params → `params get` |
| "what's the acceleration limit / max acceleration / accel cap / deceleration limit" | Get acceleration parameters | `nodes list` → find accel/decel params → `params get` |
| "set X to Y / change X parameter / configure X to Y / update X setting / adjust X / increase X to Y / decrease X to Y / lower X / raise X / turn up X / turn down X" | Set parameter value | `params describe` (type check) → `params set <node:param> <value>` → `params get` (verify) |
| "enable X / turn on X" *(as a boolean parameter)* | Set boolean param to true | `params describe` → `params set <node:param> true` → verify |
| "disable X / turn off X" *(as a boolean parameter)* | Set boolean param to false | `params describe` → `params set <node:param> false` → verify |
| "describe X parameter / what type is X / valid range of X / is X read-only / what values can X take / X constraints" | Describe parameter type and constraints | `params describe <node:param>` |
| "dump parameters / export config / save current params / dump all params / get all params from X" | Dump all params from a node | `params dump <node>` |
| "load parameters / restore config / load params from file / apply YAML to X / load config from file / use config file / load YAML params" | Load params from YAML file — **Rule 0 pre-flight required**: `params list <node>` + `params describe` each key before loading; verify with `params get` after | `params load <node> <file>` |
| "use params file / pass params file / launch with params file / --params-file / YAML params at launch / load parameters at startup" | Load params at launch time via `--params-file` | Run the node/launch with `--params-file <path>` — but first: `params list <node>` to compare YAML keys, `params describe` each to confirm types; re-verify with `params get` after the node is running |
| "delete parameter / remove param X" | Delete a parameter | `params delete <node:param>` |
| "save parameter preset / save config preset / save settings as X / save preset / snapshot parameters / save this config" | Save parameter preset to file | `params preset-save` |
| "load parameter preset / apply preset X / restore preset X / load saved settings / use preset X" | Load parameter preset | `params preset-load <name>` |
| "list presets / show saved presets / what presets exist / what parameter snapshots are there" | List saved parameter presets | `params preset-list` |
| "delete preset X / remove saved preset / clear preset X" | Delete a parameter preset | `params preset-delete <name>` |
| **— CONTROLLERS (ros2_control) —** | | |
| "load controller X / add controller X / register controller X" | Load a controller into the controller manager | `control load-controller <name>` |
| "unload controller X / remove controller X / drop controller X" | Unload a controller | `control unload-controller <name>` |
| "configure controller X / initialize controller X / set controller X to inactive / prep controller X" | Configure (transition to inactive) | `control configure-controller <name>` → verify `inactive` |
| "switch to X controller / activate X controller / enable X controller / use X controller / change controller to X / swap to X controller / put X controller in charge" | Switch active controllers | `control list-controllers` → `control switch-controllers --activate <new> --deactivate <old> --strictness STRICT` |
| "use position control / position controller / go to position mode" | Switch to position controller | Discover position controller name → switch-controllers |
| "use velocity control / velocity controller / go to velocity mode" | Switch to velocity controller | Discover velocity controller name → switch-controllers |
| "use trajectory control / joint trajectory controller / go to trajectory mode" | Switch to JointTrajectoryController | Discover trajectory controller name → switch-controllers |
| "pause controller X / stop controller X / deactivate controller X" | Set controller to inactive | `control set-controller-state <name> inactive` |
| "resume controller X / restart controller X" | Reactivate controller | `control set-controller-state <name> active` |
| "enable hardware / activate hardware component X / bring up hardware / enable actuator X / activate motor X" | Set hardware component state to active | `control set-hardware-component-state <name> active` |
| "disable hardware / deactivate hardware X / power down actuator X / disable motor X" | Set hardware component state to inactive | `control set-hardware-component-state <name> inactive` |
| **— LIFECYCLE NODES —** | | |
| "check lifecycle state / what state is X / is X active / X node status / is X configured / is X initialized" | Get lifecycle node state | `lifecycle get <node>` |
| "configure X node / initialize X lifecycle node / prep X node / set up X node" | Lifecycle configure transition | `lifecycle set <node> configure` → verify `inactive` |
| "activate X node / start X node / enable X node / bring up X node / bring X online / make X active" | Lifecycle activate transition | `lifecycle set <node> activate` → verify `active` |
| "fully start X node / configure and activate X / bring X fully online" | Full configure + activate sequence | `lifecycle set <node> configure` → verify `inactive` → `lifecycle set <node> activate` → verify `active` |
| "deactivate X node / stop X node / disable X node / pause X node / take X offline" | Lifecycle deactivate transition | `lifecycle set <node> deactivate` → verify `inactive` |
| "fully stop X node / deactivate and clean up X" | Full deactivate + cleanup sequence | `lifecycle set <node> deactivate` → `lifecycle set <node> cleanup` |
| "clean up X node / reset X lifecycle / cleanup X / unconfigure X" | Lifecycle cleanup transition | `lifecycle set <node> cleanup` → verify `unconfigured` |
| "shut down X node / shutdown X / kill X lifecycle node / finalize X" | Lifecycle shutdown transition | `lifecycle set <node> shutdown` → verify `finalized` |
| **— DIAGNOSTICS & HEALTH —** | | |
| "run health check / diagnose the robot / is everything OK / run diagnostics / check the system / ros2 doctor / any issues / anything broken / system check" | Run full ROS 2 health check | `doctor` |
| "why isn't X working / X isn't responding / X seems broken / troubleshoot X / debug X / X is not publishing" | Diagnose a specific problem | `doctor` + `nodes list` + `topics hz <X>` + `topics details <X>` + diagnose per Rule 7 |
| "is the robot ready to move / is the robot ready / can I send commands / is everything up" | Full readiness check | `doctor` + `nodes list` + `topics hz <ODOM_TOPIC>` + `control list-controllers` |
| "give me a full system overview / system summary / robot status overview / what's the full state" | Full system snapshot | `doctor` + `topics list` + `nodes list` + `control list-controllers` + `lifecycle nodes` |
| "test connectivity / test DDS / test network / test multicast / DDS reachability check" | Test DDS multicast connectivity | `doctor hello` |
| "check skill version / what version is this / what version of ros2_cli / skill info" | Get ros2_cli version | `version` |
| "is the daemon running / is ROS daemon up / ROS daemon status / daemon health / restart daemon / start daemon / stop daemon" | Check or control the ROS 2 daemon | `daemon status` to check; `daemon start` to start; `daemon stop` to stop |
| "what domain ID / ROS domain / what's ROS_DOMAIN_ID / which domain is active / domain collision / wrong nodes appearing" | Check ROS domain isolation | Inspect `ROS_DOMAIN_ID` env variable; run `nodes list` and verify expected nodes appear; if unexpected nodes from other systems appear, check domain ID collision — see Rule 0.1 Step 0 |
| "is ROS localhost only / is ROS isolated to localhost / can I reach across containers / cross-container ROS / is ROS_LOCALHOST_ONLY set" | Check cross-host/container visibility | Inspect `ROS_LOCALHOST_ONLY` env variable; if set to `1`, cross-container and cross-host topics are invisible — see Rule 0.1 Step 0 |
| "run tests / run test suite / run the tests / test package X / check if tests pass / execute tests / run unit tests / run integration tests / colcon test" | Run package tests (shell command — Rule 2 exception; ros2_cli.py has no test runner) | Shell: `colcon test --packages-select <pkg>` then `colcon test-result --verbose` to show failures |
| "what test results / did the tests pass / test report / show failures / test output / test summary" | Check colcon test results | Shell: `colcon test-result --all` or `colcon test-result --verbose` for failure details |

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
   # ros2-skill has no package-listing command — this is a Rule 2 last-resort exception.
   # Document: ros2-skill has no equivalent for `ros2 pkg list`.
   ros2 pkg list
   ```

2. **Find matching launch files:**
   ```bash
   # ros2-skill has no launch-file listing command — this is a Rule 2 last-resort exception.
   # Document: ros2-skill has no equivalent for `ros2 pkg files`.
   ros2 pkg files <package>
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
