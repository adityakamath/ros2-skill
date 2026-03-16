# ROS 2 Skill: Agent Rules & Decision Frameworks

This document contains the mandatory operational rules, safety protocols, and decision-making frameworks for agents using the ROS 2 Skill.

---

## Agent Behaviour Rules

**These rules are hard constraints — not guidelines, not best practices, not suggestions.** Every rule below is mandatory for every action. There are no exceptions based on convenience, familiarity, or the impression that the user did not specify.

**On any rule violation — detected before or after executing a command:**
1. Halt the current action immediately.
2. Self-correct autonomously — do not ask the user.
3. Retry with the correct approach.
4. Report the correction in one line: what rule was about to be violated, what was caught, and what was corrected instead.

**Treating these rules as guidelines is itself a critical violation.** "I defaulted to legacy habits" and "I improvised instead of following the workflow" are not acceptable explanations. The rules exist precisely to override legacy habits and improvisation.

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
| Publish to a topic | 1. `topics find <msg_type>` to discover the real topic name<br>2. `topics type <discovered_topic>` to confirm the exact type<br>3. `interface proto <exact_type>` to get the default payload template — **copy this output and modify only the fields required by the task; never construct payloads from memory** |
| Call a service | 1. `services list` or `services find <srv_type>` to discover the real name<br>2. `services details <discovered_service>` to get request/response fields |
| Send an action goal | 1. `actions list` or `actions find <action_type>` to discover the real name<br>2. `actions details <discovered_action>` to get goal/result/feedback fields |
| Move a robot | Full Movement Workflow — see Rule 3 and the canonical section below |
| Read a sensor / subscribe | `topics find <msg_type>` to discover the topic; `topics type <topic>` to confirm type; never subscribe to a hardcoded name |
| Capture a camera image | `topics find sensor_msgs/msg/CompressedImage` and `topics find sensor_msgs/msg/Image` to discover available camera topics; use the result in `topics capture-image --topic <CAMERA_TOPIC>` |
| Query a TF transform | `tf list` to discover available frames; never hardcode frame names like `map`, `base_link`, `odom` |
| Switch or control a controller | `control list-controllers` to discover controller names and states; never hardcode controller names; always use `--strictness STRICT` for `switch-controllers` unless explicitly instructed otherwise |
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
- ❌ Never construct a message payload from memory — always use `interface proto <type>` output as the starting template and modify only the fields required by the task
- ❌ Never revert to hardcoded or legacy behaviors after a robust introspection-driven workflow is established — even if the hardcoded name "usually works" on this specific robot
- ❌ Never bypass, skip, or abbreviate safety checks even if the user explicitly requests it — safety rules are not negotiable

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

**Only ask the user if**:
1. The discovery command returns an empty result or an error, **and**
2. There is genuinely no other way to determine the information from the live system.

### Rule 2 — ros2-skill is the only interface; never use the `ros2` CLI directly

**All ROS 2 operations must go through `ros2_cli.py`. Never call `ros2 topic list`, `ros2 node list`, `ros2 service call`, or any other `ros2 <command>` CLI directly.**

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

**There are exactly two conditions that permit asking the user:**

1. **Genuine ambiguity that cannot be resolved by introspection:**
   - Multiple packages or launch files match and you cannot determine which one the user means
   - A required argument has no match in `--show-args` and no reasonable fuzzy match exists

2. **The action is destructive, irreversible, or safety-critical:**
   - Deleting data (preset files, logs) when the target is ambiguous
   - Stopping a controller that may be load-bearing for other systems
   - Any action the user's message did not clearly and specifically authorise

If neither condition applies: **just do it.**

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
| Movement completed | Start position, end position, actual distance/angle travelled, and any anomalies — nothing else |
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
| `params set <node:param> <val>` | Run `params get <node:param>` — confirm returned value matches what was set |
| `control switch-controllers` | Run `control list-controllers` — confirm new controller is `active`, old is `inactive` |
| `lifecycle set <node> <transition>` | Run `lifecycle get <node>` — confirm the node reached the expected state |
| `actions send <action> <json>` | Check the `status` field in the result: `SUCCEEDED` = completed; `FAILED` or `CANCELED` = treat as failure, diagnose per Rule 7; any other value = not complete yet. A goal sent is not a goal accepted or completed. |
| `control configure-controller` | Run `control list-controllers` — confirm controller reached `inactive` state |
| `topics publish` (single shot, state-change intent) | Run `topics subscribe <topic> --max-messages 1 --timeout 3` or `topics hz <topic>` to confirm messages are being received |

**Never report "Done" based solely on a zero-error response.** If verification reveals the effect did not occur (param unchanged, controller not switched, lifecycle state unchanged), diagnose immediately per Rule 7 — do not report success.

**Exception:** For `publish-until` and `publish-sequence`, the command's own stop condition or duration is the success criterion. Do not add a separate `topics hz` check during an active motion sequence.

### Rule 9 — Pre-motion check: confirm the robot is stationary before commanding movement

**Before issuing any motion command**, verify the robot is not already moving:

1. Subscribe to `<ODOM_TOPIC>` for one message (2-second timeout):
   ```bash
   topics subscribe <ODOM_TOPIC> --max-messages 1 --timeout 2
   ```
2. Check `twist.twist.linear.x`, `twist.twist.linear.y`, and `twist.twist.angular.z` in the returned message.
3. **If any value is non-zero:** send `estop` immediately, wait 0.5 s, then proceed with the requested command. Report: *"Robot was already moving. Stopped before issuing new command."*
4. **If the subscribe times out (odom not publishing):** do NOT proceed with closed-loop motion. Fall back to open-loop (`publish-sequence`) and notify the user: *"Odometry is not publishing. Running open-loop — distance accuracy not guaranteed."*

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

**Namespace selection is never a reason to ask the user.** Pick one based on context, report it, proceed.

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

Before subscribing to a topic and waiting for a message, verify a publisher exists:
1. Run `topics details <topic>` — check `publisher_count`.
2. **If `publisher_count == 0`:** do not subscribe (you will timeout). Report: *"No publisher on `<topic>`. Check `nodes list` to verify the publishing node is running."* Then diagnose per Rule 7.
3. **If `publisher_count > 0` but subscribe still times out:** run `topics hz <topic>` to confirm messages are actively flowing (a publisher node may be up but not sending). Check QoS profile mismatch in `topics details <topic>` output.

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
