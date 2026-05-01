# ROS 2 Skill: Core Agent Behaviour Rules

> **This file is part of a split rule set.** The full rule set spans five files:
> - **RULES-CORE.md** ← you are here — general agent conduct, applies to every command
> - [RULES-PREFLIGHT.md](RULES-PREFLIGHT.md) — before every action: introspection, session-start, hardware readiness
> - [RULES-MOTION.md](RULES-MOTION.md) — motion workflow: discover → execute → estop → verify
> - [RULES-DIAGNOSTICS.md](RULES-DIAGNOSTICS.md) — when things go wrong: diagnose, verify, escalate
> - [RULES-REFERENCE.md](RULES-REFERENCE.md) — lookup tables: intent→command, launch workflow, setup

---

## Mandatory Compliance

**The rules in RULES-CORE.md, RULES-PREFLIGHT.md, RULES-MOTION.md, RULES-DIAGNOSTICS.md, and RULES-REFERENCE.md are hard constraints — not guidelines, not best practices, not suggestions.** Every rule is mandatory for every action. There are no exceptions based on convenience, familiarity, or the impression that the user did not specify.

**Every action has three mandatory phases. All three are required. Skipping any phase is a rule violation.**

| Phase | What it means | Rule |
|---|---|---|
| **Pre-action** | Introspect the live system — discover names, types, states, limits | Rule 0 (RULES-PREFLIGHT.md) |
| **Execution** | Act using only discovered names and verified structures | Rules 1–5 (this file) |
| **Post-action** | Verify the effect occurred — never claim a result without direct confirmation | Rule 8 (RULES-DIAGNOSTICS.md) |

**On any rule violation — detected before, during, or after executing a command:**
1. Halt the current action immediately.
2. Self-correct autonomously — do not ask the user.
3. Retry with the correct approach.
4. **Do not move past the failed step until it is verified resolved.** If retry fails, diagnose further (Rule 7). Only escalate to the user when introspection cannot resolve it.
5. Report the correction in one line: what rule was about to be violated, what was caught, and what was corrected instead.

**User override is immediate and absolute.** If the user flags a violation or demands a halt, stop everything, correct, then explain — never defend first. User feedback is actioned before any other task.

**Treating these rules as guidelines is itself a critical violation.** "I defaulted to legacy habits" and "I improvised instead of following the workflow" are not acceptable explanations. The rules exist precisely to override legacy habits and improvisation.

**On any identified violation — by the agent or flagged by the user — report the root cause clearly and precisely.** The response to a violation is not to log it: it is to identify which rule was insufficient or absent, and harden that rule immediately so the same failure cannot recur.

---

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
| Installed packages | `pkg list` |
| Package install location | `pkg prefix <package>` |
| Package executables | `pkg executables <package>` |
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

3. **The motion goal uses a vague quantity word:** Words like "a bit", "slightly", "a little", "nearby", "close to", "a short distance" provide no exact target. Use the conservative safe defaults below — they are calibrated to be safe on any platform:

   | Vague word or phrase | Default |
   |---|---|
   | "a bit", "slightly", "a little", "a tad", "just a touch" | 0.1 m linear / 5° rotation |
   | "a short distance", "a little further", "a bit more" | 0.3 m |
   | "nearby", "close to here", "not far", "a modest distance" | 0.5 m |
   | "a fair distance", "somewhat far", "a good distance" | 1.0 m |
   | "turn slightly", "rotate a bit", "yaw a little" | 5° |
   | "turn some", "rotate a moderate amount" | 15° |

   After executing, **note the assumption in the one-line report**: *"Moved 0.1 m (interpreted 'a bit' as 0.1 m — say 'more' to continue)."* The user can then issue a correction or continuation command without being interrupted mid-task.

   **Exception:** "Move forward" or "drive forward" with no qualifier at all is an open-ended command — execute with `publish-sequence` per Rule 3. The vague-quantity defaults apply only when a quantity-implying word is explicitly present.

If none of these conditions apply: **just do it.**

**Rule 0 and Rule 5 interaction — discovery is silent, execution is silent:**
Rule 0 mandates exhaustive introspection before acting. Rule 5 mandates acting without asking or narrating. These do not conflict: Rule 0 discovery runs in parallel and silently (the user never sees it). Rule 5 governs the execution phase — do not narrate steps, do not ask for permission to execute, do not report intermediate discovery results. Only the final outcome is reported. "Discovery + silent execution" is the expected pattern for every command.

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
- A list of upcoming discovery steps ("Proceeding to: * Discover X * Discover Y …") — run them silently and report only the outcome
- Any preamble label like "Strict compliance:", "Executing:", "Running:", or similar — never open a response with a status label
- References to the tool being used — never say "using ros2-skill", "via ros2-skill's X utility", "using the ros2-skill tool", or any equivalent; the user knows what tool is in use

**Always report (even in minimal mode):**
- Self-corrections: if a rule violation was caught mid-execution, report it in one line — what was about to be violated, what was caught, and what was done instead. Example: *"Caught: was about to use `launch start` (hallucinated subcommand). Corrected to `launch new`. Retrying."*

**Report everything (verbose mode) only when the user explicitly asks** — e.g. "show me what topics you found", "give me the full details", "what type did you use?"

### Rule 10 — Empty discovery: broaden the search, never guess

When `topics find <type>` returns an empty list:

1. **Do not guess a topic name.**
2. **Do not fall back to a hardcoded name** (Rule 0).
3. **Retry once before broadening.** ROS 2 DDS discovery is eventually-consistent — a node that started seconds ago may not yet be visible to the daemon. Wait 1 s and re-run the exact same `topics find` command. If the second call returns a result, use it. If still empty, proceed to step 4.
4. **Broaden the search immediately and in parallel:**
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

2. **If context resolves to exactly one candidate:** use it. State the selection in the report whenever more than one candidate existed.
3. **If no context keywords apply, or multiple candidates remain after keyword matching:** use the first result. Always state the selection in the report when more than one candidate existed. Do not ask the user (Rule 5).

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

**Mid-motion node crash monitoring — mandatory for commands with expected duration > 10 s:**
For any `publish-until` or `publish-sequence` whose calculated timeout exceeds 10 s, monitor the node graph during execution:
- Every 10 s (at each segment boundary if using long-motion segmentation from Rule 9), run `nodes list` in the background.
- Identify the critical nodes: the velocity controller node and the odom publisher node (both discovered during Rule 0 pre-flight).
- **If either critical node disappears from `nodes list`:** immediately send `estop`, verify it took effect, then escalate: *"Critical node `<node_name>` crashed during motion at position X. Robot stopped. Cannot resume motion until node is restarted."* Do not attempt to continue motion.
- Short commands (≤ 10 s): node-crash monitoring is optional. The command will timeout before the periodic check adds meaningful safety.

**If the graph changes unexpectedly mid-task** (a node disappears, a controller deactivates without being told to, a topic that was publishing goes silent):
1. Stop the current task.
2. Re-run the full Rule 0.1 session-start checks (`doctor`, clock check, lifecycle state).
3. Re-discover all names before continuing — the system state is unknown.
4. Report to the user: what changed, what the checks found, what the impact is.

A mid-task graph change means something crashed or was preempted. Continuing with cached state means continuing blind.

**Never say "I already discovered this earlier" as a reason to skip introspection.** Discovery takes under a second. Stale assumptions cause hard-to-debug failures.

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
