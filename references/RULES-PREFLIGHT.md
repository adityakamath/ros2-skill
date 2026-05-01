# ROS 2 Skill: Pre-flight & Introspection Rules

> **This file is part of a split rule set.** The full rule set spans five files:
> - [RULES-CORE.md](RULES-CORE.md) — general agent conduct, applies to every command
> - **RULES-PREFLIGHT.md** ← you are here — before every action: introspection, session-start, hardware readiness
> - [RULES-MOTION.md](RULES-MOTION.md) — motion workflow: discover → execute → estop → verify
> - [RULES-DIAGNOSTICS.md](RULES-DIAGNOSTICS.md) — when things go wrong: diagnose, verify, escalate
> - [RULES-REFERENCE.md](RULES-REFERENCE.md) — lookup tables: intent→command, launch workflow, setup

---

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
| Move a robot | Full Movement Workflow — see Rule 3 (RULES-MOTION.md) and the canonical section below |
| Read a sensor / subscribe | `topics find <msg_type>` to discover the topic; `topics type <topic>` to confirm type; never subscribe to a hardcoded name |
| Capture a camera image | `topics find sensor_msgs/msg/CompressedImage` and `topics find sensor_msgs/msg/Image` to discover available camera topics; use the result in `topics capture-image --topic <CAMERA_TOPIC>` |
| Use a camera image or depth image (any visual pipeline) | Before using the camera data: 1. Find the paired `camera_info` topic: `topics find sensor_msgs/msg/CameraInfo` — the result will share a namespace with the image topic (e.g., `/camera/camera_info` pairs with `/camera/image_raw`). 2. Subscribe to confirm calibration is present: `topics subscribe <CAMERA_INFO_TOPIC> --max-messages 1 --timeout 2` — verify `K` (intrinsic matrix) is non-zero. 3. Read the `header.frame_id` from the `camera_info` message; confirm it is present in `tf list`. **A camera with no `camera_info` publisher is uncalibrated. A camera whose `frame_id` is absent from TF will produce wrong spatial results.** Both conditions must be satisfied before using the camera for **any camera-dependent task** — including captures, streaming, object detection, depth estimation, object localisation, and point cloud processing. Do not skip this check even for "simple" image captures — an uncalibrated camera produces no calibration data for downstream use, and a misaligned frame produces wrong spatial results silently. |
| Use a sensor topic whose data will be interpreted spatially (camera, LiDAR, IMU, depth, GPS, sonar) | Before consuming the data: 1. Subscribe to the topic for 1 message to read the `header.frame_id`. 2. Run `tf list` to confirm that `frame_id` is present in the TF tree. 3. Run `tf echo <SENSOR_FRAME> <BASE_FRAME> --duration 1` (where `<BASE_FRAME>` is the robot's base frame from `tf list`) to confirm the transform is actively updating and not stale. **Never use sensor data whose frame is absent from TF or has not been updated recently** — spatial computations on stale or missing transforms produce wrong results with no error. |
| Query a TF transform | `tf list` to discover available frames; never hardcode frame names like `map`, `base_link`, `odom` |
| Switch or load a controller | 1. `control list-controllers` to discover controller names and states (never hardcode).<br>2. `control list-hardware-components` to confirm the hardware component is in `active` state.<br>3. `control list-hardware-interfaces` to confirm the relevant hardware interfaces are `available/active`.<br>**Never load, switch, or configure a controller while the hardware component is `inactive` or `unavailable`, OR while the relevant hardware interfaces are not `available/active`** — in both cases the controller will load without error but will silently discard all commands. Block the operation and escalate: *"Hardware component/interfaces not active — cannot load or switch controllers until the hardware is active."*<br>Always use `--strictness STRICT` for `switch-controllers` unless explicitly instructed otherwise.<br>4. **If `control list-controllers` errors or times out:** run `nodes list` and look for a node whose name contains `controller_manager`. If absent, escalate immediately: *"Controller manager is offline — cannot command motion."* Do not attempt any controller operation until it is confirmed running.<br>5. **Multiple velocity controllers:** when `control list-controllers` returns more than one active controller that could handle velocity commands, filter by the robot part named in the user's request — "arm" / "manipulator" → prefer controllers whose name contains `arm` or `manip`; "base" / "drive" / "mobile" → prefer controllers whose name contains `base`, `mobile`, or `diff`. If the user's request gives no part context, use the first result (Rule 11) and note the choice. |
| Any operation involving a node | `nodes list` first; never assume a node name |
| Load a composable node (`component load`) | 1. `component list` to discover available containers — **never hardcode a container name**.<br>2. Confirm the target container appears in the output before proceeding.<br>3. After loading: verify the new component appears in a follow-up `component list` and note the returned `unique_id` — it is required for `component unload`. |
| Unload a composable node (`component unload`) | 1. `component list` to discover the current containers and their loaded components — **never hardcode a `unique_id`**.<br>2. Identify the correct `unique_id` from the list output.<br>3. After unloading: run `component list` again to confirm the component is no longer present. |
| Run a composable node standalone (`component standalone`) | No pre-existing container needed — `component standalone` creates the container automatically. After running: `component list` to confirm the container and component are visible. Kill the session with `component kill <session>` when done. |
| Set a parameter | 1. `nodes list` to discover the node name<br>2. `params list <node>` to discover the parameter name (never assume it)<br>3. `params describe <node:param>` to confirm type (int/float/bool/string), valid range, and read-only status — **never set a parameter without this; wrong type silently truncates or is rejected**<br>4. `params set <node:param> <value>` with value matching the confirmed type; verify with `params get` after (Rule 8) |
| Get / list / dump parameters | `nodes list` to discover the node name; `params list <node>` to discover parameter names |
| Load parameters from a YAML file (`params load` or `--params-file` in launch) | 1. `nodes list` to confirm the target node is running.<br>2. `params list <node>` to get the current parameter names on that node.<br>3. Compare YAML file keys against the discovered names — **any YAML key that does not match a declared parameter is silently ignored; no error is raised.** Verify every key you intend to set is present in `params list` output before loading.<br>4. For each YAML key you will set, `params describe <node:param>` to confirm type — a YAML string loaded into an integer parameter silently fails or truncates.<br>5. After loading: `params get <node:param>` on each key to verify values were applied (Rule 8). |
| Run a servo / control-loop task (any node publishing velocity commands at ≥ 10 Hz) | Check CPU scheduling policy for the control node: `chrt -p $(pgrep -f <NODE_BINARY_NAME>)` — if output shows `scheduling policy: SCHED_OTHER`, the node lacks real-time priority and may exhibit jitter or periodic latency spikes under load. Optionally check isolated CPUs: `cat /sys/devices/system/cpu/isolated` — if empty, no cores are isolated for real-time use. Both checks are **advisory only**: report findings in the pre-task summary; do not block the task. Use the binary name from `pkg executables <package>` — not the ROS node name (e.g. use `controller_manager` not `/controller_manager`). |

**Parameter introspection is mandatory before any movement command.** Velocity limits can live on any node — not just nodes with "controller" in the name. Before publishing velocity, sweep **all four limit sources** in parallel:

**Source 1 — Node parameters (all nodes):**
1. Run `nodes list` to get every node currently running.
2. Run `params list <NODE>` on **every single node** in the list (run in parallel batches if there are many).
3. For each node, look for any parameter whose name contains `max`, `limit`, `vel`, `speed`, `accel`, or `scale` (case-insensitive). These are candidates for velocity limits. The `scale` keyword catches teleop nodes that publish `scale_linear`, `scale_angular`, or `max_vel_*` params — these define the operator's commanded ceiling and must be respected.
4. Run `params get <NODE>:<param>` for every candidate found across all nodes.

**Source 2 — URDF joint limits (if `robot_state_publisher` is running):**
5. Check whether a node with `robot_state_publisher` in its name exists in `nodes list`. If it does, run `params get <RSP_NODE>:robot_description` to retrieve the URDF XML. Parse the URDF for any `<limit velocity="..."/>` and `<limit acceleration="..."/>` entries on joints relevant to the requested motion type (wheel joints for linear/angular base motion, arm joints for arm commands). Extract the minimum velocity limit across all relevant joints and treat it as a binding ceiling alongside Source 1.
6. **If `robot_description` is not a parameter on the node** (some configurations use `robot_description_semantic` or load it from a file): skip URDF parsing and proceed with Sources 1, 3, 4 only.

**Source 3 — ros2_control hardware interface limits:**
7. Run `control list-hardware-interfaces` and inspect the output for commanded interfaces (those listed under `[claimed]` or with `velocity command` in their description). If the controller manager or hardware nodes expose per-joint limit parameters (commonly named `<joint>/limits/max_velocity`, `<joint>/limits/max_acceleration`, or similar), retrieve them via `params list` + `params get` on the `controller_manager` node and any hardware plugin nodes. If no such params exist, skip — this source is optional.

**Source 4 — Hardware component YAML limits (via controller_manager params):**
8. Run `params list /controller_manager` and look for any parameter matching `<joint>.*limit.*` or `<joint>.*max.*`. These often come from YAML robot configuration files loaded at launch via `--params-file` and land as regular parameters on the controller_manager node. Retrieve each candidate with `params get`.

**Binding ceiling = the minimum across all values discovered in Sources 1–4.**
9. Identify the binding ceiling: the **minimum across all discovered linear limit values** and the **minimum across all discovered angular/theta limit values**.
10. Cap your commanded velocity at that ceiling. If no limits are found across all four sources, use conservative defaults (0.2 m/s linear, 0.75 rad/s angular) and note this in the report.

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
- `ROS_DOMAIN_ID` defaults to `0` — this is correct and expected. The user may configure a different domain ID by setting the `ROS_DOMAIN_ID` environment variable before launching the skill; always respect whatever value is set. If `nodes list` or `topics list` returns an unexpectedly large or irrelevant set of names, the issue is likely an **unintended domain collision** — another system (host OS or another container) is sharing the same domain ID without intent. In container-alongside-host deployments, both sides often default to `0` and see each other's nodes — this is the most common container misconfiguration and is not the user's fault; it is a deployment issue. Verify the domain ID in use with `echo $ROS_DOMAIN_ID` and confirm with the user before proceeding if the graph looks unexpectedly large.
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

**Step 4 — Note the log directory (no command required; for diagnostic reference):**
ROS 2 writes one log file per node for the current run. Resolve the log directory in this order:
1. `$ROS_LOG_DIR` (if set)
2. `$ROS_HOME/log/` (if `ROS_HOME` is set)
3. `~/.ros/log/` (default)

Store this path. When diagnosing failures or silent node behaviour, individual node log files are here and can be read directly — even without a live graph. When the `logs` command group is available, `logs list-runs` makes recent runs discoverable at session start.

**Step 5 — Capture a compact graph snapshot (`context`):**
```bash
python3 {baseDir}/scripts/ros2_cli.py context
```
Returns topics, services, actions, and nodes in a single call. Use this to pre-load graph state at session start rather than running four separate discovery commands. Topics are capped at 50 by default (`--limit 0` for unlimited). Store the output; reference it during task planning to avoid redundant discovery round-trips.

**These checks are session-level.** Do not re-run for every command. Re-run only if the user relaunches the robot or if nodes appear/disappear unexpectedly.

**Exception — simulated time re-check before every timed command:**
Step 2 (simulated clock check) is the one session-level check that must be **repeated before every timed operation** (`publish-until`, `publish-sequence`, `topics subscribe --timeout`, etc.). A simulator can pause or reset at any point. Before any timed command, run:
```bash
python3 {baseDir}/scripts/ros2_cli.py topics subscribe /clock --max-messages 1 --timeout 2
```
If no message arrives (even though the clock topic exists): the simulator is paused. **Do not issue any timed command** — all timeouts will fire instantly or block indefinitely depending on whether ROS time is forwarding. Escalate: *"Simulator clock is not advancing. Cannot issue timed commands until the simulator is unpaused."* This check adds under 2 s to every motion pre-flight and prevents the most expensive class of sim-based failures.

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

**Odometry `frame_id` check — run once per session when odom topic is first used:**
After confirming a publisher exists on `<ODOM_TOPIC>`, subscribe for one message and read `header.frame_id`:
```bash
topics subscribe <ODOM_TOPIC> --max-messages 1 --timeout 5
```
Inspect the returned `header.frame_id` value:
- **`odom`, `odom_combined`, `robot_odom`, or any name containing `odom`**: canonical frame — proceed normally.
- **Anything else** (e.g., `world`, `map`, `base_link`, a custom name): note to the user once and proceed: *"Odometry is published in frame `<frame_id>` rather than the canonical `odom` frame. Position values are relative to `<frame_id>`."* This does not block motion but affects how position deltas should be interpreted.
- **Empty `frame_id`**: flag as a configuration problem: *"Odometry `frame_id` is empty — the publisher may be misconfigured. Position reporting may be unreliable."*
Store the `frame_id` value for the session. Use it when reporting positions and when querying TF transforms.

Before publishing to a topic intended for a subscriber:
1. Run `topics details <topic>` — check `subscriber_count`.
2. **If `subscriber_count == 0`:** there is no node listening. Still publish (the subscriber may be transient or latched), but report: *"No subscribers detected on `<topic>` — command may not reach any controller."*

**This check costs one CLI call and prevents the most common cause of subscribe timeouts.** Run it as part of any workflow that depends on receiving a message within a timeout window.

### Rule 19 — Verify QoS compatibility before `publish-until` (and before any subscribe)

`publish-until` creates a `ConditionMonitor` subscriber internally. If the odometry publisher uses `BEST_EFFORT` reliability and the subscriber uses `RELIABLE`, messages are silently discarded — the condition is never met, the loop runs to timeout, and the robot coasts (Rule 18).

**Pre-`publish-until` QoS check — this must happen BEFORE issuing `publish-until`:**

```bash
topics details <ODOM_TOPIC>
```

Inspect the output, then apply the hard gate:

| Result | Action |
|---|---|
| `publisher_count == 0` | **Do not attempt `publish-until`.** No odom publisher exists — fall back to open-loop immediately. |
| `reliability: BEST_EFFORT` | `publish-until` auto-matches QoS (M5 fix). Proceed. Verify `condition_met: true` in the output afterward. |
| `reliability: RELIABLE` or absent | Proceed with closed-loop — the subscriber will match. |
| QoS mismatch detected and auto-matching fails | **Do not attempt `publish-until`.** Fall back to `publish-sequence` immediately. Notify the user: *"Odometry QoS incompatible. Running open-loop — accuracy not guaranteed."* |

**Never attempt `publish-until` first and discover a QoS mismatch at timeout.** The gate is pre-flight, not post-hoc.

**Monitor field validation — verify the `--field` path before starting the loop:**

Before issuing `publish-until --monitor <TOPIC> --field <PATH>`, subscribe once to the monitor topic and confirm the field exists:

```bash
topics subscribe <MONITOR_TOPIC> --max-messages 1 --timeout 3
```

Inspect the JSON output and trace the full dotted field path (e.g. `pose.pose.position.x`). If the field is absent — either the path is wrong, the message type differs from expectation, or the field uses a different name — `publish-until` will run to timeout without ever evaluating a condition, and the robot will coast to its timeout limit at full speed.

**Common mistakes:**
- `twist.linear.x` vs `twist.twist.linear.x` (Odometry wraps Twist in a second `twist` field)
- `pose.position.x` vs `pose.pose.position.x` (PoseWithCovarianceStamped vs Odometry)
- `data` vs `range` vs `distance` (sensor message types vary)

If the field is not found in the subscribed message, stop and re-run `interface show <MSG_TYPE>` to find the correct path before proceeding.

**This check is already required by Rule 15 (odom subscribe).** Treating it as a distinct pre-`publish-until` step ensures it is not skipped when the agent goes directly to motion without an explicit odom-subscribe step first.

**Distinguishing QoS failure from genuine timeout:** If `publish-until` returns `condition_met: false` on the first attempt and the cause is uncertain, run `topics hz <ODOM_TOPIC> --duration 2` immediately after estop. If the rate is 0 Hz → the odom publisher was not being received (QoS mismatch or publisher offline) — fall back to open-loop. If rate > 0 Hz → odom was flowing; the condition was genuinely not met within the timeout — apply Rule 21 (re-issue for remaining distance).
