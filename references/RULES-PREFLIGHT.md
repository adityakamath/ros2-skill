# Pre-flight & Introspection Rules

Load at session start and before the first action. Related: [RULES-CORE.md](RULES-CORE.md), [RULES-MOTION.md](RULES-MOTION.md).

`CLI` means `python3 ${CLAUDE_SKILL_DIR}/scripts/ros2_cli.py`.

---

## Rule 0 — Profile first, live for the rest

**0.0 Path selection (once per session).** Run `profile show` once.
- **Path A:** it succeeded and `summary` is non-empty. Static data comes from the profile and live calls cover runtime state only.
- **Path B:** it failed or `summary` is empty. Discover everything live.

The path never flips mid-session, except after a user-requested `profile scan` completes.

**0.0a Per-field presence.** A field is present if it is non-null and non-empty. If one field is absent, fall back to live discovery **for that field only**. Field names are exact; the profile omits fields rather than storing nulls. Use `profile show` to confirm a path before reading it.

**0.0b Profile and live graph disagree → stop.** If a profile value fails live (e.g. `cmd_vel_topic` is not in `topics list`, or a named controller is missing), do not search for a different name and do not rescan mid-task. Run `nodes list` and `topics list` once, then report: *"Profile field `<f>` = `<v>` but live reports `<symptom>`. Bringup likely isn't running or the profile is stale."* Wait for the user.

### Profile fast-path (Path A: zero live calls for these)

| Need | Profile field |
|---|---|
| Velocity topic | `summary.cmd_vel_topic` |
| Velocity type | `summary.velocity_topics[].type` (entry matching the topic) |
| Odometry topic | first value of `summary.localization_config.fused_sources` |
| Velocity ceiling | `summary.safety_limits.binding.{linear_x,linear_y,angular_z}` → `--max-vel` / `--max-ang` |
| Commanded teleop speed | `summary.teleop_limits.binding` |
| TF frames | `summary.tf_frames.{odom_frame,base_frame,map_frame}` |
| Active controllers | `summary.active_controllers` (runtime state still needs a live check) |
| E-stop service | `summary.estop_config.service_name` |
| Joint commands | `summary.joint_command_topics[]` = `{controller, topic, msg_type, joints, interface_name}`. The topic is always `/<controller>/commands`, the type is always `std_msgs/msg/Float64MultiArray`, and array order is `joints`. |
| IMU | `summary.imu_topics[]` (`/<controller>/imu`, `sensor_msgs/msg/Imu`) |
| GPS | `summary.gps_topics[]` (`/<controller>/gps/fix`, `NavSatFix`) |
| LiDAR | `summary.lidar_scan_topic` |
| Drive type | `summary.drive_type` |
| E-stop (idempotent) | `summary.estop_config` = `{service_name, service_type: SetBool, toggle_service}` |
| Velocity chain | `summary.velocity_chain` = `{teleop_input, nav2_input, output, mode_service}` (SetBool: true = Nav2 input) |
| Robot services | `summary.robot_services` (joystick-bound: `record_waypoint`, `save_map`, ...), `summary.toggles` (SetBool state services) |
| Fitted hardware | `summary.hardware_fitted` (from `hardware.yaml`) |
| Always fixed | `/diagnostics`, `/joint_states` |

The CLI itself refuses `topics find` / `services find` calls that duplicate these fields. `--ignore-profile` overrides this and is logged. Use it only for deliberate Path B debugging.

**Live calls that remain mandatory in Path A** (the profile cannot know runtime state):
1. Before motion: `preflight motion --controller <NAME> --odom-topic <ODOM>` (controller active and robot stationary in one spin).
2. Before a joint, pan-tilt or arm command: `preflight joint-command --controller <NAME> --joints <j1,...>`.
3. `interface proto <VEL_TYPE>` once per session (the profile has the type but not the field layout). Joint commands do not need it.

### Live introspection (Path B, or a field is absent)

Never guess names. The velocity topic may be `/base_controller/cmd_vel`, the type may be `TwistStamped`, and odometry may not be `/odom`.

| Action | Required discovery |
|---|---|
| Publish (any topic) | `topics find <type>` → `topics type` → `interface proto <type>` (run `interface show` recursively for nested non-primitive fields) |
| Call a service / send an action goal | `services|actions list` or `find` → `details` |
| Read a sensor | `topics find <type>` → `topics type` |
| Camera image | `topics find sensor_msgs/msg/CompressedImage` and `.../Image`, then `topics capture-image`. Camera topics are never in the profile. No calibration check per capture (see Rule 0.6). |
| Spatial use of sensor data (detection, depth, point clouds) | Read `header.frame_id`, confirm it is in `tf list`, and `tf echo <SENSOR> <BASE> --duration 1` is fresh. Cameras also need a `camera_info` topic with a non-zero `K`, verified once per camera launch (Rule 0.6). |
| TF query | `tf list`; never hardcode frame names |
| Switch controllers | `control list-controllers`; `list-hardware-components` and `list-hardware-interfaces` must be active/available (otherwise commands are silently discarded: block and escalate). Use `--strictness STRICT`. If the listing errors, check `nodes list` for `controller_manager`; if it is absent, escalate. With several velocity controllers, pick by the part named in the request ("arm" vs "base"), else take the first and say so. |
| Node, component, parameter work | `nodes list` first. `component load/unload`: `component list` first, note the `unique_id`, and verify afterwards. `params set`: `params list` → `params describe` (type, range, read-only) → `set` → `get` to verify. `params load`: YAML keys that don't match a declared parameter are **silently ignored**, so compare against `params list`, check types, then verify with `get`. |
| Servo loops (≥ 10 Hz publishers) | Advisory only: `chrt -p $(pgrep -f <binary>)`. `SCHED_OTHER` means possible jitter. Report it and do not block. |

**Velocity-limit sweep (only when `safety_limits.binding` is absent).** Take the minimum over all of these. If nothing is found, use 0.2 m/s and 0.75 rad/s and say so.
1. Every node's `params list`, filtered for `max|limit|vel|speed|accel|scale`, then `params get`.
2. URDF `<limit velocity>` from `robot_state_publisher:robot_description`.
3. `control list-hardware-interfaces` plus per-joint limit params.
4. `/controller_manager` params matching `<joint>.*(limit|max)`.

## Rule 0.1 — Session start (once per session)

1. **Environment:** check `ROS_DOMAIN_ID` (default 0). An unexpectedly large graph usually means a domain collision with another host or container. `ROS_LOCALHOST_ONLY=1` hides remote nodes. If doctor reports "daemon not running", use `daemon start` and re-run doctor.
2. `doctor --exclude-packages`. Stop and tell the user on critical failures.
3. Simulated time: `topics find rosgraph_msgs/msg/Clock`. If it exists, confirm it advances with `topics subscribe /clock --max-messages 1 --timeout 3`. **Repeat that subscribe before every timed command.** A paused simulator makes timeouts misbehave: escalate, do not issue timed commands.
4. Lifecycle: `lifecycle nodes`, then `lifecycle get` on each.
5. Log directory: `$ROS_LOG_DIR`, else `$ROS_HOME/log`, else `~/.ros/log` (for diagnostics, no command).
6. `context` for a one-call graph snapshot (topics capped at 50).
7. `profile show`. If there is no profile, run `profile scan --name <robot>` (one robot), `--packages a,b --name label` (several), or bare `profile scan`. Use `safety_limits.binding` as `--max-vel` / `--max-ang`; this makes the four-source sweep unnecessary.
8. Optional speedup: `daemon-fast start`. Without it the daemon starts itself in the background on the first `snapshot`, `look`, `preflight motion`, `stop`, `publish-until` or `plan --run` and exits after 30 idle minutes (`ROS2_SKILL_NO_AUTOSTART=1` prevents that).

Do not repeat these per command. Re-run only if the robot is relaunched or nodes appear or vanish unexpectedly.

## Rule 0.6 — Rescan after `launch new` (only if sources changed)

When the user reports that a `launch new` (or `run new`) they ran is up, check whether any profile source is newer than the profile:

```bash
find "<profile.workspace>/src" \( -name package.xml -o -name '*.launch.*' -o -name '*.yaml' -o -name '*.urdf' -o -name '*.xacro' -o -name '*.srdf' \) -newer "<profile_json>" -print -quit
```

Any output means one silent `profile scan` with the profile's original `name`/`packages`, then `profile show`. No output means skip. Skip the check if the launch failed or the profile was just rescanned. A missing `generated_at` at session start is reported once and nothing else.

**Camera calibration:** if the launch brought up a camera driver, check once that a `camera_info` topic exists and `K` is non-zero. Do not repeat it per capture or per spatial task, unless the camera session is relaunched or the driver crashes and restarts.

## Rule 0.7 — When `profile scan` is allowed

Only when the user asks, when no profile exists yet, or after a launch with the Rule 0.6 gate positive (one scan per launch). Never because it seems prudent, a field looks missing (use per-field fallback), a live call surprised you (escalate per 0.0b), or time has passed.

## Rule 14 — Lifecycle state

For lifecycle-managed nodes, `lifecycle get <node>` before using their interfaces. `unconfigured`/`inactive` nodes silently drop messages. Use `lifecycle set <node> configure` and then `activate`, and verify the state. `finalized` or `error` means do not use the node.

## Rule 15 — Publisher counts and QoS before subscribing

Before waiting on a topic, run `topics details <topic>`:
- If `publisher_count == 0`, do not subscribe (it will time out). Report it and diagnose with Rule 7.
- Check QoS compatibility. A BEST_EFFORT publisher and a RELIABLE subscriber deliver nothing; `topics qos-check <topic>` reports compatibility (`topics subscribe` and `publish-until` auto-match the publisher's reliability).
- If it is still silent, run `topics hz`.
- Before the first odom subscribe of a session, do this check and read `header.frame_id`. A name containing `odom` is canonical. Anything else: tell the user once the frame it's in. An empty `frame_id`: flag a publisher misconfiguration.
- When publishing, `subscriber_count == 0` still publishes but warns that no node is listening.

## Rule 19 — Before `publish-until`

1. `topics details <ODOM>`. If `publisher_count == 0`, fall back to open loop immediately. BEST_EFFORT is auto-matched. If matching fails, use `publish-sequence` and tell the user. Never find a QoS mismatch only at timeout.
2. Subscribe once to the monitor topic and confirm the `--field` path exists. Common mistakes are `twist.linear.x` vs `twist.twist.linear.x` and `pose.position.x` vs `pose.pose.position.x`. If it is missing, check `interface show <type>`. A wrong field makes the robot run to timeout at full speed.
3. After an uncertain `condition_met: false`, run `estop` and then `topics hz <ODOM>`. At 0 Hz the odom was not received (QoS or publisher down) and the move should fall back to open loop. At a normal rate the condition truly was not met, so apply Rule 21.
