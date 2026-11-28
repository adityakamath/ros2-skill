# Motion Rules

Load before any command that moves the robot (cmd_vel, joints, Nav2). Related: [RULES-PREFLIGHT.md](RULES-PREFLIGHT.md), [RULES-DIAGNOSTICS.md](RULES-DIAGNOSTICS.md).

`CLI` below means `python3 ${CLAUDE_SKILL_DIR}/scripts/ros2_cli.py`.

---

## Rule 3 — Movement algorithm

1. **Velocity topic and type.** Path A: `summary.cmd_vel_topic` and the matching `summary.velocity_topics[].type`. Path B: `topics find geometry_msgs/msg/Twist` and `.../TwistStamped` in parallel, then `topics type`. Run `interface proto <VEL_TYPE>` once per session; never build the payload from memory. Never assume `/cmd_vel`.
2. **Odometry topic.** Path A: first value of `summary.localization_config.fused_sources`. Path B: `topics find nav_msgs/msg/Odometry`. Check the rate with `topics hz <ODOM> --duration 2`:
   - ≥ 5 Hz: closed loop.
   - 1–4 Hz: too slow. Warn, use open loop.
   - 0 Hz or timeout: treat as absent.
   - TF can answer position queries (`tf lookup`) but cannot drive `publish-until`.
3. **Method.** Closed loop (`publish-until --monitor <ODOM>`) is mandatory when odometry is usable, even for "simple" moves. `publish-sequence` is the fallback only when odometry is unavailable. Tell the user it is open loop and not accurate. Open-ended moves use `publish-sequence` with a zero payload as the last message.
4. **Execute** with discovered names only, always with `--max-vel` / `--max-ang` set to the binding safety limits:
   ```bash
   CLI topics publish-until <VEL> '<payload>' --monitor <ODOM> --field pose.pose.position --euclidean --delta <N> --timeout <T> --max-vel <V> --max-ang <W>
   CLI topics publish-until <VEL> '<payload>' --monitor <ODOM> --rotate <±deg> --degrees --timeout <T> --max-vel <V> --max-ang <W>
   ```
   - Rotation always uses `--rotate` (there is no `--yaw`). Positive is CCW and negative is CW. The sign of `angular.z` **must match** the sign of `--rotate`, or the command times out.
   - Timeout is calculated, never a default. Linear: `ceil(2·N/v)` s. Rotation: `ceil(2·(θ·π/180)/ω)` s. Minimum 15 s.

## Rule 9 — Pre-motion check (before every motion command)

- **Combined check:** `preflight motion --controller <NAME> --odom-topic <ODOM> [--source teleop|nav2]`. It confirms the controller is active and the robot is stationary in one spin, and when the profile knows them it also reads the e-stop state, the twist-switch drive mode and the battery. Read `ready_for_motion` and, when false, `blocked_by` (each entry says why and how to fix it). `--source` is the velocity input the command uses: `teleop` for `topics publish*`, `nav2` for `nav2 go|rotate|mission|place go`. With a twist switch, a command on the unselected input is silently ignored (a `nav2 rotate` in teleop mode just times out).
- **Stationary:** |linear.x|, |linear.y| and |angular.z| are all < 0.01. If any is higher, `estop`, verify it, wait 0.5 s, re-read. If the estop does not take effect, stop and escalate; do not move.
- **Baseline pose:** record a fresh `pose.pose.position` and orientation from this reading for post-motion comparison. Never use a cached pose. For sequential moves, the previous move's confirmed final pose may serve as the baseline, but the velocity and odom-rate checks run fresh every time.
- **Controller or velocity-node absent:** halt, run `doctor`, and do not proceed until it is back.
- **Segmentation:** if the estimated duration is over 30 s, split into ≤ 30 s segments. Between segments run `estop`, verify it, run `topics hz <ODOM> --duration 1`, and re-baseline.
- **Nav2 preemption:** run `nav2 status`. If `active_goal` is non-null, run `nav2 cancel` before any velocity command or new `nav2 go`. Nav2 overwrites `/cmd_vel` every cycle, so a concurrent command is silently ignored. Without nav2 commands, use `actions list` and `actions cancel <goal_id>`.
- Never issue a new motion on top of an existing one without stopping first.

## Rule 17 — REP-103 / REP-105

- SI units everywhere (m, m/s, rad, rad/s, s). `--degrees` is a CLI convenience only. `angular.z` is always rad/s.
- Axes: x forward, y left, z up. `linear.y` works only on holonomic drives (`summary.drive_type`); a differential drive ignores it silently.
- Quaternions are `(x, y, z, w)`. Flat-ground yaw is `2·atan2(q.z, q.w)`, valid only when q.x ≈ q.y ≈ 0.
- Frames are `map → odom → base_link`:
  - Odometry is in `odom`, so use it for relative displacement and `publish-until`.
  - Absolute position needs `tf lookup map base_link`.
  - Take names from the profile or `tf list`; they may be namespaced.
- Before using any sensor's spatial output, confirm its `frame_id` is in `tf list` and fresh (`tf echo <SENSOR> <BASE> --duration 1`, or `tf monitor`). If `tf` hangs, suspect a cycle and run `tf validate`; halt spatial work if it reports one.

## Rule 18 — stop after publish-until / publish-sequence

After a normal completion, or a move the obstacle stop ended or refused (`stopped_by`), run `stop --verify-odom-topic <ODOM>`: it zeroes every velocity topic, waits until the robot is stationary and reports the final position. It never latches the e-stop service. After a timeout, an error or anything unsafe run `estop` instead. `plan --run` does both automatically.

`estop` does three things: it interrupts a running motion command (SIGINT, so that command's own zero burst runs), engages the robot's idempotent SetBool e-stop service when one exists (`summary.estop_config`; never a Trigger toggle, which would release on a second call), and zeroes **every** velocity topic (zeroing one input does nothing while another source is selected). A service e-stop disables motor torque, so wheels coast. Releasing it (`services call <svc> '{"data": false}'`) is the operator's decision. Afterwards run `nav2 cancel` if navigating; it cancels every Nav2 action server (navigate, waypoints, spin, backup, wait), not just NavigateToPose.

On every failed exit (timeout or error), `estop` is the first action, before diagnosing, reading odom, reporting or retrying; on a normal exit it is `stop`. Then verify that all velocity axes are < 0.01 within 5 s (10 s above 20 kg). Otherwise report a critical failure.

- The CLI sends a zero-velocity burst in a `finally` block, but this is best effort. SIGKILL or a crash skips it.
- The CLI has no SIGTERM handler. If a motion command's process dies abruptly, treat the robot as possibly still moving and `estop` first.

## Rule 20 — Deceleration zone

`publish-until` auto-computes the slow-down zone (`slow_last`, `slow_factor`) from the commanded speed with default accel and minimum speed (0.5 m/s², 1.0 rad/s², floor 0.125 m/s / 0.375 rad/s). If `braking measure` has been run, the stored deceleration replaces the default (`params_source: measured braking`). With a profile loaded it does not sweep live node parameters (that cost ~2 s and finds nothing on most robots); pass `--slow-last` / `--slow-factor` to override. The values are reported under `decel_zone`. Do not pass `--slow-last` unless overshoot was observed. If it overshoots, increase `--slow-last` or decrease `--slow-factor` and report the values used.

## Rule 21 — After a publish-until timeout

1. `estop` and verify it (Rule 18).
2. Read odom once and compare with the baseline to get the remaining distance or angle.
3. If it is within tolerance (≥ target − 0.05 m, or − 3°), report success.
4. Otherwise re-issue **only the remainder**, never the original full command. If odom is unavailable, do not re-issue.
5. Two consecutive timeouts on the same move: escalate with position, target, remaining, and the suspected cause (QoS, odom dropout, controller).

## Rule 22 — Size limits

Reject without explicit user confirmation: linear > 50 m or rotation > 3600° in one command. Execute and state the assumption for 10–50 m or 360–3600° (and use a decel zone for multi-turn). The limits do not apply to open-loop `publish-sequence` bounded by durations.

## Rule 23 — New command while moving

The CLI enforces one motion command at a time (a lock file): a second motion command is refused with `blocked: true` and the owner's pid while the first runs.

`estop` and verify before handling anything new, even when the robot was nearly done. A "stop/halt" request is a direct `estop`. Never run two motion commands in parallel.

## Rule 24 — Branching and retries

| Situation | Autonomous retries | Then |
|---|---|---|
| `publish-until` timeout | 1 (remainder only) | escalate |
| Stopped for an obstacle (`stopped_by`) | 0: never retry or drive around it unasked | report what was there |
| Verification failure | 2 | escalate as critical |
| Empty discovery | 1 (broadened) | ask or declare unavailable |
| Other step failure | 1 | escalate |
| Safety failure (estop, controller offline) | 0 | escalate immediately |

Known fallbacks run without asking, but the user is told what failed and the accuracy cost (e.g. closed loop → open loop when odom or QoS fails). Escalation is one message: what was tried, what failed, state (position, controller), and one recommended next step. Never present a menu of options.

## Rule 25 — Obstacle stop

Every base command with a linear velocity is guarded: `plan --run`, `topics publish-until`, `publish`, `publish-sequence` and `nav2 move-timed`. Nothing extra to call: do not subscribe to a sensor to decide whether to start or when to stop, and never approximate a stop with `publish-until --above` / `--below` (one field, such as a single LiDAR beam) or a subscribe-then-publish loop.

- **What is watched** comes from the profile (`obstacle_stop` in the loaded context): every LaserScan beam, every point of each PointCloud2 and the whole depth image, against the path the commanded velocity sweeps (straight, strafe or arc) with the robot's radius; and a running Nav2 collision monitor (its stop state, or it cutting the command to under half). Sensors that are not publishing are listed in `sources_unavailable`. Clouds in a `map` or `odom` frame are processed data and are not used.
- **Behaviour:** no motion unless a fresh sensor covers the direction of travel; stop at `clearance` (0.15 m beyond the footprint, plus reaction and braking distance); stop when **any** sensor that was covering the move goes quiet or stops covering it, even if another still looks that way (with another watching, it gets 1-2 s for dropped frames; the only watching sensor gets none beyond its own period). Something a sensor last saw on the path is still counted: if the distance driven since then reaches it, the move stops (`obstacle` with `estimated: true`).
- **Covers** means: a LaserScan with no blind patch wider than 15° within 30° of the direction of travel (NaN, 0 and returns on the robot itself are blind; dark or glass surfaces can cause such a patch and a refusal); a point cloud with at least 5 returns there, on both sides; a depth image from a roughly level camera with valid depth near the horizon in every 5° slice up to 10° either side. Floor alone, zeros or invalid pixels are not "clear".
- **Single and chained messages:** a guarded `topics publish` without `--duration` / `--timeout` watches for 0.5 s and then zeroes the velocity (use `publish-until` or a duration to keep moving). `publish-sequence` checks every step; the base holds at zero while a step waits for its sensor, and the sequence ends at a refused step (`stopped_at_index`).
- **Driving until an obstacle:** `plan "drive forward until obstacle" --run` (also "back up until 30 cm from the wall", "strafe left until obstacle up to 2 m"; 10 m limit unless given). Directly: `publish-until ... --euclidean --delta <MAX> --until-obstacle [--obstacle-stop <CLEARANCE>]`; success means an obstacle was reached.
- **Results:** `stopped_by` = `obstacle` | `collision_monitor` (success with `--until-obstacle`, otherwise a failed move), `max_distance` (limit reached, nothing met), `sensor_lost` / `no_sensor` (failure: report which sensor and why). `obstacle` gives `distance_m` along the path, `bearing_deg` and `source`. Report `sources_used`, and any source that was unavailable.
- **Blocked** (`blocked: true`, nothing moves): the profile names no usable range sensor (no profile, nothing listed, or everything ignored), or no footprint radius. First run `profile context` with the sensors running. Otherwise ask the user for the missing fact and store it: `profile set-obstacle --radius M`, `--add-source scan|pointcloud|depth:TOPIC` for a sensor the scan missed (also `--max-height M`, `--ignore-source TOPIC` for a sensor that sees the robot itself). `--no-sensors` records that a robot has none, which lets it drive unguarded: only on the user's word. Never guess a value to get past a block.
- `--no-obstacle-stop` only when the user explicitly asks to drive without obstacle detection (or to push against something).
- **Limits, say them when they matter:** a 2D LiDAR sees one height only; a camera sees only its field of view and has a near blind zone; glass and dark or thin objects may return nothing; coverage is judged around the starting direction of an arc; returns from the robot's own body just outside its radius can stop a move (`--ignore-source`); the age of sensor data is judged by arrival, not by its header stamp; turning on the spot and Nav2 goals are not covered (Nav2 has its own costmaps); it is not a safety-rated stop and does not replace the e-stop or watching the robot.

## Cancel vs estop

| Situation | Action |
|---|---|
| Graceful abort of a running goal | `actions cancel` |
| Robot moving unsafely, goal server dead, or coasting after the goal ended | `estop` (then `actions cancel` to clear goal state) |
| Goal rejected or timed out, robot still | `actions cancel` |

When in doubt, `estop` first.

## Motion failure recovery

- A `publish-until` timeout means odometry never reached the delta. It does not mean the command is unavailable. Run `estop`, read odom, run `topics hz <ODOM>`, run `control list-controllers`, then report actual distance, odom status and controller state.
- If odom stops updating mid-motion, `estop` and do not keep publishing, because that is a runaway risk. Check `topics details <ODOM>` and `topics hz`.
- `Could not detect message type`: the topic has no publisher yet. Check `topics details`, wait, or pass `--msg-type`.

## Moves hosted by the fast daemon

With `daemon-fast` running, `topics publish-until` (euclidean or `--rotate`), `stop` and `preflight motion` are served inside the daemon, which keeps the publisher and odometry subscription warm. The same limits apply (checked in the CLI and again in the daemon), and halting works from any side:

- the daemon publishes only while the CLI's socket is open: Ctrl-C, `kill`, `kill -9` or a crashed CLI all end the move and send the zero burst;
- `estop` and `stop` cancel the running move first;
- if the daemon itself is killed, publishing stops and the base controller's `cmd_vel_timeout` (0.5 s on LeKiwi) ends the motion: measured, the base is stationary about 1 s after a `kill -9` of the daemon at 0.1 m/s;
- a request the daemon may have received is never retried locally (no double move): the CLI reports `cancelled` and zeroes velocity. A request it never received runs in-process as before.

Guarded moves are hosted too: the daemon keeps the range sensors and TF subscribed (large clouds are dropped after 60 s unused), so the obstacle stop adds no start-up step. A daemon from an older version of the skill refuses a guarded move; the CLI then runs it in-process and restarts the daemon.

Conditions other than a distance or rotation (`--above`, `--below`, `--equals`) and `--dry-run` always run in-process.
