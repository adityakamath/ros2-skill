# Guides

Load when the task needs one of these topics. Rules are in the `RULES-*.md` files.

`CLI` means `python3 ${CLAUDE_SKILL_DIR}/scripts/ros2_cli.py`.

---

## Robot capabilities

`profile scan|rescan` infers a `capabilities` section from what it finds (Nav2, cmd_vel, odometry, drive type). The loaded context lists the names; read the details with `profile show` only when a task needs them. Each capability has `inputs`, `constraints`, `maps_to` (the action or topic to use) and a `result_schema`.

| Capability | Requires |
|---|---|
| `navigate_to_pose`, `follow_waypoints` | Nav2 |
| `move_distance`, `rotate_to_heading`, `arc_movement` | cmd_vel + odometry |
| `strafe`, `diagonal_move`, `crab_walk` | `drive_type=holonomic_*` with `linear_y` |

Capabilities refresh only on `profile rescan` (Rule 0.7).

## Collision-aware velocity pipelines

Many robots route velocity through teleop input → optional mode switch → collision monitor → controller input. To find where a command is lost, test each stage in turn with `topics publish <topic> '<json>' --msg-type geometry_msgs/msg/TwistStamped --duration 2.0 --max-vel 0.1`:

1. **Controller input topic** (e.g. `/base_controller/cmd_vel`). It bypasses all safety and proves that the motors work.
2. **Collision-monitor input** (e.g. `/cmd_vel_presafety`). This tests collision detection without the mode switch.
3. **Teleop topic**, with the mode switch set for teleop (`services list`, then `services call <switch> '<json>'`; the signature varies by robot).

Trace the chain with `topics details <topic>` (publishers/subscribers) and `nodes list`. To tell blocked from clear directions, compare odom travel per direction (front is `linear.x > 0`, back is `< 0`, left is `angular.z > 0` or `linear.y > 0` if holonomic). Test `linear.y` on holonomic robots.

| Symptom | Likely cause |
|---|---|
| Every direction blocked | Mode switch not on teleop |
| Rotates but doesn't translate | Platform motion disabled or controller inactive |
| Commands ignored | `control list-controllers` shows the controller is not active |
| Moves less than commanded | Collision monitor scaling by obstacle proximity |

## Drive types

`summary.drive_type` is `differential`, `holonomic_omni`, `holonomic_mecanum` or `ackermann`. Differential: `linear.x` and `angular.z` only. Holonomic adds `linear.y` (strafe), diagonal (x+y) and crab-walk (x+y+z).

## Camera rotation

`topics capture-image` rotates using, in order: a `camera_rotation_overrides` match, then the URDF sensor-mount heuristic. The heuristic is often wrong or missing (vendor link names, REP-103 optical-frame rotations). If an image comes out rotated, record a one-time fix instead of retrying:

```bash
CLI profile set-camera-rotation <topic_substring> <0|90|180|-90> --note "<why>"
CLI profile annotate "<why>" --name <robot>
```

## Multiple profiles

`annotate` and `set-camera-rotation` default to `--name robot` and auto-detect only when exactly one `*_profile.json` exists. With several profile files, always pass `--name <robot_name>` (see `profile list`), or the command edits a `robot_profile.json` stub that can then hijack default-profile selection.

## Safety policy

Stored in `.presets/safety.json` and enforced before `topics publish`, `publish-sequence`, `publish-until` and `nav2 go`. Defaults (when no file exists): velocity limits on and rejecting (0.5 m/s linear, 1.0 rad/s angular); geofence, command filter and heartbeat off.  Use `safety disable` / `safety enable` to turn checks off and on; there is no per-command bypass.

```bash
CLI safety show
CLI safety set-velocity --linear 0.5 --angular 1.0 [--axis linear_x_max --value 0.3] [--topic <cmd_vel>]
CLI safety set-geofence --mode circle --cx 0 --cy 0 --radius 10     # or --mode rectangle --xmin .. --xmax .. --ymin .. --ymax ..
CLI safety geofence enable|disable
CLI safety block topics publish        # blocklist a command; `unblock` removes it
CLI safety validate --topic <cmd_vel> --msg '<twist_json>' [--msg-type <type>]   # dry run
CLI safety enable|disable
CLI safety reset --yes
CLI safety heartbeat start [--timeout 5] [--poll 1]   # tmux watchdog; estops if no ping within timeout
CLI safety heartbeat ping|status|stop
```

The heartbeat is for remote-control cases where the operator's connection may drop: something must `ping` at least once per timeout.

## Missions

File-backed missions run detached and survive across calls (state in `.artifacts/missions/<id>.json`). If the state file disappears or becomes unreadable mid-run, the runner cancels Nav2 and sends a zero-velocity burst.

```bash
CLI nav2 mission run --name patrol --steps '[{"type":"go","x":1.0,"y":0.0,"yaw":0},{"type":"rotate","degrees":90},{"type":"go","x":0.0,"y":1.0}]'
CLI nav2 mission run --file steps.json [--mission-id <id>]
CLI nav2 mission status|pause|resume|cancel <mission_id>
```

`mission cancel` signals the runner, so the step in progress is interrupted at once (its Nav2 goal is cancelled and velocity zeroed); `pause` takes effect after the current step.

Step `type` values: `go`, `rotate`, `move_timed`, `initial_pose`, `localize`, `costmap_clear`, `map_load`, `mode_set`. The other keys in a step are that command's arguments (`CLI nav2 <type> --help`). Flow: queued → running → paused → completed, failed or canceled.

## Named places and maps

```bash
CLI nav2 place save <name> [--x X --y Y --yaw DEG] [--snapshot]   # without coordinates, samples the current pose; --snapshot stores a camera frame
CLI nav2 place list | go <name> [--feedback] | delete <name>
CLI nav2 map list|save|load|delete
CLI nav2 mode get|set mapfree|mapping|navigation
```

Places persist in `.artifacts/places/` (a frame, if saved, is `<name>.jpg` beside it), outside every profile, so they survive profile switches. Re-use them rather than asking for coordinates again. Free-form facts (dock location, map date) go in `notes remember|recall|forget` (`.artifacts/notes.json`).

## Robots with a twist switch and a service e-stop (e.g. LeKiwi)

- **Velocity chain.** `joy_teleop` -> `/cmd_vel_teleop`, Nav2 -> `/cmd_vel_smoothed`, `twist_switch_node` selects one (`/twist_switch`, SetBool, true = Nav2) -> `/cmd_vel_presafety` -> collision monitor -> `/base_controller/cmd_vel`. Teleop-topic commands only move the robot in teleop mode and Nav2 commands only in Nav2 mode. `preflight motion --source` reports a mismatch and the fix.
- **State without polling.** Servers that enable service introspection replay their last call, so `services state` reads the current e-stop, mode and patrol state. `null` means the toggle was never called since the server started (the defaults are e-stop off, teleop mode).
- **E-stop semantics.** `/emergency_stop` (SetBool) disables motor torque: wheels coast. `/emergency_stop_toggle` is the joystick's Trigger and flips state, so the skill never calls it.
- **Look around.** A fixed camera means turning the base: `plan "look around N views"` captures a frame, reads the pose with `tf lookup`, and rotates 360/N between views. Claude reads the saved images directly.
- **On blocks.** With the wheels off the ground, Nav2 Spin/NavigateToPose never complete (the EKF yaw does not change); the command times out, cancels its goal and zeroes velocity (it used to leave the wheels spinning). Use `publish-until` for stand tests. In `sim:=true` a spin completes normally.
- **Smoke tests.** With the stack running (`sim:=true` or the real robot, started by you): `pytest tests/test_sim_smoke.py --sim`. Add `ROS2_SIM_MOTION=1` to also drive a 0.1 m closed-loop move.
