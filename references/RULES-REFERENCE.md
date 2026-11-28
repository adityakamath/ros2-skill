# Reference: intent → command, launch, setup

Load when you need the command for an intent that isn't obvious. Flags and output formats are in `CLI <cmd> --help` and [COMMANDS.md](COMMANDS.md).

`CLI` means `python3 ${CLAUDE_SKILL_DIR}/scripts/ros2_cli.py`.

---

## Motion vocabulary

Any of these words is a motion command and triggers the full Rule 3 workflow: move, go, drive, travel, head, advance, roll, navigate, forward/back/reverse, left/right, rotate, turn, spin, yaw, pivot, strafe, slide, sidestep. Odometry availability, not the user's wording, picks the method:

| Odom ≥ 5 Hz | Target given | Method |
|---|---|---|
| yes | distance or angle | `publish-until` (closed loop) |
| yes | none (open-ended) | `publish-sequence` with a zero payload last |
| no | distance or angle | `publish-sequence` with a calculated duration; tell the user it is open loop |
| no | none | `publish-sequence` with a zero payload last; tell the user |

A bare "move" or "go" with no direction and no target is the one case to ask: *"Which direction, and how far?"* "Stop/halt/freeze" means `estop`. "Turn around" is 180°. Strafe needs a holonomic drive (positive `linear.y` is left).

## Intent → command

Anything not listed: `CLI <group> --help`.

| Intent | Command |
|---|---|
| What topics / nodes / services / actions exist | `topics list`, `nodes list`, `services list`, `actions list` |
| Is node/topic X alive | `nodes list`; `topics details <t>` (publisher_count) + `topics hz <t>` |
| Topic type / fields / payload template | `topics type`, `interface show <type>`, `interface proto <type>` |
| Publisher/subscriber counts, QoS | `topics details <t>`, `topics qos-check <t>` |
| Rate / bandwidth / delay | `topics hz`, `topics bw`, `topics delay` |
| Read sensor data (LiDAR, IMU, GPS, joints, ...) | Profile topic, else `topics find <type>` → `topics subscribe <t> --max-messages 1` |
| Several readings at once (battery, pose, tilt, LiDAR, ToF, depth, wheels, e-stop/mode, controllers) | `snapshot [--only a,b]` (one call; `--rates` adds topic rates) |
| Battery | `snapshot --only battery` (or `topics battery`) |
| Diagnostics / "what's wrong" | `doctor`, `doctor diagnostics [--level warn]`, `topics diag`, then Rule 7 |
| Remember a fact (dock location, map date) / recall it | `notes remember <text> [--tag T]`, `notes recall [words]`, `notes forget <id>` |
| Measure how far the base coasts, tune the slow-down zone | `braking measure` (moves the robot about 0.8 m), `braking show`, `braking clear` |
| Picture / "what do you see" / did the view change | `look [--diff S]` (colour camera, auto-rotated; read the `path`); other cameras: `topics capture-image` |
| Camera photo (specific topic) | `topics capture-image --topic <t> --output ${CLAUDE_SKILL_DIR}/.artifacts/<name>.jpg` (auto-rotates per profile; `--no-profile` to disable; check `image_rotated_deg`) |
| Depth at a pixel | `topics depth-point --topic <t> --u <col> --v <row>` |
| Wrong image orientation | `profile set-camera-rotation <topic_substring> <0\|90\|180\|-90> --note "<why>"` |
| TF lookup / stream / health | `tf lookup <src> <tgt>`, `tf echo`, `tf monitor`, `tf validate`; conversions: `tf euler-from-quaternion`, `tf quaternion-from-euler`, `tf transform-point` |
| Where is the robot (map) | `tf lookup map base_link`. Pose relative to start: odom (Rule 8). |
| Call a service | `services find|list` → `services details` → `services call <svc> '<json>'`; check the response fields |
| Send / cancel / watch an action | `actions details` → `actions send <a> '<json>' --timeout N`; `actions cancel`; `actions echo` |
| State of e-stop / twist switch / waypoint follow | `services state /emergency_stop /twist_switch /waypoint_follow` (null = never toggled) |
| Switch velocity input (teleop <-> Nav2) | `services call <velocity_chain.mode_service> '{"data": true|false}'` (true = Nav2); `plan` inserts this itself |
| Look around (fixed camera) | `plan "look around 4 views"` (capture, `tf lookup`, `nav2 rotate`) |
| Patrol, waypoints, map via robot services | `plan "start patrol"`, `"record waypoint"`, `"reset waypoints"`, `"save the map"` |
| Do a plain motion goal now (fastest path) | `plan "<goal>" --run` (one call: check, move, stop, verify). Chains with "then"/"and": turn (direction optional, "turn around"), "undo that"/"rotate back", "report the battery", "is the path ahead clear", "look at X", "remember that ...". Refuses the whole goal if any part is not understood |
| Routine stop | `stop --verify-odom-topic <ODOM>` (never latches the e-stop) |
| Multi-step goal in words | `plan "<goal>"` (compiles to commands, executes nothing; unknown places become warnings) |
| Navigate to X,Y | `nav2 go X Y [--yaw D]` (Rule 9 Nav2 preemption first); see SKILL.md Nav2 section |
| Set initial pose | `nav2 initial-pose X Y --yaw D` |
| Clear costmap | `nav2 costmap-clear [--layer ...]` |
| Parameter get / set / describe | `params get|set|describe <node:param>` (set needs describe first, verify after) |
| Dump / load params | `params dump <node>`, `params load <node> <file>` (Rule 0 checks first) |
| Parameter presets | `params preset-save|load|list|delete` |
| Controllers | `control list-controllers`, `load-controller`, `configure-controller`, `switch-controllers --activate X --deactivate Y --strictness STRICT`, `set-controller-state`, `view-controller-chains`, `list-controller-types` |
| Hardware | `control list-hardware-components`, `list-hardware-interfaces`, `set-hardware-component-state` |
| Lifecycle | `lifecycle nodes|get|set <node> configure|activate|deactivate|cleanup|shutdown` (verify each with `get`) |
| Launch / run | **User runs it** (SKILL.md hard constraint 1). Give them `launch new <pkg> <file> [name:=value ...]`, `run new <pkg> <exe>`, or `launch|run restart|kill <session>`; `launch list [keyword]` and `run list` are read-only |
| Standalone component, node kill | **User runs it.** Give them `component standalone <pkg> <plugin>`, `component kill <session>` or `nodes kill <node> --confirm` |
| Foxglove | **User runs it.** Give them `launch foxglove` (or `foxglove start`) and `foxglove stop`; `foxglove status` is read-only |
| Bag | `bag info <file>`. Recording/playback has no CLI command; give the user `ros2 bag record|play` |
| Logs | `logs list-runs|query|tail|node-summary` (works without a live graph) |
| Packages | `pkg list|prefix|executables|xml|create` |
| Profile | `profile context` (loaded with the skill; creates/updates), `scan|show|rescan|list|annotate|set-camera-rotation|set-obstacle` (Rules 0.0, 0.6, 0.7) |
| Drive until an obstacle | `plan "drive forward until obstacle" --run` (RULES-MOTION.md Rule 25) |
| Daemon | `daemon status|start|stop`, `daemon-fast status|start|stop` |
| Domain / localhost problems | Check `ROS_DOMAIN_ID` / `ROS_LOCALHOST_ONLY`, then `nodes list` (Rule 0.1 step 1) |
| Run package tests | Shell: `colcon test --packages-select <pkg>`, then `colcon test-result --verbose` (the CLI has no test runner) |
| Add a note to the profile | `profile annotate "<text>"` |

## Launch files

For a vague request ("run the bringup"), find candidates with `launch list <keyword>` (or Path A `summary.launch_files`) using the cue word (bringup, navigation/nav2, camera, arm/moveit, sim/gazebo). One clear match means give the user the command. Several that context can't separate: ask which one. None: widen the search, then ask for the exact name.

Launch arguments are positional `name:=value` pairs. The CLI always passes them through; a `NOTICE:` about an unrecognised arg is informational and ROS 2 is the real validator. Use `profile show --section <launch_file>` to see declared args. `--config-path` is for YAML node-parameter files only, never launch args.

## Local workspace sourcing

System ROS is assumed to be sourced. The skill also sources a local workspace automatically (`$ROS2_LOCAL_WS`, else `~/ros2_ws`, `~/colcon_ws`, `~/dev_ws`, `~/workspace`, `~/ros2`). If it is found but not built, the skill warns and runs without it.

## Delivering images to Discord (only if asked)

Capture with `topics capture-image`, then `python3 ${CLAUDE_SKILL_DIR}/scripts/discord_tools.py send-image --path <file> --channel-id <ID> --delete`. Use `--config <path>` to override config auto-detection, and omit `--delete` to keep the file. Don't use any other mechanism to send files to Discord.

## Setup

Needs python3, `rclpy` and a sourced ROS 2 environment (`source /opt/ros/$ROS_DISTRO/setup.bash`) on the machine that talks to the robot. Check with `CLI version`.
