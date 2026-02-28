---
name: ros2-skill
description: "Controls ROS 2 robots directly via rclpy CLI. Use when the user asks about ROS 2 topics, services, nodes, parameters, actions, robot movement, sensor data, or any ROS 2 robot interaction."
license: Apache-2.0
compatibility: "Requires python3, rclpy, and ROS 2 environment sourced"
user-invocable: true
metadata: {"openclaw": {"emoji": "🤖", "requires": {"bins": ["python3", "ros2"], "pip": ["rclpy", "rosidl-runtime-py"]}, "category": "robotics", "tags": ["ros2", "robotics", "rclpy"]}, "author": ["adityakamath", "lpigeon"], "version": "3.0.0"}
---

# ROS 2 Skill

Controls and monitors ROS 2 robots directly via rclpy.

**Architecture:** Agent → `ros2_cli.py` → rclpy → ROS 2

All commands output JSON. Errors contain `{"error": "..."}`.

For full command reference with arguments, options, and output examples, see [references/COMMANDS.md](references/COMMANDS.md).

---

## Setup

### 1. Source ROS 2 environment

```bash
source /opt/ros/${ROS_DISTRO}/setup.bash
```

### 2. Install dependencies

```bash
pip install rclpy rosidl-runtime-py
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

## Command Quick Reference

| Category | Command | Description |
|----------|---------|-------------|
| Connection | `version` | Detect ROS 2 distro |
| Safety | `estop` | Emergency stop for mobile robots |
| Topics | `topics list` | List all active topics with types |
| Topics | `topics type <topic>` | Get message type of a topic |
| Topics | `topics details <topic>` | Get topic publishers/subscribers |
| Topics | `topics info <topic>` | Alias for `topics details` |
| Topics | `topics message <msg_type>` | Get message field structure |
| Topics | `topics subscribe <topic>` | Subscribe and receive messages |
| Topics | `topics echo <topic>` | Alias for `topics subscribe` |
| Topics | `topics sub <topic>` | Alias for `topics subscribe` |
| Topics | `topics publish <topic> <json>` | Publish a message to a topic |
| Topics | `topics pub <topic> <json>` | Alias for `topics publish` |
| Topics | `topics publish-sequence <topic> <msgs> <durs>` | Publish message sequence |
| Topics | `topics pub-seq <topic> <msgs> <durs>` | Alias for `topics publish-sequence` |
| Topics | `topics publish-until <topic> <json>` | Publish while monitoring; stop on condition |
| Topics | `topics publish-continuous <topic> <json>` | Alias for `topics publish` |
| Topics | `topics hz <topic>` | Measure topic publish rate |
| Topics | `topics find <msg_type>` | Find topics by message type |
| Topics | `topics bw <topic>` | Measure topic bandwidth (bytes/s) |
| Topics | `topics delay <topic>` | Measure header-stamp end-to-end latency |
| Services | `services list` | List all available services |
| Services | `services type <service>` | Get service type |
| Services | `services details <service>` | Get service request/response fields |
| Services | `services info <service>` | Alias for `services details` |
| Services | `services call <service> <json>` | Call a service |
| Services | `services find <service_type>` | Find services by service type |
| Nodes | `nodes list` | List all active nodes |
| Nodes | `nodes ls` | Alias for `nodes list` |
| Nodes | `nodes details <node>` | Get node topics/services/actions |
| Nodes | `nodes info <node>` | Alias for `nodes details` |
| Params | `params list <node>` | List node parameters |
| Params | `params ls <node>` | Alias for `params list` |
| Params | `params get <node:param>` | Get parameter value |
| Params | `params set <node:param> <value>` | Set parameter value |
| Params | `params describe <node:param>` | Describe parameter type and constraints |
| Params | `params dump <node>` | Export all parameters for a node as JSON |
| Params | `params load <node> <json>` | Bulk-set parameters from JSON |
| Params | `params delete <node> <param>` | Delete a parameter |
| Actions | `actions list` | List action servers |
| Actions | `actions ls` | Alias for `actions list` |
| Actions | `actions details <action>` | Get action goal/result/feedback fields |
| Actions | `actions info <action>` | Alias for `actions details` |
| Actions | `actions type <action>` | Get action server type |
| Actions | `actions send <action> <json>` | Send action goal |
| Actions | `actions send-goal <action> <json>` | Alias for `actions send` |
| Actions | `actions cancel <action>` | Cancel all in-flight goals |

---

## Key Commands

### version

```bash
python3 {baseDir}/scripts/ros2_cli.py version
```

### topics list / type / details / message

```bash
python3 {baseDir}/scripts/ros2_cli.py topics list
python3 {baseDir}/scripts/ros2_cli.py topics type /turtle1/cmd_vel
python3 {baseDir}/scripts/ros2_cli.py topics details /turtle1/cmd_vel
python3 {baseDir}/scripts/ros2_cli.py topics message geometry_msgs/Twist
```

### topics subscribe / echo / sub

`echo` and `sub` are aliases for `subscribe`. Without `--duration`: returns first message. With `--duration`: collects multiple messages.

```bash
python3 {baseDir}/scripts/ros2_cli.py topics subscribe /turtle1/pose
python3 {baseDir}/scripts/ros2_cli.py topics echo /odom
python3 {baseDir}/scripts/ros2_cli.py topics sub /odom --duration 10 --max-messages 50
```

### topics publish / pub / publish-continuous

`pub` and `publish-continuous` are aliases for `publish`. Without `--duration` / `--timeout`: single-shot. With `--duration` or `--timeout` (equivalent): publishes repeatedly at `--rate` Hz for the specified seconds, then stops. **Use `--duration` for velocity commands** — most robot controllers stop if they don't receive continuous `cmd_vel` messages.

```bash
# Single-shot
python3 {baseDir}/scripts/ros2_cli.py topics publish /trigger '{"data": ""}'

# Move forward 3 seconds (velocity — use --duration or --timeout)
python3 {baseDir}/scripts/ros2_cli.py topics pub /cmd_vel \
  '{"linear":{"x":1.0,"y":0,"z":0},"angular":{"x":0,"y":0,"z":0}}' --duration 3

# Equivalent with --timeout
python3 {baseDir}/scripts/ros2_cli.py topics publish /cmd_vel \
  '{"linear":{"x":0.3,"y":0,"z":0},"angular":{"x":0,"y":0,"z":0}}' --timeout 5

# Stop
python3 {baseDir}/scripts/ros2_cli.py topics publish /cmd_vel \
  '{"linear":{"x":0,"y":0,"z":0},"angular":{"x":0,"y":0,"z":0}}'
```

Options: `--duration SECONDS` (or `--timeout SECONDS` — identical), `--rate HZ` (default 10)

Output when `--duration`/`--timeout` is used includes `stopped_by: "timeout" | "keyboard_interrupt"`.

### topics publish-sequence / pub-seq

`pub-seq` is an alias for `publish-sequence`. Publish a sequence of messages, each repeated at `--rate` Hz for its corresponding duration. Arrays must have the same length.

```bash
# Forward 3s then stop
python3 {baseDir}/scripts/ros2_cli.py topics pub-seq /cmd_vel \
  '[{"linear":{"x":1.0,"y":0,"z":0},"angular":{"x":0,"y":0,"z":0}},{"linear":{"x":0,"y":0,"z":0},"angular":{"x":0,"y":0,"z":0}}]' \
  '[3.0, 0.5]'

# Draw a square (turtlesim)
python3 {baseDir}/scripts/ros2_cli.py topics publish-sequence /turtle1/cmd_vel \
  '[{"linear":{"x":2},"angular":{"z":0}},{"linear":{"x":0},"angular":{"z":1.5708}},{"linear":{"x":2},"angular":{"z":0}},{"linear":{"x":0},"angular":{"z":1.5708}},{"linear":{"x":2},"angular":{"z":0}},{"linear":{"x":0},"angular":{"z":1.5708}},{"linear":{"x":2},"angular":{"z":0}},{"linear":{"x":0},"angular":{"z":1.5708}},{"linear":{"x":0},"angular":{"z":0}}]' \
  '[1,1,1,1,1,1,1,1,0.5]'
```

Options: `--rate HZ` (default 10)

### topics publish-until

Publish to a topic at a fixed rate while monitoring a second topic. Stops when a condition on the monitored field is satisfied, or after the safety timeout.

```bash
# Move forward until odometry x position increases by 1.0 m
python3 {baseDir}/scripts/ros2_cli.py topics publish-until /cmd_vel \
  '{"linear":{"x":0.3,"y":0,"z":0},"angular":{"x":0,"y":0,"z":0}}' \
  --monitor /odom --field pose.pose.position.x --delta 1.0 --timeout 30

# Stop when lidar range at index 0 drops below 0.5 m
python3 {baseDir}/scripts/ros2_cli.py topics publish-until /cmd_vel \
  '{"linear":{"x":0.2,"y":0,"z":0},"angular":{"x":0,"y":0,"z":0}}' \
  --monitor /scan --field ranges.0 --below 0.5 --timeout 60
```

Required: `--monitor <topic>`, `--field <dot.path>`, and exactly one of `--delta N`, `--above N`, `--below N`, `--equals V`

Options: `--rate HZ` (default 10), `--timeout SECONDS` (default 60), `--msg-type TYPE`, `--monitor-msg-type TYPE`

### topics publish-continuous

Alias for `topics publish`. Use `topics publish --timeout SECONDS` instead.

```bash
# Preferred
python3 {baseDir}/scripts/ros2_cli.py topics publish /cmd_vel \
  '{"linear":{"x":0.3,"y":0,"z":0},"angular":{"x":0,"y":0,"z":0}}' --timeout 5

# Also works (identical behaviour)
python3 {baseDir}/scripts/ros2_cli.py topics publish-continuous /cmd_vel \
  '{"linear":{"x":0.3,"y":0,"z":0},"angular":{"x":0,"y":0,"z":0}}' --timeout 5
```

### topics hz

Measure the publish rate of a topic. Reports rate (Hz), min/max delta, and standard deviation.

```bash
python3 {baseDir}/scripts/ros2_cli.py topics hz /turtle1/pose
python3 {baseDir}/scripts/ros2_cli.py topics hz /scan --window 20 --timeout 15
```

Options: `--window N` (intervals to sample, default 10), `--timeout SEC` (default 10)

### topics find

Find all topics publishing a specific message type. Accepts both `/msg/` and short formats.

```bash
python3 {baseDir}/scripts/ros2_cli.py topics find geometry_msgs/msg/Twist
python3 {baseDir}/scripts/ros2_cli.py topics find geometry_msgs/Twist
```

### services list / type / details / info

`info` is an alias for `details`.

```bash
python3 {baseDir}/scripts/ros2_cli.py services list
python3 {baseDir}/scripts/ros2_cli.py services type /spawn
python3 {baseDir}/scripts/ros2_cli.py services details /spawn
python3 {baseDir}/scripts/ros2_cli.py services info /spawn
```

### services call

```bash
python3 {baseDir}/scripts/ros2_cli.py services call /reset '{}'
python3 {baseDir}/scripts/ros2_cli.py services call /spawn \
  '{"x":3.0,"y":3.0,"theta":0.0,"name":"turtle2"}'
```

### services find

Find all services of a specific type. Accepts both `/srv/` and short formats.

```bash
python3 {baseDir}/scripts/ros2_cli.py services find std_srvs/srv/Empty
python3 {baseDir}/scripts/ros2_cli.py services find std_srvs/Empty
```

### nodes list / details / info

`info` is an alias for `details`. Output includes publishers, subscribers, services, action servers, and action clients.

```bash
python3 {baseDir}/scripts/ros2_cli.py nodes list
python3 {baseDir}/scripts/ros2_cli.py nodes details /turtlesim
python3 {baseDir}/scripts/ros2_cli.py nodes info /turtlesim
```

### params list / get / set

Uses `node:param_name` format or space-separated `node param_name` format.

```bash
python3 {baseDir}/scripts/ros2_cli.py params list /turtlesim
python3 {baseDir}/scripts/ros2_cli.py params get /turtlesim:background_r
python3 {baseDir}/scripts/ros2_cli.py params get /base_controller base_frame_id
python3 {baseDir}/scripts/ros2_cli.py params set /turtlesim:background_r 255
python3 {baseDir}/scripts/ros2_cli.py params set /base_controller base_frame_id base_link_new
```

### actions list / details / info / type / send / send-goal

`info` is an alias for `details`. `send-goal` is an alias for `send`.

```bash
python3 {baseDir}/scripts/ros2_cli.py actions list
python3 {baseDir}/scripts/ros2_cli.py actions details /turtle1/rotate_absolute
python3 {baseDir}/scripts/ros2_cli.py actions info /turtle1/rotate_absolute
python3 {baseDir}/scripts/ros2_cli.py actions type /turtle1/rotate_absolute
python3 {baseDir}/scripts/ros2_cli.py actions send /turtle1/rotate_absolute \
  '{"theta":3.14}'
python3 {baseDir}/scripts/ros2_cli.py actions send-goal /turtle1/rotate_absolute \
  '{"theta":3.14}'
```

### actions send --feedback

Add `--feedback` to collect feedback messages during goal execution. Feedback is included in the result as `feedback_msgs`.

```bash
python3 {baseDir}/scripts/ros2_cli.py actions send /turtle1/rotate_absolute \
  '{"theta":3.14}' --feedback
```

### actions cancel

Cancel all in-flight goals on an action server.

```bash
python3 {baseDir}/scripts/ros2_cli.py actions cancel /turtle1/rotate_absolute
```

Options: `--timeout SECONDS` (default 5)

### topics bw

Measure topic bandwidth. Reports bytes/s, bytes/msg, and rate.

```bash
python3 {baseDir}/scripts/ros2_cli.py topics bw /camera/image_raw
python3 {baseDir}/scripts/ros2_cli.py topics bw /scan --window 20 --timeout 15
```

Options: `--window N` (samples, default 10), `--timeout SEC` (default 10)

### topics delay

Measure end-to-end latency between `header.stamp` and wall clock. Requires messages with `header.stamp`.

```bash
python3 {baseDir}/scripts/ros2_cli.py topics delay /odom
python3 {baseDir}/scripts/ros2_cli.py topics delay /scan --window 20
```

Options: `--window N` (samples, default 10), `--timeout SEC` (default 10)

### params describe / dump / load / delete

Advanced parameter operations.

```bash
# Describe a parameter (type, constraints, read-only status)
python3 {baseDir}/scripts/ros2_cli.py params describe /turtlesim:background_r

# Export all parameters for a node
python3 {baseDir}/scripts/ros2_cli.py params dump /turtlesim

# Bulk-set parameters from JSON
python3 {baseDir}/scripts/ros2_cli.py params load /turtlesim \
  '{"background_r":255,"background_g":0,"background_b":0}'

# Delete a parameter
python3 {baseDir}/scripts/ros2_cli.py params delete /turtlesim background_r
```

---

## Workflow Examples

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

Always check the message structure first, then publish movement, and always stop after.

```bash
python3 {baseDir}/scripts/ros2_cli.py topics message geometry_msgs/Twist
python3 {baseDir}/scripts/ros2_cli.py topics publish-sequence /cmd_vel \
  '[{"linear":{"x":1.0,"y":0,"z":0},"angular":{"x":0,"y":0,"z":0}},{"linear":{"x":0,"y":0,"z":0},"angular":{"x":0,"y":0,"z":0}}]' \
  '[2.0, 0.5]'
```

### 3. Read Sensor Data

```bash
python3 {baseDir}/scripts/ros2_cli.py topics type /scan
python3 {baseDir}/scripts/ros2_cli.py topics message sensor_msgs/LaserScan
python3 {baseDir}/scripts/ros2_cli.py topics subscribe /scan --duration 3
python3 {baseDir}/scripts/ros2_cli.py topics subscribe /odom --duration 10 --max-messages 50
```

### 4. Use Services

```bash
python3 {baseDir}/scripts/ros2_cli.py services list
python3 {baseDir}/scripts/ros2_cli.py services details /spawn
python3 {baseDir}/scripts/ros2_cli.py services call /spawn \
  '{"x":3.0,"y":3.0,"theta":0.0,"name":"turtle2"}'
```

### 5. Actions

```bash
python3 {baseDir}/scripts/ros2_cli.py actions list
python3 {baseDir}/scripts/ros2_cli.py actions details /turtle1/rotate_absolute
python3 {baseDir}/scripts/ros2_cli.py actions send /turtle1/rotate_absolute \
  '{"theta":1.57}'
```

### 6. Change Parameters

```bash
python3 {baseDir}/scripts/ros2_cli.py params list /turtlesim
python3 {baseDir}/scripts/ros2_cli.py params get /turtlesim:background_r
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

| User intent | Monitor topic | Field | Condition |
|-------------|--------------|-------|-----------|
| Move forward N m (straight) | `/odom` | `pose.pose.position.x` | `--delta N` |
| Move backward N m | `/odom` | `pose.pose.position.x` | `--delta -N` |
| Move sideways N m | `/odom` | `pose.pose.position.y` | `--delta N` |
| Rotate N rad | `/odom` | `pose.pose.orientation.z` | `--delta N` |
| Joint reach angle | `/joint_states` | `position.0` (index of joint) | `--equals A` or `--delta D` |
| Stop near obstacle | `/scan` | `ranges.0` (front index) | `--below 0.5` |
| Stop at range | `/range` | `range` | `--below D` |
| Stop at temperature | `/temperature` | `temperature` | `--above T` |
| Stop at battery level | `/battery` | `percentage` | `--below P` |

For **diagonal movement or Euclidean distance**, monitor the axis with the dominant displacement (`position.x` or `position.y`), or use a conservative `--delta` on total time instead and verify final position after.

#### Examples

```bash
# "Drive forward 1 meter" — discovered /odom is nav_msgs/Odometry
python3 {baseDir}/scripts/ros2_cli.py topics publish-until /cmd_vel \
  '{"linear":{"x":0.2,"y":0,"z":0},"angular":{"x":0,"y":0,"z":0}}' \
  --monitor /odom --field pose.pose.position.x --delta 1.0 --timeout 30

# "Move arm until joint_3 reaches 1.5 rad" — joint index 2 (0-based)
python3 {baseDir}/scripts/ros2_cli.py topics publish-until /arm/cmd \
  '{"joint_3_velocity":0.2}' \
  --monitor /joint_states --field position.2 --equals 1.5 --timeout 20

# "Stop when obstacle within 0.4 m"
python3 {baseDir}/scripts/ros2_cli.py topics publish-until /cmd_vel \
  '{"linear":{"x":0.2,"y":0,"z":0},"angular":{"x":0,"y":0,"z":0}}' \
  --monitor /scan --field ranges.0 --below 0.4 --timeout 60

# "Stop when temperature exceeds 50°C"
python3 {baseDir}/scripts/ros2_cli.py topics publish-until /heater/cmd \
  '{"power":1.0}' \
  --monitor /temperature --field temperature --above 50.0 --timeout 120
```

---

## Safety Notes

**Destructive commands** (can move the robot or change state):
- `topics publish` / `topics publish-sequence` — sends movement or control commands
- `topics publish-until` — publishes continuously until a condition or timeout; always specify a conservative `--timeout`
- `topics publish-continuous` — publishes for a fixed duration; `--timeout` is required
- `services call` — can reset, spawn, kill, or change robot state
- `params set` — modifies runtime parameters
- `actions send` — triggers robot actions (rotation, navigation, etc.)

**Always stop the robot after movement.** The last message in any `publish-sequence` should be all zeros:
```json
{"linear":{"x":0,"y":0,"z":0},"angular":{"x":0,"y":0,"z":0}}
```

**Always check JSON output for errors before proceeding.**

---

## Troubleshooting

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
