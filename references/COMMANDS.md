# Command Reference

Full reference for all `ros2_cli.py` commands with arguments, options, and output examples.

---

## version

Detect the ROS 2 version and distro name.

```bash
python3 {baseDir}/scripts/ros2_cli.py version
```

Output:
```json
{"version": "2", "distro": "humble", "domain_id": 0}
```

---

## estop

Emergency stop for mobile robots. Auto-detects the velocity command topic and message type, then publishes zero velocity to stop the robot.

**Note:** This command is for mobile robots only (differential drive, omnidirectional, etc.). It will NOT work for robotic arms or manipulators.

```bash
python3 {baseDir}/scripts/ros2_cli.py estop
python3 {baseDir}/scripts/ros2_cli.py estop --topic /cmd_vel_nav
```

| Option | Default | Description |
|--------|---------|-------------|
| `--topic` | auto-detect | Custom velocity topic (default: auto-detect from /cmd_vel, /cmd_vel_nav, etc.) |

Output:
```json
{"success": true, "topic": "/cmd_vel", "type": "geometry_msgs/Twist", "message": "Emergency stop activated (mobile robot stopped)"}
```

If no velocity topic is found:
```json
{"error": "Could not find velocity command topic", "hint": "This command is for mobile robots only (not arms). Ensure the robot has a /cmd_vel topic."}
```

---

## topics list

List all active topics with their message types.

```bash
python3 {baseDir}/scripts/ros2_cli.py topics list
```

Output:
```json
{
  "topics": ["/turtle1/cmd_vel", "/turtle1/pose", "/rosout"],
  "types": ["geometry_msgs/Twist", "turtlesim/Pose", "rcl_interfaces/msg/Log"],
  "count": 3
}
```

---

## topics type `<topic>`

Get the message type of a specific topic.

| Argument | Required | Description |
|----------|----------|-------------|
| `topic` | Yes | Topic name (e.g. `/cmd_vel`, `/turtle1/pose`) |

```bash
python3 {baseDir}/scripts/ros2_cli.py topics type /turtle1/cmd_vel
```

Output:
```json
{"topic": "/turtle1/cmd_vel", "type": "geometry_msgs/Twist"}
```

---

## topics details `<topic>`

Get topic details including message type, publishers, and subscribers.

| Argument | Required | Description |
|----------|----------|-------------|
| `topic` | Yes | Topic name |

```bash
python3 {baseDir}/scripts/ros2_cli.py topics details /turtle1/cmd_vel
```

Output:
```json
{
  "topic": "/turtle1/cmd_vel",
  "type": "geometry_msgs/Twist",
  "publishers": [],
  "subscribers": ["/turtlesim"]
}
```

---

## topics message `<message_type>`

Get the field structure of a message type.

| Argument | Required | Description |
|----------|----------|-------------|
| `message_type` | Yes | Full message type (e.g. `geometry_msgs/Twist`, `sensor_msgs/LaserScan`) |

```bash
python3 {baseDir}/scripts/ros2_cli.py topics message geometry_msgs/Twist
```

Output:
```json
{
  "message_type": "geometry_msgs/Twist",
  "structure": {
    "linear": {"x": 0.0, "y": 0.0, "z": 0.0},
    "angular": {"x": 0.0, "y": 0.0, "z": 0.0}
  }
}
```

---

## topics subscribe `<topic>` [options]

Subscribe to a topic and receive messages. Also available as `topics echo` (identical alias).

| Argument | Required | Description |
|----------|----------|-------------|
| `topic` | Yes | Topic to subscribe to |

| Option | Default | Description |
|--------|---------|-------------|
| `--duration SECONDS` | _(none)_ | Collect messages for this duration. Without this, returns first message only |
| `--max-messages N` / `--max-msgs N` | `100` | Maximum number of messages to collect (only with `--duration`) |
| `--timeout SECONDS` | `5` | Timeout waiting for first message |

**Single message (default):**
```bash
python3 {baseDir}/scripts/ros2_cli.py topics subscribe /turtle1/pose
```

Output:
```json
{
  "msg": {"x": 5.544, "y": 5.544, "theta": 0.0, "linear_velocity": 0.0, "angular_velocity": 0.0}
}
```

**Collect over time:**
```bash
python3 {baseDir}/scripts/ros2_cli.py topics subscribe /odom --duration 10 --max-messages 50
```

Output:
```json
{
  "topic": "/odom",
  "collected_count": 50,
  "messages": [
    {"header": {}, "pose": {"pose": {"position": {"x": 0.1, "y": 0.0, "z": 0.0}}}},
    "..."
  ]
}
```

---

## topics publish `<topic>` `<json_message>` [options]

Publish a message to a topic. Without `--duration`, sends once. With `--duration`, publishes repeatedly at `--rate` Hz — required for velocity commands since most robot controllers stop if they don't receive continuous commands.

| Argument | Required | Description |
|----------|----------|-------------|
| `topic` | Yes | Topic to publish to |
| `json_message` | Yes | JSON string of the message payload |

| Option | Default | Description |
|--------|---------|-------------|
| `--msg-type TYPE` | `std_msgs/msg/String` | Message type |
| `--duration SECONDS` | _(none)_ | Publish repeatedly for this duration |
| `--rate HZ` | `10` | Publish rate in Hz (used with `--duration`) |

**Single-shot (trigger, one-time command):**
```bash
python3 {baseDir}/scripts/ros2_cli.py topics publish /turtle1/cmd_vel \
  '{"linear":{"x":2.0,"y":0,"z":0},"angular":{"x":0,"y":0,"z":0}}'
```

**Move forward for 3 seconds (recommended for velocity):**
```bash
python3 {baseDir}/scripts/ros2_cli.py topics publish /cmd_vel \
  '{"linear":{"x":1.0,"y":0,"z":0},"angular":{"x":0,"y":0,"z":0}}' --duration 3
```

Output:
```json
{"success": true, "topic": "/cmd_vel", "msg_type": "geometry_msgs/Twist", "duration": 3.0, "rate": 10.0, "published_count": 30}
```

**Rotate left for 2 seconds:**
```bash
python3 {baseDir}/scripts/ros2_cli.py topics publish /cmd_vel \
  '{"linear":{"x":0,"y":0,"z":0},"angular":{"x":0,"y":0,"z":0.5}}' --duration 2
```

**Stop (single-shot is fine for stop):**
```bash
python3 {baseDir}/scripts/ros2_cli.py topics publish /cmd_vel \
  '{"linear":{"x":0,"y":0,"z":0},"angular":{"x":0,"y":0,"z":0}}'
```

---

## topics publish-sequence `<topic>` `<json_messages>` `<json_durations>` [options]

Publish a sequence of messages. Each message is published repeatedly at `--rate` Hz for its corresponding duration. The arrays must have the same length.

| Argument | Required | Description |
|----------|----------|-------------|
| `topic` | Yes | Topic to publish to |
| `json_messages` | Yes | JSON array of messages to publish in order |
| `json_durations` | Yes | JSON array of durations (seconds) for each message |

| Option | Default | Description |
|--------|---------|-------------|
| `--msg-type TYPE` | `std_msgs/msg/String` | Message type |
| `--rate HZ` | `10` | Publish rate in Hz |

**Move forward 3 seconds then stop:**
```bash
python3 {baseDir}/scripts/ros2_cli.py topics publish-sequence /cmd_vel \
  '[{"linear":{"x":1.0,"y":0,"z":0},"angular":{"x":0,"y":0,"z":0}},{"linear":{"x":0,"y":0,"z":0},"angular":{"x":0,"y":0,"z":0}}]' \
  '[3.0, 0.5]'
```

Output:
```json
{"success": true, "published_count": 35, "topic": "/cmd_vel", "rate": 10.0}
```

**Draw a square (turtlesim):**
```bash
python3 {baseDir}/scripts/ros2_cli.py topics publish-sequence /turtle1/cmd_vel \
  '[{"linear":{"x":2},"angular":{"z":0}},{"linear":{"x":0},"angular":{"z":1.5708}},{"linear":{"x":2},"angular":{"z":0}},{"linear":{"x":0},"angular":{"z":1.5708}},{"linear":{"x":2},"angular":{"z":0}},{"linear":{"x":0},"angular":{"z":1.5708}},{"linear":{"x":2},"angular":{"z":0}},{"linear":{"x":0},"angular":{"z":1.5708}},{"linear":{"x":0},"angular":{"z":0}}]' \
  '[1,1,1,1,1,1,1,1,0.5]'
```

---

## topics publish-until `<topic>` `<json_message>` [options]

Publish a message at a fixed rate while simultaneously monitoring a second topic. Stops as soon as a structured condition on the monitored field is satisfied, or after a safety timeout. Use this to move a robot until it reaches a target pose, until a sensor threshold is crossed, etc.

| Argument | Required | Description |
|----------|----------|-------------|
| `topic` | Yes | Topic to publish to |
| `json_message` | Yes | JSON string of the message payload |

| Option | Required | Default | Description |
|--------|----------|---------|-------------|
| `--monitor TOPIC` | Yes | — | Topic to watch for the stop condition |
| `--field DOT.PATH` | Yes | — | Dot-separated field path in monitor messages (e.g. `pose.pose.position.x`, `ranges.0`) |
| `--delta N` | One of these | — | Stop when field changes by ±N from its value at start |
| `--above N` | One of these | — | Stop when field > N |
| `--below N` | One of these | — | Stop when field < N |
| `--equals V` | One of these | — | Stop when field == V (numeric: epsilon 1e-9; fallback: string comparison) |
| `--rate HZ` | No | `10` | Publish rate in Hz |
| `--timeout SECONDS` | No | `60` | Safety stop if condition is not met within this duration |
| `--msg-type TYPE` | No | auto-detect | Override publish topic message type |
| `--monitor-msg-type TYPE` | No | auto-detect | Override monitor topic message type |

Exactly one of `--delta`, `--above`, `--below`, `--equals` is required.

**Move forward until x-position increases by 1 m:**
```bash
python3 {baseDir}/scripts/ros2_cli.py topics publish-until /cmd_vel \
  '{"linear":{"x":0.3,"y":0,"z":0},"angular":{"x":0,"y":0,"z":0}}' \
  --monitor /odom --field pose.pose.position.x --delta 1.0 --timeout 30
```

**Stop when front lidar range drops below 0.5 m:**
```bash
python3 {baseDir}/scripts/ros2_cli.py topics publish-until /cmd_vel \
  '{"linear":{"x":0.2,"y":0,"z":0},"angular":{"x":0,"y":0,"z":0}}' \
  --monitor /scan --field ranges.0 --below 0.5 --timeout 60
```

Output — condition met:
```json
{
  "success": true, "condition_met": true,
  "topic": "/cmd_vel", "monitor_topic": "/odom",
  "field": "pose.pose.position.x", "operator": "delta", "threshold": 1.0,
  "start_value": 0.12, "end_value": 1.15,
  "duration": 4.2, "published_count": 42,
  "start_msg": {}, "end_msg": {}
}
```

Output — timeout (condition not met):
```json
{
  "success": false, "condition_met": false,
  "error": "Timeout after 30s: condition not met",
  "start_value": 0.12, "end_value": 0.43,
  "duration": 30.0, "published_count": 298
}
```

---

## topics publish-continuous `<topic>` `<json_message>` [options]

Publish a message at a fixed rate for a mandatory bounded duration. Stops early on `Ctrl+C` (local use). `--timeout` is required — indefinite publishing is disabled for safety because AI agents running over Discord, Telegram, etc. cannot send `Ctrl+C`.

| Argument | Required | Description |
|----------|----------|-------------|
| `topic` | Yes | Topic to publish to |
| `json_message` | Yes | JSON string of the message payload |

| Option | Required | Default | Description |
|--------|----------|---------|-------------|
| `--timeout SECONDS` | **Yes** | — | Maximum publish duration in seconds |
| `--rate HZ` | No | `10` | Publish rate in Hz |
| `--msg-type TYPE` | No | auto-detect | Override message type |

**Publish velocity for 5 seconds:**
```bash
python3 {baseDir}/scripts/ros2_cli.py topics publish-continuous /cmd_vel \
  '{"linear":{"x":0.3,"y":0,"z":0},"angular":{"x":0,"y":0,"z":0}}' \
  --timeout 5
```

Output:
```json
{
  "success": true, "topic": "/cmd_vel",
  "published_count": 50, "duration": 5.0,
  "rate": 10.0, "stopped_by": "timeout"
}
```

`stopped_by` is `"timeout"` when the duration elapsed normally, or `"keyboard_interrupt"` when stopped with `Ctrl+C`.

---

## services list

List all available services.

```bash
python3 {baseDir}/scripts/ros2_cli.py services list
```

Output:
```json
{
  "services": ["/clear", "/kill", "/reset", "/spawn", "/turtle1/set_pen", "/turtle1/teleport_absolute"],
  "count": 6
}
```

---

## services type `<service>`

Get the type of a specific service.

| Argument | Required | Description |
|----------|----------|-------------|
| `service` | Yes | Service name (e.g. `/spawn`, `/reset`) |

```bash
python3 {baseDir}/scripts/ros2_cli.py services type /spawn
```

Output:
```json
{"service": "/spawn", "type": "turtlesim/Spawn"}
```

---

## services details `<service>`

Get service details including type, request fields, and response fields.

| Argument | Required | Description |
|----------|----------|-------------|
| `service` | Yes | Service name |

```bash
python3 {baseDir}/scripts/ros2_cli.py services details /spawn
```

Output:
```json
{
  "service": "/spawn",
  "type": "turtlesim/Spawn",
  "request": {"x": "float32", "y": "float32", "theta": "float32", "name": "string"},
  "response": {"name": "string"}
}
```

---

## services call `<service>` `<json_request>`

Call a service with a JSON request payload.

| Argument | Required | Description |
|----------|----------|-------------|
| `service` | Yes | Service name |
| `json_request` | Yes | JSON string of the request arguments |

**Reset turtlesim:**
```bash
python3 {baseDir}/scripts/ros2_cli.py services call /reset '{}'
```

**Spawn a new turtle:**
```bash
python3 {baseDir}/scripts/ros2_cli.py services call /spawn \
  '{"x":3.0,"y":3.0,"theta":0.0,"name":"turtle2"}'
```

Output:
```json
{"service": "/spawn", "success": true, "result": {"name": "turtle2"}}
```

**Set pen color:**
```bash
python3 {baseDir}/scripts/ros2_cli.py services call /turtle1/set_pen \
  '{"r":255,"g":0,"b":0,"width":3,"off":0}'
```

---

## nodes list

List all active ROS nodes.

```bash
python3 {baseDir}/scripts/ros2_cli.py nodes list
```

Output:
```json
{
  "nodes": ["/turtlesim", "/ros2cli"],
  "count": 2
}
```

---

## nodes details `<node>`

Get node details including topics and services.

| Argument | Required | Description |
|----------|----------|-------------|
| `node` | Yes | Node name (e.g. `/turtlesim`) |

```bash
python3 {baseDir}/scripts/ros2_cli.py nodes details /turtlesim
```

Output:
```json
{
  "node": "/turtlesim",
  "publishers": ["/turtle1/color_sensor", "/turtle1/pose", "/rosout"],
  "subscribers": ["/turtle1/cmd_vel"],
  "services": ["/clear", "/kill", "/reset", "/spawn", "/turtle1/set_pen", "/turtle1/teleport_absolute"]
}
```

---

## params list `<node>`

List all parameters for a specific node.

| Argument | Required | Description |
|----------|----------|-------------|
| `node` | Yes | Node name (e.g. `/turtlesim`) |

```bash
python3 {baseDir}/scripts/ros2_cli.py params list /turtlesim
```

Output:
```json
{
  "node": "/turtlesim",
  "parameters": ["/turtlesim:background_r", "/turtlesim:background_g", "/turtlesim:background_b", "/turtlesim:use_sim_time"],
  "count": 4
}
```

---

## params get `<node:param_name>` or `<node> <param_name>`

Get a parameter value. Accepts either colon-separated or space-separated format.

| Argument | Required | Description |
|----------|----------|-------------|
| `name` | Yes | Node name with optional `:param_name` suffix, or node name alone |
| `param_name` | No | Parameter name (when using space-separated format) |

```bash
# Colon format
python3 {baseDir}/scripts/ros2_cli.py params get /turtlesim:background_r

# Space-separated format (equivalent)
python3 {baseDir}/scripts/ros2_cli.py params get /base_controller base_frame_id
```

Output:
```json
{"name": "/turtlesim:background_r", "value": "69", "exists": true}
```

---

## params set `<node:param_name>` `<value>` or `<node> <param_name> <value>`

Set a parameter value. Accepts either colon-separated or space-separated format.

| Argument | Required | Description |
|----------|----------|-------------|
| `name` | Yes | Node name with optional `:param_name` suffix, or node name alone |
| `value` | Yes | New value to set (when using colon format) |
| `param_name` | No | Parameter name (when using space-separated format) |
| `extra_value` | No | Value to set (when using space-separated format) |

```bash
# Colon format
python3 {baseDir}/scripts/ros2_cli.py params set /turtlesim:background_r 255

# Space-separated format (equivalent)
python3 {baseDir}/scripts/ros2_cli.py params set /base_controller base_frame_id base_link_new
```

Output:
```json
{"name": "/turtlesim:background_r", "value": "255", "success": true}
```

If the parameter is read-only:
```json
{"name": "/base_controller:base_frame_id", "value": "base_link_new", "success": false, "error": "Parameter is read-only and cannot be changed at runtime", "read_only": true}
```

---

## actions list

List all available action servers.

```bash
python3 {baseDir}/scripts/ros2_cli.py actions list
```

Output:
```json
{
  "actions": ["/turtle1/rotate_absolute"],
  "count": 1
}
```

---

## actions details `<action>`

Get action details including goal, result, and feedback field structures.

| Argument | Required | Description |
|----------|----------|-------------|
| `action` | Yes | Action server name |

```bash
python3 {baseDir}/scripts/ros2_cli.py actions details /turtle1/rotate_absolute
```

Output:
```json
{
  "action": "/turtle1/rotate_absolute",
  "action_type": "turtlesim/action/RotateAbsolute",
  "goal": {"theta": "float32"},
  "result": {"delta": "float32"},
  "feedback": {"remaining": "float32"}
}
```

---

## actions send `<action>` `<json_goal>`

Send an action goal and wait for the result.

| Argument | Required | Description |
|----------|----------|-------------|
| `action` | Yes | Action server name |
| `json_goal` | Yes | JSON string of the goal arguments |

```bash
python3 {baseDir}/scripts/ros2_cli.py actions send /turtle1/rotate_absolute \
  '{"theta":3.14}'
```

Output:
```json
{
  "action": "/turtle1/rotate_absolute",
  "success": true,
  "goal_id": "goal_1709312000000",
  "result": {"delta": -1.584}
}
```

If timeout:
```json
{
  "action": "/turtle1/rotate_absolute",
  "goal_id": "goal_1709312000000",
  "success": false,
  "error": "Timeout after 5.0s"
}
```
