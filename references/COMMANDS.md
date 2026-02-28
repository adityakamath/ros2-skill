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

## topics details `<topic>` / topics info `<topic>`

Get topic details including message type, publishers, and subscribers. `topics info` is an alias for `topics details`.

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

## topics subscribe `<topic>` [options] / topics echo / topics sub

Subscribe to a topic and receive messages. `topics echo` and `topics sub` are identical aliases.

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

## topics publish `<topic>` `<json_message>` [options] / topics pub / topics publish-continuous

Publish a message to a topic. `pub` and `publish-continuous` are aliases for `publish` — all three share the same handler.

Without `--duration` / `--timeout`, sends once (single-shot). With either flag, publishes repeatedly at `--rate` Hz for the specified seconds, then stops. `--duration` and `--timeout` are interchangeable — use whichever is more natural. Output for the repeated path includes `stopped_by: "timeout" | "keyboard_interrupt"`.

| Argument | Required | Description |
|----------|----------|-------------|
| `topic` | Yes | Topic to publish to |
| `json_message` | Yes | JSON string of the message payload |

| Option | Default | Description |
|--------|---------|-------------|
| `--msg-type TYPE` | auto-detect | Message type override |
| `--duration SECONDS` | _(none)_ | Publish repeatedly for this duration (`--timeout` is an identical alias) |
| `--timeout SECONDS` | _(none)_ | Alias for `--duration` |
| `--rate HZ` | `10` | Publish rate in Hz |

**Single-shot (trigger, one-time command):**
```bash
python3 {baseDir}/scripts/ros2_cli.py topics publish /turtle1/cmd_vel \
  '{"linear":{"x":2.0,"y":0,"z":0},"angular":{"x":0,"y":0,"z":0}}'
```

Output (single-shot):
```json
{"success": true, "topic": "/turtle1/cmd_vel", "msg_type": "geometry_msgs/Twist"}
```

**Move forward for 3 seconds (recommended for velocity):**
```bash
python3 {baseDir}/scripts/ros2_cli.py topics publish /cmd_vel \
  '{"linear":{"x":1.0,"y":0,"z":0},"angular":{"x":0,"y":0,"z":0}}' --duration 3
```

Output (repeated):
```json
{"success": true, "topic": "/cmd_vel", "msg_type": "geometry_msgs/Twist", "duration": 3.002, "rate": 10.0, "published_count": 30, "stopped_by": "timeout"}
```

**Equivalent using `--timeout`:**
```bash
python3 {baseDir}/scripts/ros2_cli.py topics publish /cmd_vel \
  '{"linear":{"x":0.3,"y":0,"z":0},"angular":{"x":0,"y":0,"z":0}}' --timeout 5
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

## topics publish-sequence `<topic>` `<json_messages>` `<json_durations>` [options] / topics pub-seq

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

**Discovery workflow** — before running, introspect the live robot:
1. `topics find nav_msgs/msg/Odometry` — find odometry/feedback topic
2. `topics message <type>` — inspect field paths
3. `topics subscribe <monitor_topic> --duration 2` — read current value (baseline for `--delta`)

See the _Goal-Oriented Commands_ workflow section in `SKILL.md` for a full lookup table and examples.

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

## topics publish-continuous

**Alias for `topics publish`.** See [topics publish](#topics-publish-topic-json_message-options--topics-pub--topics-publish-continuous) for full documentation.

`--timeout` and `--duration` are interchangeable on this alias. Example:

```bash
python3 {baseDir}/scripts/ros2_cli.py topics publish-continuous /cmd_vel \
  '{"linear":{"x":0.3,"y":0,"z":0},"angular":{"x":0,"y":0,"z":0}}' \
  --timeout 5
```

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

## services details `<service>` / services info `<service>`

Get service details including type, request fields, and response fields. `services info` is an alias for `services details`.

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

## nodes details `<node>` / nodes info `<node>`

Get node details including topics, services, action servers, and action clients. `nodes info` is an alias for `nodes details`.

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
  "services": ["/clear", "/kill", "/reset", "/spawn", "/turtle1/set_pen", "/turtle1/teleport_absolute"],
  "action_servers": ["/turtle1/rotate_absolute"],
  "action_clients": []
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

## actions details `<action>` / actions info `<action>`

Get action details including goal, result, and feedback field structures. `actions info` is an alias for `actions details`.

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

## actions send `<action>` `<json_goal>` / actions send-goal

Send an action goal and wait for the result. `actions send-goal` is an alias for `actions send`.

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

---

## topics hz `<topic>` [options]

Measure the publish rate of a topic. Collects inter-message intervals over a sliding window and reports rate, min/max delta, and standard deviation.

| Argument | Required | Description |
|----------|----------|-------------|
| `topic` | Yes | Topic name |

| Option | Default | Description |
|--------|---------|-------------|
| `--window N` | `10` | Number of inter-message intervals to sample |
| `--timeout SECONDS` | `10` | Max wait time; returns error if fewer than 2 messages arrive |

```bash
python3 {baseDir}/scripts/ros2_cli.py topics hz /turtle1/pose
python3 {baseDir}/scripts/ros2_cli.py topics hz /scan --window 20 --timeout 15
```

Output:
```json
{
  "topic": "/turtle1/pose",
  "rate": 62.4831,
  "min_delta": 0.015832,
  "max_delta": 0.016201,
  "std_dev": 0.000089,
  "samples": 10
}
```

If fewer than 2 messages arrive within the timeout:
```json
{"error": "Fewer than 2 messages received within 10.0s on '/turtle1/pose'"}
```

---

## topics find `<message_type>`

Find all topics publishing a specific message type. Accepts both `/msg/` and short format (normalised for comparison).

| Argument | Required | Description |
|----------|----------|-------------|
| `message_type` | Yes | Message type (e.g. `geometry_msgs/msg/Twist` or `geometry_msgs/Twist`) |

```bash
python3 {baseDir}/scripts/ros2_cli.py topics find geometry_msgs/msg/Twist
python3 {baseDir}/scripts/ros2_cli.py topics find geometry_msgs/Twist
```

Output:
```json
{
  "message_type": "geometry_msgs/msg/Twist",
  "topics": ["/cmd_vel", "/turtle1/cmd_vel"],
  "count": 2
}
```

If no topics match:
```json
{"message_type": "geometry_msgs/msg/Twist", "topics": [], "count": 0}
```

---

## services find `<service_type>`

Find all services of a specific service type. Accepts both `/srv/` and short format (normalised for comparison).

| Argument | Required | Description |
|----------|----------|-------------|
| `service_type` | Yes | Service type (e.g. `std_srvs/srv/Empty` or `std_srvs/Empty`) |

```bash
python3 {baseDir}/scripts/ros2_cli.py services find std_srvs/srv/Empty
python3 {baseDir}/scripts/ros2_cli.py services find std_srvs/Empty
```

Output:
```json
{
  "service_type": "std_srvs/srv/Empty",
  "services": ["/clear", "/reset"],
  "count": 2
}
```

---

## actions type `<action>`

Get the type of an action server.

| Argument | Required | Description |
|----------|----------|-------------|
| `action` | Yes | Action server name |

```bash
python3 {baseDir}/scripts/ros2_cli.py actions type /turtle1/rotate_absolute
```

Output:
```json
{"action": "/turtle1/rotate_absolute", "type": "turtlesim/action/RotateAbsolute"}
```

If the action is not found in the ROS graph:
```json
{"error": "Action '/turtle1/rotate_absolute' not found in the ROS graph"}
```

---

## actions send `<action>` `<json_goal>` `[--feedback]` / actions send-goal

Send an action goal and wait for the result. The `--feedback` flag collects intermediate feedback messages during execution.

| Argument | Required | Description |
|----------|----------|-------------|
| `action` | Yes | Action server name |
| `json_goal` | Yes | JSON string of the goal arguments |

| Option | Default | Description |
|--------|---------|-------------|
| `--feedback` | _(off)_ | Collect feedback messages; added to output as `feedback_msgs` |
| `--timeout SECONDS` | `30` | Timeout waiting for result |

```bash
python3 {baseDir}/scripts/ros2_cli.py actions send /turtle1/rotate_absolute '{"theta":3.14}'

# With feedback collection
python3 {baseDir}/scripts/ros2_cli.py actions send /turtle1/rotate_absolute \
  '{"theta":3.14}' --feedback
```

Output (without `--feedback`):
```json
{
  "action": "/turtle1/rotate_absolute",
  "success": true,
  "goal_id": "goal_1709312000000",
  "result": {"delta": -1.584}
}
```

Output (with `--feedback`):
```json
{
  "action": "/turtle1/rotate_absolute",
  "success": true,
  "goal_id": "goal_1709312000000",
  "result": {"delta": -1.584},
  "feedback_msgs": [
    {"remaining": 2.1},
    {"remaining": 1.4},
    {"remaining": 0.0}
  ]
}
```

---

## actions cancel `<action>` [options]

Cancel all in-flight goals on an action server. Sends a `CancelGoal` request with zero UUID and zero timestamp, which cancels all goals per the ROS 2 specification.

| Argument | Required | Description |
|----------|----------|-------------|
| `action` | No | Action server name (required in practice) |

| Option | Default | Description |
|--------|---------|-------------|
| `--timeout SECONDS` | `5` | Service call timeout |

```bash
python3 {baseDir}/scripts/ros2_cli.py actions cancel /turtle1/rotate_absolute
```

Output:
```json
{
  "action": "/turtle1/rotate_absolute",
  "return_code": 0,
  "cancelled_goals": []
}
```

`return_code` 0 = success. `cancelled_goals` is a list of goal IDs that were cancelled (may be empty if no goals were active at cancel time).

---

## topics bw `<topic>` [options]

Measure the bandwidth of a topic in bytes per second. Serialises each received message to count its byte size.

| Argument | Required | Description |
|----------|----------|-------------|
| `topic` | Yes | Topic name |

| Option | Default | Description |
|--------|---------|-------------|
| `--window N` | `10` | Number of message samples to collect |
| `--timeout SECONDS` | `10` | Max wait time; returns error if fewer than 2 messages arrive |

```bash
python3 {baseDir}/scripts/ros2_cli.py topics bw /camera/image_raw
python3 {baseDir}/scripts/ros2_cli.py topics bw /scan --window 20 --timeout 15
```

Output:
```json
{
  "topic": "/camera/image_raw",
  "bw": 9437184.0,
  "bytes_per_msg": 921604,
  "rate": 10.24,
  "samples": 10
}
```

`bw` is in bytes/s. `bytes_per_msg` is the mean serialised message size.

If fewer than 2 messages arrive:
```json
{"error": "Fewer than 2 messages received within 10.0s on '/camera/image_raw'"}
```

---

## topics delay `<topic>` [options]

Measure the end-to-end latency between a message's `header.stamp` and the wall clock time at which it is received. Requires messages with a `std_msgs/Header` header field.

| Argument | Required | Description |
|----------|----------|-------------|
| `topic` | Yes | Topic name |

| Option | Default | Description |
|--------|---------|-------------|
| `--window N` | `10` | Number of latency samples to collect |
| `--timeout SECONDS` | `10` | Max wait time |

```bash
python3 {baseDir}/scripts/ros2_cli.py topics delay /odom
python3 {baseDir}/scripts/ros2_cli.py topics delay /scan --window 20
```

Output:
```json
{
  "topic": "/odom",
  "mean_delay": 0.003241,
  "min_delay": 0.001823,
  "max_delay": 0.005012,
  "std_dev": 0.000891,
  "samples": 10
}
```

All delay values are in seconds.

If the message has no `header.stamp`:
```json
{"error": "Messages on '/cmd_vel' have no header.stamp field"}
```

---

## params describe `<node:param_name>` or `<node> <param_name>`

Describe a parameter's type and metadata via the `DescribeParameters` service.

| Argument | Required | Description |
|----------|----------|-------------|
| `name` | Yes | Node name with `:param_name` suffix, or node name alone |
| `param_name` | No | Parameter name (when using space-separated format) |

| Option | Default | Description |
|--------|---------|-------------|
| `--timeout SECONDS` | `5` | Service call timeout |

```bash
python3 {baseDir}/scripts/ros2_cli.py params describe /turtlesim:background_r
python3 {baseDir}/scripts/ros2_cli.py params describe /turtlesim background_r
```

Output:
```json
{
  "name": "/turtlesim:background_r",
  "type": "integer",
  "description": "",
  "read_only": false,
  "dynamic_typing": false,
  "additional_constraints": ""
}
```

---

## params dump `<node>` [options]

Export all parameters for a node as a flat `{param_name: value}` dict. Calls `ListParameters` then `GetParameters` internally.

| Argument | Required | Description |
|----------|----------|-------------|
| `node` | Yes | Node name (e.g. `/turtlesim`) |

| Option | Default | Description |
|--------|---------|-------------|
| `--timeout SECONDS` | `5` | Service call timeout |

```bash
python3 {baseDir}/scripts/ros2_cli.py params dump /turtlesim
```

Output:
```json
{
  "node": "/turtlesim",
  "parameters": {
    "background_r": 69,
    "background_g": 86,
    "background_b": 255,
    "use_sim_time": false
  }
}
```

---

## params load `<node>` `<json_or_file>` [options]

Bulk-set parameters on a node from a JSON string or a JSON file path. Each parameter is set via `SetParameters`. Reports per-parameter success/failure.

| Argument | Required | Description |
|----------|----------|-------------|
| `node` | Yes | Node name |
| `params` | Yes | JSON string `{"param": value}` or path to a JSON file |

| Option | Default | Description |
|--------|---------|-------------|
| `--timeout SECONDS` | `5` | Service call timeout |

```bash
# From JSON string
python3 {baseDir}/scripts/ros2_cli.py params load /turtlesim \
  '{"background_r":255,"background_g":0,"background_b":0}'

# From a JSON file
python3 {baseDir}/scripts/ros2_cli.py params load /turtlesim /tmp/turtlesim_params.json
```

Output:
```json
{
  "node": "/turtlesim",
  "results": [
    {"name": "background_r", "value": 255, "success": true},
    {"name": "background_g", "value": 0, "success": true},
    {"name": "background_b", "value": 0, "success": true}
  ],
  "loaded": 3,
  "failed": 0
}
```

---

## params delete `<node>` `<param_name>` [`<extra_names>` ...] [options]

Delete one or more parameters from a node via the `DeleteParameters` service.

| Argument | Required | Description |
|----------|----------|-------------|
| `node` | Yes | Node name |
| `param_name` | Yes | First parameter name to delete |
| `extra_names` | No | Additional parameter names to delete |

| Option | Default | Description |
|--------|---------|-------------|
| `--timeout SECONDS` | `5` | Service call timeout |

```bash
python3 {baseDir}/scripts/ros2_cli.py params delete /turtlesim background_r
python3 {baseDir}/scripts/ros2_cli.py params delete /turtlesim background_r background_g background_b
```

Output:
```json
{"node": "/turtlesim", "deleted": ["background_r"], "count": 1}
```

---

## interface show `<message_type>` / interface proto `<message_type>`

Get the field structure of a message, service, or action type. `interface proto` is an alias for `interface show`. Both use the same handler as `topics message` — all three commands are interchangeable.

| Argument | Required | Description |
|----------|----------|-------------|
| `message_type` | Yes | Message/service/action type (e.g. `geometry_msgs/Twist`) |

```bash
python3 {baseDir}/scripts/ros2_cli.py interface show geometry_msgs/Twist
python3 {baseDir}/scripts/ros2_cli.py interface proto std_srvs/SetBool
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

## interface list

List all available ROS 2 interfaces (messages, services, and actions) across all installed packages using `rosidl_runtime_py`.

```bash
python3 {baseDir}/scripts/ros2_cli.py interface list
```

Output:
```json
{
  "messages": ["geometry_msgs/msg/Twist", "sensor_msgs/msg/LaserScan", "..."],
  "services": ["std_srvs/srv/Empty", "std_srvs/srv/SetBool", "..."],
  "actions": ["turtlesim/action/RotateAbsolute", "..."],
  "count": 512
}
```

---

## interface packages

List all packages that contain at least one ROS 2 interface.

```bash
python3 {baseDir}/scripts/ros2_cli.py interface packages
```

Output:
```json
{"packages": ["action_msgs", "geometry_msgs", "nav_msgs", "sensor_msgs", "std_msgs", "turtlesim", "..."], "count": 42}
```

---

## interface package `[<package>]`

List all interfaces (messages, services, actions) in a specific package.

| Argument | Required | Description |
|----------|----------|-------------|
| `package` | No | Package name (e.g. `geometry_msgs`) |

```bash
python3 {baseDir}/scripts/ros2_cli.py interface package geometry_msgs
```

Output:
```json
{
  "package": "geometry_msgs",
  "messages": ["geometry_msgs/msg/Accel", "geometry_msgs/msg/Pose", "geometry_msgs/msg/Twist", "geometry_msgs/msg/Vector3", "..."],
  "services": [],
  "actions": []
}
```
