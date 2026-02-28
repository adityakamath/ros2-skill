# Command Reference

Full reference for all `ros2_cli.py` commands with arguments, options, ROS 2 CLI equivalents, and output examples.

All commands output JSON. Errors return `{"error": "..."}`.

---

## version

Detect the ROS 2 version and distro name.

**ROS 2 CLI equivalent:** `ros2 doctor --report` (verbose), or `echo $ROS_DISTRO`

```bash
python3 {baseDir}/scripts/ros2_cli.py version
```

Output:
```json
{"version": "2", "distro": "humble", "domain_id": 0}
```

---

## estop

Emergency stop for mobile robots. Auto-detects the velocity command topic and message type, then publishes zero velocity.

**Note:** For mobile robots only (differential drive, omnidirectional, etc.). Does NOT work for robotic arms or manipulators.

**ROS 2 CLI equivalent:** `ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist '{}'`

| Option | Default | Description |
|--------|---------|-------------|
| `--topic TOPIC` | auto-detect | Override velocity topic (detects from `/cmd_vel`, `/cmd_vel_nav`, etc.) |

```bash
python3 {baseDir}/scripts/ros2_cli.py estop
python3 {baseDir}/scripts/ros2_cli.py estop --topic /cmd_vel_nav
```

Output:
```json
{"success": true, "topic": "/cmd_vel", "type": "geometry_msgs/Twist", "message": "Emergency stop activated (mobile robot stopped)"}
```

Error (no velocity topic found):
```json
{"error": "Could not find velocity command topic", "hint": "This command is for mobile robots only (not arms). Ensure the robot has a /cmd_vel topic."}
```

---

## topics list / topics ls

List all active topics with their message types.

**Aliases:** `topics ls`

**ROS 2 CLI equivalent:** `ros2 topic list -t`

```bash
python3 {baseDir}/scripts/ros2_cli.py topics list
python3 {baseDir}/scripts/ros2_cli.py topics ls
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

**ROS 2 CLI equivalent:** `ros2 topic type /topic`

| Argument | Required | Description |
|----------|----------|-------------|
| `topic` | Yes | Topic name (e.g. `/cmd_vel`, `/turtle1/pose`) |

```bash
python3 {baseDir}/scripts/ros2_cli.py topics type /turtle1/cmd_vel
python3 {baseDir}/scripts/ros2_cli.py topics type /scan
```

Output:
```json
{"topic": "/turtle1/cmd_vel", "type": "geometry_msgs/Twist"}
```

---

## topics details `<topic>` / topics info `<topic>`

Get topic details including message type, publishers, and subscribers.

**Aliases:** `topics info`

**ROS 2 CLI equivalent:** `ros2 topic info /topic`

| Argument | Required | Description |
|----------|----------|-------------|
| `topic` | Yes | Topic name |

```bash
python3 {baseDir}/scripts/ros2_cli.py topics details /turtle1/cmd_vel
python3 {baseDir}/scripts/ros2_cli.py topics info /turtle1/cmd_vel
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

## topics message `<message_type>` / topics message-structure / topics message-struct

Get the full field structure of a message type as a JSON template.

**Aliases:** `topics message-structure`, `topics message-struct`

**ROS 2 CLI equivalent:** `ros2 interface show geometry_msgs/msg/Twist`

| Argument | Required | Description |
|----------|----------|-------------|
| `message_type` | Yes | Full message type (e.g. `geometry_msgs/Twist`, `sensor_msgs/LaserScan`) |

```bash
python3 {baseDir}/scripts/ros2_cli.py topics message geometry_msgs/Twist
python3 {baseDir}/scripts/ros2_cli.py topics message-structure geometry_msgs/msg/Twist
python3 {baseDir}/scripts/ros2_cli.py topics message-struct sensor_msgs/LaserScan
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

Subscribe to a topic and receive messages. Without `--duration`, returns the first message received. With `--duration`, collects multiple messages for the specified number of seconds.

**Aliases:** `topics echo`, `topics sub`

**ROS 2 CLI equivalent:** `ros2 topic echo /topic`

| Argument | Required | Description |
|----------|----------|-------------|
| `topic` | Yes | Topic to subscribe to |

| Option | Default | Description |
|--------|---------|-------------|
| `--msg-type TYPE` | auto-detect | Override message type (usually not needed) |
| `--duration SECONDS` | _(none)_ | Collect messages for this duration; without this flag, returns first message only |
| `--max-messages N` / `--max-msgs N` | `100` | Maximum messages to collect (only applies with `--duration`) |
| `--timeout SECONDS` | `5` | Timeout waiting for first message (single-message mode only) |

**Wait for first message (single-message mode):**
```bash
python3 {baseDir}/scripts/ros2_cli.py topics subscribe /turtle1/pose
python3 {baseDir}/scripts/ros2_cli.py topics echo /odom
python3 {baseDir}/scripts/ros2_cli.py topics sub /scan
```

Output (single message):
```json
{
  "msg": {"x": 5.544, "y": 5.544, "theta": 0.0, "linear_velocity": 0.0, "angular_velocity": 0.0}
}
```

**Collect messages over time:**
```bash
python3 {baseDir}/scripts/ros2_cli.py topics subscribe /odom --duration 5
python3 {baseDir}/scripts/ros2_cli.py topics echo /scan --duration 10 --max-messages 50
python3 {baseDir}/scripts/ros2_cli.py topics sub /joint_states --duration 3 --max-msgs 20
```

Output (multiple messages):
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

Error (timeout):
```json
{"error": "Timeout waiting for message"}
```

---

## topics publish `<topic>` `<json_message>` [options] / topics pub / topics publish-continuous

Publish a message to a topic. Without `--duration`/`--timeout`, sends once (single-shot). With either flag, publishes repeatedly at `--rate` Hz for the specified duration, then stops.

**Aliases:** `topics pub`, `topics publish-continuous` (all three share the same handler)

**ROS 2 CLI equivalent:** `ros2 topic pub /topic geometry_msgs/msg/Twist '{"linear": {"x": 1.0}}'`

| Argument | Required | Description |
|----------|----------|-------------|
| `topic` | Yes | Topic to publish to |
| `json_message` | Yes | JSON string of the message payload |

| Option | Default | Description |
|--------|---------|-------------|
| `--msg-type TYPE` | auto-detect | Override message type |
| `--duration SECONDS` | _(none)_ | Publish repeatedly for this many seconds; identical to `--timeout` |
| `--timeout SECONDS` | _(none)_ | Alias for `--duration`; interchangeable |
| `--rate HZ` | `10` | Publish rate in Hz |

**Single-shot (one message):**
```bash
python3 {baseDir}/scripts/ros2_cli.py topics publish /trigger '{"data": ""}'
python3 {baseDir}/scripts/ros2_cli.py topics pub /turtle1/cmd_vel \
  '{"linear":{"x":2.0,"y":0,"z":0},"angular":{"x":0,"y":0,"z":0}}'
```

Output (single-shot):
```json
{"success": true, "topic": "/turtle1/cmd_vel", "msg_type": "geometry_msgs/Twist"}
```

**Publish for a duration (recommended for velocity commands):**
```bash
# Move forward for 3 seconds
python3 {baseDir}/scripts/ros2_cli.py topics publish /cmd_vel \
  '{"linear":{"x":1.0,"y":0,"z":0},"angular":{"x":0,"y":0,"z":0}}' --duration 3

# Rotate left for 2 seconds (using --timeout alias)
python3 {baseDir}/scripts/ros2_cli.py topics pub /cmd_vel \
  '{"linear":{"x":0,"y":0,"z":0},"angular":{"x":0,"y":0,"z":0.5}}' --timeout 2

# Stop the robot
python3 {baseDir}/scripts/ros2_cli.py topics publish /cmd_vel \
  '{"linear":{"x":0,"y":0,"z":0},"angular":{"x":0,"y":0,"z":0}}'

# Using publish-continuous alias
python3 {baseDir}/scripts/ros2_cli.py topics publish-continuous /cmd_vel \
  '{"linear":{"x":0.3,"y":0,"z":0},"angular":{"x":0,"y":0,"z":0}}' --timeout 5
```

Output (duration mode):
```json
{"success": true, "topic": "/cmd_vel", "msg_type": "geometry_msgs/Twist", "duration": 3.002, "rate": 10.0, "published_count": 30, "stopped_by": "timeout"}
```

`stopped_by` is `"timeout"` when the duration expires normally, or `"keyboard_interrupt"` if stopped early with Ctrl+C.

---

## topics publish-sequence `<topic>` `<json_messages>` `<json_durations>` [options] / topics pub-seq

Publish a sequence of messages in order. Each message is repeated at `--rate` Hz for its corresponding duration before moving to the next. Arrays must be the same length. The final message should usually be all-zeros to stop the robot.

**Aliases:** `topics pub-seq`

**ROS 2 CLI equivalent:** No direct equivalent (requires scripting multiple `ros2 topic pub` calls)

| Argument | Required | Description |
|----------|----------|-------------|
| `topic` | Yes | Topic to publish to |
| `json_messages` | Yes | JSON array of message objects, published in order |
| `json_durations` | Yes | JSON array of durations in seconds — one per message |

| Option | Default | Description |
|--------|---------|-------------|
| `--msg-type TYPE` | auto-detect | Override message type |
| `--rate HZ` | `10` | Publish rate in Hz for each step |

**Move forward 3 seconds, then stop:**
```bash
python3 {baseDir}/scripts/ros2_cli.py topics publish-sequence /cmd_vel \
  '[{"linear":{"x":1.0,"y":0,"z":0},"angular":{"x":0,"y":0,"z":0}},{"linear":{"x":0,"y":0,"z":0},"angular":{"x":0,"y":0,"z":0}}]' \
  '[3.0, 0.5]'
```

**Forward 2s, turn left 1s, forward 2s, stop:**
```bash
python3 {baseDir}/scripts/ros2_cli.py topics pub-seq /cmd_vel \
  '[{"linear":{"x":0.5},"angular":{"z":0}},{"linear":{"x":0},"angular":{"z":0.8}},{"linear":{"x":0.5},"angular":{"z":0}},{"linear":{"x":0},"angular":{"z":0}}]' \
  '[2.0, 1.0, 2.0, 0.5]'
```

**Draw a square (turtlesim):**
```bash
python3 {baseDir}/scripts/ros2_cli.py topics publish-sequence /turtle1/cmd_vel \
  '[{"linear":{"x":2},"angular":{"z":0}},{"linear":{"x":0},"angular":{"z":1.5708}},{"linear":{"x":2},"angular":{"z":0}},{"linear":{"x":0},"angular":{"z":1.5708}},{"linear":{"x":2},"angular":{"z":0}},{"linear":{"x":0},"angular":{"z":1.5708}},{"linear":{"x":2},"angular":{"z":0}},{"linear":{"x":0},"angular":{"z":1.5708}},{"linear":{"x":0},"angular":{"z":0}}]' \
  '[1,1,1,1,1,1,1,1,0.5]'
```

Output:
```json
{"success": true, "published_count": 35, "topic": "/cmd_vel", "rate": 10.0}
```

Error (array length mismatch):
```json
{"error": "messages and durations arrays must have the same length"}
```

---

## topics publish-until `<topic>` `<json_message>` [options]

Publish a message at a fixed rate while simultaneously monitoring a second topic. Stops as soon as a condition on the monitored field is satisfied, or after the safety timeout. Supports single-field conditions and N-dimensional Euclidean distance.

**ROS 2 CLI equivalent:** No equivalent (requires custom scripting)

**Discovery workflow:** Before running, always introspect the robot:
1. `topics find nav_msgs/msg/Odometry` — find the feedback topic
2. `topics message nav_msgs/msg/Odometry` — inspect field paths
3. `topics subscribe /odom --duration 2` — read current value (baseline for `--delta`)

| Argument | Required | Description |
|----------|----------|-------------|
| `topic` | Yes | Topic to publish command messages to |
| `json_message` | Yes | JSON string of the message payload |

| Option | Required | Default | Description |
|--------|----------|---------|-------------|
| `--monitor TOPIC` | Yes | — | Topic to watch for the stop condition |
| `--field PATH [PATH...]` | Yes | — | One or more dot-separated field paths in the monitor message (e.g. `pose.pose.position.x`). Provide multiple paths with `--euclidean`. |
| `--euclidean` | No | off | Compute Euclidean distance across all `--field` paths; requires `--delta`. Works for any number of numeric fields (2D, 3D, joint-space, etc.) |
| `--delta N` | One required | — | Stop when field changes by ±N from first observed value; or when Euclidean distance ≥ N with `--euclidean` |
| `--above N` | One required | — | Stop when field value > N (single-field only) |
| `--below N` | One required | — | Stop when field value < N (single-field only) |
| `--equals V` | One required | — | Stop when field value == V (single-field only) |
| `--rate HZ` | No | `10` | Publish rate in Hz |
| `--timeout SECONDS` | No | `60` | Safety stop if condition not met within this time |
| `--msg-type TYPE` | No | auto-detect | Override publish topic message type |
| `--monitor-msg-type TYPE` | No | auto-detect | Override monitor topic message type |

**Move forward until x-position increases by 1 m (straight path):**
```bash
python3 {baseDir}/scripts/ros2_cli.py topics publish-until /cmd_vel \
  '{"linear":{"x":0.3,"y":0,"z":0},"angular":{"x":0,"y":0,"z":0}}' \
  --monitor /odom --field pose.pose.position.x --delta 1.0 --timeout 30
```

**Move 2 m in XY plane (Euclidean — works for curved/diagonal paths):**
```bash
python3 {baseDir}/scripts/ros2_cli.py topics publish-until /cmd_vel \
  '{"linear":{"x":0.2,"y":0,"z":0},"angular":{"x":0,"y":0,"z":0.3}}' \
  --monitor /odom \
  --field pose.pose.position.x pose.pose.position.y \
  --euclidean --delta 2.0 --timeout 60
```

**Move until joint_3 reaches 1.5 rad:**
```bash
python3 {baseDir}/scripts/ros2_cli.py topics publish-until /arm/cmd \
  '{"joint_3_velocity":0.2}' \
  --monitor /joint_states --field position.2 --equals 1.5 --timeout 20
```

**Stop when front lidar range drops below 0.5 m:**
```bash
python3 {baseDir}/scripts/ros2_cli.py topics publish-until /cmd_vel \
  '{"linear":{"x":0.2,"y":0,"z":0},"angular":{"x":0,"y":0,"z":0}}' \
  --monitor /scan --field ranges.0 --below 0.5 --timeout 60
```

**Stop when temperature exceeds 50°C:**
```bash
python3 {baseDir}/scripts/ros2_cli.py topics publish-until /heater/cmd \
  '{"power":1.0}' \
  --monitor /temperature --field temperature --above 50.0 --timeout 120
```

Output — single-field condition met:
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

Output — Euclidean condition met:
```json
{
  "success": true, "condition_met": true,
  "topic": "/cmd_vel", "monitor_topic": "/odom",
  "fields": ["pose.pose.position.x", "pose.pose.position.y"],
  "operator": "euclidean_delta", "threshold": 2.0,
  "start_values": [0.0, 0.0], "end_values": [1.42, 1.41],
  "euclidean_distance": 2.003,
  "duration": 9.8, "published_count": 98,
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

## topics hz `<topic>` [options]

Measure the publish rate of a topic. Collects inter-message intervals over a sliding window and reports rate, min/max delta, and standard deviation.

**ROS 2 CLI equivalent:** `ros2 topic hz /topic`

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
python3 {baseDir}/scripts/ros2_cli.py topics hz /odom --window 5
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

Error (insufficient messages):
```json
{"error": "Fewer than 2 messages received within 10.0s on '/turtle1/pose'"}
```

---

## topics find `<message_type>`

Find all topics publishing a specific message type. Accepts both `/msg/` and short formats (normalised for comparison).

**ROS 2 CLI equivalent:** `ros2 topic find geometry_msgs/msg/Twist`

| Argument | Required | Description |
|----------|----------|-------------|
| `message_type` | Yes | Message type (e.g. `geometry_msgs/msg/Twist` or `geometry_msgs/Twist`) |

```bash
python3 {baseDir}/scripts/ros2_cli.py topics find geometry_msgs/msg/Twist
python3 {baseDir}/scripts/ros2_cli.py topics find geometry_msgs/Twist
python3 {baseDir}/scripts/ros2_cli.py topics find nav_msgs/msg/Odometry
python3 {baseDir}/scripts/ros2_cli.py topics find sensor_msgs/msg/LaserScan
```

Output:
```json
{
  "message_type": "geometry_msgs/msg/Twist",
  "topics": ["/cmd_vel", "/turtle1/cmd_vel"],
  "count": 2
}
```

No matches:
```json
{"message_type": "geometry_msgs/msg/Twist", "topics": [], "count": 0}
```

---

## topics bw `<topic>` [options]

Measure the bandwidth of a topic in bytes per second. Serialises each received message to measure its size.

**ROS 2 CLI equivalent:** `ros2 topic bw /topic`

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
python3 {baseDir}/scripts/ros2_cli.py topics bw /odom --window 5
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

`bw` is bytes/s. `bytes_per_msg` is mean serialised message size. Error if fewer than 2 messages:
```json
{"error": "Fewer than 2 messages received within 10.0s on '/camera/image_raw'"}
```

---

## topics delay `<topic>` [options]

Measure the end-to-end latency between a message's `header.stamp` and the wall clock at receipt. Requires messages with a `std_msgs/Header` field.

**ROS 2 CLI equivalent:** `ros2 topic delay /topic`

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
python3 {baseDir}/scripts/ros2_cli.py topics delay /camera/image_raw --window 5 --timeout 15
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

All delay values in seconds. Error if no `header.stamp`:
```json
{"error": "Messages on '/cmd_vel' have no header.stamp field"}
```

---

## services list / services ls

List all available services.

**Aliases:** `services ls`

**ROS 2 CLI equivalent:** `ros2 service list -t`

```bash
python3 {baseDir}/scripts/ros2_cli.py services list
python3 {baseDir}/scripts/ros2_cli.py services ls
```

Output:
```json
{
  "services": ["/clear", "/kill", "/reset", "/spawn", "/turtle1/set_pen", "/turtle1/teleport_absolute"],
  "types": ["std_srvs/Empty", "turtlesim/Kill", "std_srvs/Empty", "turtlesim/Spawn", "turtlesim/SetPen", "turtlesim/TeleportAbsolute"],
  "count": 6
}
```

---

## services type `<service>`

Get the type of a specific service.

**ROS 2 CLI equivalent:** `ros2 service type /service`

| Argument | Required | Description |
|----------|----------|-------------|
| `service` | Yes | Service name (e.g. `/spawn`, `/reset`) |

```bash
python3 {baseDir}/scripts/ros2_cli.py services type /spawn
python3 {baseDir}/scripts/ros2_cli.py services type /reset
```

Output:
```json
{"service": "/spawn", "type": "turtlesim/Spawn"}
```

---

## services details `<service>` / services info `<service>`

Get service details including type, request fields, and response fields.

**Aliases:** `services info`

**ROS 2 CLI equivalent:** `ros2 service info /service`

| Argument | Required | Description |
|----------|----------|-------------|
| `service` | Yes | Service name |

```bash
python3 {baseDir}/scripts/ros2_cli.py services details /spawn
python3 {baseDir}/scripts/ros2_cli.py services info /spawn
python3 {baseDir}/scripts/ros2_cli.py services details /turtle1/set_pen
```

Output:
```json
{
  "service": "/spawn",
  "type": "turtlesim/Spawn",
  "request": {"x": 0.0, "y": 0.0, "theta": 0.0, "name": ""},
  "response": {"name": ""}
}
```

---

## services call `<service>` `<json_request>` [options]

Call a service with a JSON request payload and return the response.

**ROS 2 CLI equivalent:** `ros2 service call /service turtlesim/srv/Spawn '{"x": 3.0}'`

| Argument | Required | Description |
|----------|----------|-------------|
| `service` | Yes | Service name |
| `json_request` | Yes | JSON string of the request arguments |

| Option | Default | Description |
|--------|---------|-------------|
| `--service-type TYPE` | auto-detect | Override service type (e.g. `std_srvs/srv/SetBool`) |
| `--timeout SECONDS` | `5` | Timeout waiting for service availability and response |

```bash
# Reset turtlesim
python3 {baseDir}/scripts/ros2_cli.py services call /reset '{}'

# Spawn a new turtle
python3 {baseDir}/scripts/ros2_cli.py services call /spawn \
  '{"x":3.0,"y":3.0,"theta":0.0,"name":"turtle2"}'

# Set pen color
python3 {baseDir}/scripts/ros2_cli.py services call /turtle1/set_pen \
  '{"r":255,"g":0,"b":0,"width":3,"off":0}'

# Toggle a boolean service
python3 {baseDir}/scripts/ros2_cli.py services call /enable_motors \
  '{"data":true}' --service-type std_srvs/srv/SetBool

# With longer timeout for slow services
python3 {baseDir}/scripts/ros2_cli.py services call /run_calibration '{}' --timeout 30
```

Output:
```json
{"service": "/spawn", "success": true, "result": {"name": "turtle2"}}
```

Error (service not available):
```json
{"error": "Service not available: /spawn"}
```

---

## services find `<service_type>`

Find all services of a specific service type. Accepts both `/srv/` and short formats.

**ROS 2 CLI equivalent:** `ros2 service find std_srvs/srv/Empty`

| Argument | Required | Description |
|----------|----------|-------------|
| `service_type` | Yes | Service type (e.g. `std_srvs/srv/Empty` or `std_srvs/Empty`) |

```bash
python3 {baseDir}/scripts/ros2_cli.py services find std_srvs/srv/Empty
python3 {baseDir}/scripts/ros2_cli.py services find std_srvs/Empty
python3 {baseDir}/scripts/ros2_cli.py services find turtlesim/srv/Spawn
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

## services echo `<service>` [options]

Echo service request/response events published on `<service>/_service_event`. Requires service introspection to be explicitly enabled on both the client and server via `configure_introspection()`.

**ROS 2 CLI equivalent:** `ros2 service echo /service` (Jazzy+)

| Argument | Required | Description |
|----------|----------|-------------|
| `service` | Yes | Service name (e.g. `/spawn`) |

| Option | Default | Description |
|--------|---------|-------------|
| `--duration SECONDS` | _(none)_ | Collect events for this many seconds; without this, returns first event only |
| `--max-messages N` / `--max-events N` | `100` | Maximum events to collect (only with `--duration`) |
| `--timeout SECONDS` | `5` | Timeout waiting for first event |

**Note:** This command requires service introspection to be enabled on the server/client:
```python
node.configure_introspection(
    clock, qos_profile, ServiceIntrospectionState.CONTENTS  # or METADATA
)
```

```bash
# Wait for first service event
python3 {baseDir}/scripts/ros2_cli.py services echo /spawn

# Collect events for 10 seconds
python3 {baseDir}/scripts/ros2_cli.py services echo /spawn --duration 10

# With custom timeout
python3 {baseDir}/scripts/ros2_cli.py services echo /my_service --timeout 15 --duration 5
```

Output (single event):
```json
{"service": "/spawn", "event": {"info": {}, "request": [{"x": 3.0, "y": 3.0}], "response": []}}
```

Output (duration mode):
```json
{
  "service": "/spawn",
  "event_topic": "/spawn/_service_event",
  "collected_count": 3,
  "events": [{"info": {}, "request": [{}], "response": []}]
}
```

Error (introspection not enabled):
```json
{
  "error": "No service event topic found: /spawn/_service_event",
  "hint": "Service introspection must be enabled on the server/client via configure_introspection(clock, qos, ServiceIntrospectionState.METADATA or CONTENTS)."
}
```

---

## nodes list / nodes ls

List all active ROS 2 nodes.

**Aliases:** `nodes ls`

**ROS 2 CLI equivalent:** `ros2 node list`

```bash
python3 {baseDir}/scripts/ros2_cli.py nodes list
python3 {baseDir}/scripts/ros2_cli.py nodes ls
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

Get node details: publishers, subscribers, services, action servers, and action clients.

**Aliases:** `nodes info`

**ROS 2 CLI equivalent:** `ros2 node info /node`

| Argument | Required | Description |
|----------|----------|-------------|
| `node` | Yes | Node name (e.g. `/turtlesim`) |

```bash
python3 {baseDir}/scripts/ros2_cli.py nodes details /turtlesim
python3 {baseDir}/scripts/ros2_cli.py nodes info /turtlesim
python3 {baseDir}/scripts/ros2_cli.py nodes details /robot_state_publisher
```

Output:
```json
{
  "node": "/turtlesim",
  "publishers": ["/turtle1/color_sensor", "/turtle1/pose", "/rosout"],
  "subscribers": ["/turtle1/cmd_vel"],
  "services": ["/clear", "/kill", "/reset", "/spawn", "/turtle1/set_pen"],
  "action_servers": ["/turtle1/rotate_absolute"],
  "action_clients": []
}
```

---

## params list `<node>` / params ls `<node>`

List all parameters for a specific node.

**Aliases:** `params ls`

**ROS 2 CLI equivalent:** `ros2 param list /node`

| Argument | Required | Description |
|----------|----------|-------------|
| `node` | Yes | Node name (e.g. `/turtlesim`) |

| Option | Default | Description |
|--------|---------|-------------|
| `--timeout SECONDS` | `5` | Service call timeout |

```bash
python3 {baseDir}/scripts/ros2_cli.py params list /turtlesim
python3 {baseDir}/scripts/ros2_cli.py params ls /turtlesim
python3 {baseDir}/scripts/ros2_cli.py params list /robot_state_publisher --timeout 10
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

**ROS 2 CLI equivalent:** `ros2 param get /turtlesim background_r`

| Argument | Required | Description |
|----------|----------|-------------|
| `name` | Yes | Node name with `:param_name` suffix, or just the node name when using space format |
| `param_name` | No | Parameter name (when using space-separated format) |

| Option | Default | Description |
|--------|---------|-------------|
| `--timeout SECONDS` | `5` | Service call timeout |

```bash
# Colon format
python3 {baseDir}/scripts/ros2_cli.py params get /turtlesim:background_r

# Space-separated format
python3 {baseDir}/scripts/ros2_cli.py params get /base_controller base_frame_id

python3 {baseDir}/scripts/ros2_cli.py params get /turtlesim:use_sim_time
python3 {baseDir}/scripts/ros2_cli.py params get /turtlesim background_g --timeout 10
```

Output:
```json
{"name": "/turtlesim:background_r", "value": "69", "exists": true}
```

---

## params set `<node:param_name>` `<value>` or `<node> <param_name> <value>`

Set a parameter value. Accepts colon-separated or space-separated format.

**ROS 2 CLI equivalent:** `ros2 param set /turtlesim background_r 255`

| Argument | Required | Description |
|----------|----------|-------------|
| `name` | Yes | Node name with `:param_name` suffix, or just the node name |
| `value` | Yes | New value (colon format) |
| `param_name` | No | Parameter name (space format) |
| `extra_value` | No | Value to set (space format) |

| Option | Default | Description |
|--------|---------|-------------|
| `--timeout SECONDS` | `5` | Service call timeout |

```bash
# Colon format
python3 {baseDir}/scripts/ros2_cli.py params set /turtlesim:background_r 255

# Space-separated format
python3 {baseDir}/scripts/ros2_cli.py params set /base_controller base_frame_id base_link_new

python3 {baseDir}/scripts/ros2_cli.py params set /turtlesim:background_g 0
python3 {baseDir}/scripts/ros2_cli.py params set /turtlesim:background_b 0
python3 {baseDir}/scripts/ros2_cli.py params set /turtlesim:use_sim_time true
```

Output:
```json
{"name": "/turtlesim:background_r", "value": "255", "success": true}
```

Read-only parameter:
```json
{"name": "/base_controller:base_frame_id", "value": "base_link_new", "success": false, "error": "Parameter is read-only and cannot be changed at runtime", "read_only": true}
```

---

## params describe `<node:param_name>` or `<node> <param_name>`

Describe a parameter's type, description, and constraints via the `DescribeParameters` service.

**ROS 2 CLI equivalent:** `ros2 param describe /turtlesim background_r`

| Argument | Required | Description |
|----------|----------|-------------|
| `name` | Yes | Node name with `:param_name` suffix, or node name alone |
| `param_name` | No | Parameter name (space-separated format) |

| Option | Default | Description |
|--------|---------|-------------|
| `--timeout SECONDS` | `5` | Service call timeout |

```bash
python3 {baseDir}/scripts/ros2_cli.py params describe /turtlesim:background_r
python3 {baseDir}/scripts/ros2_cli.py params describe /turtlesim background_r
python3 {baseDir}/scripts/ros2_cli.py params describe /base_controller base_frame_id --timeout 10
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

Export all parameters for a node as a flat `{param_name: value}` dict. Internally calls `ListParameters` then `GetParameters`.

**ROS 2 CLI equivalent:** `ros2 param dump /turtlesim`

| Argument | Required | Description |
|----------|----------|-------------|
| `node` | Yes | Node name (e.g. `/turtlesim`) |

| Option | Default | Description |
|--------|---------|-------------|
| `--timeout SECONDS` | `5` | Service call timeout |

```bash
python3 {baseDir}/scripts/ros2_cli.py params dump /turtlesim
python3 {baseDir}/scripts/ros2_cli.py params dump /robot_state_publisher --timeout 10
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

**ROS 2 CLI equivalent:** `ros2 param load /turtlesim /path/to/params.yaml`

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

python3 {baseDir}/scripts/ros2_cli.py params load /base_controller \
  '{"max_vel_x":1.5,"max_vel_theta":2.0}' --timeout 10
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

**ROS 2 CLI equivalent:** `ros2 param delete /turtlesim background_r`

| Argument | Required | Description |
|----------|----------|-------------|
| `name` | Yes | Node name |
| `param_name` | Yes | First parameter name to delete |
| `extra_names` | No | Additional parameter names to delete in one call |

| Option | Default | Description |
|--------|---------|-------------|
| `--timeout SECONDS` | `5` | Service call timeout |

```bash
# Delete one parameter
python3 {baseDir}/scripts/ros2_cli.py params delete /turtlesim background_r

# Delete multiple parameters in one call
python3 {baseDir}/scripts/ros2_cli.py params delete /turtlesim background_r background_g background_b

python3 {baseDir}/scripts/ros2_cli.py params delete /base_controller max_vel_x --timeout 10
```

Output:
```json
{"node": "/turtlesim", "deleted": ["background_r"], "count": 1}
```

---

## actions list / actions ls

List all available action servers.

**Aliases:** `actions ls`

**ROS 2 CLI equivalent:** `ros2 action list -t`

```bash
python3 {baseDir}/scripts/ros2_cli.py actions list
python3 {baseDir}/scripts/ros2_cli.py actions ls
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

Get action details including goal, result, and feedback field structures.

**Aliases:** `actions info`

**ROS 2 CLI equivalent:** `ros2 action info /action`

| Argument | Required | Description |
|----------|----------|-------------|
| `action` | Yes | Action server name |

```bash
python3 {baseDir}/scripts/ros2_cli.py actions details /turtle1/rotate_absolute
python3 {baseDir}/scripts/ros2_cli.py actions info /turtle1/rotate_absolute
python3 {baseDir}/scripts/ros2_cli.py actions details /navigate_to_pose
```

Output:
```json
{
  "action": "/turtle1/rotate_absolute",
  "action_type": "turtlesim/action/RotateAbsolute",
  "goal": {"theta": 0.0},
  "result": {"delta": 0.0},
  "feedback": {"remaining": 0.0}
}
```

---

## actions type `<action>`

Get the type of an action server. Resolves the type by inspecting the `/_action/feedback` topic and stripping the `_FeedbackMessage` suffix.

**ROS 2 CLI equivalent:** Shown in `ros2 action info /action` output

| Argument | Required | Description |
|----------|----------|-------------|
| `action` | Yes | Action server name |

| Option | Default | Description |
|--------|---------|-------------|
| `--timeout SECONDS` | `5` | Max wait time |

```bash
python3 {baseDir}/scripts/ros2_cli.py actions type /turtle1/rotate_absolute
python3 {baseDir}/scripts/ros2_cli.py actions type /navigate_to_pose
```

Output:
```json
{"action": "/turtle1/rotate_absolute", "type": "turtlesim/action/RotateAbsolute"}
```

Error (not found):
```json
{"error": "Action '/turtle1/rotate_absolute' not found in the ROS graph"}
```

---

## actions send `<action>` `<json_goal>` [options] / actions send-goal

Send an action goal and wait for the result. Optionally collects intermediate feedback messages.

**Aliases:** `actions send-goal`

**ROS 2 CLI equivalent:** `ros2 action send_goal /action turtlesim/action/RotateAbsolute '{"theta": 3.14}'`

| Argument | Required | Description |
|----------|----------|-------------|
| `action` | Yes | Action server name |
| `json_goal` | Yes | JSON string of the goal arguments |

| Option | Default | Description |
|--------|---------|-------------|
| `--feedback` | off | Collect feedback messages during execution; adds `feedback_msgs` to output |
| `--timeout SECONDS` | `30` | Timeout waiting for result |

```bash
# Basic goal
python3 {baseDir}/scripts/ros2_cli.py actions send /turtle1/rotate_absolute \
  '{"theta":3.14}'

# Using alias
python3 {baseDir}/scripts/ros2_cli.py actions send-goal /turtle1/rotate_absolute \
  '{"theta":1.57}'

# With feedback collection
python3 {baseDir}/scripts/ros2_cli.py actions send /turtle1/rotate_absolute \
  '{"theta":3.14}' --feedback

# Navigate to pose (Nav2)
python3 {baseDir}/scripts/ros2_cli.py actions send /navigate_to_pose \
  '{"pose":{"header":{"frame_id":"map"},"pose":{"position":{"x":1.0,"y":0.0,"z":0.0},"orientation":{"w":1.0}}}}' \
  --timeout 120 --feedback
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

Error (timeout):
```json
{
  "action": "/turtle1/rotate_absolute",
  "goal_id": "goal_1709312000000",
  "success": false,
  "error": "Timeout after 30.0s"
}
```

---

## actions cancel `<action>` [options]

Cancel all in-flight goals on an action server. Sends a `CancelGoal` request with zero UUID and zero timestamp — per the ROS 2 spec, this cancels all goals.

**ROS 2 CLI equivalent:** No direct equivalent (requires custom scripting with `action_msgs`)

| Argument | Required | Description |
|----------|----------|-------------|
| `action` | Yes | Action server name |

| Option | Default | Description |
|--------|---------|-------------|
| `--timeout SECONDS` | `5` | Service call timeout |

```bash
python3 {baseDir}/scripts/ros2_cli.py actions cancel /turtle1/rotate_absolute
python3 {baseDir}/scripts/ros2_cli.py actions cancel /navigate_to_pose
python3 {baseDir}/scripts/ros2_cli.py actions cancel /navigate_to_pose --timeout 10
```

Output:
```json
{
  "action": "/turtle1/rotate_absolute",
  "return_code": 0,
  "cancelled_goals": 0
}
```

`return_code`: 0 = success, 1 = rejected, 2 = unknown goal, 3 = goal already terminated.

Error (server not available):
```json
{"error": "Action server '/turtle1/rotate_absolute' not available"}
```

---

## actions echo `<action>` [options]

Echo live action feedback and status messages from an action server. Subscribes to `<action>/_action/feedback` and (if available) `<action>/_action/status`. No introspection required — these are standard action topics.

**ROS 2 CLI equivalent:** `ros2 action echo /action`

| Argument | Required | Description |
|----------|----------|-------------|
| `action` | Yes | Action server name |

| Option | Default | Description |
|--------|---------|-------------|
| `--duration SECONDS` | _(none)_ | Collect feedback for this many seconds; without this, returns first feedback message |
| `--max-messages N` / `--max-msgs N` | `100` | Maximum feedback messages to collect (only with `--duration`) |
| `--timeout SECONDS` | `5` | Timeout waiting for first feedback message |

```bash
# Wait for first feedback message
python3 {baseDir}/scripts/ros2_cli.py actions echo /turtle1/rotate_absolute

# Collect feedback for 10 seconds while a goal is running
python3 {baseDir}/scripts/ros2_cli.py actions echo /navigate_to_pose --duration 10

# Collect up to 20 feedback messages with a 30-second window
python3 {baseDir}/scripts/ros2_cli.py actions echo /navigate_to_pose \
  --duration 30 --max-messages 20
```

Output (single feedback):
```json
{
  "action": "/turtle1/rotate_absolute",
  "feedback": {"feedback": {"remaining": 1.42}}
}
```

Output (duration mode):
```json
{
  "action": "/navigate_to_pose",
  "collected_count": 5,
  "feedback": [
    {"feedback": {"current_pose": {}, "distance_remaining": 2.1}},
    {"feedback": {"current_pose": {}, "distance_remaining": 1.7}}
  ],
  "status": [{"status_list": [{"status": 2}]}]
}
```

Error (action server not found):
```json
{"error": "Action server not found: /turtle1/rotate_absolute"}
```

---

## actions find `<action_type>`

Find all action servers of a specific action type. Accepts both `/action/` and short formats (normalised for comparison). Mirrors `topics find` and `services find`.

**ROS 2 CLI equivalent:** `ros2 action find turtlesim/action/RotateAbsolute`

| Argument | Required | Description |
|----------|----------|-------------|
| `action_type` | Yes | Action type (e.g. `turtlesim/action/RotateAbsolute` or `turtlesim/RotateAbsolute`) |

```bash
python3 {baseDir}/scripts/ros2_cli.py actions find turtlesim/action/RotateAbsolute
python3 {baseDir}/scripts/ros2_cli.py actions find turtlesim/RotateAbsolute
python3 {baseDir}/scripts/ros2_cli.py actions find nav2_msgs/action/NavigateToPose
```

Output:
```json
{
  "action_type": "turtlesim/action/RotateAbsolute",
  "actions": ["/turtle1/rotate_absolute"],
  "count": 1
}
```

No matches:
```json
{"action_type": "turtlesim/action/RotateAbsolute", "actions": [], "count": 0}
```
