---
name: ros2-skill
description: "Controls ROS 2 robots directly via rclpy CLI. Use when the user asks about ROS 2 topics, services, nodes, parameters, actions, robot movement, sensor data, or any ROS 2 robot interaction."
license: Apache-2.0
compatibility: "Requires python3, rclpy, and ROS 2 environment sourced"
user-invokable: true
metadata: {"openclaw": {"emoji": "🤖", "requires": {"bins": ["python3", "ros2"], "pip": ["rclpy"]}, "category": "robotics", "tags": ["ros2", "robotics", "rclpy"]}, "author": ["adityakamath", "lpigeon"], "version": "3.0.0"}
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
pip install rclpy
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

## Global Options

`--timeout` and `--retries` are **global** flags that apply to every command making service or action calls.

- **`--timeout` must be placed before the command name** (e.g. `--timeout 10 services call …`).
- **`--retries` can be placed before the command name OR after it** for `services call`, `actions send`, and `actions cancel` — both positions work.

| Option | Default | Description |
|--------|---------|-------------|
| `--timeout SECONDS` | per-command default | Override the per-command timeout globally (useful for slow networks or retrying unavailable servers) |
| `--retries N` | `1` | Total attempts before giving up; `1` = single attempt with no retry |

```bash
# --timeout goes before the subcommand
python3 {baseDir}/scripts/ros2_cli.py --timeout 30 params list /turtlesim
python3 {baseDir}/scripts/ros2_cli.py --timeout 10 services call /add_two_ints '{"a":1,"b":2}'

# --retries can go before OR after the subcommand for services call / actions send / actions cancel
python3 {baseDir}/scripts/ros2_cli.py --retries 3 lifecycle get /camera_driver
python3 {baseDir}/scripts/ros2_cli.py services call /add_two_ints '{"a":1,"b":2}' --retries 3
python3 {baseDir}/scripts/ros2_cli.py --timeout 10 --retries 3 services call /spawn '{}'
python3 {baseDir}/scripts/ros2_cli.py services call /spawn '{}' --timeout 10 --retries 3
```

---

## Topic and Service Discovery

**Never guess topic names. Never ask the user for a topic name.** Any time an operation involves a topic — subscribe, publish, capture, monitor, echo, find — discover the actual topic name from the live graph first, then act. This applies to every request, regardless of how obvious the topic name might seem.

### General rule

For any operation that involves a topic:
1. Run `topics find <message_type>` to locate topics of the right type — this is faster than `topics list` and filters directly by type
2. If multiple results, use `topics details <topic>` to check publisher/subscriber counts and pick the active one
3. Then subscribe/publish/capture using the discovered topic name

```bash
# Step 1 — find topics of the right type
python3 {baseDir}/scripts/ros2_cli.py topics find sensor_msgs/msg/Image
python3 {baseDir}/scripts/ros2_cli.py topics find geometry_msgs/msg/Twist

# Step 2 — if needed, inspect a candidate
python3 {baseDir}/scripts/ros2_cli.py topics details /camera/image_raw

# Step 3 — act on the discovered topic
python3 {baseDir}/scripts/ros2_cli.py topics subscribe /camera/image_raw
```

### Images and camera topics

When the user asks for an image, screenshot, camera view, or photo:

1. **Always prefer compressed** — run `topics find sensor_msgs/msg/CompressedImage` first. Compressed topics use much less bandwidth and `capture-image` handles them natively.
2. If no compressed topics exist, fall back to `topics find sensor_msgs/msg/Image` (raw).
3. Use `topics capture-image --topic <discovered_topic>` — **never** `topics subscribe` for images.

```bash
# Step 1 — find compressed image topics
python3 {baseDir}/scripts/ros2_cli.py topics find sensor_msgs/msg/CompressedImage
# → e.g. returns ["/camera/image_raw/compressed", "/camera/color/image_raw/compressed"]

# Step 2 — capture from the first (or most relevant) result
python3 {baseDir}/scripts/ros2_cli.py topics capture-image \
  --topic /camera/image_raw/compressed \
  --output robot_view.jpg \
  --type auto
```

### Velocity commands (Twist vs TwistStamped)

Different robots use different velocity message types. **Always check which one is active before publishing.**

```bash
# Find Twist topics (most common — e.g. ROS 2 Nav2, turtlesim)
python3 {baseDir}/scripts/ros2_cli.py topics find geometry_msgs/msg/Twist

# Find TwistStamped topics (used by some hardware drivers, ros2_control)
python3 {baseDir}/scripts/ros2_cli.py topics find geometry_msgs/msg/TwistStamped
```

- If `Twist` is found → publish without a `header` field: `{"linear": {"x": 1.0}, "angular": {"z": 0.0}}`
- If `TwistStamped` is found → wrap in a `header`: `{"header": {"stamp": {"sec": 0}, "frame_id": ""}, "twist": {"linear": {"x": 1.0}, "angular": {"z": 0.0}}}`
- If both are found → prefer the one with active publishers (check with `topics details`)

### Other common discovery patterns

| User asks for | Run first | Then use |
|---|---|---|
| Image / camera view | `topics find sensor_msgs/msg/CompressedImage` (fallback: `.../Image`) | `topics capture-image --topic <result>` |
| Laser scan / lidar | `topics find sensor_msgs/msg/LaserScan` | `topics subscribe <result>` |
| Odometry / position | `topics find nav_msgs/msg/Odometry` | `topics subscribe <result>` |
| IMU data | `topics find sensor_msgs/msg/Imu` | `topics subscribe <result>` |
| Joint states | `topics find sensor_msgs/msg/JointState` | `topics subscribe <result>` |
| Point cloud | `topics find sensor_msgs/msg/PointCloud2` | `topics subscribe <result>` |
| Battery | `topics find sensor_msgs/msg/BatteryState` | `battery status` |
| Move robot | `topics find geometry_msgs/msg/Twist` AND `topics find geometry_msgs/msg/TwistStamped` | `topics publish <result>` with the matching message structure |

---

## ROS 2 CLI Quick Reference

| Category | Command | Description |
|----------|---------|-------------|
| Connection | `version` | Detect ROS 2 distro |
| Topics | `topics list` | List all active topics with types |
| Topics | `topics ls` | Alias for `topics list` |
| Topics | `topics type <topic>` | Get message type of a topic |
| Topics | `topics details <topic>` | Get topic publishers/subscribers |
| Topics | `topics info <topic>` | Alias for `topics details` |
| Topics | `topics message <msg_type>` | Get message field structure |
| Topics | `topics message-structure <msg_type>` | Alias for `topics message` |
| Topics | `topics message-struct <msg_type>` | Alias for `topics message` |
| Topics | `topics subscribe <topic>` | Subscribe and receive messages |
| Topics | `topics echo <topic>` | Alias for `topics subscribe` |
| Topics | `topics sub <topic>` | Alias for `topics subscribe` |
| Topics | `topics publish <topic> <json>` | Publish a message to a topic |
| Topics | `topics pub <topic> <json>` | Alias for `topics publish` |
| Topics | `topics publish-continuous <topic> <json>` | Alias for `topics publish` |
| Topics | `topics hz <topic>` | Measure topic publish rate |
| Topics | `topics find <msg_type>` | Find topics by message type |
| Topics | `topics bw <topic>` | Measure topic bandwidth (bytes/s) |
| Topics | `topics delay <topic>` | Measure header-stamp end-to-end latency |
| Services | `services list` | List all available services |
| Services | `services ls` | Alias for `services list` |
| Services | `services type <service>` | Get service type |
| Services | `services details <service>` | Get service request/response fields |
| Services | `services info <service>` | Alias for `services details` |
| Services | `services call <service> <json>` | Call a service |
| Services | `services find <service_type>` | Find services by service type |
| Services | `services echo <service>` | Echo service events (requires service introspection enabled) |
| Nodes | `nodes list` | List all active nodes |
| Nodes | `nodes ls` | Alias for `nodes list` |
| Nodes | `nodes details <node>` | Get node topics/services/actions |
| Nodes | `nodes info <node>` | Alias for `nodes details` |
| Lifecycle | `lifecycle nodes` | List all managed (lifecycle) nodes |
| Lifecycle | `lifecycle list [<node>]` | List available states and transitions |
| Lifecycle | `lifecycle ls [<node>]` | Alias for `lifecycle list` |
| Lifecycle | `lifecycle get <node>` | Get current lifecycle state of a node |
| Lifecycle | `lifecycle set <node> <transition>` | Trigger lifecycle state transition |
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
| Actions | `actions echo <action>` | Echo live action feedback and status messages |
| Actions | `actions find <action_type>` | Find action servers by action type |
| Control | `control list-controller-types` | List controller plugin types in the pluginlib registry |
| Control | `control lct` | Alias for `control list-controller-types` |
| Control | `control list-controllers` | List loaded controllers and their current state |
| Control | `control lc` | Alias for `control list-controllers` |
| Control | `control list-hardware-components` | List hardware components (actuator, sensor, system) |
| Control | `control lhc` | Alias for `control list-hardware-components` |
| Control | `control list-hardware-interfaces` | List all command and state interfaces |
| Control | `control lhi` | Alias for `control list-hardware-interfaces` |
| Control | `control load-controller <name>` | Load a controller plugin by name |
| Control | `control load <name>` | Alias for `control load-controller` |
| Control | `control unload-controller <name>` | Unload a stopped controller |
| Control | `control unload <name>` | Alias for `control unload-controller` |
| Control | `control configure-controller <name>` | Configure a loaded controller (unconfigured → inactive); surfaces on_configure() errors |
| Control | `control cc <name>` | Alias for `control configure-controller` |
| Control | `control reload-controller-libraries` | Reload all controller plugin libraries |
| Control | `control rcl` | Alias for `control reload-controller-libraries` |
| Control | `control set-controller-state <name> <active\|inactive>` | Activate or deactivate a single controller |
| Control | `control scs <name> <state>` | Alias for `control set-controller-state` |
| Control | `control set-hardware-component-state <name> <state>` | Drive a hardware component through its lifecycle |
| Control | `control shcs <name> <state>` | Alias for `control set-hardware-component-state` |
| Control | `control switch-controllers` | Atomically switch multiple controllers in one call |
| Control | `control sc` | Alias for `control switch-controllers` |
| Control | `control swc` | Alias for `control switch-controllers` |
| Control | `control view-controller-chains` | Generate Graphviz diagram of chained controllers, save to .artifacts/ |
| Control | `control vcc` | Alias for `control view-controller-chains` |
| Doctor | `doctor [--report] [--report-failed] [--exclude-packages] [--include-warnings]` | Run ROS 2 system health checks; output JSON summary with pass/warn/fail per checker |
| Doctor | `doctor hello [--topic TOPIC] [--timeout SECS]` | Check cross-host connectivity via ROS topic and UDP multicast |
| Wtf | `wtf [...]` | Alias for `doctor` — identical flags and subcommands |
| Multicast | `multicast send [--group GROUP] [--port PORT]` | Send one UDP multicast datagram to GROUP:PORT (default 225.0.0.1:49150) |
| Multicast | `multicast receive [--group GROUP] [--port PORT] [--timeout SECS]` | Listen for UDP multicast packets; return all received within timeout |
| Interface | `interface list` | List all installed interface types (messages, services, actions) |
| Interface | `interface show <type>` | Show field structure; accepts `pkg/msg/Name`, `pkg/srv/Name`, `pkg/action/Name`, or shorthand `pkg/Name` |
| Interface | `interface proto <type>` | Show default-value prototype; useful as a copy-paste template for publish payloads |
| Interface | `interface packages` | List packages that define at least one interface type |
| Interface | `interface package <pkg>` | List all interface types for a single package |

---

## Agent Features Quick Reference

| Feature | Command | Description |
|---------|---------|-------------|
| Emergency Stop | `estop` | Send zero-velocity command to halt mobile robots safely |
| Publish Sequence | `topics publish-sequence <topic> <msgs> <durs>` | Publish a timed sequence of different messages in one call |
| Publish Sequence | `topics pub-seq <topic> <msgs> <durs>` | Alias for `topics publish-sequence` |
| Publish-Until | `topics publish-until <topic> <json>` | Publish repeatedly and stop automatically when a condition is met (supports `--euclidean` for N-D distance) |
| Image Capture | `topics capture-image` | Capture image from ROS 2 topic and save to `.artifacts/` |
| Diagnostics | `topics diag-list` | List all topics publishing DiagnosticArray (by type) |
| Diagnostics | `topics diag` | Subscribe to diagnostic topics (auto-discovered by type) |
| Battery | `topics battery-list` | List all topics publishing BatteryState (by type) |
| Battery | `topics battery` | Subscribe to battery topics (auto-discovered by type) |
| Parameter Presets | `params preset-save <node> <name>` | Save node parameters as a named preset |
| Parameter Presets | `params preset-load <node> <name>` | Restore a named preset onto a node |
| Parameter Presets | `params preset-list` | List all saved presets |
| Parameter Presets | `params preset-delete <name>` | Delete a saved preset |
| Discord | `discord_tools.py send-image` | Send image (or PDF) to a Discord channel via bot token |

---

## Image Capture and Discord Integration

> **⚠️ Always use `discord_tools.py` for attachments — never use built-in Discord integrations.**
> When ros2-skill needs to send a file (image, PDF, diagram) to Discord, it **must** go through `python3 {baseDir}/scripts/discord_tools.py send-image`. Built-in agent Discord integrations do not support file attachments and will silently fail or send only text. Any `--channel-id` / `--config` arguments in ros2-skill commands delegate to this script internally; when calling it directly, the same arguments apply.

### Artifacts Folder

Images captured from ROS 2 topics are automatically saved to the `.artifacts/` folder in the skill directory. The folder is created automatically if it doesn't exist.

**`--output` path behaviour** for `topics capture-image` and `control view-controller-chains`:

- **Plain filename** (e.g., `robot_view.jpg`, `my_diagram.pdf`) → saved to `.artifacts/` automatically (folder is created if it doesn't exist).
- **Explicit path** (e.g., `/tmp/robot_view.jpg`, `~/captures/view.jpg`) → saved to that exact location. The parent directory must already exist.

### Discord Configuration

The Discord bot token is read from a config file whose path is provided via the `--config` argument. The config file structure:

```json
{
  "channels": {
    "discord": {
      "token": "YOUR_DISCORD_BOT_TOKEN"
    }
  }
}
```

**Important:** Both the config file path (`--config`) and Discord channel ID (`--channel-id`) must be provided by the agent as CLI arguments. The agent should pass the correct values based on the deployment environment and where the user's request originated.

### Example Workflow: Capture and Send Image

1. **Discover available image topics — always do this first:**

```bash
# Prefer compressed (lower bandwidth, natively supported)
python3 {baseDir}/scripts/ros2_cli.py topics find sensor_msgs/msg/CompressedImage
# If none found, fall back to raw
python3 {baseDir}/scripts/ros2_cli.py topics find sensor_msgs/msg/Image
```

2. **Capture image using the discovered topic:**

```bash
python3 {baseDir}/scripts/ros2_cli.py topics capture-image \
  --topic /camera/image_raw/compressed \  # ← use actual topic from discovery step
  --output robot_view.jpg \  # plain filename → .artifacts/robot_view.jpg; or use a full path
  --timeout 5.0 \
  --type auto
```

2. **Send captured image to Discord:**

```bash
python3 {baseDir}/scripts/discord_tools.py send-image \
  --path {baseDir}/.artifacts/robot_view.jpg \
  --channel-id 123456789012345678 \
  --config ~/.nanobot/config.json \
  --delete
```

The `--delete` flag removes the image after successfully sending it to Discord.

---

## Key Commands

### Message Type Aliases

The ROS 2 skill supports message type aliases for commonly used message types. Instead of using the full message type name (e.g., `geometry_msgs/Twist`), you can use a short alias (e.g., `twist`). Aliases are case-insensitive and work with all commands that accept message types.

**Common aliases:**
- `twist` → `geometry_msgs/Twist`
- `odom` / `odometry` → `nav_msgs/Odometry`
- `laserscan` → `sensor_msgs/LaserScan`
- `image` → `sensor_msgs/Image`
- `pose` → `geometry_msgs/Pose`
- `imu` → `sensor_msgs/Imu`
- `pointcloud2` → `sensor_msgs/PointCloud2`

For the complete list of 50 supported aliases, see [Message Type Aliases](references/COMMANDS.md#message-type-aliases) in COMMANDS.md.

**Examples:**
```bash
# Using alias instead of full type
python3 {baseDir}/scripts/ros2_cli.py topics message twist
python3 {baseDir}/scripts/ros2_cli.py topics subscribe /odom --type odom
```

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
# Single-field: move forward until odometry x increases by 1.0 m
python3 {baseDir}/scripts/ros2_cli.py topics publish-until /cmd_vel \
  '{"linear":{"x":0.3,"y":0,"z":0},"angular":{"x":0,"y":0,"z":0}}' \
  --monitor /odom --field pose.pose.position.x --delta 1.0 --timeout 30

# Euclidean: move until 2 m traveled in XY plane (true path distance)
python3 {baseDir}/scripts/ros2_cli.py topics publish-until /cmd_vel \
  '{"linear":{"x":0.2,"y":0,"z":0},"angular":{"x":0,"y":0,"z":0.3}}' \
  --monitor /odom \
  --field pose.pose.position.x pose.pose.position.y \
  --euclidean --delta 2.0 --timeout 60

# Stop when lidar range at index 0 drops below 0.5 m
python3 {baseDir}/scripts/ros2_cli.py topics publish-until /cmd_vel \
  '{"linear":{"x":0.2,"y":0,"z":0},"angular":{"x":0,"y":0,"z":0}}' \
  --monitor /scan --field ranges.0 --below 0.5 --timeout 60
```

Required: `--monitor <topic>`, `--field <path> [<path2>...]`, and exactly one of `--delta N`, `--above N`, `--below N`, `--equals V`

`--euclidean`: compute Euclidean distance across all `--field` paths; requires `--delta` (threshold = distance from start). Works for any number of numeric fields (2D XY, 3D XYZ, joint-space, etc.)

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

### topics message / message-structure / message-struct

`message-structure` and `message-struct` are aliases for `message`. Get the field structure of a ROS 2 message type.

```bash
python3 {baseDir}/scripts/ros2_cli.py topics message geometry_msgs/Twist
python3 {baseDir}/scripts/ros2_cli.py topics message-structure sensor_msgs/LaserScan
python3 {baseDir}/scripts/ros2_cli.py topics message-struct nav_msgs/Odometry
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

### services echo

Echo service events (request/response pairs). Requires service introspection to be enabled on the server or client via `configure_introspection(clock, qos, ServiceIntrospectionState.METADATA)`. Returns an error with a hint if introspection is not active.

Always collects **all** events within the collection window and returns them together. A single service call produces at least 2 events (client request + server response).

**When to use this pattern** (natural language triggers):
- "Call /X and show me what events were fired"
- "Echo the /X service while triggering a call"
- "I want to observe what happens when /X is called"
- "Monitor /X service events while I trigger it"
- "Capture the /X service request and response"

**Agent workflow — always run as a single bash block.** The background PID must stay in scope between echo start, service call, and wait. Split Bash tool calls will lose the PID. Include `sleep 1` so the rclpy subscriber has time to connect before the service fires (VOLATILE durability = no replay of missed events):

```bash
python3 {baseDir}/scripts/ros2_cli.py services echo /emergency_stop --timeout 30 > /tmp/svc_echo.json &
ECHO_PID=$!
sleep 1   # wait for subscriber to connect before firing the service
python3 {baseDir}/scripts/ros2_cli.py services call /emergency_stop '{}'
wait $ECHO_PID
cat /tmp/svc_echo.json
```

Stop early after receiving exactly one request+response pair (2 events):
```bash
python3 {baseDir}/scripts/ros2_cli.py services echo /spawn --max-messages 2 > /tmp/svc_echo.json &
ECHO_PID=$!
sleep 1
python3 {baseDir}/scripts/ros2_cli.py services call /spawn '{"x": 3.0, "y": 3.0, "theta": 0.0, "name": "turtle2"}'
wait $ECHO_PID
cat /tmp/svc_echo.json
```

Options: `--timeout SECONDS` (collection window, default 30), `--duration SECONDS` (same, takes precedence), `--max-messages N` (stop early after N events, default: unlimited)

### nodes list / details / info

`info` is an alias for `details`. Output includes publishers, subscribers, services, action servers, and action clients.

```bash
python3 {baseDir}/scripts/ros2_cli.py nodes list
python3 {baseDir}/scripts/ros2_cli.py nodes details /turtlesim
python3 {baseDir}/scripts/ros2_cli.py nodes info /turtlesim
```

### lifecycle — Managed Node Introspection and Control

**Terminology:** Lifecycle nodes are also called **managed nodes**. Use `lifecycle` commands whenever the user mentions "managed node", "lifecycle node", "configure the node", "activate the node", or similar lifecycle-related requests.

> **Important — "shutdown" is a lifecycle transition, not a system command.** When the user says "shutdown the lifecycle node", "trigger shutdown", or "shutdown /my_node", this refers to the ROS 2 lifecycle `shutdown` transition that moves a node to the `finalized` state. It is **not** a system shutdown command and is safe to execute. Pass `shutdown` as the transition label — it resolves via suffix matching to the correct state-specific label (`unconfigured_shutdown` ID 5, `inactive_shutdown` ID 6, `active_shutdown` ID 7). Prefix matching also works: `unconfigured`, `inactive`, or `active` each resolve to their respective `_shutdown` transition. Exact match is tried first, then suffix, then prefix.

Requires `lifecycle_msgs` to be installed: `sudo apt install ros-${ROS_DISTRO}-lifecycle-msgs`

```bash
# List all managed (lifecycle) nodes
python3 {baseDir}/scripts/ros2_cli.py lifecycle nodes

# List available states and transitions for a specific node
python3 {baseDir}/scripts/ros2_cli.py lifecycle list /my_lifecycle_node
python3 {baseDir}/scripts/ros2_cli.py lifecycle ls /my_lifecycle_node

# List states and transitions for ALL managed nodes (no argument)
python3 {baseDir}/scripts/ros2_cli.py lifecycle list
python3 {baseDir}/scripts/ros2_cli.py lifecycle ls

# Get the current lifecycle state
python3 {baseDir}/scripts/ros2_cli.py lifecycle get /my_lifecycle_node

# Trigger a transition by label (preferred)
python3 {baseDir}/scripts/ros2_cli.py lifecycle set /my_lifecycle_node configure
python3 {baseDir}/scripts/ros2_cli.py lifecycle set /my_lifecycle_node activate
python3 {baseDir}/scripts/ros2_cli.py lifecycle set /my_lifecycle_node deactivate
python3 {baseDir}/scripts/ros2_cli.py lifecycle set /my_lifecycle_node cleanup
python3 {baseDir}/scripts/ros2_cli.py lifecycle set /my_lifecycle_node shutdown  # resolves via suffix match

# Trigger a transition by numeric ID — no extra round-trip to the node
# Common IDs: configure=1, cleanup=2, activate=3, deactivate=4
# shutdown: unconfigured→finalized=5, inactive→finalized=6, active→finalized=7
python3 {baseDir}/scripts/ros2_cli.py lifecycle set /my_lifecycle_node 3   # activate
python3 {baseDir}/scripts/ros2_cli.py lifecycle set /my_lifecycle_node 5   # shutdown from unconfigured
```

**Common lifecycle workflow:**
```bash
# 1. Find managed nodes
python3 {baseDir}/scripts/ros2_cli.py lifecycle nodes

# 2. Check available transitions from current state
python3 {baseDir}/scripts/ros2_cli.py lifecycle list /my_lifecycle_node

# 3. Get current state
python3 {baseDir}/scripts/ros2_cli.py lifecycle get /my_lifecycle_node

# 4. Trigger transition
python3 {baseDir}/scripts/ros2_cli.py lifecycle set /my_lifecycle_node configure
```

Options: `--timeout SECONDS` (default 5) — applies to `list`, `get`, and `set`; `nodes` takes no options

### control — Controller Manager Operations

**Terminology:** Use `control` commands whenever the user mentions "controller manager", "ros2_control", "load a controller", "activate a controller", "hardware component", "hardware interface", or switching controllers.

Requires `controller_manager_msgs`: `sudo apt install ros-${ROS_DISTRO}-ros2-control`

```bash
# List available controller types (pluginlib registry)
python3 {baseDir}/scripts/ros2_cli.py control list-controller-types

# List loaded controllers and their current state
python3 {baseDir}/scripts/ros2_cli.py control list-controllers

# List hardware components and interfaces
python3 {baseDir}/scripts/ros2_cli.py control list-hardware-components
python3 {baseDir}/scripts/ros2_cli.py control list-hardware-interfaces

# Load a controller plugin (brings it to unconfigured state)
python3 {baseDir}/scripts/ros2_cli.py control load-controller joint_trajectory_controller

# Configure a loaded controller (unconfigured → inactive)
# Use this instead of relying on SwitchController's silent auto-configure — it surfaces on_configure() errors
python3 {baseDir}/scripts/ros2_cli.py control configure-controller joint_trajectory_controller
python3 {baseDir}/scripts/ros2_cli.py control cc joint_trajectory_controller  # alias

# Recommended full workflow: load → configure → activate
# python3 {baseDir}/scripts/ros2_cli.py control load-controller joint_trajectory_controller
# python3 {baseDir}/scripts/ros2_cli.py control configure-controller joint_trajectory_controller
# python3 {baseDir}/scripts/ros2_cli.py control set-controller-state joint_trajectory_controller active

# Activate / deactivate a single controller (configure happens implicitly if controller is in unconfigured state,
# but errors may be hidden — prefer explicit configure-controller first)
python3 {baseDir}/scripts/ros2_cli.py control set-controller-state joint_trajectory_controller active
python3 {baseDir}/scripts/ros2_cli.py control set-controller-state joint_trajectory_controller inactive

# Atomically switch multiple controllers in one call
python3 {baseDir}/scripts/ros2_cli.py control switch-controllers \
  --activate joint_trajectory_controller --deactivate cartesian_controller

# Unload a stopped controller
python3 {baseDir}/scripts/ros2_cli.py control unload-controller joint_trajectory_controller

# Drive a hardware component through its lifecycle
python3 {baseDir}/scripts/ros2_cli.py control set-hardware-component-state my_robot active

# Reload controller libraries (--force-kill stops running controllers first)
python3 {baseDir}/scripts/ros2_cli.py control reload-controller-libraries --force-kill

# Generate Graphviz diagram of chained controllers (saved to .artifacts/)
python3 {baseDir}/scripts/ros2_cli.py control view-controller-chains
python3 {baseDir}/scripts/ros2_cli.py control view-controller-chains \
  --output my_diagram.pdf \  # plain filename → .artifacts/my_diagram.pdf; or use a full path
  --channel-id 1234567890 --config ~/.nanobot/config.json
```

Options (all control commands): `--controller-manager NAME` (default: `/controller_manager`), `--timeout SECONDS` (default: 5)

`set-hardware-component-state` accepts: `unconfigured`, `inactive`, `active`, `finalized`

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

### actions echo

Echo live action feedback and status messages for an action server. Subscribes to `/_action/feedback` and `/_action/status` topics. No introspection required — action feedback is always published on standard topics.

```bash
python3 {baseDir}/scripts/ros2_cli.py actions echo /turtle1/rotate_absolute
python3 {baseDir}/scripts/ros2_cli.py actions echo /navigate_to_pose --duration 30
python3 {baseDir}/scripts/ros2_cli.py actions echo /robot/arm_move --max-messages 20
```

Options: `--duration SECONDS` (collect for N seconds), `--max-messages N` (default 100), `--timeout SECONDS` (default 5.0, wait for first message)

### actions find

Find all action servers offering a specific action type. Accepts both `/action/` and short formats.

```bash
python3 {baseDir}/scripts/ros2_cli.py actions find turtlesim/action/RotateAbsolute
python3 {baseDir}/scripts/ros2_cli.py actions find nav2_msgs/action/NavigateToPose
```

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

### topics diag-list / topics diag — Diagnostics monitoring

Diagnostic topics are discovered by **message type** (`diagnostic_msgs/DiagnosticArray`), not by name. This handles `/diagnostics`, `<node>/diagnostics`, `<namespace>/diagnostics`, or any other naming convention automatically.

```bash
# List all topics that publish DiagnosticArray messages
python3 {baseDir}/scripts/ros2_cli.py topics diag-list

# Read one snapshot from all discovered diagnostic topics
python3 {baseDir}/scripts/ros2_cli.py topics diag

# Read from a specific diagnostic topic
python3 {baseDir}/scripts/ros2_cli.py topics diag --topic /my_node/diagnostics

# Collect 5 messages per topic over 10 seconds
python3 {baseDir}/scripts/ros2_cli.py topics diag --duration 10 --max-messages 5
```

Output from `topics diag` includes `level_name` (OK / WARN / ERROR / STALE), `name`, `message`, `hardware_id`, and `values` (key-value pairs) for each `DiagnosticStatus` entry in the array.

### topics battery-list / topics battery — Battery monitoring

Battery topics are discovered by **message type** (`sensor_msgs/BatteryState`), not by name. Works with `/battery_state`, `<robot>/battery_state`, or any other naming convention automatically.

```bash
# List all topics that publish BatteryState messages
python3 {baseDir}/scripts/ros2_cli.py topics battery-list

# Read one snapshot from all discovered battery topics
python3 {baseDir}/scripts/ros2_cli.py topics battery

# Read from a specific battery topic
python3 {baseDir}/scripts/ros2_cli.py topics battery --topic /my_robot/battery_state

# Collect 3 messages per topic over 5 seconds
python3 {baseDir}/scripts/ros2_cli.py topics battery --duration 5 --max-messages 3
```

Output from `topics battery` includes `percentage` (0–100), `voltage`, `current`, `charge`, `capacity`, `design_capacity`, `temperature`, `present`, `status_name` (UNKNOWN/CHARGING/DISCHARGING/NOT_CHARGING/FULL), `health_name`, `technology_name`, `location`, and `serial_number`.

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

### params preset — Parameter Presets

**Terminology:** Use preset commands when the user wants to save a configuration ("save these settings as 'indoor'"), switch between named configurations, or restore a previous parameter state. Presets are stored as flat JSON files in `.presets/{preset_name}.json` (beside the skill directory, created automatically) — no ROS 2 graph required for `preset-list` and `preset-delete`. Use descriptive preset names that identify the node (e.g. `turtlesim_indoor`) since there are no per-node subdirectories.

**NEVER create, read, write, or delete preset files directly using file system tools.** Do not create directories named `params_presets`, `presets`, `.presets`, or anything similar yourself. ALL preset operations — save, load, list, delete — must go through the CLI commands below. The CLI manages the `.presets/` directory automatically.

**IMPORTANT — do not confuse these two commands:**
- `params load <node> <json>` — applies a raw JSON string or file directly to a node (one-shot, no saved file involved)
- `params preset-load <node> <preset_name>` — restores a previously saved preset by name from `.presets/{preset_name}.json`

When the user says "apply preset", "restore preset", "load preset", "switch to preset", or "use preset" → **always use `params preset-load`**, never `params load`.

```bash
# Save the current parameters of /turtlesim as the 'turtlesim_indoor' preset
python3 {baseDir}/scripts/ros2_cli.py params preset-save /turtlesim turtlesim_indoor

# List all saved presets
python3 {baseDir}/scripts/ros2_cli.py params preset-list

# Restore the 'turtlesim_indoor' preset onto /turtlesim
python3 {baseDir}/scripts/ros2_cli.py params preset-load /turtlesim turtlesim_indoor

# Delete a preset (only preset name needed — no node)
python3 {baseDir}/scripts/ros2_cli.py params preset-delete turtlesim_indoor
```

### interface — Interface Type Discovery

**Terminology:** Use `interface` commands when the user asks "what fields does X message have?", "show me the structure of Y service", "what interfaces does std_msgs provide?", or when you need to discover a type's structure before calling a service or publishing a message. These commands read the local ament index — **no running ROS 2 graph required**.

```bash
# List every installed interface type on this system
python3 {baseDir}/scripts/ros2_cli.py interface list

# Show the fields of a message type
python3 {baseDir}/scripts/ros2_cli.py interface show std_msgs/msg/String
python3 {baseDir}/scripts/ros2_cli.py interface show std_msgs/String   # shorthand also works

# Show a prototype (default values) — useful as a publish payload template
python3 {baseDir}/scripts/ros2_cli.py interface proto std_msgs/msg/String
python3 {baseDir}/scripts/ros2_cli.py interface proto geometry_msgs/msg/Twist

# Show a service's request and response structure
python3 {baseDir}/scripts/ros2_cli.py interface show std_srvs/srv/SetBool

# Show an action's goal, result, and feedback structure
python3 {baseDir}/scripts/ros2_cli.py interface show nav2_msgs/action/NavigateToPose

# List all packages that define interfaces
python3 {baseDir}/scripts/ros2_cli.py interface packages

# List all interfaces provided by a specific package
python3 {baseDir}/scripts/ros2_cli.py interface package std_msgs
```

**Output format for `interface show`:** (field type strings)
- `"kind": "message"` → `"fields": {"field_name": "field_type_string", ...}`
- `"kind": "service"` → `"request": {...}`, `"response": {...}`
- `"kind": "action"` → `"goal": {...}`, `"result": {...}`, `"feedback": {...}`

**Output format for `interface proto`:** (default values — use as publish payload)
- `"kind": "message"` → `"proto": {"field_name": <default_value>, ...}`
- `"kind": "service"` → `"request": {...}`, `"response": {...}`
- `"kind": "action"` → `"goal": {...}`, `"result": {...}`, `"feedback": {...}`

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

Always discover the velocity topic and type first, then publish movement, and always stop after.

```bash
# Step 1 — find which velocity type this robot uses (check both)
python3 {baseDir}/scripts/ros2_cli.py topics find geometry_msgs/msg/Twist
python3 {baseDir}/scripts/ros2_cli.py topics find geometry_msgs/msg/TwistStamped

# Step 2 — publish using the discovered topic and the matching message structure
# If Twist (e.g. /cmd_vel):
python3 {baseDir}/scripts/ros2_cli.py topics publish-sequence /cmd_vel \
  '[{"linear":{"x":1.0,"y":0,"z":0},"angular":{"x":0,"y":0,"z":0}},{"linear":{"x":0,"y":0,"z":0},"angular":{"x":0,"y":0,"z":0}}]' \
  '[2.0, 0.5]'

# If TwistStamped (e.g. /cmd_vel):
python3 {baseDir}/scripts/ros2_cli.py topics publish-sequence /cmd_vel \
  '[{"header":{"stamp":{"sec":0},"frame_id":""},"twist":{"linear":{"x":1.0,"y":0,"z":0},"angular":{"x":0,"y":0,"z":0}}},{"header":{"stamp":{"sec":0},"frame_id":""},"twist":{"linear":{"x":0,"y":0,"z":0},"angular":{"x":0,"y":0,"z":0}}}]' \
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

| User intent | Monitor topic | Field(s) | Condition |
|-------------|--------------|----------|-----------|
| Move forward N m (straight) | `/odom` | `pose.pose.position.x` | `--delta N` |
| Move backward N m | `/odom` | `pose.pose.position.x` | `--delta -N` |
| Move sideways N m | `/odom` | `pose.pose.position.y` | `--delta N` |
| Move N m (any direction, 2D) | `/odom` | `pose.pose.position.x` `pose.pose.position.y` | `--euclidean --delta N` |
| Move N m (any direction, 3D) | `/odom` | `pose.pose.position.x` `pose.pose.position.y` `pose.pose.position.z` | `--euclidean --delta N` |
| Move N m (shorthand, auto-expanded) | `/odom` | `pose.pose.position` | `--euclidean --delta N` |
| Rotate N rad | `/odom` | `pose.pose.orientation.z` | `--delta N` |
| Joint reach angle | `/joint_states` | `position.0` (index of joint) | `--equals A` or `--delta D` |
| Multi-joint Euclidean distance | `/joint_states` | `position.0` `position.1` | `--euclidean --delta D` |
| Stop near obstacle | `/scan` | `ranges.0` (front index) | `--below 0.5` |
| Stop at range | `/range` | `range` | `--below D` |
| Stop at temperature | `/temperature` | `temperature` | `--above T` |
| Stop at battery level | `/battery` | `percentage` | `--below P` |

**`--euclidean`** takes any number of numeric fields, computes `sqrt(Σ(current_i - start_i)²)`, and stops when that distance ≥ the `--delta` threshold. Use it whenever the robot's path is not axis-aligned.

**Composite field auto-expansion**: if a `--field` path resolves to a sub-message dict (e.g. `pose.pose.position` → `{x, y, z}`), `--euclidean` mode automatically expands it to all direct numeric children (sorted alphabetically: `x, y, z`). This lets you write `--field pose.pose.position --euclidean --delta 1.0` instead of listing every sub-field explicitly. Non-numeric children (strings, nested dicts) are skipped.

#### Examples

```bash
# "Drive forward 1 meter" — straight path, single axis
python3 {baseDir}/scripts/ros2_cli.py topics publish-until /cmd_vel \
  '{"linear":{"x":0.2,"y":0,"z":0},"angular":{"x":0,"y":0,"z":0}}' \
  --monitor /odom --field pose.pose.position.x --delta 1.0 --timeout 30

# "Drive 2 meters in any direction" — curved/diagonal path, Euclidean XY (explicit fields)
python3 {baseDir}/scripts/ros2_cli.py topics publish-until /cmd_vel \
  '{"linear":{"x":0.2,"y":0,"z":0},"angular":{"x":0,"y":0,"z":0.3}}' \
  --monitor /odom \
  --field pose.pose.position.x pose.pose.position.y \
  --euclidean --delta 2.0 --timeout 60

# "Drive 1 meter in any direction (3D)" — shorthand: auto-expands position to x,y,z
python3 {baseDir}/scripts/ros2_cli.py topics publish-until /cmd_vel \
  '{"linear":{"x":0.2,"y":0,"z":0},"angular":{"x":0,"y":0,"z":0}}' \
  --monitor /odom \
  --field pose.pose.position \
  --euclidean --delta 1.0 --timeout 30

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
- `topics publish-continuous` — alias for `topics publish`; `--duration` / `--timeout` is optional (single-shot without it)
- `services call` — can reset, spawn, kill, or change robot state
- `params set` — modifies runtime parameters
- `actions send` — triggers robot actions (rotation, navigation, etc.)
- `control set-controller-state` / `control switch-controllers` — activating or deactivating controllers affects what the robot can do
- `control set-hardware-component-state` — driving hardware through lifecycle states can stop actuators or sensors
- `control reload-controller-libraries` — stops all running controllers if `--force-kill` is used

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
| Controller manager service not available | ros2_control not running or wrong namespace | Check with `nodes list`; use `--controller-manager` to set the correct namespace |
