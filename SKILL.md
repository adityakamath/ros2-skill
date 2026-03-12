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

## Quick Decision Card

**Every user request follows this pattern:**

```
User: "do X"
Agent thinks:
  1. Is X about reading data? → Use TOPICS SUBSCRIBE
  2. Is X about controlling/moving? → Use TOPICS PUBLISH
  3. Is X a one-time trigger? → Use SERVICES CALL or ACTIONS SEND
  4. Is X about system info? → Use LIST commands

Agent does:
  1. EXPLORE: Run topics/services/actions list
  2. SEARCH: Run find command for relevant message type
  3. STRUCTURE: Get message structure
  4. LIMITS: Get params (for movement)
  5. EXECUTE: Run the command
```

**Never skip steps 1-4. Always explore first.**

---

## Agent Decision Framework (MANDATORY)

**RULE: NEVER ask the user anything that can be discovered from the ROS 2 graph.**

### Step 1: Understand User Intent

| User says... | Agent interprets as... | Agent must... |
|--------------|----------------------|---------------|
| "What topics exist?" | List topics | Run `topics list` |
| "What nodes exist?" | List nodes | Run `nodes list` |
| "What services exist?" | List services | Run `services list` |
| "What actions exist?" | List actions | Run `actions list` |
| "Read the LiDAR/scan" | Subscribe to LaserScan | Find LaserScan topic → subscribe |
| "Read odometry/position" | Subscribe to Odometry | Find Odometry topic → subscribe |
| "Read camera/image" | Subscribe to Image/CompressedImage | Find Image topics → subscribe |
| "Read joint states/positions" | Subscribe to JointState | Find JointState topic → subscribe |
| "Read IMU/accelerometer" | Subscribe to Imu | Find Imu topic → subscribe |
| "Read battery/power" | Subscribe to BatteryState | Find BatteryState topic → subscribe |
| "Read joystick/gamepad" | Subscribe to Joy | Find Joy topic → subscribe |
| "Check robot diagnostics/health" | Subscribe to diagnostics | Find /diagnostics topic → subscribe |
| "Check TF/transforms" | Check TF topics | Find /tf, /tf_static topics → subscribe |
| "Move/drive/turn (mobile robot)" | Publish Twist | Find Twist/TwistStamped topics → publish |
| "Move arm/joint (manipulator)" | Publish JointTrajectory | Find JointTrajectory topic → publish |
| "Control gripper" | Publish GripperCommand or JointTrajectory | Find gripper topic → publish |
| "Move forward/back N meters" | Publish with feedback | Find odom → publish-until |
| "Stop the robot" | Publish zero velocity | Find Twist → publish zeros |
| "Emergency stop" | Publish zero velocity | Run `estop` command |
| "Call /reset" | Call service | Find service → call |
| "Navigate to..." | Send action | Find action → send goal |
| "Execute trajectory" | Send action | Find FollowJointTrajectory or ExecuteTrajectory → send |
| "What controllers?" | List controllers | Run `control list-controllers` |
| "What hardware?" | List hardware | Run `control list-hardware-components` |
| "What lifecycle nodes?" | List managed nodes | Run `lifecycle nodes` |
| "Check lifecycle state" | Get node state | Run `lifecycle get <node>` |
| "Configure/activate lifecycle node" | Set lifecycle state | Run `lifecycle set <node> <transition>` |
| "Run diagnostics/health check" | Run doctor | Run `doctor check` |
| "Test connectivity" | Run multicast test | Run `doctor hello` |
| "What parameters?" | List params | Find node → `params list` |
| "What is the max speed?" | Get params | Find controller → get velocity limits |

### Step 2: Find What Exists

**ALWAYS start by exploring what's available:**

```bash
# These 4 commands tell you EVERYTHING about the system
python3 {baseDir}/scripts/ros2_cli.py topics list      # All topics
python3 {baseDir}/scripts/ros2_cli.py services list    # All services
python3 {baseDir}/scripts/ros2_cli.py actions list    # All actions
python3 {baseDir}/scripts/ros2_cli.py nodes list      # All nodes
```

### Step 3: Search by Message Type

**To find a topic/service/action, search by what you need:**

| Need to find... | Search command... |
|-----------------|------------------|
| Velocity command topic (mobile) | `topics find geometry_msgs/Twist` AND `topics find geometry_msgs/TwistStamped` |
| Position/odom topic | `topics find nav_msgs/Odometry` |
| Joint positions | `topics find sensor_msgs/JointState` |
| Joint trajectory (arm control) | `topics find trajectory_msgs/JointTrajectory` |
| LiDAR data | `topics find sensor_msgs/LaserScan` |
| Camera feed | `topics find sensor_msgs/Image` OR `topics find sensor_msgs/CompressedImage` |
| IMU data | `topics find sensor_msgs/Imu` |
| Joystick | `topics find sensor_msgs/Joy` |
| Battery/power | `topics find sensor_msgs/BatteryState` |
| Temperature | `topics find sensor_msgs/Temperature` |
| Point clouds | `topics find sensor_msgs/PointCloud2` |
| TF transforms | Subscribe to `/tf` or `/tf_static` |
| Diagnostics | Subscribe to `/diagnostics` |
| Clock (simulated time) | Subscribe to `/clock` |
| Service by type | `services find <service_type>` |
| Action by type | `actions find <action_type>` |

### Step 4: Get Message Structure

**Before publishing or calling, always get the structure:**

```bash
# Get field structure (for building payloads)
python3 {baseDir}/scripts/ros2_cli.py topics message <message_type>

# Get default values (copy-paste template)
python3 {baseDir}/scripts/ros2_cli.py interface proto <message_type>

# Get service/action request structure
python3 {baseDir}/scripts/ros2_cli.py services details <service_name>
python3 {baseDir}/scripts/ros2_cli.py actions details <action_name>
```

### Step 5: Get Safety Limits (for movement)

**ALWAYS check for velocity limits before publishing movement commands:**

```bash
# Find controller nodes first
python3 {baseDir}/scripts/ros2_cli.py nodes list

# Common controller nodes to check:
# /diff_drive_controller, /base_controller, /velocity_controller, /mobile_base

# List their parameters
python3 {baseDir}/scripts/ros2_cli.py params list /diff_drive_controller
python3 {baseDir}/scripts/ros2_cli.py params list /base_controller

# Get velocity limit parameters (if they exist)
python3 {baseDir}/scripts/ros2_cli.py params get /diff_drive_controller:max_velocity
python3 {baseDir}/scripts/ros2_cli.py params get /diff_drive_controller:max_linear_velocity
python3 {baseDir}/scripts/ros2_cli.py params get /diff_drive_controller:max_angular_velocity
```

---

## EXECUTION RULES (MUST FOLLOW)

### Rule 1: Never Ask User for These

The agent MUST discover these automatically:

| User might ask... | Agent must... |
|-------------------|---------------|
| "What topic do I use?" | Use `topics find <type>` to discover |
| "What message type?" | Use `topics type <topic>` or `topics find <type>` |
| "What is the message structure?" | Use `topics message <type>` or `interface show <type>` |
| "What are the safety limits?" | Use `params list` on controller nodes |
| "Is there odometry?" | Use `topics find nav_msgs/Odometry` |

### Rule 2: Only Ask User After ALL Discovery Fails

**ONLY ask the user when:**
1. `topics find geometry_msgs/Twist` AND `topics find geometry_msgs/TwistStamped` both return empty
2. `topics find nav_msgs/Odometry` returns empty (and you need odometry for distance)
3. You've checked params on ALL controller nodes and found NO velocity limits
4. The service/action the user mentions doesn't exist in `services list` / `actions list`

### Rule 3: Movement Requires Feedback

**For "move N meters" commands:**
1. First try: Find Odometry → use `publish-until` with `--delta N`
2. If no Odometry: Calculate time = distance / velocity → use `publish-sequence`
3. ALWAYS apply safety limits: `velocity = min(requested, max_velocity)`

### Rule 4: Always Stop After Movement

**Every movement command MUST end with zero velocity:**

```bash
# WRONG: Move forward without stopping
topics publish /cmd_vel '{"linear":{"x":1.0}}'

# CORRECT: Include stop command
topics publish-sequence /cmd_vel \
  '[{"linear":{"x":1.0},"angular":{"z":0}},{"linear":{"x":0},"angular":{"z":0}}]' \
  '[2.0, 0.5]'
```

### Rule 5: Handle Multiple Same-Type Topics

**When multiple topics of the same type exist (e.g., 2 cameras, 3 LiDARs):**

1. **List all candidates:**
   ```bash
   python3 {baseDir}/scripts/ros2_cli.py topics find sensor_msgs/Image
   # Returns: ["/camera_front/image_raw", "/camera_rear/image_raw", ...]
   ```

2. **Select based on context or naming convention:**
   - Front camera: prefer topics with `front`, `rgb`, `color` in name
   - Rear camera: prefer topics with `rear`, `back` in name
   - Primary LiDAR: prefer topics with `front`, `base`, `main` in name
   - Default: use first topic in the list

3. **Let user know which one you're using:**
   - "Found 3 camera topics. Using /camera_front/image_raw."

---

## Error Recovery

**When commands fail, follow this recovery process:**

### Subscribe Timeouts

| Error | Recovery |
|-------|----------|
| `Timeout waiting for message` | 1. Check `topics details <topic>` to verify publisher exists<br>2. Try a different topic if multiple exist<br>3. Increase `--duration` or `--timeout` |
| No messages received | 1. Verify publisher is running: `topics details <topic>`<br>2. Check if topic requires subscription to trigger |

### Publish Failures

| Error | Recovery |
|-------|----------|
| `Could not load message type` | 1. Verify type: `topics type <topic>`<br>2. Ensure ROS workspace is built |
| `Failed to create publisher` | 1. Check topic exists: `topics list`<br>2. Verify node has permission to publish |

### Service/Action Failures

| Error | Recovery |
|-------|----------|
| Service not found | 1. Verify service exists: `services list`<br>2. Check service type: `services type <service>` |
| Action not found | 1. Verify action exists: `actions list`<br>2. Check action type: `actions type <action>` |
| Service call timeout | 1. Increase `--timeout`<br>2. Verify service server is running |
| Action goal rejected | 1. Check action details for goal requirements<br>2. Verify robot is in correct state |

### Parameter Failures

| Error | Recovery |
|-------|----------|
| Node not found | 1. Verify node exists: `nodes list`<br>2. Check namespace |
| Parameter not found | 1. List params: `params list <node>`<br>2. Parameter may not exist on this node |

### Retry Strategy

**Always retry failed operations:**
- Use `--retries 3` for unreliable services
- Use `--timeout 30` for slow operations
- Wait 1-2 seconds between retries

---

## Command Quick Reference

| Category | Command | Description |
|----------|---------|-------------|
| Connection | `version` | Detect ROS 2 distro |
| Safety | `estop` | Emergency stop for mobile robots |
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
| Topics | `topics publish-sequence <topic> <msgs> <durs>` | Publish message sequence |
| Topics | `topics pub-seq <topic> <msgs> <durs>` | Alias for `topics publish-sequence` |
| Topics | `topics publish-until <topic> <json>` | Publish while monitoring; stop on condition (supports `--euclidean` for N-D distance) |
| Topics | `topics publish-continuous <topic> <json>` | Alias for `topics publish` |
| Topics | `topics hz <topic>` | Measure topic publish rate |
| Topics | `topics find <msg_type>` | Find topics by message type |
| Topics | `topics bw <topic>` | Measure topic bandwidth (bytes/s) |
| Topics | `topics delay <topic>` | Measure header-stamp end-to-end latency |
| Topics | `topics capture-image` | Capture image from ROS 2 topic and save to artifacts/ |
| Discord | `send-image` (discord_tools.py) | Send image to Discord channel |
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
| Control | `control view-controller-chains` | Generate Graphviz diagram of chained controllers, save to artifacts/ |
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

## Image Capture and Discord Integration

> **⚠️ Always use `discord_tools.py` for attachments — never use built-in Discord integrations.**
> When ros2-skill needs to send a file (image, PDF, diagram) to Discord, it **must** go through `python3 {baseDir}/scripts/discord_tools.py send-image`. Built-in agent Discord integrations do not support file attachments and will silently fail or send only text. Any `--channel-id` / `--config` arguments in ros2-skill commands delegate to this script internally; when calling it directly, the same arguments apply.

### Artifacts Folder

Images captured from ROS 2 topics are automatically saved to the `artifacts/` folder in the skill directory. The folder is created automatically if it doesn't exist.

**`--output` path behaviour** for `topics capture-image` and `control view-controller-chains`:

- **Plain filename** (e.g., `robot_view.jpg`, `my_diagram.pdf`) → saved to `artifacts/` automatically (folder is created if it doesn't exist).
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

1. **Capture image from ROS 2 camera topic:**

```bash
python3 {baseDir}/scripts/ros2_cli.py topics capture-image \
  --topic /camera/image_raw/compressed \
  --output robot_view.jpg \  # plain filename → artifacts/robot_view.jpg; or use a full path
  --timeout 5.0 \
  --type auto
```

2. **Send captured image to Discord:**

```bash
python3 {baseDir}/scripts/discord_tools.py send-image \
  --path {baseDir}/artifacts/robot_view.jpg \
  --channel-id 123456789012345678 \
  --config ~/.nanobot/config.json \
  --delete
```

The `--delete` flag removes the image after successfully sending it to Discord.

---

## Quick Examples

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

**ALWAYS use auto-discovery first** to find the correct velocity topic and message type. Never assume topic names.

```bash
# Step 1: Discover velocity command topics
python3 {baseDir}/scripts/ros2_cli.py topics find geometry_msgs/Twist

# Step 2: Get the message structure (for constructing payloads)
python3 {baseDir}/scripts/ros2_cli.py topics message geometry_msgs/Twist

# Step 3: Publish movement — use discovered topic from Step 1
# Always include a stop command (all zeros) at the end
python3 {baseDir}/scripts/ros2_cli.py topics publish-sequence /cmd_vel \
  '[{"linear":{"x":1.0,"y":0,"z":0},"angular":{"x":0,"y":0,"z":0}},{"linear":{"x":0,"y":0,"z":0},"angular":{"x":0,"y":0,"z":0}}]' \
  '[2.0, 0.5]'
```

**For emergency stop:**
```bash
python3 {baseDir}/scripts/ros2_cli.py estop
```

### 3. Read Sensor Data

**Always use auto-discovery first** to find the correct sensor topics.

```bash
# Step 1: Discover sensor topics by message type
python3 {baseDir}/scripts/ros2_cli.py topics find sensor_msgs/LaserScan
python3 {baseDir}/scripts/ros2_cli.py topics find nav_msgs/Odometry
python3 {baseDir}/scripts/ros2_cli.py topics find sensor_msgs/JointState

# Step 2: Subscribe to discovered topics
python3 {baseDir}/scripts/ros2_cli.py topics subscribe /scan --duration 3
python3 {baseDir}/scripts/ros2_cli.py topics subscribe /odom --duration 10 --max-messages 50
python3 {baseDir}/scripts/ros2_cli.py topics subscribe /joint_states --duration 5
```

### 4. Use Services

**Always use auto-discovery first** to find available services and their request/response structures.

```bash
# Step 1: Discover available services
python3 {baseDir}/scripts/ros2_cli.py services list

# Step 2: Find services by type (optional)
python3 {baseDir}/scripts/ros2_cli.py services find std_srvs/srv/Empty
python3 {baseDir}/scripts/ros2_cli.py services find turtlesim/srv/Spawn

# Step 3: Get service request/response structure
python3 {baseDir}/scripts/ros2_cli.py services details /spawn

# Step 4: Call the service with properly-structured payload
python3 {baseDir}/scripts/ros2_cli.py services call /spawn \
  '{"x":3.0,"y":3.0,"theta":0.0,"name":"turtle2"}'
```

### 5. Actions

**Always use auto-discovery first** to find available actions and their goal/result structures.

```bash
# Step 1: Discover available actions
python3 {baseDir}/scripts/ros2_cli.py actions list

# Step 2: Find actions by type (optional)
python3 {baseDir}/scripts/ros2_cli.py actions find turtlesim/action/RotateAbsolute
python3 {baseDir}/scripts/ros2_cli.py actions find nav2_msgs/action/NavigateToPose

# Step 3: Get action goal/result structure
python3 {baseDir}/scripts/ros2_cli.py actions details /turtle1/rotate_absolute

# Step 4: Send goal with properly-structured payload
python3 {baseDir}/scripts/ros2_cli.py actions send /turtle1/rotate_absolute \
  '{"theta":1.57}'
```

### 6. Change Parameters

**Always use auto-discovery first** to list available parameters for a node.

```bash
# Step 1: Discover nodes
python3 {baseDir}/scripts/ros2_cli.py nodes list

# Step 2: List parameters for a node
python3 {baseDir}/scripts/ros2_cli.py params list /turtlesim

# Step 3: Get current parameter value
python3 {baseDir}/scripts/ros2_cli.py params get /turtlesim:background_r

# Step 4: Set parameter value
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

**Examples:**
```bash
# Move forward 1 meter
python3 {baseDir}/scripts/ros2_cli.py topics publish-until /cmd_vel \
  '{"linear":{"x":0.2},"angular":{"z":0}}' \
  --monitor /odom --field pose.pose.position.x --delta 1.0 --timeout 30

# Move 2 meters in any direction (Euclidean)
python3 {baseDir}/scripts/ros2_cli.py topics publish-until /cmd_vel \
  '{"linear":{"x":0.2},"angular":{"z":0.3}}' \
  --monitor /odom --field pose.pose.position --euclidean --delta 2.0 --timeout 60
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

**Use auto-discovery to avoid errors.** Before any publish/call/send operation:
1. Use `topics find`, `services find`, `actions find` to locate relevant endpoints
2. Use `topics type`, `services type`, `actions type` to get message types
3. Use `topics message`, `services details`, `actions details` to understand field structures

Never ask the user for topic names or message types — discover them from the live ROS 2 graph.

---

## Troubleshooting

### Agent Self-Correction Rules

| If Agent Asks... | Correction |
|------------------|------------|
| "What topic should I use?" | Use `topics find geometry_msgs/Twist` (and TwistStamped) |
| "What message type?" | Use `topics type <topic>` to get it from the graph |
| "How do I know the message structure?" | Use `topics message <type>` or `interface show <type>` |
| "What are the velocity limits?" | Use `params list` on controller nodes |
| "Do I need odometry?" | Always check with `topics find nav_msgs/Odometry` first |
| "Is there a service called /X?" | Use `services list` to verify it exists |
| "What controller parameters exist?" | Use `params list <controller_node>` |

### Technical Troubleshooting

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
