# ROS 2 Skill Examples

This guide provides practical, high-value examples for common robotics tasks using the ROS 2 Skill CLI.

## 1. Robot Motion & Control

**Always discover topic names before moving — never hardcode `/cmd_vel` or `/odom`.**

```bash
# Step 1: discover the velocity command topic (Twist or TwistStamped)
python3 scripts/ros2_cli.py topics find geometry_msgs/msg/Twist
python3 scripts/ros2_cli.py topics find geometry_msgs/msg/TwistStamped
# Step 2: confirm exact type and payload structure
python3 scripts/ros2_cli.py topics type <VEL_TOPIC>
# Step 3: discover the odometry topic
python3 scripts/ros2_cli.py topics find nav_msgs/msg/Odometry
```

Use the discovered `<VEL_TOPIC>` and `<ODOM_TOPIC>` in all motion commands below — not hardcoded names.

### Publish a Velocity Sequence
Move forward at 0.5 m/s for 2 seconds, then stop (open-loop — no distance guarantee).
```bash
python3 scripts/ros2_cli.py topics publish-sequence <VEL_TOPIC> \
  '[{"linear":{"x":0.5},"angular":{"z":0.0}}, {"linear":{"x":0.0},"angular":{"z":0.0}}]' \
  '[2.0, 0.5]'
```

### Drive Until Condition (Closed-Loop)
Drive forward at 0.2 m/s until the robot has moved 1.0 meter from its starting position.
```bash
python3 scripts/ros2_cli.py topics publish-until <VEL_TOPIC> \
  '{"linear":{"x":0.2},"angular":{"z":0.0}}' \
  --monitor <ODOM_TOPIC> --field pose.pose.position.x --delta 1.0
```

For diagonal/curved paths, use `--euclidean` across both X and Y:
```bash
python3 scripts/ros2_cli.py topics publish-until <VEL_TOPIC> \
  '{"linear":{"x":0.2},"angular":{"z":0.1}}' \
  --monitor <ODOM_TOPIC> --field pose.pose.position.x pose.pose.position.y \
  --euclidean --delta 1.0
```

### Drive with Deceleration Zone (Closed-Loop, Smooth Stop)
Use `--slow-last` to ramp down to `--slow-factor` speed over the last N meters.
Prevents overshoot on long moves — always use for moves > 2 m.
```bash
python3 scripts/ros2_cli.py topics publish-until <VEL_TOPIC> \
  '{"linear":{"x":0.4},"angular":{"z":0.0}}' \
  --monitor <ODOM_TOPIC> --field pose.pose.position.x --delta 3.0 \
  --slow-last 0.5 --slow-factor 0.3
```
`--slow-last 0.5` activates decel when 0.5 m remains. `--slow-factor 0.3` scales velocity to 30% during decel.

### Rotate by Angle (Closed-Loop)
Rotate 90 degrees CCW (positive `--rotate`, positive `angular.z`):
```bash
python3 scripts/ros2_cli.py topics publish-until <VEL_TOPIC> \
  '{"linear":{"x":0.0},"angular":{"z":0.5}}' \
  --monitor <ODOM_TOPIC> --rotate 90 --degrees --timeout 30
```

Rotate 90 degrees CW (negative `--rotate`, negative `angular.z`):
```bash
python3 scripts/ros2_cli.py topics publish-until <VEL_TOPIC> \
  '{"linear":{"x":0.0},"angular":{"z":-0.5}}' \
  --monitor <ODOM_TOPIC> --rotate -90 --degrees --timeout 30
```

---

## 2. System Diagnostics & Battery

### Monitor Battery Status
Auto-discover all battery topics and return a one-shot status report.
```bash
python3 scripts/ros2_cli.py topics battery
```

### Continuous Diagnostics
Watch diagnostic updates for 10 seconds to catch intermittent hardware warnings.
```bash
python3 scripts/ros2_cli.py topics diag --duration 10 --max-messages 5
```

---

## 3. Parameters & Configuration

**Always discover the node name first — never hardcode it.**

```bash
# Discover the node name
python3 scripts/ros2_cli.py nodes list
```

### Save and Restore Presets
Capture the current state of a node's parameters and restore them later.
```bash
# Save current settings (use the node name discovered above)
python3 scripts/ros2_cli.py params preset-save <NODE_NAME> my_preset

# Restore later
python3 scripts/ros2_cli.py params preset-load <NODE_NAME> my_preset
```

### Bulk Update from File
Update multiple parameters at once using a JSON file.
```bash
python3 scripts/ros2_cli.py params load <NODE_NAME> ./configs/params.json
```

---

## 4. Hardware & Controllers (ros2_control)

### List Hardware & Interfaces
Check which hardware components are registered and what interfaces they provide.
```bash
python3 scripts/ros2_cli.py control lhc
python3 scripts/ros2_cli.py control lhi
```

### Atomic Controller Switch
**Always discover controller names before switching — never hardcode them.**

```bash
# Discover available controllers and their current states
python3 scripts/ros2_cli.py control list-controllers
```

Deactivate one controller and activate another in a single atomic transaction:
```bash
python3 scripts/ros2_cli.py control switch-controllers \
  --deactivate <ACTIVE_CONTROLLER> \
  --activate <INACTIVE_CONTROLLER> \
  --strictness STRICT
```

---

## 5. Transforms (TF2)

**Always discover frame names first — never hardcode `map`, `base_link`, `odom`, or any frame name.**

```bash
# Discover all available TF frames
python3 scripts/ros2_cli.py tf list
```

### Lookup Transform
Get the current transform between two frames (use names from `tf list`):
```bash
python3 scripts/ros2_cli.py tf lookup <SOURCE_FRAME> <TARGET_FRAME>
```

### Transform a Point
Transform a coordinate from one frame to another:
```bash
python3 scripts/ros2_cli.py tf transform-point <TARGET_FRAME> <SOURCE_FRAME> 1.0 0.5 0.0
```

---

## 6. Vision & Perception

**Always discover the camera topic — never hardcode `/camera/image_raw/compressed` or similar.**

```bash
# Discover available image topics
python3 scripts/ros2_cli.py topics find sensor_msgs/msg/CompressedImage
python3 scripts/ros2_cli.py topics find sensor_msgs/msg/Image
```

### Capture Image
Grab a frame from the discovered camera topic:
```bash
python3 scripts/ros2_cli.py topics capture-image \
  --topic <CAMERA_TOPIC> \
  --output robot_view.jpg
```

If multiple camera topics are found, prefer the one matching the user's context (front, rear, color, depth). If ambiguous, use the first result.

### Visualise Controller Chains
Generate a Graphviz diagram of how controllers are chained together.
```bash
python3 scripts/ros2_cli.py control vcc --output controller_map.pdf
```

---

## 7. Services

**Always discover the service name and request structure — never hardcode them.**

```bash
# Discover available services
python3 scripts/ros2_cli.py services list

# Get the request/response structure for the target service
python3 scripts/ros2_cli.py services details <SERVICE_NAME>
```

### Reliable Service Call
Increase timeout and retry count for a call on a slow or unreliable system:
```bash
python3 scripts/ros2_cli.py --timeout 30 --retries 3 services call <SERVICE_NAME> \
  '<json_request>'
```

---

## 8. Deployment & Session Management

### Launch a Package
Start a ROS 2 launch file in a managed tmux session.
```bash
python3 scripts/ros2_cli.py launch new my_robot_bringup robot.launch.py
```

### Run with Presets
Run a node and automatically apply a parameter preset before it starts.
```bash
python3 scripts/ros2_cli.py run new turtlesim turtlesim_node --presets indoor_mode
```

---

## 9. Interface Discovery

### Prototype Payloads
Get a JSON template for a message type to use in a `publish` command.
```bash
python3 scripts/ros2_cli.py interface proto geometry_msgs/msg/Twist
```
