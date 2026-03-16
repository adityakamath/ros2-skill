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

### Save and Restore Presets
Capture the current state of a node's parameters (e.g., camera settings) and restore them later.
```bash
# Save current settings
python3 scripts/ros2_cli.py params preset-save /camera_server indoor_mode

# Restore later
python3 scripts/ros2_cli.py params preset-load /camera_server indoor_mode
```

### Bulk Update from File
Update multiple parameters at once using a JSON file.
```bash
python3 scripts/ros2_cli.py params load /turtlesim ./configs/standard_colors.json
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
Deactivate a 'position' controller and activate a 'velocity' controller in a single transaction.
```bash
python3 scripts/ros2_cli.py control switch-controllers \
  --deactivate joint_position_controller \
  --activate joint_velocity_controller \
  --strictness STRICT
```

---

## 5. Transforms (TF2)

### Lookup Transform
Get the current transform between the `map` and `base_link` frames.
```bash
python3 scripts/ros2_cli.py tf lookup map base_link
```

### Transform a Point
Transform a coordinate from the `sensor_frame` to the `odom` frame.
```bash
python3 scripts/ros2_cli.py tf transform-point odom sensor_frame 1.0 0.5 0.0
```

---

## 6. Vision & Perception

### Capture Image
Grab a frame from a compressed camera stream.
```bash
python3 scripts/ros2_cli.py topics capture-image \
  --topic /camera/image_raw/compressed \
  --output robot_view.jpg
```

### Visualise Controller Chains
Generate a Graphviz diagram of how controllers are chained together.
```bash
python3 scripts/ros2_cli.py control vcc --output controller_map.pdf
```

---

## 7. Global Overrides

### Reliable Service Calls
Increase timeout and retry count for a service call on a high-latency network.
```bash
python3 scripts/ros2_cli.py --timeout 30 --retries 3 services call /spawn \
  '{"x": 2.0, "y": 2.0, "name": "extra_turtle"}'
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
