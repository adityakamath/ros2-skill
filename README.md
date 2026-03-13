# ROS 2 Skill

![Status](https://img.shields.io/badge/Status-Active-green)
[![ClawHub](https://img.shields.io/badge/ClawHub-ros2--skill-orange)](https://clawhub.ai/adityakamath/ros2-skill)
![Static Badge](https://img.shields.io/badge/ROS%202-Supported-green)
[![Repo](https://img.shields.io/badge/Repo-adityakamath%2Fros2--skill-purple)](https://github.com/adityakamath/ros2-skill)
[![Blog](https://img.shields.io/badge/Blog-kamathrobotics.com-darkorange)](https://kamathrobotics.com)
[![Ask DeepWiki (Experimental)](https://deepwiki.com/badge.svg)](https://deepwiki.com/adityakamath/ros2-skill)
![Python](https://img.shields.io/badge/python-3.10%2B-blue)
![Static Badge](https://img.shields.io/badge/License-Apache%202.0-blue)

[Agent Skill](https://agentskills.io) for ROS 2 robot control via rclpy.

```text
Agent (LLM) → ros2_cli.py → rclpy → ROS 2
```

## Overview

An AI agent skill that lets agents control ROS 2 robots through natural language. The agent reads `SKILL.md`, understands available commands, and executes `ros2_cli.py` to interact with ROS 2 directly via rclpy — no rosbridge required, perfect for on-board deployment.

The long-term goal is full parity with the `ros2` CLI — every command available in a terminal, made accessible to an AI agent. Beyond that baseline, ros2-skill adds capabilities that only make sense in an agent context: goal-conditioned publishing, sensor image capture, system diagnostics, and external integrations like Discord reporting.

## Quick Start (CLI)

```bash
# Source ROS 2 environment
source /opt/ros/${ROS_DISTRO}/setup.bash

# Run commands
python3 scripts/ros2_cli.py version
python3 scripts/ros2_cli.py topics list
python3 scripts/ros2_cli.py nodes list

# Move robot forward for 3 seconds
python3 scripts/ros2_cli.py topics publish /cmd_vel \
  '{"linear":{"x":1.0,"y":0,"z":0},"angular":{"x":0,"y":0,"z":0}}' --duration 3

# Read sensor data
python3 scripts/ros2_cli.py topics subscribe /scan --duration 3
```

## Quick Start (AI Agent)

**ros2-skill** works with any AI agent that supports [Agent Skills](https://agentskills.io). For easy setup, I recommend using [nanobot](https://github.com/HKUDS/nanobot), a lightweight alternative to [OpenClaw](https://github.com/openclaw/openclaw) that can run directly on-board the ROS 2 robot's computer. Install **ros2-skill** from [ClawHub](https://clawhub.ai/adityakamath/ros2-skill) and talk to your robot:

- "What topics are available?"
- "Move the robot forward 1 meter"
- "Trigger the emergency stop"

**Agent Workflow:** The agent automatically:
1. Understands user intent (subscribe/publish/call/send)
2. Discovers relevant topics, services, actions from the live graph
3. Finds message types and structures
4. Applies safety limits from parameters
5. Executes the command

No user clarification needed — the agent uses ros2-skill tools to answer all its own questions.

## Supported Commands

| Category | Commands |
| -------- | -------- |
| Connection | `version` |
| Topics | `list`, `type`, `details`, `message`, `subscribe`, `publish`, `hz`, `bw`, `delay`, `find` |
| Services | `list`, `type`, `details`, `call`, `find`, `echo` |
| Nodes | `list`, `details` |
| Parameters | `list`, `get`, `set`, `describe`, `dump`, `load`, `delete`, `preset-*` |
| Actions | `list`, `details`, `type`, `send`, `cancel`, `echo`, `find` |
| Lifecycle | `nodes`, `list`, `get`, `set` |
| Control | `list-controller-types`, `list-controllers`, `list-hardware-components`, `list-hardware-interfaces`, `load-controller`, `unload-controller`, `configure-controller`, `reload-controller-libraries`, `set-controller-state`, `set-hardware-component-state`, `switch-controllers`, `view-controller-chains` |
| Doctor | `check` (default), `hello` |
| Wtf | alias for `doctor` — same commands |
| Multicast | `send`, `receive` |
| Interface | `list`, `show`, `proto`, `packages`, `package` |
| Launch | `run`, `list`, `ls`, `kill` |

All commands output JSON. See [`SKILL.md`](SKILL.md) for quick reference and [`references/COMMANDS.md`](references/COMMANDS.md) for full details with examples.

## Agent Features

Capabilities that go beyond standard `ros2` CLI parity — designed specifically for AI agents operating on mobile robots:

| Feature | Command(s) | Description |
|---------|------------|-------------|
| **Emergency stop** | `estop` | Send zero-velocity command to halt mobile robots safely |
| **Publish sequence** | `topics publish-sequence` | Publish a timed sequence of different messages in one call |
| **Publish-until** | `topics publish-until` | Publish repeatedly and stop automatically when a condition is met (supports Euclidean distance and rotation) |
| **Image capture** | `topics capture-image` | Grab a frame from any ROS 2 image topic and save to `.artifacts/` |
| **Diagnostics monitoring** | `topics diag-list`, `topics diag` | Discover and read `DiagnosticArray` topics by type, with human-readable level names |
| **Battery monitoring** | `topics battery-list`, `topics battery` | Discover and read `BatteryState` topics by type, with decoded status, health, and technology names |
| **Parameter presets** | `params preset-save/load/list/delete` | Save and restore complete parameter sets for a node by name |
| **Launch files** | `launch run/list/kill/restart/foxglove` | Run launch files in tmux sessions, list/kill/restart running sessions, launch foxglove_bridge |
| **Discord integration** | `discord_tools.py send-image` | Send images (or PDFs) to a Discord channel via bot token |

### Global Options

Place these **before** the command name to apply a setting to every service/action call:

| Option | Default | Description |
|--------|---------|-------------|
| `--timeout SECONDS` | per-command default | Override the per-command timeout (useful for slow networks) |
| `--retries N` | `1` | Total attempts before giving up; `1` = no retry |

```bash
python3 scripts/ros2_cli.py --timeout 30 --retries 3 lifecycle get /camera_driver
```

### Message Type Aliases

The skill supports 50 message type aliases for commonly used ROS 2 message types. Use short names instead of full type names:

- `twist` → `geometry_msgs/Twist`
- `odom` → `nav_msgs/Odometry`
- `laserscan` → `sensor_msgs/LaserScan`
- `image` → `sensor_msgs/Image`

**Example:**
```bash
# Using alias
python3 scripts/ros2_cli.py topics message twist

# Equivalent to
python3 scripts/ros2_cli.py topics message geometry_msgs/Twist
```

See [Message Type Aliases](references/COMMANDS.md#message-type-aliases) for the full list.

See [`EXAMPLES.md`](EXAMPLES.md) for usage examples including image capture and Discord integration.

---

## How It Works

1. The agent platform loads `SKILL.md` into the agent's system prompt
2. The agent platform substitutes `{baseDir}` with the actual skill installation path
3. User asks something like "move the robot forward"
4. **Agent thinks:** "This requires publishing velocity commands. I need to find Twist topics, get the message structure, check safety limits, then publish."
5. **Agent auto-discovers:**
   - `topics find geometry_msgs/Twist` + `TwistStamped` → finds `/cmd_vel`
   - `topics message geometry_msgs/Twist` → gets structure
   - `params list /diff_drive_controller` → gets safety limits
6. Agent executes: `python3 {baseDir}/scripts/ros2_cli.py topics publish /cmd_vel ...`
7. `ros2_cli.py` uses rclpy to communicate with ROS 2 and returns JSON
8. Agent parses the JSON and responds in natural language

The agent never asks for clarification — it automatically discovers topics, services, actions, message types, topic names, and safety limits from the live ROS 2 graph.

## File Structure

```
ros2-skill/
├── SKILL.md                   # Skill document (loaded into agent's system prompt)
├── scripts/
│   ├── ros2_cli.py            # Entry point — parser, dispatch table, re-exports
│   ├── ros2_utils.py          # Shared infrastructure (ROS2CLI node, output, msg helpers)
│   ├── ros2_topic.py          # Topic commands + estop
│   ├── ros2_node.py           # Node commands
│   ├── ros2_param.py          # Parameter commands
│   ├── ros2_service.py        # Service commands
│   ├── ros2_action.py         # Action commands
│   ├── ros2_lifecycle.py      # Lifecycle (managed node) commands
│   ├── ros2_interface.py      # Interface type discovery commands
│   ├── ros2_doctor.py         # Doctor / Wtf system diagnostics
│   ├── ros2_multicast.py      # Multicast (UDP) diagnostics
│   ├── ros2_control.py        # Controller manager commands
│   └── discord_tools.py       # Discord integration
├── references/
│   └── COMMANDS.md            # Full command reference with output examples
└── tests/
    └── test_ros2_cli.py       # Unit tests
```

## Requirements

- Python 3.10+
- ROS 2 (Humble, Iron, Jazzy, or compatible distribution), environment sourced

**Optional:**
- `opencv-python` and `numpy` — required for `topics capture-image`
- `requests` — required for `discord_tools.py send-image`

## Testing

```bash
source /opt/ros/${ROS_DISTRO}/setup.bash
python3 -m pytest tests/ -v
```

Tests that require a live ROS 2 environment will skip gracefully if one is not available.

## Changelog

See [CHANGELOG.md](CHANGELOG.md).

---

Adapted from [ros-skill](https://github.com/lpigeon/ros-skill) by [@lpigeon](https://github.com/lpigeon).
