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

See the [OpenClaw tutorial](examples/openclaw.md) for full setup and usage.

## Supported Commands

| Category | Commands |
| -------- | -------- |
| Connection | `version` |
| Safety | `estop` |
| Topics | `list`, `ls`, `type`, `details`, `info`, `message`, `message-structure`, `message-struct`, `subscribe`, `echo`, `sub`, `publish`, `pub`, `publish-sequence`, `pub-seq`, `publish-until`, `publish-continuous`, `hz`, `bw`, `delay`, `find`, `capture-image` |
| Services | `list`, `ls`, `type`, `details`, `info`, `call`, `find`, `echo` |
| Nodes | `list`, `ls`, `details`, `info` |
| Parameters | `list`, `ls`, `get`, `set`, `describe`, `dump`, `load`, `delete` |
| Actions | `list`, `ls`, `details`, `info`, `type`, `send`, `send-goal`, `cancel`, `echo`, `find` |
| Lifecycle | `nodes`, `list`, `ls`, `get`, `set` |
| Control | `list-controller-types`, `lct`, `list-controllers`, `lc`, `list-hardware-components`, `lhc`, `list-hardware-interfaces`, `lhi`, `load-controller`, `load`, `unload-controller`, `unload`, `reload-controller-libraries`, `rcl`, `set-controller-state`, `scs`, `set-hardware-component-state`, `shcs`, `switch-controllers`, `sc`, `swc`, `view-controller-chains`, `vcc` |
| Discord | `send-image` (in `discord_tools.py`) |

All commands output JSON. See [`SKILL.md`](SKILL.md) for quick reference and [`references/COMMANDS.md`](references/COMMANDS.md) for full details with examples.

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

---

## Media and Artifacts Folder

Images and other media are saved in the `artifacts/` folder (must be created manually in the root of the skill). This folder is used for storing captured images and other files to be sent via Discord or other integrations.

---

## Configuration

Discord integration requires a config file with the following structure:

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

---

## Image Capture Example

Capture an image from a ROS 2 topic and save to `artifacts/`:

```bash
python3 scripts/ros2_cli.py topics capture-image --topic /camera/image_raw/compressed --output test.jpg --timeout 5 --type auto
```

---

## Discord Send Example

Send an image to a Discord channel using the CLI tool:

```bash
python3 scripts/discord_tools.py send-image --path artifacts/test.jpg --channel-id 123456789012345678 --config ~/.nanobot/config.json --delete
```

---

## How It Works

1. The agent platform loads `SKILL.md` into the agent's system prompt
2. `{baseDir}` in commands is replaced with the actual skill installation path
3. User asks something like "move the robot forward"
4. Agent executes: `python3 {baseDir}/scripts/ros2_cli.py topics publish /cmd_vel ...`
5. `ros2_cli.py` uses rclpy to communicate with ROS 2 and returns JSON
6. Agent parses the JSON and responds in natural language

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
│   ├── ros2_control.py        # Controller manager commands
│   └── discord_tools.py       # Discord integration
├── references/
│   └── COMMANDS.md            # Full command reference with output examples
└── tests/
    └── test_ros2_cli.py       # Unit tests
```

## Requirements

- Python 3.10+
- ROS 2 environment sourced (`source /opt/ros/${ROS_DISTRO}/setup.bash`)

## Testing

```bash
source /opt/ros/${ROS_DISTRO}/setup.bash
python3 -m pytest tests/ -v
```

Note: Some tests require a running ROS 2 environment.

## Changelog

See [CHANGELOG.md](CHANGELOG.md).

---

Adapted from [ros-skill](https://github.com/lpigeon/ros-skill) by [@lpigeon](https://github.com/lpigeon).
