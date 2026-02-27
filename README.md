# ROS Skill

![Static Badge](https://img.shields.io/badge/ROS-Available-green)
![Static Badge](https://img.shields.io/badge/ROS2-Available-green)
![Static Badge](https://img.shields.io/badge/Linux-Supported-green)
![Static Badge](https://img.shields.io/badge/macOS-Supported-green)
![Static Badge](https://img.shields.io/badge/Windows-Supported-green)
![Static Badge](https://img.shields.io/badge/License-Apache%202.0-blue)
![Python](https://img.shields.io/badge/python-3.10%2B-blue)
[![ClawHub](https://img.shields.io/badge/ClawHub-ros--skill-orange)](https://clawhub.ai/lpigeon/ros-skill)

[Agent Skill](https://agentskills.io) for ROS/ROS2 robot control via rosbridge WebSocket.

```text
Agent (LLM) → ros_cli.py → rosbridge (WebSocket :9090) → ROS/ROS2 Robot
```

## Overview

An AI agent skill that lets agents control ROS robots through natural language. The agent reads `SKILL.md`, understands available commands, and executes `ros_cli.py` to interact with any ROS/ROS2 system via rosbridge.

## Quick Start (CLI)

Use `ros_cli.py` directly from the command line.

```bash
# Install
pip install websocket-client

# Launch rosbridge on the robot (ROS 2)
sudo apt install ros-${ROS_DISTRO}-rosbridge-server
ros2 launch rosbridge_server rosbridge_websocket_launch.xml
```

```bash
# Connect
python scripts/ros_cli.py connect

# Explore
python scripts/ros_cli.py topics list
python scripts/ros_cli.py nodes list

# Move robot forward for 3 seconds
python scripts/ros_cli.py topics publish /cmd_vel geometry_msgs/Twist \
  '{"linear":{"x":1.0,"y":0,"z":0},"angular":{"x":0,"y":0,"z":0}}' --duration 3

# Read sensor data
python scripts/ros_cli.py topics subscribe /scan sensor_msgs/LaserScan --timeout 3
```

## Quick Start (AI Agent)

**ros-skill** works with any AI agent that supports [Agent Skills](https://agentskills.io). For easy setup, we recommend [OpenClaw](https://github.com/openclaw/openclaw) — install **ros-skill** from [ClawHub](https://clawhub.ai/lpigeon/ros-skill) and talk to your robot:

- "Connect to the ROS robot"
- "Move the robot forward 1 meter"
- "What topics are available?"

See the [OpenClaw tutorial](examples/openclaw.md) for full setup and usage.

## Supported Commands

| Category | Commands |
| -------- | -------- |
| Connection | `connect`, `version` |
| Topics | `list`, `type`, `details`, `message`, `subscribe`, `publish`, `publish-sequence` |
| Services | `list`, `type`, `details`, `call` |
| Nodes | `list`, `details` |
| Parameters | `list`, `get`, `set` (ROS 2 only) |
| Actions | `list`, `details`, `send` (ROS 2 only) |

All commands output JSON. See `SKILL.md` for quick reference and `references/COMMANDS.md` for full details with output examples.

## How It Works

1. The agent platform loads `SKILL.md` into the agent's system prompt
2. `{baseDir}` in commands is replaced with the actual skill installation path
3. User asks something like "move the robot forward"
4. Agent executes: `python {baseDir}/scripts/ros_cli.py topics publish /cmd_vel ...`
5. `ros_cli.py` connects to rosbridge via WebSocket and returns JSON
6. Agent parses the JSON and responds in natural language

## File Structure

```text
ros-skill/
├── SKILL.md              # Skill document (loaded into agent's system prompt)
├── scripts/
│   └── ros_cli.py        # Standalone CLI tool (all ROS operations)
├── references/
│   └── COMMANDS.md       # Full command reference with output examples
├── examples/
│   ├── turtlesim.md      # Turtlesim tutorial
│   ├── sensor-monitor.md # Sensor monitoring workflows
│   └── openclaw.md       # OpenClaw integration tutorial
└── tests/
    └── test_ros_cli.py   # Unit tests (52 tests, no ROS environment needed)
```

## Requirements

- Python 3.10+
- `websocket-client` (pip package)
- rosbridge running on the robot (default port 9090)

## Testing

```bash
pip install websocket-client
python3 -m pytest tests/ -v
```

All 52 tests run without a ROS environment (WebSocket calls are mocked).