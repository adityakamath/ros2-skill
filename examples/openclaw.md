# OpenClaw Tutorial

Tutorial for installing ros-skill and controlling ROS robots with OpenClaw.

## What is OpenClaw?

[OpenClaw](https://github.com/openclaw/openclaw) is a personal AI assistant that runs on your own devices. It works across messaging channels like WhatsApp, Telegram, and Slack, and can be extended with Skills.

## Step 1: Install OpenClaw

```bash
npm install -g openclaw@latest

openclaw onboard --install-daemon
```

The setup wizard will guide you through gateway, workspace, channel, and skill configuration.

## Step 2: Install ros-skill from ClawHub

[ClawHub](https://docs.openclaw.ai/tools/clawhub) is a public skills registry for OpenClaw. Install the CLI and ros-skill:

```bash
# Install ClawHub CLI
npm install -g clawhub

# Install ros-skill
clawhub install ros-skill
```

By default, skills install into `./skills/` in your working directory. Move it to one of OpenClaw's skill directories:

- `~/.openclaw/skills/` — managed skills (available globally)
- `~/.openclaw/workspace/skills/` — workspace skills (highest priority)

You can also search for it first:

```bash
clawhub search "ros-skill"
```

To update later:

```bash
clawhub update ros-skill
```


## Step 3: Setup rosbridge on the robot

ros-skill communicates with ROS robots via rosbridge WebSocket.

**ROS 1:**

```bash
sudo apt install ros-${ROS_DISTRO}-rosbridge-server
roslaunch rosbridge_server rosbridge_websocket.launch
```

**ROS 2:**

```bash
sudo apt install ros-${ROS_DISTRO}-rosbridge-server
ros2 launch rosbridge_server rosbridge_websocket_launch.xml
```

The default port is `9090`.

## Step 4: Talk to your robot

Control your robot with natural language in the OpenClaw chat.

### Connect

> "Connect to the ROS robot"

The agent runs `ros_cli.py connect` to check connectivity.

For a remote robot:

> "Connect to the ROS robot at 192.168.1.100"

### Explore

- "What topics are available?"
- "What nodes are running?"
- "What is the message type of /cmd_vel?"

### Move

- "Move the robot forward 1 meter"
- "Rotate the robot 90 degrees to the left"
- "Move in a square pattern"

### Read sensors

- "Read lidar data from the /scan topic"
- "Monitor IMU sensor data for 5 seconds"

### Services

- "Show available services"
- "Reset the turtlesim position"

## Example: Turtlesim with OpenClaw

Try the full workflow with turtlesim.

### 1. Launch turtlesim

```bash
ros2 run turtlesim turtlesim_node
ros2 launch rosbridge_server rosbridge_websocket_launch.xml
```

### 2. Chat with OpenClaw

Chat with OpenClaw like this:

- "Connect to ROS and show me what topics are available"
- "Move the turtle forward for 2 seconds"
- "What is the turtle's current position?"
- "Draw a square with the turtle"
- "Change the background color to red"

The agent automatically combines ros-skill commands to execute your requests.


## Architecture

```text
User (Chat) → OpenClaw → ros-skill → ros_cli.py → rosbridge (:9090) → ROS/ROS2 Robot
```

OpenClaw understands natural language, translates it into ros-skill CLI commands, and controls the robot through rosbridge.
