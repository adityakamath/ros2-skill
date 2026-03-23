---
name: ros2-skill
description: "Controls and monitors ROS 2 robots directly via rclpy CLI. Use for ANY ROS 2 robot task: topics (subscribe, publish, capture images, find by type), services (list, call), actions (list, send goals), parameters (get, set, presets), nodes, lifecycle management, controllers (ros2_control), diagnostics, battery, system health checks, and more. When in doubt, use this skill — it covers the full ROS 2 operation surface. Never tell the user you cannot do something ROS 2-related without checking this skill first."
license: Apache-2.0
compatibility: "Requires python3, rclpy, and ROS 2 environment sourced"
user-invokable: true
metadata:
  openclaw:
    emoji: "🤖"
    requires:
      bins: ["python3", "ros2"]
      pip: ["rclpy"]
    category: "robotics"
    tags: ["ros2", "robotics", "rclpy"]
  author: ["adityakamath", "lpigeon"]
  version: "1.0.5"
---

# ROS 2 Skill

Controls and monitors ROS 2 robots directly via rclpy. This skill provides a unified JSON interface for standard ROS 2 operations and agent-optimized workflows like closed-loop movement and image capture.

## 🚀 Capabilities

- **Introspection:** List and find topics, nodes, services, and actions.
- **Data Access:** Subscribe to any topic, capture camera images, and monitor diagnostics/battery.
- **Interactions:** Call services and send action goals with feedback monitoring.
- **Movement:** Agent-optimized `publish-until` (closed-loop with odom) and `publish-sequence` (timed).
- **Configuration:** Get/set parameters, use presets, and manage lifecycle nodes.
- **System:** Run `ros2 doctor`, manage `launch` files via tmux, and control `ros2_control` hardware.
- **Packages:** List installed packages, resolve prefix paths, enumerate executables, and read `package.xml` manifests — no live graph required.

## 🏗️ Architecture

- **Entry Point:** `scripts/ros2_cli.py` — **this is the only file you should ever run directly.**
- **Interface:** Agent → `ros2_cli.py` → rclpy → ROS 2 Graph
- **Format:** All commands output JSON. Errors contain `{"error": "..."}`.

> ⚠️ **Internal modules — do not run directly.**
> All `scripts/ros2_*.py` files other than `ros2_cli.py` are **internal
> implementation modules**, not standalone scripts. Running one directly
> prints an error to stderr and exits with code 1 — it performs no ROS
> operation. Always invoke commands through `ros2_cli.py`:
> ```bash
> python3 scripts/ros2_cli.py <command> [subcommand] [args]
> python3 scripts/ros2_cli.py --help   # list all commands
> ```

## 📚 Documentation Reference

To maintain performance and accuracy, this skill uses **Progressive Disclosure**:

1. **[references/RULES.md](references/RULES.md) (CRITICAL):** Mandatory Agent Behaviour Rules (0–26), Safety Protocols, and Decision Frameworks. **These are hard constraints — not guidelines. Read and follow before performing any robot action. Violation = immediate halt, self-correct, retry.**
2. **[references/COMMANDS.md](references/COMMANDS.md):** Full technical reference for all CLI subcommands, flags, and JSON payload structures.
3. **[EXAMPLES.md](EXAMPLES.md):** Practical walkthroughs for common tasks like "Move N meters" or "Capture Camera Image".

## 🛠️ Setup & Preconditions

### 1. Source ROS 2
```bash
source /opt/ros/${ROS_DISTRO}/setup.bash
```

### 2. Verify Connection
Before any operation, verify the ROS 2 environment is active:
```bash
python3 scripts/ros2_cli.py version
```

### 3. Safety First
Always check for velocity limits and active nodes before issuing movement commands. If a command hangs or the robot moves unsafely, use:
```bash
python3 scripts/ros2_cli.py estop
```
