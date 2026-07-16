# ROS 2 Skill

![Status](https://img.shields.io/badge/Status-Active-green) [![Repo](https://img.shields.io/badge/Repo-adityakamath%2Fros2--skill-purple?style=flat&logo=github&logoSize=auto)](https://github.com/adityakamath/ros2-skill) [![Tests](https://github.com/adityakamath/ros2-skill/actions/workflows/pytest.yml/badge.svg?branch=main)](https://github.com/adityakamath/ros2-skill/actions/workflows/pytest.yml) [![Blog](https://img.shields.io/badge/Blog-kamathrobotics.com-darkorange?style=flat&logo=hashnode&logoSize=auto)](https://kamathrobotics.com) [![Ask DeepWiki (Experimental)](https://deepwiki.com/badge.svg)](https://deepwiki.com/adityakamath/ros2-skill) ![Python](https://img.shields.io/badge/python-3.10%2B-blue?style=flat&logo=python&logoColor=white) ![Static Badge](https://img.shields.io/badge/License-Apache%202.0-blue)

[Agent Skill](https://agentskills.io) for ROS 2 robot control via rclpy.

```text
Agent (LLM) -> ros2-skill -> rclpy -> ROS 2 robot
```

## Overview

An AI agent skill that lets agents control ROS 2 robots through natural language. The agent reads `SKILL.md`, understands available commands, and executes `ros2_cli.py` to interact with ROS 2 directly via rclpy. No rosbridge required.

The long-term goal is full parity with the `ros2` CLI — every command available in a terminal, made accessible to an AI agent. Beyond that baseline, ros2-skill adds capabilities that only make sense in an agent context: goal-conditioned publishing, sensor image capture, system diagnostics, and external integrations like Discord reporting.

## Quick Start

**ros2-skill** is tool-agnostic: any AI agent/runtime that can load and execute [Agent Skills](https://agentskills.io) can use ros2-skill to control ROS 2 robots.

Common deployment pattern:
- Run the agent session on the robot computer (for example nanobot, Claude, or Claude Code) and load `SKILL.md` + the rules in `references/RULES-*.md`
- If needed, connect to that on-robot agent session remotely via your connector/messaging layer; the remote client talks to the agent session, and that session executes ros2-skill locally on the robot

Once configured, you can talk to your robot naturally:

- "What topics are available?"
- "Move the robot forward 1 meter"
- "Trigger the emergency stop"

**Agent Workflow:** The agent automatically:
1. Understands user intent (subscribe/publish/call/send/run/launch)
2. Uses Path A when a profile is loaded (profile-first static resolution)
3. Uses Path B when profile data is missing or incomplete (live introspection fallback)
4. Uses live introspection for runtime-only state checks in both paths
5. Finds message types and structures
6. Applies safety limits and preflight checks
7. Executes the command

No user clarification needed. The agent uses ros2-skill tools to answer its own discovery and verification questions, including finding and launching the robot stack if it is not already running.

For any deployment, load `SKILL.md` (command reference) plus the operational rules under [`references/RULES.md`](references/RULES.md).

> **Note:** `ros2_cli.py` is intended for debugging and development only. Normal usage is through the chat interface or messaging gateway of your agent platform. See [`references/CLI.md`](references/CLI.md) for the full command list.

## How It Works

1. The agent platform loads `SKILL.md` into the agent's system prompt
2. The agent platform substitutes `{baseDir}` with the actual skill installation path
3. User asks something like "move the robot forward"
4. **Agent thinks:** "This requires publishing velocity commands. I need to find Twist topics, get the message structure, check safety limits, then publish."
5. **Agent resolves requirements:**
  - Path A (profile loaded): reads static fields from profile (`cmd_vel_topic`, types, limits, frames)
  - Path B (no profile): discovers topics/types/limits from the live graph
  - Uses `interface proto` and preflight checks before actuation
6. Agent executes: `python3 {baseDir}/scripts/ros2_cli.py <command> ...`
7. `ros2_cli.py` uses rclpy to communicate with ROS 2 and returns JSON
8. Agent parses the JSON and responds in natural language

The agent minimizes clarification and follows a resolve -> act -> verify loop, with profile-first behavior and runtime safety checks.

## File Structure

```
ros2-skill/
├── SKILL.md                   # Skill document (loaded into agent's system prompt)
├── scripts/
│   ├── ros2_cli.py            # Entry point — parser, dispatch table, re-exports
│   ├── ros2_utils.py          # Shared infrastructure (ROS2CLI node, output, msg helpers)
│   ├── ros2_topic.py          # Topic commands + estop + battery + diag
│   ├── ros2_node.py           # Node commands
│   ├── ros2_param.py          # Parameter commands + presets
│   ├── ros2_service.py        # Service commands
│   ├── ros2_action.py         # Action commands
│   ├── ros2_lifecycle.py      # Lifecycle (managed node) commands
│   ├── ros2_interface.py      # Interface type discovery commands
│   ├── ros2_doctor.py         # Doctor / Wtf system diagnostics
│   ├── ros2_fastd.py          # Persistent fast daemon for low-latency hot paths
│   ├── ros2_foxglove.py       # Foxglove Bridge start/stop/status helpers
│   ├── ros2_multicast.py      # Multicast (UDP) diagnostics
│   ├── ros2_nav2.py           # Nav2 commands (go/status/rotate/map/mode/localize/diagnose)
│   ├── ros2_control.py        # Controller manager commands
│   ├── ros2_tf.py             # TF2 transform commands and math helpers
│   ├── ros2_launch.py         # Launch file session management (tmux)
│   ├── ros2_logs.py           # ROS log introspection commands
│   ├── ros2_preflight.py      # Combined motion/joint preflight checks
│   ├── ros2_profile.py        # Robot profile scan/show/list/annotate helpers
│   ├── ros2_run.py            # Executable session management (tmux)
│   ├── ros2_system.py         # Battery/shutdown/reboot commands
│   ├── ros2_bag.py            # Bag file utilities (info)
│   ├── ros2_component.py      # Composable node utilities (types, list, load, unload, kill, standalone)
│   ├── ros2_pkg.py            # Package utilities (list, prefix, executables, xml)
│   ├── ros2_daemon.py         # ROS 2 daemon management
│   └── discord_tools.py       # Discord integration
├── references/
│   ├── CLI.md                 # Supported commands, agent features, global options
│   ├── COMMANDS.md            # Full command reference with output examples
│   ├── EXAMPLES.md            # Practical usage guide for agents
│   ├── PROFILE.md             # Profile schema and extraction behavior
│   ├── RULES.md               # Rules index
│   ├── RULES-CORE.md          # Core hard constraints
│   ├── RULES-PREFLIGHT.md     # Session start and path model
│   ├── RULES-MOTION.md        # Motion safety and verification
│   ├── RULES-DIAGNOSTICS.md   # Error diagnosis workflow
│   └── RULES-REFERENCE.md     # Reference-specific task rules
├── tests/
│   ├── test_ros2_cli.py       # Unit tests for parser/dispatch/logic helpers
│   ├── test_eval_assets.py    # Eval asset schema and quality checks
│   └── evals/                 # Trigger and behavior evaluation corpora
│       ├── evals.json
│       └── eval_queries.json
├── .github/
│   └── workflows/
│       └── pytest.yml         # Unified pytest CI (tests + eval asset checks)
└── CHANGELOG.md               # Version history
```

## Requirements

- Python 3.9+
- ROS 2, environment sourced
- `tmux` — required for `launch` and `run` commands (session management)

**Optional:**
- `opencv-python` and `numpy` — required for `topics capture-image`
- `requests` — required for `discord_tools.py send-image`

## Output Directories

Commands that produce file artifacts write to subdirectories of the skill installation root:

| Directory | Contents |
|-----------|----------|
| `.artifacts/` | Captured images, logs, and all other generated outputs |
| `.presets/` | Saved parameter presets (`params preset-save` / `params preset-load`) |
| `.profiles/` | Robot profiles |

## Discord Setup

`discord_tools.py send-image` can use the same config as nanobot, or a standalone config if you are using another tool.

Config resolution order:
1. An explicit `--config <PATH>` argument
2. `~/.nanobot/config.json` if it exists
3. `~/.config/ros2-skill/discord_tools.json` if it exists

Use the same JSON schema in either location:

```json
{
  "channels": {
    "discord": {
      "token": "YOUR_BOT_TOKEN"
    }
  }
}
```

If you already use nanobot, no extra setup is needed. If you use another tool,
create `~/.config/ros2-skill/discord_tools.json` with the same schema.

See [`references/RULES-REFERENCE.md`](references/RULES-REFERENCE.md) for the full Discord image-sending workflow.

## Testing

```bash
source /opt/ros/${ROS_DISTRO}/setup.bash
python3 -m pytest tests/ -v
```

Tests that require a live ROS 2 environment will skip gracefully if one is not available.

`tests/` is the unified test root:
- `test_ros2_cli.py` validates parser/dispatch and command logic helpers
- `test_eval_assets.py` validates eval corpus quality and schema under `tests/evals/`

CI runs the same unified command via GitHub Actions workflow `.github/workflows/pytest.yml`.

Tested with ROS 2 Kilted and [nanobot](https://github.com/HKUDS/nanobot) on a Raspberry Pi.

## Changelog

Current version: **v1.0.8**. See [CHANGELOG.md](CHANGELOG.md) for the full history.

---

Adapted from [ros-skill](https://github.com/lpigeon/ros-skill) by [@lpigeon](https://github.com/lpigeon).
