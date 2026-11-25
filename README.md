# ros2-skill

[![Tests](https://github.com/adityakamath/ros2-skill/actions/workflows/pytest.yml/badge.svg?branch=main)](https://github.com/adityakamath/ros2-skill/actions/workflows/pytest.yml) ![License](https://img.shields.io/badge/License-Apache%202.0-blue)

[Agent Skill](https://agentskills.io) that lets an AI agent (built for Claude Code) operate and inspect a ROS 2 robot through one JSON CLI on top of rclpy. No rosbridge.

```text
agent -> ros2_cli.py -> rclpy -> ROS 2 graph
```

The agent reads `SKILL.md` (short) and loads `references/` on demand. Static facts (topics, limits, frames, controllers) come from a robot profile built once from the workspace; live calls are for runtime state only. Motion is closed loop with velocity limits, a pre-motion check (controller, odometry, e-stop, drive mode, battery), one motion command at a time, and an `estop` that works on robots with a twist switch.

## Install

Needs python3, `rclpy`, a sourced ROS 2 environment (Kilted or Jazzy), and `tmux` for `launch`/`run`. Optional: `opencv-python` + `numpy` (`topics capture-image`), `requests` (`discord_tools.py`).

```bash
git clone https://github.com/adityakamath/ros2-skill ~/.claude/skills/ros2-skill
source /opt/ros/$ROS_DISTRO/setup.bash
python3 ~/.claude/skills/ros2-skill/scripts/ros2_cli.py version
```

Then ask Claude, e.g. "what topics are there?", "drive forward 1 m", "look around", "stop". Build the profile once with `profile scan --name <robot>` (the agent does this when none exists). `ros2_cli.py --help` lists every command; [`references/COMMANDS.md`](references/COMMANDS.md) is generated from it.

## Layout

| Path | Contents |
|---|---|
| `SKILL.md` | What Claude loads: rules index, hard constraints, core commands |
| `references/` | `RULES-*.md` (conduct, preflight, motion, diagnostics, intent map), `GUIDES.md`, `PROFILE.md`, `EXAMPLES.md`, generated `COMMANDS.md` |
| `scripts/` | `ros2_cli.py` (entry point and dispatch) plus one module per command group; `gen_commands_doc.py` regenerates `COMMANDS.md` |
| `tests/` | Unit tests, evals (`tests/evals/`), opt-in live and smoke tests |

Generated at runtime inside the skill directory (gitignored): `.artifacts/` (captures, missions, places), `.presets/` (parameter presets, `safety.json`), `.profiles/`, `.fastd/`.

## Tests

```bash
source /opt/ros/$ROS_DISTRO/setup.bash
python3 -m pytest tests/ -q                      # unit tests; ROS-dependent ones skip without ROS
pytest tests/test_sim_smoke.py --sim             # read-only smoke tests against a running graph (sim:=true or the robot)
ROS2_SIM_MOTION=1 pytest tests/test_sim_smoke.py --sim   # also a 0.1 m closed-loop move
pytest tests/test_live_validation.py --live      # drives the real robot
```

After changing a command's flags, run `python3 scripts/gen_commands_doc.py` (a test fails if `COMMANDS.md` is stale).

## Discord (optional)

`scripts/discord_tools.py send-image` reads a bot token from `--config <path>`, `~/.nanobot/config.json`, or `~/.config/ros2-skill/discord_tools.json`:

```json
{"channels": {"discord": {"token": "YOUR_BOT_TOKEN"}}}
```

## Credits

Adapted from [ros-skill](https://github.com/lpigeon/ros-skill) by [@lpigeon](https://github.com/lpigeon). Apache-2.0. History: [CHANGELOG.md](CHANGELOG.md).
