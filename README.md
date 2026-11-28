# ros2-skill

![Status](https://img.shields.io/badge/Status-Active-green) [![Tests](https://github.com/adityakamath/ros2-skill/actions/workflows/pytest.yml/badge.svg?branch=main)](https://github.com/adityakamath/ros2-skill/actions/workflows/pytest.yml) [![ROS 2](https://img.shields.io/badge/ROS_2-Kilted%20%7C%20Jazzy-22314E?logo=ros&logoColor=white)](https://docs.ros.org/en/kilted/) ![License](https://img.shields.io/badge/License-Apache%202.0-blue)

[Agent Skill](https://agentskills.io) that lets an AI agent (built for Claude Code) operate and inspect a ROS 2 robot through one JSON CLI on top of rclpy. No rosbridge.

```text
agent -> ros2_cli.py -> rclpy -> ROS 2 graph
```

The agent reads `SKILL.md` (short) and loads `references/` on demand. Static facts (topics, limits, frames, controllers, range sensors) come from a robot profile built from the workspace and loaded into the agent's context with the skill; live calls are for runtime state only. Motion is closed loop with velocity limits, a pre-motion check (controller, odometry, e-stop, drive mode, battery), one motion command at a time, an obstacle stop, and an `estop` that works on robots with a twist switch.

## Install

Needs python3, `rclpy`, a sourced ROS 2 environment (Kilted or Jazzy), and `tmux` for `launch`/`run`. Optional: `opencv-python` + `numpy` (`topics capture-image`), `requests` (`discord_tools.py`).

```bash
git clone https://github.com/adityakamath/ros2-skill ~/.claude/skills/ros2-skill
source /opt/ros/$ROS_DISTRO/setup.bash
python3 ~/.claude/skills/ros2-skill/scripts/ros2_cli.py version
```

Then ask Claude, e.g. "what topics are there?", "drive forward 1 m", "look around", "stop". The profile is created and kept up to date when the skill loads (`profile context`); `profile scan --name <robot>` builds one by hand. `ros2_cli.py --help` lists every command; [`references/COMMANDS.md`](references/COMMANDS.md) is generated from it.

## Speed

`plan "<goal>" --run` compiles and runs a plain goal in one call. The fast daemon (`daemon-fast`, started automatically on the first command that benefits, 30-minute idle exit, `ROS2_SKILL_NO_FASTD=1` to disable) keeps the sensors and the velocity publisher warm: `snapshot` and `look` answer in about 0.3 s, and a closed-loop move starts about 0.15 s after the command. Halting a daemon-hosted move works from Ctrl-C, `kill`, `estop`, or a lost daemon (the controller's `cmd_vel_timeout` ends it); see `references/RULES-MOTION.md`.

## Obstacle stop

Every base command with a linear velocity checks the whole path it is about to sweep in every LaserScan, point cloud and depth image the profile lists, and watches a running Nav2 collision monitor. It does not start towards an obstacle, stops short of one, and stops when a sensor watching the way is lost (dropped frames get a short grace while another still watches). If the profile knows no range sensor, the move is refused (`--no-obstacle-stop` per command, or `profile set-obstacle --no-sensors` for a robot without any). A single velocity message is zeroed after 0.5 s. "Drive forward until obstacle" is one call. It needs `numpy` (installed with ROS 2). It is not a safety-rated function: it cannot see what no sensor sees, and does not cover turning on the spot or Nav2 goals. See `references/RULES-MOTION.md` Rule 25.

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
ROS2_SIM_MOTION=1 pytest tests/test_sim_smoke.py --sim   # also a 0.1 m closed-loop move and an estop interrupt
tests/sim_ci.sh                                  # launches lekiwi_ros2 sim:=true and runs the smoke tests (CI: .github/workflows/sim.yml; it rewrites the profile, so it refuses on a machine with one unless SIM_CI_FORCE=1)
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
