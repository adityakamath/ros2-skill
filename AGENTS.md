# ros2-skill — Agent Guide

This document tells you how to use ros2-skill correctly on this system. Read it before executing any ROS 2 task.

`{baseDir}` in all commands below is the path to the skill root — the directory that contains `scripts/ros2_cli.py`. Resolve it from the skill metadata before running anything.

---

## What this skill does

ros2-skill gives you a structured JSON interface to a live ROS 2 robot. Use it for topics, services, actions, parameters, nodes, lifecycle, controllers, TF, diagnostics, and daemon management. All output is JSON. When in doubt about whether this skill covers something, run `--help` — it almost certainly does.

---

## Quick Reference

```bash
# --- Verify skill is working (no ROS graph required) ---
python3 {baseDir}/scripts/ros2_cli.py version

# --- Session-start checks (run once, before any task) ---
python3 {baseDir}/scripts/ros2_cli.py doctor               # DDS/graph health
python3 {baseDir}/scripts/ros2_cli.py daemon status        # is the ROS daemon running?
python3 {baseDir}/scripts/ros2_cli.py daemon start         # start it if not running

# --- Introspection — always do this before acting ---
python3 {baseDir}/scripts/ros2_cli.py nodes list
python3 {baseDir}/scripts/ros2_cli.py topics list
python3 {baseDir}/scripts/ros2_cli.py topics find geometry_msgs/msg/Twist      # discover velocity topic
python3 {baseDir}/scripts/ros2_cli.py topics find nav_msgs/msg/Odometry        # discover odom topic
python3 {baseDir}/scripts/ros2_cli.py services list
python3 {baseDir}/scripts/ros2_cli.py actions list
python3 {baseDir}/scripts/ros2_cli.py tf list                                  # discover TF frames

# --- Before publishing: get the payload template ---
python3 {baseDir}/scripts/ros2_cli.py interface proto geometry_msgs/msg/Twist

# --- Movement (publish-until closes the loop against odometry) ---
python3 {baseDir}/scripts/ros2_cli.py topics publish-until <topic> <msg_type> <payload> --distance 1.0
python3 {baseDir}/scripts/ros2_cli.py topics publish-until <topic> <msg_type> <payload> --rotate 90 --degrees

# --- Emergency stop (always available) ---
python3 {baseDir}/scripts/ros2_cli.py estop

# --- See all available commands and flags ---
python3 {baseDir}/scripts/ros2_cli.py --help
python3 {baseDir}/scripts/ros2_cli.py <command> --help
python3 {baseDir}/scripts/ros2_cli.py <command> <subcommand> --help
```

### Commands that work without a live ROS 2 graph

These commands do not require ROS to be running or nodes to be active:

| Command | Purpose |
|---|---|
| `version` | Verify the skill is installed and reachable |
| `daemon status / start / stop` | Manage the ROS daemon |
| `bag info <file>` | Inspect a recorded bag file |
| `component types` | List available component types |
| `--help` on any command | Inspect flags and subcommands |

All other commands require an active ROS 2 graph (sourced environment + running nodes).

---

## ⚠️ Entry Point — CRITICAL

**Use `ros2_cli.py`. Never use anything else.**

```bash
python3 {baseDir}/scripts/ros2_cli.py <command> [subcommand] [args]
```

Every other `ros2_*.py` file in `scripts/` is an internal submodule. Running one directly **prints an error and exits — it performs no ROS operation.** Calling the `ros2` CLI directly returns unstructured text and bypasses the skill's retry logic, timeouts, and safety checks.

| Mistake | What actually happens | Correct form |
|---|---|---|
| `python3 {baseDir}/scripts/ros2_daemon.py status` | Error printed, exits immediately, no ROS operation | `python3 {baseDir}/scripts/ros2_cli.py daemon status` |
| `python3 {baseDir}/scripts/ros2_topic.py list` | Error printed, exits immediately, no ROS operation | `python3 {baseDir}/scripts/ros2_cli.py topics list` |
| `ros2 daemon start` | Unstructured text output, no JSON, no retry logic | `python3 {baseDir}/scripts/ros2_cli.py daemon start` |
| `ros2 node list` | Unstructured text, fragile to parse | `python3 {baseDir}/scripts/ros2_cli.py nodes list` |
| `ros2 topic pub /cmd_vel ...` | Bypasses velocity limits and safety checks | `python3 {baseDir}/scripts/ros2_cli.py topics publish-until ...` |

---

## Session Start

Run these checks **once per session**, before any task. They take seconds and catch the most common silent failure causes.

**Step 0 — Domain sanity:**
If `nodes list` returns an unexpectedly large or irrelevant set, `ROS_DOMAIN_ID` is colliding with another system. Default is `0`. In container-alongside-host setups both sides default to `0` and see each other's nodes.

**Step 1 — Health check:**
```bash
python3 {baseDir}/scripts/ros2_cli.py doctor
```
If critical failures are reported (DDS issues, no nodes found), stop and tell the user. Do not attempt to operate a robot that fails its health check.

**Step 2 — Daemon check:**
```bash
python3 {baseDir}/scripts/ros2_cli.py daemon status
```
If the daemon is not running, start it and verify:
```bash
python3 {baseDir}/scripts/ros2_cli.py daemon start
python3 {baseDir}/scripts/ros2_cli.py daemon status
```

**Step 3 — Simulated time (if applicable):**
```bash
python3 {baseDir}/scripts/ros2_cli.py topics find rosgraph_msgs/msg/Clock
```
If `/clock` is found, subscribe for one message to confirm the simulator is not paused before issuing timed commands.

**Step 4 — Lifecycle nodes (if present):**
```bash
python3 {baseDir}/scripts/ros2_cli.py lifecycle nodes
```
Nodes in `unconfigured` or `inactive` state silently fail when their topics or services are used. Activate them before proceeding.

---

## Output Format

All commands return JSON. Errors return `{"error": "..."}`. Parse the output as JSON — never rely on text pattern matching.

```bash
python3 {baseDir}/scripts/ros2_cli.py topics list
# → {"topics": [{"name": "/cmd_vel", "type": "geometry_msgs/msg/Twist"}, ...]}

python3 {baseDir}/scripts/ros2_cli.py daemon status
# → {"status": "running", "domain_id": 0, "output": "..."}

python3 {baseDir}/scripts/ros2_cli.py topics subscribe /scan --max-messages 1
# → {"topic": "/scan", "messages": [...]}
```

---

## Output Folders

All outputs produced by ros2-skill commands are stored in hidden folders inside the skill directory. The skill creates these folders automatically if they do not exist. Never use `/tmp` or any other location.

| Folder | Contents |
|---|---|
| `{baseDir}/.artifacts/` | Captured images, logs, and all other generated outputs |
| `{baseDir}/.presets/` | Saved parameter presets (`params preset-save` / `params preset-load`) |
| `{baseDir}/.profiles/` | Robot profiles |

When in doubt about which folder to use, use `.artifacts/`.

---

## Core Rules (condensed from RULES.md)

Full mandatory rules are in `references/RULES.md`. These are the ones most commonly violated:

### 1 — Discover before you act. Never hardcode names.

**Never assume topic, node, service, action, controller, or TF frame names.** Always query the live system first. Names vary per robot configuration.

| What you need | Discovery command |
|---|---|
| Velocity command topic | `topics find geometry_msgs/msg/Twist` + `topics find geometry_msgs/msg/TwistStamped` |
| Odometry topic | `topics find nav_msgs/msg/Odometry` |
| Camera topic | `topics find sensor_msgs/msg/Image` + `topics find sensor_msgs/msg/CompressedImage` |
| Node names | `nodes list` |
| Service names | `services list` or `services find <type>` |
| Action server names | `actions list` or `actions find <type>` |
| TF frame names | `tf list` |
| Controller names | `control list-controllers` |
| Parameter names on a node | `params list <node>` |

### 2 — Get the payload template before publishing or calling

Never construct message payloads from memory. Always:
```bash
python3 {baseDir}/scripts/ros2_cli.py interface proto <msg_type>
```
Copy the output. Modify only the fields required by the task.

### 3 — Never ask the user for names the robot can provide

If a topic, service, node, or parameter name can be discovered from the live system, discover it. Only ask the user if discovery returns empty and there is genuinely no other way.

### 4 — Verify every action after executing it

Never claim a result without confirming it. Subscribe to the relevant topic or call `params get` to verify the effect occurred. For movement, wait until velocity ≈ 0 before reading position.

### 5 — Never invent commands or flags

If a subcommand or flag is not in `references/COMMANDS.md` or `--help` output, it does not exist. Run `--help` on the parent command, find the correct form, and retry silently — do not report a guess-and-fail to the user.

### 6 — On any rule violation: halt, self-correct, retry, report

If you catch a rule violation (before or after executing a command):
1. Halt immediately.
2. Self-correct autonomously — do not ask the user.
3. Retry with the correct approach.
4. Report in **one line**: what was about to go wrong, what was caught, what was corrected instead.

Never ask the user to diagnose an error you caused. If retry fails, diagnose further before escalating.

---

## Safety

The emergency stop is always available and always takes priority:
```bash
python3 {baseDir}/scripts/ros2_cli.py estop
```

**Before issuing any velocity command:**
1. Run `nodes list` to get all running nodes
2. Run `params list <node>` on every node and look for parameters containing `max`, `limit`, `vel`, `speed`, or `accel`
3. Run `params get <node:param>` for each candidate found
4. Cap your commanded velocity at the minimum discovered limit across all nodes
5. If no limits are found anywhere, use conservative defaults: **0.2 m/s linear, 0.75 rad/s angular**

Safety checks are never optional. Do not bypass them even if the user requests it.

**Confirm before any action that moves the robot or interacts with hardware.** State what you are about to do and wait for acknowledgement before executing movement commands, hardware state changes, or any irreversible action.

---

## When Things Go Wrong

| Symptom | Likely cause | Fix |
|---|---|---|
| Command prints error and exits with no ROS output | Called a `ros2_*.py` submodule directly | Use `ros2_cli.py <command>` |
| `{"error": "ros2 not found"}` or similar | ROS 2 not sourced | `source /opt/ros/${ROS_DISTRO}/setup.bash` |
| `nodes list` returns empty / wrong nodes | ROS daemon not running, or wrong `ROS_DOMAIN_ID` | `daemon status`, then `daemon start`; check `ROS_DOMAIN_ID` |
| Node is there but its topics/services do nothing | Lifecycle node in `inactive` state | `lifecycle get <node>` → `lifecycle set <node> activate` |
| Topic found but no messages arriving | Simulator paused, or publisher stopped | `topics hz <topic>` to check publish rate |
| Movement command returns but robot does not move | Controller not active or wrong topic used | `control list-controllers`, re-run topic discovery |
| `--help` returns JSON error instead of help text | ROS 2 not sourced | Source ROS 2 environment first |

---

## Reference Documents

The skill uses progressive disclosure — start here, go deeper only if needed:

| Document | When to read it |
|---|---|
| `references/RULES.md` | **Hard constraints** — not guidelines, not best practices. Read before any robot operation. They override all other instructions. Violations are critical errors requiring immediate self-correction. |
| `references/COMMANDS.md` | Complete command reference with all flags and JSON output examples |
| `EXAMPLES.md` | Practical walkthroughs (move N metres, capture image, etc.) |
| `SKILL.md` | Skill overview and capability summary |
