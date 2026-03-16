# ros2-skill — Agent Guide

This document tells you how to use the ros2-skill correctly and how to avoid the mistakes that have caused silent failures in the past. Read it before executing any ROS 2 task.

---

## What this skill does

ros2-skill gives you a structured JSON interface to a live ROS 2 robot. Use it for topics, services, actions, parameters, nodes, lifecycle, controllers, diagnostics, and daemon management. When in doubt about whether this skill covers something, check `--help` — it almost certainly does.

---

## Setup

Before running any command, ROS 2 must be sourced:

```bash
source /opt/ros/${ROS_DISTRO}/setup.bash
```

The skill base directory on this system is the directory containing `scripts/ros2_cli.py`. Find it from the skill metadata if unsure. Commands below use `{baseDir}` as a placeholder for that path.

Verify the skill is working:

```bash
python3 {baseDir}/scripts/ros2_cli.py version
```

---

## ⚠️ Entry Point — CRITICAL

**There is exactly one way to call this skill:**

```bash
python3 {baseDir}/scripts/ros2_cli.py <command> [subcommand] [args]
```

**Never run any other file directly.** Every `ros2_*.py` file in `scripts/` other than `ros2_cli.py` is an internal submodule. Running one directly prints an error and exits without performing any ROS operation.

| ✅ Correct | ❌ Wrong |
|---|---|
| `python3 {baseDir}/scripts/ros2_cli.py daemon status` | `python3 {baseDir}/scripts/ros2_daemon.py status` |
| `python3 {baseDir}/scripts/ros2_cli.py topics list` | `python3 {baseDir}/scripts/ros2_topic.py list` |
| `python3 {baseDir}/scripts/ros2_cli.py daemon start` | `ros2 daemon start` |
| `python3 {baseDir}/scripts/ros2_cli.py nodes list` | `ros2 node list` |

**Also never call the `ros2` CLI directly.** The skill returns structured JSON with consistent fields. The raw `ros2` CLI returns human-readable text that is brittle to parse and bypasses retry logic, timeouts, and safety checks.

---

## Session Start

Run these checks once at the start of every session, before any other task:

**Step 0 — Domain sanity:**
If `nodes list` returns an unexpectedly large or irrelevant set, `ROS_DOMAIN_ID` may be colliding with another system. Default is `0`. In container-alongside-host setups, both sides default to `0` and see each other.

**Step 1 — Health check:**
```bash
python3 {baseDir}/scripts/ros2_cli.py doctor
```
If critical failures are reported (DDS issues, no nodes found), stop and tell the user.

**Step 2 — Daemon check:**
```bash
python3 {baseDir}/scripts/ros2_cli.py daemon status
```
If the daemon is not running, start it and verify:
```bash
python3 {baseDir}/scripts/ros2_cli.py daemon start
python3 {baseDir}/scripts/ros2_cli.py daemon status
```

**Step 3 — Simulated time:**
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
# Example — all output is structured
python3 {baseDir}/scripts/ros2_cli.py topics list
# → {"topics": [{"name": "/cmd_vel", "type": "geometry_msgs/msg/Twist"}, ...]}

python3 {baseDir}/scripts/ros2_cli.py daemon status
# → {"status": "running", "domain_id": 0, "output": "..."}
```

---

## Core Rules (condensed from RULES.md)

Full rules are in `references/RULES.md`. These are the ones most frequently violated:

### Discover before you act — never hardcode

**Never assume topic, node, service, action, or TF frame names.** Always query the live system first.

| What you need | How to get it |
|---|---|
| Velocity topic | `topics find geometry_msgs/msg/Twist` + `topics find geometry_msgs/msg/TwistStamped` |
| Odometry topic | `topics find nav_msgs/msg/Odometry` |
| Camera topics | `topics find sensor_msgs/msg/Image` + `topics find sensor_msgs/msg/CompressedImage` |
| Node names | `nodes list` |
| Service names | `services list` or `services find <type>` |
| Action names | `actions list` or `actions find <type>` |
| TF frames | `tf list` |
| Controller names | `control list-controllers` |
| Parameter names | `params list <node>` |

### Never construct payloads from memory

Before publishing to any topic or calling any service, get the payload template:
```bash
python3 {baseDir}/scripts/ros2_cli.py interface proto <msg_type>
```
Copy the output and modify only the fields required by the task.

### Never ask the user for names the robot can provide

If a topic, service, node, or parameter name can be discovered from the live system, discover it. Only ask the user if discovery returns an empty result and there is genuinely no other way.

### Verify after every action

Never claim a result without confirming it. Subscribe to the relevant topic or call `params get` to verify the effect occurred. For movement, wait until velocity ≈ 0 before reading position.

### If a command or flag is not in COMMANDS.md or `--help`, it does not exist

Run `--help` before using any subcommand or flag you are not certain about:
```bash
python3 {baseDir}/scripts/ros2_cli.py <command> --help
python3 {baseDir}/scripts/ros2_cli.py <command> <subcommand> --help
```
Do not invent subcommands or flags. Do not try a guess and report the failure to the user — run `--help`, find the correct form, and retry silently.

---

## Safety

The emergency stop is always available:
```bash
python3 {baseDir}/scripts/ros2_cli.py estop
```

Before issuing any velocity command:
1. Run `nodes list` to get all running nodes
2. Run `params list <node>` on **every** node and look for parameters containing `max`, `limit`, `vel`, `speed`, or `accel`
3. Run `params get <node:param>` for each candidate
4. Cap your commanded velocity at the minimum discovered limit
5. If no limits are found anywhere, use conservative defaults: **0.2 m/s linear, 0.75 rad/s angular**

---

## Common Mistakes

These mistakes have caused failures on this system before. Do not repeat them.

**Calling a submodule directly:**
```bash
# ❌ WRONG — prints error and exits immediately, performs no ROS operation
python3 {baseDir}/scripts/ros2_daemon.py status

# ✅ CORRECT
python3 {baseDir}/scripts/ros2_cli.py daemon status
```

**Calling the ros2 CLI directly instead of the skill:**
```bash
# ❌ WRONG — unstructured output, bypasses skill safety and retry logic
ros2 daemon start
ros2 topic list

# ✅ CORRECT
python3 {baseDir}/scripts/ros2_cli.py daemon start
python3 {baseDir}/scripts/ros2_cli.py topics list
```

**Hardcoding topic or node names:**
```bash
# ❌ WRONG — the actual topic name varies by robot configuration
python3 {baseDir}/scripts/ros2_cli.py topics subscribe /cmd_vel

# ✅ CORRECT — discover first
python3 {baseDir}/scripts/ros2_cli.py topics find geometry_msgs/msg/Twist
# then use the discovered name
python3 {baseDir}/scripts/ros2_cli.py topics subscribe /base/cmd_vel
```

**Skipping the session-start checks:**
The daemon may not be running. Lifecycle nodes may be inactive. DDS may be misconfigured. The session-start checks catch all of these before they cause silent failures mid-task.

---

## Reference Documents

Read these for full detail. The skill uses progressive disclosure — start here, go deeper only if needed:

| Document | When to read it |
|---|---|
| `references/RULES.md` | Full mandatory rules — read before any non-trivial robot operation |
| `references/COMMANDS.md` | Complete command reference with all flags and JSON output examples |
| `EXAMPLES.md` | Practical walkthroughs for common tasks (move N metres, capture image, etc.) |
| `SKILL.md` | Skill overview and capability summary |
