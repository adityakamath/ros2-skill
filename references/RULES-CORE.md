# Core Rules

Load at session start. These are hard constraints for any command that touches the robot or its live graph. They do not apply when you are editing the skill's own source, running its tests, or updating its docs: verify once and move on.

`CLI` means `python3 ${CLAUDE_SKILL_DIR}/scripts/ros2_cli.py`. Related: [RULES-PREFLIGHT.md](RULES-PREFLIGHT.md), [RULES-MOTION.md](RULES-MOTION.md), [RULES-DIAGNOSTICS.md](RULES-DIAGNOSTICS.md), [RULES-REFERENCE.md](RULES-REFERENCE.md).

Every action has three phases: **resolve** (Rule 0), **act**, **verify** (Rule 8). On a rule violation, stop, self-correct, retry, and note the correction in one line. If the user flags a violation, stop and correct before anything else.

---

## Rule 0.5 — Never invent commands or flags

If a command or flag is not in `CLI --help` (or COMMANDS.md), it does not exist. Run `CLI <command> --help` before any call you are unsure of. On "unknown command", run `--help` on the parent, pick the right subcommand and retry without asking the user. Asking is acceptable only after `--help` and COMMANDS.md both fail.

## Rule 1 — Resolve names, never ask for them

Never ask the user for a name, type or ID that the profile (Path A) or a live command can provide. The profile covers static fields (see the fast-path table in RULES-PREFLIGHT.md). Everything else is live:

| Need | Command |
|---|---|
| Topic / its type | `topics list`, `topics find <type>` / `topics type <t>` |
| Service / action | `services|actions list|find|details` |
| Node, its interfaces | `nodes list`, `nodes details <n>` |
| Parameters | `params list|describe|get` |
| Hardware | `control list-hardware-components` |
| Packages | `pkg list|prefix|executables` |
| Message fields | `interface show|proto <type>` (recurse into nested non-primitive types) |

Ask only if the profile field is absent and the live command comes back empty.

## Rule 2 — `ros2_cli.py` is the only interface

Use `CLI` for everything. Never call `ros2 ...` directly, and never run another `scripts/ros2_*.py` module directly. Use the `ros2` tool only if no skill command can do the job, and then say why. Before telling the user something can't be done, check `CLI --help` and COMMANDS.md.

## Rule 4 — Infer the goal, resolve the details

Work at goal level, take static details from the profile, and read runtime state live. "Take a picture" means `topics capture-image`. "Move forward 1 m" means the Rule 3 workflow. "Pan 15°" means current joint position live + 15°. "Battery?" means `topics battery`.

## Rule 5 — Execute, don't ask

The user's message is the approval. Do not ask "shall I proceed?" or narrate and wait. Run the command. Ask only when:

1. **Ambiguity** introspection cannot resolve (several matching packages or launch files, a required argument with no reasonable match).
2. **The action is destructive or the user did not authorise it** (deleting data, stopping a controller other systems may rely on).
3. **The motion quantity is vague.** Do not ask. Use these defaults and state the assumption in the report (*"Moved 0.1 m (read 'a bit' as 0.1 m)"*):

| Phrase | Default |
|---|---|
| a bit, slightly, a little | 0.1 m / 5° |
| a short distance, a bit more | 0.3 m |
| nearby, not far | 0.5 m |
| a fair distance, a good distance | 1.0 m |
| turn some, rotate a moderate amount | 15° |

"Move forward" with no qualifier is open-ended: use `publish-sequence` (Rule 3).

**Exception — processes: starting, restarting and killing them.** The user does this themselves. SKILL.md hard constraint 1 lists the commands. Give the exact command and say what to check afterwards. Once the user says a launch is up, continue (Rule 0.6 applies).

## Rule 6 — Minimal reporting

One reply per request, after all calls complete. Report the outcome, not the process:

- Success: one line (*"Moved 1.02 m forward."*).
- Movement: start pose, end pose and actual distance or angle, all from **fresh odom reads** (Rule 8), plus any anomaly (timeout, open-loop fallback, assumption).
- Failure or safety trigger: what happened, what was sent, one suggested next step.
- Self-correction: one line.

Don't narrate topic names, types, discovery steps, profile reads or the tools used. Go verbose only when asked.

## Rule 10 — Empty discovery

When `topics find` is empty: don't guess a name. Wait 1 s and retry once (DDS discovery lags), then run `topics list` and `nodes list` in parallel. Missing nodes mean the stack isn't up: run `doctor` and report. Running nodes without the topic: check `nodes details`. If both lists are empty, the stack is offline: run `doctor`, report it and stop. Report what was searched and one next step. The same applies to `services find` and `actions find`.

## Rule 11 — Use discovered names verbatim

Never strip, add or "clean up" a namespace or slash. With several candidates of one type, prefer the one matching the user's context word (front/rear, arm, a robot name, main/base). Otherwise take the first result and say which was used. If results span several robot namespaces and nothing identifies the robot, ask once which robot, then stick with it.

## Rule 12 — Parallelise independent calls

Issue independent discovery or runtime-state calls together, never in sequence. A dependent call (topic name → its type) waits for its input. In Path A there is no static discovery, so only runtime-state calls remain.

## Rule 13 — Profile data persists; runtime state does not

The profile is valid all session. Do not re-discover profile-covered fields between tasks. Re-check runtime state each task: controller active, node presence, lifecycle state, current odom or joint values.

"Fresh introspection is safer" is not a reason to re-discover static names in Path A. A name does not change between tasks, and live re-discovery hides profile–live disagreements that Rule 0.0b exists to surface. If the user asks why you did live discovery, acknowledge the mistake and use the profile next time.

If the graph changes unexpectedly mid-task (a node vanishes, a controller deactivates, a topic goes silent), stop, re-run the Rule 0.1 checks, re-check runtime state, and report what changed.

For `publish-until`/`publish-sequence` over 10 s, run `nodes list` every 10 s (at segment boundaries). If the velocity controller or odom publisher disappears, `estop`, verify, and escalate with the position.

## Rule 14 — Path A: forbidden static discovery

In Path A, do not run these to resolve static data:

| Forbidden | Use |
|---|---|
| `topics find` Twist / TwistStamped, `topics type <cmd_vel>` | `summary.cmd_vel_topic`, `summary.velocity_topics[].type` |
| `topics find nav_msgs/msg/Odometry` | `summary.localization_config.fused_sources` |
| `tf list` for frame names | `summary.tf_frames` |
| `services find std_srvs/srv/SetBool` | `summary.estop_config.service_name` |
| `params list` limit sweeps | `summary.safety_limits.binding` |
| `control list-controllers` to learn names | `summary.active_controllers` (live call is for `state` only) |
| `pkg list` / `pkg executables` for robot packages | `summary.packages`, `summary.launch_files` |

Allowed live calls: `control list-controllers` for state, `topics subscribe`/`hz`, `lifecycle get`, `nodes list`, `preflight ...`, `interface proto`. The CLI refuses violating `topics find` / `services find` calls and names the profile field. `--ignore-profile` bypasses this and is logged.

---

## Quick decision card

- Read data → `topics subscribe`. One-shot trigger → `services call` / `actions send`. System info → list commands.
- Movement → Rule 3 (RULES-MOTION.md). Closed loop (`publish-until`) whenever odom is usable, otherwise `publish-sequence` and say so.
- Velocity limits → `summary.safety_limits.binding`, else the sweep in RULES-PREFLIGHT.md.
