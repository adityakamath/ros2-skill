# Diagnostics & Verification Rules

Load when something fails, and for the post-action verification table. Related: [RULES-CORE.md](RULES-CORE.md), [RULES-MOTION.md](RULES-MOTION.md).

`CLI` means `python3 ${CLAUDE_SKILL_DIR}/scripts/ros2_cli.py`.

---

## Rule 7 — Diagnose failures yourself

On any error (wrong subcommand, bad flag, type mismatch, timeout, odd output): introspect (`--help`, COMMANDS.md, `topics details`, `nodes list`), correct, retry. Then report briefly: what failed, what introspection showed, what was done. Don't ask the user to interpret an error, don't offer a menu, and don't write "you may need to…". Finish the investigation and act. Escalate only when introspection cannot resolve it (hardware fault, stack down), with one recommended next step.

- **CLI `hint` field:** execute the suggested recovery. It is a directive, not a question.
- **Log elevation before escalating** when the cause is still unclear: `services find rcl_interfaces/srv/SetLoggerLevel`, then `services call <svc> '{"logger_name": "<node-or-empty>", "level": 10}'`, retry, read the output, and reset to 20.
- **Stalled node** (listed, inputs publishing per `topics hz`, no output): suspect a `SingleThreadedExecutor` blocked by a long callback or a `ReentrantCallbackGroup`. Look in DEBUG logs for a callback that enters and never returns. If source is available, check the executor in `main()`. The fix is a `MultiThreadedExecutor` or a non-blocking callback.
- **Commands sent, robot not moving** (odom ≈ 0 for more than 2 s). Do these in order, all before telling the user:
  1. `topics hz <VEL_TOPIC> --duration 2`. At 0 Hz the publish itself failed.
  2. Compare the commanded speed with the binding limit. If it exceeds the limit, the controller silently discards it. After `estop`, reissue at 90% of the ceiling and report it.
  3. Otherwise check `control list-controllers`, `control list-hardware-components` and `nodes list`.
- **Motion lockout.** After any motion timeout, error or unexpected stop, do not issue another motion until you have: (1) `estop`, (2) a fresh odom read for the actual pose, (3) a diagnosis (odom rate, controller state), and (4) reported cause and pose. Example: *"Timed out after 60 s. Odom 0 Hz, no velocity controller running, robot did not move. Bring up the controller stack and check odom before retrying."*

### Session errors (tmux)

Launch, run and component sessions are the user's to start and stop (see Rule 5 in RULES-CORE.md). Diagnose from the JSON, then give the user the exact recovery command:

| Condition | What to tell the user |
|---|---|
| "Session already exists" | `launch kill <session>` (or `run kill` / `component kill` for `run_` / `comp_` sessions), then the original command |
| `container_found_at` (component standalone) | Rerun with `--container-type component_container_isolated` |
| `container_started: true` | Container is slow to start; rerun with `--timeout 30` |
| `container_started: false` | Container crashed; check `run list` and report the real state |
| Stale session not cleared | `tmux kill-session -t <session>`, verify with `tmux list-sessions` |
| `session_killed` in the response | The CLI already cleaned up; rerun the original command |

### Other recoveries

| Error | Recovery |
|---|---|
| `Timeout waiting for message` | `topics details` for a publisher, QoS check (Rule 15), then a longer `--timeout` |
| `Could not load message type` | `topics type <topic>`; make sure the workspace is built and sourced |
| Service not found | `services list`, `services type` |
| Action goal rejected | `actions details`; check robot state |

## Rule 8 — Verify the effect

A zero-error return means the request was delivered. It does not mean the effect occurred. Don't say "done", "applied" or "succeeded" before verifying:

| Operation | Verify |
|---|---|
| `params set` | `params describe` before (type, range, read-only), `params get` after |
| `control switch-controllers` / `load` / `configure` | `control list-controllers` shows the expected state, and `list-hardware-components` is still `active` (an inactive component silently discards commands) |
| `lifecycle set` | `lifecycle get` shows the expected state |
| `actions send` | Always pass `--timeout`. Check `status`: SUCCEEDED means done, FAILED or CANCELED is a failure (Rule 7). On a hang, `actions cancel <goal_id>` and diagnose. Never orphan a goal. A navigation goal with no feedback progress for 10 s is stuck: cancel it. |
| `services call` | Inspect the response (`success`, `status`, `error`) and treat an indicated failure as one |
| `topics publish` (state change) | `topics subscribe <t> --max-messages 1 --timeout 3` or `topics hz` |
| `estop` | Odom `linear.x/y` and `angular.z` all < 0.01 within 5 s (10 s above 20 kg). Otherwise this is a **critical failure**: stop everything and report that the controller or topic may be offline. |
| Movement completion | (1) `topics hz <ODOM> --duration 1`, flag it if < 5 Hz; (2) read odom until stationary (all < 0.01, else wait 0.5 s and repeat); (3) then read once more and report pose from **that** reading only. If `pose.covariance[0]` or `[35]` is > 0.1, say the estimate may be unreliable. |
| Motion timeout or error | Rule 18 estop, fresh odom pose, `topics hz` + `control list-controllers`, report actual pose against target and the cause. No further motion until the cause is known. |

Never read odom for reporting while the robot is moving or decelerating. If verification fails, diagnose, fix and re-verify. After one automatic retry plus two corrected attempts (3 total), escalate as critical. Mid-motion, do not poll `topics hz` (the command's own stop condition governs). The post-motion odom read is still mandatory.

## Rule 16 — Multi-step tasks

Execute step N, verify it (Rule 8), then start step N+1. If step N fails, stop and diagnose (Rule 7). Never pipeline dependent steps. Independent discovery inside a step runs in parallel. After a motion step, its verified end pose is the next step's baseline: skip the odom subscribe, but still check node presence (`nodes list`). Rule 9's velocity and odom-rate checks still run.

| Request | Sequence |
|---|---|
| Move 1 m, then rotate 90° right | `publish-until` forward → verify delta → `publish-until` rotate → verify |
| Configure the arm controller, then send a trajectory | `configure-controller` → verify `inactive` → `switch-controllers --activate` → verify `active` → `actions send` |
| Set max speed 0.3, then move | `params set` → `params get` → motion workflow |

## Fields that explain a motion result

| Field | Meaning and action |
|---|---|
| `cause` / `limited_downstream` | A limiter (collision monitor, smoother) cut the speed (`commanded_mps` vs `executed_mps`). The robot was stopped normally and the e-stop is not latched. Read `snapshot --only scan` for what is close; do not retry blindly. |
| `slow` | The move finished but took over twice its cruise time plus 3 s: the same limiter was throttling it. Report it. |
| `cancelled: true` | `estop`, Ctrl-C, `kill` or a lost daemon interrupted the run. Velocity was zeroed and a Nav2 goal cancelled; nothing was retried. Find out why before starting another move. |
| `refused` (plan) | Part of the goal was not understood, so nothing ran. Fix or split the goal using `warnings`. |
| `source: "fast daemon"` | The move was hosted by the daemon (see RULES-MOTION.md). |
