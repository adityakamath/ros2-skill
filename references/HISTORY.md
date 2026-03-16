# ROS 2 Skill: Session History

This file is an append-only log maintained by the agent. It records rule violations, unexpected behaviours, and self-corrections that occurred during robot operation sessions.

**When to append:** At the end of any session that involved a rule violation, unexpected behaviour, unexpected robot state, or self-correction — no matter how minor.

**When NOT to append:** Sessions where all rules were followed and all commands succeeded without correction. Clean sessions produce no entry.

---

## Entry Format

```
### YYYY-MM-DD — <one-line session summary>

**Task:** What the user asked for.
**Violation / Unexpected behaviour:** What went wrong and which rule was involved.
**Self-correction:** What was caught, what was corrected, and how it was verified.
**Root cause:** Why it happened (legacy habit, hallucinated command, stale state, etc.).
**Rule reference:** Which rule(s) were relevant (e.g., Rule 0.5, Rule 7).
**Outcome:** Final result after correction.
```

---

## Log

<!-- Entries are appended below in reverse-chronological order (newest first). -->

### 2026-03-16 — Agent reported stale mid-rotation quaternion as final post-rotation orientation

**Task:** Rotate the robot and report the resulting yaw.
**Violation:** Computed yaw from a quaternion (z=0.00767, w=0.99997 → ~0.88°) captured during the rotation rather than subscribing fresh after the robot stopped. The actual post-rotation quaternion was (z=0.3418, w=0.9398 → ~39.97°).
**Self-correction:** Caught post-hoc via user prompt. Should have issued a fresh `topics subscribe <ODOM_TOPIC> --max-messages 1` after `publish-until` completed and the robot was stationary, then computed yaw from that reading.
**Root cause:** Used an intermediate/cached sensor value from mid-action as if it were the final settled state. Post-action verification was performed with a transient reading, not a fresh one.
**Rule reference:** Rule 8 (verify the effect — post-action sensor reads must be fresh), Rule 0 (never rely on intermediate values).
**Outcome:** Rule 8 updated with "movement completion" verification row; Rule 0 "Never" list updated with explicit prohibition on reporting sensor values from mid-action readings.

### 2026-03-16 — Agent used hallucinated `launch start` subcommand, asked user for permission to retry

**Task:** Run a launch file.
**Violation:** Used `launch start` (non-existent subcommand). On error, asked the user "Would you like me to retry?" instead of self-correcting.
**Self-correction:** Caught post-hoc via user feedback. Correct subcommand is `launch new`. Should have run `launch --help` before attempting, then retried silently.
**Root cause:** Treated rules as guidelines; defaulted to legacy habits (asking user, improvising subcommand names).
**Rule reference:** Rule 0.5 (hallucination), Rule 5 (execute, don't ask), Rule 7 (self-correct before reporting).
**Outcome:** Rules 0.5, 5, 7 hardened; HISTORY.md created; three-phase enforcement added to preamble.
