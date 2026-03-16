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

### 2026-03-16 — Agent used hallucinated `launch start` subcommand, asked user for permission to retry

**Task:** Run a launch file.
**Violation:** Used `launch start` (non-existent subcommand). On error, asked the user "Would you like me to retry?" instead of self-correcting.
**Self-correction:** Caught post-hoc via user feedback. Correct subcommand is `launch new`. Should have run `launch --help` before attempting, then retried silently.
**Root cause:** Treated rules as guidelines; defaulted to legacy habits (asking user, improvising subcommand names).
**Rule reference:** Rule 0.5 (hallucination), Rule 5 (execute, don't ask), Rule 7 (self-correct before reporting).
**Outcome:** Rules 0.5, 5, 7 hardened; HISTORY.md created; three-phase enforcement added to preamble.
