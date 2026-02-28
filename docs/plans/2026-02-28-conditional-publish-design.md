# Conditional Publish & Continuous Publish — Design

**Date:** 2026-02-28
**Branch:** claude/loving-easley
**File:** `scripts/ros2_cli.py`

---

## Problem

The CLI only supports one-shot subscribe and fixed-duration/fixed-count publish. It cannot publish while monitoring a topic, stop conditionally, or publish indefinitely.

---

## Solution: Two new subcommands

### `topics publish-until`

Publishes a message at a fixed rate while simultaneously monitoring a separate topic. Stops as soon as a user-defined condition on the monitor topic is satisfied, or after a safety timeout.

```bash
topics publish-until <topic> <json_msg> \
  --monitor <topic>           # topic to watch (required)
  --field <dot.path>          # field path, e.g. pose.pose.position.x (required)
  --delta <float>             # stop when field changes by ±N from start value
  --above <float>             # stop when field > N (absolute)
  --below <float>             # stop when field < N (absolute)
  --equals <value>            # stop when field == value
  --rate <float>              # publish Hz (default: 10)
  --timeout <float>           # safety stop in seconds (default: 60)
  --msg-type <type>           # override publish topic type (auto-detected)
  --monitor-msg-type <type>   # override monitor topic type (auto-detected)
```

Exactly one of `--delta`, `--above`, `--below`, `--equals` is required (enforced in handler).

### `topics publish-continuous`

Publishes indefinitely until `Ctrl+C` or an optional hard timeout.

```bash
topics publish-continuous <topic> <json_msg> \
  --rate <float>    # publish Hz (default: 10)
  --timeout <float> # optional hard stop in seconds (default: none)
  --msg-type <type> # override type (auto-detected)
```

---

## Architecture

### New classes

**`resolve_field(d, path)`** (module-level helper)
- Splits dot-separated `path` and walks the nested dict/list
- Integer segments index into lists: `ranges.0` → `d["ranges"][0]`
- Raises `KeyError` / `IndexError` / `TypeError` on bad path

**`ConditionMonitor(Node)`**
- Subscribes to the monitor topic
- On first message: captures `start_value` and `start_msg`
- On each message: resolves `field`, evaluates condition, sets `stop_event` if met
- Stores `current_value` and `end_msg` on every message (last write wins)
- Sets `field_error` string and `stop_event` if field path can't be resolved

### Threading model

```
Main thread                         Spin thread (daemon)
──────────────────────────────      ─────────────────────────────
rclpy.init()                        executor.spin()
create TopicPublisher                 → ConditionMonitor.callback()
create ConditionMonitor                   → resolves field
MultiThreadedExecutor.add(both)           → sets stop_event if met
Thread(target=executor.spin).start()
publish loop:
  while not stop_event.is_set():
    check timeout → break if hit
    publisher.pub.publish(msg)
    time.sleep(interval)
stop_event.set()  (cleanup)
executor.shutdown(wait=False)
spin_thread.join(timeout=2.0)
rclpy.shutdown()
emit result JSON
```

**Condition-met vs timeout distinction:** `stop_event` is only set by `ConditionMonitor`. The timeout causes a `break` without setting `stop_event`. After the loop, `condition_met = stop_event.is_set()`.

### Condition operators

| Flag | Fires when |
|------|-----------|
| `--delta N` | `(current - start) >= N` if N≥0, `<= N` if N<0 |
| `--above N` | `current > N` |
| `--below N` | `current < N` |
| `--equals V` | numeric: `abs(current - V) < 1e-9`; fallback: `str(current) == str(V)` |

---

## Output

### `publish-until` — condition met
```json
{
  "success": true, "condition_met": true,
  "topic": "/cmd_vel", "monitor_topic": "/odom",
  "field": "pose.pose.position.x", "operator": "delta", "threshold": 1.0,
  "start_value": 0.12, "end_value": 1.15,
  "duration": 4.2, "published_count": 42,
  "start_msg": {}, "end_msg": {}
}
```

### `publish-until` — timeout
```json
{
  "success": false, "condition_met": false,
  "error": "Timeout after 30s: condition not met",
  "start_value": 0.12, "end_value": 0.43,
  "duration": 30.0, "published_count": 298
}
```

### `publish-continuous` — stopped
```json
{
  "success": true, "topic": "/cmd_vel",
  "published_count": 152, "duration": 15.2,
  "rate": 10.0, "stopped_by": "keyboard_interrupt"
}
```
`stopped_by`: `"keyboard_interrupt"` or `"timeout"`.

---

## Error cases

- No operator or multiple operators → `{"error": "Specify exactly one of --delta, --above, --below, --equals"}`
- Monitor topic type unresolvable → `{"error": "Could not detect message type for monitor topic: ..."}`
- Field path not found on first message → `{"error": "Field '...' not found in monitor message: ..."}`
- `--equals` value type mismatch → falls back to string comparison
