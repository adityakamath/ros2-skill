#!/usr/bin/env bash
# Run the skill's smoke tests (including the motion and estop-interrupt tests) against lekiwi_ros2 in MuJoCo.
# Needs a sourced ROS 2 + built lekiwi_ros2 workspace (see .github/workflows/sim.yml). Usage: tests/sim_ci.sh [launch-log]
set -u
LOG="${1:-sim_launch.log}"
SKILL="$(cd "$(dirname "$0")/.." && pwd)"

# `profile scan` below rewrites the robot profile: on a developer machine that would replace the real one.
if [ -z "${CI:-}" ] && [ -z "${SIM_CI_FORCE:-}" ] && ls "$SKILL"/.profiles/*_profile.json >/dev/null 2>&1; then
  echo "refusing: $SKILL/.profiles has a profile that 'profile scan' would overwrite. Back it up, then set SIM_CI_FORCE=1." >&2
  exit 4
fi

setsid ros2 launch lekiwi_bringup lekiwi.launch.py sim:=true >"$LOG" 2>&1 &
LAUNCH=$!
cleanup() { kill -INT "$LAUNCH" 2>/dev/null; sleep 5; pkill -INT -f "[r]os2 launch lekiwi_bringup" 2>/dev/null; }
trap cleanup EXIT

# The graph is ready once /scan, the odometry and the velocity chain are all visible.
ready=0
for _ in $(seq 1 90); do
  topics="$(ros2 topic list 2>/dev/null)"
  if grep -qx "/scan" <<<"$topics" && grep -qx "/base_controller/odom" <<<"$topics" && grep -qx "/cmd_vel_presafety" <<<"$topics"; then
    ready=1; break
  fi
  sleep 2
done
if [ "$ready" != 1 ]; then echo "sim graph never came up; see $LOG" >&2; exit 2; fi
sleep 10  # controllers need a moment after the topics appear

python3 "$SKILL/scripts/ros2_cli.py" profile scan --name lekiwi >/dev/null || { echo "profile scan failed" >&2; exit 3; }
cd "$SKILL" && ROS2_SIM_MOTION=1 python3 -m pytest tests/test_sim_smoke.py --sim -v
