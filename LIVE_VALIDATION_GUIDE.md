# Live Validation Guide

## Overview

This guide walks through live validation testing on lekiwi_ros2 hardware. The robot is placed on a platform so wheels can rotate but the robot won't move, allowing safe testing of motion commands.

## Prerequisites

1. **ROS 2 Environment**: Ensure ROS 2 is sourced
   ```bash
   source /opt/ros/humble/setup.bash  # or your distro
   source ~/ros2_ws/install/setup.bash
   ```

2. **Robot Hardware**: lekiwi_ros2 on platform (wheels free to rotate)

3. **Safety Check**: Verify robot is secured on platform before running motion tests

## Quick Start

### 1. Basic Connectivity Test
```bash
cd /home/ubuntu/.nanobot/workspace/skills/ros2-skill

# Test version and ROS availability
python3 scripts/ros2_cli.py version

# Discover ROS graph
python3 scripts/ros2_cli.py context
```

Expected output:
- `rclpy_available: true`
- Topics, services, and nodes discovered

### 2. Run Live Test Suite
```bash
# Run all live validation tests
python3 -m pytest tests/test_live_validation.py -v --live

# Run specific test class
python3 -m pytest tests/test_live_validation.py::TestLiveConnectivity -v --live

# Run with timing benchmarks
python3 -m pytest tests/test_live_validation.py::TestLiveTimingBenchmark -v --live
```

### 3. Manual Command Testing

#### List Available Topics
```bash
python3 scripts/ros2_cli.py topics list
```

#### Test Safe Velocity Command (Zero)
```bash
python3 scripts/ros2_cli.py topics publish /cmd_vel \
  --msg-type geometry_msgs/msg/Twist \
  --data '{"linear": {"x": 0.0, "y": 0.0, "z": 0.0}, "angular": {"x": 0.0, "y": 0.0, "z": 0.0}}' \
  --once
```

#### Test Small Velocity (Safe on Platform)
```bash
python3 scripts/ros2_cli.py topics publish /cmd_vel \
  --msg-type geometry_msgs/msg/Twist \
  --data '{"linear": {"x": 0.1, "y": 0.0, "z": 0.0}, "angular": {"x": 0.0, "y": 0.0, "z": 0.0}}' \
  --once
```

#### Check Safety Configuration
```bash
python3 scripts/ros2_cli.py safety show
```

#### Test Safety Blocking (High Velocity)
```bash
# Should be blocked or capped by safety limits
python3 scripts/ros2_cli.py topics publish /cmd_vel \
  --msg-type geometry_msgs/msg/Twist \
  --data '{"linear": {"x": 10.0, "y": 0.0, "z": 0.0}, "angular": {"x": 0.0, "y": 0.0, "z": 0.0}}' \
  --once
```

## Test Categories

### Category 1: Connectivity (Always Safe)
- ✅ `version` - Check ROS availability
- ✅ `context` - Discover ROS graph
- ✅ `topics list` - List available topics
- ✅ `nodes list` - List active nodes
- ✅ `services list` - List available services

### Category 2: Motion Commands (Safe on Platform)
- ✅ Zero velocity publish
- ✅ Small velocity publish (0.1 m/s)
- ⚠️ Large velocity publish (should be blocked by safety)
- ✅ Velocity topic echo

### Category 3: Safety Enforcement
- ✅ Show safety config
- ✅ Velocity limit enforcement
- ✅ Safety policy validation

### Category 4: System Status
- ✅ Lifecycle node listing
- ✅ Parameter node listing
- ℹ️ Battery status (if available)

### Category 5: Performance Benchmarks
- ⏱️ Topics list timing (target: <5s)
- ⏱️ Context discovery timing (target: <10s)
- ⏱️ Safety check timing (target: <1s)

## Safety Guidelines

### Before Testing
1. ✅ Verify robot is secured on platform
2. ✅ Check emergency stop is accessible
3. ✅ Start with zero velocity commands
4. ✅ Verify safety limits are enabled

### During Testing
1. ⚠️ Always start with `--once` flag for publish
2. ⚠️ Use small velocities (0.1 m/s) for motion tests
3. ⚠️ Monitor robot behavior during tests
4. 🛑 Use emergency stop if unexpected behavior

### After Testing
1. ✅ Send zero velocity command
2. ✅ Review test results
3. ✅ Document any issues or unexpected behavior

## Expected Results

### Pass Criteria
- All connectivity tests succeed
- Zero velocity publishes work
- Excessive velocities are blocked or capped
- Safety config loads correctly
- Command timing is reasonable (<10s for discovery)

### Known Limitations
- Nav2 navigation tests require map and localization
- Some lifecycle nodes may not be available
- Battery status may not be present on all robots

## Troubleshooting

### Issue: "rclpy not available"
**Solution**: Source ROS 2 environment
```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
```

### Issue: "No topics found"
**Solution**: Verify robot launch is running
```bash
ros2 node list  # Should show active nodes
ros2 topic list  # Should show active topics
```

### Issue: "Velocity command blocked"
**Solution**: Check safety configuration
```bash
python3 scripts/ros2_cli.py safety show
# Adjust limits if needed:
python3 scripts/ros2_cli.py safety set-velocity --linear 1.0
```

### Issue: "Command timeout"
**Solution**: Increase timeout or check ROS daemon
```bash
# Check daemon status
python3 scripts/ros2_cli.py daemon status

# Or use direct ROS commands
ros2 topic list  # Should complete quickly
```

## Recording Results

### Create Validation Report
```bash
# Run tests and save output
python3 -m pytest tests/test_live_validation.py -v --live 2>&1 | tee validation_results_$(date +%Y%m%d_%H%M%S).log

# Extract timing data
grep "timing:" validation_results_*.log

# Count passed tests
grep -c "PASSED" validation_results_*.log
```

### Document Issues
For any failing tests, record:
1. Command executed
2. Expected result
3. Actual result
4. Error message
5. Robot state at time of test
6. Reproduction steps

## Next Steps

After completing live validation:
1. ✅ Review test results
2. ✅ Document any command failures
3. ✅ Update safety limits if needed
4. ✅ Proceed to optimization testing
5. ✅ Benchmark command timing for hot-path improvements

## Advanced Testing

### Test with Different Safety Modes
```bash
# Set to warn mode (allows but logs)
python3 scripts/ros2_cli.py safety set-velocity --on-violation warn

# Test excessive velocity (should warn)
python3 scripts/ros2_cli.py topics publish /cmd_vel --msg-type geometry_msgs/msg/Twist --data '{"linear": {"x": 2.0, "y": 0.0, "z": 0.0}, "angular": {"x": 0.0, "y": 0.0, "z": 0.0}}' --once

# Set to cap mode (scales down)
python3 scripts/ros2_cli.py safety set-velocity --on-violation cap

# Test excessive velocity (should be capped)
python3 scripts/ros2_cli.py topics publish /cmd_vel --msg-type geometry_msgs/msg/Twist --data '{"linear": {"x": 2.0, "y": 0.0, "z": 0.0}, "angular": {"x": 0.0, "y": 0.0, "z": 0.0}}' --once

# Reset to reject mode (recommended)
python3 scripts/ros2_cli.py safety set-velocity --on-violation reject
```

### Profile Command Performance
```bash
# Use time command for detailed profiling
time python3 scripts/ros2_cli.py topics list
time python3 scripts/ros2_cli.py context
time python3 scripts/ros2_cli.py safety show

# Multiple runs to get average
for i in {1..5}; do time python3 scripts/ros2_cli.py topics list 2>&1; done
```

## Contact & Support

For issues or questions about live validation:
- Check existing test failures in logs
- Review safety configuration
- Verify ROS environment is properly sourced
- Ensure robot hardware is properly connected
