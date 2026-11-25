#!/usr/bin/env python3
"""Tests for robot capability inference system (ros2_capabilities.py)."""

import json
import sys
import os

# Add scripts directory to path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "scripts"))

from ros2_capabilities import (
    infer_capabilities,
    generate_capability_summary,
    validate_capability_inputs,
)


def test_nav2_capabilities():
    """Test that Nav2 presence triggers navigate_to_pose and follow_waypoints."""
    profile = {
        "has_nav2": True,
        "cmd_vel_topic": "/cmd_vel",
        "velocity_topics": [{"topic": "/cmd_vel", "type": "geometry_msgs/msg/Twist"}],
        "odom_frame_ids": {"odom_topic": "/odom"},
        "safety_limits": {"binding": {"linear_x": 0.5, "angular_z": 1.0}},
        "nav2_config": {"max_planning_distance": 50.0},
    }
    
    caps = infer_capabilities(profile)
    
    assert "navigate_to_pose" in caps, "Missing navigate_to_pose capability"
    assert "follow_waypoints" in caps, "Missing follow_waypoints capability"
    
    nav_cap = caps["navigate_to_pose"]
    assert nav_cap["inferred_from"] == ["has_nav2", "nav2_actions.NavigateToPose"]
    assert "x" in nav_cap["inputs"]
    assert "y" in nav_cap["inputs"]
    assert "theta" in nav_cap["inputs"]
    assert nav_cap["maps_to"]["action"] == "/navigate_to_pose"
    
    print("✓ Nav2 capabilities test passed")


def test_basic_movement_capabilities():
    """Test move_distance, rotate_to_heading, arc_movement for all drive types."""
    profile = {
        "has_nav2": False,
        "drive_type": "differential",
        "cmd_vel_topic": "/cmd_vel",
        "velocity_topics": [{"topic": "/cmd_vel", "type": "geometry_msgs/msg/Twist"}],
        "odom_frame_ids": {"odom_topic": "/odom"},
        "safety_limits": {"binding": {"linear_x": 0.3, "angular_z": 0.8}},
    }
    
    caps = infer_capabilities(profile)
    
    assert "move_distance" in caps, "Missing move_distance capability"
    assert "rotate_to_heading" in caps, "Missing rotate_to_heading capability"
    assert "arc_movement" in caps, "Missing arc_movement capability"
    
    # Verify constraints are set from safety limits
    move_cap = caps["move_distance"]
    assert move_cap["constraints"]["max_velocity"] == 0.3
    
    rotate_cap = caps["rotate_to_heading"]
    assert rotate_cap["constraints"]["max_angular_velocity"] == 0.8
    
    print("✓ Basic movement capabilities test passed")


def test_holonomic_capabilities():
    """Test holonomic-specific capabilities (strafe, diagonal, crab_walk)."""
    profile = {
        "has_nav2": False,
        "drive_type": "holonomic_omni",
        "cmd_vel_topic": "/cmd_vel_teleop",
        "velocity_topics": [{"topic": "/cmd_vel_teleop", "type": "geometry_msgs/msg/TwistStamped"}],
        "odom_frame_ids": {"odom_topic": "/odometry/filtered"},
        "safety_limits": {
            "binding": {
                "linear_x": 0.2,
                "linear_y": 0.17,  # Key: linear.y support
                "angular_z": 0.68,
            }
        },
    }
    
    caps = infer_capabilities(profile)
    
    # Holonomic robots should have all basic capabilities + holonomic-specific ones
    assert "move_distance" in caps
    assert "rotate_to_heading" in caps
    assert "arc_movement" in caps
    assert "strafe" in caps, "Missing strafe capability for holonomic robot"
    assert "diagonal_move" in caps, "Missing diagonal_move capability"
    assert "crab_walk" in caps, "Missing crab_walk capability"
    
    # Verify holonomic constraints
    strafe_cap = caps["strafe"]
    assert strafe_cap["constraints"]["max_velocity"] == 0.17
    assert "linear_y" in str(strafe_cap["inferred_from"])
    
    diagonal_cap = caps["diagonal_move"]
    assert diagonal_cap["constraints"]["max_linear_x"] == 0.2
    assert diagonal_cap["constraints"]["max_linear_y"] == 0.17
    
    crab_cap = caps["crab_walk"]
    assert crab_cap["constraints"]["max_linear_x"] == 0.2
    assert crab_cap["constraints"]["max_linear_y"] == 0.17
    assert crab_cap["constraints"]["max_angular_z"] == 0.68
    assert "3dof" in str(crab_cap["inferred_from"]).lower()
    
    print("✓ Holonomic capabilities test passed")


def test_differential_drive_no_strafe():
    """Test that differential drive robots don't get holonomic capabilities."""
    profile = {
        "drive_type": "differential",
        "cmd_vel_topic": "/cmd_vel",
        "velocity_topics": [{"topic": "/cmd_vel", "type": "geometry_msgs/msg/Twist"}],
        "odom_frame_ids": {"odom_topic": "/odom"},
        "safety_limits": {
            "binding": {
                "linear_x": 0.5,
                "angular_z": 1.0,
                # No linear_y for differential drive
            }
        },
    }
    
    caps = infer_capabilities(profile)
    
    # Should have basic movement capabilities
    assert "move_distance" in caps
    assert "rotate_to_heading" in caps
    assert "arc_movement" in caps
    
    # Should NOT have holonomic capabilities
    assert "strafe" not in caps, "Differential drive should not have strafe"
    assert "diagonal_move" not in caps, "Differential drive should not have diagonal_move"
    assert "crab_walk" not in caps, "Differential drive should not have crab_walk"
    
    print("✓ Differential drive (no strafe) test passed")


def test_capability_input_validation():
    """Test input validation against capability schemas."""
    capability = {
        "inputs": {
            "distance": {"type": "float", "unit": "meters"},
            "velocity": {"type": "float", "unit": "m/s", "default": 0.1},
            "timeout": {"type": "float", "unit": "seconds", "default": 30.0},
        }
    }
    
    # Valid inputs
    valid, error = validate_capability_inputs(capability, {"distance": 1.5})
    assert valid, f"Valid inputs rejected: {error}"
    
    # Valid with all inputs
    valid, error = validate_capability_inputs(capability, {
        "distance": 2.0,
        "velocity": 0.2,
        "timeout": 15.0,
    })
    assert valid, f"Valid inputs with defaults rejected: {error}"
    
    # Missing required input (no default)
    valid, error = validate_capability_inputs(capability, {"velocity": 0.1})
    assert not valid, "Should reject missing required input"
    assert "distance" in error.lower()
    
    # Wrong type
    valid, error = validate_capability_inputs(capability, {"distance": "not a number"})
    assert not valid, "Should reject wrong type"
    assert "number" in error.lower() or "float" in error.lower()
    
    print("✓ Capability input validation test passed")


def test_capability_summary_generation():
    """Test human-readable summary generation."""
    profile = {
        "has_nav2": True,
        "drive_type": "holonomic_omni",
        "cmd_vel_topic": "/cmd_vel",
        "velocity_topics": [{"topic": "/cmd_vel", "type": "geometry_msgs/msg/Twist"}],
        "odom_frame_ids": {"odom_topic": "/odom"},
        "safety_limits": {
            "binding": {
                "linear_x": 0.2,
                "linear_y": 0.17,
                "angular_z": 0.68,
            }
        },
    }
    
    caps = infer_capabilities(profile)
    summary = generate_capability_summary(caps)
    
    assert len(summary) > 0, "Summary should not be empty"
    assert any("navigate_to_pose" in line for line in summary)
    assert any("strafe" in line for line in summary)
    
    # Check format
    for line in summary:
        assert line.startswith("• "), "Each line should start with bullet"
        assert ":" in line, "Each line should have description separator"
    
    print("✓ Capability summary generation test passed")


def test_lekiwi_profile_integration():
    """Test capability inference on real lekiwi profile data."""
    # Simulated lekiwi profile summary fields
    lekiwi_summary = {
        "robot_type": "mobile_base",
        "robot_features": ["pantilt"],
        "has_nav2": True,
        "drive_type": "holonomic_omni",
        "cmd_vel_topic": "/cmd_vel_teleop",
        "velocity_topics": [
            {"topic": "/cmd_vel_teleop", "type": "geometry_msgs/msg/TwistStamped"}
        ],
        "odom_frame_ids": {
            "odom_topic": "/odometry/filtered",
            "odom_frame_id": "odom",
            "base_frame_id": "base_footprint",
        },
        "safety_limits": {
            "binding": {
                "linear_x": 0.2,
                "linear_y": 0.17,
                "angular_z": 0.68,
            }
        },
        "nav2_config": {
            "controller_plugins": ["FollowPath"],
            "planner_plugins": ["GridBased"],
        },
    }
    
    caps = infer_capabilities(lekiwi_summary)
    
    # LeKiwi should have ALL capability types
    expected_caps = [
        "navigate_to_pose",  # Nav2
        "follow_waypoints",  # Nav2
        "move_distance",  # Basic movement
        "rotate_to_heading",  # Basic movement
        "arc_movement",  # Basic movement
        "strafe",  # Holonomic
        "diagonal_move",  # Holonomic
        "crab_walk",  # Holonomic (3-DOF)
    ]
    
    for cap_name in expected_caps:
        assert cap_name in caps, f"LeKiwi should have {cap_name} capability"
    
    # Verify specific constraints match LeKiwi hardware
    assert caps["move_distance"]["constraints"]["max_velocity"] == 0.2
    assert caps["strafe"]["constraints"]["max_velocity"] == 0.17
    assert caps["rotate_to_heading"]["constraints"]["max_angular_velocity"] == 0.68
    
    print("✓ LeKiwi profile integration test passed")
    print(f"  Inferred {len(caps)} capabilities: {', '.join(caps.keys())}")


def test_minimal_profile():
    """Test graceful handling of minimal profile (no capabilities)."""
    minimal_profile = {
        "robot_type": "unknown",
        "has_nav2": False,
        # No cmd_vel_topic, no odometry, no drive_type
    }
    
    caps = infer_capabilities(minimal_profile)
    
    # Should return empty dict, not crash
    assert isinstance(caps, dict), "Should return dict even with minimal profile"
    assert len(caps) == 0, "Minimal profile should have no capabilities"
    
    print("✓ Minimal profile test passed")


def run_all_tests():
    """Run all capability inference tests."""
    print("\n=== Running Capability Inference Tests ===\n")
    
    tests = [
        test_nav2_capabilities,
        test_basic_movement_capabilities,
        test_holonomic_capabilities,
        test_differential_drive_no_strafe,
        test_capability_input_validation,
        test_capability_summary_generation,
        test_lekiwi_profile_integration,
        test_minimal_profile,
    ]
    
    failed = []
    for test_func in tests:
        try:
            test_func()
        except AssertionError as e:
            print(f"✗ {test_func.__name__} FAILED: {e}")
            failed.append(test_func.__name__)
        except Exception as e:
            print(f"✗ {test_func.__name__} ERROR: {e}")
            failed.append(test_func.__name__)
    
    print(f"\n=== Test Results ===")
    print(f"Passed: {len(tests) - len(failed)}/{len(tests)}")
    if failed:
        print(f"Failed: {', '.join(failed)}")
        return 1
    else:
        print("All tests passed!")
        return 0


if __name__ == "__main__":
    sys.exit(run_all_tests())
