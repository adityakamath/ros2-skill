#!/usr/bin/env python3
"""ROS 2 robot capability inference and registration.

Analyzes robot profile data to infer high-level capabilities (navigate_to_pose,
move_distance, strafe, dock, etc.) from discovered ROS primitives (Nav2 actions,
velocity topics, odometry, sensors, controllers).

Each capability includes:
- inferred_from: profile fields that triggered this capability
- inputs: required parameters with types
- constraints: safety limits and operational boundaries
- maps_to: ROS actions/topics/services used to execute this capability
- result_schema: expected output format

This module is used by profile scan/rescan to automatically populate the
`capabilities` section of robot profiles.
"""

import re
from typing import Dict, List, Any, Optional, Set, Tuple


# ---------------------------------------------------------------------------
# Capability templates
# ---------------------------------------------------------------------------

def _capability_navigate_to_pose(nav2_config: Dict, safety_limits: Dict) -> Dict:
    """Generate navigate_to_pose capability from Nav2 action availability."""
    return {
        "description": "Navigate to a goal pose (x, y, theta) using Nav2 planner and controller",
        "inferred_from": ["has_nav2", "nav2_actions.NavigateToPose"],
        "inputs": {
            "x": {"type": "float", "unit": "meters", "description": "Target X position in map frame"},
            "y": {"type": "float", "unit": "meters", "description": "Target Y position in map frame"},
            "theta": {"type": "float", "unit": "radians", "description": "Target orientation"},
            "frame_id": {"type": "string", "default": "map", "description": "Reference frame for goal"},
        },
        "constraints": {
            "requires_map": True,
            "requires_localization": True,
            "max_planning_distance": nav2_config.get("max_planning_distance", 50.0) if nav2_config else 50.0,
        },
        "maps_to": {
            "action": "/navigate_to_pose",
            "action_type": "nav2_msgs/action/NavigateToPose",
        },
        "result_schema": {
            "status": {"type": "enum", "values": ["succeeded", "canceled", "aborted"]},
            "distance_traveled": {"type": "float", "unit": "meters"},
            "time_elapsed": {"type": "float", "unit": "seconds"},
        },
    }


def _capability_follow_waypoints(nav2_config: Dict) -> Dict:
    """Generate follow_waypoints capability from Nav2 action availability."""
    return {
        "description": "Navigate through a sequence of waypoints using Nav2",
        "inferred_from": ["has_nav2", "nav2_actions.FollowWaypoints"],
        "inputs": {
            "waypoints": {
                "type": "array",
                "item_type": {"x": "float", "y": "float", "theta": "float"},
                "description": "List of waypoint poses to visit in order",
            },
            "frame_id": {"type": "string", "default": "map"},
        },
        "constraints": {
            "requires_map": True,
            "requires_localization": True,
            "min_waypoints": 1,
            "max_waypoints": 50,
        },
        "maps_to": {
            "action": "/follow_waypoints",
            "action_type": "nav2_msgs/action/FollowWaypoints",
        },
        "result_schema": {
            "status": {"type": "enum", "values": ["succeeded", "canceled", "aborted"]},
            "waypoints_completed": {"type": "int"},
            "total_distance": {"type": "float", "unit": "meters"},
        },
    }


def _capability_move_distance(cmd_vel_topic: str, vel_type: str, 
                                safety_limits: Dict, odom_topic: str) -> Dict:
    """Generate move_distance capability from cmd_vel + odometry."""
    max_linear = safety_limits.get("binding", {}).get("linear_x", 0.5)
    
    return {
        "description": "Move a specified distance in a given direction using closed-loop odometry",
        "inferred_from": ["cmd_vel_topic", "odometry", "velocity_topics"],
        "inputs": {
            "distance": {"type": "float", "unit": "meters", "description": "Distance to travel (positive=forward, negative=backward)"},
            "velocity": {"type": "float", "unit": "m/s", "default": max_linear * 0.5, "description": "Desired linear velocity"},
            "timeout": {"type": "float", "unit": "seconds", "default": 30.0},
        },
        "constraints": {
            "max_velocity": max_linear,
            "max_distance_per_command": 10.0,
        },
        "maps_to": {
            "command": "topics publish-until",
            "topic": cmd_vel_topic,
            "msg_type": vel_type,
            "monitor": odom_topic,
            "monitor_field": "pose.pose.position",
            "termination": "euclidean",
        },
        "result_schema": {
            "status": {"type": "enum", "values": ["succeeded", "timeout", "preempted"]},
            "distance_traveled": {"type": "float", "unit": "meters"},
            "time_elapsed": {"type": "float", "unit": "seconds"},
        },
    }


def _capability_rotate_to_heading(cmd_vel_topic: str, vel_type: str,
                                    safety_limits: Dict, odom_topic: str) -> Dict:
    """Generate rotate_to_heading capability from cmd_vel + odometry."""
    max_angular = safety_limits.get("binding", {}).get("angular_z", 1.0)
    
    return {
        "description": "Rotate to a target heading using closed-loop odometry",
        "inferred_from": ["cmd_vel_topic", "odometry", "angular_velocity_support"],
        "inputs": {
            "angle": {"type": "float", "unit": "radians", "description": "Angle to rotate (positive=CCW, negative=CW)"},
            "velocity": {"type": "float", "unit": "rad/s", "default": max_angular * 0.5},
            "degrees": {"type": "boolean", "default": False, "description": "If true, interpret angle as degrees"},
            "timeout": {"type": "float", "unit": "seconds", "default": 30.0},
        },
        "constraints": {
            "max_angular_velocity": max_angular,
            "max_rotation_per_command": 6.28,  # 2π radians (full rotation)
        },
        "maps_to": {
            "command": "topics publish-until",
            "topic": cmd_vel_topic,
            "msg_type": vel_type,
            "monitor": odom_topic,
            "termination": "rotation",
        },
        "result_schema": {
            "status": {"type": "enum", "values": ["succeeded", "timeout", "preempted"]},
            "angle_rotated": {"type": "float", "unit": "radians"},
            "time_elapsed": {"type": "float", "unit": "seconds"},
        },
    }


def _capability_strafe(drive_type: str, cmd_vel_topic: str, vel_type: str,
                       safety_limits: Dict, odom_topic: str) -> Dict:
    """Generate strafe capability for holonomic robots with linear.y support."""
    max_linear_y = safety_limits.get("binding", {}).get("linear_y", 0.17)
    
    return {
        "description": "Strafe laterally (left/right) using holonomic drive capability",
        "inferred_from": ["drive_type=holonomic_*", "linear_y_support"],
        "inputs": {
            "distance": {"type": "float", "unit": "meters", "description": "Distance to strafe (positive=left, negative=right)"},
            "velocity": {"type": "float", "unit": "m/s", "default": max_linear_y * 0.5},
            "timeout": {"type": "float", "unit": "seconds", "default": 30.0},
        },
        "constraints": {
            "max_velocity": max_linear_y,
            "max_distance_per_command": 5.0,
        },
        "maps_to": {
            "command": "topics publish-until",
            "topic": cmd_vel_topic,
            "msg_type": vel_type,
            "monitor": odom_topic,
            "payload_field": "twist.linear.y",  # Strafe uses linear.y
        },
        "result_schema": {
            "status": {"type": "enum", "values": ["succeeded", "timeout", "preempted"]},
            "distance_traveled": {"type": "float", "unit": "meters"},
            "time_elapsed": {"type": "float", "unit": "seconds"},
        },
    }


def _capability_diagonal_move(drive_type: str, cmd_vel_topic: str, vel_type: str,
                               safety_limits: Dict) -> Dict:
    """Generate diagonal movement capability for holonomic robots."""
    max_x = safety_limits.get("binding", {}).get("linear_x", 0.2)
    max_y = safety_limits.get("binding", {}).get("linear_y", 0.17)
    
    return {
        "description": "Move diagonally combining forward/backward with left/right strafe",
        "inferred_from": ["drive_type=holonomic_*", "linear_x_and_y_support"],
        "inputs": {
            "x_velocity": {"type": "float", "unit": "m/s", "description": "Forward/backward velocity"},
            "y_velocity": {"type": "float", "unit": "m/s", "description": "Left/right strafe velocity"},
            "duration": {"type": "float", "unit": "seconds", "description": "Movement duration"},
        },
        "constraints": {
            "max_linear_x": max_x,
            "max_linear_y": max_y,
        },
        "maps_to": {
            "command": "topics publish",
            "topic": cmd_vel_topic,
            "msg_type": vel_type,
        },
        "result_schema": {
            "status": {"type": "enum", "values": ["succeeded", "timeout"]},
            "duration": {"type": "float", "unit": "seconds"},
        },
    }


def _capability_crab_walk(drive_type: str, cmd_vel_topic: str, vel_type: str,
                          safety_limits: Dict) -> Dict:
    """Generate crab-walk capability (simultaneous x, y, and rotation)."""
    return {
        "description": "Move with full 3-DOF control: linear.x, linear.y, and angular.z simultaneously",
        "inferred_from": ["drive_type=holonomic_*", "3dof_control"],
        "inputs": {
            "x_velocity": {"type": "float", "unit": "m/s"},
            "y_velocity": {"type": "float", "unit": "m/s"},
            "angular_velocity": {"type": "float", "unit": "rad/s"},
            "duration": {"type": "float", "unit": "seconds"},
        },
        "constraints": {
            "max_linear_x": safety_limits.get("binding", {}).get("linear_x", 0.2),
            "max_linear_y": safety_limits.get("binding", {}).get("linear_y", 0.17),
            "max_angular_z": safety_limits.get("binding", {}).get("angular_z", 0.68),
        },
        "maps_to": {
            "command": "topics publish",
            "topic": cmd_vel_topic,
            "msg_type": vel_type,
        },
        "result_schema": {
            "status": {"type": "enum", "values": ["succeeded", "timeout"]},
            "duration": {"type": "float", "unit": "seconds"},
        },
    }


def _capability_arc_movement(cmd_vel_topic: str, vel_type: str, safety_limits: Dict) -> Dict:
    """Generate arc movement capability (linear + angular simultaneously)."""
    return {
        "description": "Move in an arc combining linear velocity with rotation",
        "inferred_from": ["cmd_vel_topic", "differential_or_holonomic_drive"],
        "inputs": {
            "linear_velocity": {"type": "float", "unit": "m/s"},
            "angular_velocity": {"type": "float", "unit": "rad/s"},
            "duration": {"type": "float", "unit": "seconds"},
        },
        "constraints": {
            "max_linear": safety_limits.get("binding", {}).get("linear_x", 0.2),
            "max_angular": safety_limits.get("binding", {}).get("angular_z", 0.68),
        },
        "maps_to": {
            "command": "topics publish",
            "topic": cmd_vel_topic,
            "msg_type": vel_type,
        },
        "result_schema": {
            "status": {"type": "enum", "values": ["succeeded", "timeout"]},
            "duration": {"type": "float", "unit": "seconds"},
        },
    }


# ---------------------------------------------------------------------------
# Capability inference engine
# ---------------------------------------------------------------------------

def infer_capabilities(profile_summary: Dict) -> Dict[str, Dict]:
    """
    Infer high-level robot capabilities from profile summary data.
    
    Args:
        profile_summary: The 'summary' section of a robot profile
        
    Returns:
        Dictionary of capabilities keyed by capability name
    """
    capabilities = {}
    
    # Extract key profile fields
    has_nav2 = profile_summary.get("has_nav2", False)
    drive_type = profile_summary.get("drive_type")
    cmd_vel_topic = profile_summary.get("cmd_vel_topic")
    safety_limits = profile_summary.get("safety_limits", {})
    odom_frame_ids = profile_summary.get("odom_frame_ids", {})
    odom_topic = odom_frame_ids.get("odom_topic", "/odom")
    nav2_config = profile_summary.get("nav2_config", {})
    
    # Get velocity topic type
    velocity_topics = profile_summary.get("velocity_topics", [])
    vel_type = velocity_topics[0]["type"] if velocity_topics else "geometry_msgs/msg/Twist"
    
    # ---------------------------------------------------------------------------
    # Nav2-based capabilities
    # ---------------------------------------------------------------------------
    if has_nav2:
        capabilities["navigate_to_pose"] = _capability_navigate_to_pose(nav2_config, safety_limits)
        capabilities["follow_waypoints"] = _capability_follow_waypoints(nav2_config)
    
    # ---------------------------------------------------------------------------
    # Basic movement capabilities (requires cmd_vel + odometry)
    # ---------------------------------------------------------------------------
    if cmd_vel_topic and odom_topic:
        capabilities["move_distance"] = _capability_move_distance(
            cmd_vel_topic, vel_type, safety_limits, odom_topic
        )
        capabilities["rotate_to_heading"] = _capability_rotate_to_heading(
            cmd_vel_topic, vel_type, safety_limits, odom_topic
        )
        capabilities["arc_movement"] = _capability_arc_movement(
            cmd_vel_topic, vel_type, safety_limits
        )
    
    # ---------------------------------------------------------------------------
    # Holonomic-specific capabilities (requires linear.y support)
    # ---------------------------------------------------------------------------
    if drive_type and ("holonomic" in drive_type.lower() or "mecanum" in drive_type.lower()):
        # Check if linear.y is actually available in safety limits
        has_linear_y = "linear_y" in safety_limits.get("binding", {})
        
        if has_linear_y and cmd_vel_topic and odom_topic:
            capabilities["strafe"] = _capability_strafe(
                drive_type, cmd_vel_topic, vel_type, safety_limits, odom_topic
            )
            capabilities["diagonal_move"] = _capability_diagonal_move(
                drive_type, cmd_vel_topic, vel_type, safety_limits
            )
            capabilities["crab_walk"] = _capability_crab_walk(
                drive_type, cmd_vel_topic, vel_type, safety_limits
            )
    
    return capabilities


def generate_capability_summary(capabilities: Dict[str, Dict]) -> List[str]:
    """
    Generate a human-readable summary of inferred capabilities.
    
    Args:
        capabilities: Dictionary of capabilities from infer_capabilities()
        
    Returns:
        List of capability description strings
    """
    summary = []
    for name, cap in capabilities.items():
        desc = cap.get("description", name)
        inferred = ", ".join(cap.get("inferred_from", []))
        summary.append(f"• {name}: {desc} (from: {inferred})")
    return summary


def validate_capability_inputs(capability: Dict, inputs: Dict) -> tuple[bool, Optional[str]]:
    """
    Validate user inputs against capability input schema.
    
    Args:
        capability: Capability dictionary with 'inputs' schema
        inputs: User-provided inputs to validate
        
    Returns:
        Tuple of (valid: bool, error_message: Optional[str])
    """
    required_inputs = capability.get("inputs", {})
    
    for input_name, input_spec in required_inputs.items():
        # Check if required input is missing (no default means required)
        if "default" not in input_spec and input_name not in inputs:
            return False, f"Missing required input: {input_name}"
        
        # Type checking
        if input_name in inputs:
            value = inputs[input_name]
            expected_type = input_spec.get("type")
            
            if expected_type == "float" and not isinstance(value, (int, float)):
                return False, f"Input '{input_name}' must be a number, got {type(value).__name__}"
            elif expected_type == "int" and not isinstance(value, int):
                return False, f"Input '{input_name}' must be an integer, got {type(value).__name__}"
            elif expected_type == "string" and not isinstance(value, str):
                return False, f"Input '{input_name}' must be a string, got {type(value).__name__}"
            elif expected_type == "boolean" and not isinstance(value, bool):
                return False, f"Input '{input_name}' must be a boolean, got {type(value).__name__}"
    
    return True, None


# ---------------------------------------------------------------------------
# Integration with profile system
# ---------------------------------------------------------------------------

def add_capabilities_to_profile(profile: Dict) -> Dict:
    """
    Add capabilities section to an existing profile.
    
    Args:
        profile: Complete robot profile dictionary
        
    Returns:
        Profile with 'capabilities' section added to summary
    """
    summary = profile.get("summary", {})
    capabilities = infer_capabilities(summary)
    
    # Add capabilities to profile summary
    if "summary" not in profile:
        profile["summary"] = {}
    
    profile["summary"]["capabilities"] = capabilities
    
    return profile


if __name__ == "__main__":
    sample_profile = {
        "has_nav2": True,
        "drive_type": "holonomic_omni",
        "cmd_vel_topic": "/cmd_vel_teleop",
        "velocity_topics": [{"topic": "/cmd_vel_teleop", "type": "geometry_msgs/msg/TwistStamped"}],
        "odom_frame_ids": {"odom_topic": "/odometry/filtered"},
        "safety_limits": {"binding": {"linear_x": 0.2, "linear_y": 0.17, "angular_z": 0.68}},
        "nav2_config": {"max_planning_distance": 50.0},
    }

    caps = infer_capabilities(sample_profile)
    print(generate_capability_summary(caps))
