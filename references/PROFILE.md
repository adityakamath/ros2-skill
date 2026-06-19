# Profile JSON Field Reference

Load this file when you need to look up a specific profile field name, understand the full JSON shape, or check robot-type detection rules.

## JSON Shape

```
{
  "summary": {                          ← always load; compact
    ← Fields with no detected value are ABSENT (never null/[]/{}). Missing key = not detected.
    "robot_type": "mobile_base",        ← primary type (see types below)
    "robot_features": ["pantilt"],      ← supplementary features alongside primary type
    "robot_type_evidence": {            ← signals that drove the detected type
      "mobile_base": ["topic:/cmd_vel", "ament:nav2"],
      "pantilt": ["pkg:my_pantilt_driver"]
    },
    "packages": ["my_bringup", "my_nav"],
    "launch_files": ["bringup.launch.py", "nav2_bringup.launch.py"],  ← filenames from workspace; keys into detail
    "urdf_files": ["/path/to/robot.urdf.xacro"],  ← primary packages only; deduplicated
    "velocity_topics": [{"topic": "/cmd_vel", "type": "geometry_msgs/msg/Twist"}],  ← topic+type objects
    "safety_limits": {
      "sources": [                        ← one entry per YAML config that had a velocity limit
        {"file": "teleop_joy.yaml", "path": "...", "linear_x": 0.5, "linear_y": 0.3, "angular_z": 1.0},
        {"file": "nav2_params.yaml", "path": "...", "linear_x": 0.3, "angular_z": 0.8}
        ← axes with no limit are absent from each source entry
      ],
      "binding": {                        ← most restrictive per axis across all sources + URDF
        "linear_x": 0.3, "angular_z": 0.8
        ← linear_y only present for holonomic robots
      }
    },
    "has_lidar": true, "has_camera": true, "has_imu": true, "has_nav2": false,
    "sensor_mounts": [          ← sensor/actuator links from URDF; unresolved xacro variables excluded
      {                         ← sensor_type: camera|depth_camera|lidar|imu|sonar|gps|gripper
        "joint": "camera_joint", "link": "camera_link",
        "sensor_type": "camera",
        "xyz": [0.1, 0.0, 0.5],        ← position relative to parent link
        "rpy": [3.14159, 0.0, 0.0],    ← orientation; roll ≈ π → upside-down
        "image_rotation_deg": 180       ← visual sensors only; capture-image applies this
      }
    ],
    ← Drive / kinematics (present when ros2_control YAML was found)
    "drive_type": "differential",        ← differential|holonomic_omni|mecanum|ackermann|bicycle|tricycle
    "kinematics": {"wheel_radius": 0.05, "wheel_separation": 0.2},
    "controller_update_rate_hz": 100,
    "cmd_vel_topic": "/base_controller/cmd_vel",
    "odom_frame_ids": {"odom_frame_id": "odom", "base_frame_id": "base_link"},
    "active_controllers": ["base_controller", "joint_state_broadcaster"],
    "controller_plugins": ["diff_drive_controller/DiffDriveController", "joint_state_broadcaster/JointStateBroadcaster"],
    ← Hardware
    "hardware_interfaces": [{"name": "...", "plugin": "...", "joints": [...], "command_interfaces": [...], "state_interfaces": [...], "hardware_params": {...}}],
    "mock_hardware_available": false,    ← true when enable_mock_mode=true or a "mock"/"fake" launch arg exists
    "imu_config": {"plugin": "...", "state_interfaces": [...], "hardware_params": {...}, "broadcaster": {"frame_id": "imu_link", "publish_rate": 100}},
    ← Sensors
    "lidar_config": {"topic": "/scan", "frame_id": "lidar_link"},
    "camera_configs": [...],
    "sensor_filter_pipeline": [{"name": "range_filter", "type": "laser_filters/LaserScanRangeFilter", "source_file": "...", "params": {...}}],
    ← Navigation
    "localization_config": {"method": "ekf", "frequency_hz": 50, "fused_sources": {"odom0": "/base_controller/odom"}},
    "nav2_config": {"planner_plugins": [...], "controller_plugins": [...], "behavior_plugins": [...]},
    "maps": [{"name": "map", "type": "occupancy", "resolution": 0.05, "image": "map.pgm", "file": "map.yaml", "path": "..."}],
    ← Teleop / e-stop
    "teleop_config": {"cmd_vel_topic": "/cmd_vel", "joy_topic": "/joy", "scales": {"scale_linear_x": 0.5, "scale_angular_z": 1.0}},
    "estop_config": {"topic": "/e_stop", "service_type": "std_srvs/srv/SetBool", "activate_buttons": [0], "deactivate_buttons": [1]},
    ← TF / launch
    "tf_frames": {"urdf_links": ["base_link", "imu_link", ...], "map_frame": "map", "odom_frame": "odom", "base_frame": "base_link"},
    "launch_configurations": {"config": {"default": "base", "choices": ["base", "pantilt"], "description": "..."}},
    ← Package metadata
    "package_dependencies": {"my_robot_bringup": ["rclpy", "nav2_bringup", ...]}
  },
  "annotations": [              ← user-added free-text notes; ALWAYS read at session start
    {
      "added_at": "2026-05-12T...",
      "note": "Left motor encoder is worn — odometry drifts right."
    }
  ],
  "detail": {                           ← load on demand per launch file; null-stripped like summary
    "bringup.launch.py": {
      "path": "...", "package": "...",
      "launch_args": {
        ← unified: AST defaults + live-resolved values merged; no null values
        "config":       {"default": "base", "choices": ["base", "k2"], "description": "hw config"},
        "use_sim_time": {"default": "false"},
        "serial_port":  {"default": "/dev/ttySERVO", "description": "Serial port for motors"}
      },
      "includes": [                      ← sub-launch files included by this file
        {
          "source": "pkg:nav2_bringup/launch/bringup_launch.py",
          "package": "nav2_bringup", "file": "bringup_launch.py",
          "args_forwarded": {            ← "$ref" = pass-through; literal = hardcoded
            "use_sim_time": "$use_sim_time",
            "map_yaml_file": "$map_yaml"
          }
        }
      ],
      "yaml_files": [...], "urdf_files": [...], "joint_limits": {...}
    },
    ...
  }
}
```

## Robot Types

`robot_type` values: `humanoid` · `legged` · `aerial` · `underwater` · `surface_vessel` · `mobile_manipulator` · `arm` · `mobile_base` · `unknown`

## Detection Rules (applied in strict order)

1. **Humanoid** — specific platform package name (NAO, Atlas, Valkyrie…) **or** ≥ 4 URDF joints with torso/neck/shoulder/elbow patterns. Generic terms like "walking"/"balance" are intentionally excluded (too broad).
2. **Legged** — specific quadruped/hexapod package name (Spot, Go1, ANYmal…) or ≥ 4 URDF joints with FL/FR/RL/RR leg naming.
3. **Aerial / underwater / surface\_vessel** — platform-specific package names or source keywords.
4. **Mobile manipulator** — both mobile-base *and* arm signals present.
5. **Arm** — arm/gripper/MoveIt package or ≥ 3 non-wheel URDF joints.
6. **Mobile base** — velocity topics (`/cmd_vel`) or Nav2 present, or explicit diff-drive/Ackermann packages.

**If detection is wrong:** `profile scan --robot-type <type>` to override. `robot_type_evidence` always shows what matched.

## Velocity Limits

`summary.safety_limits.binding.linear_x` → `--max-vel` ceiling; `.angular_z` → `--max-ang` ceiling (Rule 28). `binding.linear_y` is set for holonomic robots. `sources` lists every config file that contributed a limit. Use `summary.launch_files` to see available launch files; load any one's full detail with `profile show --section <filename>`. A missing `default` key in `launch_args` means the argument is required with no declared default.
