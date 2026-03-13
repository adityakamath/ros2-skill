#!/usr/bin/env python3
"""Shared utilities for ROS 2 Skill CLI modules.

Provides type resolution, message serialization, output helpers, and the
base ROS2CLI node class consumed by every domain module.
"""

import importlib
import json
import os
import re
import sys

try:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import (qos_profile_system_default, QoSProfile,
                           ReliabilityPolicy, DurabilityPolicy, HistoryPolicy)
    from rcl_interfaces.msg import Parameter, ParameterValue
except ImportError as e:
    print(json.dumps({"error": f"Missing ROS 2 dependency: {e}. Source ROS 2 setup.bash or install the missing package."}))
    sys.exit(1)


# ---------------------------------------------------------------------------
# QoS helpers
# ---------------------------------------------------------------------------

_SERVICE_EVENT_QOS = None  # initialised lazily after rclpy import guard above


def _get_service_event_qos():
    """Return (and cache) the QoS profile for service event topics."""
    global _SERVICE_EVENT_QOS
    if _SERVICE_EVENT_QOS is None:
        _SERVICE_EVENT_QOS = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=100,
        )
    return _SERVICE_EVENT_QOS


# ---------------------------------------------------------------------------
# Message type aliases
# ---------------------------------------------------------------------------

MSG_ALIASES = {
    # std_msgs
    'string': 'std_msgs/String',
    'int32': 'std_msgs/Int32',
    'int64': 'std_msgs/Int64',
    'uint8': 'std_msgs/UInt8',
    'float32': 'std_msgs/Float32',
    'float64': 'std_msgs/Float64',
    'bool': 'std_msgs/Bool',
    'header': 'std_msgs/Header',
    'empty': 'std_msgs/Empty',
    'colorrgba': 'std_msgs/ColorRGBA',

    # geometry_msgs
    'twist': 'geometry_msgs/Twist',
    'pose': 'geometry_msgs/Pose',
    'posearray': 'geometry_msgs/PoseArray',
    'point': 'geometry_msgs/Point',
    'pointstamped': 'geometry_msgs/PointStamped',
    'quaternion': 'geometry_msgs/Quaternion',
    'vector3': 'geometry_msgs/Vector3',
    'posestamped': 'geometry_msgs/PoseStamped',
    'twiststamped': 'geometry_msgs/TwistStamped',
    'transform': 'geometry_msgs/Transform',
    'transformstamped': 'geometry_msgs/TransformStamped',
    'wrench': 'geometry_msgs/Wrench',
    'accel': 'geometry_msgs/Accel',
    'polygon': 'geometry_msgs/Polygon',
    'polygonstamped': 'geometry_msgs/PolygonStamped',

    # sensor_msgs
    'laserscan': 'sensor_msgs/LaserScan',
    'image': 'sensor_msgs/Image',
    'compressedimage': 'sensor_msgs/CompressedImage',
    'pointcloud2': 'sensor_msgs/PointCloud2',
    'imu': 'sensor_msgs/Imu',
    'camerainfo': 'sensor_msgs/CameraInfo',
    'jointstate': 'sensor_msgs/JointState',
    'range': 'sensor_msgs/Range',
    'temperature': 'sensor_msgs/Temperature',
    'batterystate': 'sensor_msgs/BatteryState',
    'navsatfix': 'sensor_msgs/NavSatFix',
    'fluidpressure': 'sensor_msgs/FluidPressure',
    'magneticfield': 'sensor_msgs/MagneticField',

    # nav_msgs
    'odometry': 'nav_msgs/Odometry',
    'odom': 'nav_msgs/Odometry',
    'path': 'nav_msgs/Path',
    'occupancygrid': 'nav_msgs/OccupancyGrid',
    'mapmetadata': 'nav_msgs/MapMetaData',
    'gridcells': 'nav_msgs/GridCells',

    # visualization_msgs
    'marker': 'visualization_msgs/Marker',
    'markerarray': 'visualization_msgs/MarkerArray',

    # action_msgs
    'goalstatus': 'action_msgs/GoalStatus',
    'goalstatusarray': 'action_msgs/GoalStatusArray',

    # trajectory_msgs
    'jointtrajectory': 'trajectory_msgs/JointTrajectory',
    'jointtrajectorypoint': 'trajectory_msgs/JointTrajectoryPoint',
}


# ---------------------------------------------------------------------------
# Type resolution helpers
# ---------------------------------------------------------------------------

def get_msg_type(type_str):
    if not type_str:
        return None

    # Check for aliases (case-insensitive)
    if type_str.lower() in MSG_ALIASES:
        type_str = MSG_ALIASES[type_str.lower()]

    # Normalize to pkg, msg_name components
    if '/msg/' in type_str:
        pkg, msg_name = type_str.split('/msg/', 1)
        msg_name = msg_name.strip()
    elif '/srv/' in type_str:
        pkg, msg_name = type_str.split('/srv/', 1)
        msg_name = msg_name.strip()
        try:
            module = importlib.import_module(f"{pkg}.srv")
            return getattr(module, msg_name)
        except Exception:
            return None
    elif '/action/' in type_str:
        pkg, msg_name = type_str.split('/action/', 1)
        msg_name = msg_name.strip()
        try:
            module = importlib.import_module(f"{pkg}.action")
            return getattr(module, msg_name)
        except Exception:
            return None
    elif '/' in type_str:
        pkg, msg_name = type_str.rsplit('/', 1)
    elif '.' in type_str:
        parts = type_str.split('.')
        try:
            module = importlib.import_module('.'.join(parts[:-1]))
            return getattr(module, parts[-1])
        except Exception:
            return None
    else:
        return None

    try:
        module = importlib.import_module(f"{pkg}.msg")
        return getattr(module, msg_name)
    except Exception:
        pass

    return None


def get_action_type(type_str):
    """Import a ROS 2 action type class from a type string."""
    if not type_str:
        return None

    if '/action/' in type_str:
        pkg, action_name = type_str.split('/action/', 1)
        action_name = action_name.strip()
    elif '/' in type_str:
        pkg, action_name = type_str.rsplit('/', 1)
    else:
        return None

    try:
        module = importlib.import_module(f"{pkg}.action")
        return getattr(module, action_name)
    except Exception:
        pass

    return None


def get_srv_type(type_str):
    """Import a ROS 2 service type class from a type string."""
    if not type_str:
        return None

    if '/srv/' in type_str:
        pkg, srv_name = type_str.split('/srv/', 1)
        srv_name = srv_name.strip()
    elif '/' in type_str:
        pkg, srv_name = type_str.rsplit('/', 1)
    else:
        return None

    try:
        module = importlib.import_module(f"{pkg}.srv")
        return getattr(module, srv_name)
    except Exception:
        pass

    return None


def get_msg_error(msg_type):
    """Generate helpful error message when message type cannot be loaded."""
    ros_distro = os.environ.get('ROS_DISTRO', '')
    hint = "ROS 2 message types use /msg/ format (e.g., geometry_msgs/msg/Twist)"
    if ros_distro:
        hint += f". Ensure ROS 2 workspace is built: cd ~/ros2_ws && colcon build && source install/setup.bash"
    else:
        hint += ". Ensure ROS 2 environment is sourced: source /opt/ros/<distro>/setup.bash"
    return {
        "error": f"Unknown message type: {msg_type}",
        "hint": hint,
        "ros_distro": ros_distro if ros_distro else None,
        "troubleshooting": [
            "1. Source ROS 2: source /opt/ros/<distro>/setup.bash",
            "2. If using custom messages, build workspace: cd ~/ros2_ws && colcon build",
            "3. Verify: python3 -c 'from geometry_msgs.msg import Twist; print(Twist)'"
        ]
    }


# ---------------------------------------------------------------------------
# Message serialization
# ---------------------------------------------------------------------------

def msg_to_dict(msg):
    result = {}
    for field in msg.get_fields_and_field_types():
        value = getattr(msg, field, None)
        if value is None:
            continue
        if hasattr(value, 'get_fields_and_field_types'):
            result[field] = msg_to_dict(value)
        elif isinstance(value, (bytes, bytearray)):
            result[field] = list(value)
        elif isinstance(value, (list, tuple)):
            result[field] = [
                msg_to_dict(v) if hasattr(v, 'get_fields_and_field_types')
                else v.tolist() if hasattr(v, 'tolist')
                else v
                for v in value
            ]
        elif hasattr(value, 'tolist'):
            result[field] = value.tolist()
        else:
            result[field] = value
    return result


def dict_to_msg(msg_type, data):
    msg = msg_type()
    field_types = msg.get_fields_and_field_types()
    for key, value in data.items():
        if not hasattr(msg, key):
            continue
        if isinstance(value, dict):
            setattr(msg, key, dict_to_msg(getattr(msg, key).__class__, value))
        elif isinstance(value, list) and value and isinstance(value[0], dict):
            field_type_str = field_types.get(key, '')
            m = re.search(r'sequence<([^,>]+)(?:,\s*\d+\s*)?>', field_type_str) or re.search(r'^(.+?)\[\d*\]$', field_type_str)
            if m:
                elem_class = get_msg_type(m.group(1).strip())
                if elem_class:
                    setattr(msg, key, [dict_to_msg(elem_class, v) for v in value])
                else:
                    setattr(msg, key, value)
            else:
                setattr(msg, key, value)
        else:
            setattr(msg, key, value)
    return msg


def resolve_field(d, path):
    """Resolve a dot-separated field path into a nested dict/list structure."""
    current = d
    for part in path.split('.'):
        if isinstance(current, list):
            current = current[int(part)]
        else:
            current = current[part]
    return current


# ---------------------------------------------------------------------------
# Output helpers
# ---------------------------------------------------------------------------

def _json_default(obj):
    """Fallback JSON encoder for types not handled by msg_to_dict."""
    if hasattr(obj, 'tolist'):
        return obj.tolist()
    return str(obj)


def output(data):
    print(json.dumps(data, indent=2, ensure_ascii=False, default=_json_default))


# ---------------------------------------------------------------------------
# Base ROS 2 node
# ---------------------------------------------------------------------------

class ROS2CLI(Node):
    def __init__(self, node_name='ros2_cli'):
        super().__init__(node_name)

    def get_topic_names(self):
        return self.get_topic_names_and_types()

    def get_service_names(self):
        return self.get_service_names_and_types()


# ---------------------------------------------------------------------------
# Misc helpers
# ---------------------------------------------------------------------------

def get_msg_fields(msg_type_str):
    try:
        msg_class = get_msg_type(msg_type_str)
        if msg_class is None:
            return {}
        msg = msg_class()
        return msg_to_dict(msg)
    except Exception:
        return {}


def parse_node_param(name):
    if ':' in name:
        parts = name.split(':', 1)
        return parts[0], parts[1]
    return name, None


def resolve_output_path(filename_or_path):
    """Resolve an --output argument to an absolute path.

    If *filename_or_path* contains no directory component (plain filename),
    the file is placed in the ``.artifacts/`` directory next to this package,
    creating it when necessary.  Otherwise the value is treated as an explicit
    path and returned as an absolute path (parent directories are not created).
    """
    if os.path.dirname(filename_or_path):
        # Caller supplied an explicit directory — honour it as-is.
        return os.path.abspath(filename_or_path)
    # Plain filename: resolve to .artifacts/ beside this script.
    artifacts_dir = os.path.join(os.path.dirname(__file__), '..', '.artifacts')
    artifacts_dir = os.path.abspath(artifacts_dir)
    os.makedirs(artifacts_dir, exist_ok=True)
    return os.path.join(artifacts_dir, filename_or_path)


# ---------------------------------------------------------------------------
# Local workspace helpers
# ---------------------------------------------------------------------------

def source_local_ws(user_provided_ws=None):
    """Get local ROS 2 workspace path to source before running commands.
    
    System ROS is assumed to be already sourced (via systemd or manually).
    This helper finds the local workspace to source on top of system ROS.
    
    Args:
        user_provided_ws: Optional user-provided workspace path (from ROS2_LOCAL_WS env var)
    
    Search order:
    1. ROS2_LOCAL_WS environment variable
    2. ~/ros2_ws
    3. ~/colcon_ws
    4. ~/dev_ws
    5. ~/workspace
    6. ~/ros2
    
    Returns:
        tuple: (path_to_setup_bash, status)
            - ("/path/to/local_setup.bash", "found") - workspace found and built
            - (None, "not_built") - workspace found but local_setup.bash doesn't exist
            - (None, "not_found") - no workspace found
    """
    import os
    
    # Common workspace patterns to search (in priority order)
    ws_patterns = [
        os.environ.get('ROS2_LOCAL_WS'),  # User override
        '~/ros2_ws',      # Common default
        '~/colcon_ws',    # Common default
        '~/dev_ws',       # Common default
        '~/workspace',    # Generic
        '~/ros2',        # Generic
    ]
    
    def find_setup_files(ws_path):
        """Find valid setup files in workspace, handling various build layouts."""
        if not ws_path or not os.path.exists(ws_path):
            return None
        
        # Resolve symlinks
        ws_path = os.path.realpath(ws_path)
        
        # Try different setup file locations and build types
        possible_setups = [
            ('install/local_setup.bash', 'install/setup.bash'),
            ('install/setup.bash', 'install/setup.bash'),
            ('build/local_setup.bash', 'build/setup.bash'),
            ('build/setup.bash', 'build/setup.bash'),
            ('devel/local_setup.bash', 'devel/setup.bash'),
            ('devel/setup.bash', 'devel/setup.bash'),
            ('install/local_setup.bash', 'install/local_setup.bash'),  # merge-install
            ('setup.bash', 'setup.bash'),  # root-level (merge-install)
        ]
        
        for local_setup, fallback in possible_setups:
            local_path = os.path.join(ws_path, local_setup)
            if os.path.exists(local_path):
                return local_path
        
        return None
    
    best_status = "not_found"
    best_path = None
    
    for ws_pattern in ws_patterns:
        if not ws_pattern:
            continue
            
        ws_path = os.path.expanduser(ws_pattern)
        
        # Skip if path doesn't exist
        if not os.path.exists(ws_path):
            continue
        
        setup_path = find_setup_files(ws_path)
        
        if setup_path:
            return setup_path, "found"
        elif best_status == "not_found":
            # Mark as found but not built - continue searching for better options
            best_path = ws_path
            best_status = "not_built"
    
    # Return best effort: found a workspace but none are built
    if best_path:
        return None, "not_built"
    
    # No workspace found
    return None, "not_found"
