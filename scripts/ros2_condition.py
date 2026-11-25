#!/usr/bin/env python3
"""Stop-condition tracking for closed-loop moves, free of rclpy so the fast daemon and the CLI share one implementation."""

import math
import threading

from ros2_utils import resolve_field


def quaternion_to_yaw(q):
    """Yaw (rad) from a quaternion (x, y, z, w)."""
    x, y, z, w = q
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def normalize_angle(angle):
    """Wrap an angle to [-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))


class ConditionTracker:
    """Evaluates a stop condition on each odometry (or any monitored) message dict: ``update(msg_dict)``."""

    def __init__(self, field, operator, threshold, stop_event, euclidean=False, rotate=None):
        self.euclidean = euclidean
        self.rotate = rotate
        if isinstance(field, list):
            self.fields = field
            self.field = field[0]
        else:
            self.fields = [field]
            self.field = field
        self.operator = operator
        self.threshold = threshold
        self.stop_event = stop_event

        self.start_value = None
        self.current_value = None
        self.start_values = None
        self.current_values = None
        self.euclidean_distance = None

        # Rotation-specific
        self.start_yaw = None
        self.last_yaw = None
        self.accumulated_rotation = 0.0
        self.rotation_delta = None

        self.start_msg = None
        self.end_msg = None
        self.field_error = None
        self.lock = threading.Lock()

    def update(self, msg_dict):
        with self.lock:
            if self.stop_event.is_set():
                return


            if self.euclidean:
                values = []
                for fp in self.fields:
                    try:
                        v = resolve_field(msg_dict, fp)
                    except (KeyError, IndexError, TypeError) as e:
                        self.field_error = f"Field '{fp}' not found in monitor message: {e}"
                        self.stop_event.set()
                        return

                    if isinstance(v, dict):
                        numeric_children = [
                            (k, float(v[k]))
                            for k in sorted(v.keys())
                            if isinstance(v[k], (int, float))
                        ]
                        if not numeric_children:
                            self.field_error = (
                                f"Field '{fp}' resolves to a dict with no direct numeric "
                                f"sub-fields (keys: {sorted(v.keys())}). "
                                f"Specify a leaf field such as {fp}.x"
                            )
                            self.stop_event.set()
                            return
                        values.extend(val for _, val in numeric_children)
                    else:
                        try:
                            values.append(float(v))
                        except (TypeError, ValueError) as e:
                            self.field_error = f"Field '{fp}' not numeric in monitor message: {e}"
                            self.stop_event.set()
                            return

                if self.start_values is None:
                    self.start_values = values[:]
                    self.start_msg = msg_dict

                self.current_values = values
                dist = math.sqrt(
                    sum((c - s) ** 2 for c, s in zip(values, self.start_values))
                )
                self.euclidean_distance = dist

                if dist >= float(self.threshold):
                    self.end_msg = msg_dict
                    self.stop_event.set()

            elif self.rotate is not None:
                try:
                    quat = None
                    for path in ['pose.pose.orientation', 'orientation']:
                        try:
                            quat = resolve_field(msg_dict, path)
                            if quat and isinstance(quat, dict):
                                break
                        except (KeyError, IndexError, TypeError):
                            continue

                    if not quat or not isinstance(quat, dict):
                        self.field_error = "Could not find quaternion (pose.pose.orientation) in odometry message for rotation monitoring"
                        self.stop_event.set()
                        return

                    try:
                        qx = float(quat.get('x', 0))
                        qy = float(quat.get('y', 0))
                        qz = float(quat.get('z', 0))
                        qw = float(quat.get('w', 1))
                    except (TypeError, ValueError) as e:
                        self.field_error = f"Quaternion values must be numeric: {e}"
                        self.stop_event.set()
                        return

                    current_yaw = quaternion_to_yaw((qx, qy, qz, qw))

                    if self.start_yaw is None:
                        self.start_yaw = current_yaw
                        self.last_yaw = current_yaw
                        self.start_msg = msg_dict
                        return

                    step = normalize_angle(current_yaw - self.last_yaw)
                    self.accumulated_rotation += step
                    self.last_yaw = current_yaw
                    self.rotation_delta = self.accumulated_rotation

                    target = self.rotate
                    if target > 0 and self.accumulated_rotation >= target:
                        self.end_msg = msg_dict
                        self.stop_event.set()
                    elif target < 0 and self.accumulated_rotation <= target:
                        self.end_msg = msg_dict
                        self.stop_event.set()

                except Exception as e:
                    self.field_error = f"Rotation monitoring error: {e}"
                    self.stop_event.set()
                    return

            else:
                try:
                    value = resolve_field(msg_dict, self.field)
                except (KeyError, IndexError, TypeError, ValueError) as e:
                    self.field_error = f"Field '{self.field}' not found in monitor message: {e}"
                    self.stop_event.set()
                    return

                try:
                    numeric_value = float(value)
                except (TypeError, ValueError):
                    numeric_value = None

                if self.start_value is None:
                    self.start_value = value
                    self.start_msg = msg_dict

                self.current_value = value

                condition_met = False
                if self.operator == 'delta':
                    if numeric_value is not None and self.start_value is not None:
                        try:
                            delta = numeric_value - float(self.start_value)
                            thr = float(self.threshold)
                            condition_met = delta >= thr if thr >= 0 else delta <= thr
                        except (TypeError, ValueError):
                            pass
                elif self.operator == 'above':
                    if numeric_value is not None:
                        condition_met = numeric_value > float(self.threshold)
                elif self.operator == 'below':
                    if numeric_value is not None:
                        condition_met = numeric_value < float(self.threshold)
                elif self.operator == 'equals':
                    if numeric_value is not None:
                        try:
                            condition_met = abs(numeric_value - float(self.threshold)) < 1e-9
                        except (TypeError, ValueError):
                            condition_met = str(value) == str(self.threshold)
                    else:
                        condition_met = str(value) == str(self.threshold)

                if condition_met:
                    self.end_msg = msg_dict
                    self.stop_event.set()
