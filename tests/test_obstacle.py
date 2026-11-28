#!/usr/bin/env python3
"""Obstacle stop: geometry on whole sensor messages, the guard's start/stop decisions, and every path that uses it."""

import contextlib
import io
import json
import math
import os
import sys
import tempfile
import threading
import time
import types
import unittest
from pathlib import Path
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))

try:
    import numpy as np
except ImportError:  # the geometry needs numpy; everything else here does not
    np = None

import ros2_fastmotion  # noqa: E402
import ros2_motion_core  # noqa: E402
import ros2_motionhost  # noqa: E402
import ros2_obstacle  # noqa: E402
import ros2_plan  # noqa: E402
import ros2_range_sources  # noqa: E402

RADIUS = 0.17
SCAN_SOURCE = {"type": "scan", "topic": "/scan"}
SENSING = {"base_frame": "base_footprint", "floor_z": 0.0, "radius": RADIUS, "sources": [SCAN_SOURCE],
           "max_height": 0.5, "monitor": None}
MOVE = {"clearance": 0.15, "until": False, "decel": 0.5}


def summary(**extra):
    base = {"cmd_vel_topic": "/cmd_vel_teleop",
            "obstacle_sensing": {"version": 1, "base_frame": "base_footprint", "sources": [SCAN_SOURCE],
                                 "footprint_radius_m": RADIUS, "live_verified": True}}
    base.update(extra)
    return base


def twist(vx=0.0, vy=0.0, wz=0.0):
    return {"linear": {"x": vx, "y": vy, "z": 0.0}, "angular": {"x": 0.0, "y": 0.0, "z": wz}}


def scan(obstacles=(), clear=3.0, beams=360, angle_min=-math.pi, angle_max=math.pi):
    """A LaserScan with every beam at *clear* except the ``(bearing_deg, range_m, width_deg)`` obstacles."""
    inc = (angle_max - angle_min) / beams
    ranges = [clear] * beams
    for bearing, distance, width in obstacles:
        for i in range(beams):
            diff = (math.degrees(angle_min + i * inc) - bearing + 180) % 360 - 180
            if abs(diff) <= width / 2:
                ranges[i] = distance
    return types.SimpleNamespace(ranges=ranges, angle_min=angle_min, angle_increment=inc, range_min=0.02, range_max=12.0,
                                 header=types.SimpleNamespace(frame_id="base_footprint"))


def check(msg, velocity, trigger=0.25, rotation=None, translation=None, sensing=SENSING):
    return ros2_obstacle.evaluate("scan", ros2_obstacle.scan_points(msg), np.eye(3) if rotation is None else rotation,
                                  np.zeros(3) if translation is None else translation, velocity, trigger, sensing)


@unittest.skipIf(np is None, "numpy not installed")
class TestGeometry(unittest.TestCase):
    def distance(self, point, velocity):
        return float(ros2_obstacle.travel_distance(np.array([point], dtype=float), *velocity, RADIUS)[0][0])

    def test_straight_ahead_distance_is_to_the_footprint_not_the_centre(self):
        self.assertAlmostEqual(self.distance((1.0, 0.0), (0.2, 0, 0)), 1.0 - RADIUS, places=6)

    def test_anything_within_the_footprint_width_is_on_the_path(self):
        self.assertTrue(math.isfinite(self.distance((1.0, RADIUS - 0.01), (0.2, 0, 0))))
        self.assertEqual(self.distance((1.0, RADIUS + 0.01), (0.2, 0, 0)), math.inf)

    def test_behind_is_ignored_forward_and_seen_in_reverse(self):
        self.assertEqual(self.distance((-0.5, 0.0), (0.2, 0, 0)), math.inf)
        self.assertAlmostEqual(self.distance((-0.5, 0.0), (-0.2, 0, 0)), 0.5 - RADIUS, places=6)

    def test_strafe_and_diagonal(self):
        self.assertAlmostEqual(self.distance((0.0, 0.6), (0, 0.2, 0)), 0.6 - RADIUS, places=6)
        self.assertAlmostEqual(self.distance((0.5, 0.5), (0.1, 0.1, 0)), math.hypot(0.5, 0.5) - RADIUS, places=6)

    def test_arc_follows_the_curve(self):
        velocity = (0.2, 0.0, 0.4)   # left turn, radius 0.5 m, centre (0, 0.5)
        on_arc = (0.5, 0.5)          # a quarter circle ahead
        arc = self.distance(on_arc, velocity)
        self.assertTrue(0.4 < arc < 0.5 * math.pi / 2, arc)
        self.assertEqual(self.distance((1.2, 0.0), velocity), math.inf)   # straight ahead, but the robot turns away

    def test_point_inside_the_footprint_blocks_only_motion_towards_it(self):
        self.assertEqual(self.distance((0.1, 0.0), (0.2, 0, 0)), 0.0)
        self.assertEqual(self.distance((0.1, 0.0), (-0.2, 0, 0)), math.inf)

    def test_trigger_grows_with_speed(self):
        slow, fast = (ros2_obstacle.trigger_distance(v, 0.15, 0.5) for v in (0.05, 0.4))
        self.assertTrue(0.15 < slow < fast)


@unittest.skipIf(np is None, "numpy not installed")
class TestScan(unittest.TestCase):
    def test_clear_scan_covers_and_finds_nothing(self):
        result = check(scan(), (0.2, 0, 0))
        self.assertTrue(result["covers"])
        self.assertIsNone(result["obstacle"])

    def test_whole_scan_is_used_not_only_the_beam_ahead(self):
        # 40 degrees off the heading: outside a narrow forward sector, but the front left of the robot would hit it
        result = check(scan([(40, 0.28, 4)]), (0.2, 0, 0))
        self.assertIsNotNone(result["obstacle"], result)
        self.assertAlmostEqual(result["obstacle"]["bearing_deg"], 40, delta=3)

    def test_something_beside_the_robot_does_not_stop_a_forward_move(self):
        self.assertIsNone(check(scan([(90, 0.25, 10)]), (0.2, 0, 0))["obstacle"])

    def test_same_scan_stops_a_strafe_towards_it(self):
        self.assertIsNotNone(check(scan([(90, 0.3, 10)]), (0, 0.2, 0))["obstacle"])

    def test_rear_obstacle_stops_only_reverse(self):
        msg = scan([(180, 0.3, 10)])
        self.assertIsNone(check(msg, (0.2, 0, 0))["obstacle"])
        self.assertIsNotNone(check(msg, (-0.2, 0, 0))["obstacle"])

    def test_sensor_mounted_backwards_is_transformed(self):
        yaw_pi = ros2_obstacle.rotation_matrix(0, 0, 1, 0)
        msg = scan([(180, 0.3, 10)])   # behind the sensor = ahead of the robot
        self.assertIsNotNone(check(msg, (0.2, 0, 0), rotation=yaw_pi)["obstacle"])

    def test_a_scan_that_does_not_look_that_way_does_not_count(self):
        front_only = scan(angle_min=-1.0, angle_max=1.0, beams=100)
        self.assertTrue(check(front_only, (0.2, 0, 0))["covers"])
        self.assertFalse(check(front_only, (-0.2, 0, 0))["covers"])

    def test_all_invalid_beams_do_not_count_as_clear(self):
        msg = scan()
        msg.ranges = [float("nan")] * len(msg.ranges)
        self.assertFalse(check(msg, (0.2, 0, 0))["covers"])

    def test_open_space_reported_as_infinity_is_clear(self):
        msg = scan()
        msg.ranges = [float("inf")] * len(msg.ranges)
        result = check(msg, (0.2, 0, 0))
        self.assertTrue(result["covers"])
        self.assertIsNone(result["obstacle"])

    def test_a_blind_patch_straight_ahead_is_not_clear(self):
        for value in (float("nan"), 0.0):
            msg = scan()
            msg.ranges[165:196] = [value] * 31   # 30 degrees with no information in the way of the robot
            self.assertFalse(check(msg, (0.2, 0, 0))["covers"], value)
            self.assertTrue(check(msg, (-0.2, 0, 0))["covers"])

    def test_a_narrow_dead_spot_is_tolerated(self):
        msg = scan()
        msg.ranges[176:185] = [float("nan")] * 9
        self.assertTrue(check(msg, (0.2, 0, 0))["covers"])

    def test_returns_from_inside_the_robot_are_ignored(self):
        self.assertIsNone(check(scan([(0, 0.08, 10)]), (0.2, 0, 0))["obstacle"])

    def test_too_close_to_measure_is_an_obstacle(self):
        msg = scan()
        msg.ranges[178:183] = [float("-inf")] * 5
        self.assertIsNotNone(check(msg, (0.2, 0, 0))["obstacle"])

    def test_one_stray_beam_is_ignored(self):
        msg = scan()
        msg.ranges[180] = 0.25
        self.assertIsNone(check(msg, (0.2, 0, 0))["obstacle"])


def cloud(points, step=16):
    data = bytearray()
    for p in points:
        data += np.array(p, dtype="<f4").tobytes() + b"\0" * (step - 12)
    fields = [types.SimpleNamespace(name=n, offset=i * 4, datatype=7) for i, n in enumerate("xyz")]
    return types.SimpleNamespace(fields=fields, width=len(points), height=1, point_step=step, row_step=step * len(points),
                                 data=bytes(data), is_bigendian=False)


def wall(x, z, n=40, half=0.3):
    return [(x, -half + 2 * half * i / (n - 1), z) for i in range(n)]


@unittest.skipIf(np is None, "numpy not installed")
class TestCloudAndDepth(unittest.TestCase):
    def check_cloud(self, points, velocity=(0.2, 0, 0)):
        return ros2_obstacle.evaluate("cloud", ros2_obstacle.cloud_points(cloud(points)), np.eye(3), np.zeros(3), velocity,
                                      0.25, SENSING)

    def test_low_obstacle_a_lidar_would_miss(self):
        self.assertIsNotNone(self.check_cloud(wall(0.35, 0.06))["obstacle"])

    def test_floor_and_overhead_points_are_not_obstacles(self):
        result = self.check_cloud(wall(0.35, 0.0) + wall(0.35, 1.2))
        self.assertTrue(result["covers"])
        self.assertIsNone(result["obstacle"])

    def test_the_robot_seeing_itself_is_ignored(self):
        self.assertIsNone(self.check_cloud([(0.05, 0.0, 0.1)] * 200)["obstacle"])

    def test_a_cloud_covers_only_with_real_returns_around_the_way_ahead(self):
        self.assertFalse(self.check_cloud(wall(1.0, 2.0))["covers"])                  # ceiling only
        self.assertFalse(self.check_cloud([(0.0, 0.0, 0.0)] * 64)["covers"])          # invalid zones
        self.assertFalse(self.check_cloud([(0.05, 0.0, 0.1)] * 200)["covers"])        # the robot itself
        self.assertFalse(self.check_cloud([(1.0, 0.0, 0.0)])["covers"])               # one stray return
        self.assertFalse(self.check_cloud([(0.8, 0.2 + i / 100, 0.1) for i in range(20)])["covers"])   # one side only

    def test_non_finite_points_are_dropped_and_padding_respected(self):
        points, total = ros2_obstacle.cloud_points(cloud([(float("nan"),) * 3, (1.0, 2.0, 3.0)], step=32))
        self.assertEqual(total, 2)
        self.assertEqual(points.tolist(), [[1.0, 2.0, 3.0]])

    def depth_image(self, millimetres, width=640, height=400):
        image = np.full((height, width), millimetres, dtype="<u2")
        return types.SimpleNamespace(encoding="16UC1", width=width, height=height, step=width * 2, data=image.tobytes(),
                                     is_bigendian=False)

    def check_depth(self, millimetres, velocity=(0.2, 0, 0)):
        k = [400.0, 0, 320.0, 0, 400.0, 200.0, 0, 0, 1]
        image = self.depth_image(millimetres) if isinstance(millimetres, int) else millimetres
        parsed = ros2_obstacle.depth_points(image, k)
        optical = np.array([[0, 0, 1], [-1, 0, 0], [0, -1, 0]], dtype=float)   # optical frame -> base
        return ros2_obstacle.evaluate("depth", parsed, optical, np.array([0.09, 0.0, 0.15]), velocity, 0.25, SENSING)

    def test_depth_wall_close_ahead(self):
        self.assertIsNotNone(self.check_depth(220)["obstacle"])

    def test_depth_wall_far_ahead(self):
        result = self.check_depth(2000)
        self.assertTrue(result["covers"])
        self.assertIsNone(result["obstacle"])

    def test_depth_covers_only_what_the_lens_sees(self):
        self.assertFalse(self.check_depth(2000, velocity=(-0.2, 0, 0))["covers"])
        self.assertFalse(self.check_depth(2000, velocity=(0.1, 0.17, 0))["covers"])   # 60 degrees off the lens axis

    def test_a_blinded_depth_camera_is_not_a_clear_path(self):
        self.assertFalse(self.check_depth(0)["covers"])   # covered lens, or everything inside the minimum range
        image = self.depth_image(2000)
        pixels = np.frombuffer(image.data, dtype="<u2").reshape(400, 640).copy()
        pixels[:, 160:480] = 0                            # blind in the middle only
        image.data = pixels.tobytes()
        self.assertFalse(self.check_depth(image)["covers"])


@unittest.skipIf(np is None, "numpy not installed")
class TestGuardOn3DSensors(unittest.TestCase):
    """Messages go through the guard itself, from subscription callback to verdict."""

    OPTICAL = ([[0, 0, 1], [-1, 0, 0], [0, -1, 0]], [0.09, 0.0, 0.15])   # camera pose: optical frame -> base

    def arm(self, source, message, pose=None, info=None):
        class Guard(FakeGuard):
            def _transform(self, frame):
                if pose is None:
                    return np.eye(3), np.zeros(3)
                return np.array(pose[0], dtype=float), np.array(pose[1], dtype=float)

        guard = Guard(FakeNode(), dict(SENSING, sources=[source]), None, time.monotonic)
        if info is not None:
            guard._on_info(guard.slots[0], info)
        message.header = types.SimpleNamespace(frame_id="sensor")
        guard._on_data(guard.slots[0], message)
        return guard, guard.arm((0.2, 0, 0), MOVE, timeout=0.1)

    def depth(self, millimetres):
        image = np.full((400, 640), millimetres, dtype="<u2")
        message = types.SimpleNamespace(encoding="16UC1", width=640, height=400, step=1280, data=image.tobytes(),
                                        is_bigendian=False)
        info = types.SimpleNamespace(k=[400.0, 0, 320.0, 0, 400.0, 200.0, 0, 0, 1], width=640, height=400)
        source = {"type": "depth", "topic": "/depth", "info_topic": "/camera_info"}
        return self.arm(source, message, self.OPTICAL, info)

    def test_depth_clear_ahead(self):
        guard, trip = self.depth(2000)
        self.assertIsNone(trip, trip)
        self.assertIsNone(guard.poll())
        self.assertEqual(guard.report()["sources_used"], ["/depth"])

    def test_depth_obstacle_ahead(self):
        _, trip = self.depth(220)
        self.assertEqual(trip["stopped_by"], "obstacle", trip)
        self.assertEqual(trip["obstacle"]["source"], "/depth")

    def test_cloud_obstacle_ahead(self):
        _, trip = self.arm({"type": "pointcloud", "topic": "/points"}, cloud(wall(0.35, 0.1)))
        self.assertEqual(trip["stopped_by"], "obstacle", trip)


@unittest.skipIf(np is None, "numpy not installed")
class TestBlindSensors(unittest.TestCase):
    def test_a_covered_lidar_is_not_a_clear_path(self):
        self.assertFalse(check(scan(clear=0.05), (0.2, 0, 0))["covers"])
        self.assertFalse(check(scan([(0, 0.10, 40)]), (0.2, 0, 0))["covers"])
        self.assertTrue(check(scan([(180, 0.10, 40)]), (0.2, 0, 0))["covers"])   # robot parts behind do not matter ahead

    def depth(self, image):
        k = [450.0, 0, 320.0, 0, 450.0, 200.0, 0, 0, 1]
        message = types.SimpleNamespace(encoding="16UC1", width=640, height=400, step=1280,
                                        data=image.astype("<u2").tobytes(), is_bigendian=False)
        optical = np.array([[0, 0, 1], [-1, 0, 0], [0, -1, 0]], dtype=float)
        return ros2_obstacle.evaluate("depth", ros2_obstacle.depth_points(message, k), optical,
                                      np.array([0.1, 0.0, 0.2]), (0.2, 0, 0), 0.25, SENSING)

    def test_depth_blind_in_the_middle(self):
        image = np.full((400, 640), 3000)
        half = int(450 * math.tan(math.radians(6)))
        image[:, 320 - half:320 + half] = 0
        self.assertFalse(self.depth(image)["covers"])

    def test_floor_alone_says_nothing_about_what_stands_ahead(self):
        image = np.zeros((400, 640))
        for row in range(300, 400):
            image[row] = int(1000 * 0.2 * 450 / (row - 200))
        self.assertFalse(self.depth(image)["covers"])

    def test_open_sky_above_the_horizon_is_fine(self):
        image = np.full((400, 640), 3000)
        image[:190] = 0
        self.assertTrue(self.depth(image)["covers"])


class TestResolve(unittest.TestCase):
    def resolve(self, s=None, msg=None, topic="/cmd_vel_teleop", **kw):
        return ros2_obstacle.resolve(summary() if s is None else s, twist(0.2) if msg is None else msg, topic, **kw)

    def test_linear_base_motion_is_guarded_by_default(self):
        cfg = self.resolve()
        self.assertTrue(cfg["active"])
        self.assertEqual(cfg["move"], {"clearance": 0.15, "until": False, "decel": 0.5})
        self.assertFalse(cfg["unverified"])

    def test_without_a_profile_any_linear_command_is_held_to_be_the_base(self):
        cfg = ros2_obstacle.resolve({}, twist(0.2), "/base_controller/reference")
        self.assertTrue(cfg["blocked"], cfg)

    def test_stamped_twist(self):
        self.assertTrue(self.resolve(msg={"header": {}, "twist": twist(0.0, 0.1)})["active"])

    def test_turning_on_the_spot_and_other_topics_are_not_guarded(self):
        self.assertFalse(self.resolve(msg=twist(wz=0.5))["active"])
        self.assertFalse(self.resolve(topic="/servo_node/delta_twist_cmds")["active"])

    def test_no_known_sensor_blocks_the_move(self):
        s = summary()
        s["obstacle_sensing"]["sources"] = []
        for result in (self.resolve(s), self.resolve(s, until=True), self.resolve({}, topic="/cmd_vel"),
                       self.resolve({"cmd_vel_topic": "/cmd_vel_teleop"})):
            self.assertTrue(result["blocked"] and result["obstacle_stop_blocked"], result)

    def test_a_robot_declared_without_sensors_may_move(self):
        s = summary()
        s["obstacle_sensing"]["sources"] = []
        declared = {"no_range_sensors": True}
        self.assertFalse(self.resolve(s, overrides=declared)["active"])
        self.assertTrue(self.resolve(s, overrides=declared, until=True)["blocked"])
        self.assertTrue(self.resolve(overrides=declared)["active"])   # a listed sensor is still used

    def test_a_source_added_by_hand_is_used(self):
        s = summary()
        s["obstacle_sensing"]["sources"] = []
        added = ros2_range_sources.source_from_spec("depth:/oak/stereo/image_raw")
        self.assertEqual(added["info_topic"], "/oak/stereo/camera_info")
        result = self.resolve(s, overrides={"add_sources": [added]})
        self.assertEqual(result["sensing"]["sources"], [added])
        self.assertIsNone(ros2_range_sources.source_from_spec("/scan"))

    def test_nonsense_numbers_are_refused(self):
        for kwargs in ({"clearance": float("nan")}, {"clearance": -1.0}, {"radius": float("nan")}, {"radius": 0.0}):
            self.assertTrue(self.resolve(**kwargs).get("blocked"), kwargs)

    def test_an_empty_twist_wrapper_does_not_hide_the_velocity(self):
        self.assertEqual(ros2_obstacle.twist_of({**twist(0.2), "twist": {}}), (0.2, 0.0, 0.0))

    def test_unknown_robot_size_blocks_the_move(self):
        s = summary()
        del s["obstacle_sensing"]["footprint_radius_m"]
        result = self.resolve(s)
        self.assertTrue(result["blocked"])
        self.assertIn("set-obstacle", result["hint"])
        self.assertEqual(self.resolve(s, radius=0.2)["sensing"]["radius"], 0.2)
        self.assertEqual(self.resolve(s, overrides={"footprint_radius_m": 0.19})["sensing"]["radius"], 0.19)

    def test_disabled_only_without_until(self):
        self.assertFalse(self.resolve(disabled=True)["active"])
        self.assertTrue(self.resolve(disabled=True, until=True)["blocked"])

    def test_ignoring_every_source_blocks(self):
        self.assertIn("ignored", self.resolve(overrides={"ignore_sources": ["/scan"]})["error"])

    def test_prepare_guard_reads_nothing_for_a_rotation(self):
        args = types.SimpleNamespace()
        with patch("ros2_profile_context.guard_inputs", side_effect=AssertionError("must not be called")):
            self.assertEqual(ros2_motion_core.prepare_guard(args, twist(wz=1.0), "/cmd_vel"), (None, None))

    def test_prepare_guard_never_drives_blind_after_a_failure(self):
        with patch("ros2_profile_context.guard_inputs", side_effect=OSError("disk")):
            err, guard = ros2_motion_core.prepare_guard(types.SimpleNamespace(), twist(0.2), "/cmd_vel")
        self.assertTrue(err["blocked"])
        self.assertIsNone(guard)


class Clock:
    def __init__(self):
        self.now = 100.0

    def __call__(self):
        return self.now


class FakeNode:
    def __init__(self, publishers=1):
        self.publishers = publishers

    def count_publishers(self, topic):
        return self.publishers


class FakeGuard(ros2_obstacle.ObstacleGuard):
    """The real decision logic without ROS subscriptions or TF."""

    def connect(self):
        for slot in self.slots:
            slot.sub = object()

    def _transform(self, frame):
        return np.eye(3), np.zeros(3)


@unittest.skipIf(np is None, "numpy not installed")
class TestGuard(unittest.TestCase):
    def guard(self, sensing=SENSING, publishers=1, clock=None):
        return FakeGuard(FakeNode(publishers), sensing, None, clock or time.monotonic)

    def feed_later(self, guard, msg, delay=0.05):
        timer = threading.Timer(delay, lambda: guard._on_data(guard.slots[0], msg))
        timer.start()
        self.addCleanup(timer.cancel)

    def test_arms_when_the_way_is_clear(self):
        guard = self.guard()
        self.feed_later(guard, scan())
        self.assertIsNone(guard.arm((0.2, 0, 0), MOVE))
        self.assertIsNone(guard.poll())

    def test_refuses_to_start_towards_an_obstacle(self):
        guard = self.guard()
        guard._on_data(guard.slots[0], scan([(0, 0.3, 20)]))
        trip = guard.arm((0.2, 0, 0), MOVE)
        self.assertEqual(trip["stopped_by"], "obstacle")
        self.assertEqual(trip["obstacle"]["source"], "/scan")

    def test_refuses_to_start_without_a_publisher(self):
        trip = self.guard(publishers=0).arm((0.2, 0, 0), MOVE, timeout=0.1)
        self.assertEqual(trip["stopped_by"], "no_sensor")
        self.assertEqual(trip["obstacle"]["sources"], {"/scan": "no publisher"})

    def test_a_new_message_sets_the_stop_event_at_once(self):
        guard, stop = self.guard(), threading.Event()
        guard._on_data(guard.slots[0], scan())
        self.assertIsNone(guard.arm((0.2, 0, 0), MOVE, stop))
        guard._on_data(guard.slots[0], scan([(0, 0.3, 20)]))
        self.assertTrue(stop.is_set())
        self.assertEqual(guard.tripped["stopped_by"], "obstacle")

    def test_stops_when_the_sensor_goes_quiet(self):
        clock = Clock()
        guard = self.guard(clock=clock)
        guard._on_data(guard.slots[0], scan())
        self.assertIsNone(guard.arm((0.2, 0, 0), MOVE))
        clock.now += 2.0
        self.assertEqual(guard.poll()["stopped_by"], "sensor_lost")

    def test_losing_one_of_two_covering_sensors_stops(self):
        clock = Clock()
        sensing = dict(SENSING, sources=[SCAN_SOURCE, {"type": "scan", "topic": "/low_scan"}])
        guard = self.guard(sensing, clock=clock)
        for slot in guard.slots:
            guard._on_data(slot, scan())
        self.assertIsNone(guard.arm((0.2, 0, 0), MOVE))
        for _ in range(15):
            clock.now += 0.1
            guard._on_data(guard.slots[0], scan())   # the first stays alive, the second says nothing more
            if guard.poll():
                break
        self.assertEqual(guard.tripped["stopped_by"], "sensor_lost")
        self.assertEqual(list(guard.tripped["obstacle"]["sources"]), ["/low_scan"])
        self.assertIn("/low_scan", guard.report()["sources_unavailable"])

    def two_at_5fps(self):
        clock = Clock()
        sensing = dict(SENSING, sources=[SCAN_SOURCE, {"type": "scan", "topic": "/low_scan"}])
        guard = self.guard(sensing, clock=clock)
        for _ in range(5):
            clock.now += 0.2
            for slot in guard.slots:
                guard._on_data(slot, scan())
        self.assertIsNone(guard.arm((0.2, 0, 0), MOVE))
        return guard, clock

    def test_two_dropped_frames_are_forgiven_while_another_sensor_watches(self):
        guard, clock = self.two_at_5fps()
        for _ in range(7):
            clock.now += 0.1
            guard._on_data(guard.slots[0], scan())
            self.assertIsNone(guard.poll())

    def test_the_only_watching_sensor_gets_no_extra_grace(self):
        clock = Clock()
        guard = self.guard(clock=clock)
        for _ in range(5):
            clock.now += 0.2
            guard._on_data(guard.slots[0], scan())
        self.assertIsNone(guard.arm((0.2, 0, 0), MOVE))
        clock.now += 0.7
        self.assertEqual(guard.poll()["stopped_by"], "sensor_lost")

    def test_something_seen_ahead_is_reached_by_driving_even_without_a_newer_view(self):
        guard, clock = self.two_at_5fps()
        clock.now += 0.1
        guard._on_data(guard.slots[0], scan())
        guard._on_data(guard.slots[1], scan([(0, 0.55, 20)]))   # ahead, not yet close enough to stop
        self.assertIsNone(guard.poll())
        blind = scan(clear=float("nan"))
        for _ in range(9):
            clock.now += 0.1
            guard._on_data(guard.slots[0], scan())
            guard._on_data(guard.slots[1], blind)               # now too close for it to measure
            if guard.poll():
                break
        self.assertEqual(guard.tripped["stopped_by"], "obstacle", guard.tripped)
        self.assertTrue(guard.tripped["obstacle"]["estimated"])
        self.assertIn("estimated", ros2_obstacle.outcome(guard.tripped, MOVE)["error"])

    def test_a_late_discovery_is_waited_for_after_another_sensor_covers(self):
        sensing = dict(SENSING, sources=[SCAN_SOURCE, {"type": "scan", "topic": "/low_scan"}])
        guard = self.guard(sensing)
        guard.node.count_publishers = lambda topic: 0 if topic == "/low_scan" else 1
        guard.slots[1].since = time.monotonic()
        alive = threading.Event()
        self.addCleanup(alive.set)

        def feed():
            while not alive.wait(0.05):
                guard._on_data(guard.slots[0], scan())

        threading.Thread(target=feed, daemon=True).start()
        timer = threading.Timer(0.8, lambda: guard._on_data(guard.slots[1], scan([(0, 0.3, 20)])))
        timer.start()
        self.addCleanup(timer.cancel)
        trip = guard.arm((0.2, 0, 0), MOVE, timeout=3.0)
        self.assertEqual(trip["obstacle"]["source"], "/low_scan", trip)

    def test_waits_for_a_sensor_not_yet_discovered(self):
        sensing = dict(SENSING, sources=[SCAN_SOURCE, {"type": "scan", "topic": "/low_scan"}])
        guard = self.guard(sensing)
        guard.node.count_publishers = lambda topic: 0 if topic == "/low_scan" else 1
        guard.slots[1].since = time.monotonic()   # subscribed a moment ago
        guard._on_data(guard.slots[0], scan())
        timer = threading.Timer(0.1, lambda: guard._on_data(guard.slots[1], scan([(0, 0.3, 20)])))
        timer.start()
        self.addCleanup(timer.cancel)
        self.assertEqual(guard.arm((0.2, 0, 0), MOVE)["obstacle"]["source"], "/low_scan")

    def test_does_not_wait_again_for_a_sensor_silent_for_long(self):
        sensing = dict(SENSING, sources=[SCAN_SOURCE, {"type": "scan", "topic": "/dead"}])
        guard = self.guard(sensing)
        guard.slots[1].since = time.monotonic() - 60
        guard._on_data(guard.slots[0], scan())
        waited = []
        started = time.monotonic()
        self.assertIsNone(guard.arm((0.2, 0, 0), MOVE, on_wait=lambda: waited.append(1)))
        self.assertLess(time.monotonic() - started, 0.2)
        self.assertEqual(waited, [])

    def test_a_caller_already_moving_is_told_to_hold_while_it_waits(self):
        guard, waited = self.guard(), []
        self.feed_later(guard, scan(), delay=0.1)
        self.assertIsNone(guard.arm((0.2, 0, 0), MOVE, on_wait=lambda: waited.append(1)))
        self.assertEqual(waited, [1])

    def test_a_guard_shut_down_mid_move_says_stop(self):
        guard = self.guard()
        guard._on_data(guard.slots[0], scan())
        self.assertIsNone(guard.arm((0.2, 0, 0), MOVE))
        guard.disarm()
        self.assertEqual(guard.poll()["stopped_by"], "sensor_lost")

    def test_a_cloud_in_the_map_frame_is_not_a_sensor(self):
        guard = self.guard(dict(SENSING, sources=[{"type": "pointcloud", "topic": "/points"}]))
        guard.slots[0].raw = False
        msg = cloud(wall(1.0, 0.1))
        msg.header = types.SimpleNamespace(frame_id="map")
        guard._on_data(guard.slots[0], msg)
        self.assertEqual(guard.arm((0.2, 0, 0), MOVE, timeout=0.1)["stopped_by"], "no_sensor")

    def test_a_paused_sequence_step_is_not_watched(self):
        command = ros2_obstacle.CommandGuard({"active": True, "sensing": SENSING, "move": MOVE})
        guard = self.guard()
        command.host = types.SimpleNamespace(guard=guard)
        guard._on_data(guard.slots[0], scan())
        self.assertIsNone(command.arm(twist(0.2)))
        self.assertIsNone(command.arm(twist(wz=0.5), cfg=None))   # a turn on the spot after a straight step
        self.assertIsNone(command.poll())
        self.assertIsNone(guard.motion)

    def test_collision_monitor_stop_state(self):
        guard = self.guard(dict(SENSING, monitor={"state_topic": "/collision_monitor_state"}))
        guard._on_data(guard.slots[0], scan())
        self.assertIsNone(guard.arm((0.2, 0, 0), MOVE))
        guard._on_state(types.SimpleNamespace(action_type=1, polygon_name="Front"))
        self.assertEqual(guard.poll(), {"stopped_by": "collision_monitor", "obstacle": {"action": "stop", "polygon": "Front"}})

    def test_collision_monitor_cutting_the_speed(self):
        clock = Clock()
        guard = self.guard(clock=clock)
        guard._on_data(guard.slots[0], scan())
        self.assertIsNone(guard.arm((0.2, 0, 0), MOVE))

        def velocity(x):
            return types.SimpleNamespace(linear=types.SimpleNamespace(x=x, y=0.0))

        for _ in range(8):
            clock.now += 0.1
            guard._on_data(guard.slots[0], scan())
            guard._on_velocity("in", velocity(0.2))
            guard._on_velocity("out", velocity(0.02))
            if guard.poll():
                break
        self.assertEqual(guard.tripped["stopped_by"], "collision_monitor")

    def test_report(self):
        guard = self.guard()
        guard._on_data(guard.slots[0], scan([(0, 1.0, 20)]))
        guard.arm((0.2, 0, 0), MOVE)
        report = guard.report()
        self.assertEqual(report["sources_used"], ["/scan"])
        self.assertAlmostEqual(report["gap_now_m"], 1.0 - RADIUS - ros2_obstacle.FOOTPRINT_MARGIN_M, delta=0.02)


class StubGuard:
    """What the motion host needs from a guard: trips after *after* polls, or at arm time."""

    def __init__(self, at_arm=None, after=None, kind="obstacle"):
        self.at_arm, self.after, self.kind = at_arm, after, kind
        self.tripped, self.polls, self.disarmed, self.stop_event = None, 0, False, None

    def has_data(self):
        return True

    def arm(self, velocity, move, stop_event=None, timeout=None):
        self.velocity, self.stop_event = velocity, stop_event
        if self.at_arm:
            self.trip()
        return self.tripped

    def trip(self):
        self.tripped = {"stopped_by": self.kind, "obstacle": {"distance_m": 0.2, "bearing_deg": 0.0, "source": "/scan"}}
        self.stop_event.set()

    def poll(self):
        self.polls += 1
        if self.after is not None and self.polls > self.after:
            self.trip()
        return self.tripped

    def report(self):
        return {"active": True}

    def disarm(self):
        self.disarmed = True


class FakeHub:
    def __init__(self):
        self.tracker, self.node, self.sent, self.zeros = None, None, [], 0

    def publish(self, topic, data):
        self.sent.append(data)

    def zero(self, topic=None):
        self.zeros += 1


def move_params(until, **extra):
    params = {"topic": "/cmd_vel", "msg": twist(0.2), "msg_type": "geometry_msgs/msg/Twist", "monitor": "/odom",
              "monitor_msg_type": "nav_msgs/msg/Odometry", "fields": ["pose.pose.position"], "euclidean": True,
              "threshold": 5.0, "rotate": None, "timeout": 2.0, "rate": 10.0, "slow_zone": 0,
              "guard": {"active": True, "sensing": SENSING, "move": dict(MOVE, until=until)}}
    params.update(extra)
    return params


@patch.object(ros2_motionhost, "_types", lambda node, topic, given: given)
@patch("ros2_safety.validate_publish", lambda topic, data, mtype: (True, "ok", None, {}))
class TestGuardedDaemonMove(unittest.TestCase):
    def run_move(self, guard, params):
        hub = FakeHub()
        ros2_motionhost._CANCEL.clear()
        with patch.object(ros2_motionhost, "_hub", lambda *a, **k: hub), \
                patch.object(ros2_motionhost, "_guard_host", lambda *a: types.SimpleNamespace(guard=guard)):
            return hub, ros2_motionhost.run_move(None, params, None)

    def test_nothing_is_published_towards_an_obstacle(self):
        guard = StubGuard(at_arm=True)
        hub, result = self.run_move(guard, move_params(until=False))
        self.assertEqual(hub.sent, [])
        self.assertGreaterEqual(hub.zeros, 4)
        self.assertFalse(result["success"])
        self.assertTrue(result["error"].startswith("not moved: an obstacle"), result["error"])
        self.assertTrue(guard.disarmed)

    def test_until_obstacle_is_success(self):
        hub, result = self.run_move(StubGuard(after=3), move_params(until=True))
        self.assertEqual(len(hub.sent), 3)
        self.assertGreaterEqual(hub.zeros, 4)
        self.assertTrue(result["success"] and result["condition_met"], result)
        self.assertEqual(result["stopped_by"], "obstacle")
        self.assertNotIn("error", result)

    def test_plain_move_stopped_by_an_obstacle_is_a_failure(self):
        _hub, result = self.run_move(StubGuard(after=2), move_params(until=False))
        self.assertFalse(result["success"] or result["condition_met"])
        self.assertIn("stopped for an obstacle", result["error"])

    def test_losing_the_sensor_is_a_failure_even_when_looking_for_an_obstacle(self):
        _hub, result = self.run_move(StubGuard(after=2, kind="sensor_lost"), move_params(until=True))
        self.assertFalse(result["success"])
        self.assertEqual(result["stopped_by"], "sensor_lost")

    def test_a_guard_that_cannot_start_means_no_motion(self):
        hub = FakeHub()
        with patch.object(ros2_motionhost, "_hub", lambda *a, **k: hub), \
                patch.object(ros2_motionhost, "_guard_host", side_effect=RuntimeError("no tf")):
            result = ros2_motionhost.run_move(None, move_params(until=False), None)
        self.assertTrue(result["blocked"])
        self.assertEqual(hub.sent, [])
        self.assertFalse(ros2_motionhost._MOVE_LOCK.locked())


class TestClient(unittest.TestCase):
    ARGS = types.SimpleNamespace(topic="/cmd_vel", msg_type=None, monitor="/odom", monitor_msg_type=None, timeout=5,
                                 rate=None, slow_last=None, slow_factor=0.25)

    def plan(self, guard):
        return {"dry_run": False, "euclidean": True, "rotate_angle": None, "operator": "delta", "threshold": 0.3,
                "msg_data": {}, "field_paths": ["pose.pose.position"], "use_degrees": False, "clamp_notices": [],
                "safety_action": "ok", "safety_warning": None, "guard": guard}

    def ops(self, guard, reply):
        sent = []

        def request(op, params, timeout):
            sent.append((op, params))
            return "ok", reply

        with patch.object(ros2_fastmotion, "_request", request), \
                patch.object(ros2_fastmotion, "restart_stale_daemon", lambda: sent.append(("restart", None))):
            return sent, ros2_fastmotion.try_publish_until(self.ARGS, self.plan(guard))

    def test_a_guarded_move_uses_its_own_op(self):
        guard = {"active": True, "sensing": SENSING, "move": MOVE}
        sent, _ = self.ops(guard, {"success": True})
        self.assertEqual(sent[0][0], "guarded_move")
        self.assertEqual(sent[0][1]["guard"], guard)

    def test_an_old_daemon_never_moves_unguarded(self):
        sent, result = self.ops({"active": True, "sensing": SENSING, "move": MOVE}, {"error": "Unknown fastd op: guarded_move"})
        self.assertIsNone(result)                       # run it here, guarded
        self.assertEqual(sent[-1], ("restart", None))

    def test_an_unguarded_move_says_why(self):
        sent, result = self.ops({"active": False, "reason": "no sensor"}, {"success": True})
        self.assertEqual(sent[0][0], "move")
        self.assertEqual(result["obstacle_stop"], {"active": False, "reason": "no sensor"})


class TestPlan(unittest.TestCase):
    SUMMARY = {"cmd_vel_topic": "/cmd_vel", "capabilities": {"move_distance": {}, "strafe": {}, "rotate_to_heading": {}},
               "localization_config": {"fused_sources": {"odom": "/odom"}},
               "safety_limits": {"binding": {"linear_x": 0.2, "linear_y": 0.2}},
               "obstacle_sensing": {"sources": [SCAN_SOURCE], "live_verified": True}}

    def compile(self, goal, s=None):
        return ros2_plan.compile_plan(goal, self.SUMMARY if s is None else s, [])

    def test_phrasings(self):
        cases = {
            "drive forward until obstacle": ("forward", 10.0, None),
            "drive forward till you see an obstacle": ("forward", 10.0, None),
            "drive until there is a wall": ("forward", 10.0, None),
            "back up until you are 30 cm from the wall": ("back", 10.0, 0.3),
            "move back until something is in the way": ("back", 10.0, None),
            "strafe left until obstacle": ("left", 10.0, None),
            "go right till an object": ("right", 10.0, None),
            "drive forward at 0.1 m/s until an obstacle, up to 3 m": ("forward", 3.0, None),
            "drive forward 2 m or until you see a wall": ("forward", 2.0, None),
            "keep driving until you hit something": ("forward", 10.0, None),
            "move forward until the path is blocked": ("forward", 10.0, None),
        }
        for goal, (direction, limit, gap) in cases.items():
            with self.subTest(goal=goal):
                result = self.compile(goal)
                self.assertEqual(result["warnings"], [])
                step = result["steps"][-1]
                self.assertEqual(step["capability"], "move_until_obstacle")
                self.assertEqual((step["inputs"]["direction"], step["inputs"]["max_distance_m"],
                                  step["inputs"].get("clearance_m")), (direction, limit, gap))
                self.assertIn("--until-obstacle", step["commands"][0])

    def test_velocity_signs(self):
        for goal, key, sign in (("drive back until obstacle", "x", -1), ("go right until obstacle", "y", -1),
                                ("strafe left until obstacle", "y", 1)):
            import shlex
            argv = shlex.split(self.compile(goal)["steps"][-1]["commands"][0])
            self.assertEqual(math.copysign(1, json.loads(argv[3])["linear"][key]), sign, goal)

    def test_other_conditions_are_not_guessed(self):
        for goal in ("drive forward until the wall is gone", "drive forward until battery is low",
                     "drive forward 2 m until battery is low", "drive forward until you have passed the obstacle",
                     "strafe until obstacle"):
            with self.subTest(goal=goal):
                result = self.compile(goal)
                self.assertEqual(result["steps"], [])
                self.assertTrue(result["warnings"])

    def test_no_sensor_in_the_profile(self):
        s = dict(self.SUMMARY, obstacle_sensing={"sources": [], "live_verified": True})
        self.assertIn("no range sensor", self.compile("drive forward until obstacle", s)["warnings"][0])

    def test_following_steps_are_kept(self):
        result = self.compile("drive forward until obstacle then turn left 90")
        self.assertEqual([s["text"] for s in result["steps"]][-1], "turn left 90")

    def test_a_refused_obstacle_stop_does_not_latch_the_estop(self):
        plan = self.compile("drive forward 1 m")
        plan["preflight"] = None
        calls = []

        def run(argv):
            calls.append(argv[0])
            if argv[:2] == ["topics", "publish-until"]:
                return ros2_obstacle.resolve({}, twist(0.2), "/cmd_vel"), 0
            return {}, 0

        self.assertFalse(ros2_plan.run_plan(plan, run, "/odom")["ok"])
        self.assertNotIn("estop", calls)

    def test_an_obstacle_stop_does_not_latch_the_estop(self):
        plan = self.compile("drive forward 1 m")
        plan["preflight"] = None
        calls = []

        def run(argv):
            calls.append(argv[0])
            if argv[:2] == ["topics", "publish-until"]:
                return {"success": False, "stopped_by": "obstacle", "error": "stopped for an obstacle"}, 0
            return {}, 0

        report = ros2_plan.run_plan(plan, run, "/odom")
        self.assertFalse(report["ok"])
        self.assertIn("stop", calls)
        self.assertNotIn("estop", calls)
        self.assertEqual(report["steps"][-1]["result"]["stopped_by"], "obstacle")


class TestRangeSources(unittest.TestCase):
    """Fixtures shaped like the LeKiwi workspace."""

    YAML = {
        "laser.yaml": {"/**/laser_node": {"ros__parameters": {"laser_scan_topic_name": "scan"}}},
        "nav2.yaml": {"/**/collision_monitor": {"ros__parameters": {
            "base_frame_id": "base_footprint", "cmd_vel_in_topic": "cmd_vel_presafety",
            "cmd_vel_out_topic": "base_controller/cmd_vel", "state_topic": "collision_monitor_state",
            "polygons": ["FootprintApproach"], "FootprintApproach": {"enabled": True}, "observation_sources": ["scan"],
            "scan": {"topic": "scan", "type": "scan"}}},
            "/**/local_costmap": {"local_costmap": {"ros__parameters": {"robot_radius": 0.17, "voxel_layer": {
                "observation_sources": "scan", "scan": {"topic": "/scan", "data_type": "LaserScan"}}}}}},
    }

    def build(self, source_text):
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "tof.launch.py"
            path.write_text(source_text)
            return ros2_range_sources.build(list(self.YAML), self.YAML.get, [str(path)],
                                            [{"laser_scan_topic_name": "scan"}], {"robot_radius": 0.17}, "base_footprint",
                                            ["base_footprint", "base_link"])

    def test_lidar_cloud_depth_radius_and_monitor(self):
        sensing = self.build('Node(remappings=[("pointcloud", "tof/pointcloud")])\n'
                             'self.create_publisher(Image, "gemini2/depth/image_raw", 10)\n'
                             'self.create_publisher(OccupancyGrid, "map", 10)\n')
        self.assertEqual([(s["type"], s["topic"]) for s in sensing["sources"]],
                         [("scan", "/scan"), ("pointcloud", "/tof/pointcloud"), ("depth", "/gemini2/depth/image_raw")])
        self.assertEqual(sensing["sources"][2]["info_topic"], "/gemini2/depth/camera_info")
        self.assertEqual(sensing["footprint_radius_m"], 0.17)
        self.assertEqual(sensing["collision_monitor"]["cmd_vel_out"], "/base_controller/cmd_vel")
        self.assertEqual(sensing["collision_monitor"]["enabled"], ["FootprintApproach"])
        self.assertFalse(sensing["live_verified"])

    def test_the_running_graph_adds_what_the_sources_did_not_say(self):
        sensing = self.build("")
        graph = {"/scan": "sensor_msgs/msg/LaserScan", "/scan_raw": "sensor_msgs/msg/LaserScan",
                 "/camera/depth/points": "sensor_msgs/msg/PointCloud2", "/global_costmap/voxel_points":
                 "sensor_msgs/msg/PointCloud2", "/camera/color/image_raw": "sensor_msgs/msg/Image"}
        self.assertTrue(ros2_range_sources.merge_live(sensing, graph))
        self.assertEqual([s["topic"] for s in sensing["sources"]], ["/scan", "/camera/depth/points"])
        self.assertTrue(sensing["live_verified"])
        self.assertFalse(ros2_range_sources.merge_live(sensing, graph))

    def test_depth_topics_by_name_and_processed_clouds(self):
        graph = {"/oak/stereo/image_raw": "sensor_msgs/msg/Image", "/oak/stereo/left/image_raw": "sensor_msgs/msg/Image",
                 "/cloud_registered": "sensor_msgs/msg/PointCloud2", "/scan_matched_points2": "sensor_msgs/msg/PointCloud2",
                 "/cam/depth_registered/points": "sensor_msgs/msg/PointCloud2"}
        self.assertEqual(sorted(s["topic"] for s in ros2_range_sources.live_sources(graph)),
                         ["/cam/depth_registered/points", "/oak/stereo/image_raw"])

    def test_an_empty_graph_confirms_nothing(self):
        sensing = self.build("")
        self.assertFalse(ros2_range_sources.merge_live(sensing, {"/rosout": "rcl_interfaces/msg/Log"}))
        self.assertFalse(sensing["live_verified"])


class TestProfileContext(unittest.TestCase):
    def setUp(self):
        import ros2_profile
        import ros2_profile_context
        self.profile, self.context = ros2_profile, ros2_profile_context
        tmp = tempfile.TemporaryDirectory()
        self.addCleanup(tmp.cleanup)
        root = Path(tmp.name)
        self.ws = root / "ws"
        package = self.ws / "src" / "bot_bringup"
        (package / "config").mkdir(parents=True)
        (package / "package.xml").write_text("<package><name>bot_bringup</name></package>")
        (package / "config" / "laser.yaml").write_text(
            "/**/laser_node:\n  ros__parameters:\n    laser_scan_topic_name: scan\n    frame_id: laser_frame\n")
        (package / "config" / "nav2.yaml").write_text(
            "/**/local_costmap:\n  local_costmap:\n    ros__parameters:\n      robot_radius: 0.2\n")
        env = {k: v for k, v in os.environ.items() if not k.startswith(("ROS2_SKILL_PROFILE", "ROS_NAMESPACE"))}
        env.update(ROS2_LOCAL_WS=str(self.ws))
        for patcher in (patch.object(ros2_profile, "_PROFILES_DIR", root / "profiles"),
                        patch.dict(os.environ, env, clear=True),
                        patch.object(ros2_profile, "_WS_CANDIDATES", [str(self.ws)]),
                        patch.object(ros2_profile_context, "_live_graph", lambda timeout=2.0: {})):
            patcher.start()
            self.addCleanup(patcher.stop)
        ros2_profile._RECORD_CACHE.clear()

    def run_context(self):
        buffer = io.StringIO()
        with contextlib.redirect_stdout(buffer):
            self.context.cmd_profile_context(types.SimpleNamespace())
        return json.loads(buffer.getvalue()), buffer.getvalue()

    def test_creates_the_profile_once_and_prints_it_compactly(self):
        first, text = self.run_context()
        self.assertEqual(first["profile_updated"], "created", first)
        self.assertNotIn("\n", text.strip())
        self.assertEqual([s["topic"] for s in first["obstacle_stop"]["sources"]], ["/scan"])
        self.assertEqual(first["obstacle_stop"]["footprint_radius_m"], 0.2)
        second, _ = self.run_context()
        self.assertNotIn("profile_updated", second)

    def test_changed_sources_are_rescanned_and_overrides_kept(self):
        self.run_context()
        with contextlib.redirect_stdout(io.StringIO()):
            self.context.cmd_profile_set_obstacle(types.SimpleNamespace(
                name=None, clear=False, radius=0.25, max_height=None, ignore_source=None))
        config = self.ws / "src" / "bot_bringup" / "config" / "laser.yaml"
        config.write_text(config.read_text().replace("scan\n", "front_scan\n", 1))
        later = time.time() + 5
        os.utime(config, (later, later))
        result, _ = self.run_context()
        self.assertEqual(result["profile_updated"], "rescanned: sources changed", result)
        self.assertEqual(result["obstacle_stop"]["footprint_radius_m"], 0.25)
        self.assertIn("/front_scan", [s["topic"] for s in result["obstacle_stop"]["sources"]])

    def test_an_older_profile_gains_the_sensing_section(self):
        self.run_context()
        record = self.profile.load_profile_record()
        del record["summary"]["obstacle_sensing"]
        self.profile._save_profile(record, name=record["robot_name"])
        result, _ = self.run_context()
        self.assertEqual(result["profile_updated"], "added obstacle sensing", result)

    def test_a_motion_command_builds_a_missing_profile(self):
        with patch.dict(os.environ, {"ROS2_SKILL_NO_AUTOPROFILE": ""}):
            built, _overrides = self.context.guard_inputs()
        self.assertEqual(built["obstacle_sensing"]["footprint_radius_m"], 0.2)

    def test_never_fails(self):
        with patch.object(self.context, "ensure_current", side_effect=RuntimeError("boom")):
            result, _ = self.run_context()
        self.assertIn("boom", result["profile_problem"])


if __name__ == "__main__":
    unittest.main()
