#!/usr/bin/env python3
"""Obstacle stop for base velocity commands.

Which sensors exist, the robot's size and the collision monitor come from the robot profile
(``summary.obstacle_sensing``, see ros2_range_sources.py). Their data is read from the running stack:

  * every LaserScan beam, every PointCloud2 point and the whole depth image are checked against the path the
    commanded velocity sweeps (straight, strafing or an arc), not one beam or a fixed sector;
  * the collision monitor, when running and acting on the command, counts as an obstacle too;
  * without a fresh sensor looking in the direction of travel the robot does not move, and a move stops if
    that sensor goes quiet.

``resolve`` and the geometry are free of rclpy; ROS objects are created only by ``GuardHost``.
"""

import math
import threading
import time

DEFAULT_CLEARANCE_M = 0.15   # free distance kept between the footprint and an obstacle
FOOTPRINT_MARGIN_M = 0.03    # added to the profile's radius
REACTION_S = 0.3             # sensor period + publish loop + controller
DEFAULT_DECEL = 0.5          # m/s², when `braking measure` has not been run
STALE_MIN_S, STALE_MAX_S = 0.5, 1.5
READY_TIMEOUT_S = 1.5        # longest wait for first data before a move (subscriptions already warm)
COLD_TIMEOUT_S = 3.0         # the same, for subscriptions created by this command
DISCOVERY_S = 1.0            # a new subscription is waited for even before its publisher has been discovered
SETTLE_S = 0.6               # once one source covers the direction, longest extra wait for the others
MIN_LINEAR_MPS = 1e-3
MAX_PUBLISH_INTERVAL_S = 0.1  # a guarded move checks for loss of sensing at least this often
MIN_HEIGHT_M, MAX_HEIGHT_M = 0.04, 0.5   # 3D points outside this band above the floor are floor or overhead
SELF_RADIUS_FRACTION = 0.8   # 3D points this close to the centre (x footprint radius) are the robot seeing itself
MIN_POINTS_SCAN, MIN_POINTS_INSIDE = 2, 5
MIN_POINTS_DENSE, SPARSE_CLOUD = 10, 100
MAX_CLOUD_POINTS = 20000
DEPTH_STRIDE = 4
DENSE_EVAL_PERIOD_S = 0.08
SECTOR_HALF_DEG, SECTOR_MAX_GAP_DEG = 30.0, 15.0   # a scan covers a direction only without a wider blind patch around it
MIN_COVER_POINTS, MIN_COVER_SIDE = 5, 2   # cloud returns near the direction of travel, and per side of it
DEPTH_SIDE_DEG, DEPTH_SLICE_DEG, DEPTH_BAND_DEG, MIN_DEPTH_VALID = 10.0, 5.0, 10.0, 0.2   # see _depth_covered
LOSS_GRACE_S, LOSS_GRACE_MAX_S = 1.0, 2.0   # a lost sensor, while another still watches the same way
ONE_SHOT_S = 0.5             # how long a single velocity message is watched before it is zeroed
DEFAULT_PERIOD_S = 0.1       # assumed sensor period until two messages have been seen
MONITOR_RATIO, MONITOR_HOLD_S = 0.5, 0.5
HEAVY_BYTES, HEAVY_IDLE_S = 1_500_000, 60.0   # large clouds are unsubscribed in the daemon after this long unused
_ACTION_STOP = 1             # nav2_msgs/msg/CollisionMonitorState.STOP

TWIST_TYPES = {f"geometry_msgs{q}{t}" for q in ("/msg/", "/") for t in ("Twist", "TwistStamped")}


# ---------------------------------------------------------------------------
# What to guard (pure)
# ---------------------------------------------------------------------------

def twist_of(msg_data):
    """``(vx, vy, wz)`` of a Twist / TwistStamped dict, or None when the payload is not a velocity."""
    if not isinstance(msg_data, dict):
        return None
    inner = msg_data.get("twist")
    body = inner if isinstance(inner, dict) and ("linear" in inner or "angular" in inner) else msg_data
    if "linear" not in body and "angular" not in body:
        return None
    lin, ang = body.get("linear") or {}, body.get("angular") or {}
    try:
        return float(lin.get("x", 0.0)), float(lin.get("y", 0.0)), float(ang.get("z", 0.0))
    except (TypeError, ValueError, AttributeError):
        return None


def is_base_topic(topic, summary):
    """True when *topic* drives the base (as opposed to, say, an arm servo that also takes a Twist)."""
    summary = summary or {}
    known = {summary.get("cmd_vel_topic")}
    known.update(v.get("topic") for v in summary.get("velocity_topics") or [] if isinstance(v, dict))
    chain = summary.get("velocity_chain") or {}
    known.update(chain.get(k) for k in ("teleop_input", "nav2_input", "output"))
    monitor = (summary.get("obstacle_sensing") or {}).get("collision_monitor") or {}
    known.update(monitor.get(k) for k in ("cmd_vel_in", "cmd_vel_out"))
    name = "/" + str(topic or "").strip("/")
    if not summary:
        return True   # nothing known about this robot: assume the command drives it
    return name in {("/" + k.strip("/")) for k in known if k} or "cmd_vel" in name


def resolve(summary, msg_data, topic, msg_type="", overrides=None, clearance=None, radius=None, until=False,
            disabled=False, decel=None):
    """Decide how a velocity command is guarded. Pure.

    Returns ``{"active": False, "reason": ...}``, ``{"error": ..., "blocked": True}`` or
    ``{"active": True, "sensing": {...}, "move": {...}}``. *until* means the caller asked to drive until an
    obstacle; that, or an explicit *clearance*, turns every "cannot guard this" into an error.
    """
    demanded = until or clearance is not None

    def block(error, hint=None):
        return {"error": error, "blocked": True, "obstacle_stop_blocked": True, **({"hint": hint} if hint else {})}

    def skip(reason):
        """Nothing for an obstacle stop to do here; an error when the caller asked for one."""
        if demanded:
            return block(f"obstacle stop not possible: {reason}")
        return {"active": False, "reason": reason}

    if disabled:
        if until:
            return block("--until-obstacle cannot be combined with --no-obstacle-stop")
        return {"active": False, "reason": "disabled with --no-obstacle-stop"}
    velocity = twist_of(msg_data)
    if velocity is None or (msg_type and msg_type not in TWIST_TYPES):
        return skip("not a velocity command")
    if not is_base_topic(topic, summary):
        return skip(f"{topic} is not a base velocity topic in the profile")
    if math.hypot(velocity[0], velocity[1]) < MIN_LINEAR_MPS:
        return skip("no linear motion (turning on the spot does not move a circular footprint)")
    sensing = dict((summary or {}).get("obstacle_sensing") or {})
    overrides = overrides or {}
    ignored = set(overrides.get("ignore_sources") or [])
    listed = list(sensing.get("sources") or [])
    known = {s.get("topic") for s in listed}
    listed += [s for s in overrides.get("add_sources") or [] if s.get("topic") not in known]
    sources = [s for s in listed if s.get("topic") not in ignored]
    if not sources:
        if overrides.get("no_range_sensors") and not listed and not demanded:
            return {"active": False, "reason": "declared to have no range sensor (profile set-obstacle --no-sensors)"}
        why = "every range sensor in the profile is ignored" if listed else \
            "the profile lists no range sensor (LaserScan, PointCloud2 or depth image)" if sensing else \
            "the profile could not say which range sensors the robot has"
        return block(f"obstacle stop not possible: {why}",
                     "Run `profile context` with the sensors running, or name one: profile set-obstacle --add-source "
                     "scan:/scan. Only if the user says to drive without obstacle detection: --no-obstacle-stop for this "
                     "command, or profile set-obstacle --no-sensors for a robot that has none")
    radius = radius if radius is not None else overrides.get("footprint_radius_m") or sensing.get("footprint_radius_m")
    if not radius or not math.isfinite(float(radius)) or float(radius) <= 0:
        return block("obstacle stop needs the robot's footprint radius and the profile has no usable one",
                     "Measure it once and store it: profile set-obstacle --radius <metres> "
                     "(or pass --robot-radius for this command)")
    if clearance is not None and not (math.isfinite(clearance) and clearance >= 0):
        return block("--obstacle-stop clearance must be a number, 0 or more")
    base_frame = sensing.get("base_frame") or "base_link"
    return {
        "active": True,
        "unverified": not sensing.get("live_verified"),   # the profile has not yet been checked against a running stack
        "sensing": {"base_frame": base_frame, "floor_z": float(sensing.get("floor_z_m") or 0.0),
                    "radius": float(radius), "sources": sources,
                    "max_height": float(overrides.get("max_height_m") or MAX_HEIGHT_M),
                    "monitor": sensing.get("collision_monitor")},
        "move": {"clearance": DEFAULT_CLEARANCE_M if clearance is None else float(clearance), "until": bool(until),
                 "decel": float(decel or DEFAULT_DECEL)},
    }


def trigger_distance(speed, clearance, decel):
    """Distance along the path at which to stop: the clearance plus what the robot covers while reacting and braking."""
    return clearance + speed * REACTION_S + speed * speed / (2.0 * max(decel, 0.05))


# ---------------------------------------------------------------------------
# Geometry (numpy)
# ---------------------------------------------------------------------------

def travel_distance(points, vx, vy, wz, radius):
    """For points ``(N, 2)`` in the base frame: how far the base travels along the commanded path before its
    circular footprint (``radius``) touches each one, ``inf`` if never. A point already inside the footprint
    counts as 0 when the motion goes towards it and ``inf`` when it moves away.

    Returns ``(distance, inside)`` arrays.
    """
    import numpy as np

    x, y = points[:, 0], points[:, 1]
    speed = math.hypot(vx, vy)
    distance = np.full(x.shape, np.inf)
    inside = (x * x + y * y) <= radius * radius
    if speed < 1e-6:
        return distance, inside
    distance[inside & (x * vx + y * vy > 0)] = 0.0
    turn_radius = speed / abs(wz) if abs(wz) > 1e-9 else math.inf
    if turn_radius > 50.0:   # straight for every practical purpose
        along = (x * vx + y * vy) / speed
        lateral = np.abs(x * vy - y * vx) / speed
        hit = (~inside) & (lateral < radius) & (along > 0)
        distance[hit] = np.maximum(0.0, along[hit] - np.sqrt(radius * radius - lateral[hit] ** 2))
        return distance, inside
    cx, cy = -vy / wz, vx / wz                      # centre of rotation
    start = math.atan2(-cy, -cx)                    # angle of the base seen from the centre
    qx, qy = x - cx, y - cy
    r = np.maximum(np.hypot(qx, qy), 1e-9)
    k = (turn_radius ** 2 + r ** 2 - radius ** 2) / (2.0 * turn_radius * r)
    half = np.arccos(np.clip(k, -1.0, 1.0))
    delta = np.mod((np.arctan2(qy, qx) - start) * math.copysign(1.0, wz), 2.0 * math.pi)
    hit = (~inside) & (k <= 1.0)
    distance[hit] = turn_radius * np.maximum(0.0, delta[hit] - half[hit])
    return distance, inside


def rotation_matrix(x, y, z, w):
    import numpy as np

    n = math.sqrt(x * x + y * y + z * z + w * w) or 1.0
    x, y, z, w = x / n, y / n, z / n, w / n
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def scan_points(msg):
    """A LaserScan as ``(points (N,3) in its frame, beam directions (M,3), usable mask (M,), too-close mask (N,),
    beam index of each point (N,))``.

    ``-inf`` means closer than the sensor can measure (REP-117) and becomes a point at ``range_min``.
    ``+inf`` is a clear beam; NaN and values outside the range limits carry no information.
    """
    import numpy as np

    ranges = np.asarray(msg.ranges, dtype=np.float64)
    angles = float(msg.angle_min) + np.arange(ranges.size) * float(msg.angle_increment)
    directions = np.stack([np.cos(angles), np.sin(angles), np.zeros_like(angles)], axis=1)
    low, high = float(msg.range_min), float(msg.range_max) or math.inf
    near = np.isneginf(ranges)
    ranges = np.where(near, max(low, 0.0), ranges)
    with np.errstate(invalid="ignore"):
        valid = np.isfinite(ranges) & (ranges >= low - 1e-6) & (ranges <= high + 1e-6) & (ranges > 0 if low > 0 else ranges >= 0)
    usable = valid | np.isposinf(ranges)
    return directions[valid] * ranges[valid, None], directions, usable, near[valid], np.nonzero(valid)[0]


def _bytes(data):
    """A message's byte array as a uint8 view (no copy for array.array / bytes)."""
    import numpy as np

    return np.asarray(data, dtype=np.uint8) if isinstance(data, list) else np.frombuffer(data, dtype=np.uint8)


def cloud_points(msg):
    """The finite x, y, z of a PointCloud2 as ``(N,3)``, thinned to about ``MAX_CLOUD_POINTS``. Also the total count."""
    import numpy as np

    fields = {f.name: f for f in msg.fields}
    if not all(k in fields for k in "xyz"):
        raise ValueError("cloud has no x/y/z fields")
    kinds = {7: "f4", 8: "f8"}
    if any(fields[k].datatype not in kinds for k in "xyz"):
        raise ValueError("cloud x/y/z are not floating point")
    width, height, step = int(msg.width), int(msg.height), int(msg.point_step)
    total = width * height
    if total == 0:
        return np.empty((0, 3)), 0
    row = int(msg.row_step) or width * step
    data = _bytes(msg.data)
    data = data[: height * row].reshape(height, row)[:, : width * step].reshape(total, step)
    stride = max(1, total // MAX_CLOUD_POINTS)
    data = data[::stride]
    order = ">" if msg.is_bigendian else "<"
    columns = []
    for key in "xyz":
        f = fields[key]
        kind = order + kinds[f.datatype]
        size = int(kind[-1])
        columns.append(np.ascontiguousarray(data[:, f.offset: f.offset + size]).view(kind).reshape(-1).astype(np.float64))
    points = np.stack(columns, axis=1)
    keep = np.isfinite(points).all(axis=1) & (np.abs(points).max(axis=1) > 1e-6)   # (0,0,0) marks an invalid zone
    return points[keep], total


def depth_points(msg, k):
    """A depth image (16UC1 millimetres or 32FC1 metres) and camera matrix ``k`` as ``(points, view)``: the points
    ``(N,3)`` in the optical frame, and the sampled grid ``(ray x per column, ray y per row, has-depth mask)``."""
    import numpy as np

    if msg.encoding in ("16UC1", "mono16"):
        dtype, scale = "u2", 0.001
    elif msg.encoding == "32FC1":
        dtype, scale = "f4", 1.0
    else:
        raise ValueError(f"unsupported depth encoding {msg.encoding}")
    fx, fy, cx, cy = float(k[0]), float(k[4]), float(k[2]), float(k[5])
    if fx <= 0 or fy <= 0:
        raise ValueError("camera_info has no intrinsics")
    dtype = (">" if msg.is_bigendian else "<") + dtype
    size = int(dtype[-1])
    image = _bytes(msg.data)[: int(msg.height) * int(msg.step)].reshape(int(msg.height), int(msg.step))
    image = image[::DEPTH_STRIDE, : int(msg.width) * size]
    image = np.ascontiguousarray(image).view(dtype)[:, ::DEPTH_STRIDE].astype(np.float64) * scale
    valid = np.isfinite(image) & (image > 0)
    rows, cols = np.nonzero(valid)
    z = image[rows, cols]
    u, v = cols * DEPTH_STRIDE, rows * DEPTH_STRIDE
    column = (np.arange(image.shape[1]) * DEPTH_STRIDE - cx) / fx
    row = (np.arange(image.shape[0]) * DEPTH_STRIDE - cy) / fy
    return np.stack([(u - cx) * z / fx, (v - cy) * z / fy, z], axis=1), (column, row, valid)


def _offsets(vectors, travel):
    """Horizontal angle (degrees) of each vector from the direction of travel (radians)."""
    import numpy as np

    bearings = np.arctan2(vectors[:, 1], vectors[:, 0])
    return np.degrees(np.angle(np.exp(1j * (bearings - travel))))


def _sector_covered(offsets):
    """True when the sector around the direction of travel has informative beams with no wide blind patch in it."""
    import numpy as np

    seen = np.sort(offsets[np.abs(offsets) <= SECTOR_HALF_DEG])
    if seen.size == 0:
        return False
    return float(np.diff(np.concatenate([[-SECTOR_HALF_DEG], seen, [SECTOR_HALF_DEG]])).max()) <= SECTOR_MAX_GAP_DEG


def _depth_covered(view, rotation, travel):
    """True when a depth camera looks level in the direction of travel and sees something all the way across it.

    Only pixels whose ray is close to horizontal count: floor in the lower rows says nothing about what stands in
    front of the camera, closer than it can measure. Each narrow slice around the direction needs its own depths.
    """
    import numpy as np

    axis = rotation @ np.array([0.0, 0.0, 1.0])
    if math.hypot(axis[0], axis[1]) <= 0.3:
        return False
    column, row, valid = view
    x, y = np.meshgrid(column, row)
    rays = np.stack([x.ravel(), y.ravel(), np.ones(x.size)], axis=1) @ rotation.T
    level = np.abs(np.degrees(np.arctan2(rays[:, 2], np.hypot(rays[:, 0], rays[:, 1])))) <= DEPTH_BAND_DEG
    offsets, seen = _offsets(rays[level], travel), valid.ravel()[level]
    for low in np.arange(-DEPTH_SIDE_DEG, DEPTH_SIDE_DEG, DEPTH_SLICE_DEG):
        inside = (offsets >= low) & (offsets < low + DEPTH_SLICE_DEG)
        if not inside.any() or seen[inside].mean() < MIN_DEPTH_VALID:
            return False
    return True


def evaluate(kind, parsed, rotation, translation, velocity, trigger, sensing):
    """Check one sensor's data against the commanded motion.

    Returns ``{"covers": bool, "obstacle": {...}|None, "ahead": metres|None}``. *covers* means this message really
    looked in the direction of travel: no information is never "clear". *ahead* is the nearest distance along the
    path to anything the sensor sees; *next* the distance to the nearest group of returns large enough to count.
    """
    import numpy as np

    vx, vy, wz = velocity
    travel = math.atan2(vy, vx)
    radius = sensing["radius"] + FOOTPRINT_MARGIN_M
    self_radius = sensing["radius"] * SELF_RADIUS_FRACTION
    if kind == "scan":
        points, directions, usable, near, beam = parsed
        points = points @ rotation.T + translation
        keep = near | (np.hypot(points[:, 0], points[:, 1]) > self_radius)
        usable = usable.copy()
        usable[beam[~keep]] = False   # a beam that ends on the robot (or on a cover over the sensor) saw nothing
        covers = _sector_covered(_offsets(directions[usable] @ rotation.T, travel))
        points = points[keep]
        need = MIN_POINTS_SCAN
    else:
        if kind == "cloud":
            points, total = parsed
            need = MIN_POINTS_SCAN if total <= SPARSE_CLOUD else MIN_POINTS_DENSE
            view = None
        else:
            points, view = parsed
            need = MIN_POINTS_DENSE
        points = points @ rotation.T + translation
        height = points[:, 2] - sensing["floor_z"]
        outside = np.hypot(points[:, 0], points[:, 1]) > self_radius
        if view is None:   # returns around the direction of travel, on both sides of it; the floor counts, the ceiling not
            offsets = _offsets(points[outside & (height <= sensing["max_height"])], travel)
            offsets = offsets[np.abs(offsets) <= SECTOR_HALF_DEG]
            covers = bool(offsets.size >= MIN_COVER_POINTS and (offsets <= 0).sum() >= MIN_COVER_SIDE
                          and (offsets >= 0).sum() >= MIN_COVER_SIDE)
        else:
            covers = _depth_covered(view, rotation, travel)
        points = points[(height >= MIN_HEIGHT_M) & (height <= sensing["max_height"]) & outside]
    distance, inside = travel_distance(points[:, :2], vx, vy, wz, radius)
    finite = np.isfinite(distance)
    ahead = float(distance[finite].min()) if finite.any() else None
    obstacle = None
    outside_hits = np.nonzero((distance <= trigger) & ~inside)[0]
    inside_hits = np.nonzero((distance <= trigger) & inside)[0]
    # Points already inside the footprint need more agreement on a LiDAR (a few returns off the robot's own parts).
    inside_need = MIN_POINTS_INSIDE if kind == "scan" else min(MIN_POINTS_INSIDE, need)
    both = np.concatenate([inside_hits, outside_hits])
    if inside_hits.size >= inside_need:
        hits = both
    elif outside_hits.size >= need:
        hits = outside_hits
    else:
        hits = both if both.size >= max(need, inside_need) else both[:0]
    if hits.size:
        order = hits[np.argsort(distance[hits])]
        pick = order[min(need, order.size) - 1]   # the need-th nearest: one stray point does not set the figure
        obstacle = {"distance_m": round(float(distance[pick]), 3),
                    "bearing_deg": round(math.degrees(math.atan2(points[pick, 1], points[pick, 0])), 1),
                    "range_m": round(float(math.hypot(points[pick, 0], points[pick, 1])), 3), "points": int(order.size)}
        if inside_hits.size >= inside_need:
            obstacle["inside_footprint"] = True
    outer = distance[finite & ~inside]
    following = float(np.partition(outer, need - 1)[need - 1]) if outer.size >= need else None
    return {"covers": covers, "obstacle": obstacle, "ahead": ahead, "next": following}


# ---------------------------------------------------------------------------
# ROS side
# ---------------------------------------------------------------------------

class _Slot:
    def __init__(self, source):
        self.topic = source["topic"]
        self.kind = {"scan": "scan", "pointcloud": "cloud", "depth": "depth"}.get(source.get("type"), source.get("type"))
        self.info_topic = source.get("info_topic")
        self.sub = self.info_sub = None
        self.cls = None
        self.msg = None          # latest message (raw bytes for large sensors)
        self.raw = False
        self.size = 0
        self.stamp = 0.0
        self.period = None
        self.k = None
        self.parsed = None       # (stamp, frame, data)
        self.error = None
        self.last_eval = 0.0
        self.result = None       # (stamp, evaluation)
        self.since = None        # when the subscription was created
        self.last_ok = None      # when a message was last evaluated successfully
        self.covered_at = None   # receipt time of the newest message that covered the armed motion
        self.expect = None       # (receipt time, distance) of what that message saw next on the path

    def stale_after(self):
        return min(STALE_MAX_S, max(STALE_MIN_S, 3.0 * self.period)) if self.period else STALE_MIN_S

    def fresh(self, now):
        return self.msg is not None and now - self.stamp <= self.stale_after()


class ObstacleGuard:
    """Subscriptions for one sensing setup, evaluated against whatever motion is armed.

    ``node`` must be spun by someone else. Every new message is checked as it arrives while armed, so the stop
    reaches the publish loop within one callback (``stop_event``).
    """

    def __init__(self, node, sensing, tf_buffer, clock=time.monotonic):
        self.node, self.sensing, self.tf, self.clock = node, sensing, tf_buffer, clock
        self.lock = threading.RLock()
        self.slots = [_Slot(s) for s in sensing["sources"]]
        self.monitor = {"state": None, "state_at": 0.0, "in": None, "out": None, "low_since": None, "subs": {}}
        self.motion = None
        self.stop_event = None
        self.tripped = None
        self.armed_at = 0.0
        self.nearest = None
        self.used = set()
        self.last_used = clock()
        self.monitor_looked = None
        self.connect()

    # -- subscriptions ------------------------------------------------------
    def connect(self):
        """Create whatever subscriptions the graph now allows. Safe to call repeatedly."""
        from rclpy.qos import QoSProfile, ReliabilityPolicy
        from ros2_utils import get_msg_type

        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)  # matches reliable and best-effort publishers
        types = {"scan": "sensor_msgs/msg/LaserScan", "cloud": "sensor_msgs/msg/PointCloud2", "depth": "sensor_msgs/msg/Image"}
        for slot in self.slots:
            if slot.sub is not None or slot.kind not in types:
                continue
            slot.cls = get_msg_type(types[slot.kind])
            if slot.cls is None:
                slot.error = "message type unavailable"
                continue
            slot.raw = slot.kind != "scan"
            if slot.kind == "depth" and slot.info_sub is None:
                info = get_msg_type("sensor_msgs/msg/CameraInfo")
                slot.info_sub = self.node.create_subscription(info, slot.info_topic, lambda m, s=slot: self._on_info(s, m), qos)
            slot.sub = self.node.create_subscription(slot.cls, slot.topic, lambda m, s=slot: self._on_data(s, m), qos,
                                                     raw=slot.raw)
            slot.since = self.clock()
        self._connect_monitor(qos)

    def _connect_monitor(self, qos):
        cfg = self.sensing.get("monitor")
        if not cfg:
            return
        from ros2_utils import get_msg_type

        graph = None
        subs = self.monitor["subs"]
        for key in ("cmd_vel_in", "cmd_vel_out", "state_topic"):
            topic = cfg.get(key)
            if not topic or key in subs:
                continue
            if graph is None:
                if self.monitor_looked is not None and self.clock() - self.monitor_looked < 1.0:
                    return
                self.monitor_looked = self.clock()
                graph = dict(self.node.get_topic_names_and_types())
            kind = (graph.get(topic) or [None])[0]
            if kind is None:
                continue
            cls = get_msg_type(kind)
            if cls is None:
                continue
            if key == "state_topic":
                callback = self._on_state
            else:
                callback = (lambda m, k="in" if key == "cmd_vel_in" else "out": self._on_velocity(k, m))
            subs[key] = self.node.create_subscription(cls, topic, callback, 10)

    def drop_idle_heavy(self):
        """Unsubscribe large sensors that have not been needed for a while (they cost bandwidth even unread)."""
        with self.lock:
            if self.motion is not None or self.clock() - self.last_used < HEAVY_IDLE_S:
                return
            for slot in self.slots:
                if slot.sub is not None and slot.size > HEAVY_BYTES:
                    self.node.destroy_subscription(slot.sub)
                    slot.sub, slot.msg, slot.parsed, slot.result = None, None, None, None

    # -- callbacks -----------------------------------------------------------
    def _on_info(self, slot, msg):
        slot.k = list(msg.k)

    def _on_data(self, slot, msg):
        now = self.clock()
        with self.lock:
            if slot.msg is not None:
                gap = now - slot.stamp
                if gap < 5.0:
                    slot.period = gap if slot.period is None else 0.7 * slot.period + 0.3 * gap
            slot.msg, slot.stamp = msg, now
            if slot.raw:
                slot.size = len(msg)
            if self.motion is None or self.tripped is not None:
                return
            if slot.kind != "scan" and now - slot.last_eval < DENSE_EVAL_PERIOD_S:
                return
            result = self._evaluate(slot)
            if result and result["obstacle"]:
                self._trip("obstacle", {"source": slot.topic, **result["obstacle"]})

    def _on_state(self, msg):
        with self.lock:
            self.monitor["state"] = (int(getattr(msg, "action_type", 0)), str(getattr(msg, "polygon_name", "")))
            self.monitor["state_at"] = self.clock()
            if self.motion is not None and self.tripped is None and self.monitor["state"][0] == _ACTION_STOP:
                self._trip("collision_monitor", {"action": "stop", "polygon": self.monitor["state"][1]})

    def _on_velocity(self, key, msg):
        body = getattr(msg, "twist", msg)
        with self.lock:
            self.monitor[key] = (math.hypot(body.linear.x, body.linear.y), self.clock())

    # -- evaluation ----------------------------------------------------------
    def _evaluate(self, slot):
        """Evaluate the slot's latest message against the armed motion; cached per message."""
        if slot.msg is None or self.motion is None:
            return None
        key = (slot.stamp, self.motion["velocity"], self.motion["trigger"])
        if slot.result is not None and slot.result[0] == key:
            return slot.result[1]
        try:
            if slot.parsed is None or slot.parsed[0] != slot.stamp:
                msg = slot.msg
                if slot.raw:
                    from rclpy.serialization import deserialize_message
                    msg = deserialize_message(msg, slot.cls)
                if slot.kind == "scan":
                    data = scan_points(msg)
                elif slot.kind == "cloud":
                    data = cloud_points(msg)
                else:
                    if slot.k is None:
                        slot.error = f"no camera_info on {slot.info_topic}"
                        return None
                    data = depth_points(msg, slot.k)
                slot.parsed = (slot.stamp, msg.header.frame_id, data)
            _, frame, data = slot.parsed
            if slot.kind == "cloud" and str(frame).strip("/").rsplit("/", 1)[-1] in ("map", "odom"):
                raise ValueError(f"cloud in the {frame} frame is processed data, not a live view from a sensor")
            rotation, translation = self._transform(frame)
            speed = math.hypot(self.motion["velocity"][0], self.motion["velocity"][1])
            unseen = speed * min(slot.period or DEFAULT_PERIOD_S, STALE_MAX_S)   # travel until the next message
            result = evaluate(slot.kind, data, rotation, translation, self.motion["velocity"],
                              self.motion["trigger"] + unseen, self.sensing)
        except Exception as exc:  # a bad message or missing transform makes this sensor unusable, never "clear"
            slot.error = f"{type(exc).__name__}: {exc}"
            slot.result = None
            return None
        slot.error = None
        slot.last_eval = slot.last_ok = self.clock()
        slot.result = (key, result)
        if result["covers"]:
            self.used.add(slot.topic)
            slot.expect = (slot.stamp, result["next"])
        if result["ahead"] is not None and (self.nearest is None or result["ahead"] < self.nearest):
            self.nearest = result["ahead"]
        return result

    def _transform(self, frame):
        import numpy as np
        import rclpy

        base = self.sensing["base_frame"]
        frame = (frame or "").lstrip("/")
        if not frame:
            raise ValueError("message has no frame_id")
        if frame == base:
            return np.eye(3), np.zeros(3)
        t = self.tf.lookup_transform(base, frame, rclpy.time.Time()).transform
        return (rotation_matrix(t.rotation.x, t.rotation.y, t.rotation.z, t.rotation.w),
                np.array([t.translation.x, t.translation.y, t.translation.z]))

    def _trip(self, why, detail):
        self.tripped = {"stopped_by": why, "obstacle": detail}
        if self.stop_event is not None:
            self.stop_event.set()

    def _survey(self):
        """Evaluate every fresh source. Returns ``(covering topics, {topic: why it is not usable})``."""
        now, covering, unavailable = self.clock(), [], {}
        for slot in self.slots:
            if not slot.fresh(now):
                unavailable[slot.topic] = "no data" if slot.msg is None else "stale data"
                continue
            result = self._evaluate(slot)
            if result is None:
                unavailable[slot.topic] = slot.error or "not evaluated"
                continue
            if result["covers"]:
                covering.append(slot.topic)
                slot.covered_at = slot.stamp
            if result["obstacle"] and self.tripped is None:
                self._trip("obstacle", {"source": slot.topic, **result["obstacle"]})
        self._expect_overdue(now)
        return covering, unavailable

    def _expect_overdue(self, now):
        """Something a sensor saw on the path stays there while that sensor sends nothing better: stop when the
        distance driven since would have brought the robot to it (late frames, or a camera now too close to see)."""
        speed = math.hypot(self.motion["velocity"][0], self.motion["velocity"][1])
        for slot in self.slots:
            if self.tripped is not None or slot.expect is None or slot.expect[1] is None:
                continue
            stamp, distance = slot.expect
            period = min(slot.period or DEFAULT_PERIOD_S, STALE_MAX_S)
            late = now - max(stamp, self.armed_at) - period
            if late > 0 and distance - speed * late <= self.motion["trigger"] + speed * period:
                self._trip("obstacle", {"source": slot.topic, "distance_m": round(max(0.0, distance - speed * late), 3),
                                        "estimated": True})

    # -- per move -------------------------------------------------------------
    def _awaited(self, slot, now):
        """Whether it is worth delaying the start for a source that is not usable yet."""
        if slot.since is not None and now - slot.since < DISCOVERY_S:
            return True
        if not self.node.count_publishers(slot.topic):
            return False
        last = slot.last_ok if slot.last_ok is not None else slot.since
        return last is not None and now - last < COLD_TIMEOUT_S   # silent or broken for long: do not wait again

    def arm(self, velocity, move, stop_event=None, timeout=READY_TIMEOUT_S, on_wait=None):
        """Start watching *velocity* ``(vx, vy, wz)`` and wait for usable data.

        Returns None when it is safe to start, otherwise the reason not to: ``{"stopped_by": "obstacle" |
        "no_sensor", ...}``. *on_wait* is called once if the answer is not immediate (a caller that is already
        moving stops there).
        """
        speed = math.hypot(velocity[0], velocity[1])
        with self.lock:
            self.connect()
            self.motion = {"velocity": tuple(velocity), "move": move,
                           "trigger": trigger_distance(speed, move["clearance"], move["decel"])}
            self.stop_event, self.tripped, self.nearest, self.used = stop_event, None, None, set()
            self.monitor["low_since"] = None
            for slot in self.slots:
                slot.covered_at = slot.expect = slot.result = None
            self.armed_at = self.last_used = self.clock()
        started, covered_at = self.clock(), None
        while True:
            with self.lock:
                covering, unavailable = self._survey()
                if self.tripped is not None:
                    return self.tripped
                now = self.clock()
                waiting = [s for s in self.slots if s.topic in unavailable and s.sub is not None and self._awaited(s, now)]
            if covering:
                covered_at = covered_at or now
                discovering = any(s.since is not None and now - s.since < DISCOVERY_S for s in waiting)
                if not waiting or (now - covered_at >= SETTLE_S and not discovering):
                    return None
            if now - started >= timeout:
                if covering:
                    return None
                with self.lock:
                    for slot in self.slots:   # say "nothing is publishing" rather than a bare "no data"
                        if slot.topic in unavailable and slot.msg is None and not self.node.count_publishers(slot.topic):
                            unavailable[slot.topic] = "no publisher"
                    self._trip("no_sensor", {"reason": "no fresh sensor data covers the direction of travel",
                                             "bearing_deg": round(math.degrees(math.atan2(velocity[1], velocity[0])), 1),
                                             "sources": unavailable})
                    return self.tripped
            if on_wait is not None:
                on_wait()
                on_wait = None
            time.sleep(0.02)
            with self.lock:
                self.connect()

    def poll(self):
        """Call every publish cycle of an armed move: the reason to stop now, or None.

        Every sensor that has covered this move must keep doing so: losing one (say the camera that sees below the
        LiDAR) ends it, even while another still looks that way. With such another watching, the lost one gets a
        short grace for dropped frames; the only watching sensor gets none beyond its own period.
        """
        with self.lock:
            if self.motion is None:
                if self.tripped is None:
                    self._trip("sensor_lost", {"reason": "the obstacle guard was shut down during the move"})
                return self.tripped
            if self.tripped is None:
                covering, unavailable = self._survey()
                now = self.clock()
                covered = [s for s in self.slots if s.covered_at is not None]
                current = [s for s in covered if now - s.covered_at <= s.stale_after()]
                lost = {}
                for s in covered:
                    limit = s.stale_after()
                    if any(other is not s for other in current):
                        limit = max(limit, LOSS_GRACE_S, min(LOSS_GRACE_MAX_S, 4.0 * (s.period or DEFAULT_PERIOD_S)))
                    if now - s.covered_at > limit:
                        lost[s.topic] = unavailable.get(s.topic, "no longer covers the direction of travel")
                if self.tripped is None and (lost or not any(s.covered_at is not None for s in self.slots)):
                    self._trip("sensor_lost", {"reason": "a sensor watching the direction of travel stopped reporting it",
                                               "sources": lost or unavailable})
            if self.tripped is None:
                self._poll_monitor()
            self.last_used = self.clock()
            return self.tripped

    def _poll_monitor(self):
        now, m = self.clock(), self.monitor
        entering, leaving = m["in"], m["out"]
        limiting = (entering and leaving and entering[1] >= self.armed_at and now - entering[1] < 0.5
                    and now - leaving[1] < 0.5 and entering[0] > 0.02 and leaving[0] < MONITOR_RATIO * entering[0])
        if not limiting:
            m["low_since"] = None
            return
        m["low_since"] = m["low_since"] or now
        if now - m["low_since"] >= MONITOR_HOLD_S:
            state = m["state"]
            self._trip("collision_monitor", {"commanded_mps": round(entering[0], 3), "passed_mps": round(leaving[0], 3),
                                             **({"polygon": state[1]} if state and state[1] else {})})

    def report(self):
        with self.lock:
            now = self.clock()
            out = {"active": True, "sources_used": sorted(self.used)}
            idle = {s.topic: s.error or ("no data" if s.msg is None else "stale data")
                    for s in self.slots if s.error or not s.fresh(now)}
            if idle:
                out["sources_unavailable"] = idle
            if self.motion:
                out.update(clearance_m=self.motion["move"]["clearance"], trigger_m=round(self.motion["trigger"], 3))
            if self.nearest is not None:
                out["nearest_on_path_m"] = round(self.nearest, 3)
            gaps = [r["ahead"] for r in (self._evaluate(s) for s in self.slots if s.fresh(now)) if r and r["ahead"] is not None]
            if gaps:
                out["gap_now_m"] = round(min(gaps), 3)   # free distance along the commanded path when this was written
            if self.sensing.get("monitor"):
                out["collision_monitor"] = "watched" if "cmd_vel_out" in self.monitor["subs"] else "not running"
            return out

    def has_data(self):
        with self.lock:
            return any(s.msg is not None for s in self.slots)

    def disarm(self):
        with self.lock:
            self.motion, self.stop_event, self.last_used = None, None, self.clock()


class GuardHost:
    """An ObstacleGuard with its own node, TF listener (unless one is supplied) and executor thread."""

    _count = 0

    def __init__(self, sensing, tf_buffer=None):
        from rclpy.executors import SingleThreadedExecutor
        from rclpy.node import Node

        GuardHost._count += 1
        self.node = Node(f"skill_obstacle_guard_{GuardHost._count}")
        self.listener = None
        if tf_buffer is None:
            from tf2_ros import Buffer, TransformListener
            tf_buffer = Buffer()
            self.listener = TransformListener(tf_buffer, self.node)
        self.guard = ObstacleGuard(self.node, sensing, tf_buffer)
        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.node)
        self.thread = threading.Thread(target=self._spin, daemon=True)
        self._closed = False
        self.thread.start()

    def _spin(self):
        while not self._closed:
            try:
                self.executor.spin_once(timeout_sec=0.1)
            except Exception:
                if self._closed:
                    return
                time.sleep(0.05)

    def close(self):
        self._closed = True
        try:
            self.guard.disarm()
            self.thread.join(timeout=1.0)
            self.executor.shutdown()
            self.node.destroy_node()
        except Exception:
            pass


_OWN = object()


class CommandGuard:
    """The guard for one command run in this process. Every method is a no-op when *cfg* is None or inactive."""

    def __init__(self, cfg):
        self.cfg, self.host, self.paused = cfg, None, False
        self.active = bool(cfg and cfg.get("active"))

    def arm(self, msg_data, stop_event=None, cfg=_OWN, on_wait=None):
        """Watch the motion in *msg_data*. Returns None when it may start, else why not.

        *cfg* replaces the command's guard for one step of a sequence; None means that step needs no watching.
        """
        cfg = self.cfg if cfg is _OWN else cfg
        if not (cfg and cfg.get("active")):
            self.pause()
            return None
        if self.host is None:
            self.cfg, self.active = cfg, True
            self.host = GuardHost(cfg["sensing"])
        self.cfg, self.paused = cfg, False
        timeout = READY_TIMEOUT_S if self.host.guard.has_data() else COLD_TIMEOUT_S
        return self.host.guard.arm(twist_of(msg_data), cfg["move"], stop_event, timeout=timeout, on_wait=on_wait)

    def pause(self):
        """Stop watching (a step without linear motion)."""
        self.paused = True
        if self.host is not None:
            self.host.guard.disarm()

    def poll(self):
        return self.host.guard.poll() if self.host is not None and not self.paused else None

    @property
    def tripped(self):
        return self.host.guard.tripped if self.host is not None else None

    @property
    def interval_cap(self):
        return MAX_PUBLISH_INTERVAL_S if self.active else math.inf

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()
        return False

    def annotate(self, result, met=False):
        return annotate(result, self.cfg, self.host.guard if self.host else None, met)

    def close(self):
        host, self.host = self.host, None
        if host is None:
            return
        confirm_sources(self.cfg, host.node)
        host.close()


def confirm_sources(cfg, node):
    """Once per robot: record in the profile which range topics the running stack really publishes. Best effort."""
    if not (cfg and cfg.get("active") and cfg.get("unverified")):
        return
    try:
        from ros2_profile_context import published_topics, save_live_sources
        save_live_sources(published_topics(node))
    except Exception:
        pass


def annotate(result, cfg, guard=None, met=False):
    """Add the obstacle-stop report to a velocity command's result; the guard's verdict replaces it if it ended the move."""
    if not cfg:
        return result
    if not cfg.get("active"):
        result["obstacle_stop"] = {"active": False, "reason": cfg.get("reason")}
        return result
    if guard is None:
        return result
    move = cfg["move"]
    result["obstacle_stop"] = guard.report()
    if guard.tripped and not met:
        verdict = outcome(guard.tripped, move)
        if result.get("published_count") == 0 and verdict.get("error", "").startswith("stopped"):
            verdict["error"] = "not moved: " + verdict["error"].split("stopped", 1)[1].lstrip(": ").replace("for ", "", 1)
        for key in ("error", "limited_downstream", "message"):
            result.pop(key, None)
        result.update(verdict)
        if "condition_met" in result:
            result["condition_met"] = verdict["success"]
    elif guard.tripped:
        result.update(stopped_by=guard.tripped["stopped_by"], obstacle=dict(guard.tripped["obstacle"]))
    elif met and move.get("until"):
        result.update(stopped_by="max_distance", message="reached the distance limit without meeting an obstacle")
    return result


def outcome(trip, move):
    """Result fields for a move that the guard stopped (or refused to start)."""
    why = trip["stopped_by"]
    reached = why in ("obstacle", "collision_monitor")
    detail = dict(trip["obstacle"])
    out = {"stopped_by": why, "obstacle": detail}
    if reached and move.get("until"):
        out["success"] = True
        return out
    out["success"] = False
    if why == "obstacle":
        bearing = f" at bearing {detail['bearing_deg']} deg" if "bearing_deg" in detail else \
            " (estimated from the last view of it)"
        where = f"{detail.get('distance_m')} m ahead on the path{bearing} ({detail.get('source')})"
        out["error"] = f"stopped for an obstacle {where}"
    elif why == "collision_monitor":
        out["error"] = "stopped: the collision monitor is holding the robot back (obstacle close in the direction of motion)"
    elif why == "sensor_lost":
        out["error"] = "stopped: obstacle sensing was lost during the move"
    else:
        out["error"] = "not moved: no obstacle sensor covers the direction of travel right now"
        out["hint"] = ("Start the sensor (see `sources`), or fix the profile (`profile context`). Only if the user asks to "
                       "drive without obstacle detection: --no-obstacle-stop")
    return out
