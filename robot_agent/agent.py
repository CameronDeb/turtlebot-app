#!/usr/bin/env python3
"""TurtleBot 4 command agent: a small HTTP server on the robot that turns web-app requests into ROS 2 actions.

Runs on the robot's Raspberry Pi next to the stock bringup. Uses only what the stock TurtleBot 4 image already has
(rclpy, tf2_ros, irobot_create_msgs, numpy); nothing to pip install.

Endpoints (JSON; every request except GET /ping needs the header `X-Agent-Token: <AGENT_TOKEN>`):
  GET  /ping                          liveness, no auth
  GET  /status                        pose (odom -> base_link), battery, dock, sensor ages, busy
  GET  /photo                         a fresh camera frame: base64 PNG (+ aligned 16-bit depth PNG when available)
  GET  /range?bearing=deg&width=deg  nearest lidar return within +-width/2 of a bearing (base_link, + = left)
  GET  /map                           obstacles from the learned map (slam_toolbox), in the odom frame, near the robot
  GET  /map_grid                      the whole learned map for exploration: zlib+base64 bytes (0 free, 100 occupied,
                                      255 unknown), row-major from the origin, plus the map -> odom transform
  GET  /scan                          current lidar returns (base_link, robot body removed, thinned to ~3 cm) and
                                      the robot pose at that moment, for route planning on the PC
  POST /drive   {"distance": m, "speed": m/s, "check_clear": bool, "margin": m}   forward only
  POST /rotate  {"angle_deg": deg (+ = left/CCW), "speed": rad/s}
  POST /undock  {}
  POST /dock    {}
  POST /stop    {}                    cancels the running motion (works while another request is in progress)
  POST /motors  {"enabled": true}     clears the Create 3 e-stop (a short press of the power button or the
                                      controller's B/Circle button toggles it and cuts drive power)

Obstacle handling for /drive with check_clear (the default): the move is shortened to stop `margin` before the
nearest lidar return in the robot's path (it is refused only if there is no room at all), and the lidar keeps
watching while the robot drives: if something appears in the rest of its path (the move would now end closer than
`margin`), the robot stops DYN_STOP_DIST before it (outcome "obstacle"); anything within STOP_GAP stops it at once.

Guards on every drive and rotate (always on, whatever check_clear says):
  - no progress: if the robot hasn't moved STALL_DIST (or turned STALL_ANGLE) in STALL_TIME while it should be
    moving, the motion is canceled (outcome "stalled"). This catches pushing against things the lidar can't see
    (chair bases, cables, low bars below the 19 cm scan plane).
  - Create 3 hazards (bump, cliff, wheel drop, stall) cancel the motion (outcome "bumped" / "cliff" / ...). Only
    works if /hazard_detection is republished to the Pi (turtlebot4_bringup republisher.yaml; off by default).

Learned map (AGENT_MAPPING=1, the default): the first time the agent sees the robot on its dock it starts
slam_toolbox (online async, the TurtleBot 4's slam settings made lighter). If a saved map exists it is continued with
the dock as its first pose, so every session adds to the same map. The map is saved to MAP_BASE when the robot docks,
every SAVE_EVERY s while mapping, and when the agent stops (its first pose is always the dock, so saving anywhere is
fine; only loading needs the robot on the dock).

Motion endpoints return 200 with "outcome" = succeeded | canceled | aborted | obstacle | stalled | bumped | cliff |
wheel_drop plus the pose before and after, so the caller can log what really happened even when a motion was stopped part-way. Errors that prevent a
motion from starting (busy, blocked path, no lidar, motors off, bad input) return 4xx/5xx with
{"ok": false, "error": "..."}.

Run:  AGENT_TOKEN=<secret> ./run_agent.sh        (see README.md)
"""
import argparse
import base64
import subprocess
import hmac
import json
import math
import os
import signal
import struct
import threading
import time
import zlib
from urllib.parse import parse_qs
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import numpy as np
import rclpy
import tf2_ros
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import TwistStamped
from irobot_create_msgs.action import Dock, DriveDistance, RotateAngle, Undock
from irobot_create_msgs.msg import DockStatus, HazardDetectionVector, WheelStatus
from irobot_create_msgs.srv import EStop
from std_srvs.srv import Trigger
from nav_msgs.msg import OccupancyGrid
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from rclpy.time import Time
from sensor_msgs.msg import BatteryState, Image, LaserScan

VERSION = "1.8"

# Motion limits (the web app uses smaller ones; these are the hard caps on the robot side).
MAX_DRIVE_M = 5.0
MAX_DRIVE_SPEED = 0.3  # m/s, Create 3 maximum is 0.306
DEFAULT_DRIVE_SPEED = 0.15
MAX_ROTATE_DEG = 360.0
MAX_ROTATE_SPEED = 1.0  # rad/s
DEFAULT_ROTATE_SPEED = 0.5

# Clear-ahead check against the 2D lidar, in base_link (x forward, y left).
BODY_HALF = 0.19  # lidar points inside this square are the robot itself (TurtleBot 4 radius ~0.17 m + posts)
BODY_FRONT = 0.17  # front bumper distance from base_link
PATH_HALF_WIDTH = 0.21  # half width of the corridor the robot sweeps
DEFAULT_MARGIN = 0.10  # stop this far before the nearest obstacle in the path
STOP_GAP = 0.06  # while driving: cancel if anything is closer than this to the bumper
DYN_STOP_DIST = 0.25  # while driving: stop this far from something that appeared in the path
LOOK_AHEAD = 6.0  # how far ahead the path is checked (m)
STALL_TIME = 0.8  # s without progress while moving -> stop (at 0.15 m/s it normally covers 12 cm in that time)
STALL_DIST = 0.02  # m
STALL_ANGLE = 0.05  # rad (~3 deg)
HAZARD_MAX_AGE = 0.5  # s
HAZARD_NAMES = {1: "bumped", 2: "cliff", 3: "stalled", 4: "wheel_drop"}  # HazardDetection.type
SCAN_MAX_AGE = (
    1.0  # s; the lidar motor stops while docked, so a docked robot has no fresh scan
)

CAMERA_RGB = (
    "/oakd_rgbd/rgb/image_raw"  # our RGB-D node (tools/oakd_rgbd_start.sh), 640x360
)
CAMERA_DEPTH = "/oakd_rgbd/stereo/image_raw"  # depth aligned to that RGB, 16UC1 mm
CAMERA_STOCK = "/oakd/rgb/preview/image_raw"  # stock bringup preview, 250x250, no depth
PHOTO_WAIT = 4.0  # s to wait for a frame captured after the request
# Camera topics are only subscribed while pictures are being taken: decoding 30 frames/s in Python cost ~60% of a
# Pi core. They are released CAMERA_IDLE s after the last use (a Find keeps them alive between pictures).
CAMERA_IDLE = 30.0
# Pinhole intrinsics per topic (fx, fy, cx, cy); the robot publishes no camera_info for these.
# Stock preview: 1080P sensor, ISP 1280x720 (HFOV ~69 deg), centre-cropped to 720x720 and scaled to 250x250.
# Our RGB-D node: 640x360, from the calibration in turtlebot-gs/tools/bag_to_tum.py.
INTRINSICS = {
    CAMERA_STOCK: (323.5, 323.5, 125.0, 125.0),
    CAMERA_RGB: (514.0, 514.0, 324.5, 180.0),
}


# Learned map
MAP_DIR = os.path.expanduser("~/robot_agent/maps")
MAP_BASE = os.path.join(MAP_DIR, "lab")  # slam_toolbox writes lab.posegraph + lab.data
TB4_SLAM_PARAMS = "/opt/ros/jazzy/share/turtlebot4_navigation/config/slam.yaml"
SAVE_EVERY = 120.0  # s
MAP_RADIUS = 8.0  # m: /map only returns obstacles this close to the robot
MAP_OCCUPIED = 65  # occupancy value (0-100) counted as an obstacle


class AgentError(Exception):
    def __init__(self, status, message):
        super().__init__(message)
        self.status = status
        self.message = message


# ---------------------------------------------------------------- images


def image_to_array(msg):
    """sensor_msgs/Image -> numpy array (HxWx3 uint8 RGB, HxW uint8 mono, or HxW uint16)."""
    enc = msg.encoding.lower()
    layouts = {
        "bgr8": (3, np.uint8),
        "rgb8": (3, np.uint8),
        "bgra8": (4, np.uint8),
        "rgba8": (4, np.uint8),
        "mono8": (1, np.uint8),
        "8uc1": (1, np.uint8),
        "mono16": (1, np.uint16),
        "16uc1": (1, np.uint16),
    }
    if enc not in layouts:
        raise AgentError(500, f"unsupported image encoding {msg.encoding!r}")
    ch, dtype = layouts[enc]
    item = np.dtype(dtype).itemsize
    rows = np.frombuffer(bytes(msg.data), np.uint8).reshape(msg.height, msg.step)[
        :, : msg.width * ch * item
    ]
    rows = np.ascontiguousarray(rows)
    if item == 2:
        arr = rows.view(">u2" if msg.is_bigendian else "<u2").astype(np.uint16)
    else:
        arr = rows
    arr = arr.reshape(msg.height, msg.width, ch)
    if enc in ("bgr8", "bgra8"):
        arr = arr[..., [2, 1, 0]]
    elif enc == "rgba8":
        arr = arr[..., :3]
    return arr[..., 0] if ch == 1 else np.ascontiguousarray(arr)


def png_bytes(arr):
    """Minimal PNG encoder (stdlib zlib): HxW uint8/uint16 grayscale or HxWx3 uint8 RGB."""
    if arr.ndim == 2:
        color_type, ch = 0, 1
    elif arr.ndim == 3 and arr.shape[2] == 3:
        color_type, ch = 2, 3
    else:
        raise ValueError("png_bytes needs HxW or HxWx3")
    h, w = arr.shape[:2]
    if arr.dtype == np.uint16:
        bit_depth, data = 16, arr.astype(">u2")
    elif arr.dtype == np.uint8:
        bit_depth, data = 8, arr
    else:
        raise ValueError("png_bytes needs uint8 or uint16")
    raw = np.frombuffer(np.ascontiguousarray(data).tobytes(), np.uint8).reshape(h, -1)
    raw = np.concatenate(
        [np.zeros((h, 1), np.uint8), raw], axis=1
    )  # filter type 0 on every row

    def chunk(tag, payload):
        return (
            struct.pack(">I", len(payload))
            + tag
            + payload
            + struct.pack(">I", zlib.crc32(tag + payload) & 0xFFFFFFFF)
        )

    ihdr = struct.pack(">IIBBBBB", w, h, bit_depth, color_type, 0, 0, 0)
    assert raw.shape[1] == 1 + w * ch * (bit_depth // 8)
    return (
        b"\x89PNG\r\n\x1a\n"
        + chunk(b"IHDR", ihdr)
        + chunk(b"IDAT", zlib.compress(raw.tobytes(), 6))
        + chunk(b"IEND", b"")
    )


def yaw_of(q):
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    )


def wrap(a):
    return (a + math.pi) % (2.0 * math.pi) - math.pi


# ---------------------------------------------------------------- ROS side


class RobotBridge(Node):
    def __init__(self):
        super().__init__("turtlebot_web_agent")
        cb = ReentrantCallbackGroup()
        self._data_lock = threading.Lock()
        self._latest = {}  # topic -> (msg, monotonic receive time)
        self._motion_lock = threading.Lock()
        self._goal_lock = threading.Lock()
        self._active_goal = None
        self._active_name = None
        self._stop = threading.Event()

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.cmd_vel = self.create_publisher(TwistStamped, "/cmd_vel", 10)
        self.action_names = {
            "drive": "/drive_distance",
            "rotate": "/rotate_angle",
            "dock": "/dock",
            "undock": "/undock",
        }
        types = {
            "drive": DriveDistance,
            "rotate": RotateAngle,
            "dock": Dock,
            "undock": Undock,
        }
        self.actions = {
            k: ActionClient(self, types[k], name, callback_group=cb)
            for k, name in self.action_names.items()
        }
        self.estop = self.create_client(EStop, "/e_stop", callback_group=cb)
        # The OAK-D sometimes drops off USB while the motors draw current and comes back unbooted; the stock
        # driver then hangs. Its own stop/start services bring it back.
        self.cam_stop = self.create_client(Trigger, "/oakd/stop_camera", callback_group=cb)
        self.cam_start = self.create_client(Trigger, "/oakd/start_camera", callback_group=cb)
        self._cb = cb
        self._cam_lock = threading.Lock()
        self._cam_subs = []
        self._cam_since = None  # when the camera subscriptions were created
        self._cam_used = 0.0
        self.create_timer(2.0, self._release_camera, callback_group=cb)
        map_qos = QoSProfile(
            depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL
        )
        self.create_subscription(OccupancyGrid, "/map", self._store("/map"), map_qos, callback_group=cb)
        for topic, mtype in (
            ("/scan", LaserScan),
            ("/battery_state", BatteryState),
            ("/dock_status", DockStatus),
            ("/wheel_status", WheelStatus),
            ("/hazard_detection", HazardDetectionVector),
        ):
            self.create_subscription(
                mtype,
                topic,
                self._store(topic),
                qos_profile_sensor_data,
                callback_group=cb,
            )

    def want_camera(self):
        """Subscribe to the camera topics (if not already) and keep them for CAMERA_IDLE s."""
        with self._cam_lock:
            self._cam_used = time.monotonic()
            if self._cam_subs:
                return
            for topic in (CAMERA_RGB, CAMERA_DEPTH, CAMERA_STOCK):
                self._cam_subs.append(
                    self.create_subscription(
                        Image, topic, self._store(topic), qos_profile_sensor_data, callback_group=self._cb
                    )
                )
            self._cam_since = time.monotonic()

    def _release_camera(self):
        with self._cam_lock:
            if not self._cam_subs or time.monotonic() - self._cam_used < CAMERA_IDLE:
                return
            for sub in self._cam_subs:
                self.destroy_subscription(sub)
            self._cam_subs, self._cam_since = [], None
        with self._data_lock:
            for topic in (CAMERA_RGB, CAMERA_DEPTH, CAMERA_STOCK):
                self._latest.pop(topic, None)

    def _store(self, topic):
        def cb(msg):
            with self._data_lock:
                self._latest[topic] = (msg, time.monotonic())

        return cb

    def latest(self, topic):
        with self._data_lock:
            return self._latest.get(topic, (None, None))

    def age(self, topic):
        _, t = self.latest(topic)
        return None if t is None else round(time.monotonic() - t, 2)

    # ---- state

    def pose(self):
        try:
            tf = self.tf_buffer.lookup_transform("odom", "base_link", Time())
        except tf2_ros.TransformException as e:
            raise AgentError(
                503,
                f"no odom -> base_link transform yet ({e.__class__.__name__}); is the bringup running?",
            )
        t = tf.transform.translation
        return {
            "x": round(t.x, 4),
            "y": round(t.y, 4),
            "yaw": round(yaw_of(tf.transform.rotation), 5),
        }

    def map_to_odom(self):
        """[x, y, yaw] of the learned map's frame in odom (p_odom = R(yaw) p_map + t), or None without a map.
        Odom restarts from zero when the base reboots; the map frame doesn't, so the app keeps where objects are
        in map coordinates."""
        try:
            tf = self.tf_buffer.lookup_transform("odom", "map", Time())
        except tf2_ros.TransformException:
            return None
        t = tf.transform.translation
        return [round(t.x, 4), round(t.y, 4), round(yaw_of(tf.transform.rotation), 5)]

    def status(self):
        try:
            pose, pose_error = self.pose(), None
        except AgentError as e:
            pose, pose_error = None, e.message
        battery, _ = self.latest("/battery_state")
        dock, _ = self.latest("/dock_status")
        pct = None
        if battery is not None and not math.isnan(battery.percentage):
            pct = round(battery.percentage * 100.0, 1)
        return {
            "ok": True,
            "agent_version": VERSION,
            "pose": pose,
            "pose_error": pose_error,
            "battery_percent": pct,
            "battery_age": self.age("/battery_state"),
            "docked": None if dock is None else bool(dock.is_docked),
            "dock_visible": None if dock is None else bool(dock.dock_visible),
            "wheels_enabled": self.wheels_enabled(),
            "scan_age": self.age("/scan"),
            "camera": {
                "rgbd_age": self.age(CAMERA_RGB),
                "depth_age": self.age(CAMERA_DEPTH),
                "stock_age": self.age(CAMERA_STOCK),
            },
            "busy": self._motion_lock.locked(),
            "mapping": self.mapper.status() if getattr(self, "mapper", None) else None,
            "map_to_odom": self.map_to_odom(),
            "active": self._active_name,
            "actions_ready": {k: c.server_is_ready() for k, c in self.actions.items()},
        }

    # ---- lidar

    def scan_points(self):
        """Fresh lidar returns in base_link (x forward, y left), robot body removed: (bx, by) arrays."""
        scan, received = self.latest("/scan")
        if scan is None or time.monotonic() - received > SCAN_MAX_AGE:
            raise AgentError(
                409,
                "no recent lidar scan, so the path can't be checked (the lidar stops while docked: "
                "undock first, or turn off the obstacle check)",
            )
        try:
            tf = self.tf_buffer.lookup_transform(
                "base_link", scan.header.frame_id, Time()
            )
        except tf2_ros.TransformException as e:
            raise AgentError(
                503,
                f"no transform base_link -> {scan.header.frame_id} ({e.__class__.__name__})",
            )
        tx, ty, tyaw = (
            tf.transform.translation.x,
            tf.transform.translation.y,
            yaw_of(tf.transform.rotation),
        )
        r = np.asarray(scan.ranges, dtype=np.float64)
        a = scan.angle_min + np.arange(r.size) * scan.angle_increment
        valid = (
            np.isfinite(r) & (r >= max(scan.range_min, 0.05)) & (r <= scan.range_max)
        )
        r, a = r[valid], a[valid]
        lx, ly = r * np.cos(a), r * np.sin(a)
        bx = tx + math.cos(tyaw) * lx - math.sin(tyaw) * ly
        by = ty + math.sin(tyaw) * lx + math.cos(tyaw) * ly
        own_body = (np.abs(bx) < BODY_HALF) & (np.abs(by) < BODY_HALF)
        return bx[~own_body], by[~own_body]

    def gap_ahead(self):
        """Free distance in front of the bumper along the corridor the robot sweeps (inf if clear to LOOK_AHEAD)."""
        bx, by = self.scan_points()
        in_path = (
            (bx > 0) & (bx < BODY_FRONT + LOOK_AHEAD) & (np.abs(by) < PATH_HALF_WIDTH)
        )
        if not in_path.any():
            return math.inf
        return max(float(bx[in_path].min()) - BODY_FRONT, 0.0)

    def map_obstacles(self):
        """Occupied cells of the learned map within MAP_RADIUS of the robot, in the odom frame."""
        grid, t = self.latest("/map")
        if grid is None:
            return {"ok": True, "available": False, "points": []}
        try:
            tf = self.tf_buffer.lookup_transform("odom", grid.header.frame_id or "map", Time())
        except tf2_ros.TransformException as e:
            return {"ok": True, "available": False, "points": [], "error": e.__class__.__name__}
        pose = self.pose()
        info = grid.info
        data = np.asarray(grid.data, dtype=np.int16).reshape(info.height, info.width)
        rows, cols = np.nonzero(data >= MAP_OCCUPIED)
        res = info.resolution
        gx = info.origin.position.x + (cols + 0.5) * res
        gy = info.origin.position.y + (rows + 0.5) * res
        tx, ty, tyaw = tf.transform.translation.x, tf.transform.translation.y, yaw_of(tf.transform.rotation)
        ox = tx + math.cos(tyaw) * gx - math.sin(tyaw) * gy
        oy = ty + math.sin(tyaw) * gx + math.cos(tyaw) * gy
        near = np.hypot(ox - pose["x"], oy - pose["y"]) <= MAP_RADIUS
        pts = np.round(np.stack([ox[near], oy[near]], axis=1), 3).tolist()
        return {"ok": True, "available": True, "points": pts, "age": round(time.monotonic() - t, 1)}

    def map_grid(self):
        grid, t = self.latest("/map")
        if grid is None:
            return {"ok": True, "available": False}
        try:
            tf = self.tf_buffer.lookup_transform("odom", grid.header.frame_id or "map", Time())
        except tf2_ros.TransformException as e:
            return {"ok": True, "available": False, "error": e.__class__.__name__}
        d = np.asarray(grid.data, dtype=np.int16)
        cells = np.where(d < 0, 255, np.where(d >= MAP_OCCUPIED, 100, 0)).astype(np.uint8)
        info = grid.info
        return {
            "ok": True,
            "available": True,
            "resolution": info.resolution,
            "width": info.width,
            "height": info.height,
            "origin": [info.origin.position.x, info.origin.position.y, yaw_of(info.origin.orientation)],
            "map_to_odom": [tf.transform.translation.x, tf.transform.translation.y, yaw_of(tf.transform.rotation)],
            "data": base64.b64encode(zlib.compress(cells.tobytes(), 6)).decode("ascii"),
            "age": round(time.monotonic() - t, 1),
        }

    def scan_for_planning(self, cell=0.03):
        bx, by = self.scan_points()
        pose = self.pose()
        seen, pts = set(), []
        for x, y in zip(bx.tolist(), by.tolist()):
            key = (round(x / cell), round(y / cell))
            if key not in seen:
                seen.add(key)
                pts.append([round(x, 3), round(y, 3)])
        return {"ok": True, "pose": pose, "points": pts}

    def range_at(self, bearing_deg, width_deg):
        """Nearest lidar return (distance from base_link) within +-width/2 of a bearing, or None."""
        bx, by = self.scan_points()
        ang = np.degrees(np.arctan2(by, bx))
        diff = (ang - bearing_deg + 180.0) % 360.0 - 180.0
        sel = np.abs(diff) <= width_deg / 2.0
        if not sel.any():
            return None
        return round(float(np.hypot(bx[sel], by[sel]).min()), 3)

    def hazard(self, ignore=()):
        """Name of a current Create 3 hazard (bumped, cliff, stalled, wheel_drop), or None."""
        msg, t = self.latest("/hazard_detection")
        if msg is None or time.monotonic() - t > HAZARD_MAX_AGE:
            return None
        for d in msg.detections:
            name = HAZARD_NAMES.get(d.type)
            if name and name not in ignore:
                return name
        return None

    def progress_watch(self, linear, check_gap=False, ignore_hazards=(), planned=0.0, margin=DEFAULT_MARGIN):
        """A watch() for run_motion: stops on hazards, on no progress, and (check_gap) on the lidar: on anything
        within STOP_GAP, and on anything new in the rest of the path (planned m) once it is DYN_STOP_DIST away."""
        state = {"t": time.monotonic(), "pose": None, "start": None}

        def watch():
            h = self.hazard(ignore_hazards)
            if h:
                return h
            now = time.monotonic()
            try:
                pose = self.pose()
            except AgentError:
                pose = None
            if pose is not None:
                ref = state["pose"]
                if state["start"] is None:
                    state["start"] = pose
                if ref is None:
                    state["pose"], state["t"] = pose, now
                else:
                    if linear:
                        moved = math.hypot(pose["x"] - ref["x"], pose["y"] - ref["y"]) >= STALL_DIST
                    else:
                        moved = abs(wrap(pose["yaw"] - ref["yaw"])) >= STALL_ANGLE
                    if moved:
                        state["pose"], state["t"] = pose, now
                    elif now - state["t"] > STALL_TIME:
                        return "stalled"
            if check_gap:
                try:
                    gap = self.gap_ahead()
                    travelled = 0.0
                    if pose is not None and state["start"] is not None:
                        travelled = math.hypot(pose["x"] - state["start"]["x"], pose["y"] - state["start"]["y"])
                    remaining = max(planned - travelled, 0.0)
                    blocks_path = gap - remaining < margin - 0.03  # the move would now end too close to it
                    if gap < STOP_GAP or (blocks_path and gap < DYN_STOP_DIST):
                        return "obstacle"
                except AgentError:
                    pass  # a momentarily stale scan doesn't stop the robot; the other guards still do
            return None

        return watch

    def wheels_enabled(self):
        wheels, t = self.latest("/wheel_status")
        if wheels is None or time.monotonic() - t > 2.0:
            return None
        return bool(wheels.wheels_enabled)

    def clear_estop(self):
        if not self.estop.wait_for_service(timeout_sec=3.0):
            raise AgentError(503, "/e_stop service not available")
        fut = self.estop.call_async(EStop.Request(e_stop_on=False))
        if not wait_future(fut, 5.0):
            raise AgentError(504, "/e_stop: no answer")
        deadline = time.monotonic() + 3.0
        while time.monotonic() < deadline and self.wheels_enabled() is not True:
            time.sleep(0.1)
        enabled = self.wheels_enabled()
        return {"ok": enabled is True, "wheels_enabled": enabled, "message": fut.result().message}

    def camera_geometry(self, frame):
        """Camera position (m) in base_link and its optical axis as yaw / pitch (deg, pitch + = looking down)."""
        if not frame:
            return None
        try:
            tf = self.tf_buffer.lookup_transform("base_link", frame, Time())
        except tf2_ros.TransformException:
            return None
        q = tf.transform.rotation
        # optical frames look along +z, other camera frames along +x
        vx, vy, vz = (0.0, 0.0, 1.0) if "optical" in frame else (1.0, 0.0, 0.0)
        x, y, z, w = q.x, q.y, q.z, q.w
        ux, uy, uz = y * vz - z * vy, z * vx - x * vz, x * vy - y * vx
        ux, uy, uz = ux + w * vx, uy + w * vy, uz + w * vz
        dx = vx + 2.0 * (y * uz - z * uy)
        dy = vy + 2.0 * (z * ux - x * uz)
        dz = vz + 2.0 * (x * uy - y * ux)
        t = tf.transform.translation
        return {
            "x": round(t.x, 4),
            "y": round(t.y, 4),
            "z": round(t.z, 4),
            "yaw_deg": round(math.degrees(math.atan2(dy, dx)), 2),
            "pitch_deg": round(-math.degrees(math.asin(max(-1.0, min(1.0, dz)))), 2),
        }

    # ---- actions

    def run_motion(self, name, goal, timeout, watch=None):
        """Send one action goal and wait for it. Returns (outcome, result or None).

        watch: optional callable polled every 0.1 s while the goal runs; if it returns a reason (e.g. "stalled")
        the goal is canceled and that reason is the outcome."""
        if not self._motion_lock.acquire(blocking=False):
            raise AgentError(409, f"robot is busy ({self._active_name})")
        try:
            self._stop.clear()
            client = self.actions[name]
            if name in ("drive", "rotate") and self.wheels_enabled() is False:
                raise AgentError(
                    409,
                    "the robot's motors are off (e-stop). A short press of the Create 3 power button or the "
                    "controller's B/Circle button toggles it; press 'Motors on' in the app to turn them back on",
                )
            if not client.wait_for_server(timeout_sec=3.0):
                raise AgentError(
                    503, f"{self.action_names[name]} action server not available"
                )
            self._active_name = name
            send = client.send_goal_async(goal)
            if not wait_future(send, 5.0):
                raise AgentError(504, f"{name}: no answer to the goal request")
            handle = send.result()
            if not handle.accepted:
                raise AgentError(409, f"{name}: goal rejected by the robot")
            with self._goal_lock:
                self._active_goal = handle
            if self._stop.is_set():  # stop arrived while the goal was being accepted
                handle.cancel_goal_async()
            result_future = handle.get_result_async()
            if watch is not None:
                end = time.monotonic() + timeout
                while not result_future.done() and time.monotonic() < end:
                    try:
                        hit = watch()
                    except AgentError:
                        hit = None
                    if hit:
                        handle.cancel_goal_async()
                        self.publish_zero()
                        wait_future(result_future, 3.0)
                        print(f"{name}: stopped ({hit})", flush=True)
                        return hit, None
                    time.sleep(0.1)
            if not wait_future(result_future, timeout):
                handle.cancel_goal_async()
                self.publish_zero()
                wait_future(result_future, 3.0)
                raise AgentError(
                    504, f"{name}: timed out after {timeout:.0f} s; motion canceled"
                )
            res = result_future.result()
            outcome = {
                GoalStatus.STATUS_SUCCEEDED: "succeeded",
                GoalStatus.STATUS_CANCELED: "canceled",
                GoalStatus.STATUS_ABORTED: "aborted",
            }.get(res.status, f"status_{res.status}")
            return outcome, res.result
        finally:
            with self._goal_lock:
                self._active_goal = None
            self._active_name = None
            self._motion_lock.release()

    def stop(self):
        self._stop.set()
        with self._goal_lock:
            handle = self._active_goal
        if handle is not None:
            handle.cancel_goal_async()
        self.publish_zero()
        return handle is not None

    def publish_zero(self):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"
        for _ in range(3):
            self.cmd_vel.publish(msg)

    # ---- camera

    def restart_camera(self):
        print("camera: no frames, restarting the OAK-D (stop_camera / start_camera)", flush=True)
        for client, wait in ((self.cam_stop, 20.0), (self.cam_start, 40.0)):
            if not client.wait_for_service(timeout_sec=3.0):
                return False
            fut = client.call_async(Trigger.Request())
            if not wait_future(fut, wait):
                return False
            time.sleep(2.0)
        time.sleep(4.0)
        return True

    def photo(self):
        try:
            return self._photo()
        except AgentError as e:
            if e.status != 503 or not self.restart_camera():
                raise
            return self._photo()

    def _photo(self):
        self.want_camera()
        requested = time.monotonic()
        deadline = requested + PHOTO_WAIT
        rgbd_running = self.count_publishers(CAMERA_RGB) > 0
        rgb = topic = None
        while time.monotonic() < deadline:
            msg, t = self.latest(CAMERA_RGB)
            if msg is not None and t > requested:
                rgb, topic = msg, CAMERA_RGB
                break
            stock, ts = self.latest(CAMERA_STOCK)
            rgbd_recent = t is not None and time.monotonic() - t < 2.0
            # right after subscribing, give our RGB-D node a moment before settling for the stock preview
            settling = rgbd_running and time.monotonic() - (self._cam_since or 0.0) < 1.5
            if stock is not None and ts > requested and not rgbd_recent and not settling:
                rgb, topic = stock, CAMERA_STOCK
                break
            time.sleep(0.03)
        if rgb is None:
            raise AgentError(
                503,
                "no camera frames: start the camera (stock bringup preview or our oakd_rgbd node)",
            )
        depth = None
        if topic == CAMERA_RGB:
            while time.monotonic() < deadline:
                dmsg, dt = self.latest(CAMERA_DEPTH)
                if dmsg is not None and dt > requested:
                    depth = dmsg
                    break
                time.sleep(0.03)
        try:
            pose = self.pose()
        except AgentError:
            pose = None
        camera = self.camera_geometry(rgb.header.frame_id)
        fx, fy, cx, cy = INTRINSICS.get(topic, (None, None, None, None))
        rgb_arr = image_to_array(rgb)
        if rgb_arr.ndim == 2:
            rgb_arr = np.stack([rgb_arr] * 3, axis=-1)
        out = {
            "ok": True,
            "topic": topic,
            "width": int(rgb.width),
            "height": int(rgb.height),
            "stamp": rgb.header.stamp.sec + rgb.header.stamp.nanosec * 1e-9,
            "pose": pose,
            "rgb_png": base64.b64encode(png_bytes(rgb_arr.astype(np.uint8))).decode(
                "ascii"
            ),
            "depth_png": None,
            "intrinsics": None if fx is None else {"fx": fx, "fy": fy, "cx": cx, "cy": cy},
            "camera": camera,
        }
        if depth is not None:
            d = image_to_array(depth)
            if d.dtype == np.uint16:
                out["depth_png"] = base64.b64encode(png_bytes(d)).decode("ascii")
        return out


class Mapper:
    """Runs slam_toolbox while the agent runs, anchored at the dock, and saves the map (see the module doc)."""

    def __init__(self, bridge, enabled):
        from slam_toolbox.srv import SerializePoseGraph

        self.bridge, self.enabled = bridge, enabled
        self.srv_type = SerializePoseGraph
        self.save_client = bridge.create_client(SerializePoseGraph, "/slam_toolbox/serialize_map")
        self.proc = None
        self.loaded = False
        self.message = "off" if not enabled else "waiting for the robot to be on its dock"
        self.last_docked = None
        self.last_save = 0.0
        self.lock = threading.Lock()
        self._stop = threading.Event()
        threading.Thread(target=self._loop, daemon=True).start()

    def status(self):
        return {
            "enabled": self.enabled,
            "running": self.proc is not None and self.proc.poll() is None,
            "continued_saved_map": self.loaded,
            "message": self.message,
            "saved": os.path.exists(MAP_BASE + ".posegraph"),
        }

    def _docked(self):
        dock, t = self.bridge.latest("/dock_status")
        if dock is None or time.monotonic() - t > 5.0:
            return None
        return bool(dock.is_docked)

    def _loop(self):
        while not self._stop.wait(2.0):
            if not self.enabled:
                continue
            try:
                self._tick()
            except Exception as e:  # never let mapping take the agent down
                self.message = f"mapping error: {e!r}"
                print("mapper:", repr(e), flush=True)

    def _adopt(self):
        """A slam_toolbox started by an earlier agent (agent restarts leave mapping running): keep using it."""
        out = subprocess.run(["pgrep", "-f", "[a]sync_slam_toolbox_node.*" + MAP_DIR], capture_output=True, text=True)
        pids = [int(x) for x in out.stdout.split()]
        if not pids:
            return False
        self.proc = _Adopted(pids[0])
        self.loaded = os.path.exists(MAP_BASE + ".posegraph")
        self.message = "learning (carried on from before the agent restarted)"
        self.last_save = time.monotonic()
        print(f"mapper: adopted running slam_toolbox pid {pids[0]}", flush=True)
        return True

    def _tick(self):
        docked = self._docked()
        if self.proc is None and self.last_docked is None and self._adopt():
            self.last_docked = docked
            return
        running = self.proc is not None and self.proc.poll() is None
        if self.proc is not None and not running:
            self.message = f"slam_toolbox exited (code {self.proc.returncode}); restarts next time on the dock"
            self.proc = None
        if self.proc is None:
            if docked:
                self._start()
        else:
            if self.last_docked is False and docked is True:
                self.save("docked")
            elif time.monotonic() - self.last_save > SAVE_EVERY:
                self.save("periodic")
        self.last_docked = docked

    def _start(self):
        import yaml

        os.makedirs(MAP_DIR, exist_ok=True)
        # a slam_toolbox left over from an agent that was killed hard would fight this one for the map frame
        subprocess.run(["pkill", "-f", "[a]sync_slam_toolbox_node"], check=False)
        subprocess.run(["pkill", "-f", "[s]lam_params_file:=" + MAP_DIR], check=False)
        time.sleep(1.0)
        with open(TB4_SLAM_PARAMS) as f:
            params = yaml.safe_load(f)
        p = params["slam_toolbox"]["ros__parameters"]
        p.update(
            {
                "use_sim_time": False,
                "mode": "mapping",
                "enable_interactive_mode": False,
                "transform_publish_period": 0.05,
                "map_update_interval": 2.0,
                "minimum_time_interval": 0.5,
            }
        )
        self.loaded = os.path.exists(MAP_BASE + ".posegraph")
        if self.loaded:
            p["map_file_name"] = MAP_BASE
            p["map_start_at_dock"] = True
        path = os.path.join(MAP_DIR, "slam_params.yaml")
        with open(path, "w") as f:
            yaml.safe_dump(params, f)
        log = open(os.path.join(MAP_DIR, "slam.log"), "ab")
        self.proc = subprocess.Popen(
            ["ros2", "launch", "slam_toolbox", "online_async_launch.py",
             f"slam_params_file:={path}", "use_sim_time:=false"],
            stdout=log, stderr=subprocess.STDOUT, stdin=subprocess.DEVNULL, start_new_session=True,
        )
        self.last_save = time.monotonic()
        self.message = "learning (continuing the saved map)" if self.loaded else "learning (new map)"
        print(f"mapper: started slam_toolbox, pid {self.proc.pid}, loaded={self.loaded}", flush=True)

    def save(self, why):
        with self.lock:
            if self.proc is None or self.proc.poll() is not None:
                return False
            self.last_save = time.monotonic()
            if self.bridge.latest("/map")[0] is None:
                return False  # nothing mapped yet (the lidar is off on the dock): don't save an empty map
            if not self.save_client.wait_for_service(timeout_sec=2.0):
                self.message = "learning (save service not ready yet)"
                return False
            fut = self.save_client.call_async(self.srv_type.Request(filename=MAP_BASE))
            if not wait_future(fut, 30.0) or fut.result().result != 0:
                self.message = "learning (last save FAILED)"
                print(f"mapper: save ({why}) failed", flush=True)
                return False
            self.message = f"learning; saved ({why}) at {time.strftime('%H:%M:%S')}"
            print(f"mapper: saved map ({why})", flush=True)
            return True

    def shutdown(self, stop_mapping=False):
        """Save the map. Mapping keeps running (the next agent adopts it) unless stop_mapping / AGENT_MAPPING=0."""
        self._stop.set()
        if self.proc is not None and self.proc.poll() is None:
            self.save("agent stopping")
            if stop_mapping:
                try:
                    os.killpg(os.getpgid(self.proc.pid), signal.SIGINT)
                except Exception:
                    pass


class _Adopted:
    """Popen-like handle for a slam_toolbox process this agent didn't start."""

    def __init__(self, pid):
        self.pid = pid
        self.returncode = None

    def poll(self):
        try:
            os.kill(self.pid, 0)
            return None
        except OSError:
            self.returncode = -1
            return -1


def wait_future(future, timeout):
    end = time.monotonic() + timeout
    while not future.done():
        if time.monotonic() > end:
            return False
        time.sleep(0.02)
    return True


# ---------------------------------------------------------------- HTTP side


def number(body, key, default=None, lo=None, hi=None):
    v = body.get(key, default)
    if (
        v is None
        or isinstance(v, bool)
        or not isinstance(v, (int, float))
        or not math.isfinite(v)
    ):
        raise AgentError(400, f"'{key}' must be a number")
    if (lo is not None and v < lo) or (hi is not None and v > hi):
        raise AgentError(400, f"'{key}' must be between {lo} and {hi}")
    return float(v)


def make_handler(bridge, token):
    class Handler(BaseHTTPRequestHandler):
        server_version = f"TurtleBotAgent/{VERSION}"

        def log_message(self, fmt, *args):
            print(
                time.strftime("%H:%M:%S"), self.address_string(), fmt % args, flush=True
            )

        def send_json(self, status, obj):
            data = json.dumps(obj).encode()
            self.send_response(status)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(data)))
            self.end_headers()
            self.wfile.write(data)

        def authorized(self):
            given = self.headers.get("X-Agent-Token", "")
            return hmac.compare_digest(given.encode(), token.encode())

        def body(self):
            n = int(self.headers.get("Content-Length") or 0)
            if n > 10_000:
                raise AgentError(413, "request body too large")
            if n == 0:
                return {}
            try:
                obj = json.loads(self.rfile.read(n))
            except ValueError:
                raise AgentError(400, "body must be JSON")
            if not isinstance(obj, dict):
                raise AgentError(400, "body must be a JSON object")
            return obj

        def handle_any(self, method):
            path = self.path.split("?", 1)[0].rstrip("/") or "/"
            try:
                if method == "GET" and path == "/ping":
                    return self.send_json(200, {"ok": True, "agent_version": VERSION})
                if not self.authorized():
                    raise AgentError(401, "missing or wrong X-Agent-Token")
                route = (method, path)
                if route == ("GET", "/status"):
                    if "camera=1" in self.path:
                        bridge.want_camera()  # the caller is about to take pictures: start receiving frames
                    return self.send_json(200, bridge.status())
                if route == ("GET", "/photo"):
                    return self.send_json(200, bridge.photo())
                if route == ("GET", "/range"):
                    q = parse_qs(self.path.split("?", 1)[1] if "?" in self.path else "")
                    try:
                        bearing = float(q.get("bearing", ["0"])[0])
                        width = float(q.get("width", ["6"])[0])
                    except ValueError:
                        raise AgentError(400, "bearing and width must be numbers")
                    if not (math.isfinite(bearing) and 0.5 <= width <= 90):
                        raise AgentError(400, "width must be between 0.5 and 90 degrees")
                    return self.send_json(
                        200, {"ok": True, "range": bridge.range_at(bearing, width)}
                    )
                if route == ("GET", "/map_grid"):
                    return self.send_json(200, bridge.map_grid())
                if route == ("GET", "/map"):
                    return self.send_json(200, bridge.map_obstacles())
                if route == ("GET", "/scan"):
                    return self.send_json(200, bridge.scan_for_planning())
                if route == ("POST", "/motors"):
                    if self.body().get("enabled") is not True:
                        raise AgentError(400, 'send {"enabled": true}')
                    return self.send_json(200, bridge.clear_estop())
                if route == ("POST", "/stop"):
                    self.body()
                    return self.send_json(
                        200, {"ok": True, "canceled_active": bridge.stop()}
                    )
                if route == ("POST", "/drive"):
                    return self.send_json(200, self.drive(self.body()))
                if route == ("POST", "/rotate"):
                    return self.send_json(200, self.rotate(self.body()))
                if route in (("POST", "/dock"), ("POST", "/undock")):
                    self.body()
                    return self.send_json(200, self.dock(path[1:]))
                raise AgentError(404, f"no route {method} {path}")
            except AgentError as e:
                self.send_json(e.status, {"ok": False, "error": e.message})
            except (
                Exception
            ) as e:  # keep the server alive; report the problem to the caller
                print("error:", repr(e), flush=True)
                self.send_json(500, {"ok": False, "error": f"agent error: {e!r}"})

        def motion_reply(self, before, outcome, extra):
            time.sleep(0.25)  # let TF catch up with the final pose
            try:
                after = bridge.pose()
            except AgentError:
                after = None
            return {
                "ok": outcome == "succeeded",
                "outcome": outcome,
                "pose_before": before,
                "pose_after": after,
                **extra,
            }

        def drive(self, body):
            distance = number(body, "distance", lo=0.01, hi=MAX_DRIVE_M)
            speed = number(
                body, "speed", DEFAULT_DRIVE_SPEED, lo=0.02, hi=MAX_DRIVE_SPEED
            )
            margin = number(body, "margin", DEFAULT_MARGIN, lo=0.0, hi=1.0)
            check = body.get("check_clear", True)
            if not isinstance(check, bool):
                raise AgentError(400, "'check_clear' must be true or false")
            before = bridge.pose()
            if bridge.hazard() == "bumped":
                raise AgentError(409, "the bumper is pressed: something is touching the front of the robot")
            requested, gap = distance, None
            if check:
                gap = bridge.gap_ahead()
                room = gap - margin
                if room < 0.02:
                    raise AgentError(
                        409,
                        f"path blocked: obstacle {gap:.2f} m ahead of the bumper (no room to move forward)",
                    )
                distance = min(distance, room)
            watch = bridge.progress_watch(linear=True, check_gap=check, planned=distance, margin=margin)
            goal = DriveDistance.Goal(distance=distance, max_translation_speed=speed)
            outcome, _ = bridge.run_motion(
                "drive", goal, timeout=distance / speed * 2.0 + 10.0, watch=watch
            )
            return self.motion_reply(
                before,
                outcome,
                {
                    "requested": {"distance": requested},
                    "planned_distance": round(distance, 3),
                    "shortened": distance < requested - 1e-6,
                    "obstacle_gap": None if gap is None or math.isinf(gap) else round(gap, 3),
                },
            )

        def rotate(self, body):
            angle = number(body, "angle_deg", lo=-MAX_ROTATE_DEG, hi=MAX_ROTATE_DEG)
            speed = number(
                body, "speed", DEFAULT_ROTATE_SPEED, lo=0.05, hi=MAX_ROTATE_SPEED
            )
            before = bridge.pose()
            goal = RotateAngle.Goal(angle=math.radians(angle), max_rotation_speed=speed)
            # Turning is how the robot gets free after a bump, so an existing bump doesn't stop a turn.
            outcome, _ = bridge.run_motion(
                "rotate",
                goal,
                timeout=abs(math.radians(angle)) / speed * 2.0 + 10.0,
                watch=bridge.progress_watch(linear=False, ignore_hazards=("bumped",)),
            )
            return self.motion_reply(
                before, outcome, {"requested": {"angle_deg": angle}}
            )

        def dock(self, which):
            try:
                before = bridge.pose()
            except AgentError:
                before = None
            goal = Dock.Goal() if which == "dock" else Undock.Goal()
            outcome, result = bridge.run_motion(which, goal, timeout=120.0)
            docked = None if result is None else bool(result.is_docked)
            reply = self.motion_reply(before, outcome, {"is_docked": docked})
            if which == "dock" and outcome == "succeeded" and not docked:
                reply["ok"] = False
            if which == "undock" and outcome == "succeeded" and docked:
                reply["ok"] = False
            return reply

        def do_GET(self):
            self.handle_any("GET")

        def do_POST(self):
            self.handle_any("POST")

    return Handler


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--bind", default=os.environ.get("AGENT_BIND", "0.0.0.0"))
    ap.add_argument(
        "--port", type=int, default=int(os.environ.get("AGENT_PORT", "8765"))
    )
    args = ap.parse_args()
    token = os.environ.get("AGENT_TOKEN", "")
    if len(token) < 16:
        raise SystemExit(
            "Set AGENT_TOKEN to a secret of at least 16 characters (the web app sends the same value)."
        )

    # Own the signals: rclpy's default handlers would swallow SIGTERM/SIGINT and leave the HTTP server running.
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    bridge = RobotBridge()
    # One spin thread is enough: HTTP requests run in their own threads and only wait on futures and cached
    # messages. (The multi-threaded executor cost ~40% of a Pi core just idling.)
    executor = SingleThreadedExecutor()
    executor.add_node(bridge)
    spin = threading.Thread(target=executor.spin, daemon=True)
    spin.start()

    bridge.mapper = Mapper(bridge, os.environ.get("AGENT_MAPPING", "1") == "1")
    server = ThreadingHTTPServer((args.bind, args.port), make_handler(bridge, token))
    server.daemon_threads = True

    def on_signal(signum, frame):
        print(f"signal {signum}: stopping", flush=True)
        bridge.stop()
        threading.Thread(target=server.shutdown, daemon=True).start()

    signal.signal(signal.SIGTERM, on_signal)
    signal.signal(signal.SIGINT, on_signal)
    print(f"TurtleBot agent {VERSION} listening on {args.bind}:{args.port}", flush=True)
    try:
        server.serve_forever()
    finally:
        bridge.stop()
        bridge.mapper.shutdown()
        server.server_close()
        executor.shutdown()
        bridge.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
