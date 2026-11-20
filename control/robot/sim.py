"""In-process simulated TurtleBot, for trying the app without the robot and for tests.

Matches the real robot's behaviour where the app depends on it: starts docked at the origin facing the dock (+x);
undock backs up 0.35 m and turns 180 deg; dock only works from within 1 m while facing the dock; the lidar check
fails while docked; a forward move is shortened to stop 0.10 m before an obstacle in its path (refused only when
there is no room); a motion can be stopped part-way; the motors can be switched off (e-stop).

Obstacles are points the lidar sees. Low obstacles are points it can't see (chair bases, cables): driving into one
stops the robot like the agent's no-progress guard does (outcome "stalled"). Objects are small things on the floor
(below the lidar) that the camera sees and the simulated detector reports, so the whole object search can be tried
without the robot.
"""

import io
import math
import threading
import time

from PIL import Image, ImageDraw

from .base import MotionResult, PhotoData, Pose, RobotBusy, RobotClient, RobotError

BODY_FRONT = 0.17
PATH_HALF_WIDTH = 0.21
MARGIN = 0.10
IMAGE_W, IMAGE_H = 640, 360
INTRINSICS = {"fx": 514.0, "fy": 514.0, "cx": 320.0, "cy": 180.0}
CAMERA = {"x": 0.0, "y": 0.0, "z": 0.244, "yaw_deg": 0.0, "pitch_deg": 0.0}
OBJECT_RADIUS = 0.04
OBJECT_HEIGHT = 0.12


def _wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


class SimRobotClient(RobotClient):
    name = "sim"

    def __init__(self, delay=1.0, obstacles=(), objects=(), low_obstacles=()):
        self.delay = delay  # seconds per motion
        self.obstacles = list(obstacles)  # (x, y) points the lidar sees
        self.objects = list(objects)  # (label, x, y) small objects on the floor the camera sees
        self.low_obstacles = list(low_obstacles)  # (x, y) points below the lidar: the robot gets stuck on them
        # (x, y) points that only show up once the robot is moving (a person or chair moved into its way); the
        # lidar watch stops the robot 0.25 m before one in its path, then it is an ordinary obstacle
        self.appearing = []
        self._lock = threading.Lock()
        self._busy = threading.Lock()
        self._stop = threading.Event()
        self.x = self.y = self.yaw = 0.0
        self.docked = True
        self.wheels_enabled = True

    # ---- state

    def _pose(self):
        return Pose(round(self.x, 4), round(self.y, 4), round(self.yaw, 5))

    def _sees_dock(self):
        if self.docked:
            return True
        bearing = math.atan2(-self.y, -self.x)
        return math.hypot(self.x, self.y) < 1.0 and abs(_wrap(bearing - self.yaw)) < math.radians(30)

    def _local(self, px, py):
        """World point -> (ahead, left) in the robot frame."""
        dx, dy = px - self.x, py - self.y
        return dx * math.cos(self.yaw) + dy * math.sin(self.yaw), -dx * math.sin(self.yaw) + dy * math.cos(self.yaw)

    def status(self):
        with self._lock:
            return {
                "connected": True,
                "pose": self._pose(),
                "pose_error": None,
                "battery_percent": 87.0,
                "docked": self.docked,
                "dock_visible": self._sees_dock(),
                "wheels_enabled": self.wheels_enabled,
                "busy": self._busy.locked(),
                "active": None,
                "scan_age": None if self.docked else 0.1,
                "camera": {"rgbd_age": 0.1, "depth_age": 0.1, "stock_age": None},
            }

    def motors_on(self):
        with self._lock:
            self.wheels_enabled = True
        return {"ok": True, "wheels_enabled": True, "message": ""}

    # ---- motion

    def _animate(self, steps, step_fn):
        """Apply step_fn `steps` times over self.delay seconds; False if stopped part-way."""
        for _ in range(steps):
            if self._stop.is_set():
                return False
            with self._lock:
                step_fn()
            if self.delay:
                time.sleep(self.delay / steps)
        return True

    def _motion(self, fn, raw=None):
        if not self._busy.acquire(blocking=False):
            raise RobotBusy("robot is busy")
        try:
            self._stop.clear()
            with self._lock:
                before = self._pose()
            outcome, docked = fn()
            with self._lock:
                after = self._pose()
            return MotionResult(outcome, before, after, is_docked=docked, raw=raw)
        finally:
            self._busy.release()

    def _check_motors(self):
        if not self.wheels_enabled:
            raise RobotError("the robot's motors are off (e-stop); press 'Motors on'")

    def gap_ahead(self):
        gap = math.inf
        for ox, oy in self.obstacles:
            ahead, side = self._local(ox, oy)
            if abs(side) < PATH_HALF_WIDTH and ahead > 0:
                gap = min(gap, max(ahead - BODY_FRONT, 0.0))
        return gap

    def drive(self, distance, check_clear=True):
        if not 0 < distance <= 5.0:
            raise RobotError("'distance' must be between 0.01 and 5.0")
        requested, gap = distance, None
        with self._lock:
            self._check_motors()
            if check_clear:
                if self.docked:
                    raise RobotError(
                        "no recent lidar scan, so the path can't be checked (the lidar stops while docked: "
                        "undock first, or turn off the obstacle check)"
                    )
                gap = self.gap_ahead()
                if gap - MARGIN < 0.02:
                    raise RobotError(f"path blocked: obstacle {gap:.2f} m ahead of the bumper (no room to move forward)")
                distance = min(distance, gap - MARGIN)
        raw = {
            "requested": {"distance": requested},
            "planned_distance": round(distance, 3),
            "shortened": distance < requested - 1e-6,
            "obstacle_gap": None if gap is None or math.isinf(gap) else round(gap, 3),
        }

        def fn():
            steps = 20
            step = distance / steps
            stuck = []

            def s():
                if stuck:
                    return
                if check_clear and self.appearing:
                    self.obstacles.extend(self.appearing)
                    self.appearing = []
                if check_clear:
                    gap, remaining = self.gap_ahead(), distance - step * done_steps[0]
                    if gap < 0.06 or (gap - remaining < MARGIN - 0.03 and gap < 0.25):  # same rule as the agent
                        stuck.append("obstacle")
                        return
                for ox, oy in self.low_obstacles:
                    ahead, side = self._local(ox, oy)
                    if abs(side) < PATH_HALF_WIDTH and 0 < ahead - BODY_FRONT < step:
                        stuck.append("stalled")  # the bumper reaches it: stuck, like the real no-progress guard
                        return
                self.docked = False
                self.x += step * math.cos(self.yaw)
                self.y += step * math.sin(self.yaw)
                done_steps[0] += 1

            done_steps = [0]
            done = self._animate(steps, s)
            if stuck:
                return ("obstacle" if stuck[0] == "obstacle" else "stalled"), None
            return ("succeeded" if done else "canceled"), None

        return self._motion(fn, raw)

    def rotate(self, angle_deg):
        if not -360 <= angle_deg <= 360:
            raise RobotError("'angle_deg' must be between -360 and 360")
        with self._lock:
            self._check_motors()

        def fn():
            steps = 20
            step = math.radians(angle_deg) / steps

            def s():
                self.yaw = _wrap(self.yaw + step)

            return ("succeeded" if self._animate(steps, s) else "canceled"), None

        return self._motion(fn)

    def undock(self):
        def fn():
            with self._lock:
                if not self.docked:
                    return "aborted", False
                self.docked = False

            def back():
                self.x -= 0.35 / 10

            def turn():
                self.yaw = _wrap(self.yaw + math.pi / 10)

            done = self._animate(10, back) and self._animate(10, turn)
            return ("succeeded" if done else "canceled"), False

        return self._motion(fn)

    def dock(self):
        def fn():
            with self._lock:
                if not self._sees_dock():
                    return "aborted", False
            if self.delay:
                time.sleep(self.delay)
            with self._lock:
                self.x = self.y = self.yaw = 0.0
                self.docked = True
            return "succeeded", True

        return self._motion(fn)

    def stop(self):
        self._stop.set()

    def scan(self):
        with self._lock:
            if self.docked:
                raise RobotError("no recent lidar scan (the lidar stops while docked)")
            pose = self._pose()
            pts = [(x, y) for x, y in self.obstacles if math.hypot(x - self.x, y - self.y) < 8.0]
        return pose, pts

    def range_at(self, bearing_deg, width_deg=6.0):
        with self._lock:
            if self.docked:
                raise RobotError("no recent lidar scan (the lidar stops while docked)")
            best = None
            for ox, oy in self.obstacles:
                ahead, side = self._local(ox, oy)
                ang = math.degrees(math.atan2(side, ahead))
                if abs((ang - bearing_deg + 180) % 360 - 180) <= width_deg / 2:
                    d = math.hypot(ahead, side)
                    best = d if best is None else min(best, d)
            return None if best is None else round(best, 3)

    # ---- camera

    def _project(self):
        """Objects in view: [{"label", "box": [x0, y0, x1, y1], "distance"}], boxes clipped to the image."""
        fx, fy, cx, cy = (INTRINSICS[k] for k in ("fx", "fy", "cx", "cy"))
        out = []
        for label, ox, oy in self.objects:
            ahead, left = self._local(ox, oy)
            ahead -= CAMERA["x"]
            if ahead < 0.15:
                continue
            u = cx - fx * left / ahead
            half = fx * OBJECT_RADIUS / ahead
            v_bottom = cy + fy * CAMERA["z"] / ahead
            v_top = cy + fy * (CAMERA["z"] - OBJECT_HEIGHT) / ahead
            x0, x1 = max(u - half, 0), min(u + half, IMAGE_W - 1)
            y0, y1 = max(v_top, 0), min(v_bottom, IMAGE_H - 1)
            if x1 - x0 < 2 or y1 - y0 < 2:
                continue
            out.append({"label": label, "box": [x0, y0, x1, y1], "distance": math.hypot(ahead, left)})
        return out

    def photo(self):
        with self._lock:
            pose = self._pose()
            seen = self._project()
        img = Image.new("RGB", (IMAGE_W, IMAGE_H), (40, 60, 90))
        draw = ImageDraw.Draw(img)
        hue = int((math.degrees(pose.yaw) % 360) / 360 * 200)
        for i in range(0, IMAGE_W, 40):
            draw.line([(i, 0), (i, IMAGE_H)], fill=(60 + hue // 3, 80, 110))
        draw.rectangle([0, INTRINSICS["cy"], IMAGE_W, IMAGE_H], fill=(150, 150, 140))
        for obj in seen:
            draw.rectangle(obj["box"], fill=(200, 50, 50))
            draw.text((obj["box"][0], max(obj["box"][1] - 12, 0)), obj["label"], fill=(255, 255, 255))
        draw.text((20, 20), "SIMULATED TURTLEBOT", fill=(255, 255, 255))
        draw.text(
            (20, 40),
            f"x {pose.x:+.2f} m   y {pose.y:+.2f} m   heading {math.degrees(pose.yaw):+.0f} deg",
            fill=(255, 255, 255),
        )
        buf = io.BytesIO()
        img.save(buf, "PNG")
        depth = Image.new("I;16", (IMAGE_W, IMAGE_H), 0)  # no depth: object search uses the floor geometry
        dbuf = io.BytesIO()
        depth.save(dbuf, "PNG")
        return PhotoData(
            buf.getvalue(),
            dbuf.getvalue(),
            IMAGE_W,
            IMAGE_H,
            "simulator",
            pose,
            intrinsics=dict(INTRINSICS),
            camera=dict(CAMERA),
            sim_objects=seen,
        )
