"""In-process simulated TurtleBot, for trying the app without the robot and for tests.

Matches the real robot's behaviour where the app depends on it: starts docked at the origin facing the dock (+x);
undock backs up 0.35 m and turns 180 deg; dock only works from within 1 m while facing the dock; the lidar check
fails while docked; a motion can be stopped part-way.
"""

import io
import math
import threading
import time

from PIL import Image, ImageDraw

from .base import MotionResult, PhotoData, Pose, RobotBusy, RobotClient, RobotError


def _wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


class SimRobotClient(RobotClient):
    name = "sim"

    def __init__(self, delay=1.0, obstacles=()):
        self.delay = delay  # seconds per motion
        self.obstacles = list(obstacles)  # (x, y) points the clear-ahead check looks for
        self._lock = threading.Lock()
        self._busy = threading.Lock()
        self._stop = threading.Event()
        self.x = self.y = self.yaw = 0.0
        self.docked = True

    # ---- state

    def _pose(self):
        return Pose(round(self.x, 4), round(self.y, 4), round(self.yaw, 5))

    def _sees_dock(self):
        if self.docked:
            return True
        bearing = math.atan2(-self.y, -self.x)
        return math.hypot(self.x, self.y) < 1.0 and abs(_wrap(bearing - self.yaw)) < math.radians(30)

    def status(self):
        with self._lock:
            return {
                "connected": True,
                "pose": self._pose(),
                "pose_error": None,
                "battery_percent": 87.0,
                "docked": self.docked,
                "dock_visible": self._sees_dock(),
                "busy": self._busy.locked(),
                "active": None,
                "scan_age": None if self.docked else 0.1,
                "camera": {"rgbd_age": 0.1, "depth_age": 0.1, "stock_age": None},
            }

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

    def _motion(self, fn):
        if not self._busy.acquire(blocking=False):
            raise RobotBusy("robot is busy")
        try:
            self._stop.clear()
            with self._lock:
                before = self._pose()
            outcome, docked = fn()
            with self._lock:
                after = self._pose()
            return MotionResult(outcome, before, after, is_docked=docked)
        finally:
            self._busy.release()

    def drive(self, distance, check_clear=True):
        if not 0 < distance <= 3.0:
            raise RobotError("'distance' must be between 0.01 and 3.0")
        if check_clear:
            with self._lock:
                if self.docked:
                    raise RobotError(
                        "no recent lidar scan, so the path can't be checked (the lidar stops while docked: "
                        "undock first, or turn off the obstacle check)"
                    )
                for ox, oy in self.obstacles:
                    dx, dy = ox - self.x, oy - self.y
                    ahead = dx * math.cos(self.yaw) + dy * math.sin(self.yaw)
                    side = -dx * math.sin(self.yaw) + dy * math.cos(self.yaw)
                    if abs(side) < 0.21 and 0.17 < ahead < 0.17 + distance + 0.15:
                        raise RobotError(f"path blocked: obstacle {ahead - 0.17:.2f} m ahead of the bumper")

        def fn():
            steps = 20
            step = distance / steps

            def s():
                self.docked = False
                self.x += step * math.cos(self.yaw)
                self.y += step * math.sin(self.yaw)

            return ("succeeded" if self._animate(steps, s) else "canceled"), None

        return self._motion(fn)

    def rotate(self, angle_deg):
        if not -360 <= angle_deg <= 360:
            raise RobotError("'angle_deg' must be between -360 and 360")

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

    # ---- camera

    def photo(self):
        with self._lock:
            pose = self._pose()
        w, h = 640, 360
        img = Image.new("RGB", (w, h), (40, 60, 90))
        draw = ImageDraw.Draw(img)
        hue = int((math.degrees(pose.yaw) % 360) / 360 * 200)
        for i in range(0, w, 40):
            draw.line([(i, 0), (i, h)], fill=(60 + hue // 3, 80, 110))
        draw.rectangle([0, h * 2 // 3, w, h], fill=(150, 150, 140))
        draw.text((20, 20), "SIMULATED TURTLEBOT", fill=(255, 255, 255))
        draw.text(
            (20, 40),
            f"x {pose.x:+.2f} m   y {pose.y:+.2f} m   heading {math.degrees(pose.yaw):+.0f} deg",
            fill=(255, 255, 255),
        )
        buf = io.BytesIO()
        img.save(buf, "PNG")
        depth = Image.new("I;16", (w, h), 2000)
        dbuf = io.BytesIO()
        depth.save(dbuf, "PNG")
        return PhotoData(buf.getvalue(), dbuf.getvalue(), w, h, "simulator", pose)
