"""Client for robot_agent/agent.py running on the TurtleBot (JSON over HTTP, stdlib only)."""

import base64
import json
import math
import socket
import urllib.error
import urllib.parse
import urllib.request

from .base import MotionResult, PhotoData, Pose, RobotBusy, RobotClient, RobotError, RobotUnreachable

DRIVE_SPEED = 0.15  # m/s
ROTATE_SPEED = 0.5  # rad/s


class HttpRobotClient(RobotClient):
    name = "agent"

    def __init__(self, base_url, token, timeout=5.0):
        self.base_url = base_url.rstrip("/")
        self.token = token
        self.timeout = timeout

    def _request(self, method, path, body=None, timeout=None):
        data = None if body is None else json.dumps(body).encode()
        req = urllib.request.Request(
            self.base_url + path,
            data=data,
            method=method,
            headers={"X-Agent-Token": self.token, "Content-Type": "application/json"},
        )
        try:
            with urllib.request.urlopen(req, timeout=timeout or self.timeout) as resp:
                return json.loads(resp.read())
        except urllib.error.HTTPError as e:
            try:
                message = json.loads(e.read()).get("error") or str(e)
            except ValueError:
                message = str(e)
            if e.code == 409 and "busy" in message:
                raise RobotBusy(message) from None
            raise RobotError(message) from None
        except (urllib.error.URLError, socket.timeout, TimeoutError, ConnectionError) as e:
            reason = getattr(e, "reason", e)
            raise RobotUnreachable(f"can't reach the robot agent at {self.base_url} ({reason})") from None
        except ValueError:
            raise RobotError("the robot agent sent a reply that isn't JSON") from None

    def status(self):
        d = self._request("GET", "/status", timeout=3.0)
        return {
            "connected": True,
            "pose": Pose.from_dict(d.get("pose")),
            "pose_error": d.get("pose_error"),
            "battery_percent": d.get("battery_percent"),
            "docked": d.get("docked"),
            "dock_visible": d.get("dock_visible"),
            "wheels_enabled": d.get("wheels_enabled"),
            "mapping": d.get("mapping"),
            "busy": d.get("busy", False),
            "active": d.get("active"),
            "scan_age": d.get("scan_age"),
            "camera": d.get("camera", {}),
        }

    @staticmethod
    def _motion(d):
        return MotionResult(
            outcome=d.get("outcome", "aborted"),
            pose_before=Pose.from_dict(d.get("pose_before")),
            pose_after=Pose.from_dict(d.get("pose_after")),
            is_docked=d.get("is_docked"),
            raw=d,
        )

    def drive(self, distance, check_clear=True):
        body = {"distance": distance, "speed": DRIVE_SPEED, "check_clear": check_clear}
        return self._motion(self._request("POST", "/drive", body, timeout=distance / DRIVE_SPEED * 2 + 30))

    def rotate(self, angle_deg):
        rad = math.radians(abs(angle_deg))
        body = {"angle_deg": angle_deg, "speed": ROTATE_SPEED}
        return self._motion(self._request("POST", "/rotate", body, timeout=rad / ROTATE_SPEED * 2 + 30))

    def dock(self):
        return self._motion(self._request("POST", "/dock", {}, timeout=150))

    def undock(self):
        return self._motion(self._request("POST", "/undock", {}, timeout=150))

    def stop(self):
        self._request("POST", "/stop", {}, timeout=3.0)

    def photo(self):
        d = self._request("GET", "/photo", timeout=20.0)
        return PhotoData(
            rgb_png=base64.b64decode(d["rgb_png"]),
            depth_png=base64.b64decode(d["depth_png"]) if d.get("depth_png") else None,
            width=int(d["width"]),
            height=int(d["height"]),
            topic=d.get("topic", ""),
            pose=Pose.from_dict(d.get("pose")),
            intrinsics=d.get("intrinsics"),
            camera=d.get("camera"),
        )

    def range_at(self, bearing_deg, width_deg=6.0):
        q = urllib.parse.urlencode({"bearing": f"{bearing_deg:.2f}", "width": f"{width_deg:.2f}"})
        return self._request("GET", f"/range?{q}", timeout=5.0).get("range")

    def warm_camera(self):
        self._request("GET", "/status?camera=1", timeout=3.0)

    def scan(self):
        d = self._request("GET", "/scan", timeout=5.0)
        pose = Pose.from_dict(d["pose"])
        c, s = math.cos(pose.yaw), math.sin(pose.yaw)
        pts = [(pose.x + c * x - s * y, pose.y + s * x + c * y) for x, y in d.get("points", [])]
        return pose, pts

    def map_points(self):
        try:
            d = self._request("GET", "/map", timeout=5.0)
        except RobotError:
            return []  # an agent without mapping, or no map yet: plan from the live scan alone
        return [tuple(p) for p in d.get("points", [])]

    def motors_on(self):
        return self._request("POST", "/motors", {"enabled": True}, timeout=15.0)
