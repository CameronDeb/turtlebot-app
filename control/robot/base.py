"""What the web app needs from a robot, independent of how it is reached (real agent or simulator)."""

import math
from dataclasses import dataclass


class RobotError(Exception):
    """The robot refused or failed a request (message is shown to the user)."""


class RobotBusy(RobotError):
    pass


class RobotUnreachable(RobotError):
    pass


@dataclass(frozen=True)
class Pose:
    """Planar pose in the robot's odom frame: metres and radians (yaw 0 = +x, counter-clockwise positive)."""

    x: float
    y: float
    yaw: float

    @classmethod
    def from_dict(cls, d):
        if d is None:
            return None
        return cls(float(d["x"]), float(d["y"]), float(d["yaw"]))

    def as_dict(self):
        return {"x": self.x, "y": self.y, "yaw": self.yaw}

    def distance_to(self, other):
        return math.hypot(other.x - self.x, other.y - self.y)


@dataclass
class MotionResult:
    """Outcome of one motion. outcome is succeeded, canceled or aborted; poses can be None if unknown."""

    outcome: str
    pose_before: Pose | None
    pose_after: Pose | None
    is_docked: bool | None = None
    raw: dict | None = None

    @property
    def succeeded(self):
        return self.outcome == "succeeded"


@dataclass
class PhotoData:
    rgb_png: bytes
    depth_png: bytes | None
    width: int
    height: int
    topic: str
    pose: Pose | None
    # Pinhole intrinsics {"fx", "fy", "cx", "cy"} in pixels, when known.
    intrinsics: dict | None = None
    # Camera mount in base_link: {"x", "y", "z" (m), "yaw_deg", "pitch_deg" (+ = looking down)}, when known.
    camera: dict | None = None
    # Simulator only: ground truth of the objects in view, used by the simulated detector.
    sim_objects: list | None = None


class RobotClient:
    """Interface. Implementations: HttpRobotClient (robot_agent on the TurtleBot) and SimRobotClient."""

    name = "robot"

    def status(self) -> dict:
        """{"connected", "pose": Pose|None, "battery_percent", "docked", "dock_visible", "busy", ...}"""
        raise NotImplementedError

    def drive(self, distance, check_clear=True) -> MotionResult:
        raise NotImplementedError

    def rotate(self, angle_deg) -> MotionResult:
        raise NotImplementedError

    def dock(self) -> MotionResult:
        raise NotImplementedError

    def undock(self) -> MotionResult:
        raise NotImplementedError

    def stop(self) -> None:
        raise NotImplementedError

    def photo(self) -> PhotoData:
        raise NotImplementedError

    def range_at(self, bearing_deg, width_deg=6.0) -> float | None:
        """Nearest lidar return (m from the robot centre) within +-width/2 of a bearing (+ = left), or None."""
        raise NotImplementedError

    def warm_camera(self) -> None:
        """Tell the robot pictures are coming, so it starts receiving camera frames (the agent only subscribes to
        the camera while it is being used)."""

    def scan(self):
        """Current lidar returns for route planning: (pose, [(x, y), ...] in the odom frame)."""
        raise NotImplementedError

    def map_points(self):
        """Obstacles from the robot's learned map near it, [(x, y), ...] in the odom frame ([] if no map yet)."""
        return []

    def motors_on(self) -> dict:
        """Clear the base's e-stop. Returns {"ok", "wheels_enabled", "message"}."""
        raise NotImplementedError

    def pose(self) -> Pose:
        pose = self.status().get("pose")
        if pose is None:
            raise RobotError("robot pose unknown (no odom transform)")
        return pose
