"""Geometry for "return to start / dock".

The trip keeps the robot's pose after every motion (waypoints). Returning retraces them in reverse order: from the
current pose, turn to face the previous waypoint and drive to it, and so on back to the start. That is the user's
moves undone in reverse, but always driving forward (the camera, lidar and cliff sensors face forward, and the
Create 3 refuses long backward drives), and each leg is re-aimed from the measured pose so errors don't pile up.
"""

import math
from dataclasses import dataclass

POSITION_TOLERANCE = 0.05  # m: closer than this counts as "there"
HEADING_TOLERANCE = math.radians(2)
MAX_LEG = 2.0  # m per drive command; longer legs are split and re-aimed


def wrap(a):
    """Angle to (-pi, pi]."""
    a = (a + math.pi) % (2 * math.pi) - math.pi
    return math.pi if a == -math.pi else a


@dataclass(frozen=True)
class Leg:
    turn: float  # radians, + = left
    distance: float  # metres, >= 0


def leg_to(pose, x, y, tolerance=POSITION_TOLERANCE):
    """Turn + drive that takes `pose` to (x, y); None if already there. distance is capped at MAX_LEG."""
    dx, dy = x - pose.x, y - pose.y
    dist = math.hypot(dx, dy)
    if dist < tolerance:
        return None
    return Leg(turn=wrap(math.atan2(dy, dx) - pose.yaw), distance=min(dist, MAX_LEG))


def turn_to(pose, yaw):
    """Turn (radians) to face heading `yaw`; 0.0 if already within tolerance."""
    t = wrap(yaw - pose.yaw)
    return 0.0 if abs(t) < HEADING_TOLERANCE else t


def turn_toward(pose, x, y):
    """Turn (radians) to face the point (x, y)."""
    return turn_to(pose, math.atan2(y - pose.y, x - pose.x))


def return_targets(waypoints):
    """Waypoints to visit on the way home, newest first. The dock waypoint itself is never driven to (the robot
    stops at the undock spot and docks from there)."""
    usable = [w for w in waypoints if w.kind != "dock" and not w.retraced]
    return list(reversed(usable))
