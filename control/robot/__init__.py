"""Robot access. get_robot() returns the client chosen by settings.ROBOT_BACKEND (one shared instance)."""

import threading

from django.conf import settings

from .base import MotionResult, PhotoData, Pose, RobotBusy, RobotClient, RobotError, RobotUnreachable

__all__ = [
    "MotionResult",
    "PhotoData",
    "Pose",
    "RobotBusy",
    "RobotClient",
    "RobotError",
    "RobotUnreachable",
    "get_robot",
    "set_robot",
]

_robot = None
_lock = threading.Lock()


def get_robot():
    global _robot
    with _lock:
        if _robot is None:
            if settings.ROBOT_BACKEND == "agent":
                from .http import HttpRobotClient

                _robot = HttpRobotClient(settings.ROBOT_AGENT_URL, settings.ROBOT_AGENT_TOKEN)
            elif settings.ROBOT_BACKEND == "sim":
                from .sim import SimRobotClient

                _robot = SimRobotClient(delay=settings.ROBOT_SIM_DELAY)
            else:
                raise RuntimeError(f"ROBOT_BACKEND must be 'agent' or 'sim', not {settings.ROBOT_BACKEND!r}")
        return _robot


def set_robot(robot):
    """Replace the shared client (tests)."""
    global _robot
    with _lock:
        _robot = robot
