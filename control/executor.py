"""Runs commands on the robot, one at a time, in a background thread.

Every command (button, typed text, later an LLM plan) becomes a Job. The executor runs it step by step, records the
robot's pose after every motion into the active Trip (so the robot can always find its way back), saves pictures
into the archive, and stops between steps as soon as the user presses Stop.
"""

import logging
import math
import threading

from django.db import close_old_connections, connections, transaction
from django.db.models import Max
from django.utils import timezone

from . import planner
from .archive import save_photo
from .commands import describe, validate
from .models import Job, Trip, Waypoint
from .robot import RobotError, get_robot

logger = logging.getLogger(__name__)

MAX_TRIES_PER_TARGET = 4


class Busy(Exception):
    pass


class Canceled(Exception):
    pass


def active_trip():
    return Trip.objects.filter(status=Trip.ACTIVE).order_by("-id").first()


class JobContext:
    """What a command handler can do. Motions go through here so each one is recorded and stoppable."""

    def __init__(self, job, robot, cancel):
        self.job = job
        self.robot = robot
        self.cancel = cancel
        self.result = {}

    def check(self):
        if self.cancel.is_set():
            raise Canceled()

    # ---- trips

    def _trip_start_info(self):
        """Before a recorded motion with no active trip: (pose, docked) to start one from."""
        if active_trip() is not None:
            return None
        status = self.robot.status()
        return status.get("pose"), bool(status.get("docked"))

    def _trip_for(self, start_info, result):
        """The active trip, or a new one starting where the robot was before this motion. A trip only begins once
        the robot has actually moved, so refused commands don't leave empty trips behind."""
        trip = active_trip()
        if trip is not None:
            return trip
        pose, docked = start_info if start_info else (None, False)
        pose = result.pose_before or pose
        if pose is None:
            raise RobotError("robot pose unknown (no odom transform), so moves can't be recorded for the return")
        with transaction.atomic():
            trip = Trip.objects.create(
                started_docked=docked,
                dock_x=pose.x if docked else None,
                dock_y=pose.y if docked else None,
            )
            Waypoint.objects.create(
                trip=trip,
                seq=0,
                kind=Waypoint.DOCK if docked else Waypoint.START,
                x=pose.x,
                y=pose.y,
                yaw=pose.yaw,
                job=self.job,
            )
        return trip

    def record(self, trip, pose, kind=Waypoint.MOVE):
        if pose is None:
            raise RobotError(
                "the robot's pose after the move is unknown, so return-to-start can't use this trip; "
                "check the robot's odometry (TF odom -> base_link)"
            )
        with transaction.atomic():
            last = trip.waypoints.aggregate(m=Max("seq"))["m"]
            Waypoint.objects.create(
                trip=trip,
                seq=0 if last is None else last + 1,
                kind=kind,
                x=pose.x,
                y=pose.y,
                yaw=pose.yaw,
                job=self.job,
            )

    @staticmethod
    def _finish_motion(result, what):
        if result.outcome == "canceled":
            raise Canceled()
        if not result.succeeded:
            raise RobotError(f"{what} didn't finish (robot reported '{result.outcome}': bumper, cliff or wheel stall?)")

    # ---- primitives

    def _motion(self, call, what, record):
        self.check()
        start_info = self._trip_start_info() if record else None
        result = call()
        if record:
            self.record(self._trip_for(start_info, result), result.pose_after)
        self._finish_motion(result, what)
        return result

    def drive(self, distance, check_clear=True, record=True):
        return self._motion(
            lambda: self.robot.drive(distance, check_clear=check_clear), f"driving {distance:.2f} m", record
        )

    def rotate(self, degrees, record=True):
        return self._motion(lambda: self.robot.rotate(degrees), f"turning {degrees:.0f}°", record)

    def photo(self, scan_index=None):
        self.check()
        data = self.robot.photo()
        try:
            photo = save_photo(data, job=self.job, trip=active_trip(), scan_index=scan_index)
        except ValueError as e:
            raise RobotError(str(e)) from None
        self.result.setdefault("photo_ids", []).append(photo.id)
        return photo


# ---------------------------------------------------------------- command handlers


def _moved(result):
    if result.pose_before is None or result.pose_after is None:
        return None
    return result.pose_before.distance_to(result.pose_after)


def do_forward(ctx, p):
    r = ctx.drive(p["distance"], p["check_clear"])
    moved = _moved(r)
    return f"Drove {moved:.2f} m." if moved is not None else "Drove forward."


def do_turn(ctx, p):
    r = ctx.rotate(p["degrees"])
    if r.pose_before and r.pose_after:
        turned = math.degrees(planner.wrap(r.pose_after.yaw - r.pose_before.yaw))
        return f"Turned {abs(turned):.0f}° {'left' if turned >= 0 else 'right'}."
    return "Turned."


def do_photo(ctx, p):
    photo = ctx.photo()
    return f"Saved photo #{photo.id}."


def do_scan(ctx, p):
    steps = p["steps"]
    step_deg = 360.0 / steps
    ids = []
    for i in range(steps):
        ids.append(ctx.photo(scan_index=i).id)
        ctx.rotate(step_deg)
    return f"Scanned 360°: {len(ids)} photos (#{ids[0]}–#{ids[-1]})."


def do_undock(ctx, p):
    status = ctx.robot.status()
    if status.get("docked") is False:
        raise RobotError("the robot is already off the dock")
    pose = status.get("pose")
    if pose is None:
        raise RobotError("robot pose unknown (no odom transform)")
    # Being on the dock means any earlier trip is over; start a new one from here.
    Trip.objects.filter(status=Trip.ACTIVE).update(status=Trip.ENDED, ended_at=timezone.now())
    with transaction.atomic():
        trip = Trip.objects.create(started_docked=True, dock_x=pose.x, dock_y=pose.y)
        Waypoint.objects.create(trip=trip, seq=0, kind=Waypoint.DOCK, x=pose.x, y=pose.y, yaw=pose.yaw, job=ctx.job)
    ctx.check()
    r = ctx.robot.undock()
    if r.pose_after is not None:
        ctx.record(trip, r.pose_after)
    ctx._finish_motion(r, "undocking")
    if r.is_docked:
        raise RobotError("undock finished but the robot still reports it is docked")
    return "Undocked."


def _dock_and_close(ctx, trip):
    r = ctx.robot.dock()
    if r.outcome == "canceled":
        raise Canceled()
    if not (r.succeeded and r.is_docked):
        raise RobotError(
            "docking failed: the robot couldn't see the dock. Drive to within about 1 m of it, facing it, "
            "then press Dock"
        )
    if trip is not None:
        trip.status, trip.ended_at = Trip.DOCKED, timezone.now()
        trip.save(update_fields=["status", "ended_at"])


def do_dock(ctx, p):
    ctx.check()
    _dock_and_close(ctx, active_trip())
    return "Docked."


def do_return_home(ctx, p):
    trip = active_trip()
    if trip is None:
        raise RobotError("nothing to return from: no moves have been recorded since the robot was last home")
    targets = planner.return_targets(trip.path())
    for target in targets:
        # Commit to this target first: if we're stopped on the way, the next Return heads straight here.
        trip.waypoints.filter(seq__gt=target.seq, retraced=False).update(retraced=True)
        for _ in range(MAX_TRIES_PER_TARGET):
            ctx.check()
            pose = ctx.robot.pose()
            leg = planner.leg_to(pose, target.x, target.y)
            if leg is None:
                break
            if abs(leg.turn) >= planner.HEADING_TOLERANCE:
                ctx.rotate(math.degrees(leg.turn), record=False)
            ctx.drive(leg.distance, check_clear=p["check_clear"], record=False)
        else:
            pose = ctx.robot.pose()
            if planner.leg_to(pose, target.x, target.y) is not None:
                raise RobotError(
                    f"couldn't reach waypoint {target.seq} at ({target.x:.2f}, {target.y:.2f}); "
                    f"stopped at ({pose.x:.2f}, {pose.y:.2f})"
                )
    pose = ctx.robot.pose()
    if trip.started_docked:
        turn = planner.turn_toward(pose, trip.dock_x, trip.dock_y)
        if turn:
            ctx.rotate(math.degrees(turn), record=False)
        ctx.check()
        _dock_and_close(ctx, trip)
        return "Back on the dock."
    home = targets[-1] if targets else None
    if home is not None:
        turn = planner.turn_to(pose, home.yaw)
        if turn:
            ctx.rotate(math.degrees(turn), record=False)
    trip.status, trip.ended_at = Trip.RETURNED, timezone.now()
    trip.save(update_fields=["status", "ended_at"])
    return "Back at the start, facing the way it started."


def do_sequence(ctx, p):
    done = []
    for i, step in enumerate(p["steps"], 1):
        ctx.check()
        try:
            msg = HANDLERS[step["kind"]](ctx, step["params"])
        except RobotError as e:
            raise RobotError(f"step {i} ({describe(step['kind'], step['params'])}): {e}") from None
        done.append(msg)
    return " ".join(done)


HANDLERS = {
    "forward": do_forward,
    "turn": do_turn,
    "photo": do_photo,
    "scan": do_scan,
    "undock": do_undock,
    "dock": do_dock,
    "return_home": do_return_home,
    "sequence": do_sequence,
}


# ---------------------------------------------------------------- executor


class Executor:
    def __init__(self):
        self._lock = threading.Lock()
        self._job_id = None
        self._cancel = threading.Event()
        self._cleaned = False
        self.run_inline = False  # tests: run jobs in the calling thread

    def _cleanup_once(self):
        """Jobs left 'running' by a previous server process can't be running any more."""
        if self._cleaned:
            return
        self._cleaned = True
        Job.objects.filter(state=Job.RUNNING).exclude(id=self._job_id).update(
            state=Job.FAILED,
            message="Interrupted: the web server restarted while this was running.",
            finished_at=timezone.now(),
        )

    @property
    def current_job_id(self):
        return self._job_id

    def submit(self, kind, params=None, source="button", text=""):
        """Validate and start a command. Raises CommandError (bad input) or Busy."""
        params = validate(kind, params or {})
        with self._lock:
            self._cleanup_once()
            if self._job_id is not None:
                raise Busy("the robot is still running the previous command; wait or press Stop")
            job = Job.objects.create(kind=kind, params=params, source=source, text=text)
            self._job_id = job.id
            self._cancel.clear()
        if self.run_inline:
            self._run(job.id, inline=True)
            job.refresh_from_db()
        else:
            threading.Thread(target=self._run, args=(job.id,), name=f"job-{job.id}", daemon=True).start()
        return job

    def stop(self):
        """Stop the current command (and any motion) right away."""
        self._cancel.set()
        try:
            get_robot().stop()
        except RobotError as e:
            logger.warning("stop: %s", e)
            return str(e)
        return None

    def _run(self, job_id, inline=False):
        if not inline:
            close_old_connections()
        job = Job.objects.get(id=job_id)
        ctx = JobContext(job, get_robot(), self._cancel)
        try:
            job.message = HANDLERS[job.kind](ctx, job.params) or ""
            job.state = Job.SUCCEEDED
        except Canceled:
            job.state, job.message = Job.CANCELED, "Stopped."
        except RobotError as e:
            text = str(e)
            job.state, job.message = Job.FAILED, text[:1].upper() + text[1:]
        except Exception as e:  # never leave a job 'running' forever
            logger.exception("job %s failed", job_id)
            job.state, job.message = Job.FAILED, f"Internal error: {e!r}"
        finally:
            job.result = ctx.result
            job.finished_at = timezone.now()
            job.save(update_fields=["state", "message", "result", "finished_at"])
            with self._lock:
                self._job_id = None
            if not inline:
                connections.close_all()


executor = Executor()
