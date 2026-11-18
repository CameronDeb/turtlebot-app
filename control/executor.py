"""Runs commands on the robot, one at a time, in a background thread.

Every command (button, typed text, later an LLM plan) becomes a Job. The executor runs it step by step, records the
robot's pose after every motion into the active Trip (so the robot can always find its way back), saves pictures
into the archive, and stops between steps as soon as the user presses Stop.
"""

import logging
import math
import threading
import time

from django.db import close_old_connections, connections, transaction
from django.db.models import Max
from django.utils import timezone

from . import planner, vision
from .archive import save_photo
from .commands import describe, validate
from .detector import DetectorError, get_detector
from .models import Job, Trip, Waypoint
from .robot import RobotError, get_robot

logger = logging.getLogger(__name__)

MAX_TRIES_PER_TARGET = 4

# Object search
BODY_FRONT = 0.17  # bumper distance from the robot centre (m)
FIND_MAX_LEG = 1.0  # drive at most this far before looking again
FIND_MAX_LEGS = 12
FIND_CENTER_DEG = 2.5  # turn toward the object when it is further off-centre than this
FIND_UNKNOWN_STEP = 0.30  # leg length when the distance can't be estimated
FIND_NEAR = 0.9  # m: an object lost from view closer than this has gone below the camera's view


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
        if result.outcome == "obstacle":
            return  # stopped by the lidar watch: the caller decides what that means
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
        return self.save(self.robot.photo(), scan_index=scan_index)

    def save(self, data, scan_index=None, detections=None):
        try:
            photo = save_photo(data, job=self.job, trip=active_trip(), scan_index=scan_index, detections=detections)
        except ValueError as e:
            raise RobotError(str(e)) from None
        self.result.setdefault("photo_ids", []).append(photo.id)
        return photo

    def look(self, query, scan_index=None):
        """Take a picture, find `query` in it, locate each hit, archive the picture with its detections."""
        self.check()
        data = self.robot.photo()
        try:
            dets = get_detector().detect(data, query)
        except DetectorError as e:
            raise RobotError(str(e)) from None
        located = [{**d, **vision.locate(d, data, range_at=self.robot.range_at)} for d in dets]
        photo = self.save(data, scan_index=scan_index, detections=located)
        return photo, data, located


# ---------------------------------------------------------------- command handlers


def _moved(result):
    if result.pose_before is None or result.pose_after is None:
        return None
    return result.pose_before.distance_to(result.pose_after)


def do_forward(ctx, p):
    r = ctx.drive(p["distance"], p["check_clear"])
    moved = _moved(r)
    raw = r.raw or {}
    done = f"Drove {moved:.2f} m" if moved is not None else "Drove forward"
    if r.outcome == "obstacle":
        return f"{done}, then stopped: something came right up to the front of the robot."
    if raw.get("shortened"):
        gap = raw.get("obstacle_gap")
        where = f" {gap:.2f} m ahead" if gap is not None else ""
        return f"{done} of the {p['distance']:g} m asked, stopping short of an obstacle{where}."
    return done + "."


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


def _pick(dets):
    """The detection to go for: among the strong ones (>= 75% of the best score), the most central."""
    if not dets:
        return None
    top = max(d["score"] for d in dets)
    strong = [d for d in dets if d["score"] >= 0.75 * top]
    return min(strong, key=lambda d: abs(d["bearing"]))


def _ids(ctx):
    ids = ctx.result.get("photo_ids", [])
    if not ids:
        return ""
    return f" (photo #{ids[0]})" if len(ids) == 1 else f" (photos #{ids[0]}–#{ids[-1]})"


def _fill_distance(det, expected):
    """When the picture can't give a distance (or only a doubtful one), fall back on dead reckoning."""
    if expected is None:
        return det
    if det["distance"] is None:
        return {**det, "distance": expected}
    if det["cut_off_bottom"] and "depth" not in det["cues"]:
        return {**det, "distance": min(det["distance"], expected)}
    return det


SENSOR_WAIT = 30.0  # s; the TurtleBot's power saver stops the camera and lidar on the dock, and they take a while


def _wait_for_sensors(ctx, timeout=SENSOR_WAIT):
    """After undocking, wait until the lidar and a camera are streaming again."""
    end = time.monotonic() + timeout
    while True:
        ctx.check()
        s = ctx.robot.status()
        cams = s.get("camera") or {}
        cam_ok = any(age is not None and age < 1.0 for age in cams.values())
        scan_ok = s.get("scan_age") is not None and s["scan_age"] < 1.0
        if cam_ok and scan_ok:
            return
        if time.monotonic() > end:
            missing = " and ".join(n for n, ok in (("camera", cam_ok), ("lidar", scan_ok)) if not ok)
            raise RobotError(f"the {missing} didn't start within {timeout:.0f} s of leaving the dock")
        time.sleep(0.5)


def do_find(ctx, p):
    """Look around for an object (open-vocabulary detection) and, if asked, drive up to it."""
    query, standoff = p["object"], p["standoff"]
    if ctx.robot.status().get("docked"):
        do_undock(ctx, {})
    _wait_for_sensors(ctx)

    # 1. Search: pictures all the way round, overlapping, until the object shows up.
    photo, data, dets = ctx.look(query, scan_index=0)
    step = max(20.0, min(60.0, vision.horizontal_fov_deg(data) * 0.8))
    n = math.ceil(360.0 / step)
    step = 360.0 / n
    looked = 1
    while not dets:
        if looked >= n:
            raise RobotError(
                f"couldn't see \"{query}\" anywhere around the robot{_ids(ctx)}. Try another spot or describe "
                "it differently (e.g. \"blue cup\" instead of \"mug\")"
            )
        ctx.rotate(step)
        photo, data, dets = ctx.look(query, scan_index=looked)
        looked += 1
    target = _pick(dets)
    seen = f"Saw \"{query}\" (score {target['score']:.2f}) in photo #{photo.id}"

    if not p["approach"]:
        if abs(target["bearing"]) > FIND_CENTER_DEG:
            ctx.rotate(target["bearing"])
        far = f", about {target['distance']:.1f} m away" if target["distance"] else ""
        return f"{seen}{far}, and turned to face it."

    # 2. Approach: turn to it, drive part of the way, look again; repeat.
    travelled, lost, last = 0.0, 0, target
    for _ in range(FIND_MAX_LEGS):
        ctx.check()
        if abs(last["bearing"]) > FIND_CENTER_DEG:
            expected = last["distance"]
            ctx.rotate(last["bearing"])
            photo, data, dets = ctx.look(query)
            cand = _pick(dets)
            if cand is None:
                lost += 1
                if lost >= 2:
                    raise RobotError(f"lost sight of the {query} after turning toward it{_ids(ctx)}")
                last = {**last, "bearing": 0.0}
            else:
                lost, last = 0, _fill_distance(cand, expected)
        dist = last["distance"]
        gap = None if dist is None else dist - BODY_FRONT
        move = FIND_UNKNOWN_STEP if gap is None else gap - standoff
        if gap is not None and move < 0.05:
            return f"{seen}. Drove {travelled:.2f} m and stopped about {max(gap, 0):.2f} m in front of it{_ids(ctx)}."
        move = min(move, FIND_MAX_LEG, p["max_range"] - travelled)
        if move < 0.05:
            raise RobotError(f"drove {travelled:.2f} m (the search limit) without reaching the {query}{_ids(ctx)}")
        r = ctx.drive(move, check_clear=True)
        moved = _moved(r)
        moved = move if moved is None else moved
        travelled += moved
        raw = r.raw or {}
        if r.outcome == "obstacle":
            raise RobotError(
                f"stopped after {travelled:.2f} m: something came right up to the front of the robot{_ids(ctx)}"
            )
        if raw.get("shortened") and moved < move - 0.03:
            ob = raw.get("obstacle_gap")
            if gap is not None and ob is not None and abs(ob - gap) < 0.25:
                # The lidar sees the object itself (it is tall enough): this is as close as is safe.
                return (
                    f"{seen}. Drove {travelled:.2f} m and stopped about {max(ob - moved, 0):.2f} m in front of "
                    f"it{_ids(ctx)}."
                )
            where = f"{ob:.2f} m ahead" if ob is not None else "ahead"
            raise RobotError(
                f"something is in the way ({where}) between the robot and the {query}; stopped after "
                f"{travelled:.2f} m. Steer around it and run Find again{_ids(ctx)}"
            )
        remaining = None if dist is None else dist - moved
        photo, data, dets = ctx.look(query)
        cand = _pick(dets)
        if cand is None:
            if remaining is not None and remaining - BODY_FRONT < FIND_NEAR:
                # Close and no longer in the picture: it has gone below the camera's view. Finish on dead reckoning.
                creep = min(remaining - BODY_FRONT - standoff, 0.5)
                if creep > 0.05:
                    r = ctx.drive(creep, check_clear=True)
                    crept = _moved(r)
                    crept = creep if crept is None else crept
                    travelled += crept
                    remaining -= crept
                return (
                    f"{seen}. Drove {travelled:.2f} m; it is now below the camera's view, about "
                    f"{max(remaining - BODY_FRONT, 0):.2f} m in front of the bumper{_ids(ctx)}."
                )
            lost += 1
            if lost >= 2:
                raise RobotError(f"lost sight of the {query} after driving {travelled:.2f} m{_ids(ctx)}")
            last = {**last, "bearing": 0.0, "distance": remaining}
            continue
        lost, last = 0, _fill_distance(cand, remaining)
    raise RobotError(f"didn't reach the {query} after {FIND_MAX_LEGS} legs ({travelled:.2f} m){_ids(ctx)}")


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
    "find": do_find,
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
