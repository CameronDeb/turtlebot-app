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

from . import pathplan, planner, vision
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
FIND_MAX_LEGS = 40  # backstop; the real limit is FIND_NO_PROGRESS
FIND_CENTER_DEG = 2.5  # turn toward the object when it is further off-centre than this
FIND_UNKNOWN_STEP = 0.30  # leg length when the distance can't be estimated
FIND_NEAR = 0.9  # m: an object lost from view closer than this has gone below the camera's view
FIND_MAX_DODGES = 8  # re-plans after something appears in the path
FIND_NO_PROGRESS = 4  # legs in a row without getting FIND_PROGRESS closer to the object -> give up
FIND_PROGRESS = 0.15  # m
NO_GO_RADIUS = 0.12  # m: a spot where the robot got blocked is planned around as if it were an obstacle this big


# Motions stopped by the agent's guards (outcome -> what to tell the user)
GUARD_STOPS = {
    "stalled": "the robot stopped making progress, probably pushing against something low that the lidar can't "
    "see (a chair base, a cable). Back off or turn away before trying again",
    "bumped": "the bumper hit something",
    "cliff": "a cliff sensor saw a drop",
    "wheel_drop": "a wheel dropped (lifted or over an edge)",
}


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
        self.stuck_spots = []  # places the robot got stuck this job: never retraced by Return

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
        stuck_here = kind == Waypoint.MOVE and any(
            math.hypot(pose.x - x, pose.y - y) < r for x, y, r in self.stuck_spots
        )
        with transaction.atomic():
            last = trip.waypoints.aggregate(m=Max("seq"))["m"]
            Waypoint.objects.create(
                trip=trip,
                seq=0 if last is None else last + 1,
                retraced=stuck_here,
                kind=kind,
                x=pose.x,
                y=pose.y,
                yaw=pose.yaw,
                job=self.job,
            )

    @staticmethod
    def _finish_motion(result, what, allow=()):
        if result.outcome == "canceled":
            raise Canceled()
        if result.outcome == "obstacle" or result.outcome in allow:
            return  # stopped by the lidar watch (or an allowed guard): the caller decides what that means
        if result.outcome in GUARD_STOPS:
            moved = _moved(result)
            after = f" after {moved:.2f} m" if moved is not None and moved >= 0.01 else ""
            raise RobotError(f"{what} stopped{after}: {GUARD_STOPS[result.outcome]}")
        if not result.succeeded:
            raise RobotError(f"{what} didn't finish (robot reported '{result.outcome}': bumper, cliff or wheel stall?)")

    # ---- primitives

    def _motion(self, call, what, record, allow=()):
        self.check()
        start_info = self._trip_start_info() if record else None
        result = call()
        if record:
            self.record(self._trip_for(start_info, result), result.pose_after)
        self._finish_motion(result, what, allow)
        return result

    def drive(self, distance, check_clear=True, record=True, allow=()):
        return self._motion(
            lambda: self.robot.drive(distance, check_clear=check_clear), f"driving {distance:.2f} m", record, allow
        )

    def forget_spot(self, xy, radius=0.3):
        """Drop recorded waypoints near a spot where the robot got stuck, so Return doesn't drive back into it."""
        self.stuck_spots.append((xy[0], xy[1], radius))
        trip = active_trip()
        if trip is None:
            return
        for w in trip.waypoints.filter(kind=Waypoint.MOVE, retraced=False):
            if math.hypot(w.x - xy[0], w.y - xy[1]) < radius:
                w.retraced = True
                w.save(update_fields=["retraced"])

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
        return f"{done}, then stopped: something appeared in its path."
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
        ctx.robot.warm_camera()
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

    # 2. Approach along a planned route. It keeps following a route while it stays clear, remembers every spot
    # where it got blocked (and plans around those from then on), and gives up only when it stops getting closer.
    goal = _to_odom(target, data.pose or ctx.robot.pose())
    if goal is None:
        raise RobotError(f"{seen}, but couldn't tell how far away it is{_ids(ctx)}")
    hfov = vision.horizontal_fov_deg(data)
    r_goal = standoff + BODY_FRONT
    travelled, misses, legs, dodges, stalls = 0.0, 0, 0, 0, 0
    no_go = []  # (x, y) spots where the robot got blocked: planned around for the rest of this search
    seen_here = {}  # everything the lidar has seen during this search (5 cm cells): stops it flip-flopping between
    # two routes when an obstacle is visible from one spot but hidden from the other
    route, route_goal = None, None
    boxed_in = False  # no way to the standoff distance: heading for the closest reachable spot instead
    best = math.inf  # closest it has been to the object
    no_progress = 0
    for _ in range(FIND_MAX_LEGS):
        ctx.check()
        pose, points = ctx.robot.scan()
        for x, y in points:
            seen_here.setdefault((round(x / 0.05), round(y / 0.05)), (x, y))
        # what the robot sees now + saw earlier in this search + the learned map (minus what the lidar now sees
        # through) + where it got blocked
        learned = pathplan.drop_seen_through((pose.x, pose.y), points, ctx.robot.map_points())
        points = list(seen_here.values()) + learned + _no_go_points(no_go)
        start_xy = (pose.x, pose.y)
        keep = (
            route
            and route_goal is not None
            and math.dist(route_goal, goal) < 0.5
            and pathplan.still_clear(start_xy, route, points)
        )
        if not keep:
            route = pathplan.plan(start_xy, goal, r_goal, points)
            route_goal = goal
            if route is None:
                # Boxed in (furniture all round it, or places it got stuck): go as close as it safely can.
                closest = pathplan.plan_closest(start_xy, goal, points)
                here = math.dist(start_xy, goal)
                if closest is None or not closest[0] or closest[1] > here - 0.1:
                    if boxed_in or legs:
                        boxed_in = True
                        break  # already as close as it can get
                    raise RobotError(
                        f"{seen}, but there's no clear way to it: furniture, gaps narrower than about 0.6 m, or "
                        f"places it already got stuck are in the way{_ids(ctx)}"
                    )
                route, boxed_in = closest[0], True
        if not route:
            break
        budget = min(FIND_MAX_LEG, p["max_range"] - travelled)
        if budget < 0.05:
            raise RobotError(f"drove {travelled:.2f} m (the search limit) without reaching the {query}{_ids(ctx)}")
        legs += 1
        arrived = False
        while route and budget >= 0.05:
            wx, wy = route[0]
            pose = ctx.robot.pose()
            turn = planner.wrap(math.atan2(wy - pose.y, wx - pose.x) - pose.yaw)
            if abs(math.degrees(turn)) > 3.0:
                ctx.rotate(math.degrees(turn))
            remaining = math.hypot(wx - pose.x, wy - pose.y)
            step = min(remaining, budget)
            if step < 0.03:
                route.pop(0)
                continue
            r = ctx.drive(step, check_clear=True, allow=("stalled",))
            moved = _moved(r)
            moved = step if moved is None else moved
            travelled += moved
            budget -= moved
            raw = r.raw or {}
            blocked_at = None
            if r.outcome == "stalled":
                stalls += 1  # pushing against something the lidar can't see: mark it, back away, plan around it
                if stalls > 3:
                    raise RobotError(f"got stuck {stalls} times on the way to the {query}{_ids(ctx)}")
                blocked_at = _ahead(r.pose_after or ctx.robot.pose(), BODY_FRONT + 0.05)
            elif r.outcome == "obstacle":
                dodges += 1  # something appeared in the path: stopped short of it; re-plan around it
                if dodges > FIND_MAX_DODGES:
                    raise RobotError(
                        f"kept finding new obstacles in the way to the {query}; stopped after {travelled:.2f} m"
                        f"{_ids(ctx)}"
                    )
            elif raw.get("shortened") and moved < step - 0.03:
                gap = raw.get("obstacle_gap")
                # it stopped 0.10 m short of what the lidar saw in the path
                blocked_at = _ahead(r.pose_after or ctx.robot.pose(), BODY_FRONT + 0.10)
            if blocked_at is not None and math.dist(blocked_at, goal) < r_goal + 0.3:
                arrived = True  # that's the object itself, seen by the lidar: as close as it safely gets
                break
            if blocked_at is not None:
                no_go.append(blocked_at)
            if r.outcome != "succeeded" or blocked_at is not None:
                route = None  # re-plan from a fresh scan, around the new no-go spot
                if r.outcome == "stalled":
                    ctx.rotate(180.0)  # turn away from what it is stuck on (it can't see it to drive round it)
                    stuck_pose = ctx.robot.pose()
                    ctx.forget_spot((stuck_pose.x, stuck_pose.y))
                break
            if step >= remaining - 0.03:
                route.pop(0)
        if arrived:
            break
        # Progress: closer to the object than ever before?
        pose = ctx.robot.pose()
        dist = math.hypot(goal[0] - pose.x, goal[1] - pose.y)
        if dist < best - FIND_PROGRESS:
            best, no_progress = dist, 0
        else:
            no_progress += 1
            if no_progress >= FIND_NO_PROGRESS:
                raise RobotError(
                    f"stopped making progress toward the {query} ({dist - BODY_FRONT:.1f} m away after "
                    f"{travelled:.1f} m of driving); it may be boxed in by furniture{_ids(ctx)}"
                )
        # Look again when the object should be visible (not when it's already below the camera's view).
        if dist - BODY_FRONT > FIND_NEAR:
            bearing = math.degrees(planner.wrap(math.atan2(goal[1] - pose.y, goal[0] - pose.x) - pose.yaw))
            if abs(bearing) > hfov * 0.35:
                ctx.rotate(bearing)
            photo, data, dets = ctx.look(query)
            match = _match(dets, data.pose or ctx.robot.pose(), goal)
            if match is not None:
                misses = 0
                new_goal = _to_odom(match, data.pose or ctx.robot.pose())
                if new_goal is not None:
                    # blend: one noisy sighting shouldn't throw the route away; closer views count for more
                    w = 0.7 if match["score"] >= 0.6 and dist < 2.0 else 0.4
                    goal = (goal[0] + w * (new_goal[0] - goal[0]), goal[1] + w * (new_goal[1] - goal[1]))
            else:
                misses += 1  # hidden behind something on the way round: keep going to where it was seen
                if misses >= 6:
                    raise RobotError(f"lost sight of the {query} after driving {travelled:.2f} m{_ids(ctx)}")
    else:
        raise RobotError(f"didn't reach the {query} after {FIND_MAX_LEGS} legs ({travelled:.2f} m){_ids(ctx)}")

    # 3. Face it and take a last picture for the record.
    pose = ctx.robot.pose()
    bearing = math.degrees(planner.wrap(math.atan2(goal[1] - pose.y, goal[0] - pose.x) - pose.yaw))
    if abs(bearing) > FIND_CENTER_DEG:
        ctx.rotate(bearing)
    pose = ctx.robot.pose()
    extra = math.hypot(goal[0] - pose.x, goal[1] - pose.y) - BODY_FRONT - standoff
    if extra > 0.04 and not boxed_in:  # the route ends on a 5 cm grid: close the last few centimetres straight in
        try:
            r = ctx.drive(min(extra, 0.3), check_clear=True, allow=("stalled",))
            travelled += _moved(r) or 0.0
        except RobotError:
            pass  # no room (the lidar sees the object itself): this is as close as it gets
    ctx.photo()
    pose = ctx.robot.pose()
    gap = max(math.hypot(goal[0] - pose.x, goal[1] - pose.y) - BODY_FRONT, 0.0)
    how = "" if legs <= 1 else f" along a planned route ({legs} legs)"
    if boxed_in:
        return (
            f"{seen}. It's boxed in by furniture, so it drove {travelled:.2f} m{how} to the closest spot it could "
            f"safely reach, about {gap:.2f} m from it, and is facing it{_ids(ctx)}."
        )
    return f"{seen}. Drove {travelled:.2f} m{how} and stopped about {gap:.2f} m in front of it{_ids(ctx)}."


def _ahead(pose, d):
    return pose.x + d * math.cos(pose.yaw), pose.y + d * math.sin(pose.yaw)


def _no_go_points(spots):
    """Each no-go spot as a small ring of points, so the planner's keep-out closes it off."""
    pts = []
    for x, y in spots:
        pts.append((x, y))
        pts.extend(
            (x + NO_GO_RADIUS * math.cos(a * math.pi / 4), y + NO_GO_RADIUS * math.sin(a * math.pi / 4))
            for a in range(8)
        )
    return pts


def _to_odom(det, pose):
    """A located detection (bearing deg, distance m from base_link) -> (x, y) in the odom frame, or None."""
    if det.get("distance") is None or pose is None:
        return None
    a = pose.yaw + math.radians(det["bearing"])
    return pose.x + det["distance"] * math.cos(a), pose.y + det["distance"] * math.sin(a)


def _match(dets, pose, goal):
    """The detection that is (probably) the object we are heading for, or None."""
    dist = math.hypot(goal[0] - pose.x, goal[1] - pose.y)
    radius = max(0.6, 0.35 * dist)
    best = None
    for d in dets:
        xy = _to_odom(d, pose)
        if xy is None:
            ang = math.degrees(planner.wrap(math.atan2(goal[1] - pose.y, goal[0] - pose.x) - pose.yaw))
            ok = abs(ang - d["bearing"]) < 10.0  # no distance: accept if it is in the right direction
        else:
            ok = math.dist(xy, goal) <= radius
        if ok and (best is None or d["score"] > best["score"]):
            best = d
    return best


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
