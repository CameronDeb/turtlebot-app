"""Object memory: where the robot has seen things, kept between sessions (B1 of the verification-run plan).

Every located detection the robot makes (Find, or a 360 deg scan with a list of objects) becomes a *sighting*. A
sighting near an existing entry with the same label (MERGE_RADIUS) updates that entry: weighted running average of
the position, one more view, the viewing direction, scores and times. Otherwise it starts a new entry, so two
chairs stay two chairs.

Positions are stored in the learned map's frame. Odom restarts at zero whenever the base reboots, but the map
(slam_toolbox, continued from the saved pose graph) keeps its frame, so map coordinates stay valid between
sessions. Without a map, positions are kept in odom and marked frame="odom".

The revisit policy (B3) reads these entries: views, viewing directions and scores feed the reliability term; times
and status feed the staleness term.
"""

import math

from django.db import transaction
from django.utils import timezone

from .detector import clean_query
from .models import ObjectMemory

MERGE_RADIUS = 0.5  # m: a sighting this close to an entry with the same label is the same object
MIN_SCORE = 0.40  # weaker detections aren't remembered (Grounding DINO false positives sit around 0.3-0.4)
MAX_DISTANCE = 4.0  # m: positions from further away are too rough to remember
DIR_BIN_DEG = 20.0  # viewing directions closer than this count as the same view


def to_map(xy, m2o):
    """odom (x, y) -> map (x, y), with m2o = (tx, ty, yaw) of the map frame in odom. m2o None -> unchanged."""
    if m2o is None:
        return xy
    tx, ty, yaw = m2o
    dx, dy = xy[0] - tx, xy[1] - ty
    c, s = math.cos(yaw), math.sin(yaw)
    return c * dx + s * dy, -s * dx + c * dy


def to_odom(xy, m2o):
    """map (x, y) -> odom (x, y)."""
    if m2o is None:
        return xy
    tx, ty, yaw = m2o
    c, s = math.cos(yaw), math.sin(yaw)
    return tx + c * xy[0] - s * xy[1], ty + s * xy[0] + c * xy[1]


def _sighting_weight(score, distance):
    """Closer, more confident sightings count for more in the position average."""
    d = 1.0 if distance is None else max(0.5, distance)
    return score / d


def _add_dir(dirs, deg):
    deg = round(deg % 360.0, 1)
    for d in dirs:
        if abs((d - deg + 180.0) % 360.0 - 180.0) < DIR_BIN_DEG:
            return dirs
    return sorted(dirs + [deg])


def record_sighting(label, xy_odom, score, robot_xy_odom, m2o, distance=None, photo=None, when=None, weight=1.0):
    """Remember one sighting. Positions in odom; m2o converts them to the map frame. Returns the entry, or None if
    the sighting is too weak or too far to be worth keeping."""
    label = clean_query(label)
    if not label or score < MIN_SCORE or (distance is not None and distance > MAX_DISTANCE):
        return None
    when = when or timezone.now()
    frame = "odom" if m2o is None else "map"
    x, y = to_map(xy_odom, m2o)
    rx, ry = to_map(robot_xy_odom, m2o)
    view_dir = math.degrees(math.atan2(ry - y, rx - x))
    w = _sighting_weight(score, distance) * weight
    with transaction.atomic():
        near = [
            e for e in ObjectMemory.objects.select_for_update().filter(label=label, frame=frame)
            if math.hypot(e.x - x, e.y - y) <= MERGE_RADIUS
        ]
        if near:
            e = min(near, key=lambda e: math.hypot(e.x - x, e.y - y))
            total = e.weight + w
            e.x = (e.x * e.weight + x * w) / total
            e.y = (e.y * e.weight + y * w) / total
            e.weight = total
            e.views += 1
        else:
            e = ObjectMemory(label=label, x=x, y=y, frame=frame, first_seen=when, weight=w, views=1)
        e.status = ObjectMemory.PRESENT
        e.last_seen = when
        e.view_dirs = _add_dir(list(e.view_dirs or []), view_dir)
        e.best_score = max(e.best_score, score)
        e.last_score = score
        e.last_distance = None if distance is None else round(distance, 3)
        if photo is not None:
            e.last_photo = photo
        e.save()
    return e


def remember_detections(label, dets, pose, m2o, photo=None):
    """Record every located detection from one picture (dets from vision.locate: bearing deg, distance m).
    Returns the entries touched."""
    out = []
    if pose is None:
        return out
    for d in dets:
        if d.get("distance") is None:
            continue
        a = pose.yaw + math.radians(d["bearing"])
        xy = (pose.x + d["distance"] * math.cos(a), pose.y + d["distance"] * math.sin(a))
        e = record_sighting(label, xy, d["score"], (pose.x, pose.y), m2o, distance=d["distance"], photo=photo)
        if e is not None:
            out.append(e)
    return out


def entries(frame=None):
    qs = ObjectMemory.objects.all()
    return qs if frame is None else qs.filter(frame=frame)
