"""Where is a detected object? Pixel box -> bearing and distance from the robot (base_link, + bearing = left).

Three distance cues, used together:
  depth   median of the depth image inside the box (only with our RGB-D camera node)
  floor   the box's bottom edge is where the object touches the floor; with the camera's height and tilt that
          ray hits the floor at a known distance (needs the bottom edge to be inside the image)
  lidar   nearest lidar return along the bearing; the lidar is 19 cm up, so it sees tall things (chairs, boxes,
          people) but looks over small objects on the floor, and may see whatever is behind them
See pick_distance() for how they are combined. The approach re-measures after every leg of at most 1 m, and the
lidar obstacle check stops it in front of anything in the way, so one bad estimate can't run the robot into things.
"""

import io
import math
import statistics

from PIL import Image

EDGE_PX = 3  # a box within this many pixels of the bottom edge is cut off by the image


def bearing_deg(u, intrinsics, camera=None):
    """Bearing of image column u, in degrees, + = left of the robot's heading."""
    yaw = (camera or {}).get("yaw_deg", 0.0)
    return math.degrees(math.atan2(intrinsics["cx"] - u, intrinsics["fx"])) + yaw


def to_base(d_cam, bearing, camera=None):
    """A point at horizontal distance d_cam from the camera along `bearing` -> (distance, bearing) from base_link."""
    cam = camera or {}
    b = math.radians(bearing)
    px = cam.get("x", 0.0) + d_cam * math.cos(b)
    py = cam.get("y", 0.0) + d_cam * math.sin(b)
    return math.hypot(px, py), math.degrees(math.atan2(py, px))


def floor_distance(v_bottom, height, intrinsics, camera):
    """Horizontal distance from the camera to where the ray through image row v_bottom meets the floor."""
    if camera is None or camera.get("z") is None:
        return None
    if v_bottom >= height - EDGE_PX:
        return None  # the object's bottom is cut off: the true contact point is nearer than the edge suggests
    down = math.atan2(v_bottom - intrinsics["cy"], intrinsics["fy"]) + math.radians(camera.get("pitch_deg", 0.0))
    if down < math.radians(1.0):
        return None  # at or above the horizon: not on the floor
    return camera["z"] / math.tan(down)


def depth_distance(depth_png, box):
    """Median depth (m) in the upper half of the box, or None (no depth image, or too few valid pixels)."""
    if not depth_png:
        return None
    try:
        depth = Image.open(io.BytesIO(depth_png))
        depth.load()
    except Exception:
        return None
    x0, y0, x1, y1 = (int(round(v)) for v in box)
    x0, y0 = max(x0, 0), max(y0, 0)
    x1, y_mid = min(x1, depth.width - 1), min(max(int((y0 + y1) / 2), y0 + 1), depth.height - 1)
    if x1 <= x0 or y_mid <= y0:
        return None
    patch = depth.crop((x0, y0, x1 + 1, y_mid + 1))
    data = patch.get_flattened_data() if hasattr(patch, "get_flattened_data") else patch.getdata()
    valid = [v / 1000.0 for v in data if 200 < v < 10000]  # millimetres
    if len(valid) < 20:
        return None
    return float(statistics.median(valid))


def locate(det, photo, range_at=None):
    """Estimate where a detection is. Returns a dict with bearing (deg), distance (m from base_link or None),
    the individual cues, and whether the box touches the bottom of the image."""
    intr = photo.intrinsics
    if not intr:
        w = photo.width
        intr = {"fx": w / (2 * math.tan(math.radians(34.5))), "fy": w / (2 * math.tan(math.radians(34.5))),
                "cx": w / 2, "cy": photo.height / 2}
    cam = photo.camera or {}
    x0, y0, x1, y1 = det["box"]
    bearing_cam = bearing_deg((x0 + x1) / 2, intr, cam)
    half_width = abs(bearing_deg(x0, intr, cam) - bearing_deg(x1, intr, cam)) / 2
    cues = {}
    d = depth_distance(photo.depth_png, det["box"])
    if d is not None:
        cues["depth"] = to_base(d, bearing_cam, cam)[0]
    d = floor_distance(y1, photo.height, intr, cam)
    if d is not None:
        cues["floor"] = to_base(d, bearing_cam, cam)[0]
    bearing = to_base(2.0, bearing_cam, cam)[1]  # bearing from base_link of a point ~2 m out
    if range_at is not None:
        try:
            r = range_at(bearing, max(4.0, 2 * half_width))
        except Exception:
            r = None
        if r is not None:
            cues["lidar"] = r
    distance = pick_distance(cues)
    return {
        "bearing": round(bearing, 2),
        "distance": None if distance is None else round(distance, 3),
        "cues": {k: round(v, 3) for k, v in cues.items()},
        "cut_off_bottom": y1 >= photo.height - EDGE_PX,
        "half_width_deg": round(half_width, 2),
    }


LIDAR_AGREE = 0.3  # m: lidar and camera estimates this close are taken to be the same object


def pick_distance(cues):
    """Combine the cues. Depth wins when present. Otherwise the floor estimate, tightened by the lidar when the two
    agree (a tall object the lidar also sees). A lidar return much nearer than the floor estimate is something
    *in front of* the object, not the object: the obstacle check deals with that while driving."""
    if "depth" in cues:
        return cues["depth"]
    floor, lidar = cues.get("floor"), cues.get("lidar")
    if floor is not None:
        if lidar is not None and abs(lidar - floor) <= LIDAR_AGREE:
            return min(lidar, floor)
        return floor
    return lidar


def horizontal_fov_deg(photo):
    intr = photo.intrinsics
    if not intr:
        return 69.0
    return math.degrees(2 * math.atan2(photo.width / 2, intr["fx"]))
