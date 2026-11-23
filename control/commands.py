"""The command vocabulary. Buttons, typed text and (later) an LLM all produce these, and validate() is the one
gate every command passes before the robot sees it."""

import math

MAX_FORWARD_M = 5.0  # obstacles shorten a move (the lidar watches while driving), so long moves are fine
MIN_FORWARD_M = 0.05
MAX_TURN_DEG = 360.0
MAX_SEQUENCE = 20
MAX_SCAN_OBJECTS = 8  # one detector call per object per picture (~0.2 s each)
FIND_STANDOFF_M = (0.10, 1.0, 0.30)  # min, max, default gap left between the bumper and the object
FIND_MAX_RANGE_M = (0.5, 50.0, 25.0)  # backstop on total driving; the real stop is "no progress" (executor)

KINDS = {
    "forward": "Drive forward",
    "turn": "Turn",
    "photo": "Take a picture",
    "scan": "Scan 360°",
    "undock": "Undock",
    "dock": "Dock",
    "return_home": "Return to start / dock",
    "find": "Find an object",
    "sequence": "Sequence",
}


class CommandError(ValueError):
    pass


def _number(params, key, lo, hi, default=None):
    v = params.get(key, default)
    if isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v):
        raise CommandError(f"'{key}' must be a number")
    if not lo <= v <= hi:
        raise CommandError(f"'{key}' must be between {lo:g} and {hi:g}")
    return v


def _bool(params, key, default):
    v = params.get(key, default)
    if not isinstance(v, bool):
        raise CommandError(f"'{key}' must be true or false")
    return v


def validate(kind, params=None, _nested=False):
    """Return normalized params for a command, or raise CommandError."""
    if params is None:
        params = {}
    if not isinstance(params, dict):
        raise CommandError("params must be an object")
    if kind not in KINDS:
        raise CommandError(f"unknown command '{kind}'")
    if kind == "forward":
        return {
            "distance": round(float(_number(params, "distance", MIN_FORWARD_M, MAX_FORWARD_M)), 3),
            "check_clear": _bool(params, "check_clear", True),
        }
    if kind == "turn":
        deg = float(_number(params, "degrees", -MAX_TURN_DEG, MAX_TURN_DEG))
        if abs(deg) < 1:
            raise CommandError("'degrees' must be at least 1 (positive = left, negative = right)")
        return {"degrees": round(deg, 2)}
    if kind == "scan":
        steps = _number(params, "steps", 4, 12, default=8)
        if int(steps) != steps:
            raise CommandError("'steps' must be a whole number")
        out = {"steps": int(steps)}
        objs = params.get("objects")
        if objs:
            from .detector import clean_query

            if isinstance(objs, str):
                objs = objs.split(",")
            if not isinstance(objs, list) or not all(isinstance(o, str) for o in objs):
                raise CommandError("'objects' must be a list of names, e.g. [\"chair\", \"backpack\"]")
            labels = []
            for o in objs:
                o = clean_query(o)
                if o and o not in labels:
                    labels.append(o)
            if len(labels) > MAX_SCAN_OBJECTS:
                raise CommandError(f"look for at most {MAX_SCAN_OBJECTS} objects in one scan")
            if labels:
                out["objects"] = labels
        return out
    if kind == "find":
        obj = params.get("object")
        if not isinstance(obj, str):
            raise CommandError("'object' must be text, e.g. \"red mug\"")
        from .detector import clean_query

        obj = clean_query(obj)
        if not obj:
            raise CommandError("say what to look for, e.g. \"red mug\"")
        lo, hi, default = FIND_STANDOFF_M
        rlo, rhi, rdefault = FIND_MAX_RANGE_M
        return {
            "object": obj,
            "approach": _bool(params, "approach", True),
            "standoff": round(float(_number(params, "standoff", lo, hi, default)), 3),
            "max_range": round(float(_number(params, "max_range", rlo, rhi, rdefault)), 3),
        }
    if kind == "return_home":
        return {"check_clear": _bool(params, "check_clear", True)}
    if kind == "sequence":
        if _nested:
            raise CommandError("a sequence can't contain another sequence")
        steps = params.get("steps")
        if not isinstance(steps, list) or not steps:
            raise CommandError("'steps' must be a non-empty list")
        if len(steps) > MAX_SEQUENCE:
            raise CommandError(f"at most {MAX_SEQUENCE} steps")
        out = []
        for i, step in enumerate(steps, 1):
            if not isinstance(step, dict) or "kind" not in step:
                raise CommandError(f"step {i} must be an object with a 'kind'")
            try:
                out.append({"kind": step["kind"], "params": validate(step["kind"], step.get("params") or {}, True)})
            except CommandError as e:
                raise CommandError(f"step {i}: {e}") from None
        return {"steps": out}
    return {}  # photo, undock, dock take no parameters


def describe(kind, params):
    """Short human description, e.g. 'Drive forward 0.5 m'."""
    if kind == "forward":
        return f"Drive forward {params['distance']:g} m" + ("" if params.get("check_clear", True) else " (no obstacle check)")
    if kind == "turn":
        d = params["degrees"]
        return f"Turn {'left' if d > 0 else 'right'} {abs(d):g}°"
    if kind == "scan":
        objs = params.get("objects")
        return f"Scan 360° ({params['steps']} pictures)" + (f", looking for {', '.join(objs)}" if objs else "")
    if kind == "find":
        return f"Find \"{params['object']}\"" + (" and go to it" if params.get("approach", True) else "")
    if kind == "sequence":
        return " → ".join(describe(s["kind"], s["params"]) for s in params["steps"])
    return KINDS.get(kind, kind)
