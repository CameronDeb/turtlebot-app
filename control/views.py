import json
import math
from datetime import datetime, timedelta

from django.conf import settings
from django.contrib import messages
from django.core.paginator import Paginator
from django.http import JsonResponse
from django.shortcuts import get_object_or_404, redirect, render
from django.utils import timezone
from django.views.decorators.http import require_GET, require_http_methods, require_POST

from .commands import KINDS, MAX_FORWARD_M, CommandError, describe
from .executor import Busy, active_trip, executor
from .interpreter import InterpretError, get_interpreter
from .models import Job, ObjectMemory, Photo, Trip
from .robot import RobotError, get_robot

FORWARD_CHOICES = [0.1, 0.25, 0.5, 1.0, 1.5, 2.0, 3.0]
TURN_CHOICES = [15, 30, 45, 90, 180]


def control_page(request):
    return render(
        request,
        "control/control.html",
        {
            "forward_choices": [d for d in FORWARD_CHOICES if d <= MAX_FORWARD_M],
            "turn_choices": TURN_CHOICES,
            "backend": get_robot().name,
            "time_zone": settings.TIME_ZONE,
        },
    )


def archive_page(request):
    photos = Photo.objects.select_related("trip")
    date = request.GET.get("date", "")
    trip = request.GET.get("trip", "")
    scans = request.GET.get("scans") == "1"
    if date:
        try:
            day = datetime.strptime(date, "%Y-%m-%d").date()
        except ValueError:
            messages.error(request, "Date must look like 2026-11-16.")
            day = None
        if day:
            start = timezone.make_aware(datetime.combine(day, datetime.min.time()))
            photos = photos.filter(taken_at__gte=start, taken_at__lt=start + timedelta(days=1))
    if trip.isdigit():
        photos = photos.filter(trip_id=int(trip))
    if scans:
        photos = photos.filter(scan_index__isnull=False)
    page = Paginator(photos, 24).get_page(request.GET.get("page"))
    query = request.GET.copy()
    query.pop("page", None)
    return render(
        request,
        "control/archive.html",
        {
            "page": page,
            "date": date,
            "trip": trip,
            "scans": scans,
            "total": Photo.objects.count(),
            "trips": Trip.objects.filter(photos__isnull=False).distinct()[:50],
            "query": query.urlencode(),
        },
    )


@require_http_methods(["GET", "POST"])
def photo_detail(request, pk):
    photo = get_object_or_404(Photo.objects.select_related("trip", "job"), pk=pk)
    if request.method == "POST":
        photo.note = request.POST.get("note", "")[:2000]
        photo.save(update_fields=["note"])
        messages.success(request, "Note saved.")
        return redirect(photo)
    newer = Photo.objects.filter(taken_at__gt=photo.taken_at).order_by("taken_at").first()
    older = Photo.objects.filter(taken_at__lt=photo.taken_at).order_by("-taken_at").first()
    return render(
        request,
        "control/photo_detail.html",
        {
            "photo": photo,
            "heading_deg": None if photo.yaw is None else round(math.degrees(photo.yaw)),
            "newer": newer,
            "older": older,
        },
    )


# ---------------------------------------------------------------- JSON API


def _json_body(request):
    try:
        body = json.loads(request.body or b"{}")
    except ValueError:
        return None
    return body if isinstance(body, dict) else None


def _error(message, status):
    return JsonResponse({"ok": False, "error": message}, status=status)


def _job_dict(job):
    d = job.as_dict()
    d["description"] = describe(job.kind, job.params) if job.params is not None else job.kind
    d["photo_ids"] = (job.result or {}).get("photo_ids", [])
    return d


@require_GET
def api_state(request):
    robot = get_robot()
    try:
        s = robot.status()
        pose = s.get("pose")
        robot_state = {
            "connected": True,
            "backend": robot.name,
            "pose": None
            if pose is None
            else {"x": pose.x, "y": pose.y, "yaw": pose.yaw, "heading_deg": round(math.degrees(pose.yaw))},
            "pose_error": s.get("pose_error"),
            "battery_percent": s.get("battery_percent"),
            "docked": s.get("docked"),
            "dock_visible": s.get("dock_visible"),
            "wheels_enabled": s.get("wheels_enabled"),
            "mapping": s.get("mapping"),
            "busy": s.get("busy"),
            "scan_age": s.get("scan_age"),
            "camera": s.get("camera", {}),
        }
    except RobotError as e:
        robot_state = {"connected": False, "backend": robot.name, "error": str(e)}

    current = Job.objects.filter(id=executor.current_job_id).first() if executor.current_job_id else None
    trip = active_trip()
    trip_state = None
    if trip is not None:
        path = trip.path()
        trip_state = {
            "id": trip.id,
            "started_docked": trip.started_docked,
            "dock": None if trip.dock_x is None else [trip.dock_x, trip.dock_y],
            "path": [[w.x, w.y] for w in path],
            "moves": sum(1 for w in path if w.kind == "move"),
        }
    latest = Photo.objects.first()
    return JsonResponse(
        {
            "ok": True,
            "robot": robot_state,
            "job": _job_dict(current) if current else None,
            "recent_jobs": [_job_dict(j) for j in Job.objects.all()[:12]],
            "trip": trip_state,
            "latest_photo": latest.as_dict() if latest else None,
            "memory": [m.as_dict() for m in ObjectMemory.objects.order_by("-last_seen")[:40]],
        }
    )


def _submit(kind, params, source, text=""):
    try:
        job = executor.submit(kind, params, source=source, text=text)
    except CommandError as e:
        return _error(str(e), 400)
    except Busy as e:
        return _error(str(e), 409)
    return JsonResponse({"ok": True, "job": _job_dict(job)})


@require_POST
def api_submit(request):
    body = _json_body(request)
    if body is None:
        return _error("body must be a JSON object", 400)
    kind = body.get("kind")
    if kind not in KINDS:
        return _error(f"unknown command '{kind}'", 400)
    return _submit(kind, body.get("params") or {}, source="button")


@require_POST
def api_stop(request):
    problem = executor.stop()
    if problem:
        return JsonResponse({"ok": False, "error": f"Stop sent, but the robot didn't confirm: {problem}"}, status=502)
    return JsonResponse({"ok": True})


@require_POST
def api_interpret(request):
    """{"text": "...", "run": bool} -> the parsed steps; with run=true they are started as one job."""
    body = _json_body(request)
    if body is None or not isinstance(body.get("text"), str):
        return _error("send {\"text\": \"...\"}", 400)
    text = body["text"].strip()[:500]
    try:
        steps = get_interpreter().interpret(text)
    except InterpretError as e:
        return _error(str(e), 400)
    preview = [describe(s["kind"], s["params"]) for s in steps]
    if not body.get("run"):
        return JsonResponse({"ok": True, "steps": steps, "preview": preview})
    if len(steps) == 1:
        response = _submit(steps[0]["kind"], steps[0]["params"], source="text", text=text)
    else:
        response = _submit("sequence", {"steps": steps}, source="text", text=text)
    if response.status_code == 200:
        data = json.loads(response.content)
        data["preview"] = preview
        return JsonResponse(data)
    return response


@require_POST
def api_motors(request):
    """Turn the drive motors back on after an e-stop (Create 3 power button / controller B)."""
    if executor.current_job_id is not None:
        return _error("wait until the current command has finished", 409)
    try:
        result = get_robot().motors_on()
    except RobotError as e:
        return _error(str(e), 502)
    if not result.get("ok"):
        return _error("the robot didn't turn its motors back on; check the light ring, or restart the base", 502)
    return JsonResponse({"ok": True})


@require_GET
def api_photos(request):
    try:
        limit = max(1, min(int(request.GET.get("limit", 20)), 100))
    except ValueError:
        return _error("limit must be a number", 400)
    return JsonResponse({"ok": True, "photos": [p.as_dict() for p in Photo.objects.all()[:limit]]})
