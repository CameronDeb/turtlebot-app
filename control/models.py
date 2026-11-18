from django.db import models
from django.urls import reverse
from django.utils import timezone


class Trip(models.Model):
    """One outing from a start spot. Its waypoints are the path back home ("return to dock" retraces them)."""

    ACTIVE, RETURNED, DOCKED, ENDED = "active", "returned", "docked", "ended"
    STATUS_CHOICES = [
        (ACTIVE, "Active"),
        (RETURNED, "Returned to start"),
        (DOCKED, "Docked"),
        (ENDED, "Ended"),
    ]

    started_at = models.DateTimeField(auto_now_add=True)
    ended_at = models.DateTimeField(null=True, blank=True)
    status = models.CharField(max_length=10, choices=STATUS_CHOICES, default=ACTIVE)
    # Set when the trip began by undocking: where the robot sat on the dock (odom frame).
    started_docked = models.BooleanField(default=False)
    dock_x = models.FloatField(null=True, blank=True)
    dock_y = models.FloatField(null=True, blank=True)

    class Meta:
        ordering = ["-id"]

    def __str__(self):
        return f"Trip {self.id} ({self.get_status_display()})"

    def path(self):
        """Waypoints still to retrace, oldest first."""
        return list(self.waypoints.filter(retraced=False).order_by("seq"))


class Waypoint(models.Model):
    """Robot pose (odom frame) at the start of a trip and after every motion."""

    START, DOCK, MOVE = "start", "dock", "move"
    KIND_CHOICES = [(START, "Start"), (DOCK, "On the dock"), (MOVE, "After a move")]

    trip = models.ForeignKey(Trip, on_delete=models.CASCADE, related_name="waypoints")
    seq = models.PositiveIntegerField()
    kind = models.CharField(max_length=5, choices=KIND_CHOICES, default=MOVE)
    x = models.FloatField()
    y = models.FloatField()
    yaw = models.FloatField()
    retraced = models.BooleanField(default=False)
    job = models.ForeignKey(
        "Job", null=True, blank=True, on_delete=models.SET_NULL, related_name="+"
    )
    created_at = models.DateTimeField(auto_now_add=True)

    class Meta:
        ordering = ["trip", "seq"]
        constraints = [
            models.UniqueConstraint(fields=["trip", "seq"], name="unique_waypoint_seq")
        ]

    def __str__(self):
        return f"{self.trip_id}#{self.seq} ({self.x:.2f}, {self.y:.2f})"


class Job(models.Model):
    """One command from the user (a button press or a typed sentence) and what happened."""

    RUNNING, SUCCEEDED, FAILED, CANCELED = "running", "succeeded", "failed", "canceled"
    STATE_CHOICES = [
        (RUNNING, "Running"),
        (SUCCEEDED, "Succeeded"),
        (FAILED, "Failed"),
        (CANCELED, "Stopped"),
    ]

    kind = models.CharField(max_length=20)
    params = models.JSONField(default=dict, blank=True)
    source = models.CharField(max_length=10, default="button")  # button | text | api
    text = models.TextField(blank=True)  # the typed sentence, for text commands
    state = models.CharField(max_length=10, choices=STATE_CHOICES, default=RUNNING)
    message = models.TextField(blank=True)
    result = models.JSONField(default=dict, blank=True)
    created_at = models.DateTimeField(auto_now_add=True)
    finished_at = models.DateTimeField(null=True, blank=True)

    class Meta:
        ordering = ["-id"]

    def __str__(self):
        return f"Job {self.id} {self.kind} ({self.state})"

    def as_dict(self):
        return {
            "id": self.id,
            "kind": self.kind,
            "params": self.params,
            "source": self.source,
            "text": self.text,
            "state": self.state,
            "message": self.message,
            "created_at": self.created_at.isoformat() if self.created_at else None,
            "finished_at": self.finished_at.isoformat() if self.finished_at else None,
        }


def _dated(folder, instance, filename):
    """Archive layout: <folder>/<YYYY>/<MM>/<DD>/<file>, by the local (Arizona) date the photo was taken."""
    return f"{folder}/{timezone.localtime(instance.taken_at):%Y/%m/%d}/{filename}"


def _photo_path(instance, filename):
    return _dated("photos", instance, filename)


def _thumb_path(instance, filename):
    return _dated("thumbs", instance, filename)


def _depth_path(instance, filename):
    return _dated("depth", instance, filename)


class Photo(models.Model):
    """A picture from the robot's camera, kept in the local archive (MEDIA_ROOT)."""

    taken_at = models.DateTimeField()
    image = models.ImageField(upload_to=_photo_path, width_field="width", height_field="height")
    width = models.PositiveIntegerField(null=True, blank=True)
    height = models.PositiveIntegerField(null=True, blank=True)
    thumbnail = models.ImageField(upload_to=_thumb_path, blank=True)
    depth = models.FileField(upload_to=_depth_path, blank=True)  # 16-bit PNG, millimetres, aligned to the image
    camera_topic = models.CharField(max_length=100, blank=True)
    x = models.FloatField(null=True, blank=True)
    y = models.FloatField(null=True, blank=True)
    yaw = models.FloatField(null=True, blank=True)
    trip = models.ForeignKey(
        Trip, null=True, blank=True, on_delete=models.SET_NULL, related_name="photos"
    )
    job = models.ForeignKey(
        Job, null=True, blank=True, on_delete=models.SET_NULL, related_name="photos"
    )
    scan_index = models.PositiveIntegerField(null=True, blank=True)  # position within a 360 deg scan
    note = models.TextField(blank=True)
    # Filled in later by image processing (object search): [{"label", "score", "box": [x0, y0, x1, y1]}, ...]
    detections = models.JSONField(default=list, blank=True)

    class Meta:
        ordering = ["-taken_at", "-id"]

    def __str__(self):
        return f"Photo {self.id} at {self.taken_at:%Y-%m-%d %H:%M:%S}"

    def get_absolute_url(self):
        return reverse("control:photo_detail", args=[self.id])

    def as_dict(self):
        return {
            "id": self.id,
            "taken_at": self.taken_at.isoformat(),
            "url": self.image.url,
            "thumbnail_url": self.thumbnail.url if self.thumbnail else self.image.url,
            "detail_url": self.get_absolute_url(),
            "width": self.width,
            "height": self.height,
            "pose": None if self.x is None else {"x": self.x, "y": self.y, "yaw": self.yaw},
            "has_depth": bool(self.depth),
            "detections": self.detections or [],
        }
