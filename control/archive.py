"""Saving robot pictures into the local archive (MEDIA_ROOT/photos, thumbs, depth; indexed by the Photo model)."""

import io
import uuid

from django.core.files.base import ContentFile
from django.utils import timezone
from PIL import Image

from .models import Photo

THUMB_SIZE = (320, 320)


def save_photo(data, job=None, trip=None, scan_index=None):
    """Store a PhotoData from the robot and return the saved Photo."""
    try:
        img = Image.open(io.BytesIO(data.rgb_png))
        img.load()
    except Exception as e:
        raise ValueError(f"the robot sent an image that can't be read ({e})") from None
    taken = timezone.now()
    base = f"tb_{timezone.localtime(taken):%Y%m%d_%H%M%S}_{uuid.uuid4().hex[:6]}"
    photo = Photo(
        taken_at=taken,
        camera_topic=data.topic,
        job=job,
        trip=trip,
        scan_index=scan_index,
    )
    if data.pose is not None:
        photo.x, photo.y, photo.yaw = data.pose.x, data.pose.y, data.pose.yaw
    photo.image.save(base + ".png", ContentFile(data.rgb_png), save=False)
    thumb = img.convert("RGB")
    thumb.thumbnail(THUMB_SIZE)
    buf = io.BytesIO()
    thumb.save(buf, "JPEG", quality=85)
    photo.thumbnail.save(base + ".jpg", ContentFile(buf.getvalue()), save=False)
    if data.depth_png:
        photo.depth.save(base + "_depth.png", ContentFile(data.depth_png), save=False)
    photo.save()
    return photo
