"""Open-vocabulary object detection for the object search.

HttpDetector talks to detector/serve.py (Grounding DINO on the lab PC's GPU, running in WSL). If that server isn't
up, it is started automatically (settings.DETECTOR_AUTOSTART) and the first search waits for the model to load.
SimDetector reads the simulator's ground truth, so the search can be tried and tested without a robot or a GPU.
"""

import base64
import json
import logging
import re
import socket
import subprocess
import threading
import time
import urllib.error
import urllib.request
from pathlib import Path

from django.conf import settings

logger = logging.getLogger(__name__)

STOPWORDS = {"a", "an", "the", "my", "some", "this", "that"}


class DetectorError(Exception):
    pass


def clean_query(text):
    """'  The RED mug!! ' -> 'red mug' (letters, digits, spaces and hyphens; at most 60 characters)."""
    words = re.sub(r"[^a-z0-9\- ]+", " ", (text or "").lower()).split()
    while words and words[0] in STOPWORDS:
        words.pop(0)
    return " ".join(words)[:60].strip()


class Detector:
    def detect(self, photo_data, query):
        """-> [{"label", "score", "box": [x0, y0, x1, y1]}], best first."""
        raise NotImplementedError


class SimDetector(Detector):
    def detect(self, photo_data, query):
        wanted = set(clean_query(query).split()) - STOPWORDS
        out = []
        for obj in photo_data.sim_objects or []:
            words = set(clean_query(obj["label"]).split())
            if wanted and wanted <= words:
                out.append({"label": obj["label"], "score": 0.9, "box": [round(v, 1) for v in obj["box"]]})
        return out


class HttpDetector(Detector):
    def __init__(self, url, threshold=0.35, autostart=True):
        self.url = url.rstrip("/")
        self.threshold = threshold
        self.autostart = autostart
        self._start_lock = threading.Lock()
        self._process = None

    def _post(self, path, body, timeout):
        req = urllib.request.Request(
            self.url + path, data=json.dumps(body).encode(), method="POST",
            headers={"Content-Type": "application/json"},
        )
        with urllib.request.urlopen(req, timeout=timeout) as resp:
            return json.loads(resp.read())

    def alive(self):
        try:
            with urllib.request.urlopen(self.url + "/ping", timeout=2.0) as resp:
                return json.loads(resp.read()).get("ok") is True
        except (urllib.error.URLError, socket.timeout, TimeoutError, ConnectionError, ValueError):
            return False

    def ensure_running(self, wait=180.0):
        if self.alive():
            return
        if not self.autostart:
            raise DetectorError(f"the object detector isn't running at {self.url} (start detector/start_detector.sh)")
        with self._start_lock:
            if self.alive():
                return
            if self._process is None or self._process.poll() is not None:
                self._process = start_wsl_detector()
            end = time.monotonic() + wait
            while time.monotonic() < end:
                if self.alive():
                    return
                if self._process.poll() is not None:
                    raise DetectorError(
                        "the object detector stopped while starting; see archive/detector.log"
                    )
                time.sleep(1.0)
        raise DetectorError("the object detector didn't start within 3 minutes; see archive/detector.log")

    def detect(self, photo_data, query):
        self.ensure_running()
        body = {
            "image_png": base64.b64encode(photo_data.rgb_png).decode("ascii"),
            "prompt": clean_query(query),
            "threshold": self.threshold,
        }
        try:
            d = self._post("/detect", body, timeout=60.0)
        except urllib.error.HTTPError as e:
            raise DetectorError(f"the object detector failed: {e.read()[:200]!r}") from None
        except (urllib.error.URLError, socket.timeout, TimeoutError, ConnectionError) as e:
            raise DetectorError(f"can't reach the object detector ({getattr(e, 'reason', e)})") from None
        dets = d.get("detections", [])
        return sorted(dets, key=lambda x: -x["score"])


def start_wsl_detector():
    """Start detector/serve.py inside WSL (conda env settings.DETECTOR_CONDA_ENV) in the background."""
    base = Path(settings.BASE_DIR).resolve()
    drive, rest = base.drive.rstrip(":").lower(), base.as_posix().split(":", 1)[-1]
    script = f"/mnt/{drive}{rest}/detector/start_detector.sh"
    log_path = Path(settings.MEDIA_ROOT) / "detector.log"
    log_path.parent.mkdir(parents=True, exist_ok=True)
    log = open(log_path, "ab")
    cmd = ["wsl", "-d", settings.DETECTOR_WSL_DISTRO, "--", "bash", script, settings.DETECTOR_CONDA_ENV]
    logger.info("starting object detector: %s", " ".join(cmd))
    flags = getattr(subprocess, "CREATE_NO_WINDOW", 0)
    return subprocess.Popen(cmd, stdout=log, stderr=subprocess.STDOUT, stdin=subprocess.DEVNULL, creationflags=flags)


_detector = None
_lock = threading.Lock()


def get_detector():
    global _detector
    with _lock:
        if _detector is None:
            backend = settings.DETECTOR_BACKEND
            if backend == "auto":
                backend = "sim" if settings.ROBOT_BACKEND == "sim" else "http"
            if backend == "sim":
                _detector = SimDetector()
            elif backend == "http":
                _detector = HttpDetector(
                    settings.DETECTOR_URL, settings.DETECTOR_THRESHOLD, settings.DETECTOR_AUTOSTART
                )
            else:
                raise RuntimeError(f"DETECTOR_BACKEND must be auto, sim or http, not {backend!r}")
        return _detector


def set_detector(detector):
    """Replace the shared detector (tests)."""
    global _detector
    with _lock:
        _detector = detector
