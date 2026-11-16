import json
import tempfile
import shutil

from django.test import TestCase, override_settings
from django.urls import reverse

from control.executor import executor
from control.models import Job, Photo
from control.robot import set_robot
from control.robot.sim import SimRobotClient

TMP_MEDIA = tempfile.mkdtemp(prefix="tb_test_views_")


def tearDownModule():
    shutil.rmtree(TMP_MEDIA, ignore_errors=True)


@override_settings(MEDIA_ROOT=TMP_MEDIA)
class ViewTests(TestCase):
    def setUp(self):
        self.robot = SimRobotClient(delay=0)
        set_robot(self.robot)
        executor.run_inline = True

    def tearDown(self):
        executor.run_inline = False
        set_robot(None)

    def post(self, name, body):
        return self.client.post(reverse(name), json.dumps(body), content_type="application/json")

    def test_pages_render(self):
        for name in ("control:control", "control:archive"):
            r = self.client.get(reverse(name))
            self.assertEqual(r.status_code, 200)
        self.assertContains(self.client.get(reverse("control:control")), "Simulator mode")

    def test_state(self):
        r = self.client.get(reverse("control:api_state"))
        data = r.json()
        self.assertTrue(data["robot"]["connected"])
        self.assertTrue(data["robot"]["docked"])
        self.assertIsNone(data["job"])
        self.assertIsNone(data["trip"])

    def test_submit_and_state_shows_trip(self):
        r = self.post("control:api_submit", {"kind": "undock"})
        self.assertEqual(r.status_code, 200, r.content)
        r = self.post("control:api_submit", {"kind": "turn", "params": {"degrees": 90}})
        self.assertEqual(r.json()["job"]["state"], "succeeded")
        data = self.client.get(reverse("control:api_state")).json()
        self.assertEqual(data["trip"]["moves"], 2)
        self.assertTrue(data["trip"]["started_docked"])
        self.assertEqual(data["recent_jobs"][0]["description"], "Turn left 90°")

    def test_submit_validation(self):
        self.assertEqual(self.post("control:api_submit", {"kind": "forward", "params": {"distance": 7}}).status_code, 400)
        self.assertEqual(self.post("control:api_submit", {"kind": "teleport"}).status_code, 400)
        r = self.client.post(reverse("control:api_submit"), "not json", content_type="application/json")
        self.assertEqual(r.status_code, 400)
        self.assertEqual(self.client.get(reverse("control:api_submit")).status_code, 405)

    def test_busy_returns_409(self):
        executor._job_id = 999999
        try:
            self.assertEqual(self.post("control:api_submit", {"kind": "photo"}).status_code, 409)
        finally:
            executor._job_id = None

    def test_interpret_preview_and_run(self):
        r = self.post("control:api_interpret", {"text": "take a picture then turn right 45"})
        self.assertEqual(r.json()["preview"], ["Take a picture", "Turn right 45°"])
        self.assertEqual(Job.objects.count(), 0)
        r = self.post("control:api_interpret", {"text": "take a picture then turn right 45", "run": True})
        self.assertEqual(r.status_code, 200, r.content)
        job = Job.objects.get()
        self.assertEqual((job.kind, job.source, job.state), ("sequence", "text", "succeeded"))
        self.assertEqual(self.post("control:api_interpret", {"text": "fly away"}).status_code, 400)

    def test_archive_and_detail(self):
        self.post("control:api_submit", {"kind": "photo"})
        photo = Photo.objects.get()
        r = self.client.get(reverse("control:archive"))
        self.assertContains(r, f"#{photo.id}")
        r = self.client.get(reverse("control:archive") + "?date=bad")
        self.assertEqual(r.status_code, 200)
        r = self.client.get(photo.get_absolute_url())
        self.assertContains(r, "Depth (16-bit PNG")
        r = self.client.post(photo.get_absolute_url(), {"note": "kitchen <b>mug</b>"})
        self.assertEqual(r.status_code, 302)
        photo.refresh_from_db()
        self.assertEqual(photo.note, "kitchen <b>mug</b>")
        self.assertContains(self.client.get(photo.get_absolute_url()), "kitchen &lt;b&gt;mug&lt;/b&gt;")
        # served from the archive folder
        r = self.client.get(photo.image.url)
        self.assertEqual(r.status_code, 200)
        self.assertEqual(r["Content-Type"], "image/png")

    def test_stop(self):
        self.assertEqual(self.client.post(reverse("control:api_stop")).status_code, 200)

    def test_offline_robot(self):
        from control.robot.http import HttpRobotClient

        set_robot(HttpRobotClient("http://127.0.0.1:9", "x" * 20, timeout=1))
        data = self.client.get(reverse("control:api_state")).json()
        self.assertFalse(data["robot"]["connected"])
        self.assertIn("can't reach", data["robot"]["error"])
