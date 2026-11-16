"""End-to-end test against a running robot agent (robot_agent/agent.py), e.g. with robot_agent/dev/fake_create3.py.

Skipped unless E2E_AGENT_URL and E2E_AGENT_TOKEN are set. The fake robot must be started with
`--obstacle -1.6 0` (a box behind the dock, in the robot's path after undocking).
NEVER point this at the real robot: it drives without asking.
"""

import math
import os
import shutil
import tempfile
import time
import unittest

from django.test import TransactionTestCase, override_settings

from control.executor import executor
from control.models import Job, Photo, Trip
from control.robot import set_robot
from control.robot.http import HttpRobotClient

URL = os.environ.get("E2E_AGENT_URL")
TOKEN = os.environ.get("E2E_AGENT_TOKEN", "")
TMP_MEDIA = tempfile.mkdtemp(prefix="tb_e2e_")


def tearDownModule():
    shutil.rmtree(TMP_MEDIA, ignore_errors=True)


@unittest.skipUnless(URL, "set E2E_AGENT_URL / E2E_AGENT_TOKEN to run against a (fake) robot agent")
@override_settings(MEDIA_ROOT=TMP_MEDIA)
class AgentEndToEnd(TransactionTestCase):
    def setUp(self):
        self.robot = HttpRobotClient(URL, TOKEN, timeout=5)
        set_robot(self.robot)
        executor.run_inline = False

    def tearDown(self):
        set_robot(None)

    def run_cmd(self, kind, timeout=120, **params):
        job = executor.submit(kind, params)
        deadline = time.time() + timeout
        while time.time() < deadline:
            job.refresh_from_db()
            if job.state != Job.RUNNING and executor.current_job_id is None:
                return job
            time.sleep(0.1)
        self.fail(f"{kind} did not finish")

    def test_full_outing(self):
        s = self.robot.status()
        self.assertTrue(s["docked"], "start the fake robot fresh (docked)")
        self.assertEqual(self.run_cmd("forward", distance=0.5).state, Job.FAILED)  # lidar silent on dock

        job = self.run_cmd("undock")
        self.assertEqual(job.state, Job.SUCCEEDED, job.message)
        job = self.run_cmd("forward", distance=1.0)
        self.assertEqual(job.state, Job.FAILED)
        self.assertIn("blocked", job.message)
        for kind, params in [("forward", {"distance": 0.5}), ("turn", {"degrees": 90}), ("forward", {"distance": 0.5})]:
            job = self.run_cmd(kind, **params)
            self.assertEqual(job.state, Job.SUCCEEDED, job.message)
        pose = self.robot.pose()
        self.assertAlmostEqual(pose.x, -0.85, places=2)
        self.assertAlmostEqual(pose.y, -0.5, places=2)

        job = self.run_cmd("photo")
        self.assertEqual(job.state, Job.SUCCEEDED, job.message)
        photo = Photo.objects.get()
        self.assertEqual((photo.width, photo.height), (640, 360))
        self.assertTrue(photo.depth)
        self.assertEqual(photo.camera_topic, "/oakd_rgbd/rgb/image_raw")

        self.assertEqual(self.run_cmd("scan", steps=4).state, Job.SUCCEEDED)
        self.assertEqual(Photo.objects.count(), 5)

        # Stop part-way through a long turn; the partial turn is recorded and Return still works.
        job = executor.submit("turn", {"degrees": 300})
        time.sleep(1.5)
        executor.stop()
        while executor.current_job_id is not None:
            time.sleep(0.05)
        job.refresh_from_db()
        self.assertEqual(job.state, Job.CANCELED)

        job = self.run_cmd("return_home")
        self.assertEqual(job.state, Job.SUCCEEDED, job.message)
        self.assertTrue(self.robot.status()["docked"])
        self.assertEqual(Trip.objects.get().status, Trip.DOCKED)
        pose = self.robot.pose()
        self.assertAlmostEqual(math.hypot(pose.x, pose.y), 0.0, places=3)
