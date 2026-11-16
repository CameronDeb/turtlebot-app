import math
import shutil
import tempfile
import threading
import time
from pathlib import Path

from django.test import TestCase, TransactionTestCase, override_settings

from control.executor import Busy, executor
from control.models import Job, Photo, Trip
from control.robot import set_robot
from control.robot.sim import SimRobotClient

TMP_MEDIA = tempfile.mkdtemp(prefix="tb_test_media_")


def tearDownModule():
    shutil.rmtree(TMP_MEDIA, ignore_errors=True)


@override_settings(MEDIA_ROOT=TMP_MEDIA)
class ExecutorSimTests(TestCase):
    def setUp(self):
        self.robot = SimRobotClient(delay=0)
        set_robot(self.robot)
        executor.run_inline = True

    def tearDown(self):
        executor.run_inline = False
        set_robot(None)

    def run_cmd(self, kind, **params):
        return executor.submit(kind, params)

    def assertPose(self, x, y, yaw=None, places=3):
        pose = self.robot.status()["pose"]
        self.assertAlmostEqual(pose.x, x, places=places)
        self.assertAlmostEqual(pose.y, y, places=places)
        if yaw is not None:
            self.assertAlmostEqual(math.cos(pose.yaw), math.cos(yaw), places=places)
            self.assertAlmostEqual(math.sin(pose.yaw), math.sin(yaw), places=places)

    def test_forward_blocked_while_docked_by_obstacle_check(self):
        job = self.run_cmd("forward", distance=0.5)
        self.assertEqual(job.state, Job.FAILED)
        self.assertIn("lidar", job.message)
        self.assertFalse(Trip.objects.exists())  # nothing moved, so no trip

    def test_undock_moves_and_return_to_dock(self):
        self.assertEqual(self.run_cmd("undock").state, Job.SUCCEEDED)
        self.assertPose(-0.35, 0.0, math.pi)
        for kind, params in [
            ("forward", {"distance": 1.0}),
            ("turn", {"degrees": 90}),
            ("forward", {"distance": 0.5}),
            ("turn", {"degrees": -45}),
            ("forward", {"distance": 0.25}),
        ]:
            job = self.run_cmd(kind, **params)
            self.assertEqual(job.state, Job.SUCCEEDED, job.message)
        trip = Trip.objects.get(status=Trip.ACTIVE)
        self.assertTrue(trip.started_docked)
        self.assertEqual(len(trip.path()), 7)  # dock + after undock + 5 moves

        job = self.run_cmd("return_home")
        self.assertEqual(job.state, Job.SUCCEEDED, job.message)
        self.assertEqual(job.message, "Back on the dock.")
        self.assertTrue(self.robot.status()["docked"])
        trip.refresh_from_db()
        self.assertEqual(trip.status, Trip.DOCKED)
        self.assertIsNone(Trip.objects.filter(status=Trip.ACTIVE).first())

    def test_return_to_start_without_dock(self):
        self.robot.docked = False
        self.robot.x, self.robot.y, self.robot.yaw = 2.0, 1.0, 0.3
        for kind, params in [("forward", {"distance": 1.0}), ("turn", {"degrees": 120}), ("forward", {"distance": 0.8})]:
            self.assertEqual(self.run_cmd(kind, **params).state, Job.SUCCEEDED)
        job = self.run_cmd("return_home")
        self.assertEqual(job.state, Job.SUCCEEDED, job.message)
        self.assertPose(2.0, 1.0, 0.3)
        self.assertEqual(Trip.objects.get().status, Trip.RETURNED)
        # nothing left to return from
        self.assertEqual(self.run_cmd("return_home").state, Job.FAILED)

    def test_return_from_turns_only_restores_heading(self):
        self.robot.docked = False
        self.run_cmd("turn", degrees=90)
        self.run_cmd("turn", degrees=45)
        self.assertEqual(self.run_cmd("return_home").state, Job.SUCCEEDED)
        self.assertPose(0, 0, 0)

    def test_return_resumes_after_interruption(self):
        self.robot.docked = False
        self.run_cmd("forward", distance=1.0)
        self.run_cmd("turn", degrees=90)
        self.run_cmd("forward", distance=1.0)
        # Simulate a return stopped half-way along its first leg: the robot is between waypoints.
        trip = Trip.objects.get()
        path = trip.path()
        trip.waypoints.filter(seq__gt=path[-2].seq).update(retraced=True)
        self.robot.x, self.robot.y = 1.0, 0.5
        job = self.run_cmd("return_home")
        self.assertEqual(job.state, Job.SUCCEEDED, job.message)
        self.assertPose(0, 0, 0)

    def test_blocked_path_fails_cleanly(self):
        self.robot.docked = False
        self.robot.obstacles = [(0.8, 0.0)]
        job = self.run_cmd("forward", distance=1.0)
        self.assertEqual(job.state, Job.FAILED)
        self.assertIn("blocked", job.message)
        job = self.run_cmd("forward", distance=1.0, check_clear=False)
        self.assertEqual(job.state, Job.SUCCEEDED)

    def test_photo_is_archived(self):
        job = self.run_cmd("photo")
        self.assertEqual(job.state, Job.SUCCEEDED, job.message)
        photo = Photo.objects.get()
        self.assertEqual(job.result["photo_ids"], [photo.id])
        self.assertEqual((photo.width, photo.height), (640, 360))
        for f in (photo.image, photo.thumbnail, photo.depth):
            self.assertTrue(Path(f.path).is_file(), f.name)
        self.assertRegex(photo.image.name, r"^photos/\d{4}/\d{2}/\d{2}/tb_\d{8}_\d{6}_[0-9a-f]{6}\.png$")

    def test_scan_takes_photos_and_ends_facing_same_way(self):
        self.robot.docked = False
        self.robot.yaw = 0.7
        job = self.run_cmd("scan", steps=6)
        self.assertEqual(job.state, Job.SUCCEEDED, job.message)
        self.assertEqual(Photo.objects.count(), 6)
        self.assertEqual(sorted(Photo.objects.values_list("scan_index", flat=True)), list(range(6)))
        self.assertPose(0, 0, 0.7)

    def test_sequence_reports_failing_step(self):
        self.robot.docked = False
        self.robot.obstacles = [(1.0, 0.0)]
        job = self.run_cmd(
            "sequence",
            steps=[{"kind": "photo"}, {"kind": "forward", "params": {"distance": 1.5}}, {"kind": "photo"}],
        )
        self.assertEqual(job.state, Job.FAILED)
        self.assertIn("Step 2", job.message)
        self.assertEqual(Photo.objects.count(), 1)

    def test_dock_fails_when_dock_not_in_view(self):
        self.robot.docked = False
        self.robot.x, self.robot.yaw = -0.5, math.pi  # facing away from the dock
        job = self.run_cmd("dock")
        self.assertEqual(job.state, Job.FAILED)
        self.assertIn("couldn't see the dock", job.message)

    def test_undock_when_not_docked(self):
        self.robot.docked = False
        self.assertEqual(self.run_cmd("undock").state, Job.FAILED)


@override_settings(MEDIA_ROOT=TMP_MEDIA)
class ExecutorThreadTests(TransactionTestCase):
    """The real (background thread) path: busy rejection and Stop."""

    def setUp(self):
        self.robot = SimRobotClient(delay=3.0)
        self.robot.docked = False
        set_robot(self.robot)
        executor.run_inline = False

    def tearDown(self):
        executor.stop()
        deadline = time.time() + 10
        while executor.current_job_id is not None and time.time() < deadline:
            time.sleep(0.05)
        set_robot(None)

    def wait_done(self, job, timeout=10):
        deadline = time.time() + timeout
        while time.time() < deadline:
            job.refresh_from_db()
            if job.state != Job.RUNNING:
                return job
            time.sleep(0.05)
        self.fail("job did not finish")

    def test_busy_then_stop(self):
        job = executor.submit("forward", {"distance": 2.0})
        time.sleep(0.3)
        with self.assertRaises(Busy):
            executor.submit("photo", {})
        executor.stop()
        job = self.wait_done(job)
        self.assertEqual(job.state, Job.CANCELED)
        pose = self.robot.status()["pose"]
        self.assertGreater(pose.x, 0.0)
        self.assertLess(pose.x, 2.0)
        # The partial move is recorded, so Return brings the robot back from where it stopped.
        trip = Trip.objects.get()
        self.assertAlmostEqual(trip.path()[-1].x, pose.x, places=3)
        self.robot.delay = 0
        back = self.wait_done(executor.submit("return_home", {}))
        self.assertEqual(back.state, Job.SUCCEEDED, back.message)
        self.assertAlmostEqual(self.robot.status()["pose"].x, 0.0, places=3)

    def test_threads_release_after_failure(self):
        self.robot.delay = 0
        job = self.wait_done(executor.submit("undock", {}))
        self.assertEqual(job.state, Job.FAILED)
        self.assertIsNone(executor.current_job_id)
        self.assertEqual(self.wait_done(executor.submit("photo", {})).state, Job.SUCCEEDED)


class StartupCleanupTests(TestCase):
    def test_stale_running_jobs_marked_failed(self):
        stale = Job.objects.create(kind="forward", params={"distance": 1})
        executor._cleaned = False
        executor.run_inline = True
        set_robot(SimRobotClient(delay=0))
        try:
            with override_settings(MEDIA_ROOT=TMP_MEDIA):
                executor.submit("photo", {})
        finally:
            executor.run_inline = False
            set_robot(None)
        stale.refresh_from_db()
        self.assertEqual(stale.state, Job.FAILED)
        self.assertIn("restarted", stale.message)
