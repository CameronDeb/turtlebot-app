import math
import shutil
import tempfile

from django.test import SimpleTestCase, TestCase, override_settings

from control import vision
from control.commands import CommandError, validate
from control.detector import SimDetector, clean_query, set_detector
from control.executor import executor
from control.interpreter import InterpretError, RuleBasedInterpreter
from control.models import Job, Photo, Trip
from control.robot import set_robot
from control.robot.sim import SimRobotClient

TMP_MEDIA = tempfile.mkdtemp(prefix="tb_test_find_")


def tearDownModule():
    shutil.rmtree(TMP_MEDIA, ignore_errors=True)


@override_settings(MEDIA_ROOT=TMP_MEDIA)
class FindTests(TestCase):
    def setUp(self):
        self.robot = SimRobotClient(delay=0)
        self.robot.docked = False  # start off the dock at the origin, facing +x
        set_robot(self.robot)
        set_detector(SimDetector())
        executor.run_inline = True

    def tearDown(self):
        executor.run_inline = False
        set_robot(None)
        set_detector(None)

    def find(self, obj, **params):
        return executor.submit("find", {"object": obj, **params})

    def gap_to(self, ox, oy):
        """Distance from the robot's bumper to an object, and the object's bearing (deg) from the heading."""
        r = self.robot
        d = math.hypot(ox - r.x, oy - r.y)
        bearing = math.degrees(math.atan2(oy - r.y, ox - r.x) - r.yaw)
        return d - 0.17, (bearing + 180) % 360 - 180

    def test_finds_object_ahead_and_stops_at_standoff(self):
        self.robot.objects = [("red mug", 2.4, 0.3)]
        job = self.find("the Red Mug")
        self.assertEqual(job.state, Job.SUCCEEDED, job.message)
        gap, bearing = self.gap_to(2.4, 0.3)
        self.assertAlmostEqual(gap, 0.30, delta=0.08)
        self.assertLess(abs(bearing), 6)
        first = Photo.objects.order_by("id").first()
        self.assertEqual(first.detections[0]["label"], "red mug")
        self.assertIn("floor", first.detections[0]["cues"])
        self.assertTrue(job.message.startswith('Saw "red mug"'))

    def test_searches_all_the_way_round(self):
        self.robot.objects = [("backpack", -1.5, -1.2)]  # behind and to the right
        job = self.find("backpack", standoff=0.5)
        self.assertEqual(job.state, Job.SUCCEEDED, job.message)
        gap, bearing = self.gap_to(-1.5, -1.2)
        self.assertAlmostEqual(gap, 0.5, delta=0.08)
        self.assertLess(abs(bearing), 6)
        self.assertGreater(Photo.objects.filter(scan_index__isnull=False).count(), 2)

    def test_not_found_reports_and_does_not_drive(self):
        self.robot.objects = [("chair", 1.5, 0.0)]
        job = self.find("umbrella")
        self.assertEqual(job.state, Job.FAILED)
        self.assertIn("see \"umbrella\" anywhere", job.message)
        self.assertAlmostEqual(math.hypot(self.robot.x, self.robot.y), 0.0, places=6)
        self.assertGreaterEqual(Photo.objects.count(), 8)  # pictures all the way round

    def test_drives_around_an_obstacle_in_the_way(self):
        self.robot.objects = [("red mug", 3.0, 0.0)]
        self.robot.obstacles = [(1.2, 0.0)]  # a box between robot and mug
        job = self.find("red mug")
        self.assertEqual(job.state, Job.SUCCEEDED, job.message)
        gap, bearing = self.gap_to(3.0, 0.0)
        self.assertAlmostEqual(gap, 0.30, delta=0.06)
        self.assertLess(abs(bearing), 4)
        self.assertIn("planned route", job.message)
        for w in Trip.objects.get().waypoints.all():  # every stop along the way kept clear of the box
            self.assertGreater(math.hypot(w.x - 1.2, w.y), 0.29)

    def test_goes_round_a_table_instead_of_under_the_chairs(self):
        # Today's real run, as a test: the plant is behind a table with chairs; the lidar sees their legs ~0.45 m
        # apart (it can see the plant through the gaps), but not the chair bases on the floor between them.
        legs = [(1.5, y / 100) for y in range(-110, 111, 45)] + [(2.1, y / 100) for y in range(-110, 111, 45)]
        self.robot.obstacles = legs
        self.robot.low_obstacles = [(1.5, y / 100) for y in range(-110, 71, 5)]  # sled bases between the legs
        self.robot.objects = [("potted plant", 3.2, 0.0)]
        job = self.find("potted plant")
        self.assertEqual(job.state, Job.SUCCEEDED, job.message)
        gap, _ = self.gap_to(3.2, 0.0)
        self.assertAlmostEqual(gap, 0.30, delta=0.06)

    def test_no_route_reports_instead_of_ramming(self):
        wall = [(1.0, y / 100) for y in range(-400, 401, 10)]  # a wall the lidar sees, object behind it
        self.robot.obstacles = wall
        self.robot.objects = [("box", 2.0, 0.0)]
        job = self.find("box")
        self.assertEqual(job.state, Job.FAILED)
        self.assertIn("no clear way", job.message)
        self.assertAlmostEqual(math.hypot(self.robot.x, self.robot.y), 0.0, places=6)

    def test_low_obstacle_stops_find_instead_of_ramming(self):
        # Today's real run: the plant was seen through a gap under chairs whose low bases the lidar can't see.
        self.robot.objects = [("potted plant", 3.0, 0.0)]
        self.robot.low_obstacles = [(1.0, 0.0)]  # chair base on the floor
        job = self.find("potted plant")
        self.assertEqual(job.state, Job.FAILED)
        self.assertIn("stopped making progress", job.message)
        self.assertLess(self.robot.x, 1.0 - 0.17 + 0.01)  # stopped at the bar, not through it
        self.assertEqual(executor.submit("return_home", {}).state, Job.SUCCEEDED)  # and can still get back
        self.assertAlmostEqual(math.hypot(self.robot.x, self.robot.y), 0.0, places=2)

    def test_tall_object_seen_by_lidar(self):
        self.robot.objects = [("trash can", 2.0, 0.0)]
        self.robot.obstacles = [(1.96, 0.0)]  # its front surface, seen by the lidar
        job = self.find("trash can", standoff=0.15)
        self.assertEqual(job.state, Job.SUCCEEDED, job.message)
        gap, _ = self.gap_to(1.96, 0.0)
        self.assertGreaterEqual(gap, 0.09)
        self.assertLess(gap, 0.25)

    def test_where_is_turns_without_driving(self):
        self.robot.objects = [("chair", 0.0, 2.0)]  # to the left
        job = self.find("chair", approach=False)
        self.assertEqual(job.state, Job.SUCCEEDED, job.message)
        self.assertAlmostEqual(math.hypot(self.robot.x, self.robot.y), 0.0, places=6)
        _, bearing = self.gap_to(0.0, 2.0)
        self.assertLess(abs(bearing), 4)
        self.assertIn("turned to face it", job.message)

    def test_undocks_first_and_return_brings_it_back(self):
        self.robot.docked = True
        self.robot.objects = [("box", -2.0, 0.0)]
        job = self.find("box")
        self.assertEqual(job.state, Job.SUCCEEDED, job.message)
        self.assertEqual(executor.submit("return_home", {}).state, Job.SUCCEEDED)
        self.assertTrue(self.robot.docked)


class FindParsingTests(SimpleTestCase):
    def test_validate(self):
        self.assertEqual(
            validate("find", {"object": "  The RED mug!! "}),
            {"object": "red mug", "approach": True, "standoff": 0.3, "max_range": 4.0},
        )
        for bad in ({}, {"object": ""}, {"object": "!!"}, {"object": 3}, {"object": "x", "standoff": 5}):
            with self.assertRaises(CommandError):
                validate("find", bad)
        self.assertEqual(clean_query("my blue backpack"), "blue backpack")

    def test_interpreter(self):
        i = RuleBasedInterpreter()
        self.assertEqual(i.interpret("find the red mug")[0]["params"]["object"], "red mug")
        self.assertTrue(i.interpret("go to the chair")[0]["params"]["approach"])
        self.assertFalse(i.interpret("where is the backpack?")[0]["params"]["approach"])
        self.assertEqual(i.interpret("go to the dock")[0]["kind"], "return_home")
        self.assertEqual(i.interpret("turn to the left")[0], {"kind": "turn", "params": {"degrees": 90.0}})
        steps = i.interpret("undock, then find the water bottle")
        self.assertEqual([s["kind"] for s in steps], ["undock", "find"])
        with self.assertRaises(InterpretError):
            i.interpret("face the dock")


class VisionTests(SimpleTestCase):
    INTR = {"fx": 323.5, "fy": 323.5, "cx": 125.0, "cy": 125.0}
    CAM = {"x": -0.06, "y": 0.0, "z": 0.244, "yaw_deg": 0.0, "pitch_deg": 0.0}

    def test_bearing_sign(self):
        self.assertGreater(vision.bearing_deg(10, self.INTR), 0)  # left of the image = left of the robot
        self.assertLess(vision.bearing_deg(240, self.INTR), 0)
        self.assertAlmostEqual(vision.bearing_deg(125, self.INTR), 0.0)

    def test_floor_distance(self):
        # a point 1.0 m ahead on the floor appears at row cy + fy * h / d
        v = 125 + 323.5 * 0.244 / 1.0
        self.assertAlmostEqual(vision.floor_distance(v, 250, self.INTR, self.CAM), 1.0, places=3)
        self.assertIsNone(vision.floor_distance(249, 250, self.INTR, self.CAM))  # cut off by the image edge
        self.assertIsNone(vision.floor_distance(120, 250, self.INTR, self.CAM))  # above the horizon
        self.assertAlmostEqual(vision.horizontal_fov_deg(type("P", (), {"intrinsics": self.INTR, "width": 250})), 42.25, 1)
