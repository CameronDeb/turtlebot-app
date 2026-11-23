import math
import shutil
import tempfile

from django.test import SimpleTestCase, TestCase, override_settings
from django.urls import reverse

from control import memory
from control.commands import CommandError, validate
from control.detector import SimDetector, set_detector
from control.executor import executor
from control.interpreter import RuleBasedInterpreter
from control.models import Job, ObjectMemory
from control.robot import set_robot
from control.robot.sim import SimRobotClient

TMP_MEDIA = tempfile.mkdtemp(prefix="tb_test_memory_")


def tearDownModule():
    shutil.rmtree(TMP_MEDIA, ignore_errors=True)


class FrameTests(SimpleTestCase):
    def test_round_trip(self):
        m2o = (1.2, -0.7, 0.8)
        for p in [(0.0, 0.0), (2.0, 1.0), (-3.5, 0.25)]:
            back = memory.to_map(memory.to_odom(p, m2o), m2o)
            self.assertAlmostEqual(back[0], p[0], places=9)
            self.assertAlmostEqual(back[1], p[1], places=9)

    def test_known_transform(self):
        # map frame rotated 90 deg and shifted (1, 0) in odom: map (1, 0) is odom (1, 1)
        x, y = memory.to_odom((1.0, 0.0), (1.0, 0.0, math.pi / 2))
        self.assertAlmostEqual(x, 1.0)
        self.assertAlmostEqual(y, 1.0)

    def test_no_map_leaves_points_alone(self):
        self.assertEqual(memory.to_map((1.0, 2.0), None), (1.0, 2.0))


class SightingTests(TestCase):
    def test_merges_nearby_sightings_of_the_same_label(self):
        a = memory.record_sighting("chair", (2.0, 1.0), 0.8, (0.0, 0.0), (0, 0, 0), distance=2.2)
        b = memory.record_sighting("Chair", (2.1, 1.1), 0.6, (0.0, 2.0), (0, 0, 0), distance=2.0)
        self.assertEqual(a.id, b.id)
        b.refresh_from_db()
        self.assertEqual(b.views, 2)
        self.assertEqual(len(b.view_dirs), 2)  # seen from two different directions
        self.assertTrue(2.0 < b.x < 2.1 and 1.0 < b.y < 1.1)
        self.assertEqual(b.best_score, 0.8)
        self.assertEqual(b.last_score, 0.6)

    def test_two_objects_with_the_same_label_stay_apart(self):
        memory.record_sighting("chair", (2.0, 1.0), 0.8, (0, 0), (0, 0, 0))
        memory.record_sighting("chair", (2.0, -1.0), 0.8, (0, 0), (0, 0, 0))
        self.assertEqual(ObjectMemory.objects.filter(label="chair").count(), 2)

    def test_weak_or_far_sightings_are_not_kept(self):
        self.assertIsNone(memory.record_sighting("chair", (2, 1), 0.3, (0, 0), (0, 0, 0)))
        self.assertIsNone(memory.record_sighting("chair", (6, 0), 0.9, (0, 0), (0, 0, 0), distance=6.0))
        self.assertEqual(ObjectMemory.objects.count(), 0)

    def test_survives_an_odom_reset(self):
        # Session 1: map and odom line up; the plant is at map (2, 1).
        first = memory.record_sighting("potted plant", (2.0, 1.0), 0.7, (0, 0), (0.0, 0.0, 0.0))
        # Session 2: the base rebooted, so odom restarted somewhere else; slam reports where the map now sits.
        m2o = (-1.0, 0.5, math.pi / 2)
        plant_odom = memory.to_odom((2.0, 1.0), m2o)
        second = memory.record_sighting("potted plant", plant_odom, 0.7, memory.to_odom((0, 0), m2o), m2o)
        self.assertEqual(first.id, second.id)
        second.refresh_from_db()
        self.assertEqual(second.views, 2)
        self.assertAlmostEqual(second.x, 2.0, places=6)
        self.assertAlmostEqual(second.y, 1.0, places=6)

    def test_without_a_map_positions_are_kept_in_odom(self):
        e = memory.record_sighting("chair", (2.0, 1.0), 0.8, (0, 0), None)
        self.assertEqual(e.frame, "odom")
        # a later sighting with a map doesn't merge into the odom entry (different frames)
        e2 = memory.record_sighting("chair", (2.0, 1.0), 0.8, (0, 0), (0, 0, 0))
        self.assertNotEqual(e.id, e2.id)


@override_settings(MEDIA_ROOT=TMP_MEDIA)
class RobotMemoryTests(TestCase):
    def setUp(self):
        self.robot = SimRobotClient(delay=0)
        self.robot.docked = False
        set_robot(self.robot)
        set_detector(SimDetector())
        executor.run_inline = True

    def tearDown(self):
        executor.run_inline = False
        set_robot(None)
        set_detector(None)

    def test_find_remembers_where_the_object_is(self):
        self.robot.objects = [("backpack", 2.5, 0.4)]
        job = executor.submit("find", {"object": "backpack"})
        self.assertEqual(job.state, Job.SUCCEEDED, job.message)
        entries = list(ObjectMemory.objects.filter(label="backpack"))
        self.assertEqual(len(entries), 1, [str(e) for e in entries])
        e = entries[0]
        self.assertEqual(e.frame, "map")
        self.assertLess(math.hypot(e.x - 2.5, e.y - 0.4), 0.2)
        self.assertGreaterEqual(e.views, 2)  # first look, looks on the way, the final estimate
        self.assertIsNotNone(e.last_photo_id)
        self.assertIn(e.id, job.result["memory_ids"])

    def test_positions_are_stored_in_the_map_frame(self):
        self.robot.map_frame = (1.0, -0.5, 0.3)  # the sim's odom is its world; the map sits elsewhere
        self.robot.objects = [("backpack", 2.5, 0.4)]
        job = executor.submit("find", {"object": "backpack", "approach": False})
        self.assertEqual(job.state, Job.SUCCEEDED, job.message)
        e = ObjectMemory.objects.get(label="backpack")
        ox, oy = memory.to_odom((e.x, e.y), self.robot.map_frame)
        self.assertLess(math.hypot(ox - 2.5, oy - 0.4), 0.25)
        self.assertGreater(math.hypot(e.x - 2.5, e.y - 0.4), 0.5)  # really converted

    def test_scan_with_objects_remembers_each_one(self):
        self.robot.objects = [("chair", 2.0, 0.0), ("chair", -2.0, 0.5), ("backpack", 0.3, 2.0)]
        job = executor.submit("scan", {"steps": 8, "objects": ["chair", "backpack", "red mug"]})
        self.assertEqual(job.state, Job.SUCCEEDED, job.message)
        self.assertEqual(ObjectMemory.objects.filter(label="chair").count(), 2)
        self.assertEqual(ObjectMemory.objects.filter(label="backpack").count(), 1)
        self.assertFalse(ObjectMemory.objects.filter(label="red mug").exists())
        self.assertIn("Looked for chair, backpack, red mug", job.message)

    def test_plain_scan_remembers_nothing(self):
        self.robot.objects = [("chair", 2.0, 0.0)]
        job = executor.submit("scan", {"steps": 8})
        self.assertEqual(job.state, Job.SUCCEEDED, job.message)
        self.assertEqual(ObjectMemory.objects.count(), 0)

    def test_state_api_lists_memory(self):
        memory.record_sighting("chair", (2.0, 1.0), 0.8, (0, 0), (0, 0, 0))
        d = self.client.get(reverse("control:api_state")).json()
        self.assertEqual([m["label"] for m in d["memory"]], ["chair"])


class ScanCommandTests(SimpleTestCase):
    def test_validate_objects(self):
        self.assertEqual(validate("scan", {"objects": "Chair, the backpack,, chair"}),
                         {"steps": 8, "objects": ["chair", "backpack"]})
        self.assertEqual(validate("scan", {}), {"steps": 8})
        with self.assertRaises(CommandError):
            validate("scan", {"objects": [f"thing{i}" for i in range(9)]})
        with self.assertRaises(CommandError):
            validate("scan", {"objects": [3]})

    def test_interpreter(self):
        kind, params = RuleBasedInterpreter().parse_clause("scan for chair, backpack and potted plant")
        self.assertEqual(kind, "scan")
        self.assertEqual(params["objects"], ["chair", "backpack", "potted plant"])

    def test_interpreter_keeps_the_list_together(self):
        steps = RuleBasedInterpreter().interpret("scan for chair, backpack and potted plant then dock")
        self.assertEqual([s["kind"] for s in steps], ["scan", "dock"])
        self.assertEqual(steps[0]["params"]["objects"], ["chair", "backpack", "potted plant"])
        steps = RuleBasedInterpreter().interpret("forward 50 cm, turn left 90 and take a picture")
        self.assertEqual([s["kind"] for s in steps], ["forward", "turn", "photo"])
