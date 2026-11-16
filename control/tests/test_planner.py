import math
from types import SimpleNamespace

from django.test import SimpleTestCase

from control import planner
from control.robot import Pose


class PlannerTests(SimpleTestCase):
    def test_wrap(self):
        self.assertAlmostEqual(planner.wrap(3 * math.pi / 2), -math.pi / 2)
        self.assertAlmostEqual(planner.wrap(-math.pi), math.pi)
        self.assertAlmostEqual(planner.wrap(0.1), 0.1)

    def test_leg_to(self):
        leg = planner.leg_to(Pose(1, 1, math.pi / 2), 1, 0)  # facing +y, target straight behind
        self.assertAlmostEqual(abs(leg.turn), math.pi)
        self.assertAlmostEqual(leg.distance, 1.0)
        leg = planner.leg_to(Pose(0, 0, 0), 0, 1)
        self.assertAlmostEqual(leg.turn, math.pi / 2)
        self.assertIsNone(planner.leg_to(Pose(0, 0, 0), 0.03, 0.0))

    def test_leg_capped(self):
        self.assertEqual(planner.leg_to(Pose(0, 0, 0), 5, 0).distance, planner.MAX_LEG)

    def test_turns(self):
        self.assertEqual(planner.turn_to(Pose(0, 0, 0.01), 0.0), 0.0)
        self.assertAlmostEqual(planner.turn_to(Pose(0, 0, math.radians(170)), math.radians(-170)), math.radians(20))
        self.assertAlmostEqual(planner.turn_toward(Pose(1, 0, 0), 0, 0), math.pi)

    def test_return_targets_order_and_filter(self):
        w = lambda seq, kind="move", retraced=False: SimpleNamespace(seq=seq, kind=kind, retraced=retraced)
        targets = planner.return_targets([w(0, "dock"), w(1), w(2), w(3, retraced=True)])
        self.assertEqual([t.seq for t in targets], [2, 1])
