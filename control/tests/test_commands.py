from django.test import SimpleTestCase

from control.commands import CommandError, describe, validate
from control.interpreter import InterpretError, RuleBasedInterpreter


class ValidateTests(SimpleTestCase):
    def test_forward_defaults_and_limits(self):
        self.assertEqual(validate("forward", {"distance": 0.5}), {"distance": 0.5, "check_clear": True})
        for bad in (0, 0.01, 2.5, -1, "1", None, True, float("nan")):
            with self.assertRaises(CommandError):
                validate("forward", {"distance": bad})
        with self.assertRaises(CommandError):
            validate("forward", {"distance": 1, "check_clear": "no"})

    def test_turn(self):
        self.assertEqual(validate("turn", {"degrees": -90}), {"degrees": -90.0})
        for bad in (0, 0.5, 361, -400, "left"):
            with self.assertRaises(CommandError):
                validate("turn", {"degrees": bad})

    def test_scan_steps(self):
        self.assertEqual(validate("scan", {}), {"steps": 8})
        with self.assertRaises(CommandError):
            validate("scan", {"steps": 2})
        with self.assertRaises(CommandError):
            validate("scan", {"steps": 6.5})

    def test_unknown_kind(self):
        with self.assertRaises(CommandError):
            validate("fly", {})

    def test_sequence(self):
        seq = validate("sequence", {"steps": [{"kind": "forward", "params": {"distance": 1}}, {"kind": "photo"}]})
        self.assertEqual(seq["steps"][1], {"kind": "photo", "params": {}})
        with self.assertRaises(CommandError):
            validate("sequence", {"steps": []})
        with self.assertRaises(CommandError):
            validate("sequence", {"steps": [{"kind": "sequence", "params": {"steps": [{"kind": "photo"}]}}]})
        with self.assertRaisesMessage(CommandError, "step 2"):
            validate("sequence", {"steps": [{"kind": "photo"}, {"kind": "forward", "params": {"distance": 9}}]})

    def test_describe(self):
        self.assertEqual(describe("turn", {"degrees": -45.0}), "Turn right 45°")
        self.assertEqual(describe("forward", {"distance": 0.25, "check_clear": True}), "Drive forward 0.25 m")


class InterpreterTests(SimpleTestCase):
    def setUp(self):
        self.i = RuleBasedInterpreter()

    def kinds(self, text):
        return [(s["kind"], s["params"]) for s in self.i.interpret(text)]

    def test_sentence(self):
        self.assertEqual(
            self.kinds("forward 50 cm, turn left 90 and take a picture"),
            [
                ("forward", {"distance": 0.5, "check_clear": True}),
                ("turn", {"degrees": 90.0}),
                ("photo", {}),
            ],
        )

    def test_variants(self):
        self.assertEqual(self.kinds("go forward 1.5 m")[0][1]["distance"], 1.5)
        self.assertEqual(self.kinds("move 2 feet forward")[0][1]["distance"], 0.61)
        self.assertEqual(self.kinds("turn right 30 degrees")[0][1]["degrees"], -30.0)
        self.assertEqual(self.kinds("rotate 45 to the left")[0][1]["degrees"], 45.0)
        self.assertEqual(self.kinds("turn around")[0][1]["degrees"], 180.0)
        self.assertEqual(self.kinds("Please scan the room")[0][0], "scan")
        self.assertEqual(self.kinds("go home")[0][0], "return_home")
        self.assertEqual(self.kinds("return to the dock")[0][0], "return_home")
        self.assertEqual(self.kinds("undock then forward")[1][1]["distance"], 0.5)
        self.assertEqual(self.kinds("forward 0.25. photo")[0][1]["distance"], 0.25)

    def test_rejects_unknown_and_out_of_range(self):
        with self.assertRaisesMessage(InterpretError, "don't understand"):
            self.i.interpret("make me a sandwich")
        with self.assertRaisesMessage(InterpretError, "between"):
            self.i.interpret("forward 5 m")
        with self.assertRaises(InterpretError):
            self.i.interpret("   ")
