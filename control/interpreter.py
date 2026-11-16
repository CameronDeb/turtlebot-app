"""Typed text -> commands.

Today: RuleBasedInterpreter understands short phrases ("forward 50 cm, turn left 90 and take a picture").
Later: an LLM-backed class with the same interpret(text) method can be selected with settings.COMMAND_INTERPRETER.
Whatever produces the steps, they are checked by commands.validate() before anything moves, so an interpreter can
never send the robot something the buttons couldn't.
"""

import re

from django.conf import settings
from django.utils.module_loading import import_string

from .commands import MAX_SEQUENCE, CommandError, validate


class InterpretError(ValueError):
    pass


class Interpreter:
    """Interface: interpret(text) -> list of {"kind": str, "params": dict}, already validated."""

    def interpret(self, text):
        raise NotImplementedError


def get_interpreter():
    return import_string(settings.COMMAND_INTERPRETER)()


_NUM = r"(\d+(?:\.\d+)?)"
_SPLIT = re.compile(r"\s*(?:,|;|\bthen\b|\band\b|\.(?!\d))\s*", re.I)
_UNITS = {
    "m": 1.0, "meter": 1.0, "meters": 1.0, "metre": 1.0, "metres": 1.0,
    "cm": 0.01, "centimeter": 0.01, "centimeters": 0.01, "centimetre": 0.01, "centimetres": 0.01,
    "ft": 0.3048, "foot": 0.3048, "feet": 0.3048,
    "in": 0.0254, "inch": 0.0254, "inches": 0.0254,
}
_UNIT = r"(m|meters?|metres?|cm|centimet(?:er|re)s?|ft|foot|feet|in|inch(?:es)?)?"

_FORWARD = re.compile(rf"^(?:go|move|drive|head)?\s*(?:forward|ahead|straight)(?:\s+(?:by\s+)?{_NUM}\s*{_UNIT})?$", re.I)
_FORWARD_NUM_FIRST = re.compile(rf"^(?:go|move|drive)\s+{_NUM}\s*{_UNIT}\s*(?:forward|ahead|straight)?$", re.I)
_TURN = re.compile(rf"^(?:turn|rotate|spin)\s+(left|right)(?:\s+(?:by\s+)?{_NUM}\s*(?:°|deg|degrees?)?)?$", re.I)
_TURN_NUM_FIRST = re.compile(rf"^(?:turn|rotate|spin)\s+{_NUM}\s*(?:°|deg|degrees?)?\s+(?:to\s+the\s+)?(left|right)$", re.I)
_TURN_AROUND = re.compile(r"^(?:turn|spin)\s+around$", re.I)
_PHOTO = re.compile(r"^(?:take\s+(?:a\s+)?)?(?:picture|photo|pic|snapshot|snap)(?:\s+of\s+.*)?$", re.I)
_SCAN = re.compile(r"^(?:do\s+a\s+)?(?:scan|look\s+around)(?:\s+(?:the\s+)?(?:room|area|around))?$", re.I)
_RETURN = re.compile(
    r"^(?:return|go\s+back|come\s+back|go\s+home|head\s+home)(?:\s+(?:home|to\s+(?:the\s+)?(?:dock|start|base)))?$", re.I
)
_UNDOCK = re.compile(r"^(?:undock|leave\s+the\s+dock|get\s+off\s+the\s+dock)$", re.I)
_DOCK = re.compile(r"^(?:dock|go\s+dock|dock\s+now)$", re.I)

DEFAULT_FORWARD_M = 0.5
DEFAULT_TURN_DEG = 90.0


def _metres(num, unit):
    return float(num) * _UNITS[(unit or "m").lower()]


class RuleBasedInterpreter(Interpreter):
    """Small fixed grammar; one command per clause. Unknown clauses are reported, never guessed."""

    def parse_clause(self, clause):
        c = re.sub(r"^(?:please\s+|now\s+|and\s+)+", "", clause.strip(), flags=re.I).strip()
        c = re.sub(r"\s+(?:please|now)$", "", c, flags=re.I)
        if m := _FORWARD.match(c):
            dist = _metres(m.group(1), m.group(2)) if m.group(1) else DEFAULT_FORWARD_M
            return "forward", {"distance": dist}
        if m := _FORWARD_NUM_FIRST.match(c):
            return "forward", {"distance": _metres(m.group(1), m.group(2))}
        if m := _TURN.match(c):
            deg = float(m.group(2)) if m.group(2) else DEFAULT_TURN_DEG
            return "turn", {"degrees": deg if m.group(1).lower() == "left" else -deg}
        if m := _TURN_NUM_FIRST.match(c):
            deg = float(m.group(1))
            return "turn", {"degrees": deg if m.group(2).lower() == "left" else -deg}
        if _TURN_AROUND.match(c):
            return "turn", {"degrees": 180.0}
        if _PHOTO.match(c):
            return "photo", {}
        if _SCAN.match(c):
            return "scan", {}
        if _RETURN.match(c):
            return "return_home", {}
        if _UNDOCK.match(c):
            return "undock", {}
        if _DOCK.match(c):
            return "dock", {}
        raise InterpretError(f"I don't understand \"{clause.strip()}\"")

    def interpret(self, text):
        clauses = [c for c in _SPLIT.split(text or "") if c and c.strip()]
        if not clauses:
            raise InterpretError("type a command, e.g. \"forward 50 cm, turn left 90, take a picture\"")
        if len(clauses) > MAX_SEQUENCE:
            raise InterpretError(f"at most {MAX_SEQUENCE} steps at a time")
        steps = []
        for clause in clauses:
            kind, params = self.parse_clause(clause)
            try:
                steps.append({"kind": kind, "params": validate(kind, params)})
            except CommandError as e:
                raise InterpretError(f"\"{clause.strip()}\": {e}") from None
        return steps
