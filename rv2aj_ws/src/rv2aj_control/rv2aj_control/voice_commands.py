"""Turn recognised speech into TCP commands.

Examples: "move up 20", "go left 15 millimeters", "move forward 2 cm",
"move down twenty five", "move right" (uses the default distance),
"go to point 2", "go home", "start loop", "end loop",
"stop" anywhere in the phrase -> stop.
"""

import re
from dataclasses import dataclass

# Up/down and left/right use the robot base frame (Z, Y). Forward/back use
# "R": out along the arm's reach / back toward the base, whichever way the
# arm is pointing (base-frame X was wrong whenever the arm faced away from +X).
DIRECTIONS = {
    "up": ("Z", +1),
    "down": ("Z", -1),
    "left": ("Y", +1),
    "right": ("Y", -1),
    "forward": ("R", +1),
    "back": ("R", -1),
}

SYNONYMS = {
    "upward": "up", "upwards": "up", "raise": "up", "higher": "up",
    "downward": "down", "downwards": "down", "lower": "down",
    "forwards": "forward", "front": "forward", "ahead": "forward", "foreword": "forward",
    "backward": "back", "backwards": "back", "backs": "back",
    "write": "right", "rite": "right", "wright": "right", "lefts": "left",
}

STOP_WORDS = {"stop", "halt", "freeze", "abort", "emergency"}
KEYWORDS = {"move", "go", "robot", "moves", "moving"}

UNIT_SCALE = {
    "mm": 1.0, "millimeter": 1.0, "millimeters": 1.0, "millimetre": 1.0, "millimetres": 1.0,
    "cm": 10.0, "centimeter": 10.0, "centimeters": 10.0, "centimetre": 10.0, "centimetres": 10.0,
}

_SMALL = {
    "zero": 0, "one": 1, "two": 2, "three": 3, "four": 4, "five": 5, "six": 6, "seven": 7,
    "eight": 8, "nine": 9, "ten": 10, "eleven": 11, "twelve": 12, "thirteen": 13,
    "fourteen": 14, "fifteen": 15, "sixteen": 16, "seventeen": 17, "eighteen": 18,
    "nineteen": 19,
}
_TENS = {"twenty": 20, "thirty": 30, "forty": 40, "fifty": 50, "sixty": 60,
         "seventy": 70, "eighty": 80, "ninety": 90}


@dataclass
class VoiceCommand:
    kind: str                 # "stop", "move", "goto_point", "home", "start_loop", "end_loop", "none"
    axis: str = ""
    sign: int = 0
    distance_mm: float = 0.0
    direction: str = ""
    point: str = ""
    reason: str = ""

    def describe(self):
        if self.kind == "stop":
            return "STOP"
        if self.kind == "move":
            return f"{self.direction} {self.distance_mm:g} mm"
        if self.kind == "goto_point":
            return f"go to {self.point}"
        if self.kind == "home":
            return "go home"
        if self.kind == "start_loop":
            return "start loop"
        if self.kind == "end_loop":
            return "end loop"
        return f"ignored ({self.reason})"


def _tokens(text):
    text = text.lower().replace("-", " ")
    return re.findall(r"\d+(?:\.\d+)?|[a-z]+", text)


def _numbers(tokens):
    """Return the numbers spoken, as digits or words ("twenty five" -> 25)."""
    found = []
    current = None
    for tok in tokens:
        if re.fullmatch(r"\d+(?:\.\d+)?", tok):
            if current is not None:
                found.append(current)
                current = None
            found.append(float(tok))
        elif tok in _SMALL or tok in _TENS:
            value = _SMALL.get(tok, _TENS.get(tok))
            if current is None:
                current = value
            elif tok in _SMALL and current % 10 == 0 and current >= 20 and value < 10:
                current += value  # "twenty five"
            elif current >= 100 and current % 100 == 0:
                current += value  # "one hundred twenty"
            else:
                found.append(current)
                current = value
        elif tok == "hundred":
            current = (current or 1) * 100
        elif tok == "and" and current is not None:
            continue
        elif current is not None:
            found.append(current)
            current = None
    if current is not None:
        found.append(current)
    return [float(n) for n in found]


LOOP_END_WORDS = {"end", "stop", "finish", "cancel", "exit"}
LOOP_START_WORDS = {"start", "begin", "run", "play"}
POINT_WORDS = {"point", "p", "pee", "position"}


def _point_name(tokens):
    """'go to point 2' / 'go to P2' / 'move to point three' -> 'P2'."""
    for i, tok in enumerate(tokens[:-1]):
        if tok in POINT_WORDS:
            numbers = _numbers(tokens[i + 1:i + 3])
            if numbers and numbers[0] == int(numbers[0]) and numbers[0] > 0:
                return f"P{int(numbers[0])}"
    return None


def parse_command(text, require_keyword=True, default_mm=5.0):
    tokens = [SYNONYMS.get(t, t) for t in _tokens(text)]
    if not tokens:
        return VoiceCommand("none", reason="nothing heard")

    if "loop" in tokens and any(t in LOOP_END_WORDS for t in tokens):
        return VoiceCommand("end_loop")
    if any(t in STOP_WORDS for t in tokens):
        return VoiceCommand("stop")
    if "loop" in tokens and any(t in LOOP_START_WORDS for t in tokens):
        return VoiceCommand("start_loop")

    if "home" in tokens and not any(t in DIRECTIONS for t in tokens):
        if require_keyword and not any(t in KEYWORDS for t in tokens):
            return VoiceCommand("none", reason="say 'go home'")
        return VoiceCommand("home")

    point = _point_name(tokens)
    if point and not any(t in DIRECTIONS for t in tokens):
        if require_keyword and not any(t in KEYWORDS for t in tokens):
            return VoiceCommand("none", reason="say 'go to' before the point")
        return VoiceCommand("goto_point", point=point)

    directions = {t for t in tokens if t in DIRECTIONS}
    if not directions:
        return VoiceCommand("none", reason="no direction word")
    if len(directions) > 1:
        return VoiceCommand("none", reason="more than one direction: " + ", ".join(sorted(directions)))
    if require_keyword and not any(t in KEYWORDS for t in tokens):
        return VoiceCommand("none", reason="say 'move' before the direction")

    numbers = _numbers(tokens)
    if len(numbers) > 1:
        return VoiceCommand("none", reason="more than one number")
    distance = numbers[0] if numbers else default_mm
    distance *= next((UNIT_SCALE[t] for t in tokens if t in UNIT_SCALE), 1.0)
    if distance <= 0:
        return VoiceCommand("none", reason="distance must be above zero")

    direction = directions.pop()
    axis, sign = DIRECTIONS[direction]
    return VoiceCommand("move", axis=axis, sign=sign, distance_mm=distance, direction=direction)
