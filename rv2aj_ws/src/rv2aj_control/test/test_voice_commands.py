import pytest

from rv2aj_control.voice_commands import parse_command


@pytest.mark.parametrize("text,keyword,expected", [
    ("move up 20", True, ("move", "Z", 1, 20)),
    ("go left 15 millimeters", True, ("move", "Y", 1, 15)),
    ("move forward 2 cm", True, ("move", "R", 1, 20)),
    ("move back 7.5", True, ("move", "R", -1, 7.5)),
    ("move down twenty five", True, ("move", "Z", -1, 25)),
    ("move right", True, ("move", "Y", -1, 5)),
    ("move write 10", True, ("move", "Y", -1, 10)),
    ("up 20", True, ("none",)),
    ("up 20", False, ("move", "Z", 1, 20)),
    ("stop", True, ("stop",)),
    ("move up stop", True, ("stop",)),
    ("move up and left", True, ("none",)),
    ("move up 20 30", True, ("none",)),
    ("what time is it", True, ("none",)),
    ("go to point 2", True, ("goto_point", "P2")),
    ("go to P3", True, ("goto_point", "P3")),
    ("move to point three", True, ("goto_point", "P3")),
    ("go home", True, ("home",)),
    ("move home", True, ("home",)),
    ("home", True, ("none",)),
    ("start loop", True, ("start_loop",)),
    ("end loop", True, ("end_loop",)),
    ("stop the loop", True, ("end_loop",)),
])
def test_parse(text, keyword, expected):
    cmd = parse_command(text, require_keyword=keyword, default_mm=5)
    if cmd.kind == "move":
        got = (cmd.kind, cmd.axis, cmd.sign, cmd.distance_mm)
    elif cmd.kind == "goto_point":
        got = (cmd.kind, cmd.point)
    else:
        got = (cmd.kind,)
    assert got == expected
