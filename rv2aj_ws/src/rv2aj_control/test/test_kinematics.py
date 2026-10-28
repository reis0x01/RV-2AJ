import random

import pytest

from rv2aj_control.kinematics import IKError, fk, ik, straight_line

# (J1, J2, J3, J5, J6) and the controller's own PPOSF for that pose, read from this robot.
REAL = [
    ((-158.60, -6.49, 145.55, -34.60, 61.33), (-136.21, -53.38, 409.55, 100.94, 104.46)),
    ((-148.60, -11.49, 140.55, -29.59, 51.34), (-124.15, -75.79, 432.32, 75.78, 99.47)),
]


@pytest.mark.parametrize("joints,pose", REAL)
def test_fk_matches_controller(joints, pose):
    p = fk(*joints)
    for key, value in zip("XYZAB", pose):
        assert p[key] == pytest.approx(value, abs=0.02)


def test_ik_round_trip():
    rng = random.Random(1)
    for _ in range(500):
        q = {"J1": rng.uniform(-150, 150), "J2": rng.uniform(-60, 120), "J3": rng.uniform(-110, 120),
             "J5": rng.uniform(-90, 90), "J6": rng.uniform(-200, 200)}
        p = fk(q["J1"], q["J2"], q["J3"], q["J5"], q["J6"])
        s = ik(p["X"], p["Y"], p["Z"], p["B"], q)
        p2 = fk(s["J1"], s["J2"], s["J3"], s["J5"], s["J6"])
        assert all(p2[k] == pytest.approx(p[k], abs=1e-6) for k in "XYZB")


def test_straight_line_keeps_pitch_and_reaches_target():
    start = {"J1": -148.6, "J2": -11.49, "J3": 140.55, "J5": -29.59, "J6": 51.34}
    p0 = fk(-148.6, -11.49, 140.55, -29.59, 51.34)
    q = straight_line(start, (0.0, 0.0, 20.0))[-1]
    p = fk(q["J1"], q["J2"], q["J3"], q["J5"], q["J6"])
    assert p["Z"] == pytest.approx(p0["Z"] + 20, abs=1e-6)
    assert p["B"] == pytest.approx(p0["B"], abs=1e-9)
    assert q["J6"] == start["J6"]


def test_out_of_reach():
    with pytest.raises(IKError):
        ik(600.0, 0.0, 300.0, 90.0, {"J1": 0, "J2": 0, "J3": 90, "J5": 0, "J6": 0})


def test_frozen_j5_straight_line_keeps_j5(monkeypatch):
    from rv2aj_control import kinematics as k
    monkeypatch.setattr(k, "FROZEN_JOINTS", frozenset({"J5"}))
    q0 = {"J1": -115.58, "J2": -62.28, "J3": 110.18, "J5": -82.05, "J6": -80.68}
    p0 = k.fk(**{j.lower(): v for j, v in q0.items()})
    end = k.straight_line(q0, (0.0, 0.0, -50.0), 5.0)[-1]
    p1 = k.fk(**{j.lower(): v for j, v in end.items()})
    assert end["J5"] == q0["J5"]
    assert abs(p1["Z"] - (p0["Z"] - 50.0)) < 0.05 and abs(p1["X"] - p0["X"]) < 0.05 and abs(p1["Y"] - p0["Y"]) < 0.05


def test_zero_offsets_roundtrip(monkeypatch):
    """With the controller zeros shifted, fk(readings) is the real pose and ik returns readings."""
    from rv2aj_control import kinematics as k
    monkeypatch.setattr(k, "ZERO_OFFSETS_DEG", {"J1": 0.0, "J2": -70.0, "J3": 60.0, "J5": -175.0, "J6": 0.0})
    physical = {"J1": -30.0, "J2": 10.0, "J3": 80.0, "J5": 20.0, "J6": 0.0}
    readings = {"J1": -30.0, "J2": -60.0, "J3": 140.0, "J5": -155.0, "J6": 0.0}
    monkeypatch.setattr(k, "ZERO_OFFSETS_DEG", {"J1": 0.0, "J2": 0.0, "J3": 0.0, "J5": 0.0, "J6": 0.0})
    p_true = k.fk(*(physical[j] for j in ("J1", "J2", "J3", "J5", "J6")))
    monkeypatch.setattr(k, "ZERO_OFFSETS_DEG", {"J1": 0.0, "J2": -70.0, "J3": 60.0, "J5": -175.0, "J6": 0.0})
    p = k.fk(*(readings[j] for j in ("J1", "J2", "J3", "J5", "J6")))
    assert all(abs(p[a] - p_true[a]) < 1e-6 for a in ("X", "Y", "Z", "B"))
    q = k.ik(p["X"], p["Y"], p["Z"], p["B"], readings)
    assert all(abs(q[j] - readings[j]) < 1e-6 for j in ("J1", "J2", "J3", "J5"))
