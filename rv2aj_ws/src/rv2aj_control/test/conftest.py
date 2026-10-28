import pytest


@pytest.fixture(autouse=True)
def generic_robot(monkeypatch):
    """Tests describe a generic RV-2AJ with every joint working and wide limits.
    The real robot's joint_limits.yaml (measured ranges, frozen_joints) must not
    leak into them; tests of those features set them explicitly."""
    from rv2aj_control import kinematics
    monkeypatch.setattr(kinematics, "FROZEN_JOINTS", frozenset())
    monkeypatch.setattr(kinematics, "JOINT_LIMITS_DEG", {j: (-360.0, 360.0) for j in ("J1", "J2", "J3", "J5", "J6")})
    monkeypatch.setattr(kinematics, "ZERO_OFFSETS_DEG", {j: 0.0 for j in ("J1", "J2", "J3", "J5", "J6")})
