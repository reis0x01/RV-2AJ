"""Exact kinematics of the RV-2AJ (5 axes: J1, J2, J3, J5, J6; no J4).

Dimensions from the Mitsubishi RV-1A/RV-2AJ Standard Specifications Manual
(BFP-A8050, Fig. 2-5): shoulder (J2) axis 300 mm above the mounting face,
upper arm 250 mm, forearm 160 mm, J5 axis to flange face 72 mm, no lateral
offsets. Base frame and angle conventions are the controller's own and were
checked against poses reported by this robot (PPOSF vs JPOSF, < 0.05 mm):

  X, Y, Z   flange centre in mm, Z up, X forward at J1 = 0
  B         tool pitch from vertical = J2 + J3 + J5
  A         Mitsubishi's 5-axis roll: A = J1 * cos(B) + J6   (degrees)

All angles are degrees. Joint limits come from rv2aj_description's
config/joint_limits.yaml; the controller's MEJAR limits are the real authority.
"""

import math

SHOULDER_HEIGHT = 300.0
UPPER_ARM = 250.0
FOREARM = 160.0
WRIST_TO_FLANGE = 72.0

# Operating ranges from the manual, except J3: this robot's J3 origin is
# miscalibrated, so it reads up to ~146° at real poses (upper end widened) and
# the controller alarmed at 12.3° when moving J3 down (2026-09-24), so the
# lower end is raised to 15°.
def _load_limits():
    """Joint limits from rv2aj_description/config/joint_limits.yaml, the same file
    the URDF uses, so MoveIt and the planner always agree."""
    import os
    import yaml
    from ament_index_python.packages import get_package_share_directory
    path = os.path.join(get_package_share_directory("rv2aj_description"), "config", "joint_limits.yaml")
    with open(path) as f:
        data = yaml.safe_load(f)
    return {j: (float(data[j][0]), float(data[j][1])) for j in ("J1", "J2", "J3", "J5", "J6")}


JOINT_LIMITS_DEG = _load_limits()


def _load_frozen():
    """frozen_joints from the same joint_limits.yaml: joints that must not move."""
    import os
    import yaml
    from ament_index_python.packages import get_package_share_directory
    path = os.path.join(get_package_share_directory("rv2aj_description"), "config", "joint_limits.yaml")
    with open(path) as f:
        data = yaml.safe_load(f) or {}
    return frozenset(str(j).upper() for j in (data.get("frozen_joints") or []))


FROZEN_JOINTS = _load_frozen()


def _load_offsets():
    """zero_offsets_deg from joint_limits.yaml: the joint READING when that joint is
    physically at the model's zero (J2/J3/J5: arm straight up). The controller's
    origin can be set so that its zeros differ from the real straight-up pose;
    all geometry here uses (reading - offset). Joint limits stay in readings."""
    import os
    import yaml
    from ament_index_python.packages import get_package_share_directory
    path = os.path.join(get_package_share_directory("rv2aj_description"), "config", "joint_limits.yaml")
    with open(path) as f:
        data = yaml.safe_load(f) or {}
    raw = data.get("zero_offsets_deg") or {}
    return {j: float(raw.get(j, 0.0)) for j in ("J1", "J2", "J3", "J5", "J6")}


ZERO_OFFSETS_DEG = _load_offsets()


def calibrated():
    """True when the model corrects the controller's zeros (then the controller's own
    X/Y/Z and straight-line moves are in a wrong frame and must not be used)."""
    return any(abs(v) > 1e-9 for v in ZERO_OFFSETS_DEG.values())


class IKError(ValueError):
    pass


def fk(j1, j2, j3, j5, j6):
    """Joint READINGS (deg) -> dict(X, Y, Z, A, B) in mm / deg (zero offsets applied)."""
    o = ZERO_OFFSETS_DEG
    j1, j2, j3, j5, j6 = j1 - o["J1"], j2 - o["J2"], j3 - o["J3"], j5 - o["J5"], j6 - o["J6"]
    t2 = math.radians(j2)
    t23 = math.radians(j2 + j3)
    pitch = j2 + j3 + j5
    tp = math.radians(pitch)
    r = UPPER_ARM * math.sin(t2) + FOREARM * math.sin(t23) + WRIST_TO_FLANGE * math.sin(tp)
    z = SHOULDER_HEIGHT + UPPER_ARM * math.cos(t2) + FOREARM * math.cos(t23) + WRIST_TO_FLANGE * math.cos(tp)
    t1 = math.radians(j1)
    return {
        "X": r * math.cos(t1),
        "Y": r * math.sin(t1),
        "Z": z,
        "A": j1 * math.cos(tp) + j6,
        "B": pitch,
    }


def _wrap(angle):
    return (angle + 180.0) % 360.0 - 180.0


def ik(x, y, z, pitch_deg, seed, j6=None, ref=None):
    """Position + tool pitch -> joints (deg) nearest to `seed`. With J5 frozen
    the pitch is not kept: J5 stays at the seed's value and the pitch follows."""
    if "J5" not in FROZEN_JOINTS:
        return _ik(x, y, z, pitch_deg, seed, j6, ref)
    j5 = seed["J5"]
    o = ZERO_OFFSETS_DEG
    pitch = pitch_deg
    for _ in range(40):  # pitch = J2 + J3 + J5: fixed point, the 72 mm wrist offset makes it converge fast
        q = _ik(x, y, z, pitch, seed, j6, ref)
        new_pitch = (q["J2"] - o["J2"]) + (q["J3"] - o["J3"]) + (j5 - o["J5"])
        if abs(new_pitch - pitch) < 1e-6:
            break
        pitch = new_pitch
    q = _ik(x, y, z, pitch, seed, j6, ref)
    if abs(q["J5"] - j5) > 0.01:
        raise IKError(f"({x:.1f}, {y:.1f}, {z:.1f}) is not reachable with J5 held at {j5:.1f}°")
    q["J5"] = j5
    return q


def _ik(x, y, z, pitch_deg, seed, j6=None, ref=None):
    """Position + tool pitch -> joints (deg) nearest to `seed`.

    seed: dict with J1, J2, J3, J5, J6 (the current pose). J6 is kept from
    the seed unless given. Raises IKError when the point is out of reach or
    every solution violates the joint limits. `ref` (default: the seed) is the
    pose used for the "no worse than now" limit rule.
    """
    ref = seed if ref is None else ref
    tp = math.radians(pitch_deg)
    candidates = []
    base_j1 = math.degrees(math.atan2(y, x))
    radius = math.hypot(x, y)
    # Reaching forward (r > 0) or back over the base (r < 0, J1 turned 180°).
    o = ZERO_OFFSETS_DEG
    for j1, r in ((base_j1, radius), (_wrap(base_j1 + 180.0), -radius)):
        j1 = j1 + o["J1"]
        j1 = seed["J1"] + _wrap(j1 - seed["J1"])  # continuous with the seed (readings)
        rw = r - WRIST_TO_FLANGE * math.sin(tp)
        zw = z - SHOULDER_HEIGHT - WRIST_TO_FLANGE * math.cos(tp)
        c3 = (rw * rw + zw * zw - UPPER_ARM ** 2 - FOREARM ** 2) / (2 * UPPER_ARM * FOREARM)
        if c3 < -1.0 - 1e-9 or c3 > 1.0 + 1e-9:
            continue
        c3 = max(-1.0, min(1.0, c3))
        for sign in (1.0, -1.0):
            t3 = sign * math.acos(c3)
            t2 = math.atan2(rw, zw) - math.atan2(FOREARM * math.sin(t3), UPPER_ARM + FOREARM * math.cos(t3))
            j2 = math.degrees(t2)
            j3 = math.degrees(t3)
            j5 = pitch_deg - j2 - j3
            # model angles -> controller readings
            q = {"J1": j1, "J2": j2 + o["J2"], "J3": j3 + o["J3"], "J5": j5 + o["J5"],
                 "J6": seed["J6"] if j6 is None else j6}
            candidates.append(q)

    valid = [q for q in candidates if within_limits(q, ref)]
    if not valid:
        if not candidates:
            raise IKError(f"({x:.1f}, {y:.1f}, {z:.1f}) is out of reach")
        raise IKError(f"({x:.1f}, {y:.1f}, {z:.1f}) needs joint angles outside the limits")
    return min(valid, key=lambda q: sum((q[k] - seed[k]) ** 2 for k in q))


def _joint_ok(j, value, ref, margin):
    lo, hi = JOINT_LIMITS_DEG[j]
    if lo - margin <= value <= hi + margin:
        return True
    # "No worse than now": the controller's own limits (parameter MEJAR) can be
    # wider than the manual's, so a joint already past the modelled limit may
    # stay there or move back toward the range, never further out.
    if ref is not None and j in ref:
        r = ref[j]
        if value < lo and r < lo and value >= r - margin:
            return True
        if value > hi and r > hi and value <= r + margin:
            return True
    return False


def within_limits(q, ref=None, margin=1e-6):
    return all(_joint_ok(j, q[j], ref, margin) for j in JOINT_LIMITS_DEG if j in q)


def limit_violations(q, ref=None, margin=1e-6):
    out = []
    for j, (lo, hi) in JOINT_LIMITS_DEG.items():
        if j in q and not _joint_ok(j, q[j], ref, margin):
            already = ref is not None and j in ref and not lo <= ref[j] <= hi
            out.append(f"{j} {q[j]:.1f}° (limits {lo:g}..{hi:g}"
                       + (f"; already at {ref[j]:.1f}°, may only move back toward the range)" if already else ")"))
    return out


def straight_line(start_joints, delta_xyz, step_mm=5.0):
    """Joint waypoints for a straight TCP line from the current pose.

    Keeps the tool pitch B and J6 fixed (what the controller does for a 5-axis
    linear move) and solves IK every `step_mm`. Returns a list of joint dicts,
    excluding the start pose. Raises IKError if any point is unreachable or
    the solution jumps (elbow flip / J1 swing) between consecutive points.
    """
    p0 = fk(**{k.lower(): v for k, v in start_joints.items()})
    length = math.sqrt(sum(d * d for d in delta_xyz))
    steps = max(1, math.ceil(length / step_mm))
    waypoints = []
    seed = dict(start_joints)
    for i in range(1, steps + 1):
        f = i / steps
        q = ik(p0["X"] + f * delta_xyz[0], p0["Y"] + f * delta_xyz[1], p0["Z"] + f * delta_xyz[2],
               p0["B"], seed, ref=start_joints)
        jump = max(abs(q[k] - seed[k]) for k in q)
        if jump > 30.0:
            raise IKError(f"the straight line passes a singularity (joint jump of {jump:.0f}°)")
        waypoints.append(q)
        seed = q
    return waypoints
