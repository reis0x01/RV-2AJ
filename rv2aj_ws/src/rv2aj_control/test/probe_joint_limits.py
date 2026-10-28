#!/usr/bin/env python3
"""
Find a joint's real limit on the robot: step it in one direction until the
controller refuses / alarms or the joint stops short (touched something), then
reset the alarm, back off one step and return to where it started along the
same path. Uses the driver's own joint_move action (not the planner), so the
software limits do not stop it first. The operator must be at the E-stop.

    python3 probe_joint_limits.py J6 +     # J6 in the positive direction
Results are appended to ~/.ros/rv2aj_joint_limit_probe.txt
"""

import math
import os
import sys
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rv2aj_interfaces.action import JointMove
from rv2aj_interfaces.msg import RobotStatus
from rv2aj_interfaces.srv import Connect, SetOverride
from std_srvs.srv import SetBool, Trigger

STEP_DEG = 5.0
SPEED_PERCENT = 5
MAX_TRAVEL = {"J1": 360.0, "J2": 200.0, "J3": 200.0, "J5": 200.0, "J6": 420.0}
IDX = {"J1": 0, "J2": 1, "J3": 2, "J5": 4, "J6": 5}
LOG = os.path.expanduser("~/.ros/rv2aj_joint_limit_probe.txt")

# The robot stands on a table (its base mounting plane is Z = 0). Before every
# step the elbow, wrist, flange and gripper tip heights of the pose after the
# step are computed; the step is refused if any would come closer than
# TABLE_MARGIN_MM to the table. Geometry from kinematics.py (manual BFP-A8050).
SHOULDER_Z, UPPER_ARM, FOREARM, WRIST_TO_FLANGE = 300.0, 250.0, 160.0, 72.0
GRIPPER_MM = 120.0         # flange to gripper finger tips, generous
TABLE_MARGIN_MM = 150.0   # station parts and conveyor rails stick up above the table (photo 2026-09-28)


def lowest_point(j):
    """(name, Z mm) of the lowest arm point for joint READINGS J2/J3/J5 (zero offsets applied)."""
    from rv2aj_control.kinematics import ZERO_OFFSETS_DEG as o
    j2, j3, j5 = j["J2"] - o["J2"], j["J3"] - o["J3"], j["J5"] - o["J5"]
    t2, t23 = math.radians(j2), math.radians(j2 + j3)
    tp = math.radians(j2 + j3 + j5)
    elbow = SHOULDER_Z + UPPER_ARM * math.cos(t2)
    wrist = elbow + FOREARM * math.cos(t23)
    flange = wrist + WRIST_TO_FLANGE * math.cos(tp)
    tip = flange + GRIPPER_MM * math.cos(tp)
    return min((("elbow", elbow), ("wrist", wrist), ("flange", flange), ("gripper tip", tip)), key=lambda p: p[1])


class Probe(Node):
    def __init__(self):
        super().__init__("joint_limit_probe", namespace="/rv2aj")
        self.status = None
        self.create_subscription(RobotStatus, "status", lambda m: setattr(self, "status", m), 10)
        self.move = ActionClient(self, JointMove, "joint_move")
        self.srv = {n: self.create_client(t, n) for n, t in
                    (("servo", SetBool), ("reset_alarm", Trigger), ("set_override", SetOverride), ("stop", Trigger),
                     ("connect", Connect))}

    def spin_for(self, seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.05)

    def call(self, name, req):
        fut = self.srv[name].call_async(req)
        while not fut.done():
            rclpy.spin_once(self, timeout_sec=0.05)
        return fut.result()

    def angle(self, joint):
        self.spin_for(0.3)
        return self.status.joints_deg[IDX[joint]]

    def joints(self):
        self.spin_for(0.3)
        return {j: self.status.joints_deg[i] for j, i in IDX.items()}

    def step(self, joint, offset):
        goal = JointMove.Goal(relative=True, joint=joint, offset_deg=float(offset))
        fut = self.move.send_goal_async(goal)
        while not fut.done():
            rclpy.spin_once(self, timeout_sec=0.05)
        handle = fut.result()
        if not handle.accepted:
            return False, "goal rejected by the driver"
        res = handle.get_result_async()
        while not res.done():
            rclpy.spin_once(self, timeout_sec=0.05)
        r = res.result().result
        return r.success, r.message


def log(line):
    print(line, flush=True)
    with open(LOG, "a") as f:
        f.write(time.strftime("%Y-%m-%d %H:%M:%S  ") + line + "\n")


def rearm(p, why):
    """Reset any alarm, switch the servo off and on, set the probe speed."""
    p.call("reset_alarm", Trigger.Request())
    p.spin_for(1.0)
    if p.status.alarm:
        log(f"({why}) alarm could not be reset over serial: reset it on the pendant")
        return False
    p.call("servo", SetBool.Request(data=False))
    p.spin_for(1.5)
    r = p.call("servo", SetBool.Request(data=True))
    p.spin_for(2.0)
    o = p.call("set_override", SetOverride.Request(percent=SPEED_PERCENT))
    log(f"({why}) servo re-armed: {r.success} {r.message}; speed {SPEED_PERCENT}%: {o.success}")
    return r.success and o.success


def main():
    joint, sign = sys.argv[1].upper(), (1 if sys.argv[2] == "+" else -1)
    rclpy.init()
    p = Probe()
    p.move.wait_for_server(timeout_sec=10)
    for s in p.srv.values():
        s.wait_for_service(timeout_sec=10)
    p.spin_for(1.0)
    st = p.status
    if st is None or not st.connected:
        log("not connected to the robot; nothing done")
        return
    # On this robot the first move after an alarm reset fails (encoder alarm) unless the
    # servo is switched off and on again, so every probe starts with a fresh servo-on.
    if not rearm(p, "start"):
        log("could not re-arm the servo; nothing moved")
        return
    start = p.angle(joint)
    log(f"=== {joint} {'+' if sign > 0 else '-'} from {start:.2f} deg, {STEP_DEG:g} deg steps at {SPEED_PERCENT}% ===")
    last_good, travelled, reason = start, 0.0, "travel cap reached"
    while travelled < MAX_TRAVEL[joint]:
        after_step = p.joints()
        after_step[joint] += sign * STEP_DEG
        part, z = lowest_point(after_step)
        if z < TABLE_MARGIN_MM:
            reason = f"table guard: the next step would bring the {part} to {z:.0f} mm above the table"
            last_good = p.angle(joint)
            break
        before = p.angle(joint)
        ok, msg = p.step(joint, sign * STEP_DEG)
        p.spin_for(0.4)
        after = p.angle(joint)
        moved = (after - before) * sign
        alarm = p.status.alarm
        log(f"  {before:8.2f} -> {after:8.2f}  moved {moved:+.2f}  ok={ok} alarm={alarm}  {msg}")
        if ok and not alarm and moved > STEP_DEG * 0.8:
            last_good, travelled = after, travelled + moved
            continue
        reason = (f"alarm ({p.status.last_error or p.status.state_raw})" if alarm else
                  f"refused: {msg}" if moved < 0.05 else f"stopped short after {moved:.2f} deg: {msg}")
        if moved > 0.05 and not alarm and ok:
            last_good = after
        break
    log(f"RESULT {joint} {'+' if sign > 0 else '-'}: last good angle {last_good:.2f} deg ({reason})")
    if not p.status.connected:
        # The driver puts the robot in a safe state after a rejected move (STOP, servo off, link
        # dropped): reconnect before resetting and returning.
        r = p.call("connect", Connect.Request(port="/dev/ttyUSB0"))
        log(f"reconnect after the refusal: {r.success} {r.message}")
        p.spin_for(2.0)
    if not rearm(p, "return"):
        return
    # back to the start along the same (known free) path, in steps of at most 30 deg
    here = p.angle(joint)
    remaining = start - here
    while abs(remaining) > 0.05:
        d = math.copysign(min(30.0, abs(remaining)), remaining)
        ok, msg = p.step(joint, d)
        p.spin_for(0.4)
        if not ok:
            log(f"return step failed: {msg} (at {p.angle(joint):.2f}); stopping here")
            return
        remaining = start - p.angle(joint)
    log(f"back at {p.angle(joint):.2f} deg")
    p.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
