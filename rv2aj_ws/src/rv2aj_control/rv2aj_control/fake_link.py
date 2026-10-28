"""Simulated CR1-571 for use_fake_hardware and tests.

Speaks the same command subset as the real controller (as verified on
hardware), with motion that takes time and scales with the override.
The reported TCP always follows the joints through the RV-2AJ kinematics
(kinematics.py, checked against the real controller), and TCP moves are
solved with the same IK, so the simulator agrees with MoveIt and the robot.
"""

import re
import threading
import time

from . import kinematics
from .serial_link import SerialLink, SerialLinkError

JOINT_SPEED_DEG_S = 60.0   # at 100 % override


class _Axis:
    """Values that move linearly from a start to a target over time."""

    def __init__(self, values):
        self.start = list(values)
        self.target = list(values)
        self.t0 = 0.0
        self.duration = 0.0

    def now(self):
        if self.duration <= 0:
            return list(self.target)
        f = min(1.0, (time.monotonic() - self.t0) / self.duration)
        return [a + (b - a) * f for a, b in zip(self.start, self.target)]

    def move_to(self, target, speed):
        self.start = self.now()
        self.target = list(target)
        distance = max(abs(a - b) for a, b in zip(self.start, self.target))
        self.duration = distance / speed if speed > 0 else 0.0
        self.t0 = time.monotonic()

    def halt(self):
        self.target = self.now()
        self.start = list(self.target)
        self.duration = 0.0

    def moving(self):
        return self.duration > 0 and time.monotonic() - self.t0 < self.duration


class FakeLink(SerialLink):
    def __init__(self):
        super().__init__()
        self._open = False
        self._sim_lock = threading.Lock()
        self.joints = _Axis([-148.60, -11.49, 140.55, 0.0, -29.59, 51.34])
        self.override = 10
        self.control = False
        self.servo = False
        self.alarm = False
        self.jtmp = None
        self.ptmp = None
        self.sent = []
        self.hand = None
        self.line_inputs = 0     # simulated Station 4 order bits (set with line/set sim_order)
        self.line_outputs = 0
        self.io = {("IN", 8): 0, ("OUT", 8): 0, ("IN", 900): 0, ("OUT", 900): 0}  # other simulated I/O bytes

    @property
    def is_open(self):
        return self._open

    def connect(self, port="fake"):
        self._open = True

    def disconnect(self):
        self._open = False

    def send(self, command):
        with self._sim_lock:
            if not self._open:
                raise SerialLinkError("port not open")
            self.sent.append(command)
            time.sleep(0.005)
            reply = self._handle(command)
        return reply + "\r"

    def _tool(self):
        j = self.joints.now()
        p = kinematics.fk(j[0], j[1], j[2], j[4], j[5])
        return [p["X"], p["Y"], p["Z"], p["A"], p["B"]]

    # -- command handling ------------------------------------------------

    def _ok(self):
        return "Qok" if self.alarm else "QoK"

    def _error(self, code="R4220"):
        self.alarm = True
        return f"Qe{code}000000"

    def _handle(self, c):
        k = self._ok()
        if c == "STATE":
            return f"{k}1.MB4;10;30;4;01060;1;1;;;;;;;;;REP;START;1;0"
        if c == "JPOSF":
            j = self.joints.now()
            fields = ";".join("J4;****" if i == 3 else f"J{i + 1};{v:.2f}" for i, v in enumerate(j))
            return f"{k}{fields};;****,****;30;0.00;00000000"
        if c == "PPOSF":
            p = self._tool()
            return (f"{k}X;{p[0]:.2f};Y;{p[1]:.2f};Z;{p[2]:.2f};A;{p[3]:.2f};B;{p[4]:.2f};;"
                    f"6,0;30;0.00;00000000")
        if c in ("OPEN", "CLOSE"):
            return k
        if c == "CNTLON":
            self.control = True
            return k
        if c == "CNTLOFF":
            self.control = False
            self.servo = False
            return k
        if c == "SRVON":
            if not self.control:
                return self._error("R6010")
            self.servo = True
            return k
        if c == "SRVOFF":
            self.servo = False
            self.joints.halt()
            return k
        if c == "RSTALRM":
            self.alarm = False
            return "QoK"
        if c == "STOP":
            self.joints.halt()
            return k
        if c == "VALM_INB(0)":
            return f"{k}M_INB=+{self.line_inputs}"
        if c == "VALM_OUTB(0)":
            return f"{k}M_OUTB=+{self.line_outputs}"
        m = re.fullmatch(r"VALM_(IN|OUT)B\((8|900)\)", c)
        if m:
            return f"{k}M_{m[1]}B({m[2]})=+{self.io[(m[1], int(m[2]))]}"
        m = re.fullmatch(r"EXECM_OUT\((\d+)\)=([01])", c)
        if m:
            if not self.control:
                return self._error("R6010")
            n, v = int(m[1]), int(m[2])
            if n < 8:
                self.line_outputs = self.line_outputs & ~(1 << n) | (v << n)
            else:
                base = 8 if n < 16 else 900
                b = n - base
                self.io[("OUT", base)] = self.io[("OUT", base)] & ~(1 << b) | (v << b)
            return k
        m = re.fullmatch(r"EXECJOVRD (\d+)", c)
        if m:
            self.override = max(1, min(100, int(m.group(1))))
            return k
        if c.startswith("RUN"):
            return k
        m = re.fullmatch(r"EXECH(OPEN|CLOSE) (\d+)", c)
        if m:
            self.hand = m.group(1).lower()
            return k

        if c == "EXECJTMP=J_CURR":
            self.jtmp = [round(v, 2) for v in self.joints.now()]
            return k
        if re.fullmatch(r"EXECJTMP=.*\(.*", c):
            # Tuple forms (JTMP=(...), J_CURR+(...)) are not confirmed on the real CR1-571;
            # J_CURR+(...) raised L4220 there. Reject them so the simulation cannot hide their use.
            return self._error()
        m = re.fullmatch(r"EXECJTMP\.J(\d)=JTMP\.J\1\+RAD\((-?\d+(?:\.\d+)?)\)", c)
        if m and self.jtmp:
            self.jtmp[int(m.group(1)) - 1] = round(self.jtmp[int(m.group(1)) - 1] + float(m.group(2)), 2)
            return k
        if c == "VALJTMP" and self.jtmp:
            return f"{k}JTMP=(" + ",".join(f"{v:+.2f}" for v in self.jtmp) + ")"
        if c == "EXECMOV JTMP" and self.jtmp:
            if not self.servo:
                return self._error("H0010")
            self.jtmp[3] = 0.0
            self.joints.move_to(self.jtmp, JOINT_SPEED_DEG_S * self.override / 100)
            return k

        if c == "EXECPTMP=P_CURR":
            self.ptmp = [round(v, 2) for v in self._tool()]
            return k
        m = re.fullmatch(r"EXECPTMP\.([XYZ])=PTMP\.\1([+-])(\d+(?:\.\d+)?)", c)
        if m and self.ptmp:
            i = "XYZ".index(m.group(1))
            d = float(m.group(3)) * (1 if m.group(2) == "+" else -1)
            self.ptmp[i] = round(self.ptmp[i] + d, 2)
            return k
        if c == "VALPTMP" and self.ptmp:
            return f"{k}PTMP=(" + ",".join(f"{v:+.2f}" for v in self.ptmp) + ")(6,0)"
        if c == "EXECMOV PTMP" and self.ptmp:
            if not self.servo:
                return self._error("H0010")
            j = self.joints.now()
            seed = {"J1": j[0], "J2": j[1], "J3": j[2], "J5": j[4], "J6": j[5]}
            try:
                q = kinematics.ik(self.ptmp[0], self.ptmp[1], self.ptmp[2], self.ptmp[4], seed)
            except kinematics.IKError:
                return self._error("SIM_OUT_OF_REACH")
            target = [q["J1"], q["J2"], q["J3"], 0.0, q["J5"], q["J6"]]
            self.joints.move_to(target, JOINT_SPEED_DEG_S * self.override / 100)
            return k

        return self._error()
