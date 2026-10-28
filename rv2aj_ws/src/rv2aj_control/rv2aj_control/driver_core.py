"""Hardware side of the driver, independent of ROS.

Owns the serial link and a single worker thread that runs queued commands
one at a time and polls STATE / PPOSF / JPOSF in between. Motions are
tracked until the robot settles on the target (or stops short).

Safety rules enforced here, whoever the caller is:
  - Connecting only starts the read-only poll. Servo, moves and RUN need
    explicit calls.
  - Moves need: connected, servo on, no other motion running, and stay
    within the configured size limits.
  - STOP is sent immediately (not queued) and also cancels a queued move.
  - Any link error or non-Qo reply drops the link: STOP, SRVOFF and
    CNTLOFF are attempted, the port is closed, and nothing reconnects
    until connect() is called again.
  - The speed override resets to the configured default on every connect.
"""

import dataclasses
import math
import queue
import threading
import time
from concurrent.futures import Future

from .serial_link import (
    OUT_BUSY,
    OUT_STATION_READY,
    RADIAL,
    STEPPABLE_JOINTS,
    TCP_AXES,
    SerialLinkError,
    is_ok_reply,
    parse_jposf,
    parse_pposf,
    set_line_error_code,
    set_line_outputs,
    set_output_bit,
)

JOINT_NAMES = ("J1", "J2", "J3", "J4", "J5", "J6")


def _alarm_number(state):
    """Error number (and axis) from STATE field 5, e.g. "29060" -> "9060", "810693" -> "1069 (axis 3)"."""
    try:
        field = state.strip()[3:].split(";")[4].strip()
    except IndexError:
        return ""
    if not field.isdigit():
        return ""
    if len(field) == 6:
        return f"{int(field[1:5])} (axis {field[5]})"
    return str(int(field[-4:]))
MOVABLE_JOINTS = tuple(j for j in JOINT_NAMES if int(j[1]) in STEPPABLE_JOINTS)


@dataclasses.dataclass
class Limits:
    default_override: int = 10
    max_tcp_move_mm: float = 50.0
    max_joint_step_deg: float = 45.0
    # JPOSF is read every cycle (one query is ~0.1 s at 9600 baud); STATE and
    # PPOSF only every poll_interval_s (moving_poll_interval_s while moving).
    joint_poll_interval_s: float = 0.03
    poll_interval_s: float = 1.0
    moving_poll_interval_s: float = 0.5
    servo_on_settle_s: float = 1.5
    joint_tolerance_deg: float = 0.05
    tcp_tolerance_mm: float = 0.2
    reached_settle_s: float = 0.2    # at target and unchanged this long -> reached
    stopped_settle_s: float = 1.0    # unchanged this long away from target -> stopped
    motion_timeout_s: float = 120.0


@dataclasses.dataclass
class Status:
    connected: bool = False
    servo_on: bool = False
    alarm: bool = False
    alarm_text: str = ""        # "9060 User High level error" while an alarm is active
    moving: bool = False
    active_motion: str = ""
    override_percent: int = 10
    joints: dict = dataclasses.field(default_factory=dict)   # J1..J6 -> deg or None
    tool: dict = dataclasses.field(default_factory=dict)     # X,Y,Z,A,B -> mm/deg
    state_raw: str = ""
    last_error: str = ""
    gripper: str = "unknown"   # last *commanded* state; unknown until commanded
    line_inputs: int = -1      # transfer line (XPA4) input bits 0-7, -1 until read
    line_outputs: int = -1     # transfer line output bits 0-7, -1 until read
    inputs_8_15: int = -1      # further I/O (pneumatics), -1 until read
    outputs_8_15: int = -1
    hand_inputs: int = -1      # 900-907
    hand_outputs: int = -1

    def copy(self):
        return dataclasses.replace(self, joints=dict(self.joints), tool=dict(self.tool))


class Motion:
    """A move in progress. Wait on `done`; then read success/message/final."""

    def __init__(self, label, space):
        self.label = label
        self.space = space          # "joints" or "tool"
        self.targets = None         # set once the controller accepted the move
        self.last = None
        self.stable_since = None
        self.started = time.monotonic()
        self.cancelled = False
        self.done = threading.Event()
        self.success = False
        self.message = ""
        self.final = {}

    def finish(self, success, message, final=None):
        if self.done.is_set():
            return
        self.success = success
        self.message = message
        self.final = dict(final or {})
        self.done.set()


class _Cancelled(Exception):
    pass


def _frozen():
    """Joints frozen in joint_limits.yaml (hardware fault): they are never moved."""
    try:
        from .kinematics import FROZEN_JOINTS
        return FROZEN_JOINTS
    except Exception:  # no ROS package index (plain unit tests): nothing frozen
        return frozenset()


def _calibrated():
    try:
        from .kinematics import calibrated
        return calibrated()
    except Exception:
        return False


def _calibrated_tool(joints):
    """Tool X/Y/Z/A/B from the corrected model when zero offsets are set, else None."""
    if not _calibrated() or any(joints.get(j) is None for j in ("J1", "J2", "J3", "J5", "J6")):
        return None
    from .kinematics import fk
    return fk(joints["J1"], joints["J2"], joints["J3"], joints["J5"], joints["J6"])


class DriverCore:
    def __init__(self, link_factory, limits=None, on_status=None, on_log=None):
        self._link_factory = link_factory
        self.limits = limits or Limits()
        self._on_status = on_status or (lambda status: None)
        self._on_log = on_log or (lambda level, text: None)
        self._lock = threading.RLock()
        self._link = None
        self._thread = None
        self._running = False
        self._jobs = queue.Queue()
        self._motion = None
        self._io_rotate = 0        # which higher I/O byte to refresh next (round-robin)
        self._last_motion_end = 0.0  # monotonic time the last move ended (0 = long ago)
        self._status = Status(override_percent=self.limits.default_override)

    # -- state -------------------------------------------------------------

    def status(self):
        with self._lock:
            status = self._status.copy()
            status.moving = self._motion is not None
            status.active_motion = self._motion.label if self._motion else ""
            return status

    # Callback failures must never kill the worker thread or a caller.
    def _publish(self):
        try:
            self._on_status(self.status())
        except Exception as e:  # noqa: BLE001
            print(f"rv2aj driver: status callback failed: {e}")

    def _log(self, level, text):
        try:
            self._on_log(level, text)
        except Exception as e:  # noqa: BLE001
            print(f"rv2aj driver: log callback failed ({e}): {text}")

    # -- connection --------------------------------------------------------

    def connect(self, port):
        with self._lock:
            if self._running:
                return False, "already connected"
            link = self._link_factory()
            try:
                link.connect(port)
            except SerialLinkError as e:
                self._status.last_error = str(e)
                return False, str(e)
            self._link = link
            self._jobs = queue.Queue()
            self._motion = None
            self._status = Status(connected=True, override_percent=self.limits.default_override)
            self._running = True
            self._thread = threading.Thread(target=self._worker, name="rv2aj-serial", daemon=True)
            self._thread.start()
        self._log("info", f"Connected to {port}; servo is off")
        self._publish()
        return True, f"connected to {port}"

    def disconnect(self):
        if not self.status().connected:
            return True, "not connected"
        self._shutdown("disconnected")
        self._log("info", "Disconnected (STOP, SRVOFF, CNTLOFF sent)")
        return True, "disconnected"

    def _fail(self, message):
        self._log("error", f"{message}. Link dropped, reconnect to continue.")
        with self._lock:
            self._status.last_error = message
        self._shutdown(f"link error: {message}")

    def _shutdown(self, reason):
        with self._lock:
            was_running = self._running
            self._running = False
            link, thread = self._link, self._thread
            motion, self._motion = self._motion, None
            jobs, self._jobs = self._jobs, queue.Queue()
        if thread is not None and thread is not threading.current_thread():
            thread.join(timeout=5)
        if was_running and link is not None and link.is_open:
            for command in ("STOP", "SRVOFF", "CNTLOFF"):
                try:
                    link.send(command)
                except SerialLinkError:
                    break
        if link is not None:
            link.disconnect()
        while True:
            try:
                _, _, future = jobs.get_nowait()
            except queue.Empty:
                break
            if not future.done():
                future.set_exception(SerialLinkError(reason))
        if motion is not None:
            motion.finish(False, f"aborted: {reason}")
        with self._lock:
            self._status.connected = False
            self._status.servo_on = False
        self._publish()

    # -- queued commands ---------------------------------------------------

    def _submit(self, name, fn):
        future = Future()
        with self._lock:
            if not self._running:
                future.set_exception(SerialLinkError("not connected"))
                return future
            self._jobs.put((name, fn, future))
        return future

    def _call(self, name, fn, timeout=15.0):
        """Run fn on the worker thread and wait. Returns (ok, result_or_message)."""
        future = self._submit(name, fn)
        try:
            return True, future.result(timeout=timeout)
        except SerialLinkError as e:
            return False, str(e)
        except Exception as e:  # noqa: BLE001 - report any failure to the caller
            return False, f"{name} failed: {e}"

    def servo(self, on):
        if not self.status().connected:
            return False, "not connected"
        link = self._link
        if on:
            override = self.status().override_percent

            def servo_on():
                link.expect_ok("CNTLON")
                link.expect_ok("SRVON")
                time.sleep(self.limits.servo_on_settle_s)
                link.expect_ok(f"EXECJOVRD {override}")
                with self._lock:
                    self._status.servo_on = True

            ok, msg = self._call("servo on", servo_on)
            if ok:
                self._log("warn", f"Servo ON, override {override}%")
                self._publish()
                return True, f"servo on, override {override}%"
            return False, msg

        with self._lock:
            if self._motion is not None:
                self._motion.cancelled = True

        def servo_off():
            link.expect_ok("SRVOFF")
            with self._lock:
                self._status.servo_on = False

        ok, msg = self._call("servo off", servo_off)
        if ok:
            self._log("info", "Servo OFF")
            self._publish()
            return True, "servo off"
        return False, msg

    def reset_alarm(self):
        """RSTALRM; if the servo was on, also switch it off and on again: on this robot
        the first move after an alarm reset fails otherwise (verified 2026-09-28)."""
        link = self._link
        servo_was_on = self.status().servo_on
        override = self.status().override_percent

        def reset():
            link.expect_ok("RSTALRM")
            if servo_was_on:
                link.expect_ok("SRVOFF")
                time.sleep(1.0)
                link.expect_ok("CNTLON")
                link.expect_ok("SRVON")
                time.sleep(self.limits.servo_on_settle_s)
                link.expect_ok(f"EXECJOVRD {override}")

        ok, msg = self._call("reset alarm", reset, timeout=20.0)
        if ok:
            self._log("info", "Alarm reset sent" + (" (servo switched off and on again)" if servo_was_on else ""))
            return True, "alarm reset" + (", servo re-armed" if servo_was_on else "")
        return False, msg

    def set_override(self, percent):
        percent = int(percent)
        if not 1 <= percent <= 100:
            return False, "override must be 1-100 %"
        link = self._link

        def set_it():
            link.expect_ok("CNTLON")   # EXECJOVRD needs operation rights (else L6020 "operation is disabled")
            link.expect_ok(f"EXECJOVRD {percent}")
            with self._lock:
                self._status.override_percent = percent

        ok, msg = self._call("set override", set_it)
        if ok:
            self._log("info", f"Override set to {percent}%")
            self._publish()
            return True, f"override {percent}%"
        return False, msg

    def gripper(self, open_):
        """Open or close the pneumatic gripper. Needs no servo power; never
        sent automatically. Records the commanded state only on success."""
        if not self.status().connected:
            return False, "not connected"
        link = self._link
        word = "open" if open_ else "close"

        def run():
            link.expect_ok("CNTLON")   # EXEC commands need operation rights
            (link.open_gripper if open_ else link.close_gripper)()
            with self._lock:
                self._status.gripper = "open" if open_ else "closed"

        ok, msg = self._call(f"gripper {word}", run)
        if ok:
            self._log("info", f"Gripper {word} command accepted (EXECH{word.upper()})")
            self._publish()
            return True, f"gripper {word} accepted"
        with self._lock:
            self._status.gripper = "unknown"
        self._publish()
        return False, msg

    def line_set(self, signal, value):
        """Set a transfer-line signal (ready / busy / error_code) with read-back.
        Explicit requests only; never sent automatically."""
        if not self.status().connected:
            return False, "not connected", -1
        link = self._link
        signal = signal.strip().lower()
        if signal == "sim_order":
            if not hasattr(link, "line_inputs"):
                return False, "sim_order is only available with the simulated controller", -1
            link.line_inputs = (link.line_inputs & 0b00111111) | ((int(value) & 3) << 6)
            return True, f"simulated order bits {int(value) & 3:02b}", self.status().line_outputs
        if signal.startswith("sim_in") and signal[6:].isdigit():
            # Simulated controller only: set one input bit (e.g. "sim_in5" for the
            # auto-run part sensor), so auto-run can be exercised without hardware.
            if not hasattr(link, "line_inputs"):
                return False, "sim inputs are only available with the simulated controller", -1
            bit = int(signal[6:])
            if not 0 <= bit <= 7:
                return False, "sim input bit must be 0-7", -1
            if value:
                link.line_inputs |= (1 << bit)
            else:
                link.line_inputs &= ~(1 << bit)
            return True, f"sim IN{bit}={1 if value else 0}", self.status().line_outputs
        if signal in ("ready", "busy"):
            if value not in (0, 1):
                return False, "value must be 0 or 1", -1
            bit = OUT_STATION_READY if signal == "ready" else OUT_BUSY
            job = lambda: set_line_outputs(link, {bit: value})
        elif signal.startswith("out") and signal[3:].isdigit():
            # Any single output 0-15 or 900-907 (operator unlocked all outputs 2026-10-16).
            if value not in (0, 1):
                return False, "value must be 0 or 1", -1
            n = int(signal[3:])
            fields = {0: "line_outputs", 8: "outputs_8_15", 900: "hand_outputs"}
            result = {}

            def run_any():
                base, byte = set_output_bit(link, n, value)
                with self._lock:
                    setattr(self._status, fields[base], byte)
                result["outputs"] = byte

            ok, msg = self._call(f"OUT{n}={value}", run_any)
            if ok:
                self._log("info", f"Output OUT{n} = {value} (byte {result['outputs']:08b})")
                self._publish()
                return True, f"OUT{n} = {value}", self.status().line_outputs
            return False, msg, -1
        elif signal == "error_code":
            if not 0 <= value <= 3:
                return False, "error code must be 0-3", -1
            job = lambda: set_line_error_code(link, value)
        else:
            return False, f"unknown signal {signal!r} (ready, busy, error_code, out<n>)", -1
        result = {}

        def run():
            result["outputs"] = job()
            with self._lock:
                self._status.line_outputs = result["outputs"]

        ok, msg = self._call(f"line {signal}={value}", run)
        if ok:
            self._log("info", f"Transfer line: {signal} = {value} (outputs {result['outputs']:08b})")
            self._publish()
            return True, f"{signal} = {value}", result["outputs"]
        return False, msg, -1

    def run_program(self, program):
        if self._motion is not None:
            return False, "a move is in progress"
        ok, msg = self._call("run program", lambda: self._link.expect_ok(f"RUN {int(program)}"))
        if ok:
            self._log("warn", f"RUN sent for program {program} (untested on this controller)")
            return True, f"RUN {program} sent"
        return False, msg

    def stop(self):
        """Send STOP now, bypassing the queue, and cancel any move."""
        with self._lock:
            link = self._link
            motion = self._motion
            if motion is not None:
                motion.cancelled = True
        if link is None or not link.is_open:
            return False, "not connected"
        try:
            reply = link.send("STOP")
        except SerialLinkError as e:
            self._fail(f"STOP: {e}")
            return False, str(e)
        self._log("warn", f"STOP sent ({reply.strip()})")
        return True, "STOP sent"

    # -- motions -------------------------------------------------------------

    def _check_can_move(self):
        status = self.status()
        if not status.connected:
            return "not connected"
        if not status.servo_on:
            return "servo is off"
        if status.moving:
            return f"busy: {status.active_motion}"
        return None

    def tcp_move(self, axis, distance_mm):
        if _calibrated():
            return None, ("the controller's own straight-line move uses its uncorrected zeros; "
                          "use the planner's TCP move (joint_limits.yaml zero_offsets_deg)")
        if _frozen():
            return None, (f"{', '.join(sorted(_frozen()))} frozen (encoder fault): the controller's own straight-line "
                          "move would use it; use the planner's TCP move instead")
        return self._tcp_move(axis, distance_mm)

    def _tcp_move(self, axis, distance_mm):
        """Start a relative TCP move. Returns (Motion, None) or (None, reason)."""
        axis = axis.upper()
        if axis not in TCP_AXES and axis != RADIAL:
            return None, f"unknown axis {axis!r}; use X, Y, Z or R"
        if distance_mm == 0 or not math.isfinite(distance_mm):
            return None, "distance must be non-zero"
        if abs(distance_mm) > self.limits.max_tcp_move_mm:
            return None, f"{abs(distance_mm):g} mm is over the {self.limits.max_tcp_move_mm:g} mm limit"
        link = self._link
        override = self.status().override_percent
        where = "along arm" if axis == RADIAL else axis
        label = f"TCP {where} {distance_mm:+g} mm"
        return self._start_motion(label, "tool",
                                  lambda: link.send_tcp_move(axis, distance_mm, override))

    def joint_step(self, joint, offset_deg):
        joint = joint.upper()
        if joint not in MOVABLE_JOINTS:
            return None, f"{joint} cannot be moved on this robot"
        if joint in _frozen():
            return None, f"{joint} is frozen (encoder fault) and does not move"
        if offset_deg == 0 or not math.isfinite(offset_deg):
            return None, "offset must be non-zero"
        if abs(offset_deg) > self.limits.max_joint_step_deg:
            return None, f"{abs(offset_deg):g}° is over the {self.limits.max_joint_step_deg:g}° step limit"
        link = self._link
        override = self.status().override_percent
        n = int(joint[1])

        def run():
            return {joint: link.send_step_move(n, offset_deg, override)}

        return self._start_motion(f"{joint} step {offset_deg:+g}°", "joints", run)

    def joint_move(self, targets_deg, label="joint move"):
        values = [0.0 if v is None or not math.isfinite(v) else float(v) for v in targets_deg]
        if len(values) != 6:
            return None, "need 6 joint values"
        now = self.status().joints
        for j in _frozen():  # a frozen joint keeps its current angle whatever the caller asked
            if now.get(j) is not None and math.isfinite(now[j]):
                values[JOINT_NAMES.index(j)] = now[j]
        link = self._link
        override = self.status().override_percent

        def run():
            reached = link.send_joint_move(values, override)
            return {j: reached[i] for i, j in enumerate(JOINT_NAMES) if j in MOVABLE_JOINTS}

        return self._start_motion(label, "joints", run)

    def _start_motion(self, label, space, run):
        with self._lock:
            reason = self._check_can_move()
            if reason:
                return None, reason
            motion = Motion(label, space)
            self._motion = motion

        def job():
            if motion.cancelled:
                raise _Cancelled()
            targets = run()
            with self._lock:
                motion.targets = targets

        future = self._submit(label, job)
        future.add_done_callback(lambda f: self._motion_job_done(motion, f))
        self._log("info", f"{label} at {self.status().override_percent}%")
        self._publish()
        return motion, None

    def _motion_job_done(self, motion, future):
        error = future.exception()
        if error is None:
            return
        with self._lock:
            if self._motion is motion:
                self._motion = None
                self._last_motion_end = time.monotonic()
        if isinstance(error, _Cancelled):
            motion.finish(False, "cancelled before it started")
        else:
            motion.finish(False, str(error))
        self._publish()

    def _check_settle(self):
        with self._lock:
            motion = self._motion
            if motion is None or motion.targets is None:
                return
            readings = self._status.joints if motion.space == "joints" else self._status.tool
            current = {k: readings.get(k) for k in motion.targets}
        if any(v is None for v in current.values()):
            return

        now = time.monotonic()
        if motion.last is not None and all(abs(current[k] - motion.last[k]) < 0.01 for k in current):
            if motion.stable_since is None:
                motion.stable_since = now
        else:
            motion.stable_since = None
        motion.last = current
        stable_for = now - motion.stable_since if motion.stable_since is not None else 0.0

        tolerance = self.limits.tcp_tolerance_mm if motion.space == "tool" else self.limits.joint_tolerance_deg
        reached = all(abs(current[k] - t) <= tolerance for k, t in motion.targets.items())
        timed_out = now - motion.started > self.limits.motion_timeout_s
        if not ((reached and stable_for >= self.limits.reached_settle_s)
                or stable_for >= self.limits.stopped_settle_s or timed_out):
            return

        with self._lock:
            if self._motion is motion:
                self._motion = None
                self._last_motion_end = time.monotonic()
        unit = " mm" if motion.space == "tool" else "°"
        if timed_out and not reached:
            try:
                self._link.send("STOP")
            except SerialLinkError:
                pass
            motion.finish(False, f"timed out after {self.limits.motion_timeout_s:g} s; STOP sent", current)
            self._log("error", f"{motion.label}: timed out, STOP sent")
        elif reached:
            motion.finish(True, "reached", current)
            self._log("info", f"{motion.label}: reached")
        else:
            short = ", ".join(f"{k} {current[k]:.2f}{unit} (target {t:.2f}{unit})"
                              for k, t in motion.targets.items() if abs(current[k] - t) > tolerance)
            why = "stopped" if motion.cancelled else "stopped short"
            alarm = self.status().alarm_text
            if alarm and not motion.cancelled:
                why = f"controller alarm {alarm}, stopped"
            motion.finish(False, f"{why}: {short}", current)
            self._log("warn", f"{motion.label}: {why}: {short}")

    # -- worker thread ------------------------------------------------------

    def _worker(self):
        last_full = 0.0
        while self._running:
            if not self._run_jobs():
                return
            motion = self._motion
            full_every = self.limits.moving_poll_interval_s if motion else self.limits.poll_interval_s
            # A TCP-space move is judged on PPOSF, so read it fresh every cycle then.
            full = (motion is not None and motion.space == "tool") or time.monotonic() - last_full >= full_every
            if not self._poll(full):
                return
            if full:
                last_full = time.monotonic()
            self._check_settle()
            self._publish()
            self._sleep(self.limits.joint_poll_interval_s)

    def _sleep(self, seconds):
        end = time.monotonic() + seconds
        while self._running and time.monotonic() < end and self._jobs.empty():
            time.sleep(0.02)

    def _run_jobs(self):
        while self._running:
            try:
                name, fn, future = self._jobs.get_nowait()
            except queue.Empty:
                return True
            try:
                result = fn()
            except SerialLinkError as e:
                future.set_exception(e)
                self._fail(f"{name}: {e}")
                return False
            except Exception as e:  # noqa: BLE001 - surfaced to the caller via the future
                future.set_exception(e)
                continue
            future.set_result(result)
        return False

    def _query(self, command):
        """Read-only query. A garbled or empty reply (line noise, not a 'Q...'
        answer) is retried once; a real controller reply is never retried."""
        reply = self._link.send(command)
        if not reply.startswith("Q"):
            self._log("warn", f"{command}: garbled reply {reply!r}, retrying once")
            time.sleep(0.05)
            reply = self._link.send(command)
        return reply

    def _poll(self, full=True):
        """JPOSF every call; STATE (link/alarm check) and PPOSF when `full`."""
        state = pposf = line_in = line_out = None
        extra = {}
        try:
            if full:
                state = self._query("STATE")
                if not is_ok_reply(state):
                    self._fail(f"STATE returned {state.strip()!r}")
                    return False
                pposf = self._query("PPOSF")
                line_in = self._query("VALM_INB(0)")
                line_out = self._query("VALM_OUTB(0)")
                # Refresh ONE of the higher I/O bytes per full poll, round-robin, so the
                # Robot IO panel's IN8-15 / OUT8-15 / hand bytes stay live even during a
                # loop (they all refresh within a few polls) — without the 4 back-to-back
                # reads that used to stall the next move by ~0.4 s at each point (operator
                # 2026-10-20/21). Byte 0 (order / ready / busy / error) is read every full
                # poll above; these higher bytes are less time-critical. Skipped only when
                # a real job is already waiting, so a queued move never sits behind a read.
                IO_BYTES = (("inputs_8_15", "VALM_INB(8)"), ("outputs_8_15", "VALM_OUTB(8)"),
                            ("hand_inputs", "VALM_INB(900)"), ("hand_outputs", "VALM_OUTB(900)"))
                settled = self._motion is None and (time.monotonic() - self._last_motion_end) > 1.0
                if self._jobs.empty():
                    if settled:
                        # truly idle (operator watching): read all 4 so IO is instant
                        for field, cmd in IO_BYTES:
                            if not self._jobs.empty():
                                break      # a move was just queued: stop, let it go
                            extra[field] = self._query(cmd)
                    else:
                        # just moved / mid-loop: one per poll so the next move is not delayed
                        field, cmd = IO_BYTES[self._io_rotate % len(IO_BYTES)]
                        self._io_rotate += 1
                        extra[field] = self._query(cmd)
            jposf = self._query("JPOSF")
        except SerialLinkError as e:
            self._fail(str(e))
            return False
        alarm_text = None
        if state is not None and state[2:3].islower() and not self._status.alarm:
            # STATE field 5 holds the error: "29060" -> 9060, "810693" -> 1069 on axis 3.
            alarm_text = _alarm_number(state)
            if alarm_text:
                try:
                    mes = self._query(f"ERRORMES{alarm_text.split()[0]}")
                    if is_ok_reply(mes):
                        alarm_text += " " + mes.strip()[3:]
                except SerialLinkError:
                    pass
        with self._lock:
            if state is not None:
                self._status.state_raw = state.strip()
                alarm = state[2:3].islower()
                if alarm and not self._status.alarm:
                    self._status.alarm_text = alarm_text or "unknown"
                    self._log("warn", f"Controller alarm {self._status.alarm_text}; use reset_alarm")
                if not alarm:
                    self._status.alarm_text = ""
                self._status.alarm = alarm
            if pposf is not None and is_ok_reply(pposf):
                self._status.tool = parse_pposf(pposf)
                tool = _calibrated_tool(self._status.joints)
                if tool:  # the controller's X/Y/Z is in its (wrongly zeroed) frame: use the corrected model
                    self._status.tool = tool
            for reply, field in [(line_in, "line_inputs"), (line_out, "line_outputs")] + \
                    [(r, f) for f, r in extra.items()]:
                if reply is not None and is_ok_reply(reply):
                    try:
                        setattr(self._status, field, int(reply.strip().rsplit("=", 1)[1]))
                    except (IndexError, ValueError):
                        pass
            if is_ok_reply(jposf):
                parsed = parse_jposf(jposf)
                self._status.joints = {j: parsed.get(j) for j in JOINT_NAMES}
        return True
