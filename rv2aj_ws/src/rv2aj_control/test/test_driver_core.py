import time

import pytest

from rv2aj_control.driver_core import DriverCore, Limits
from rv2aj_control.fake_link import FakeLink


@pytest.fixture
def core():
    fakes = []

    def factory():
        fakes.append(FakeLink())
        return fakes[-1]

    c = DriverCore(factory, Limits(poll_interval_s=0.1, moving_poll_interval_s=0.05, servo_on_settle_s=0.05))
    c.fakes = fakes
    yield c
    c.disconnect()


def wait(motion, timeout=10):
    assert motion.done.wait(timeout)
    return motion


def test_moves_need_connection_and_servo(core):
    assert core.tcp_move("Z", 5)[1] == "not connected"
    assert core.connect("fake")[0]
    time.sleep(0.3)
    assert core.tcp_move("Z", 5)[1] == "servo is off"
    assert core.servo(True)[0]
    m = wait(core.tcp_move("Z", 5)[0])
    assert m.success and m.final["Z"] == pytest.approx(437.32, abs=0.05)


def test_limits_and_busy(core):
    core.connect("fake")
    core.servo(True)
    assert "limit" in core.tcp_move("X", 80)[1]
    assert "step limit" in core.joint_step("J1", 60)[1]
    assert "cannot be moved" in core.joint_step("J4", 1)[1]
    m, _ = core.tcp_move("Z", 20)
    assert core.tcp_move("Z", 5)[1].startswith("busy")
    wait(m)


def test_stop_halts_motion(core):
    core.connect("fake")
    core.servo(True)
    m, _ = core.joint_step("J1", 10)
    time.sleep(0.3)
    assert core.stop()[0]
    wait(m)
    assert not m.success and "stopped" in m.message


def test_controller_error_drops_link_and_resets_override(core):
    core.connect("fake")
    core.servo(True)
    core.set_override(40)
    fake = core.fakes[-1]
    # Arm fully stretched forward: any further +X is out of reach.
    fake.joints.target = fake.joints.start = [0.0, 90.0, 0.0, 0.0, 0.0, 0.0]
    time.sleep(0.3)
    m = wait(core.tcp_move("X", 10)[0])
    time.sleep(0.3)
    assert not m.success
    assert not core.status().connected
    assert fake.sent[-3:] == ["STOP", "SRVOFF", "CNTLOFF"]
    core.connect("fake")
    assert core.status().override_percent == 10 and not core.status().servo_on


def test_absolute_joint_move_uses_only_verified_syntax(core):
    """GOTO / MoveIt joint moves must use the forms verified on the real
    CR1-571 (JTMP=J_CURR, JTMP.Jn=JTMP.Jn+RAD(d), VALJTMP, EXECMOV JTMP), never
    the unconfirmed tuple form JTMP=(...)."""
    core.connect("fake")
    core.servo(True)
    link = core.fakes[-1]
    link.sent.clear()
    target = [-120.0, 0.0, 130.0, None, -20.0, 10.0]
    reached = link.send_joint_move(target, 10)
    assert reached[0] == pytest.approx(-120.0, abs=0.02) and reached[2] == pytest.approx(130.0, abs=0.02)
    assert not any("JTMP=(" in c or "+(" in c for c in link.sent)
    assert "EXECJTMP=J_CURR" in link.sent and "EXECMOV JTMP" in link.sent
    jtmp = [c for c in link.sent if "JTMP" in c]  # the background poller also sends JPOSF/STATE/PPOSF
    assert all(c in ("EXECJTMP=J_CURR", "VALJTMP", "EXECMOV JTMP") or
               __import__("re").fullmatch(r"EXECJTMP\.J(\d)=JTMP\.J\1\+RAD\(-?\d+\.\d{2}\)", c) for c in jtmp)


def test_frozen_joint_never_moves(core, monkeypatch):
    from rv2aj_control import driver_core
    monkeypatch.setattr(driver_core, "_frozen", lambda: frozenset({"J5"}))
    core.connect("fake")
    core.servo(True)
    time.sleep(0.3)
    assert core.joint_step("J5", 5)[0] is None                      # a J5 step is refused
    assert core.tcp_move("Z", 5)[0] is None                         # the controller's own line would move J5
    j5 = core.status().joints["J5"]
    m = wait(core.joint_move([-120.0, 0.0, 130.0, None, j5 + 20.0, 10.0])[0])
    assert m.success and abs(core.status().joints["J5"] - j5) < 0.05  # J5 target ignored


def test_higher_io_poll_instant_when_idle_rotating_when_busy():
    """The Robot IO panel's higher bytes (inputs_8_15, outputs_8_15, hand_inputs,
    hand_outputs):
      - when the robot has been idle a moment (operator watching): all 4 read every
        poll, so a sensor change shows at once;
      - just after a move / mid-loop: ONE per poll, round-robin, so the 4 reads never
        stall the next move ~0.4 s at each point (2026-10-20/21);
      - with a job already waiting: skipped, so a queued move is never delayed.
    Byte 0 (order/ready/busy/error) is read every full poll regardless. Deterministic:
    _poll() is called directly, not via the worker thread."""
    import time
    from rv2aj_control.driver_core import DriverCore, Limits
    from rv2aj_control.fake_link import FakeLink

    EXTRA = ("VALM_INB(8)", "VALM_OUTB(8)", "VALM_INB(900)", "VALM_OUTB(900)")

    def extra_io_commands(sent):
        return [c for c in sent if c.startswith(EXTRA)]

    fake = FakeLink()
    fake.connect("fake")
    c = DriverCore(lambda: fake, Limits())
    c._link = fake
    c._motion = None

    # Settled (last move was long ago): all four higher bytes read in one poll.
    c._last_motion_end = 0.0
    fake.sent.clear()
    assert c._poll(full=True)
    assert {cmd.split("=")[0] for cmd in extra_io_commands(fake.sent)} == set(EXTRA), (
        "when idle, all four higher-I/O bytes should refresh every poll for instant reading")

    # Just moved (within the 1 s window, e.g. between loop points): only one per poll,
    # cycling, so the next move is not delayed by four reads.
    seen = []
    for _ in range(4):
        c._last_motion_end = time.monotonic()   # pretend a move just ended
        fake.sent.clear()
        assert c._poll(full=True)
        got = extra_io_commands(fake.sent)
        assert len(got) == 1, f"mid-loop should read exactly one higher-I/O byte per poll, got {got}"
        seen.append(got[0])
    assert {cmd.split("=")[0] for cmd in seen} >= set(EXTRA), (
        f"round-robin should still cover all four bytes over four polls, saw {seen}")

    # A job is waiting: skip the higher-I/O read entirely.
    c._last_motion_end = 0.0
    fake.sent.clear()
    c._jobs.put(("test job", lambda: "ok", None))
    assert c._poll(full=True)
    assert extra_io_commands(fake.sent) == [], (
        "a waiting job must make the higher-I/O read back off so a queued move is not delayed")
    c._jobs.get_nowait()
