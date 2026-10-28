import time

import pytest

from rv2aj_control import serial_link
from rv2aj_control.driver_core import DriverCore, Limits
from rv2aj_control.fake_link import FakeLink
from rv2aj_control.serial_link import GripperCommandRejected, SerialLink


class ScriptedLink(SerialLink):
    """Replies with a fixed string and records what was sent."""

    def __init__(self, reply):
        super().__init__()
        self.reply = reply
        self.sent = []

    def send(self, command):
        self.sent.append(command)
        return self.reply


def test_gripper_sends_exec_form_with_hand_number():
    link = ScriptedLink("QoK\r")
    link.open_gripper()
    link.close_gripper()
    assert link.sent == [f"EXECHOPEN {serial_link.HAND_NUMBER}", f"EXECHCLOSE {serial_link.HAND_NUMBER}"]


def test_rejection_raises_with_exact_reply_and_no_retry():
    link = ScriptedLink("QeR422000000\r")
    with pytest.raises(GripperCommandRejected) as err:
        link.open_gripper()
    assert "QeR422000000" in str(err.value)
    assert "unconfirmed" in str(err.value)
    assert link.sent == [f"EXECHOPEN {serial_link.HAND_NUMBER}"]   # one form only, never a guess


class RejectingHandLink(FakeLink):
    def _handle(self, c):
        if c.startswith("EXECH"):
            return self._error("R4220")
        return super()._handle(c)


def make_core(link_cls):
    links = []

    def factory():
        links.append(link_cls())
        return links[-1]

    core = DriverCore(factory, Limits(poll_interval_s=0.1, moving_poll_interval_s=0.05, servo_on_settle_s=0.05))
    core.links = links
    return core


def test_state_unknown_until_commanded_and_no_command_on_connect():
    core = make_core(FakeLink)
    core.connect("fake")
    time.sleep(0.3)
    assert core.status().gripper == "unknown"
    assert not any(c.startswith("EXECH") for c in core.links[-1].sent)
    assert core.status().servo_on is False
    assert core.gripper(False) == (True, "gripper close accepted")   # no servo needed
    assert core.status().gripper == "closed"
    assert core.gripper(True)[0] and core.status().gripper == "open"
    core.disconnect()


def test_rejected_gripper_reports_reply_and_state_becomes_unknown():
    core = make_core(RejectingHandLink)
    core.connect("fake")
    time.sleep(0.2)
    ok, message = core.gripper(True)
    assert not ok
    assert "QeR4220000000" in message and "EXECHOPEN" in message
    assert core.status().gripper == "unknown"
    core.disconnect()
