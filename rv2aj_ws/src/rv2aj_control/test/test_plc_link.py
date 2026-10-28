from rv2aj_control.plc_link import PlcHandshake, PlcInputs, SequencerView

BUSY, DONE, ERROR, ARMED, HEARTBEAT = 1, 2, 4, 8, 16


def plc_bytes(part_ready=False, heartbeat=False, missed=False, passes=0):
    return bytearray([int(part_ready) | int(heartbeat) << 1 | int(missed) << 2, 0,
                      *passes.to_bytes(2, "big", signed=True)])


class Line:
    """A handshake plus a PLC whose heartbeat toggles every second, stepped in 0.1 s."""

    def __init__(self):
        self.hs = PlcHandshake()
        self.t = 0.0
        self.seq = SequencerView()
        self.part_ready = self.missed = False
        self.passes = 0

    def step(self, n=1):
        out, events = 0, []
        for _ in range(n):
            self.t += 0.1
            hb = int(self.t) % 2 == 1
            out, ev = self.hs.update(plc_bytes(self.part_ready, hb, self.missed, self.passes), self.seq, self.t)
            events += ev
        return out, events


def test_parse_db10_bytes():
    p = PlcInputs.parse(plc_bytes(part_ready=True, heartbeat=False, missed=True, passes=2))
    assert (p.part_ready, p.heartbeat, p.missed_part, p.pass_count) == (True, False, True, 2)


def test_full_pass_handshake():
    line = Line()
    line.seq = SequencerView(armed=True)
    out, _ = line.step(5)
    assert out & ARMED and not out & (BUSY | DONE | ERROR)

    line.part_ready = True
    line.step()
    assert line.hs.part_ready                  # trigger reaches the sequencer

    line.seq = SequencerView(armed=True, busy=True)   # pass started
    line.part_ready = False                    # PLC answers by clearing the request
    out, _ = line.step()
    assert out & BUSY and not out & (DONE | ERROR)
    assert not line.hs.part_ready

    line.seq = SequencerView(armed=True, busy=False, pass_count=1, last_pass_ok=True)
    out, events = line.step()
    assert out & DONE and not out & (BUSY | ERROR)
    assert any("Done" in text for _, text in events)
    out, _ = line.step(25)
    assert not out & DONE                      # a 2 s pulse, not a level
    line.seq = SequencerView(armed=True, busy=False, pass_count=2, last_pass_ok=True)
    assert line.step()[0] & DONE

    # A retry pass clears the old Done when it starts.
    line.seq = SequencerView(armed=True, busy=True, pass_count=2, last_pass_ok=True)
    out, _ = line.step()
    assert out & BUSY and not out & DONE


def test_no_done_before_sequencer_status():
    line = Line()
    line.seq = None                            # bridge started, sequencer status not in yet
    line.step(3)
    line.seq = SequencerView(armed=True, pass_count=5, last_pass_ok=True)   # earlier passes
    out, events = line.step()
    assert not out & DONE and not events


def test_reconnect_does_not_repeat_done():
    line = Line()
    line.seq = SequencerView(armed=True)
    line.step()
    line.seq = SequencerView(armed=True, pass_count=1, last_pass_ok=True)
    assert line.step()[0] & DONE
    line.hs.disconnected()
    assert not line.hs.output_byte() & DONE    # the PLC must not see Done rise again


def test_failed_pass_sets_error_until_rearmed():
    line = Line()
    line.seq = SequencerView(armed=True, pass_count=3)
    line.step()                                # baseline: old passes are not reported again
    assert not line.hs.output_byte() & (DONE | ERROR)
    line.seq = SequencerView(armed=False, pass_count=4, last_pass_ok=False)   # STOP disarms
    out, events = line.step()
    assert out & ERROR and not out & DONE
    assert any(level == "warning" for level, _ in events)
    line.seq = SequencerView(armed=False, pass_count=4)
    assert line.step()[0] & ERROR              # stays until re-armed
    line.seq = SequencerView(armed=True, pass_count=4)
    out, _ = line.step()
    assert out & ARMED and not out & ERROR


def test_our_heartbeat_toggles():
    line = Line()
    seen = {bool(line.step()[0] & HEARTBEAT) for _ in range(20)}
    assert seen == {True, False}


def test_plc_heartbeat_lost_blocks_trigger():
    hs = PlcHandshake()
    seq = SequencerView(armed=True)
    hs.update(plc_bytes(part_ready=True), seq, 0.0)
    assert hs.part_ready
    _, events = hs.update(plc_bytes(part_ready=True), seq, 3.5)   # heartbeat never moved
    assert not hs.plc_alive and not hs.part_ready
    assert any("heartbeat stopped" in text for _, text in events)
    hs.update(plc_bytes(part_ready=True, heartbeat=True), seq, 3.6)
    assert hs.part_ready


def test_disconnect_blocks_trigger():
    hs = PlcHandshake()
    hs.update(plc_bytes(part_ready=True), SequencerView(armed=True), 0.0)
    hs.disconnected()
    assert not hs.part_ready


def test_missed_part_warns_once():
    line = Line()
    line.missed = True
    _, events = line.step(5)
    assert sum("MissedPart" in text for _, text in events) == 1


def test_only_byte4_bits_written():
    line = Line()
    line.seq = SequencerView(armed=True, busy=True)
    out, _ = line.step()
    assert 0 <= out < 32


# -- maintenance and line stop (bytes 5..7) ------------------------------------------

MAINT, STOP, RESET = 1, 16, 32
MAINT_ACTIVE, LINE_STOPPED = 8, 16


def db10(b7=0, b6=0, heartbeat=False):
    return plc_bytes(heartbeat=heartbeat) + bytearray([0, 0, b6, b7])


def test_parse_station_bytes():
    p = PlcInputs.parse(db10(b7=0b10010 | MAINT_ACTIVE, b6=0b100011))
    assert p.car == (True, False, False) and p.parts == (True, False, True)
    assert p.valve == (False, True, False) and p.maint_active and p.line_stopped
    old = PlcInputs.parse(plc_bytes())   # a v0.2 DB without the station bytes
    assert old.car == (False, False, False) and not old.line_stopped


def test_maintenance_refused_while_armed():
    hs = PlcHandshake()
    hs.update(db10(), SequencerView(armed=True), 0.0)
    ok, _ = hs.set_maintenance(True)
    assert not ok and not hs.maint
    hs.update(db10(), SequencerView(), 0.1)
    ok, _ = hs.set_maintenance(True)
    assert ok and hs.maintenance_byte(0.1) & MAINT


def test_release_needs_plc_maintenance_and_is_a_pulse():
    hs = PlcHandshake()
    hs.update(db10(), SequencerView(), 0.0)
    hs.set_maintenance(True)
    ok, message = hs.release(2, 0.0)
    assert not ok and "not active" in message          # PLC has not confirmed yet
    hs.update(db10(b7=MAINT_ACTIVE), SequencerView(), 0.1)
    ok, _ = hs.release(2, 0.1)
    assert ok and hs.maintenance_byte(0.2) & (1 << 2)
    assert not hs.maintenance_byte(1.0) & (1 << 2)     # pulse over


def test_release_station1_refused_while_robot_busy():
    hs = PlcHandshake()
    hs.update(db10(), SequencerView(), 0.0)
    hs.set_maintenance(True)
    hs.update(db10(b7=MAINT_ACTIVE), SequencerView(busy=True), 0.1)
    assert not hs.release(1, 0.1)[0]
    assert hs.release(3, 0.1)[0]


def test_line_stop_latches_until_reset():
    hs = PlcHandshake()
    hs.update(db10(), SequencerView(), 0.0)
    hs.set_maintenance(True)
    hs.stop_line()
    assert hs.maintenance_byte(0.0) == STOP              # maintenance dropped
    assert not hs.set_maintenance(True)[0]
    hs.disconnected()
    assert hs.maintenance_byte(0.0) & STOP               # survives a lost link
    assert not hs.reset_line(0.0)[0]                     # PLC not seen: no reset
    hs.update(db10(b7=LINE_STOPPED), SequencerView(), 0.1)
    ok, _ = hs.reset_line(0.1)
    assert ok and hs.maintenance_byte(0.2) == RESET
    assert hs.maintenance_byte(1.0) == 0


def test_line_stop_change_is_logged():
    hs = PlcHandshake()
    _, events = hs.update(db10(b7=LINE_STOPPED), SequencerView(), 0.0)
    assert any("STOPPED" in text for _, text in events)
    _, events = hs.update(db10(), SequencerView(), 0.1)
    assert any("cleared" in text for _, text in events)
