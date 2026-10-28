"""PLC <-> laptop handshake for the conveyor line, independent of ROS.

The S7-1200 (plc/LineControl.scl) keeps the link in DB10 "PC_Link":

  byte 0  PLC -> PC   bit0 PartReady, bit1 PLC_Heartbeat, bit2 MissedPart
  byte 2  PLC -> PC   PassCount (Int, big-endian, bytes 2..3)
  byte 4  PC -> PLC   bit0 Busy, bit1 Done, bit2 Error, bit3 Armed, bit4 Heartbeat
  byte 5  PC -> PLC   bit0 MaintMode, bit1-3 St1-3_Release (pulses), bit4 LineStop,
                      bit5 LineReset (pulse)
  byte 6  PLC -> PC   bit0/1 St1 car/parts, bit2/3 St2, bit4/5 St3
  byte 7  PLC -> PC   bit0-2 St1-3 piston down (valve on), bit3 MaintActive, bit4 LineStopped

The laptop only ever writes bytes 4..5 (snap7 writes whole bytes, so each side owns its own).

Handshake for one robot pass at station 1:
  1. PLC sets PartReady (a car with parts is held, auto-run armed, laptop alive).
  2. The sequencer starts a pass -> Busy = 1, Done = Error = 0.
  3. PLC clears PartReady.
  4. Pass over, arm back home -> Busy = 0 and Done = 1 (or Error = 1 if it failed / was stopped).
  5. PLC counts the pass, then asks again (parts left) or releases the car.
Done is a 2 s pulse (the PLC counts its rising edge), and is dropped with the link: a
Done still held after a reconnect or a PLC restart would count a pass that never ran.
Error stays set until the operator re-arms auto-run.

Maintenance (panel "Maintenance & safety" tab): MaintMode pauses the PLC's automatic
decisions; a release pulse then lowers one piston for one car. LineStop holds every
car and is latched in the PLC until a LineReset pulse (sent by reset_line()), so
neither a lost link nor a restarted bridge clears it.
"""

import dataclasses

DB_NUMBER = 10
PLC_BYTES = 8         # bytes 0..7, read (the PC's own bytes 4..5 are read back too)
PC_BYTE = 4           # bytes 4..5, written

PLC_ALIVE_TIMEOUT_S = 3.0   # PLC heartbeat toggles at 1 Hz
PC_HEARTBEAT_HALF_S = 0.5   # our heartbeat toggles every 0.5 s (PLC watchdog is 3 s)
DONE_PULSE_S = 2.0          # Done stays on this long, then drops
COMMAND_PULSE_S = 0.5       # release / line-reset pulses (PLC counts the rising edge)
STATIONS = (1, 2, 3)


@dataclasses.dataclass
class PlcInputs:
    part_ready: bool = False
    heartbeat: bool = False
    missed_part: bool = False
    pass_count: int = 0
    car: tuple = (False, False, False)      # stations 1..3
    parts: tuple = (False, False, False)
    valve: tuple = (False, False, False)    # piston down, car passing
    maint_active: bool = False
    line_stopped: bool = False

    @classmethod
    def parse(cls, data):
        b0 = data[0]
        # A DB10 from before the station bytes (v0.2) is only 6 bytes long.
        b6, b7 = (data[6], data[7]) if len(data) >= 8 else (0, 0)
        return cls(part_ready=bool(b0 & 1), heartbeat=bool(b0 & 2), missed_part=bool(b0 & 4),
                   pass_count=int.from_bytes(bytes(data[2:4]), "big", signed=True),
                   car=tuple(bool(b6 >> 2 * i & 1) for i in range(3)),
                   parts=tuple(bool(b6 >> 2 * i + 1 & 1) for i in range(3)),
                   valve=tuple(bool(b7 >> i & 1) for i in range(3)),
                   maint_active=bool(b7 & 8), line_stopped=bool(b7 & 16))


@dataclasses.dataclass
class SequencerView:
    """What the handshake needs from sequencer/status."""
    armed: bool = False
    busy: bool = False          # a pass (or any sequencer move) is running
    pass_count: int = 0         # auto passes finished since the sequencer started
    last_pass_ok: bool = False


class PlcHandshake:
    """Turns PLC bytes + sequencer state into our output byte and a few events.
    Call update() once per poll; it returns (byte4, events), events being
    (level, text) tuples for the log."""

    def __init__(self):
        self.busy = self.done = self.error = self.armed = False
        self.heartbeat = False
        self._hb_time = None
        self._done_time = None
        self._seen_passes = None       # sequencer pass count we last acted on
        self._plc_hb = None
        self._plc_hb_time = None
        self._missed_prev = False
        self._stopped_prev = False
        self.plc = PlcInputs()
        self.plc_alive = False
        self.maint = False          # maintenance mode requested
        self.line_stop = False      # line stop requested
        self._release_until = {}    # station -> pulse end time
        self._reset_until = None

    @property
    def part_ready(self):
        """The trigger for the sequencer: only while the PLC is alive."""
        return self.plc_alive and self.plc.part_ready

    def output_byte(self):
        return (int(self.busy) | int(self.done) << 1 | int(self.error) << 2
                | int(self.armed) << 3 | int(self.heartbeat) << 4)

    def maintenance_byte(self, now):
        """Byte 5. Pulses end on their own; the line-stop request stays until reset_line()."""
        out = int(self.maint) | int(self.line_stop) << 4
        for station, until in list(self._release_until.items()):
            if now < until:
                out |= 1 << station
            else:
                del self._release_until[station]
        if self._reset_until is not None:
            if now < self._reset_until:
                out |= 1 << 5
            else:
                self._reset_until = None
        return out

    def set_maintenance(self, on):
        """Returns (ok, message). Not while auto-run is armed or a pass is running."""
        if on and (self.armed or self.busy):
            return False, "disarm auto-run (and let the pass finish) before maintenance mode"
        if on and self.line_stop:
            return False, "the line is stopped: reset it first"
        self.maint = bool(on)
        self._release_until.clear()
        return True, "maintenance mode requested" if on else "maintenance mode off"

    def release(self, station, now):
        """One car through the stopper at `station` (maintenance mode only)."""
        if station not in STATIONS:
            return False, f"no station {station}"
        if not self.plc_alive:
            return False, "PLC not reachable"
        if self.line_stop or self.plc.line_stopped:
            return False, "the line is stopped"
        if not (self.maint and self.plc.maint_active):
            return False, "maintenance mode is not active on the PLC"
        if station == 1 and self.busy:
            return False, "the robot is busy at station 1"
        self._release_until[station] = now + COMMAND_PULSE_S
        return True, f"station {station}: releasing one car"

    def stop_line(self):
        """Stop request: maintenance off, no releases, every car held until reset_line()."""
        self.line_stop = True
        self.maint = False
        self._release_until.clear()
        self._reset_until = None

    def reset_line(self, now):
        """Drop the stop request and pulse LineReset (the PLC clears its latch on the edge)."""
        if not self.plc_alive:
            return False, "PLC not reachable: the line stays stopped"
        self.line_stop = False
        self._reset_until = now + COMMAND_PULSE_S
        return True, "line reset sent"

    def disconnected(self):
        """Link lost: nothing may trigger a pass until the PLC is seen again.
        A line-stop request stays (the PLC has it latched anyway); maintenance mode and
        pulses are dropped (the PLC stops the line when it loses us during maintenance)."""
        self.plc_alive = False
        self.done = False
        self.maint = False
        self._release_until.clear()
        self._reset_until = None
        self._plc_hb = None
        self._plc_hb_time = None

    def update(self, data, seq, now):
        """seq is None until the first sequencer/status arrives: then we are not
        armed and not busy, and pass results are not tracked yet (a default count
        of 0 as baseline would report the sequencer's earlier passes as a new Done)."""
        events = []
        known = seq is not None
        seq = seq or SequencerView()
        self.plc = PlcInputs.parse(data)

        # PLC heartbeat watchdog.
        if self._plc_hb is None or self.plc.heartbeat != self._plc_hb:
            self._plc_hb = self.plc.heartbeat
            self._plc_hb_time = now
        alive = now - self._plc_hb_time < PLC_ALIVE_TIMEOUT_S
        if alive != self.plc_alive:
            events.append(("info", "PLC heartbeat OK") if alive else
                          ("warning", "PLC heartbeat stopped (PLC in STOP?): no new robot passes"))
        self.plc_alive = alive

        # Our heartbeat.
        if self._hb_time is None or now - self._hb_time >= PC_HEARTBEAT_HALF_S:
            self.heartbeat = not self.heartbeat
            self._hb_time = now

        # Armed; re-arming clears an old error.
        if seq.armed and not self.armed:
            self.error = False
        self.armed = seq.armed

        # Busy rising: a new pass, the old result no longer applies.
        if seq.busy and not self.busy:
            self.done = self.error = False
        self.busy = seq.busy

        # A pass finished (the first status only sets the baseline).
        if not known:
            pass
        elif self._seen_passes is None:
            self._seen_passes = seq.pass_count
        elif seq.pass_count != self._seen_passes:
            self._seen_passes = seq.pass_count
            self.done = seq.last_pass_ok
            self.error = not seq.last_pass_ok
            self._done_time = now
            events.append(("info", "Pass done: told the PLC (Done)") if seq.last_pass_ok else
                          ("warning", "Pass failed or stopped: told the PLC (Error); re-arm auto-run to clear"))

        if self.done and now - self._done_time >= DONE_PULSE_S:
            self.done = False

        if self.plc.missed_part and not self._missed_prev:
            events.append(("warning", "UYARI: robot iki denemede arabayı boşaltamadı, araba parçayla "
                                      "gönderildi (PLC MissedPart). Tutucu noktalarını kontrol edin."))
        self._missed_prev = self.plc.missed_part
        if self.plc.line_stopped != self._stopped_prev:
            events.append(("warning", "Conveyor line STOPPED: all cars held (reset on the Maintenance tab)")
                          if self.plc.line_stopped else ("info", "Conveyor line stop cleared"))
        self._stopped_prev = self.plc.line_stopped
        return self.output_byte(), events
