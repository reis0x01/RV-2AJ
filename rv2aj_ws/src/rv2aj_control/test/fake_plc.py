"""Fake conveyor PLC for testing plc_bridge without the S7-1200.

Hosts DB10 "PC_Link" with python-snap7's server and plays station 1 of
plc/LineControl.scl: a car with parts arrives, PartReady is raised (once the
laptop is armed and alive), cleared on Busy, and after Done the car leaves if the
robot emptied it, gets a second pass if not, and leaves with MissedPart after 2.

  python3 test/fake_plc.py [--port 11020] [--cars 2] [--stubborn]

--stubborn: parts never disappear, to see the retry + MissedPart path.
Maintenance (byte 5) as in the SCL: MaintMode pauses station 1's decisions, a
St1_Release edge sends the car away, LineStop latches until a LineReset edge.
Stations 2 and 3 always show a car without parts.
Then run the bridge against it:
  ros2 run rv2aj_control plc_bridge --ros-args -r __ns:=/rv2aj \
      -p plc_ip:=127.0.0.1 -p plc_port:=11020
"""

import argparse
import time

from snap7.server import Server
from snap7.type import SrvArea


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", type=int, default=11020)
    ap.add_argument("--cars", type=int, default=2)
    ap.add_argument("--stubborn", action="store_true")
    args = ap.parse_args()

    db = bytearray(8)
    server = Server(log=False)
    server.register_area(SrvArea.DB, 10, db)
    server.start(tcp_port=args.port)
    print(f"fake PLC: DB10 on port {args.port}, {args.cars} car(s)", flush=True)

    def bit(byte, n):
        return bool(db[byte] >> n & 1)

    def set_bit(byte, n, value):
        db[byte] = db[byte] | (1 << n) if value else db[byte] & ~(1 << n)

    t0 = time.monotonic()
    last_beat, beat_time = None, t0
    prev_done = False
    prev_rel1 = prev_reset = False
    stopped = False
    valve_until = 0.0
    cars, passes, part_on, idle_since = args.cars, 0, args.cars > 0, t0
    try:
        while cars > 0:
            now = time.monotonic()
            set_bit(0, 1, int(now - t0) % 2 == 1)                   # PLC heartbeat, 1 Hz
            busy, done, error, armed, beat = (bit(4, n) for n in range(5))
            if beat != last_beat:
                last_beat, beat_time = beat, now
            alive = now - beat_time < 3.0
            maint_mode, rel1, line_stop, line_reset = bit(5, 0), bit(5, 1), bit(5, 4), bit(5, 5)
            if line_stop or (maint_mode and not alive):
                if not stopped:
                    print("  LINE STOP latched", flush=True)
                stopped = True
            elif line_reset and not prev_reset and alive and stopped:
                stopped = False
                print("  line stop cleared", flush=True)
            prev_reset = line_reset
            maint = maint_mode and alive and not stopped
            auto = not maint and not stopped
            if maint and rel1 and not prev_rel1 and not busy:
                print("  maintenance: station 1 released by hand", flush=True)
                valve_until = now + 1.5
                cars -= 1
                passes, part_on, idle_since = 0, cars > 0, now
            prev_rel1 = rel1
            if stopped:
                valve_until = 0.0
            db[6] = int(cars > 0) | int(part_on) << 1 | 1 << 2 | 1 << 4
            db[7] = int(now < valve_until) | int(maint) << 3 | int(stopped) << 4
            if busy:
                idle_since = now
            settled = not busy and now - idle_since >= 1.0

            if done and not prev_done:                              # a pass finished
                passes += 1
                part_on = args.stubborn
                print(f"  PC_Done: pass {passes} on this car", flush=True)
            prev_done = done
            db[2:4] = passes.to_bytes(2, "big", signed=True)

            if auto and part_on and passes < 2 and settled and armed and not error and alive:
                if not bit(0, 0):
                    print("  car with parts at station 1 -> PartReady", flush=True)
                set_bit(0, 0, True)
            if busy or not armed or not alive or not part_on or not auto:
                set_bit(0, 0, False)

            if auto and settled and (not part_on or passes >= 2):   # release the car
                set_bit(0, 2, part_on)
                valve_until = now + 1.5
                print("  car released" + (" WITH PARTS (MissedPart)" if part_on else " empty"), flush=True)
                cars -= 1
                passes, part_on, idle_since = 0, cars > 0, now
            time.sleep(0.05)
        print("fake PLC: all cars done", flush=True)
        time.sleep(1.0)
    finally:
        server.stop()


if __name__ == "__main__":
    main()
