"""Bridge between the conveyor PLC (S7-1200, DB10 "PC_Link") and the sequencer.

Topics: plc/part_ready (Bool, the auto-run trigger when auto_trigger = plc)
        plc/state      (String JSON, latched: connected, plc_alive, part_ready,
                        pass_count, missed_part, stations, maintenance, line stop)
Reads:  sequencer/status (armed, pass running, pass results)
Services (panel "Maintenance & safety"):
        plc/maintenance (SetBool)          pause the PLC's automatic decisions
        plc/release_1 .. plc/release_3     one car through that stopper (maintenance only)
        plc/line_stop, plc/line_reset      hold every car (latched in the PLC) / clear it

The PLC is read and written with python-snap7 (needs "Permit access with PUT/GET"
in TIA Portal). Only bytes 4..5 of DB10 are written; see plc_link.py for the handshake.
plc_ip "" disables the bridge. A lost link is retried every reconnect_s; while
it is down plc/part_ready is false, so no pass can start from the PLC.
"""

import json
import threading
import time

import rclpy
import snap7
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import Bool, String
from std_srvs.srv import SetBool, Trigger

from rv2aj_interfaces.msg import SequencerStatus

from .plc_link import PC_BYTE, PLC_BYTES, STATIONS, PlcHandshake, SequencerView


class PlcBridgeNode(Node):
    def __init__(self):
        super().__init__("plc_bridge")
        self.ip = str(self.declare_parameter("plc_ip", "").value).strip()
        self.port = int(self.declare_parameter("plc_port", 102).value)
        self.rack = int(self.declare_parameter("rack", 0).value)
        self.slot = int(self.declare_parameter("slot", 1).value)
        self.db = int(self.declare_parameter("db_number", 10).value)
        self.poll_s = float(self.declare_parameter("poll_s", 0.1).value)
        self.reconnect_s = float(self.declare_parameter("reconnect_s", 2.0).value)

        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.ready_pub = self.create_publisher(Bool, "plc/part_ready", 10)
        self.state_pub = self.create_publisher(String, "plc/state", latched)
        self.create_subscription(SequencerStatus, "sequencer/status", self._on_sequencer, latched)

        self.lock = threading.Lock()
        self.seq = None      # until the first sequencer/status
        self.hs = PlcHandshake()
        self.client = snap7.client.Client()
        self.connected = False
        self._last_state = None
        self._stop = threading.Event()
        self.create_service(SetBool, "plc/maintenance", self._srv_maintenance)
        for station in STATIONS:
            self.create_service(Trigger, f"plc/release_{station}",
                                lambda req, res, n=station: self._srv_release(n, res))
        self.create_service(Trigger, "plc/line_stop", self._srv_line_stop)
        self.create_service(Trigger, "plc/line_reset", self._srv_line_reset)
        if not self.ip:
            self.get_logger().info("plc_ip is empty: PLC bridge disabled")
            self._publish_state()
            return
        self.get_logger().info(f"PLC bridge: {self.ip}:{self.port} DB{self.db}")
        self.thread = threading.Thread(target=self._run, daemon=True)
        self.thread.start()

    def _on_sequencer(self, msg):
        with self.lock:
            self.seq = SequencerView(armed=msg.auto_mode,
                                     busy=msg.loop_active or bool(msg.current_point),
                                     pass_count=msg.auto_pass_count,
                                     last_pass_ok=msg.last_pass_ok)

    def _run(self):
        while not self._stop.is_set():
            if not self.connected:
                try:
                    self.client.connect(self.ip, self.rack, self.slot, tcp_port=self.port)
                    self.connected = True
                    self.get_logger().info(f"PLC connected ({self.ip})")
                except Exception as e:   # snap7 raises RuntimeError/OSError subclasses
                    self._publish_ready(False)
                    self._publish_state()
                    self.get_logger().warning(f"PLC {self.ip} not reachable: {e}",
                                              throttle_duration_sec=30.0)
                    self._stop.wait(self.reconnect_s)
                    continue
            try:
                data = self.client.db_read(self.db, 0, PLC_BYTES)
                with self.lock:
                    now = time.monotonic()
                    out, events = self.hs.update(data, self.seq, now)
                    maint = self.hs.maintenance_byte(now)
                self.client.db_write(self.db, PC_BYTE, bytearray([out, maint]))
            except Exception as e:
                self.get_logger().warning(f"PLC link lost: {e}")
                self._drop()
                continue
            # One call site per severity: rclpy refuses to change it on the same line.
            for level, text in events:
                if level == "warning":
                    self.get_logger().warning(text)
                else:
                    self.get_logger().info(text)
            self._publish_ready(self.hs.part_ready)
            self._publish_state()
            self._stop.wait(self.poll_s)

    # -- maintenance services ----------------------------------------------------------

    def _disabled(self, response):
        if self.ip:
            return False
        response.success, response.message = False, "PLC bridge disabled (plc_ip is empty)"
        return True

    def _srv_maintenance(self, request, response):
        if self._disabled(response):
            return response
        with self.lock:
            response.success, response.message = self.hs.set_maintenance(request.data)
        self.get_logger().info(f"Maintenance: {response.message}")
        self._publish_state()
        return response

    def _srv_release(self, station, response):
        if self._disabled(response):
            return response
        with self.lock:
            response.success, response.message = self.hs.release(station, time.monotonic())
        if response.success:
            self.get_logger().info(response.message)
        return response

    def _srv_line_stop(self, request, response):
        if self._disabled(response):
            return response
        with self.lock:
            self.hs.stop_line()
            connected = self.connected and self.hs.plc_alive
        response.success = connected
        response.message = ("line stop sent: all cars held" if connected else
                            "PLC not reachable: line stop will be sent when it is back")
        self.get_logger().warning(f"Line stop requested ({response.message})")
        self._publish_state()
        return response

    def _srv_line_reset(self, request, response):
        if self._disabled(response):
            return response
        with self.lock:
            response.success, response.message = self.hs.reset_line(time.monotonic())
        self.get_logger().info(f"Line reset: {response.message}")
        self._publish_state()
        return response

    def _drop(self):
        self.connected = False
        with self.lock:
            self.hs.disconnected()
        try:
            self.client.disconnect()
        except Exception:
            pass
        self._publish_ready(False)
        self._publish_state()

    def _publish_ready(self, value):
        self.ready_pub.publish(Bool(data=bool(value)))

    def _publish_state(self):
        p = self.hs.plc
        state = {"enabled": bool(self.ip), "connected": self.connected,
                 "plc_alive": self.hs.plc_alive, "part_ready": self.hs.part_ready,
                 "pass_count": p.pass_count, "missed_part": p.missed_part,
                 "stations": [{"car": p.car[i], "parts": p.parts[i], "valve": p.valve[i]} for i in range(3)],
                 "maint_requested": self.hs.maint, "maint_active": p.maint_active,
                 "line_stop_requested": self.hs.line_stop, "line_stopped": p.line_stopped}
        if state != self._last_state:
            self._last_state = state
            self.state_pub.publish(String(data=json.dumps(state)))

    def destroy_node(self):
        self._stop.set()
        thread = getattr(self, "thread", None)
        if thread is not None:
            thread.join(timeout=2.0)
        if self.connected:
            try:
                # Leave the PLC "not busy, not armed, no maintenance" (a line stop stays latched there).
                self.client.db_write(self.db, PC_BYTE, bytearray([0, int(self.hs.line_stop) << 4]))
                self.client.disconnect()
            except Exception:
                pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = PlcBridgeNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
