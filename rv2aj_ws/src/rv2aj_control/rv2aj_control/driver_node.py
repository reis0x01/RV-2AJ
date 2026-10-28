"""ROS 2 driver for the RV-2AJ / CR1-571 over RS-232C.

The only node that talks to the serial port. See driver_core.py for the
safety rules; this file only maps them onto ROS interfaces.

Topics:   status (rv2aj_interfaces/RobotStatus), joint_states, tcp_position
Services: connect, disconnect, servo, stop, reset_alarm, set_override, run_program,
          gripper/open, gripper/close
Actions:  tcp_move, joint_move (cancel = STOP)
"""

import math
import time

import rclpy
from geometry_msgs.msg import PointStamped
from rclpy.action import ActionServer, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_srvs.srv import SetBool, Trigger

from rv2aj_interfaces.action import JointMove, TcpMove
from rv2aj_interfaces.msg import RobotStatus
from rv2aj_interfaces.srv import Connect, RunProgram, SetOverride, LineSignal

from .driver_core import JOINT_NAMES, MOVABLE_JOINTS, DriverCore, Limits
from .fake_link import FakeLink
from .serial_link import SerialLink

FEEDBACK_PERIOD_S = 0.2


class DriverNode(Node):
    def __init__(self):
        super().__init__("driver")
        p = self.declare_parameter
        self.port = p("port", "/dev/ttyUSB0").value
        fake = p("use_fake_hardware", False).value
        auto_connect = p("auto_connect", True).value
        self.frame_id = p("frame_id", "rv2aj_base").value
        # Simulator only: start pose J1..J6 in degrees (e.g. a real robot pose to
        # debug); read on every connect.
        p("fake_initial_joints_deg", [0.0])
        limits = Limits(
            default_override=int(p("default_override_percent", 10).value),
            max_tcp_move_mm=float(p("max_tcp_move_mm", 50.0).value),
            max_joint_step_deg=float(p("max_joint_step_deg", 45.0).value),
            joint_poll_interval_s=float(p("joint_poll_interval_s", 0.03).value),
            poll_interval_s=float(p("poll_interval_s", 1.0).value),
            moving_poll_interval_s=float(p("moving_poll_interval_s", 0.5).value),
        )

        self._last_joint_state = None
        self.status_pub = self.create_publisher(RobotStatus, "status", 10)
        self.joint_pub = self.create_publisher(JointState, "joint_states", 10)
        self.tcp_pub = self.create_publisher(PointStamped, "tcp_position", 10)

        def fake_link():
            link = FakeLink()
            fake_start = list(self.get_parameter("fake_initial_joints_deg").value)
            if len(fake_start) == 6:
                link.joints.start = list(fake_start)
                link.joints.target = list(fake_start)
            return link

        self.core = DriverCore(
            fake_link if fake else SerialLink,
            limits,
            on_status=self._publish_status,
            on_log=self._log,
        )

        group = ReentrantCallbackGroup()
        self.create_service(Connect, "connect", self._connect, callback_group=group)
        self.create_service(Trigger, "disconnect", self._disconnect, callback_group=group)
        self.create_service(SetBool, "servo", self._servo, callback_group=group)
        self.create_service(Trigger, "stop", self._stop, callback_group=group)
        self.create_service(Trigger, "reset_alarm", self._reset_alarm, callback_group=group)
        self.create_service(SetOverride, "set_override", self._set_override, callback_group=group)
        self.create_service(RunProgram, "run_program", self._run_program, callback_group=group)
        self.create_service(Trigger, "gripper/open", self._gripper_open, callback_group=group)
        self.create_service(Trigger, "gripper/close", self._gripper_close, callback_group=group)
        self.create_service(LineSignal, "line/set", self._line_set, callback_group=group)

        common = dict(callback_group=group, cancel_callback=self._cancel)
        ActionServer(self, TcpMove, "tcp_move", self._execute_tcp, **common)
        ActionServer(self, JointMove, "joint_move", self._execute_joint, **common)

        # MoveIt wants joint states newer than each request; serial polls are ~1 Hz
        # when idle (0.3 s while moving), so republish the latest reading at 10 Hz.
        self.create_timer(0.1, self._republish_joint_state)

        if fake:
            self.get_logger().warning("use_fake_hardware: simulated controller, no serial port")
        if auto_connect:
            ok, msg = self.core.connect("fake" if fake else self.port)
            if not ok:
                self.get_logger().error(f"Auto-connect failed: {msg}")
        self._publish_status(self.core.status())

    # -- publishing --------------------------------------------------------

    def _log(self, level, text):
        if not rclpy.ok():
            print(f"[driver] {text}")
            return
        # rclpy requires a fixed severity per call site, so one call per level.
        logger = self.get_logger()
        if level == "error":
            logger.error(text)
        elif level == "warn":
            logger.warning(text)
        else:
            logger.info(text)

    def _publish_status(self, s):
        if not rclpy.ok():
            return
        stamp = self.get_clock().now().to_msg()
        msg = RobotStatus()
        msg.header.stamp = stamp
        msg.header.frame_id = self.frame_id
        msg.connected = s.connected
        msg.servo_on = s.servo_on
        msg.alarm = s.alarm
        msg.moving = s.moving
        msg.active_motion = s.active_motion
        msg.override_percent = s.override_percent
        msg.joints_deg = [math.nan if s.joints.get(j) is None else float(s.joints[j]) for j in JOINT_NAMES]
        nan = math.nan
        msg.x_mm, msg.y_mm, msg.z_mm = (float(s.tool.get(k, nan)) for k in ("X", "Y", "Z"))
        msg.a_deg, msg.b_deg = (float(s.tool.get(k, nan)) for k in ("A", "B"))
        msg.state_raw = s.state_raw
        msg.last_error = s.last_error
        msg.gripper = s.gripper
        msg.line_inputs = int(s.line_inputs)
        msg.line_outputs = int(s.line_outputs)
        msg.inputs_8_15 = int(s.inputs_8_15)
        msg.outputs_8_15 = int(s.outputs_8_15)
        msg.hand_inputs = int(s.hand_inputs)
        msg.hand_outputs = int(s.hand_outputs)
        self.status_pub.publish(msg)

        if s.connected and all(s.joints.get(j) is not None for j in MOVABLE_JOINTS):
            js = JointState()
            js.header.stamp = stamp
            js.name = [f"j{j[1]}" for j in MOVABLE_JOINTS]
            js.position = [math.radians(s.joints[j]) for j in MOVABLE_JOINTS]
            self.joint_pub.publish(js)
            self._last_joint_state = js
        else:
            self._last_joint_state = None
        if s.connected and all(k in s.tool for k in ("X", "Y", "Z")):
            pt = PointStamped()
            pt.header.stamp = stamp
            pt.header.frame_id = self.frame_id
            pt.point.x, pt.point.y, pt.point.z = (s.tool[k] / 1000.0 for k in ("X", "Y", "Z"))
            self.tcp_pub.publish(pt)

    def _republish_joint_state(self):
        js = self._last_joint_state
        if js is not None and self.core.status().connected:
            js.header.stamp = self.get_clock().now().to_msg()
            self.joint_pub.publish(js)

    # -- services ----------------------------------------------------------

    def _connect(self, request, response):
        response.success, response.message = self.core.connect(request.port or self.port)
        return response

    def _disconnect(self, request, response):
        response.success, response.message = self.core.disconnect()
        return response

    def _servo(self, request, response):
        response.success, response.message = self.core.servo(request.data)
        return response

    def _stop(self, request, response):
        response.success, response.message = self.core.stop()
        return response

    def _reset_alarm(self, request, response):
        response.success, response.message = self.core.reset_alarm()
        return response

    def _set_override(self, request, response):
        response.success, response.message = self.core.set_override(request.percent)
        return response

    def _line_set(self, request, response):
        response.success, response.message, response.line_outputs = self.core.line_set(request.signal,
                                                                                     int(request.value))
        return response

    def _gripper_open(self, request, response):
        response.success, response.message = self.core.gripper(True)
        return response

    def _gripper_close(self, request, response):
        response.success, response.message = self.core.gripper(False)
        return response

    def _run_program(self, request, response):
        response.success, response.message = self.core.run_program(request.program)
        return response

    # -- actions -------------------------------------------------------------

    def _cancel(self, goal_handle):
        self.core.stop()
        return CancelResponse.ACCEPT

    def _run_motion(self, goal_handle, start, result, feedback, fill_feedback, fill_result):
        motion, reason = start()
        if motion is None:
            self.get_logger().warning(f"Move refused: {reason}")
            goal_handle.abort()
            result.success = False
            result.message = reason
            fill_result(result, self.core.status())
            return result
        while not motion.done.wait(FEEDBACK_PERIOD_S):
            fill_feedback(feedback, self.core.status())
            goal_handle.publish_feedback(feedback)
        result.success = motion.success
        result.message = motion.message
        fill_result(result, self.core.status())
        if motion.success:
            goal_handle.succeed()
        elif goal_handle.is_cancel_requested:
            goal_handle.canceled()
        else:
            goal_handle.abort()
        return result

    @staticmethod
    def _xyz(status):
        return [float(status.tool.get(k, math.nan)) for k in ("X", "Y", "Z")]

    @staticmethod
    def _joints(status):
        return [math.nan if status.joints.get(j) is None else float(status.joints[j]) for j in JOINT_NAMES]

    def _execute_tcp(self, goal_handle):
        goal = goal_handle.request
        return self._run_motion(
            goal_handle,
            lambda: self.core.tcp_move(goal.axis, goal.distance_mm),
            TcpMove.Result(), TcpMove.Feedback(),
            lambda fb, s: setattr(fb, "current_xyz_mm", self._xyz(s)),
            lambda r, s: setattr(r, "final_xyz_mm", self._xyz(s)),
        )

    def _execute_joint(self, goal_handle):
        goal = goal_handle.request
        if goal.relative:
            start = lambda: self.core.joint_step(goal.joint, goal.offset_deg)  # noqa: E731
        else:
            start = lambda: self.core.joint_move(list(goal.target_deg), "joint move")  # noqa: E731
        return self._run_motion(
            goal_handle, start, JointMove.Result(), JointMove.Feedback(),
            lambda fb, s: setattr(fb, "current_deg", self._joints(s)),
            lambda r, s: setattr(r, "final_deg", self._joints(s)),
        )

    def shutdown(self):
        self.core.disconnect()


def main(args=None):
    rclpy.init(args=args)
    node = DriverNode()
    executor = MultiThreadedExecutor(num_threads=6)
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.shutdown()
        time.sleep(0.1)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
