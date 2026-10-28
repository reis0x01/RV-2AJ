"""Translate spoken phrases (voice/text) into robot commands.

Each phrase is parsed by voice_commands.parse_command and published as a
rv2aj_interfaces/MotionCommand on voice/command (including ignored ones,
with the reason), then carried out:

  stop / halt / freeze   -> driver stop service + sequencer loop/stop
  move up 20, go left 5  -> planner planned/tcp_move (MoveIt-validated), or the
                            driver's tcp_move when tcp_action is set to it
  go to point 2          -> sequencer points/goto
  go home                -> sequencer home/goto
  start loop / end loop  -> sequencer loop/start, loop/stop

Anything that publishes on voice/text is translated, so typed commands work
too:  ros2 topic pub --once /rv2aj/voice/text std_msgs/String "{data: 'move up 5'}"
"""

import rclpy
from rcl_interfaces.msg import SetParametersResult
from rclpy.action import ActionClient
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger

from rv2aj_interfaces.action import TcpMove
from rv2aj_interfaces.msg import MotionCommand
from rv2aj_interfaces.srv import PointName

from .voice_commands import parse_command


class TranslatorNode(Node):
    def __init__(self):
        super().__init__("translator")
        p = self.declare_parameter
        self.require_keyword = p("require_keyword", True).value
        self.default_distance_mm = float(p("default_distance_mm", 5.0).value)
        self.max_distance_mm = float(p("max_distance_mm", 50.0).value)
        # planned/tcp_move = MoveIt-validated (planner node); tcp_move = straight to the driver.
        tcp_action = p("tcp_action", "planned/tcp_move").value

        self.create_subscription(String, "voice/text", self._on_text, 10)
        self.command_pub = self.create_publisher(MotionCommand, "voice/command", 10)
        self.tcp_client = ActionClient(self, TcpMove, tcp_action)
        self.stop_client = self.create_client(Trigger, "stop")
        self.goto_client = self.create_client(PointName, "points/goto")
        self.loop_start_client = self.create_client(Trigger, "loop/start")
        self.loop_stop_client = self.create_client(Trigger, "loop/stop")
        self.home_client = self.create_client(Trigger, "home/goto")
        self.add_on_set_parameters_callback(self._on_params)

    def _on_params(self, params):
        for param in params:
            if param.name == "require_keyword":
                self.require_keyword = bool(param.value)
            elif param.name == "default_distance_mm":
                self.default_distance_mm = float(param.value)
            elif param.name == "max_distance_mm":
                self.max_distance_mm = float(param.value)
        return SetParametersResult(successful=True)

    def _on_text(self, msg):
        text = msg.data.strip()
        cmd = parse_command(text, require_keyword=self.require_keyword,
                            default_mm=self.default_distance_mm)

        out = MotionCommand()
        out.text = text
        if cmd.kind == "move":
            out.kind = MotionCommand.KIND_TCP
            out.axis = cmd.axis
            out.distance_mm = float(cmd.sign * cmd.distance_mm)
            if cmd.distance_mm > self.max_distance_mm:
                out.kind = MotionCommand.KIND_NONE
                out.reason = f"{cmd.distance_mm:g} mm is over the {self.max_distance_mm:g} mm limit"
        elif cmd.kind == "stop":
            out.kind = MotionCommand.KIND_STOP
        elif cmd.kind == "goto_point":
            out.kind = MotionCommand.KIND_GOTO_POINT
            out.point = cmd.point
        elif cmd.kind == "home":
            out.kind = MotionCommand.KIND_HOME
        elif cmd.kind == "start_loop":
            out.kind = MotionCommand.KIND_START_LOOP
        elif cmd.kind == "end_loop":
            out.kind = MotionCommand.KIND_END_LOOP
        else:
            out.kind = MotionCommand.KIND_NONE
            out.reason = cmd.reason
        self.command_pub.publish(out)

        summary = cmd.describe() if out.kind != MotionCommand.KIND_NONE else f"ignored ({out.reason})"
        self.get_logger().info(f"“{text}” → {summary}")
        self._execute(out)

    def _execute(self, cmd):
        if cmd.kind == MotionCommand.KIND_STOP:
            self._call(self.stop_client, Trigger.Request(), "stop")
            self._call(self.loop_stop_client, Trigger.Request(), "loop/stop", quiet_if_missing=True)
        elif cmd.kind == MotionCommand.KIND_END_LOOP:
            self._call(self.loop_stop_client, Trigger.Request(), "loop/stop")
        elif cmd.kind == MotionCommand.KIND_START_LOOP:
            self._call(self.loop_start_client, Trigger.Request(), "loop/start")
        elif cmd.kind == MotionCommand.KIND_HOME:
            self._call(self.home_client, Trigger.Request(), "home/goto")
        elif cmd.kind == MotionCommand.KIND_GOTO_POINT:
            request = PointName.Request()
            request.name = cmd.point
            self._call(self.goto_client, request, "points/goto")
        elif cmd.kind == MotionCommand.KIND_TCP:
            self._send_tcp(cmd)

    def _call(self, client, request, name, quiet_if_missing=False):
        if not client.service_is_ready():
            if not quiet_if_missing:
                self.get_logger().error(f"Service {name} is not available")
            return
        future = client.call_async(request)

        def done(f):
            response = f.result()
            if response is not None and not response.success:
                self.get_logger().warning(f"{name}: {response.message}")

        future.add_done_callback(done)

    def _send_tcp(self, cmd):
        if not self.tcp_client.server_is_ready():
            self.get_logger().error(f"{self.tcp_client._action_name} is not available (is the stack running?)")
            return
        goal = TcpMove.Goal()
        goal.axis = cmd.axis
        goal.distance_mm = cmd.distance_mm
        future = self.tcp_client.send_goal_async(goal)
        future.add_done_callback(self._goal_sent)

    def _goal_sent(self, future):
        handle = future.result()
        if handle is None or not handle.accepted:
            self.get_logger().warning("TCP move was not accepted")
            return
        handle.get_result_async().add_done_callback(self._goal_done)

    def _goal_done(self, future):
        result = future.result().result
        if not result.success:
            self.get_logger().warning(f"TCP move: {result.message}")


def main(args=None):
    rclpy.init(args=args)
    node = TranslatorNode()
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
