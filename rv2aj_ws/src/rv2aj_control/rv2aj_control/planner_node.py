"""MoveIt-validated motion planning in front of the driver.

Offers the same actions as the driver, under planned/: planned/tcp_move and
planned/joint_move. Every move is turned into joint waypoints, checked by
MoveIt (/check_state_validity: self-collision and the planning scene) and
against the joint limits, and only then executed waypoint by waypoint via
the driver's joint_move action. The driver moves between waypoints with
EXECMOV, i.e. a straight line in joint space, which is exactly the path that
was validated.

  TCP moves:   exact 5-axis IK along a straight line (pitch and J6 kept, as
               the controller does), then the fewest waypoints that stay
               within `line_tolerance_mm` of the true straight line.
  Joint moves: the direct joint-space line if MoveIt finds it collision
               free; otherwise an OMPL plan from /plan_kinematic_path,
               shortcut to the fewest waypoints whose segments are valid.

The controller commands each waypoint becomes are published on
planner/robot_commands and logged, so the translation to the robot's own
language is visible.

It is also MoveIt's trajectory controller (/rv2aj_arm_controller/
follow_joint_trajectory), so any MoveIt trajectory execution goes through
the same validation and waypoint execution.
"""

import math
import threading
import time

import rclpy
from moveit_msgs.msg import (
    Constraints,
    DisplayTrajectory,
    JointConstraint,
    MotionPlanRequest,
    RobotState,
    RobotTrajectory,
)
from control_msgs.action import FollowJointTrajectory
from moveit_msgs.srv import GetMotionPlan, GetStateValidity
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from rv2aj_interfaces.action import JointMove, TcpMove
from rv2aj_interfaces.msg import RobotStatus

from . import kinematics
from .kinematics import IKError

JOINTS = ("J1", "J2", "J3", "J5", "J6")          # the robot's fitted joints
URDF_NAMES = {"J1": "j1", "J2": "j2", "J3": "j3", "J5": "j5", "J6": "j6"}
ALL_JOINTS = ("J1", "J2", "J3", "J4", "J5", "J6")
MIN_RADIAL_RADIUS_MM = 50.0


class PlanningError(Exception):
    pass


class PlannerNode(Node):
    def __init__(self):
        super().__init__("planner")
        p = self.declare_parameter
        self.check_step_deg = float(p("check_step_deg", 1.0).value)
        self.line_step_mm = float(p("line_step_mm", 5.0).value)
        self.line_tolerance_mm = float(p("line_tolerance_mm", 1.0).value)
        self.max_tcp_move_mm = float(p("max_tcp_move_mm", 50.0).value)
        self.max_joint_step_deg = float(p("max_joint_step_deg", 45.0).value)
        self.planning_time_s = float(p("planning_time_s", 5.0).value)
        self.group = p("planning_group", "arm").value

        self.robot = None
        self.busy = False
        self.current_goal = None
        self.lock = threading.Lock()
        group = ReentrantCallbackGroup()

        self.create_subscription(RobotStatus, "status", self._on_status, 10, callback_group=group)
        self.commands_pub = self.create_publisher(String, "planner/robot_commands", 10)
        self.display_pub = self.create_publisher(DisplayTrajectory, "/display_planned_path", 10)
        self.validity = self.create_client(GetStateValidity, "/check_state_validity", callback_group=group)
        self.planner = self.create_client(GetMotionPlan, "/plan_kinematic_path", callback_group=group)
        self.driver = ActionClient(self, JointMove, "joint_move", callback_group=group)

        common = dict(callback_group=group, goal_callback=self._accept, cancel_callback=self._cancel)
        ActionServer(self, TcpMove, "planned/tcp_move", self._execute_tcp, **common)
        ActionServer(self, JointMove, "planned/joint_move", self._execute_joint, **common)
        # MoveIt's trajectory controller (see moveit_controllers.yaml).
        ActionServer(self, FollowJointTrajectory, "/rv2aj_arm_controller/follow_joint_trajectory",
                     self._execute_trajectory, **common)

    # -- plumbing ------------------------------------------------------------

    def _on_status(self, msg):
        self.robot = msg

    def _accept(self, _goal):
        with self.lock:
            if self.busy:
                self.get_logger().warning("Move refused: planner is busy with another move")
                return GoalResponse.REJECT
            self.busy = True
        return GoalResponse.ACCEPT

    def _cancel(self, _goal_handle):
        handle = self.current_goal
        if handle is not None:
            handle.cancel_goal_async()   # driver sends STOP
        return CancelResponse.ACCEPT

    def _wait(self, future, timeout):
        end = time.monotonic() + timeout
        while not future.done():
            if time.monotonic() > end:
                raise PlanningError("timed out waiting for MoveIt")
            time.sleep(0.005)
        return future.result()

    def _current(self):
        r = self.robot
        if r is None or not r.connected:
            raise PlanningError("robot not connected")
        if not r.servo_on:
            raise PlanningError("servo is off")
        if r.moving:
            raise PlanningError(f"busy: {r.active_motion}")
        q = {j: r.joints_deg[ALL_JOINTS.index(j)] for j in JOINTS}
        if any(math.isnan(v) for v in q.values()):
            raise PlanningError("joint angles not known yet")
        return q

    # -- MoveIt checks ------------------------------------------------------------

    def _robot_state(self, q):
        js = JointState()
        js.name = [URDF_NAMES[j] for j in JOINTS]
        js.position = [math.radians(q[j]) for j in JOINTS]
        return RobotState(joint_state=js)

    def _state_valid(self, q, ref=None):
        violations = kinematics.limit_violations(q, ref)
        if violations:
            return False, "outside joint limits: " + ", ".join(violations)
        if not self.validity.service_is_ready():
            raise PlanningError("MoveIt (move_group) is not running")
        request = GetStateValidity.Request()
        request.robot_state = self._robot_state(q)
        request.group_name = self.group
        result = self._wait(self.validity.call_async(request), 5.0)
        if result.valid:
            return True, ""
        contacts = sorted({f"{c.contact_body_1}-{c.contact_body_2}" for c in result.contacts})
        return False, "collision: " + (", ".join(contacts) if contacts else "invalid state")

    def _segment_valid(self, qa, qb, ref=None):
        """Check the joint-space straight line qa->qb, which is what EXECMOV follows.
        `ref` is the pose the move starts from, for the "no worse than now" limit rule."""
        span = max(abs(qb[j] - qa[j]) for j in JOINTS)
        steps = max(1, math.ceil(span / self.check_step_deg))
        for i in range(1, steps + 1):
            f = i / steps
            ok, why = self._state_valid({j: qa[j] + f * (qb[j] - qa[j]) for j in JOINTS}, ref if ref else qa)
            if not ok:
                return False, why
        return True, ""

    def _path_valid(self, q0, waypoints):
        previous = q0
        for q in waypoints:
            ok, why = self._segment_valid(previous, q, q0)
            if not ok:
                return False, why
            previous = q
        return True, ""

    # -- planning -----------------------------------------------------------------

    def _plan_tcp(self, axis, distance_mm, vector_mm=(0.0, 0.0, 0.0)):
        if axis == "V":
            distance_mm = math.sqrt(sum(v * v for v in vector_mm))
        if distance_mm == 0 or abs(distance_mm) > self.max_tcp_move_mm:
            raise PlanningError(f"distance must be non-zero and at most {self.max_tcp_move_mm:g} mm")
        q0 = self._current()
        p0 = kinematics.fk(q0["J1"], q0["J2"], q0["J3"], q0["J5"], q0["J6"])
        if axis == "R":
            radius = math.hypot(p0["X"], p0["Y"])
            if radius < MIN_RADIAL_RADIUS_MM:
                raise PlanningError("TCP is too close to the base axis to define forward/back")
            delta = (distance_mm * p0["X"] / radius, distance_mm * p0["Y"] / radius, 0.0)
        elif axis in ("X", "Y", "Z"):
            delta = tuple(distance_mm if a == axis else 0.0 for a in "XYZ")
        elif axis == "V":
            delta = tuple(float(v) for v in vector_mm)
        else:
            raise PlanningError(f"unknown axis {axis!r}")
        try:
            dense = kinematics.straight_line(q0, delta, step_mm=self.line_step_mm)
        except IKError as e:
            raise PlanningError(str(e)) from e
        ok, why = self._path_valid(q0, dense)
        if not ok:
            raise PlanningError(f"MoveIt rejected the path: {why}")
        waypoints = self._fewest_line_waypoints(q0, dense, p0, delta)
        ok, why = self._path_valid(q0, waypoints)
        if not ok:
            raise PlanningError(f"MoveIt rejected the path: {why}")
        return q0, waypoints

    def _fewest_line_waypoints(self, q0, dense, p0, delta):
        """Keep only waypoints needed so joint-space segments stay near the line."""
        length = math.sqrt(sum(d * d for d in delta))
        unit = [d / length for d in delta]

        def off_line(qa, qb):
            worst = 0.0
            for k in range(1, 11):
                f = k / 10
                q = {j: qa[j] + f * (qb[j] - qa[j]) for j in JOINTS}
                p = kinematics.fk(q["J1"], q["J2"], q["J3"], q["J5"], q["J6"])
                v = [p["X"] - p0["X"], p["Y"] - p0["Y"], p["Z"] - p0["Z"]]
                along = sum(a * b for a, b in zip(v, unit))
                worst = max(worst, math.sqrt(max(0.0, sum(a * a for a in v) - along * along)))
            return worst

        kept, start, i = [], q0, 0
        while i < len(dense):
            j = len(dense) - 1
            while j > i and off_line(start, dense[j]) > self.line_tolerance_mm:
                j -= 1
            kept.append(dense[j])
            start, i = dense[j], j + 1
        return kept

    def _plan_joints(self, goal):
        q0 = self._current()
        if goal.relative:
            joint = goal.joint.upper()
            if joint not in JOINTS:
                raise PlanningError(f"{joint} cannot be moved on this robot")
            if goal.offset_deg == 0 or abs(goal.offset_deg) > self.max_joint_step_deg:
                raise PlanningError(f"offset must be non-zero and at most {self.max_joint_step_deg:g}°")
            if joint in kinematics.FROZEN_JOINTS:
                raise PlanningError(f"{joint} is frozen (encoder fault) and does not move; see joint_limits.yaml frozen_joints")
            target = dict(q0)
            target[joint] += goal.offset_deg
        else:
            target = {j: float(goal.target_deg[ALL_JOINTS.index(j)]) for j in JOINTS}
            for j in kinematics.FROZEN_JOINTS & set(JOINTS):
                target[j] = q0[j]  # a frozen joint keeps its current angle (home / points)
        ok, why = self._state_valid(target, q0)
        if not ok:
            raise PlanningError(f"goal is not valid: {why}")

        ok, why = self._segment_valid(q0, target, q0)
        if ok:
            return q0, [target]
        self.get_logger().info(f"Direct joint path blocked ({why}); asking OMPL for a detour")
        return q0, self._ompl_plan(q0, target)

    def _ompl_plan(self, q0, target):
        if not self.planner.service_is_ready():
            raise PlanningError("MoveIt (move_group) is not running")
        mpr = MotionPlanRequest()
        mpr.group_name = self.group
        mpr.start_state = self._robot_state(q0)
        mpr.allowed_planning_time = self.planning_time_s
        mpr.num_planning_attempts = 3
        constraints = Constraints()
        for j in JOINTS:
            constraints.joint_constraints.append(JointConstraint(
                joint_name=URDF_NAMES[j], position=math.radians(target[j]),
                tolerance_above=1e-4, tolerance_below=1e-4, weight=1.0))
        mpr.goal_constraints.append(constraints)
        for j in kinematics.FROZEN_JOINTS & set(JOINTS):  # OMPL detours must not move a frozen joint either
            mpr.path_constraints.joint_constraints.append(JointConstraint(
                joint_name=URDF_NAMES[j], position=math.radians(q0[j]),
                tolerance_above=1e-3, tolerance_below=1e-3, weight=1.0))
        response = self._wait(self.planner.call_async(GetMotionPlan.Request(motion_plan_request=mpr)),
                              self.planning_time_s + 5.0).motion_plan_response
        if response.error_code.val != 1:
            raise PlanningError(f"OMPL found no valid path (MoveIt error {response.error_code.val})")
        names = response.trajectory.joint_trajectory.joint_names
        path = [{j: math.degrees(pt.positions[names.index(URDF_NAMES[j])]) for j in JOINTS}
                for pt in response.trajectory.joint_trajectory.points[1:]]
        path[-1] = target
        return self._shortcut(q0, path)

    def _shortcut(self, q0, path):
        """Fewest waypoints: from each kept point jump to the furthest point whose
        joint-space segment MoveIt accepts, then re-validate the whole result."""
        kept, start, i = [], q0, 0
        while i < len(path):
            j = len(path) - 1
            while j > i and not self._segment_valid(start, path[j], q0)[0]:
                j -= 1
            kept.append(path[j])
            start, i = path[j], j + 1
        ok, why = self._path_valid(q0, kept)
        if not ok:
            raise PlanningError(f"path failed validation: {why}")
        return kept

    # -- translation + execution ----------------------------------------------------

    def _publish_plan(self, q0, waypoints, label):
        traj = JointTrajectory()
        traj.joint_names = [URDF_NAMES[j] for j in JOINTS]
        for i, q in enumerate([q0] + waypoints):
            pt = JointTrajectoryPoint()
            pt.positions = [math.radians(q[j]) for j in JOINTS]
            pt.time_from_start.sec = i
            traj.points.append(pt)
        display = DisplayTrajectory()
        display.trajectory_start = self._robot_state(q0)
        display.trajectory.append(RobotTrajectory(joint_trajectory=traj))
        self.display_pub.publish(display)

        lines = [f"# {label}: {len(waypoints)} validated waypoint(s)"]
        for n, q in enumerate(waypoints, 1):
            values = ",".join(f"{q.get(j, 0.0):.2f}" for j in ALL_JOINTS)
            lines += [f"# waypoint {n}", f"1;1;EXECJTMP=({values})", "1;1;VALJTMP", "1;1;EXECMOV JTMP"]
        self.commands_pub.publish(String(data="\n".join(lines)))
        self.get_logger().info(f"{label}: {len(waypoints)} waypoint(s), MoveIt-validated → "
                               + "; ".join("EXECMOV (" + ", ".join(f"{q[j]:.1f}" for j in JOINTS) + ")"
                                           for q in waypoints))

    def _execute_waypoints(self, goal_handle, waypoints, feedback, fill_feedback):
        if not self.driver.server_is_ready():
            raise PlanningError("driver joint_move action is not available")
        for n, q in enumerate(waypoints, 1):
            if goal_handle.is_cancel_requested:
                return False, "cancelled"
            goal = JointMove.Goal()
            goal.relative = False
            goal.target_deg = [q.get(j, 0.0) for j in ALL_JOINTS]
            handle = self._wait(self.driver.send_goal_async(goal), 10.0)
            if not handle.accepted:
                return False, f"driver refused waypoint {n}"
            self.current_goal = handle
            result_future = handle.get_result_async()
            while not result_future.done():
                if self.robot is not None and feedback is not None:
                    fill_feedback(feedback, self.robot)
                    goal_handle.publish_feedback(feedback)
                time.sleep(0.2)
            self.current_goal = None
            result = result_future.result().result
            if not result.success:
                return False, f"waypoint {n}/{len(waypoints)}: {result.message}"
        return True, "reached"

    def _run(self, goal_handle, plan, label, result, feedback, fill_feedback, fill_result):
        try:
            try:
                q0, waypoints = plan()
            except PlanningError as e:
                self.get_logger().warning(f"{label} refused: {e}")
                result.success, result.message = False, str(e)
                goal_handle.abort()
                return result
            self._publish_plan(q0, waypoints, label)
            try:
                ok, message = self._execute_waypoints(goal_handle, waypoints, feedback, fill_feedback)
            except PlanningError as e:
                ok, message = False, str(e)
            result.success, result.message = ok, message
            if self.robot is not None:
                fill_result(result, self.robot)
            if ok:
                goal_handle.succeed()
            elif goal_handle.is_cancel_requested:
                goal_handle.canceled()
            else:
                self.get_logger().warning(f"{label}: {message}")
                goal_handle.abort()
            return result
        finally:
            self.current_goal = None
            with self.lock:
                self.busy = False

    @staticmethod
    def _xyz(robot):
        return [robot.x_mm, robot.y_mm, robot.z_mm]

    def _execute_tcp(self, goal_handle):
        goal = goal_handle.request
        axis = goal.axis.upper()
        if axis == "V":
            label = "TCP jog (" + ", ".join(f"{v:+.1f}" for v in goal.vector_mm) + ") mm"
        else:
            where = "along arm" if axis == "R" else axis
            label = f"TCP {where} {goal.distance_mm:+g} mm"
        return self._run(
            goal_handle, lambda: self._plan_tcp(axis, goal.distance_mm, list(goal.vector_mm)),
            label, TcpMove.Result(), TcpMove.Feedback(),
            lambda fb, r: setattr(fb, "current_xyz_mm", self._xyz(r)),
            lambda res, r: setattr(res, "final_xyz_mm", self._xyz(r)),
        )

    def _execute_joint(self, goal_handle):
        goal = goal_handle.request
        label = f"{goal.joint.upper()} step {goal.offset_deg:+g}°" if goal.relative else "joint move"
        return self._run(
            goal_handle, lambda: self._plan_joints(goal), label, JointMove.Result(), JointMove.Feedback(),
            lambda fb, r: setattr(fb, "current_deg", list(r.joints_deg)),
            lambda res, r: setattr(res, "final_deg", list(r.joints_deg)),
        )

    def _execute_trajectory(self, goal_handle):
        """MoveIt trajectory execution: re-validate MoveIt's trajectory, shortcut it
        to the fewest valid waypoints and run them through the driver."""
        result = FollowJointTrajectory.Result()
        try:
            trajectory = goal_handle.request.trajectory
            names = list(trajectory.joint_names)
            index = {j: names.index(URDF_NAMES[j]) for j in JOINTS if URDF_NAMES[j] in names}
            try:
                if len(index) != len(JOINTS):
                    raise PlanningError(f"trajectory joints {names} do not match {list(URDF_NAMES.values())}")
                q0 = self._current()
                path = [{j: math.degrees(pt.positions[index[j]]) for j in JOINTS} for pt in trajectory.points]
                moved = [j for j in kinematics.FROZEN_JOINTS & set(JOINTS) if any(abs(q[j] - q0[j]) > 0.05 for q in path)]
                if moved:
                    raise PlanningError(f"{', '.join(moved)} is frozen (encoder fault); this trajectory moves it")
                path = [q for q in path if max(abs(q[j] - q0[j]) for j in JOINTS) > 0.01] or [q0]
                # Prefer the straight joint path to MoveIt's goal (what EXECMOV does);
                # fall back to MoveIt's own path only when that is blocked.
                direct_ok, _ = self._segment_valid(q0, path[-1], q0)
                waypoints = [path[-1]] if direct_ok else self._shortcut(q0, path)
            except PlanningError as e:
                self.get_logger().warning(f"MoveIt trajectory refused: {e}")
                result.error_code = FollowJointTrajectory.Result.INVALID_GOAL
                result.error_string = str(e)
                goal_handle.abort()
                return result

            self._publish_plan(q0, waypoints, f"MoveIt trajectory ({len(trajectory.points)} points)")
            try:
                ok, message = self._execute_waypoints(goal_handle, waypoints, None, None)
            except PlanningError as e:
                ok, message = False, str(e)
            if ok:
                result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
                goal_handle.succeed()
            else:
                self.get_logger().warning(f"MoveIt trajectory: {message}")
                result.error_code = FollowJointTrajectory.Result.PATH_TOLERANCE_VIOLATED
                result.error_string = message
                if goal_handle.is_cancel_requested:
                    goal_handle.canceled()
                else:
                    goal_handle.abort()
            return result
        finally:
            self.current_goal = None
            with self.lock:
                self.busy = False


def main(args=None):
    rclpy.init(args=args)
    node = PlannerNode()
    executor = MultiThreadedExecutor(num_threads=6)
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
