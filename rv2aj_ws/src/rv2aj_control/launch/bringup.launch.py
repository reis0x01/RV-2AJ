"""Bring up the full RV-2AJ stack.

  ros2 launch rv2aj_control bringup.launch.py                          # real robot
  ros2 launch rv2aj_control bringup.launch.py use_fake_hardware:=true  # simulator
  ros2 launch rv2aj_control bringup.launch.py use_moveit:=false        # no MoveIt, direct moves
  args: port, gui, voice, points_file, plc

rv2aj nodes run in /rv2aj; MoveIt (move_group, robot_state_publisher)
runs in the root namespace and follows /rv2aj/joint_states.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node

# VS Code (snap) terminals leak snap GTK paths that crash PyQt5; strip them for GUIs.
SNAP_GTK_VARS = ("GTK_PATH", "GDK_PIXBUF_MODULEDIR", "GDK_PIXBUF_MODULE_FILE", "LOCPATH",
                 "GIO_MODULE_DIR", "GTK_IM_MODULE_FILE", "SNAP_LIBRARY_PATH",
                 "GSETTINGS_SCHEMA_DIR", "GTK_EXE_PREFIX")


SIM_DIR = "~/.ros/rv2aj_sim"
SIM_POINTS = SIM_DIR + "/points.json"


def _seed_sim_points():
    real, sim = os.path.expanduser("~/.ros/rv2aj_points.json"), os.path.expanduser(SIM_POINTS)
    os.makedirs(os.path.dirname(sim), exist_ok=True)
    if not os.path.exists(sim) and os.path.exists(real):
        import shutil
        shutil.copyfile(real, sim)


def generate_launch_description():
    config = os.path.join(get_package_share_directory("rv2aj_control"), "config", "rv2aj.yaml")
    moveit_launch = os.path.join(get_package_share_directory("rv2aj_moveit_config"), "launch", "moveit.launch.py")
    use_moveit = LaunchConfiguration("use_moveit")
    moveit_on = IfCondition(use_moveit)
    prefix = PythonExpression(["'planned/' if '", use_moveit, "' == 'true' else ''"])
    gui_env = {k: v for k, v in os.environ.items() if k not in SNAP_GTK_VARS}
    # The simulator keeps its own points, so testing never writes
    # into the real robot's files. The first run copies the real points in.
    sim = LaunchConfiguration("use_fake_hardware")
    _seed_sim_points()
    points_file = PythonExpression([
        "'", LaunchConfiguration("points_file"), "' if '", LaunchConfiguration("points_file"), "' != 'auto' else ",
        "('", SIM_POINTS, "' if '", sim, "' == 'true' else '~/.ros/rv2aj_points.json')"])

    def node(executable, condition=None, extra=None, **kwargs):
        params = [config] + ([extra] if extra else [])
        return Node(package="rv2aj_control", executable=executable, name=executable,
                    namespace="rv2aj", parameters=params, output="screen",
                    condition=condition, **kwargs)

    return LaunchDescription([
        DeclareLaunchArgument("use_fake_hardware", default_value="false",
                              description="Simulate the controller instead of opening the serial port"),
        DeclareLaunchArgument("port", default_value="/dev/ttyUSB0"),
        DeclareLaunchArgument("use_moveit", default_value="true",
                              description="Validate every move with MoveIt before it reaches the robot"),
        DeclareLaunchArgument("gui", default_value="true"),
        DeclareLaunchArgument("voice", default_value="true"),
        DeclareLaunchArgument("points_file", default_value="auto",
                              description="auto: ~/.ros/rv2aj_points.json, or ~/.ros/rv2aj_sim/points.json in simulation"),
        DeclareLaunchArgument("plc", default_value="true",
                              description="Conveyor PLC bridge (S7-1200 over the network; idle if plc_ip is empty)"),

        node("driver", extra={"use_fake_hardware": LaunchConfiguration("use_fake_hardware"),
                              "port": LaunchConfiguration("port")}),
        node("planner", condition=moveit_on),
        node("sequencer", extra={"points_file": points_file,
                                 "joint_action": [prefix, "joint_move"]}),
        node("plc_bridge", condition=IfCondition(LaunchConfiguration("plc"))),
        node("translator", extra={"tcp_action": [prefix, "tcp_move"]}),
        node("voice", condition=IfCondition(LaunchConfiguration("voice"))),
        node("gui", condition=IfCondition(LaunchConfiguration("gui")), env=gui_env,
             extra={"use_planner": use_moveit}),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(moveit_launch),
            launch_arguments={"joint_states_topic": "/rv2aj/joint_states"}.items(),
            condition=moveit_on,
        ),
    ])
