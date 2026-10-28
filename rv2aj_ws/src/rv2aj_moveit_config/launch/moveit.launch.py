"""MoveIt for the RV-2AJ: robot_state_publisher and move_group. Trajectory
execution runs through rv2aj_control's planner (its FollowJointTrajectory
controller).

The robot's joint states come from the rv2aj driver (/rv2aj/joint_states).

  ros2 launch rv2aj_moveit_config moveit.launch.py
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def moveit_config():
    description = os.path.join(get_package_share_directory("rv2aj_description"), "urdf", "rv2aj.urdf.xacro")
    return (
        MoveItConfigsBuilder("rv2aj", package_name="rv2aj_moveit_config")
        .robot_description(file_path=description)
        .robot_description_semantic(file_path="config/rv2aj.srdf")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .joint_limits(file_path="config/joint_limits.yaml")
        .trajectory_execution(file_path="config/moveit_controllers.yaml", moveit_manage_controllers=False)
        .planning_pipelines(pipelines=["ompl"], default_planning_pipeline="ompl")
        .planning_scene_monitor(publish_robot_description=True, publish_robot_description_semantic=True)
        .to_moveit_configs()
    )


def generate_launch_description():
    config = moveit_config()
    joint_states = LaunchConfiguration("joint_states_topic")

    return LaunchDescription([
        DeclareLaunchArgument("joint_states_topic", default_value="/rv2aj/joint_states"),
        Node(package="robot_state_publisher", executable="robot_state_publisher", output="log",
             parameters=[config.robot_description], remappings=[("joint_states", joint_states)]),
        Node(package="moveit_ros_move_group", executable="move_group", output="screen",
             parameters=[config.to_dict(), {
                 "allow_trajectory_execution": True,
                 # The robot stops at each waypoint, so it is slower than MoveIt's timing.
                 "trajectory_execution.execution_duration_monitoring": False,
             }],
             remappings=[("joint_states", joint_states)]),
    ])
