cmake_minimum_required(VERSION 3.8)
project(rv2aj_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/RobotStatus.msg"
  "msg/MotionCommand.msg"
  "msg/TeachPoint.msg"
  "msg/SequencerStatus.msg"
  "srv/Connect.srv"
  "srv/SetOverride.srv"
  "srv/RunProgram.srv"
  "srv/SavePoint.srv"
  "srv/SetPointSpeed.srv"
  "srv/SetPointIO.srv"
  "srv/SetAutoRun.srv"
  "srv/PointName.srv"
  "srv/LineSignal.srv"
  "action/TcpMove.action"
  "action/JointMove.action"
  DEPENDENCIES std_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
