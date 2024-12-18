cmake_minimum_required(VERSION 3.5)
project(hydra_msgs)

find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

set(msg_files "msg/ActiveLayer.msg" "msg/DsgUpdate.msg")
set(srv_files "srv/GetDsg.srv" "srv/QueryFreespace.srv")

rosidl_generate_interfaces(${PROJECT_NAME}
  ${srv_files}
  ${msg_files}
  DEPENDENCIES std_msgs
)

ament_export_dependencies(rosidl_default_runtime std_msgs)

ament_package()
