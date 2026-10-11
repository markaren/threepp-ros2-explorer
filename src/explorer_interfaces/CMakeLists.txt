cmake_minimum_required(VERSION 3.21)
project(explorer_interfaces)

# This package contains no code we wrote: only message, service and action definitions.
# rosidl reads them at build time and generates the C++ headers and the Python classes,
# which is how a C++ node and a Python node end up speaking exactly the same types.

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ExplorationStatus.msg"
  "srv/SpawnObstacle.srv"
  "action/Explore.action"
  DEPENDENCIES geometry_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
