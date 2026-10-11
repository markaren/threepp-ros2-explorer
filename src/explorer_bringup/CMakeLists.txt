cmake_minimum_required(VERSION 3.16)
project(explorer_bringup NONE)

# A package without code: it only installs the launch files, the parameter file and
# the RViz configuration, so that `ros2 launch explorer_bringup ...` finds them.
# (LANGUAGES NONE: there is nothing to compile, so CMake need not look for a compiler.)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch config rviz DESTINATION share/${PROJECT_NAME})

ament_package()
