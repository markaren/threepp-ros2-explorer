cmake_minimum_required(VERSION 3.16)
project(explorer_safety LANGUAGES CXX)

# The smallest C++ package in the workspace: one source file, one node, no threepp.
# If you are new to ament_cmake, start here and read explorer_sim's CMakeLists after.

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(safety_monitor src/safety_monitor.cpp)
target_link_libraries(safety_monitor PRIVATE
        rclcpp::rclcpp
        ${geometry_msgs_TARGETS}
        ${sensor_msgs_TARGETS}
        ${std_msgs_TARGETS}
)

if (MSVC)
    target_compile_definitions(safety_monitor PRIVATE _USE_MATH_DEFINES NOMINMAX)
    target_compile_options(safety_monitor PRIVATE /W4)
endif ()

# `ros2 run explorer_safety safety_monitor` looks in lib/<package name>/
install(TARGETS safety_monitor DESTINATION lib/${PROJECT_NAME})

ament_package()
