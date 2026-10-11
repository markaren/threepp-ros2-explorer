cmake_minimum_required(VERSION 3.28)
project(explorer_sim LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ---- ROS 2 dependencies -------------------------------------------------------------
# All of these come from the pixi environment. find_package locates them because the
# environment's Library/ folder is on CMAKE_PREFIX_PATH (colcon and the environment's
# activation take care of that).
find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(explorer_interfaces REQUIRED)   # our own messages, built by the package next door

# OpenCV also comes from the pixi environment. "One owner per library" (Part 5 of the
# book): inside a ROS 2 workspace the conda environment owns OpenCV. Do not add a
# second copy through vcpkg.
find_package(OpenCV REQUIRED COMPONENTS core imgproc imgcodecs)

# ---- threepp ------------------------------------------------------------------------
# threepp is not a ROS package and not in the conda environment, so CMake fetches its
# source and builds it as part of this package. The tag is the same release as the
# `threepp` wheel the Python nodes use (2026.10.11 on PyPI).
#
# Only the library is built: examples and tests are off by default when threepp is
# consumed this way. It has no dependencies of its own to install (GLFW is bundled).
include(FetchContent)
set(THREEPP_WITH_AUDIO OFF)   # no sound in this demo
FetchContent_Declare(
        threepp
        GIT_REPOSITORY https://github.com/markaren/threepp.git
        GIT_TAG 2026-10-11
        GIT_SHALLOW TRUE
        # Keep threepp's own install rules out of our install/ tree: it is linked
        # statically into the executable, so nothing of it is needed at run time.
        EXCLUDE_FROM_ALL
)
FetchContent_MakeAvailable(threepp)

# ---- the node -----------------------------------------------------------------------
# Two halves that do not include each other's libraries:
#   simulator.cpp, scene.cpp, window.cpp, world.cpp   threepp only  (no ROS headers)
#   sim_node.cpp                                      ROS only      (no threepp headers)
# main.cpp wires them together. Building the simulator half as its own library makes
# the rule checkable: it does not link rclcpp, so a stray ROS include fails to compile.
add_library(simulator STATIC
        src/simulator.cpp   # what happens: physics, sensors
        src/scene.cpp       # what is drawn
        src/window.cpp      # the window and the frame loop
        src/world.cpp       # the building as data, collision
)
target_include_directories(simulator PUBLIC include)
target_link_libraries(simulator PUBLIC threepp::threepp)

add_executable(sim
        src/main.cpp
        src/sim_node.cpp
)
target_include_directories(sim PRIVATE include)
target_link_libraries(sim PRIVATE
        simulator
        rclcpp::rclcpp
        ament_index_cpp::ament_index_cpp
        tf2_ros::tf2_ros
        ${geometry_msgs_TARGETS}
        ${nav_msgs_TARGETS}
        ${sensor_msgs_TARGETS}
        ${std_srvs_TARGETS}
        ${explorer_interfaces_TARGETS}
        opencv_core opencv_imgproc opencv_imgcodecs
)

if (MSVC)
    # M_PI and friends, and no min/max macros from windows.h
    target_compile_definitions(simulator PUBLIC _USE_MATH_DEFINES NOMINMAX)
endif ()

# `ros2 run explorer_sim sim` looks for executables in lib/<package name>/
install(TARGETS sim DESTINATION lib/${PROJECT_NAME})

# The textures and the HDRI (see assets/README.md). At run time the node finds them
# with ament_index_cpp::get_package_share_directory("explorer_sim").
install(DIRECTORY assets DESTINATION share/${PROJECT_NAME})

ament_package()
