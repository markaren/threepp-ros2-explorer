cmake_minimum_required(VERSION 3.21)
project(threepp_ros2_demo LANGUAGES C CXX)

# This file exists ONLY so CLion (a CMake IDE) has something to open.
# colcon still drives the real build. See "ROS2 Workspace" in the book: two build
# systems, one source of truth.
#
# What it gives you in CLion:
#   * code navigation, completion and error highlighting in the C++ packages
#   * one build target per package (colcon_build_<package>) and colcon_build_all,
#     which run the same build as `pixi run -e jazzy build`
#
# Start CLion from inside the environment, so it can find colcon and ROS:
#   pixi shell -e jazzy
#   clion .          (or the full path to clion64.exe)

# Where find_package looks for ROS 2: this workspace's own pixi environment first,
# then the course's shared one in C:\robostack.
if (WIN32)
    list(APPEND CMAKE_PREFIX_PATH
            "${CMAKE_SOURCE_DIR}/.pixi/envs/jazzy/Library"
            "C:/robostack/.pixi/envs/jazzy/Library"
    )
endif ()
# Packages of this workspace that are already built (our own messages).
list(APPEND CMAKE_PREFIX_PATH "${CMAKE_SOURCE_DIR}/install")

find_package(ament_cmake REQUIRED)

# The C++ packages, so the IDE indexes them. explorer_sim needs the generated message
# headers of explorer_interfaces: build once with colcon before opening the project.
if (EXISTS "${CMAKE_SOURCE_DIR}/install/share/explorer_interfaces")
    add_subdirectory(src/explorer_safety)
    add_subdirectory(src/explorer_sim)
else ()
    message(WARNING "Build the workspace once first (pixi run -e jazzy build), then reload CMake.")
endif ()

# ---- colcon build targets ----
file(GLOB _package_xmls RELATIVE "${CMAKE_SOURCE_DIR}/src" "src/*/package.xml")
foreach (_xml IN LISTS _package_xmls)
    get_filename_component(_package "${_xml}" DIRECTORY)
    add_custom_target(colcon_build_${_package}
            COMMAND cmd /c build.bat --packages-select ${_package}
            WORKING_DIRECTORY "${CMAKE_SOURCE_DIR}"
            USES_TERMINAL
            COMMENT "colcon build, package ${_package}"
    )
endforeach ()

add_custom_target(colcon_build_all
        COMMAND cmd /c build.bat
        WORKING_DIRECTORY "${CMAKE_SOURCE_DIR}"
        USES_TERMINAL
        COMMENT "colcon build, all packages"
)
