#ifndef EXPLORER_SIM_TYPES_HPP
#define EXPLORER_SIM_TYPES_HPP

// The data that crosses the border between the simulator and the ROS node.
//
// Everything here is plain C++: no threepp types and no ROS types. That is deliberate.
// The simulator half of this program does not know ROS exists, the ROS half does not
// know threepp exists, and these structs are the whole vocabulary they share. You could
// replace the ROS half with MQTT or raw sockets (Part 4) and the simulator would not
// notice.
//
// All coordinates follow the ROS convention: x forward, y left, z up, metres, radians.

#include <chrono>
#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace explorer_sim {

    // Wall-clock time. ROS time is the system clock unless a node is told to use
    // simulated time, so a system_clock time point converts to a ROS stamp without loss.
    using TimePoint = std::chrono::system_clock::time_point;

    struct Pose2D {
        double x = 0;
        double y = 0;
        double yaw = 0;// rotation about z, counter-clockwise seen from above
    };

    // ROS -> simulator: what cmd_vel last asked for.
    struct VelocityCommand {
        double linear = 0; // m/s along the robot's x
        double angular = 0;// rad/s about the robot's z
        std::chrono::steady_clock::time_point received;// for the "no command for 0.5 s" watchdog
    };

    // ROS -> simulator: values that can be changed while the simulator runs
    // (ros2 param set /sim lidar_noise 0.05).
    struct Tuning {
        double lidarNoise = 0.01;// standard deviation of the range noise, m
        double odomDrift = 0.0;  // how much the wheel odometry lies. 0 = perfect
    };

    struct LaserScan {
        double angleMin = 0;      // angle of beam 0 in the laser frame, rad
        double angleIncrement = 0;// rad between neighbouring beams
        double rangeMin = 0;
        double rangeMax = 0;
        std::vector<float> ranges;// one per beam. +inf = nothing within rangeMax
    };

    // Simulator -> ROS: one sensor tick. Every item counts (each one becomes a tf
    // sample), so these travel through a queue, not a mailbox.
    struct Telemetry {
        TimePoint stamp;
        Pose2D odom;        // where the wheel odometry believes the robot is
        Pose2D truth;       // where it really is
        double linear = 0;  // current speed, m/s
        double angular = 0; // current turn rate, rad/s
        std::optional<LaserScan> scan;// present on the ticks where the lidar fires
    };

    // Simulator -> ROS: one picture from the robot's camera. Only the newest matters,
    // so these go through a mailbox. A frame is 690 kB, so the mailbox holds a
    // shared_ptr to an immutable frame: handing it over copies a pointer, not pixels.
    struct CameraFrame {
        TimePoint stamp;// when it was rendered, the same instant as a lidar scan
        int width = 0;
        int height = 0;
        double fovY = 0;// vertical field of view, rad
        std::vector<unsigned char> rgb;// width * height * 3 bytes, row 0 = top of the image
    };
    using CameraFramePtr = std::shared_ptr<const CameraFrame>;

    // The answer to a "drop a crate here" request.
    struct SpawnResult {
        bool success = false;
        std::string message;
    };

}// namespace explorer_sim

#endif//EXPLORER_SIM_TYPES_HPP
