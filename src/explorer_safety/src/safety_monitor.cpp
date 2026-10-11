// safety_monitor: the robot's reflex.
//
// The planner (explorer_nav, Python) decides where to go and publishes velocity
// commands on cmd_vel_nav. Those commands do not go to the motors directly. They pass
// through this node first:
//
//     explorer --cmd_vel_nav--> safety_monitor --cmd_vel--> sim (the motors)
//                                     ^
//                                    scan
//
// If the lidar sees something in the strip of floor the robot is about to drive over,
// the forward speed is scaled down, to zero if it is close. Turning is always allowed,
// so the planner can steer away.
//
// Why a separate node, and why C++? A reflex must be simple, fast and independent of
// the clever code. This node has no map and no plan: if the Python planner crashes,
// stalls in the garbage collector or plans through a crate that was dropped a moment
// ago, this still stops the robot. It also fails safe: with no recent scan it lets
// nothing through.
//
// This is also the smallest complete rclcpp node in the workspace: two subscriptions,
// two publishers, parameters, and shared state between callbacks. A good file to start
// reading with.

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/bool.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <mutex>

class SafetyMonitor: public rclcpp::Node {
public:
    SafetyMonitor(): Node("safety_monitor") {
        robotRadius_ = declare_parameter("robot_radius", 0.22);
        lateralMargin_ = declare_parameter("lateral_margin", 0.04);
        stopClearance_ = declare_parameter("stop_clearance", 0.10);
        slowClearance_ = declare_parameter("slow_clearance", 0.50);
        scanTimeout_ = declare_parameter("scan_timeout", 0.5);

        // Best effort, shallow history: the profile for sensor streams. It must be
        // compatible with the publisher's QoS or no data flows, without any error.
        scanSub_ = create_subscription<sensor_msgs::msg::LaserScan>(
                "scan", rclcpp::SensorDataQoS(),
                [this](const sensor_msgs::msg::LaserScan& scan) { onScan(scan); });

        commandSub_ = create_subscription<geometry_msgs::msg::Twist>(
                "cmd_vel_nav", 10,
                [this](const geometry_msgs::msg::Twist& command) { onCommand(command); });

        commandPub_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

        // "Is the reflex holding the robot back right now?" changes rarely, and a
        // dashboard that starts late still needs the current answer. Transient local
        // makes the publisher keep the last message for late joiners (a "latched" topic).
        stopPub_ = create_publisher<std_msgs::msg::Bool>("safety_stop", rclcpp::QoS(1).reliable().transient_local());
        publishStopState(false);
    }

private:
    // Runs when a scan arrives: work out how much free floor lies ahead and remember it.
    void onScan(const sensor_msgs::msg::LaserScan& scan) {
        // Only returns inside the strip the robot would sweep when driving straight
        // ahead matter: in front of it (forward > 0) and no wider than the robot plus
        // a small margin. A wall running alongside does not count.
        const double halfWidth = robotRadius_ + lateralMargin_;
        double nearest = std::numeric_limits<double>::infinity();

        for (size_t i = 0; i < scan.ranges.size(); ++i) {
            const double range = scan.ranges[i];
            if (!std::isfinite(range) || range < scan.range_min) {
                continue;// no return on this beam
            }
            const double angle = scan.angle_min + static_cast<double>(i) * scan.angle_increment;
            const double forward = range * std::cos(angle);
            const double sideways = range * std::sin(angle);
            if (forward > 0 && std::abs(sideways) < halfWidth) {
                nearest = std::min(nearest, forward);
            }
        }

        // The two callbacks of this node share clearance_ and lastScanTime_. With the
        // default single-threaded executor they can never run at the same time, and
        // this lock would not be needed. But that is a property of how main() spins
        // the node, not of this class: switch to a MultiThreadedExecutor and the same
        // code would be a data race. The lock makes the class correct either way, and
        // an uncontended lock costs next to nothing. (Chapter 7, exercise 4.)
        std::lock_guard<std::mutex> lock(mutex_);
        clearance_ = nearest - robotRadius_;// measured from the robot's front edge
        lastScanTime_ = now();
    }

    // Runs when the planner sends a command: let it through, slowed or stopped as needed.
    void onCommand(const geometry_msgs::msg::Twist& command) {
        double clearance;
        rclcpp::Time lastScanTime;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            clearance = clearance_;
            lastScanTime = lastScanTime_;
        }

        geometry_msgs::msg::Twist safe = command;
        bool holdingBack = false;

        if ((now() - lastScanTime).seconds() > scanTimeout_) {
            // No recent scan: the lidar has failed or its driver has crashed. We are
            // blind, so we do not move at all.
            safe = geometry_msgs::msg::Twist();
            holdingBack = true;
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "no recent scan: holding the robot still");
        } else if (command.linear.x > 0) {
            // 1 when there is plenty of room, falling linearly to 0 at stop_clearance.
            const double scale = std::clamp((clearance - stopClearance_) / (slowClearance_ - stopClearance_), 0.0, 1.0);
            safe.linear.x = command.linear.x * scale;
            // The flag has hysteresis: it is raised at stop_clearance and only lowered
            // again a few centimetres further out. Without that, lidar noise makes the
            // clearance hop back and forth over the threshold and the flag flickers.
            holdingBack = holdingBack_ ? clearance < stopClearance_ + 0.04 : scale == 0.0;
        }

        commandPub_->publish(safe);
        if (holdingBack != holdingBack_) {
            publishStopState(holdingBack);
        }
    }

    void publishStopState(bool holdingBack) {
        holdingBack_ = holdingBack;
        std_msgs::msg::Bool msg;
        msg.data = holdingBack;
        stopPub_->publish(msg);
    }

    // Parameters.
    double robotRadius_;
    double lateralMargin_;
    double stopClearance_;
    double slowClearance_;
    double scanTimeout_;

    // Shared between the two callbacks, guarded by mutex_.
    std::mutex mutex_;
    double clearance_ = 0.0;// free floor ahead of the robot's front edge, m
    rclcpp::Time lastScanTime_{0, 0, RCL_ROS_TIME};

    // Only used by onCommand.
    bool holdingBack_ = false;

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scanSub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr commandSub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr commandPub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr stopPub_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SafetyMonitor>());// single-threaded executor: callbacks run one at a time
    rclcpp::shutdown();
    return 0;
}
