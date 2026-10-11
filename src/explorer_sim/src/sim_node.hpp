#ifndef EXPLORER_SIM_SIM_NODE_HPP
#define EXPLORER_SIM_SIM_NODE_HPP

// The ROS half of the simulator. No threepp headers in this file or in sim_node.cpp.
//
// The node translates between ROS messages and the plain structs in types.hpp:
//
//   cmd_vel (Twist)            ->  Mailbox<VelocityCommand>
//   parameters                 ->  Mailbox<Tuning>
//   Telemetry (from the queue) ->  tf, odom, ground_truth, scan
//   Mailbox<CameraFramePtr>    ->  camera/image/compressed, camera/camera_info
//   services                   ->  Simulator::request...() and a future for the answer

#include "explorer_sim/mailbox.hpp"
#include "explorer_sim/simulator.hpp"
#include "explorer_sim/types.hpp"

#include "explorer_interfaces/srv/spawn_obstacle.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "tf2_ros/static_transform_broadcaster.h"
#include "tf2_ros/transform_broadcaster.h"

#include <memory>

namespace explorer_sim {

    class SimNode: public rclcpp::Node {
    public:
        // The node's ends of the channels. main() owns them.
        struct Channels {
            Mailbox<VelocityCommand>& commands;
            Mailbox<Tuning>& tuning;
            Mailbox<CameraFramePtr>& camera;
        };

        explicit SimNode(Channels channels);

        // The simulator's settings, read from this node's parameters.
        [[nodiscard]] Simulator::Config simulatorConfig() const;

        // Create the services. They call into the simulator, so this happens once it
        // exists. The simulator must outlive the node's callbacks.
        void connectServices(Simulator& simulator);

        // Publish one sensor tick: tf, odometry, ground truth and (if present) the scan.
        // Called from the publisher thread, not from an executor callback. That is
        // allowed: publish() is thread-safe.
        void publishTelemetry(const Telemetry& sample);

    private:
        void declareParameters();
        void publishStaticTransforms();
        void publishCamera();
        rcl_interfaces::msg::SetParametersResult onParametersChanged(const std::vector<rclcpp::Parameter>& parameters);

        Channels channels_;

        rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmdVelSub_;

        rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scanPub_;
        rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odomPub_;
        rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr truthPub_;
        rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr imagePub_;
        rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr cameraInfoPub_;
        std::unique_ptr<tf2_ros::TransformBroadcaster> tfBroadcaster_;
        std::unique_ptr<tf2_ros::StaticTransformBroadcaster> staticTfBroadcaster_;

        rclcpp::TimerBase::SharedPtr cameraTimer_;
        TimePoint lastCameraStamp_{};
        int jpegQuality_ = 80;

        rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr resetService_;
        rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr screenshotService_;
        rclcpp::Service<explorer_interfaces::srv::SpawnObstacle>::SharedPtr spawnService_;

        rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameterCallback_;
    };

}// namespace explorer_sim

#endif//EXPLORER_SIM_SIM_NODE_HPP
