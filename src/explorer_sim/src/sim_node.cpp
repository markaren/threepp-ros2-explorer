#include "sim_node.hpp"

#include "explorer_sim/world.hpp"

#include "ament_index_cpp/get_package_share_directory.hpp"

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include <chrono>
#include <cmath>
#include <optional>

using namespace std::chrono_literals;

namespace explorer_sim {

    namespace {

        // A std::chrono time point as a ROS time stamp. Both count nanoseconds since
        // 1970, so this is a change of type, not of value.
        rclcpp::Time toRosTime(TimePoint time) {
            const auto ns = std::chrono::duration_cast<std::chrono::nanoseconds>(time.time_since_epoch()).count();
            return rclcpp::Time(ns, RCL_ROS_TIME);
        }

        // A rotation about z as a quaternion. ROS messages store orientation as a
        // quaternion (x, y, z, w). For a pure yaw only z and w are non-zero.
        geometry_msgs::msg::Quaternion yawToQuaternion(double yaw) {
            geometry_msgs::msg::Quaternion q;
            q.z = std::sin(yaw / 2);
            q.w = std::cos(yaw / 2);
            return q;
        }

        // Wait for an answer from the simulator thread. std::nullopt if it does not
        // come: the simulator is shutting down, or its loop is stuck.
        template<typename T>
        std::optional<T> await(std::future<T> future) {
            if (future.wait_for(2s) != std::future_status::ready) {
                return std::nullopt;
            }
            try {
                return future.get();
            } catch (const std::future_error&) {
                // The simulator dropped the job without running it (broken promise).
                return std::nullopt;
            }
        }

        rcl_interfaces::msg::ParameterDescriptor describe(const std::string& text, bool readOnly = false) {
            rcl_interfaces::msg::ParameterDescriptor descriptor;
            descriptor.description = text;
            descriptor.read_only = readOnly;
            return descriptor;
        }

    }// namespace

    SimNode::SimNode(Channels channels)
        : Node("sim"), channels_(channels) {

        declareParameters();

        // ---- in: velocity commands -----------------------------------------------------
        // This callback runs on the executor thread. It does the one thing a callback
        // should do when another thread owns the data: put the newest value in the
        // mailbox and return. The simulator reads it when it is ready.
        cmdVelSub_ = create_subscription<geometry_msgs::msg::Twist>(
                "cmd_vel", 10,
                [this](const geometry_msgs::msg::Twist& msg) {
                    channels_.commands.set({msg.linear.x, msg.angular.z, std::chrono::steady_clock::now()});
                });

        // ---- out: sensors --------------------------------------------------------------
        // SensorDataQoS = best effort, keep only the last few. For a lidar or a camera a
        // lost message is replaced by a fresher one a moment later, and a slow subscriber
        // must never make the publisher queue up stale data. (Chapter 7, exercise 1.)
        scanPub_ = create_publisher<sensor_msgs::msg::LaserScan>("scan", rclcpp::SensorDataQoS());
        imagePub_ = create_publisher<sensor_msgs::msg::CompressedImage>("camera/image/compressed", rclcpp::SensorDataQoS());
        cameraInfoPub_ = create_publisher<sensor_msgs::msg::CameraInfo>("camera/camera_info", rclcpp::SensorDataQoS());

        // Odometry is small and consumers may want every sample: default QoS
        // (reliable, keep the last 10).
        odomPub_ = create_publisher<nav_msgs::msg::Odometry>("odom", 10);
        truthPub_ = create_publisher<geometry_msgs::msg::PoseStamped>("ground_truth", 10);

        tfBroadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
        staticTfBroadcaster_ = std::make_unique<tf2_ros::StaticTransformBroadcaster>(*this);
        publishStaticTransforms();

        // The camera publisher is the one from the book ("Camera & Detections"): a
        // timer on the executor thread takes the newest frame from the mailbox, encodes
        // it as JPEG and publishes it. The timer runs at twice the camera rate so a new
        // frame never waits long. Encoding costs a few milliseconds, and this way the
        // render loop does not pay for them.
        const double sensorRate = get_parameter("sensor_rate").as_double();
        cameraTimer_ = create_wall_timer(std::chrono::duration<double>(0.5 / sensorRate), [this] { publishCamera(); });

        // Some parameters can be changed while the node runs:
        //   ros2 param set /sim lidar_noise 0.05
        parameterCallback_ = add_on_set_parameters_callback(
                [this](const std::vector<rclcpp::Parameter>& parameters) { return onParametersChanged(parameters); });
        channels_.tuning.set({get_parameter("lidar_noise").as_double(), get_parameter("odom_drift").as_double()});
    }

    void SimNode::declareParameters() {
        // Declaring a parameter gives it a name, a default and a description
        // (`ros2 param describe /sim lidar_noise`). A launch file or the command line
        // can override the default. read_only = fixed once the node has started.
        declare_parameter("headless", false, describe("Run without a visible window", true));
        declare_parameter("width", 1280, describe("Window width in pixels", true));
        declare_parameter("height", 720, describe("Window height in pixels", true));
        declare_parameter("odom_rate", 30.0, describe("Odometry and tf samples per second", true));
        declare_parameter("sensor_rate", 10.0, describe("Lidar scans and camera frames per second", true));
        declare_parameter("lidar_beams", 360, describe("Beams per lidar revolution", true));
        declare_parameter("lidar_range", 8.0, describe("Maximum lidar range in metres", true));
        declare_parameter("camera_width", 640, describe("Camera image width in pixels", true));
        declare_parameter("camera_height", 360, describe("Camera image height in pixels", true));
        declare_parameter("camera_fov", 50.0, describe("Vertical field of view of the camera in degrees", true));
        declare_parameter("jpeg_quality", 80, describe("JPEG quality of the camera stream, 1 to 100", true));
        declare_parameter("screenshot_dir", ".", describe("Where the ~/screenshot service writes its PNG files", true));

        declare_parameter("lidar_noise", 0.01, describe("Standard deviation of the lidar range noise in metres. Can be changed while running."));
        declare_parameter("odom_drift", 0.0, describe("How much the wheel odometry lies. 0 = perfect, 1 = about 1.7 degrees of heading error per metre. Can be changed while running."));

        jpegQuality_ = static_cast<int>(get_parameter("jpeg_quality").as_int());
    }

    Simulator::Config SimNode::simulatorConfig() const {
        Simulator::Config config;
        config.headless = get_parameter("headless").as_bool();
        config.width = static_cast<int>(get_parameter("width").as_int());
        config.height = static_cast<int>(get_parameter("height").as_int());
        config.odomRate = get_parameter("odom_rate").as_double();
        config.sensorRate = get_parameter("sensor_rate").as_double();
        config.lidarBeams = static_cast<int>(get_parameter("lidar_beams").as_int());
        config.lidarRangeMax = get_parameter("lidar_range").as_double();
        config.cameraWidth = static_cast<int>(get_parameter("camera_width").as_int());
        config.cameraHeight = static_cast<int>(get_parameter("camera_height").as_int());
        config.cameraFovYDeg = get_parameter("camera_fov").as_double();
        config.screenshotDir = get_parameter("screenshot_dir").as_string();
        // Files a package installs (here: textures) live in its "share" directory. The
        // ament index knows where that is, wherever the workspace was built.
        config.assetDir = ament_index_cpp::get_package_share_directory("explorer_sim") + "/assets";
        return config;
    }

    rcl_interfaces::msg::SetParametersResult SimNode::onParametersChanged(const std::vector<rclcpp::Parameter>& parameters) {
        // ROS asks before it applies a change. Answering successful = false rejects it,
        // and the caller of `ros2 param set` sees the reason.
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;

        Tuning tuning = channels_.tuning.latest().value_or(Tuning{});
        for (const auto& parameter : parameters) {
            if (parameter.get_name() == "lidar_noise" || parameter.get_name() == "odom_drift") {
                const double value = parameter.as_double();
                if (value < 0.0 || value > 5.0) {
                    result.successful = false;
                    result.reason = parameter.get_name() + " must be between 0 and 5";
                    return result;
                }
                (parameter.get_name() == "lidar_noise" ? tuning.lidarNoise : tuning.odomDrift) = value;
            }
        }
        channels_.tuning.set(tuning);
        return result;
    }

    void SimNode::publishStaticTransforms() {
        // Where the sensors are bolted to the robot. These never change, so they are
        // published once on tf_static (which late joiners still receive).
        const auto stamp = now();

        geometry_msgs::msg::TransformStamped laser;
        laser.header.stamp = stamp;
        laser.header.frame_id = "base_link";
        laser.child_frame_id = "laser";
        laser.transform.translation.z = robot::laserHeight;
        laser.transform.rotation.w = 1.0;

        geometry_msgs::msg::TransformStamped camera;
        camera.header.stamp = stamp;
        camera.header.frame_id = "base_link";
        camera.child_frame_id = "camera_link";
        camera.transform.translation.x = robot::cameraForward;
        camera.transform.translation.z = robot::cameraHeight;
        camera.transform.rotation.w = 1.0;

        staticTfBroadcaster_->sendTransform({laser, camera});
    }

    void SimNode::publishTelemetry(const Telemetry& sample) {
        if (!rclcpp::ok()) {
            return;// ROS is shutting down
        }
        // Everything from one sensor tick carries the same stamp: the moment the
        // simulator sampled it, not the moment we got around to publishing it.
        const rclcpp::Time stamp = toRosTime(sample.stamp);

        // odom -> base_link, as a transform (for tf2) ...
        geometry_msgs::msg::TransformStamped transform;
        transform.header.stamp = stamp;
        transform.header.frame_id = "odom";
        transform.child_frame_id = "base_link";
        transform.transform.translation.x = sample.odom.x;
        transform.transform.translation.y = sample.odom.y;
        transform.transform.rotation = yawToQuaternion(sample.odom.yaw);
        tfBroadcaster_->sendTransform(transform);

        // ... and as an Odometry message, which also carries the velocity.
        nav_msgs::msg::Odometry odom;
        odom.header = transform.header;
        odom.child_frame_id = "base_link";
        odom.pose.pose.position.x = sample.odom.x;
        odom.pose.pose.position.y = sample.odom.y;
        odom.pose.pose.orientation = transform.transform.rotation;
        odom.twist.twist.linear.x = sample.linear;
        odom.twist.twist.angular.z = sample.angular;
        odomPub_->publish(odom);

        // Where the robot really is. A real robot cannot publish this. A simulator
        // can, and comparing it with odom shows how far the odometry has drifted.
        geometry_msgs::msg::PoseStamped truth;
        truth.header.stamp = stamp;
        truth.header.frame_id = "odom";
        truth.pose.position.x = sample.truth.x;
        truth.pose.position.y = sample.truth.y;
        truth.pose.orientation = yawToQuaternion(sample.truth.yaw);
        truthPub_->publish(truth);

        if (sample.scan) {
            sensor_msgs::msg::LaserScan scan;
            scan.header.stamp = stamp;
            scan.header.frame_id = "laser";
            scan.angle_min = static_cast<float>(sample.scan->angleMin);
            scan.angle_increment = static_cast<float>(sample.scan->angleIncrement);
            scan.angle_max = scan.angle_min + scan.angle_increment * static_cast<float>(sample.scan->ranges.size() - 1);
            scan.range_min = static_cast<float>(sample.scan->rangeMin);
            scan.range_max = static_cast<float>(sample.scan->rangeMax);
            scan.ranges = sample.scan->ranges;
            scanPub_->publish(scan);
        }
    }

    void SimNode::publishCamera() {
        const auto newest = channels_.camera.latest();
        if (!newest || !*newest) {
            return;// the simulator has not rendered a frame yet
        }
        const CameraFrame& frame = **newest;
        if (frame.stamp == lastCameraStamp_) {
            return;// a mailbox keeps returning the newest value: this one is already sent
        }
        lastCameraStamp_ = frame.stamp;

        sensor_msgs::msg::CompressedImage image;
        image.header.stamp = toRosTime(frame.stamp);// when it was rendered
        image.header.frame_id = "camera_link";
        image.format = "jpeg";

        // Wrap the pixels in a cv::Mat without copying them, convert RGB (what the
        // renderer produces) to BGR (what OpenCV expects), and let imencode write the
        // JPEG straight into the message.
        const cv::Mat rgb(frame.height, frame.width, CV_8UC3, const_cast<unsigned char*>(frame.rgb.data()));
        cv::Mat bgr;
        cv::cvtColor(rgb, bgr, cv::COLOR_RGB2BGR);
        cv::imencode(".jpg", bgr, image.data, {cv::IMWRITE_JPEG_QUALITY, jpegQuality_});
        imagePub_->publish(image);

        // The camera's intrinsics, so no subscriber has to hard-code a field of view.
        // Pinhole model: a point at (X, Y, Z) in front of the camera lands on pixel
        //   u = fx * X / Z + cx,   v = fy * Y / Z + cy.
        const double fy = (frame.height / 2.0) / std::tan(frame.fovY / 2.0);
        const double fx = fy;// square pixels
        const double cx = frame.width / 2.0;
        const double cy = frame.height / 2.0;

        sensor_msgs::msg::CameraInfo info;
        info.header = image.header;
        info.width = static_cast<uint32_t>(frame.width);
        info.height = static_cast<uint32_t>(frame.height);
        info.distortion_model = "plumb_bob";
        info.d = {0, 0, 0, 0, 0};// a rendered image has no lens distortion
        info.k = {fx, 0, cx, 0, fy, cy, 0, 0, 1};
        info.r = {1, 0, 0, 0, 1, 0, 0, 0, 1};
        info.p = {fx, 0, cx, 0, 0, fy, cy, 0, 0, 0, 1, 0};
        cameraInfoPub_->publish(info);
    }

    void SimNode::connectServices(Simulator& simulator) {
        // A service callback runs on the executor thread, but only the simulator
        // thread may touch the scene. So each callback posts a request, gets a
        // std::future back, and waits for the simulator to fulfil it (at most one
        // frame, about 16 ms). While it waits, this node's other callbacks wait too:
        // acceptable for a quick operation, and the reason long-running work is an
        // action and not a service.

        resetService_ = create_service<std_srvs::srv::Trigger>(
                "reset_world",
                [this, &simulator](const std_srvs::srv::Trigger::Request::SharedPtr,
                                   std_srvs::srv::Trigger::Response::SharedPtr response) {
                    const auto done = await(simulator.requestReset());
                    response->success = done.value_or(false);
                    response->message = response->success ? "robot at the start, dropped crates removed" : "the simulator did not answer";
                    RCLCPP_INFO(get_logger(), "reset_world: %s", response->message.c_str());
                });

        spawnService_ = create_service<explorer_interfaces::srv::SpawnObstacle>(
                "spawn_obstacle",
                [this, &simulator](const explorer_interfaces::srv::SpawnObstacle::Request::SharedPtr request,
                                   explorer_interfaces::srv::SpawnObstacle::Response::SharedPtr response) {
                    const auto result = await(simulator.requestSpawn(request->x, request->y, request->size));
                    response->success = result && result->success;
                    response->message = result ? result->message : "the simulator did not answer";
                    RCLCPP_INFO(get_logger(), "spawn_obstacle: %s", response->message.c_str());
                });

        // "~" is this node's own name, so the service is /sim/screenshot.
        screenshotService_ = create_service<std_srvs::srv::Trigger>(
                "~/screenshot",
                [&simulator](const std_srvs::srv::Trigger::Request::SharedPtr,
                             std_srvs::srv::Trigger::Response::SharedPtr response) {
                    const auto file = await(simulator.requestScreenshot());
                    response->success = file.has_value();
                    response->message = file.value_or("the simulator did not answer");
                });
    }

}// namespace explorer_sim
