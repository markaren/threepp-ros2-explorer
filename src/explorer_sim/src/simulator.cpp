// What happens: the robot's physics, its two sensors, and the requests other threads
// can make. No ROS headers in this file, or in the other two that make up the simulator
// (scene.cpp: what is drawn, window.cpp: the frame loop that calls all of this).
//
// Reading order: step() is the physics, castLidar() and captureCamera() are the
// sensors. The functions at the bottom are the simulator's thread-safe public face.

#include "simulator_impl.hpp"

namespace explorer_sim {

    namespace {

        // Move `current` towards `target`, but by no more than maxStep.
        double approach(double current, double target, double maxStep) {
            return current + std::clamp(target - current, -maxStep, maxStep);
        }

    }// namespace

    // ---- physics --------------------------------------------------------------------

    // Advance the robot by one fixed timestep.
    void Simulator::Impl::step(double dt) {
        // 1. What does the controller want? A real robot must not keep driving on a
        //    stale command when its controller has crashed, so a command older than
        //    0.5 s counts as "stop".
        double targetLinear = 0;
        double targetAngular = 0;
        commandFresh = false;
        if (const auto command = channels.commands.latest()) {
            const auto age = std::chrono::steady_clock::now() - command->received;
            if (age < std::chrono::milliseconds(500)) {
                commandFresh = true;
                targetLinear = std::clamp(command->linear, -robot::maxLinear, robot::maxLinear);
                targetAngular = std::clamp(command->angular, -robot::maxAngular, robot::maxAngular);
            }
        }

        // 2. Motors cannot change speed instantly.
        linear = approach(linear, targetLinear, robot::linearAccel * dt);
        angular = approach(angular, targetAngular, robot::angularAccel * dt);

        // 3. Integrate the motion of a differential-drive robot.
        const double beforeX = truth.x;
        const double beforeY = truth.y;
        truth.x += linear * std::cos(truth.yaw) * dt;
        truth.y += linear * std::sin(truth.yaw) * dt;
        truth.yaw = std::remainder(truth.yaw + angular * dt, 2 * pi);

        // 4. Walls are solid.
        if (world.pushOut(truth.x, truth.y, robot::radius)) {
            // Do not let speed build up while pushing against something.
            linear *= 0.9;
        }

        // (For the eye only: how far each wheel turned. A wheel on the outside of a
        //  curve travels further than the one on the inside.)
        wheelLeftTurn += (linear - angular * robot::wheelSeparation / 2) * dt / robot::wheelRadius;
        wheelRightTurn += (linear + angular * robot::wheelSeparation / 2) * dt / robot::wheelRadius;

        // 5. Wheel odometry: the robot adds up how far it thinks it moved. With
        //    odomDrift = 0 that is exact. With drift, the odometry over-counts the
        //    distance a little and believes it turns slightly while driving
        //    straight, like a robot with one wheel a millimetre larger than the
        //    other. The error grows with every metre: that is why real robots need
        //    more than odometry.
        const double moved = (truth.x - beforeX) * std::cos(truth.yaw) + (truth.y - beforeY) * std::sin(truth.yaw);
        const double odomMoved = moved * (1.0 + 0.02 * tuning.odomDrift);
        const double odomTurned = angular * dt + 0.03 * tuning.odomDrift * moved;
        odom.x += odomMoved * std::cos(odom.yaw);
        odom.y += odomMoved * std::sin(odom.yaw);
        odom.yaw = std::remainder(odom.yaw + odomTurned, 2 * pi);
    }

    // ---- sensors --------------------------------------------------------------------

    // Cast the lidar: one ray per beam into the scene graph, exactly as the book
    // describes a simulated lidar. threepp's Raycaster does the ray/triangle work.
    LaserScan Simulator::Impl::castLidar() {
        LaserScan scan;
        const int beams = config.lidarBeams;
        scan.angleMin = -pi;
        scan.angleIncrement = 2 * pi / beams;
        scan.rangeMin = config.lidarRangeMin;
        scan.rangeMax = config.lidarRangeMax;
        scan.ranges.resize(static_cast<size_t>(beams));

        // The Raycaster works with world matrices. Make sure they are current: a
        // crate may have been added since the last frame was drawn.
        scene->updateMatrixWorld();

        const Vector3 origin = toThree(truth.x, truth.y, robot::laserHeight);
        raycaster.nearPlane = 0;
        raycaster.farPlane = static_cast<float>(scan.rangeMax);
        std::normal_distribution<double> noise(0.0, 1.0);

        auto* points = lidarPoints->geometry()->getAttribute<float>("position");
        auto* rays = lidarRays->geometry()->getAttribute<float>("position");

        for (int i = 0; i < beams; ++i) {
            const double beamAngle = truth.yaw + scan.angleMin + i * scan.angleIncrement;
            const double dirX = std::cos(beamAngle);
            const double dirY = std::sin(beamAngle);
            raycaster.set(origin, toThree(dirX, dirY, 0));

            double range = std::numeric_limits<double>::infinity();
            for (const auto& hit : raycaster.intersectObjects(lidarTargets)) {
                range = std::min(range, static_cast<double>(hit.distance));
            }

            if (std::isfinite(range)) {
                // A real lidar is never exact. The noise is what makes this sensor
                // worth testing against.
                range += noise(rng) * tuning.lidarNoise;
                range = std::clamp(range, scan.rangeMin, scan.rangeMax);
            }
            scan.ranges[static_cast<size_t>(i)] = static_cast<float>(range);

            // Where to draw this beam. A beam that hit nothing is drawn to full range.
            const double drawn = std::isfinite(range) ? range : scan.rangeMax;
            const Vector3 end = toThree(truth.x + dirX * drawn, truth.y + dirY * drawn, robot::laserHeight);
            const auto index = static_cast<size_t>(i);
            // A beam without a return has no dot: park it inside the lidar itself.
            const Vector3& dot = std::isfinite(range) ? end : origin;
            points->setXYZ(index, dot.x, dot.y, dot.z);
            // The beam is drawn from just outside the robot's body. It really starts
            // at the lidar, but 360 lines meeting in one point look like a glowing
            // blob on the robot, as if it were scanning itself. (It is not: the robot
            // is not among the lidar's targets.)
            const double clear = std::min(robot::radius + 0.06, drawn);
            const Vector3 start = toThree(truth.x + dirX * clear, truth.y + dirY * clear, robot::laserHeight);
            rays->setXYZ(index * 2, start.x, start.y, start.z);
            rays->setXYZ(index * 2 + 1, end.x, end.y, end.z);
        }
        points->needsUpdate();
        rays->needsUpdate();
        return scan;
    }

    // Render the robot's camera and hand the picture to the mailbox.
    void Simulator::Impl::captureCamera(GLRenderer& renderer, TimePoint stamp) {
        lookFrom(true);
        const bool captured = cameraSensor->capture(renderer, *scene);
        lookFrom(false);
        if (!captured) {
            return;
        }
        auto frame = std::make_shared<CameraFrame>();
        frame->stamp = stamp;
        frame->width = static_cast<int>(cameraSensor->width());
        frame->height = static_cast<int>(cameraSensor->height());
        frame->fovY = config.cameraFovYDeg * pi / 180.0;
        frame->rgb = cameraSensor->image();// one copy, made here on the render thread
        channels.camera.set(std::move(frame));
    }

    // ---- things other threads (or the mouse) can ask for ----------------------------

    SpawnResult Simulator::Impl::spawnCrate(double x, double y, double size) {
        if (size <= 0) size = 0.6;
        size = std::clamp(size, 0.2, 1.5);
        const double half = size / 2;

        if (x - half < world.minX + 0.1 || x + half > world.maxX - 0.1 ||
            y - half < world.minY + 0.1 || y + half > world.maxY - 0.1) {
            return {false, "that spot is outside the building"};
        }
        // Not on top of the robot: leave it at least a hand's width.
        const double reach = std::hypot(half, half) + robot::radius + 0.05;
        if (std::hypot(x - truth.x, y - truth.y) < reach) {
            return {false, "that spot is on top of the robot"};
        }
        // Not into a rack, a wall, a cone or another crate.
        if (!world.isFree(x, y, half)) {
            return {false, "something is already standing there"};
        }

        const Box crate{x, y, size, size, size, Box::Kind::Crate, "crate"};
        world.dropped.push_back(crate);// from now on the robot collides with it ...
        addCrateVisual(crate);         // ... the lidar hits it, and it is drawn

        char text[96];
        std::snprintf(text, sizeof(text), "crate %.1f m dropped at (%.2f, %.2f)", size, x, y);
        return {true, text};
    }

    bool Simulator::Impl::reset() {
        removeDroppedVisuals();
        world.dropped.clear();

        truth = {};
        odom = {};
        linear = 0;
        angular = 0;
        return true;
    }

    // =================================================================================
    // The public face. Everything a caller on another thread can reach.

    Simulator::Simulator(Config config, Channels channels)
        : pimpl_(std::make_unique<Impl>(std::move(config), channels)) {}

    Simulator::~Simulator() = default;

    void Simulator::requestStop() {
        pimpl_->stopRequested = true;
    }

    std::future<SpawnResult> Simulator::requestSpawn(double x, double y, double size) {
        Impl* impl = pimpl_.get();
        return impl->post<SpawnResult>([impl, x, y, size] { return impl->spawnCrate(x, y, size); });
    }

    std::future<bool> Simulator::requestReset() {
        Impl* impl = pimpl_.get();
        return impl->post<bool>([impl] { return impl->reset(); });
    }

    std::future<std::string> Simulator::requestScreenshot() {
        // This one cannot be answered inside the job: the picture only exists after
        // the frame has been drawn. The job just parks the promise where the render
        // loop will find it.
        Impl* impl = pimpl_.get();
        auto promise = std::make_shared<std::promise<std::string>>();
        auto future = promise->get_future();
        if (!impl->loopEnded) {
            impl->jobs.push([impl, promise] { impl->pendingScreenshot = promise; });
        }
        return future;
    }

}// namespace explorer_sim
