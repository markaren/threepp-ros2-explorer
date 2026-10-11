#ifndef EXPLORER_SIM_SIMULATOR_IMPL_HPP
#define EXPLORER_SIM_SIMULATOR_IMPL_HPP

// The simulator's private parts. Nothing outside src/ includes this file, and nothing
// in it knows about ROS.
//
// The simulator is one class, Simulator::Impl, spread over three source files: one per
// question a reader might come with.
//
//   simulator.cpp   what happens?    physics, lidar, camera, requests from other threads
//   scene.cpp       what is drawn?   the warehouse: materials, models, lights, overlays
//   window.cpp      in what order?   the window and the frame loop, Simulator::run()
//
// This header declares the class, so that all three can add their part to it.

#include "explorer_sim/simulator.hpp"
#include "explorer_sim/world.hpp"

#include "threepp/canvas/Monitor.hpp"
#include "threepp/core/Raycaster.hpp"
#include "threepp/helpers/CameraSensor.hpp"
#include "threepp/loaders/FontLoader.hpp"
#include "threepp/loaders/GLTFLoader.hpp"
#include "threepp/loaders/RGBELoader.hpp"
#include "threepp/loaders/TextureLoader.hpp"
#include "threepp/objects/Reflector.hpp"
#include "threepp/objects/TextSprite.hpp"
#include "threepp/postprocessing/postprocessing.hpp"
#include "threepp/threepp.hpp"

#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <functional>
#include <limits>
#include <numbers>
#include <random>
#include <thread>
#include <vector>

namespace explorer_sim {

    // threepp's names (Scene, Mesh, Vector3, ...) are on almost every line of the three
    // files. This header is private to them, so the shortcut does not leak any further.
    using namespace threepp;

    inline constexpr double pi = std::numbers::pi;

    // Layer 1 holds everything that is drawn for the HUMAN and that the robot's own
    // camera must not see: lidar beams, the ring that marks the robot, the floor's
    // mirror image. The window's camera shows layers 0 and 1, the robot's camera only
    // layer 0.
    inline constexpr unsigned int debugLayer = 1;

    // The window shows the building on a plain dark backdrop, like a model on a
    // table. The robot's own camera sees the warehouse photo instead (see backdrop()).
    inline constexpr unsigned int windowBackground = 0x10141a;

    // ---- ROS frame <-> threepp frame ------------------------------------------------
    // ROS:     x forward, y left, z up.
    // threepp: x right,   y up,   z towards the viewer (like three.js).
    // Both are right-handed. We keep x, put ROS z on threepp y, and ROS y on threepp -z.
    // A yaw about ROS z is then the same angle about threepp y.
    inline Vector3 toThree(double x, double y, double z = 0) {
        return {static_cast<float>(x), static_cast<float>(z), static_cast<float>(-y)};
    }

    // =================================================================================

    struct Simulator::Impl {
        Config config;
        Channels channels;

        // ---- what other threads may touch ---------------------------------------------
        ThreadSafeQueue<std::function<void()>> jobs;// work posted to the simulator thread
        std::atomic<bool> stopRequested{false};

        // ---- everything below belongs to the simulator thread --------------------------
        World world = World::warehouse();

        // The robot's state.
        Pose2D truth;      // where it is
        Pose2D odom;       // where its wheel odometry thinks it is
        double linear = 0; // current speed, m/s
        double angular = 0;// current turn rate, rad/s
        bool commandFresh = false;

        Tuning tuning;
        std::mt19937 rng{42};// fixed seed: the same run gives the same noise

        // The lidar's ray caster, and what its rays are tested against.
        Raycaster raycaster;
        std::vector<Object3D*> lidarTargets;

        // ---- the scene (scene.cpp). Created in run(), on the thread that owns OpenGL ----
        std::shared_ptr<Scene> scene;
        std::shared_ptr<Texture> warehousePhoto;// the HDRI, when it was found
        std::shared_ptr<Material> crateMaterial;
        std::shared_ptr<Reflector> floorPolish; // the mirror image in the floor
        bool mirrorDue = false;                 // see buildFloorPolish()

        std::shared_ptr<Group> robotModel;
        std::shared_ptr<Mesh> robotMarker;                // a ring on the floor, so a human finds the robot
        std::shared_ptr<CameraSensor> cameraSensor;       // the robot's camera, as a sensor
        std::shared_ptr<PerspectiveCamera> robotEyeCamera;// the same view, for the inset in the window
        std::shared_ptr<Points> lidarPoints;              // a dot where each beam ends
        std::shared_ptr<LineSegments> lidarRays;          // the beams

        // Parts of the robot model that move (nullptr when the model has none).
        Object3D* wheelLeft = nullptr;
        Object3D* wheelRight = nullptr;
        Object3D* lidarHead = nullptr;
        double wheelLeftTurn = 0;// rad a wheel still has to be turned on screen
        double wheelRightTurn = 0;

        // Crates dropped while running: for each, the box the lidar hits and what is drawn.
        std::vector<std::shared_ptr<Mesh>> droppedMeshes;
        std::vector<std::shared_ptr<Object3D>> droppedVisuals;

        // A crate that has just been dropped and is still falling (for the eye only:
        // the robot's lidar sees it from the moment it is dropped).
        struct Falling {
            std::shared_ptr<Object3D> visual;
            double height = 0;
            double speed = 0;
        };
        std::vector<Falling> falling;

        // A screenshot request waiting for the next rendered frame.
        std::shared_ptr<std::promise<std::string>> pendingScreenshot;
        int screenshotCount = 0;

        Impl(Config c, Channels ch): config(std::move(c)), channels(ch) {}

        // Run `job` on the simulator thread and deliver its return value through a
        // future. This is the promise/future pair from Part 3 of the book: the caller
        // keeps the future, the job carries the promise across the thread border.
        template<typename Result>
        std::future<Result> post(std::function<Result()> job) {
            auto promise = std::make_shared<std::promise<Result>>();
            auto future = promise->get_future();
            jobs.push([promise, job = std::move(job)] {
                promise->set_value(job());
            });
            return future;
        }

        // ---- scene.cpp: what is drawn ---------------------------------------------------
        void buildScene();
        void buildFloorPolish(float sizeX, float sizeY, const Vector3& centre);
        void buildLamps();
        void buildLidarVisuals();
        void backdrop(bool warehouse);
        void addCrateVisual(const Box& crate);
        void removeDroppedVisuals();
        void updateRobotModel(double elapsed);
        void animateFalling(double elapsed);

        // ---- simulator.cpp: what happens ------------------------------------------------
        void step(double dt);
        LaserScan castLidar();
        void captureCamera(GLRenderer& renderer, TimePoint stamp);
        SpawnResult spawnCrate(double x, double y, double size);
        bool reset();
    };

}// namespace explorer_sim

#endif//EXPLORER_SIM_SIMULATOR_IMPL_HPP
