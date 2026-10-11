#ifndef EXPLORER_SIM_SIMULATOR_HPP
#define EXPLORER_SIM_SIMULATOR_HPP

// The simulator: a threepp scene with a robot in it, a fixed-timestep physics loop,
// and two simulated sensors (lidar and camera).
//
// This header contains no threepp types and no ROS types. The threepp side is hidden
// behind the Pimpl idiom (see "The Pimpl Idiom" in the book), so the ROS node can
// include this file without pulling a rendering engine into its translation unit.
//
// Threads. The simulator belongs to ONE thread: the one that calls run(). OpenGL is
// thread-affine, so that same thread renders, steps the physics, casts the lidar and
// reads the camera back. Other threads talk to it only through:
//
//   * the channels passed to the constructor (mailboxes and a queue), and
//   * the request...() functions below, which hand a job to the simulator thread and
//     return a std::future for its answer.

#include "explorer_sim/mailbox.hpp"
#include "explorer_sim/thread_safe_queue.hpp"
#include "explorer_sim/types.hpp"

#include <future>
#include <memory>
#include <string>

namespace explorer_sim {

    class Simulator {
    public:
        struct Config {
            bool headless = false;// true: no visible window (for tests and servers)
            int width = 1280;
            int height = 720;
            std::string view = "overview";// the camera to start with: overview, follow or chase
            bool effects = true;// ambient occlusion, bloom and floor reflections (P toggles)

            double odomRate = 30.0;  // odometry samples per second
            double sensorRate = 10.0;// lidar scans and camera frames per second

            int lidarBeams = 360;
            double lidarRangeMin = 0.12;
            double lidarRangeMax = 8.0;

            int cameraWidth = 640;
            int cameraHeight = 360;
            double cameraFovYDeg = 50.0;// vertical field of view

            std::string screenshotDir = ".";
            std::string assetDir;// folder with the textures. Empty = plain colours
        };

        // How the simulator reaches the outside world. It does not own these: whoever
        // creates the simulator creates them first and keeps them alive longer.
        struct Channels {
            Mailbox<VelocityCommand>& commands;   // in:  newest velocity command
            Mailbox<Tuning>& tuning;              // in:  newest noise settings
            ThreadSafeQueue<Telemetry>& telemetry;// out: every sensor tick
            Mailbox<CameraFramePtr>& camera;      // out: newest camera frame
        };

        Simulator(Config config, Channels channels);
        ~Simulator();

        Simulator(const Simulator&) = delete;
        Simulator& operator=(const Simulator&) = delete;

        // Open the window and run until it is closed or requestStop() is called.
        // Call this from the main thread. It does not return before then.
        void run();

        // ---- everything below is thread-safe: call it from any thread ---------------

        // Ask run() to return. Used when ROS shuts down first (Ctrl-C).
        void requestStop();

        // Drop a crate at (x, y). size is the side length in metres, 0 = default.
        std::future<SpawnResult> requestSpawn(double x, double y, double size);

        // Put the robot back at the start and remove all dropped crates.
        std::future<bool> requestReset();

        // Save the next rendered frame as a PNG. The future delivers the file's path.
        std::future<std::string> requestScreenshot();

    private:
        struct Impl;
        std::unique_ptr<Impl> pimpl_;
    };

}// namespace explorer_sim

#endif//EXPLORER_SIM_SIMULATOR_HPP
