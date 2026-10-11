// explorer_sim: the simulated world and robot.
//
// Three threads, and this file is where you can see all of them:
//
//   main thread        the simulator: render loop, physics, lidar, camera (owns OpenGL)
//   executor thread    ROS callbacks: cmd_vel, services, parameters, the camera timer
//   publisher thread   takes sensor ticks off a queue and publishes them
//
// Data crosses between them in exactly two ways, and in no other:
//
//   * through the four channel objects declared at the top of main(): three mailboxes
//     and one queue (drawn below), and
//   * through the simulator's request...() functions, which put a job in a queue for
//     the main thread and hand back a future (used by the service callbacks).
//
// All of these are locked on the inside, so none of the code that uses them needs a
// mutex of its own.
//
//                 Mailbox<VelocityCommand>
//   executor  ---------------------------->  simulator
//   thread        Mailbox<Tuning>            (main thread)
//             ---------------------------->
//                 Mailbox<CameraFramePtr>
//             <----------------------------
//
//                 ThreadSafeQueue<Telemetry>
//   publisher <----------------------------  simulator
//   thread

#include "sim_node.hpp"

#include "explorer_sim/mailbox.hpp"
#include "explorer_sim/simulator.hpp"
#include "explorer_sim/thread_safe_queue.hpp"
#include "explorer_sim/types.hpp"

#include "rclcpp/rclcpp.hpp"

#include <exception>
#include <memory>
#include <thread>

using namespace explorer_sim;

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    // The four channel objects. Declared first, so they are destroyed last: after
    // every thread that uses them has ended.
    Mailbox<VelocityCommand> commands;
    Mailbox<Tuning> tuning;
    Mailbox<CameraFramePtr> camera;
    ThreadSafeQueue<Telemetry> telemetry;

    auto node = std::make_shared<SimNode>(SimNode::Channels{commands, tuning, camera});
    Simulator simulator(node->simulatorConfig(), {commands, tuning, telemetry, camera});
    node->connectServices(simulator);

    // Ctrl-C makes ROS shut down. The render loop knows nothing about ROS, so tell it.
    rclcpp::on_shutdown([&simulator] { simulator.requestStop(); });

    // The executor gets its own thread, because the render loop below keeps the main
    // thread for itself until the window closes. From here on, every callback of the
    // node runs concurrently with the simulator.
    //
    // An exception that escapes a thread's function ends the whole program on the
    // spot (std::terminate). So each thread catches what its work may throw, says
    // what happened, and asks the simulator to stop: an orderly exit instead.
    std::jthread executor([node, &simulator] {
        try {
            rclcpp::spin(node);
        } catch (const std::exception& error) {
            RCLCPP_FATAL(node->get_logger(), "a ROS callback failed: %s", error.what());
            simulator.requestStop();
        }
    });

    // The publisher thread is the consumer loop from Part 2 of the book: sleep until
    // an item arrives, handle it, repeat, and end cleanly when the queue is closed.
    std::jthread publisher([&telemetry, node, &simulator] {
        try {
            while (const auto sample = telemetry.waitAndPop()) {
                node->publishTelemetry(*sample);
            }
        } catch (const std::exception& error) {
            RCLCPP_FATAL(node->get_logger(), "publishing failed: %s", error.what());
            simulator.requestStop();
        }
    });

    int exitCode = 0;
    try {
        simulator.run();// returns when the window is closed, or after Ctrl-C
    } catch (const std::exception& error) {
        RCLCPP_FATAL(node->get_logger(), "the simulator stopped with an error: %s", error.what());
        exitCode = 1;
    } catch (...) {
        // Whatever it was, the threads below must still be told to finish.
        RCLCPP_FATAL(node->get_logger(), "the simulator stopped with an unknown error");
        exitCode = 1;
    }

    // Shut down in the reverse order of starting up.
    telemetry.close();  // the publisher drains what is left, sees nullopt and returns
    publisher.join();
    rclcpp::shutdown(); // makes spin() return
    executor.join();
    return exitCode;
}
