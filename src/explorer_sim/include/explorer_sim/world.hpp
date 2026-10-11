#ifndef EXPLORER_SIM_WORLD_HPP
#define EXPLORER_SIM_WORLD_HPP

// The building as data: where the walls, shelves, crates and cones are, and the one
// physical question the simulator asks of it: "the robot wants to be here, is that
// allowed?".
//
// No graphics in this file. The simulator turns these boxes into threepp meshes, but
// the collision test below works on the numbers alone, which also makes it easy to
// test without opening a window.
//
// ROS convention throughout: x forward, y left, z up, metres.

#include <vector>

namespace explorer_sim {

    // Sizes of the robot, shared by the simulator (to draw and collide it) and the ROS
    // node (to publish where the sensors are mounted).
    namespace robot {
        inline constexpr double radius = 0.22;       // the chassis is a disc
        inline constexpr double laserHeight = 0.30;  // lidar above the floor
        inline constexpr double cameraForward = 0.18;// camera ahead of the centre
        inline constexpr double cameraHeight = 0.25;
        inline constexpr double maxLinear = 0.8;     // m/s
        inline constexpr double maxAngular = 2.0;    // rad/s
        inline constexpr double linearAccel = 1.5;   // m/s^2
        inline constexpr double angularAccel = 5.0;  // rad/s^2
    }// namespace robot

    // An axis-aligned box standing on the floor.
    struct Box {
        enum class Kind { Wall,
                          Shelf,
                          Crate,
                          Desk };

        double cx = 0, cy = 0;// centre
        double sx = 1, sy = 1;// size along x and y
        double height = 1;
        Kind kind = Kind::Wall;
    };

    // A traffic cone. All cones have the same size.
    struct Cone {
        static constexpr double baseRadius = 0.20;
        static constexpr double height = 0.70;

        double x = 0, y = 0;
    };

    class World {
    public:
        // The outer walls sit on these lines.
        double minX = -2, maxX = 24;
        double minY = -8, maxY = 8;

        std::vector<Box> boxes;  // the building and its furniture: never changes
        std::vector<Box> dropped;// crates added while running (spawn_obstacle, mouse click)
        std::vector<Cone> cones;

        // The warehouse of the demo: a loading dock where the robot starts, a hall with
        // shelf aisles, and two offices at the far end.
        static World warehouse();

        // Move a disc of the given radius out of everything it overlaps. (x, y) is
        // changed in place. Returns true if it was touching something.
        //
        // This is all the "physics" the robot needs: it cannot drive through things,
        // and it slides along a wall it meets at an angle.
        bool pushOut(double& x, double& y, double radius) const;

        // True if a disc of the given radius at (x, y) is inside the building and
        // touches nothing. Used to refuse a crate dropped on top of something.
        [[nodiscard]] bool isFree(double x, double y, double radius) const;
    };

}// namespace explorer_sim

#endif//EXPLORER_SIM_WORLD_HPP
