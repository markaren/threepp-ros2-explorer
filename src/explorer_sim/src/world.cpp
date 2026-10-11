#include "explorer_sim/world.hpp"

#include <algorithm>
#include <cmath>

namespace explorer_sim {

    namespace {

        constexpr double wallThickness = 0.2;
        constexpr double wallHeight = 1.4;// low enough to look over from the 3D view

        // A wall along the y axis at a fixed x, from y0 to y1.
        Box wallAlongY(double x, double y0, double y1) {
            return {x, (y0 + y1) / 2, wallThickness, y1 - y0, wallHeight, Box::Kind::Wall};
        }

        // A wall along the x axis at a fixed y, from x0 to x1.
        Box wallAlongX(double y, double x0, double x1) {
            return {(x0 + x1) / 2, y, x1 - x0, wallThickness, wallHeight, Box::Kind::Wall};
        }

        // Push a disc out of one box. Returns true if they overlapped.
        bool pushOutOfBox(const Box& box, double& x, double& y, double radius) {
            const double halfX = box.sx / 2;
            const double halfY = box.sy / 2;

            // The point of the box closest to the disc's centre.
            const double nearestX = std::clamp(x, box.cx - halfX, box.cx + halfX);
            const double nearestY = std::clamp(y, box.cy - halfY, box.cy + halfY);
            const double dx = x - nearestX;
            const double dy = y - nearestY;
            const double distance = std::hypot(dx, dy);

            if (distance >= radius) {
                return false;// not touching
            }

            if (distance > 1e-9) {
                // The centre is outside the box: move straight away from the nearest point
                // until the disc only just touches it.
                const double push = radius - distance;
                x += dx / distance * push;
                y += dy / distance * push;
                return true;
            }

            // The centre is inside the box (it can happen when a crate is dropped very
            // close). Leave through the nearest face.
            const double toLeft = x - (box.cx - halfX);
            const double toRight = (box.cx + halfX) - x;
            const double toBottom = y - (box.cy - halfY);
            const double toTop = (box.cy + halfY) - y;
            const double smallest = std::min({toLeft, toRight, toBottom, toTop});
            if (smallest == toLeft) {
                x = box.cx - halfX - radius;
            } else if (smallest == toRight) {
                x = box.cx + halfX + radius;
            } else if (smallest == toBottom) {
                y = box.cy - halfY - radius;
            } else {
                y = box.cy + halfY + radius;
            }
            return true;
        }

        // Push a disc out of a cone (a smaller disc, seen from above).
        bool pushOutOfCone(const Cone& cone, double& x, double& y, double radius) {
            // At the height of the robot's chassis the cone is a little thinner than at
            // its base.
            const double coneRadius = Cone::baseRadius * 0.85;
            const double dx = x - cone.x;
            const double dy = y - cone.y;
            const double distance = std::hypot(dx, dy);
            const double minDistance = radius + coneRadius;
            if (distance >= minDistance || distance < 1e-9) {
                return false;
            }
            x = cone.x + dx / distance * minDistance;
            y = cone.y + dy / distance * minDistance;
            return true;
        }

    }// namespace

    World World::warehouse() {
        World w;
        using Kind = Box::Kind;

        // ---- outer walls -----------------------------------------------------------
        w.boxes.push_back(wallAlongX(w.minY, w.minX - 0.1, w.maxX + 0.1));
        w.boxes.push_back(wallAlongX(w.maxY, w.minX - 0.1, w.maxX + 0.1));
        w.boxes.push_back(wallAlongY(w.minX, w.minY, w.maxY));
        w.boxes.push_back(wallAlongY(w.maxX, w.minY, w.maxY));

        // ---- wall between the loading dock (x < 5) and the storage hall ------------
        // Two doors: a wide one straight ahead of the robot, a narrow one up north.
        w.boxes.push_back(wallAlongY(5, -8, -1));
        w.boxes.push_back(wallAlongY(5, 1, 5.5));
        w.boxes.push_back(wallAlongY(5, 7, 8));

        // ---- wall between the hall and the offices (x > 17) ------------------------
        w.boxes.push_back(wallAlongY(17, -8, -6.5));
        w.boxes.push_back(wallAlongY(17, -5, 2));
        w.boxes.push_back(wallAlongY(17, 3.5, 8));

        // ---- wall between the two offices, with a connecting door ------------------
        w.boxes.push_back(wallAlongX(0, 17, 20));
        w.boxes.push_back(wallAlongX(0, 21.5, 24));

        // ---- pallet racks in the hall ----------------------------------------------
        // Four rows (aisles A to D), each of two bays with a cross passage between
        // them. A 4 m wide main corridor runs between the inner rows.
        int rackCount = 0;
        for (const double y : {5.0, 2.4, -2.4, -5.0}) {
            for (const double x : {8.7, 13.3}) {
                // Two differently loaded models, alternated so the rows do not look cloned.
                const char* model = (rackCount % 3 == 1) ? "rack_b" : "rack_a";
                // Every other bay is turned half a turn, which shows its other side.
                w.boxes.push_back({x, y, 3.4, 0.8, 2.0, Kind::Rack, model, (rackCount % 2) * 2});
                ++rackCount;
            }
        }

        // ---- loading dock (the start position, the origin, is kept clear) ----------
        w.boxes.push_back({-0.8, 5.6, 1.0, 1.0, 0.9, Kind::Crate, "crate"});
        w.boxes.push_back({0.5, 6.7, 1.0, 1.0, 0.6, Kind::Crate, "crate"});
        w.boxes.push_back({1.9, -6.7, 0.8, 0.8, 0.5, Kind::Crate, "crate"});
        w.boxes.push_back({3.2, -5.8, 1.2, 0.8, 0.9, Kind::Prop, "pallet_a"});
        w.boxes.push_back({-1.0, -3.0, 0.8, 1.2, 0.9, Kind::Prop, "pallet_b", 1});
        w.boxes.push_back({-1.55, -7.55, 0.5, 0.5, 0.88, Kind::Prop, "barrel"});
        w.boxes.push_back({-0.95, -7.6, 0.5, 0.5, 0.88, Kind::Prop, "barrel"});
        w.boxes.push_back({-1.55, -6.95, 0.5, 0.5, 0.88, Kind::Prop, "barrel", 1});
        w.boxes.push_back({4.25, 3.3, 1.1, 1.6, 1.38, Kind::Prop, "cart"});
        w.boxes.push_back({-1.78, -4.9, 0.2, 0.66, 2.1, Kind::Prop, "ladder"});

        // ---- main corridor: things left standing in the way ------------------------
        w.boxes.push_back({9.0, 0.9, 0.8, 0.8, 0.7, Kind::Crate, "crate"});
        w.boxes.push_back({13.6, -0.8, 1.2, 0.8, 0.9, Kind::Prop, "pallet_a"});

        // ---- offices ---------------------------------------------------------------
        // The desk model is 0.95 m deep and 2 m wide, with its drawers on its +x side.
        // Each desk stands against a wall, turned so the drawers face the room.
        w.boxes.push_back({22.3, 7.35, 2.0, 0.95, 0.79, Kind::Desk, "desk", 3});
        w.boxes.push_back({19.3, 7.35, 2.0, 0.95, 0.79, Kind::Desk, "desk", 3});
        w.boxes.push_back({23.37, 2.6, 0.95, 2.0, 0.79, Kind::Desk, "desk", 2});
        w.boxes.push_back({18.4, -0.63, 2.0, 0.95, 0.79, Kind::Desk, "desk", 3});
        w.boxes.push_back({22.6, -7.37, 2.0, 0.95, 0.79, Kind::Desk, "desk", 1});
        w.boxes.push_back({22.3, 6.45, 0.47, 0.45, 0.46, Kind::Prop, "stool"});
        w.boxes.push_back({22.45, 2.4, 0.47, 0.45, 0.46, Kind::Prop, "stool"});
        w.boxes.push_back({22.6, -6.45, 0.47, 0.45, 0.46, Kind::Prop, "stool"});
        w.boxes.push_back({21.0, -4.2, 1.0, 1.0, 0.8, Kind::Crate, "crate"});
        w.boxes.push_back({17.65, 6.9, 0.8, 1.2, 0.9, Kind::Prop, "pallet_b", 1});

        // ---- traffic cones, for the camera to find ---------------------------------
        w.cones = {
                {2.4, 5.4},  // loading dock
                {9.0, 6.6},  // aisle A
                {15.5, -3.7},// where aisle D opens into the cross passage
                {16.1, 0.9}, // end of the main corridor
                {20.5, 5.0}, // north office
                {22.5, -3.0},// south office
        };

        // ---- signs -----------------------------------------------------------------
        // A sign's readable face looks along its yaw. Walls are 0.2 m thick, so a sign
        // on a wall sits 0.1 m from the wall's centre line.
        const double signZ = 1.12;
        w.decorations = {
                {"sign_exit", w.minX + 0.11, 0.0, signZ, 0},        // behind the robot's start
                {"sign_storage", 5 - 0.11, -2.4, signZ, 180},       // dock side, beside the wide door
                {"sign_dock", 5 + 0.11, 3.2, signZ, 0},             // hall side
                {"sign_office", 17 - 0.11, -1.5, signZ, 180},       // hall side, between the office doors
                // One letter on the end of each rack row, facing the dock.
                {"sign_aisle_A", 7.0 - 0.03, 5.0, 1.05, 180},
                {"sign_aisle_B", 7.0 - 0.03, 2.4, 1.05, 180},
                {"sign_aisle_C", 7.0 - 0.03, -2.4, 1.05, 180},
                {"sign_aisle_D", 7.0 - 0.03, -5.0, 1.05, 180},
                // Totes left on top of things. (On the floor they would be a trap: at
                // 25 cm they are lower than the lidar's scan plane, so the robot could
                // not see them. A real problem for real robots with a 2D lidar.)
                {"tote", 22.9, 7.3, 0.79, 20},
                {"tote", 18.0, -0.6, 0.79, -10},
                {"tote", -0.8, 5.6, 0.9, 35},
                {"tote", 21.0, -4.2, 0.8, 75},
        };

        // ---- floor markings --------------------------------------------------------
        // The edges of the main corridor ...
        w.floorLines.push_back({5.4, 1.75, 16.6, 1.75});
        w.floorLines.push_back({5.4, -1.75, 16.6, -1.75});
        // ... a line along the foot of every rack row ...
        //     (the inner rows share the corridor's edge lines)
        for (const double y : {5.6, 4.4, 3.0, -3.0, -4.4, -5.6}) {
            w.floorLines.push_back({6.8, y, 15.2, y, 0.05});
        }
        // ... and a square pad where the robot is parked.
        const double pad = 0.6;
        w.floorLines.push_back({-pad, -pad, pad, -pad});
        w.floorLines.push_back({pad, -pad, pad, pad});
        w.floorLines.push_back({pad, pad, -pad, pad});
        w.floorLines.push_back({-pad, pad, -pad, -pad});

        return w;
    }

    bool World::pushOut(double& x, double& y, double radius) const {
        bool touched = false;
        // Two passes: in a corner, leaving one box can push the disc into its neighbour.
        for (int pass = 0; pass < 2; ++pass) {
            for (const auto& box : boxes) {
                touched |= pushOutOfBox(box, x, y, radius);
            }
            for (const auto& box : dropped) {
                touched |= pushOutOfBox(box, x, y, radius);
            }
            for (const auto& cone : cones) {
                touched |= pushOutOfCone(cone, x, y, radius);
            }
        }
        return touched;
    }

    bool World::isFree(double x, double y, double radius) const {
        if (x - radius < minX || x + radius > maxX || y - radius < minY || y + radius > maxY) {
            return false;
        }
        double px = x;
        double py = y;
        return !pushOut(px, py, radius);
    }

}// namespace explorer_sim
