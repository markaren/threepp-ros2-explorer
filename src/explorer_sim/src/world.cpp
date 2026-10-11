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

        // ---- outer walls -----------------------------------------------------------
        w.boxes.push_back(wallAlongX(w.minY, w.minX - 0.1, w.maxX + 0.1));
        w.boxes.push_back(wallAlongX(w.maxY, w.minX - 0.1, w.maxX + 0.1));
        w.boxes.push_back(wallAlongY(w.minX, w.minY, w.maxY));
        w.boxes.push_back(wallAlongY(w.maxX, w.minY, w.maxY));

        // ---- wall between the loading dock (x < 5) and the hall --------------------
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

        // ---- shelves in the hall ---------------------------------------------------
        // Four rows, each split in two so there is a cross passage in the middle.
        // A 4 m wide main corridor runs between the inner rows.
        for (const double y : {-5.0, -2.4, 2.4, 5.0}) {
            w.boxes.push_back({8.7, y, 3.4, 0.8, 1.1, Box::Kind::Shelf});
            w.boxes.push_back({13.3, y, 3.4, 0.8, 1.1, Box::Kind::Shelf});
        }

        // ---- crates ----------------------------------------------------------------
        // Loading dock. The start position (0, 0) is kept clear.
        w.boxes.push_back({-0.8, 5.6, 1.0, 1.0, 0.9, Box::Kind::Crate});
        w.boxes.push_back({0.5, 6.7, 1.0, 1.0, 0.6, Box::Kind::Crate});
        w.boxes.push_back({3.2, -5.8, 1.2, 0.8, 0.8, Box::Kind::Crate});
        w.boxes.push_back({1.9, -6.7, 0.8, 0.8, 0.5, Box::Kind::Crate});
        w.boxes.push_back({-1.0, -3.0, 1.2, 1.0, 0.4, Box::Kind::Crate});
        // Main corridor: two crates left standing in the way.
        w.boxes.push_back({9.0, 0.9, 0.8, 0.8, 0.7, Box::Kind::Crate});
        w.boxes.push_back({13.6, -0.8, 0.9, 0.9, 0.9, Box::Kind::Crate});
        // South office.
        w.boxes.push_back({21.0, -4.2, 1.0, 1.0, 0.8, Box::Kind::Crate});

        // ---- desks in the offices --------------------------------------------------
        w.boxes.push_back({22.5, 6.6, 1.6, 0.8, 0.75, Box::Kind::Desk});
        w.boxes.push_back({19.0, 6.9, 1.6, 0.8, 0.75, Box::Kind::Desk});
        w.boxes.push_back({23.2, 2.6, 0.8, 1.6, 0.75, Box::Kind::Desk});
        w.boxes.push_back({18.3, -1.3, 1.4, 0.8, 0.75, Box::Kind::Desk});
        w.boxes.push_back({22.8, -6.6, 1.6, 0.8, 0.75, Box::Kind::Desk});

        // ---- traffic cones, for the camera to find ---------------------------------
        w.cones = {
                {3.6, 3.2},  // loading dock
                {9.0, 6.6},  // north aisle
                {12.5, -3.2},// between the southern shelf rows
                {16.1, 0.9}, // end of the main corridor
                {20.5, 5.0}, // north office
                {22.5, -3.0},// south office
        };

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
