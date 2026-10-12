#ifndef EXPLORER_SIM_OVERLAY_HPP
#define EXPLORER_SIM_OVERLAY_HPP

// What the simulator's window says in words: a status card, a legend, a strip of key
// hints, and the card that frames the robot's camera picture.
//
// This is the layout. The drawing tools it uses (rounded cards, text) are in hud.hpp.
// The frame loop only hands it the numbers (update) and the window's size (layout).

#include "hud.hpp"

#include <array>
#include <cstddef>
#include <string>
#include <vector>

namespace explorer_sim {

    class Overlay {
    public:
        // The numbers in the status card.
        struct Readouts {
            double speed = 0;     // m/s
            double headingDeg = 0;// degrees, counter-clockwise from +x
            double x = 0;         // m
            double y = 0;
            bool receivingCommands = false;// false: the watchdog has stopped the robot
            std::size_t droppedCrates = 0;
        };

        // A rectangle of the window, in pixels, measured from its bottom left corner.
        struct Rect {
            int x = 0, y = 0, width = 0, height = 0;
        };

        // pictureAspect: width / height of the robot's camera picture.
        // windowWidth: in pixels, as the window opens. It limits how large the cards are drawn.
        Overlay(float pictureAspect, int windowWidth);

        // Put everything in its place for a window of this size.
        void layout(int windowWidth, int windowHeight);

        void update(const Readouts& readouts);
        void showCameraName(const std::string& name);// "overview", "follow" or "chase"
        void showEffects(bool on);

        // Where in the window the robot's camera picture belongs (inside its card).
        // The picture is not part of this layer: the frame loop draws it there.
        [[nodiscard]] Rect cameraPicture() const { return picture_; }

        void draw(threepp::GLRenderer& renderer);

    private:
        // A readout: a small name above a value.
        struct Tile {
            Hud::Label* name = nullptr;
            Hud::Label* value = nullptr;
            float x = 0;
        };
        // A key cap and what the key does.
        struct Hint {
            threepp::Mesh* cap = nullptr;
            threepp::Mesh* capEdge = nullptr;
            Hud::Label* key = nullptr;
            Hud::Label* what = nullptr;
            float capWidth = 0;
            float slot = 0;// how much of the strip this hint takes
        };
        struct LegendRow {
            threepp::Mesh* dot = nullptr;
            Hud::Label* label = nullptr;
        };

        Tile makeTile(const char* name, float x);
        Hud::Label* addHint(const std::string& key, const std::string& what, const std::string& widest);

        Hud hud_;
        float s_;// the scale the cards are drawn at: the display's (1 at 100 %, 1.5 at 150 %), if the window has room
        float pictureAspect_;
        Rect picture_;

        threepp::Mesh* statusCard_ = nullptr;
        threepp::Mesh* statusEdge_ = nullptr;
        Hud::Label* kicker_ = nullptr;
        Hud::Label* headline_ = nullptr;
        Tile speed_, heading_, position_, command_, crates_;
        threepp::Mesh* commandLive_ = nullptr;
        threepp::Mesh* commandIdle_ = nullptr;

        threepp::Mesh* legendCard_ = nullptr;
        threepp::Mesh* legendEdge_ = nullptr;
        std::array<LegendRow, 3> legend_;

        threepp::Mesh* keysCard_ = nullptr;
        threepp::Mesh* keysEdge_ = nullptr;
        std::vector<Hint> hints_;
        Hud::Label* cameraHint_ = nullptr;
        Hud::Label* effectsHint_ = nullptr;

        threepp::Mesh* cameraCard_ = nullptr;
        threepp::Mesh* cameraEdge_ = nullptr;
        Hud::Label* cameraKicker_ = nullptr;
        Hud::Label* cameraTopic_ = nullptr;
    };

}// namespace explorer_sim

#endif//EXPLORER_SIM_OVERLAY_HPP
