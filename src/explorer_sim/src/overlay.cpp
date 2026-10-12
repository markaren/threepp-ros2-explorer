#include "overlay.hpp"

#include "threepp/canvas/Monitor.hpp"

#include <algorithm>
#include <cstdio>

using namespace threepp;

namespace explorer_sim {

    namespace {

        // The palette: one dark card colour, two greys for text, one accent.
        const Color ink(0xf2f5fa);   // text that matters
        const Color dim(0x9fb2cc);   // names, notes
        const Color accent(0x19d3ff);// the one highlight colour
        const Color card(0x0d131e);
        const Color edge(0xffffff);
        constexpr float cardAlpha = 0.80f;

        // Sizes below are written in pixels at 100 % display scaling. Every one of them
        // is multiplied by one scale factor (s_), so the layer reads the same on a 4K
        // laptop screen and on a projector, as long as the window is wide enough.
        constexpr float statusW = 500, statusH = 110;
        constexpr float legendW = 216, legendH = 92;
        constexpr float keysH = 38, capH = 22;
        constexpr float margin = 16;
        // The cards are laid out for a window this wide at 100 %.
        constexpr float designW = 1280;

        // The display's scale factor, but no more than the window has room for. The
        // window's size is given in real pixels: on a display at 200 % a window 1280 wide
        // has room for the cards at 100 %, and one 2560 wide for the cards at 200 %.
        float scaleFor(int windowWidth) {
            const float display = std::max(1.f, monitor::contentScale().first);
            return std::clamp(static_cast<float>(windowWidth) / designW, 1.f, display);
        }

    }// namespace

    Overlay::Overlay(float pictureAspect, int windowWidth)
        : s_(scaleFor(windowWidth)), pictureAspect_(pictureAspect) {
        const float s = s_;

        // The order of the calls is the order of drawing: a card first, then what is on it.

        // ---- status card, top left: a headline and five readouts -----------------------
        statusCard_ = &hud_.panel(statusW * s, statusH * s, 13 * s, card, cardAlpha);
        statusEdge_ = &hud_.frame(statusW * s, statusH * s, 13 * s, 1.2f * s, edge, 0.10f);
        kicker_ = &hud_.text("GROUND TRUTH", 11.5f * s, accent, Hud::Face::Semibold);
        headline_ = &hud_.text("The world as it really is", 19 * s, ink, Hud::Face::Semibold);
        speed_ = makeTile("SPEED", 0);
        heading_ = makeTile("HEADING", 98);
        position_ = makeTile("POSITION", 190);
        command_ = makeTile("CMD_VEL", 312);
        crates_ = makeTile("CRATES", 420);
        // A green dot while velocity commands arrive, an amber one when they do not.
        commandLive_ = &hud_.dot(4 * s, Color(0x39d98a));
        commandIdle_ = &hud_.dot(4 * s, Color(0xffb020));

        // ---- legend card, top right ------------------------------------------------------
        // What the coloured overlays in the 3D picture mean. They are drawn for the
        // viewer only: the robot's own camera does not see them.
        legendCard_ = &hud_.panel(legendW * s, legendH * s, 13 * s, card, cardAlpha);
        legendEdge_ = &hud_.frame(legendW * s, legendH * s, 13 * s, 1.2f * s, edge, 0.10f);
        legend_ = {{
                {&hud_.dot(4.5f * s, Color(0xff2d75)), &hud_.text("where a lidar beam ends", 12.5f * s, ink)},
                {&hud_.dot(4.5f * s, Color(0x4aa9c4)), &hud_.text("the lidar's beams", 12.5f * s, ink)},
                {&hud_.dot(4.5f * s, accent), &hud_.text("ring that marks the robot", 12.5f * s, ink)},
        }};

        // ---- key strip, bottom left: every key as a little key cap ----------------------
        keysCard_ = &hud_.panel(10, 10, 12 * s, card, cardAlpha);// its width is known once the hints are
        keysEdge_ = &hud_.frame(10, 10, 12 * s, 1.2f * s, edge, 0.10f);
        addHint("Click", "drop a crate", "drop a crate");
        cameraHint_ = addHint("F", "camera: overview", "camera: overview");
        addHint("L", "lidar beams", "lidar beams");
        effectsHint_ = addHint("P", "effects: on", "effects: off");
        addHint("R", "reset", "reset");
        float keysW = 6 * s;
        for (const Hint& hint : hints_) keysW += hint.slot;
        Hud::reshapePanel(*keysCard_, keysW, keysH * s, 12 * s);
        Hud::reshapeFrame(*keysEdge_, keysW, keysH * s, 12 * s, 1.2f * s);

        // ---- camera card, bottom right ----------------------------------------------------
        // The mount and the caption of the robot's camera picture. Sized in layout().
        cameraCard_ = &hud_.panel(10, 10, 13 * s, card, 0.92f);
        cameraEdge_ = &hud_.frame(10, 10, 13 * s, 1.2f * s, edge, 0.10f);
        cameraKicker_ = &hud_.text("ROBOT CAMERA", 11.5f * s, accent, Hud::Face::Semibold);
        cameraTopic_ = &hud_.text("camera/image/compressed", 11.5f * s, dim, Hud::Face::Mono);
    }

    Overlay::Tile Overlay::makeTile(const char* name, float x) {
        return {&hud_.text(name, 10.5f * s_, dim, Hud::Face::Semibold),
                &hud_.text("", 17 * s_, ink, Hud::Face::Numeric),
                x * s_};
    }

    // `widest` is the longest text this hint will ever show, so the strip does not have
    // to be laid out again when the text changes.
    Hud::Label* Overlay::addHint(const std::string& key, const std::string& what, const std::string& widest) {
        const float s = s_;
        Hint hint;
        // The cap is created before the letter on it (drawing order), so its width is
        // measured, not taken from the label.
        hint.capWidth = hud_.measure(key, 11.5f * s, Hud::Face::Semibold) + 14 * s;
        hint.cap = &hud_.panel(hint.capWidth, capH * s, 5 * s, Color(0x2a3648), 1.f);
        hint.capEdge = &hud_.frame(hint.capWidth, capH * s, 5 * s, 1.f * s, edge, 0.22f);
        hint.key = &hud_.text(key, 11.5f * s, ink, Hud::Face::Semibold);
        hint.what = &hud_.text(what, 12.5f * s, dim);
        hint.slot = hint.capWidth + 7 * s + hud_.measure(widest, 12.5f * s) + 20 * s;
        hints_.push_back(hint);
        return hint.what;
    }

    // A text is placed by the left end of its baseline, a card by its bottom left
    // corner, and y counts up from the bottom of the window.
    void Overlay::layout(int windowWidth, int windowHeight) {
        hud_.resize(windowWidth, windowHeight);
        const float s = s_;
        const float m = margin * s;
        const auto width = static_cast<float>(windowWidth);
        const float top = static_cast<float>(windowHeight) - m;

        // Status card.
        statusCard_->position.set(m, top - statusH * s, 0);
        statusEdge_->position.copy(statusCard_->position);
        const float textLeft = m + 18 * s;
        kicker_->place(textLeft, top - 26 * s);
        headline_->place(textLeft, top - 50 * s);
        for (const Tile* tile : {&speed_, &heading_, &position_, &command_, &crates_}) {
            tile->name->place(textLeft + tile->x, top - 74 * s);
            tile->value->place(textLeft + tile->x, top - 95 * s);
        }
        command_.value->place(textLeft + command_.x + 14 * s, top - 95 * s);// room for the dot
        commandLive_->position.set(textLeft + command_.x + 4 * s, top - 89 * s, 0);
        commandIdle_->position.copy(commandLive_->position);

        // Legend card.
        const float legendLeft = width - m - legendW * s;
        legendCard_->position.set(legendLeft, top - legendH * s, 0);
        legendEdge_->position.copy(legendCard_->position);
        for (std::size_t i = 0; i < legend_.size(); ++i) {
            const float baseline = top - 29 * s - static_cast<float>(i) * 24 * s;
            legend_[i].dot->position.set(legendLeft + 20 * s, baseline + 4.5f * s, 0);
            legend_[i].label->place(legendLeft + 34 * s, baseline);
        }

        // Key strip.
        keysCard_->position.set(m, m, 0);
        keysEdge_->position.copy(keysCard_->position);
        float x = m + 12 * s;
        for (const Hint& hint : hints_) {
            hint.cap->position.set(x, m + (keysH - capH) * s / 2, 0);
            hint.capEdge->position.copy(hint.cap->position);
            hint.key->place(x + 7 * s, m + keysH * s / 2 - 4 * s);
            hint.what->place(x + hint.capWidth + 7 * s, m + keysH * s / 2 - 4.5f * s);
            x += hint.slot;
        }

        // Camera card: 27 % of the window's width, in the picture's own shape.
        picture_.width = std::max(240, windowWidth * 27 / 100);
        picture_.height = static_cast<int>(static_cast<float>(picture_.width) / pictureAspect_);
        const float pad = 10 * s, captionH = 26 * s;
        const float cardW = static_cast<float>(picture_.width) + 2 * pad;
        const float cardH = static_cast<float>(picture_.height) + 2 * pad + captionH;
        const float cardLeft = width - m - cardW;
        Hud::reshapePanel(*cameraCard_, cardW, cardH, 13 * s);
        Hud::reshapeFrame(*cameraEdge_, cardW, cardH, 13 * s, 1.2f * s);
        cameraCard_->position.set(cardLeft, m, 0);
        cameraEdge_->position.copy(cameraCard_->position);
        picture_.x = static_cast<int>(cardLeft + pad);
        picture_.y = static_cast<int>(m + pad);
        const float captionY = m + pad + static_cast<float>(picture_.height) + 9 * s;
        cameraKicker_->place(cardLeft + pad + 2 * s, captionY);
        cameraTopic_->place(cardLeft + cardW - pad - 2 * s - cameraTopic_->width(), captionY);
    }

    void Overlay::update(const Readouts& readouts) {
        char text[48];
        std::snprintf(text, sizeof(text), "%.2f m/s", readouts.speed);
        speed_.value->set(text);
        std::snprintf(text, sizeof(text), "%.0f deg", readouts.headingDeg);
        heading_.value->set(text);
        std::snprintf(text, sizeof(text), "%.1f, %.1f", readouts.x, readouts.y);
        position_.value->set(text);
        command_.value->set(readouts.receivingCommands ? "receiving" : "none");
        commandLive_->visible = readouts.receivingCommands;
        commandIdle_->visible = !readouts.receivingCommands;
        crates_.value->set(std::to_string(readouts.droppedCrates));
    }

    void Overlay::showCameraName(const std::string& name) {
        cameraHint_->set("camera: " + name);
    }

    void Overlay::showEffects(bool on) {
        effectsHint_->set(on ? "effects: on" : "effects: off");
    }

    void Overlay::draw(GLRenderer& renderer) {
        hud_.draw(renderer);
    }

}// namespace explorer_sim
