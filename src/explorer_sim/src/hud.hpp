#ifndef EXPLORER_SIM_HUD_HPP
#define EXPLORER_SIM_HUD_HPP

// A small 2D layer drawn over the finished 3D picture: rounded cards, text, dots.
//
// It is an ordinary threepp scene, looked at through an orthographic camera whose
// units are pixels (x from the left edge, y from the BOTTOM edge), and drawn last,
// without clearing what is already there. Nothing in it is a picture of text: letters
// are triangles made from a font's outlines (Text2D), cards are rounded rectangles made
// from a Shape. So everything stays sharp at any size.
//
// The fonts are the system's (Segoe UI and Bahnschrift on Windows, DejaVu on Linux).
// Where none is found, threepp's built-in font is used.
//
// The Python side of this demo gets the same thing ready-made: threepp.lesson.Hud.

#include "threepp/extras/core/Font.hpp"
#include "threepp/objects/Text.hpp"
#include "threepp/threepp.hpp"

#include <array>
#include <memory>
#include <string>
#include <vector>

namespace explorer_sim {

    class Hud {
    public:
        enum class Face { Regular,
                          Semibold,
                          Numeric,// digits that all have the same width, so numbers do not jitter
                          Mono };

        // One line of text.
        class Label {
        public:
            // Turning a string into triangles costs time, so this does nothing when the
            // text is the one already shown.
            void set(const std::string& text);
            void setColor(const threepp::Color& color);
            void place(float x, float y);// where the left end of the baseline goes
            [[nodiscard]] float width() const;

        private:
            friend class Hud;
            Label(const threepp::Font& font, float size, const threepp::Color& color, const std::string& text);

            const threepp::Font* font_;
            float size_;
            std::string text_;
            std::shared_ptr<threepp::Text2D> mesh_;
        };

        Hud();

        // Each of these adds one thing to the layer and returns it, for the caller to
        // place (position = its bottom left corner), hide or change. What is added
        // later is drawn on top of what was added before.
        Label& text(const std::string& text, float size, const threepp::Color& color, Face face = Face::Regular);
        threepp::Mesh& panel(float width, float height, float radius, const threepp::Color& fill, float alpha);
        threepp::Mesh& frame(float width, float height, float radius, float thickness, const threepp::Color& color, float alpha);
        threepp::Mesh& dot(float radius, const threepp::Color& color);

        // How wide a text would be, without creating it (to size the card behind it).
        [[nodiscard]] float measure(const std::string& text, float size, Face face = Face::Regular) const;

        // Give a panel or a frame a new size (after the window was resized).
        static void reshapePanel(threepp::Mesh& panel, float width, float height, float radius);
        static void reshapeFrame(threepp::Mesh& frame, float width, float height, float radius, float thickness);

        void resize(int width, int height);
        void draw(threepp::GLRenderer& renderer);

    private:
        threepp::Mesh& add(const std::shared_ptr<threepp::BufferGeometry>& geometry, const threepp::Color& color, float alpha);

        threepp::Scene scene_;
        std::shared_ptr<threepp::OrthographicCamera> camera_;
        std::array<threepp::Font, 4> fonts_;
        std::vector<std::unique_ptr<Label>> labels_;
        int drawOrder_ = 0;
    };

}// namespace explorer_sim

#endif//EXPLORER_SIM_HUD_HPP
