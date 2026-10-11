#include "hud.hpp"

#include "threepp/loaders/FontLoader.hpp"

#include <cstdlib>
#include <filesystem>
#include <initializer_list>
#include <optional>

using namespace threepp;

namespace explorer_sim {

    namespace {

        // The first of `files` that exists in one of the system's font folders.
        std::optional<Font> loadSystemFont(std::initializer_list<const char*> files) {
            std::vector<std::filesystem::path> folders;
#ifdef _MSC_VER
#pragma warning(suppress : 4996)// getenv is fine here: read once, at start, on one thread
#endif
            if (const char* windows = std::getenv("WINDIR")) {
                folders.emplace_back(std::filesystem::path(windows) / "Fonts");
            }
            folders.emplace_back("/usr/share/fonts/truetype/dejavu");

            FontLoader loader;
            for (const char* file : files) {
                for (const auto& folder : folders) {
                    const auto path = folder / file;
                    if (std::filesystem::exists(path)) {
                        if (auto font = loader.load(path)) {
                            return font;
                        }
                    }
                }
            }
            return std::nullopt;
        }

        // The outline of a rectangle with rounded corners, its bottom left corner at
        // (x, y). Each corner is a quadratic curve whose control point is the corner
        // the rectangle would have had.
        void traceRoundedRect(Path& path, float x, float y, float width, float height, float radius) {
            radius = std::min(radius, std::min(width, height) / 2);
            path.moveTo(x + radius, y);
            path.lineTo(x + width - radius, y);
            path.quadraticCurveTo(x + width, y, x + width, y + radius);
            path.lineTo(x + width, y + height - radius);
            path.quadraticCurveTo(x + width, y + height, x + width - radius, y + height);
            path.lineTo(x + radius, y + height);
            path.quadraticCurveTo(x, y + height, x, y + height - radius);
            path.lineTo(x, y + radius);
            path.quadraticCurveTo(x, y, x + radius, y);
        }

        std::shared_ptr<BufferGeometry> panelGeometry(float width, float height, float radius) {
            Shape shape;
            traceRoundedRect(shape, 0, 0, width, height, radius);
            return ShapeGeometry::create(shape, 8);
        }

        // A frame is a rounded rectangle with a slightly smaller one cut out of it.
        std::shared_ptr<BufferGeometry> frameGeometry(float width, float height, float radius, float thickness) {
            Shape shape;
            traceRoundedRect(shape, 0, 0, width, height, radius);
            auto hole = std::make_shared<Path>();
            traceRoundedRect(*hole, thickness, thickness, width - 2 * thickness, height - 2 * thickness, std::max(radius - thickness, 0.f));
            shape.holes.push_back(hole);
            return ShapeGeometry::create(shape, 8);
        }

        // A flat colour that ignores the 3D picture's depth buffer (or a wall close to
        // the camera could hide the text) and may be see-through.
        std::shared_ptr<MeshBasicMaterial> flat(const Color& color, float alpha) {
            auto material = MeshBasicMaterial::create();
            material->color = color;
            material->opacity = alpha;
            material->transparent = true;
            material->depthTest = false;
            material->depthWrite = false;
            return material;
        }

    }// namespace

    // ---- Label ----------------------------------------------------------------------

    Hud::Label::Label(const Font& font, float size, const Color& color, const std::string& text)
        : font_(&font), size_(size), text_(text),
          mesh_(Text2D::create(TextGeometry::Options(font, size), text, flat(color, 1.f))) {}

    void Hud::Label::set(const std::string& text) {
        if (text != text_) {
            text_ = text;
            mesh_->setText(text);
        }
    }

    void Hud::Label::setColor(const Color& color) {
        mesh_->setColor(color);
    }

    void Hud::Label::place(float x, float y) {
        mesh_->position.set(x, y, 0);
    }

    float Hud::Label::width() const {
        return font_->advance(text_, size_);
    }

    // ---- Hud ------------------------------------------------------------------------

    Hud::Hud()
        : camera_(std::make_shared<OrthographicCamera>(0.f, 1.f, 1.f, 0.f, -10.f, 10.f)) {

        const Font fallback = FontLoader().defaultFont();
        fonts_[static_cast<size_t>(Face::Regular)] = loadSystemFont({"segoeui.ttf", "DejaVuSans.ttf"}).value_or(fallback);
        fonts_[static_cast<size_t>(Face::Semibold)] = loadSystemFont({"seguisb.ttf", "segoeuib.ttf", "DejaVuSans-Bold.ttf"}).value_or(fallback);
        fonts_[static_cast<size_t>(Face::Numeric)] = loadSystemFont({"bahnschrift.ttf", "DejaVuSansMono.ttf"}).value_or(fallback);
        fonts_[static_cast<size_t>(Face::Mono)] = loadSystemFont({"consola.ttf", "DejaVuSansMono.ttf"}).value_or(fallback);
    }

    Mesh& Hud::add(const std::shared_ptr<BufferGeometry>& geometry, const Color& color, float alpha) {
        auto mesh = Mesh::create(geometry, flat(color, alpha));
        // See-through things are drawn in renderOrder. Counting up gives "what was
        // added later is on top", which is how one thinks about a 2D layout.
        mesh->renderOrder = drawOrder_++;
        scene_.add(mesh);
        return *mesh;
    }

    Hud::Label& Hud::text(const std::string& text, float size, const Color& color, Face face) {
        labels_.push_back(std::unique_ptr<Label>(new Label(fonts_[static_cast<size_t>(face)], size, color, text)));
        Label& label = *labels_.back();
        label.mesh_->renderOrder = drawOrder_++;
        scene_.add(label.mesh_);
        return label;
    }

    float Hud::measure(const std::string& text, float size, Face face) const {
        return fonts_[static_cast<size_t>(face)].advance(text, size);
    }

    Mesh& Hud::panel(float width, float height, float radius, const Color& fill, float alpha) {
        return add(panelGeometry(width, height, radius), fill, alpha);
    }

    Mesh& Hud::frame(float width, float height, float radius, float thickness, const Color& color, float alpha) {
        return add(frameGeometry(width, height, radius, thickness), color, alpha);
    }

    Mesh& Hud::dot(float radius, const Color& color) {
        return add(CircleGeometry::create(radius, 24), color, 1.f);
    }

    void Hud::reshapePanel(Mesh& panel, float width, float height, float radius) {
        panel.setGeometry(panelGeometry(width, height, radius));
    }

    void Hud::reshapeFrame(Mesh& frame, float width, float height, float radius, float thickness) {
        frame.setGeometry(frameGeometry(width, height, radius, thickness));
    }

    void Hud::resize(int width, int height) {
        camera_->right = static_cast<float>(width);
        camera_->top = static_cast<float>(height);
        camera_->updateProjectionMatrix();
    }

    void Hud::draw(GLRenderer& renderer) {
        // autoClear off: draw over the picture instead of wiping it first.
        const bool autoClear = renderer.autoClear;
        renderer.autoClear = false;
        renderer.render(scene_, *camera_);
        renderer.autoClear = autoClear;
    }

}// namespace explorer_sim
