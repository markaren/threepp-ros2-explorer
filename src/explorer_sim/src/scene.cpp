// What is drawn: the warehouse as a threepp scene.
//
// Nothing in this file decides how the robot moves or what its sensors measure. It
// turns the World (a list of boxes, see world.hpp) into something to look at: textured
// walls, 3D models, lamps, a polished floor, and the overlays that show a human what the
// lidar is doing. The one thing here that the sensors do depend on is the list of
// lidar targets, which is filled while the building is put together.
//
// Reading order: buildScene() is the table of contents. Everything above it is a helper
// it calls.

#include "simulator_impl.hpp"

namespace explorer_sim {

    namespace {

        std::shared_ptr<MeshStandardMaterial> makeMaterial(const Color& color, float roughness = 0.8f, float metalness = 0.0f) {
            auto material = MeshStandardMaterial::create();
            material->color = color;
            material->roughness = roughness;
            material->metalness = metalness;
            return material;
        }

        std::shared_ptr<MeshStandardMaterial> makeGlowing(const Color& color, float intensity = 1.5f) {
            auto material = makeMaterial(color, 0.4f);
            material->emissive = color;
            material->emissiveIntensity = intensity;
            return material;
        }

        // ---- textures ------------------------------------------------------------------
        // The surfaces are photo-scanned materials from polyhaven.com (CC0): a colour
        // map, a normal map (fine relief) and a roughness map (where it is shiny).
        // They are installed with the package, in share/explorer_sim/assets.

        std::shared_ptr<Texture> loadTexture(const std::filesystem::path& file, bool isColour) {
            if (!std::filesystem::exists(file)) {
                return nullptr;
            }
            TextureLoader loader;
            // Colour maps are stored in sRGB and must be decoded to linear light before
            // any lighting maths. Normal and roughness maps are data, not colours.
            auto texture = isColour ? loader.load(file, ColorSpace::sRGB) : loader.load(file);
            if (texture) {
                texture->wrapS = texture->wrapT = TextureWrapping::Repeat;
                texture->anisotropy = 8;// keeps the floor sharp at grazing angles
            }
            return texture;
        }

        // A material from one Poly Haven texture set. `tint` multiplies the colour map.
        // Without the files (dir empty or not found) it is a plain coloured material,
        // so the simulator still runs, just less pretty.
        std::shared_ptr<MeshStandardMaterial> makeSurface(const std::filesystem::path& dir, const std::string& name,
                                                          const std::string& colourFile, const Color& plainColour,
                                                          const Color& tint, float roughness, float metalness) {
            auto material = makeMaterial(plainColour, roughness, metalness);
            if (dir.empty()) {
                return material;
            }
            if (auto colour = loadTexture(dir / colourFile, true)) {
                material->map = colour;
                material->color = tint;
            }
            if (auto normal = loadTexture(dir / (name + "_nor_gl.jpg"), false)) {
                material->normalMap = normal;
            }
            if (auto rough = loadTexture(dir / (name + "_Rough.jpg"), false)) {
                material->roughnessMap = rough;
                material->roughness = 1.0f;// the map now decides
            }
            return material;
        }

        // A box whose texture repeats every `tile` metres on every face, whatever the
        // size of the box. A plain BoxGeometry stretches the texture once over each
        // face, which would make a 26 m wall and a 1 m crate look wrong side by side.
        std::shared_ptr<BufferGeometry> makeTiledBox(float width, float height, float depth, float tile) {
            auto geometry = BoxGeometry::create(width, height, depth);
            // BoxGeometry emits its six faces in the order +x, -x, +y, -y, +z, -z, four
            // vertices each. u and v run along these edges of the box:
            const float alongU[6] = {depth, depth, width, width, width, width};
            const float alongV[6] = {height, height, depth, depth, height, height};
            auto* uv = geometry->getAttribute<float>("uv");
            for (size_t face = 0; face < 6; ++face) {
                for (size_t corner = 0; corner < 4; ++corner) {
                    const size_t i = face * 4 + corner;
                    uv->setXY(i, uv->getX(i) * alongU[face] / tile, uv->getY(i) * alongV[face] / tile);
                }
            }
            return geometry;
        }

        // One box of the world as a mesh. The box stands on the floor, so its centre is
        // half its height up.
        std::shared_ptr<Mesh> makeBoxMesh(const Box& box, const std::shared_ptr<Material>& material, float tile) {
            // (width along x, height along y, depth along z) in threepp axes.
            auto geometry = makeTiledBox(static_cast<float>(box.sx), static_cast<float>(box.height), static_cast<float>(box.sy), tile);
            auto mesh = Mesh::create(geometry, material);
            mesh->position.copy(toThree(box.cx, box.cy, box.height / 2));
            mesh->castShadow = true;
            mesh->receiveShadow = true;
            return mesh;
        }

        // ---- 3D models -----------------------------------------------------------------
        // The racks, pallets, signs and the robot are glTF files in assets/models (made
        // in Blender, or taken from polyhaven.com: see the README there). They were
        // exported with x forward and y up, so they drop into the scene as they are.

        // Load assets/models/<name>.glb. nullptr if there is no such file, in which case
        // the caller draws something simpler.
        std::shared_ptr<Group> loadModel(const std::filesystem::path& assets, const std::string& name) {
            if (assets.empty() || name.empty()) {
                return nullptr;
            }
            const auto file = assets / "models" / (name + ".glb");
            if (!std::filesystem::exists(file)) {
                return nullptr;
            }
            GLTFLoader loader;
            auto result = loader.load(file);
            if (!result || !result->scene) {
                return nullptr;
            }
            result->scene->traverseType<Mesh>([](Mesh& mesh) {
                mesh.castShadow = true;
                mesh.receiveShadow = true;
            });
            return result->scene;
        }

        // Stand a model where a box of the world is: centred on the box's footprint,
        // on the floor, turned by the box's quarter turns and scaled to its size.
        std::shared_ptr<Object3D> fitModelToBox(const std::shared_ptr<Group>& model, const Box& box) {
            auto holder = Group::create();
            holder->position.copy(toThree(box.cx, box.cy, 0));
            holder->rotation.y = static_cast<float>(box.turns * pi / 2);

            // The box's footprint is given in world axes. After an odd number of quarter
            // turns the model's own x runs along the world's y.
            const bool sideways = box.turns % 2 != 0;
            const auto wantX = static_cast<float>(sideways ? box.sy : box.sx);
            const auto wantZ = static_cast<float>(sideways ? box.sx : box.sy);

            model->updateMatrixWorld(true);
            Box3 bounds;
            bounds.setFromObject(*model);
            const Vector3 size = bounds.getSize();
            const Vector3 centre = bounds.getCenter();

            const float scaleX = wantX / size.x;
            const float scaleZ = wantZ / size.z;
            // A crate is stretched to the height asked for. Anything else keeps its
            // proportions as well as the footprint allows.
            const float scaleY = box.kind == Box::Kind::Crate ? static_cast<float>(box.height) / size.y : (scaleX + scaleZ) / 2;

            model->scale.set(scaleX, scaleY, scaleZ);
            model->position.set(-centre.x * scaleX, -bounds.min().y * scaleY, -centre.z * scaleZ);
            holder->add(model);
            return holder;
        }

        // The robot built from primitives: the stand-in when assets/models/robot.glb is
        // missing. Its local frame is the ROS base_link frame converted to threepp axes:
        // +x is forward, +y is up.
        std::shared_ptr<Group> makeRobotModel() {
            auto robotGroup = Group::create();
            const auto r = static_cast<float>(robot::radius);

            const auto shell = makeMaterial(0xeef1f5, 0.35f, 0.1f);
            const auto dark = makeMaterial(0x232a33, 0.6f, 0.2f);
            const auto rubber = makeMaterial(0x14171b, 0.9f);
            const auto glow = makeGlowing(0x19d3ff);

            auto addPart = [&](const std::shared_ptr<BufferGeometry>& geometry, const std::shared_ptr<Material>& material, const Vector3& position) {
                auto mesh = Mesh::create(geometry, material);
                mesh->position.copy(position);
                mesh->castShadow = true;
                robotGroup->add(mesh);
                return mesh;
            };

            addPart(CylinderGeometry::create(r, r, 0.12f, 48), shell, {0, 0.10f, 0});              // chassis
            addPart(CylinderGeometry::create(r * 0.82f, r * 0.82f, 0.025f, 48), dark, {0, 0.172f, 0});// top deck
            addPart(BoxGeometry::create(0.16f, 0.012f, 0.05f), glow, {0.08f, 0.19f, 0});            // "this way is forward"

            // Drive wheels. A cylinder's axis is y, the axle must lie along z: turn it 90 degrees about x.
            for (const float side : {-1.f, 1.f}) {
                auto wheel = addPart(CylinderGeometry::create(0.07f, 0.07f, 0.04f, 24), rubber, {0, 0.07f, side * (r - 0.01f)});
                wheel->rotation.x = static_cast<float>(pi / 2);
            }

            // Lidar: a mast and a puck at the height the beams leave from.
            const auto laserY = static_cast<float>(robot::laserHeight);
            addPart(CylinderGeometry::create(0.02f, 0.02f, laserY - 0.19f, 12), dark, {0, (laserY + 0.19f) / 2 - 0.02f, 0});
            addPart(CylinderGeometry::create(0.055f, 0.055f, 0.05f, 32), dark, {0, laserY, 0});
            addPart(CylinderGeometry::create(0.057f, 0.057f, 0.008f, 32), glow, {0, laserY, 0});

            // Camera housing, just behind the point the camera looks out from.
            const auto camX = static_cast<float>(robot::cameraForward);
            const auto camY = static_cast<float>(robot::cameraHeight);
            addPart(BoxGeometry::create(0.02f, camY - 0.18f, 0.03f), dark, {camX - 0.04f, (camY + 0.18f) / 2, 0});
            addPart(BoxGeometry::create(0.05f, 0.05f, 0.10f), dark, {camX - 0.03f, camY, 0});

            return robotGroup;
        }

    }// namespace

    // =================================================================================

    void Simulator::Impl::buildScene() {
        scene = Scene::create();
        scene->background = Color(windowBackground);
        const std::filesystem::path assets = config.assetDir;

        // Image-based lighting: a 360 degree photo of a real warehouse (an HDRI)
        // lights the scene and shows up in reflections. The robot's camera also sees
        // it as the world beyond the walls (see backdrop()).
        const auto hdriFile = assets / "empty_warehouse_01_1k.hdr";
        if (!assets.empty() && std::filesystem::exists(hdriFile)) {
            if (auto hdri = RGBELoader().load(hdriFile)) {
                warehousePhoto = hdri;
                scene->environment = hdri;
                scene->environmentIntensity = 0.7f;
                scene->backgroundIntensity = 0.9f;
            }
        }

        // Light, in three layers, the way a film set is lit:
        //   fill   a dim sky/ground light and the HDRI, so no shadow is pitch black
        //   key    daylight from the roof windows: one directional light, the only
        //          one that casts shadows
        //   pools  the lamps hanging over the aisles (see buildLamps)
        scene->add(HemisphereLight::create(0xdfe9ff, 0x3a3f47, warehousePhoto ? 0.55f : 1.0f));

        // The building itself: floor, walls, ceiling. If the one made in Blender is
        // installed it is used, with its light already in it. Otherwise the plain
        // model below is built: a floor, and further down a box for every wall.
        bakedBuilding = buildBakedBuilding(assets);

        const double centreX = (world.minX + world.maxX) / 2;
        const auto sizeX = static_cast<float>(world.maxX - world.minX);
        const auto sizeY = static_cast<float>(world.maxY - world.minY);
        const Vector3 floorCentre = toThree(centreX, (world.minY + world.maxY) / 2, 0);
        if (!bakedBuilding) {
            auto sun = DirectionalLight::create(0xfff4e6, 1.7f);
            sun->position.copy(toThree(centreX - 13, -10, 17));
            auto sunTarget = Object3D::create();
            sunTarget->position.copy(toThree(centreX, 0, 0));
            scene->add(sunTarget);
            sun->setTarget(*sunTarget);
            sun->castShadow = true;
            sun->shadow->mapSize.set(4096, 4096);
            sun->shadow->bias = -0.0004f;
            sun->shadow->radius = 3.f;// soften the shadow's edge
            auto shadowCamera = sun->shadow->camera->as<OrthographicCamera>();
            shadowCamera->left = shadowCamera->bottom = -18;
            shadowCamera->right = shadowCamera->top = 18;
            shadowCamera->nearPlane = 1;
            shadowCamera->farPlane = 60;
            scene->add(sun);

            auto floorGeometry = PlaneGeometry::create(sizeX, sizeY);
            // Repeat the concrete every 4 m instead of stretching one photo over 26 m.
            auto* floorUv = floorGeometry->getAttribute<float>("uv");
            for (size_t i = 0; i < static_cast<size_t>(floorUv->count()); ++i) {
                floorUv->setXY(i, floorUv->getX(i) * sizeX / 4.f, floorUv->getY(i) * sizeY / 4.f);
            }
            auto floorMaterial = makeSurface(assets, "concrete_floor_worn_001", "concrete_floor_worn_001_Diffuse.jpg",
                                             0x70757c, Color(1.45f, 1.45f, 1.5f), 0.92f, 0.f);
            auto floor = Mesh::create(floorGeometry, floorMaterial);
            floor->rotation.x = static_cast<float>(-pi / 2);// a PlaneGeometry stands upright: lay it down
            floor->position.copy(floorCentre);
            floor->receiveShadow = true;
            scene->add(floor);
        }
        buildFloorPolish(sizeX, sizeY, floorCentre);
        buildLamps();

        // The building. One material per kind of thing.
        //
        // The colours are not only decoration: the cone detector looks for
        // saturated orange, so nothing else in the building may be orange.
        const auto wallMaterial = makeSurface(assets, "painted_plaster_wall", "painted_plaster_wall_Diffuse.jpg",
                                              0xe6eaef, Color(1.3f, 1.32f, 1.36f), 0.9f, 0.f);
        const auto shelfMaterial = makeSurface(assets, "blue_metal_plate", "blue_metal_plate_Diffuse.jpg",
                                               0x4c6a88, Color(1.7f, 1.9f, 2.2f), 0.7f, 0.55f);
        // The walls are cut off at 1.4 m so the camera can look in from above, like
        // an architect's model. A dark cap makes the cut read as a cut.
        const auto capMaterial = makeMaterial(0x15181d, 0.6f);
        const auto deskMaterial = makeMaterial(0xf5f7fa, 0.35f);
        // The planks are a greyscale version of Poly Haven's "weathered_planks": the
        // grain comes from the photo, the teal from the tint.
        crateMaterial = makeSurface(assets, "weathered_planks", "weathered_planks_grey.jpg",
                                    0x2a9d8f, 0x2fb3a3, 0.85f, 0.f);

        for (const auto& box : world.boxes) {
            std::shared_ptr<Material> material = wallMaterial;
            float tile = 2.5f;// metres of surface per repeat of the texture
            if (box.kind == Box::Kind::Rack) {
                material = shelfMaterial;
                tile = 1.7f;
            } else if (box.kind == Box::Kind::Crate || box.kind == Box::Kind::Prop) {
                material = crateMaterial;
                tile = 1.0f;
            } else if (box.kind == Box::Kind::Desk) {
                material = deskMaterial;
            }

            // Every box is a mesh: that is what the lidar's rays are tested against.
            auto proxy = makeBoxMesh(box, material, tile);
            scene->add(proxy);
            lidarTargets.push_back(proxy.get());

            if (bakedBuilding && (box.kind == Box::Kind::Wall || bakedProps)) {
                // What is LOOKED at here is in the building's own files, with its
                // light baked in. This box stays as what the lidar's rays hit.
                proxy->visible = false;
                continue;
            }

            if (box.kind == Box::Kind::Wall) {
                auto cap = Mesh::create(BoxGeometry::create(static_cast<float>(box.sx) + 0.02f, 0.03f, static_cast<float>(box.sy) + 0.02f), capMaterial);
                cap->position.copy(toThree(box.cx, box.cy, box.height + 0.015));
                scene->add(cap);
            }

            // If the box has a 3D model, the model is what is drawn and the box is
            // hidden. A hidden mesh is still hit by the Raycaster, which does not
            // care whether an object is drawn. So the lidar keeps seeing a simple
            // box, the eye sees a detailed rack.
            if (auto model = loadModel(assets, box.model)) {
                scene->add(fitModelToBox(model, box));
                proxy->visible = false;
            }
        }

        // Signs on the walls and on the rack ends. Nothing collides with them and
        // the lidar passes underneath, so they are only added to the scene.
        for (const auto& decoration : world.decorations) {
            if (auto model = loadModel(assets, decoration.model)) {
                model->position.copy(toThree(decoration.x, decoration.y, decoration.z));
                model->rotation.y = static_cast<float>(decoration.yawDeg * pi / 180);
                scene->add(model);
            }
        }

        // Painted lines on the floor: thin slabs, a few millimetres high.
        const auto paint = makeMaterial(0xe9edf2, 0.55f);
        for (const auto& line : world.floorLines) {
            if (bakedBuilding) break;// the lines are painted on the baked floor already
            const double dx = line.x1 - line.x0;
            const double dy = line.y1 - line.y0;
            auto slab = Mesh::create(BoxGeometry::create(static_cast<float>(std::hypot(dx, dy) + line.width), 0.004f, static_cast<float>(line.width)), paint);
            slab->position.copy(toThree((line.x0 + line.x1) / 2, (line.y0 + line.y1) / 2, 0.003));
            slab->rotation.y = static_cast<float>(std::atan2(dy, dx));
            slab->receiveShadow = true;
            scene->add(slab);
        }

        // Traffic cones. Their texture (tools/make_cone_texture.py) is orange plastic,
        // RGB (255, 106, 0), with one white reflective band and some scuffs. A little
        // self-illumination, through the same texture, keeps them orange in the shadow
        // of a rack. Each stands on a rubber foot, which is too low for the lidar and
        // so is not one of its targets.
        auto coneMaterial = makeMaterial(0xff6a00, 0.55f);
        if (auto coneTexture = loadTexture(assets / "cone.png", true)) {
            coneMaterial->map = coneTexture;
            coneMaterial->color = Color(0xffffff);// the texture brings the colour
            coneMaterial->emissiveMap = coneTexture;
            coneMaterial->emissive = Color(0xffffff);
        } else {
            coneMaterial->emissive = Color(0xff6a00);
        }
        coneMaterial->emissiveIntensity = 0.22f;
        auto coneGeometry = ConeGeometry::create(static_cast<float>(Cone::baseRadius), static_cast<float>(Cone::height), 32);
        auto footGeometry = CylinderGeometry::create(0.23f, 0.25f, 0.02f, 32);
        const auto footMaterial = makeMaterial(0x1d2026, 0.9f);
        for (const auto& cone : world.cones) {
            auto mesh = Mesh::create(coneGeometry, coneMaterial);
            mesh->position.copy(toThree(cone.x, cone.y, Cone::height / 2));
            mesh->castShadow = true;
            scene->add(mesh);
            lidarTargets.push_back(mesh.get());

            auto foot = Mesh::create(footGeometry, footMaterial);
            foot->position.copy(toThree(cone.x, cone.y, 0.01));
            foot->receiveShadow = true;
            scene->add(foot);
        }

        // The robot. Not a lidar target: it must not see itself.
        robotModel = Group::create();
        if (auto model = loadModel(assets, "robot")) {
            // The model names the parts that move, so they can be found here.
            wheelLeft = model->getObjectByName<Object3D>("wheel_left");
            wheelRight = model->getObjectByName<Object3D>("wheel_right");
            lidarHead = model->getObjectByName<Object3D>("lidar_head");
            robotModel->add(model);
        } else {
            robotModel->add(makeRobotModel());
        }
        scene->add(robotModel);

        // A glowing ring on the floor around the robot. The robot is 44 cm across in
        // a 26 m building: without this a human loses it in the overview.
        auto ringMaterial = MeshBasicMaterial::create();
        // A colour brighter than white (components above 1). On screen it is still
        // cyan, but the bloom pass spreads anything that bright into a glow.
        ringMaterial->color = Color(0.2f, 1.9f, 2.4f);
        ringMaterial->transparent = true;
        ringMaterial->opacity = 0.8f;
        robotMarker = Mesh::create(RingGeometry::create(0.40f, 0.45f, 64), ringMaterial);
        robotMarker->rotation.x = static_cast<float>(-pi / 2);
        robotMarker->layers.set(debugLayer);// for the human only
        scene->add(robotMarker);
        // The same ring once more, faint, and drawn whatever stands in front of it: in
        // the overview the robot is then still found when it is behind a rack.
        auto ghostMaterial = MeshBasicMaterial::create();
        ghostMaterial->color = Color(0.2f, 1.9f, 2.4f);
        ghostMaterial->transparent = true;
        ghostMaterial->opacity = 0.22f;
        ghostMaterial->depthTest = false;
        auto ghost = Mesh::create(RingGeometry::create(0.40f, 0.45f, 64), ghostMaterial);
        ghost->layers.set(debugLayer);
        ghost->renderOrder = 10;
        robotMarker->add(ghost);

        // The robot's camera: threepp's CameraSensor renders the scene from its own
        // pose into an off-screen target and reads the pixels back.
        //
        // A threepp camera looks along its local -z. The robot's forward is +x, so the
        // sensor is turned -90 degrees about the vertical axis.
        cameraSensor = std::make_shared<CameraSensor>(
                static_cast<float>(config.cameraFovYDeg),
                static_cast<unsigned int>(config.cameraWidth),
                static_cast<unsigned int>(config.cameraHeight),
                0.05f, 40.f);
        cameraSensor->position.set(static_cast<float>(robot::cameraForward), static_cast<float>(robot::cameraHeight), 0);
        cameraSensor->rotation.y = static_cast<float>(-pi / 2);
        robotModel->add(cameraSensor);

        // A second camera at the same place, for the "what the robot sees" inset in
        // the window. (The CameraSensor's picture goes to ROS as pixels. This one
        // draws straight into a corner of the window.)
        robotEyeCamera = PerspectiveCamera::create(static_cast<float>(config.cameraFovYDeg),
                                                   static_cast<float>(config.cameraWidth) / static_cast<float>(config.cameraHeight),
                                                   0.05f, 40.f);
        robotEyeCamera->position.copy(cameraSensor->position);
        robotEyeCamera->rotation.y = static_cast<float>(-pi / 2);
        robotModel->add(robotEyeCamera);

        buildLidarVisuals();
    }

    // The building as Blender made it (tools/blender/make_warehouse.py). Three kinds of
    // file, in assets/warehouse:
    //
    //   building.glb    floor, walls, ceiling, beams, doors and lamps
    //   props.glb       everything that stands in the building and never moves, at its place
    //   lm_<mesh>.jpg   one lightmap for each mesh of the building
    //   baked_<mesh>.jpg  one picture for each mesh of the props: colour and light in one
    //
    // A lightmap is a picture of how much light falls on every spot of a surface.
    // Blender's path tracer worked it out once, including the light that bounces
    // between floor, walls and ceiling and the soft shadows under everything that
    // stands in the building. That is the light that makes a room look like a room,
    // and no real-time lamp can produce it. Here a surface's colour is only multiplied
    // with it, which is the cheapest material there is: no lamp is evaluated per pixel.
    //
    // Each mesh has two sets of texture coordinates for this: the first lays the
    // colour texture on, the second gives every spot of the mesh a place of its own
    // in the lightmap.
    //
    // Returns false when the files are not installed. The caller then builds the
    // plain model.
    bool Simulator::Impl::buildBakedBuilding(const std::filesystem::path& assets) {
        const auto dir = assets / "warehouse";
        if (assets.empty() || !std::filesystem::exists(dir / "building.glb")) {
            return false;
        }
        auto building = GLTFLoader().load(dir / "building.glb");
        if (!building || !building->scene) {
            return false;
        }

        // materials.txt: what each material of the building is. The building's file
        // has only their names. One per line: "lit" (colour times lightmap) or "flat"
        // (one colour: the lamps, which shine by themselves), the name, a colour
        // texture from the assets folder or "-", and a tint.
        struct Surface {
            bool lit = true;
            std::string texture;
            Color tint{0.6f, 0.6f, 0.6f};
        };
        std::unordered_map<std::string, Surface> surfaces;
        // The lightmaps are stored darker by this factor, so that the light right
        // under a lamp (several times brighter than "white") fits into 8 bits.
        float lightmapScale = 1.f;
        {
            std::ifstream list(dir / "materials.txt");
            std::string kind, name, texture;
            float r = 0, g = 0, b = 0;
            while (list >> kind) {
                if (kind == "lightmap_scale") {
                    list >> lightmapScale;
                } else if (list >> name >> texture >> r >> g >> b) {
                    surfaces[name] = {kind == "lit", texture == "-" ? "" : texture, Color(r, g, b)};
                }
            }
        }

        TextureLoader textures;
        std::unordered_map<std::string, std::shared_ptr<Texture>> pictures;

        // The lightmap of the mesh called `object`: assets/warehouse/lm_<object>.jpg.
        auto lightmapOf = [&](const std::string& object) {
            auto& lightmap = pictures["lm_" + object];
            if (!lightmap) {
                // flipY false: like every picture that belongs to a glTF file, its
                // first row is the top of the texture.
                lightmap = textures.load(dir / ("lm_" + object + ".jpg"), ColorSpace::sRGB, false);
                lightmap->channel = 1;// the second set of texture coordinates
            }
            return lightmap;
        };
        // The loader calls the meshes of an object "walls_low_0", "walls_low_1", ...:
        // one per material. Cut the number off to get the object's own name.
        auto objectOf = [](const std::string& meshName) {
            const auto underscore = meshName.find_last_of('_');
            const bool numbered = underscore != std::string::npos && underscore + 1 < meshName.size() &&
                                  std::all_of(meshName.begin() + static_cast<std::ptrdiff_t>(underscore) + 1, meshName.end(),
                                              [](unsigned char c) { return std::isdigit(c) != 0; });
            return numbered ? meshName.substr(0, underscore) : meshName;
        };

        std::unordered_map<std::string, std::shared_ptr<Material>> made;// one per (object, material)
        building->scene->traverseType<Mesh>([&](Mesh& mesh) {
            const std::string object = objectOf(mesh.name);
            const std::string name = mesh.material()->name;
            if (object == "walls_high" || object == "ceiling") {
                upperBuilding.push_back(&mesh);
            }
            auto& material = made[object + "/" + name];
            if (!material) {
                const Surface surface = surfaces.contains(name) ? surfaces.at(name) : Surface{};
                auto basic = MeshBasicMaterial::create();
                basic->color = surface.tint;
                if (surface.lit) {
                    if (!surface.texture.empty()) {
                        auto& colour = pictures[surface.texture];
                        if (!colour) {
                            colour = textures.load(assets / surface.texture, ColorSpace::sRGB, false);
                            colour->wrapS = colour->wrapT = TextureWrapping::Repeat;
                            colour->anisotropy = 8;
                        }
                        basic->map = colour;
                    }
                    basic->lightMap = lightmapOf(object);
                    basic->lightMapIntensity = lightmapScale;
                }
                material = basic;
            }
            mesh.setMaterial(material);
        });
        scene->add(building->scene);

        // The props have one picture per room, and it holds everything: their own
        // colours, worn and dusty the way Blender made them, times the light that falls
        // on them. Nothing is left to compute but to show it.
        if (std::filesystem::exists(dir / "props.glb")) {
            if (auto props = GLTFLoader().load(dir / "props.glb"); props && props->scene) {
                std::unordered_map<std::string, std::shared_ptr<Material>> baked;// one per room
                props->scene->traverseType<Mesh>([&](Mesh& mesh) {
                    const std::string object = objectOf(mesh.name);
                    auto& material = baked[object];
                    if (!material) {
                        auto basic = MeshBasicMaterial::create();
                        basic->map = textures.load(dir / ("baked_" + object + ".jpg"), ColorSpace::sRGB, false);
                        basic->map->anisotropy = 8;
                        basic->color = Color(lightmapScale, lightmapScale, lightmapScale);
                        material = basic;
                    }
                    mesh.setMaterial(material);
                });
                scene->add(props->scene);
                bakedProps = true;
            }
        }
        return true;
    }

    // A polished floor shows a faint mirror image of what stands on it. threepp's
    // Reflector draws the scene a second time from a mirrored camera into a texture.
    // Here that texture is not shown as a perfect mirror: a small shader adds it on
    // top of the concrete, weak when looking straight down and strong at a grazing
    // angle (the Fresnel effect: look along any shiny floor and it turns into a
    // mirror). This is the recipe of threepp's MarbleRun example.
    void Simulator::Impl::buildFloorPolish(float sizeX, float sizeY, const Vector3& centre) {
        Reflector::Options options;
        options.color = Color(1.f, 1.f, 1.f);
        options.textureWidth = static_cast<unsigned int>(config.width);
        options.textureHeight = static_cast<unsigned int>(config.height);
        options.shader = Shader{
                UniformMap{
                        {"color", Uniform()},
                        {"tDiffuse", Uniform()},
                        {"textureMatrix", Uniform()}},

                R"(
                uniform mat4 textureMatrix;
                varying vec4 vUv;
                varying vec3 vWorld;

                void main() {
                    vUv = textureMatrix * vec4( position, 1.0 );
                    vec4 world = modelMatrix * vec4( position, 1.0 );
                    vWorld = world.xyz;
                    gl_Position = projectionMatrix * viewMatrix * world;
                })",

                R"(
                uniform vec3 color;
                uniform sampler2D tDiffuse;
                varying vec4 vUv;
                varying vec3 vWorld;

                void main() {
                    // The mirror image of a lamp is as bright as the lamp. Capped at
                    // white, or the bloom pass would turn it into a patch of glare.
                    vec3 reflection = min( texture2DProj( tDiffuse, vUv ).rgb, vec3( 1.0 ) );
                    vec3 toEye = normalize( cameraPosition - vWorld );
                    float fresnel = 0.03 + 0.34 * pow( 1.0 - max( toEye.y, 0.0 ), 4.0 );
                    gl_FragColor = vec4( reflection * color * fresnel, 1.0 );
                    #include <tonemapping_fragment>
                    #include <encodings_fragment>
                })"};

        floorPolish = Reflector::create(PlaneGeometry::create(sizeX, sizeY), options);
        floorPolish->rotation.x = static_cast<float>(-pi / 2);
        floorPolish->position.copy(centre);
        floorPolish->position.y = 0.002f;
        floorPolish->material()->transparent = true;
        floorPolish->material()->blending = Blending::Additive;
        floorPolish->material()->depthWrite = false;
        // Only the window's camera looks at layer 1, so the robot's own camera does
        // not pay for a mirror image it has no use for.
        floorPolish->layers.set(debugLayer);
        scene->add(floorPolish);

        // The Reflector redraws its mirror image every time it is about to be drawn.
        // With post-processing the scene is drawn several times per frame (colour,
        // then depth and normals for the ambient occlusion), and one mirror image
        // per frame is enough. So its callback is wrapped: it only runs when the
        // render loop has set mirrorDue.
        floorPolish->onBeforeRender = RenderCallback(
                [this, mirror = *floorPolish->onBeforeRender](void* r, Object3D* object, Camera* camera, BufferGeometry* geometry,
                                                              Material* material, std::optional<GeometryGroup> group) {
                    if (!mirrorDue) return;
                    mirrorDue = false;
                    mirror(r, object, camera, geometry, material, group);
                });
    }

    // The lamps: a row of LED battens hanging over every aisle. Each is a glowing
    // bar (for the eye, and for the bloom) and a spot light pointing straight down
    // (for the pool of light on the floor).
    void Simulator::Impl::buildLamps() {
        struct Lamp {
            double x, y;
            bool alongX;
        };
        const Lamp lamps[] = {
                {1.5, 4.2, true},   // loading dock
                {1.5, -4.2, true},
                {8.7, 0.0, true},   // main corridor
                {13.3, 0.0, true},
                {11.0, 3.7, true},  // aisles between the racks
                {11.0, -3.7, true},
                {11.0, 6.8, true},
                {11.0, -6.8, true},
                {20.5, 4.0, false}, // offices
                {20.5, -4.0, false},
        };
        const double lampHeight = 3.0;

        auto barMaterial = makeGlowing(0xfff4e0, 2.2f);
        auto barGeometry = BoxGeometry::create(1.6f, 0.05f, 0.09f);
        for (const auto& lamp : lamps) {
            if (!bakedBuilding) {// the baked building has its lamps hanging in it
                auto bar = Mesh::create(barGeometry, barMaterial);
                bar->position.copy(toThree(lamp.x, lamp.y, lampHeight));
                bar->rotation.y = lamp.alongX ? 0.f : static_cast<float>(pi / 2);
                scene->add(bar);
            }

            // candela, no cut-off distance, half-angle of the cone, soft edge, and
            // decay 2: the inverse-square law of real light.
            auto light = SpotLight::create(0xfff1dc, 42.f, 0.f, 1.05f, 0.85f, 2.f);
            light->position.copy(toThree(lamp.x, lamp.y, lampHeight - 0.05));
            auto target = Object3D::create();
            target->position.copy(toThree(lamp.x, lamp.y, 0));
            scene->add(target);
            light->setTarget(*target);
            scene->add(light);
        }
    }

    // What the lidar "sees", drawn for the human: a dot where each beam ends, and
    // (toggled with L) the beams themselves.
    void Simulator::Impl::buildLidarVisuals() {
        const auto beams = static_cast<size_t>(config.lidarBeams);

        auto pointGeometry = BufferGeometry::create();
        pointGeometry->setAttribute("position", FloatBufferAttribute::create(std::vector<float>(beams * 3), 3));
        auto pointMaterial = PointsMaterial::create();
        pointMaterial->color = Color(1.9f, 0.2f, 0.7f);// brighter than white: it glows
        pointMaterial->size = 4;
        pointMaterial->sizeAttenuation = false;// size in pixels, whatever the distance
        lidarPoints = Points::create(pointGeometry, pointMaterial);

        auto rayGeometry = BufferGeometry::create();
        rayGeometry->setAttribute("position", FloatBufferAttribute::create(std::vector<float>(beams * 2 * 3), 3));
        auto rayMaterial = LineBasicMaterial::create();
        rayMaterial->color = Color(0x19d3ff);
        rayMaterial->transparent = true;
        rayMaterial->opacity = 0.06f;
        lidarRays = LineSegments::create(rayGeometry, rayMaterial);

        for (Object3D* visual : {static_cast<Object3D*>(lidarPoints.get()), static_cast<Object3D*>(lidarRays.get())}) {
            visual->layers.set(debugLayer);
            // The vertices move every scan, so the bounding sphere threepp computed
            // once is wrong. Do not let it cull the object.
            visual->frustumCulled = false;
        }
        scene->add(lidarPoints);
        scene->add(lidarRays);
    }

    // Called before every picture: is the camera that takes it inside the building?
    //
    // The robot's own camera and the chase view are. They get the whole building, with
    // the walls up to the ceiling and the lamps under it. The overview looks in from
    // above: for it the building is cut open at door height, the way an architect
    // draws a section, and what is beyond the walls is a plain dark backdrop.
    //
    // (This is done by showing and hiding, not with layers like the lidar's beams,
    // because the mirror image in the floor is drawn by a camera of its own, which
    // knows nothing of our layers. Hidden is hidden for every camera.)
    void Simulator::Impl::lookFrom(bool inside) {
        for (auto* part : upperBuilding) {
            part->visible = inside;
        }
        // Without the baked building there are no walls to speak of. The robot's
        // camera then sees the photo of a real warehouse behind them.
        if (inside && warehousePhoto && !bakedBuilding) {
            scene->background = warehousePhoto;
        } else {
            scene->background = Color(windowBackground);
        }
    }

    // Let go of every mesh, material and texture. Called at the end of run(), while the
    // window and its OpenGL context still exist: GPU resources should be released by
    // the thread, and in the context, that created them.
    void Simulator::Impl::releaseScene() {
        lidarTargets.clear();
        upperBuilding.clear();
        droppedMeshes.clear();
        droppedVisuals.clear();
        falling.clear();
        wheelLeft = wheelRight = lidarHead = nullptr;
        lidarPoints.reset();
        lidarRays.reset();
        robotEyeCamera.reset();
        cameraSensor.reset();
        robotMarker.reset();
        robotModel.reset();
        floorPolish.reset();
        crateMaterial.reset();
        warehousePhoto.reset();
        scene.reset();
    }

    // A crate has been dropped into the world. Give it its two meshes: the plain box the
    // lidar's rays are tested against, and the model that is drawn.
    void Simulator::Impl::addCrateVisual(const Box& crate) {
        auto proxy = makeBoxMesh(crate, crateMaterial, 1.0f);
        scene->add(proxy);
        lidarTargets.push_back(proxy.get());
        droppedMeshes.push_back(proxy);

        // What is drawn: the crate model if there is one, else the box itself.
        // It starts three metres up and falls (see animateFalling).
        std::shared_ptr<Object3D> visual = proxy;
        if (auto model = loadModel(config.assetDir, crate.model)) {
            visual = fitModelToBox(model, crate);
            scene->add(visual);
            proxy->visible = false;
        }
        droppedVisuals.push_back(visual);
        falling.push_back({visual, 3.0, 0.0});
    }

    // Take every dropped crate out of the scene again (reset).
    void Simulator::Impl::removeDroppedVisuals() {
        for (const auto& mesh : droppedMeshes) {
            std::erase(lidarTargets, mesh.get());
            scene->remove(*mesh);
        }
        for (const auto& visual : droppedVisuals) {
            scene->remove(*visual);
        }
        droppedMeshes.clear();
        droppedVisuals.clear();
        falling.clear();
    }

    void Simulator::Impl::updateRobotModel(double elapsed) {
        robotModel->position.copy(toThree(truth.x, truth.y, 0));
        robotModel->rotation.y = static_cast<float>(truth.yaw);
        robotMarker->position.copy(toThree(truth.x, truth.y, 0.02));

        // Roll the wheels and spin the lidar. The model was made with z up and
        // exported with y up: an axle along the model's y is now along -z, the
        // lidar's vertical axis is now y.
        if (wheelLeft) wheelLeft->rotateZ(static_cast<float>(-wheelLeftTurn));
        if (wheelRight) wheelRight->rotateZ(static_cast<float>(-wheelRightTurn));
        wheelLeftTurn = 0;
        wheelRightTurn = 0;
        if (lidarHead) lidarHead->rotateY(static_cast<float>(2 * pi * 1.5 * elapsed));
    }

    // Let dropped crates fall the last metres. Only the picture moves.
    void Simulator::Impl::animateFalling(double elapsed) {
        for (auto& crate : falling) {
            crate.speed += 9.81 * elapsed;
            crate.height = std::max(0.0, crate.height - crate.speed * elapsed);
            crate.visual->position.y = static_cast<float>(crate.height);
        }
        std::erase_if(falling, [](const Falling& crate) { return crate.height <= 0.0; });
    }

}// namespace explorer_sim
