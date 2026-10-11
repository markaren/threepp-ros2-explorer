// The threepp half of the simulator. No ROS headers in this file.
//
// What happens here, every frame of the render loop:
//
//   1. run the jobs other threads have posted (drop a crate, reset, screenshot)
//   2. step the physics with a FIXED timestep until it has caught up with the clock
//   3. if a sensor tick is due: sample odometry, cast the lidar, render the camera,
//      and hand the results to the outside world
//   4. draw the window
//
// Steps 2 and 3 are the "physics vs render" pattern from the book's chapter on virtual
// environments. Step 3 has to live on this thread because the camera is an OpenGL
// render and OpenGL is thread-affine.

#include "explorer_sim/simulator.hpp"

#include "explorer_sim/world.hpp"

#include "threepp/canvas/Monitor.hpp"
#include "threepp/core/Raycaster.hpp"
#include "threepp/helpers/CameraSensor.hpp"
#include "threepp/loaders/FontLoader.hpp"
#include "threepp/loaders/RGBELoader.hpp"
#include "threepp/loaders/TextureLoader.hpp"
#include "threepp/objects/TextSprite.hpp"
#include "threepp/threepp.hpp"

#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <functional>
#include <limits>
#include <numbers>
#include <random>
#include <thread>

using namespace threepp;

namespace explorer_sim {

    namespace {

        constexpr double pi = std::numbers::pi;

        // Layer 1 holds everything that is drawn for the HUMAN and that the robot's own
        // camera must not see: lidar rays, the on-screen text. The window's camera shows
        // layers 0 and 1, the robot's camera only layer 0.
        constexpr unsigned int debugLayer = 1;

        // How the warehouse photo behind the scene is shown in the window.
        constexpr float windowBackgroundBlur = 0.5f;
        constexpr float windowBackgroundIntensity = 0.28f;

        // ---- ROS frame <-> threepp frame --------------------------------------------
        // ROS:     x forward, y left, z up.
        // threepp: x right,   y up,   z towards the viewer (like three.js).
        // Both are right-handed. We keep x, put ROS z on threepp y, and ROS y on threepp -z.
        // A yaw about ROS z is then the same angle about threepp y.

        Vector3 toThree(double x, double y, double z = 0) {
            return {static_cast<float>(x), static_cast<float>(z), static_cast<float>(-y)};
        }

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

        // The robot, built from primitives. Its local frame is the ROS base_link frame
        // converted to threepp axes: +x is forward, +y is up.
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

        // Move `current` towards `target`, but by no more than maxStep.
        double approach(double current, double target, double maxStep) {
            return current + std::clamp(target - current, -maxStep, maxStep);
        }

    }// namespace

    // =================================================================================

    struct Simulator::Impl {
        Config config;
        Channels channels;

        // ---- what other threads may touch ---------------------------------------------
        ThreadSafeQueue<std::function<void()>> jobs;// work posted to the simulator thread
        std::atomic<bool> stopRequested{false};

        // ---- everything below belongs to the simulator thread --------------------------
        World world = World::warehouse();

        // The robot's state.
        Pose2D truth;      // where it is
        Pose2D odom;       // where its wheel odometry thinks it is
        double linear = 0; // current speed, m/s
        double angular = 0;// current turn rate, rad/s
        bool commandFresh = false;

        Tuning tuning;
        std::mt19937 rng{42};// fixed seed: the same run gives the same noise

        // The scene. Created in run(), on the thread that owns the OpenGL context.
        std::shared_ptr<Scene> scene;
        std::shared_ptr<Group> robotModel;
        std::shared_ptr<CameraSensor> cameraSensor;
        std::vector<Object3D*> lidarTargets;             // what a lidar beam can hit
        std::vector<std::shared_ptr<Mesh>> droppedMeshes;// one per world.dropped entry
        std::shared_ptr<Material> crateMaterial;
        std::shared_ptr<Mesh> robotMarker;// a ring on the floor so a human finds the robot
        std::shared_ptr<Points> lidarPoints;
        std::shared_ptr<LineSegments> lidarRays;
        Raycaster raycaster;

        // A screenshot request waiting for the next rendered frame.
        std::shared_ptr<std::promise<std::string>> pendingScreenshot;
        int screenshotCount = 0;

        Impl(Config c, Channels ch): config(std::move(c)), channels(ch) {}

        // Run `job` on the simulator thread and deliver its return value through a
        // future. This is the promise/future pair from Part 3 of the book: the caller
        // keeps the future, the job carries the promise across the thread border.
        template<typename Result>
        std::future<Result> post(std::function<Result()> job) {
            auto promise = std::make_shared<std::promise<Result>>();
            auto future = promise->get_future();
            jobs.push([promise, job = std::move(job)] {
                promise->set_value(job());
            });
            return future;
        }

        // ---- building the scene --------------------------------------------------------

        void buildScene() {
            scene = Scene::create();
            scene->background = Color(0x171b22);
            const std::filesystem::path assets = config.assetDir;

            // Image-based lighting: a 360 degree photo of a real warehouse (an HDRI)
            // lights the scene and shows up in reflections. It is also the backdrop:
            // dim and blurred in the window, so the building stands out, and sharp in
            // the robot's camera (see captureCamera).
            bool hasEnvironment = false;
            const auto hdriFile = assets / "empty_warehouse_01_1k.hdr";
            if (!assets.empty() && std::filesystem::exists(hdriFile)) {
                if (auto hdri = RGBELoader().load(hdriFile)) {
                    scene->environment = hdri;
                    scene->environmentIntensity = 0.75f;
                    scene->background = hdri;
                    scene->backgroundBlurriness = windowBackgroundBlur;
                    scene->backgroundIntensity = windowBackgroundIntensity;
                    hasEnvironment = true;
                }
            }

            // A soft sky/ground fill (less of it when the HDRI already lights the
            // scene), and a sun that casts the shadows.
            scene->add(HemisphereLight::create(0xffffff, 0x8d99a8, hasEnvironment ? 0.45f : 1.1f));

            auto sun = DirectionalLight::create(0xffffff, 2.2f);
            const double centreX = (world.minX + world.maxX) / 2;
            sun->position.copy(toThree(centreX - 9, -7, 22));
            auto sunTarget = Object3D::create();
            sunTarget->position.copy(toThree(centreX, 0, 0));
            scene->add(sunTarget);
            sun->setTarget(*sunTarget);
            sun->castShadow = true;
            sun->shadow->mapSize.set(4096, 4096);
            sun->shadow->bias = -0.0004f;
            auto shadowCamera = sun->shadow->camera->as<OrthographicCamera>();
            shadowCamera->left = shadowCamera->bottom = -18;
            shadowCamera->right = shadowCamera->top = 18;
            shadowCamera->nearPlane = 1;
            shadowCamera->farPlane = 60;
            scene->add(sun);

            // Floor.
            const auto sizeX = static_cast<float>(world.maxX - world.minX);
            const auto sizeY = static_cast<float>(world.maxY - world.minY);
            auto floorGeometry = PlaneGeometry::create(sizeX, sizeY);
            // Repeat the concrete every 4 m instead of stretching one photo over 26 m.
            auto* floorUv = floorGeometry->getAttribute<float>("uv");
            for (size_t i = 0; i < static_cast<size_t>(floorUv->count()); ++i) {
                floorUv->setXY(i, floorUv->getX(i) * sizeX / 4.f, floorUv->getY(i) * sizeY / 4.f);
            }
            auto floorMaterial = makeSurface(assets, "concrete_floor_worn_001", "concrete_floor_worn_001_Diffuse.jpg",
                                             0x9ba1a9, Color(1.9f, 1.9f, 1.95f), 0.92f, 0.f);
            auto floor = Mesh::create(floorGeometry, floorMaterial);
            floor->rotation.x = static_cast<float>(-pi / 2);// a PlaneGeometry stands upright: lay it down
            floor->position.copy(toThree(centreX, (world.minY + world.maxY) / 2, 0));
            floor->receiveShadow = true;
            scene->add(floor);

            // The building. One material per kind of thing.
            //
            // The colours are not only decoration: the cone detector looks for
            // saturated orange, so nothing else in the building may be orange.
            const auto wallMaterial = makeSurface(assets, "painted_plaster_wall", "painted_plaster_wall_Diffuse.jpg",
                                                  0xe6eaef, Color(1.45f, 1.47f, 1.5f), 0.9f, 0.f);
            const auto shelfMaterial = makeSurface(assets, "blue_metal_plate", "blue_metal_plate_Diffuse.jpg",
                                                   0x4c6a88, Color(1.7f, 1.9f, 2.2f), 0.7f, 0.55f);
            const auto deskMaterial = makeMaterial(0xf5f7fa, 0.35f);
            // The planks are a greyscale version of Poly Haven's "weathered_planks": the
            // grain comes from the photo, the teal from the tint.
            crateMaterial = makeSurface(assets, "weathered_planks", "weathered_planks_grey.jpg",
                                        0x2a9d8f, 0x2fb3a3, 0.85f, 0.f);

            for (const auto& box : world.boxes) {
                std::shared_ptr<Material> material = wallMaterial;
                float tile = 2.5f;// metres of surface per repeat of the texture
                if (box.kind == Box::Kind::Shelf) {
                    material = shelfMaterial;
                    tile = 1.7f;
                } else if (box.kind == Box::Kind::Crate) {
                    material = crateMaterial;
                    tile = 1.0f;
                } else if (box.kind == Box::Kind::Desk) {
                    material = deskMaterial;
                }
                auto mesh = makeBoxMesh(box, material, tile);
                scene->add(mesh);
                lidarTargets.push_back(mesh.get());
            }

            // Traffic cones: RGB (255, 106, 0). A little self-illumination keeps them
            // orange in the shadow of a shelf.
            auto coneMaterial = makeMaterial(0xff6a00, 0.6f);
            coneMaterial->emissive = Color(0xff6a00);
            coneMaterial->emissiveIntensity = 0.25f;
            auto coneGeometry = ConeGeometry::create(static_cast<float>(Cone::baseRadius), static_cast<float>(Cone::height), 32);
            for (const auto& cone : world.cones) {
                auto mesh = Mesh::create(coneGeometry, coneMaterial);
                mesh->position.copy(toThree(cone.x, cone.y, Cone::height / 2));
                mesh->castShadow = true;
                scene->add(mesh);
                lidarTargets.push_back(mesh.get());
            }

            // The robot. Not a lidar target: it must not see itself.
            robotModel = makeRobotModel();
            scene->add(robotModel);

            // A glowing ring on the floor around the robot. The robot is 44 cm across in
            // a 26 m building: without this a human loses it in the overview.
            auto ringMaterial = MeshBasicMaterial::create();
            ringMaterial->color = Color(0x19d3ff);
            ringMaterial->transparent = true;
            ringMaterial->opacity = 0.85f;
            robotMarker = Mesh::create(RingGeometry::create(0.42f, 0.5f, 64), ringMaterial);
            robotMarker->rotation.x = static_cast<float>(-pi / 2);
            robotMarker->layers.set(debugLayer);// for the human only
            scene->add(robotMarker);

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

            buildLidarVisuals();
        }

        // What the lidar "sees", drawn for the human: a dot where each beam ends, and
        // (toggled with L) the beams themselves.
        void buildLidarVisuals() {
            const auto beams = static_cast<size_t>(config.lidarBeams);

            auto pointGeometry = BufferGeometry::create();
            pointGeometry->setAttribute("position", FloatBufferAttribute::create(std::vector<float>(beams * 3), 3));
            auto pointMaterial = PointsMaterial::create();
            pointMaterial->color = Color(0xff2d75);
            pointMaterial->size = 5;
            pointMaterial->sizeAttenuation = false;// size in pixels, whatever the distance
            lidarPoints = Points::create(pointGeometry, pointMaterial);

            auto rayGeometry = BufferGeometry::create();
            rayGeometry->setAttribute("position", FloatBufferAttribute::create(std::vector<float>(beams * 2 * 3), 3));
            auto rayMaterial = LineBasicMaterial::create();
            rayMaterial->color = Color(0x19d3ff);
            rayMaterial->transparent = true;
            rayMaterial->opacity = 0.22f;
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

        // ---- physics -------------------------------------------------------------------

        // Advance the robot by one fixed timestep.
        void step(double dt) {
            // 1. What does the controller want? A real robot must not keep driving on a
            //    stale command when its controller has crashed, so a command older than
            //    0.5 s counts as "stop".
            double targetLinear = 0;
            double targetAngular = 0;
            commandFresh = false;
            if (const auto command = channels.commands.latest()) {
                const auto age = std::chrono::steady_clock::now() - command->received;
                if (age < std::chrono::milliseconds(500)) {
                    commandFresh = true;
                    targetLinear = std::clamp(command->linear, -robot::maxLinear, robot::maxLinear);
                    targetAngular = std::clamp(command->angular, -robot::maxAngular, robot::maxAngular);
                }
            }

            // 2. Motors cannot change speed instantly.
            linear = approach(linear, targetLinear, robot::linearAccel * dt);
            angular = approach(angular, targetAngular, robot::angularAccel * dt);

            // 3. Integrate the motion of a differential-drive robot.
            const double beforeX = truth.x;
            const double beforeY = truth.y;
            truth.x += linear * std::cos(truth.yaw) * dt;
            truth.y += linear * std::sin(truth.yaw) * dt;
            truth.yaw = std::remainder(truth.yaw + angular * dt, 2 * pi);

            // 4. Walls are solid.
            if (world.pushOut(truth.x, truth.y, robot::radius)) {
                // Do not let speed build up while pushing against something.
                linear *= 0.9;
            }

            // 5. Wheel odometry: the robot adds up how far it thinks it moved. With
            //    odomDrift = 0 that is exact. With drift, the odometry over-counts the
            //    distance a little and believes it turns slightly while driving
            //    straight, like a robot with one wheel a millimetre larger than the
            //    other. The error grows with every metre: that is why real robots need
            //    more than odometry.
            const double moved = (truth.x - beforeX) * std::cos(truth.yaw) + (truth.y - beforeY) * std::sin(truth.yaw);
            const double odomMoved = moved * (1.0 + 0.02 * tuning.odomDrift);
            const double odomTurned = angular * dt + 0.03 * tuning.odomDrift * moved;
            odom.x += odomMoved * std::cos(odom.yaw);
            odom.y += odomMoved * std::sin(odom.yaw);
            odom.yaw = std::remainder(odom.yaw + odomTurned, 2 * pi);
        }

        // ---- sensors -------------------------------------------------------------------

        // Cast the lidar: one ray per beam into the scene graph, exactly as the book
        // describes a simulated lidar. threepp's Raycaster does the ray/triangle work.
        LaserScan castLidar() {
            LaserScan scan;
            const int beams = config.lidarBeams;
            scan.angleMin = -pi;
            scan.angleIncrement = 2 * pi / beams;
            scan.rangeMin = config.lidarRangeMin;
            scan.rangeMax = config.lidarRangeMax;
            scan.ranges.resize(static_cast<size_t>(beams));

            // The Raycaster works with world matrices. Make sure they are current: a
            // crate may have been added since the last frame was drawn.
            scene->updateMatrixWorld();

            const Vector3 origin = toThree(truth.x, truth.y, robot::laserHeight);
            raycaster.nearPlane = 0;
            raycaster.farPlane = static_cast<float>(scan.rangeMax);
            std::normal_distribution<double> noise(0.0, 1.0);

            auto* points = lidarPoints->geometry()->getAttribute<float>("position");
            auto* rays = lidarRays->geometry()->getAttribute<float>("position");

            for (int i = 0; i < beams; ++i) {
                const double beamAngle = truth.yaw + scan.angleMin + i * scan.angleIncrement;
                const double dirX = std::cos(beamAngle);
                const double dirY = std::sin(beamAngle);
                raycaster.set(origin, toThree(dirX, dirY, 0));

                double range = std::numeric_limits<double>::infinity();
                for (const auto& hit : raycaster.intersectObjects(lidarTargets)) {
                    range = std::min(range, static_cast<double>(hit.distance));
                }

                if (std::isfinite(range)) {
                    // A real lidar is never exact. The noise is what makes this sensor
                    // worth testing against.
                    range += noise(rng) * tuning.lidarNoise;
                    range = std::clamp(range, scan.rangeMin, scan.rangeMax);
                }
                scan.ranges[static_cast<size_t>(i)] = static_cast<float>(range);

                // Where to draw this beam. A beam that hit nothing is drawn to full range.
                const double drawn = std::isfinite(range) ? range : scan.rangeMax;
                const Vector3 end = toThree(truth.x + dirX * drawn, truth.y + dirY * drawn, robot::laserHeight);
                const auto index = static_cast<size_t>(i);
                // A beam without a return has no dot: park it inside the lidar itself.
                const Vector3& dot = std::isfinite(range) ? end : origin;
                points->setXYZ(index, dot.x, dot.y, dot.z);
                rays->setXYZ(index * 2, origin.x, origin.y, origin.z);
                rays->setXYZ(index * 2 + 1, end.x, end.y, end.z);
            }
            points->needsUpdate();
            rays->needsUpdate();
            return scan;
        }

        // Render the robot's camera and hand the picture to the mailbox.
        void captureCamera(GLRenderer& renderer, TimePoint stamp) {
            // The window shows the warehouse photo dim and blurred. The robot's camera
            // should see it as it is: switch for the duration of this one render.
            scene->backgroundBlurriness = 0.f;
            scene->backgroundIntensity = 0.9f;
            const bool captured = cameraSensor->capture(renderer, *scene);
            scene->backgroundBlurriness = windowBackgroundBlur;
            scene->backgroundIntensity = windowBackgroundIntensity;
            if (!captured) {
                return;
            }
            auto frame = std::make_shared<CameraFrame>();
            frame->stamp = stamp;
            frame->width = static_cast<int>(cameraSensor->width());
            frame->height = static_cast<int>(cameraSensor->height());
            frame->fovY = config.cameraFovYDeg * pi / 180.0;
            frame->rgb = cameraSensor->image();// one copy, made here on the render thread
            channels.camera.set(std::move(frame));
        }

        // ---- things other threads (or the keyboard) can ask for -------------------------

        SpawnResult spawnCrate(double x, double y, double size) {
            if (size <= 0) size = 0.6;
            size = std::clamp(size, 0.2, 1.5);
            const double half = size / 2;

            if (x - half < world.minX + 0.1 || x + half > world.maxX - 0.1 ||
                y - half < world.minY + 0.1 || y + half > world.maxY - 0.1) {
                return {false, "that spot is outside the building"};
            }
            // Not on top of the robot: leave it at least a hand's width.
            const double reach = std::hypot(half, half) + robot::radius + 0.05;
            if (std::hypot(x - truth.x, y - truth.y) < reach) {
                return {false, "that spot is on top of the robot"};
            }

            const Box crate{x, y, size, size, size, Box::Kind::Crate};
            world.dropped.push_back(crate);
            auto mesh = makeBoxMesh(crate, crateMaterial, 1.0f);
            scene->add(mesh);
            lidarTargets.push_back(mesh.get());
            droppedMeshes.push_back(mesh);

            char text[96];
            std::snprintf(text, sizeof(text), "crate %.1f m dropped at (%.2f, %.2f)", size, x, y);
            return {true, text};
        }

        bool reset() {
            for (const auto& mesh : droppedMeshes) {
                std::erase(lidarTargets, mesh.get());
                scene->remove(*mesh);
            }
            droppedMeshes.clear();
            world.dropped.clear();

            truth = {};
            odom = {};
            linear = 0;
            angular = 0;
            return true;
        }

        void updateRobotModel() const {
            robotModel->position.copy(toThree(truth.x, truth.y, 0));
            robotModel->rotation.y = static_cast<float>(truth.yaw);
            robotMarker->position.copy(toThree(truth.x, truth.y, 0.02));
        }
    };

    // =================================================================================

    Simulator::Simulator(Config config, Channels channels)
        : pimpl_(std::make_unique<Impl>(std::move(config), channels)) {}

    Simulator::~Simulator() = default;

    void Simulator::requestStop() {
        pimpl_->stopRequested = true;
    }

    std::future<SpawnResult> Simulator::requestSpawn(double x, double y, double size) {
        Impl* impl = pimpl_.get();
        return impl->post<SpawnResult>([impl, x, y, size] { return impl->spawnCrate(x, y, size); });
    }

    std::future<bool> Simulator::requestReset() {
        Impl* impl = pimpl_.get();
        return impl->post<bool>([impl] { return impl->reset(); });
    }

    std::future<std::string> Simulator::requestScreenshot() {
        // This one cannot be answered inside the job: the picture only exists after
        // the frame has been drawn. The job just parks the promise where the render
        // loop will find it.
        Impl* impl = pimpl_.get();
        auto promise = std::make_shared<std::promise<std::string>>();
        auto future = promise->get_future();
        impl->jobs.push([impl, promise] { impl->pendingScreenshot = promise; });
        return future;
    }

    void Simulator::run() {
        Impl& sim = *pimpl_;
        const Config& config = sim.config;

        // ---- window, renderer, scene ---------------------------------------------------
        Canvas canvas(Canvas::Parameters()
                              .title("explorer_sim: the world as it really is")
                              .size(config.width, config.height)
                              .antialiasing(4)
                              .headless(config.headless));
        GLRenderer renderer(canvas);
        renderer.shadowMap().enabled = true;
        renderer.shadowMap().type = ShadowMap::PFC;
        // The HDRI holds real light levels, far brighter than a screen can show. A tone
        // mapper squeezes them into range. "Neutral" does it without shifting hues,
        // which matters here: the cone detector depends on orange staying orange.
        renderer.toneMapping = ToneMapping::Neutral;
        renderer.toneMappingExposure = 1.0f;

        sim.buildScene();

        // The window's camera: an overview of the whole building from the south.
        PerspectiveCamera camera(50, canvas.aspect(), 0.1f, 200);
        camera.layers.enable(debugLayer);
        const double centreX = (sim.world.minX + sim.world.maxX) / 2;
        camera.position.copy(toThree(centreX, -17, 19));
        OrbitControls controls(camera, canvas);
        controls.target.copy(toThree(centreX, -0.5, 0));
        controls.maxPolarAngle = static_cast<float>(pi / 2 - 0.05);// stay above the floor
        controls.update();

        canvas.onWindowResize([&](WindowSize size) {
            camera.aspect = size.aspect();
            camera.updateProjectionMatrix();
            renderer.setSize(size);
        });

        // ---- on-screen text --------------------------------------------------------------
        // A TextSprite with screenSpace = true is positioned in pixels, relative to an
        // anchor on the viewport (0,0 = bottom left, 1,1 = top right).
        FontLoader fontLoader;
        const auto font = fontLoader.defaultFont();
        const float textScale = monitor::contentScale().first;

        auto makeText = [&](float size, const Color& color, float anchorY, float offsetY, TextSprite::VerticalAlignment align) {
            auto text = TextSprite::create(font, size * textScale);
            text->setColor(color);
            text->setVerticalAlignment(align);
            text->screenSpace = true;
            text->screenAnchor.set(0.f, anchorY);
            text->position.set(14.f * textScale, offsetY * textScale, 0.f);
            text->layers.set(debugLayer);
            sim.scene->add(text);
            return text;
        };
        auto title = makeText(22, 0xffffff, 1.f, -12.f, TextSprite::VerticalAlignment::Below);
        title->setText("GROUND TRUTH   what the world really looks like");
        auto status = makeText(16, 0x9fe8ff, 1.f, -44.f, TextSprite::VerticalAlignment::Below);
        auto help = makeText(15, 0xc8d0da, 0.f, 12.f, TextSprite::VerticalAlignment::Above);
        help->setText("left click: drop a crate    L: lidar beams    F: follow the robot    R: reset    Esc: quit");

        // ---- keyboard and mouse ----------------------------------------------------------
        bool followRobot = false;
        sim.lidarRays->visible = false;

        KeyAdapter keys(KeyAdapter::Mode::KEY_PRESSED, [&](KeyEvent event) {
            if (event.key == Key::L) sim.lidarRays->visible = !sim.lidarRays->visible;
            if (event.key == Key::F) followRobot = !followRobot;
            if (event.key == Key::R) sim.reset();
        });
        canvas.addKeyListener(keys);

        // A click drops a crate where the mouse points. A drag is the orbit control's
        // business, so only a press and release at (nearly) the same pixel counts.
        struct ClickListener: MouseListener {
            std::function<void(const Vector2&)> onClick;
            Vector2 pressedAt;
            Vector2 current;

            void onMouseMove(const Vector2& pos) override { current = pos; }
            void onMouseDown(int button, const Vector2& pos) override {
                if (button == 0) pressedAt = pos;
            }
            void onMouseUp(int button, const Vector2& pos) override {
                if (button == 0 && pos.distanceTo(pressedAt) < 4) onClick(pos);
            }
        } clicks;
        clicks.onClick = [&](const Vector2& pixel) {
            // Pixel -> normalised device coordinates (-1..1) -> a ray from the camera.
            const auto size = canvas.size();
            const Vector2 ndc{pixel.x / static_cast<float>(size.width()) * 2 - 1,
                              -(pixel.y / static_cast<float>(size.height())) * 2 + 1};
            Raycaster picker;
            picker.setFromCamera(ndc, camera);
            const auto& ray = picker.ray;
            if (ray.direction.y >= -1e-4f) return;// pointing at the sky
            // Where does the ray meet the floor (threepp y = 0)?
            const float t = -ray.origin.y / ray.direction.y;
            const float hitX = ray.origin.x + ray.direction.x * t;
            const float hitZ = ray.origin.z + ray.direction.z * t;
            sim.spawnCrate(hitX, -hitZ, 0);// back to ROS axes: y = -z
        };
        canvas.addMouseListener(clicks);

        // ---- the loop --------------------------------------------------------------------
        using Clock = std::chrono::steady_clock;
        const double physicsDt = 1.0 / 240.0;// fixed physics timestep
        const double tickPeriod = 1.0 / config.odomRate;
        const int ticksPerScan = std::max(1, static_cast<int>(std::lround(config.odomRate / config.sensorRate)));

        auto lastFrame = Clock::now();
        auto nextFrame = lastFrame;
        double physicsAccumulator = 0;
        double tickAccumulator = 0;
        double statusAccumulator = 1;
        int tickCount = 0;
        Pose2D lastFollowed;

        canvas.animate([&] {
            if (sim.stopRequested) {
                canvas.close();
                return;
            }

            // 1. Jobs from other threads. tryPop never blocks: no job, no waiting.
            while (auto job = sim.jobs.tryPop()) {
                (*job)();
            }
            if (const auto newTuning = sim.channels.tuning.latest()) {
                sim.tuning = *newTuning;
            }

            // 2. Physics: fixed steps until caught up with the wall clock.
            const auto now = Clock::now();
            // If the program was paused (a breakpoint, a dragged window), do not try
            // to simulate the whole gap at once.
            const double elapsed = std::min(std::chrono::duration<double>(now - lastFrame).count(), 0.1);
            lastFrame = now;
            physicsAccumulator += elapsed;
            while (physicsAccumulator >= physicsDt) {
                sim.step(physicsDt);
                physicsAccumulator -= physicsDt;
            }
            sim.updateRobotModel();

            // 3. Sensors, at their own rate (not the display's).
            tickAccumulator += elapsed;
            if (tickAccumulator >= tickPeriod) {
                tickAccumulator = std::fmod(tickAccumulator, tickPeriod);

                Telemetry sample;
                sample.stamp = std::chrono::system_clock::now();
                sample.odom = sim.odom;
                sample.truth = sim.truth;
                sample.linear = sim.linear;
                sample.angular = sim.angular;

                if (tickCount++ % ticksPerScan == 0) {
                    // Lidar and camera fire on the same tick and carry the same stamp,
                    // like a hardware-synchronised sensor rig. Downstream code can pair
                    // an image with its scan by comparing stamps.
                    sample.scan = sim.castLidar();
                    sim.captureCamera(renderer, sample.stamp);
                }
                sim.channels.telemetry.push(std::move(sample));
            }

            // 4. Draw.
            if (followRobot) {
                // Move the camera and its orbit centre along with the robot.
                const Vector3 delta = toThree(sim.truth.x - lastFollowed.x, sim.truth.y - lastFollowed.y, 0);
                camera.position.add(delta);
                controls.target.add(delta);
                controls.update();
            }
            lastFollowed = sim.truth;

            statusAccumulator += elapsed;
            if (statusAccumulator > 0.2) {// the text does not need 60 updates a second
                statusAccumulator = 0;
                char text[200];
                std::snprintf(text, sizeof(text), "x %6.2f m   y %6.2f m   heading %4.0f deg   speed %.2f m/s   cmd_vel: %s   dropped crates: %zu",
                              sim.truth.x, sim.truth.y, sim.truth.yaw * 180 / pi, sim.linear,
                              sim.commandFresh ? "receiving" : "none (stopped)", sim.world.dropped.size());
                status->setText(text);
            }

            renderer.render(*sim.scene, camera);

            if (sim.pendingScreenshot) {
                std::error_code ignored;
                std::filesystem::create_directories(config.screenshotDir, ignored);
                const auto file = std::filesystem::absolute(
                        std::filesystem::path(config.screenshotDir) / ("sim_" + std::to_string(sim.screenshotCount++) + ".png"));
                renderer.writeFramebuffer(file);
                sim.pendingScreenshot->set_value(file.string());
                sim.pendingScreenshot.reset();
            }

            // A hidden window is not held back by the display's refresh rate and would
            // spin as fast as the CPU allows. Hold it to 60 frames a second with a
            // rolling deadline (sleep_until, not sleep_for: see "Real-Time & Timing").
            if (config.headless) {
                nextFrame += std::chrono::microseconds(16667);
                if (nextFrame < Clock::now()) nextFrame = Clock::now();// fell behind: do not try to catch up
                std::this_thread::sleep_until(nextFrame);
            }
        });

        // The loop is over. Jobs still in the queue will never run: destroy them, which
        // breaks their promises and wakes any thread still waiting on a future.
        while (sim.jobs.tryPop()) {
        }
        sim.pendingScreenshot.reset();
    }

}// namespace explorer_sim
