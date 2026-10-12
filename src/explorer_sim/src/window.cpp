// In what order: the window and the frame loop. Simulator::run() is the whole file.
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

#include "overlay.hpp"
#include "simulator_impl.hpp"

namespace explorer_sim {

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
        bool effects = config.effects;
        sim.floorPolish->visible = effects;

        // The cards and text that lie over the picture. What they show is in
        // overlay.cpp, how they are drawn in hud.cpp.
        Overlay overlay(static_cast<float>(config.cameraWidth) / static_cast<float>(config.cameraHeight),
                        canvas.size().width());
        overlay.showEffects(effects);

        // The window's camera. It starts with an overview of the whole building from
        // the south, framed a little to the left so the inset in the bottom right
        // corner covers floor outside the building and not the building itself.
        PerspectiveCamera camera(50, canvas.aspect(), 0.1f, 200);
        camera.layers.enable(debugLayer);
        const double centreX = (sim.world.minX + sim.world.maxX) / 2;
        const Vector3 overviewPosition = toThree(centreX + 3.0, -14.6, 22.1);// steep: it has to see into the aisles
        const Vector3 overviewTarget = toThree(centreX + 3.0, -1.9, 0);
        camera.position.copy(overviewPosition);
        OrbitControls controls(camera, canvas);
        controls.target.copy(overviewTarget);
        controls.maxPolarAngle = static_cast<float>(pi / 2 - 0.05);// stay above the floor
        controls.update();

        // Three ways to look at the scene, cycled with F:
        //   Overview  the whole building, mouse orbits freely
        //   Follow    the mouse still orbits, but around the robot, which stays centred
        //   Chase     the camera rides behind the robot, like a racing game
        enum class View { Overview,
                          Follow,
                          Chase };
        View view = View::Overview;
        if (config.view == "follow") view = View::Follow;
        if (config.view == "chase") view = View::Chase;

        auto enterView = [&](View next) {
            view = next;
            controls.enabled = view != View::Chase;// in chase mode the camera is not the mouse's
            const Vector3 robotAt = toThree(sim.truth.x, sim.truth.y, 0);
            if (view == View::Overview) {
                camera.position.copy(overviewPosition);
                controls.target.copy(overviewTarget);
            } else if (view == View::Follow) {
                camera.position.copy(robotAt).add(toThree(-3.0, -4.5, 5.0));
                controls.target.copy(robotAt);
            } else {
                // Start right behind the robot, so the view does not swing in from afar.
                camera.position.copy(toThree(sim.truth.x - 2.2 * std::cos(sim.truth.yaw),
                                             sim.truth.y - 2.2 * std::sin(sim.truth.yaw), 1.9));
            }
            if (controls.enabled) controls.update();
            overlay.showCameraName(view == View::Overview ? "overview" : view == View::Follow ? "follow" : "chase");
        };
        enterView(view);
        unsigned handledViewRequest = 0;// the last camera request (through the "view" parameter) acted on

        // ---- post-processing --------------------------------------------------------------
        // Without it, a frame is: draw the scene, done. With it, the frame goes through
        // a chain of passes, each a full-screen image operation on the result of the one
        // before (an EffectComposer runs the chain):
        //
        //   RenderPass        draw the scene into an off-screen image
        //   GTAOPass          ambient occlusion: darken creases and the places where
        //                     things stand on the floor, where little light gets in.
        //                     This is what makes objects look planted instead of pasted on
        //   UnrealBloomPass   let very bright pixels (lamps, LEDs, laser dots) bleed
        //                     into a glow, as they do in a camera or an eye
        //
        // Press P to switch it all off and see what each frame looked like in 1998.
        const auto windowSize = canvas.size();
        EffectComposer::Options composerOptions;
        composerOptions.samples = 4;// anti-aliasing for the off-screen images
        EffectComposer composer(renderer, composerOptions);
        composer.addPass(std::make_shared<RenderPass>(*sim.scene, camera));

        GTAOParameters aoParameters;
        aoParameters.radius = 0.45f;// metres: how large a crease still darkens
        aoParameters.distanceExponent = 1.f;
        aoParameters.thickness = 1.f;
        aoParameters.scale = 1.1f;
        aoParameters.samples = 16;
        aoParameters.distanceFallOff = 1.f;
        aoParameters.screenSpaceRadius = false;
        PoissonDenoiseParameters denoiseParameters;
        denoiseParameters.lumaPhi = 10.f;
        denoiseParameters.depthPhi = 2.f;
        denoiseParameters.normalPhi = 3.f;
        denoiseParameters.radius = 4.f;
        denoiseParameters.radiusExponent = 1.f;
        denoiseParameters.rings = 2.f;
        denoiseParameters.samples = 16;
        composer.addPass(std::make_shared<GTAOPass>(*sim.scene, camera, windowSize.width(), windowSize.height(), aoParameters, denoiseParameters));

        // strength, radius, threshold. The threshold is above 1: only light sources bloom.
        composer.addPass(std::make_shared<UnrealBloomPass>(
                Vector2(static_cast<float>(windowSize.width()), static_cast<float>(windowSize.height())), 0.2f, 0.4f, 1.2f));

        // Lay the overlay out now, and again whenever the window changes size.
        overlay.layout(windowSize.width(), windowSize.height());
        canvas.onWindowResize([&](WindowSize size) {
            camera.aspect = size.aspect();
            camera.updateProjectionMatrix();
            renderer.setSize(size);
            composer.setSize(size.width(), size.height());
            overlay.layout(size.width(), size.height());
        });

        // ---- keyboard and mouse ----------------------------------------------------------

        KeyAdapter keys(KeyAdapter::Mode::KEY_PRESSED, [&](KeyEvent event) {
            if (event.key == Key::L) sim.lidarRays->visible = !sim.lidarRays->visible;
            if (event.key == Key::F) {
                enterView(view == View::Overview ? View::Follow : view == View::Follow ? View::Chase : View::Overview);
            }
            if (event.key == Key::R) sim.reset();
            if (event.key == Key::P) {
                effects = !effects;
                sim.floorPolish->visible = effects;
                overlay.showEffects(effects);
            }
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
                // A request to switch camera (ros2 param set /sim view chase). The
                // mailbox keeps returning the newest value, so act once per request:
                // each request carries a number one higher than the last.
                if (sim.tuning.viewRequest != handledViewRequest) {
                    handledViewRequest = sim.tuning.viewRequest;
                    enterView(sim.tuning.view == "chase" ? View::Chase : sim.tuning.view == "follow" ? View::Follow : View::Overview);
                }
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
            sim.updateRobotModel(elapsed);
            sim.animateFalling(elapsed);

            // 3. Sensors, at their own rate (not the display's). A tick happens inside a
            //    frame, so the rate is "30 a second, as long as the window draws at least
            //    30 frames a second". On a machine that draws slower, the sensors slow
            //    down with it: ticks that were missed are not made up for, because a
            //    lidar scan of a moment that has passed would be a scan of the wrong pose.
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
            if (view == View::Follow) {
                // Move the camera and its orbit centre along with the robot.
                const Vector3 delta = toThree(sim.truth.x - lastFollowed.x, sim.truth.y - lastFollowed.y, 0);
                camera.position.add(delta);
                controls.target.add(delta);
                controls.update();
            } else if (view == View::Chase) {
                // Where the camera wants to be: behind and above the robot. It moves a
                // fraction of the way there each frame, which smooths out the robot's
                // sharp turns. (1 - exp(-k dt) makes the smoothing independent of the
                // frame rate.)
                const double c = std::cos(sim.truth.yaw);
                const double sn = std::sin(sim.truth.yaw);

                // If the robot has just come round a corner, there is a wall where the
                // camera wants to be. Probe backwards from the robot with a ray, and
                // stay in front of whatever it hits.
                double back = 2.2;
                Raycaster probe(toThree(sim.truth.x, sim.truth.y, 0.6), toThree(-c, -sn, 0), 0.f, static_cast<float>(back + 0.35));
                for (const auto& hit : probe.intersectObjects(sim.lidarTargets)) {
                    back = std::min(back, static_cast<double>(hit.distance) - 0.35);
                }
                back = std::max(back, 0.5);
                const Vector3 wanted = toThree(sim.truth.x - back * c, sim.truth.y - back * sn, 1.9);
                camera.position.lerp(wanted, static_cast<float>(1.0 - std::exp(-3.0 * elapsed)));
                camera.lookAt(toThree(sim.truth.x + 1.0 * c, sim.truth.y + 1.0 * sn, 0.35));
            }
            lastFollowed = sim.truth;

            statusAccumulator += elapsed;
            if (statusAccumulator > 0.2) {// the readouts do not need 60 updates a second
                statusAccumulator = 0;
                overlay.update({sim.linear, sim.truth.yaw * 180 / pi, sim.truth.x, sim.truth.y,
                                sim.commandFresh, sim.world.dropped.size()});
            }

            // The chase view is inside the building and sees its ceiling. The other
            // two look in from above, at a building cut open (see lookFrom).
            sim.robotMarker->visible = view != View::Chase;
            sim.lookFrom(view == View::Chase);
            if (effects) {
                sim.mirrorDue = true;// one mirror image of the floor for this frame
                composer.render(static_cast<float>(elapsed));
            } else {
                renderer.render(*sim.scene, camera);
            }
            sim.lookFrom(false);

            // The 2D layer, over the finished picture.
            overlay.draw(renderer);

            // The picture in the camera card: the same scene once more, from the
            // robot's camera, drawn into a rectangle of the window. The scissor test
            // confines all drawing (and clearing) to the rectangle that was set last.
            {
                const auto window = canvas.size();
                const Overlay::Rect picture = overlay.cameraPicture();
                renderer.setScissorTest(true);

                // A one pixel frame: clear a slightly larger rectangle in the frame's colour.
                renderer.setScissor(picture.x - 1, picture.y - 1, picture.width + 2, picture.height + 2);
                renderer.setClearColor(Color(0x19d3ff));
                renderer.clear();

                renderer.setScissor(picture.x, picture.y, picture.width, picture.height);
                renderer.setViewport(picture.x, picture.y, picture.width, picture.height);
                sim.lookFrom(true);
                renderer.render(*sim.scene, *sim.robotEyeCamera);
                sim.lookFrom(false);

                renderer.setScissorTest(false);
                renderer.setViewport(0, 0, window.width(), window.height());
            }

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

        // The loop is over. From now on post() refuses new jobs. Those still in the
        // queue will never run: destroy them, which breaks their promises and wakes any
        // thread still waiting on a future.
        sim.loopEnded = true;
        while (sim.jobs.tryPop()) {
        }
        sim.pendingScreenshot.reset();

        // Free the scene now, before the window and its OpenGL context go (they are
        // local variables of this function and die when it returns).
        sim.releaseScene();
    }

}// namespace explorer_sim
