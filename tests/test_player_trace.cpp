// THE PLAYER, DRIVEN BY A SCRIPT (the refactor's safety net, ADR-0246). The real PlayerSystem on the real
// movement_lab, headless: the level through LevelLoader on the NullRenderer, Jolt single-threaded, the pad and the
// view set by a script each frame, and every fixed step written down -- where the body is, what the pose says,
// what last happened. Each scenario is one way of moving: walking and jumping, a pull-up, a climb, a swim, a
// parachute, a wingsuit, a hard fall, the grapple, a board, the swing and the bars, going limp.
//
//   RT_PLAYER_TRACE_OUT=<dir>    write each scenario's trace there (<dir>/<name>.trace)
//   RT_PLAYER_TRACE_BASE=<dir>   ...and compare it, step by step, with the one recorded there
//
// Without either the scenarios still run, and each checks the moves it is there for actually happened.

#include "test_framework.h"

#include "../src/engine/asset_manager.h"
#include "../src/engine/components.h"
#include "../src/engine/interact_broker.h"
#include "../src/engine/level_loader.h"
#include "../src/engine/mesh_uploader.h"
#include "../src/engine/player_pose.h"
#include "../src/engine/system.h"
#include "../src/engine/frame_stats.h"
#include "../src/engine/systems/physics_system.h"
#include "../src/engine/systems/player_system.h"
#include "../src/renderer/renderer.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <functional>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

using namespace engine;

namespace {

std::string labPath() {
#ifdef RT_SOURCE_DIR
    return std::string(RT_SOURCE_DIR) + "/assets/levels/movement_lab.json";
#else
    return "assets/levels/movement_lab.json";
#endif
}

// Everything a frame needs, owned here: the app's FrameContext, built the way Application::makeContext does.
struct Rig {
    std::unique_ptr<Renderer> renderer = Renderer::create();
    RendererMeshUploader uploader{*renderer};
    AssetManager assets{uploader};
    World world;
    RenderView view;
    SimClock clock;
    Settings settings;
    JobSystem jobs{0};   // synchronous: the same steps every run
    EventBus events;
    DebugDraw debug;
    AudioEngine audio;
    FrameStats stats;
    InputState input;
    InputMap actions;
    PlayerInputs players;
    XrState xr;
    bool quit = false;
    StateTransition transition;
    FlyCameraController fly;
    PhysicsSystem physics;
    PlayerSystem player{fly, physics};
    GamepadState pad;
    double time = 0;

    FrameContext ctx(double interpolation = 1.0) {
        return FrameContext{world, *renderer, assets, view, clock, settings, jobs, events, debug, audio, stats,
                            input, actions, players, xr, 1280, 720, 1280, 720, clock.fixedStep(), interpolation,
                            quit, transition, false};
    }

    bool load() {
        if (!LevelLoader::load(labPath(), world, *renderer, view, assets, /*editorMode=*/false)) return false;
        pad.connected = true;
        // CameraSystem's movement axes and the gun's trigger (the systems that bind them are not here)
        actions.bindAxis("cam_forward", GamepadAxis::LeftY, -1.0);
        actions.bindAxis("cam_right", GamepadAxis::LeftX, 1.0);
        actions.bindButton("fire", GamepadAxis::RightTrigger);
        FrameContext c = ctx();
        physics.initialize(nullptr);
        physics.createBodies(world);
        player.onStart(c);
        fly.positionLocked = true;
        return true;
    }

    Entity body() {
        Entity out;
        world.each<Transform, CharacterController, ControlledBy>(
            [&](Entity e, Transform&, CharacterController&, ControlledBy&) { out = e; });
        return out;
    }

    // Put the body somewhere before the first step (feet on what is under it), facing `yaw`.
    void place(const Vec3& feet, Real yaw, Real pitch = 0) {
        const Entity e = body();
        CharacterController* cc = world.get<CharacterController>(e);
        Transform* t = world.get<Transform>(e);
        const Vec3 centre = feet + Vec3(0, cc->halfHeight + cc->radius + 0.02, 0);
        physics.physicsWorld().setCharacterPosition(cc->characterId, centre);
        t->position = centre;
        fly.yaw = yaw;
        fly.pitch = pitch;
    }

    // E on whatever the playground offers (the broker's choice, made here)
    bool takePlaygroundOffer() {
        const Entity e = body();
        const InteractOffers* o = world.get<InteractOffers>(e);
        if (!o) return false;
        for (const InteractOffer& offer : o->list)
            if (offer.provider == "playground") {
                InteractCommand c;
                c.provider = offer.provider;
                c.entity = offer.entity;
                c.key = offer.key;
                world.add<InteractCommand>(e, c);
                return true;
            }
        return false;
    }

    // One frame the way Application runs one: input, update, a fixed step, render.
    void frame() {
        actions.beginFrame();
        actions.updateGamepad(pad);
        FrameContext c = ctx();
        if (InteractOffers* o = world.get<InteractOffers>(body())) o->list.clear();   // (the broker clears them)
        player.update(c);
        physics.fixedUpdate(c);
        player.fixedUpdate(c);
        player.render(c);
        time += clock.fixedStep();
    }
};

// The pad, by name
struct Pad {
    GamepadState& p;
    void stick(Real forward, Real right) {
        p.axes[static_cast<std::size_t>(GamepadAxis::LeftY)] = static_cast<float>(-forward);
        p.axes[static_cast<std::size_t>(GamepadAxis::LeftX)] = static_cast<float>(right);
    }
    void button(GamepadButton b, bool down) { p.buttons[static_cast<std::size_t>(b)] = down; }
    void jump(bool d) { button(GamepadButton::A, d); }
    void crouch(bool d) { button(GamepadButton::B, d); }
    void run(bool d) { button(GamepadButton::LeftThumb, d); }
    void fire(bool d) { p.axes[static_cast<std::size_t>(GamepadAxis::RightTrigger)] = d ? 1.0f : -1.0f; }
};

// What one step looks like, written down: the telemetry's words, then the numbers at full precision.
std::string telemetryField(const std::string& tele, const std::string& key) {
    const std::string k = key + " ";
    const std::size_t at = tele.find(k);
    if (at == std::string::npos) return "";
    std::size_t from = at + k.size();
    if (tele[from] == '"') {
        const std::size_t end = tele.find('"', from + 1);
        return tele.substr(from + 1, end - from - 1);
    }
    const std::size_t end = tele.find(' ', from);
    return tele.substr(from, end == std::string::npos ? std::string::npos : end - from);
}

std::string traceLine(Rig& r, int step) {
    const Entity e = r.body();
    const Transform* t = r.world.get<Transform>(e);
    const PlayerPose* p = r.world.get<PlayerPose>(e);
    const std::string tele = r.settings.getString("player.telemetry", "");
    std::ostringstream o;
    o.precision(9);
    o << step << " | " << telemetryField(tele, "mode") << " | " << telemetryField(tele, "ground") << " | "
      << telemetryField(tele, "hook") << " | " << telemetryField(tele, "last") << " |";
    auto v = [&](const Vec3& a) { o << ' ' << a.x << ' ' << a.y << ' ' << a.z; };
    v(t->position);
    if (p) {
        o << ' ' << static_cast<int>(p->mode) << ' ' << p->speed << ' ' << p->stride << ' ' << p->crouched << ' '
          << p->stamina << ' ' << p->landed << ' ' << p->progress << ' ' << p->climbCycle << ' ' << p->swimPhase
          << ' ' << p->swimStroke << ' ' << p->canopyOpen << ' ' << p->kick << ' ' << p->reel << ' ' << p->ropePoints
          << ' ' << p->hookShots << ' ' << p->hookBites << ' ' << p->ropeSnaps << ' ' << p->chuteOpens << ' '
          << p->ollies << ' ' << p->boardLands << ' ' << p->grabs << ' ' << p->grounded;
        v(p->handL);
        v(p->handR);
        v(p->canopyHeading);
        v(p->boardHeading);
        if (p->ropePoints > 0) v(p->rope[p->ropePoints - 1]);
        if (p->mode == PlayerPose::Mode::Ragdoll) v(p->ragdollPos[0]);
    }
    return o.str();
}

struct Scenario {
    std::string name;
    Real seconds;
    std::function<void(Rig&)> setup;            // where it starts, what is in hand
    std::function<void(Rig&, Pad&, Real)> script;   // each frame: the pad and the view at time t
    std::vector<std::string> expect;            // events that must happen (the telemetry's "last")
};

bool between(Real t, Real a, Real b) { return t >= a && t < b; }
// one frame at `a`: a press
bool tap(Real t, Real a) { return t >= a - 1e-9 && t < a + 1.0 / 60.0 - 1e-9; }

std::vector<Scenario> scenarios() {
    std::vector<Scenario> s;
    // WALKING: walk, run, a tapped hop, a held jump, a crouch under nothing
    s.push_back({"walk_jump", 6.0, [](Rig& r) { r.place(Vec3(0, 0, 0), 0); },
                 [](Rig&, Pad& p, Real t) {
                     p.stick(between(t, 0.2, 5.0) ? 1.0 : 0.0, between(t, 3.6, 4.4) ? 0.6 : 0.0);
                     p.run(between(t, 1.2, 2.6));
                     p.jump(tap(t, 1.6) || between(t, 2.4, 3.0));
                     p.crouch(between(t, 3.4, 4.2));
                 },
                 {"jump"}});
    // A PULL-UP from standing: jump at the 1.1 m (green) block
    s.push_back({"mantle_ground", 3.0, [](Rig& r) { r.place(Vec3(-3.6, 0, -8.0), 0); },
                 [](Rig&, Pad& p, Real t) {
                     p.stick(between(t, 0.0, 2.5) ? 1.0 : 0.0, 0);
                     p.jump(tap(t, 0.8));
                 },
                 {"up"}});
    // JUMP, GRAB, PULL UP: the 2.2 m (yellow) block
    s.push_back({"mantle_air", 3.5, [](Rig& r) { r.place(Vec3(6.8, 0, -8.0), 0); },
                 [](Rig&, Pad& p, Real t) {
                     p.stick(between(t, 0.0, 3.0) ? 1.0 : 0.0, 0);
                     p.jump(tap(t, 0.6));
                 },
                 {"up"}});
    // A CLIMB: the 8 m wall (grabbed off a jump), a lunge, across, up and over the top; then the 15 m one: up,
    // and kicked off it
    s.push_back({"climb", 16.0, [](Rig& r) { r.place(Vec3(-22.6, 0, -9.0), -90); },
                 [](Rig& r, Pad& p, Real t) {
                     p.stick(between(t, 0.0, 9.0) ? 1.0 : 0.0, between(t, 4.0, 5.0) ? 0.8 : 0.0);
                     p.jump(tap(t, 1.0) || tap(t, 3.0));
                     if (tap(t, 10.0)) r.place(Vec3(-22.6, 0, -4.0), -90);
                     if (t >= 10.0) {
                         p.stick(t < 12.8 ? 1.0 : t < 13.6 ? -1.0 : 0.0, 0);
                         p.jump(tap(t, 10.6) || tap(t, 13.2));
                     }
                 },
                 {"grab the wall", "lunge", "jump off the wall"}});
    // A WALL JUMP: run at the inside corner's wall, jump, jump again off it
    // (the stick let go in the air, or the hands take the wall instead)
    s.push_back({"wall_jump", 4.0, [](Rig& r) { r.place(Vec3(-14.0, 0, 29.0), -90); },
                 [](Rig&, Pad& p, Real t) {
                     p.stick(between(t, 0.0, 0.6) ? 1.0 : 0.0, 0);
                     p.run(true);
                     p.jump(between(t, 0.55, 0.75) || tap(t, 0.85));
                 },
                 {"jump off the wall"}});
    // A SWIM: down the steps, the crawl, a dive, back up, treading water
    s.push_back({"swim", 14.0, [](Rig& r) { r.place(Vec3(18.6, 0.05, 0), 90); },
                 [](Rig&, Pad& p, Real t) {
                     p.stick(between(t, 0.0, 8.0) ? 1.0 : 0.0, between(t, 6.0, 7.0) ? 0.7 : 0.0);
                     p.run(between(t, 4.0, 5.0));
                     p.crouch(between(t, 8.5, 9.6));
                     p.jump(between(t, 10.5, 11.2));
                 },
                 {"afloat"}});
    // THE PLUNGE off the 6 m board
    s.push_back({"plunge", 6.0, [](Rig& r) { r.place(Vec3(32.0, 6.0, 0), -90); },
                 [](Rig&, Pad& p, Real t) { p.stick(between(t, 0.0, 1.2) ? 1.0 : 0.0, 0); },
                 {"afloat"}});
    // THE PARACHUTE off the tower: walk off, open it, steer, dive, flare, land
    s.push_back({"parachute", 30.0, [](Rig& r) {
                     r.place(Vec3(32.5, 72.0, 38.5), 180);
                     r.actions.injectPress("slot_5");
                 },
                 [](Rig& r, Pad& p, Real t) {
                     p.stick(between(t, 0.3, 1.2) || between(t, 6.0, 8.0) ? 1.0 : between(t, 10.0, 11.0) ? -1.0 : 0.0, 0);
                     p.jump(tap(t, 2.4));
                     if (between(t, 4.0, 6.0)) r.fly.yaw += 0.6;
                 },
                 {"parachute open", "landed the chute*"}});
    // THE WINGSUIT off the tower, then its chute
    s.push_back({"wingsuit", 30.0, [](Rig& r) {
                     r.place(Vec3(32.5, 72.0, 38.5), 180);
                     r.actions.injectPress("slot_6");
                 },
                 [](Rig& r, Pad& p, Real t) {
                     p.stick(between(t, 0.3, 1.2) ? 1.0 : 0.0, 0);
                     p.jump(tap(t, 2.0) || tap(t, 7.0));
                     r.fly.pitch = between(t, 2.0, 4.5) ? -35.0 : between(t, 4.5, 7.0) ? -12.0 : 0.0;
                 },
                 {"wingsuit spread", "parachute open (off the wingsuit)"}});
    // THE WINGSUIT dived into the ground: the one impact rule (ADR-0247)
    s.push_back({"wingsuit_crash", 16.0, [](Rig& r) {
                     r.place(Vec3(32.5, 72.0, 38.5), 180);
                     r.actions.injectPress("slot_6");
                 },
                 [](Rig& r, Pad& p, Real t) {
                     p.stick(between(t, 0.3, 1.2) ? 1.0 : 0.0, 0);
                     p.jump(tap(t, 2.0));
                     r.fly.pitch = t > 2.0 ? -60.0 : 0.0;
                 },
                 {"wingsuit spread", "crashed the wingsuit*"}});
    // A HARD FALL off the tower, nothing on your back: limp, then up
    s.push_back({"hard_fall", 14.0, [](Rig& r) { r.place(Vec3(32.5, 72.0, 38.5), 180); },
                 [](Rig&, Pad& p, Real t) {
                     p.stick(between(t, 0.3, 1.2) ? 1.0 : 0.0, 0);
                     p.jump(tap(t, 11.0));
                 },
                 {"landed hard*", "got up"}});
    // THE GRAPPLE: hooked on a gantry's beam from the ground, pumped, let go, hooked on the next
    s.push_back({"grapple", 14.0, [](Rig& r) {
                     r.place(Vec3(0, 0, 52.0), 180, 62.0);
                     r.actions.injectPress("slot_7");
                 },
                 [](Rig& r, Pad& p, Real t) {
                     p.fire(between(t, 0.5, 5.0) || between(t, 6.0, 11.0));
                     p.stick(between(t, 2.0, 4.5) || between(t, 7.0, 10.0) ? 1.0 : 0.0, 0);
                     p.jump(between(t, 8.0, 8.6));
                     r.fly.pitch = t < 5.5 ? 62.0 : 28.0;
                 },
                 {"let go of the rope"}});
    // THE SKATEBOARD down the skate run: kick, an ollie off the deck, carve, brake
    s.push_back({"skateboard", 16.0, [](Rig& r) {
                     r.place(Vec3(-106.0, 4.0, 0), 90);
                     r.actions.injectPress("slot_8");
                 },
                 [](Rig&, Pad& p, Real t) {
                     p.stick(between(t, 0.2, 3.0) ? 1.0 : between(t, 11.0, 13.0) ? -1.0 : 0.0,
                             between(t, 6.0, 7.0) ? 0.8 : between(t, 8.0, 9.0) ? -0.8 : 0.0);
                     p.jump(tap(t, 2.0) || tap(t, 9.5));
                 },
                 {"ollie"}});
    // THE SCOOTER into the ledges, fast: off and limp
    s.push_back({"scooter_crash", 10.0, [](Rig& r) {
                     r.place(Vec3(-3.6, 0, 20.0), 0);
                     r.actions.injectPress("slot_9");
                 },
                 [](Rig&, Pad& p, Real t) {
                     p.stick(between(t, 0.2, 7.0) ? 1.0 : 0.0, 0);
                     p.run(true);
                 },
                 {"on the scooter", "bailed*"}});
    // THE SWING: on, pumped in time, jumped off at speed
    s.push_back({"swing", 12.0, [](Rig& r) { r.place(Vec3(-5.1, 0, 25.4), 180); },
                 [](Rig& r, Pad& p, Real t) {
                     if (tap(t, 0.5)) r.takePlaygroundOffer();
                     p.stick(t > 1.0 && t < 10.0 ? (std::fmod(t, 2.8) < 1.4 ? 1.0 : -1.0) : 0.0, 0);
                     p.jump(tap(t, 10.5));
                 },
                 {"on the swing"}});
    // THE MONKEY BARS: hand over hand along, off the end
    s.push_back({"bars", 18.0, [](Rig& r) { r.place(Vec3(4.4, 0, 26.0), 90); },
                 [](Rig& r, Pad& p, Real t) {
                     if (tap(t, 0.5)) r.takePlaygroundOffer();
                     p.stick(between(t, 1.0, 5.0) ? 1.0 : between(t, 5.0, 7.0) ? -1.0 : 0.0, 0);
                     p.crouch(between(t, 8.0, 8.5));
                     if (tap(t, 9.0)) r.takePlaygroundOffer();   // on again, and along to the far end and off it
                     if (t > 9.5) p.stick(1.0, 0);
                 },
                 {"on the monkey bars", "dropped off the bars", "off the end of the bars"}});
    // LIMP on purpose, mid-run, and up again
    s.push_back({"limp", 8.0, [](Rig& r) { r.place(Vec3(0, 0, 0), 0); },
                 [](Rig& r, Pad& p, Real t) {
                     p.stick(between(t, 0.0, 1.0) ? 1.0 : 0.0, 0);
                     p.run(true);
                     if (tap(t, 0.8) || tap(t, 2.0)) r.actions.injectPress("player_limp");   // (G again while limp)
                     p.jump(tap(t, 4.0));
                 },
                 {"went limp", "got up"}});
    return s;
}

// The trace's numbers, one line at a time, against the recorded one: words equal, numbers within `tol`.
bool sameLine(const std::string& a, const std::string& b, double tol, std::string& why) {
    std::istringstream sa(a), sb(b);
    std::string wa, wb;
    int field = 0;
    while (true) {
        const bool ea = !(sa >> wa), eb = !(sb >> wb);
        if (ea || eb) {
            if (ea != eb) why = "different lengths";
            return ea == eb;
        }
        ++field;
        char* endA = nullptr;
        char* endB = nullptr;
        const double na = std::strtod(wa.c_str(), &endA), nb = std::strtod(wb.c_str(), &endB);
        const bool numbers = endA && *endA == 0 && endB && *endB == 0 && !wa.empty() && !wb.empty();
        if (numbers ? std::fabs(na - nb) > tol * std::max(1.0, std::fabs(nb)) : wa != wb) {
            why = "field " + std::to_string(field) + ": " + wa + " vs " + wb;
            return false;
        }
    }
}

}  // namespace

TEST_CASE(the_player_moves_as_scripted_in_the_movement_lab) {
    const char* outDir = std::getenv("RT_PLAYER_TRACE_OUT");
    const char* baseDir = std::getenv("RT_PLAYER_TRACE_BASE");
    const char* only = std::getenv("RT_PLAYER_TRACE_ONLY");
    for (const Scenario& sc : scenarios()) {
        if (only && *only && sc.name.find(only) == std::string::npos) continue;
        Rig r;
        CHECK(r.load());
        if (!r.body().valid()) {
            CHECK(r.body().valid());
            return;
        }
        sc.setup(r);
        Pad pad{r.pad};
        pad.fire(false);
        std::vector<std::string> lines, events;
        const int steps = static_cast<int>(sc.seconds / r.clock.fixedStep() + 0.5);
        for (int i = 0; i < steps; ++i) {
            sc.script(r, pad, r.time);
            r.frame();
            lines.push_back(traceLine(r, i));
            const std::string last = telemetryField(r.settings.getString("player.telemetry", ""), "last");
            if (events.empty() || events.back() != last) events.push_back(last);
        }
        std::string said;
        for (const std::string& e : events) said += (said.empty() ? "" : ", ") + e;
        std::printf("    %s: %s\n", sc.name.c_str(), said.c_str());
        for (const std::string& want : sc.expect) {
            const bool prefix = !want.empty() && want.back() == '*';
            const std::string stem = prefix ? want.substr(0, want.size() - 1) : want;
            const bool seen = std::any_of(events.begin(), events.end(), [&](const std::string& e) {
                return prefix ? e.compare(0, stem.size(), stem) == 0 : e == want;
            });
            if (!seen) std::printf("    %s: never \"%s\"\n", sc.name.c_str(), want.c_str());
            CHECK(seen);
        }
        if (outDir && *outDir) {
            std::ofstream f(std::string(outDir) + "/" + sc.name + ".trace");
            for (const std::string& l : lines) f << l << '\n';
        }
        if (baseDir && *baseDir) {
            std::ifstream f(std::string(baseDir) + "/" + sc.name + ".trace");
            CHECK(f.good());
            std::string want;
            int i = 0;
            bool same = true;
            while (same && std::getline(f, want)) {
                std::string why;
                if (i >= static_cast<int>(lines.size())) {
                    std::printf("    %s: the trace ends at step %d\n", sc.name.c_str(), i);
                    same = false;
                } else if (!sameLine(lines[i], want, 1e-6, why)) {
                    std::printf("    %s: step %d differs (%s)\n      was %s\n      now %s\n", sc.name.c_str(), i,
                                why.c_str(), want.c_str(), lines[i].c_str());
                    same = false;
                }
                ++i;
            }
            CHECK(same);
        }
    }
}
