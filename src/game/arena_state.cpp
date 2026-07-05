#include "arena_state.h"
#include "../engine/components.h"
#include "../engine/editor_bridge.h"
#include "../engine/level_loader.h"
#include "../engine/asset_manager.h"
#include "../engine/systems/dev_control_system.h"
#include "../engine/systems/camera_system.h"
#include "../engine/systems/motion_system.h"
#include "../engine/systems/day_night_system.h"
#include "../engine/systems/terrain_lod_system.h"
#include "../apps/citysim/city_render.h"
#include "../apps/citysim/city_spectate.h"
#ifdef RT_ENABLE_PHYSICS
#include "../apps/citysim/city_physics.h"
#include "../apps/citysim/city_vehicles.h"
#include "../apps/citysim/city_walkers.h"
#include "../apps/citysim/city_player_body.h"
#endif
#include "../engine/systems/vehicle_system.h"
#include "../engine/systems/render_system.h"
#include "../engine/systems/camera_panel_system.h"
#ifdef RT_ENABLE_PHYSICS
#include "../engine/systems/physics_system.h"
#include "../engine/systems/player_system.h"
#include "../engine/systems/shooting_system.h"
#ifdef RT_ENABLE_SCRIPTING
#include "../engine/scripting/script_system.h"
#include "../engine/scripting/script_behaviour.h"
#include "../renderer/event.h"
#include <fstream>
#include <sstream>
#include <vector>
#endif
#endif
#include "../log.h"

// Recipe hot-reload (watch_scripts) file plumbing — unconditional: the watcher
// compiles in every build and simply no-ops on the web (baked assets).
#include <nlohmann/json.hpp>
#include <filesystem>
#include <fstream>
#include <functional>
#include <system_error>

using namespace engine;

#if defined(RT_ENABLE_PHYSICS) && defined(RT_ENABLE_SCRIPTING)
namespace {
std::string readTextFile(const std::string& path) {
    std::ifstream in(path);
    if (!in) return {};
    std::stringstream ss;
    ss << in.rdbuf();
    return ss.str();
}
}  // namespace
#endif

ArenaState::ArenaState(Window& window, Renderer& renderer,
                       const std::string& levelFile,
                       EditorFactory makeEditorState, EditorBridge* bridge)
    : PlayingState(window), arenaRenderer(renderer), levelFile(levelFile),
      makeEditorState(std::move(makeEditorState)), bridge(bridge)
{
    // With an editor to return to, Esc stops the playtest instead of quitting.
    addSystem<DevControlSystem>(this->makeEditorState == nullptr);
    auto& camSys = addSystem<CameraSystem>();
#ifdef RT_ENABLE_PHYSICS
    auto& physSys = addSystem<PhysicsSystem>();
    addSystem<PlayerSystem>(camSys.flyController(), physSys);
#ifdef RT_ENABLE_SCRIPTING
    // The shooting gun is now a Lua ScriptBehaviour on the player (ADR-0024);
    // ScriptSystem drives it. The script is attached in onEnter, once the player
    // entity exists. (Replaces the C++ ShootingSystem's bullet spawning.)
    addSystem<ScriptSystem>();
#else
    addSystem<ShootingSystem>(camSys.flyController(), physSys, renderer);
#endif
#endif
    addSystem<MotionSystem>();
    // Agent-based city: drivers + pedestrians with acceleration, signals,
    // perception, and bounded-radius steering over the road network (ADR-0060).
    auto& citySys = addSystem<citysim::CityRenderSystem>();
    // Spectate camera (ADR-0064 follow-up): K cycles the view through the city's
    // agents and follows one (drive/walk their life with ring + cone lit). Reads
    // poses + drives ctx.view.camera only — no physics — so it sits OUTSIDE the
    // Jolt block, after CameraSystem/PlayerSystem so its override wins the frame.
    addSystem<citysim::CitySpectateSystem>(citySys);
#ifdef RT_ENABLE_PHYSICS
    // Motion authority per regime (ADR-0062 rethink): AMBIENT traffic is moved by
    // ONE authority — the CitySim planner — drawn instanced and collided via
    // kinematic proxies (CityPhysicsSystem), which is what keeps forty cars
    // flowing. Full Jolt dynamics is an INTERACTION response: CityVehicleSystem
    // PROMOTES an ambient car to a real Vehicle when the player commandeers it
    // (registered before VehicleSystem so the same enter press then seats the
    // player). Walkers are physics characters (CityWalkerSystem) — their plant is
    // trivially controllable, so bodies-follow-planner works for them everywhere.
    citySys.setPedsExternallyOwned(true);
    addSystem<citysim::CityPhysicsSystem>(citySys, physSys);   // car proxies + poles
    addSystem<citysim::CityVehicleSystem>(citySys, physSys);   // commandeer promotion
    addSystem<citysim::CityWalkerSystem>(citySys, physSys);    // spawn + drive walkers
    // The player IS a person in third person (ADR-0064): draws the walker body
    // over the on-foot shoulder camera, walk cycle shared with the crowd. After
    // PlayerSystem (line 60) so the capsule pose it reads is already this tick's.
    addSystem<citysim::CityPlayerBodySystem>();
    addSystem<VehicleSystem>(physSys, camSys);   // drives real cars: player + promoted
#else
    // physics-off build: no collider/vehicle/walker bridges; the spectate camera
    // above still follows the sim ghosts (citySys is consumed there).
#endif
    addSystem<DayNightSystem>();
#ifdef RT_ENABLE_PHYSICS
    addSystem<TerrainLodSystem>(&physSys);  // CDLOD draws + near-node colliders (ADR-0036)
#else
    addSystem<TerrainLodSystem>();          // CDLOD draws only (no physics build)
#endif
    addSystem<RenderSystem>();
    addSystem<CameraPanelSystem>(camSys);
}

// --- recipe hot-reload (the BUILDING LAB loop, building-grammar-plan.md P1) ---

// mtime as an opaque tick count; 0 = missing/unreadable (treated as unchanged,
// so a mid-save window never triggers a reload of a half-written file).
static long long fileStamp(const std::string& path) {
    std::error_code ec;
    auto t = std::filesystem::last_write_time(path, ec);
    if (ec) return 0;
    return static_cast<long long>(t.time_since_epoch().count());
}

void ArenaState::collectWatchFiles() {
    watchFiles.clear();
#ifndef __EMSCRIPTEN__
    std::ifstream f(levelFile);
    if (!f) return;
    nlohmann::json root = nlohmann::json::parse(f, nullptr, false);
    if (!root.is_object() || !root.value("watch_scripts", false)) return;
    const std::string levelDir =
        std::filesystem::path(levelFile).parent_path().string();
    auto add = [&](const std::string& path) {
        long long s = fileStamp(path);
        if (s != 0) watchFiles.push_back({path, s});
    };
    add(levelFile);   // level edits (opts/knobs) reload too
    for (const auto& ent : root.value("entities", nlohmann::json::array())) {
        if (ent.value("shape", std::string()) != "script") continue;
        const std::string file = ent.value("file", std::string());
        if (file.empty()) continue;
        // The same candidate paths the loader tries (loadScriptCode).
        for (const std::string& path :
             {file, std::string("assets/scripts/") + file, levelDir + "/" + file})
            if (fileStamp(path) != 0) { add(path); break; }
    }
    if (!watchFiles.empty())
        LOG_INFO << "watch_scripts: reloading on changes to "
                 << watchFiles.size() << " file(s)";
#endif
}

// Patch the lab level's script-entity `opts` on disk — the regenerate
// "button": mutate via fn, write back; the caller reloads. The panel/editor
// route goes through the same file, so keys, Lua edits, and (later) an
// inspector all share one source of truth.
static bool patchLabOpts(const std::string& levelFile,
                         const std::function<void(nlohmann::json&)>& fn) {
    std::ifstream f(levelFile);
    if (!f) return false;
    nlohmann::json root = nlohmann::json::parse(f, nullptr, false);
    if (!root.is_object() || !root.contains("entities")) return false;
    for (auto& ent : root["entities"]) {
        if (ent.value("shape", std::string()) != "script") continue;
        if (!ent["opts"].is_object()) ent["opts"] = nlohmann::json::object();
        fn(ent["opts"]);
        if (ent["opts"].contains("seed")) ent["seed"] = ent["opts"]["seed"];
        std::ofstream o(levelFile);
        if (!o) return false;
        o << root.dump(2) << "\n";
        return true;
    }
    return false;
}

bool ArenaState::watchedFilesChanged() {
    bool changed = false;
    for (WatchedFile& w : watchFiles) {
        long long s = fileStamp(w.path);
        if (s != 0 && s != w.stamp) { w.stamp = s; changed = true; }
    }
    return changed;
}

void ArenaState::update(FrameContext& ctx) {
    PlayingState::update(ctx);
    if (makeEditorState && ctx.actions.pressed("quit"))
        ctx.transition.replaceWith(makeEditorState());
    // A watched recipe changed on disk: replace this state with a fresh one on
    // the same level — the identical clean reset the editor→play loop uses
    // (systems, physics, and assets all rebuilt), so a saved Lua edit shows up
    // in seconds without touching the app.
    if (!watchFiles.empty()) {
        // Lab regenerate keys (the "button to see different styles"): U bumps
        // the seed, O cycles the cladding style. Both PATCH the level file's
        // opts (persisting the pick) and reload straight away.
        bool patched = false;
        if (ctx.actions.pressed("lab_reseed"))
            patched = patchLabOpts(levelFile, [](nlohmann::json& o) {
                const long long s = o.value("seed", 7);
                o["seed"] = static_cast<int>((s * 1103515245LL + 12345LL) % 99991LL) + 1;
            });
        if (ctx.actions.pressed("lab_style"))
            patched = patchLabOpts(levelFile, [](nlohmann::json& o) {
                static const char* kStyles[] = {"brick",  "stucco", "concrete",
                                                "metal",  "glass",  "painted"};
                const std::string cur = o.value("style", "brick");
                int idx = 0;
                for (int i = 0; i < 6; ++i)
                    if (cur == kStyles[i]) idx = i;
                o["style"] = kStyles[(idx + 1) % 6];
            }) || patched;
        if (patched) {
            ctx.transition.replaceWith(std::make_unique<ArenaState>(
                window, arenaRenderer, levelFile, makeEditorState, bridge));
            return;
        }
        watchTimer += 1.0;                 // frames; ~every half second at 60 fps
        if (watchTimer >= 30.0) {
            watchTimer = 0.0;
            if (watchedFilesChanged())
                ctx.transition.replaceWith(std::make_unique<ArenaState>(
                    window, arenaRenderer, levelFile, makeEditorState, bridge));
        }
    }
}

void ArenaState::onEnter(FrameContext& ctx) {
    // Always start from the document (edit mode may have just rewritten it).
    ctx.world.destroyAll();
    ctx.assets.clear();   // free the previous level's deduped GPU meshes
    ctx.settings.setBool("cameraFreeLook", false);
    ctx.settings.setBool("cameraDetachEnabled", true);
    collectWatchFiles();   // recipe hot-reload, when the level opts in
    if (!watchFiles.empty()) {
        ctx.actions.bindButton("lab_reseed", KeyCode::U);   // new seed, same style
        ctx.actions.bindButton("lab_style", KeyCode::O);    // cycle cladding style
    }
    if (!LevelLoader::load(levelFile, ctx.world, arenaRenderer, ctx.view, ctx.assets)) {
        LOG_ERROR << "Failed to load level: " << levelFile;
    }

#if defined(RT_ENABLE_PHYSICS) && defined(RT_ENABLE_SCRIPTING)
    // Attach the Lua gun (ADR-0024) to the player and bind its fire action. The
    // player is the entity PlayerSystem drives (Transform + CharacterController +
    // ControlledBy). Collected first, then tagged, so we never add a component
    // mid-iteration (World::each contract, ADR-0006).
    ctx.actions.bindButton("fire", MouseButton::Left);
    ctx.actions.bindButton("fire", GamepadButton::RightBumper);
    ctx.actions.bindButton("slot_1", KeyCode::Num1);   // bare hands (start here)
    ctx.actions.bindButton("slot_2", KeyCode::Num2);   // draw the gun
    {
        std::string gun = readTextFile("assets/scripts/gun.lua");
        if (gun.empty()) {
            LOG_WARN << "assets/scripts/gun.lua not found; player has no gun";
        } else {
            std::vector<Entity> players;
            ctx.world.each<Transform, ControlledBy>(
                [&](Entity e, Transform&, ControlledBy&) {
                    players.push_back(e);
                });
            for (Entity e : players) {
                if (!ctx.world.has<ScriptBehaviour>(e)) {
                    ScriptBehaviour sb;
                    sb.source = gun;
                    ctx.world.add<ScriptBehaviour>(e, sb);
                }
            }
        }
    }
    // Vehicles are spawned by the level loader from the level's "vehicles" block
    // (ADR-0059), so they are data-driven rather than hardcoded here.
#endif

    // Play From Here (one-shot flag): start the player at the editor's view
    // instead of the document's spawn. The position rides the same settings
    // the editor camera already persists on state exit (flyEye*), and the
    // facing carries over for free — the play camera restores flyYaw/Pitch.
    if (ctx.settings.getBool("playFromHere", false)) {
        ctx.settings.setBool("playFromHere", false);
        // Drop the capsule in from a little above the eye rather than placing its
        // base exactly at the camera point: the eye sits in open (renderable)
        // space, but the capsule extends downward from its origin, so origin-at-
        // eye can bury the feet in the floor/ledge the camera hovers over —
        // leaving the controller wedged (penetrating on all sides, unable to
        // move). Starting a touch higher lets gravity settle it cleanly onto
        // whatever is underneath.
        constexpr Real kSpawnDropMargin = 1.5;
        Vec3 here(ctx.settings.getDouble("flyEyeX", 0.0),
                  ctx.settings.getDouble("flyEyeY", 1.5) + kSpawnDropMargin,
                  ctx.settings.getDouble("flyEyeZ", 0.0));
        ctx.world.each<Transform, ControlledBy>(
            [&](Entity e, Transform& t, ControlledBy&) {
                t.position = here;
                if (auto* prev = ctx.world.get<PrevTransform>(e))
                    prev->value = t;
            });
        // Bodies spawn from these transforms when PhysicsSystem starts below;
        // gravity settles the capsule onto whatever is underneath.
    }

    // Placed cameras persist in a sidecar next to the level; CameraSystem
    // loads/saves it (must be set before PlayingState::onEnter starts systems).
    // The level path itself feeds the panel's offline-render button.
    ctx.settings.setString("cameraStorePath", levelFile + ".cameras.json");
    ctx.settings.setString("levelPath", levelFile);
    PlayingState::onEnter(ctx);

    // The shell watches the playtest read-only: live hierarchy/inspector,
    // editing disabled (EditorBridge::editable() is false without an editor).
    if (bridge) bridge->attachObserver(&ctx.world, levelFile);
}

void ArenaState::onExit(FrameContext& ctx) {
    if (bridge) bridge->detach();
    PlayingState::onExit(ctx);
}
