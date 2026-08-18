#include "arena_state.h"
#include "../engine/components.h"
#include "../engine/editor_bridge.h"
#include "../engine/level_loader.h"
#include "../engine/script_assets.h"
#include "../engine/asset_root.h"
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
#include "../apps/citysim/city_possess.h"
#endif
#include "../engine/systems/vehicle_system.h"
#include "../engine/systems/render_system.h"
#include "../engine/systems/xr_camera_system.h"
#include "../engine/systems/debug_draw_system.h"
#include "../engine/systems/audio_system.h"
#include "../engine/audio/sfx.h"
#include "../engine/systems/camera_panel_system.h"
#include "../engine/systems/planet_lab_system.h"
#include <algorithm>
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

namespace {

// The arena's sound content (ADR-0071): synthesizes the procedural clips
// (sfx.*, no sound files) and registers them by name, then turns physics
// Collision events into spatial impact cues — the full chain
// Jolt ContactListener -> EventBus -> PlaySound -> AudioEngine. Register
// after AudioSystem so the clips exist before the first cue resolves.
class ArenaSoundSystem : public System {
public:
    explicit ArenaSoundSystem(AudioSystem& audioSystem)
        : audioSystem(audioSystem) {}

    void onStart(FrameContext& ctx) override {
        if (!ctx.audio.ready()) return;
        uint32_t rate = ctx.audio.sampleRate();
        auto shot = sfx::gunshot(rate, 7);
        audioSystem.registerClip(
            "sfx/shot", ctx.audio.createClip(shot.data(), shot.size(), 1, rate));
        auto knock = sfx::impact(rate, 11);
        audioSystem.registerClip(
            "sfx/impact",
            ctx.audio.createClip(knock.data(), knock.size(), 1, rate));

#ifdef RT_ENABLE_PHYSICS
        EventBus* events = &ctx.events;
        collisionSub = ctx.events.subscribe<Collision>(
            [events](const Collision& hit) {
                // Gentle/resting touches stay silent; harder hits get louder
                // and slightly brighter. The cue is deferred to the frame
                // dispatch like every other reaction.
                if (hit.speed < 2.0) return;
                PlaySound cue;
                cue.clip = "sfx/impact";
                cue.spatial = true;
                cue.position = hit.position;
                cue.range = 45.0;
                double severity = std::min(hit.speed / 14.0, 1.0);
                cue.volume = static_cast<float>(0.25 + 0.75 * severity);
                cue.pitch = static_cast<float>(0.85 + 0.35 * severity);
                events->enqueue(cue);
            });
#endif
    }

    void onStop(FrameContext& ctx) override {
        ctx.events.unsubscribe(collisionSub);
        collisionSub = EventBus::INVALID_SUBSCRIPTION;
    }

private:
    AudioSystem& audioSystem;
    EventBus::SubscriptionId collisionSub = EventBus::INVALID_SUBSCRIPTION;
};

}  // namespace

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
    citySys_ = &citySys;   // polled in update() for the rebuild-roads button
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
    addSystem<citysim::CityVehicleSystem>(citySys);            // commandeer promotion
    addSystem<citysim::CityWalkerSystem>(citySys, physSys);    // spawn + drive walkers
    // The player IS a person in third person (ADR-0064): draws the walker body
    // over the on-foot shoulder camera, walk cycle shared with the crowd. After
    // PlayerSystem (line 60) so the capsule pose it reads is already this tick's.
    addSystem<citysim::CityPlayerBodySystem>();
    // The control channel's avatar (ADR-0079): consumes possess.cmd one-shots,
    // produces AgentDriver commands. AFTER CityWalkerSystem (commandeered
    // pedestrians already have bodies), BEFORE VehicleSystem (same-tick command
    // consumption) — and after PlayerSystem, so its chase-camera write wins.
    addSystem<citysim::CityPossessSystem>(citySys, physSys);
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
    // LAST camera writer: when a headset tracks, rewrite the shared camera to
    // follow the head (culling/lamps/audio/Lua) and hand the gameplay camera
    // to the renderer as the locomotion-base hint. Must stay after every
    // other camera writer and before RenderSystem. Inert without a headset.
    addSystem<XrCameraSystem>();
    addSystem<RenderSystem>();
    addSystem<DebugDrawSystem>();   // ctx.debug lines on top of the scene (ADR-0067)
    // After the camera systems so the listener follows this frame's view.
    auto& audioSys = addSystem<AudioSystem>();   // AudioSource/PlaySound -> AudioEngine (ADR-0069)
    // Procedural clips + Collision -> impact cues (ADR-0071).
    addSystem<ArenaSoundSystem>(audioSys);
    addSystem<CameraPanelSystem>(camSys);
    addSystem<PlanetLabSystem>();     // Debug > Planet Lab (RT_ENABLE_IMGUI)
}

// --- recipe hot-reload (the BUILDING LAB loop, building-grammar-plan.md P1) ---

// mtime as an opaque tick count; 0 = missing/unreadable (treated as unchanged,
// so a mid-save window never triggers a reload of a half-written file).
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

// Reseed the road recipe on disk (the "rebuild road graph" button): bump the
// seed of every shape:"road" entity's generate recipe, so a reload regrows the
// whole city — roads, terrain grading, and buildings — from a fresh graph.
static bool patchRoadSeed(const std::string& levelFile) {
    std::ifstream f(levelFile);
    if (!f) return false;
    nlohmann::json root = nlohmann::json::parse(f, nullptr, false);
    if (!root.is_object() || !root.contains("entities")) return false;
    bool patched = false;
    for (auto& ent : root["entities"]) {
        if (ent.value("shape", std::string()) != "road") continue;
        if (!ent.contains("road") || !ent["road"].contains("generate")) continue;
        auto& gen = ent["road"]["generate"];
        const long long s = gen.value("seed", 5);
        gen["seed"] = static_cast<int>((s * 1103515245LL + 12345LL) % 99991LL) + 1;
        patched = true;
    }
    if (!patched) return false;
    std::ofstream o(levelFile);
    if (!o) return false;
    o << root.dump(2) << "\n";
    return true;
}

void ArenaState::update(FrameContext& ctx) {
    PlayingState::update(ctx);
    if (makeEditorState && ctx.actions.pressed("quit"))
        ctx.transition.replaceWith(makeEditorState());
    // Rebuild-road-graph button (device ask): reseed every generated road on
    // disk and reload — roads, terrain conform, and buildings all regrow from
    // the fresh graph, the same clean reset the editor→play loop uses. Works on
    // any level with a shape:"road" generate recipe (the living city).
    if (citySys_ && citySys_->consumeRebuildRoadsRequest()) {
        if (patchRoadSeed(levelFile))
            ctx.transition.replaceWith(std::make_unique<ArenaState>(
                window, arenaRenderer, levelFile, makeEditorState, bridge));
        return;
    }
    // A watched recipe changed on disk: replace this state with a fresh one on
    // the same level — the identical clean reset the editor→play loop uses
    // (systems, physics, and assets all rebuilt), so a saved Lua edit shows up
    // in seconds without touching the app.
    if (!watch.empty()) {
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
        if (watch.tick())
            ctx.transition.replaceWith(std::make_unique<ArenaState>(
                window, arenaRenderer, levelFile, makeEditorState, bridge));
    }
}

void ArenaState::onEnter(FrameContext& ctx) {
    // Always start from the document (edit mode may have just rewritten it).
    ctx.world.destroyAll();
    ctx.assets.clear();   // free the previous level's deduped GPU meshes
    ctx.settings.setBool("cameraFreeLook", false);
    ctx.settings.setBool("cameraDetachEnabled", true);
    if (!LevelLoader::load(levelFile, ctx.world, arenaRenderer, ctx.view, ctx.assets)) {
        LOG_ERROR << "Failed to load level: " << levelFile;
    }

    // AFTER the load, not before: the watch list is seeded from the files the
    // load actually opened, which is the only way it can include a module a
    // recipe `require`d. Running it first meant guessing from the level JSON.
    watch.collect(levelFile);   // recipe hot-reload, when the level opts in
    if (!watch.empty()) {
        ctx.actions.bindButton("lab_reseed", KeyCode::U);   // new seed, same style
        ctx.actions.bindButton("lab_style", KeyCode::O);    // cycle cladding style
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
        // Through the asset root: inside a sandboxed bundle (visionOS) the CWD
        // is a temp dir and the bare relative path finds nothing — no gun.
        std::string gun = readTextFile(engine::assetPath("assets/scripts/gun.lua"));
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
