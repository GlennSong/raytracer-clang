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
    // perception, and bounded-radius steering over the road network (ADR-0059).
    addSystem<citysim::CityRenderSystem>();
#ifdef RT_ENABLE_PHYSICS
    addSystem<VehicleSystem>(physSys, camSys);   // drivable physics cars (ADR-0058)
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

void ArenaState::update(FrameContext& ctx) {
    PlayingState::update(ctx);
    if (makeEditorState && ctx.actions.pressed("quit"))
        ctx.transition.replaceWith(makeEditorState());
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
    // (ADR-0058), so they are data-driven rather than hardcoded here.
#endif

    // Play From Here (one-shot flag): start the player at the editor's view
    // instead of the document's spawn. The position rides the same settings
    // the editor camera already persists on state exit (flyEye*), and the
    // facing carries over for free — the play camera restores flyYaw/Pitch.
    if (ctx.settings.getBool("playFromHere", false)) {
        ctx.settings.setBool("playFromHere", false);
        Vec3 here(ctx.settings.getDouble("flyEyeX", 0.0),
                  ctx.settings.getDouble("flyEyeY", 1.5),
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
