#include "arena_state.h"
#include "../engine/level_loader.h"
#include "../engine/systems/dev_control_system.h"
#include "../engine/systems/camera_system.h"
#include "../engine/systems/motion_system.h"
#include "../engine/systems/render_system.h"
#ifdef RT_ENABLE_PHYSICS
#include "../engine/systems/physics_system.h"
#include "../engine/systems/player_system.h"
#include "../engine/systems/shooting_system.h"
#endif
#include "../log.h"

using namespace engine;

ArenaState::ArenaState(Window& window, Renderer& renderer,
                       const std::string& levelFile)
    : PlayingState(window), arenaRenderer(renderer), levelFile(levelFile)
{
    addSystem<DevControlSystem>();
    auto& camSys = addSystem<CameraSystem>();
#ifdef RT_ENABLE_PHYSICS
    auto& physSys = addSystem<PhysicsSystem>();
    addSystem<PlayerSystem>(camSys.flyController(), physSys);
    addSystem<ShootingSystem>(camSys.flyController(), physSys, renderer);
#endif
    addSystem<MotionSystem>();
    addSystem<RenderSystem>();
}

void ArenaState::onEnter(FrameContext& ctx) {
    if (!LevelLoader::load(levelFile, ctx.world, arenaRenderer, ctx.view)) {
        LOG_ERROR << "Failed to load level: " << levelFile;
    }
    PlayingState::onEnter(ctx);
}
