// A LEVEL WITH A CROWD OF RESIDENTS, LOADED AND BUILT (ADR-0446), and one number over a crowd run's end state: what the
// crowd tests share (test_brain_crowd.cpp on the traffic lab's roads, test_river_town_crowd.cpp on river_town's).
#ifndef RAYTRACER_TESTS_CROWD_TEST_RIG_H
#define RAYTRACER_TESTS_CROWD_TEST_RIG_H

#include "../src/apps/citysim/city_brain.h"
#include "../src/apps/citysim/city_render.h"
#include "../src/engine/asset_manager.h"
#include "../src/engine/components.h"
#include "../src/engine/level_loader.h"
#include "../src/engine/mesh_uploader.h"
#include "../src/engine/world.h"
#include "../src/hash.h"
#include "../src/renderer/renderer.h"

#include <cmath>
#include <cstdint>
#include <memory>
#include <string>

namespace crowdtest {

// The level `level` (a file under assets/levels) through the real LevelLoader and CityRenderSystem, a NullRenderer
// underneath.
struct LoadedCrowd {
    std::unique_ptr<engine::Renderer> renderer = engine::Renderer::create();
    std::unique_ptr<engine::RendererMeshUploader> uploader;
    std::unique_ptr<engine::AssetManager> assets;
    engine::World world;
    engine::RenderView view;
    citysim::CityRenderSystem city;
    bool ok = false;
    explicit LoadedCrowd(const std::string& level) {
        uploader = std::make_unique<engine::RendererMeshUploader>(*renderer);
        assets = std::make_unique<engine::AssetManager>(*uploader);
        if (!engine::LevelLoader::load(std::string(RT_SOURCE_DIR) + "/assets/levels/" + level, world, *renderer, view,
                                       *assets, false))
            return;
        ok = city.build(world, assets.get(), nullptr);
    }
};

// One number over the run's end state: the driving's own state hash, then every resident's brain record byte for byte
// and their place to the centimetre, then every household car's place to the centimetre.
inline uint64_t crowdHash(const citysim::CityBrain& b, const citysim::CitySim& sim, engine::World& world) {
    uint64_t h = sim.drivingV2()->stateHash();
    const auto mix = [&](uint32_t v) { h = h * 1099511628211ull ^ engine::hashPair(static_cast<uint32_t>(h >> 32), v); };
    for (int who = 0; who < b.residentCount(); ++who) {
        const citysim::brain::AgentBrain& a = b.machine()->agent(static_cast<citysim::brain::AgentId>(who));
        const unsigned char* bytes = reinterpret_cast<const unsigned char*>(&a);
        for (std::size_t i = 0; i < sizeof(a); i += 4) {
            uint32_t w = 0;
            for (std::size_t k = 0; k < 4 && i + k < sizeof(a); ++k) w |= static_cast<uint32_t>(bytes[i + k]) << (8 * k);
            mix(w);
        }
        const engine::Vec2 p = sim.agents()[static_cast<std::size_t>(b.residentAgent(who))].pos;
        mix(static_cast<uint32_t>(static_cast<int32_t>(std::lround(p.x * 100))));
        mix(static_cast<uint32_t>(static_cast<int32_t>(std::lround(p.y * 100))));
    }
    for (const int car : b.cars()) {
        const engine::Transform* t = world.get<engine::Transform>(b.carEntity(car));
        if (!t) continue;
        mix(static_cast<uint32_t>(static_cast<int32_t>(std::lround(t->position.x * 100))));
        mix(static_cast<uint32_t>(static_cast<int32_t>(std::lround(t->position.z * 100))));
    }
    return h;
}

}  // namespace crowdtest

#endif
