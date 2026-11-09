#ifndef RAYTRACER_ENGINE_LIGHT_SYSTEM_H
#define RAYTRACER_ENGINE_LIGHT_SYSTEM_H

#include "../system.h"
#include "../event_bus.h"
#include "../lights/light_board.h"
#include "../lights/light_kinds.h"   // LightBook
#include <cstdint>
#include <unordered_map>
#include <vector>

namespace engine {

// THE ONE LIGHT SYSTEM (ADR-0362). Every frame, just before RenderSystem: the street's lamps and the stations' canopy
// lamps (StreetFurniture, RailTracks: static, so held in a grid of cells and only the cells in reach read), and every
// channel the producers published on the board (ctx.view.lights: headlights, the flashlight, rooms, cabs, beacons,
// traffic), picked by pool (light_picker.h) into the frame's lights, which RenderSystem adds to the level's own.
// The budgets follow the renderer: a forward one's 32 (the pools' counts, which the level's light block sets), a
// clustered one's thousands (the pools' `many`); either way the set is trimmed to what the renderer can light, by
// importance. `overlay lamplights off` (LampLightsRequest) empties the street's pool; `[lights]` logs the pick at night.
class LightSystem : public System {
public:
    void onStart(FrameContext& ctx) override;
    void render(FrameContext& ctx) override;

    static constexpr double kCell = 64.0;   // the static lamps' grid cell (m)

private:
    void indexStatics(FrameContext& ctx);

    bool lampLights_ = true;
    EventBus::Subscription onLampLights_;
    std::vector<LightCandidate> cands_;   // reused
    // the static emitters, by cell
    std::vector<LightCandidate> statics_;
    std::unordered_map<uint64_t, std::vector<uint32_t>> cells_;
    std::size_t streetSeen_ = static_cast<std::size_t>(-1), railSeen_ = static_cast<std::size_t>(-1);
    // the book at real candela (the physical sky, ADR-0369), and the loaded book it was made from
    LightBook physicalBook_;
    const LightBook* physicalFrom_ = nullptr;
    std::size_t lastPicked_ = 0, lastCands_ = 0;
    uint64_t frames_ = 0;
};

}  // namespace engine

#endif
