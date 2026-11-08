#ifndef RAYTRACER_ENGINE_PROBE_SYSTEM_H
#define RAYTRACER_ENGINE_PROBE_SYSTEM_H

#include "../system.h"
#include "../lights/probe_sites.h"
#include "../procgen/city/shape_grammar.h"   // StoreyPlan
#include <cstddef>
#include <cstdint>
#include <deque>
#include <vector>

namespace engine {

// THE REFLECTION PROBES' PLACES AND SCHEDULE (ADR-0374). Once a level's roads are in, the sites along every street
// (probe_sites.h); each frame the nearest 64 of them hold the renderer's slots (a slot keeps its probe while the
// camera stays near), and a probe that has just joined is captured face by face, nearest first, `facesPerFrame` a
// frame, on ctx.view.probes for RenderSystem. The renderer relights them as the light changes (ADR-0373), so a probe
// is captured once per visit, not per frame. Off when the renderer cannot capture, or RT_PROBES=0.
class ProbeSystem : public System {
public:
    void render(FrameContext& ctx) override;

    static constexpr int kSlots = 64;   // the renderer's (VulkanRenderer kMaxProbes)
    static constexpr int kFacesPerFrame = 2;

private:
    void placeSites(FrameContext& ctx);
    void placeStorey(FrameContext& ctx);   // (ADR-0380)

    std::vector<ProbeSite> sites_;   // the street's [0, streetCount_), then the camera's storey's (ADR-0380)
    std::size_t streetCount_ = 0;
    std::size_t roadsSeen_ = static_cast<std::size_t>(-1);
    // the storey the camera stands on: its building (and that building's storey plans) and which storey (0: none)
    const void* storeyRecord_ = nullptr;
    std::vector<StoreyPlan> storeyPlans_;
    uint64_t storeyKey_ = 0;
    std::vector<int> resident_ = std::vector<int>(kSlots, -1);
    struct Pending {
        int slot = 0, site = 0, face = 0;
    };
    std::deque<Pending> pending_;
    Vec3 lastEye_{1e30, 0, 0};
};

}  // namespace engine

#endif
