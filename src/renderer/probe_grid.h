#ifndef RAYTRACER_RENDERER_PROBE_GRID_H
#define RAYTRACER_RENDERER_PROBE_GRID_H

// WHICH PROBE A PIXEL READS (ADR-0374). The street's probes overlap and there are 64 of them; a pixel testing every
// box cost the scene pass 7.6 ms on river_town. They stand in a plan, so a plan grid finds them: `kSize` x `kSize`
// cells over the probes' boxes, each listing the two probes nearest its middle whose boxes reach it. A pixel reads
// its cell and tests those two boxes (height too), the first that holds it wins. Built when the probes change, not
// per frame. Pure (CPU), so the tests can hold it; the renderer uploads it after the probes.

#include "renderer.h"   // ReflectionProbe
#include <cstdint>
#include <vector>

namespace engine {

struct ProbeGrid {
    static constexpr int kSize = 64;
    float originX = 0, originZ = 0;   // the grid's min corner
    float cell = 1;                   // m a side
    // per cell: (first probe + 1) | (second probe + 1) << 8; 0 none
    std::vector<uint32_t> cells = std::vector<uint32_t>(kSize * kSize, 0u);
};

// `probes`' boxes into the grid (an inside-out box, an empty slot, is left out)
void buildProbeGrid(const std::vector<ReflectionProbe>& probes, ProbeGrid& out);

}  // namespace engine

#endif
