#ifndef RAYTRACER_ENGINE_PROCGEN_DRAINAGE_H
#define RAYTRACER_ENGINE_PROCGEN_DRAINAGE_H

// THE DRAINAGE FLOOD (ADR-0279, #117 item 21): where the water on a grid goes. A PRIORITY FLOOD from the outlets (the
// grid's edge, the sea) inward: each cell is reached from its lowest open neighbour, its FILLED height the higher of
// its own and that neighbour's plus `risePerMetre` per metre crossed (a tiny rise keeps a filled hollow sloping to its
// spill; 0 keeps a lake level), and it drains into that neighbour. The result is a tree rooted at the outlets, visited
// outlets first. The routing of the mountain-growing erosion (stream power, which then re-routes each cell to its
// steepest or a drawn downhill neighbour) and the terrain maps' drainage (stream power at K = 0). Hydrology's rivers
// and lakes join it at the next river re-bake: the switch moves a few river stretches, so it lands with
// kHydroCarveCodeTag's bump (ADR-0279).

#include <cstdint>
#include <vector>

namespace engine {

struct DrainageFlood {
    std::vector<double> filled;          // the filled surface
    std::vector<int32_t> receiver;       // the cell each drains into (flooded from); -1 at an outlet
    std::vector<double> receiverLength;  // metres to it (the cell, or its diagonal)
    std::vector<int32_t> order;          // outlets first, every cell after its receiver
    std::vector<uint8_t> seen;           // (workspace: reused across floods of one size)
};

// Flood `h` (n x n, row-major, `cell` m apart) from every cell with `outlet` set. Reuses `out`'s storage.
void floodDrainage(const std::vector<double>& h, int n, double cell, const std::vector<uint8_t>& outlet,
                   double risePerMetre, DrainageFlood& out);

}  // namespace engine

#endif
