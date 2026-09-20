#ifndef RAYTRACER_ENGINE_PROCGEN_CITY_ROADS_LANES_TERRAIN_CONFORM_H
#define RAYTRACER_ENGINE_PROCGEN_CITY_ROADS_LANES_TERRAIN_CONFORM_H

// Heightfield conform (lab form): cut and fill around every at-grade footprint, terrain
// under bridges left alone. Each grid node has ONE owner — the nearest non-bridge lane or
// road envelope within conformW, ties to the higher rank and to lanes before envelopes —
// and owners apply sequentially on the already-modified grid, so passes compose. The
// target is the same blended deck field the mesh uses. The integration form of this pass
// is a set of TerrainFlatten regions (roadNetConformRegions); see ADR-0083.

#include "engine/procgen/city/roads/lanes/pavement.h"
#include "engine/procgen/city/roads/lanes/terrain_recipe.h"
#include <string>
#include <vector>

namespace engine {
namespace roads::lanes {

struct ExcessSample { Vec2 xy; double excess; int owner; std::string nodeOwner; double nodeDist; };
struct ConformStats { double cutM3 = 0, fillM3 = 0; int samples = 0, above = 0; double maxExcess = -1e300; std::vector<ExcessSample> worst;
                      std::vector<std::string> nodeOwner; std::vector<double> nodeDist; };   // per grid node: who graded it (diagnostic)

void conformGrid(const RoadLabGraph& g, const LaneSet& L, const DeckHeight& H, HeightGrid& grid, ConformStats& stats);
// Deck vertices and centroids where the conformed terrain rises more than 1 cm above the deck.
void terrainVsDeck(const std::vector<Surface>& decks, const HeightGrid& grid, ConformStats& stats);

}  // namespace roads::lanes
}  // namespace engine

#endif
