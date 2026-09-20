#include "road_builder.h"

#include "../../deprecated/roads/road_net_mesh.h"   // the DEPRECATED lattice

namespace engine::roads {

namespace {

// The lattice builder (roads-v2 .. 2026-09): sweep each chain of the sampled,
// constrained graph as a lattice body, fill each junction with a Coons patch,
// and band the whole asphalt union with kerb + sidewalk. Every shipped level
// builds its roads this way, so this wrapper is a forward and nothing else —
// the same three calls the loader used to make itself, in the same order, with
// the same arguments. When a level can say `"builder": "lanes"` and mean it,
// this and procgen/deprecated/roads go together.
class LatticeRoadBuilder final : public RoadBuilder {
public:
    const char* name() const override { return "lattice"; }

    RoadProducts build(const RoadBuildInput& in) const override {
        RoadProducts p;
        if (!in.road) return p;
        p.mesh = buildRoadNetMesh(*in.road, in.ground, &p.bands, &p.deck);
        return p;
    }

    std::vector<TerrainFlatten> flattens(const RoadBuildInput& in) const override {
        if (!in.road) return {};
        return roadNetConformRegions(*in.road, in.ground);
    }

    RoadGraph navGraph(const RoadBuildInput& in) const override {
        if (!in.road) return {};
        return navRoadGraph(*in.road, in.ground);
    }
};

}  // namespace

void registerLatticeRoadBuilder() {
    registerRoadBuilder(std::make_unique<LatticeRoadBuilder>());
}

}  // namespace engine::roads
