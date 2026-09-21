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
        RoadMesh m;
        m.mesh = buildRoadNetMesh(*in.road, in.ground, &p.bands, &p.deck);
        m.name = "road";
        m.albedo = Vec3(1, 1, 1);        // the hue rides in the vertex colour
        m.roughness = 0.93f;
        m.markings = in.road->look.markings;
        p.meshes.push_back(std::move(m));
        return p;
    }

    // The lattice never makes a ground — it only ever describes edits to the
    // level's own terrain.
    GroundPlan ground(const RoadBuildInput& in) const override {
        GroundPlan g;
        if (in.road) g.carve = roadNetConformRegions(*in.road, in.ground);
        return g;
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
