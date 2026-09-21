// The LANES builder behind the roads module's interface (ADR-0089).
//
// The lattice meshes a road entity on the spot. Lanes cannot: a lane-built city is
// produced once per LEVEL, through the bundle (ADR-0084), because it is expensive
// and because the loader, rt_bake and the editor's bake button must all get the
// same city to the byte. So this builder does not build — it OBTAINS, and hands
// the three products out of what the city producer made: the conformed ground, the
// cell meshes, and the class-faithful twin that everything routes on.
//
// That is also why it needs the level rather than just the road entity.
#include "road_builder.h"

#include "lanes/city_producer.h"
#include "lanes/lots_producer.h"

#include <memory>
#include "../../../bundle/bake.h"
#include "../../../bundle/codecs.h"
#include "../../../../log.h"

#include <map>
#include <mutex>
#include <optional>

namespace engine::roads {

namespace {

class LaneRoadBuilder final : public RoadBuilder {
public:
    const char* name() const override { return "lanes"; }

    GroundPlan ground(const RoadBuildInput& in) const override {
        GroundPlan g;
        const lanes::CityProducts* p = products(in);
        if (!p || !p->hasTerrain) return g;      // nothing to hand over: the level keeps its own
        auto grid = std::make_shared<GroundGrid>();
        grid->x0 = p->ground.x0; grid->y0 = p->ground.y0; grid->res = p->ground.res;
        grid->nx = p->ground.nx; grid->ny = p->ground.ny; grid->z = p->ground.z;
        g.replace = grid;
        return g;
    }

    RoadProducts build(const RoadBuildInput& in) const override {
        RoadProducts out;
        const lanes::CityProducts* p = products(in);
        if (!p) return out;
        out.row = p->row;                              // the lot pass keeps out of the freeway
        for (const lanes::Ring& h : p->holes) out.holes.push_back(h);   // un-inset: the caller insets by its own sidewalk
        for (const lanes::CityCellMesh& c : p->cells) {
            const bundle::PackedMesh& pm = c.mesh;
            if (pm.vertexCount() == 0 || pm.idx.empty()) continue;
            // The level's own terrain wins over the lab's flat copy of it: drawing
            // both z-fights and costs 400k triangles.
            if (pm.name == "terrain" && in.level.contains("terrain")) continue;
            RoadMesh m;
            m.mesh = bundle::unpackMesh(pm);
            m.name = pm.name;
            m.albedo = Vec3(pm.albedo[0], pm.albedo[1], pm.albedo[2]);
            m.roughness = pm.roughness;
            if (pm.name == "guardrail") m.metallic = 0.18f;   // weathered galvanising, not a mirror
            m.collidable = !pm.paint() && pm.collidable();
            m.friction = pm.friction;
            m.cx = c.cx; m.cz = c.cz;
            out.meshes.push_back(std::move(m));
        }
        return out;
    }

    // What everything routes on: the twin derived from the built pavement, not from
    // the plan that produced it (the pavement is the source of truth).
    RoadGraph navGraph(const RoadBuildInput& in) const override {
        const lanes::CityProducts* p = products(in);
        return p ? p->nav : RoadGraph{};
    }
    bool ownsNavGraph() const override { return true; }

private:
    // One obtain per level, not one per question: ground(), build() and navGraph()
    // are three views of the same city.
    const lanes::CityProducts* products(const RoadBuildInput& in) const {
        // A caller working from the level's JSON ALONE has no level to obtain against
        // (the terrain pre-pass, cityPrePassForLevel). It gets nothing, by design and
        // without a complaint: a lane-built city's ground reaches the terrain through the
        // level's bundle before that pre-pass runs, not through this call.
        if (in.levelPath.empty()) return nullptr;
        // WHICH city: the ordinal names a section of the bundle and counts CITY entities,
        // which is not the same as counting road entities. An entity the level does not
        // call a city has none, and says so once.
        const int ordinal = in.entityIndex >= 0
                                ? lanes::cityOrdinalForEntity(in.level, in.entityIndex)
                                : in.ordinal;
        std::lock_guard<std::mutex> lock(mu_);
        // ONE level's products at a time. A city is hundreds of megabytes of packed cell
        // meshes; holding the last three levels the viewer visited would be a leak that
        // looks like a cache. Every entity of a level loads together, so this never
        // thrashes.
        if (in.levelPath != level_) { cache_.clear(); level_ = in.levelPath; }
        const std::string key = std::to_string(ordinal);
        auto it = cache_.find(key);
        if (it != cache_.end()) return it->second ? &*it->second : nullptr;
        auto& slot = cache_[key];
        if (ordinal < 0) {
            LOG_ERROR << "[roads/lanes] " << in.levelPath << " entity " << in.entityIndex
                      << " asks for the lanes builder but is not one of the level's city entities";
            return nullptr;
        }
        bundle::LevelInputs li;
        li.levelPath = in.levelPath;
        li.level = in.level;
        const std::size_t slash = in.levelPath.find_last_of('/');
        li.levelDir = slash == std::string::npos ? "." : in.levelPath.substr(0, slash);
        std::string status;
        // One obtain per level per process — the terrain pre-pass asks for the same
        // city, and under RT_NOCACHE a second obtain would BUILD it a second time.
        std::shared_ptr<bundle::Bundle> b = lanes::levelCityBundle(li, &status);
        if (!b) {
            LOG_ERROR << "[roads/lanes] no city products for " << in.levelPath << ": " << status;
            return nullptr;
        }
        lanes::CityProducts p;
        std::string err;
        if (!lanes::readCityProducts(*b, ordinal, p, &err)) {
            LOG_ERROR << "[roads/lanes] " << err;
            return nullptr;
        }
        LOG_INFO << "[roads/lanes] " << in.levelPath << " e" << ordinal << ": " << p.cells.size()
                 << " cell meshes, " << p.nav.edges.size() << " nav edges, " << p.holes.size()
                 << " pavement holes, bundle " << status;
        slot = std::move(p);
        return &*slot;
    }

    mutable std::mutex mu_;
    mutable std::string level_;      // which level cache_ holds
    mutable std::map<std::string, std::optional<lanes::CityProducts>> cache_;
};

}  // namespace

void registerLanesRoadBuilder() {
    registerRoadBuilder(std::make_unique<LaneRoadBuilder>());
}

}  // namespace engine::roads
