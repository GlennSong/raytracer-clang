#include "engine/city_grow.h"

#include "engine/script_assets.h"
#include "log.h"

#ifdef RT_ROADS_LANES
#include "engine/procgen/city/roads/lanes/block_audit.h"
#endif

namespace engine {

std::vector<Poly2> cityBlocksFromHoles(const std::vector<Poly2>& holes, const std::vector<std::vector<Vec2>>* water) {
#ifdef RT_ROADS_LANES
    // The holes stop at the back of the DRAWN sidewalk (pavementHoles), so a block begins
    // just behind it — not `citysim.sidewalk` further in, which left a grass strip.
    return roads::lanes::blocksFromHoles(holes, 1.5, roads::lanes::kBlockMarginBehindSidewalk,
                                         roads::lanes::kMinBlockWidth, water);
#else
    (void)holes;
    (void)water;
    return {};
#endif
}

std::vector<std::string> cityGrowScriptFiles(const std::string& levelDir) {
    std::vector<std::string> out;
    for (const char* name : {"style_book.lua", "archetype_book.lua"})
        if (const std::string p = resolveScriptPath(name, levelDir); !p.empty()) out.push_back(p);
    return out;
}

NetLotResult growCity(const CityGrowInputs& in, LotGrowSetup* setupOut) {
    static const std::vector<RoadEntity> kNoNets;
    const std::vector<RoadEntity>& nets = in.nets ? *in.nets : kNoNets;
    LotGrowSetup s = lotGrowSetupForLevel(in.citysim, in.levelDir, in.padGround, nets, in.groundWith,
                                          in.groundMeshCell, in.enterableAt);
    NetLotResult r;

    const bool built = (in.blocks && !in.blocks->empty()) || (in.holes && !in.holes->empty());
    if (built) {
        // A WHOLE-CITY BUILDER'S BLOCKS. They are exact to the kerb and already inset by the
        // sidewalk (Clipper): the same parceller and grammar, no road graph to derive faces
        // from, and no miter inset to reject them.
        const std::vector<Poly2> derived = in.blocks ? std::vector<Poly2>() : cityBlocksFromHoles(*in.holes, in.water);
        const std::vector<Poly2>& blocks = in.blocks ? *in.blocks : derived;
        s.lp.roadMargin = 0;   // the block already begins behind the drawn sidewalk
        s.lp.smoothGround = in.smoothGround;
        s.lp.streetHeight = in.streetHeight;
#ifdef RT_ROADS_LANES
        s.lp.sidewalkRise = roads::lanes::lanesSidewalkRise();   // paving meets the drawn sidewalk (ADR-0086)
        s.lp.padFeatherInside = static_cast<Real>(roads::lanes::lanesPadFalloff(in.citysim.value("sidewalk", 4.0)));
#endif
        // The width of the pavement beside them, which the lattice gets from its own net's
        // look and a whole-city builder has to be asked for: the lot pass reaches its paved
        // plates out to the band and walks every door to it, and a 0 left both doing nothing.
        if (s.lp.sidewalkWidth <= 0 && in.pavedSidewalk > 0)
            s.lp.sidewalkWidth = static_cast<Real>(in.pavedSidewalk);
        // THE STREETS, so a door knows which way to face: growLotBuildings aims each
        // building's faceDir at the nearest point on this graph.
        const RoadGraph* lotRoads = (in.streets && !in.streets->edges.empty()) ? in.streets : nullptr;
        // ...but NOT the lattice's clearance. `roadClear` is sidewalk + 0.6 m from each
        // centreline's half-width — the lattice's way of keeping a building off a pavement
        // its blocks do not know about. These blocks are cut from the BUILT pavement, so the
        // pavement is already outside them and that clearance counted it twice: measured on
        // metro_lanes, 1082 buildings with 0 extra against 977 with it.
        constexpr Real kBuiltPavementClear = 0;
        r.lots = growLotBuildings(blocks, s.lp, &r.plan, s.planOnly ? nullptr : &r.parts, lotRoads,
                                  kBuiltPavementClear, (s.wantFlat && !s.planOnly) ? &r.flatParts : nullptr,
                                  &r.gradeFlatten);
        LOG_INFO << "[citylots] lots on " << blocks.size() << " built-pavement blocks: " << r.lots.size()
                 << " buildings, " << r.plan.lots.size() << " lots";
    } else {
        r = growLotBuildingsOnNets(nets, s.lp, s.ep, s.roadClear, in.netGround, in.freewayROW, s.wantFlat,
                                   !s.planOnly);
    }
    if (setupOut) *setupOut = std::move(s);
    return r;
}

}  // namespace engine
