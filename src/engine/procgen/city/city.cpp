#include "city.h"

#include "parcel.h"
#include "../../mesh_builder.h"
#include <algorithm>
#include <cmath>

namespace engine {
namespace {

struct Rng {
    uint32_t s;
    explicit Rng(uint32_t seed) : s(seed ? seed : 0x1234567u) {}
    uint32_t next() { s ^= s << 13; s ^= s >> 17; s ^= s << 5; return s; }
    Real unit() { return (next() >> 8) * (1.0 / 16777216.0); }
    Real range(Real a, Real b) { return a + (b - a) * unit(); }
    int  irange(int a, int b) { return a + static_cast<int>(unit() * (b - a + 1)); }
};

uint32_t hash2(int a, uint32_t seed) {
    uint32_t h = seed * 2654435761u + static_cast<uint32_t>(a) * 40503u + 0x9e3779b9u;
    h ^= h >> 15; h *= 0x2c1b3c6du; h ^= h >> 12; h *= 0x297a2d39u; h ^= h >> 15;
    return h;
}

// Building parameters drawn for a district (ADR-0038 §7: downtown towers grade
// down to residential; parks build nothing).
// Pick a facade style for a district. The style distribution is the visible half
// of an archetype: high-rise = glass/metal towers, commercial = concrete/glass/
// brick offices, residential = brick/stucco/painted walk-ups, industrial = metal.
FacadeStyle styleForDistrict(District d, Rng& rng) {
    Real r = rng.unit();
    switch (d) {
        case District::HighRise:
            return r < 0.6 ? FacadeStyle::GlassCurtain
                 : (r < 0.85 ? FacadeStyle::Metal : FacadeStyle::Concrete);
        case District::Commercial:
            return r < 0.4 ? FacadeStyle::Concrete
                 : (r < 0.65 ? FacadeStyle::GlassCurtain
                 : (r < 0.9 ? FacadeStyle::Brick : FacadeStyle::Stucco));
        case District::Industrial:
            return FacadeStyle::Metal;
        case District::Residential:
        default:
            return r < 0.45 ? FacadeStyle::Brick
                 : (r < 0.75 ? FacadeStyle::Stucco : FacadeStyle::Painted);
    }
}

// The building-archetype library (the "variety of buildings" axis): each district
// draws a building from architecturally-grounded parameters — floor counts and
// heights, facade system, setbacks, massing, ornamentation. This is the C++
// vocabulary the Lua city.lua archetypes mirror (ADR-0028).
BuildingParams paramsForDistrict(District d, Rng& rng, uint32_t seed) {
    BuildingParams p;
    p.seed = rng.next() ^ seed;
    FacadeStyle style = styleForDistrict(d, rng);
    p.wallColor = facadeColor(style, p.seed);
    p.curtainWall = (style == FacadeStyle::GlassCurtain);
    switch (d) {
        case District::HighRise:                 // glass/metal towers
            p.floors = rng.irange(16, 45);
            p.groundRetail = true;
            p.floorHeight = rng.range(3.6, 4.0);  // taller commercial floors
            p.bayWidth = rng.range(3.8, 4.8);
            if (p.floors > 22) { p.setbackFloors = rng.irange(7, 11); p.setbackEvery = rng.range(2.0, 4.5); }
            break;
        case District::Commercial:               // office mid-rises
            p.floors = rng.irange(5, 13);
            p.groundRetail = true;
            p.floorHeight = rng.range(3.4, 3.8);
            p.bayWidth = rng.range(3.2, 4.2);
            break;
        case District::Industrial:               // low, wide metal warehouses
            p.floors = rng.irange(1, 2);
            p.groundRetail = false;
            p.solidFacade = true;
            p.groundHeight = rng.range(6.0, 9.0);  // tall single volume
            p.floorHeight = rng.range(4.5, 6.0);
            p.bayWidth = rng.range(7.0, 11.0);     // few big bays
            break;
        case District::Residential:
        default:                                 // brick/stucco walk-ups
            p.floors = rng.irange(2, 5);
            p.groundRetail = (rng.unit() < 0.25);  // occasional corner shop
            p.floorHeight = rng.range(3.0, 3.4);
            p.bayWidth = rng.range(3.0, 3.6);
            break;
    }

    // Massing variety (not every building is a box): some high-rise towers are
    // round (curved glass), and an "old town" pocket builds tiered pagodas.
    if (d == District::HighRise && p.curtainWall && rng.unit() < 0.35) {
        p.shape = BuildingShape::Cylinder;
        p.sides = 36;
    }

    // Ornamentation by archetype: a glass curtain wall and a metal shed stay
    // clean; traditional masonry gets a base course, a ground-floor cornice, an
    // awning, and — on shorter brick/concrete buildings — pilasters.
    bool plain = p.curtainWall || p.solidFacade;
    p.baseCourse = !p.solidFacade;
    p.stringCourse = !plain;
    p.awning = !plain;
    p.pilasters = !plain && p.floors <= 12 &&
                  (style == FacadeStyle::Brick || style == FacadeStyle::Concrete);
    p.parapet = p.solidFacade ? 0.0 : human::PARAPET;
    p.trimColor = plain ? Vec3(0.50, 0.52, 0.55)
                : (style == FacadeStyle::Brick ? Vec3(0.84, 0.82, 0.76)
                                               : Vec3(0.78, 0.77, 0.73));
    return p;
}

// A terrain-conforming ribbon along an edge: subdivided into ~7 m segments, each
// of the four corners sampled to the ground height there (+ yBias). The road thus
// undulates over hills *and* banks across cross-slopes — flat where the ground is
// flat (NYC), rolling up/down steep inclines where it isn't (San Francisco).
// Per-segment normals so steep roads shade correctly.
void emitRibbon(RenderMesh& mesh, const CityParams& cp, const Vec2& a, const Vec2& b,
                Real width, const Vec3& col, Real yBias) {
    Vec2 dir = b - a;
    Real len = dir.length();
    if (len < 1e-4) return;
    dir = dir / len;
    Vec2 half = perp(dir) * (width * 0.5);
    int segs = std::max(1, static_cast<int>(std::ceil(len / 7.0)));
    auto corner = [&](const Vec2& c, Real side) {
        Vec2 p = c + half * side;
        return Vec3(p.x, cityGroundAt(cp, p) + yBias, p.y);
    };
    for (int i = 0; i < segs; ++i) {
        Vec2 c0 = lerp(a, b, static_cast<Real>(i) / segs);
        Vec2 c1 = lerp(a, b, static_cast<Real>(i + 1) / segs);
        Vec3 l0 = corner(c0, -1), r0 = corner(c0, 1);
        Vec3 r1 = corner(c1, 1), l1 = corner(c1, -1);
        Vec3 nrm = normalize(cross(r0 - l0, l1 - l0));
        if (nrm.y < 0) nrm = nrm * -1;
        uint32_t base = static_cast<uint32_t>(mesh.vertices.size());
        auto v = [&](const Vec3& p) {
            Vertex vert(p, nrm, Vec3(dir.x, 0, dir.y), 0, 0); vert.color = col; return vert;
        };
        mesh.vertices.push_back(v(l0));
        mesh.vertices.push_back(v(r0));
        mesh.vertices.push_back(v(r1));
        mesh.vertices.push_back(v(l1));
        mesh.indices.insert(mesh.indices.end(),
                            {base, base + 1, base + 2, base, base + 2, base + 3});
    }
}

// A cheap stylized tree (trunk cylinder + two foliage cones), vertex-coloured so
// it bakes on a white material like the rest of the city. Planted at `base` (its
// foot), facing +Y. ~80 triangles; for street/park scatter, not a hero asset.
void emitTree(RenderMesh& out, const Vec3& base, Real height, Real canopy,
              uint32_t seed) {
    Vec3 trunkCol(0.32, 0.22, 0.14), leafCol(0.20, 0.42, 0.16);
    Real trunkH = height * 0.45, trunkR = height * 0.04;
    auto place = [&](RenderMesh m, const Vec3& col, Real y) {
        for (Vertex& v : m.vertices) v.color = col;
        MeshBuilder::transform(m, Mat4::translate(base.x, base.y + y, base.z));
        MeshBuilder::append(out, m);
    };
    // Cylinder + cone are centre-origin (y from -h/2..+h/2); translate by half
    // their height so the foot/base lands where intended.
    place(MeshBuilder::cylinder(static_cast<float>(trunkR),
                                static_cast<float>(trunkH), 7), trunkCol, trunkH * 0.5);
    Real leafSeed = (seed % 7) * 0.03;
    Real h0 = canopy * 0.9, h1 = canopy * 0.7;
    place(MeshBuilder::cone(static_cast<float>(canopy * (0.62 + leafSeed)),
                            static_cast<float>(h0), 8), leafCol, trunkH + h0 * 0.5);
    place(MeshBuilder::cone(static_cast<float>(canopy * (0.42 + leafSeed)),
                            static_cast<float>(h1), 8), leafCol,
          trunkH + canopy * 0.4 + h1 * 0.5);
}

}  // namespace

District districtAt(const CityParams& params, const Vec2& p, int blockIndex) {
    if (hash2(blockIndex, params.seed) % 1000 < params.parkFraction * 1000)
        return District::Park;
    Real d = (p - params.center).length();
    if (d < params.downtownRadius) return District::HighRise;
    if (d < params.midtownRadius) return District::Commercial;
    // Outer ring: an industrial zone clustered on the west edge (a believable
    // "area"), the rest residential. Coastal would key off a shoreline field.
    if (p.x - params.center.x < -0.28 * params.extent) return District::Industrial;
    return District::Residential;
}

CityModel generateCity(const CityParams& cp) {
    CityModel model;
    model.parts.resize(static_cast<std::size_t>(PartId::Count));
    for (int i = 0; i < static_cast<int>(PartId::Count); ++i)
        model.parts[i].materialIndex = i;

    // 1. Roads -> 2. planarize -> 3. block faces.
    GridRoadParams gp;
    gp.center = cp.center; gp.extent = cp.extent; gp.cellSize = cp.cellSize;
    gp.jitter = cp.roadJitter; gp.seed = cp.seed;
    RoadGraph graph = planarize(gridRoads(gp));
    model.blocks = extractBlocks(graph);
    model.blockCount = static_cast<int>(model.blocks.size());

    // Pave the street corridor (ADR-0038 §3.5): a light **sidewalk** ribbon
    // filling the gap between blocks (so the inter-block space reads as paved
    // city, not grass), then the dark **asphalt** carriageway on top. Both drape
    // on the ground. Parks (handled below) overlay green on their block.
    Real corridor = (8.0 + cp.sidewalk) * 2.0;     // ~ block-to-block gap
    Vec3 sidewalkCol(0.50, 0.50, 0.49), asphaltCol(0.13, 0.13, 0.14);
    for (const RoadEdge& e : graph.edges) {
        emitRibbon(model.roads, cp, graph.nodes[e.a].pos, graph.nodes[e.b].pos,
                   corridor, sidewalkCol, 0.03);
        emitRibbon(model.roads, cp, graph.nodes[e.a].pos, graph.nodes[e.b].pos,
                   e.width, asphaltCol, 0.06);
    }

    Rng rng(cp.seed);

    // 4. Per block: inset to the buildable footprint, subdivide into lots, grow a
    //    building per occupied lot (ADR-0038 §3).
    for (int bi = 0; bi < model.blockCount; ++bi) {
        const Poly2& block = model.blocks[bi];
        Vec2 c = centroid(block);
        District dist = districtAt(cp, c, bi);
        if (dist == District::Park) {
            // Parks get trees instead of buildings (ADR-0038 §3.5).
            if (cp.scatterTrees) {
                Poly2 green = inset(block, 6.0);
                if (green.size() >= 3) {
                    Rng prng(hash2(bi, cp.seed ^ 0x70a7u));
                    int n = std::max(2, static_cast<int>(area(green) / 140.0));
                    Vec2 lo, hi; bounds(green, lo, hi);
                    for (int k = 0; k < n * 3 && model.treeCount < 100000; ++k) {
                        Vec2 p(prng.range(lo.x, hi.x), prng.range(lo.y, hi.y));
                        if (!pointInPolygon(green, p)) continue;
                        Real h = prng.range(5.0, 8.5);
                        emitTree(model.props, Vec3(p.x, cityGroundAt(cp, p), p.y), h,
                                 h * 0.45, prng.next());
                        ++model.treeCount;
                    }
                }
            }
            continue;
        }

        // "Old town" pocket (off-centre, low-rise): builds tiered pagodas — a
        // clustered East-Asian quarter rather than scattering them at random.
        Vec2 oldTownC = cp.center + Vec2(cp.extent * 0.42, -cp.extent * 0.34);
        bool oldTown = dist != District::HighRise && dist != District::Industrial &&
                       (c - oldTownC).length() < cp.extent * 0.24;

        Real roadInset = 8.0 + cp.sidewalk;   // half a local road + sidewalk
        Poly2 foot = inset(block, roadInset);
        if (foot.size() < 3 || area(foot) < 60) continue;

        ParcelParams pp;
        pp.seed = hash2(bi, cp.seed ^ 0xabcdu);
        pp.targetArea = (dist == District::HighRise) ? 900
                      : (dist == District::Commercial) ? 520
                      : (dist == District::Industrial) ? 1500   // big warehouse lots
                      : 340;
        pp.minArea = (dist == District::HighRise) ? 280
                   : (dist == District::Industrial) ? 600 : 110;
        std::vector<Lot> lots = subdivideBlock(foot, pp, static_cast<int>(dist));
        model.lotCount += static_cast<int>(lots.size());

        for (const Lot& lot : lots) {
            if (rng.unit() > cp.buildChance) continue;        // plaza / empty
            if (lot.area < 50) continue;
            // Pull the building in from the lot lines a little (setback to the lot).
            Poly2 site = inset(lot.footprint, 1.2);
            if (site.size() < 3 || area(site) < 30) site = lot.footprint;

            BuildingParams bp = paramsForDistrict(dist, rng, cp.seed);
            if (oldTown) {
                bp.shape = BuildingShape::Pagoda;
                bp.tiers = 3 + 2 * static_cast<int>(rng.next() % 3);   // 3 / 5 / 7
                bp.floorHeight = rng.range(3.0, 3.8);
                bp.curtainWall = false; bp.solidFacade = false;
            }
            // Foundation sits at the MIN ground height under the footprint, so a
            // building on a slope never floats (its uphill side is buried, its
            // downhill side flush) — ADR-0038 §3.4.4.
            Real baseY = cityGroundAt(cp, centroid(site));
            for (const Vec2& v : site) baseY = std::min(baseY, cityGroundAt(cp, v));
            Scope scope = scopeFromFootprint(site, baseY, 10.0 /*unused*/);
            BuildingMesh bm = growBuilding(scope, bp);

            for (const RenderMesh& part : bm.parts) {
                int mi = part.materialIndex;
                if (mi < 0 || mi >= static_cast<int>(model.parts.size())) continue;
                MeshBuilder::append(model.parts[mi], part);
            }
            MeshBuilder::append(model.hlodProxy, bm.proxy);   // distant-city LOD
            model.buildings.push_back({centroid(site), baseY, bm.height, dist});
        }
    }

    // Street trees: walk each road and plant a tree on each verge at intervals,
    // just outside the carriageway (in the sidewalk gap, before the building
    // setback), draped on the ground (ADR-0038 §3.5).
    if (cp.scatterTrees) {
        Rng trng(cp.seed ^ 0x57eeu);
        for (const RoadEdge& e : graph.edges) {
            Vec2 a = graph.nodes[e.a].pos, b = graph.nodes[e.b].pos;
            Vec2 d = b - a; Real len = d.length();
            if (len < cp.streetTreeSpacing * 1.4) continue;   // skip short stubs
            d = d / len;
            Vec2 nrm = perp(d);
            Real verge = e.width * 0.5 + 1.8;
            int n = static_cast<int>(len / cp.streetTreeSpacing);
            for (int k = 1; k < n; ++k) {
                Real t = (k + (trng.unit() - 0.5) * 0.3) * cp.streetTreeSpacing;
                Vec2 on = a + d * t;
                for (Real s : {Real(1), Real(-1)}) {
                    if (trng.unit() < 0.25) continue;        // gappy, not a hedge
                    Vec2 p = on + nrm * (verge * s);
                    Real h = trng.range(4.5, 7.0);
                    emitTree(model.props, Vec3(p.x, cityGroundAt(cp, p), p.y), h,
                             h * 0.4, trng.next());
                    ++model.treeCount;
                }
            }
        }
    }

    // Ground plane under the whole city — only when flat (on terrain, the terrain
    // mesh is the ground; the city drapes onto it).
    if (!cp.groundAt) {
        Real g = cp.extent + cp.cellSize;
        model.ground = MeshBuilder::plane(static_cast<float>(g * 2), static_cast<float>(g * 2));
        MeshBuilder::transform(model.ground, Mat4::translate(cp.center.x, cp.baseY, cp.center.y));
        for (Vertex& v : model.ground.vertices) v.color = Vec3(0.22, 0.23, 0.20);
    }

    // Drop empty parts so consumers don't bind material slots with no geometry.
    model.parts.erase(
        std::remove_if(model.parts.begin(), model.parts.end(),
                       [](const RenderMesh& m) { return m.vertices.empty(); }),
        model.parts.end());
    return model;
}

RenderMesh CityModel::mergedBuildings() const {
    RenderMesh m;
    for (const RenderMesh& p : parts) MeshBuilder::append(m, p);
    return m;
}

}  // namespace engine
