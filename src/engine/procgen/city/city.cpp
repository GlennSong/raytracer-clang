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
BuildingParams paramsForDistrict(District d, Rng& rng, uint32_t seed) {
    BuildingParams p;
    p.seed = rng.next() ^ seed;
    switch (d) {
        case District::Downtown:
            p.floors = rng.irange(12, 38);
            p.groundRetail = true;
            p.bayWidth = rng.range(3.6, 4.6);
            if (p.floors > 20) { p.setbackFloors = 8; p.setbackEvery = rng.range(2.0, 4.0); }
            p.wallColor = Vec3(0.62, 0.64, 0.68);
            break;
        case District::Midtown:
            p.floors = rng.irange(4, 9);
            p.groundRetail = true;
            p.bayWidth = rng.range(3.2, 4.0);
            p.wallColor = Vec3(0.70, 0.66, 0.60);
            break;
        case District::Residential:
        default:
            p.floors = rng.irange(1, 3);
            p.groundRetail = false;
            p.bayWidth = rng.range(3.0, 3.6);
            p.wallColor = Vec3(0.74, 0.71, 0.64);
            break;
    }
    return p;
}

// A road ribbon for one edge: a quad of `width` centred on the centreline, with
// each end draped to the ground height there (+ a small bias). Flat terrain ->
// a flat ribbon; rolling terrain -> the road follows it (gentle slopes; a long
// road over a big hill would need subdivision, out of scope).
void emitRoad(RenderMesh& mesh, const CityParams& cp, const Vec2& a, const Vec2& b,
              Real width) {
    Vec2 dir = b - a;
    Real len = dir.length();
    if (len < 1e-4) return;
    dir = dir / len;
    Vec2 n = perp(dir) * (width * 0.5);
    Real ya = cityGroundAt(cp, a) + 0.05, yb = cityGroundAt(cp, b) + 0.05;
    Vec3 col(0.13, 0.13, 0.14);
    Vec3 normal(0, 1, 0);
    Vec3 p0(a.x - n.x, ya, a.y - n.y), p1(a.x + n.x, ya, a.y + n.y);
    Vec3 p2(b.x + n.x, yb, b.y + n.y), p3(b.x - n.x, yb, b.y - n.y);
    uint32_t base = static_cast<uint32_t>(mesh.vertices.size());
    auto v = [&](const Vec3& p, float u, float vv) {
        Vertex vert(p, normal, Vec3(dir.x, 0, dir.y), u, vv); vert.color = col; return vert;
    };
    mesh.vertices.push_back(v(p0, 0, 0));
    mesh.vertices.push_back(v(p1, 1, 0));
    mesh.vertices.push_back(v(p2, 1, 1));
    mesh.vertices.push_back(v(p3, 0, 1));
    mesh.indices.insert(mesh.indices.end(),
                        {base, base + 1, base + 2, base, base + 2, base + 3});
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
    if (d < params.downtownRadius) return District::Downtown;
    if (d < params.midtownRadius) return District::Midtown;
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

    // Road surface (draped on the ground).
    for (const RoadEdge& e : graph.edges)
        emitRoad(model.roads, cp, graph.nodes[e.a].pos, graph.nodes[e.b].pos, e.width);

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

        Real roadInset = 8.0 + cp.sidewalk;   // half a local road + sidewalk
        Poly2 foot = inset(block, roadInset);
        if (foot.size() < 3 || area(foot) < 60) continue;

        ParcelParams pp;
        pp.seed = hash2(bi, cp.seed ^ 0xabcdu);
        pp.targetArea = (dist == District::Downtown) ? 900 : (dist == District::Midtown ? 520 : 360);
        pp.minArea = (dist == District::Downtown) ? 280 : 110;
        std::vector<Lot> lots = subdivideBlock(foot, pp, static_cast<int>(dist));
        model.lotCount += static_cast<int>(lots.size());

        for (const Lot& lot : lots) {
            if (rng.unit() > cp.buildChance) continue;        // plaza / empty
            if (lot.area < 50) continue;
            // Pull the building in from the lot lines a little (setback to the lot).
            Poly2 site = inset(lot.footprint, 1.2);
            if (site.size() < 3 || area(site) < 30) site = lot.footprint;

            BuildingParams bp = paramsForDistrict(dist, rng, cp.seed);
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
