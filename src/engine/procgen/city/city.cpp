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

// A flat road ribbon for one edge: a quad of `width` centred on the centreline,
// at ground + a small bias so it sits above the ground plane.
void emitRoad(RenderMesh& mesh, const Vec2& a, const Vec2& b, Real width, Real y) {
    Vec2 dir = b - a;
    Real len = dir.length();
    if (len < 1e-4) return;
    dir = dir / len;
    Vec2 n = perp(dir) * (width * 0.5);
    Vec3 col(0.13, 0.13, 0.14);
    Vec3 normal(0, 1, 0);
    Vec3 p0(a.x - n.x, y, a.y - n.y), p1(a.x + n.x, y, a.y + n.y);
    Vec3 p2(b.x + n.x, y, b.y + n.y), p3(b.x - n.x, y, b.y - n.y);
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

    // Road surface.
    for (const RoadEdge& e : graph.edges)
        emitRoad(model.roads, graph.nodes[e.a].pos, graph.nodes[e.b].pos,
                 e.width, cp.baseY + 0.03);

    Rng rng(cp.seed);

    // 4. Per block: inset to the buildable footprint, subdivide into lots, grow a
    //    building per occupied lot (ADR-0038 §3).
    for (int bi = 0; bi < model.blockCount; ++bi) {
        const Poly2& block = model.blocks[bi];
        Vec2 c = centroid(block);
        District dist = districtAt(cp, c, bi);
        if (dist == District::Park) continue;

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
            Scope scope = scopeFromFootprint(site, cp.baseY, 10.0 /*unused*/);
            BuildingMesh bm = growBuilding(scope, bp);

            for (const RenderMesh& part : bm.parts) {
                int mi = part.materialIndex;
                if (mi < 0 || mi >= static_cast<int>(model.parts.size())) continue;
                MeshBuilder::append(model.parts[mi], part);
            }
            model.buildings.push_back({centroid(site), bm.height, dist});
        }
    }

    // Ground plane under the whole city.
    Real g = cp.extent + cp.cellSize;
    model.ground = MeshBuilder::plane(static_cast<float>(g * 2), static_cast<float>(g * 2));
    MeshBuilder::transform(model.ground, Mat4::translate(cp.center.x, cp.baseY, cp.center.y));
    for (Vertex& v : model.ground.vertices) v.color = Vec3(0.22, 0.23, 0.20);

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
