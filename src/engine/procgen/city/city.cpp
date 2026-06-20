#include "city.h"

#include "parcel.h"
#include "street_kit.h"
#include "../tree.h"
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
// Cladding follows the structural system, which follows height (ADR-0040):
// load-bearing masonry doesn't scale, so ~12 storeys is where the wall stops
// holding the building up and becomes a lightweight curtain wall hung off a
// frame — glass + metal/precast, never brick. Corrugated metal is industrial
// only. So material is chosen by storey count, not the district directly.
FacadeStyle styleForHeight(District d, int floors, Rng& rng) {
    if (d == District::Industrial) return FacadeStyle::Metal;   // corrugated sheds
    Real r = rng.unit();
    if (floors >= 12) {
        // High / super-tall: a glass curtain wall, or a precast/stone-clad
        // concrete frame. No masonry — you can't hang brick this high.
        return r < 0.62 ? FacadeStyle::GlassCurtain : FacadeStyle::Concrete;
    }
    if (floors >= 5) {
        // Mid-rise framed: precast/concrete or masonry infill, the odd glass box.
        return r < 0.42 ? FacadeStyle::Concrete
             : (r < 0.80 ? FacadeStyle::Brick : FacadeStyle::GlassCurtain);
    }
    // Low-rise load-bearing masonry: brick walk-ups, stucco, painted.
    return r < 0.5 ? FacadeStyle::Brick
         : (r < 0.8 ? FacadeStyle::Stucco : FacadeStyle::Painted);
}

// The building-archetype library (the "variety of buildings" axis): each district
// draws a building from architecturally-grounded parameters — floor counts and
// heights, facade system, setbacks, massing, ornamentation. This is the C++
// vocabulary the Lua city.lua archetypes mirror (ADR-0028).
BuildingParams paramsForDistrict(District d, Rng& rng, uint32_t seed) {
    BuildingParams p;
    p.seed = rng.next() ^ seed;
    // Massing first (floor count + dimensions), so cladding can follow height.
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

    // Cladding follows the now-known height (ADR-0040): tall ⇒ glass/precast,
    // short ⇒ masonry, never a 40-storey brick tower.
    FacadeStyle style = styleForHeight(d, p.floors, rng);
    p.wallColor = facadeColor(style, p.seed);
    p.curtainWall = (style == FacadeStyle::GlassCurtain);
    switch (style) {
        case FacadeStyle::Brick:    p.wallPart = PartId::Brick;    break;
        case FacadeStyle::Concrete: p.wallPart = PartId::Concrete; break;
        case FacadeStyle::Stucco:   p.wallPart = PartId::Stucco;   break;
        case FacadeStyle::Metal:    p.wallPart = PartId::Metal;    break;
        default:                    p.wallPart = PartId::Wall;     break;  // Painted/Glass
    }

    // Massing variety (not every building is a box): some high-rise towers are
    // round (curved glass), and an "old town" pocket builds tiered pagodas.
    if (d == District::HighRise && p.curtainWall && rng.unit() < 0.35) {
        p.shape = BuildingShape::Cylinder;
        p.sides = 36;
    }

    // Ornamentation by archetype: a glass curtain wall and a metal shed stay
    // clean; traditional masonry gets a base course, a ground-floor cornice, an
    // awning, and — on shorter masonry buildings — base piers (pilasters live on
    // the base only, ADR-0040; growBuilding caps them with the string course).
    bool plain = p.curtainWall || p.solidFacade;
    p.baseCourse = !p.solidFacade;
    p.stringCourse = !plain;
    p.awning = !plain;
    p.pilasters = !plain && p.floors <= 8 &&
                  (style == FacadeStyle::Brick || style == FacadeStyle::Concrete);
    p.parapet = p.solidFacade ? 0.0 : human::PARAPET;
    p.trimColor = plain ? Vec3(0.50, 0.52, 0.55)
                : (style == FacadeStyle::Brick ? Vec3(0.84, 0.82, 0.76)
                                               : Vec3(0.78, 0.77, 0.73));
    return p;
}

// Engine winding convention: a triangle's geometric normal points OPPOSITE the
// outward shading normal (geo·normal < 0), matching MeshBuilder::box — required
// for the viewer's back-face culling (the offline tracer is two-sided, so it
// never reveals a flipped winding). All hand-wound city surfaces go through these.
void pushTri(RenderMesh& m, const Vec3& a, const Vec3& b, const Vec3& c,
             const Vec3& nrm, const Vec3& col) {
    Vec3 geo = cross(b - a, c - a);
    uint32_t base = static_cast<uint32_t>(m.vertices.size());
    auto v = [&](const Vec3& p) { Vertex vt(p, nrm, Vec3(1, 0, 0), 0, 0); vt.color = col; return vt; };
    if (dot(geo, nrm) <= 0) {
        m.vertices.push_back(v(a)); m.vertices.push_back(v(b)); m.vertices.push_back(v(c));
    } else {
        m.vertices.push_back(v(a)); m.vertices.push_back(v(c)); m.vertices.push_back(v(b));
    }
    m.indices.push_back(base); m.indices.push_back(base + 1); m.indices.push_back(base + 2);
}
void pushQuad(RenderMesh& m, const Vec3& a, const Vec3& b, const Vec3& c,
              const Vec3& d, const Vec3& nrm, const Vec3& col) {
    pushTri(m, a, b, c, nrm, col);
    pushTri(m, a, c, d, nrm, col);
}

// A flat, horizontal paved polygon at height `y` (centroid fan), vertex-coloured.
// The block apron / sidewalk: a real street is graded FLAT, so this does NOT drape
// — the terrain is cut/filled to meet it (emitRetainingSkirt).
void emitFlatPolygon(RenderMesh& mesh, const Poly2& poly, Real y, const Vec3& col) {
    if (poly.size() < 3) return;
    Vec2 c = centroid(poly);
    Vec3 center(c.x, y, c.y), n(0, 1, 0);
    const std::size_t cnt = poly.size();
    for (std::size_t i = 0; i < cnt; ++i) {
        Vec2 a = poly[i], b = poly[(i + 1) % cnt];
        pushTri(mesh, center, Vec3(a.x, y, a.y), Vec3(b.x, y, b.y), n, col);
    }
}

// A retaining wall around a flat pad: a vertical skirt from the pad edge (topY)
// down to the terrain at each boundary vertex (fill where the pad is above grade;
// where terrain is higher we'd cut, but the pad is graded above terrain so the
// skirt only ever fills down). This is the curb/retaining edge that lets a flat
// block meet sloping ground (ADR-0038; the user's "human-built, not draped").
void emitRetainingSkirt(RenderMesh& mesh, const Poly2& poly, Real topY,
                        const std::function<Real(const Vec2&)>& bottomAt,
                        const Vec3& col) {
    const std::size_t cnt = poly.size();
    for (std::size_t i = 0; i < cnt; ++i) {
        Vec2 a = poly[i], b = poly[(i + 1) % cnt];
        Real ya = std::min(bottomAt(a), topY) - 0.05;
        Real yb = std::min(bottomAt(b), topY) - 0.05;
        if (topY - ya < 0.05 && topY - yb < 0.05) continue;
        Vec3 ba(a.x, ya, a.y), bb(b.x, yb, b.y);
        Vec3 ta(a.x, topY, a.y), tb(b.x, topY, b.y);
        Vec2 dir = normalize(b - a);
        Vec3 nrm(dir.y, 0, -dir.x);           // outward (pad interior on the left)
        pushQuad(mesh, ba, bb, tb, ta, nrm, col);
    }
}

// A flat road carriageway between two graded endpoints (yA..yB): flat ACROSS its
// width at each point, gently sloped ALONG its length — a human-built street, not
// a sheet draped over the terrain. Per-segment normals.
void emitFlatRoad(RenderMesh& mesh, const Vec2& a, const Vec2& b, Real yA, Real yB,
                  Real width, const Vec3& col, Real thickness = 0.0) {
    Vec2 dir = b - a;
    Real len = dir.length();
    if (len < 1e-4) return;
    dir = dir / len;
    Vec2 u = perp(dir);                  // unit across (toward the right kerb)
    Vec2 across = u * (width * 0.5);
    int segs = std::max(1, static_cast<int>(std::ceil(len / 8.0)));
    for (int i = 0; i < segs; ++i) {
        Real t0 = static_cast<Real>(i) / segs, t1 = static_cast<Real>(i + 1) / segs;
        Real y0 = yA + (yB - yA) * t0, y1 = yA + (yB - yA) * t1;
        Vec2 c0 = lerp(a, b, t0), c1 = lerp(a, b, t1);
        Vec3 l0(c0.x - across.x, y0, c0.y - across.y), r0(c0.x + across.x, y0, c0.y + across.y);
        Vec3 r1(c1.x + across.x, y1, c1.y + across.y), l1(c1.x - across.x, y1, c1.y - across.y);
        Vec3 n = normalize(cross(r0 - l0, l1 - l0)); if (n.y < 0) n = n * -1;
        pushQuad(mesh, l0, r0, r1, l1, n, col);
        // A slab of `thickness`: vertical kerb lips down each long edge so the
        // carriageway stands proud of the (cut-to-grade) ground instead of being
        // coplanar with it (no z-fighting, and it reads as a built road).
        if (thickness > 0.0) {
            Vec3 down(0, -thickness, 0);
            Vec3 rl(u.x, 0, u.y);   // outward on the right edge, inward-flip on left
            pushQuad(mesh, r0, r1, r1 + down, r0 + down, rl, col);          // right kerb
            pushQuad(mesh, l1, l0, l0 + down, l1 + down, rl * -1, col);     // left kerb
        }
    }
}

// A painted line along a street (lane/centre marking): a thin flat road, offset
// from the centreline by `offset`, raised slightly so it reads on the asphalt.
void emitLaneLine(RenderMesh& mesh, const Vec2& a, const Vec2& b, Real yA, Real yB,
                  Real offset, Real lineWidth, const Vec3& col) {
    Vec2 dir = b - a;
    if (dir.lengthSquared() < 1e-8) return;
    Vec2 n = perp(normalize(dir)) * offset;
    emitFlatRoad(mesh, a + n, b + n, yA + 0.02, yB + 0.02, lineWidth, col);
}

// A short stair run from `topY` (sidewalk/apron) down to `botY` (street), centred
// at `mid`, facing `out` (toward the street), `width` wide. Treads ~0.16 m high.
void emitStairs(RenderMesh& mesh, const Vec3& mid, const Vec2& outDir, Real topY,
                Real botY, Real width, const Vec3& col) {
    Real drop = topY - botY;
    int steps = std::max(2, static_cast<int>(std::ceil(drop / 0.16)));
    Real rise = drop / steps, tread = 0.3;
    Vec2 o = normalize(outDir);
    Vec2 side = perp(o);
    for (int s = 0; s < steps; ++s) {
        Real y = topY - rise * (s + 1);
        Vec2 c = Vec2(mid.x, mid.z) + o * (tread * s + tread * 0.5);
        RenderMesh box = MeshBuilder::box(Vec3(width, rise + 0.02, tread));
        Real yaw = std::atan2(o.x, o.y);
        MeshBuilder::transform(box, Mat4::trs(Vec3(c.x, y + rise * 0.5, c.y),
                                              Quat::fromAxisAngle(Vec3(0, 1, 0), yaw),
                                              Vec3(1, 1, 1)));
        for (Vertex& v : box.vertices) v.color = col;
        uint32_t base = static_cast<uint32_t>(mesh.vertices.size());
        mesh.vertices.insert(mesh.vertices.end(), box.vertices.begin(), box.vertices.end());
        for (uint32_t idx : box.indices) mesh.indices.push_back(base + idx);
    }
    (void)side;
}


// A zebra crosswalk centred at `center`, bars perpendicular to pedestrian travel
// (i.e. running along the road `dir`), repeated across the road width. Flat at y.
void emitCrosswalk(RenderMesh& mesh, const Vec2& center, const Vec2& dir,
                   Real roadW, Real y, const Vec3& col) {
    Vec2 d = normalize(dir), across = perp(d);
    Real depth = 2.6, bar = 0.55, gap = 0.5;
    int n = std::max(1, static_cast<int>(roadW / (bar + gap)));
    Vec3 nrm(0, 1, 0);
    for (int i = 0; i < n; ++i) {
        Real s = -roadW * 0.5 + bar * 0.5 + i * (bar + gap);
        if (std::fabs(s) > roadW * 0.5 - bar * 0.4) continue;
        Vec2 bc = center + across * s;
        auto corner = [&](Real ld, Real la) {
            Vec2 p = bc + d * (depth * 0.5 * ld) + across * (bar * 0.5 * la);
            return Vec3(p.x, y + 0.03, p.y);
        };
        pushQuad(mesh, corner(-1, -1), corner(1, -1), corner(1, 1), corner(-1, 1), nrm, col);
    }
}

// A real street tree (ADR-0041): the parametric L-system tree (procgen/tree
// growTree — generalized-cylinder branches + alpha-cut leaf cards), with a
// thicker trunk/limbs than the forest default and no surface roots (it sits in a
// pit). Bark and leaves are kept separate (opaque vs alpha-cut leaf material) and
// the tree is *instanced* across the city — one prototype, hundreds of
// placements — instead of the old baked sphere-blob canopy.
struct CityTree {
    RenderMesh bark;
    RenderMesh leaves;
};

CityTree makeCityTree(uint32_t seed) {
    TreeParams tp;
    tp.iterations      = 4;        // branch orders: a believable crown, still light
    tp.trunkLength     = 1.6f;
    tp.lengthFalloff   = 0.80f;
    tp.leaderFalloff   = 0.88f;
    tp.branchAngle     = 38.0f;
    tp.angleJitter     = 16.0f;
    tp.branchesPerNode = 2;
    tp.terminalFraction = 0.36f;
    tp.terminalForks   = 3;
    tp.droop           = 0.26f;
    tp.wander          = 0.07f;
    tp.rootCount       = 0;        // street trees sit in a pit — no buttress roots
    tp.tipRadius       = 0.03f;
    tp.radiusScale     = 1.7f;     // thicker trunk + limbs (the trunks were too thin)
    tp.ringSegments    = 5;
    tp.leaves          = true;     // real alpha-cut leaf cards, not sphere blobs
    tp.leafSize        = 0.20f;
    tp.leavesPerTip    = 4;
    tp.leafClump       = 1.0f;
    tp.maxLeafCards    = 600;      // budget: many instances share this one proto
    tp.barkColor       = Vec3(0.32, 0.23, 0.16);
    tp.leafColor       = Vec3(0.20, 0.42, 0.15);
    TreeMesh tm = growTree(tp, seed ? seed : 1u);
    CityTree ct;
    ct.bark = std::move(tm.branches);
    ct.leaves = std::move(tm.leaves);
    return ct;
}

// A tree pit: a small square of dark soil under a street tree, where the sidewalk
// is cut away for the roots. Baked flat just above the apron (vertex-coloured).
void emitTreePit(RenderMesh& out, const Vec3& base, Real half) {
    Vec3 soil(0.16, 0.12, 0.08), n(0, 1, 0);
    Real y = base.y + 0.02;
    Vec3 a(base.x - half, y, base.z - half), b(base.x + half, y, base.z - half),
         c(base.x + half, y, base.z + half), d(base.x - half, y, base.z + half);
    pushQuad(out, a, b, c, d, n, soil);
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

    // Road-grade solver: assign each intersection an elevation, then Laplacian-
    // smooth it along the graph so connected streets share gentle, consistent
    // grades (engineered, not following every bump). Streets are then flat across
    // their width and gently sloped between these node grades — human-built.
    const int nNodes = static_cast<int>(graph.nodes.size());
    std::vector<Real> nodeGrade(nNodes);
    for (int i = 0; i < nNodes; ++i) nodeGrade[i] = cityGroundAt(cp, graph.nodes[i].pos);
    if (cp.groundAt) {
        std::vector<std::vector<int>> adj(nNodes);
        for (const RoadEdge& e : graph.edges) { adj[e.a].push_back(e.b); adj[e.b].push_back(e.a); }
        std::vector<Real> next(nodeGrade);
        // Smoothing: streets should read engineered, not bumpy. A few Laplacian
        // passes relax local terrain bumps between adjacent intersections while
        // preserving the broad slope across the city (a linear ramp is harmonic,
        // so it survives the relaxation; only high-frequency noise is removed).
        // Kept to a handful of passes: the road graph is only a few nodes wide,
        // so over-iterating would diffuse the whole slope toward its mean and
        // pull foundations off the terrain.
        for (int iter = 0; iter < 6; ++iter) {
            for (int i = 0; i < nNodes; ++i) {
                if (adj[i].empty()) continue;
                Real avg = 0; for (int j : adj[i]) avg += nodeGrade[j];
                avg /= adj[i].size();
                next[i] = nodeGrade[i] * 0.5 + avg * 0.5;
            }
            nodeGrade.swap(next);
        }
    }
    // Nearest-node grade at any XZ (block vertices ARE nodes, so this is exact).
    auto gradeAt = [&](const Vec2& p) {
        if (!cp.groundAt) return cp.baseY;
        int best = 0; Real bestD = 1e30;
        for (int i = 0; i < nNodes; ++i) {
            Real d = (graph.nodes[i].pos - p).lengthSquared();
            if (d < bestD) { bestD = d; best = i; }
        }
        return nodeGrade[best];
    };

    // Flat carriageways between graded intersections, with painted lane lines: a
    // yellow centre line + white edge lines.
    Vec3 sidewalkCol(0.52, 0.52, 0.50), asphaltCol(0.12, 0.12, 0.13),
         retainCol(0.42, 0.42, 0.42), yellow(0.72, 0.62, 0.12), white(0.78, 0.78, 0.76);
    const Real roadThickness = 0.12;   // carriageway slab depth over the cut ground
    for (const RoadEdge& e : graph.edges) {
        Vec2 a = graph.nodes[e.a].pos, b = graph.nodes[e.b].pos;
        Real yA = nodeGrade[e.a], yB = nodeGrade[e.b];
        Real w = 12.0;   // fill the corridor (= 2 x apron setback), so road meets curb
        emitFlatRoad(model.roads, a, b, yA, yB, w, asphaltCol, roadThickness);
        emitLaneLine(model.roads, a, b, yA, yB, 0.0, 0.22, yellow);          // centre
        emitLaneLine(model.roads, a, b, yA, yB, w * 0.5 - 0.5, 0.16, white); // edges
        emitLaneLine(model.roads, a, b, yA, yB, -(w * 0.5 - 0.5), 0.16, white);
        // Cut/fill the terrain to the carriageway grade (just under the slab) so
        // the ground meets the road instead of poking through it. A touch wider
        // than the road so the kerb lips land on level earth.
        if (cp.groundAt)
            model.flatten.push_back(makeFlattenRamp(
                Vec3(a.x, 0, a.y), Vec3(b.x, 0, b.y), yA - roadThickness,
                yB - roadThickness, w * 0.5 + 0.5, 5.0));
    }

    // Street furniture. Lamp posts run along the verges mid-block; intersections
    // get a coordinated kit (crosswalks + stop bars + corner traffic signals)
    // driven off the junction node, so a 4-way crossing reads as one engineered
    // intersection instead of four crosswalks fighting in the middle (Phase 2).
    const Real corridorHalf = 6.0;     // = apron setback, so road meets curb
    const Real roadW = 12.0;
    std::vector<int> degree(nNodes, 0);
    std::vector<std::vector<int>> incident(nNodes);   // node -> neighbour nodes
    for (const RoadEdge& e : graph.edges) {
        ++degree[e.a]; ++degree[e.b];
        incident[e.a].push_back(e.b); incident[e.b].push_back(e.a);
    }
    // Lamp posts and traffic signals are repeated furniture, so they are
    // *instanced* (ADR-0041): one prototype each, a transform per placement,
    // emitted as instance groups below instead of baked into model.props.
    std::vector<Mat4> lampXf, signalXf;

    // Mid-block lamp posts along each edge, alternating verge side.
    for (const RoadEdge& e : graph.edges) {
        Vec2 a = graph.nodes[e.a].pos, b = graph.nodes[e.b].pos;
        Vec2 d = b - a; Real len = d.length();
        if (len < 18) continue;
        d = d / len;
        Vec2 nrm = perp(d);
        Real yA = nodeGrade[e.a], yB = nodeGrade[e.b];
        Real off = roadW * 0.5 + 1.3;
        int nl = std::max(1, static_cast<int>(len / 30.0));
        for (int k = 1; k < nl; ++k) {
            Real t = static_cast<Real>(k) / nl;
            Vec2 on = lerp(a, b, t);
            Real y = yA + (yB - yA) * t;
            Real side = (k % 2 == 0) ? 1.0 : -1.0;
            Vec2 p = on + nrm * (off * side);
            lampXf.push_back(Mat4::translate(p.x, y, p.y));
        }
    }
    // Coordinated intersection decoration at every junction (degree >= 3). For
    // each arm: a crosswalk set just outside the curb returns, a stop bar behind
    // it, and a traffic signal + walk button on the near-right corner.
    const Real crossSet = corridorHalf + 1.4;    // crosswalk centre, clear of corner
    for (int ni = 0; ni < nNodes; ++ni) {
        if (degree[ni] < 3) continue;
        Vec2 node = graph.nodes[ni].pos;
        Real y = nodeGrade[ni];
        for (int oj : incident[ni]) {
            Vec2 other = graph.nodes[oj].pos;
            Vec2 toNode = node - other;              // travel toward the node
            Real len = toNode.length();
            if (len < 2 * crossSet + 3.0) continue;  // arm too short to mark
            Vec2 d = toNode / len;                   // approach direction
            Vec2 cwCenter = node - d * crossSet;     // crosswalk across this arm
            emitCrosswalk(model.roads, cwCenter, d, roadW, y, white);
            emitStopBar(model.roads, node - d * (crossSet + 1.6), d, roadW, y, white);
            // Near-right corner (drive-on-the-right): back from the node along the
            // arm and out to the right kerb. Signal head faces approaching traffic.
            Vec2 right(d.y, -d.x);
            Vec2 corner = node - d * corridorHalf + right * (corridorHalf + 0.6);
            Vec2 face = d * -1;                       // head faces approaching traffic
            Real yaw = std::atan2(face.x, face.y);
            signalXf.push_back(Mat4::trs(Vec3(corner.x, y, corner.y),
                                         Quat::fromAxisAngle(Vec3(0, 1, 0), yaw),
                                         Vec3(1, 1, 1)));
        }
    }

    // Emit the lamp + signal instance groups (opaque, vertex-coloured props).
    auto pushFurniture = [&](RenderMesh proto, std::vector<Mat4> xf, float rough) {
        if (proto.vertices.empty() || xf.empty()) return;
        CityInstanceGroup g;
        g.proto = std::move(proto);
        g.transforms = std::move(xf);
        g.roughness = rough;
        model.instanceGroups.push_back(std::move(g));
    };
    pushFurniture(streetLamp(), std::move(lampXf), 0.5f);
    pushFurniture(trafficSignalProto(), std::move(signalXf), 0.5f);

    Rng rng(cp.seed);

    // Pre-generate a few real (L-system) tree variants once; placements are
    // collected per variant and emitted as instanced groups (ADR-0041), so the
    // city pays for ~3 prototypes, not ~600 baked trees.
    std::vector<CityTree> treeVar;
    std::vector<std::vector<Mat4>> treeXf;
    if (cp.scatterTrees) {
        for (int i = 0; i < 3; ++i) treeVar.push_back(makeCityTree(cp.seed * 7u + i + 1));
        treeXf.resize(treeVar.size());
    }
    auto plantTree = [&](const Vec2& p, Rng& tr) {
        if (treeVar.empty()) return;
        std::size_t vi = tr.next() % treeVar.size();
        Vec3 base(p.x, cityGroundAt(cp, p), p.y);
        Real scale = tr.range(1.0, 1.4), yaw = tr.range(0, 6.283);
        treeXf[vi].push_back(Mat4::trs(base, Quat::fromAxisAngle(Vec3(0, 1, 0), yaw),
                                       Vec3(scale, scale, scale)));
        emitTreePit(model.props, base, 0.7);
        ++model.treeCount;
    };

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
                        plantTree(p, prng);
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

        // Flat block grade. The terrain is CUT to the pad (model.flatten), so the
        // pad sits at the block-CENTROID street grade plus a low curb — not the
        // highest adjacent corner, which made the foundation tower over the street
        // and poke up at the entrance on rolling ground (ADR-0040). One curb high,
        // so the building meets the sidewalk cleanly.
        const Real curbHeight = 0.12;
        const Real apronThickness = 0.12;
        Real gradeY = cp.baseY + 0.12;
        if (cp.groundAt) {
            gradeY = gradeAt(c) + curbHeight;
        }
        // Paved apron (sidewalk + block interior), flat at grade, snapped to the
        // road edge so the sidewalk meets the curb meets the carriageway with no
        // grass gap. The curb/retaining skirt drops to the STREET grade (gradeAt),
        // not the raw terrain, so it reads as an engineered curb.
        // Round the apron corners: the sidewalk corners ARE the intersection
        // corners, so a constant-radius fillet gives the rounded curb returns a
        // real street has (Phase 2 street kit) instead of knife-edge corners.
        Poly2 apron = roundPolygonCorners(inset(block, 6.0), 4.0, 4);
        if (apron.size() >= 3) {
            emitFlatPolygon(model.pavement, apron, gradeY, sidewalkCol);
            emitRetainingSkirt(model.pavement, apron, gradeY, gradeAt, retainCol);
            // Grade the terrain flat under the apron, a sidewalk-thickness below
            // its surface, so the block sits on level earth (no poke-through) and
            // the apron stands proud of the ground.
            if (cp.groundAt) {
                std::vector<Vec3> fp;
                fp.reserve(apron.size());
                for (const Vec2& v : apron) fp.push_back(Vec3(v.x, 0, v.y));
                model.flatten.push_back(
                    makeFlattenPad(std::move(fp), gradeY - apronThickness, 5.0));
            }
            // Steps where the curb is too tall to be a plain wall — one short run
            // mid-edge per long, steep block edge (a stoop down to the street).
            const std::size_t an = apron.size();
            for (std::size_t i = 0; i < an; ++i) {
                Vec2 ea = apron[i], eb = apron[(i + 1) % an];
                Real elen = distance(ea, eb);
                if (elen < 9.0) continue;
                Vec2 emid = lerp(ea, eb, 0.5);
                Real streetY = gradeAt(emid);
                if (gradeY - streetY < 0.7) continue;            // gentle: wall is fine
                Vec2 outDir = perp(normalize(eb - ea)) * -1;     // toward the street (CCW: outward = -left)
                emitStairs(model.pavement, Vec3(emid.x, 0, emid.y), outDir,
                           gradeY, streetY + 0.05, std::min(Real(4.0), elen * 0.4),
                           sidewalkCol * 0.96);
            }
        }

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
            // Reject sliver lots — a long, thin footprint extrudes into an
            // impossibly narrow blade. Need a real minimum short side.
            {
                Vec2 llo, lhi; bounds(lot.footprint, llo, lhi);
                if (std::min(lhi.x - llo.x, lhi.y - llo.y) < 6.0) continue;
            }
            // Pull the building in from the lot lines a little (setback to the lot).
            Poly2 site = inset(lot.footprint, 1.2);
            if (site.size() < 3 || area(site) < 30) site = lot.footprint;

            BuildingParams bp = paramsForDistrict(dist, rng, cp.seed);
            // A round tower needs a big, squarish lot or it ends up a pencil — only
            // keep the cylinder when the footprint can carry a wide one, else build
            // a box (ADR-0040).
            if (bp.shape == BuildingShape::Cylinder) {
                Vec2 lo, hi; bounds(site, lo, hi);
                Real w = hi.x - lo.x, dpt = hi.y - lo.y;
                Real diam = std::min(w, dpt);
                Real aspect = std::max(w, dpt) / std::max(Real(0.1), diam);
                if (diam < 18.0 || aspect > 1.35) bp.shape = BuildingShape::Box;
            }
            // Face the street: the lot sits inside the block, so the direction
            // from the block centre out to the lot points toward the perimeter
            // road. The entrance is placed on the face most aligned with it.
            Vec2 sc = centroid(site);
            Vec2 fd = sc - c;
            if (fd.lengthSquared() > 1e-6) {
                fd = normalize(fd);
                bp.faceDir = Vec3(fd.x, 0, fd.y);
            }
            if (oldTown) {
                bp.shape = BuildingShape::Pagoda;
                bp.tiers = 3 + 2 * static_cast<int>(rng.next() % 3);   // 3 / 5 / 7
                bp.floorHeight = rng.range(3.0, 3.8);
                bp.curtainWall = false; bp.solidFacade = false;
            }
            // The building sits flat on the block's graded pad (all buildings on a
            // block share gradeY) — no per-building podium needed; the block apron
            // + retaining skirt already level the site.
            Real baseY = gradeY;
            Scope scope = scopeFromFootprint(site, baseY, 10.0 /*unused*/);
            BuildingMesh bm = growBuilding(scope, bp);

            for (const RenderMesh& part : bm.parts) {
                int mi = part.materialIndex;
                if (mi < 0 || mi >= static_cast<int>(model.parts.size())) continue;
                MeshBuilder::append(model.parts[mi], part);
            }
            MeshBuilder::append(model.hlodProxy, bm.proxy);   // distant-city LOD

            Vec3 footC = scope.corner(0.5, 0, 0.5);
            CityBuilding cb;
            cb.site = centroid(site);
            cb.baseY = baseY;
            cb.height = bm.height;
            cb.district = dist;
            cb.boxCenter = Vec3(footC.x, baseY + bm.height * 0.5, footC.z);
            cb.boxHalf = Vec3(scope.size.x * 0.5, bm.height * 0.5, scope.size.z * 0.5);
            cb.yaw = std::atan2(scope.axis[2].x, scope.axis[2].z);
            cb.round = (bp.shape == BuildingShape::Cylinder);
            if (cb.round) {   // collider radius = the cylinder's radius, not the lot box
                Real rad = std::min(scope.size.x, scope.size.z) * 0.5 * 0.96;
                cb.boxHalf.x = cb.boxHalf.z = rad;
            }
            model.buildings.push_back(cb);
        }
    }

    // Street trees: a curated, evenly-spaced row on each verge, parallel to the
    // road, set back from the intersection so they don't crowd the crossing — a
    // planted streetscape, not the old haphazard scatter (ADR-0041; user feedback).
    if (cp.scatterTrees) {
        Rng trng(cp.seed ^ 0x57eeu);
        for (const RoadEdge& e : graph.edges) {
            Vec2 a = graph.nodes[e.a].pos, b = graph.nodes[e.b].pos;
            Vec2 d = b - a; Real len = d.length();
            d = d / std::max(len, Real(1e-4));
            Vec2 nrm = perp(d);
            Real verge = e.width * 0.5 + 1.8;
            Real margin = 9.0;                       // keep clear of the intersection
            Real usable = len - 2 * margin;
            if (usable < cp.streetTreeSpacing) continue;
            int n = std::max(1, static_cast<int>(usable / cp.streetTreeSpacing));
            Real step = usable / n;                  // even spacing, ends inset
            for (int k = 0; k <= n; ++k) {
                Vec2 on = a + d * (margin + step * k);
                for (Real s : {Real(1), Real(-1)}) {
                    Vec2 p = on + nrm * (verge * s);
                    plantTree(p, trng);
                }
            }
        }
    }

    // Collapse every tree placement into a handful of instanced groups (ADR-0041):
    // one bark + one leaf group per variant, sharing the variant's transforms.
    for (std::size_t vi = 0; vi < treeVar.size(); ++vi) {
        if (treeXf[vi].empty()) continue;
        if (!treeVar[vi].bark.vertices.empty()) {
            CityInstanceGroup g;
            g.proto = treeVar[vi].bark;
            g.transforms = treeXf[vi];
            g.roughness = 0.85f;
            model.instanceGroups.push_back(std::move(g));
        }
        if (!treeVar[vi].leaves.vertices.empty()) {
            CityInstanceGroup g;
            g.proto = treeVar[vi].leaves;
            g.transforms = treeXf[vi];
            g.roughness = 0.7f;
            g.alphaFoliage = true;
            model.instanceGroups.push_back(std::move(g));
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
