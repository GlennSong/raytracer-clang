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
                  Real width, const Vec3& col) {
    Vec2 dir = b - a;
    Real len = dir.length();
    if (len < 1e-4) return;
    dir = dir / len;
    Vec2 across = perp(dir) * (width * 0.5);
    int segs = std::max(1, static_cast<int>(std::ceil(len / 8.0)));
    for (int i = 0; i < segs; ++i) {
        Real t0 = static_cast<Real>(i) / segs, t1 = static_cast<Real>(i + 1) / segs;
        Real y0 = yA + (yB - yA) * t0, y1 = yA + (yB - yA) * t1;
        Vec2 c0 = lerp(a, b, t0), c1 = lerp(a, b, t1);
        Vec3 l0(c0.x - across.x, y0, c0.y - across.y), r0(c0.x + across.x, y0, c0.y + across.y);
        Vec3 r1(c1.x + across.x, y1, c1.y + across.y), l1(c1.x - across.x, y1, c1.y - across.y);
        Vec3 n = normalize(cross(r0 - l0, l1 - l0)); if (n.y < 0) n = n * -1;
        pushQuad(mesh, l0, r0, r1, l1, n, col);
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

// A street lamp post: a thin pole + a bright lamp-head box (street furniture).
void emitLamp(RenderMesh& out, const Vec3& base) {
    Vec3 poleCol(0.16, 0.16, 0.18), lampCol(1.0, 0.88, 0.55);
    Real h = 4.6;
    auto place = [&](RenderMesh m, const Vec3& col, Real y) {
        for (Vertex& v : m.vertices) v.color = col;
        MeshBuilder::transform(m, Mat4::translate(base.x, base.y + y, base.z));
        MeshBuilder::append(out, m);
    };
    place(MeshBuilder::cylinder(0.07f, static_cast<float>(h), 6), poleCol, h * 0.5);
    place(MeshBuilder::box(Vec3(0.5, 0.2, 0.32)), lampCol, h);
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
        // Light smoothing: remove local bumps for gentle grades, but keep the
        // broad slope (heavy smoothing would flatten the hills — not wanted).
        for (int iter = 0; iter < 5; ++iter) {
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
    for (const RoadEdge& e : graph.edges) {
        Vec2 a = graph.nodes[e.a].pos, b = graph.nodes[e.b].pos;
        Real yA = nodeGrade[e.a], yB = nodeGrade[e.b];
        Real w = 12.0;   // fill the corridor (= 2 x apron setback), so road meets curb
        emitFlatRoad(model.roads, a, b, yA, yB, w, asphaltCol);
        emitLaneLine(model.roads, a, b, yA, yB, 0.0, 0.22, yellow);          // centre
        emitLaneLine(model.roads, a, b, yA, yB, w * 0.5 - 0.5, 0.16, white); // edges
        emitLaneLine(model.roads, a, b, yA, yB, -(w * 0.5 - 0.5), 0.16, white);
    }

    // Street furniture: lamp posts along the verges (alternating sides), and zebra
    // crosswalks on each approach to an intersection (degree >= 3 node).
    std::vector<int> degree(nNodes, 0);
    for (const RoadEdge& e : graph.edges) { ++degree[e.a]; ++degree[e.b]; }
    for (const RoadEdge& e : graph.edges) {
        Vec2 a = graph.nodes[e.a].pos, b = graph.nodes[e.b].pos;
        Vec2 d = b - a; Real len = d.length();
        if (len < 18) continue;
        d = d / len;
        Vec2 nrm = perp(d);
        Real w = 12.0;   // fill the corridor (= 2 x apron setback), so road meets curb
        Real yA = nodeGrade[e.a], yB = nodeGrade[e.b];
        // Lamp posts every ~30 m, alternating verge side.
        Real off = w * 0.5 + 1.3;
        int nl = std::max(1, static_cast<int>(len / 30.0));
        for (int k = 1; k < nl; ++k) {
            Real t = static_cast<Real>(k) / nl;
            Vec2 on = lerp(a, b, t);
            Real y = yA + (yB - yA) * t;
            Real side = (k % 2 == 0) ? 1.0 : -1.0;
            Vec2 p = on + nrm * (off * side);
            emitLamp(model.props, Vec3(p.x, y, p.y));
        }
        // Crosswalks at intersection ends (set back from the node).
        if (degree[e.a] >= 3)
            emitCrosswalk(model.roads, a + d * 5.0, d, w, yA, white);
        if (degree[e.b] >= 3)
            emitCrosswalk(model.roads, b - d * 5.0, d, w, yB, white);
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

        // Flat block grade tied to the street grades: the pad sits a curb height
        // (0.15 m) above the highest adjacent intersection, so every block meets
        // its streets with a consistent curb. The whole block — apron + every
        // building — shares this one level, the way a real graded block does.
        Real gradeY = cp.baseY;
        if (cp.groundAt) {
            Real mx = -1e30;
            for (const Vec2& v : block) mx = std::max(mx, gradeAt(v));
            gradeY = mx + 0.15;
        } else {
            gradeY = cp.baseY + 0.15;
        }
        // Paved apron (sidewalk + block interior), flat at grade, snapped to the
        // road edge so the sidewalk meets the curb meets the carriageway with no
        // grass gap. The curb/retaining skirt drops to the STREET grade (gradeAt),
        // not the raw terrain, so it reads as an engineered curb.
        Poly2 apron = inset(block, 6.0);
        if (apron.size() >= 3) {
            emitFlatPolygon(model.pavement, apron, gradeY, sidewalkCol);
            emitRetainingSkirt(model.pavement, apron, gradeY, gradeAt, retainCol);
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
            model.buildings.push_back(cb);
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
