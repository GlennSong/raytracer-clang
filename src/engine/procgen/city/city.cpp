#include "city.h"

#include "parcel.h"
#include "district.h"     // buildDistrict: the real road-network tech (ADR-0066)
#include "street_kit.h"
#include "road_mesh.h"
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

    // Window ELEMENT + quoins by cladding (building-grammar-plan.md P2): brick
    // walk-ups get segmental-arched sashes with voussoir hoods and 2x2 lights;
    // stucco mixes round arches in; concrete/precast keeps flat heads with a
    // header band; glass/metal stays a clean skin.
    switch (style) {
        case FacadeStyle::Brick:
            p.window.head = OpeningStyle::Head::Segmental;
            p.window.hood = OpeningStyle::Hood::Arch;
            p.window.lightsX = 2; p.window.lightsY = 2;
            p.quoins = (rng.unit() < 0.6);
            break;
        case FacadeStyle::Stucco:
            p.window.head = (rng.unit() < 0.4) ? OpeningStyle::Head::Round
                                               : OpeningStyle::Head::Flat;
            p.window.hood = (p.window.head == OpeningStyle::Head::Round)
                                ? OpeningStyle::Hood::Arch : OpeningStyle::Hood::Band;
            p.window.lightsX = 2; p.window.lightsY = 1;
            p.quoins = (rng.unit() < 0.5);
            break;
        case FacadeStyle::Concrete:
            p.window.head = OpeningStyle::Head::Flat;
            p.window.hood = OpeningStyle::Hood::Band;
            p.window.lightsX = 1; p.window.lightsY = 2;
            break;
        default: break;   // painted/glass/metal keep the plain defaults
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

    // 1. Roads -> 2. planarize -> 3. block faces. Two road sources (ADR-0066):
    // the classic regular GRID, or the DISTRICT subdivision tech (arterials +
    // irregular streets — what grown.json drives on), which hands back both the
    // planarized graph AND its block polygons, so the same lot/building pipeline
    // below runs on the real road network.
    RoadGraph graph;
    if (cp.districtRoads) {
        DistrictParams dpp;
        dpp.center = cp.center;
        dpp.radius = cp.extent;
        dpp.arterials = cp.arterials;
        dpp.blockSizeMin = cp.blockSizeMin;
        dpp.blockSizeMax = cp.blockSizeMax;
        dpp.arteryWidth = cp.arteryWidth;
        dpp.streetWidth = cp.streetWidth;
        dpp.irregular = cp.irregular;
        dpp.jitter = cp.roadJitter;
        dpp.seed = cp.seed;
        DistrictNet dn = buildDistrict(dpp);
        graph = dn.graph;              // already planarized + connected
        model.blocks = dn.blocks;      // buildDistrict emits the block faces itself
    } else {
        GridRoadParams gp;
        gp.center = cp.center; gp.extent = cp.extent; gp.cellSize = cp.cellSize;
        gp.jitter = cp.roadJitter; gp.seed = cp.seed;
        graph = planarize(gridRoads(gp));
        model.blocks = extractBlocks(graph);
    }
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

    // Roads as ONE junction-aware surface (ADR-0044/0048). Rather than a full-width
    // ribbon per edge (which stacks and z-fights where streets cross) or a uniform
    // dense SDF grid (which spends polygons everywhere), buildRoadMesh trims each
    // ribbon back to the curb corner, fills only the intersection with a pad, runs a
    // simple strip between junctions (split only where the terrain bends), and draws
    // the lane markings + crosswalks on the trimmed span — so the markings stop at the
    // intersection and a crossing lands exactly at each junction mouth. The block
    // aprons remain the sidewalks, so the carriageway carries no sidewalk band here.
    Vec3 sidewalkCol(0.52, 0.52, 0.50), asphaltCol(0.12, 0.12, 0.13),
         retainCol(0.42, 0.42, 0.42), yellow(0.72, 0.62, 0.12), white(0.78, 0.78, 0.76);
    const Real roadThickness = 0.12;   // carriageway slab depth over the cut ground
    // Corridor / apron setback basis. GRID roads fill a uniform 12 m ribbon.
    // DISTRICT roads keep buildDistrict's real per-edge widths (arterials ~13 m,
    // streets ~7 m), so a narrow street draws narrow instead of a fat 12 m ribbon
    // overrunning its sidewalk (user feedback); the apron sizes to the widest road.
    const Real roadW = cp.districtRoads ? 13.0 : 12.0;
    if (!cp.districtRoads)
        for (RoadEdge& e : graph.edges) e.width = roadW;   // grid: one ribbon width

    auto appendMesh = [](RenderMesh& dst, const RenderMesh& src) {
        uint32_t base = static_cast<uint32_t>(dst.vertices.size());
        dst.vertices.insert(dst.vertices.end(), src.vertices.begin(), src.vertices.end());
        for (uint32_t idx : src.indices) dst.indices.push_back(base + idx);
    };

    // Smooth street grade: project a point onto its nearest road edge and lerp the
    // node grades along it — flat across, linear along each edge — so the carriageway
    // sits level with the block aprons at the curb.
    auto roadGradeAt = [&](double x, double z) -> double {
        Vec2 pt(x, z); Real bestD = 1e30, bestY = cp.baseY;
        for (const RoadEdge& e : graph.edges) {
            Vec2 a = graph.nodes[e.a].pos, b = graph.nodes[e.b].pos, ab = b - a;
            Real L2 = ab.lengthSquared();
            Real t = L2 > 1e-9 ? std::clamp(dot(pt - a, ab) / L2, Real(0), Real(1)) : Real(0);
            Vec2 proj = a + ab * t; Real d = (pt - proj).lengthSquared();
            if (d < bestD) { bestD = d; bestY = nodeGrade[e.a] + (nodeGrade[e.b] - nodeGrade[e.a]) * t; }
        }
        return bestY;
    };

    RoadMeshParams rmp;
    rmp.heightAt = [&](double x, double z) { return roadGradeAt(x, z); };
    rmp.lift = roadThickness;
    rmp.color = asphaltCol;
    rmp.minSetback = roadW * 0.5 + 0.5;     // pad clears the curb corners
    rmp.cornerRadius = 3.0;                  // rounded kerb returns at intersections
    rmp.sidewalkWidth = 0.0;                // the block aprons are the sidewalks
    rmp.laneMarkings = true;
    rmp.laneWidth = 3.6; rmp.markWidth = 0.18; rmp.markLift = 0.04;
    rmp.laneColor = white; rmp.centerColor = yellow;
    rmp.crosswalks = true; rmp.crosswalkColor = white;
    appendMesh(model.roads, buildRoadMesh(graph, rmp));

    // Cut the terrain to the carriageway grade under each road so the ground meets
    // the road surface instead of poking through it.
    if (cp.groundAt)
        for (const RoadEdge& e : graph.edges) {
            Vec2 a = graph.nodes[e.a].pos, b = graph.nodes[e.b].pos;
            model.flatten.push_back(makeFlattenRamp(
                Vec3(a.x, 0, a.y), Vec3(b.x, 0, b.y),
                nodeGrade[e.a] - roadThickness, nodeGrade[e.b] - roadThickness,
                roadW * 0.5 + 0.5, 5.0));
        }

    // Street furniture. Lamp posts run along the verges mid-block; intersections
    // get a coordinated kit (crosswalks + stop bars + corner traffic signals)
    // driven off the junction node, so a 4-way crossing reads as one engineered
    // intersection instead of four crosswalks fighting in the middle (Phase 2).
    const Real corridorHalf = 6.0;     // = apron setback, so road meets curb
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
            // The crosswalk itself is drawn by buildRoadMesh at the junction mouth;
            // here we add a stop bar just behind it, draped on the road surface (so it
            // isn't buried where the grade rises away from the node).
            Vec2 sbCenter = node - d * (crossSet + 1.6);
            Real sbY = roadGradeAt(sbCenter.x, sbCenter.y) + roadThickness + 0.02;
            emitStopBar(model.roads, sbCenter, d, roadW, sbY, white);
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
            if (lot.court) continue;   // block-interior court (v2 step 10) is
                                       // open space, never a building lot
            if (rng.unit() > cp.buildChance) continue;        // plaza / empty
            if (lot.area < 50) continue;
            // Reject sliver lots — a long, thin footprint extrudes into an
            // impossibly narrow blade. Need a real minimum short side AND a
            // bounded aspect ratio, or an irregular block's off-cuts become
            // knife-edge buildings (user feedback).
            {
                // Real short side via the OBB — an axis-aligned bbox misses a
                // thin DIAGONAL sliver (both bbox sides look large), which then
                // shrink-fits to a degenerate box far from its site.
                const OBB2 lo = orientedBoundingBox(lot.footprint);
                const Real shortSide = lo.half[1 - lo.longAxis()] * 2.0;
                const Real longSide = lo.half[lo.longAxis()] * 2.0;
                if (shortSide < 9.0) continue;                  // too skinny to be a building
                if (longSide > shortSide * 3.5) continue;       // long thin blade
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
            // A triangular / thin-parallelogram lot can pass the OBB
            // short-side gate yet inscribe a sub-metre building (the shrink-fit
            // rectangle fits in the lot's narrow throat). Skip it — a 0.8 m
            // "building" is a wall, not a structure.
            if (scope.size.x < 3.0 || scope.size.z < 3.0) continue;
            BuildingMesh bm = growBuilding(scope, bp);

            for (const RenderMesh& part : bm.parts) {
                int mi = part.materialIndex;
                if (mi < 0 || mi >= static_cast<int>(model.parts.size())) continue;
                MeshBuilder::append(model.parts[mi], part);
            }
            MeshBuilder::append(model.hlodProxy, bm.proxy);   // distant-city LOD

            Vec3 footC = scope.corner(0.5, 0, 0.5);
            CityBuilding cb;
            // The building's SITE is where the building actually sits (its
            // scope centre), not the lot centroid — on a triangular/trapezoid
            // corner lot the inscribed building legitimately offsets from the
            // lot centroid, and agents should route to the building, not the
            // empty corner.
            cb.site = Vec2(footC.x, footC.z);
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

    // Surface the road GRAPH (Living City, ADR-0066): the same planarized graph the
    // carriageway was meshed over — its edges now carry the drawn ribbon width. A
    // host can spawn a RoadNet from it so the citysim drives the generated streets
    // (and buildings become places), turning a procedural city into a living one.
    model.roadGraph = graph;
    return model;
}

RenderMesh CityModel::mergedBuildings() const {
    RenderMesh m;
    for (const RenderMesh& p : parts) MeshBuilder::append(m, p);
    return m;
}

}  // namespace engine
