#include "engine/procgen/city/roads/lanes/deck_mesh.h"
#include "engine/procgen/city/roads/lanes/road_graph_spec.h"
#include "engine/procgen/city/roads/lanes/block_audit.h"

#include "engine/level_params.h"
#include "engine/procgen/city/roads/lanes/geom2d.h"
#include "engine/procgen/city/roads/lanes/road_twin.h"
#include "engine/procgen/city/roads/lanes/polyline_ops.h"
#include <iomanip>
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <set>
#include <sstream>

namespace engine {
namespace roads::lanes {

namespace {
std::vector<size_t> dpKeep(const Ring& pts, double tol) {
    std::vector<char> keep(pts.size(), 0); if (pts.size() < 3) return {};
    keep.front() = 1; keep.back() = 1; std::vector<std::pair<size_t, size_t>> stack{{0, pts.size() - 1}};
    while (!stack.empty()) {
        auto [a, b] = stack.back(); stack.pop_back(); if (b <= a + 1) continue;
        const Vec2 d = pts[b] - pts[a]; const double L = std::hypot(d.x, d.y); size_t best = a; double bd = -1;
        for (size_t i = a + 1; i < b; ++i) { const Vec2 v = pts[i] - pts[a]; const double dist = L > 1e-9 ? std::fabs(v.x * d.y - v.y * d.x) / L : std::hypot(v.x, v.y); if (dist > bd) { bd = dist; best = i; } }
        if (bd > tol) { keep[best] = 1; stack.push_back({a, best}); stack.push_back({best, b}); }
    }
    std::vector<size_t> out; for (size_t i = 0; i < pts.size(); ++i) if (keep[i]) out.push_back(i); return out;
}
}  // namespace

std::vector<Ring> pavementHoles(const Result& r, double minArea) {
    // The buildable space is what the pavement's holes enclose MINUS any pavement inside them. A hole is not a
    // block when other paved polygons sit inside it: a ground-level freeway ring whose frontage road never
    // touches it has one hole — the whole city — and taking it as a block parcelled lots over every street
    // (the slip-ramp metro, 2026-09-07). Hole-shaped pieces (an annulus: the band between the ring and the
    // frontage road) are verge, not blocks, and are dropped; the inner polygon's own holes are the blocks.
    std::vector<Ring> holes; const PolySet paved = unionSets(r.pavement.surface, r.pavement.shoulder);
    PolySet enclosed;
    for (const Polygon2& p : paved) for (const Ring& h : p.holes) if (std::fabs(ringArea(h)) >= minArea) for (const Polygon2& q : fromRing(h)) enclosed.push_back(q);
    if (enclosed.empty()) return holes;
    for (const Polygon2& piece : differenceSets(unionSets(enclosed, PolySet{}), paved)) {
        if (!piece.holes.empty() || std::fabs(ringArea(piece.outer)) < minArea) continue;
        holes.push_back(piece.outer);
    }
    return holes;
}

std::vector<Poly2> blocksFromHoles(const std::vector<Ring>& holes, double simplify, double insetBy) {
    std::vector<Poly2> blocks; std::vector<Ring> rings;
    for (const Ring& h : holes) {
        if (insetBy <= 0) { rings.push_back(h); continue; }
        // A hole may split when inset. A piece that vanishes under a further 3 m inset is a strip under 2*(inset+3) m
        // wide — the verge between a freeway and its frontage road, a median — not a block (2026-09-07).
        for (const Polygon2& q : offsetSet(fromRing(h), -insetBy)) if (std::fabs(ringArea(q.outer)) >= 135.0 && !offsetSet(PolySet{q}, -3.0).empty()) rings.push_back(q.outer);
    }
    for (const Ring& h : rings) {
        Ring closed = h; closed.push_back(h.front());   // DP wants an open run: split the loop at its first point
        std::vector<size_t> keep = dpKeep(closed, simplify);
        Poly2 poly; for (size_t i : keep) if (i + 1 < closed.size()) poly.push_back(closed[i]);
        if (poly.size() < 3) continue;
        double a = 0; for (size_t i = 0; i < poly.size(); ++i) { const Vec2& u = poly[i]; const Vec2& v = poly[(i + 1) % poly.size()]; a += u.x * v.y - v.x * u.y; }
        if (a < 0) std::reverse(poly.begin(), poly.end());   // CCW, as extractBlocks would hand them
        blocks.push_back(std::move(poly));
    }
    return blocks;
}

std::vector<Poly2> sceneBlocks(const Result& r, double simplify, double minArea, double insetBy) { return blocksFromHoles(pavementHoles(r, minArea), simplify, insetBy); }

double lanesSidewalkRise() { return Rules{}.skirtDrop + kSidewalkLift; }

double lanesPadFalloff(double sidewalk) { (void)sidewalk; return 2.0; }   // the feather lives INSIDE the block: footprints are inset by it, so the ramp ends at the block line

namespace {
// A flatten footprint shrunk by its own feather and clipped to the block, so the graded plane plus its
// ramp never leaves the block: the block edge is the back of the sidewalk, and anything past it lifts
// pavement (the flatten outranks the road). Returns the pieces (a footprint can split).
PolySet blocksInset(const std::vector<Poly2>& blocks, double falloff) {   // every block, inset by the feather, as one set: no per-lot lookup (a concave block's centroid can lie outside it)
    PolySet all; for (const Poly2& b : blocks) all = unionSets(all, offsetSet(fromRing(b), -falloff)); return all;
}
std::vector<std::vector<Vec3>> insideBlock(const std::vector<Vec3>& polygon, const PolySet& blocks, double falloff) {
    Ring ring; for (const Vec3& v : polygon) ring.emplace_back(v.x, v.z);
    PolySet ps = intersectSets(offsetSet(fromRing(ring), -falloff), blocks);   // the blocks inset too: a terrace wider than its block would otherwise feather from the block line outward
    std::vector<std::vector<Vec3>> out;
    for (const Polygon2& pg : ps) { if (pg.outer.size() < 3 || std::fabs(ringArea(pg.outer)) < 1.0) continue; std::vector<Vec3> poly; for (const Vec2& q : pg.outer) poly.push_back(Vec3(q.x, 0, q.y)); out.push_back(std::move(poly)); }
    return out;
}
}  // namespace

std::vector<TerrainFlatten> clipPadsToBlocks(const std::vector<LotBuilding>& lots, const std::vector<Poly2>& blocks, double sidewalk) {
    std::vector<TerrainFlatten> out; const double falloff = lanesPadFalloff(sidewalk); const PolySet inside = blocksInset(blocks, falloff);
    for (const LotBuilding& lb : lots) {
        if (lb.type == "park" || lb.type == "green" || lb.plan.size() < 3) continue;
        const TerrainFlatten raw = lotPadFlatten(lb, 2.2 + falloff, falloff);   // the apron grows by the feather the inset takes back
        for (std::vector<Vec3>& poly : insideBlock(raw.polygon, inside, falloff)) out.push_back(makeFlattenPad(std::move(poly), lb.groundY, falloff));
    }
    return out;
}

std::vector<TerrainFlatten> lanesTerraces(const std::vector<TerrainFlatten>& grades, const std::vector<Poly2>& blocks, double sidewalk) {
    std::vector<TerrainFlatten> out; const double falloff = lanesPadFalloff(sidewalk); const PolySet inside = blocksInset(blocks, falloff);
    for (const TerrainFlatten& g : grades) {
        if (g.polygon.size() < 3) continue;
        for (std::vector<Vec3>& poly : insideBlock(g.polygon, inside, falloff)) { TerrainFlatten f = g; f.polygon = std::move(poly); f.falloff = falloff; f.minX = f.minZ = 1e300; f.maxX = f.maxZ = -1e300; for (const Vec3& v : f.polygon) { f.minX = std::min(f.minX, v.x); f.maxX = std::max(f.maxX, v.x); f.minZ = std::min(f.minZ, v.z); f.maxZ = std::max(f.maxZ, v.z); } out.push_back(std::move(f)); }
    }
    return out;
}

std::vector<PadConflict> padsOnPavement(const Result& r, const std::vector<TerrainFlatten>& pads, double kerb) {
    std::vector<PadConflict> out; if (!r.hasTerrain || r.terrain.nx < 2 || r.conform.nodeOwner.size() != r.terrain.z.size()) return out;
    const HeightGrid& G = r.terrain;
    auto smooth = [](double u) { u = std::clamp(u, 0.0, 1.0); return u * u * (3 - 2 * u); };
    for (size_t pi = 0; pi < pads.size(); ++pi) {
        const TerrainFlatten& f = pads[pi]; if (f.polygon.size() < 3) continue;
        Ring ring; for (const Vec3& v : f.polygon) ring.emplace_back(v.x, v.z);
        const int i0 = std::max(0, static_cast<int>(std::floor((f.minX - f.falloff - G.x0) / G.res))), i1 = std::min(G.nx - 1, static_cast<int>(std::ceil((f.maxX + f.falloff - G.x0) / G.res)));
        const int j0 = std::max(0, static_cast<int>(std::floor((f.minZ - f.falloff - G.y0) / G.res))), j1 = std::min(G.ny - 1, static_cast<int>(std::ceil((f.maxZ + f.falloff - G.y0) / G.res)));
        PadConflict pc; pc.pad = static_cast<int>(pi);
        for (int j = j0; j <= j1; ++j) for (int i = i0; i <= i1; ++i) {
            const size_t n = static_cast<size_t>(j) * G.nx + i; if (r.conform.nodeOwner[n] == "-" || r.conform.nodeDist[n] > 0.01) continue;   // not on pavement or sidewalk
            const Vec2 q(G.x0 + i * G.res, G.y0 + j * G.res); double w = 1.0;
            if (!pointInRing(ring, q)) {
                double d = 1e300; for (size_t k = 0; k < ring.size(); ++k) { const Vec2& a = ring[k]; const Vec2& b = ring[(k + 1) % ring.size()]; const Vec2 ab = b - a; const double L2 = dot(ab, ab); const double t = L2 > 1e-12 ? std::clamp(dot(q - a, ab) / L2, 0.0, 1.0) : 0.0; d = std::min(d, distance(q, a + ab * t)); }
                if (d >= f.falloff) continue; w = smooth(1.0 - d / f.falloff);
            }
            const double plane = f.c + f.dx * q.x + f.dz * q.y, ground = G.z[n], rise = (plane - ground) * w;
            if (rise <= kerb) continue;
            pc.area += G.cellArea(); if (rise > pc.rise) { pc.rise = rise; pc.at = q; pc.what = r.conform.nodeOwner[n]; }
        }
        if (pc.area > 0) out.push_back(pc);
    }
    return out;
}

BlockAudit auditBlocks(const Result& r, const nlohmann::json& citysim, bool fromScene) {
    BlockAudit a;
    RoadEntity twin = roadTwin(r);
    EdgeBlockParams ep; LotParams lp;
    readLotGrowParams(citysim.is_object() ? citysim : nlohmann::json::object(), ep, lp);
    const HeightGrid* grid = r.hasTerrain ? &r.terrain : nullptr;
    RoadGroundFn ground = [grid](double x, double z) { return grid ? grid->sample(x, z) : 0.0; };
    lp.ground = [grid](Real x, Real z) { return static_cast<Real>(grid ? grid->sample(x, z) : 0.0); };
    RoadGraph row;   // the freeway right-of-way, as the loader hands it: freeway + ramp edges of the twin
    for (const RoadNode& n : twin.graph.nodes) row.nodes.push_back(n);
    for (const RoadEdge& e : twin.graph.edges) if (e.klass == RoadClass::Freeway || e.klass == RoadClass::Ramp) row.edges.push_back(e);
    const double roadClear = citysim.is_object() ? citysim.value("sidewalk", 4.0) + 0.6 : 4.6;
    if (fromScene) {
        // ONE representation: the blocks ARE the pavement's holes, exact to the kerb, so the pass needs no
        // road graph to keep them off the asphalt — no clearance graph, no right-of-way band, no twin.
        const double sidewalk = lp.roadMargin; lp.roadMargin = 0;   // inset here, robustly; the pass's own inset is a no-op
        std::vector<Poly2> blocks = sceneBlocks(r, 1.5, 2000.0, sidewalk);
        a.holesGiven = blocks.size();
        const bool wantParts = std::getenv("LANELAB_AUDIT_PARTS") != nullptr;   // grow the building meshes too (slow): `lanelab_tool blocks --at` lists them
        a.grown.lots = growLotBuildings(blocks, lp, &a.grown.plan, wantParts ? &a.grown.parts : nullptr, nullptr, 0.0, wantParts ? &a.grown.flatParts : nullptr, &a.grown.gradeFlatten);
        {   // pads and terraces against the pavement: as the loader built them before, and as it builds them now
            std::vector<TerrainFlatten> raw, now;   // `sidewalk` from the block inset above
            for (const LotBuilding& lb : a.grown.lots) { if (lb.type == "park" || lb.type == "green" || lb.plan.size() < 3) continue; raw.push_back(lotPadFlatten(lb)); }
            raw.insert(raw.end(), a.grown.gradeFlatten.begin(), a.grown.gradeFlatten.end());
            now = clipPadsToBlocks(a.grown.lots, blocks, sidewalk);
            for (TerrainFlatten& f : lanesTerraces(a.grown.gradeFlatten, blocks, sidewalk)) now.push_back(std::move(f));
            a.padConflictsRaw = padsOnPavement(r, raw); a.padConflicts = padsOnPavement(r, now);
            if (std::getenv("LANELAB_TRACE_PADS")) for (const PadConflict& pc : a.padConflicts) {
                const TerrainFlatten& f = now[static_cast<size_t>(pc.pad)]; const bool terrace = pc.pad >= static_cast<int>(now.size() - lanesTerraces(a.grown.gradeFlatten, blocks, sidewalk).size());
                std::fprintf(stderr, "pad-conflict: %s #%d bbox [%.0f %.0f]-[%.0f %.0f] falloff %.1f plane c %.2f dx %.4f dz %.4f; +%.2f m over %.0f m2 at (%.0f, %.0f) on %s\n", terrace ? "terrace" : "pad", pc.pad, f.minX, f.minZ, f.maxX, f.maxZ, f.falloff, f.c, f.dx, f.dz, pc.rise, pc.area, pc.at.x, pc.at.y, pc.what.c_str());
            }
            for (const PadConflict& pc : a.padConflictsRaw) a.padAreaRaw += pc.area; for (const PadConflict& pc : a.padConflicts) a.padArea += pc.area;
        }
        // holes the pass returned nothing for: a block foot's centroid inside the hole claims it
        for (const Poly2& h : blocks) {
            bool claimed = false;
            for (const Poly2& f : a.grown.plan.blocks) { Vec2 c; for (const Vec2& q : f) c += q; c = c / static_cast<double>(f.size()); if (pointInRing(h, c)) { claimed = true; break; } }
            if (claimed) continue;
            double ar = 0, shortest = 1e300; Vec2 c;
            for (size_t i = 0; i < h.size(); ++i) { const Vec2& u = h[i]; const Vec2& v = h[(i + 1) % h.size()]; ar += u.x * v.y - v.x * u.y; shortest = std::min(shortest, (v - u).length()); c += u; }
            Poly2 foot = h; double fa = 0; for (size_t i = 0; i < foot.size(); ++i) { const Vec2& u = foot[i]; const Vec2& v = foot[(i + 1) % foot.size()]; fa += u.x * v.y - v.x * u.y; }
            a.dropped.push_back({std::fabs(ar / 2), h.size(), shortest, c / static_cast<double>(h.size()), foot.size() < 3 ? std::string("inset rejected the polygon") : std::fabs(fa / 2) < lp.minLotArea * 1.5 ? "too small after the inset" : "parceller found no lot"});
        }
    } else {
        std::vector<RoadEntity> nets{twin};
        a.grown = growLotBuildingsOnNets(nets, lp, ep, roadClear, ground, row.edges.empty() ? nullptr : &row, false, false);
    }
    // the audit
    const PolySet& paved = r.pavement.surface;
    for (const Poly2& foot : a.grown.plan.blocks) {
        if (foot.size() < 3) continue;
        BlockReport b; b.foot = foot; b.vertices = foot.size();
        Ring ring(foot.begin(), foot.end()); PolySet fs = fromRing(ring);
        b.area = std::fabs(ringArea(ring));
        const PolySet opened = offsetSet(offsetSet(fs, -4.0), 4.0);
        b.thinArea = std::max(0.0, b.area - setArea(opened));
        for (const Polygon2& piece : differenceSets(fs, opened)) {   // each thin piece: a spike if it is long
            double longest = 0; const Ring& o = piece.outer;
            for (size_t i = 0; i < o.size(); ++i) for (size_t j = i + 1; j < o.size(); ++j) longest = std::max(longest, (o[i] - o[j]).length());
            if (longest > 25.0) b.spikeArea += std::fabs(ringArea(piece.outer));
        }
        b.pavedArea = setArea(intersectSets(fs, paved));
        {   // convex hull (monotone chain) for the wedge-vs-spike call
            std::vector<Vec2> P(foot.begin(), foot.end()); std::sort(P.begin(), P.end(), [](const Vec2& u, const Vec2& v) { return u.x < v.x || (u.x == v.x && u.y < v.y); });
            auto cross2 = [](const Vec2& o, const Vec2& u, const Vec2& v) { return (u.x - o.x) * (v.y - o.y) - (u.y - o.y) * (v.x - o.x); };
            std::vector<Vec2> H(2 * P.size()); size_t k = 0;
            for (size_t i = 0; i < P.size(); ++i) { while (k >= 2 && cross2(H[k - 2], H[k - 1], P[i]) <= 0) --k; H[k++] = P[i]; }
            for (size_t i = P.size() - 1, t = k + 1; i > 0; --i) { while (k >= t && cross2(H[k - 2], H[k - 1], P[i - 1]) <= 0) --k; H[k++] = P[i - 1]; }
            H.resize(k > 1 ? k - 1 : 0); double ha = 0; for (size_t i = 0; i < H.size(); ++i) { const Vec2& u = H[i]; const Vec2& v = H[(i + 1) % H.size()]; ha += u.x * v.y - v.x * u.y; }
            ha = std::fabs(ha / 2); b.convexity = ha > 1e-6 ? b.area / ha : 1.0;
        }
        for (const Vec2& q : foot) b.centroid += q; b.centroid = b.centroid / static_cast<double>(foot.size());
        if (b.pavedArea > 1.0) {
            std::set<std::string> roads;
            for (size_t li = 0; li < r.lanes.lanes.size(); ++li) {
                const Lane& l = r.lanes.lanes[li]; if (l.parent < 0 || r.pavement.footprints[li].empty()) continue;
                if (setArea(intersectSets(fs, r.pavement.footprints[li])) > 0.5) roads.insert(r.graph.edges[static_cast<size_t>(l.parent)].id);
            }
            for (const std::string& s : roads) b.roads += (b.roads.empty() ? "" : " ") + s;
        }
        if (!b.ok()) ++a.broken;
        a.thinTotal += b.thinArea; a.pavedTotal += b.pavedArea; a.spikeTotal += b.spikeArea;
        a.blocks.push_back(std::move(b));
    }
    return a;
}

void writeBlocksSvg(const Result& r, const BlockAudit& a, const std::string& path) {
    const std::array<double, 4> b = r.graph.bounds(); const double W = b[1] - b[0], H = b[3] - b[2], s = 1600.0 / std::max(W, H);
    auto X = [&](double x) { return (x - b[0]) * s; }; auto Y = [&](double y) { return (b[3] - y) * s; };
    std::ofstream f(path);
    f << "<svg xmlns='http://www.w3.org/2000/svg' width='" << W * s << "' height='" << H * s << "' viewBox='0 0 " << W * s << " " << H * s << "'>\n";
    f << "<rect width='100%' height='100%' fill='#eef0e6'/>\n";
    auto ring = [&](const Ring& rg) { for (size_t i = 0; i < rg.size(); ++i) f << (i ? " L" : "M") << X(rg[i].x) << " " << Y(rg[i].y); f << " Z"; };
    f << "<path fill='#8d8d8d' fill-rule='evenodd' stroke='none' d='";
    for (const Polygon2& p : r.pavement.surface) { ring(p.outer); for (const Ring& h : p.holes) ring(h); }
    f << "'/>\n";
    for (const BlockReport& blk : a.blocks) {
        f << "<polygon fill='" << (blk.ok() ? "#cfe3c4" : "#f4b5b0") << "' fill-opacity='0.75' stroke='" << (blk.ok() ? "#3f6b3a" : "#c0392b") << "' stroke-width='" << 0.8 * s << "' points='";
        for (const Vec2& q : blk.foot) f << X(q.x) << "," << Y(q.y) << " ";
        f << "'/>\n";
    }
    for (const Poly2& l : a.grown.plan.lots) { f << "<polygon fill='none' stroke='#8c5a3c' stroke-width='" << 0.35 * s << "' points='"; for (const Vec2& q : l) f << X(q.x) << "," << Y(q.y) << " "; f << "'/>\n"; }
    for (const LotBuilding& lb : a.grown.lots) { if (lb.plan.size() < 3) continue; f << "<polygon fill='#2b2b2b' stroke='none' points='"; for (const Vec2& q : lb.plan) f << X(q.x) << "," << Y(q.y) << " "; f << "'/>\n"; }
    for (const BlockReport& blk : a.blocks) if (!blk.ok()) f << "<text x='" << X(blk.centroid.x) << "' y='" << Y(blk.centroid.y) << "' font-size='" << 6 * s << "' fill='#c0392b' text-anchor='middle'>thin " << static_cast<int>(blk.thinArea) << " / paved " << static_cast<int>(blk.pavedArea) << "</text>\n";
    f << "</svg>\n";
}

std::string summary(const BlockAudit& a) {
    std::ostringstream o;
    o << "pads: " << a.padConflicts.size() << " lift pavement or sidewalk (" << static_cast<int>(a.padArea) << " m²); unclipped they were " << a.padConflictsRaw.size() << " (" << static_cast<int>(a.padAreaRaw) << " m²)";
    for (size_t i = 0; i < a.padConflicts.size() && i < 6; ++i) { const PadConflict& pc = a.padConflicts[i]; o << (i == 0 ? ": " : "; ") << pc.what << " +" << std::fixed << std::setprecision(2) << pc.rise << " m at (" << static_cast<int>(pc.at.x) << ", " << static_cast<int>(pc.at.y) << ")"; }
    o << "\n";
    o << "blocks: " << a.blocks.size() << " of " << a.holesGiven << " pavement holes (" << a.grown.plan.lots.size() << " lots, " << a.grown.lots.size() << " built); broken " << a.broken
      << " — spikes " << static_cast<int>(a.spikeTotal) << " m² (thin " << static_cast<int>(a.thinTotal) << "), on pavement " << static_cast<int>(a.pavedTotal) << " m²\n";
    std::vector<const BlockReport*> worst; for (const BlockReport& b : a.blocks) if (!b.ok()) worst.push_back(&b);
    std::sort(worst.begin(), worst.end(), [](const BlockReport* x, const BlockReport* y) { return x->spikeArea + x->pavedArea > y->spikeArea + y->pavedArea; });
    for (size_t i = 0; i < worst.size() && i < 12; ++i) {
        const BlockReport& b = *worst[i];
        o << "  block " << static_cast<int>(b.area) << " m², " << b.vertices << " vertices at (" << static_cast<int>(b.centroid.x) << ", " << static_cast<int>(b.centroid.y) << "): spikes " << static_cast<int>(b.spikeArea) << " m² (thin " << static_cast<int>(b.thinArea) << "), paved " << static_cast<int>(b.pavedArea) << " m², convexity " << static_cast<int>(b.convexity * 100) << "%" << (b.roads.empty() ? "" : " under " + b.roads) << "\n";
    }
    if (!a.dropped.empty()) {
        o << "  holes that became no block: " << a.dropped.size() << "\n";
        std::vector<const DroppedHole*> d; for (const DroppedHole& x : a.dropped) d.push_back(&x);
        std::sort(d.begin(), d.end(), [](const DroppedHole* x, const DroppedHole* y) { return x->area > y->area; });
        for (size_t i = 0; i < d.size() && i < 12; ++i) o << "    " << static_cast<int>(d[i]->area) << " m², " << d[i]->vertices << " vertices, shortest edge " << std::fixed << std::setprecision(1) << d[i]->shortestEdge << " m at (" << static_cast<int>(d[i]->centroid.x) << ", " << static_cast<int>(d[i]->centroid.y) << "): " << d[i]->why << "\n";
    }
    return o.str();
}

}  // namespace roads::lanes
}  // namespace engine
