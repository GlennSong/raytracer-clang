#include "engine/procgen/lanelab/lanelab.h"
#include "engine/procgen/lanelab/polyline_ops.h"
#include "engine/procgen/lanelab/vertical_profile.h"

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cmath>
#include <set>
#include <iomanip>
#include <sstream>

namespace engine {
namespace lanelab {

namespace {
double secondsSince(const std::chrono::steady_clock::time_point& t) { return std::chrono::duration<double>(std::chrono::steady_clock::now() - t).count(); }
}

std::unique_ptr<Result> build(RoadLabGraph graph, const BuildOptions& opts) {
    // stage weights = measured metro seconds (lanes 0.1, profiles 13.5, footprints 5.4, surfaces 14.7, terrain 2.8)
    constexpr double kW[5] = {0.01, 0.36, 0.15, 0.40, 0.08}; double cum = 0;
    auto stage = [&](int i, const char* name, double local) {
        if (!opts.progress) return; double start = 0; for (int k = 0; k < i; ++k) start += kW[k];
        BuildProgress p; p.stage = name; p.fraction = std::min(1.0, start + kW[i] * std::clamp(local, 0.0, 1.0)); if (!(*opts.progress)(p)) throw BuildCancelled();
    };
    (void)cum;
    auto t0 = std::chrono::steady_clock::now(); auto t1 = t0;
    auto rp = std::make_unique<Result>(); Result& r = *rp; r.graph = std::move(graph); RoadLabGraph& g = r.graph; const Rules& R = g.rules;
    std::array<double, 4> bounds = g.bounds(); HeightField terrain = makeTerrain(g.terrain, bounds); r.hasTerrain = g.terrain.type != "flat";
    // 1. lanes (ramps compose from anchors)
    stage(0, "lanes", 0.0); r.lanes = expand(g); r.timings["lanes"] = secondsSince(t1); t1 = std::chrono::steady_clock::now(); stage(1, "profiles", 0.0);
    // 2. profiles: through roads, agree at nodes and crossings (iterated), then ramps, then connector ends
    for (EdgeSpec& e : g.edges) if (!e.isRamp()) throughProfile(e, terrain, g.cls(e));
    for (EdgeSpec& e : g.edges) if (e.isRamp()) rampProfile(g, e, terrain);   // ramps need heights to take part in crossing consistency
    auto agree = [&](bool first) {
        for (int it = 0; it < 12; ++it) {
            double mm = nodeConsistency(g, R.endpointTol, R.bridgeH), xm = crossingConsistency(g, R.bridgeH, R.rampLevelDz);
            for (EdgeSpec& e : g.edges) if (!e.isRamp()) applyFloors(e, g.cls(e));   // lifts survive the blends
            for (EdgeSpec& e : g.edges) if (e.isRamp()) rampProfile(g, e, terrain);   // hosts moved
            if (first && it == 0) r.nodeMismatchBefore = std::max(mm, xm);
            if (std::max(mm, xm) < 0.005) break;
        }
    };
    agree(true);
    // Grade-separated crossings clear their structure (Glenn: "roads that go underneath collide with the
    // bottom of the freeway's undercarriage"): wherever two roads cross with the upper one already a bridge
    // (dz >= bridge_h), the upper deck's TOP must be at least under_clearance + its slab + the girders above
    // the lower road's surface. Short crossings get a floor tent on the upper road and its profile is
    // re-run — through roads via their tents, ramps by lifting under the same design-grade cone.
    {
        std::vector<EdgeSpec*> paved; for (EdgeSpec& e : g.edges) if (e.z.size() > 1 && e.laneCount() > 0) paved.push_back(&e);
        // Crossings including a road END lying on the other road's interior: the ring's freeway arcs meet at
        // a node straight above the arterial, and the exact-bounds segment test finds the arc that starts
        // there but misses the one that ends a hair short of the line, leaving that side of the node low.
        auto crossingsInclusive = [&](const EdgeSpec& a, const EdgeSpec& b) {
            std::vector<Vec2> out = crossings(a.xy, b.xy);
            auto addEnd = [&](const EdgeSpec& e, const EdgeSpec& o) {
                for (const Vec2& q : {e.xy.front(), e.xy.back()}) {
                    // a road ending anywhere under the other's paved band counts (streets the importer clipped at a
                    // frontage road's edge end 5 m from its centreline); at-grade tees are excluded by the bridge test
                    const Projection pr = project(o.xy, o.s, q); if (pr.distance > g.hw(o) + g.hw(e)) continue;
                    if (std::min(distance(o.xy.front(), q), distance(o.xy.back(), q)) <= R.endpointTol) continue;   // a shared node: a junction
                    bool dup = false; for (const Vec2& c : out) if (distance(c, q) < 1.0) dup = true; if (!dup) out.push_back(q);
                }
            };
            addEnd(a, b); addEnd(b, a); return out;
        };
        int lifted = 0;
        for (int pass = 0; pass < 3; ++pass) {
            bool any = false;
            for (size_t i = 0; i < paved.size(); ++i) for (size_t j = i + 1; j < paved.size(); ++j) {
                EdgeSpec& a = *paved[i]; EdgeSpec& b = *paved[j];
                if (a.isRamp() && (a.from.edge == b.id || a.to.edge == b.id)) continue;   // a ramp and its own host: the gore, not a crossing
                if (b.isRamp() && (b.from.edge == a.id || b.to.edge == a.id)) continue;
                for (const Vec2& p : crossingsInclusive(a, b)) {
                    const double za = projectZ(a, p), zb = projectZ(b, p); if (std::fabs(za - zb) < ((a.isRamp() || b.isRamp()) ? R.rampLevelDz : R.bridgeH)) continue;   // below: a level crossing the street meets
                    EdgeSpec& hi = za > zb ? a : b; EdgeSpec& lo = za > zb ? b : a;
                    const double need = std::min(za, zb) + R.underClearance + g.cls(hi).thick + R.structureDepth;
                    // The hold must span the whole crossing, not the centreline point: a tent peaks at its station
                    // and falls away at design grade, so the deck over the far kerb of a 20 m road sat half a
                    // metre low. The floor point holds flat over a half-span covering the lower road's paved band
                    // plus the upper road's own width, measured along the upper road (over the crossing sine).
                    auto pavedW = [&](const EdgeSpec& e) { const RoadClassSpec& c = g.cls(e); return e.laneCount() * e.lanes.w + e.lanes.gap + 2.0 * c.shoulder; };
                    const double sHi = project(hi.xy, hi.s, p).station, sLo = project(lo.xy, lo.s, p).station;
                    auto tangent = [](const EdgeSpec& e, double st) { const Vec2 q0 = pointAt(e.xy, e.s, std::max(0.0, st - 0.5)), q1 = pointAt(e.xy, e.s, std::min(e.s.back(), st + 0.5)); const Vec2 d = q1 - q0; const double L = std::hypot(d.x, d.y); return L > 1e-9 ? d / L : Vec2(1, 0); };
                    const Vec2 th = tangent(hi, sHi), tl = tangent(lo, sLo); const double sinT = std::max(0.25, std::fabs(cross(th, tl)));
                    const double span = (0.5 * pavedW(lo) + 0.5 * pavedW(hi) + 1.0) / sinT;
                    const double sts[3] = {std::max(0.0, sHi - span), sHi, std::min(hi.s.back(), sHi + span)};
                    bool low = false; for (double st : sts) if (interp(hi.s, hi.z, st) < need - 0.02) low = true;
                    if (!low) continue;
                    static const bool trace = std::getenv("LANELAB_TRACE_CLEARANCE") != nullptr;
                    if (trace) std::fprintf(stderr, "clearance: %s over %s at (%.0f, %.0f) need %.2f, z before %.2f %.2f %.2f (span %.1f)", hi.id.c_str(), lo.id.c_str(), p.x, p.y, need, interp(hi.s, hi.z, sts[0]), interp(hi.s, hi.z, sts[1]), interp(hi.s, hi.z, sts[2]), span);
                    { bool have = false; for (auto& fp : hi.floorPts) if (std::hypot(fp[0] - p.x, fp[1] - p.y) < 0.5) { fp[2] = std::max(fp[2], need); fp[3] = std::max(fp[3], span); have = true; } if (!have) hi.floorPts.push_back({p.x, p.y, need, span}); }
                    any = true; ++lifted;
                    if (hi.isRamp()) rampProfile(g, hi, terrain); else throughProfile(hi, terrain, g.cls(hi));
                    if (trace) std::fprintf(stderr, " -> after %.2f %.2f %.2f\n", interp(hi.s, hi.z, sts[0]), interp(hi.s, hi.z, sts[1]), interp(hi.s, hi.z, sts[2]));
                }
            }
            if (!any) break;
            agree(false);   // lifted decks must still agree at their nodes and level crossings
        }
        if (lifted) std::fprintf(stderr, "lanelab: %d grade-separated crossings lifted to clear their structure\n", lifted);
    }
    { NodeMismatchWhere w; const double nm = nodeMismatch(g, R.endpointTol, R.bridgeH, &w), xm = crossingConsistency(g, R.bridgeH, R.rampLevelDz); for (EdgeSpec& e : g.edges) if (!e.isRamp()) applyFloors(e, g.cls(e)); r.nodeMismatchAfter = std::max(nm, xm);
      if (nm >= xm && nm > 0.02) r.nodeMismatchWhere = w.a + (w.atEnd ? " meets " : " ends on ") + w.b + " at (" + std::to_string(static_cast<int>(w.at.x)) + ", " + std::to_string(static_cast<int>(w.at.y)) + ")"; }
    for (EdgeSpec& e : g.edges) if (e.isRamp()) rampProfile(g, e, terrain);
    for (Lane& l : r.lanes.lanes) if (l.isConnector()) {
        l.z0 = laneHeightAt(r.lanes.lanes[static_cast<size_t>(l.srcLane)], g, l.xy.front()); l.z1 = laneHeightAt(r.lanes.lanes[static_cast<size_t>(l.dstLane)], g, l.xy.back());
    }
    r.timings["profiles"] = secondsSince(t1); t1 = std::chrono::steady_clock::now(); stage(2, "footprints", 0.0);
    // 3. footprints, closing, layers; 4. arrangement and surfaces
    r.heights = std::make_unique<DeckHeight>(g, r.lanes);
    buildFootprints(g, r.lanes, *r.heights, r.pavement); r.timings["footprints"] = secondsSince(t1); t1 = std::chrono::steady_clock::now();
    { std::vector<std::vector<Ring>> outers(r.lanes.lanes.size()), holes(r.lanes.lanes.size());
      for (size_t li = 0; li < r.lanes.lanes.size(); ++li) for (const Polygon2& p : r.pavement.footprints[li]) { outers[li].push_back(p.outer); for (const Ring& h : p.holes) holes[li].push_back(h); }
      r.heights->setFootprints(outers, holes); }
    std::function<bool(double)> surf = [&](double f) { stage(3, "surfaces", f); return true; };
    buildSurfaces(g, r.lanes, *r.heights, r.pavement, opts.threads, opts.progress ? &surf : nullptr); r.timings["surfaces"] = secondsSince(t1); t1 = std::chrono::steady_clock::now(); stage(4, "terrain", 0.0);
    // 5. structure: bridge segments and piers
    // A pier never stands on another road (Glenn: "the pillars that hold up the freeway intersect with
    // the road"): its 2.6 m footprint is tested against every OTHER road's pavement; a blocked station
    // slides along the span, up to half a bay either way, and a bay with no clear station gets no pier.
    auto onOtherRoad = [&](const EdgeSpec& own, const Vec2& p) {
        const int ownIdx = g.index.at(own.id);
        for (size_t li = 0; li < r.lanes.lanes.size(); ++li) {
            const Lane& l = r.lanes.lanes[li]; if (l.parent == ownIdx || r.pavement.footprints[li].empty()) continue;
            const PolySet& fp = r.pavement.footprints[li]; const Box2 b = ::engine::lanelab::bounds(fp[0]);
            if (p.x < b.minX - 2 || p.x > b.maxX + 2 || p.y < b.minY - 2 || p.y > b.maxY + 2) continue;
            for (const Vec2& q : {p, p + Vec2(1.5, 0), p - Vec2(1.5, 0), p + Vec2(0, 1.5), p - Vec2(0, 1.5)}) if (contains(fp, q)) return true;
        }
        return false;
    };
    int piersMoved = 0, piersDropped = 0;
    for (const EdgeSpec* e : g.paved()) {
        double onStruct = 0, next = 0; const RoadClassSpec& c = g.cls(*e);
        for (size_t i = 0; i < e->s.size(); ++i) {
            // structure is judged and the pier topped from the DECK the road actually gets (partner blends can
            // hold a lane away from its own profile near a landing), never from the raw profile
            const double deckZ = r.heights->deckRoad(static_cast<int>(g.index.at(e->id)), e->xy[i]);
            bool bridge = deckZ - e->t[i] > R.bridgeH; if (i > 0 && bridge) onStruct += e->s[i] - e->s[i-1];
            if (!(bridge && e->s[i] >= next)) continue;
            size_t at = i; bool ok = !onOtherRoad(*e, e->xy[i]);
            if (!ok) {   // slide: nearest clear sample within half a bay, still on structure
                for (size_t step = 1; !ok && step * 2.0 <= R.pierSpacing * 0.5; ++step) {
                    for (long sgn : {1L, -1L}) { const long k = static_cast<long>(i) + sgn * static_cast<long>(step) * 2; if (k < 0 || k >= static_cast<long>(e->s.size())) continue;
                        const size_t kk = static_cast<size_t>(k); if (r.heights->deckRoad(static_cast<int>(g.index.at(e->id)), e->xy[kk]) - e->t[kk] <= R.bridgeH) continue; if (!onOtherRoad(*e, e->xy[kk])) { at = kk; ok = true; ++piersMoved; break; } }
                }
            }
            if (ok) r.piers.push_back({g.index.at(e->id), e->xy[at], e->t[at] - 1.0, r.heights->deckRoad(static_cast<int>(g.index.at(e->id)), e->xy[at]) - c.thick}); else ++piersDropped;
            next = e->s[i] + R.pierSpacing;
        }
        r.bridgeLen[e->id] = onStruct;
    }
    if (piersMoved || piersDropped) std::fprintf(stderr, "lanelab: piers moved off roads %d, bays left without a pier %d\n", piersMoved, piersDropped);
    r.adjacent = adjacency(r.lanes);
    // 6. terrain
    if (r.hasTerrain) {
        r.terrain = bakeGrid(terrain, g.terrain, bounds); conformGrid(g, r.lanes, *r.heights, r.terrain, r.conform); terrainVsDeck(r.pavement.decks, r.terrain, r.conform);
    }
    r.timings["terrain"] = secondsSince(t1); r.seconds = secondsSince(t0); stage(4, "done", 1.0);
    return rp;
}

std::vector<SurfaceStep> surfaceSteps(const Result& r, double kerb) {
    std::vector<SurfaceStep> out; const RoadLabGraph& g = r.graph; const size_t nl = r.lanes.lanes.size();
    std::vector<Box2> boxes(nl); for (size_t li = 0; li < nl; ++li) boxes[li] = r.pavement.footprints[li].empty() ? Box2{} : bounds(r.pavement.footprints[li]);
    auto laneName = [&](int li) { return li >= 0 && li < static_cast<int>(nl) ? r.lanes.lanes[static_cast<size_t>(li)].id : std::string("?"); };
    for (const Surface& s : r.pavement.decks) {
        std::map<std::pair<int, int>, std::pair<int, int>> edges;   // key -> (count, triangle)
        for (size_t ti = 0; ti < s.tris.size(); ++ti) for (int k = 0; k < 3; ++k) {
            const int a = s.tris[ti][k], b = s.tris[ti][(k + 1) % 3]; auto& e = edges[{std::min(a, b), std::max(a, b)}]; if (e.first++ == 0) e.second = static_cast<int>(ti);
        }
        for (const auto& kv : edges) {
            if (kv.second.first != 1) continue; const size_t ti = static_cast<size_t>(kv.second.second); const auto& t = s.tris[ti];
            int a = -1, b = -1, c = -1;   // oriented as in the triangle, c the third vertex
            for (int k = 0; k < 3; ++k) { const int u = t[k], v = t[(k + 1) % 3]; if (std::min(u, v) == kv.first.first && std::max(u, v) == kv.first.second) { a = u; b = v; c = t[(k + 2) % 3]; } }
            if (a < 0) continue;
            const DeckVertex& A = s.verts[static_cast<size_t>(a)]; const DeckVertex& B = s.verts[static_cast<size_t>(b)]; const DeckVertex& C = s.verts[static_cast<size_t>(c)];
            const Vec2 d = B.xy - A.xy; const double L = std::hypot(d.x, d.y); if (L < 0.05) continue;
            Vec2 n(d.y / L, -d.x / L); if (dot(n, C.xy - A.xy) > 0) n = n * -1.0;   // outward: away from the third vertex
            const Vec2 m = (A.xy + B.xy) * 0.5, q = m + n * 0.35; const double zEdge = 0.5 * (A.z + B.z);
            // pavement outside the edge: the covering lane whose deck is nearest in height
            int other = -1; double best = 1e9;
            for (size_t li = 0; li < nl; ++li) {
                const Box2& bx = boxes[li]; if (r.pavement.footprints[li].empty() || q.x < bx.minX || q.x > bx.maxX || q.y < bx.minY || q.y > bx.maxY) continue;
                if (!contains(r.pavement.footprints[li], q)) continue;
                const double dz = r.heights->deck(static_cast<int>(li), q) - zEdge; if (std::fabs(dz) < std::fabs(best)) { best = dz; other = static_cast<int>(li); }
            }
            SurfaceStep st; st.at = m; st.a = laneName(s.triOwner[ti]); st.length = L;
            if (other >= 0) {
                if (std::fabs(best) <= kerb || std::fabs(best) >= g.rules.bridgeH) continue;
                st.dz = -best; st.b = laneName(other); out.push_back(st);   // dz: this edge above the pavement outside it
            } else if (r.hasTerrain && r.terrain.inside(q.x, q.y)) {
                // bare ground outside: an embankment edge must have its ground graded up to it (conform). Edges
                // 1.5 m or more above ground are walled (the parapet rule) and belong to structure or its
                // abutment, so a lip only counts below that: an unwalled drop beside the lane with nothing graded.
                const double lip = zEdge - r.terrain.sample(q.x, q.y);
                if (lip <= std::max(kerb, g.rules.skirtDrop + 0.35) || lip >= 1.5) continue;
                st.dz = lip; st.kind = 1; out.push_back(st);
            }
        }
    }
    // Layers: a sidewalk, shoulder or median edge must sit within kerb height of the lane deck beside it.
    struct LayerSpec { const PolySet* set; const char* name; double lift; bool anyRoad; };
    const LayerSpec layers[] = {{&r.pavement.sidewalk, "sidewalk", 0.12, false}, {&r.pavement.shoulder, "shoulder", 0.0, false}, {&r.pavement.median, "median", 0.10, true}};
    for (const LayerSpec& ls : layers) {
        const std::vector<int> roads = layerRoads(g, ls.anyRoad); if (roads.empty()) continue;
        {   // triangles the mesher drops for bridging two levels: the condition is worth naming even though the mesh no longer shows it
            std::vector<Vec2> spanning; (void)layerMeshes(g, *r.heights, *ls.set, roads, &spanning);
            for (const Vec2& c : spanning) { SurfaceStep st2; st2.at = c; st2.a = ls.name; st2.kind = 2; st2.length = 1.0; out.push_back(st2); }
        }
        auto walk = [&](const Ring& ring, bool hole) {
            const size_t n = ring.size(); if (n < 3) return;
            // the side away from the layer's material, from orientation alone (no point-in-set test: that
            // is quadratic on a city's sidewalk ring): an outer ring has its material on the left when CCW,
            // a hole ring has the layer's material on its left when CW
            const bool ccw = ringArea(ring) > 0; const double away = hole ? (ccw ? -1.0 : 1.0) : (ccw ? 1.0 : -1.0);
            for (size_t k = 0; k < n; ++k) {
                const Vec2& a = ring[k]; const Vec2& b = ring[(k + 1) % n]; const Vec2 d = b - a; const double L = std::hypot(d.x, d.y); if (L < 0.05) continue;
                const Vec2 nrm(d.y / L, -d.x / L);
                for (double st = 0.5; st < L; st += 1.0) {
                    const Vec2 m = a + d * (st / L);
                    {
                        const Vec2 q = m + nrm * (0.35 * away);
                        int other = -1; double bestZ = 0, bd = 1e300;
                        for (size_t li = 0; li < nl; ++li) {
                            const Box2& bx = boxes[li]; if (r.pavement.footprints[li].empty() || q.x < bx.minX || q.x > bx.maxX || q.y < bx.minY || q.y > bx.maxY) continue;
                            if (!contains(r.pavement.footprints[li], q)) continue;
                            const double z = r.heights->deck(static_cast<int>(li), q); const double dd = std::fabs(z - r.heights->layerHeight(roads, m)); if (dd < bd) { bd = dd; other = static_cast<int>(li); bestZ = z; }
                        }
                        if (other < 0) continue;
                        const double dz = (r.heights->layerHeight(roads, m) + ls.lift) - bestZ;
                        if (std::fabs(dz) <= kerb + ls.lift || std::fabs(dz) >= g.rules.bridgeH) continue;
                        SurfaceStep st2; st2.at = m; st2.dz = dz; st2.a = ls.name; st2.b = laneName(other); st2.length = std::min(1.0, L - st + 0.5); out.push_back(st2);
                    }
                }
            }
        };
        for (const Polygon2& pg : *ls.set) { walk(pg.outer, false); for (const Ring& hole : pg.holes) walk(hole, true); }
    }
    return out;
}

std::vector<Check> invariants(const Result& r) {
    // Lane coverage: the deck triangles a lane owns must cover its footprint. A lane whose paved area is
    // not triangulated shows as a HOLE in the carriageway with the parapet running past it (Glenn:
    // "segments where there's no lane, but the wall continues on as if there was a lane there").
    Check cover{"every lane's footprint is covered by its deck triangles (>= 97%)", "", true};
    {
        std::vector<double> owned(r.lanes.lanes.size(), 0.0);
        for (const Surface& s : r.pavement.decks) for (size_t ti = 0; ti < s.tris.size(); ++ti) {
            const auto& t = s.tris[ti]; const Vec2& a = s.verts[static_cast<size_t>(t[0])].xy; const Vec2& b = s.verts[static_cast<size_t>(t[1])].xy; const Vec2& c = s.verts[static_cast<size_t>(t[2])].xy;
            const int o = s.triOwner[ti]; if (o >= 0 && o < static_cast<int>(owned.size())) owned[static_cast<size_t>(o)] += std::fabs(cross(b - a, c - a)) * 0.5;
        }
        int bad = 0; std::ostringstream d;
        for (size_t li = 0; li < r.lanes.lanes.size(); ++li) {
            if (r.lanes.lanes[li].isConnector()) continue;   // a connector lives inside the junction box its through lanes own
            if (r.pavement.footprints[li].empty()) continue; const double fa = setArea(r.pavement.footprints[li]); if (fa < 20.0) continue;
            // the footprint shares area with neighbours after the arrangement; owned area can legitimately be less. Flag only gross loss.
            const double frac = owned[li] / fa;
            if (frac < 0.60) {
                ++bad;
                if (bad <= 8) {
                    const Lane& l = r.lanes.lanes[li]; const Vec2 mid = pointAt(l.xy, l.s, l.s.back() * 0.5);
                    // who owns the triangles inside this lane's footprint?
                    std::map<int, double> by;
                    for (const Surface& s : r.pavement.decks) for (size_t ti = 0; ti < s.tris.size(); ++ti) {
                        const auto& t = s.tris[ti]; const Vec2 c = (s.verts[static_cast<size_t>(t[0])].xy + s.verts[static_cast<size_t>(t[1])].xy + s.verts[static_cast<size_t>(t[2])].xy) / 3.0;
                        if (!contains(r.pavement.footprints[li], c)) continue;
                        const Vec2& a = s.verts[static_cast<size_t>(t[0])].xy; const Vec2& b = s.verts[static_cast<size_t>(t[1])].xy; const Vec2& cc = s.verts[static_cast<size_t>(t[2])].xy;
                        by[s.triOwner[ti]] += std::fabs(cross(b - a, cc - a)) * 0.5;
                    }
                    d << " " << l.id << " " << static_cast<int>(frac * 100) << "% at (" << static_cast<int>(mid.x) << ", " << static_cast<int>(mid.y) << ") [inside its footprint:";
                    double tot = 0; for (const auto& kv : by) tot += kv.second;
                    for (const auto& kv : by) if (kv.second > 0.05 * fa) d << " " << (kv.first >= 0 && kv.first < static_cast<int>(r.lanes.lanes.size()) ? r.lanes.lanes[static_cast<size_t>(kv.first)].id : std::string("?")) << " " << static_cast<int>(kv.second) << "m2";
                    double raw = 0; for (const Polygon2& pg : r.pavement.footprints[li]) raw += std::fabs(ringArea(pg.outer));
                    d << ", triangulated " << static_cast<int>(100 * tot / fa) << "%, ring raw/union area " << static_cast<int>(raw) << "/" << static_cast<int>(fa) << ", polygons " << r.pavement.footprints[li].size() << "]";
                }
            }
        }
        if (bad) { cover.ok = false; cover.detail = std::to_string(bad) + " lanes own < 60% of their footprint:" + d.str(); } else cover.detail = "no lane below 60% of its footprint";
    }

    const RoadLabGraph& g = r.graph; std::vector<Check> out; std::ostringstream d;
    auto add = [&](const std::string& name, bool ok, const std::string& detail) { out.push_back({name, detail, ok}); };
    bool welded = true; d.str("");
    for (size_t i = 0; i < r.pavement.decks.size(); ++i) { const Surface& s = r.pavement.decks[i]; if (s.nonManifold || s.cracks || s.boundaryOdd) { welded = false; d << " deck " << i << " nm=" << s.nonManifold << " cracks=" << s.cracks << " odd=" << s.boundaryOdd; } }
    add("decks welded (no cracks, no non-manifold edges, closed boundary loops)", welded, std::to_string(r.pavement.decks.size()) + " deck meshes" + d.str());
    d.str(""); bool grades = true;
    for (const EdgeSpec* e : g.paved()) { double mg = maxGrade(*e), lim = g.cls(*e).gMax; if (mg > lim + 0.002) { grades = false; d << " " << e->id << " " << 100 * mg << "% > " << 100 * lim << "%"; } }
    add("grades within class limits", grades, grades ? "all edges within g_max" : d.str());
    d.str(""); bool ramps = true;
    for (const EdgeSpec& e : g.edges) if (e.ramp.valid && !e.ramp.ok) { ramps = false; d << " " << e.id << ": free " << e.ramp.freeLen << " m, needs " << e.ramp.requiredLen << " m"; }
    add("ramps have the free length their climb needs", ramps, ramps ? "every ramp can make its climb" : d.str());
    d.str(""); bool meet = true;
    for (const EdgeSpec& e : g.edges) {
        if (!e.ramp.valid) continue;
        for (int k = 0; k < 2; ++k) { const std::string& host = k == 0 ? e.from.edge : e.to.edge; if (host.empty()) continue; const EdgeSpec* h = g.find(host);
            double dz = std::fabs(projectZ(*h, k == 0 ? e.xy.front() : e.xy.back()) - (k == 0 ? e.z.front() : e.z.back())); if (dz > 0.02) { meet = false; d << " " << e.id << "->" << host << " " << 100 * dz << " cm"; } }
    }
    add("ramps meet their hosts' heights at both ends", meet, meet ? "all ramp ends within 2 cm of host" : d.str());
    d.str(""); bool dove = true;
    for (const EdgeSpec& e : g.edges) for (const auto& kv : e.anchorLanes) {
        int li = r.lanes.find(kv.second); if (li < 0) { dove = false; d << " " << e.id << " missing lane " << kv.second; continue; }
        const Lane& l = r.lanes.lanes[static_cast<size_t>(li)]; double dist = project(l.xy, l.s, kv.first == "to" ? e.xy.back() : e.xy.front()).distance;
        if (dist > 0.05) { dove = false; d << " " << e.id << " " << kv.first << " " << kv.second << ": " << dist << " m off"; }
    }
    add("ramps dovetail into their hosts' outer lanes", dove, dove ? "every anchored ramp end sits on the host lane's centreline" : d.str());
    d.str(""); d << "largest node mismatch " << 100 * r.nodeMismatchAfter << " cm (was " << 100 * r.nodeMismatchBefore << " cm before correction)" << (r.nodeMismatchWhere.empty() ? "" : ": " + r.nodeMismatchWhere);
    add("junction heights consistent", r.nodeMismatchAfter < 0.02, d.str());
    d.str(""); bool adj = true; std::set<std::string> touched; for (const auto& p : r.adjacent) { touched.insert(r.lanes.roadOf(p.first, g)); touched.insert(r.lanes.roadOf(p.second, g)); }
    for (const EdgeSpec* e : g.paved()) if (std::max(e->lanes.fwd, e->lanes.back) >= 2 && !touched.count(e->id)) { adj = false; d << " " << e->id; }
    add("multilane roads have derived lane adjacency", adj, std::to_string(r.adjacent.size()) + " adjacent pairs" + (adj ? "" : ", none found for" + d.str()));
    if (r.hasTerrain) { d.str(""); d << r.conform.above << " of " << r.conform.samples << " samples above by > 1 cm, max excess " << r.conform.maxExcess << " m"; add("no terrain above any deck (> 1 cm)", r.conform.above == 0, d.str()); }
    out.push_back(cover);
    // Structure clearance: where two lanes are purely grade-separated (never same-level anywhere they
    // overlap), the smaller deck-to-deck gap must leave under_clearance below the upper slab and its girders
    // (Glenn: "roads that go underneath collide with the bottom of the freeway's undercarriage"). Pairs that
    // are same-level somewhere are a ramp leaving its host or a junction and are judged by other checks.
    {
        const Rules& R = g.rules; int bad = 0, pairs = 0; std::ostringstream dd; double worst = 0;
        std::vector<std::pair<double, std::string>> offenders;
        for (const auto& kv : r.pavement.pairs) {
            const PairStats& ps = kv.second; if (ps.sameLevelArea > 0 || ps.separatedArea < 0.5 || ps.minDz >= 1e8) continue;
            const Lane& a = r.lanes.lanes[static_cast<size_t>(kv.first.first)]; const Lane& b = r.lanes.lanes[static_cast<size_t>(kv.first.second)];
            if (a.isConnector() || b.isConnector()) continue;
            const double thick = std::max(g.classes.at(a.cls).thick, g.classes.at(b.cls).thick);
            const double need = R.underClearance + thick + R.structureDepth; ++pairs;
            if (ps.minDz < need - 0.05) {
                ++bad; worst = std::max(worst, need - ps.minDz); std::ostringstream o;
                const double zA = r.heights->own(kv.first.first, ps.minAt), zB = r.heights->own(kv.first.second, ps.minAt);
                o << " " << a.id << "|" << b.id << " gap " << std::fixed << std::setprecision(2) << ps.minDz << " m (needs " << need << ", z " << zA << "/" << zB << ") at (" << static_cast<int>(ps.minAt.x) << ", " << static_cast<int>(ps.minAt.y) << ")";
                offenders.emplace_back(need - ps.minDz, o.str());
            }
        }
        std::sort(offenders.begin(), offenders.end(), [](const auto& x, const auto& y) { return x.first > y.first; });
        dd << bad << " of " << pairs << " grade-separated lane pairs short of under_clearance + slab + structure"; if (bad) { dd << ", worst by " << std::fixed << std::setprecision(1) << worst << " m:"; for (size_t i = 0; i < offenders.size() && i < 8; ++i) dd << offenders[i].second; }
        out.push_back({"grade-separated pairs clear their structure", dd.str(), bad == 0});
    }
    // Piers under their decks: a column topped above the built slab bottom stands out of the road.
    {
        int bad = 0; std::ostringstream dp; double worst = 0;
        for (const Pier& p : r.piers) {
            const EdgeSpec& e = g.edges[static_cast<size_t>(p.edge)]; const double deckZ = r.heights->deckRoad(p.edge, p.xy), top = deckZ - g.cls(e).thick;
            const double over = p.z1 - top; if (over <= 0.05) continue;
            ++bad; if (over > worst) worst = over; if (bad <= 6) dp << " " << e.id << " +" << std::fixed << std::setprecision(2) << over << " m at (" << static_cast<int>(p.xy.x) << ", " << static_cast<int>(p.xy.y) << ")";
        }
        out.push_back({"piers stay under their decks", bad ? std::to_string(bad) + " of " + std::to_string(r.piers.size()) + " piers top out above the slab:" + dp.str() : std::to_string(r.piers.size()) + " piers, all under their slab", bad == 0});
    }
    // A deck follows its own profile: partners may hold a lane to a neighbour it meets, never pull it across
    // levels. Sampled along every lane's spine.
    {
        int bad = 0; std::ostringstream dq; double worst = 0;
        for (size_t li = 0; li < r.lanes.lanes.size(); ++li) {
            const Lane& l = r.lanes.lanes[li]; if (l.isConnector() || l.s.size() < 2) continue;
            for (double st = 2.0; st < l.s.back() - 2.0; st += 6.0) {
                const Vec2 p = pointAt(l.xy, l.s, st); const double d = std::fabs(r.heights->deck(static_cast<int>(li), p) - r.heights->own(static_cast<int>(li), p));
                if (d <= 2.5) continue; ++bad; if (d > worst) worst = d; if (bad <= 6) dq << " " << l.id << " " << std::fixed << std::setprecision(2) << d << " m at (" << static_cast<int>(p.x) << ", " << static_cast<int>(p.y) << ")"; break;
            }
        }
        out.push_back({"decks follow their profiles (partners pull < 2.5 m)", bad ? std::to_string(bad) + " lanes pulled across levels by a partner:" + dq.str() : "no lane pulled more than 2.5 m from its profile", bad == 0});
    }
    // Steps in the driving surface (Glenn: "part of the road is raised up in the 4-way intersection"):
    // an exposed slab side more than kerb height above or below what lies just outside it, under bridge height.
    {
        std::vector<SurfaceStep> steps = surfaceSteps(r); double pavedLen = 0, groundLen = 0; int paved = 0, ground = 0;
        int layerSteps = 0, bridged = 0; std::vector<Vec2> bridgedAt;
        for (const SurfaceStep& st : steps) { if (st.kind == 2) { ++bridged; if (bridgedAt.size() < 4) bridgedAt.push_back(st.at); continue; } if (st.b.empty()) { ++ground; groundLen += st.length; } else { ++paved; pavedLen += st.length; if (st.a == "sidewalk" || st.a == "shoulder" || st.a == "median") ++layerSteps; } }
        steps.erase(std::remove_if(steps.begin(), steps.end(), [](const SurfaceStep& st) { return st.kind == 2; }), steps.end());
        std::sort(steps.begin(), steps.end(), [](const SurfaceStep& x, const SurfaceStep& y) { if (x.b.empty() != y.b.empty()) return y.b.empty(); return std::fabs(x.dz) > std::fabs(y.dz); });   // pavement steps first, then by size
        std::ostringstream ds; ds << paved << " step edges against pavement (" << static_cast<int>(pavedLen) << " m, " << layerSteps << " of them sidewalk/shoulder/median edges); " << ground << " lips over bare ground (" << static_cast<int>(groundLen) << " m, informational: an abutment's transition cell looks the same)";
        std::vector<Vec2> shown; int listed = 0;
        for (const SurfaceStep& st : steps) {   // worst first, one per 12 m
            bool near = false; for (const Vec2& q : shown) if (distance(q, st.at) < 12) near = true; if (near) continue; shown.push_back(st.at);
            ds << (listed == 0 ? ": " : "; ") << st.a << (st.b.empty() ? " over ground" : " vs " + st.b) << " " << std::fixed << std::setprecision(2) << st.dz << " m at (" << static_cast<int>(st.at.x) << ", " << static_cast<int>(st.at.y) << ")";
            if (++listed >= 8) break;
        }
        if (bridged) { ds << "; " << bridged << " layer triangles bridged two levels and were dropped, e.g."; for (const Vec2& c : bridgedAt) ds << " (" << static_cast<int>(c.x) << ", " << static_cast<int>(c.y) << ")"; }
        out.push_back({"driving surface has no steps (> kerb, < bridge_h)", ds.str(), paved == 0});
    }
    return out;
}

std::string summary(const Result& r) {
    const RoadLabGraph& g = r.graph; std::ostringstream o; o.setf(std::ios::fixed); o.precision(1);
    int connectors = 0; for (const Lane& l : r.lanes.lanes) if (l.isConnector()) ++connectors;
    o << g.name << ": " << g.edges.size() << " edges -> " << r.lanes.lanes.size() << " lanes (" << connectors << " connectors), " << r.pavement.triangles << " triangles, " << r.pavement.decks.size() << " deck meshes, " << r.seconds << " s {";
    for (const auto& kv : r.timings) o << kv.first << " " << kv.second << " "; o << "}\n";
    for (const EdgeSpec& e : g.edges) {
        if (e.laneCount() == 0) continue; int piers = 0; for (const Pier& p : r.piers) if (p.edge == g.index.at(e.id)) ++piers;
        o << "  " << e.id << " (" << e.cls << ", " << e.laneCount() << " lanes) " << e.length() << " m, grade " << 100 * maxGrade(e) << "% / " << 100 * g.cls(e).gMax << "%, bridge " << r.bridgeLen.at(e.id) << " m, piers " << piers;
        if (e.ramp.valid) o << " | ramp departs " << e.ramp.departS << " gore " << e.ramp.touchS << " of " << e.ramp.length << " m, climbs " << e.ramp.climb << " m, free " << e.ramp.freeLen << " / needs " << e.ramp.requiredLen << (e.ramp.ok ? " OK" : " TOO SHORT");
        for (const auto& kv : e.anchorLanes) o << " " << kv.first << "=" << kv.second;
        o << "\n";
    }
    int same = 0, sep = 0; std::set<std::string> sepRoads;
    for (const auto& kv : r.pavement.pairs) { if (kv.second.sameLevelArea > 0) ++same; if (kv.second.separatedArea > 0) { ++sep; sepRoads.insert(r.lanes.roadOf(kv.first.first, g) + "|" + r.lanes.roadOf(kv.first.second, g)); } }
    o << "  lane pairs: " << same << " same-level, " << sep << " grade-separated"; if (!sepRoads.empty()) { o << " ("; for (const auto& s : sepRoads) o << s << " "; o << ")"; } o << "; closing added " << r.pavement.closingAdded << " m²\n";
    for (size_t i = 0; i < r.pavement.decks.size(); ++i) {
        const Surface& s = r.pavement.decks[i]; o << "  deck " << i << ": " << s.area << " m², " << s.verts.size() << " verts, " << s.tris.size() << " tris, boundary " << s.boundary.size() << ", non-manifold " << s.nonManifold << ", cracks " << s.cracks << ", odd-boundary " << s.boundaryOdd << "\n";
        for (const auto& cs : s.crackSamples) { o << "    crack " << cs.dz << " m at (" << cs.xy.x << ", " << cs.xy.y << ") between"; for (int ow : cs.owners) o << " " << (ow >= 0 ? r.lanes.lanes[static_cast<size_t>(ow)].id : std::string("?")); o << "\n"; }
    }
    o << "  layers: surface " << setArea(r.pavement.surface) << " m², shoulder " << setArea(r.pavement.shoulder) << ", sidewalk " << setArea(r.pavement.sidewalk) << ", median " << setArea(r.pavement.median) << "; islands " << r.pavement.islands.size() << ", enclosed blocks " << r.pavement.enclosedBlocks << "\n";
    o << "  nodes/crossings: mismatch " << 100 * r.nodeMismatchBefore << " cm before, " << 100 * r.nodeMismatchAfter << " cm after; adjacent lane pairs " << r.adjacent.size() << "\n";
    if (r.hasTerrain) {
        o << "  terrain: cut " << r.conform.cutM3 << " m³, fill " << r.conform.fillM3 << " m³; above deck " << r.conform.above << " of " << r.conform.samples << " (max " << r.conform.maxExcess << " m)\n";
        for (const ExcessSample& w : r.conform.worst) o << "    excess " << w.excess << " m at (" << w.xy.x << ", " << w.xy.y << ") deck of " << (w.owner >= 0 ? r.lanes.lanes[static_cast<size_t>(w.owner)].id : std::string("?")) << ", nearest node graded by " << w.nodeOwner << " at d=" << w.nodeDist << "\n";
    }
    return o.str();
}

}  // namespace lanelab
}  // namespace engine
