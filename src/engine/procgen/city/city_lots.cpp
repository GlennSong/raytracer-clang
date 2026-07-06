#include "city_lots.h"

#include "parcel.h"          // subdivideBlock, Lot, ParcelParams
#include "road_network.h"    // RoadGraph (edge blocks walk its chains)
#include "architect.h"       // DistrictMap + archetype tables (the architect pass)
#include "shape_grammar.h"   // scopeFromFootprint, growBuilding — REAL buildings
#include "road_mesh.h"       // triangulatePolygon (lot-shaped park pads)
#include "../../mesh_builder.h"   // MeshBuilder::append (merge parts by PartId)
#include <algorithm>
#include <array>
#include <cmath>

namespace engine {

namespace {
// A small deterministic hash RNG so the whole pass reproduces from `seed` (no
// global rng, no Math.random) — one stream per lot, mixed from stable inputs.
struct Hash {
    uint32_t s;
    explicit Hash(uint32_t seed) : s(seed ? seed : 0x9e3779b9u) {}
    uint32_t next() {
        s ^= s << 13; s ^= s >> 17; s ^= s << 5;
        return s;
    }
    Real unit() { return (next() & 0xffffff) / static_cast<Real>(0x1000000); }
    Real range(Real a, Real b) { return a + (b - a) * unit(); }
};
uint32_t mix(uint32_t a, uint32_t b) {
    uint32_t h = a * 0x85ebca6bu ^ (b + 0x9e3779b9u + (a << 6) + (a >> 2));
    h ^= h >> 15; h *= 0xc2b2ae35u; h ^= h >> 13;
    return h;
}

// A slab in the lot's OWN shape (device: "square green lots don't fit the
// blocks"): triangulated top + a side skirt, world-space, ground at y=0.
// Vertices stay white so the caller tints the whole pad via material albedo.
RenderMesh padMeshFor(const Poly2& poly, Real h) {
    RenderMesh m;
    const Vec3 white(1, 1, 1);
    for (const std::array<int, 3>& t : triangulatePolygon(poly))
        MeshBuilder::emitTri(m, Vec3(poly[t[0]].x, h, poly[t[0]].y),
                             Vec3(poly[t[1]].x, h, poly[t[1]].y),
                             Vec3(poly[t[2]].x, h, poly[t[2]].y),
                             Vec3(0, 1, 0), white);
    for (std::size_t i = 0; i < poly.size(); ++i) {
        const Vec2& a = poly[i];
        const Vec2& b = poly[(i + 1) % poly.size()];
        Vec2 d = b - a;
        if (d.length() < 1e-6) continue;
        Vec2 n = normalize(Vec2(d.y, -d.x));   // CCW plan: outward
        MeshBuilder::emitQuad(m, Vec3(a.x, 0, a.y), Vec3(b.x, 0, b.y),
                              Vec3(b.x, h, b.y), Vec3(a.x, h, a.y),
                              Vec3(n.x, 0, n.y), white);
    }
    return m;
}

Vec3 colorFor(const std::string& t) {
    if (t == "home")   return {0.72, 0.55, 0.45};
    if (t == "shop")   return {0.82, 0.70, 0.42};
    if (t == "office") return {0.55, 0.62, 0.72};
    if (t == "civic")  return {0.80, 0.80, 0.85};
    if (t == "park")   return {0.35, 0.60, 0.35};
    return {0.72, 0.70, 0.64};
}
}  // namespace

std::vector<LotBuilding> growLotBuildings(const std::vector<Poly2>& blocks,
                                          const LotParams& p, LotPlanDebug* debug,
                                          std::vector<RenderMesh>* outParts,
                                          const RoadGraph* roads, Real roadClearance) {
    std::vector<LotBuilding> out;
    // The ARCHITECT's district map (P5): radial rings + seeded old-town and
    // industrial quarters. Every lot asks the architect what belongs here.
    DistrictMap districts;
    districts.center = p.center;
    districts.innerRadius = p.innerRadius;
    districts.midRadius = p.midRadius;
    districts.seed = p.seed;
    // Road-clearance corner test (device: buildings poking onto the street): a
    // building box corner must stay `edge width/2 + roadClearance` from every
    // road centreline. Checked against the SAMPLED graph the asphalt is meshed
    // from, so a curvy road that bows into a straight-edged block still pushes
    // the building back. Folded into scopeFromFootprint's shrink-to-fit below.
    auto clearOfRoads = [&](const Vec2& c) {
        if (!roads) return true;
        for (const RoadEdge& e : roads->edges) {
            if (e.a < 0 || e.b < 0 || e.a >= static_cast<int>(roads->nodes.size()) ||
                e.b >= static_cast<int>(roads->nodes.size())) continue;
            const Vec2& a = roads->nodes[e.a].pos;
            const Vec2& b = roads->nodes[e.b].pos;
            Vec2 ab = b - a;
            Real len2 = ab.lengthSquared();
            Real t = len2 > 1e-12 ? dot(c - a, ab) / len2 : 0.0;
            t = t < 0 ? 0 : (t > 1 ? 1 : t);
            Vec2 q(a.x + ab.x * t, a.y + ab.y * t);
            if ((c - q).length() < e.width * 0.5 + roadClearance) return false;
        }
        return true;
    };
    if (outParts) {
        outParts->assign(static_cast<std::size_t>(PartId::Count), RenderMesh{});
        for (std::size_t i = 0; i < outParts->size(); ++i)
            (*outParts)[i].materialIndex = static_cast<int>(i);
    }
    for (std::size_t bi = 0; bi < blocks.size(); ++bi) {
        const Poly2& block = blocks[bi];
        if (block.size() < 3) continue;
        // Pull in from the road edge to the buildable interior (road + sidewalk).
        Poly2 foot = inset(block, p.roadMargin);
        if (foot.size() < 3 || area(foot) < p.minLotArea * 1.5) continue;
        if (debug) debug->blocks.push_back(foot);

        ParcelParams pp;
        pp.seed = mix(static_cast<uint32_t>(bi), p.seed);
        pp.targetArea = 480;
        pp.minArea = p.minLotArea;
        pp.minEdge = p.minShort;
        // DENSITY is a district decision too (device: "feels like a small
        // town"): urban quarters parcel small and build nearly wall-to-wall;
        // the financial core keeps big tower plates; suburbs keep their yards.
        const DistrictTag blockTag = districts.tagAt(centroid(foot));
        Real lotSetback = p.lotSetback;
        Real buildChance = p.buildChance;
        switch (blockTag) {
            case DistrictTag::Financial:
                pp.targetArea = 560; lotSetback = 1.0;
                buildChance = std::min(Real(1), p.buildChance + 0.06); break;
            case DistrictTag::Commercial:
                pp.targetArea = 300; lotSetback = 0.7;
                buildChance = std::min(Real(1), p.buildChance + 0.06); break;
            case DistrictTag::OldTown:
                pp.targetArea = 210; pp.minArea = std::min(p.minLotArea, Real(80));
                lotSetback = 0.5; buildChance = 0.98; break;
            case DistrictTag::Industrial:
                pp.targetArea = 700; lotSetback = 1.2; break;
            case DistrictTag::Residential:
                pp.targetArea = 400; break;
        }
        // Sometimes a WHOLE small block is a park (device: "the green space
        // doesn't conform to the city block"): the pad is the block's own
        // road-inset interior, so its edges follow the surrounding streets
        // exactly — a real city square, not a leftover parcel.
        if ((blockTag == DistrictTag::Commercial ||
             blockTag == DistrictTag::Residential) &&
            area(foot) < 2600.0) {
            Hash blockRng(mix(pp.seed, 0xB10Cu));
            if (blockRng.unit() < 0.10) {
                OBB2 gb = orientedBoundingBox(foot);
                LotBuilding g;
                g.site = centroid(foot);
                g.width = 2 * gb.half[0];
                g.depth = 2 * gb.half[1];
                g.height = 0.25;
                g.yaw = std::atan2(gb.axis[0].y, gb.axis[0].x);
                g.type = "park";
                g.color = colorFor("park");
                g.pad = foot;
                g.padMesh = padMeshFor(foot, g.height);
                out.push_back(std::move(g));
                continue;
            }
        }
        std::vector<Lot> lots = subdivideBlock(foot, pp);
        if (debug)
            for (const Lot& lot : lots) debug->lots.push_back(lot.footprint);

        for (std::size_t li = 0; li < lots.size(); ++li) {
            const Lot& lot = lots[li];
            if (area(lot.footprint) < pp.minArea) continue;
            Hash rng(mix(pp.seed, static_cast<uint32_t>(li) + 1));
            // An unbuilt lot is reported as a GREEN (device: "empty lots had
            // vegetation like trees and grass"), not silently dropped — the
            // caller plants grass + trees on it. Same for lots the sliver /
            // fill gates reject below.
            auto emitGreen = [&]() {
                OBB2 gb = orientedBoundingBox(lot.footprint);
                LotBuilding g;
                g.site = centroid(lot.footprint);
                g.width = 2 * gb.half[0];
                g.depth = 2 * gb.half[1];
                g.height = 0.12;
                g.yaw = std::atan2(gb.axis[0].y, gb.axis[0].x);
                g.type = "green";
                g.color = Vec3(0.32, 0.52, 0.30);
                g.pad = lot.footprint;
                g.padMesh = padMeshFor(g.pad, g.height);
                out.push_back(std::move(g));
            };
            if (rng.unit() > buildChance) {
                if (debug) debug->rejChance++;
                emitGreen(); continue;   // plaza / gap
            }

            // Building set back from its own lot lines (district-tuned).
            Poly2 site = inset(lot.footprint, lotSetback);
            if (site.size() < 3 || area(site) < 30) site = lot.footprint;

            OBB2 obb = orientedBoundingBox(site);
            const Real w = 2 * obb.half[0], d = 2 * obb.half[1];
            const Real shortSide = std::min(w, d), longSide = std::max(w, d);
            // Dense districts parcel small — an 8 m rowhouse plan is CORRECT
            // in old town — so the sliver floor relaxes there.
            const Real minShort =
                (blockTag == DistrictTag::OldTown ||
                 blockTag == DistrictTag::Commercial)
                    ? std::min(p.minShort, p.minShortUrban) : p.minShort;
            if (shortSide < minShort) {                                         // sliver
                if (debug) debug->rejSliver++;
                emitGreen(); continue;
            }
            if (longSide > shortSide * p.maxAspect) {                           // knife blade
                if (debug) debug->rejAspect++;
                emitGreen(); continue;
            }
            // How much of its oriented box the lot actually fills. PLAN massing
            // takes the polygon itself, so off-cut lots (L / triangle /
            // flatiron wedges) are buildable now — the very shapes that make
            // interesting buildings (device: "take advantage of the weirder
            // lots"). Only truly degenerate slivers go green here; the BOX
            // fallback below still demands the old 0.72 fill, since a box on a
            // low-fill lot is the "malformed overhanging mass" bug.
            const Real fill = area(site) / std::max(Real(1e-6), w * d);
            if (fill < 0.45) { if (debug) debug->rejFill++; emitGreen(); continue; }

            LotBuilding b;
            b.site = centroid(site);
            b.width = w;
            b.depth = d;
            b.yaw = std::atan2(obb.axis[0].y, obb.axis[0].x);
            const DistrictTag tag = districts.tagAt(b.site);
            // Coreness peaks the skyline: 1 at the city centre, 0 at the
            // financial district's rim — the architect grows the skyscraper
            // cluster from it. sqrt widens the peak so the cluster is a
            // CLUSTER, not one tall building at the exact centre.
            const Real coreness = std::sqrt(std::max(
                Real(0), 1.0 - (b.site - p.center).length() /
                                   std::max(Real(1), p.innerRadius)));
            BuildingRecipe rec =
                architectPick(tag, shortSide, area(site),
                              mix(pp.seed, static_cast<uint32_t>(li) * 7u + 3u),
                              coreness);
            b.type = rec.placeType;
            b.color = colorFor(b.type);
            if (rec.massing == BuildingRecipe::Massing::Park) {
                b.height = 0.3;   // a low green pad in the lot's own shape
                b.pad = lot.footprint;
                b.padMesh = padMeshFor(b.pad, b.height);
                out.push_back(std::move(b));
                continue;
            }
            // Grow a REAL building that FITS the lot: its oriented footprint IS the
            // scope — shrunk until it sits inside the lot AND clear of every road
            // corridor (clearOfRoads), so no mass overhangs a sidewalk. Height
            // comes from floors. Parts keep their shape-grammar PartId, merged
            // into outParts so the caller binds the SAME PBR material recipes the
            // shape:"city" pipeline uses — not a flattened vertex-colour blob.
            BuildingParams bp = rec.params;   // the architect's recipe
            // The door (and the retail front) faces the nearest STREET, not a
            // fixed +Z: aim faceDir at the closest point on the road network.
            if (roads) {
                Real best = 1e30;
                Vec2 q = b.site;
                for (const RoadEdge& e : roads->edges) {
                    if (e.a < 0 || e.b < 0 ||
                        e.a >= static_cast<int>(roads->nodes.size()) ||
                        e.b >= static_cast<int>(roads->nodes.size())) continue;
                    const Vec2& ra = roads->nodes[e.a].pos;
                    const Vec2& rb = roads->nodes[e.b].pos;
                    Vec2 ab = rb - ra;
                    Real len2 = ab.lengthSquared();
                    Real t = len2 > 1e-12 ? dot(b.site - ra, ab) / len2 : 0.0;
                    t = t < 0 ? 0 : (t > 1 ? 1 : t);
                    Vec2 cp(ra.x + ab.x * t, ra.y + ab.y * t);
                    Real dd = (cp - b.site).length();
                    if (dd < best) { best = dd; q = cp; }
                }
                Vec2 dir = q - b.site;
                if (dir.length() > 1e-6) {
                    dir = normalize(dir);
                    bp.faceDir = Vec3(dir.x, 0, dir.y);
                }
            }

            // PLAN massing (P3.c): the building takes the LOT'S OWN SHAPE — the
            // simplified site polygon (short edges merged so no micro-facades),
            // inset progressively until every vertex clears the road corridors.
            // Falls back to the shrink-fit box when the plan can't clear.
            Poly2 plan = site;
            {   // merge sub-2.5 m edges + drop near-collinear vertices
                Poly2 s;
                for (std::size_t vi = 0; vi < plan.size(); ++vi) {
                    const Vec2& prev = s.empty() ? plan.back() : s.back();
                    if ((plan[vi] - prev).length() < 2.5 && !s.empty()) continue;
                    s.push_back(plan[vi]);
                }
                Poly2 s2;
                for (std::size_t vi = 0; vi < s.size(); ++vi) {
                    Vec2 a = s[(vi + s.size() - 1) % s.size()], m2 = s[vi],
                         c2 = s[(vi + 1) % s.size()];
                    if (std::fabs(cross(normalize(m2 - a), normalize(c2 - m2))) < 0.03)
                        continue;
                    s2.push_back(m2);
                }
                // FLATIRON prows: a very acute corner would grow a knife-edge
                // facade — ROUND it into a short chord arc instead (device:
                // "rounded at the ends ... rather than becoming a perfect
                // corner"), the classic flatiron nose. Quadratic bezier
                // through the cut points with the sharp corner as control.
                Poly2 s3;
                for (std::size_t vi = 0; vi < s2.size(); ++vi) {
                    Vec2 a = s2[(vi + s2.size() - 1) % s2.size()], m2 = s2[vi],
                         c2 = s2[(vi + 1) % s2.size()];
                    Vec2 d0 = normalize(m2 - a), d1 = normalize(c2 - m2);
                    const Real cut = 3.0;
                    if (dot(d0, d1) < -0.45 && (m2 - a).length() > cut * 2 &&
                        (c2 - m2).length() > cut * 2) {
                        Vec2 p0 = m2 - d0 * cut, p1 = m2 + d1 * cut;
                        for (int k = 0; k <= 4; ++k) {
                            Real t = k / 4.0, mt = 1 - t;
                            s3.push_back(p0 * (mt * mt) + m2 * (2 * mt * t) +
                                         p1 * (t * t));
                        }
                    } else {
                        s3.push_back(m2);
                    }
                }
                if (s3.size() >= 3) plan = s3;
            }
            bool planOk = plan.size() >= 3 && area(plan) > p.minLotArea * 0.5;
            if (planOk && roads) {
                bool fit = false;
                for (Real t : {Real(0), Real(0.8), Real(1.6), Real(2.6), Real(3.6)}) {
                    Poly2 cand = t > 0 ? inset(plan, t) : plan;
                    if (cand.size() < 3 || area(cand) < 40) break;
                    // The MESH is the guarantee (lot buildings are visual-only
                    // — no box collider since the invisible-walls fix), so the
                    // plan's own vertices clearing the corridors is what keeps
                    // facades off the street.
                    bool clear = true;
                    for (const Vec2& v : cand)
                        if (!clearOfRoads(v)) { clear = false; break; }
                    if (clear) { plan = cand; fit = true; break; }
                }
                // Corner-pocket rescue: a curvy road bows into ONE corner of
                // the lot, and a global inset shrinks the whole plan to
                // nothing before that corner clears. Nudge just the offending
                // vertices toward the centroid instead — the rest of the lot
                // keeps its footprint (this was ~20% of all lots going green).
                if (!fit) {
                    Poly2 cand = plan;
                    const Vec2 c0 = centroid(cand);
                    bool ok = cand.size() >= 3;
                    for (Vec2& v : cand) {
                        int guard = 0;
                        while (!clearOfRoads(v) && guard++ < 8)
                            v = v + (c0 - v) * 0.18;
                        if (!clearOfRoads(v)) { ok = false; break; }
                    }
                    if (ok && area(cand) > std::max(Real(40), area(plan) * 0.5)) {
                        plan = cand;
                        fit = true;
                    }
                }
                if (!fit && debug) debug->rejClear++;   // (box fallback may still build)
                planOk = fit;
            }
            // COURTYARD massing (device: "a lot of same-y looking ones"): a
            // big boxy mid-rise lot carves a court into its BACK side — away
            // from the street — so the mass reads as an L / U / T plan with
            // wings instead of yet another extruded rectangle. The carve is
            // inward-only, so road clearance established above still holds.
            if (planOk && rec.massing == BuildingRecipe::Massing::LotPlan &&
                rec.params.floors >= 3 && area(plan) > 280 && rng.unit() < 0.5) {
                OBB2 cb = orientedBoundingBox(plan);
                if (area(plan) > 0.85 * (4 * cb.half[0] * cb.half[1])) {
                    Vec2 face(bp.faceDir.x, bp.faceDir.z);
                    const Real da = dot(cb.axis[0], face), db = dot(cb.axis[1], face);
                    const int backAxis = std::fabs(da) >= std::fabs(db) ? 0 : 1;
                    const Real backSign = (backAxis == 0 ? da : db) > 0 ? -1.0 : 1.0;
                    Vec2 v = cb.axis[backAxis] * backSign;      // toward the back
                    Vec2 u = cb.axis[1 - backAxis];             // along the back edge
                    const Real hu = cb.half[1 - backAxis], hv = cb.half[backAxis];
                    const Real w = 2 * hu * rng.range(0.30, 0.45);
                    const Real dpt = std::min(2 * hv * 0.35, rng.range(4.5, 7.5));
                    // Wings must stay walls, not slivers.
                    if (2 * hv - dpt > 6.0 && 2 * hu - w > 6.0 && dpt > 3.0) {
                        // Court at a corner (an L) or centred-ish (a U/T).
                        Real s0 = rng.unit() < 0.4
                                      ? (rng.unit() < 0.5 ? -hu + 2.5 : hu - w - 2.5)
                                      : -w * 0.5 + rng.range(-0.15, 0.15) * hu;
                        s0 = std::max(-hu + 2.5, std::min(hu - w - 2.5, s0));
                        const Vec2 C = cb.center;
                        Poly2 cand{C - v * hv - u * hu, C - v * hv + u * hu,
                                   C + v * hv + u * hu, C + v * hv + u * (s0 + w),
                                   C + v * (hv - dpt) + u * (s0 + w),
                                   C + v * (hv - dpt) + u * s0,
                                   C + v * hv + u * s0, C + v * hv - u * hu};
                        if (area(cand) < 0) std::reverse(cand.begin(), cand.end());
                        // The court rect is the plan's OBB — on a not-quite-
                        // rect plan its corners can poke past the cleared
                        // polygon, so re-check them against the roads.
                        bool ok = true;
                        if (roads)
                            for (const Vec2& q : cand)
                                if (!clearOfRoads(q)) { ok = false; break; }
                        if (ok) plan = cand;
                    }
                }
            }
            // A HOUSE sits on a small centred rectangle, not the whole lot —
            // the rest of the parcel reads as its yard (residential realism).
            if (planOk && rec.massing == BuildingRecipe::Massing::RectYard) {
                const Real hw2 = std::min(obb.half[0] - 1.6, Real(7.0));
                const Real hd2 = std::min(obb.half[1] - 1.6, Real(6.0));
                if (hw2 > 3.2 && hd2 > 3.2) {
                    Vec2 c = obb.center;
                    Poly2 house{c - obb.axis[0] * hw2 - obb.axis[1] * hd2,
                                c + obb.axis[0] * hw2 - obb.axis[1] * hd2,
                                c + obb.axis[0] * hw2 + obb.axis[1] * hd2,
                                c - obb.axis[0] * hw2 + obb.axis[1] * hd2};
                    // The house must sit ON its own lot (a low-fill off-cut's
                    // OBB centre can be outside the polygon) and off the road.
                    bool clear = true;
                    for (const Vec2& v : house)
                        if (!pointInPolygon(site, v) ||
                            (roads && !clearOfRoads(v))) { clear = false; break; }
                    if (clear) plan = house;
                }
            }
            // The architect's ROUND towers: a chord-tessellated circle plan
            // inscribed in the lot (a real drum, cornices and tiers included).
            if (planOk && rec.massing == BuildingRecipe::Massing::Circle &&
                shortSide > 17) {
                const Real rad = shortSide * 0.5 - 1.6;
                Poly2 circ;
                for (int k = 0; k < 16; ++k) {
                    Real a2 = 2.0 * 3.14159265358979323846 * k / 16;
                    circ.push_back(b.site + Vec2(std::cos(a2), std::sin(a2)) * rad);
                }
                bool clear = true;
                if (roads)
                    for (const Vec2& v : circ)
                        if (!clearOfRoads(v)) { clear = false; break; }
                if (clear) plan = circ;
            }

            // PLAN QUALITY (device: "a really degenerate triangle with sharp
            // edges and barely no space"): the OBB short side overestimates a
            // wedge's usable width, so gauge the finished plan by its
            // inradius-ish 4*area/perimeter. Too pinched → green; merely
            // wedge-shaped → the recipe shrinks to what the floor plate can
            // actually carry (no skyscraper on a knife of a lot).
            if (planOk) {
                Real per = 0;
                for (std::size_t vi = 0; vi < plan.size(); ++vi)
                    per += (plan[(vi + 1) % plan.size()] - plan[vi]).length();
                const Real effShort = std::min(
                    shortSide, 4 * area(plan) / std::max(per, Real(1e-6)));
                if (effShort < 5.5) {
                    if (debug) debug->rejFill++;
                    emitGreen(); continue;
                }
                const Real qq = effShort / std::max(shortSide, Real(1e-6));
                if (qq < 0.8)
                    bp.floors = std::max(1, static_cast<int>(bp.floors * qq));
            }

            BuildingMesh bm;
            if (planOk) {
                bm = growPlanBuilding(plan, bp);
                OBB2 pb = orientedBoundingBox(plan);
                b.site = pb.center;
                b.width = 2 * pb.half[0];
                b.depth = 2 * pb.half[1];
                b.yaw = std::atan2(pb.axis[0].y, pb.axis[0].x);
            } else {
                // The box fallback fills the OBB: on a low-fill lot that IS
                // the overhanging-mass bug, so those go green instead.
                if (fill < 0.72) { if (debug) debug->rejBox++; emitGreen(); continue; }
                Scope scope = scopeFromFootprint(site, 0.0, 10.0, clearOfRoads);
                const Real fitShort = std::min(scope.size.x, scope.size.z);
                if (fitShort < p.minShort * 0.75) {
                    if (debug) debug->rejBox++;
                    emitGreen(); continue;
                }
                Vec3 sc = scope.center();
                b.site = Vec2(sc.x, sc.z);
                b.width = scope.size.x;
                b.depth = scope.size.z;
                b.yaw = std::atan2(scope.axis[0].z, scope.axis[0].x);
                bm = growBuilding(scope, bp);
            }
            if (bm.parts.empty()) continue;
            if (outParts)
                for (const RenderMesh& part : bm.parts) {
                    const int mi = part.materialIndex;
                    if (mi >= 0 && mi < static_cast<int>(outParts->size()))
                        MeshBuilder::append((*outParts)[mi], part);
                }
            b.height = bm.height > 0 ? bm.height : 8.0;
            out.push_back(std::move(b));
        }
    }
    return out;
}


namespace {
// Distance from p to segment [a,b].
Real segDist(const Vec2& p, const Vec2& a, const Vec2& b) {
    Vec2 ab = b - a;
    Real len2 = ab.lengthSquared();
    Real t = len2 > 1e-12 ? dot(p - a, ab) / len2 : 0.0;
    t = t < 0 ? 0 : (t > 1 ? 1 : t);
    Vec2 q(a.x + ab.x * t, a.y + ab.y * t);
    return (p - q).length();
}
}  // namespace

std::vector<Poly2> edgeBlocks(const RoadGraph& roads,
                              const std::vector<Poly2>& closedBlocks,
                              const EdgeBlockParams& p) {
    std::vector<Poly2> out;
    const int n = static_cast<int>(roads.nodes.size());
    if (n == 0) return out;

    // Adjacency + degree, then walk maximal CHAINS between non-degree-2 ends
    // (each chain is one road run between junctions / dead ends).
    std::vector<std::vector<std::pair<int, int>>> adj(n);   // node -> {edge, other}
    for (std::size_t ei = 0; ei < roads.edges.size(); ++ei) {
        const RoadEdge& e = roads.edges[ei];
        if (e.a < 0 || e.b < 0 || e.a >= n || e.b >= n || e.a == e.b) continue;
        adj[e.a].push_back({static_cast<int>(ei), e.b});
        adj[e.b].push_back({static_cast<int>(ei), e.a});
    }
    std::vector<uint8_t> used(roads.edges.size(), 0);

    // A candidate rectangle is kept only if its centre is truly OPEN ground:
    // inside no closed block, and no OTHER road passes near it.
    auto isOpen = [&](const Vec2& c) {
        for (const Poly2& b : closedBlocks)
            if (pointInPolygon(b, c)) return false;
        for (const RoadEdge& e : roads.edges) {
            if (e.a < 0 || e.b < 0 || e.a >= n || e.b >= n) continue;
            if (segDist(c, roads.nodes[e.a].pos, roads.nodes[e.b].pos) <
                p.margin + p.depth * 0.35)
                return false;   // some road runs through/near this ground
        }
        return true;
    };

    for (int start = 0; start < n; ++start) {
        if (adj[start].size() == 2) continue;   // chain interior, not an end
        for (const auto& [e0, n0] : adj[start]) {
            if (used[e0]) continue;
            // Walk the chain from `start` through degree-2 nodes.
            std::vector<Vec2> line{roads.nodes[start].pos};
            int prev = start, cur = n0, edge = e0;
            used[edge] = 1;
            line.push_back(roads.nodes[cur].pos);
            while (adj[cur].size() == 2) {
                const auto& [ea, na] = adj[cur][0];
                const auto& [eb, nb] = adj[cur][1];
                int nextEdge = (na == prev && !used[eb]) ? eb
                             : (nb == prev && !used[ea]) ? ea
                             : -1;
                if (nextEdge < 0) break;
                prev = cur;
                cur = roads.edges[nextEdge].a == cur ? roads.edges[nextEdge].b
                                                     : roads.edges[nextEdge].a;
                used[nextEdge] = 1;
                line.push_back(roads.nodes[cur].pos);
            }
            // Arc length; subdivide into pieces within [minLen, maxLen].
            Real total = 0;
            for (std::size_t i = 1; i < line.size(); ++i)
                total += (line[i] - line[i - 1]).length();
            if (total < p.minLen) continue;
            const int pieces = std::max(1, static_cast<int>(total / p.maxLen) + 
                                           (std::fmod(total, p.maxLen) > p.minLen ? 1 : 0));
            const Real pieceLen = total / pieces;
            // Point at arc-length s along the polyline.
            auto at = [&](Real s) {
                for (std::size_t i = 1; i < line.size(); ++i) {
                    Real seg = (line[i] - line[i - 1]).length();
                    if (s <= seg || i + 1 == line.size())
                        return line[i - 1] + (line[i] - line[i - 1]) *
                                                 (seg > 1e-9 ? s / seg : 0.0);
                    s -= seg;
                }
                return line.back();
            };
            for (int k = 0; k < pieces; ++k) {
                Vec2 a = at(k * pieceLen + 2.0);          // small end setbacks so
                Vec2 b = at((k + 1) * pieceLen - 2.0);    // neighbours don't touch
                Vec2 d = b - a;
                Real len = d.length();
                if (len < p.minLen * 0.6) continue;
                d = d / len;
                Vec2 nrm(-d.y, d.x);
                for (Real side : {Real(1), Real(-1)}) {
                    Vec2 c = (a + b) * 0.5 + nrm * side * (p.margin + p.depth * 0.5);
                    if (!isOpen(c)) continue;
                    Vec2 i0 = a + nrm * side * p.margin;
                    Vec2 i1 = b + nrm * side * p.margin;
                    Vec2 o1 = b + nrm * side * (p.margin + p.depth);
                    Vec2 o0 = a + nrm * side * (p.margin + p.depth);
                    Poly2 rect{i0, i1, o1, o0};
                    ensureCCW(rect);
                    // Rim blocks from DIFFERENT chains can land on the same open
                    // ground near a corner — two overlapping blocks grew two
                    // buildings through each other (device: "buildings
                    // intersecting with one another"). First-come wins; a rect
                    // overlapping an accepted one (SAT on the convex quads) is
                    // dropped.
                    bool overlaps = false;
                    const Poly2& rectRef = rect;
                    for (const Poly2& q : out) {
                        bool separated = false;
                        for (const Poly2* poly : {&rectRef, &q}) {
                            for (std::size_t ei = 0; ei < poly->size() && !separated; ++ei) {
                                Vec2 ed = (*poly)[(ei + 1) % poly->size()] - (*poly)[ei];
                                Vec2 ax(-ed.y, ed.x);
                                Real lo0 = 1e30, hi0 = -1e30, lo1 = 1e30, hi1 = -1e30;
                                for (const Vec2& v : rect) {
                                    Real t = dot(ax, v);
                                    lo0 = std::min(lo0, t); hi0 = std::max(hi0, t);
                                }
                                for (const Vec2& v : q) {
                                    Real t = dot(ax, v);
                                    lo1 = std::min(lo1, t); hi1 = std::max(hi1, t);
                                }
                                if (hi0 < lo1 || hi1 < lo0) separated = true;
                            }
                            if (separated) break;
                        }
                        if (!separated) { overlaps = true; break; }
                    }
                    if (overlaps) continue;
                    out.push_back(std::move(rect));
                }
            }
        }
    }
    return out;
}

}  // namespace engine
