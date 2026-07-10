#include "city_lots.h"

#include "parcel.h"          // subdivideBlock, Lot, ParcelParams
#include "road_net.h"        // RoadNet + navRoadGraph (growLotBuildingsOnNets)
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
RenderMesh padMeshFor(const Poly2& poly, Real h,
                      const std::function<Real(Real, Real)>& ground) {
    RenderMesh m;
    const Vec3 white(1, 1, 1);
    auto gy = [&](const Vec2& v) { return ground ? ground(v.x, v.y) : Real(0); };
    for (const std::array<int, 3>& t : triangulatePolygon(poly))
        MeshBuilder::emitTri(
            m, Vec3(poly[t[0]].x, gy(poly[t[0]]) + h, poly[t[0]].y),
            Vec3(poly[t[1]].x, gy(poly[t[1]]) + h, poly[t[1]].y),
            Vec3(poly[t[2]].x, gy(poly[t[2]]) + h, poly[t[2]].y),
            Vec3(0, 1, 0), white);
    for (std::size_t i = 0; i < poly.size(); ++i) {
        const Vec2& a = poly[i];
        const Vec2& b = poly[(i + 1) % poly.size()];
        Vec2 d = b - a;
        if (d.length() < 1e-6) continue;
        Vec2 n = normalize(Vec2(d.y, -d.x));   // CCW plan: outward
        // Skirt from the draped top down past the terrain surface.
        MeshBuilder::emitQuad(m, Vec3(a.x, gy(a) - 0.4, a.y),
                              Vec3(b.x, gy(b) - 0.4, b.y),
                              Vec3(b.x, gy(b) + h, b.y),
                              Vec3(a.x, gy(a) + h, a.y),
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
    districts.hubs = p.hubs;
    districts.hubRadius = p.hubRadius;
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
    // TERRAIN base for a plan: the LOWEST ground under its vertices so the
    // downhill corner never floats, embedded slightly on real slopes so the
    // uphill side beds in instead of hovering behind a knife-edge gap.
    auto baseYFor = [&](const Poly2& pl) -> Real {
        if (!p.ground || pl.empty()) return 0;
        Real lo = 1e30, hi = -1e30;
        for (const Vec2& v : pl) {
            const Real g = p.ground(v.x, v.y);
            lo = std::min(lo, g);
            hi = std::max(hi, g);
        }
        return lo - ((hi - lo) > 0.05 ? Real(0.25) : Real(0));
    };
    // TERRAIN pad plane (device: "the terrain should be flat under the
    // building" + "the entrance should be as level as possible with the
    // sidewalk it's next to"): the grade at the ENTRANCE side — cast a ray
    // from the plan centroid along the face direction to the boundary, step a
    // couple of metres toward the street (onto the road-conformed apron, i.e.
    // the sidewalk's own grade), and sample there. The host stamps a flatten
    // pad at this plane, so the walls meet FLAT graded earth and the front
    // door meets the sidewalk. Falls back to the vertex average when the ray
    // finds no boundary (degenerate plans).
    auto padPlaneFor = [&](const Poly2& pl, const Vec2& face) -> Real {
        if (!p.ground || pl.empty()) return 0;
        const Vec2 c = centroid(pl);
        Real tExit = -1;
        if (face.length() > Real(1e-6)) {
            const Vec2 f = normalize(face);
            const std::size_t n = pl.size();
            for (std::size_t i = 0; i < n; ++i) {
                const Vec2& a = pl[i];
                const Vec2& b = pl[(i + 1) % n];
                const Vec2 e = b - a;
                const Real den = cross(f, e);
                if (std::fabs(den) < Real(1e-9)) continue;
                const Real t = cross(a - c, e) / den;       // along the ray
                const Real u = cross(a - c, f) / den;       // along the edge
                if (t > 0 && u >= 0 && u <= 1) tExit = std::max(tExit, t);
            }
            if (tExit > 0) {
                const Vec2 E = c + f * (tExit + Real(2.0));
                return p.ground(E.x, E.y);
            }
        }
        Real sum = 0;
        for (const Vec2& v : pl) sum += p.ground(v.x, v.y);
        return sum / static_cast<Real>(pl.size());
    };
    // Plinth reveal: walls start this far above the graded pad, on a visible
    // FOUNDATION course (device: "there should be some kind of a base for the
    // building and steps to get up to the front door"). Host-tunable.
    const Real plinth = p.ground ? std::max(Real(0), p.plinth) : Real(0);
    // The foundation course: the plan outset slightly, extruded from below the
    // pad up to the wall base — a concrete band that grounds the massing and
    // hides the terrain seam. Emitted into the Concrete part like any other
    // grammar element.
    auto emitFoundation = [&](const Poly2& plIn, Real planeY, Real topY) {
        if (!outParts || plIn.size() < 3 || !p.ground) return;
        Poly2 pl = plIn;
        if (area(pl) < 0) std::reverse(pl.begin(), pl.end());   // CCW: right normal = outward
        const std::size_t nv = pl.size();
        Poly2 o(nv);
        for (std::size_t i = 0; i < nv; ++i) {
            const Vec2& a = pl[(i + nv - 1) % nv];
            const Vec2& b = pl[i];
            const Vec2& c = pl[(i + 1) % nv];
            Vec2 e0 = b - a, e1 = c - b;
            auto rn = [](Vec2 e) {
                Vec2 n(e.y, -e.x); Real l = n.length();
                return l < Real(1e-9) ? Vec2(0, 0) : n * (1 / l);
            };
            Vec2 n0 = rn(e0), n1 = rn(e1);
            Vec2 bis = n0 + n1; Real bl = bis.length();
            Vec2 mm = bl < Real(1e-9) ? n1 : bis * (1 / bl);
            const Real cosH = std::max(Real(0.35), dot(mm, n1));
            o[i] = b + mm * (Real(0.14) / cosH);
        }
        RenderMesh& m = (*outParts)[static_cast<std::size_t>(PartId::Concrete)];
        const Vec3 col(1, 1, 1);   // Concrete's surface maps carry the look
        const Real botY = planeY - Real(0.6);   // skirt below the graded pad
        for (std::size_t i = 0; i < nv; ++i) {
            const std::size_t j = (i + 1) % nv;
            Vec2 e = o[j] - o[i];
            if (e.length() < Real(1e-9)) continue;
            Vec2 n = normalize(Vec2(e.y, -e.x));
            MeshBuilder::emitQuad(m, Vec3(o[i].x, botY, o[i].y),
                                  Vec3(o[j].x, botY, o[j].y),
                                  Vec3(o[j].x, topY, o[j].y),
                                  Vec3(o[i].x, topY, o[i].y),
                                  Vec3(n.x, 0, n.y), col);
            // The exposed ledge between the foundation's outer lip and the wall.
            MeshBuilder::emitQuad(m, Vec3(o[i].x, topY, o[i].y),
                                  Vec3(o[j].x, topY, o[j].y),
                                  Vec3(pl[j].x, topY, pl[j].y),
                                  Vec3(pl[i].x, topY, pl[i].y),
                                  Vec3(0, 1, 0), col);
        }
    };
    if (outParts) {
        outParts->assign(static_cast<std::size_t>(PartId::Count), RenderMesh{});
        for (std::size_t i = 0; i < outParts->size(); ++i)
            (*outParts)[i].materialIndex = static_cast<int>(i);
    }
    // ---- PASS A: parcel every block and COLLECT the viable lots ------------
    // The landmark planner (pass B) needs to see the whole city before any lot
    // builds — a courthouse goes on the BEST financial lot, not the first one.
    struct BlockInfo {
        ParcelParams pp;
        DistrictTag tag;
        Real lotSetback, buildChance;
    };
    struct LotCand {
        std::size_t li;      // lot index within its block (the rng stream id)
        int block;           // index into binfos
        Lot lot;
        int landmark = -1;   // LandmarkKind once the planner assigns one
    };
    std::vector<BlockInfo> binfos;
    std::vector<LotCand> cands;
    for (std::size_t bi = 0; bi < blocks.size(); ++bi) {
        const Poly2& block = blocks[bi];
        if (block.size() < 3) continue;
        // Pull in from the road edge to the buildable interior (road + sidewalk).
        Poly2 foot = inset(block, p.roadMargin);
        if (foot.size() < 3 || area(foot) < p.minLotArea * 1.5) continue;
        if (debug) debug->blocks.push_back(foot);

        BlockInfo bf;
        bf.pp.seed = mix(static_cast<uint32_t>(bi), p.seed);
        bf.pp.targetArea = 480;
        bf.pp.minArea = p.minLotArea;
        bf.pp.minEdge = p.minShort;
        // DENSITY is a district decision too (device: "feels like a small
        // town"): urban quarters parcel small and build nearly wall-to-wall;
        // the financial core keeps big tower plates; suburbs keep their yards.
        bf.tag = districts.tagAt(centroid(foot));
        bf.lotSetback = p.lotSetback;
        bf.buildChance = p.buildChance;
        switch (bf.tag) {
            case DistrictTag::Financial:
                bf.pp.targetArea = 560; bf.lotSetback = 1.0;
                bf.buildChance = std::min(Real(1), p.buildChance + 0.06); break;
            case DistrictTag::Commercial:
                bf.pp.targetArea = 300; bf.lotSetback = 0.7;
                bf.buildChance = std::min(Real(1), p.buildChance + 0.06); break;
            case DistrictTag::OldTown:
                bf.pp.targetArea = 210;
                bf.pp.minArea = std::min(p.minLotArea, Real(80));
                bf.lotSetback = 0.5; bf.buildChance = 0.98; break;
            case DistrictTag::Industrial:
                bf.pp.targetArea = 700; bf.lotSetback = 1.2; break;
            case DistrictTag::Residential:
                bf.pp.targetArea = 400; break;
        }
        // Sometimes a WHOLE small block is a park (device: "the green space
        // doesn't conform to the city block"): the pad is the block's own
        // road-inset interior, so its edges follow the surrounding streets
        // exactly — a real city square, not a leftover parcel.
        if ((bf.tag == DistrictTag::Commercial ||
             bf.tag == DistrictTag::Residential) &&
            area(foot) < 2600.0) {
            Hash blockRng(mix(bf.pp.seed, 0xB10Cu));
            if (blockRng.unit() < 0.10) {
                OBB2 gb = orientedBoundingBox(foot);
                LotBuilding g;
                g.site = centroid(foot);
                g.width = 2 * gb.half[0];
                g.depth = 2 * gb.half[1];
                g.height = 0.25;
                g.yaw = std::atan2(gb.axis[0].y, gb.axis[0].x);
                g.type = "park";
                g.recipe = "park_block";
                g.color = colorFor("park");
                g.pad = foot;
                g.padMesh = padMeshFor(foot, g.height, p.ground);
                out.push_back(std::move(g));
                continue;
            }
        }
        std::vector<Lot> lots = subdivideBlock(foot, bf.pp);
        if (debug)
            for (const Lot& lot : lots) debug->lots.push_back(lot.footprint);
        binfos.push_back(bf);
        for (std::size_t li = 0; li < lots.size(); ++li) {
            if (area(lots[li].footprint) < bf.pp.minArea) continue;
            cands.push_back({li, static_cast<int>(binfos.size()) - 1,
                             lots[li], -1});
        }
    }

    // ---- PASS B: the LANDMARK planner ---------------------------------------
    // Civic anchors are PLACED, never rolled: quotas per city (one courthouse,
    // one hospital, a school per residential quarter...) filled by the best-
    // scoring eligible lot — biggest, and for the courthouse most central.
    {
        int nRes = 0, nCom = 0, nFin = 0, nOld = 0, nInd = 0;
        for (const LotCand& c : cands) {
            switch (binfos[c.block].tag) {
                case DistrictTag::Residential: ++nRes; break;
                case DistrictTag::Commercial:  ++nCom; break;
                case DistrictTag::Financial:   ++nFin; break;
                case DistrictTag::OldTown:     ++nOld; break;
                case DistrictTag::Industrial:  ++nInd; break;
            }
        }
        struct Want {
            LandmarkKind kind;
            int count;
            Real minShort, minArea;
            bool wantCore;   // score by centrality too (the courthouse)
            DistrictTag tagA, tagB;   // eligible districts (B may repeat A)
        };
        const int total = static_cast<int>(cands.size());
        const Want wants[] = {
            {LandmarkKind::Capitol, (nFin + nCom) >= 6 ? 1 : 0, 13.0, 300.0,
             true, DistrictTag::Financial, DistrictTag::Commercial},
            {LandmarkKind::University, total >= 60 ? 1 : 0, 15.0, 380.0, false,
             DistrictTag::Residential, DistrictTag::Commercial},
            {LandmarkKind::Courthouse, nFin >= 2 ? 1 : 0, 12.0, 260.0, true,
             DistrictTag::Financial, DistrictTag::Financial},
            {LandmarkKind::Hospital, nCom >= 6 ? 1 : 0, 15.0, 380.0, false,
             DistrictTag::Commercial, DistrictTag::Commercial},
            {LandmarkKind::School, nRes >= 6 ? 1 + nRes / 50 : 0, 13.0, 320.0,
             false, DistrictTag::Residential, DistrictTag::Residential},
            {LandmarkKind::Police, nCom >= 4 ? 1 : 0, 9.0, 140.0, false,
             DistrictTag::Commercial, DistrictTag::Commercial},
            {LandmarkKind::Fire, (nCom + nInd) >= 8 ? 1 + total / 150 : 0,
             11.0, 220.0, false, DistrictTag::Commercial,
             DistrictTag::Industrial},
            {LandmarkKind::Market, nOld >= 3 ? 1 : (nCom >= 8 ? 1 : 0),
             10.0, 180.0, false,
             nOld >= 3 ? DistrictTag::OldTown : DistrictTag::Commercial,
             nOld >= 3 ? DistrictTag::OldTown : DistrictTag::Commercial},
        };
        for (const Want& w : wants) {
            for (int k = 0; k < w.count; ++k) {
                int best = -1;
                Real bestScore = -1;
                // Preferred thresholds first; if no lot in the district can
                // carry them (small towns parcel small), relax once — the
                // quarter still gets its school, just a modest one.
                for (Real relax : {Real(1.0), Real(0.72)}) {
                    for (std::size_t ci = 0; ci < cands.size(); ++ci) {
                        const LotCand& c = cands[ci];
                        if (c.landmark >= 0) continue;
                        const DistrictTag t = binfos[c.block].tag;
                        if (t != w.tagA && t != w.tagB) continue;
                        OBB2 ob = orientedBoundingBox(c.lot.footprint);
                        const Real shortS = 2 * std::min(ob.half[0], ob.half[1]);
                        const Real ar = area(c.lot.footprint);
                        if (shortS < w.minShort * relax || ar < w.minArea * relax)
                            continue;
                        Real score = ar;
                        if (w.wantCore) {
                            const Real r =
                                (centroid(c.lot.footprint) - p.center).length();
                            score *= 0.4 + std::max(
                                Real(0),
                                1.0 - r / std::max(Real(1), p.innerRadius));
                        }
                        if (score > bestScore) {
                            bestScore = score;
                            best = static_cast<int>(ci);
                        }
                    }
                    if (best >= 0) break;
                }
                if (best < 0) break;   // no lot can carry it: skip the quota
                cands[best].landmark = static_cast<int>(w.kind);
            }
        }
    }

    // ---- PASS C: grow every lot (landmarks use their PLACED recipes) --------
    for (const LotCand& cand : cands) {
        {
            const BlockInfo& bf = binfos[cand.block];
            const ParcelParams& pp = bf.pp;
            const DistrictTag blockTag = bf.tag;
            const Real lotSetback = bf.lotSetback;
            const Real buildChance = bf.buildChance;
            const std::size_t li = cand.li;
            const Lot& lot = cand.lot;
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
                g.recipe = "green";
                g.color = Vec3(0.32, 0.52, 0.30);
                g.pad = lot.footprint;
                g.padMesh = padMeshFor(g.pad, g.height, p.ground);
                out.push_back(std::move(g));
            };
            if (cand.landmark < 0 && rng.unit() > buildChance) {
                if (debug) debug->rejChance++;
                emitGreen(); continue;   // plaza / gap (landmarks always build)
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
                cand.landmark >= 0
                    ? architectLandmark(static_cast<LandmarkKind>(cand.landmark),
                                        shortSide, area(site),
                                        mix(pp.seed,
                                            static_cast<uint32_t>(li) * 7u + 3u))
                    : architectPick(tag, shortSide, area(site),
                                    mix(pp.seed,
                                        static_cast<uint32_t>(li) * 7u + 3u),
                                    coreness);
            b.type = rec.placeType;
            b.recipe = rec.name;
            b.color = colorFor(b.type);
            if (rec.massing == BuildingRecipe::Massing::Park) {
                b.height = 0.3;   // a low green pad in the lot's own shape
                b.pad = lot.footprint;
                b.padMesh = padMeshFor(b.pad, b.height, p.ground);
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
            // The STYLE BOOK (Lua data layer) overlays look overrides by
            // recipe name — cladding, windows, colours — before growth.
            if (p.styleHook) p.styleHook(rec.name, bp);
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
                const Real hw2 = std::min(obb.half[0] - 1.6, rec.yardHalfWMax);
                const Real hd2 = std::min(obb.half[1] - 1.6, rec.yardHalfDMax);
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
            // ROWHOUSES (device: "town homes ... packed side by side"): the
            // lot becomes a terrace of narrow townhome UNITS sharing party
            // walls — each its own plan building (door, stoop, cladding,
            // sometimes its own gable) grown side by side with zero gaps.
            if (planOk && rec.massing == BuildingRecipe::Massing::RowStrip) {
                OBB2 sb = orientedBoundingBox(plan);
                const int la = sb.longAxis(), sa2 = 1 - la;
                const Real len = 2 * sb.half[la];
                const Real dep = std::min(2 * sb.half[sa2], Real(11.0));
                const int units =
                    static_cast<int>(len / rng.range(5.6, 7.0));
                Vec2 u = sb.axis[la], v = sb.axis[sa2];
                bool stripOk = units >= 3 && dep > 6.5;
                if (stripOk && roads)
                    for (int sx = -1; sx <= 1 && stripOk; sx += 2)
                        for (int sy = -1; sy <= 1; sy += 2) {
                            Vec2 corner = sb.center + u * (len * 0.5 * sx) +
                                          v * (dep * 0.5 * sy);
                            if (!clearOfRoads(corner)) { stripOk = false; break; }
                        }
                if (stripOk) {
                    const Real uw = len / units;
                    const Vec2 c0 = sb.center - u * (len * 0.5);
                    // One shared pad plane for the whole terrace: the strip is
                    // graded flat as ONE pad, so the party-wall units sit flush
                    // on it instead of staggering into the cut.
                    b.groundY = padPlaneFor(plan, Vec2(bp.faceDir.x, bp.faceDir.z));
                    const Real stripBase =
                        p.ground ? b.groundY + plinth : baseYFor(plan);
                    for (int k = 0; k < units; ++k) {
                        Poly2 up4{c0 + u * (uw * k) - v * (dep * 0.5),
                                  c0 + u * (uw * (k + 1)) - v * (dep * 0.5),
                                  c0 + u * (uw * (k + 1)) + v * (dep * 0.5),
                                  c0 + u * (uw * k) + v * (dep * 0.5)};
                        BuildingParams upar = architectRowUnit(
                            mix(bp.seed, static_cast<uint32_t>(k) * 31u + 7u),
                            bp.floors);
                        upar.faceDir = bp.faceDir;
                        if (p.styleHook) p.styleHook("rowhouse_unit", upar);
                        BuildingMesh um = growPlanBuilding(up4, upar, stripBase);
                        if (outParts)
                            for (const RenderMesh& part : um.parts) {
                                const int mi = part.materialIndex;
                                if (mi >= 0 &&
                                    mi < static_cast<int>(outParts->size()))
                                    MeshBuilder::append((*outParts)[mi], part);
                            }
                        b.height = std::max(b.height, um.height);
                    }
                    b.site = sb.center;
                    b.baseY = stripBase;
                    b.width = 2 * sb.half[0];
                    b.depth = 2 * sb.half[1];
                    b.yaw = std::atan2(sb.axis[0].y, sb.axis[0].x);
                    b.plan = {sb.center - u * (len * 0.5) - v * (dep * 0.5),
                              sb.center + u * (len * 0.5) - v * (dep * 0.5),
                              sb.center + u * (len * 0.5) + v * (dep * 0.5),
                              sb.center - u * (len * 0.5) + v * (dep * 0.5)};
                    emitFoundation(b.plan, b.groundY, b.baseY);
                    out.push_back(std::move(b));
                    continue;
                }
                // Too short/shallow for a terrace: build as one plan building.
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
            // On terrain the building rises from its graded pad plane (plus the
            // plinth reveal); every walk-up entrance earns steps to the door —
            // porticos and bay-door fronts already bring their own.
            b.groundY = padPlaneFor(planOk ? plan : site,
                                    Vec2(bp.faceDir.x, bp.faceDir.z));
            b.baseY = p.ground ? b.groundY + plinth
                               : baseYFor(planOk ? plan : site);
            if (p.ground && !bp.entranceSteps && bp.portico == 0 &&
                bp.groundBays == 0 && bp.floors > 0)
                bp.entranceSteps = true;
            if (planOk) {
                bm = growPlanBuilding(plan, bp, b.baseY);
                OBB2 pb = orientedBoundingBox(plan);
                b.site = pb.center;
                b.width = 2 * pb.half[0];
                b.depth = 2 * pb.half[1];
                b.yaw = std::atan2(pb.axis[0].y, pb.axis[0].x);
                b.plan = plan;   // the collider prism follows the massing
            } else {
                // The box fallback fills the OBB: on a low-fill lot that IS
                // the overhanging-mass bug, so those go green instead.
                if (fill < 0.72) { if (debug) debug->rejBox++; emitGreen(); continue; }
                Scope scope = scopeFromFootprint(site, b.baseY, 10.0, clearOfRoads);
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
                // The box scope IS the plan here (shrunk-fit inside the lot).
                Vec2 r2(scope.axis[0].x, scope.axis[0].z);
                Vec2 f2(scope.axis[2].x, scope.axis[2].z);
                Vec2 o2(scope.origin.x, scope.origin.z);
                b.plan = {o2, o2 + r2 * scope.size.x,
                          o2 + r2 * scope.size.x + f2 * scope.size.z,
                          o2 + f2 * scope.size.z};
            }
            if (bm.parts.empty()) continue;
            if (outParts)
                for (const RenderMesh& part : bm.parts) {
                    const int mi = part.materialIndex;
                    if (mi >= 0 && mi < static_cast<int>(outParts->size()))
                        MeshBuilder::append((*outParts)[mi], part);
                }
            b.height = bm.height > 0 ? bm.height : 8.0;
            emitFoundation(b.plan, b.groundY, b.baseY);
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

NetLotResult growLotBuildingsOnNets(const std::vector<RoadNet>& nets,
                                    const LotParams& params,
                                    const EdgeBlockParams& edgeParams,
                                    Real roadClearance) {
    NetLotResult r;
    // One combined raw planar graph across every net (the sampled navRoadGraph
    // loses faces, so the block extraction uses the nets' own nodes/edges) —
    // plus the SAMPLED centrelines (what the asphalt is actually meshed from;
    // a curvy road bows off its control chords) with real per-edge widths, for
    // the building road-clearance check.
    RoadGraph rg, rgSampled;
    for (const RoadNet& net : nets) {
        const int base = static_cast<int>(rg.nodes.size());
        for (const Vec2& n : net.nodes) rg.nodes.push_back({n});
        for (std::size_t ei = 0; ei < net.edges.size(); ++ei) {
            const auto& e = net.edges[ei];
            const float w = static_cast<float>(
                roadNetEdgeWidth(net, static_cast<int>(ei)));
            rg.edges.push_back(RoadEdge{base + e[0], base + e[1], w,
                                        RoadClass::Local, 0});
        }
        RoadGraph s = navRoadGraph(net);
        const int sBase = static_cast<int>(rgSampled.nodes.size());
        for (const auto& n : s.nodes) rgSampled.nodes.push_back(n);
        for (const auto& e : s.edges)
            rgSampled.edges.push_back(RoadEdge{sBase + e.a, sBase + e.b,
                                               e.width, e.klass, e.layer});
    }
    std::vector<Poly2> blocks = extractBlocks(rg);
    // Rim blocks: the town edge has no enclosed faces — synthesize rectangles
    // on the boundary roads' open sides so the outskirts build up too.
    std::vector<Poly2> rim = edgeBlocks(rg, blocks, edgeParams);
    blocks.insert(blocks.end(), rim.begin(), rim.end());
    r.lots = growLotBuildings(blocks, params, &r.plan, &r.parts, &rgSampled,
                              roadClearance);
    return r;
}

}  // namespace engine
