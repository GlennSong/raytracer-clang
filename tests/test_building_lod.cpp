// The building middle LOD (city-render-perf R2). The invariant that matters:
// Flat is the SAME building — same seed, same massing, same bay and opening
// layout, because both detail levels consume one facadeLayout — drawn with a
// fraction of the triangles. These tests hold the ratio and the sharing, so a
// future edit cannot quietly fork the two levels apart.
#include "test_framework.h"
#include "../src/engine/procgen/city/city_lots.h"
#include "../src/engine/procgen/city/shape_grammar.h"
#include "../src/engine/procgen/city/architect.h"   // the every-recipe census
#include "../src/engine/mesh_audit.h"                // facade cracks

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <map>
#include <string>
#include <tuple>

using namespace engine;

namespace {

std::size_t tris(const BuildingMesh& b) {
    std::size_t n = 0;
    for (const RenderMesh& p : b.parts) n += p.indices.size() / 3;
    return n;
}
std::size_t tris(const std::vector<RenderMesh>& parts) {
    std::size_t n = 0;
    for (const RenderMesh& p : parts) n += p.indices.size() / 3;
    return n;
}
const RenderMesh* part(const BuildingMesh& b, PartId id) {
    for (const RenderMesh& p : b.parts)
        if (p.materialIndex == static_cast<int>(id)) return &p;
    return nullptr;
}

BuildingParams midriseParams() {
    BuildingParams p;
    p.floors = 6;
    p.groundRetail = true;
    p.walkableGround = true;
    p.seed = 41;
    return p;
}

}  // namespace

TEST_CASE(flat_detail_is_a_fraction_of_full) {
    const Poly2 plan{{0, 0}, {22, 0}, {22, 16}, {0, 16}};
    BuildingParams p = midriseParams();
    const BuildingMesh full = growPlanBuilding(plan, p);
    const BuildingMesh flat = growPlanBuilding(plan, p, 0.0, FacadeDetail::Flat);
    CHECK(tris(flat) > 0);
    CHECK(tris(flat) * 4 < tris(full));           // the point of the tier
    // Deterministic: the same seed grows the same flat building.
    const BuildingMesh again = growPlanBuilding(plan, p, 0.0, FacadeDetail::Flat);
    CHECK(tris(again) == tris(flat));
}

TEST_CASE(flat_detail_keeps_the_openings) {
    const Poly2 plan{{0, 0}, {22, 0}, {22, 16}, {0, 16}};
    BuildingParams p = midriseParams();
    const BuildingMesh flat = growPlanBuilding(plan, p, 0.0, FacadeDetail::Flat);
    // The layout survives: windows and the door exist as flat panes — one quad
    // (2 tris) per opening, none of the frame/muntin/surround joinery.
    const RenderMesh* glass = part(flat, PartId::Glass);
    const RenderMesh* door = part(flat, PartId::Door);
    CHECK(glass != nullptr && glass->indices.size() >= 6);
    CHECK(door != nullptr && door->indices.size() >= 6);
    CHECK(part(flat, PartId::Detail) == nullptr ||
          part(flat, PartId::Detail)->indices.empty());   // no joinery at LOD1
}

TEST_CASE(flat_curtain_wall_keeps_the_banding) {
    const Poly2 plan{{0, 0}, {24, 0}, {24, 20}, {0, 20}};
    BuildingParams p = midriseParams();
    p.floors = 12;
    p.curtainWall = true;
    const BuildingMesh full = growPlanBuilding(plan, p);
    const BuildingMesh flat = growPlanBuilding(plan, p, 0.0, FacadeDetail::Flat);
    CHECK(tris(flat) > 0);
    CHECK(tris(flat) * 3 < tris(full));           // the mullion lattice is gone
    const RenderMesh* glass = part(flat, PartId::Glass);
    CHECK(glass != nullptr && !glass->indices.empty());
}

TEST_CASE(lot_growth_emits_the_flat_tier_on_request) {
    const Poly2 block{{0, 0}, {120, 0}, {120, 90}, {0, 90}};
    LotParams lp;
    lp.seed = 7;
    std::vector<RenderMesh> fullParts, flatParts;
    std::vector<LotBuilding> lots =
        growLotBuildings({block}, lp, nullptr, &fullParts, nullptr, 0.0,
                         &flatParts);
    CHECK(!lots.empty());
    CHECK(tris(flatParts) > 0);
    CHECK(tris(flatParts) * 3 < tris(fullParts));
    // Same PartId indexing discipline as the full set, so the loader binds the
    // same materials and chunks both tiers identically.
    // (Slots past kDrapedPartBase are ground-relative dressing; they bind their
    // base part's material.)
    CHECK(flatParts.size() == fullParts.size());
    for (std::size_t i = 0; i < flatParts.size(); ++i)
        CHECK(flatParts[i].materialIndex == static_cast<int>(baseSlot(i)));
}

// THE LIT WINDOWS VARY WITHIN A STOREY (device: "the way it's lit up row by
// row is odd"). Curtain walls used to light a whole storey-face from one
// hash at x = 0, so every lit floor was a full-width band. Each mullion bay
// now decides for itself against a per-storey occupancy: most storey-faces
// are MIXED (some panes lit, some dark), none are uniform bands by rule, and
// the lit share stays a minority so the tower still reads as night.
TEST_CASE(curtain_wall_lights_vary_within_a_storey) {
    const Poly2 plan{{0, 0}, {24, 0}, {24, 20}, {0, 20}};
    BuildingParams p = midriseParams();
    p.floors = 14;
    p.curtainWall = true;
    const BuildingMesh full = growPlanBuilding(plan, p);
    // Vision panes: upper-storey glass quads taller than a spandrel band
    // (emitQuad writes four consecutive vertices per pane).
    std::map<std::tuple<int, int, int>, std::pair<int, int>> faces;   // lit, dark
    auto scan = [&](const RenderMesh* m, bool lit) {
        if (!m) return;
        for (std::size_t i = 0; i + 3 < m->vertices.size(); i += 4) {
            Real y0 = 1e9, y1 = -1e9;
            for (int k = 0; k < 4; ++k) {
                y0 = std::min(y0, m->vertices[i + k].position.y);
                y1 = std::max(y1, m->vertices[i + k].position.y);
            }
            const Real h = y1 - y0;
            if (y0 < 7.0 || h < 1.2 || h > 3.6) continue;
            const Vec3& n = m->vertices[i].normal;
            auto key = std::make_tuple(static_cast<int>(std::lround(y0 * 10.0)),
                                       static_cast<int>(std::lround(n.x * 10.0)),
                                       static_cast<int>(std::lround(n.z * 10.0)));
            if (lit) ++faces[key].first; else ++faces[key].second;
        }
    };
    scan(part(full, PartId::GlassLit), true);
    scan(part(full, PartId::Glass), false);
    int total = 0, mixed = 0, allLit = 0, allDark = 0, lit = 0, panes = 0;
    for (const auto& kv : faces) {
        const int l = kv.second.first, d = kv.second.second;
        if (l + d < 4) continue;   // narrow faces cannot show variety
        ++total;
        lit += l;
        panes += l + d;
        if (l > 0 && d > 0) ++mixed;
        else if (l > 0) ++allLit;
        else ++allDark;
    }
    std::printf("    [tower] %d storey-faces: %d mixed, %d all lit, %d all dark; %d/%d panes lit\n",
                total, mixed, allLit, allDark, lit, panes);
    CHECK(total >= 20);
    CHECK(mixed * 10 >= total * 6);          // was 0 of them: bands only
    CHECK(allLit == 0);                       // no floor is a solid bar of light
    CHECK(lit * 10 > panes * 1);              // inhabited...
    CHECK(lit * 10 < panes * 6);              // ...but still night
}


// ---- ADR-0080: the open doorway ------------------------------------------

TEST_CASE(open_doorway_drops_the_leaf_and_grows_an_interior) {
    Poly2 plan = {{0, 0}, {20, 0}, {20, 15}, {0, 15}};
    BuildingParams p = midriseParams();
    const BuildingMesh closed = growPlanBuilding(plan, p);
    p.openDoorway = true;
    const BuildingMesh open = growPlanBuilding(plan, p);
    // The painted leaf is gone at Full detail...
    const RenderMesh* doorClosed = part(closed, PartId::Door);
    CHECK(doorClosed && !doorClosed->indices.empty());
    const RenderMesh* doorOpen = part(open, PartId::Door);
    CHECK(!doorOpen || doorOpen->indices.empty());
    // ...replaced by a real interior shell (inner walls + ceiling), which the
    // closed building does not have at all.
    const RenderMesh* interior = part(open, PartId::Interior);
    CHECK(interior && !interior->indices.empty());
    const RenderMesh* woodFloor = part(open, PartId::InteriorFloor);
    CHECK(woodFloor && !woodFloor->indices.empty());
    // The grain is AUTHORED (plank-direction UVs, ADR-0080): world-metre
    // coordinates, so at least one vertex sits past the first tile.
    bool authoredUV = false;
    for (const Vertex& v : woodFloor->vertices)
        if (std::fabs(v.u) > 1.0f || std::fabs(v.v) > 1.0f) authoredUV = true;
    CHECK(authoredUV);
    const RenderMesh* interiorClosed = part(closed, PartId::Interior);
    CHECK(!interiorClosed || interiorClosed->indices.empty());
    // The entrance attach carries the aperture for the collider notch.
    bool found = false;
    for (const AttachPoint& a : open.attaches)
        if (a.tag == "entrance") {
            found = true;
            CHECK(std::fabs(a.width - human::DOOR_WIDTH) < 0.8);
            CHECK(a.height > 2.0);
            CHECK(a.height < 3.0);
        }
    CHECK(found);
    // Flat LOD still paints its dark pane -- LOD1 shows a door, not a hole.
    const BuildingMesh flatOpen =
        growPlanBuilding(plan, p, 0.0, FacadeDetail::Flat);
    const RenderMesh* flatDoor = part(flatOpen, PartId::Door);
    CHECK(flatDoor && !flatDoor->indices.empty());
}

TEST_CASE(interior_layout_fits_the_plan_and_repeats) {
    Poly2 plan = {{0, 0}, {20, 0}, {20, 15}, {0, 15}};
    BuildingParams p = midriseParams();
    p.core = 1;   // the straight stair of ADR-0080; cores are test_core_plan's
    const InteriorLayout il = interiorLayout(plan, p, 0);
    CHECK(il.hasStair);
    CHECK(il.well.size() == 4);
    for (const Vec2& c : il.well) {
        CHECK(pointInPolygon(plan, c));
        CHECK(c.y > 0.5);   // never against the entrance edge (edge 0, y=0)
    }
    const InteriorLayout il2 = interiorLayout(plan, p, 0);
    CHECK(il2.hasStair == il.hasStair);
    CHECK(std::fabs(il2.stairFoot.x - il.stairFoot.x) < 1e-12);
    CHECK(std::fabs(il2.stairFoot.y - il.stairFoot.y) < 1e-12);
    // A sliver plan fits no stair and cuts no hole.
    Poly2 sliver = {{0, 0}, {5, 0}, {5, 3.0}, {0, 3.0}};
    CHECK(!interiorLayout(sliver, p, 0).hasStair);
}


// A/B INSTRUMENT for refactors that must not move a vertex (the storeyPlans
// extraction, ADR-0080 Phase 2): prints a positional hash + tri count for a
// spread of param variants. Always passes — run it before and after the
// refactor and DIFF the two prints; a moved vertex changes the hash.
// THE CENSUS OVER EVERY RECIPE (the architectural toolkit's byte-identity proof, issue #105 / ADR-0174): one line
// per recipe, seed and plan with a positional hash of the exterior parts and one of the interior's. The first act
// of every refactor step is to run this into a file; the last is to diff. Only the lines a step means to change may
// change.
// NO FACADE CRACKS, EVERY RECIPE (Glenn, 2026-10-16: "some of the twisting buildings have cracks along the edges"):
// engine::auditFacadeCracks over each recipe's exterior on two lots, two seeds -- a level ray from outside that meets
// the BACK of a face first came in through a gap. Above the ground storey (its doors are open by design).
// RT_CRACKS_ONLY=<recipe> narrows it; RT_CRACKS_SHOW=<n> prints n samples a building.
TEST_CASE(building_facades_have_no_cracks_every_recipe) {
    // an instrument for now (~8 min over every recipe): RT_CRACKS=1 runs it
    if (!std::getenv("RT_CRACKS") && !std::getenv("RT_CRACKS_ONLY")) { std::printf("    [cracks] skipped (RT_CRACKS=1 runs it)\n"); return; }
    const Poly2 rect24 = {{-12, -10}, {12, -10}, {12, 10}, {-12, 10}};
    const Poly2 rect40 = {{-20, -20}, {20, -20}, {20, 20}, {-20, 20}};
    const char* only = std::getenv("RT_CRACKS_ONLY");
    const int show = std::getenv("RT_CRACKS_SHOW") ? std::atoi(std::getenv("RT_CRACKS_SHOW")) : 2;
    size_t leaking = 0, total = 0;
    for (int i = 0; i < architectRecipeCount(); ++i) {
        const char* name = architectRecipeName(i);
        if (only && std::string(only) != name) continue;
        for (uint32_t seed : {3u, 17u}) {
            BuildingRecipe r;
            if (!architectRecipeByName(name, seed, 0.6, 24, 600, r)) continue;
            if (r.massing == BuildingRecipe::Massing::Park || r.massing == BuildingRecipe::Massing::Plaza) continue;
            const char* planNames[2] = {"rect24", "rect40"};
            const Poly2* plans[2] = {&rect24, &rect40};
            for (int pi = 0; pi < 2; ++pi) {
                const BuildingMesh bm = growPlanBuilding(*plans[pi], r.params, 1.5);
                std::vector<std::string> names; names.reserve(bm.parts.size());
                for (const RenderMesh& part : bm.parts) names.push_back("part " + std::to_string(part.materialIndex));
                std::vector<NamedMeshRef> refs;
                double top = 1.5;
                for (size_t k = 0; k < bm.parts.size(); ++k) {
                    refs.push_back({names[k], &bm.parts[k]});
                    for (const Vertex& v : bm.parts[k].vertices) top = std::max(top, static_cast<double>(v.position.y));
                }
                const FacadeCrackAudit a = auditFacadeCracks(refs, 0.0, 0.0, 60.0, 1.5 + 6.0, top - 0.3);
                ++total;
                if (a.cracks == 0) continue;
                ++leaking;
                std::map<std::string, size_t> byPart; for (const auto& c : a.all) ++byPart[c.mesh];
                std::string parts; for (const auto& [n, c] : byPart) parts += " " + n + ":" + std::to_string(c);
                std::printf("    [cracks] %-22s seed %2u %-6s %zu of %zu rays leak (%zu storeys; %zu over the roof) via%s\n", name, seed, planNames[pi], a.cracks, a.hits, a.byStorey.size(), a.overRoof, parts.c_str());
                for (int k = 0; k < show && k < static_cast<int>(a.all.size()); ++k) {
                    const auto& c = a.all[static_cast<size_t>(k) * (a.all.size() / static_cast<size_t>(std::max(show, 1)))];
                    std::printf("        in at %.2f %.2f %.2f bearing %.0f deg, sees the back of %s %.1f m on\n", c.from.x + c.dir.x * 50, c.from.y, c.from.z + c.dir.z * 50,
                                std::atan2(-c.dir.z, -c.dir.x) * 57.29578, c.mesh.c_str(), c.t - 50.0);
                    if (std::getenv("RT_CRACKS_TRI"))
                        std::printf("          triangle (%.2f %.2f %.2f) (%.2f %.2f %.2f) (%.2f %.2f %.2f)\n", c.tri[0].x, c.tri[0].y, c.tri[0].z,
                                    c.tri[1].x, c.tri[1].y, c.tri[1].z, c.tri[2].x, c.tri[2].y, c.tri[2].z);
                }
            }
        }
    }
    std::printf("    [cracks] %zu of %zu buildings leak\n", leaking, total);
}

TEST_CASE(building_mesh_hash_census_every_recipe) {
    auto hashOf = [](const BuildingMesh& bm, std::size_t& nv, std::size_t& nt) {
        uint64_t h = 1469598103934665603ull;
        nv = nt = 0;
        for (const RenderMesh& part : bm.parts) {
            nt += part.indices.size() / 3;
            nv += part.vertices.size();
            for (const Vertex& vx : part.vertices) {
                const double q[3] = {vx.position.x, vx.position.y, vx.position.z};
                for (double d : q) {
                    const int64_t k = static_cast<int64_t>(std::llround(d * 4096.0));
                    h ^= static_cast<uint64_t>(k);
                    h *= 1099511628211ull;
                }
            }
        }
        return h;
    };
    const Poly2 rect24 = {{-12, -10}, {12, -10}, {12, 10}, {-12, 10}};
    const Poly2 lPlan = {{-14, -12}, {14, -12}, {14, 0}, {0, 0}, {0, 12}, {-14, 12}};
    const Poly2 rect40 = {{-20, -20}, {20, -20}, {20, 20}, {-20, 20}};
    int lines = 0;
    const char* only = std::getenv("RT_CENSUS_ONLY");   // one recipe (a hunt under valgrind)
    for (int i = 0; i < architectRecipeCount(); ++i) {
        const char* name = architectRecipeName(i);
        if (only && std::string(only) != name) continue;
        for (uint32_t seed : {3u, 17u}) {
            BuildingRecipe r;
            if (!architectRecipeByName(name, seed, 0.6, 24, 600, r)) continue;
            if (r.massing == BuildingRecipe::Massing::Park || r.massing == BuildingRecipe::Massing::Plaza) continue;
            r.params.openDoorway = true;
            const char* planNames[3] = {"rect24", "L", "rect40"};
            const Poly2* plans[3] = {&rect24, &lPlan, &rect40};
            for (int pi = 0; pi < 3; ++pi) {
                std::size_t ev, et, iv, it;
                const uint64_t he = hashOf(growPlanBuilding(*plans[pi], r.params, 1.5), ev, et);
                const uint64_t hi = hashOf(growInterior(*plans[pi], r.params, 1.5), iv, it);
                std::printf("    [census] %-22s seed %2u %-6s ext=%016llx tris=%zu  int=%016llx tris=%zu\n", name, seed, planNames[pi],
                            static_cast<unsigned long long>(he), et, static_cast<unsigned long long>(hi), it);
                ++lines;
                // RT_CENSUS_PARTS=<recipe>: that recipe's exterior, part by part (hunting a run-to-run difference)
                if (const char* want = std::getenv("RT_CENSUS_PARTS"); want && std::string(want) == name) {
                    const BuildingMesh bm = growPlanBuilding(*plans[pi], r.params, 1.5);
                    for (const RenderMesh& part : bm.parts) {
                        BuildingMesh one;
                        one.parts.push_back(part);
                        std::size_t pv, pt;
                        const uint64_t ph = hashOf(one, pv, pt);
                        // ...and the same positions sorted: equal here but not above means the ORDER varies
                        std::vector<std::tuple<double, double, double>> ps;
                        for (const Vertex& vx : part.vertices) ps.emplace_back(vx.position.x, vx.position.y, vx.position.z);
                        std::sort(ps.begin(), ps.end());
                        uint64_t sh = 1469598103934665603ull;
                        for (const auto& [x, y, z] : ps)
                            for (double d : {x, y, z}) { sh ^= static_cast<uint64_t>(static_cast<int64_t>(std::llround(d * 4096.0))); sh *= 1099511628211ull; }
                        std::printf("    [census-part] %s seed %u %s part %d hash=%016llx sorted=%016llx verts=%zu tris=%zu\n", name, seed, planNames[pi],
                                    part.materialIndex, static_cast<unsigned long long>(ph), static_cast<unsigned long long>(sh), pv, pt);
                        // RT_CENSUS_DUMP=<file>: that part's triangles (part RT_CENSUS_DUMP_PART, default 20), sorted, one a line
                        if (const char* dumpTo = std::getenv("RT_CENSUS_DUMP"); dumpTo && part.materialIndex == (std::getenv("RT_CENSUS_DUMP_PART") ? std::atoi(std::getenv("RT_CENSUS_DUMP_PART")) : 20)) {
                            std::vector<std::string> tris;
                            for (std::size_t t = 0; t + 2 < part.indices.size(); t += 3) {
                                char line[256];
                                const Vec3& a = part.vertices[part.indices[t]].position;
                                const Vec3& b = part.vertices[part.indices[t + 1]].position;
                                const Vec3& c = part.vertices[part.indices[t + 2]].position;
                                std::snprintf(line, sizeof line, "%s %u %s (%.3f %.3f %.3f) (%.3f %.3f %.3f) (%.3f %.3f %.3f)", name, seed, planNames[pi],
                                              a.x, a.y, a.z, b.x, b.y, b.z, c.x, c.y, c.z);
                                tris.emplace_back(line);
                            }
                            std::sort(tris.begin(), tris.end());
                            if (FILE* f = std::fopen(dumpTo, "a")) { for (const std::string& l : tris) std::fprintf(f, "%s\n", l.c_str()); std::fclose(f); }
                        }
                    }
                }
            }
        }
    }
    CHECK(lines > 0);
}

TEST_CASE(building_mesh_hash_census) {
    Poly2 plan = {{0, 0}, {22, 0}, {26, 14}, {11, 20}, {0, 14}};
    for (int v = 0; v < 6; ++v) {
        BuildingParams p = midriseParams();
        p.seed = 100 + v * 17;
        p.floors = 3 + v * 3;
        if (v == 1) { p.setbackFloors = 3; p.setbackEvery = 1.2; }
        if (v == 2) p.curtainWall = true;
        if (v == 3) p.solidFacade = true;
        if (v == 4) p.openDoorway = true;
        if (v == 5) { p.setbackFloors = 4; p.setbackEvery = 2.0;
                      p.openDoorway = true; }
        const BuildingMesh bm = growPlanBuilding(plan, p, 1.5);
        uint64_t h = 1469598103934665603ull;
        std::size_t nv = 0, nt = 0;
        for (const RenderMesh& part : bm.parts) {
            nt += part.indices.size() / 3;
            nv += part.vertices.size();
            for (const Vertex& vx : part.vertices) {
                const double q[3] = {vx.position.x, vx.position.y,
                                     vx.position.z};
                for (double d : q) {
                    const int64_t k =
                        static_cast<int64_t>(std::llround(d * 4096.0));
                    h ^= static_cast<uint64_t>(k);
                    h *= 1099511628211ull;
                }
            }
        }
        std::printf("    [mesh-hash] v%d floors=%d hash=%016llx verts=%zu "
                    "tris=%zu height=%.3f\n",
                    v, p.floors, static_cast<unsigned long long>(h), nv, nt,
                    bm.height);
    }
    CHECK(true);
}
