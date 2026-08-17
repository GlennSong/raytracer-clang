// The building middle LOD (city-render-perf R2). The invariant that matters:
// Flat is the SAME building — same seed, same massing, same bay and opening
// layout, because both detail levels consume one facadeLayout — drawn with a
// fraction of the triangles. These tests hold the ratio and the sharing, so a
// future edit cannot quietly fork the two levels apart.
#include "test_framework.h"
#include "../src/engine/procgen/city/city_lots.h"
#include "../src/engine/procgen/city/shape_grammar.h"

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
    CHECK(flatParts.size() == fullParts.size());
    for (std::size_t i = 0; i < flatParts.size(); ++i)
        CHECK(flatParts[i].materialIndex == static_cast<int>(i));
}
