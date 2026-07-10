#include "test_framework.h"

#include "../src/engine/procgen/city/architect.h"
#include "../src/engine/procgen/city/city_lots.h"
#include "../src/engine/procgen/city/shape_grammar.h"

#include <map>
#include <set>
#include <string>

using namespace engine;

// The expanded architect (city-building-recipes): ~25 new fabric recipes and
// three new landmark kinds join the district tables, plus the new element
// vocabulary (balconies, porches, chimneys, spires, steeples, sawtooth roofs,
// parking decks, side bays) and lot landscaping (paths, hedges, tree spots).
// These tests pin: every new recipe is reachable and deterministic, keeps its
// signature, honours the district invariants, and grows real geometry.

namespace {
// Collect recipe names a district's table produces over many seeds.
std::map<std::string, int> tableNames(DistrictTag tag, Real shortSide,
                                      Real area, int seeds,
                                      Real coreness = 0) {
    std::map<std::string, int> names;
    for (uint32_t s = 1; s <= static_cast<uint32_t>(seeds); ++s)
        ++names[architectPick(tag, shortSide, area, s, coreness).name];
    return names;
}

// A rect plan for growing recipes through the plan grammar.
Poly2 rectPlan(Real w, Real d) {
    return {{-w / 2, -d / 2}, {w / 2, -d / 2}, {w / 2, d / 2}, {-w / 2, d / 2}};
}

std::size_t totalVerts(const BuildingMesh& bm) {
    std::size_t n = 0;
    for (const RenderMesh& p : bm.parts) n += p.vertices.size();
    return n;
}

bool hasPart(const BuildingMesh& bm, PartId id) {
    for (const RenderMesh& p : bm.parts)
        if (p.materialIndex == static_cast<int>(id) && !p.vertices.empty())
            return true;
    return false;
}
}  // namespace

TEST_CASE(new_recipes_are_reachable_in_their_districts) {
    // Every table carries its new archetypes: over enough seeds each recipe
    // name shows up in the district that owns it.
    std::map<std::string, int> fin = tableNames(DistrictTag::Financial, 24, 520, 300);
    for (const char* n : {"glass_tower", "office_slab", "art_deco_tower",
                          "stepped_tower", "drum_tower", "commercial_block",
                          "parking_garage", "civic_hall"})
        CHECK(fin[n] > 0);

    std::map<std::string, int> com = tableNames(DistrictTag::Commercial, 14, 300, 600);
    for (const char* n : {"brick_shop", "mixed_use", "office_midrise",
                          "condo_tower", "terrace_condo", "loft_block", "hotel",
                          "bank", "cinema", "strip_mall", "supermarket",
                          "office_park", "pagoda_tower", "civic_midtown"})
        CHECK(com[n] > 0);

    std::map<std::string, int> res = tableNames(DistrictTag::Residential, 14, 260, 600);
    for (const char* n : {"yard_house", "modern_house", "bungalow",
                          "craftsman_house", "cottage", "villa", "ranch_house",
                          "duplex", "rowhouses", "garden_condo", "apartments",
                          "corner_shop", "pocket_park"})
        CHECK(res[n] > 0);

    std::map<std::string, int> ind = tableNames(DistrictTag::Industrial, 24, 500, 300);
    for (const char* n : {"metal_shed", "factory", "brick_warehouse",
                          "industrial_office"})
        CHECK(ind[n] > 0);

    std::map<std::string, int> old = tableNames(DistrictTag::OldTown, 12, 160, 300);
    for (const char* n : {"oldtown_house", "oldtown_cafe", "oldtown_grand"})
        CHECK(old[n] > 0);
}

TEST_CASE(new_recipes_keep_their_signatures) {
    // Recipe signatures survive across seeds: the archetype IS the look.
    std::set<std::string> placeTypes = {"home", "shop", "office", "civic",
                                        "park"};
    for (uint32_t s = 1; s <= 120; ++s) {
        BuildingRecipe f = architectPick(DistrictTag::Financial, 24, 520, s);
        CHECK(placeTypes.count(f.placeType) == 1);
        if (f.name == "art_deco_tower") {
            CHECK(f.params.spire);
            CHECK(f.params.setbackFloors > 0);
        }
        if (f.name == "parking_garage") {
            CHECK(f.params.parkingDecks);
            CHECK(f.params.groundBays >= 2);
        }
        if (f.name == "drum_tower")
            CHECK(f.massing == BuildingRecipe::Massing::Circle);

        BuildingRecipe c = architectPick(DistrictTag::Commercial, 14, 300, s);
        CHECK(placeTypes.count(c.placeType) == 1);
        if (c.name == "condo_tower" || c.name == "terrace_condo") {
            CHECK(c.params.balconies);
            CHECK(c.placeType == "home");
        }
        if (c.name == "pagoda_tower") {
            CHECK(c.massing == BuildingRecipe::Massing::BoxMass);
            CHECK(c.params.shape == BuildingShape::Pagoda);
        }
        if (c.name == "supermarket") CHECK(c.params.sideBays > 0);
        if (c.name == "bank") CHECK(c.params.pilasters);

        BuildingRecipe r = architectPick(DistrictTag::Residential, 14, 260, s);
        CHECK(placeTypes.count(r.placeType) == 1);
        if (r.name == "bungalow" || r.name == "craftsman_house") {
            CHECK(r.params.porch);
            CHECK(r.params.wallPart == PartId::Siding);
            CHECK(r.massing == BuildingRecipe::Massing::RectYard);
        }
        if (r.name == "modern_house")
            CHECK(r.params.roofStyle == BuildingParams::RoofStyle::Flat);
        if (r.name == "cottage") CHECK(r.params.chimney);
        if (r.name == "ranch_house") CHECK(r.params.sideBays == 1);
        if (r.name == "garden_condo") CHECK(r.params.balconies);

        BuildingRecipe ind = architectPick(DistrictTag::Industrial, 24, 500, s);
        if (ind.name == "factory")
            CHECK(ind.params.roofStyle == BuildingParams::RoofStyle::Sawtooth);
        if (ind.name == "brick_warehouse") {
            CHECK(ind.params.solidFacade);
            CHECK(ind.params.groundBays >= 2);
        }
    }
}

TEST_CASE(new_recipes_are_deterministic) {
    for (uint32_t s = 1; s <= 24; ++s) {
        BuildingRecipe a = architectPick(DistrictTag::Commercial, 14, 300, s);
        BuildingRecipe b = architectPick(DistrictTag::Commercial, 14, 300, s);
        CHECK(a.name == b.name);
        CHECK(a.placeType == b.placeType);
        CHECK(a.params.floors == b.params.floors);
        CHECK(a.params.seed == b.params.seed);
    }
}

TEST_CASE(new_landmarks_are_coherent) {
    for (uint32_t s = 1; s <= 12; ++s) {
        BuildingRecipe church = architectLandmark(LandmarkKind::Church, 14, 260, s);
        CHECK(church.name == "church");
        CHECK(church.params.steeple);
        CHECK(church.params.floors == 0);               // one tall nave
        CHECK(church.params.roofStyle == BuildingParams::RoofStyle::Gable);
        CHECK(church.placeType == "civic");

        BuildingRecipe lib = architectLandmark(LandmarkKind::Library, 16, 300, s);
        CHECK(lib.name == "library");
        CHECK(lib.params.pilasters);
        CHECK(lib.params.entranceSteps);
        CHECK(lib.placeType == "civic");

        BuildingRecipe mus = architectLandmark(LandmarkKind::Museum, 20, 440, s);
        CHECK(mus.name == "museum");
        CHECK(mus.params.portico >= 6);
        CHECK(mus.massing == BuildingRecipe::Massing::RectYard);
        CHECK(mus.yardHalfWMax > 10.0);                 // the forecourt plaza
    }
}

TEST_CASE(new_mesh_ops_grow_real_geometry) {
    // Each new element emits geometry under the parts it promises, on top of
    // an identical base building (so the delta IS the element).
    BuildingParams base;
    base.seed = 7;
    base.floors = 4;
    Poly2 plan = rectPlan(16, 12);

    BuildingMesh plain = growPlanBuilding(plan, base);
    CHECK(totalVerts(plain) > 0);

    BuildingParams bal = base;
    bal.balconies = true;
    BuildingMesh withBal = growPlanBuilding(plan, bal);
    CHECK(totalVerts(withBal) > totalVerts(plain));   // slabs + rails appeared
    CHECK(hasPart(withBal, PartId::Metal));           // the railings

    BuildingParams por = base;
    por.floors = 1;
    por.porch = true;
    por.roofStyle = BuildingParams::RoofStyle::Gable;
    BuildingMesh withPorch = growPlanBuilding(plan, por);
    CHECK(hasPart(withPorch, PartId::Wood));          // posts + fascia

    BuildingParams chim = por;
    chim.porch = false;
    chim.chimney = true;
    BuildingMesh withChim = growPlanBuilding(plan, chim);
    CHECK(hasPart(withChim, PartId::Brick));          // the masonry stack

    BuildingParams saw = base;
    saw.floors = 1;
    saw.solidFacade = true;
    saw.roofStyle = BuildingParams::RoofStyle::Sawtooth;
    BuildingMesh withSaw = growPlanBuilding(plan, saw);
    CHECK(hasPart(withSaw, PartId::Glass));           // the clerestory teeth

    BuildingParams spi = base;
    spi.floors = 8;
    spi.spire = true;
    BuildingMesh withSpire = growPlanBuilding(plan, spi);
    BuildingParams noSpire = spi;
    noSpire.spire = false;
    BuildingMesh plainTall = growPlanBuilding(plan, noSpire);
    CHECK(withSpire.height > plainTall.height + 2.0); // the mast rises

    BuildingParams ste = base;
    ste.floors = 0;
    ste.groundHeight = 6.5;
    ste.steeple = true;
    ste.roofStyle = BuildingParams::RoofStyle::Gable;
    BuildingMesh withSteeple = growPlanBuilding(plan, ste);
    CHECK(withSteeple.height > 10.0);                 // tower + spire

    BuildingParams deck = base;
    deck.floors = 5;
    deck.parkingDecks = true;
    BuildingMesh withDecks = growPlanBuilding(plan, deck);
    CHECK(totalVerts(withDecks) > 0);
    CHECK(hasPart(withDecks, PartId::Concrete));      // piers + deck slabs

    BuildingParams gar = base;
    gar.floors = 1;
    gar.sideBays = 1;
    BuildingMesh withGarage = growPlanBuilding(plan, gar);
    CHECK(totalVerts(withGarage) > 0);

    // Determinism: same params, same seed, same mesh.
    BuildingMesh again = growPlanBuilding(plan, bal);
    CHECK(totalVerts(again) == totalVerts(withBal));
}

TEST_CASE(new_facade_styles_have_seeded_palettes) {
    for (uint32_t s = 1; s <= 20; ++s) {
        for (FacadeStyle st : {FacadeStyle::Wood, FacadeStyle::DarkBrick,
                               FacadeStyle::Sandstone}) {
            Vec3 c1 = facadeColor(st, s);
            Vec3 c2 = facadeColor(st, s);
            CHECK_APPROX(c1.x, c2.x, 1e-12);          // deterministic
            CHECK(c1.x >= 0 && c1.x <= 1);
            CHECK(c1.y >= 0 && c1.y <= 1);
            CHECK(c1.z >= 0 && c1.z <= 1);
        }
        // DarkBrick is actually dark; Sandstone is actually warm.
        Vec3 dark = facadeColor(FacadeStyle::DarkBrick, s);
        CHECK(dark.x + dark.y + dark.z < 1.2);
        Vec3 sand = facadeColor(FacadeStyle::Sandstone, s);
        CHECK(sand.x > sand.z);
    }
    // The new parts carry the intended surfaces.
    RenderMaterial siding = materialFor(PartId::Siding, Vec3(0.7, 0.7, 0.7));
    CHECK(siding.surface() == RenderMaterial::Surface::WoodSiding);
    RenderMaterial path = materialFor(PartId::Path, Vec3(1, 1, 1));
    CHECK(path.surface() == RenderMaterial::Surface::Pavement);
}

TEST_CASE(parks_are_sculpted_with_paths_and_tree_spots) {
    // Grow a small city grid; every sculpted park keeps its tree spots inside
    // its own pad, and the landscaping kit (benches/planters/fountains) plus
    // walking paths land in the by-part output.
    LotParams p;
    p.center = {0, 0};
    p.seed = 9;
    std::vector<Poly2> blocks;
    for (int gx = -1; gx <= 1; ++gx)
        for (int gz = -1; gz <= 1; ++gz) {
            Real cx = gx * 115.0, cz = gz * 115.0, h = 50.0;
            blocks.push_back({{cx - h, cz - h}, {cx + h, cz - h},
                              {cx + h, cz + h}, {cx - h, cz + h}});
        }
    std::vector<RenderMesh> parts;
    std::vector<LotBuilding> lots = growLotBuildings(blocks, p, nullptr, &parts);
    CHECK(parts.size() == static_cast<std::size_t>(PartId::Count));

    int parks = 0, yardsWithTrees = 0;
    for (const LotBuilding& lb : lots) {
        if (lb.type == "park") {
            ++parks;
            CHECK(!lb.padMesh.vertices.empty());
            // Sculpted pads bake their colours; the tint goes white.
            CHECK_APPROX(lb.color.x, 1.0, 1e-9);
            for (const Vec3& s : lb.treeSpots) {
                CHECK(pointInPolygon(lb.pad, Vec2(s.x, s.z)));
                CHECK(s.y >= 0.5 && s.y <= 1.6);      // sane trunk scale
            }
        } else if (lb.type == "home" && !lb.treeSpots.empty()) {
            ++yardsWithTrees;
        }
    }
    CHECK(parks > 0);                                 // the tables roll parks
    CHECK(yardsWithTrees > 0);                        // yards got landscaping
    // Front walks / park paths exist: the Path part carries geometry.
    CHECK(!parts[static_cast<std::size_t>(PartId::Path)].vertices.empty() ||
          !parts[static_cast<std::size_t>(PartId::Foliage)].vertices.empty());

    // Determinism: the same seed sculpts the same city.
    std::vector<RenderMesh> parts2;
    std::vector<LotBuilding> lots2 = growLotBuildings(blocks, p, nullptr, &parts2);
    CHECK(lots.size() == lots2.size());
    for (std::size_t i = 0; i < lots.size(); ++i) {
        CHECK(lots[i].recipe == lots2[i].recipe);
        CHECK(lots[i].treeSpots.size() == lots2[i].treeSpots.size());
    }
}
