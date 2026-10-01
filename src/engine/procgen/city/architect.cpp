#include "architect.h"

#include <algorithm>
#include <cmath>

namespace engine {

namespace {
constexpr Real kTau = 6.283185307179586;

// The same cheap deterministic hash rng the lot pass uses (ADR-0002).
struct Hash {
    uint32_t s;
    explicit Hash(uint32_t seed) : s(seed ? seed : 0x9e3779b9u) {}
    uint32_t next() {
        s ^= s << 13; s ^= s >> 17; s ^= s << 5;
        return s;
    }
    Real unit() { return (next() & 0xffffff) / static_cast<Real>(0x1000000); }
    Real range(Real a, Real b) { return a + (b - a) * unit(); }
    int irange(int a, int b) { return a + static_cast<int>(unit() * (b - a + 1)); }
};

// Style helper: cladding + the window element + quoins, one coherent bundle.
void dress(BuildingParams& p, FacadeStyle style, Hash& rng) {
    p.wallColor = facadeColor(style, rng.next());
    switch (style) {
        case FacadeStyle::Brick:
            p.wallPart = PartId::Brick;
            // Several brick looks, not one (device: "same-y"): arched heads
            // with hood moulds, or plain flat heads with a band or bare.
            if (rng.unit() < 0.55) {
                p.window.head = OpeningStyle::Head::Segmental;
                p.window.hood = OpeningStyle::Hood::Arch;
            } else {
                p.window.head = OpeningStyle::Head::Flat;
                p.window.hood = rng.unit() < 0.6 ? OpeningStyle::Hood::Band
                                                 : OpeningStyle::Hood::None;
            }
            p.window.lightsX = rng.irange(1, 2);
            p.window.lightsY = rng.irange(1, 2);
            p.quoins = rng.unit() < 0.5;
            break;
        case FacadeStyle::Stucco:
            p.wallPart = PartId::Stucco;
            p.window.head = (rng.unit() < 0.45) ? OpeningStyle::Head::Round
                                                : OpeningStyle::Head::Flat;
            p.window.hood = (p.window.head == OpeningStyle::Head::Round)
                                ? OpeningStyle::Hood::Arch : OpeningStyle::Hood::Band;
            p.window.lightsX = rng.irange(1, 2);
            p.window.lightsY = 1;
            p.quoins = rng.unit() < 0.5;
            break;
        case FacadeStyle::Concrete:
            p.wallPart = PartId::Concrete;
            p.window.head = OpeningStyle::Head::Flat;
            p.window.hood = rng.unit() < 0.7 ? OpeningStyle::Hood::Band
                                             : OpeningStyle::Hood::None;
            p.window.lightsX = 1;
            p.window.lightsY = rng.irange(1, 2);
            break;
        case FacadeStyle::Metal:
            p.wallPart = PartId::Metal;
            break;
        case FacadeStyle::Wood:
            // Painted siding: flat-headed double-hung sashes, plain or with a
            // simple painted head casing.
            p.wallPart = PartId::Siding;
            p.window.head = OpeningStyle::Head::Flat;
            p.window.hood = rng.unit() < 0.5 ? OpeningStyle::Hood::Band
                                             : OpeningStyle::Hood::None;
            p.window.lightsX = rng.irange(1, 2);
            p.window.lightsY = 2;
            break;
        case FacadeStyle::DarkBrick:
            // Industrial masonry: big sash grids, bare heads.
            p.wallPart = PartId::Brick;
            p.window.head = rng.unit() < 0.4 ? OpeningStyle::Head::Segmental
                                             : OpeningStyle::Head::Flat;
            p.window.hood = OpeningStyle::Hood::None;
            p.window.lightsX = 2;
            p.window.lightsY = 2;
            break;
        case FacadeStyle::Sandstone:
            // Smooth warm ashlar (banks, museums, deco masonry): banded flat
            // heads, occasional quoined corners.
            p.wallPart = PartId::Stucco;
            p.window.head = OpeningStyle::Head::Flat;
            p.window.hood = OpeningStyle::Hood::Band;
            p.window.lightsX = 1;
            p.window.lightsY = rng.irange(1, 2);
            p.quoins = rng.unit() < 0.4;
            break;
        case FacadeStyle::GlassCurtain:
            p.curtainWall = true;
            break;
        default: break;   // Painted: flat wall, plain windows
    }
    bool plain = p.curtainWall || p.solidFacade;
    p.baseCourse = !p.solidFacade;
    p.stringCourse = !plain;
    p.awning = !plain;
    // Trim varies per building (warm stone / cool limestone / painted dark),
    // so identical recipes still don't read as the SAME building.
    const Real tj = rng.range(-0.05, 0.04);
    p.trimColor = plain ? Vec3(0.50, 0.52, 0.55)
                : (style == FacadeStyle::Brick ? Vec3(0.84, 0.82, 0.76)
                                               : Vec3(0.78, 0.77, 0.73));
    if (!plain) {
        p.trimColor = p.trimColor + Vec3(tj, tj, tj);
        if (rng.unit() < 0.15) p.trimColor = Vec3(0.34, 0.33, 0.32);  // dark trim
    }
}

// Slenderness cap shared by every table (device: no pencil towers). Real
// towers ARE slender — the tower recipes ask for a higher ratio so the
// skyscraper cluster can actually rise; everything else keeps the squat cap.
// STOREY HEIGHTS BY USE (buildings M1; Glenn, 2026-09-30: "our buildings aren't to scale with real buildings ...
// different heights for different use cases is a good idea"). Every storey used to be 3.2 m, a flat's: an office
// floor is 3.9-4.1 m floor to floor, a supertall condo's 4.4-4.8, a tower lobby 6-8. Applied after the recipe, ONLY
// where it left the defaults (a warehouse's 3.6, a capitol's 4.2 stand), on its own dice so no other pick moves.
// Houses, shops and walk-ups keep theirs.
void storeyHeightsByUse(BuildingRecipe& out) {
    BuildingParams& p = out.params;
    const bool fhDefault = std::fabs(p.floorHeight - human::FLOOR_HEIGHT) < 1e-9;
    const bool ghDefault = std::fabs(p.groundHeight - human::GROUND_HEIGHT) < 1e-9;
    Hash h(p.seed ^ 0x51ae7b3du);
    const std::string& n = out.name;
    const bool office = out.placeType == "office";
    const bool civic = out.placeType == "civic";
    const bool homeTower = out.placeType == "home" && p.floors >= 12;
    const bool tower = p.floors >= 15;
    if (fhDefault) {
        if (n == "pencil_tower") p.floorHeight = h.range(4.4, 4.8);
        else if (office) p.floorHeight = h.range(3.9, 4.1);
        else if (civic) p.floorHeight = h.range(3.8, 4.1);
        else if (homeTower) p.floorHeight = h.range(3.1, 3.3);
        else if (n == "hotel" || n == "commercial_block") p.floorHeight = h.range(3.4, 3.7);
    }
    if (ghDefault) {
        if (office && tower) p.groundHeight = h.range(6.0, 8.0);
        else if (office || civic) p.groundHeight = h.range(5.0, 5.6);
        else if (homeTower || n == "hotel") p.groundHeight = h.range(5.5, 6.5);
    }
}

// The SLENDERNESS cap in metres: the plate's short side times `slender` is as tall as the building may stand,
// whatever its storeys are.
void capFloors(BuildingParams& p, Real shortSide, Real slender = 1.8) {
    const Real fh = std::max(Real(2.4), p.floorHeight);
    const int maxFloors =
        std::max(1, static_cast<int>(shortSide * slender / fh) - 1);
    p.floors = std::min(p.floors, maxFloors);
}
void capFloors(BuildingRecipe& out, Real shortSide, Real slender) {
    // Homes, and the buildings that are flats over a shop (their upper floors are apartments). A hotel keeps its
    // ring of rooms -- that is a hotel.
    static const char* kFlatsOverShops[] = {"mixed_use", "oldtown_house", "oldtown_grand", "brick_shop", "corner_shop"};
    out.params.residential = out.placeType == "home";
    for (const char* n : kFlatsOverShops) if (out.name == n) out.params.residential = true;
    storeyHeightsByUse(out);
    capFloors(out.params, shortSide, slender);
    if (out.params.floors < 15) out.params.top = 0;   // a top is a tower's: the short ones keep the penthouse
}

// MASONRY DEPTH (buildings M3): window groups and full-height verticals on a punched-window building, rolled on its
// OWN dice. `vertChance` of the verticals (and a deeper reveal with them); the group size from {size, weight}.
void masonryDepth(BuildingParams& p, Real vertChance, std::initializer_list<std::pair<uint8_t, Real>> groups) {
    if (p.curtainWall) return;
    Hash h(p.seed ^ 0x2b7e1516u);
    p.verticals = h.unit() < vertChance;
    if (p.verticals) p.windowInset = std::max(p.windowInset, Real(0.22));
    Real total = 0;
    for (const auto& [g, w] : groups) total += w;
    Real r = h.unit() * total;
    for (const auto& [g, w] : groups) {
        if (r < w) { p.windowGroup = g; return; }
        r -= w;
    }
}

// THE TOP a tower stands under (buildings M2): a weighted menu {top, weight}, rolled on the building's OWN dice
// (its seed), so no other pick moves. 1 = penthouse; emitTowerTop has the rest.
void towerTop(BuildingParams& p, std::initializer_list<std::pair<uint8_t, Real>> menu) {
    Hash h(p.seed ^ 0x7c3e9a15u);
    Real total = 0;
    for (const auto& [t, w] : menu) total += w;
    Real r = h.unit() * total;
    for (const auto& [t, w] : menu) {
        if (r < w) { p.top = t; return; }
        r -= w;
    }
}

// ---- NAMED RECIPES -----------------------------------------------------------
// Each archetype is its OWN recipe (device: "other building recipes ...
// government buildings, skyscrapers, european style houses, suburban houses,
// schools, hospitals ... rather than one monolithic building recipe"). They
// share the element vocabulary — dress(), the window/door grammar, roofs,
// setback tiers — but each uses it its own way. The district tables below are
// weighted lists over these functions, so a new archetype (school, hospital,
// fire station) is one recipe + a table entry, and its place type is whatever
// the schedule sim understands.
void glassStyle(BuildingParams& p, Hash& rng, bool modern);   // the NYC-variety curtain styles (below)

struct RecipeCtx {
    Real shortSide = 0, area = 0, coreness = 0;
    bool roomy = false;
    Real slender = 1.8;   // the capFloors ratio this recipe wants
};

// The SKYSCRAPER: glass curtain, setback tiers, heights that climb hard with
// coreness — 30+ floors dead-centre, shoulders toward the ring. The lift does
// NOT need a roomy lot (downtown blocks are small pie slices where the
// arterials converge; real towers stand on small plates) — the coreness-scaled
// slender cap is what keeps them believable.
void recipeGlassTower(BuildingRecipe& out, Hash& rng, RecipeCtx& cx) {
    BuildingParams& p = out.params;
    // Density round ("taller than 40 stories"): dead-centre coreness lifts
    // the glass cluster to 55-62 floors; the rim keeps its 10-16.
    const int lift = static_cast<int>(cx.coreness * 46);
    p.floors = rng.irange(cx.roomy ? 10 : 6, cx.roomy ? 16 : 9) + lift;
    p.groundRetail = true;
    dress(p, FacadeStyle::GlassCurtain, rng);
    if (p.floors > 8) { p.setbackFloors = rng.irange(4, 6);
                        p.setbackEvery = rng.range(1.2, 1.8); }
    if (p.floors > 16) p.setbackFloors = rng.irange(5, 7);
    cx.slender = 3.0 + cx.coreness * 4.5;
    // Roomy square-ish lots sometimes go ROUND (a drum tower); otherwise, near
    // downtown, half of them stand back from the avenue in a PLAZA (the 1961
    // New York model — Seagram); the lot pass needs a deep enough site and
    // falls back to the full plan when it has not got one.
    if (cx.roomy && rng.unit() < 0.35)
        out.massing = BuildingRecipe::Massing::Circle;
    else if (cx.roomy && cx.coreness > 0.4 && rng.unit() < 0.5)
        out.massing = BuildingRecipe::Massing::TowerInPlaza;
    out.placeType = "office";
    if (p.curtainWall) glassStyle(p, rng, true);   // every glass tower its own glass (NYC variety M2)
    towerTop(p, {{2, 30}, {4, 12}, {3, 10}, {5, 10}, {6, 10}, {8, 6}, {7, 4}, {1, 18}});   // buildings M2
    out.name = "glass_tower";
}

// Concrete office slab: the downtown workhorse, modest coreness lift.
void recipeOfficeSlab(BuildingRecipe& out, Hash& rng, RecipeCtx& cx) {
    BuildingParams& p = out.params;
    p.floors = rng.irange(6, 12) +
               static_cast<int>(cx.coreness * (cx.roomy ? 8 : 3));
    p.groundRetail = true;
    dress(p, FacadeStyle::Concrete, rng);
    if (p.floors > 8) { p.setbackFloors = rng.irange(4, 6);
                        p.setbackEvery = rng.range(1.1, 1.6); }
    cx.slender = 2.4 + cx.coreness * 0.8;
    out.placeType = "office";
    towerTop(p, {{1, 70}, {2, 30}});   // buildings M2
    masonryDepth(p, 0.25, {{1, 60}, {2, 40}});   // buildings M3
    out.name = "office_slab";
}

// Downtown brick commercial block with setback tiers and ground retail.
void recipeCommercialBlock(BuildingRecipe& out, Hash& rng, RecipeCtx&) {
    BuildingParams& p = out.params;
    p.floors = rng.irange(5, 9);
    p.groundRetail = true;
    dress(p, FacadeStyle::Brick, rng);
    if (p.floors > 6) { p.setbackFloors = rng.irange(3, 4);
                        p.setbackEvery = rng.range(1.1, 1.5); }
    out.placeType = "shop";
    out.name = "commercial_block";
}

// GOVERNMENT / civic hall: low, pilastered, formal — never retail.
void recipeCivicHall(BuildingRecipe& out, Hash& rng, RecipeCtx&) {
    BuildingParams& p = out.params;
    p.floors = rng.irange(3, 5);
    p.groundRetail = false;
    p.pilasters = true;
    dress(p, FacadeStyle::Concrete, rng);
    out.placeType = "civic";
    out.name = "civic_hall";
}

// Midtown civic variant: smaller, sometimes stucco.
void recipeCivicMidtown(BuildingRecipe& out, Hash& rng, RecipeCtx&) {
    BuildingParams& p = out.params;
    p.floors = rng.irange(2, 4);
    p.groundRetail = false;
    p.pilasters = true;
    dress(p, rng.unit() < 0.5 ? FacadeStyle::Concrete : FacadeStyle::Stucco, rng);
    out.placeType = "civic";
    out.name = "civic_midtown";
}

// Main-street brick shop rows; low blocks sometimes carry a gable.
void recipeBrickShop(BuildingRecipe& out, Hash& rng, RecipeCtx&) {
    BuildingParams& p = out.params;
    p.floors = rng.irange(3, 6);
    p.groundRetail = true;
    dress(p, FacadeStyle::Brick, rng);
    if (p.floors <= 3 && rng.unit() < 0.3) {
        p.roofStyle = BuildingParams::RoofStyle::Gable;
        p.roofPitch = rng.range(0.35, 0.5);
    }
    out.placeType = "shop";
    out.name = "brick_shop";
}

// Midtown concrete office mid-rise.
void recipeOfficeMidrise(BuildingRecipe& out, Hash& rng, RecipeCtx&) {
    BuildingParams& p = out.params;
    p.floors = rng.irange(4, 8);
    p.groundRetail = true;
    dress(p, FacadeStyle::Concrete, rng);
    out.placeType = "office";
    masonryDepth(p, 0.2, {{1, 60}, {2, 40}});   // buildings M3
    out.name = "office_midrise";
}

// Stucco / painted walk-up homes over midtown streets.
void recipeWalkupHomes(BuildingRecipe& out, Hash& rng, RecipeCtx&) {
    BuildingParams& p = out.params;
    p.floors = rng.irange(3, 5);
    p.groundRetail = false;
    dress(p, rng.unit() < 0.3 ? FacadeStyle::Painted : FacadeStyle::Stucco, rng);
    out.placeType = "home";
    out.name = "walkup_homes";
}

// The EUROPEAN old-town house: dense, low, stucco/brick, round arches, quoins,
// hip roofs — one coherent look, half the ground floors are shops.
void recipeOldTownHouse(BuildingRecipe& out, Hash& rng, RecipeCtx&) {
    BuildingParams& p = out.params;
    p.floors = rng.irange(2, 3);
    p.groundRetail = (rng.unit() < 0.5);
    dress(p, rng.unit() < 0.7 ? FacadeStyle::Stucco : FacadeStyle::Brick, rng);
    p.window.head = OpeningStyle::Head::Round;
    p.window.hood = OpeningStyle::Hood::Arch;
    p.quoins = true;
    p.roofStyle = BuildingParams::RoofStyle::Hip;
    p.roofPitch = rng.range(0.5, 0.7);
    out.placeType = p.groundRetail ? "shop" : "home";
    out.name = "oldtown_house";
}

// Wide solid metal shed with a tall ground floor — the industrial workplace.
void recipeMetalShed(BuildingRecipe& out, Hash& rng, RecipeCtx&) {
    BuildingParams& p = out.params;
    p.floors = 1;
    p.groundHeight = rng.range(6.0, 9.0);
    p.solidFacade = true;
    p.groundRetail = false;
    dress(p, FacadeStyle::Metal, rng);
    p.groundBays = rng.irange(1, 3);   // loading docks on the street face
    out.placeType = "office";   // a workplace agents commute to
    out.name = "metal_shed";
}

// Plain concrete office block up front of the yards.
void recipeIndustrialOffice(BuildingRecipe& out, Hash& rng, RecipeCtx&) {
    BuildingParams& p = out.params;
    p.floors = rng.irange(2, 3);
    p.groundRetail = false;
    dress(p, FacadeStyle::Concrete, rng);
    out.placeType = "office";
    out.name = "industrial_office";
}

// The SUBURBAN house: a small centred rectangle so the lot keeps its yard,
// painted/stucco/brick, gable or hip roof.
void recipeYardHouse(BuildingRecipe& out, Hash& rng, RecipeCtx&) {
    BuildingParams& p = out.params;
    p.floors = rng.irange(1, 2);
    p.groundRetail = false;
    p.baseCourse = true;
    const Real r2 = rng.unit();
    dress(p, r2 < 0.4 ? FacadeStyle::Painted
            : r2 < 0.75 ? FacadeStyle::Stucco : FacadeStyle::Brick, rng);
    p.roofStyle = rng.unit() < 0.65 ? BuildingParams::RoofStyle::Gable
                                    : BuildingParams::RoofStyle::Hip;
    p.roofPitch = rng.range(0.45, 0.7);
    out.massing = BuildingRecipe::Massing::RectYard;
    out.placeType = "home";
    out.name = "yard_house";
}

// Low walk-up apartments on the outskirts; the low ones sometimes gabled.
void recipeApartments(BuildingRecipe& out, Hash& rng, RecipeCtx&) {
    BuildingParams& p = out.params;
    p.floors = rng.irange(2, 4);
    p.groundRetail = false;
    dress(p, rng.unit() < 0.55 ? FacadeStyle::Brick : FacadeStyle::Stucco, rng);
    if (p.floors <= 3 && rng.unit() < 0.4) {
        p.roofStyle = BuildingParams::RoofStyle::Gable;
        p.roofPitch = rng.range(0.4, 0.6);
    }
    out.placeType = "home";
    masonryDepth(p, 0.1, {{1, 70}, {2, 30}});   // buildings M3
    out.name = "apartments";
}

// The residential corner shop.
void recipeCornerShop(BuildingRecipe& out, Hash& rng, RecipeCtx&) {
    BuildingParams& p = out.params;
    p.floors = rng.irange(1, 2);
    p.groundRetail = true;
    dress(p, FacadeStyle::Brick, rng);
    out.placeType = "shop";
    out.name = "corner_shop";
}

// A pocket park: no building, the lot becomes a green with trees.
void recipePocketPark(BuildingRecipe& out, Hash&, RecipeCtx&) {
    out.massing = BuildingRecipe::Massing::Park;
    out.placeType = "park";
    out.name = "pocket_park";
}

// The urban PLAZA (device: "concrete plazas, walking paths, decorative
// fencing and staircases between elevations ... like building a building
// structure without the building"): no storeys — the lot becomes a raised
// paver podium with stairs at its street mouths, guard fencing over drops,
// a fountain, planters and benches. placeType "civic" ON PURPOSE: agents
// visit it, and the loader's pad-flatten pass (which skips park/green)
// grades the terrain under it exactly like a building's pad.
void recipePlaza(BuildingRecipe& out, Hash&, RecipeCtx&) {
    out.massing = BuildingRecipe::Massing::Plaza;
    out.placeType = "civic";
    out.name = "plaza";
}

// The HOTEL: masonry mid-rise, awninged entrance, tiers when tall — fabric,
// but distinct from offices by proportion and ornament.
void recipeHotel(BuildingRecipe& out, Hash& rng, RecipeCtx& cx) {
    BuildingParams& p = out.params;
    p.floors = rng.irange(5, 8);
    p.groundRetail = true;
    dress(p, rng.unit() < 0.5 ? FacadeStyle::Stucco : FacadeStyle::Brick, rng);
    if (p.floors > 6) { p.setbackFloors = rng.irange(3, 5);
                        p.setbackEvery = rng.range(1.0, 1.4); }
    cx.slender = 2.2;
    out.placeType = "shop";
    towerTop(p, {{5, 30}, {1, 70}});   // buildings M2
    masonryDepth(p, 0.3, {{1, 70}, {2, 30}});   // buildings M3
    out.name = "hotel";
}

// ---- The MODERN SKYLINE (new towers) ----------------------------------------

// The ART-DECO tower: dark masonry, tight setback tiers, and the stepped
// crown + mast SPIRE — the 1930s skyline silhouette.
// The 1916 New York envelope (ADR-0086 point 5): a street wall of 3-6 floors on
// the whole plan, one big setback above it, a step every 6-10 floors, and a
// shaft of 28-42 % of the plan from the second step up — the Empire State and
// Chrysler silhouette. A tower too short for the stack keeps uniform setbacks.
void streetWallEnvelope(BuildingParams& p, Hash& rng, const RecipeCtx& cx) {
    if (p.floors < 10) {
        p.setbackFloors = rng.irange(3, 5);
        p.setbackEvery = rng.range(1.2, 1.8);
        return;
    }
    p.envelope = BuildingParams::Envelope::StreetWallSetback;
    p.baseFloors = rng.irange(3, 6);
    p.setback1 = cx.roomy ? rng.range(5.0, 9.0) : rng.range(3.0, 5.0);
    p.stepFloors = rng.irange(6, 10);
    p.stepDepth = rng.range(1.5, 3.0);
    p.towerFrac = rng.range(0.28, 0.42);
    p.towerFloor = p.baseFloors + p.stepFloors * rng.irange(1, 2);
    if (p.towerFloor >= p.floors - 3) p.towerFloor = 0;   // no room for a shaft: steps only
    p.setbackFloors = 0;
    p.setbackEvery = 0;
}

void recipeArtDecoTower(BuildingRecipe& out, Hash& rng, RecipeCtx& cx) {
    BuildingParams& p = out.params;
    p.floors = rng.irange(9, 14) + static_cast<int>(cx.coreness * 20);
    p.groundRetail = true;
    dress(p, rng.unit() < 0.5 ? FacadeStyle::DarkBrick : FacadeStyle::Sandstone,
          rng);
    p.spire = true;
    p.window.lightsY = 2;
    cx.slender = 2.8 + cx.coreness * 1.5;
    streetWallEnvelope(p, rng, cx);
    out.placeType = "office";
    masonryDepth(p, 0.8, {{1, 20}, {2, 60}, {3, 20}});   // buildings M3
    out.name = "art_deco_tower";
}

// The STEPPED (wedding-cake) tower: light masonry, setbacks every few floors.
void recipeSteppedTower(BuildingRecipe& out, Hash& rng, RecipeCtx& cx) {
    BuildingParams& p = out.params;
    p.floors = rng.irange(10, 16) + static_cast<int>(cx.coreness * 14);
    p.groundRetail = true;
    dress(p, rng.unit() < 0.5 ? FacadeStyle::Sandstone : FacadeStyle::Brick, rng);
    cx.slender = 2.6 + cx.coreness * 1.2;
    streetWallEnvelope(p, rng, cx);
    out.placeType = "office";
    masonryDepth(p, 0.5, {{1, 50}, {2, 50}});   // buildings M3
    out.name = "stepped_tower";
}

// The DRUM tower: a round office tower — glass or concrete banding.
void recipeDrumTower(BuildingRecipe& out, Hash& rng, RecipeCtx& cx) {
    BuildingParams& p = out.params;
    p.floors = rng.irange(8, 14) + static_cast<int>(cx.coreness * 8);
    p.groundRetail = true;
    dress(p, rng.unit() < 0.6 ? FacadeStyle::GlassCurtain : FacadeStyle::Concrete,
          rng);
    cx.slender = 2.8 + cx.coreness * 1.2;
    out.massing = BuildingRecipe::Massing::Circle;
    out.placeType = "office";
    if (p.curtainWall) glassStyle(p, rng, true);   // every glass tower its own glass (NYC variety M2)
    towerTop(p, {{5, 40}, {2, 30}, {4, 30}});   // buildings M2
    out.name = "drum_tower";
}

// The PODIUM TOWER: a 3-5 floor full-lot podium (street wall, ground retail)
// carrying a slender curtain-wall tower — the modern downtown block. The lot
// pass grows the two masses (podium = the lot plan, tower = a centred shaft
// from podiumFloors up); this recipe just declares the split and the dress.
void recipePodiumTower(BuildingRecipe& out, Hash& rng, RecipeCtx& cx) {
    BuildingParams& p = out.params;
    const int lift = static_cast<int>(cx.coreness * 44);
    p.floors = rng.irange(cx.roomy ? 10 : 7, cx.roomy ? 16 : 10) + lift;
    p.groundRetail = true;
    dress(p, FacadeStyle::GlassCurtain, rng);
    out.podiumFloors = rng.irange(2, 4);   // 3-5 storeys with the ground floor
    cx.slender = 3.0 + cx.coreness * 4.5;
    out.massing = BuildingRecipe::Massing::PodiumTower;
    out.placeType = "office";
    if (p.curtainWall) glassStyle(p, rng, true);   // every glass tower its own glass (NYC variety M2)
    towerTop(p, {{2, 30}, {3, 15}, {4, 15}, {6, 10}, {7, 10}, {1, 20}});   // buildings M2
    out.name = "podium_tower";
}

// The PARKING GARAGE: open concrete decks over a bay-door entry.
void recipeParkingGarage(BuildingRecipe& out, Hash& rng, RecipeCtx& cx) {
    BuildingParams& p = out.params;
    p.floors = rng.irange(4, 6);
    p.groundRetail = false;
    dress(p, FacadeStyle::Concrete, rng);
    p.parkingDecks = true;
    p.groundBays = 2;              // the entry/exit ramps on the street face
    p.awning = false;
    cx.slender = 2.0;
    out.placeType = "office";
    out.name = "parking_garage";
}

// The PAGODA tower: the tiered East-Asian silhouette (box-scope massing so
// the pagoda shape can dispatch). A rare midtown flourish.
void recipePagodaTower(BuildingRecipe& out, Hash& rng, RecipeCtx&) {
    BuildingParams& p = out.params;
    p.shape = BuildingShape::Pagoda;
    p.tiers = 3 + 2 * rng.irange(0, 1);    // 3 or 5 — odd reads best
    p.floors = p.tiers;
    p.groundRetail = false;
    p.wallColor = Vec3(0.58, 0.22, 0.16);  // lacquer red
    p.trimColor = Vec3(0.30, 0.24, 0.18);
    out.massing = BuildingRecipe::Massing::BoxMass;
    out.placeType = "shop";
    out.name = "pagoda_tower";
}

// ---- NEW YORK HIGH-RISES (Glenn, 2026-09-30: "more variety of glass skyscrapers and the types of high rises
// you'd see in New York ... rules about building step back to avoid having the whole city in shadow") ---------

// A curtain style for a glass tower: tint, mullions, bay, ribbon or floor-to-ceiling, fins -- so two glass
// towers side by side are not the same building (M2 draws them; the numbers ride the regen key now).
void glassStyle(BuildingParams& p, Hash& rng, bool modern) {
    const Real r = rng.unit();
    p.glassTint = static_cast<uint8_t>(r < 0.17 ? 1 : r < 0.27 ? 2 : r < 0.39 ? 3 : r < 0.51 ? 4 : r < 0.66 ? 5
                                       : r < 0.76 ? 6 : r < 0.83 ? 7 : r < 0.93 ? 8 : 9);
    const Real m = rng.unit();
    p.mullionTone = static_cast<uint8_t>(p.glassTint == 3 ? (m < 0.7 ? 1 : 2)                    // bronze glass, bronze frames (Seagram)
                                         : m < 0.35 ? 0 : m < 0.6 ? 3 : m < 0.85 ? 2 : 4);
    p.curtainBay = modern ? rng.range(1.5, 3.0) : rng.range(1.2, 1.8);
    p.spandrelFrac = modern ? (rng.unit() < 0.6 ? 0.0 : rng.range(0.18, 0.3)) : rng.range(0.3, 0.45);
    p.fins = static_cast<uint8_t>(rng.unit() < (modern ? 0.35 : 0.2) ? rng.irange(1, 3) : 0);
}

// The SKY EXPOSURE tower: the midtown masonry classic cut by the plane -- a 60-85 ft street wall (taller on an
// avenue), a first setback, then steps up the plane (2.7:1 on a side street, 5.6:1 on an avenue) until the mass
// covers a quarter (1916) or up to 40% (1961) of the lot, and the tower goes straight up from there.
void recipeSkyExposureTower(BuildingRecipe& out, Hash& rng, RecipeCtx& cx) {
    BuildingParams& p = out.params;
    p.floors = rng.irange(10, 14) + static_cast<int>(cx.coreness * 44);   // the district rim keeps to its 10-16
    p.groundRetail = true;
    const Real r = rng.unit();
    dress(p, r < 0.40 ? FacadeStyle::Sandstone : r < 0.70 ? FacadeStyle::Brick : r < 0.85 ? FacadeStyle::DarkBrick : FacadeStyle::Concrete, rng);
    const bool avenue = rng.unit() < 0.45;
    p.envelope = BuildingParams::Envelope::SkyExposure;
    p.baseFloors = avenue ? rng.irange(7, 9) : rng.irange(5, 7);
    p.skyRatio = avenue ? 5.6 : 2.7;
    p.stepFloors = rng.irange(2, 3);
    p.setback1 = rng.range(3.0, 6.0);
    p.towerFrac = rng.unit() < 0.5 ? 0.25 : rng.range(0.30, 0.40);
    p.setbackFloors = 0;
    p.setbackEvery = 0;
    p.spire = p.floors > 30 && rng.unit() < 0.35;
    cx.slender = 3.2 + cx.coreness * 2.5;
    out.placeType = "office";
    towerTop(p, {{5, 25}, {4, 15}, {8, 10}, {7, 10}, {1, 40}});   // buildings M2
    masonryDepth(p, 0.7, {{1, 20}, {2, 50}, {3, 30}});   // buildings M3
    out.name = "sky_exposure_tower";
}

// The TOWER ON A BASE (the 1990s contextual rules): a base on the lot line up to the street wall, a 10-15 ft
// setback, and a glass tower over 30-40% of the lot.
void recipeTowerOnBase(BuildingRecipe& out, Hash& rng, RecipeCtx& cx) {
    BuildingParams& p = out.params;
    p.floors = rng.irange(10, 14) + static_cast<int>(cx.coreness * 40);
    p.groundRetail = true;
    dress(p, FacadeStyle::GlassCurtain, rng);
    glassStyle(p, rng, true);
    p.envelope = BuildingParams::Envelope::StreetWallSetback;
    p.baseFloors = rng.irange(5, 8);
    p.setback1 = rng.range(3.0, 4.5);
    p.stepFloors = 0;
    p.stepDepth = 0;
    p.towerFrac = rng.range(0.30, 0.40);
    p.towerFloor = p.baseFloors + 1;
    p.setbackFloors = 0;
    p.setbackEvery = 0;
    cx.slender = 3.4 + cx.coreness * 3.5;
    out.placeType = "office";
    towerTop(p, {{2, 35}, {5, 20}, {3, 10}, {6, 10}, {1, 25}});   // buildings M2
    out.name = "tower_on_base";
}

// The TAPERED glass tower (One World Trade Center): narrowing all the way up, its corners chamfered more and
// more toward the top -- a square base, an octagon at mid-height.
void recipeTaperedGlassTower(BuildingRecipe& out, Hash& rng, RecipeCtx& cx) {
    BuildingParams& p = out.params;
    p.floors = rng.irange(12, 16) + static_cast<int>(cx.coreness * 52);
    p.groundRetail = true;
    dress(p, FacadeStyle::GlassCurtain, rng);
    glassStyle(p, rng, true);
    p.envelope = BuildingParams::Envelope::Taper;
    p.baseFloors = rng.irange(1, 3);
    p.taperTop = rng.range(0.55, 0.80);
    p.chamferTop = rng.unit() < 0.5 ? rng.range(0.15, 0.30) : 0.0;
    p.setbackFloors = 0;
    p.setbackEvery = 0;
    cx.slender = 4.0 + cx.coreness * 4.0;
    out.placeType = "office";
    towerTop(p, {{8, 40}, {4, 20}, {2, 25}, {5, 15}});   // buildings M2
    out.name = "tapered_glass_tower";
}

// The GLASS SLAB (UN Secretariat, Lever House): a thin slab on the lot's long axis, sometimes over a low
// podium, ribbon windows in blue-green glass.
void recipeGlassSlab(BuildingRecipe& out, Hash& rng, RecipeCtx& cx) {
    BuildingParams& p = out.params;
    p.floors = rng.irange(10, 14) + static_cast<int>(cx.coreness * 24);
    p.groundRetail = rng.unit() < 0.6;
    dress(p, FacadeStyle::GlassCurtain, rng);
    glassStyle(p, rng, false);
    if (rng.unit() < 0.6) p.glassTint = static_cast<uint8_t>(rng.unit() < 0.5 ? 1 : 2);
    p.spandrelFrac = rng.range(0.35, 0.45);   // the slab's ribbon windows
    p.envelope = BuildingParams::Envelope::Slab;
    p.baseFloors = rng.unit() < 0.45 ? 2 : 1;   // Lever House stands on a two-storey podium
    p.towerFrac = rng.range(0.35, 0.55);
    p.setbackFloors = 0;
    p.setbackEvery = 0;
    cx.slender = 3.0 + cx.coreness * 2.0;
    out.placeType = "office";
    towerTop(p, {{1, 60}, {2, 30}, {6, 10}});   // buildings M2
    out.name = "glass_slab";
}

// The PENCIL tower (432 Park Avenue, 111 West 57th): a supertall on a small lot, very slender, the top
// feathered back one face at a time. A concrete grid or glass with bronze fins.
void recipePencilTower(BuildingRecipe& out, Hash& rng, RecipeCtx& cx) {
    BuildingParams& p = out.params;
    p.floors = rng.irange(12, 16) + static_cast<int>(cx.coreness * 66);   // a supertall only where the city is densest
    p.groundRetail = true;
    if (rng.unit() < 0.5) {
        dress(p, FacadeStyle::Concrete, rng);   // 432 Park's white concrete grid
        p.window.lightsX = 1;
        p.window.lightsY = 1;
    } else {
        dress(p, FacadeStyle::GlassCurtain, rng);
        glassStyle(p, rng, true);
        p.fins = static_cast<uint8_t>(rng.irange(1, 2));   // 111 W 57th's terracotta-and-bronze ribs
        p.mullionTone = 1;
    }
    p.envelope = BuildingParams::Envelope::Feathered;
    p.baseFloors = 1;
    p.featherFrom = rng.range(0.6, 0.8);
    p.stepDepth = rng.range(1.5, 2.5);
    p.setbackFloors = 0;
    p.setbackEvery = 0;
    cx.slender = 10.0 + cx.coreness * 4.0;
    out.placeType = "home";   // the supertalls on Billionaires' Row are residential
    towerTop(p, {{6, 35}, {2, 35}, {5, 30}});   // buildings M2
    out.name = "pencil_tower";
}

// ---- CONDOS + midtown living -------------------------------------------------

// The CONDO tower: a balconied residential mid-rise — every street-facing bay
// gets its slab + railing.
void recipeCondoTower(BuildingRecipe& out, Hash& rng, RecipeCtx& cx) {
    BuildingParams& p = out.params;
    p.floors = rng.irange(5, 8);
    p.groundRetail = rng.unit() < 0.5;
    const Real r = rng.unit();
    dress(p, r < 0.45 ? FacadeStyle::Concrete
            : r < 0.75 ? FacadeStyle::Painted : FacadeStyle::Stucco, rng);
    p.balconies = true;
    if (p.floors > 6) { p.setbackFloors = rng.irange(3, 5);
                        p.setbackEvery = rng.range(1.0, 1.4); }
    cx.slender = 2.2;
    out.placeType = "home";
    towerTop(p, {{1, 50}, {2, 30}, {6, 20}});   // buildings M2
    masonryDepth(p, 0.1, {{1, 70}, {2, 30}});   // buildings M3
    out.name = "condo_tower";
}

// The TERRACED condo: hard setbacks every couple of floors — stacked roof
// terraces with balcony rails on the way up.
void recipeTerraceCondo(BuildingRecipe& out, Hash& rng, RecipeCtx& cx) {
    BuildingParams& p = out.params;
    p.floors = rng.irange(4, 6);
    p.groundRetail = false;
    dress(p, rng.unit() < 0.5 ? FacadeStyle::Painted : FacadeStyle::Stucco, rng);
    p.balconies = true;
    p.setbackFloors = 2;
    p.setbackEvery = rng.range(1.6, 2.4);
    cx.slender = 2.0;
    out.placeType = "home";
    out.name = "terrace_condo";
}

// The LOFT conversion: dark industrial brick reborn as flats — big sash
// grids, bare heads, sometimes a shop below.
void recipeLoftConversion(BuildingRecipe& out, Hash& rng, RecipeCtx&) {
    BuildingParams& p = out.params;
    p.floors = rng.irange(3, 5);
    p.groundRetail = rng.unit() < 0.5;
    dress(p, FacadeStyle::DarkBrick, rng);
    p.floorHeight = 3.6;               // warehouse storeys
    out.placeType = "home";
    out.name = "loft_block";
}

// ---- Midtown commerce ----------------------------------------------------------

// The CINEMA: one tall marquee-fronted hall.
void recipeCinema(BuildingRecipe& out, Hash& rng, RecipeCtx&) {
    BuildingParams& p = out.params;
    p.floors = rng.irange(0, 1);
    p.groundHeight = rng.range(5.5, 6.5);
    p.groundRetail = true;
    dress(p, rng.unit() < 0.5 ? FacadeStyle::Painted : FacadeStyle::Stucco, rng);
    p.awning = true;                   // the marquee
    p.trimColor = Vec3(0.62, 0.18, 0.16);   // show-front red
    out.placeType = "shop";
    out.name = "cinema";
}

// The BANK branch: sandstone, pilasters, steps — small-scale civic gravity.
void recipeBank(BuildingRecipe& out, Hash& rng, RecipeCtx&) {
    BuildingParams& p = out.params;
    p.floors = rng.irange(2, 3);
    p.groundHeight = 5.0;
    p.groundRetail = false;
    p.pilasters = true;
    dress(p, FacadeStyle::Sandstone, rng);
    p.entranceSteps = true;
    p.quoins = true;
    out.placeType = "office";
    out.name = "bank";
}

// The STRIP MALL: one long awninged retail bar.
void recipeStripMall(BuildingRecipe& out, Hash& rng, RecipeCtx&) {
    BuildingParams& p = out.params;
    p.floors = 0;
    p.groundHeight = rng.range(4.0, 4.6);
    p.groundRetail = true;
    dress(p, rng.unit() < 0.5 ? FacadeStyle::Painted : FacadeStyle::Concrete,
          rng);
    p.awning = true;
    out.placeType = "shop";
    out.name = "strip_mall";
}

// The SUPERMARKET: a big solid box with a glass entrance and a loading side.
void recipeSupermarket(BuildingRecipe& out, Hash& rng, RecipeCtx&) {
    BuildingParams& p = out.params;
    p.floors = 0;
    p.groundHeight = rng.range(5.5, 7.0);
    p.groundRetail = false;
    p.solidFacade = true;
    dress(p, rng.unit() < 0.5 ? FacadeStyle::Metal : FacadeStyle::Concrete, rng);
    p.sideBays = 1;                    // the loading dock
    out.placeType = "shop";
    out.name = "supermarket";
}

// The OFFICE PARK pavilion: a low glass slab on landscaped grounds.
void recipeOfficePark(BuildingRecipe& out, Hash& rng, RecipeCtx&) {
    BuildingParams& p = out.params;
    p.floors = rng.irange(2, 3);
    p.groundRetail = false;
    dress(p, rng.unit() < 0.5 ? FacadeStyle::GlassCurtain : FacadeStyle::Concrete,
          rng);
    out.massing = BuildingRecipe::Massing::RectYard;   // the landscaped grounds
    out.yardHalfWMax = 11.0;
    out.yardHalfDMax = 8.0;
    out.placeType = "office";
    out.name = "office_park";
}

// ---- The SUBURBAN book (new houses) -------------------------------------------

// The MODERN house: flat roof, big glass, crisp white/grey render.
void recipeModernHouse(BuildingRecipe& out, Hash& rng, RecipeCtx&) {
    BuildingParams& p = out.params;
    p.floors = rng.irange(1, 2);
    p.groundRetail = false;
    dress(p, FacadeStyle::Painted, rng);
    p.wallColor = lerp(Vec3(0.88, 0.88, 0.86), Vec3(0.62, 0.63, 0.64),
                       rng.unit());
    p.window.head = OpeningStyle::Head::Flat;
    p.window.hood = OpeningStyle::Hood::None;
    p.window.lightsX = 2;
    p.window.lightsY = 1;
    p.windowInset = 0.2;
    p.parapet = 0.5;
    p.stringCourse = false;
    p.awning = false;
    out.massing = BuildingRecipe::Massing::RectYard;
    out.placeType = "home";
    out.name = "modern_house";
}

// The BUNGALOW: one storey of painted siding under a gable, with a porch.
void recipeBungalow(BuildingRecipe& out, Hash& rng, RecipeCtx&) {
    BuildingParams& p = out.params;
    p.floors = 1;
    p.groundRetail = false;
    dress(p, FacadeStyle::Wood, rng);
    p.porch = true;
    p.chimney = rng.unit() < 0.5;
    p.roofStyle = BuildingParams::RoofStyle::Gable;
    p.roofPitch = rng.range(0.35, 0.5);
    out.massing = BuildingRecipe::Massing::RectYard;
    out.placeType = "home";
    out.name = "bungalow";
}

// The CRAFTSMAN house: two storeys of siding, a full porch, a chimney.
void recipeCraftsman(BuildingRecipe& out, Hash& rng, RecipeCtx&) {
    BuildingParams& p = out.params;
    p.floors = 2;
    p.groundRetail = false;
    dress(p, FacadeStyle::Wood, rng);
    p.porch = true;
    p.chimney = true;
    p.roofStyle = BuildingParams::RoofStyle::Gable;
    p.roofPitch = rng.range(0.45, 0.6);
    out.massing = BuildingRecipe::Massing::RectYard;
    out.placeType = "home";
    out.name = "craftsman_house";
}

// The COTTAGE: small, steep-roofed, chimney smoke implied.
void recipeCottage(BuildingRecipe& out, Hash& rng, RecipeCtx&) {
    BuildingParams& p = out.params;
    p.floors = rng.irange(1, 2);
    p.groundRetail = false;
    dress(p, rng.unit() < 0.5 ? FacadeStyle::Painted : FacadeStyle::Stucco, rng);
    p.chimney = true;
    p.roofStyle = BuildingParams::RoofStyle::Gable;
    p.roofPitch = rng.range(0.65, 0.85);
    out.massing = BuildingRecipe::Massing::RectYard;
    out.yardHalfWMax = 5.5;
    out.yardHalfDMax = 5.0;
    out.placeType = "home";
    out.name = "cottage";
}

// The VILLA: a generous stucco house — hip roof, quoins, arched windows.
void recipeVilla(BuildingRecipe& out, Hash& rng, RecipeCtx&) {
    BuildingParams& p = out.params;
    p.floors = 2;
    p.groundRetail = false;
    dress(p, rng.unit() < 0.6 ? FacadeStyle::Stucco : FacadeStyle::Sandstone,
          rng);
    p.window.head = OpeningStyle::Head::Round;
    p.window.hood = OpeningStyle::Hood::Arch;
    p.quoins = true;
    p.entranceSteps = true;
    p.roofStyle = BuildingParams::RoofStyle::Hip;
    p.roofPitch = rng.range(0.4, 0.55);
    out.massing = BuildingRecipe::Massing::RectYard;
    out.yardHalfWMax = 9.0;
    out.yardHalfDMax = 7.0;
    out.placeType = "home";
    out.name = "villa";
}

// The RANCH house: one wide storey, low hip roof, an attached garage bay.
void recipeRanchHouse(BuildingRecipe& out, Hash& rng, RecipeCtx&) {
    BuildingParams& p = out.params;
    p.floors = 1;
    p.groundRetail = false;
    dress(p, rng.unit() < 0.6 ? FacadeStyle::Wood : FacadeStyle::Painted, rng);
    p.sideBays = 1;                    // the attached garage
    p.roofStyle = BuildingParams::RoofStyle::Hip;
    p.roofPitch = rng.range(0.3, 0.45);
    out.massing = BuildingRecipe::Massing::RectYard;
    out.yardHalfWMax = 11.0;
    out.yardHalfDMax = 6.5;
    out.placeType = "home";
    out.name = "ranch_house";
}

// The GARDEN condo: a low balconied walk-up on planted grounds.
void recipeGardenCondo(BuildingRecipe& out, Hash& rng, RecipeCtx&) {
    BuildingParams& p = out.params;
    p.floors = rng.irange(3, 4);
    p.groundRetail = false;
    dress(p, rng.unit() < 0.5 ? FacadeStyle::Stucco : FacadeStyle::Painted, rng);
    p.balconies = true;
    if (rng.unit() < 0.3) {
        p.roofStyle = BuildingParams::RoofStyle::Gable;
        p.roofPitch = rng.range(0.35, 0.5);
    }
    out.massing = BuildingRecipe::Massing::RectYard;
    out.yardHalfWMax = 12.0;
    out.yardHalfDMax = 9.0;
    out.placeType = "home";
    out.name = "garden_condo";
}

// ---- Old-town variety (same coherent look: low, hip, round arches) ------------

// The GRAND old-town house: three storeys, pilastered and quoined.
void recipeOldTownGrand(BuildingRecipe& out, Hash& rng, RecipeCtx&) {
    BuildingParams& p = out.params;
    p.floors = 3;
    p.groundRetail = rng.unit() < 0.4;
    p.pilasters = true;
    dress(p, FacadeStyle::Stucco, rng);
    p.window.head = OpeningStyle::Head::Round;
    p.window.hood = OpeningStyle::Hood::Arch;
    p.quoins = true;
    p.roofStyle = BuildingParams::RoofStyle::Hip;
    p.roofPitch = rng.range(0.5, 0.65);
    out.placeType = p.groundRetail ? "shop" : "home";
    out.name = "oldtown_grand";
}

// The old-town CAFE: low, awninged, arcaded shopfront.
void recipeOldTownCafe(BuildingRecipe& out, Hash& rng, RecipeCtx&) {
    BuildingParams& p = out.params;
    p.floors = rng.irange(1, 2);
    p.groundRetail = true;
    dress(p, rng.unit() < 0.6 ? FacadeStyle::Stucco : FacadeStyle::Painted, rng);
    p.window.head = OpeningStyle::Head::Round;
    p.window.hood = OpeningStyle::Hood::Arch;
    p.awning = true;
    p.roofStyle = BuildingParams::RoofStyle::Hip;
    p.roofPitch = rng.range(0.5, 0.7);
    out.placeType = "shop";
    out.name = "oldtown_cafe";
}

// ---- Industry ----------------------------------------------------------------

// The FACTORY: solid dark masonry (or metal) under a SAWTOOTH roof.
void recipeFactory(BuildingRecipe& out, Hash& rng, RecipeCtx&) {
    BuildingParams& p = out.params;
    p.floors = rng.irange(1, 2);
    p.floorHeight = 3.6;
    p.groundHeight = rng.range(4.5, 6.0);
    p.solidFacade = true;
    p.groundRetail = false;
    dress(p, rng.unit() < 0.5 ? FacadeStyle::DarkBrick : FacadeStyle::Metal, rng);
    p.groundBays = rng.irange(1, 2);
    p.roofStyle = BuildingParams::RoofStyle::Sawtooth;
    out.placeType = "office";
    out.name = "factory";
}

// The BRICK WAREHOUSE: solid dark brick, clerestory strip, loading bays.
void recipeBrickWarehouse(BuildingRecipe& out, Hash& rng, RecipeCtx& cx) {
    BuildingParams& p = out.params;
    p.floors = rng.irange(1, 2);
    p.floorHeight = 3.8;
    p.groundHeight = rng.range(4.5, 5.5);
    p.solidFacade = true;
    p.groundRetail = false;
    dress(p, FacadeStyle::DarkBrick, rng);
    p.groundBays = cx.shortSide > 18 ? 3 : 2;
    out.placeType = "office";
    out.name = "brick_warehouse";
}

// ---- LANDMARK recipes: the planner places these on the BEST lots -----------

// The SCHOOL: 2-3 floor masonry with the tall classroom window grid, a formal
// pilastered face, and a real SCHOOLYARD (RectYard massing, generous caps).
void recipeSchool(BuildingRecipe& out, Hash& rng, RecipeCtx&) {
    BuildingParams& p = out.params;
    p.floors = rng.irange(2, 3);
    p.groundRetail = false;
    p.pilasters = true;
    dress(p, rng.unit() < 0.6 ? FacadeStyle::Brick : FacadeStyle::Stucco, rng);
    p.window.head = OpeningStyle::Head::Flat;
    p.window.hood = OpeningStyle::Hood::Band;
    p.window.lightsX = 2;
    p.window.lightsY = 2;
    p.quoins = false;
    out.massing = BuildingRecipe::Massing::RectYard;   // the schoolyard
    out.yardHalfWMax = 12.0;
    out.yardHalfDMax = 9.0;
    out.placeType = "civic";
    out.name = "school";
}

// The HOSPITAL: a near-white concrete slab; the courtyard carve gives wings.
void recipeHospital(BuildingRecipe& out, Hash& rng, RecipeCtx&) {
    BuildingParams& p = out.params;
    p.floors = rng.irange(4, 6);
    p.groundRetail = false;
    dress(p, FacadeStyle::Concrete, rng);
    p.wallColor = Vec3(0.87, 0.88, 0.87);   // clinical near-white
    p.window.lightsY = 2;
    p.window.hood = OpeningStyle::Hood::Band;
    p.awning = true;                         // entrance canopy
    out.placeType = "civic";
    out.name = "hospital";
}

// The COURTHOUSE / city hall: the government face of the core — pilasters
// full height, a tall ground floor, quoins, a heavy parapet.
void recipeCourthouse(BuildingRecipe& out, Hash& rng, RecipeCtx&) {
    BuildingParams& p = out.params;
    p.floors = rng.irange(3, 4);
    p.groundHeight = 5.5;
    p.groundRetail = false;
    p.pilasters = true;
    dress(p, rng.unit() < 0.5 ? FacadeStyle::Concrete : FacadeStyle::Stucco, rng);
    p.window.head = OpeningStyle::Head::Round;
    p.window.hood = OpeningStyle::Hood::Arch;
    p.quoins = true;
    p.parapet = 1.25;
    p.trimColor = Vec3(0.88, 0.86, 0.80);
    p.portico = 4;
    p.entranceSteps = true;
    out.placeType = "civic";
    out.name = "courthouse";
}

// The POLICE STATION: squat concrete, dark trim — institutional.
void recipePolice(BuildingRecipe& out, Hash& rng, RecipeCtx&) {
    BuildingParams& p = out.params;
    p.floors = rng.irange(2, 3);
    p.groundRetail = false;
    dress(p, FacadeStyle::Concrete, rng);
    p.window.hood = OpeningStyle::Hood::Band;
    p.trimColor = Vec3(0.32, 0.33, 0.36);
    p.baseCourse = true;
    out.placeType = "civic";
    out.name = "police";
}

// The FIRE STATION: brick, VEHICLE BAY doors on the street face.
void recipeFire(BuildingRecipe& out, Hash& rng, RecipeCtx& cx) {
    BuildingParams& p = out.params;
    p.floors = rng.irange(1, 2);
    p.groundHeight = 5.0;                    // room for the engines
    p.groundRetail = false;
    dress(p, FacadeStyle::Brick, rng);
    p.window.head = OpeningStyle::Head::Flat;
    p.window.hood = OpeningStyle::Hood::Band;
    p.groundBays = cx.shortSide > 15 ? 3 : 2;
    out.placeType = "civic";
    out.name = "fire_station";
}

// The MARKET HALL: one tall arched masonry volume under a gable — the
// shopping anchor of an old town or a commercial quarter.
void recipeMarketHall(BuildingRecipe& out, Hash& rng, RecipeCtx&) {
    BuildingParams& p = out.params;
    p.floors = 0;                            // one tall hall, no upper storeys
    p.groundHeight = rng.range(6.0, 7.5);
    p.groundRetail = true;
    dress(p, FacadeStyle::Brick, rng);
    p.window.head = OpeningStyle::Head::Round;
    p.window.hood = OpeningStyle::Hood::Arch;
    p.quoins = true;
    p.roofStyle = BuildingParams::RoofStyle::Gable;
    p.roofPitch = rng.range(0.35, 0.5);
    out.placeType = "shop";
    out.name = "market_hall";
}

// The CAPITOL / town hall: the city's grandest civic front — a full portico
// (lathe colonnade + pediment) over broad steps, and a drum + colonnade +
// dome ROTUNDA crowning the roof. Placed dead centre by the planner.
void recipeCapitol(BuildingRecipe& out, Hash& rng, RecipeCtx&) {
    BuildingParams& p = out.params;
    p.floors = rng.irange(2, 3);
    p.groundHeight = 6.0;
    p.floorHeight = 4.2;
    p.groundRetail = false;
    p.pilasters = true;
    dress(p, FacadeStyle::Stucco, rng);
    p.wallColor = Vec3(0.88, 0.87, 0.82);   // civic limestone
    p.window.head = OpeningStyle::Head::Round;
    p.window.hood = OpeningStyle::Hood::Arch;
    p.quoins = true;
    p.parapet = 1.2;
    p.trimColor = Vec3(0.90, 0.88, 0.82);
    p.portico = 6;
    p.entranceSteps = true;
    p.dome = true;
    out.placeType = "civic";
    out.name = "capitol";
}

// The UNIVERSITY hall: collegiate brick, a smaller portico over steps, and a
// campus GREEN via generous RectYard caps.
void recipeUniversity(BuildingRecipe& out, Hash& rng, RecipeCtx&) {
    BuildingParams& p = out.params;
    p.floors = 3;
    p.groundRetail = false;
    p.pilasters = false;
    dress(p, FacadeStyle::Brick, rng);
    p.window.head = OpeningStyle::Head::Flat;
    p.window.hood = OpeningStyle::Hood::Band;
    p.window.lightsX = 2;
    p.window.lightsY = 2;
    p.quoins = true;
    p.portico = 4;
    p.entranceSteps = true;
    out.massing = BuildingRecipe::Massing::RectYard;   // the campus green
    out.yardHalfWMax = 13.0;
    out.yardHalfDMax = 10.0;
    out.placeType = "civic";
    out.name = "university";
}

// The CHURCH: one tall gabled nave, arched windows, and the BELL TOWER
// steeple rising through the roof — the neighbourhood's vertical accent.
void recipeChurch(BuildingRecipe& out, Hash& rng, RecipeCtx&) {
    BuildingParams& p = out.params;
    p.floors = 0;                            // one tall nave, no upper storeys
    p.groundHeight = rng.range(6.0, 7.5);
    p.groundRetail = false;
    dress(p, rng.unit() < 0.6 ? FacadeStyle::Stucco : FacadeStyle::Brick, rng);
    p.window.head = OpeningStyle::Head::Round;
    p.window.hood = OpeningStyle::Hood::Arch;
    p.quoins = true;
    p.roofStyle = BuildingParams::RoofStyle::Gable;
    p.roofPitch = rng.range(0.55, 0.75);
    p.steeple = true;
    p.entranceSteps = true;
    out.placeType = "civic";
    out.name = "church";
}

// The LIBRARY: a sandstone reading hall — pilasters, arched windows, steps.
void recipeLibrary(BuildingRecipe& out, Hash& rng, RecipeCtx&) {
    BuildingParams& p = out.params;
    p.floors = 2;
    p.groundHeight = 5.0;
    p.groundRetail = false;
    p.pilasters = true;
    dress(p, FacadeStyle::Sandstone, rng);
    p.window.head = OpeningStyle::Head::Round;
    p.window.hood = OpeningStyle::Hood::Arch;
    p.entranceSteps = true;
    if (rng.unit() < 0.5) p.portico = 4;
    out.placeType = "civic";
    out.name = "library";
}

// The MUSEUM: the gallery — a formal portico over broad steps, quoined
// sandstone, and a forecourt PLAZA via generous RectYard caps.
void recipeMuseum(BuildingRecipe& out, Hash& rng, RecipeCtx&) {
    BuildingParams& p = out.params;
    p.floors = rng.irange(2, 3);
    p.groundHeight = 5.5;
    p.groundRetail = false;
    p.pilasters = true;
    dress(p, rng.unit() < 0.6 ? FacadeStyle::Sandstone : FacadeStyle::Concrete,
          rng);
    p.quoins = true;
    p.parapet = 1.3;
    p.portico = 6;
    p.entranceSteps = true;
    out.massing = BuildingRecipe::Massing::RectYard;   // the forecourt plaza
    out.yardHalfWMax = 12.0;
    out.yardHalfDMax = 9.0;
    out.placeType = "civic";
    out.name = "museum";
}

// ROWHOUSES: the lot packs side-by-side townhome units (the lot pass splits
// the strip; architectRowUnit dresses each unit). Dense residential streets.
void recipeRowhouses(BuildingRecipe& out, Hash& rng, RecipeCtx&) {
    BuildingParams& p = out.params;
    p.floors = rng.irange(2, 3);
    p.groundRetail = false;
    dress(p, FacadeStyle::Brick, rng);   // fallback look if the strip can't split
    out.massing = BuildingRecipe::Massing::RowStrip;
    out.placeType = "home";
    out.name = "rowhouses";
}

// The DUPLEX: a wider two-family house on its yard — two awninged bays.
void recipeDuplex(BuildingRecipe& out, Hash& rng, RecipeCtx&) {
    BuildingParams& p = out.params;
    p.floors = 2;
    p.groundRetail = false;
    p.baseCourse = true;
    dress(p, rng.unit() < 0.5 ? FacadeStyle::Painted : FacadeStyle::Stucco, rng);
    p.roofStyle = rng.unit() < 0.5 ? BuildingParams::RoofStyle::Gable
                                   : BuildingParams::RoofStyle::Hip;
    p.roofPitch = rng.range(0.4, 0.6);
    out.massing = BuildingRecipe::Massing::RectYard;
    out.yardHalfWMax = 9.0;   // wider than a single-family house
    out.yardHalfDMax = 6.5;
    out.placeType = "home";
    out.name = "duplex";
}

// MIXED USE: shops below, homes above — the classic main-street block. The
// grammar already splits ground (Retail) from upper (Residential) storeys;
// this recipe leans into it with awnings + warm masonry.
void recipeMixedUse(BuildingRecipe& out, Hash& rng, RecipeCtx&) {
    BuildingParams& p = out.params;
    p.floors = rng.irange(3, 5);
    p.groundRetail = true;
    dress(p, rng.unit() < 0.6 ? FacadeStyle::Brick : FacadeStyle::Painted, rng);
    p.awning = true;
    p.stringCourse = true;
    out.placeType = "shop";
    out.name = "mixed_use";
}

// ---- THE RECIPE REGISTRY (archetype book, architect.h) ---------------------
// Every recipe body above, by the out.name string it stamps — the same
// vocabulary the style book keys on. The archetype book resolves Lua names
// against this ONCE at load; index order is arbitrary but stable in a build.
// A new recipe joins the book's vocabulary by taking a row here — nothing
// else. (The rolled ladders in architectPick and the landmark planner keep
// calling the functions directly; this table is the NAME surface.)
struct NamedRecipe {
    const char* name;
    void (*fn)(BuildingRecipe&, Hash&, RecipeCtx&);
};
const NamedRecipe kRecipeRegistry[] = {
    {"glass_tower", recipeGlassTower},
    {"office_slab", recipeOfficeSlab},
    {"commercial_block", recipeCommercialBlock},
    {"civic_hall", recipeCivicHall},
    {"civic_midtown", recipeCivicMidtown},
    {"brick_shop", recipeBrickShop},
    {"office_midrise", recipeOfficeMidrise},
    {"walkup_homes", recipeWalkupHomes},
    {"oldtown_house", recipeOldTownHouse},
    {"metal_shed", recipeMetalShed},
    {"industrial_office", recipeIndustrialOffice},
    {"yard_house", recipeYardHouse},
    {"apartments", recipeApartments},
    {"corner_shop", recipeCornerShop},
    {"pocket_park", recipePocketPark},
    {"plaza", recipePlaza},
    {"hotel", recipeHotel},
    {"art_deco_tower", recipeArtDecoTower},
    {"stepped_tower", recipeSteppedTower},
    {"drum_tower", recipeDrumTower},
    {"podium_tower", recipePodiumTower},
    {"parking_garage", recipeParkingGarage},
    {"pagoda_tower", recipePagodaTower},
    {"condo_tower", recipeCondoTower},
    {"sky_exposure_tower", recipeSkyExposureTower},
    {"tower_on_base", recipeTowerOnBase},
    {"tapered_glass_tower", recipeTaperedGlassTower},
    {"glass_slab", recipeGlassSlab},
    {"pencil_tower", recipePencilTower},
    {"terrace_condo", recipeTerraceCondo},
    {"loft_block", recipeLoftConversion},
    {"cinema", recipeCinema},
    {"bank", recipeBank},
    {"strip_mall", recipeStripMall},
    {"supermarket", recipeSupermarket},
    {"office_park", recipeOfficePark},
    {"modern_house", recipeModernHouse},
    {"bungalow", recipeBungalow},
    {"craftsman_house", recipeCraftsman},
    {"cottage", recipeCottage},
    {"villa", recipeVilla},
    {"ranch_house", recipeRanchHouse},
    {"garden_condo", recipeGardenCondo},
    {"oldtown_grand", recipeOldTownGrand},
    {"oldtown_cafe", recipeOldTownCafe},
    {"factory", recipeFactory},
    {"brick_warehouse", recipeBrickWarehouse},
    {"school", recipeSchool},
    {"hospital", recipeHospital},
    {"courthouse", recipeCourthouse},
    {"police", recipePolice},
    {"fire_station", recipeFire},
    {"market_hall", recipeMarketHall},
    {"capitol", recipeCapitol},
    {"university", recipeUniversity},
    {"church", recipeChurch},
    {"library", recipeLibrary},
    {"museum", recipeMuseum},
    {"rowhouses", recipeRowhouses},
    {"duplex", recipeDuplex},
    {"mixed_use", recipeMixedUse},
};
constexpr int kRecipeRegistryCount =
    static_cast<int>(sizeof(kRecipeRegistry) / sizeof(kRecipeRegistry[0]));
}  // namespace

int architectRecipeIndex(const std::string& name) {
    for (int i = 0; i < kRecipeRegistryCount; ++i)
        if (name == kRecipeRegistry[i].name) return i;
    return -1;
}
int architectRecipeCount() { return kRecipeRegistryCount; }
bool architectRecipeByName(const std::string& name, uint32_t seed, Real coreness, Real shortSide, Real area,
                           BuildingRecipe& out) {
    const int ri = architectRecipeIndex(name);
    if (ri < 0) return false;
    Hash rng(seed * 2654435761u ^ 0x9e3779b9u);
    out = BuildingRecipe{};
    out.params.seed = rng.next();
    RecipeCtx cx;
    cx.shortSide = shortSide;
    cx.area = area;
    cx.coreness = std::clamp(coreness, Real(0), Real(1));
    cx.roomy = area > 900;
    kRecipeRegistry[ri].fn(out, rng, cx);
    capFloors(out, shortSide, cx.slender);
    return true;
}
const char* architectRecipeName(int index) {
    return index >= 0 && index < kRecipeRegistryCount ? kRecipeRegistry[index].name
                                                      : nullptr;
}

const char* districtName(DistrictTag t) {
    switch (t) {
        case DistrictTag::Financial:   return "financial";
        case DistrictTag::Commercial:  return "commercial";
        case DistrictTag::Residential: return "residential";
        case DistrictTag::OldTown:     return "oldtown";
        case DistrictTag::Industrial:  return "industrial";
    }
    return "?";
}

DistrictTag DistrictMap::tagAt(const Vec2& p) const {
    // Polycentric geography (metropolis): zone by the NEAREST hub. The centre
    // hub (kind 0) keeps the radial reading below; a flavored hub holds its
    // flavor for hubRadius then grades to Residential, with a thin Commercial
    // collar around non-commercial cores so district seams read as main streets.
    if (!hubs.empty()) {
        std::size_t best = 0;
        Real bd = 1e30;
        for (std::size_t i = 0; i < hubs.size(); ++i) {
            const Real dd = (p - hubs[i].first).lengthSquared();
            if (dd < bd) { bd = dd; best = i; }
        }
        const int kind = hubs[best].second;
        if (kind > 0) {
            const Real r = std::sqrt(bd);
            if (r < hubRadius)
                return static_cast<DistrictTag>(std::min(kind, 4));
            if (r < hubRadius * 1.35 && kind != 2)
                return DistrictTag::Commercial;
            return DistrictTag::Residential;
        }
        // kind 0: fall through to the radial rings, measured from THIS hub.
        const Vec2 d0 = p - hubs[best].first;
        const Real r0 = d0.length();
        if (r0 < innerRadius) return DistrictTag::Financial;
        if (r0 < midRadius) return DistrictTag::Commercial;
        return DistrictTag::Residential;
    }
    const Vec2 d = p - center;
    const Real r = d.length();
    if (r < innerRadius) return DistrictTag::Financial;
    // Seeded QUARTER wedges: the old town leans against downtown in midtown;
    // industry takes a slice of the outskirts. Angles derive from the seed so
    // every city places its quarters differently but deterministically.
    Real ang = std::atan2(d.y, d.x);
    if (ang < 0) ang += kTau;
    Hash h(seed * 2654435761u + 17u);
    const Real oldTown0 = h.unit() * kTau, oldTownSpan = 0.9;   // ~52 deg pocket
    const Real ind0 = std::fmod(oldTown0 + kTau * 0.45, kTau);  // opposite-ish
    const Real indSpan = 1.15;                                  // ~66 deg wedge
    auto inWedge = [&](Real a0, Real span) {
        Real rel = ang - a0;
        if (rel < 0) rel += kTau;
        return rel < span;
    };
    if (r < midRadius)
        return inWedge(oldTown0, oldTownSpan) ? DistrictTag::OldTown
                                              : DistrictTag::Commercial;
    return inWedge(ind0, indSpan) ? DistrictTag::Industrial
                                  : DistrictTag::Residential;
}

// ONE BLOCK, ONE BUILDING (Glenn, 2026-09-21: "Maybe if only one lot can fit for whatever reason
// then it should take up the entire city block and build a plaza and a large architectural piece
// ... I would accept having 1 lot with a large massive building on it rather than a city block
// that has just a dinky lot"). The lot pass hands a block the parcel walk could not fill here as
// ONE lot, and this picks something that deserves it: downtown, a tower standing back in a plaza
// (Seagram), a podium tower whose base is the whole block, or a stepped mass on the block's own
// polygon; in a residential quarter a mansion block that fills it; an old town's grand hall; an
// industrial works. It stands taller than a lot-sized pick would (coreness floored), and the
// slenderness cap still holds it to its plate.
BuildingRecipe architectBlockLandmark(DistrictTag tag, Real shortSide, Real area, uint32_t seed,
                                      Real coreness) {
    Hash rng(seed * 2654435761u ^ 0x85ebca6bu);
    BuildingRecipe out;
    out.params.seed = rng.next();
    out.params.retailStreetOnly = true;
    RecipeCtx cx;
    cx.shortSide = shortSide;
    cx.area = area;
    cx.coreness = std::max(coreness, Real(0.4));
    cx.roomy = true;
    switch (tag) {
        case DistrictTag::Financial:
        case DistrictTag::Commercial: {
            const Real r = rng.unit();
            if (r < 0.30) {
                recipeGlassTower(out, rng, cx);
                out.massing = BuildingRecipe::Massing::TowerInPlaza;   // falls back to the block plan when too shallow
            } else if (r < 0.50) {
                recipePodiumTower(out, rng, cx);
            } else if (r < 0.65) {
                recipeSkyExposureTower(out, rng, cx);                  // the whole block cut by the plane
                out.massing = BuildingRecipe::Massing::LotPlan;
            } else if (r < 0.78) {
                recipeTaperedGlassTower(out, rng, cx);
            } else if (r < 0.88) {
                recipeGlassSlab(out, rng, cx);
            } else {
                recipeSteppedTower(out, rng, cx);
                out.massing = BuildingRecipe::Massing::LotPlan;       // the block's own polygon, stepped
            }
            break;
        }
        case DistrictTag::Residential:
            recipeApartments(out, rng, cx);
            out.params.floors = rng.irange(5, 9);
            out.massing = BuildingRecipe::Massing::LotPlan;
            cx.slender = std::max(cx.slender, Real(2.5));
            break;
        case DistrictTag::OldTown:
            recipeOldTownGrand(out, rng, cx);
            break;
        case DistrictTag::Industrial:
            recipeFactory(out, rng, cx);
            break;
    }
    capFloors(out, shortSide, cx.slender);
    return out;
}

BuildingRecipe architectPick(DistrictTag tag, Real shortSide, Real area,
                             uint32_t seed, Real coreness,
                             const ArchetypeBook* book) {
    // Decorrelate the caller's seed before drawing: xorshift's first outputs
    // stay correlated across sequential seeds, which would skew the tables'
    // branch weights for neighbouring lots.
    Hash rng(seed * 2654435761u ^ 0x9e3779b9u);
    BuildingRecipe out;
    out.params.seed = rng.next();
    out.params.retailStreetOnly = true;   // every city building has a FRONT
    RecipeCtx cx;
    cx.shortSide = shortSide;
    cx.area = area;
    cx.coreness = coreness;
    cx.roomy = shortSide > 16.0;
    const Real roll = rng.unit();

    // A Lua-authored ARCHETYPE BOOK replaces this district's weight ladder.
    // Same single roll, drawn above in the same stream position, so a book
    // carrying the ladder's own weights reproduces the city bit-for-bit;
    // the recipe BODY then draws from the same stream either way.
    if (book && !book->tables[static_cast<std::size_t>(tag)].empty()) {
        const auto& tbl = book->tables[static_cast<std::size_t>(tag)];
        int ri = tbl.back().second;        // roll at/above the last threshold
        for (const auto& [thr, idx] : tbl)
            if (roll < thr) { ri = idx; break; }
        kRecipeRegistry[ri].fn(out, rng, cx);
        capFloors(out, shortSide, cx.slender);
        return out;
    }

    // The district ARCHETYPE TABLES: weighted picks over the named recipes.
    switch (tag) {
        case DistrictTag::Financial:
            // THE SKYLINE MIX (NYC variety, 2026-09-30): the classic glass tower and podium keep a share, the
            // New York forms join -- the sky-exposure masonry cake, the tower on a base, the tapered and slab
            // glass towers, and a pencil tower where the city is densest (a small lot at the core).
            if (roll < 0.14)      recipeGlassTower(out, rng, cx);
            else if (roll < 0.22) recipePodiumTower(out, rng, cx);
            else if (roll < 0.29) recipeTowerOnBase(out, rng, cx);
            else if (roll < 0.35) recipeTaperedGlassTower(out, rng, cx);
            else if (roll < 0.40) recipeGlassSlab(out, rng, cx);
            else if (roll < 0.44) recipePencilTower(out, rng, cx);
            else if (roll < 0.52) recipeSkyExposureTower(out, rng, cx);
            else if (roll < 0.58) recipeArtDecoTower(out, rng, cx);
            else if (roll < 0.63) recipeSteppedTower(out, rng, cx);
            else if (roll < 0.67) recipeDrumTower(out, rng, cx);
            else if (roll < 0.79) recipeOfficeSlab(out, rng, cx);
            else if (roll < 0.89) recipeCommercialBlock(out, rng, cx);
            else if (roll < 0.935) recipeParkingGarage(out, rng, cx);
            else if (roll < 0.97) recipePlaza(out, rng, cx);
            else                  recipeCivicHall(out, rng, cx);
            break;
        case DistrictTag::Commercial:
            if (roll < 0.04)      recipePocketPark(out, rng, cx);
            else if (roll < 0.07) recipePlaza(out, rng, cx);
            else if (roll < 0.25) recipeBrickShop(out, rng, cx);
            else if (roll < 0.38) recipeMixedUse(out, rng, cx);
            else if (roll < 0.46) recipeOfficeMidrise(out, rng, cx);
            else if (roll < 0.49) recipePodiumTower(out, rng, cx);
            else if (roll < 0.55) recipeCondoTower(out, rng, cx);
            else if (roll < 0.61) recipeTerraceCondo(out, rng, cx);
            else if (roll < 0.67) recipeLoftConversion(out, rng, cx);
            else if (roll < 0.73) recipeHotel(out, rng, cx);
            else if (roll < 0.78) recipeBank(out, rng, cx);
            else if (roll < 0.82) recipeCinema(out, rng, cx);
            else if (roll < 0.86) recipeStripMall(out, rng, cx);
            else if (roll < 0.90) recipeSupermarket(out, rng, cx);
            else if (roll < 0.93) recipeOfficePark(out, rng, cx);
            else if (roll < 0.95) recipePagodaTower(out, rng, cx);
            else if (roll < 0.98) recipeCivicMidtown(out, rng, cx);
            else                  recipeWalkupHomes(out, rng, cx);
            break;
        case DistrictTag::OldTown:
            if (roll < 0.62)      recipeOldTownHouse(out, rng, cx);
            else if (roll < 0.82) recipeOldTownCafe(out, rng, cx);
            else                  recipeOldTownGrand(out, rng, cx);
            break;
        case DistrictTag::Industrial:
            if (roll < 0.45)      recipeMetalShed(out, rng, cx);
            else if (roll < 0.70) recipeFactory(out, rng, cx);
            else if (roll < 0.85) recipeBrickWarehouse(out, rng, cx);
            else                  recipeIndustrialOffice(out, rng, cx);
            break;
        case DistrictTag::Residential:
            if (roll < 0.07)      recipePocketPark(out, rng, cx);
            else if (roll < 0.25) recipeYardHouse(out, rng, cx);
            else if (roll < 0.31) recipeModernHouse(out, rng, cx);
            else if (roll < 0.38) recipeBungalow(out, rng, cx);
            else if (roll < 0.45) recipeCraftsman(out, rng, cx);
            else if (roll < 0.51) recipeCottage(out, rng, cx);
            else if (roll < 0.57) recipeVilla(out, rng, cx);
            else if (roll < 0.64) recipeRanchHouse(out, rng, cx);
            else if (roll < 0.72) recipeDuplex(out, rng, cx);
            else if (roll < 0.84) recipeRowhouses(out, rng, cx);
            else if (roll < 0.89) recipeGardenCondo(out, rng, cx);
            else if (roll < 0.96) recipeApartments(out, rng, cx);
            else                  recipeCornerShop(out, rng, cx);
            break;
    }
    capFloors(out, shortSide, cx.slender);
    return out;
}

const char* landmarkName(LandmarkKind k) {
    switch (k) {
        case LandmarkKind::School:     return "school";
        case LandmarkKind::Hospital:   return "hospital";
        case LandmarkKind::Courthouse: return "courthouse";
        case LandmarkKind::Police:     return "police";
        case LandmarkKind::Fire:       return "fire_station";
        case LandmarkKind::Market:     return "market_hall";
        case LandmarkKind::Capitol:    return "capitol";
        case LandmarkKind::University: return "university";
        case LandmarkKind::Church:     return "church";
        case LandmarkKind::Library:    return "library";
        case LandmarkKind::Museum:     return "museum";
        default:                       return "?";
    }
}

BuildingRecipe architectLandmark(LandmarkKind kind, Real shortSide, Real area,
                                 uint32_t seed) {
    Hash rng(seed * 2654435761u ^ 0x51ED2A9Bu);
    BuildingRecipe out;
    out.params.seed = rng.next();
    out.params.retailStreetOnly = true;
    RecipeCtx cx;
    cx.shortSide = shortSide;
    cx.area = area;
    cx.roomy = shortSide > 16.0;
    switch (kind) {
        case LandmarkKind::School:     recipeSchool(out, rng, cx); break;
        case LandmarkKind::Hospital:   recipeHospital(out, rng, cx); break;
        case LandmarkKind::Courthouse: recipeCourthouse(out, rng, cx); break;
        case LandmarkKind::Police:     recipePolice(out, rng, cx); break;
        case LandmarkKind::Fire:       recipeFire(out, rng, cx); break;
        case LandmarkKind::Market:     recipeMarketHall(out, rng, cx); break;
        case LandmarkKind::Capitol:    recipeCapitol(out, rng, cx); break;
        case LandmarkKind::University: recipeUniversity(out, rng, cx); break;
        case LandmarkKind::Church:     recipeChurch(out, rng, cx); break;
        case LandmarkKind::Library:    recipeLibrary(out, rng, cx); break;
        case LandmarkKind::Museum:     recipeMuseum(out, rng, cx); break;
        default: break;
    }
    capFloors(out, shortSide, cx.slender);
    return out;
}

std::vector<int> planLandmarks(const std::vector<LandmarkCand>& cands,
                               const Vec2& cityCenter, Real innerRadius) {
    std::vector<int> out(cands.size(), -1);

    // The clusters present, ascending: the city (0) plans first, towns after —
    // a stable order so the plan is deterministic in the candidates' order.
    std::vector<int> clusters;
    for (const LandmarkCand& c : cands)
        if (std::find(clusters.begin(), clusters.end(), c.cluster) ==
            clusters.end())
            clusters.push_back(c.cluster);
    std::sort(clusters.begin(), clusters.end());

    struct Want {
        LandmarkKind kind;
        int count;
        Real minShort, minArea;
        bool wantCore;            // score by centrality too (the courthouse)
        DistrictTag tagA, tagB;   // eligible districts (B may repeat A)
        // TOWN guarantee: after both district-strict relax passes fail, take
        // the best lot in the cluster regardless of district — a small town
        // still gets its school even when its parcels all zoned commercial.
        bool anyDistrict = false;
    };

    for (const int cl : clusters) {
        int nRes = 0, nCom = 0, nFin = 0, nOld = 0, nInd = 0, total = 0;
        for (const LandmarkCand& c : cands) {
            if (c.cluster != cl) continue;
            ++total;
            switch (c.tag) {
                case DistrictTag::Residential: ++nRes; break;
                case DistrictTag::Commercial:  ++nCom; break;
                case DistrictTag::Financial:   ++nFin; break;
                case DistrictTag::OldTown:     ++nOld; break;
                case DistrictTag::Industrial:  ++nInd; break;
            }
        }
        std::vector<Want> wants;
        if (cl == 0) {
            // The CITY keeps the full civic table (quotas as before).
            wants = {
                {LandmarkKind::Capitol, (nFin + nCom) >= 6 ? 1 : 0, 13.0, 300.0,
                 true, DistrictTag::Financial, DistrictTag::Commercial},
                {LandmarkKind::University, total >= 60 ? 1 : 0, 15.0, 380.0,
                 false, DistrictTag::Residential, DistrictTag::Commercial},
                {LandmarkKind::Courthouse, nFin >= 2 ? 1 : 0, 12.0, 260.0, true,
                 DistrictTag::Financial, DistrictTag::Financial},
                {LandmarkKind::Hospital, nCom >= 6 ? 1 : 0, 15.0, 380.0, false,
                 DistrictTag::Commercial, DistrictTag::Commercial},
                {LandmarkKind::School, nRes >= 6 ? 1 + nRes / 50 : 0, 13.0,
                 320.0, false, DistrictTag::Residential,
                 DistrictTag::Residential},
                {LandmarkKind::Police, nCom >= 4 ? 1 : 0, 9.0, 140.0, false,
                 DistrictTag::Commercial, DistrictTag::Commercial},
                {LandmarkKind::Fire, (nCom + nInd) >= 8 ? 1 + total / 150 : 0,
                 11.0, 220.0, false, DistrictTag::Commercial,
                 DistrictTag::Industrial},
                {LandmarkKind::Market, nOld >= 3 ? 1 : (nCom >= 8 ? 1 : 0),
                 10.0, 180.0, false,
                 nOld >= 3 ? DistrictTag::OldTown : DistrictTag::Commercial,
                 nOld >= 3 ? DistrictTag::OldTown : DistrictTag::Commercial},
                {LandmarkKind::Church, nRes >= 8 ? 1 + nRes / 70 : 0, 10.0,
                 180.0, false, DistrictTag::Residential, DistrictTag::OldTown},
                {LandmarkKind::Library, nCom >= 5 ? 1 : 0, 12.0, 240.0, false,
                 DistrictTag::Commercial, DistrictTag::Residential},
                {LandmarkKind::Museum, nFin >= 3 ? 1 : 0, 14.0, 320.0, true,
                 DistrictTag::Financial, DistrictTag::Commercial},
            };
        } else {
            // A satellite TOWN: its guaranteed anchors — the school and the
            // church — plus a market hall when the settlement is an old town.
            // No capitol/courthouse/hospital: those stay the city's.
            wants = {
                {LandmarkKind::School, total >= 10 ? 1 : 0, 13.0, 320.0, false,
                 DistrictTag::Residential, DistrictTag::OldTown, true},
                {LandmarkKind::Church, total >= 8 ? 1 : 0, 10.0, 180.0, false,
                 DistrictTag::Residential, DistrictTag::OldTown, true},
                {LandmarkKind::Market, nOld >= 6 ? 1 : 0, 10.0, 180.0, false,
                 DistrictTag::OldTown, DistrictTag::Commercial},
            };
        }
        for (const Want& w : wants) {
            for (int k = 0; k < w.count; ++k) {
                int best = -1;
                Real bestScore = -1;
                // Preferred thresholds first; if no lot in the district can
                // carry them (small towns parcel small), relax once — the
                // quarter still gets its school, just a modest one. Town
                // guarantees add a final any-district pass.
                struct Pass { Real relax; bool any; };
                const Pass passes[] = {
                    {1.0, false}, {0.72, false}, {0.72, w.anyDistrict}};
                const int nPasses = w.anyDistrict ? 3 : 2;
                for (int pi = 0; pi < nPasses; ++pi) {
                    const Pass& ps = passes[pi];
                    for (std::size_t ci = 0; ci < cands.size(); ++ci) {
                        const LandmarkCand& c = cands[ci];
                        if (out[ci] >= 0 || c.cluster != cl) continue;
                        if (!ps.any && c.tag != w.tagA && c.tag != w.tagB)
                            continue;
                        if (c.shortSide < w.minShort * ps.relax ||
                            c.area < w.minArea * ps.relax)
                            continue;
                        Real score = c.area;
                        if (w.wantCore) {
                            const Real r = (c.pos - cityCenter).length();
                            score *= 0.4 + std::max(
                                Real(0),
                                1.0 - r / std::max(Real(1), innerRadius));
                        }
                        if (score > bestScore) {
                            bestScore = score;
                            best = static_cast<int>(ci);
                        }
                    }
                    if (best >= 0) break;
                }
                if (best < 0) break;   // no lot can carry it: skip the quota
                out[best] = static_cast<int>(w.kind);
            }
        }
    }
    return out;
}

BuildingParams architectRowUnit(uint32_t seed, int floors) {
    Hash rng(seed * 2654435761u ^ 0x0BADCAFEu);
    BuildingParams p;
    p.seed = rng.next();
    p.floors = floors;
    p.groundRetail = false;
    p.retailStreetOnly = true;
    p.bayWidth = 2.9;               // townhome rhythm: narrow bays
    const Real r = rng.unit();
    dress(p, r < 0.40 ? FacadeStyle::Brick
            : r < 0.70 ? FacadeStyle::Painted : FacadeStyle::Stucco, rng);
    p.quoins = false;               // shared party walls, no corner masonry
    p.awning = false;
    p.entranceSteps = true;         // the stoop
    if (rng.unit() < 0.4) {
        p.roofStyle = BuildingParams::RoofStyle::Gable;
        p.roofPitch = rng.range(0.35, 0.55);
    }
    return p;
}

}  // namespace engine
