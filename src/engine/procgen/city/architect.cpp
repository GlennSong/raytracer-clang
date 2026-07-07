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
void capFloors(BuildingParams& p, Real shortSide, Real slender = 1.8) {
    const int maxFloors =
        std::max(1, static_cast<int>(shortSide * slender / 3.2) - 1);
    p.floors = std::min(p.floors, maxFloors);
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
    const int lift = static_cast<int>(cx.coreness * 26);
    p.floors = rng.irange(cx.roomy ? 10 : 6, cx.roomy ? 16 : 9) + lift;
    p.groundRetail = true;
    dress(p, FacadeStyle::GlassCurtain, rng);
    if (p.floors > 8) { p.setbackFloors = rng.irange(4, 6);
                        p.setbackEvery = rng.range(1.2, 1.8); }
    if (p.floors > 16) p.setbackFloors = rng.irange(5, 7);
    cx.slender = 3.0 + cx.coreness * 4.5;
    // Roomy square-ish lots sometimes go ROUND (a drum tower).
    if (cx.roomy && rng.unit() < 0.35)
        out.massing = BuildingRecipe::Massing::Circle;
    out.placeType = "office";
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
    out.name = "hotel";
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
}  // namespace

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

BuildingRecipe architectPick(DistrictTag tag, Real shortSide, Real area,
                             uint32_t seed, Real coreness) {
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

    // The district ARCHETYPE TABLES: weighted picks over the named recipes.
    switch (tag) {
        case DistrictTag::Financial:
            if (roll < 0.42)      recipeGlassTower(out, rng, cx);
            else if (roll < 0.75) recipeOfficeSlab(out, rng, cx);
            else if (roll < 0.90) recipeCommercialBlock(out, rng, cx);
            else                  recipeCivicHall(out, rng, cx);
            break;
        case DistrictTag::Commercial:
            if (roll < 0.05)      recipePocketPark(out, rng, cx);
            else if (roll < 0.32) recipeBrickShop(out, rng, cx);
            else if (roll < 0.48) recipeMixedUse(out, rng, cx);
            else if (roll < 0.63) recipeOfficeMidrise(out, rng, cx);
            else if (roll < 0.72) recipeHotel(out, rng, cx);
            else if (roll < 0.83) recipeCivicMidtown(out, rng, cx);
            else                  recipeWalkupHomes(out, rng, cx);
            break;
        case DistrictTag::OldTown:
            recipeOldTownHouse(out, rng, cx);
            break;
        case DistrictTag::Industrial:
            if (roll < 0.8)       recipeMetalShed(out, rng, cx);
            else                  recipeIndustrialOffice(out, rng, cx);
            break;
        case DistrictTag::Residential:
            if (roll < 0.08)      recipePocketPark(out, rng, cx);
            else if (roll < 0.44) recipeYardHouse(out, rng, cx);
            else if (roll < 0.56) recipeDuplex(out, rng, cx);
            else if (roll < 0.72) recipeRowhouses(out, rng, cx);
            else if (roll < 0.90) recipeApartments(out, rng, cx);
            else                  recipeCornerShop(out, rng, cx);
            break;
    }
    capFloors(out.params, shortSide, cx.slender);
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
        default: break;
    }
    capFloors(out.params, shortSide, cx.slender);
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
