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
// towers ARE slender — the financial table passes a higher ratio so the
// skyscraper cluster can actually rise; everything else keeps the squat cap.
void capFloors(BuildingParams& p, Real shortSide, Real slender = 1.8) {
    const int maxFloors =
        std::max(1, static_cast<int>(shortSide * slender / 3.2) - 1);
    p.floors = std::min(p.floors, maxFloors);
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
    BuildingParams& p = out.params;
    p.seed = rng.next();
    p.retailStreetOnly = true;      // every city building has a FRONT
    const bool roomy = shortSide > 16.0;
    const Real roll = rng.unit();
    Real slender = 1.8;   // capFloors ratio; tower branches raise it

    switch (tag) {
        case DistrictTag::Financial: {
            // Towers and slabs; the occasional civic hall; ground retail.
            if (roll < 0.42) {                       // glass curtain tower
                // Deep in the core (coreness → 1) this is the SKYSCRAPER
                // cluster: heights climb hard with how central the lot is —
                // 30+ floors downtown, shouldering off toward the ring. The
                // lift does NOT need a roomy lot: downtown blocks are small
                // pie slices where the arterials converge, and real towers
                // stand on small plates — the coreness-scaled slender cap is
                // what keeps them believable.
                const int lift = static_cast<int>(coreness * 22);
                p.floors = rng.irange(roomy ? 10 : 6, roomy ? 16 : 9) + lift;
                p.groundRetail = true;
                dress(p, FacadeStyle::GlassCurtain, rng);
                if (p.floors > 8) { p.setbackFloors = rng.irange(4, 6);
                                    p.setbackEvery = rng.range(1.2, 1.8); }
                if (p.floors > 16) p.setbackFloors = rng.irange(5, 7);
                slender = 3.0 + coreness * 4.0;
                // Roomy square-ish lots sometimes go ROUND (a drum tower).
                if (roomy && rng.unit() < 0.35)
                    out.massing = BuildingRecipe::Massing::Circle;
                out.placeType = "office";
            } else if (roll < 0.75) {                // concrete office slab
                p.floors = rng.irange(6, 12) +
                           static_cast<int>(coreness * (roomy ? 8 : 3));
                p.groundRetail = true;
                dress(p, FacadeStyle::Concrete, rng);
                if (p.floors > 8) { p.setbackFloors = rng.irange(4, 6);
                                    p.setbackEvery = rng.range(1.1, 1.6); }
                slender = 2.4 + coreness * 0.8;
                out.placeType = "office";
            } else if (roll < 0.90) {                // brick commercial block
                p.floors = rng.irange(5, 9);
                p.groundRetail = true;
                dress(p, FacadeStyle::Brick, rng);
                if (p.floors > 6) { p.setbackFloors = rng.irange(3, 4);
                                    p.setbackEvery = rng.range(1.1, 1.5); }
                out.placeType = "shop";
            } else {                                 // civic hall
                p.floors = rng.irange(3, 5);
                p.groundRetail = false;
                p.pilasters = true;
                dress(p, FacadeStyle::Concrete, rng);
                out.placeType = "civic";
            }
            break;
        }
        case DistrictTag::Commercial: {
            if (roll < 0.05) { out.massing = BuildingRecipe::Massing::Park;
                               out.placeType = "park"; break; }
            if (roll < 0.45) {                       // brick shop/mixed midrise
                p.floors = rng.irange(3, 6);
                p.groundRetail = true;
                dress(p, FacadeStyle::Brick, rng);
                // Low shop blocks sometimes carry a gable — main-street
                // silhouettes instead of one flat cornice line everywhere.
                if (p.floors <= 3 && rng.unit() < 0.3) {
                    p.roofStyle = BuildingParams::RoofStyle::Gable;
                    p.roofPitch = rng.range(0.35, 0.5);
                }
                out.placeType = "shop";
            } else if (roll < 0.70) {                // concrete office midrise
                p.floors = rng.irange(4, 8);
                p.groundRetail = true;
                dress(p, FacadeStyle::Concrete, rng);
                out.placeType = "office";
            } else if (roll < 0.82) {                // civic
                p.floors = rng.irange(2, 4);
                p.groundRetail = false;
                p.pilasters = true;
                dress(p, rng.unit() < 0.5 ? FacadeStyle::Concrete
                                          : FacadeStyle::Stucco, rng);
                out.placeType = "civic";
            } else {                                 // stucco/painted walk-ups
                p.floors = rng.irange(3, 5);
                p.groundRetail = false;
                dress(p, rng.unit() < 0.3 ? FacadeStyle::Painted
                                          : FacadeStyle::Stucco, rng);
                out.placeType = "home";
            }
            break;
        }
        case DistrictTag::OldTown: {
            // Dense, low, stucco/brick, round arches, hip roofs — one look.
            p.floors = rng.irange(2, 3);
            p.groundRetail = (rng.unit() < 0.5);
            dress(p, rng.unit() < 0.7 ? FacadeStyle::Stucco : FacadeStyle::Brick,
                  rng);
            p.window.head = OpeningStyle::Head::Round;
            p.window.hood = OpeningStyle::Hood::Arch;
            p.quoins = true;
            p.roofStyle = BuildingParams::RoofStyle::Hip;
            p.roofPitch = rng.range(0.5, 0.7);
            out.placeType = p.groundRetail ? "shop" : "home";
            break;
        }
        case DistrictTag::Industrial: {
            // Wide metal sheds; a plain concrete office up front now and then.
            if (roll < 0.8) {
                p.floors = 1;
                p.groundHeight = rng.range(6.0, 9.0);
                p.solidFacade = true;
                p.groundRetail = false;
                dress(p, FacadeStyle::Metal, rng);
                out.placeType = "office";     // a workplace agents commute to
            } else {
                p.floors = rng.irange(2, 3);
                p.groundRetail = false;
                dress(p, FacadeStyle::Concrete, rng);
                out.placeType = "office";
            }
            break;
        }
        case DistrictTag::Residential: {
            if (roll < 0.08) { out.massing = BuildingRecipe::Massing::Park;
                               out.placeType = "park"; break; }
            if (roll < 0.60) {                       // a HOUSE with a yard
                p.floors = rng.irange(1, 2);
                p.groundRetail = false;
                p.baseCourse = true;
                const Real r2 = rng.unit();
                dress(p, r2 < 0.4 ? FacadeStyle::Painted
                        : r2 < 0.75 ? FacadeStyle::Stucco : FacadeStyle::Brick,
                      rng);
                p.roofStyle = rng.unit() < 0.65 ? BuildingParams::RoofStyle::Gable
                                                : BuildingParams::RoofStyle::Hip;
                p.roofPitch = rng.range(0.45, 0.7);
                out.massing = BuildingRecipe::Massing::RectYard;
                out.placeType = "home";
            } else if (roll < 0.85) {                // low walk-up apartments
                p.floors = rng.irange(2, 4);
                p.groundRetail = false;
                dress(p, rng.unit() < 0.55 ? FacadeStyle::Brick
                                           : FacadeStyle::Stucco, rng);
                if (p.floors <= 3 && rng.unit() < 0.4) {
                    p.roofStyle = BuildingParams::RoofStyle::Gable;
                    p.roofPitch = rng.range(0.4, 0.6);
                }
                out.placeType = "home";
            } else {                                 // corner shop
                p.floors = rng.irange(1, 2);
                p.groundRetail = true;
                dress(p, FacadeStyle::Brick, rng);
                out.placeType = "shop";
            }
            break;
        }
    }
    (void)area;
    capFloors(p, shortSide, slender);
    return out;
}

}  // namespace engine
