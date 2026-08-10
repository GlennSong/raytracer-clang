#include "building_recipe.h"

#include "rng.h"
#include <algorithm>
#include <cmath>

namespace engine {
namespace {

Shape2 largest(const std::vector<Shape2>& v) {
    if (v.empty()) return Shape2{};
    std::size_t best = 0;
    for (std::size_t i = 1; i < v.size(); ++i)
        if (area(v[i]) > area(v[best])) best = i;
    return v[best];
}

// --- element bundles, the port of dress() -------------------------------
//
// dress() was a switch that set a dozen booleans per style. Each of these is
// the same intent expressed as registry rows, which is what makes adding a
// treatment a data edit.

void addWindows(ElementRegistry& r) {
    ElementSpec e;
    e.kind = ElementKind::Opening;
    r.add(e);
}

void addHouseDressing(ElementRegistry& r, bool porch, bool chimney) {
    addWindows(r);
    if (porch) {
        ElementSpec p;
        p.kind = ElementKind::Porch;
        p.on.anyTag = false;
        p.on.tag = EdgeTag::Street;
        p.on.floorMax = 0;
        p.on.bays = ElementSelector::Bays::Centre;
        p.depth = 2.0;
        r.add(p);
        ElementSpec s;
        s.kind = ElementKind::Steps;
        s.on.anyTag = false;
        s.on.tag = EdgeTag::Street;
        s.on.floorMax = 0;
        s.on.bays = ElementSelector::Bays::Centre;
        r.add(s);
    }
    if (chimney) {
        ElementSpec c;
        c.kind = ElementKind::RoofDeck;
        c.on.tier = ElementSelector::kTopTier;
        c.on.bays = ElementSelector::Bays::None;
        c.style = 1;                       // chimney variant
        c.weight = 0.8;
        r.add(c);
    }
}

void addMasonryDressing(ElementRegistry& r, bool quoins, bool baseCourse) {
    addWindows(r);
    ElementSpec cor;
    cor.kind = ElementKind::Cornice;
    cor.on.tier = ElementSelector::kTopTier;
    cor.on.bays = ElementSelector::Bays::None;
    r.add(cor);
    if (baseCourse) {
        ElementSpec b;
        b.kind = ElementKind::BaseCourse;
        b.on.floorMax = 0;
        b.on.bays = ElementSelector::Bays::None;
        r.add(b);
    }
    if (quoins) {
        ElementSpec q;
        q.kind = ElementKind::Quoin;
        q.on.corners = ElementSelector::Corners::Convex;
        q.on.bays = ElementSelector::Bays::None;
        r.add(q);
    }
}

void addRetailGround(ElementRegistry& r) {
    ElementSpec s;
    s.kind = ElementKind::Sign;
    s.on.anyTag = false;
    s.on.tag = EdgeTag::Street;
    s.on.floorMax = 0;
    s.on.bays = ElementSelector::Bays::Centre;
    r.add(s);
    ElementSpec a;
    a.kind = ElementKind::Awning;
    a.on.anyTag = false;
    a.on.tag = EdgeTag::Street;
    a.on.floorMax = 0;
    a.depth = 1.1;
    a.weight = 0.55;
    r.add(a);
}

void addBalconies(ElementRegistry& r, int fromFloor, Real depth,
                  ElementSelector::Bays bays = ElementSelector::Bays::All) {
    ElementSpec b;
    b.kind = ElementKind::Balcony;
    b.on.anyTag = false;
    b.on.tag = EdgeTag::Street;
    b.on.floorMin = fromFloor;
    b.on.bays = bays;
    b.depth = depth;
    r.add(b);
}

void addTowerCrown(ElementRegistry& r) {
    ElementSpec p;
    p.kind = ElementKind::Parapet;
    p.on.tier = ElementSelector::kTopTier;
    p.on.bays = ElementSelector::Bays::None;
    r.add(p);
    ElementSpec d;
    d.kind = ElementKind::RoofDeck;
    d.on.tier = ElementSelector::kTopTier;
    d.on.bays = ElementSelector::Bays::None;
    r.add(d);
}

void addPortico(ElementRegistry& r, int columns) {
    ElementSpec p;
    p.kind = ElementKind::Portico;
    p.on.anyTag = false;
    p.on.tag = EdgeTag::Street;
    p.on.floorMax = 0;
    p.on.bays = ElementSelector::Bays::Centre;
    p.depth = 3.2;
    p.style = columns;
    r.add(p);
    ElementSpec s;
    s.kind = ElementKind::Steps;
    s.on.anyTag = false;
    s.on.tag = EdgeTag::Street;
    s.on.floorMax = 0;
    s.on.bays = ElementSelector::Bays::Centre;
    r.add(s);
    ElementSpec pil;
    pil.kind = ElementKind::Pilaster;
    pil.on.anyTag = false;
    pil.on.tag = EdgeTag::Street;
    pil.on.bays = ElementSelector::Bays::Boundaries;
    r.add(pil);
}

// A compact builder so each recipe below reads as a ROW rather than a function.
struct R {
    LotRecipe r;
    R(const char* name, const char* place) {
        r.name = name;
        r.placeType = place;
    }
    R& plans(std::vector<PlanTemplate> p) { r.plans = std::move(p); return *this; }
    R& mass(MassKind m) { r.mass = m; return *this; }
    R& storeys(int lo, int hi) { r.minStoreys = lo; r.maxStoreys = hi; return *this; }
    R& band(Real h, int lo, int hi) {
        r.bands.push_back(RecipeBand{h, lo, hi});
        return *this;
    }
    R& masses(int lo, int hi) { r.minMasses = lo; r.maxMasses = hi; return *this; }
    R& roof(RoofKind k, Real lo = 0.35, Real hi = 0.55) {
        r.roof = k;
        r.roofPitchMin = lo;
        r.roofPitchMax = hi;
        return *this;
    }
    R& look(Character c, Era e) { r.character = c; r.era = e; return *this; }
    R& retail(bool v = true) { r.groundRetail = v; return *this; }
    R& cantilever(Real minSupport, Real maxOver) {
        r.support.minSupport = minSupport;
        r.support.maxOverhang = maxOver;
        return *this;
    }
    R& weight(Real w) { r.weight = w; return *this; }
    R& dress(void (*fn)(ElementRegistry&)) { fn(r.elements); return *this; }
    LotRecipe done() { return std::move(r); }
};

}  // namespace

const char* planFitName(PlanFit f) {
    switch (f) {
        case PlanFit::Template: return "template";
        case PlanFit::Clipped:  return "clipped";
        case PlanFit::Envelope: return "ENVELOPE (template discarded)";
    }
    return "?";
}

const char* massKindName(MassKind m) {
    switch (m) {
        case MassKind::Simple:      return "simple";
        case MassKind::PodiumTower: return "podium-tower";
        case MassKind::WeddingCake: return "wedding-cake";
        case MassKind::Gherkin:     return "gherkin";
        case MassKind::Pyramid:     return "pyramid";
        case MassKind::Arch:        return "arch";
        case MassKind::MultiMass:   return "multi-mass";
    }
    return "?";
}

// ---------------------------------------------------------------------------
// The book
// ---------------------------------------------------------------------------

RecipeBook stockRecipes() {
    RecipeBook book;
    auto add = [&](LotRecipe r) { book.recipes.push_back(std::move(r)); };

    // --- houses -------------------------------------------------------------
    // bungalow and craftsman COLLAPSE into one row: the originals differed by
    // one bool and a floor count.
    {
        R r("cottage_house", "home");
        r.plans({PlanTemplate::Bar, PlanTemplate::L})
            .storeys(1, 2)
            .band(3.0, 1, 2)
            .roof(RoofKind::Gable, 0.35, 0.6)
            .look(Character::Suburban, Era::PostWar)
            .weight(3.0);
        addHouseDressing(r.r.elements, true, true);
        add(r.done());
    }
    {
        R r("craftsman", "home");
        r.plans({PlanTemplate::L, PlanTemplate::T, PlanTemplate::Bar})
            .storeys(2, 2)
            .band(3.1, 2, 2)
            .roof(RoofKind::Gable, 0.45, 0.62)
            .look(Character::Suburban, Era::Interwar)
            .weight(2.0);
        addHouseDressing(r.r.elements, true, true);
        add(r.done());
    }
    {
        R r("villa", "home");
        r.plans({PlanTemplate::U, PlanTemplate::H, PlanTemplate::Cruciform})
            .storeys(2, 3)
            .band(3.6, 1, 1)
            .band(3.2, 1, 2)
            .roof(RoofKind::Hip, 0.32, 0.48)
            .look(Character::Upscale, Era::Edwardian)
            .weight(1.2);
        addMasonryDressing(r.r.elements, true, true);
        addPortico(r.r.elements, 4);
        add(r.done());
    }
    {
        R r("townhouse", "home");
        // A terrace is a rectangle in plan, but a Victorian one very often has
        // a rear closet wing — which is an L. Listing only Bar is what made a
        // whole terraced block read as identical boxes.
        r.plans({PlanTemplate::Bar, PlanTemplate::L})
            .storeys(2, 4)
            .band(3.4, 1, 1)
            .band(3.0, 1, 3)
            .roof(RoofKind::Parapet)
            .look(Character::Terraced, Era::Victorian)
            .weight(2.6);
        addMasonryDressing(r.r.elements, false, true);
        addBalconies(r.r.elements, 1, 0.55, ElementSelector::Bays::Alternate);
        add(r.done());
    }
    {
        R r("rowhouse", "home");
        r.plans({PlanTemplate::Bar, PlanTemplate::L, PlanTemplate::T})
            .storeys(2, 3)
            .band(3.2, 1, 3)
            .roof(RoofKind::Gable, 0.4, 0.55)
            .look(Character::Terraced, Era::Victorian)
            .weight(2.4);
        addMasonryDressing(r.r.elements, false, true);
        add(r.done());
    }
    {
        R r("walkup", "home");
        r.plans({PlanTemplate::Bar, PlanTemplate::U, PlanTemplate::L})
            .storeys(3, 6)
            .band(3.5, 1, 1)
            .band(3.0, 2, 5)
            .roof(RoofKind::Parapet)
            .look(Character::Terraced, Era::Interwar)
            .weight(1.8);
        addWindows(r.r.elements);
        addBalconies(r.r.elements, 1, 0.9);
        add(r.done());
    }
    {
        R r("apartment_slab", "home");
        r.plans({PlanTemplate::Bar, PlanTemplate::H})
            .storeys(6, 14)
            .band(3.4, 1, 1)
            .band(2.9, 5, 13)
            .roof(RoofKind::Flat)
            .look(Character::Commercial, Era::PostWar)
            .weight(1.0);
        addWindows(r.r.elements);
        addBalconies(r.r.elements, 1, 1.1);
        addTowerCrown(r.r.elements);
        add(r.done());
    }
    {
        // The BRIEF's cantilevered green tower — the one recipe that opts out
        // of containment, which §17.4's support rule exists to allow.
        R r("terraced_tower", "home");
        r.plans({PlanTemplate::Bar, PlanTemplate::Pinwheel, PlanTemplate::Trefoil})
            .mass(MassKind::WeddingCake)
            .storeys(10, 26)
            .band(3.5, 1, 1)
            .band(3.1, 9, 25)
            .roof(RoofKind::Flat)
            .look(Character::Downtown, Era::Contemporary)
            .cantilever(0.72, 2.4)
            .weight(0.8);
        addWindows(r.r.elements);
        addBalconies(r.r.elements, 1, 2.0);
        addTowerCrown(r.r.elements);
        add(r.done());
    }

    // --- high street --------------------------------------------------------
    {
        R r("shopfront", "shop");
        // A shop is a bar to the street with a store room behind it.
        r.plans({PlanTemplate::Bar, PlanTemplate::L, PlanTemplate::BowFront})
            .storeys(1, 3)
            .band(4.2, 1, 1)              // a retail ground floor is TALLER
            .band(3.1, 0, 2)
            .roof(RoofKind::Parapet)
            .look(Character::Commercial, Era::Interwar)
            .retail()
            .weight(3.0);
        addMasonryDressing(r.r.elements, false, true);
        addRetailGround(r.r.elements);
        add(r.done());
    }
    {
        R r("corner_shopfront", "shop");
        // The corner ENTRANCE lives in the plan, as a chamfer — which is why
        // "does this lot have a corner" had to be a tag the parceller computes.
        r.plans({PlanTemplate::ChamferedSquare, PlanTemplate::CurvedCornerSlab})
            .storeys(2, 4)
            .band(4.3, 1, 1)
            .band(3.1, 1, 3)
            .roof(RoofKind::Parapet)
            .look(Character::Commercial, Era::Interwar)
            .retail()
            .weight(1.4);
        addMasonryDressing(r.r.elements, true, true);
        addRetailGround(r.r.elements);
        add(r.done());
    }
    {
        R r("mixed_use", "shop");
        r.plans({PlanTemplate::Bar, PlanTemplate::L, PlanTemplate::Courtyard, PlanTemplate::AtriumRing})
            .storeys(3, 8)
            .band(4.4, 1, 1)
            .band(3.0, 2, 7)
            .roof(RoofKind::Parapet)
            .look(Character::Commercial, Era::Modern)
            .retail()
            .weight(2.2);
        addWindows(r.r.elements);
        addRetailGround(r.r.elements);
        addBalconies(r.r.elements, 2, 0.8, ElementSelector::Bays::Alternate);
        add(r.done());
    }
    {
        // STRIP MALL — one of the five whose name promises a complex and which
        // emitted a single prism. Now several masses.
        R r("strip_mall", "shop");
        r.plans({PlanTemplate::Bar})
            .mass(MassKind::MultiMass)
            .masses(2, 4)
            .storeys(1, 1)
            .band(4.6, 1, 1)
            .roof(RoofKind::Flat)
            .look(Character::Commercial, Era::PostWar)
            .retail()
            .weight(1.2);
        addRetailGround(r.r.elements);
        addWindows(r.r.elements);
        add(r.done());
    }
    {
        R r("market_hall", "shop");
        r.plans({PlanTemplate::Bar, PlanTemplate::BowFront, PlanTemplate::SawtoothShed})
            .mass(MassKind::MultiMass)
            .masses(2, 3)
            .storeys(1, 2)
            .band(6.5, 1, 1)              // a hall's volume, not an office's
            .band(3.4, 0, 1)
            .roof(RoofKind::Sawtooth)
            .look(Character::Commercial, Era::Victorian)
            .retail()
            .weight(0.7);
        addMasonryDressing(r.r.elements, true, true);
        add(r.done());
    }
    {
        R r("bank", "civic");
        r.plans({PlanTemplate::ChamferedSquare, PlanTemplate::Bar, PlanTemplate::Hexagon})
            .storeys(2, 5)
            .band(5.2, 1, 1)              // a banking hall
            .band(3.3, 1, 4)
            .roof(RoofKind::Parapet)
            .look(Character::Civic, Era::Edwardian)
            .weight(0.7);
        addMasonryDressing(r.r.elements, true, true);
        addPortico(r.r.elements, 6);
        add(r.done());
    }
    {
        R r("hotel", "hotel");
        // One of the SIX that overloaded floorHeight: a lobby, a mezzanine and
        // a bedroom stack are three different floor heights.
        r.plans({PlanTemplate::Bar, PlanTemplate::U, PlanTemplate::L, PlanTemplate::CourtyardWings})
            .mass(MassKind::PodiumTower)
            .storeys(5, 18)
            .band(5.4, 1, 1)              // lobby
            .band(4.0, 1, 1)              // function rooms
            .band(2.9, 3, 16)             // bedrooms
            .roof(RoofKind::Parapet)
            .look(Character::Commercial, Era::Modern)
            .weight(0.9);
        addWindows(r.r.elements);
        addRetailGround(r.r.elements);
        addTowerCrown(r.r.elements);
        add(r.done());
    }

    // --- offices and towers -------------------------------------------------
    {
        R r("office_block", "office");
        r.plans({PlanTemplate::Bar, PlanTemplate::L, PlanTemplate::Courtyard, PlanTemplate::AtriumRing})
            .storeys(4, 12)
            .band(4.0, 1, 1)
            .band(3.3, 3, 11)
            .roof(RoofKind::Flat)
            .look(Character::Commercial, Era::PostWar)
            .weight(1.6);
        addWindows(r.r.elements);
        addTowerCrown(r.r.elements);
        add(r.done());
    }
    {
        R r("office_tower", "office");
        r.plans({PlanTemplate::Bar, PlanTemplate::Octagon, PlanTemplate::ChamferedSquare, PlanTemplate::WedgeTower})
            .mass(MassKind::PodiumTower)
            .storeys(12, 45)
            .band(5.0, 1, 1)
            .band(3.4, 11, 44)
            .roof(RoofKind::Parapet)
            .look(Character::Downtown, Era::Modern)
            .retail()
            .weight(1.4);
        addWindows(r.r.elements);
        addRetailGround(r.r.elements);
        addTowerCrown(r.r.elements);
        add(r.done());
    }
    {
        R r("glass_tower", "office");
        r.plans({PlanTemplate::Bar, PlanTemplate::CurvedCornerSlab, PlanTemplate::LobedTower, PlanTemplate::Trefoil})
            .mass(MassKind::WeddingCake)
            .storeys(20, 70)
            .band(5.2, 1, 1)
            .band(3.5, 19, 69)
            .roof(RoofKind::Parapet)
            .look(Character::Downtown, Era::Contemporary)
            .retail()
            .weight(1.2);
        addWindows(r.r.elements);
        addRetailGround(r.r.elements);
        addTowerCrown(r.r.elements);
        add(r.done());
    }
    {
        // The GHERKIN: one plan, a profile curve, no loft and NO TWIST. The
        // silhouette is the profile — which is the whole reason profile and
        // loft are separate knobs.
        R r("profile_tower", "office");
        r.plans({PlanTemplate::Octagon, PlanTemplate::LobedTower, PlanTemplate::Trefoil})
            .mass(MassKind::Gherkin)
            .storeys(24, 46)
            .band(4.6, 1, 1)
            .band(3.5, 23, 45)
            .roof(RoofKind::Dome)
            .look(Character::Downtown, Era::Contemporary)
            .weight(0.35);
        addWindows(r.r.elements);
        addTowerCrown(r.r.elements);
        add(r.done());
    }
    {
        R r("arch_tower", "office");
        r.plans({PlanTemplate::Bar})
            .mass(MassKind::Arch)
            .storeys(18, 40)
            .band(5.0, 1, 1)
            .band(3.5, 17, 39)
            .roof(RoofKind::Flat)
            .look(Character::Downtown, Era::Contemporary)
            .cantilever(0.35, 1e9)
            .weight(0.25);
        addWindows(r.r.elements);
        addTowerCrown(r.r.elements);
        add(r.done());
    }
    {
        R r("stepped_tower", "office");
        r.plans({PlanTemplate::Bar, PlanTemplate::ChamferedSquare, PlanTemplate::WedgeTower})
            .mass(MassKind::WeddingCake)
            .storeys(14, 34)
            .band(5.0, 1, 1)
            .band(3.4, 13, 33)
            .roof(RoofKind::Spire)
            .look(Character::Downtown, Era::Interwar)
            .weight(0.8);
        addMasonryDressing(r.r.elements, false, true);
        addTowerCrown(r.r.elements);
        add(r.done());
    }
    {
        R r("flatiron", "office");
        r.plans({PlanTemplate::Flatiron})
            .storeys(5, 16)
            .band(4.6, 1, 1)
            .band(3.3, 4, 15)
            .roof(RoofKind::Parapet)
            .look(Character::Commercial, Era::Edwardian)
            .retail()
            .weight(0.5);
        addMasonryDressing(r.r.elements, true, true);
        addRetailGround(r.r.elements);
        add(r.done());
    }

    // --- civic --------------------------------------------------------------
    {
        // CHURCH — nave plus tower, two masses. The original emitted one prism
        // with a steeple stuck on it.
        R r("church", "civic");
        r.plans({PlanTemplate::Bar, PlanTemplate::Cruciform, PlanTemplate::CrossApse})
            .mass(MassKind::MultiMass)
            .masses(2, 2)
            .storeys(1, 2)
            .band(9.0, 1, 1)              // a nave's volume
            .roof(RoofKind::Steeple, 0.55, 0.75)
            .look(Character::Civic, Era::Victorian)
            .weight(0.6);
        addMasonryDressing(r.r.elements, true, true);
        add(r.done());
    }
    {
        R r("cathedral", "civic");
        r.plans({PlanTemplate::Cruciform, PlanTemplate::CrossApse})
            .mass(MassKind::MultiMass)
            .masses(2, 3)
            .storeys(1, 2)
            .band(14.0, 1, 1)
            .roof(RoofKind::Spire, 0.6, 0.8)
            .look(Character::Civic, Era::Victorian)
            .weight(0.2);
        addMasonryDressing(r.r.elements, true, true);
        addPortico(r.r.elements, 6);
        add(r.done());
    }
    {
        R r("civic_hall", "civic");
        r.plans({PlanTemplate::H, PlanTemplate::Bar, PlanTemplate::U, PlanTemplate::CourtyardWings})
            .storeys(2, 5)
            .band(6.0, 1, 1)              // a hall, then offices above
            .band(3.6, 1, 4)
            .roof(RoofKind::Dome)
            .look(Character::Civic, Era::Edwardian)
            .weight(0.5);
        addMasonryDressing(r.r.elements, true, true);
        addPortico(r.r.elements, 8);
        add(r.done());
    }
    {
        R r("capitol", "civic");
        r.plans({PlanTemplate::Cruciform, PlanTemplate::H, PlanTemplate::CrossApse, PlanTemplate::RadialWings})
            .storeys(2, 4)
            .band(7.5, 1, 1)
            .band(4.5, 1, 3)
            .roof(RoofKind::Dome)
            .look(Character::Civic, Era::Victorian)
            .weight(0.15);
        addMasonryDressing(r.r.elements, true, true);
        addPortico(r.r.elements, 10);
        add(r.done());
    }
    {
        R r("library", "civic");
        r.plans({PlanTemplate::Bar, PlanTemplate::U, PlanTemplate::Hexagon, PlanTemplate::AtriumRing})
            .storeys(2, 4)
            .band(5.4, 1, 1)              // reading room
            .band(3.6, 1, 3)
            .roof(RoofKind::Parapet)
            .look(Character::Civic, Era::Edwardian)
            .weight(0.6);
        addMasonryDressing(r.r.elements, true, true);
        addPortico(r.r.elements, 4);
        add(r.done());
    }
    {
        R r("museum", "civic");
        r.plans({PlanTemplate::H, PlanTemplate::Courtyard, PlanTemplate::AtriumRing, PlanTemplate::Hexagon})
            .storeys(2, 3)
            .band(6.5, 1, 1)              // gallery volumes
            .band(5.0, 1, 2)
            .roof(RoofKind::Sawtooth)
            .look(Character::Civic, Era::Victorian)
            .weight(0.4);
        addMasonryDressing(r.r.elements, true, true);
        addPortico(r.r.elements, 8);
        add(r.done());
    }
    {
        R r("school", "school");
        r.plans({PlanTemplate::H, PlanTemplate::L, PlanTemplate::U, PlanTemplate::CourtyardWings})
            .mass(MassKind::MultiMass)
            .masses(1, 3)
            .storeys(2, 3)
            .band(4.0, 1, 1)
            .band(3.6, 1, 2)
            .roof(RoofKind::Hip, 0.28, 0.4)
            .look(Character::Civic, Era::Interwar)
            .weight(0.8);
        addMasonryDressing(r.r.elements, false, true);
        add(r.done());
    }
    {
        // HOSPITAL — wards, a tower and a service block: three masses, three
        // floor heights. Another of the five that promised a complex.
        R r("hospital", "hospital");
        r.plans({PlanTemplate::H, PlanTemplate::T, PlanTemplate::Bar, PlanTemplate::RadialWings, PlanTemplate::CourtyardWings})
            .mass(MassKind::MultiMass)
            .masses(2, 4)
            .storeys(3, 10)
            .band(4.6, 1, 1)
            .band(3.4, 2, 9)
            .roof(RoofKind::Flat)
            .look(Character::Civic, Era::PostWar)
            .weight(0.5);
        addWindows(r.r.elements);
        addTowerCrown(r.r.elements);
        add(r.done());
    }

    // --- industrial and edge-of-city ---------------------------------------
    {
        R r("factory", "work");
        r.plans({PlanTemplate::Bar, PlanTemplate::L, PlanTemplate::SawtoothShed})
            .mass(MassKind::MultiMass)
            .masses(1, 3)
            .storeys(1, 3)
            .band(7.0, 1, 1)              // the shed
            .band(3.4, 0, 2)              // the office block on the front
            .roof(RoofKind::Sawtooth)
            .look(Character::Industrial, Era::Victorian)
            .weight(1.0);
        addWindows(r.r.elements);
        add(r.done());
    }
    {
        R r("warehouse", "work");
        r.plans({PlanTemplate::Bar, PlanTemplate::SawtoothShed})
            .storeys(1, 2)
            .band(8.5, 1, 1)
            .band(3.4, 0, 1)
            .roof(RoofKind::Flat)
            .look(Character::Industrial, Era::Modern)
            .weight(1.2);
        addWindows(r.r.elements);
        add(r.done());
    }
    {
        R r("big_box", "shop");
        r.plans({PlanTemplate::Bar, PlanTemplate::ChamferedSquare})
            .storeys(1, 1)
            .band(9.0, 1, 1)
            .roof(RoofKind::Flat)
            .look(Character::Commercial, Era::Modern)
            .retail()
            .weight(1.4);
        addRetailGround(r.r.elements);
        add(r.done());
    }
    {
        R r("office_park", "office");
        r.plans({PlanTemplate::L, PlanTemplate::U, PlanTemplate::Bar, PlanTemplate::Hexagon})
            .mass(MassKind::MultiMass)
            .masses(2, 5)
            .storeys(2, 5)
            .band(4.2, 1, 1)
            .band(3.4, 1, 4)
            .roof(RoofKind::Flat)
            .look(Character::Commercial, Era::Modern)
            .weight(1.2);
        addWindows(r.r.elements);
        add(r.done());
    }
    {
        R r("university_campus", "school");
        r.plans({PlanTemplate::Courtyard, PlanTemplate::H, PlanTemplate::U, PlanTemplate::CourtyardWings, PlanTemplate::RadialWings})
            .mass(MassKind::MultiMass)
            .masses(3, 6)
            .storeys(2, 6)
            .band(4.8, 1, 1)
            .band(3.6, 1, 5)
            .roof(RoofKind::Hip, 0.28, 0.42)
            .look(Character::Civic, Era::Edwardian)
            .weight(1.0);
        addMasonryDressing(r.r.elements, true, true);
        addPortico(r.r.elements, 6);
        add(r.done());
    }
    {
        R r("works", "work");
        r.plans({PlanTemplate::Bar, PlanTemplate::L, PlanTemplate::SawtoothShed})
            .mass(MassKind::MultiMass)
            .masses(2, 4)
            .storeys(1, 3)
            .band(6.5, 1, 1)
            .band(3.4, 0, 2)
            .roof(RoofKind::Flat)
            .look(Character::Industrial, Era::Interwar)
            .weight(1.0);
        addWindows(r.r.elements);
        add(r.done());
    }
    {
        R r("parking_deck", "parking");
        r.plans({PlanTemplate::Bar})
            .storeys(3, 7)
            .band(3.0, 3, 7)
            .roof(RoofKind::Flat)
            .look(Character::Commercial, Era::PostWar)
            .weight(0.6);
        add(r.done());
    }
    {
        // The LUXOR: one plan, a linear taper. No twist, no loft.
        R r("pyramid_landmark", "civic");
        r.plans({PlanTemplate::Bar, PlanTemplate::Octagon, PlanTemplate::Pentagon, PlanTemplate::Hexagon})
            .mass(MassKind::Pyramid)
            .storeys(12, 30)
            .band(6.0, 1, 1)
            .band(3.6, 11, 29)
            .roof(RoofKind::Flat)
            .look(Character::Downtown, Era::Contemporary)
            .weight(0.12);
        addWindows(r.r.elements);
        add(r.done());
    }
    return book;
}

const LotRecipe* RecipeBook::byName(const std::string& name) const {
    for (const LotRecipe& r : recipes)
        if (r.name == name) return &r;
    return nullptr;
}

std::vector<int> RecipeBook::forProgram(const LotProgram& program,
                                        int storeysAvailable) const {
    std::vector<int> out;
    // WHICH BUILDINGS BELONG HERE comes from the program, by name. Matching on
    // the storey range alone is not "a recipe that fits" — it is "a recipe of
    // roughly the right height", and it hands a residential walkup an
    // industrial works shed. That is not a hypothetical: it is what the first
    // drawn sheet showed, on the first lot.
    for (const std::string& want : program.recipes) {
        for (std::size_t i = 0; i < recipes.size(); ++i) {
            if (recipes[i].name != want) continue;
            if (recipes[i].minStoreys > storeysAvailable) break;
            // The PROGRAM's height range binds too. A walkup program that says
            // 3-6 storeys must not end up 14 storeys tall because the recipe it
            // named happens to reach that high — the program is the brief for
            // the lot, and a brief that can be ignored is not a brief.
            if (recipes[i].minStoreys > program.maxStoreys) break;
            out.push_back(static_cast<int>(i));
            break;
        }
    }
    if (!out.empty()) return out;

    // FALLBACK, for a program that names nothing or whose named recipes all
    // want more height than the plate can carry: anything of the right height
    // whose place type is not obviously wrong for the frontage.
    for (std::size_t i = 0; i < recipes.size(); ++i) {
        const LotRecipe& r = recipes[i];
        if (r.minStoreys > storeysAvailable) continue;
        if (r.maxStoreys < program.minStoreys) continue;
        if (r.minStoreys > program.maxStoreys) continue;
        if (program.frontage == FrontageKind::Direct && !r.groundRetail &&
            r.placeType == "home")
            continue;
        out.push_back(static_cast<int>(i));
    }
    return out;
}

int RecipeBook::pick(const LotProgram& program, int storeysAvailable,
                     std::uint32_t seed) const {
    std::vector<int> cand = forProgram(program, storeysAvailable);
    if (cand.empty()) return -1;
    Real total = 0;
    for (int i : cand) total += std::max(Real(0), recipes[i].weight);
    if (total <= 0) return cand.front();
    Rng rng(seed ? seed : 19u);
    Real r = rng.unit() * total;
    for (int i : cand) {
        r -= std::max(Real(0), recipes[i].weight);
        if (r <= 0) return i;
    }
    return cand.back();
}

// ---------------------------------------------------------------------------
// Assembly
// ---------------------------------------------------------------------------

BuiltBuilding buildFromRecipe(const LotRecipe& recipe,
                              const Shape2& envelope, const LotTags& tags,
                              const PaletteLibrary& palettes,
                              std::uint32_t districtSeed, std::uint32_t seed) {
    BuiltBuilding out;
    out.recipeName = recipe.name;
    if (envelope.outer.size() < 3) return out;
    Rng rng(seed ? seed : 23u);

    // HEIGHT: the recipe's range, capped by the PLATE (§17.2) — one model
    // consulted once, not a coreness constant per recipe. `tags.maxStoreys` is
    // the whole budget: a caller that also has a program should hand in the
    // smaller of the plate cap and the program's own ceiling.
    const int plateCap = std::max(1, static_cast<int>(tags.maxStoreys));
    int storeys = rng.irange(recipe.minStoreys, recipe.maxStoreys);
    storeys = std::max(1, std::min({storeys, recipe.maxStoreys, plateCap}));
    out.storeys = storeys;

    // PLAN: fit a template into the envelope's OWN ORIENTED FRAME.
    //
    // Building it against the axis-aligned bounds and clipping was wrong twice
    // over. A lot on an angled street has an AABB much larger than itself, so
    // the template overhangs and the clip eats its arms — measured, an L lost
    // half its instances and a T lost five of six, silently. And a non-
    // rectangular envelope trimmed even a plain bar. Fitting in the oriented
    // frame and SHRINKING to fit keeps the shape the recipe asked for; the
    // clip then has nothing left to do in the common case.
    const Poly2 envRing = tessellate(envelope.outer, 0.15);
    const OBB2 box = orientedBoundingBox(envRing);
    const Real w = std::max(Real(4), box.half[0] * 2);
    const Real d = std::max(Real(4), box.half[1] * 2);
    const PlanTemplate tmpl =
        recipe.plans.empty()
            ? PlanTemplate::Bar
            : recipe.plans[rng.irange(0, static_cast<int>(recipe.plans.size()) - 1)];
    Shape2 tpl = planTemplate(tmpl, w, d, seed);

    // Local (0..w, 0..d) -> the envelope's oriented frame.
    auto toFrame = [&](const Vec2& p, Real scale) {
        const Vec2 c(w * 0.5, d * 0.5);
        const Vec2 r = (p - c) * scale;
        return box.center + box.axis[0] * r.x + box.axis[1] * r.y;
    };
    auto placed = [&](Real scale) {
        Shape2 t = tpl;
        for (Edge2& e : t.outer.edges) e.a = toFrame(e.a, scale);
        for (Loop2& h : t.holes)
            for (Edge2& e : h.edges) e.a = toFrame(e.a, scale);
        orient(t);
        return t;
    };
    // Shrink until it sits inside the envelope. The OBB already bounds the
    // envelope, so scale 1 fits whenever the envelope IS its own box; anything
    // concave needs a little less.
    Real fit = 1.0;
    for (int i = 0; i < 6; ++i) {
        Shape2 cand = placed(fit);
        std::vector<Shape2> outside =
            shapeBool({cand}, {envelope}, BoolOp::Subtract);
        Real spill = 0;
        for (const Shape2& o : outside) spill += area(o);
        if (spill <= area(cand) * 0.02) break;
        fit *= 0.92;
    }
    tpl = placed(fit);
    out.plan = tmpl;
    const Real tplArea = area(tpl);
    Shape2 plan = clipToEnvelope(tpl, envelope);
    if (plan.outer.size() < 3) {
        plan = envelope;
        out.planFit = PlanFit::Envelope;
    } else if (area(plan) < tplArea * 0.98) {
        out.planFit = PlanFit::Clipped;
    }

    // Plan quality is enforced before the stack can amplify a defect.
    PlanQuality quality;
    if (!quality.ok(plan)) {
        plan = envelope;
        out.planFit = PlanFit::Envelope;
    }

    // Tag the plan's walls from the envelope's, so the facade grammar knows
    // which way the street is without re-deriving it.
    if (envelope.outer.size() >= 3) {
        for (std::size_t i = 0; i < plan.outer.size(); ++i) {
            const Vec2 m = (plan.outer.start(i) + plan.outer.end(i)) * 0.5;
            Real best = 1e30;
            EdgeTag tag = EdgeTag::Side;
            for (std::size_t j = 0; j < envelope.outer.size(); ++j) {
                const Vec2 a = envelope.outer.start(j), b = envelope.outer.end(j);
                const Vec2 e = b - a;
                const Real len2 = e.lengthSquared();
                Real u = len2 > 1e-12 ? dot(m - a, e) / len2 : 0;
                u = std::max(Real(0), std::min(Real(1), u));
                const Real dist = (m - (a + e * u)).length();
                if (dist < best) { best = dist; tag = envelope.outer.edges[j].tag; }
            }
            plan.outer.edges[i].tag = best < 2.5 ? tag : EdgeTag::Side;
        }
    }

    // BANDS: the per-band floor heights six recipes were faking by overloading
    // one number. Distribute the storey budget across the bands in order.
    std::vector<Storey> storeysOut;
    int left = storeys;
    for (std::size_t i = 0; i < recipe.bands.size() && left > 0; ++i) {
        const RecipeBand& b = recipe.bands[i];
        const bool last = i + 1 == recipe.bands.size();
        int n = last ? left : std::min(left, rng.irange(b.minCount, b.maxCount));
        n = std::min(n, left);
        if (n <= 0 && !last) continue;
        if (n <= 0) n = left;
        storeysOut.push_back(Storey{b.height, n});
        left -= n;
    }
    if (storeysOut.empty()) storeysOut.push_back(Storey{3.2, storeys});
    if (left > 0) storeysOut.back().count += left;

    // The finished plan's extent, for the mass kinds that slice it up.
    Vec2 lo, hi;
    bounds(plan, lo, hi);

    // MASS: each kind is a configuration of the same stack, not a code path.
    switch (recipe.mass) {
        case MassKind::Simple:
        case MassKind::MultiMass: {
            Tier t;
            t.plans = {plan};
            if (recipe.mass == MassKind::MultiMass) {
                // Several masses on one lot — the five recipes whose name
                // promised a complex and which emitted one prism. Split the
                // envelope into bands and stand a mass on each.
                const int n = rng.irange(recipe.minMasses, recipe.maxMasses);
                if (n > 1) {
                    t.plans.clear();
                    const bool alongX = (hi.x - lo.x) >= (hi.y - lo.y);
                    const Real span = alongX ? (hi.x - lo.x) : (hi.y - lo.y);
                    const Real gap = std::min(Real(6), span * 0.06);
                    const Real each = (span - gap * (n - 1)) / n;
                    for (int k = 0; k < n && each > 3; ++k) {
                        const Real s0 = (alongX ? lo.x : lo.y) + k * (each + gap);
                        Shape2 slice = alongX
                            ? rectShape(s0, lo.y - 1, s0 + each, hi.y + 1)
                            : rectShape(lo.x - 1, s0, hi.x + 1, s0 + each);
                        std::vector<Shape2> got =
                            shapeBool({plan}, {slice}, BoolOp::Intersect);
                        for (Shape2& g : got)
                            if (area(g) > 12) t.plans.push_back(std::move(g));
                    }
                    if (t.plans.empty()) t.plans = {plan};
                }
            }
            t.storeys = storeysOut;
            t.support = recipe.support;
            out.stack.tiers.push_back(std::move(t));
            break;
        }
        case MassKind::PodiumTower: {
            const int podiumFloors = std::min(storeys - 1, rng.irange(2, 4));
            std::vector<Shape2> shaft = offsetShape(plan, -std::min(
                Real(6), std::min(w, d) * 0.18));
            Shape2 tower = shaft.empty() ? plan : largest(shaft);
            out.stack = podiumTower(plan, tower, std::max(1, podiumFloors),
                                    std::max(1, storeys - podiumFloors),
                                    storeysOut.back().height);
            out.stack.tiers[0].support = recipe.support;
            break;
        }
        case MassKind::WeddingCake: {
            const int tiers = std::max(2, std::min(5, storeys / 6));
            out.stack = weddingCake(plan, tiers, std::max(1, storeys / tiers), 0.16,
                                    storeysOut.back().height);
            for (Tier& t : out.stack.tiers) t.support = recipe.support;
            break;
        }
        case MassKind::Gherkin:
            out.stack = gherkin(plan, storeys, storeysOut.back().height);
            break;
        case MassKind::Pyramid:
            out.stack = pyramid(plan, storeys, storeysOut.back().height);
            break;
        case MassKind::Arch: {
            const bool alongX = (hi.x - lo.x) >= (hi.y - lo.y);
            const Real span = alongX ? (hi.x - lo.x) : (hi.y - lo.y);
            const Real leg = span * 0.30;
            Shape2 a = alongX ? rectShape(lo.x, lo.y, lo.x + leg, hi.y)
                              : rectShape(lo.x, lo.y, hi.x, lo.y + leg);
            Shape2 b = alongX ? rectShape(hi.x - leg, lo.y, hi.x, hi.y)
                              : rectShape(lo.x, hi.y - leg, hi.x, hi.y);
            out.stack = archTower(a, b, storeys, storeysOut.back().height);
            break;
        }
    }
    // Give the first tier the recipe's real band structure (the convenience
    // constructors build their own).
    if (!out.stack.tiers.empty() && recipe.mass != MassKind::PodiumTower &&
        recipe.mass != MassKind::WeddingCake)
        out.stack.tiers[0].storeys = storeysOut;

    // SURFACES + ELEMENTS.
    const MaterialSet probe =
        palettes.sets.empty() ? MaterialSet{} : palettes.sets[0];
    out.surfaces = surfacesFor(out.stack, probe.window.preferredBay);
    out.placements = resolveElements(recipe.elements, out.surfaces, seed);

    // MATERIALS: one coherent set per tier, district-led.
    std::vector<int> tierStoreys;
    for (const Tier& t : out.stack.tiers) tierStoreys.push_back(t.levelCount());
    out.materials = assignMaterials(palettes, recipe.character, recipe.era,
                                    tierStoreys, districtSeed, seed);
    return out;
}

}  // namespace engine
