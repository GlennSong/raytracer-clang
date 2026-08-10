#ifndef RAYTRACER_ENGINE_PROCGEN_CITY_MASS_STACK_H
#define RAYTRACER_ENGINE_PROCGEN_CITY_MASS_STACK_H

#include "shape_ops.h"
#include <vector>

namespace engine {

// Stacking floors to make a building (docs/lot-system-plan.md §5, layer L1).
// Replaces `floors` + `setbackEvery` with a description that can express what
// the brief actually asked for: podium plus tower, wedding-cake setbacks, twin
// towers on a shared base, terraces on an exposed tier roof, and a base/shaft/
// crown that changes cladding at the transition.
//
// FOUR INDEPENDENT KNOBS, deliberately not conflated:
//
//   plan     WHAT the footprint is        (from the plan grammar)
//   profile  HOW BIG it is at height t    (a scalar curve — the Gherkin)
//   loft     WHAT IT BECOMES              (morph toward another plan — the arch)
//   twist    HOW IT ROTATES               (rarely — see the quota note)
//
// Conflating "profile" with "loft" is how a system ends up twisting everything:
// twist becomes the only knob it has for "make this interesting", and then
// every tower has one and the skyline loses its architecture. A Gherkin is ONE
// plan with a profile curve and no loft at all. A pyramid is one plan with a
// linear profile. An arch is a loft that MERGES two plans into one going up.

// --- vertical description --------------------------------------------------

// A run of identical floors. Per-band heights are what six of the existing
// recipes are already fighting for by overloading floorHeight/groundHeight — a
// hotel's ground floor is not its bedroom floor.
struct Storey {
    Real height = 3.2;
    int count = 1;
};

enum class ProfileKind {
    Constant,    // a plain prism
    Taper,       // pyramid, obelisk, Luxor
    Swell,       // the Gherkin: bulge low, nose in at the top
    Waist,       // pinched middle
};

// SUPPORT, not containment (§17.4). "Every tier is clipped to the tier below"
// is too strong — it makes Fallingwater and every cantilevered green tower
// impossible. The real constraint is that enough of a mass is held up:
//
//   support  = area(above INTERSECT below) / area(above)   >= minSupport
//   overhang = max distance of `above` beyond `below`'s edge <= maxOverhang
//
// Containment is simply minSupport = 1, which stays the default so ordinary
// buildings do not start floating. This REPLACES the clip rule rather than
// bolting a special case onto it.
struct SupportRule {
    Real minSupport = 1.0;
    Real maxOverhang = 0.0;
};

struct Tier {
    std::vector<Shape2> plans;      // >1 plan = twin towers on a shared base
    std::vector<Shape2> loftTo;     // optional morph target across the tier
    std::vector<Storey> storeys;
    ProfileKind profile = ProfileKind::Constant;
    Real profileTo = 1.0;           // Taper: the scale reached at the top
    Real twistPerLevel = 0.0;       // radians; default 0 — twist is a quota
    SupportRule support;
    int materialSet = 0;            // a tier may re-clad (base/shaft/crown)

    int levelCount() const;
    Real height() const;
};

// The foundation is ITS OWN OBJECT, not derived from the plan: usually the plan
// outset by a plinth reveal, but free to differ — a podium, a terraced base on
// a slope, a stepped foundation. It is what beds into the graded terrain, and
// the plan rests on it. This is the brief's separation, and it is also what
// fixes today's one-flat-pad-per-building limit on steep ground.
struct Foundation {
    Shape2 plan;
    Real topY = 0;          // the level the building's ground floor sits on
    Real thickness = 0.45;  // how far it beds down into the graded terrain
    int materialSet = 0;
};

struct MassStack {
    std::vector<Tier> tiers;
    Foundation base;

    Real height() const;
    int levelCount() const;
};

// --- expansion -------------------------------------------------------------

// One built floor: the plans it occupies and the slab heights it spans. This is
// what the mesher consumes — it never sees a tier, only levels.
struct Level {
    std::vector<Shape2> plans;
    Real y0 = 0, y1 = 0;
    int tier = 0;
    int indexInTier = 0;
    int materialSet = 0;
};

// Expand a stack into levels, applying profile, loft and twist per level and
// enforcing each tier's support rule against the level below. Deterministic.
std::vector<Level> expandLevels(const MassStack& stack);

// --- the rules, exposed so they can be tested and reused --------------------

Real profileAt(ProfileKind kind, Real t, Real profileTo);

// area(above INTERSECT below) / area(above). 1 = fully supported, 0 = floating.
Real supportRatio(const std::vector<Shape2>& above,
                  const std::vector<Shape2>& below);

// How far `above` reaches beyond `below`'s boundary, in metres.
Real overhangDistance(const std::vector<Shape2>& above,
                      const std::vector<Shape2>& below);

// Clip `above` until the rule holds: intersect with `below` grown by
// maxOverhang. With the default rule (minSupport 1, maxOverhang 0) this is
// exactly containment, so nothing changes for ordinary buildings.
std::vector<Shape2> enforceSupport(const std::vector<Shape2>& above,
                                   const std::vector<Shape2>& below,
                                   const SupportRule& rule);

// Morph A toward B. Runs through the SAMPLED FIELD, so the morph can change
// topology: two towers merge into one arch, a solid plate opens into a donut.
// A vertex-to-vertex interpolation could do none of that.
std::vector<Shape2> loftPlans(const std::vector<Shape2>& a,
                              const std::vector<Shape2>& b, Real t,
                              Real cell = 0.5);

// Scale about the plan set's shared centroid, then rotate. ORDER MATTERS: the
// profile is a size and the twist is a frame. Applied to the FINISHED storey
// plan — twisting the sample frame instead makes successive levels cross.
std::vector<Shape2> transformPlans(const std::vector<Shape2>& plans, Real scale,
                                   Real twist);

// --- convenience -----------------------------------------------------------

// The common shapes, as stacks. Each is a few lines of the description above —
// which is the point: they are configurations, not code paths.
MassStack simpleStack(const Shape2& plan, int floors, Real floorHeight = 3.2);
MassStack podiumTower(const Shape2& podium, const Shape2& tower, int podiumFloors,
                      int towerFloors, Real floorHeight = 3.4);
MassStack weddingCake(const Shape2& plan, int tiers, int floorsPerTier,
                      Real shrinkPerTier = 0.16, Real floorHeight = 3.4);
MassStack gherkin(const Shape2& plan, int floors, Real floorHeight = 3.6);
MassStack pyramid(const Shape2& plan, int floors, Real floorHeight = 4.0);
// Two towers that merge into one mass at the top: the Singapore arch.
MassStack archTower(const Shape2& legA, const Shape2& legB, int floors,
                    Real floorHeight = 3.5);

}  // namespace engine

#endif
