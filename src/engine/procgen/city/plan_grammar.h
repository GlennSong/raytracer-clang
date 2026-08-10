#ifndef RAYTRACER_ENGINE_PROCGEN_CITY_PLAN_GRAMMAR_H
#define RAYTRACER_ENGINE_PROCGEN_CITY_PLAN_GRAMMAR_H

#include "shape_ops.h"
#include <string>
#include <vector>

namespace engine {

// Drawing a floor plan BY RULES (docs/lot-system-plan.md §4, layer L1). The
// brief's "start with a box, iterate each edge, outset, grow wings, inset the
// back face" — made into a grammar that rewrites Shape2 regions.
//
// The point of a grammar rather than a pile of templates is that a plan becomes
// a short script over a small set of verbs, so adding a building shape is DATA
// (eventually a Lua table, §17.8) rather than a new C++ function. The named
// templates below are themselves just short scripts.

// --- selectors -------------------------------------------------------------

// Which walls an op applies to. A rule reads "outset the two longest side
// edges by 2 m" rather than hard-coding vertex numbers — which is what lets one
// script run on plans of different vertex counts.
struct EdgeSelector {
    enum class By {
        All,
        Tag,          // every edge carrying `tag`
        Index,        // explicit indices (escape hatch; prefer the others)
        Longest,      // the `count` longest edges
        Shortest,     // the `count` shortest edges
        Facing,       // outward normal within `degrees` of `dir`
        LongerThan,   // every edge at least `length` long
    };
    By by = By::All;
    EdgeTag tag = EdgeTag::None;
    std::vector<int> indices;
    int count = 1;
    Vec2 dir{0, -1};
    Real degrees = 45;
    Real length = 0;

    static EdgeSelector all();
    static EdgeSelector byTag(EdgeTag t);
    static EdgeSelector longest(int n);
    static EdgeSelector shortest(int n);
    static EdgeSelector facing(const Vec2& d, Real deg = 45);
    static EdgeSelector longerThan(Real len);
    static EdgeSelector at(std::vector<int> idx);

    // Indices into `loop`, ascending. Never returns an out-of-range index.
    std::vector<std::size_t> select(const Loop2& loop) const;
};

// --- plan quality ----------------------------------------------------------

// The invariant, checked after EVERY op rather than once at the end (§17.5).
// The reason this matters more than it looks: a mass stack amplifies a base
// plan's defects up the whole building, so one bad vertex becomes a divot
// riding a shaft for sixty storeys. Catching it at the op that made it is the
// highest-leverage check in the system.
struct PlanQuality {
    Real minEdge = 2.2;        // shorter than this is not a wall
    Real minAngleDeg = 42;     // the knife-edge gate
    Real minNotch = 2.5;       // no slot narrower than a room
    Real minArea = 12.0;

    bool ok(const Shape2& shape, std::string* why = nullptr) const;
};

// --- the grammar -----------------------------------------------------------

enum class PlanOpKind {
    Noop,        // a DESIGNED outcome with a weight, not an accident (§17.5)
    Outset,      // push selected edges out: bays, bump-outs
    Inset,       // pull selected edges in: recesses
    Wing,        // a perpendicular limb off an edge
    Court,       // subtract a bite: U, L, donut
    Notch,       // a small rectangular bite (a light slot)
    Fillet,      // round selected corners with a true arc
    Chamfer,     // cut selected corners straight
    Bow,         // bow selected walls into arcs (a bowed shopfront)
};

struct PlanOp {
    PlanOpKind kind = PlanOpKind::Noop;
    EdgeSelector on;
    Real depth = 0;      // outset/inset/wing/court depth, fillet r, chamfer cut
    Real width = 0;      // wing/court/notch width along the edge (0 = whole)
    Real along = 0.5;    // where along the edge the limb sits, 0..1
    Real weight = 1.0;   // relative probability within its step
};

// A step is a WEIGHTED CHOICE among alternatives, and `Noop` competes on equal
// terms. This is what stops the "every building has every feature" failure mode
// — the same disease as everything twisting, one level down.
struct PlanStep {
    std::vector<PlanOp> choices;
};

struct PlanScript {
    std::vector<PlanStep> steps;
};

// Apply one op. Returns the input unchanged when the op would produce nothing
// usable — ops are total functions, so a script never has to guard.
Shape2 applyPlanOp(const Shape2& shape, const PlanOp& op);

// Run a script. After every op the result is checked against `quality` and the
// op is REVERTED if it fails, so a plan is never accepted in a broken state.
// `envelope`, when given, clips after every op — containment as a guarantee
// rather than the progressive-inset search it replaces.
Shape2 runPlanScript(const Shape2& seed, const PlanScript& script,
                     std::uint32_t rngSeed, const PlanQuality& quality,
                     const Shape2* envelope = nullptr);

// Clip to an envelope, keeping the largest surviving region. Always safe.
Shape2 clipToEnvelope(const Shape2& shape, const Shape2& envelope);

// --- named templates -------------------------------------------------------

// The plan shapes the recipes name. Every one of these is a short script in the
// grammar above, fitted to a w x d frame — adding one is data, not code.
enum class PlanTemplate {
    Bar,               // the plain rectangle
    L,
    U,
    H,
    T,
    Courtyard,         // a ring with a real hole
    Flatiron,          // the wedge at an acute corner
    Cruciform,
    ChamferedSquare,   // the corner-entrance cut
    CurvedCornerSlab,  // a slab with one filleted street corner
    Pinwheel,
    Octagon,
    BowFront,          // a bar with a bowed street wall
};

// Build a template inside a w x d frame whose lower-left corner is at the
// origin. `seed` varies proportions within the template's character.
Shape2 planTemplate(PlanTemplate t, Real w, Real d, std::uint32_t seed = 1);

const char* planTemplateName(PlanTemplate t);

// Every template, for tests and for the Lot Lab's sheets.
std::vector<PlanTemplate> allPlanTemplates();

}  // namespace engine

#endif
