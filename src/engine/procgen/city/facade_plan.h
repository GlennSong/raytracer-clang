#ifndef RAYTRACER_ENGINE_PROCGEN_CITY_FACADE_PLAN_H
#define RAYTRACER_ENGINE_PROCGEN_CITY_FACADE_PLAN_H

#include "plan_grammar.h"
#include "mass_stack.h"
#include <cstdint>
#include <string>
#include <vector>

namespace engine {

// The bay grid and the element registry (docs/lot-system-plan.md §6, layer L1).
//
// This is the layer that turns "which features does this building have" from a
// struct of 18 booleans into DATA. Today a facade's bay spacing is computed
// privately inside the emitter, so nothing else can know where the bays are —
// which is why balconies cannot line up with windows, pilasters land through
// openings instead of between them, and a shopfront cannot tile. Publishing the
// grid once, and addressing everything through selectors over it, is the
// prerequisite for all of those and for interiors later.

// --- the bay grid ----------------------------------------------------------

// A wall's rhythm, computed ONCE per plan edge and shared by every consumer.
// `bays` is a whole number so the rhythm closes on the wall exactly; `bayWidth`
// is the actual spacing after that rounding, not the requested one.
struct BayGrid {
    Real wallLength = 0;
    Real bayWidth = 0;
    int bays = 0;

    // The span of bay `i` along the wall, as parameters in [0,1].
    void bayspan(int i, Real& t0, Real& t1) const;
    Real bayCentre(int i) const;      // parameter of bay i's centre
    int bayAt(Real t) const;          // which bay a parameter falls in
    int centreBay() const;            // the middle bay (the doorway bay)
};

// `preferred` is the architectural intent (a shopfront wants ~4 m, a house ~3);
// the grid rounds to whole bays and reports what it actually got.
BayGrid bayGridFor(Real wallLength, Real preferredBayWidth);

// --- openings --------------------------------------------------------------

enum class OpeningKind : uint8_t { Window, Door, Vent, Storefront, None };

// One hole in a wall: a span ALONG the wall and a sill/head height. This is the
// blueprint model — the plan owns where the openings are, and both the full and
// the flat LOD emitters read the SAME list, so the two can never disagree about
// where a building's windows are. (LOD1 already consumes this idea in
// shape_grammar's facadeLayout; this is its data-shaped home.)
struct Opening {
    Real t0 = 0, t1 = 0;      // along the wall, 0..1
    Real sill = 0, head = 0;  // metres above the floor's own slab
    OpeningKind kind = OpeningKind::Window;
    int bay = -1;             // which bay it belongs to, or -1
};

// How a wall is glazed. The "not every wall needs all the windows" axis: a
// party wall is Blank by construction, a rear wall is Sparse, a shopfront is
// Storefront.
enum class WallRole : uint8_t {
    Blank,        // party walls: no openings at all
    Sparse,       // rear/service: a few small openings
    Regular,      // the ordinary punched facade
    Storefront,   // ground-floor retail: wide glazing + a door
    Curtain,      // full glazing, banded
};

// The style knobs a wall's openings are generated from. Deliberately small —
// anything richer belongs in a recipe, not in the generator.
struct FenestrationStyle {
    Real preferredBay = 3.4;
    Real windowWidthFrac = 0.55;   // of the bay
    Real sill = 0.95;
    Real head = 2.35;
    Real doorWidth = 1.2;
    Real doorHead = 2.15;
    bool corniceBand = false;
};

// The openings for ONE wall on ONE floor. `role` and `floor` decide the mix;
// `entrance` requests that this wall/floor carry the building's door.
std::vector<Opening> fenestrate(const BayGrid& grid, WallRole role,
                                const FenestrationStyle& style, int floor,
                                bool entrance);

// The default role for a wall, from the tag its plan edge carries. This is the
// payoff of tagging edges once at plan time: a party wall is blank BY
// CONSTRUCTION rather than because a dot product guessed right.
WallRole roleForTag(EdgeTag tag, int floor, bool retail);

// --- the element registry --------------------------------------------------

// Everything that can be attached to a building. The existing emitters
// (emitPortico, emitBalconyRun, emitPorch, emitBayFront, emitSteeple, the roof
// furniture) become registered kinds with NO change to their geometry code —
// only to how they are chosen.
enum class ElementKind : uint8_t {
    Opening, Balcony, Cornice, Quoin, Pilaster, BaseCourse, Awning, Portico,
    Porch, Steps, BayWindow, Parapet, Steeple, Spire, Dome, RoofDeck, Sign,
    Planter, Lamp,
    // "This roof carries plant" — the stair/lift overrun, the water tank, the
    // air handling. ONE kind, not three: which pieces a given roof gets and how
    // they are arranged so they never overlap is a design with its own rules
    // (roof_plant.h), and a recipe asking for "a bulkhead here, a tank there"
    // would be re-deciding in data what the planner decides properly. The
    // recipe says whether the building has plant at all; the planner says what.
    RoofPlant,
};

const char* elementKindName(ElementKind k);

// Which surfaces an element lands on. Selectors COMPOSE over the axes the
// geometry already has — edge tag, floor range, bay index, corner convexity,
// tier — so a rule reads "street-facing, floors 1 and up, on every bay
// boundary" instead of naming vertices and storeys explicitly.
struct ElementSelector {
    // Edge axis. `anyTag` means "do not filter by tag".
    bool anyTag = true;
    EdgeTag tag = EdgeTag::Street;

    // Floor axis: [floorMin, floorMax], inclusive; -1 = unbounded.
    int floorMin = -1, floorMax = -1;

    // CROWN elements — a parapet, a roof deck, the cornice that finishes a
    // tier — belong on the tier's LAST level, not on every level in it. The
    // tier axis below cannot say that: a 24-storey tower is one tier, so
    // `tier = kTopTier` matched all 24 levels and grew 24 parapets, one per
    // storey. They were invisible only while a parapet was swept as a
    // zero-thickness sheet with no inside; giving it real thickness put a wall
    // through every floor. Twenty-four times the geometry, too.
    bool topFloorOnly = false;

    // Bay axis.
    enum class Bays : uint8_t { All, Centre, Ends, Alternate, Boundaries, None };
    Bays bays = Bays::All;

    // Tier axis: -1 = any; -2 = the TOP tier (whatever index that is).
    int tier = -1;
    static constexpr int kTopTier = -2;

    // Corner axis, for quoins and chamfers.
    enum class Corners : uint8_t { None, Convex, Reflex, All };
    Corners corners = Corners::None;

    // Attach to another element rather than to the building (an awning on a
    // door). Empty means "attach to the building".
    ElementKind attachTo = ElementKind::Opening;
    bool attached = false;

    bool matchesEdge(EdgeTag t) const;
    bool matchesFloor(int floor) const;
    bool matchesTier(int tier, int topTier) const;
    bool matchesBay(int bay, const BayGrid& grid) const;
};

// A registered element: what it is, where it goes, and its parameters. The
// parameter bag is deliberately generic — an element's geometry code reads what
// it needs — because the alternative is a struct that grows a field per kind,
// which is the 18-bool problem again in a new costume.
struct ElementSpec {
    ElementKind kind = ElementKind::Opening;
    ElementSelector on;
    Real depth = 0;      // balcony projection, cornice reach, porch depth
    Real height = 0;
    Real width = 0;
    int style = 0;       // index into the recipe's style tables
    Real weight = 1.0;   // <1 makes an element occasional rather than universal
};

// A building's whole dressing, as data. This is what replaces the 18 booleans:
// adding a feature is appending a row, not adding a field and a branch.
struct ElementRegistry {
    std::vector<ElementSpec> elements;

    void add(ElementSpec s) { elements.push_back(std::move(s)); }
};

// Where the resolver decided an element goes. One placement per (element,
// surface) hit — the emitters consume these.
struct ElementPlacement {
    ElementKind kind = ElementKind::Opening;
    int level = 0;          // which Level of the mass stack
    int tier = 0;
    int edge = 0;           // which plan edge of that level
    EdgeTag tag = EdgeTag::None;
    int bay = -1;           // -1 for whole-wall elements
    Real t0 = 0, t1 = 1;    // span along the wall
    Real depth = 0, height = 0, width = 0;
    int style = 0;
    int specIndex = 0;      // which ElementSpec produced this
};

// The building the registry is resolved against: its levels, and each level's
// per-edge bay grids.
struct BuildingSurfaces {
    std::vector<Level> levels;
    // grids[level][edge]
    std::vector<std::vector<BayGrid>> grids;
    int topTier = 0;
};

// Build the surface description for a stack: one bay grid per plan edge per
// level, computed ONCE so every consumer agrees.
BuildingSurfaces surfacesFor(const MassStack& stack, Real preferredBay = 3.4);

// Resolve a registry against a building. Deterministic for `seed` — the weights
// are a stable per-surface hash, not a running dice roll, so adding an element
// to the registry does not reshuffle where the others landed.
std::vector<ElementPlacement> resolveElements(const ElementRegistry& reg,
                                              const BuildingSurfaces& surfaces,
                                              std::uint32_t seed);

// --- material sides (§17.7) ------------------------------------------------

// A surface gains a SIDE. Interiors stay out of scope, but a wall's inner ring
// already exists as geometry once walls have thickness, and recording which
// side a surface faces now is the difference between interiors being a new
// material binding later and being a rewrite later.
enum class Side : uint8_t { Exterior, Interior };

// A (part, side) pair, packed. The ordinal is what a material array indexes by,
// so exterior parts keep their existing indices exactly and interiors extend
// past them — nothing that reads today's PartId ordinal changes meaning.
struct SidedPart {
    std::uint8_t part = 0;
    Side side = Side::Exterior;

    int ordinal(int partCount) const {
        return static_cast<int>(part) +
               (side == Side::Interior ? partCount : 0);
    }
};

}  // namespace engine

#endif
