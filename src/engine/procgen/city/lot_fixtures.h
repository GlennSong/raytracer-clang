#ifndef RAYTRACER_ENGINE_PROCGEN_CITY_LOT_FIXTURES_H
#define RAYTRACER_ENGINE_PROCGEN_CITY_LOT_FIXTURES_H

#include "site_plan.h"
#include "building_recipe.h"
#include <cstdint>
#include <string>
#include <vector>

namespace engine {

// Instancing, interactivity and lighting for a lot (docs/lot-system-plan.md
// §9, §10; plan phases P6-P8).
//
// WHAT IS AND IS NOT BUILDABLE HERE, stated plainly. The Jolt submodule cannot
// be fetched in this environment, and props do not yet reach the renderer as
// instances, so this module deliberately stops at the SPECIFICATION boundary:
// it decides what interactive objects a lot has, where they are, which way
// they swing, and which lights should be lit — all pure data, all testable
// headless. Turning a HingeSpec into a Jolt constraint, an InteractableSpec
// into an input prompt, and an InstanceBatch into an InstanceGroup is engine
// wiring that must be verified on device.
//
// That split is the honest one: the decisions are the hard part and they are
// finished; the bridges are mechanical and unverifiable from here.

// --- instancing (P6) -------------------------------------------------------

// Props grouped by kind, ready to become an InstanceGroup (ADR-0041 Phase 2).
// Today the city merges everything into one mesh per material class, which is
// exactly why there is nothing to attach behaviour to — an instance batch is
// the prerequisite for a prop being an entity at all.
struct InstanceBatch {
    PropKind kind = PropKind::Shrub;
    struct Xform {
        Vec2 at;
        Real yaw = 0;
        Real scale = 1;
    };
    std::vector<Xform> instances;
};

std::vector<InstanceBatch> batchProps(const std::vector<PropInstance>& props);

// --- interactivity (P7) ----------------------------------------------------

enum class InteractKind : std::uint8_t { Gate, Door, Seat, Switch, Bin };

const char* interactKindName(InteractKind k);

// A hinge, fully specified. Derived from the geometry that demanded the object
// — a GateSpec already knows its post and its swing — so nothing downstream
// re-derives an axis and gets it subtly wrong.
struct HingeSpec {
    Vec3 anchor;             // the hinge post, world space
    Vec3 axis{0, 1, 0};      // vertical for a gate
    Real minAngle = 0;
    Real maxAngle = 1.6;
    Real friction = 0.4;
    bool motor = false;      // Tier B: powered (a shop door); false = free swing
};

// The interactive object itself. `tierA` is a scripted kinematic swing that
// works with today's physics wrapper; `tierB` needs the hinge constraint the
// Jolt bridge does not expose yet. Recording both means the same spec upgrades
// without touching the recipes.
struct InteractableSpec {
    InteractKind kind = InteractKind::Gate;
    Vec3 at;
    Real yaw = 0;
    Real width = 1.2, height = 1.3;
    HingeSpec hinge;
    bool tierBCapable = false;
    std::string prompt;      // "Open gate"
    Real reach = 1.8;        // how close the player must be
};

// A trigger volume. The general primitive — useful far beyond gates — kept as
// data so the Jolt sensor wrapper has something to build from when it lands.
struct SensorSpec {
    Vec3 centre;
    Vec3 halfExtent{1, 1, 1};
    Real yaw = 0;
    int interactableIndex = -1;   // which object this sensor arms
};

// A seat: an attach point with a pose. The MARKER is cheap and lives here; the
// pose itself belongs to character-posing-plan.md and is the real work.
struct SeatSpec {
    Vec3 at;
    Real yaw = 0;
    int propIndex = -1;
};

// --- lighting (P8) ---------------------------------------------------------

enum class FixtureKind : std::uint8_t {
    PorchLight, PathLight, GateLamp, WindowGlow, SignFace, StreetLamp,
};

const char* fixtureKindName(FixtureKind k);

// A light fixture: emissive geometry PLUS an optional real light. Tier 1 of the
// plan's lighting ladder works immediately from the emissive alone — no light
// budget consumed — and tiers 2-3 upgrade the same fixture without the recipes
// changing.
struct LightSpec {
    FixtureKind kind = FixtureKind::PorchLight;
    Vec3 at;
    Vec3 color{1.0, 0.88, 0.7};
    Real intensity = 1.0;
    Real radius = 6.0;
    bool emissiveOnly = false;   // true = decoration, never takes a light slot
};

// THE HARD WALL: RT_MAX_LIGHTS is 32, forward-lit, and the street-lamp system
// already spends 14 on its own nearest-N selection. A city where every lot has
// a porch light, two path lights and a gate lamp is THOUSANDS of lights, so
// adding lot lighting without a shared budget does not dim the city — it
// starves the street lamps.
//
// LightBudget is the one mechanism that replaces one-selection-per-system: it
// ranks every candidate (street, lot, window, vehicle) by the same score and
// fills the slots once.
struct LightBudget {
    int slots = 32;
    Real reserveForStreet = 0.4;   // fraction of slots street lamps may not lose

    // Score = intensity / (1 + distance^2), the standard falloff-weighted
    // ranking, so a bright distant lamp can still beat a dim near one.
    static Real score(const LightSpec& l, const Vec3& viewer);

    // Choose which lights are lit this frame. Returns indices into `lights`,
    // best first. Emissive-only fixtures are never selected — they cost
    // nothing and are always "on".
    std::vector<int> select(const std::vector<LightSpec>& lights,
                            const Vec3& viewer) const;
};

// --- assembling a lot's fixtures -------------------------------------------

// Everything a laid-out, furnished lot contributes beyond geometry.
struct LotFixtures {
    std::vector<InstanceBatch> batches;
    std::vector<InteractableSpec> interactables;
    std::vector<SensorSpec> sensors;
    std::vector<SeatSpec> seats;
    std::vector<LightSpec> lights;
};

// Derive them from the site plan, its furnishing and the built building.
// Deterministic. Every gate in the site plan becomes an interactable with a
// hinge and a sensor; entrances become doors; benches become seats; the
// program decides which lights a lot carries.
LotFixtures fixturesFor(const SitePlan& site, const SiteFurnishing& furnishing,
                        const LotProgram& program, const BuiltBuilding& building,
                        Real groundY, std::uint32_t seed);

}  // namespace engine

#endif
