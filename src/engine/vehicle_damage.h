#ifndef RAYTRACER_ENGINE_VEHICLE_DAMAGE_H
#define RAYTRACER_ENGINE_VEHICLE_DAMAGE_H

// CAR DAMAGE (ADR-0353; Glenn, 2026-11-06: "It would be interesting if you could damage the car and bits and pieces
// can fall off until it was undrivable", "the contact point might be a place where we could play some sparks"). The
// rules, pure, so the tests can hold them:
//
//   zone      which part of the car a hit lands on (chassis frame: +Z the nose, +X the driver's left), by the
//             contact's place against the body's half-extents -- the nose carries the engine
//   harm      how much a touch costs: a knock by its closing speed above a soft threshold, a scrape by how fast it
//             slides, both shared out by the car's mass (a bus shrugs off what folds a hatchback)
//   dent      the body's own vertices pushed in round the hit, deepest at the point, smooth to nothing at the radius
//   sparks    a hard knock, or metal sliding past fast enough, throws them

#include "../rt_math.h"
#include "../renderer/renderer.h"   // RenderMesh

namespace engine {

enum class DamageZone { Front = 0, Rear, Left, Right, Roof, Under, Count };
const char* damageZoneName(DamageZone z);

// The zone `local` (chassis frame) falls in, against the body's half-extents.
DamageZone damageZone(const Vec3& local, const Vec3& halfExtent);

struct DamageRule {
    Real knockFrom = 3.0;      // m/s closing speed below which a knock leaves no mark
    Real knockFull = 28.0;     // ...and at which one knock spends a whole zone
    Real scrapePerMetre = 0.004;   // health a scrape takes per metre slid (at the reference mass)
    Real refMass = 1500.0;     // kg: heavier shares a hit out thinner
    Real sparkKnock = 4.0;     // m/s: a knock this hard sparks
    Real sparkSlide = 2.5;     // m/s: a scrape this fast sparks
};

// Health (0..1 of a zone) a touch costs: a new contact by `speed`, a persisting one by `slide` m/s for `dt` seconds.
Real touchHarm(const DamageRule& r, bool persisted, Real speed, Real slide, Real dt, Real mass);
// Sparks for a touch: how many (0 = none).
int touchSparks(const DamageRule& r, bool persisted, Real speed, Real slide);

// Push `mesh`'s vertices within `radius` of `point` along `inward` (unit) by up to `depth`, a smoothstep falloff;
// normals are recomputed when anything moved. True when a vertex moved.
bool dentMesh(RenderMesh& mesh, const Vec3& point, const Vec3& inward, Real radius, Real depth);

}  // namespace engine

#endif
