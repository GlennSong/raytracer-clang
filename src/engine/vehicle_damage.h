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

// A CAR'S DAMAGE TUNING (ADR-0355: data -- vehicles.lua KIT_DAMAGE, read per car into its spec as `damage`; these
// defaults are the table's `default`, kept the same so a car without one behaves as the tests expect).
struct DamageRule {
    Real knockFrom = 3.0;      // m/s closing speed below which a knock leaves no mark
    Real knockFull = 40.0;     // ...and at which one knock spends a whole zone (145 km/h: a wreck, not a fence)
    Real scrapePerMetre = 0.004;   // health a scrape takes per metre slid (at the reference mass)
    Real refMass = 1500.0;     // kg: heavier shares a hit out thinner
    Real sparkKnock = 4.0;     // m/s: a knock this hard sparks
    Real sparkSlide = 2.5;     // m/s: a scrape this fast sparks
    Real crashWindow = 0.33;   // s: knocks on one zone this close together are one crash (the worst counts)
    Real groundShare = 0.33;   // a knock from below (a hard landing) costs this share
    Real breakShare = 0.10;    // knocking a street piece loose costs this share
    Real zoneScale = 1.2, panelScale = 1.6;   // a knock's cost to the zone it lands on, and to the panel there
    Real dentFrom = 0.01;      // a knock costing less leaves no dent
    Real dentRadiusLo = 0.25, dentRadiusHi = 0.90;   // m: a ding to a crumple
    Real dentDepthLo = 0.04, dentDepthPer = 0.35, dentDepthMax = 0.26;   // m: depth = lo + per x cost, at most max
    Real scuffLo = 0.35, scuffPer = 1.2;   // the paint scraped toward bare metal
    Real steamBelow = 0.6, blackBelow = 0.25;   // the nose's health the hood steams below, then smokes black
    Real kickCar = 0.7, kickOut = 0.8, kickUp = 0.8;   // a panel coming off: of the car's speed, out, up (m/s)
    Real roughBelow = 0.6, roughWobble = 0.07, roughMisfire = 0.5;   // the engine's sound (vehicle_audio.h engineRough)
    Real crackFrom = 0.04, smashFrom = 0.30;   // a knock's cost that crazes the glass by it / smashes a hole in it
};

// Health (0..1 of a zone) a touch costs: a new contact by `speed`, a persisting one by `slide` m/s for `dt` seconds.
Real touchHarm(const DamageRule& r, bool persisted, Real speed, Real slide, Real dt, Real mass);
// Sparks for a touch: how many (0 = none).
int touchSparks(const DamageRule& r, bool persisted, Real speed, Real slide);

// Push `mesh`'s vertices within `radius` of `point` along `inward` (unit) by up to `depth`, a smoothstep falloff;
// normals are recomputed when anything moved. True when a vertex moved. `scuff` (0..1) scrapes the paint toward bare,
// dark metal as deep as the dent goes (Glenn: "show the visual damage ... look damaged"), and a lamp lens crushed in
// past half the dent's depth is broken: its lamp tag (u <= -1000, ADR-0342) goes and it darkens -- it no longer lights.
bool dentMesh(RenderMesh& mesh, const Vec3& point, const Vec3& inward, Real radius, Real depth, Real scuff = 0);

// GLASS (ADR-0356; owed in ADR-0355): a knock crazes the glass round the point -- its tint whitened toward the frosted
// look of a cracked pane by `strength` (0..1), smooth to nothing at `radius` -- and a hard one smashes a hole: every
// triangle whose middle lies within `radius` goes. The windows are one mesh, so a hole is the pane that was hit, not
// every window. True when the mesh changed; `smashed` counts the triangles taken out.
bool crackGlass(RenderMesh& mesh, const Vec3& point, Real radius, Real strength);
int smashGlass(RenderMesh& mesh, const Vec3& point, Real radius);

}  // namespace engine

#endif
