#ifndef RAYTRACER_APPS_CITYSIM_CITY_POSES_H
#define RAYTRACER_APPS_CITYSIM_CITY_POSES_H

// WHERE THE CITY'S BODIES ARE, EACH FIXED STEP (ADR-0454): the sim's side of a body's pose, not the drawing's.
//
// A car entity's body this step (city_car.h CityCarPose) is set here once every fixed step, after the sim's step, and
// read by its kinematic box (city_physics.cpp), the near tier's switch into physics (city_near.cpp), its seats and
// the drawing (city_render.cpp), which blends one bake to the next for smooth frames and owns nothing of the pose.
// A walker standing or walking is posed here too: their kinematic box where the city owns its walkers, and the walkers'
// drawn group, which calls walkerStandingPose.
//
// What a pose stands on and is fitted with lives here with it, given once when the city is built:
//   - the ground (CityGround): the road decks the mesher drew and the terrain under them;
//   - the drawn car models' body facts (CityCarModels): each model's wheels, and which models the buses, the fire
//     engines and the cabs are drawn as;
//   - each car's four-wheel fit memory (the tilt low-pass and its last pose).
// The drawing reads these for what it draws; it never keeps its own.

#include "activities.h"   // Ride
#include "city_car.h"     // CityCarPose, CityCarOnLane
#include "../../engine/clock.h"        // kDefaultFixedStep
#include "../../engine/components.h"
#include "../../engine/procgen/city/road_mesh.h"   // RoadDeckField
#include "../../engine/world.h"
#include "../../rt_math.h"

#include <cstdint>
#include <functional>
#include <unordered_map>
#include <utility>
#include <vector>

namespace citysim {

struct Agent;
class CitySim;
using engine::Mat4;
using engine::Vec3;

// One wheel of a drawn car model, in body-local space (+Z forward, x>0 the driver's left): a mirror of the
// scripting-only engine::WheelPlacement, so the non-scripting build still compiles.
struct CarWheel {
    Vec3 pos;
    Real radius = 0.3;
    Real width = 0.2;
    bool steered = false;
    bool driven = true;
    bool handBrake = false;
};

// THE GROUND A BODY STANDS ON.
struct CityGround {
    std::vector<engine::RoadDeckField> decks;           // the road decks the mesher drew (RoadDeck)
    std::function<double(double, double)> terrain;      // the terrain under them (may be null)
    Real roadLift = 0;                                  // the level's road lift over its terrain
    Real sidewalk = 3.5;                                // how far past a carriageway a deck still counts (m)
    // ON a road: the deck the mesher drew; off it, the terrain and the lift.
    Real groundAt(Real x, Real z) const;
    // The drawn deck nearest `refY` at (x, z), within a few metres; else refY.
    Real deckSurfaceNear(Real x, Real z, Real refY) const;
};

// THE DRAWN CAR MODELS' BODY FACTS (the fleet's drawn models, one per variant).
struct CityCarModels {
    std::vector<std::vector<CarWheel>> wheels;   // per model
    int groups = 0;                              // how many models are drawn (0 before the city is built)
    int bus = -1, fire = -1, taxi = -1;          // the models a bus, a fire engine, a cab are drawn as (-1: none)
    int count() const { return groups > 0 ? groups : 1; }   // (1 with none, so the modulo stays defined)
    // The model a car of fleet slot `slot` and body `type` is drawn as when nobody is driving it.
    int variantOf(int slot, VehicleType type) const;
};

// ONE BODY'S FOUR-WHEEL FIT MEMORY: the tilt low-pass and its last pose (a body that has not moved sits where it sat).
struct FitMemory {
    struct Pose { Real x = 0, z = 0, hx = 0, hz = 0; int slot = -1; bool valid = false; Mat4 m; } pose;
    Vec3 smoothUp{0, 0, 0};
    uint64_t step = 0;   // the step it was last posed at (a car entity's: kept only while it is posed)
    // A CAR STANDING OFF THE ROAD (ADR-0519): the pose it was given, and the place, turn and body it was given it for,
    // and the step; while none of them changes it keeps it (asked of the ground again once every kParkedReposeSteps)
    struct Parked { Vec3 at; Real qx = 0, qy = 0, qz = 0, qw = 0; int variant = -1; uint64_t step = 0; bool valid = false; } parked;
};

class CityPoses {
public:
    CityGround ground;
    CityCarModels models;

    // A NEW CITY: the fit memories go (their cars are the old city's).
    void reset() {
        carMemory_.clear();
        standingWalkers_.clear();
    }

    // THE TILT LOW-PASS: `su`, the last smoothed up-vector, blended toward this step's `up`, with a gain from the step
    // (tau ~0.7 s), so the filter behaves the same at the viewer's 60 Hz and the tests' 10 Hz.
    static Vec3 smoothUp(Vec3& su, Vec3 up, Real dt);
    // FOUR WHEELS ON THE ROAD: a car's body, its middle at (x, z) on the plan and pointing along `heading`, model
    // `slot`'s wheels on the surface under each (`surface(x, z, metres ahead of the middle)`), a plane fitted through
    // them for pitch and roll, at the height that puts the LOWEST tyre on the surface. `lineWheels`: wheels in one line
    // are enough (a two-wheeler: pitch only). `memory` (null: none): the tilt low-pass, stepped by `dt`, and the last
    // pose. False: the model has too few wheels, or no heading.
    bool bodyOnWheels(int slot, bool lineWheels, Real x, Real z, engine::Vec2 heading, FitMemory* memory, Real dt,
                      const std::function<Real(Real, Real, Real)>& surface, Mat4& out) const;
    // A car standing at rest (a parked row, or a car entity off the road): level, its middle at `pos` on the ground
    // under it or up a garage's deck (`parkedY` above -1e29), half its model's height up.
    Mat4 parkedCarPose(int variant, engine::Vec2 pos, engine::Vec2 heading, Real parkedY,
                       const std::vector<Vec3>& halfExtents) const;
    // Each drawn model's half extents (its fleet slot's body), by model.
    std::vector<Vec3> halfExtents(const CitySim& sim) const;
    // A CAR ENTITY'S BODY: between where it was as the sim's tick began (PrevTransform) and where it is (a placement
    // snaps, never slides); while the driving has it (`lane`), on its wheels on the deck nearest its lane height, else
    // level on the ground. `roadRef`: the lane height it is stood by (0 off the road).
    Mat4 carEntityPose(const CitySim& sim, const CityCarOnLane* lane, const engine::Transform& t,
                       const engine::Transform& prev, int variant, const std::vector<Vec3>& he, FitMemory* memory,
                       Real dt, Real* roadRef) const;

    // EVERY CAR ENTITY'S CityCarPose, once a fixed step after the sim's step (`dt`: the step).
    void stepCarPoses(engine::World& world, const CitySim& sim, Real dt);
    // ...one car's, by the one rule (none while it is away, a Jolt car or nowhere): for a car made between steps.
    void poseCarNow(engine::World& world, engine::Entity car, const CitySim& sim);

    // A WALKER'S BODY this step: half `height` up over the ground or their deck at their place (blended across the
    // sim's tick), turned the way they go.
    Mat4 walkerPose(const CitySim& sim, const Agent& a, Real height) const;
    // ...standing or walking, as the walkers' group draws them and their box stands: on their board's deck if they
    // ride one, scaled about their feet to their height (people.lua). `feet`: where their feet are before the board.
    Mat4 walkerStandingPose(const CitySim& sim, int ai, Real height, Real* feet) const;
    // EVERY WALKER STANDING OR WALKING (not sitting, lying, indoors or riding), by agent index: their kinematic boxes'
    // poses this step, for a host where the city owns its walkers.
    void walkerBoxPoses(const CitySim& sim, Real height, std::vector<std::pair<int, Mat4>>& out) const;
    // The deck a rider stands on (activities.lua `ride`): a skateboard's or a scooter's, above the ground (m).
    static Real rideDeck(Ride r);

private:
    void poseCar(engine::World& world, engine::Entity e, const CitySim& sim, const std::vector<Vec3>& he, Real dt);
    // EACH CAR ENTITY'S FIT MEMORY, by its number, kept while the car is posed: a car that loses its pose (a Jolt car,
    // the player's, nowhere, gone) loses its memory with it and starts afresh when it is posed again
    std::unordered_map<uint32_t, FitMemory> carMemory_;
    // A WALKER STANDING STILL (ADR-0532), by agent index: their standing pose and what it was worked out from (place,
    // heading, deck, elevation, height, board, size, whether it is blended across the tick). While none of it changes the
    // pose is theirs again, the ground under them asked once every kStillReposeSteps (each on their own step).
    struct StillWalker {
        Vec2 pos, heading;
        Real deckY = 0, elevation = 0, height = 0, scale = 0;
        uint8_t ride = 0;
        bool blended = false, valid = false;
        uint64_t step = 0;
        Mat4 pose;
        Real feet = 0;
    };
    mutable std::vector<StillWalker> standingWalkers_;
    uint64_t steps_ = 0;                                 // fixed steps posed (FitMemory::step)
    Real lastDt_ = engine::kDefaultFixedStep;            // the last step's (poseCarNow between steps)
};

}  // namespace citysim

#endif
