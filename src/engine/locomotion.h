#ifndef RAYTRACER_ENGINE_LOCOMOTION_H
#define RAYTRACER_ENGINE_LOCOMOTION_H

// HOW THE PLAYER GETS ABOUT, past walking (Glenn, 2026-10-24: "jump sometimes gets me stuck on geometry", "pull
// myself up over that ledge", "climb the rocks ... like breath of the wild", plus a movement lab to tune it in).
// The pieces PlayerSystem strings together, each one testable against a real PhysicsWorld (tests/test_locomotion):
//   - JumpGate: when a press may launch -- a moment after running off an edge (coyote time), a press just before
//     landing (the buffer), off steep ground, out of a wedge. The physics jump refused all of those.
//   - findLedge: a ledge in front of the body within reach, its top clear to stand on.
//   - MantleMove: the pull-up onto it, as a path the capsule follows (rise, then over).
//   - climb probes: the wall in front, whether it can be held, the corners round it.
// MovementTuning holds every number, live in Debug > Movement and kept in settings.json as movement.*.

#include "../rt_math.h"
#include "physics/physics_world.h"

#include <vector>

namespace engine {

class Settings;

struct MovementTuning {
    // walking
    Real walkSpeed = 6.0;          // m/s (PlayerSystem::moveSpeed until now)
    Real runScale = 1.9;           // Shift / L3
    // jumping
    Real jumpSpeed = 4.3;          // m/s up: ~0.95 m apex (the jump as it was)
    Real coyoteTime = 0.12;        // s after leaving walkable ground a press still jumps
    Real jumpBuffer = 0.15;        // s a press made in the air waits for the landing
    Real steepPush = 2.5;          // m/s away from steep ground when jumping off it
    Real wedgedTime = 0.15;        // s touching but unsupported, not falling, before it counts as standing
    Real airAccel = 4.0;           // m/s^2 the keys steer in the air
    // mantling
    Real mantleReach = 0.7;        // m past the capsule's front a wall counts as "in front"
    Real mantleMinHeight = 0.5;    // m above the feet: lower is a step, the controller walks it
    Real mantleMaxHeight = 2.1;    // m above the feet, hands at full stretch over the head
    Real groundMantleMax = 1.45;   // m: from standing, jump at a ledge this high and you pull straight up
    Real mantleRiseSpeed = 3.2;    // m/s while pulling up
    Real mantleOverTime = 0.2;     // s to roll over the edge onto the top
    // climbing
    Real climbSpeed = 1.3;         // m/s along the wall
    Real climbRunScale = 1.6;      // run held: quicker, costs double
    Real climbSteepest = 0.64;     // a surface is climbable when its normal's y is below this (steeper than walkable)
    Real climbOverhang = -0.35;    // ...and above this (a slight overhang still holds)
    Real staminaMax = 12.0;        // s of climbing
    Real staminaRegen = 4.0;       // per s, on the ground
    Real climbIdleDrain = 0.15;    // per s, hanging still
    Real climbJumpCost = 2.0;
    Real climbLungeSpeed = 4.0;    // m/s: a jump on the wall lunges this fast...
    Real climbLungeTime = 0.3;     // ...for this long
    Real wallKick = 4.5;           // m/s away from the wall when jumping off it backwards
    bool grabInAir = true;         // in the air, pushing toward a climbable wall grabs it
    bool grabFromGround = false;   // on the ground, walking into one grabs it too (else: jump at it)
    // swimming
    Real swimSpeed = 2.2;          // m/s, the crawl's average
    Real swimRunScale = 1.6;
    Real strokePeriod = 1.15;      // s per arm stroke
    Real strokeSurge = 0.55;       // speed swing over a stroke (0 = steady glide)
    Real buoyancy = 10.0;          // 1/s^2: the spring back to floating height (a plunge comes back up)
    Real waterDamping = 2.0;       // 1/s: how soon the bob settles (low = bobs longer)
    Real waterDrag = 0.2;          // 1/m: drag on a fast plunge (v*|v|): how deep a dive off the board goes

    struct Field { const char* key; Real MovementTuning::*value; Real lo, hi; const char* group; };
    static const std::vector<Field>& fields();
    struct Flag { const char* key; bool MovementTuning::*value; const char* group; };
    static const std::vector<Flag>& flags();
    void load(const Settings& s);
    void save(Settings& s) const;
};

// JUMP GATE. observe() once per fixed step after the move; wantJump() on the press; take() before the move says
// whether to launch this step (and, off steep ground, which way to push).
struct JumpGate {
    Real sinceGround = 1e9;    // s since last on walkable ground
    Real sinceLaunch = 1e9;    // s since the last launch: no second coyote jump
    Real buffered = -1.0;      // s left on a press waiting to be used
    Real wedged = 0.0;         // s touching non-walkable ground without falling

    void wantJump(const MovementTuning& t) { buffered = t.jumpBuffer; }
    void observe(GroundState g, Real verticalSpeed, Real dt);
    struct Launch { bool go = false; Vec3 push; };   // push: horizontal, added to the take-off velocity
    Launch take(GroundState g, const Vec3& groundNormal, const MovementTuning& t, Real dt);
};

// A LEDGE in front of the body: where its top edge is, which way the wall faces, and where the body stands
// once on top.
struct Ledge {
    Vec3 edge;         // the top edge, straight in front of the body (world)
    Vec3 normal;       // the wall's horizontal normal, toward the body
    Real topY = 0;     // the top surface
    Real height = 0;   // topY above the feet
    Vec3 rise;         // capsule centre pulled up level with the top, still in front of the wall
    Vec3 stand;        // capsule centre standing on the top
};
// Body: capsule centre, half-height, radius; facing: horizontal unit. maxHeight: the highest top accepted above
// the feet. False when there is no wall in reach, no top in range, or the top or the way up has no room.
bool findLedge(const PhysicsWorld& world, CharacterId id, const Vec3& centre, Real halfHeight, Real radius,
               const Vec3& facing, Real maxHeight, const MovementTuning& t, Ledge& out);

// THE PULL-UP: the capsule centre's path from where it caught the ledge, up level with the top, then over.
struct MantleMove {
    Vec3 from, rise, stand;
    Ledge ledge;
    Real t = 0, riseTime = 0, overTime = 0;
    void start(const Vec3& centre, const Ledge& l, const MovementTuning& tune);
    Vec3 at(Real time) const;   // the centre at `time` into the move
    Real total() const { return riseTime + overTime; }
    bool done() const { return t >= total(); }
    Real progress() const { return total() > 0 ? std::min(t / total(), Real(1)) : Real(1); }
};

// THE WALL IN FRONT for climbing: a ray from the capsule centre along `dir`. Climbable: steep (not walkable),
// not too far over, and not a simulated body.
struct WallHit {
    bool hit = false;
    bool climbable = false;
    Vec3 point, normal;
    Real distance = 0;
};
WallHit probeWall(const PhysicsWorld& world, const Vec3& from, const Vec3& dir, Real length,
                  const MovementTuning& t);
// The basis on a wall facing `normal`: right along it (level), up along it (the steepest way up).
void wallBasis(const Vec3& normal, Vec3& right, Vec3& up);

// BUOYANCY: one step of the swimmer's vertical speed. A body that falls in keeps its speed and is slowed by the
// water (drag on a fast plunge, damping on a slow one) while the buoyant spring pulls it back to floatY -- so it
// goes under, comes up past the float line, and bobs to rest. rise/dive (jump/crouch held) push against it.
Real swimVerticalStep(Real vy, Real centreY, Real floatY, bool rise, bool dive, Real dt, const MovementTuning& t);

// The swimmer's speed at a point in the stroke (phase 0..1): a pull that surges, a glide that eases off; the
// average over a stroke is 1.
Real strokeSpeedScale(Real phase, Real surge);
// Where in strokeSpeedScale's cycle a swimmer is, `strokes` into the crawl: two pulls a stroke (one per arm,
// PlayerArmsSystem's crawl pulls at 0.53 and 0.03), each the surge.
Real strokeSurgePhase(Real strokes);

}  // namespace engine

#endif
