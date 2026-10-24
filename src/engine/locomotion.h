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
    // gliding (slot 5 the parachute, slot 6 the wingsuit; jump in the air opens them)
    Real deployMinHeight = 4.0;    // m of air under the feet needed to open either
    Real chuteOpenTime = 0.7;      // s from the pull to a full canopy
    Real chuteSpeed = 8.0;         // m/s ahead under a full canopy, hands up
    Real chuteSink = 4.5;          // m/s down, hands up
    Real chuteDiveSpeed = 12.0;    // W: toggles down a little, quicker and steeper
    Real chuteDiveSink = 7.0;
    Real chuteFlareSpeed = 4.0;    // S: a flare, slow and soft
    Real chuteFlareSink = 2.0;
    Real chuteTurnRate = 70.0;     // deg/s the canopy turns toward where you look
    Real chuteResponse = 1.6;      // 1/s: how soon the canopy settles to a new speed
    Real suitDrag = 0.0016;        // 1/m: v*|v| along the body: 48 m/s pitched 20 deg down, 60 at 35
    Real suitInducedDrag = 5.0;    // m: what lift costs (k * lift^2 / v^2), worst when slow: no hanging in the air
    Real suitStallSpeed = 14.0;    // m/s: slower than this the suit stops flying and you drop
    Real suitTurn = 3.5;           // 1/s: how quickly the suit swings the way into where you look
    Real suitPitchDown = 60.0;     // deg the dive is held to
    Real suitPitchUp = 25.0;       // deg the pull-up is held to
    // the grapple (slot 7): hold the trigger to hang on the rope, let go to fly on
    Real grappleRange = 60.0;      // m to the farthest hook
    Real grappleMinRange = 3.0;    // m: nearer than this is no swing
    Real grappleCone = 9.0;        // deg off the aim the hook still finds a place (the snap that makes it easy)
    Real grappleReelIn = 9.0;      // m/s the rope shortens, jump held
    Real grappleReelOut = 5.0;     // m/s it lets out, crouch held
    Real grappleMinRope = 2.0;     // m: reeled all the way in
    Real grapplePump = 4.0;        // m/s^2 along the swing, W held
    Real grappleSteer = 2.5;       // m/s^2 sideways, A/D
    // boards (slot 8 the skateboard, slot 9 the scooter): ridden whenever on the ground with one in hand
    Real boardPush = 1.8;          // m/s a kick adds (less as the speed nears the most a kick reaches)
    Real boardPushMax = 8.5;       // m/s: kicking stops helping here (downhill goes faster)
    Real scooterPushMax = 7.0;
    Real boardKickTime = 0.5;      // s between kicks, W held
    Real boardRoll = 0.3;          // m/s^2 rolling loss
    Real boardBrake = 5.0;         // m/s^2, S held (a heel drag)
    Real boardCarve = 7.0;         // m/s^2 sideways grip: how tight a turn is at speed
    Real boardTurnMax = 150.0;     // deg/s: how fast it turns when slow
    Real boardOllie = 3.8;         // m/s up off the board, jump
    Real boardCrash = 6.5;         // m/s into something that stops it: off and limp
    // the playground: a swing pumped like a pendulum, monkey bars hand over hand
    Real swingPump = 1.6;          // rad/s^2 a pump adds, W/S in time with the swing
    Real swingDamping = 0.08;      // 1/s: how soon an unpumped swing dies down
    Real swingMaxDeg = 75.0;       // the highest it goes (the chains go slack past this)
    Real barsSpeed = 1.1;          // m/s hand over hand
    // going limp (the ragdoll)
    Real limpFallSpeed = 15.0;     // m/s down at touchdown that drops you in a heap (an ~11 m fall)
    Real limpImpact = 14.0;        // m/s lost in one step on the rope or in the suit: a slam
    Real getUpAfter = 1.2;         // s lying still before standing up by itself

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

// GLIDING. One step of a canopy's velocity: it turns `heading` (horizontal unit) toward `look` at the turn rate and
// settles the velocity toward ahead-and-down speeds set by the toggles (inF: +1 dive, -1 flare). `open` 0..1 is
// how far the canopy is out (an opening one still mostly falls).
Vec3 parachuteStep(const Vec3& v, Vec3& heading, const Vec3& look, Real inF, Real open, Real dt,
                   const MovementTuning& t);
// One step of a wingsuit's velocity, flown by where you look (`look`, unit; its pitch held to the suit's range):
// gravity, lift that swings the way into the facing (fading out below the stall), and drag along it. Pitched
// down it settles at a glide (~2.7:1 at 20 deg); pulled up it trades speed for height until it stalls.
Vec3 wingsuitStep(const Vec3& v, const Vec3& look, Real dt, const MovementTuning& t);
// The look direction with its pitch held to [-suitPitchDown, suitPitchUp].
Vec3 wingsuitFacing(const Vec3& look, const MovementTuning& t);

// THE GRAPPLE'S HOOK: where a shot along `look` from `eye` would catch -- the aim itself if it meets something
// solid in range, else the best of a ring of rays within the cone, preferring near the aim and above the eye.
struct GrappleAnchor {
    bool ok = false;
    Vec3 point, normal;
    Real distance = 0;
};
GrappleAnchor findGrappleAnchor(const PhysicsWorld& world, const Vec3& eye, const Vec3& look, const MovementTuning& t);
// ONE STEP ON THE ROPE: the body point `pos` moves under gravity and `accel` (pumping, steering); where that would
// take it further from `anchor` than `length`, it is held on the rope's sphere and the outward speed is taken out
// (a rope pulls, it never pushes). Returns the new velocity; `pos` is updated.
Vec3 ropeStep(Vec3& pos, const Vec3& vel, const Vec3& anchor, Real length, const Vec3& accel, Real dt);

// ONE STEP ON A BOARD: `heading` (horizontal unit) turns with A/D at the tighter of the turn rate and what the
// grip allows at this speed; W kicks (one kick per boardKickTime, up to pushMax), S brakes; the slope under the
// wheels (`groundNormal`) pulls it downhill; rolling loss slows it. `kickClock` counts down to the next kick.
// Returns the new speed (>= 0) along the new heading.
Real boardStep(Real speed, Vec3& heading, Real& kickClock, Real inF, Real inR, const Vec3& groundNormal,
               Real pushMax, Real dt, const MovementTuning& t);

// A SWING: one step of the seat's angle off the vertical (theta, rad; + toward `forward`) and its rate. Gravity
// on a pendulum of `length`, a little damping, and the pump -- W/S pushing the way it is already going adds
// energy, against it takes energy away -- held to swingMaxDeg.
void swingStep(Real& theta, Real& omega, Real length, Real pump, Real dt, const MovementTuning& t);

// The swimmer's speed at a point in the stroke (phase 0..1): a pull that surges, a glide that eases off; the
// average over a stroke is 1.
Real strokeSpeedScale(Real phase, Real surge);
// Where in strokeSpeedScale's cycle a swimmer is, `strokes` into the crawl: two pulls a stroke (one per arm,
// PlayerArmsSystem's crawl pulls at 0.53 and 0.03), each the surge.
Real strokeSurgePhase(Real strokes);

}  // namespace engine

#endif
