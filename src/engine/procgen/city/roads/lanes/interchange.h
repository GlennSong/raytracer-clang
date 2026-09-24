#ifndef RAYTRACER_ENGINE_PROCGEN_CITY_ROADS_LANES_INTERCHANGE_H
#define RAYTRACER_ENGINE_PROCGEN_CITY_ROADS_LANES_INTERCHANGE_H

// DIAMONDS: the way on and off a freeway, at the streets it crosses.
//
// Right-hand traffic IN PLAN COORDINATES — plan (x, y) is world (x, z), a reflection, so the
// city drives on the LEFT on screen, and that is the intent (Glenn, 2026-09-23). Carriageway
// `a` runs with the route on its RIGHT (-normal), `b` against
// it on the left; each ramp leaves from, or joins, its carriageway's OUTER lane. Each side of
// a crossing street gets two ramps that run in a band beside the deck and meet the street
// where the band crosses it — the off-ramp arriving from upstream, the on-ramp leaving
// downstream — so the pair makes one four-way point on the street and neither ramp lies on
// top of the other. Where that point would sit in the mouth of the street's next junction (a
// freeway between frontage roads), the ramps touch down along the road beside the freeway
// instead: the off-ramp upstream of the crossing, the on-ramp downstream of it.
//
// A ramp is as long as its climb needs at the RAMP's grade under the builder's smoothstep
// profile (steepest at 1.5x the mean), measured to the ground where it lands, plus the part
// of it within reach of that street. A diamond whose ramps would pass over another street
// partway down is refused, not built.
//
// This is the one generator. The level importer builds its routes' diamonds with it, and so
// does the city planner, whose freeway is planned rather than traced (ADR-0092) — a second
// copy would be a second answer to "where can you get on the motorway".

#include "engine/procgen/city/roads/lanes/geom2d.h"
#include "engine/procgen/terrain_field.h"

#include <nlohmann/json.hpp>

#include <functional>
#include <string>
#include <vector>

namespace engine {
namespace roads::lanes {

// A street a ramp may land on, by the id it has in the scene.
struct RampStreet {
    std::string id;
    std::vector<Vec2> xy;
    // A short minor street passing under the freeway that nothing else depends on: a diamond whose
    // ramp or lead lane would pass over it too low may CLOSE it instead of being refused
    // (DiamondResult::closed), its two ends becoming Ts on the roads either side.
    bool closable = false;
};

struct DiamondOptions {
    std::string aId, bId;        // the carriageway edge ids: a runs with the route (on its right), b against it
    std::string idPrefix = "d0"; // ramp ids are <prefix>_<n>_{a,b}_{off,on}
    double carriage = 7.0;       // centreline -> a carriageway's centreline
    double edgeReach = 12.0;     // centreline -> the outer edge of the freeway's pavement
    int freewayLanes = 3;        // lanes per carriageway, and their width: where the outer lane is
    double freewayLaneW = 3.6;
    int maxDiamonds = 8;
    double spacing = 600;        // least distance between interchanges along the route
    double decel = 80, taperOff = 72, aux = 110, taperOn = 90, approach = 60;
    double rampHalf = 4.75;      // a ramp's half-width, shoulders included
    double bandGap = 3.5;        // clear air between the deck's edge and a ramp's: more than the
                                 // pavement's `closing`, or the two fuse into one slab
    double gRamp = 0.08;         // the grade a ramp is sized at (its class's g_max)
    double landing = 20;         // the level approach across a crossing street's half-width
    double diverge = 60;         // gore -> band: the run a ramp takes to pull clear of the deck
    double mouth = 30;           // least distance from a ramp terminal to the next junction on its street,
    double crossMouth = 15;      // and the least, where there is no road beside the freeway to land on
    double splitReach = 120;     // how far along the freeway a side may land from the crossing: a
                                 // grid's cross street, cut by a junction under the deck, carries on
                                 // a block over — a split diamond
    double frontageReach = 90;   // how far out a road beside the freeway may be and still be one
    double shift = 50, along = 20;   // band -> frontage road at grade, then on its line to the terminal
    double clearance = 0;        // the deck's height over the ground at a crossing
    double window = 200, gMax = 0.06;   // the freeway's own profile, for the climb
    std::vector<std::pair<double, double>> keepOut;   // station ranges no ramp's lanes may touch (a system interchange's)
    // The deck's height at a station along the route, as the builder will make it (its floors'
    // tents over the smoothed ground). Unset: estimated as the smoothed ground + clearance, which
    // on rolling ground runs low — the floors hold the deck up over every dip between them — and
    // left ramps sized for 4 m that had to climb 6.5.
    std::function<double(double)> deck;
    HeightField ground;          // empty: a flat city, every climb is zero
};

struct DiamondResult {
    std::vector<nlohmann::json> ramps;
    int built = 0;
    int candidates = 0, rejectedOblique = 0, rejectedSpacing = 0, rejectedTerminal = 0, rejectedRoom = 0, rejectedConflict = 0;
    double squarestRejected = 0;
    std::vector<std::string> closed;   // closable streets the built diamonds closed: drop their edges
};

// `route` is the freeway's centreline (resampled); `streets` are what its ramps may land on.
DiamondResult diamondRamps(const std::vector<Vec2>& route, const std::vector<RampStreet>& streets,
                           const DiamondOptions& o);

// SYSTEM INTERCHANGE: where a radial expressway (the STEM) crosses the ring (the THROUGH road),
// the stem passing under the ring's deck at ground level. Four ramps, the movements a radial
// needs — in from outside to either way round the ring, and either way round the ring back
// out: two corner connectors (right turns) in the quadrants OUTSIDE the ring and two loops
// (left turns, 270 degrees) in the quadrants INSIDE it. The stem's inner leg carries only
// the loops' ends, close to the crossing, because inside the ring it becomes a boulevard into
// midtown within a couple of hundred metres; ring-to-midtown traffic uses the ring's diamonds.
//
// Every ramp is built in its quadrant's own frame, the two roads' stations and offsets
// blended by angle, so a ramp that runs along the curving ring stays at its band offset from
// it rather than from its tangent (the ring bends ~50 m over 300 m).
struct SystemRoad {
    std::vector<Vec2> route;     // centreline
    std::string aId, bId;        // carriageways: a with the route, on its right (-normal); b against it
    double carriage = 7.0;       // centreline -> a carriageway's centreline
    double edgeReach = 12.0;     // centreline -> the outer edge of the pavement
    int lanes = 3;               // per carriageway, and their width: where the outer lane is
    double laneW = 3.6;
};

struct SystemOptions {
    std::string idPrefix = "x0";   // ramp ids are <prefix>_<quadrant>_{loop,link}
    double rampHalf = 4.75, bandGap = 3.5;
    double loopR = 60;             // a loop's radius
    double linkR = 80;             // a corner connector's radius
    double linkReach = 190;        // how far from the crossing a corner connector leaves and joins
    double diverge = 30;           // gore -> band
    // A loop's lanes on its hosts are short: each stops clear of the other road's deck, so on
    // the ring one loop's merge and the other's exit never meet (no cloverleaf weave).
    double loopLane = 60;          // the most any loop lane (decel/aux + taper) may be
    double decel = 80, taperOff = 72, aux = 110, taperOn = 90, approach = 60;
};

// How far along the through road from the crossing a system interchange's lanes reach: a corner
// connector's gore, plus the longer of its decel and aux lanes with their tapers.
double systemReach(const SystemOptions& o);

struct SystemResult {
    std::vector<nlohmann::json> ramps;
    bool built = false;
    std::string why;                 // when not built
    double sThrough = 0, sStem = 0;  // the crossing's station on each road
    double reach = 0;                // stations either side of the crossing the ramps use, on each road
};

// `outward` is +1 when the stem's outer leg (the one that stays a freeway, away from the city)
// runs along increasing station from the crossing, -1 when it runs the other way.
SystemResult systemInterchange(const SystemRoad& through, const SystemRoad& stem, double outward, const SystemOptions& o);

}  // namespace roads::lanes
}  // namespace engine

#endif
