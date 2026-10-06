#include "city_sim.h"

#include "traffic_rules.h"   // approachStop
#include "../../engine/ai/idm.h"   // IDM car-following (roads-v2 S7)
#include "../../profile.h"

#include <algorithm>
#include <iterator>
#include <cstring>
#include <fstream>
#include <filesystem>
#include <chrono>
#include <cmath>
#include <cstdio>    // RT_CRASH_DEBUG contact telemetry
#include <cstdlib>
#include <limits>
#include <unordered_map>

namespace citysim {

using engine::Vec2;
using engine::NavGraph;

namespace {
// How many a bus carries. Beyond this it drives past a stop full.
constexpr int kBusSeats = 24;
// How long a bus stands at a stop: a base wait so a walker (or the player) who
// is nearly there can still make it, plus boarding/alighting time per person.
constexpr Real kBusDwellBase = 10.0;
constexpr Real kBusDwellPerRider = 3.0;
constexpr Real kBusDwellMax = 40.0;
// How far from its destination a driver will park and walk: a few blocks.
constexpr Real kParkSearch = 250.0;
constexpr Real kWalkSpeed = 1.4;
// "Wakes on nothing": a rest with no dwell and no commute. Large enough to mean
// never in any real session, finite so the arithmetic stays ordinary.
constexpr Real kNeverWakes = 1e9;
// 4.0, not the old 6.0: the physical tier measured the real sedan's launch
// (driving-lab envelope probe) — the brains must not plan accelerations the
// body cannot follow, or every possessed car falls behind its ghost out of
// every light and corner (roads-v2.1 R5).
constexpr Real kCarAccel = 4.0;
constexpr Real kPedAccel = 1.0;
constexpr Real kCarMinGap = 5.0, kCarSlowZone = 14.0;
constexpr Real kPedMinGap = 0.8, kPedSlowZone = 2.5;
// Clear space a STOPPED car keeps to the bumper in front (IDM's jam gap s0).
//
// This stays at 0.8 m — which really does read as bumper-to-bumper in a queue —
// because THE NETWORK CANNOT AFFORD MORE. Junction knots are ~4.5 m links, so a
// bigger standstill gap pushes a stopped queue back across the junction it came
// through and deadlocks it. Measured on the signalled-cross soak
// (cars_crash_and_stop_instead_of_ghosting, wreck-escape share of ticks, at
// headway 1.8): 0.8 m -> 7.1%, 1.0 -> 15.4%, 1.2 -> 20.6%, 1.5 -> 70.9%.
// Opening this up is a ROAD-GEOMETRY job (don't let queues stand in junction
// knots), not a constant to raise.
constexpr Real kCarBumperGap = 0.8;
// Desired time headway T (s) — what actually puts room between MOVING cars: IDM
// asks for s0 + v*T of clear road, so at 12 m/s this is ~22 m rather than the
// ~14 m the old 1.1 s left. Following further apart also rear-ends less: same
// soak, at s0 = 0.8, T = 1.1 -> 17.4% wreck-escapes, 1.4 -> 16.3%, 1.8 -> 7.1%,
// 2.2 -> 18.0% (past ~2 s they hesitate and block junctions instead).
//
// CAPPED AT 1.4 BY PEDESTRIAN SAFETY, not by flow: at 1.8 traffic runs freely
// enough that walkers_gap_accept_at_unsignalled_junctions starts recording
// walkers within 1.3 m of a fast car (0 contacts at 1.1/1.4, 3 at 1.8). The
// kerb rule's gap-accept is judging a gap that faster approach speeds close
// sooner than it expects — fix that before opening the headway further.
constexpr Real kCarHeadway = 1.4;

// The fleet of body slots (ADR-0061 Phase 4). A sedan is the player's body
// exactly (4.2 long) and its follow gap works out to the historical 5.0 m; larger
// bodies keep proportionally more room. Lengths are all >= the sedan, so adding
// them never lets cars pack tighter than before. Order is mirrored by the render
// fleet (city_render.cpp kCarVariants), slot for slot.
const VehicleBody kFleet[] = {
    {4.2, 1.80, 1.30, VehicleType::Sedan},
    {4.2, 1.80, 1.30, VehicleType::Sedan},
    {4.2, 1.80, 1.30, VehicleType::Sedan},
    {4.2, 1.82, 1.45, VehicleType::Hatchback},
    {4.2, 1.82, 1.45, VehicleType::Hatchback},
    {4.2, 1.82, 1.45, VehicleType::Hatchback},
    {4.6, 1.95, 1.70, VehicleType::SUV},
    {4.6, 1.95, 1.70, VehicleType::SUV},
    {4.6, 1.95, 1.70, VehicleType::SUV},
    {5.2, 1.95, 1.60, VehicleType::Pickup},
    {5.4, 2.00, 2.10, VehicleType::Van},
    {6.6, 2.40, 2.80, VehicleType::BoxTruck},
};
constexpr int kFleetSize = static_cast<int>(sizeof(kFleet) / sizeof(kFleet[0]));
constexpr Real kJunctionApproach = 9.0, kJunctionSpeed = 4.0;
constexpr Real kSignalApproach = 14.0;          // start braking for a light this far out
constexpr Real kCarDecel = 6.0, kPedDecel = 3.0;
constexpr Real kCarMinTurnRadius = 6.0; // tightest arc a car can trace (m)
constexpr Real kPedVisionRange = 4.5;      // how far ahead a walker perceives (m)
constexpr Real kPedVisionHalfAngle = 1.2;  // ~69 deg to each side (wide peripheral)
constexpr Real kPedMaxLateral = 1.6;       // furthest a walker leans off its path (m)
constexpr Real kPedLateralRate = 1.6;      // how fast that lean changes (m/s) — smooth, not a pop
constexpr Real kPedBodyMin = 0.5;          // hard floor: bodies never closer than this
constexpr Real kPoleClearance = 0.7;       // a walker keeps its centre this far from a pole
constexpr Real kPlayerClearance = 1.1;     // ...and this far from the PLAYER (a wide berth,
                                           // so a near miss is a step-around, not a brush)
constexpr Real kPedClearance = 4.0;     // a car aims to stop this far short of a ped/player
constexpr Real kPedSideBySide = 0.4;   // walkers this close along the way walk abreast: neither leads
constexpr Real kPedHardStop = 3.0;      // and will NOT roll closer than this (a real wall)
constexpr Real kCarBuffer = 2.0;        // bumper to bumper behind a stopped (player's) car
constexpr Real kLaneHalfCorridor = 2.0; // a car this far off our line is in another lane
// The zebra band is painted 0.5..3.6 m past the junction MOUTH (road-texture
// shaders, ADR-0062); a car held at a red must stop with its BUMPER short of
// that band — not at the node, which put a legally-waiting car visually in the
// middle of the intersection, on top of the crosswalk, looking like it ran the
// light (device round 3).
constexpr Real kCrosswalkFarEdge = 3.6;   // band's far edge past the mouth (m)
constexpr Real kStopLineMargin = 0.6;     // bumper clearance short of the band
// The cognition loop (ADR-0063): agents act on MEMORY, not on the momentary
// snapshot. A track must hold at least this much confidence to be acted on —
// with the default 4 s memory horizon that means a body is reacted to for ~3 s
// after it was last actually seen (extrapolated along where it was heading).
constexpr Real kMemoryActConfidence = 0.25;
constexpr Real kTtcHorizon = 2.0;       // brake when a collision is predicted this soon (s)
constexpr Real kCollisionRadius = 1.5;  // car-vs-person combined disc radius for that test
constexpr Real kPedAnticipation = 0.4;  // walkers dodge where a neighbour WILL be (s ahead)

// Three-tier traffic (P4). The far (V) tier traverses its real route at the
// link class speed shaped by FIXED average junction costs — it runs no sensing
// and reads no live signal state, so a junction is a modelled dwell, not a
// queue. kAgentGridCell sizes the P4.1 spatial grid; kGridPad widens its
// bounds past the graph for parked verges, doorsteps and corner blends.
constexpr Real kVSignalDelay = 4.0;     // avg dwell per SIGNALIZED junction crossed
constexpr Real kVJunctionDelay = 1.5;   // avg dwell per stop/yield junction crossed
constexpr Real kAgentGridCell = 64.0;
constexpr Real kGridPad = 250.0;

// Lane spacing that fits THIS road: split the right half of the carriageway
// evenly among the direction's lanes, so a car sits centred in its own lane and
// clearly on its (right-hand) side. A fixed lane width instead leaves a car
// hugging the centreline on a wide road, which reads as driving on the wrong
// side. laneCenter places lane i at (0.5 + i) * spacing off the centreline.
Real laneSpacing(const engine::NavLink& l, Real drivableWidth) {
    int lanes = l.lanes < 1 ? 1 : l.lanes;
    return (l.oneWay ? drivableWidth : drivableWidth * 0.5) /
           static_cast<Real>(lanes);
}

// Speed a follower may travel given the centre-to-centre gap to its leader.
Real followCap(Real freeSpeed, Real gap, Real minGap, Real slowZone) {
    if (gap <= minGap) return 0.0;
    if (gap >= slowZone) return freeSpeed;
    return freeSpeed * (gap - minGap) / (slowZone - minGap);
}

// Rotate unit vector `from` toward unit vector `to` by at most `maxRad` radians.
Vec2 rotateToward(Vec2 from, Vec2 to, Real maxRad) {
    Real fl = std::sqrt(from.x * from.x + from.y * from.y);
    Real tl = std::sqrt(to.x * to.x + to.y * to.y);
    if (fl < 1e-9) return to;
    if (tl < 1e-9) return from;
    from = Vec2(from.x / fl, from.y / fl);
    to = Vec2(to.x / tl, to.y / tl);
    Real dot = from.x * to.x + from.y * to.y;
    if (dot > 1) dot = 1; else if (dot < -1) dot = -1;
    Real ang = std::acos(dot);
    if (ang <= maxRad) return to;        // close enough: snap onto target
    Real sign = (from.x * to.y - from.y * to.x) >= 0 ? 1.0 : -1.0;
    Real a = maxRad * sign;
    Real ca = std::cos(a), sa = std::sin(a);
    return Vec2(from.x * ca - from.y * sa, from.x * sa + from.y * ca);
}

// Is in-world hour `clock` inside the window [from, to)? A window whose `from`
// is LATER than its `to` wraps midnight — a night shift leaving at 21:30 and
// coming home at 05:30 is at work from 21:30 THROUGH to 05:30, not never.
//
// The naive `clock >= from && clock < to` silently makes every wrapping window
// empty, so a night-shift agent would never depart for work at all. Mirrors
// Place::openAt (places.cpp:30-35), which already got this right.
bool inWindow(Real clock, Real from, Real to) {
    if (from == to) return true;                  // degenerate → always inside
    if (from < to) return clock >= from && clock < to;
    return clock >= from || clock < to;           // wraps midnight
}

// Personality traits read from FIXED bits of an agent's `brain` (ADR-0062): no
// rng draw, so seeded build streams are unchanged. Each helper owns its bits.

// Personal pace in [0.85, 1.15] of the nominal speed — owns brain bits 9..23.
Real paceFactor(uint32_t brain) {
    return 0.85 + 0.30 * (((brain >> 9) & 0x7FFF) / Real(0x7FFF));
}

// Think-clock stagger as a 0..1 fraction of the think period — owns bits 17..24.
Real thinkStagger(uint32_t brain) {
    return ((brain >> 17) & 0xFF) / Real(0xFF);
}

// Park/idle setback slot (0..7) spacing same-node agents along the verge — owns
// bits 3..5.
Real parkSetbackSlot(uint32_t brain) {
    return static_cast<Real>((brain >> 3) & 7);
}

// Post-fender-bender freeze (1.0..2.5 s, staggered so tangles unwind car by
// car) — owns bits 5..6.
Real crashHoldSeconds(uint32_t brain) {
    return 1.0 + static_cast<Real>((brain >> 5) & 3) * 0.5;
}
}  // namespace

int vehicleFleetSize() { return kFleetSize; }

const VehicleBody& vehicleFleetBody(int slot) {
    int s = ((slot % kFleetSize) + kFleetSize) % kFleetSize;   // wrap, handle negatives
    return kFleet[s];
}

Real CitySim::vehicleLength(int agentIndex) const {
    const Agent& a = agents_[agentIndex];
    if (a.mode == Agent::Mode::Driver && a.vehicle >= 0 &&
        a.vehicle < static_cast<int>(vehicles_.size()))
        return vehicles_[a.vehicle].length;
    return kPedBodyMin;   // a walker's footprint along its path
}

Real CitySim::pairMinGap(int follower, int leader) const {
    if (agents_[follower].mode == Agent::Mode::Pedestrian) return kPedMinGap;
    // Bumper-to-bumper: half of each body plus a clear buffer. Sedan-to-sedan =
    // 2.1 + 2.1 + 0.8 = 5.0, the historical constant, so all-sedan traffic is
    // unchanged; a longer body simply demands (and is granted) more room.
    return 0.5 * vehicleLength(follower) + 0.5 * vehicleLength(leader) + kCarBumperGap;
}

Real CitySim::junctionRadius(int node) const {
    if (node < 0 || node >= static_cast<int>(nodeBoxRadius_.size())) return 0;
    return nodeBoxRadius_[node];
}

bool CitySim::nearJunction(Vec2 pos, Real margin) const {
    if (junctions_.empty()) return false;
    // Grid-backed (P4.1): candidate junctions from the static bake, then the
    // exact per-junction radius test — the same answer the full scan gave.
    // A local buffer (not queryScratch_) so the host may call this any time.
    std::vector<int> cand;
    junctionGrid_.query(pos, maxJunctionRadius_ + margin, cand);
    for (int ji : cand) {
        const auto& j = junctions_[ji];
        Real r = j.second + margin;
        if ((pos - j.first).lengthSquared() <= r * r) return true;
    }
    return false;
}

Real CitySim::laneSpacingFor(int li) const {
    const engine::NavLink& l = nav_->links[li];
    Real w = l.width;
    if (li < static_cast<int>(bayNarrowed_.size()) && bayNarrowed_[li]) {
        // Both kerb strips are parked, so the DRIVABLE width is the carriageway
        // minus the two Parking bands — the road's own numbers, not a guess.
        // (A link with no band of its own but a parked-up reverse twin borrows
        // the twin's; 2.6 remains the belt-and-braces fallback.)
        Real strip = l.parkWidth;
        if (strip <= 0) {
            for (int ol : nav_->outLinks[l.to])
                if (nav_->links[ol].to == l.from && nav_->links[ol].parkWidth > 0) {
                    strip = nav_->links[ol].parkWidth;
                    break;
                }
        }
        if (strip <= 0) strip = 2.6;
        w = std::max(Real(4.8), w - 2.0 * strip);
    }
    return laneSpacing(l, w);
}

std::vector<Vec2> CitySim::lanePath(int agentIndex, Real step) const {
    std::vector<Vec2> out;
    if (!nav_ || agentIndex < 0 || agentIndex >= static_cast<int>(agents_.size()))
        return out;
    const Agent& a = agents_[agentIndex];
    if (step < 0.5) step = 0.5;
    for (int li : a.route.links) {
        const engine::NavLink& L = nav_->links[li];
        Real spacing = laneSpacingFor(li);
        int n = std::max(1, static_cast<int>(std::ceil(L.length / step)));
        int lane = std::min(a.lane, std::max(1, L.lanes) - 1);
        for (int k = 0; k <= n; ++k) {
            Real t = static_cast<Real>(k) / n;
            Vec2 p = (a.mode == Agent::Mode::Driver)
                         ? nav_->laneCenter(li, lane, t, spacing)
                         : nav_->sidewalkPoint(li, t);
            if (out.empty() || (p - out.back()).length() > 1e-6) out.push_back(p);
        }
    }
    return out;
}

uint32_t CitySim::rnd() {
    rng_ ^= rng_ << 13;
    rng_ ^= rng_ >> 17;
    rng_ ^= rng_ << 5;
    return rng_;
}
Real CitySim::rndUnit() { return (rnd() >> 8) * (1.0 / 16777216.0); }

// A draw from THIS AGENT'S OWN stream. Trip decisions (which lane to take, where
// to wander to) used to come off the shared `rng_`, which made every agent's
// choices depend on how many decisions every OTHER agent had made first. That is
// fine while the population is stepped in one fixed order, and fatal the moment
// anything advances an agent out of turn — which is exactly what waking a
// dormant agent from its schedule does. Per-agent streams make a decision depend
// only on how many decisions that agent has made.
//
// Seeded from `brain` by hash at build (never a draw), so the build stream — and
// every seeded scenario — is untouched. Distinct from brainUnit's stream: that
// one owns perception faults, and sharing would couple a car's lane choice to
// how often it had misperceived something.
uint32_t CitySim::tripRnd(Agent& a) {
    a.tripRng ^= a.tripRng << 13;
    a.tripRng ^= a.tripRng >> 17;
    a.tripRng ^= a.tripRng << 5;
    return a.tripRng;
}

Real CitySim::brainUnit(Agent& a) {
    a.brain ^= a.brain << 13;
    a.brain ^= a.brain >> 17;
    a.brain ^= a.brain << 5;
    return (a.brain >> 8) * (1.0 / 16777216.0);
}

void CitySim::build(const NavGraph& graph, int driverCount, int pedCount, uint32_t seed) {
    nav_ = &graph;
    // "There and back by car" for the whole build, once (engine::stronglyConnected).
    const std::vector<int> buildCarComp = engine::stronglyConnected(graph, /*onFoot=*/false);
    agents_.clear();
    vehicles_.clear();
    sensed_.clear();
    externalObstacles_.clear();   // a REBUILD must not keep the previous level's
    staticObstacles_.clear();     // player/pole positions as phantom obstacles
    clockHours_ = 8.5;   // start mid morning-rush so agents commute right away
    simSeconds_ = 0;
    faultCount_ = 0;
    rng_ = seed ? seed : 0x6c078965u;
    ids_.reset();   // fresh scene → UIDs restart at 0 (reproducible from seed)
    // Three-tier bookkeeping (P4): a rebuild starts everyone K on frame 0 with
    // no bubble centre — the host re-feeds the player each step. The knobs
    // (tieringEnabled, radii) persist like wander_: the host owns them.
    frameIndex_ = 0;
    promotions_ = 0;
    demotions_ = 0;
    haveTierCenter_ = false;
    grid_.configure(engine::Vec2(0, 0), engine::Vec2(1, 1), kAgentGridCell, 0);
    parkedGrid_.configure(engine::Vec2(0, 0), engine::Vec2(1, 1), kAgentGridCell, 0);
    junctionGrid_.configure(engine::Vec2(0, 0), engine::Vec2(1, 1),
                            kAgentGridCell, 0);
    maxJunctionRadius_ = 0;
    signals_.build(graph);
    // Curbside bays (R6b): mid-link, in the road's own PARKING BAND, well clear
    // of junction mouths; ~55% seeded with scenery cars so streets read
    // parked-along from the first frame.
    //
    // The band is the gate AND the geometry. Before the parking round this loop
    // guessed — "Local-or-Collector and at least 9 m wide" — and then hung the
    // bay at a hardcoded `width*0.5 - 1.05`, which put the outer edge of a 2.2 m
    // bay ~7 cm PAST the carriageway onto the kerb (Glenn: "it's half on the
    // sidewalk"). Now a link parks iff its road spec actually gives it a Parking
    // band, and the bay centre is that band's centre. No heuristic left.
    bays_.clear();
    baysOnLink_.assign(graph.linkCount(), {});
    // BAYS RIDE THE CHAIN, NOT THE SEGMENT. A NavLink is one SAMPLED piece of a
    // road, and the sampler collapses a straight run to a single long segment
    // while a CURVED one shatters into short ones. Gating each segment on 40 m
    // therefore parked straight streets and silently refused every curved one:
    // measured on the metro parking recipe, 826 links, 240 of them carrying a
    // parking band but under 40 m, and just TWO long enough to park on — those
    // two produced every bay in the city. Add any curvature and the whole city's
    // kerbside parking vanished.
    //
    // A chain is what the 22 m junction setback was always about: junction to
    // junction, through the degree-2 curve samples between them. Bays are laid
    // along the chain's arc length and then mapped back onto whichever segment
    // holds each station, so a curved street parks exactly like a straight one
    // of the same length. The parking BAND stays a per-segment gate — the spec
    // can change along a chain.
    std::vector<int> nextOf(graph.linkCount(), -1);
    std::vector<char> hasPrev(graph.linkCount(), 0), seen(graph.linkCount(), 0);
    for (int li = 0; li < graph.linkCount(); ++li) {
        const int n = graph.links[li].to;
        if (n < 0 || n >= graph.nodeCount()) continue;
        if (graph.junction[n]) continue;              // a real intersection ends it
        int cont = -1, count = 0;
        for (int lo : graph.outLinks[n]) {
            if (graph.links[lo].to == graph.links[li].from) continue;   // the U-turn back
            ++count;
            cont = lo;
        }
        if (count == 1 && cont != li) { nextOf[li] = cont; hasPrev[cont] = 1; }
    }
    // RT_PARK_DEBUG=1: why a street ended up with no kerbside parking. Each gate
    // counted separately, because "0 bays" on its own says nothing about which
    // one rejected it.
    int dbgChains = 0, dbgShortChain = 0, dbgNoBand = 0, dbgLayer = 0,
        dbgNoFrontage = 0, dbgPlaced = 0;
    auto layChain = [&](int first) {
        std::vector<int> chain;
        for (int li = first; li != -1 && !seen[li]; li = nextOf[li]) {
            seen[li] = 1;
            chain.push_back(li);
        }
        if (chain.empty()) return;
        ++dbgChains;
        Real total = 0;
        for (int li : chain) total += graph.links[li].length;
        if (total < 44.0) { ++dbgShortChain; return; }   // no room once both ends are set back
        std::size_t ci = 0;
        Real base = 0;
        for (Real st = 22.0; st + 22.0 < total; st += 7.0) {
            while (ci + 1 < chain.size() && base + graph.links[chain[ci]].length <= st) {
                base += graph.links[chain[ci]].length;
                ++ci;
            }
            const int li = chain[ci];
            const engine::NavLink& L = graph.links[li];
            if (L.parkWidth <= 0.0) { ++dbgNoBand; continue; }   // no Parking band here
            if (L.layer != 0 || L.elevAbsolute) { ++dbgLayer; continue; }
            // Semantic layer (#17/S7): bays only on FRONTAGE edges — never on a
            // ramp APPROACH (a street climbing to a landing has no frontage, so
            // no curbside parking on the on-ramp feeder).
            if (!(L.access & engine::road_access::kFrontage)) { ++dbgNoFrontage; continue; }
            if (L.length < 1e-3) continue;
            const engine::Vec2 dir = graph.direction(li);
            const engine::Vec2 right(dir.y, -dir.x);
            const Real local = std::min(L.length, std::max(Real(0), st - base));
            ParkingBay b;
            b.link = li;
            b.station = local;
            b.pos = graph.pointOnLink(li, local / L.length) + right * L.parkOffset;
            b.heading = dir;
            b.width = L.parkWidth;
            // EVERY BAY IS BORN FREE. This used to seed ~55% of them with
            // kBayScenery — a decorative car belonging to nobody, ~4400 of them
            // on piedmont, ~700 of which were drawn and given a Jolt proxy at
            // any moment (~1.3 M triangles). They made the city look inhabited
            // while hiding the fact that its actual population never parked
            // anywhere: a real driver could only take a bay the filler had left
            // free. Parked cars are now exclusively cars that an agent drove
            // there and will drive away again.
            b.occupant = -1;
            baysOnLink_[li].push_back(static_cast<int>(bays_.size()));
            bays_.push_back(b);
            ++dbgPlaced;
        }
    };
    for (int li = 0; li < graph.linkCount(); ++li)
        if (!hasPrev[li]) layChain(li);
    // A ring with no junction on it has no start; every link claims a
    // predecessor. Seed the leftovers so a roundabout or a loop road still parks.
    for (int li = 0; li < graph.linkCount(); ++li)
        if (!seen[li]) layChain(li);
    if (std::getenv("RT_PARK_DEBUG"))
        std::fprintf(stderr,
                     "[park] links=%d chains=%d  short-chain=%d  no-band=%d  "
                     "layer=%d  no-frontage=%d  bays=%zu\n",
                     graph.linkCount(), dbgChains, dbgShortChain, dbgNoBand, dbgLayer,
                     dbgNoFrontage, bays_.size());
    twinOf_.assign(graph.linkCount(), -1);
    departScale_.assign(graph.linkCount(), 1.0);
    for (int li = 0; li < graph.linkCount(); ++li)
        for (int ol : graph.outLinks[graph.links[li].to])
            if (graph.links[ol].to == graph.links[li].from) { twinOf_[li] = ol; break; }
    bayNarrowed_.assign(graph.linkCount(), 0);
    for (int li = 0; li < graph.linkCount(); ++li)
        if (!baysOnLink_[li].empty()) bayNarrowed_[li] = 1;
    // The reverse twin of a parked-up roadway narrows too (each direction
    // parks its own right curb; the carriageway loses both strips).
    for (int li = 0; li < graph.linkCount(); ++li) {
        if (bayNarrowed_[li]) continue;
        const engine::NavLink& L = graph.links[li];
        for (int ol : graph.outLinks[L.to])
            if (graph.links[ol].to == L.from && bayNarrowed_[ol]) {
                bayNarrowed_[li] = 1;
                break;
            }
    }
    // Goal tables (ADR-0064): a rebuild resets to the built-ins matching the
    // (persistent) wander flag; a host with custom tables re-installs after.
    goalPed_ = wander_ ? wanderGoals(false) : defaultScheduleGoals();
    goalDriver_ = wander_ ? wanderGoals(true) : defaultScheduleGoals();

    const int nAll = graph.nodeCount();
    if (nAll == 0) return;
    const int n = graph.streetNodeCount();   // spawns and random homes/works: the streets, never the walks

    // Per-node box radius (widest incident half-width) — every junction rule and
    // the launch clearance read this via junctionRadius(). Junction boxes (centre
    // + radius) additionally feed the bridge's don't-block-the-box and
    // spawn-placement checks (nearJunction).
    junctions_.clear();
    nodeBoxRadius_.assign(nAll, 0.0);
    for (int v = 0; v < nAll; ++v) {
        Real r = 0;
        // The box must FIT its local geometry: a curvy crossing gets sampled into
        // a KNOT of nodes a few metres apart, and a half-width-sized box there
        // (arterial: 6.5 m vs 4.5 m links) swallows the neighbouring nodes' stop
        // lines — every held car sits inside someone else's box, a circular wait,
        // permanent gridlock (device: "cars all stuck at one intersection"). Cap
        // the radius at 60% of the shortest incident link so adjacent boxes can
        // never overlap; open grids (80 m links) are untouched.
        //
        // "Neighbouring" means the next JUNCTION, not the next node: metro's
        // streets are chains of ~4 m links (the polyline's own vertices), and
        // capping by the adjacent link shrank every box there to ~2.5 m -- on
        // a 17 m arterial the stop line landed inside the crossing, which is
        // where cars were seen waiting (Glenn, 2026-09-19). So walk each arm
        // through its plain (degree-2) nodes to the next junction.
        // (A plain node keeps its adjacent-link cap: it has no box.)
        Real shortestArm = 1e9;
        for (int li : graph.outLinks[v]) {
            r = std::max(r, graph.links[li].width * 0.5);
            if (!graph.isJunction(v)) {
                shortestArm = std::min(shortestArm, graph.links[li].length);
                continue;
            }
            Real arm = 0;
            int cur = li;
            for (int hop = 0; hop < 64; ++hop) {
                const engine::NavLink& L = graph.links[cur];
                arm += L.length;
                if (arm >= 200.0 || graph.isJunction(L.to)) break;
                int next = -1;
                for (int ol : graph.outLinks[L.to])
                    if (graph.links[ol].to != L.from) { next = ol; break; }
                if (next < 0) break;   // dead end
                cur = next;
            }
            shortestArm = std::min(shortestArm, arm);
        }
        // A street INTERSECTION's box reaches the drawn mouth (half-width +
        // sidewalk): the stop line and the zebra are measured from there.
        if (graph.isJunction(v) && junctionPad_ > 0) {
            const engine::JunctionKind k = graph.kindOf(v);
            if (k == engine::JunctionKind::Intersection || k == engine::JunctionKind::Auto)
                r += junctionPad_;
        }
        if (shortestArm < 1e9) r = std::min(r, std::max(Real(1.5), shortestArm * 0.6));
        nodeBoxRadius_[v] = r;
        if (graph.isJunction(v)) junctions_.push_back({graph.nodes[v], r});
    }

    const int total = driverCount + pedCount;
    agents_.reserve(total);
    vehicles_.reserve(driverCount);
    for (int i = 0; i < total; ++i) {
        Agent a;
        a.uid = ids_.next();   // stable identity for relationships / memory / jobs
        a.mode = (i < driverCount) ? Agent::Mode::Driver : Agent::Mode::Pedestrian;
        // Identity, fixed for the run. Starts equal to the initial locomotion:
        // a car owner begins the day in its car, a walker on foot. Only `mode`
        // moves from here.
        a.archetype = a.mode;
        a.home = static_cast<int>(rnd() % n);
        a.work = static_cast<int>(rnd() % n);
        // Pick a work node the home can actually REACH. An unroutable pair used to
        // teleport the agent to its goal on departure (it "disappeared" and
        // reappeared); instead insist on a routable pair, and if none turns up
        // just keep the agent home (work == home -> it never commutes).
        {
            // BOTH directions must route (the graph is directed — one-way ramps):
            // a valid home->work with no work->home used to retry a failing A*
            // every single tick once the agent wanted to come home.
            // (There and back is one strongly connected component -- engine::stronglyConnected, computed
            // once for the build, not two searches a try.)
            auto commutable = [&](int h, int w) {
                return w != h && h >= 0 && w >= 0 &&
                       buildCarComp[static_cast<std::size_t>(h)] == buildCarComp[static_cast<std::size_t>(w)];
            };
            bool ok = commutable(a.home, a.work);
            for (int tries = 0; tries < 8 && !ok && n > 1; ++tries) {
                a.work = static_cast<int>(rnd() % n);
                ok = commutable(a.home, a.work);
            }
            if (!ok) a.work = a.home;   // stranded: stay put, never depart
        }
        // SHIFT DIVERSITY jitter. The two draws stay HERE, at this exact point
        // in the rng stream — the shift ladder that consumes them lives below
        // the brain seed (see there for why), and moving these would shift every
        // later draw and re-pin every seeded scenario in the suite.
        const Real j0 = rndUnit(), j1 = rndUnit();       // per-agent jitter
        a.activity = Agent::Activity::AtHome;
        a.goal = tableFor(a).entry();   // the day starts at the table's entry
        a.brain = rnd() | 1u;            // per-agent fault RNG (non-zero)
        // Trip stream: HASHED from the brain, not drawn, so the build sequence
        // (and every seeded scenario's layout) is unchanged. Must be non-zero —
        // a xorshift seeded at 0 stays at 0 forever.
        a.tripRng = (a.brain * 2654435761u) ^ 0x9E3779B9u;
        if (a.tripRng == 0) a.tripRng = 1u;
        // SHIFT DIVERSITY. A single 7.5-9.0 departure window put the whole
        // population on the road at once: one hot rush, a dead city either
        // side of it, and a sim-cost spike where nearly every agent is moving
        // in the same minutes. Real cities run shifts, so agents draw one from
        // their own brain bits (deterministic, no rng draw) — the city now has
        // hot and cold hours per district instead of one synchronised surge.
        //
        // THIS BLOCK MUST FOLLOW THE BRAIN SEED. It used to sit above it and so
        // read the default `brain = 1`, making `shift` 0 for EVERY agent: the
        // whole city took the first branch, the five archetypes below never
        // existed, and — because the build clock starts at 8.5, inside every
        // agent's [5.0, 14.0) window — the entire fleet departed on the first
        // think and spent the opening minutes of play as one city-wide jam.
        {
            const uint32_t shift = (a.brain >> 12) & 0xFF;   // 0..255
            if (shift < 26) {            // ~10% early shift (trades, transit)
                a.departWork = 5.0 + j0 * 1.5;
                a.departHome = 14.0 + j1 * 1.5;
            } else if (shift < 166) {    // ~55% standard day, widened
                a.departWork = 7.0 + j0 * 2.0;
                a.departHome = 16.0 + j1 * 2.5;
            } else if (shift < 204) {    // ~15% swing / late start
                a.departWork = 11.5 + j0 * 3.0;
                a.departHome = 20.0 + j1 * 3.0;
            } else if (shift < 227) {    // ~9% night shift (wraps midnight)
                a.departWork = 21.5 + j0 * 1.5;
                a.departHome = 5.5 + j1 * 1.5;
            } else {                     // ~11% part-time / midday
                a.departWork = 9.5 + j0 * 2.0;
                a.departHome = 14.5 + j1 * 2.0;
            }
        }
        // Personality from the brain's own bits (NOT an rnd() draw, so the build
        // stream — and every seeded test scenario — is unchanged).
        a.speedFactor = paceFactor(a.brain);
        // Stagger the think clocks so the crowd doesn't re-decide in lockstep.
        a.thinkTimer = thinkPeriod_ * thinkStagger(a.brain);
        a.restNode = a.home;
        a.pos = idlePose(a.home, a.mode, a.brain);
        // Idle bodies face along their road from the start — a parked car with
        // the default (1,0) heading sat at a random angle to the verge it was
        // parked on, which read as "failed to be placed" (device round 3).
        if (!graph.outLinks[a.home].empty())
            a.heading = graph.direction(graph.outLinks[a.home][0]);

        // A driver possesses a freshly-created car (two-way possession link). Its
        // body comes from the fleet slot matching its index, so the renderer (which
        // maps the same index to a body) draws exactly this shape + size.
        if (a.mode == Agent::Mode::Driver) {
            SimVehicle v;
            const VehicleBody& body =
                fleetBody(ambientSlotFor(static_cast<int>(vehicles_.size())));
            v.length = body.length;
            v.width = body.width;
            v.height = body.height;
            v.type = body.type;
            v.driver = static_cast<int>(agents_.size());
            v.pos = a.pos;
            a.vehicle = static_cast<int>(vehicles_.size());
            a.car = a.vehicle;   // owns it for the run; `vehicle` is only "in it now"
            vehicles_.push_back(v);
        }
        agents_.push_back(a);
    }

    // P4.1: size the spatial grid to the graph (padded for verges, doorsteps
    // and corner blends) and hash the initial cohort; the junction grid is a
    // static bake of junctions_ so nearJunction() stops scanning the list.
    {
        Vec2 lo = graph.nodes[0], hi = graph.nodes[0];
        for (const Vec2& p : graph.nodes) {
            lo.x = std::min(lo.x, p.x);
            lo.y = std::min(lo.y, p.y);
            hi.x = std::max(hi.x, p.x);
            hi.y = std::max(hi.y, p.y);
        }
        const Vec2 pad(kGridPad, kGridPad);
        grid_.configure(lo - pad, hi + pad, kAgentGridCell,
                        static_cast<int>(agents_.size()));
        vGrid_.configure(lo - pad, hi + pad, kAgentGridCell, static_cast<int>(agents_.size()));
        dGrid_.configure(lo - pad, hi + pad, kAgentGridCell, static_cast<int>(agents_.size()));
        rebuildTierLists();
        rehashAll_ = true;
        for (std::size_t i = 0; i < agents_.size(); ++i)
            grid_.place(static_cast<int>(i), agents_[i].pos);
        // PARKED CARS get their own index. A car left at the kerb is nowhere
        // near its owner — who is standing at a door tens of metres away — so
        // it cannot be found by querying agents, which is how every other body
        // in the sim is located. Placement happens only when a car parks, so
        // this costs nothing per step.
        parkedGrid_.configure(lo - pad, hi + pad, kAgentGridCell,
                              static_cast<int>(vehicles_.size()));
        for (std::size_t v = 0; v < vehicles_.size(); ++v)
            parkedGrid_.place(static_cast<int>(v), vehicles_[v].pos);
        junctionGrid_.configure(lo - pad, hi + pad, kAgentGridCell,
                                static_cast<int>(junctions_.size()));
        for (std::size_t j = 0; j < junctions_.size(); ++j) {
            junctionGrid_.place(static_cast<int>(j), junctions_[j].first);
            maxJunctionRadius_ = std::max(maxJunctionRadius_, junctions_[j].second);
        }
    }
    applyTaxiFraction();   // a rebuild re-marks the cabs (the knob persists)
    // Measure the commute here as well as after assignPlaces: a level with no
    // authored places never calls that, and scheduleSnapshot (and the host's
    // clock-scale warning) both need a travel time to be meaningful.
    measureCommute(graph);
}


namespace {
// ---- the population cache (CitySim::setPopulationCacheDir) --------------------------------------------
constexpr uint32_t kPopulationFormat = 2;   // bump when assignPlaces' rules or this record change
struct Fnv {
    uint64_t h = 1469598103934665603ull;
    void bytes(const void* p, std::size_t n) {
        const unsigned char* c = static_cast<const unsigned char*>(p);
        for (std::size_t i = 0; i < n; ++i) { h ^= c[i]; h *= 1099511628211ull; }
    }
    template <typename T> void pod(const T& v) { bytes(&v, sizeof v); }
    void real(Real v) { const double d = static_cast<double>(v); pod(d); }
    void vec(const engine::Vec2& v) { real(v.x); real(v.y); }
};
// One agent's assignment, as stored.
struct PopRecord {
    int32_t home, work, shop, restNode;
    uint32_t homePlace, workPlace, shopPlace;
    double homeDoor[2], workDoor[2], shopDoor[2], pos[2];
    double heading[2], departHome, departWork, commuteSeconds;
    uint8_t role, archetype, mode, indoors;
};
struct PopTail {
    int32_t crossTownDrivers, driversWithJobs, busCommuters, busCommuteTried;
    double driverCommute, medianAll, medianDrive, medianWalk;
};
}  // namespace

void CitySim::assignPlaces(const PlaceMap& places, const NavGraph& graph) {
    relationships_.clear();
    for (Agent& a : agents_) { a.homePlace = kNoPlace; a.workPlace = kNoPlace; }
    if (places.empty() || graph.nodeCount() == 0) return;

    // The university's residence hall is the STUDENTS' home (below), not one more house in the pool.
    std::vector<PlaceId> homes;
    for (PlaceId id : places.ofType(PlaceType::Home))
        if (places[id].campus != 3) homes.push_back(id);
    if (homes.empty()) homes = places.ofType(PlaceType::Home);
    const std::vector<PlaceId>& parks = places.ofType(PlaceType::Park);
    // A "job" is a workplace place: shop / office / civic (a park is not a job).
    std::vector<PlaceId> jobs;
    for (PlaceType t : {PlaceType::Shop, PlaceType::Office, PlaceType::Civic,
                        PlaceType::Cafe, PlaceType::Restaurant, PlaceType::Supermarket})
        for (PlaceId id : places.ofType(t)) jobs.push_back(id);
    venues_.clear();
    if (homes.empty()) return;   // nowhere to live → leave the built schedule alone

    // The nav node a place routes through (nearest to its snapped entrance), each looked up once:
    // nearestNode scans every node, and the job search below asks it again and again.
    std::vector<int> placeNode(places.places().size(), -2);
    auto nodeOf = [&](PlaceId id) {
        int& n = placeNode[static_cast<std::size_t>(id)];
        if (n == -2) n = graph.nearestNode(places[id].entrance);
        return n;
    };
    // THERE AND BACK by car: two places are commutable exactly when they sit in the same strongly
    // connected component of the road graph (findRoute's own link rules). This was two A* searches a
    // candidate -- up to 24 candidates for each of 28,000 agents -- and on an island of separate street
    // networks a failed search floods everything it can reach first: a warm island_8_nature load spent
    // ~9 of its 10.5 minutes here before the first frame.
    const std::vector<int> carComp = engine::stronglyConnected(graph, /*onFoot=*/false);
    auto sameComp = [&](int u, int v) {
        return u >= 0 && v >= 0 && carComp[static_cast<std::size_t>(u)] == carComp[static_cast<std::size_t>(v)];
    };
    auto commutable = [&](int h, int w) { return w != h && sameComp(h, w); };

    // A place's DOORSTEP: its sidewalk entrance nudged toward the building site,
    // so a resting body stands at the door — clearly off the carriageway.
    auto doorOf = [&](PlaceId id) {
        const Place& p = places[id];
        return p.entrance + (p.site - p.entrance) * 0.4;
    };
    // Everywhere people go OUT to: every place but a home. What outings and
    // lunch breaks choose from at trip time (pickOuting / pickLunch).
    for (const Place& p : places.places()) {
        if (p.type == PlaceType::Home) continue;
        Venue v;
        v.type = p.type;
        v.node = nodeOf(p.id);
        v.door = doorOf(p.id);
        v.openHour = p.openHour;
        v.closeHour = p.closeHour;
        v.campus = p.campus;
        if (v.node >= 0) venues_.push_back(v);
    }

    // Scratch for the on-foot job search, hoisted: one allocation, not one
    // per walker.
    std::vector<std::pair<Real, PlaceId>> jobDist;
    int crossTownDrivers = 0, driversWithJobs = 0, busCommuters = 0, busCommuteTried = 0;
    Real driverCommute = 0;

    // THE POPULATION CACHE: the key is everything the decisions below read.
    popCache_ = PopulationCacheReport{};
    std::string popPath;
    std::vector<PopRecord> cached;
    PopTail cachedTail{};
    const bool verify = std::getenv("RT_POPULATION_VERIFY") != nullptr;
    if (!populationCacheDir_.empty()) {
        Fnv k;
        k.pod(kPopulationFormat);
        k.pod(static_cast<int32_t>(graph.nodeCount()));
        for (const Vec2& p : graph.nodes) k.vec(p);
        for (const engine::NavLink& L : graph.links) {
            k.pod(L.from); k.pod(L.to); k.real(L.length); k.real(L.width); k.pod(L.klass); k.pod(L.walkable); k.pod(L.oneWay);
        }
        for (const Place& p : places.places()) {
            k.pod(p.id); k.pod(p.type); k.vec(p.site); k.vec(p.entrance); k.pod(p.entranceLink); k.real(p.entranceT);
            k.real(p.openHour); k.real(p.closeHour); k.pod(p.capacity); k.pod(p.authoredHours); k.pod(p.campus);
        }
        for (std::size_t i = 0; i < agents_.size(); ++i) {
            const Agent& a = agents_[i];
            k.pod(a.uid); k.pod(a.brain); k.pod(a.archetype); k.pod(a.mode); k.real(a.speedFactor);
            k.real(a.departHome); k.real(a.departWork);
            k.pod(isBus(static_cast<int>(i))); k.pod(isTaxi(static_cast<int>(i)));
        }
        k.real(busCommuteShare_); k.real(busMaxWalk_); k.real(longCommuteShare_);
        for (int r = 0; r < buses_.routeCount(); ++r) {
            const BusRoute& br = buses_.route(r);
            k.pod(br.regional); k.pod(br.network); k.real(br.pace);
            for (int nd : br.pathNodes) k.pod(nd);
            for (const BusStop& st : br.stops) { k.pod(st.node); k.pod(st.pathIndex); }
        }
        popCache_.used = true;
        popCache_.key = k.h;
        char name[40];
        std::snprintf(name, sizeof name, "%016llx.pop", static_cast<unsigned long long>(k.h));
        popPath = populationCacheDir_ + "/" + name;
        std::ifstream in(popPath, std::ios::binary);
        char magic[6] = {0};
        uint32_t fmt = 0, count = 0;
        uint64_t key = 0;
        if (in && in.read(magic, 6) && std::string(magic, 6) == "RTPOP1" && in.read(reinterpret_cast<char*>(&fmt), 4) &&
            fmt == kPopulationFormat && in.read(reinterpret_cast<char*>(&key), 8) && key == k.h &&
            in.read(reinterpret_cast<char*>(&count), 4) && count == agents_.size()) {
            cached.resize(count);
            if (in.read(reinterpret_cast<char*>(cached.data()), static_cast<std::streamsize>(count * sizeof(PopRecord))) &&
                in.read(reinterpret_cast<char*>(&cachedTail), sizeof cachedTail))
                popCache_.hit = true;
            else
                cached.clear();
        }
    }
    auto restore = [&](Agent& a, const PopRecord& r) {
        a.home = r.home; a.work = r.work; a.shop = r.shop; a.restNode = r.restNode;
        a.homePlace = r.homePlace; a.workPlace = r.workPlace; a.shopPlace = r.shopPlace;
        a.homeDoor = Vec2(r.homeDoor[0], r.homeDoor[1]); a.workDoor = Vec2(r.workDoor[0], r.workDoor[1]);
        a.shopDoor = Vec2(r.shopDoor[0], r.shopDoor[1]); a.pos = Vec2(r.pos[0], r.pos[1]);
        a.heading = Vec2(r.heading[0], r.heading[1]); a.departHome = r.departHome; a.departWork = r.departWork;
        a.commuteSeconds = r.commuteSeconds;
        a.role = static_cast<Agent::Role>(r.role); a.archetype = static_cast<Agent::Mode>(r.archetype);
        a.mode = static_cast<Agent::Mode>(r.mode); a.indoors = r.indoors != 0;
    };
    auto record = [](const Agent& a) {
        PopRecord r;
        std::memset(&r, 0, sizeof r);   // padding too: VERIFY compares records byte for byte
        r.home = a.home; r.work = a.work; r.shop = a.shop; r.restNode = a.restNode;
        r.homePlace = a.homePlace; r.workPlace = a.workPlace; r.shopPlace = a.shopPlace;
        r.homeDoor[0] = a.homeDoor.x; r.homeDoor[1] = a.homeDoor.y; r.workDoor[0] = a.workDoor.x; r.workDoor[1] = a.workDoor.y;
        r.shopDoor[0] = a.shopDoor.x; r.shopDoor[1] = a.shopDoor.y; r.pos[0] = a.pos.x; r.pos[1] = a.pos.y;
        r.heading[0] = a.heading.x; r.heading[1] = a.heading.y; r.departHome = a.departHome; r.departWork = a.departWork; r.commuteSeconds = a.commuteSeconds;
        r.role = static_cast<uint8_t>(a.role); r.archetype = static_cast<uint8_t>(a.archetype);
        r.mode = static_cast<uint8_t>(a.mode); r.indoors = a.indoors ? 1 : 0;
        return r;
    };
    if (popCache_.hit && !verify) {
        for (std::size_t i = 0; i < agents_.size(); ++i) restore(agents_[i], cached[i]);
        crossTownDrivers = cachedTail.crossTownDrivers; driversWithJobs = cachedTail.driversWithJobs;
        busCommuters = cachedTail.busCommuters; busCommuteTried = cachedTail.busCommuteTried;
        driverCommute = cachedTail.driverCommute;
        commuteSecondsMedian_ = cachedTail.medianAll; commuteSecondsDrive_ = cachedTail.medianDrive;
        commuteSecondsWalk_ = cachedTail.medianWalk;
    } else {
    for (Agent& a : agents_) {
        // Home: deterministic pick from the agent's own brain bits (no rng
        // draw). MIXED first: brains are forced odd (rnd()|1), so a raw
        // modulo could only ever reach odd home indices when homes.size()
        // is even — half the housing stock sat empty.
        std::uint32_t hmix = a.brain;
        hmix ^= hmix >> 16; hmix *= 0x7feb352dU;
        hmix ^= hmix >> 15; hmix *= 0x846ca68bU;
        hmix ^= hmix >> 16;
        PlaceId hp = homes[hmix % homes.size()];
        int hn = nodeOf(hp);
        a.homePlace = hp;
        a.home = hn;
        a.homeDoor = doorOf(hp);
        // A BUS STAYS AT ITS STOP. setBuses runs before this and seats every
        // bus at a stop on its own route; moving it "home" here sent all 24
        // back to their drivers' houses, up to 1.8 km off their loops, and the
        // first lap was spent driving back to the route. A bus still gets a
        // home -- places are per agent -- it just does not start there.
        if (!isBus(indexOf(a))) {
            a.restNode = hn;
            // Walkers start the day at their door (indoors); cars stay parked on
            // the verge (idlePose) — a car "at home" is a car parked outside it.
            a.pos = a.mode == Agent::Mode::Pedestrian ? a.homeDoor
                                                      : idlePose(hn, a.mode, a.brain);
            a.indoors = a.mode == Agent::Mode::Pedestrian;
            if (!graph.outLinks[hn].empty())
                a.heading = graph.direction(graph.outLinks[hn][0]);
        }

        // Role (Phase 4) decides how the day runs. ~1 in 5 (when the city has a
        // park) is a Stroller — no job, a day out at the park; the rest hold a job
        // and are a Shopkeeper (if their workplace is a shop) or a Commuter. All
        // deterministic from the brain's bits (no rng draw).
        a.work = a.home;
        a.role = Agent::Role::Commuter;
        const uint32_t roleRoll = (a.brain >> 20) & 0xFF;   // 0..255
        const bool stroller = !parks.empty() && roleRoll < 51;   // ~20%
        auto unit = [](uint32_t bits) { return (bits & 0xFF) / 255.0; };
        // A WALKER WITH THE DAY OFF (Glenn, 2026-09-19: "we should have non
        // workers and pedestrians who are out for a stroll or going to public
        // spaces"): about one walker in three holds no job and spends part of
        // the day out -- park, cafe, shops, a walk round the block (the
        // outing table, strollerGoals). Their outings are spread over the day
        // (early, mid-morning, afternoon, evening) so the street is never
        // just the commute.
        const bool dayOff = a.archetype == Agent::Mode::Pedestrian &&
                            !venues_.empty() && roleRoll < 85;   // ~1 in 3
        int anchor = -1;
        if (dayOff) {
            // The window mechanics need somewhere that is not home: the
            // nearest routable venue. Outings themselves choose afresh.
            Real best = 1e30;
            for (const Venue& v : venues_) {
                const Vec2 d = graph.nodes[static_cast<std::size_t>(v.node)] -
                               graph.nodes[static_cast<std::size_t>(hn)];
                const Real d2 = d.x * d.x + d.y * d.y;
                if (d2 < best && v.node != hn) { best = d2; anchor = v.node; }
            }
            if (anchor >= 0 && !commutable(hn, anchor)) anchor = -1;
        }
        if (anchor >= 0) {
            a.role = Agent::Role::Stroller;
            a.work = anchor;
            a.workPlace = kNoPlace;
            a.workDoor = graph.nodes[static_cast<std::size_t>(anchor)];
            const Real u0 = unit(a.brain >> 4), u1 = unit(a.brain >> 12);
            switch ((a.brain >> 28) & 3u) {
                case 0:  a.departWork = 6.5 + 1.5 * u0;  a.departHome = a.departWork + 2.0 + 2.0 * u1; break;
                case 1:  a.departWork = 9.0 + 2.0 * u0;  a.departHome = a.departWork + 3.0 + 3.0 * u1; break;
                case 2:  a.departWork = 12.5 + 2.0 * u0; a.departHome = a.departWork + 3.0 + 2.0 * u1; break;
                default: a.departWork = 17.5 + 1.5 * u0; a.departHome = a.departWork + 2.0 + 2.0 * u1; break;
            }
        } else if (stroller) {
            PlaceId pk = parks[(a.brain >> 4) % parks.size()];
            if (commutable(hn, nodeOf(pk))) {
                a.role = Agent::Role::Stroller;
                a.work = nodeOf(pk);              // a daytime destination, not a job
                a.workPlace = kNoPlace;
                a.workDoor = doorOf(pk);          // rests at the park, not the kerb
                a.departWork = 9.5 + 2.0 * unit(a.brain);         // late-morning out
                a.departHome = 15.5 + 1.5 * unit(a.brain >> 8);   // home mid-afternoon
            }
        } else if (!jobs.empty()) {
            // Prefer the brain-picked workplace; if it isn't routable from
            // home, scan for any that is — STARTING FROM A PER-AGENT OFFSET.
            // Scanning from index 0 funneled every fallback agent onto the
            // FIRST routable workplace: one office, one street, the whole
            // fleet parked in a line (Glenn's "cars all on one street").
            // WORKPLACE CHOICE IS GRAVITY-BIASED, not uniform. Offices, shops
            // and civic buildings concentrate downtown by district, so drawing
            // uniformly over them sent most of the city to the same few blocks
            // — a permanent downtown crush, and every agent inside the
            // player's bubble at once when he stands there. Real commuting
            // follows distance decay: sample a handful of candidates from the
            // agent's own bits and take the NEAREST routable one. Because the
            // candidates are random, long cross-town commutes still happen —
            // they just stop being the default.
            const Vec2 homePos = places[hp].site;
            PlaceId pick = kNoPlace;
            // SHORTER COMMUTES (Glenn, 2026-09-16: "the agents should have
            // shorter commutes") -- and it is NOT the same problem for both
            // modes. A walker covers ground ~5x slower, so the same home/work
            // pair costs it ~5x the day. Measured on metro_v2 after the first
            // pass: drivers 0.59 in-world hours each way, walkers 3.86.
            //
            //   ON FOOT  the nearest routable job in the CITY -- the 8 nearest
            //            by straight line, tested in order. Exact, not sampled.
            //   DRIVING  the nearest routable of 24 sampled jobs. The variety is
            //            deliberate: a city with cross-town car commutes reads
            //            better than one where everybody works next door.
            //
            // Keyed on ARCHETYPE, not mode: mode flips when a driver parks and
            // walks away, and archetype is what measureCommute samples -- branch
            // on the wrong one and the metric never moves.
            // BUS COMMUTERS (Glenn, 2026-09-23: "regional buses ... that go between towns").
            // Nobody on foot could reach another town, so a regional bus would run empty.
            // A level-set share of walkers (their own bits, no rng draw) work in ANOTHER
            // street network -- the city from a town, a town from the city -- at the
            // nearest of 24 sampled jobs there that the buses can take them to and back.
            // On the day they walk to a stop, ride, change, and walk the last stretch.
            if (a.archetype == Agent::Mode::Pedestrian && busCommuteShare_ > 0 && buses_.hasRegional()) {
                uint32_t bc = a.brain * 0x85EBCA6Bu;
                bc ^= bc >> 13; bc *= 0xC2B2AE35u; bc ^= bc >> 16;
                if (static_cast<Real>(bc & 0x3FF) < busCommuteShare_ * 1024.0) {
                    ++busCommuteTried;
                    const int homeNet = buses_.networkOf(hn);
                    const Vec2 homeAt = graph.nodes[static_cast<std::size_t>(hn)];
                    std::vector<std::pair<Real, PlaceId>> far;
                    for (int c = 0; c < 24; ++c) {
                        uint32_t hh = bc + static_cast<uint32_t>(c) * 0x9E3779B9u;
                        hh ^= hh >> 16; hh *= 0x7feb352dU; hh ^= hh >> 15;
                        const PlaceId cand = jobs[hh % jobs.size()];
                        const int net = buses_.networkOf(nodeOf(cand));
                        if (net < 0 || net == homeNet) continue;
                        far.push_back({(places[cand].site - homePos).lengthSquared(), cand});
                    }
                    std::sort(far.begin(), far.end());
                    for (const auto& fc : far) {
                        const Vec2 jobAt = graph.nodes[static_cast<std::size_t>(nodeOf(fc.second))];
                        if (buses_.planTrip(homeAt, jobAt, busMaxWalk_).valid() &&
                            buses_.planTrip(jobAt, homeAt, busMaxWalk_).valid()) {
                            pick = fc.second;
                            ++busCommuters;
                            break;
                        }
                    }
                }
            }
            if (pick != kNoPlace) {
                // a bus commuter: chosen above
            } else if (a.archetype == Agent::Mode::Pedestrian) {
                // A WALK WORTH TAKING (Glenn, 2026-09-17: "I haven't seen
                // anybody in the suburbs"). Taking the NEAREST job put walkers
                // ~70 m from home, so they reached work almost at once and sat
                // indoors: measured, moving fell 847 -> 377 at 17:00 and
                // 332 -> 76 at noon. What fills a street is not headcount but
                // the SHARE OF THE DAY spent walking -- at a 5% outdoor share
                // you need ~70k walkers for the street life 12k gives at 30%.
                // So aim for a real commute: the nearest routable job at least
                // kWalkJobFloor out, and only if none of those route does the
                // nearest of all win.
                constexpr Real kWalkJobFloor = 300.0;   // metres, straight line
                constexpr Real kWalkJobCeil  = 900.0;   // "a local place to work"
                jobDist.clear();
                jobDist.reserve(jobs.size());
                for (PlaceId cand : jobs) {
                    const Vec2 d = places[cand].site - homePos;
                    jobDist.push_back({d.x * d.x + d.y * d.y, cand});
                }
                const auto byDist = [](const std::pair<Real, PlaceId>& x,
                                       const std::pair<Real, PlaceId>& y) {
                    if (x.first != y.first) return x.first < y.first;
                    return x.second < y.second;   // deterministic ties
                };
                // [begin, split) are far enough to be a commute; [split, end) are not.
                // BAND, not a floor. The floor alone stopped walkers working
                // 70 m away and emptying the streets; a ceiling stops them
                // crossing town. Rank: inside the band first, then beyond it,
                // then nearer than it -- nearest first within each group.
                const auto split = std::stable_partition(
                    jobDist.begin(), jobDist.end(),
                    [&](const std::pair<Real, PlaceId>& e) {
                        return e.first >= kWalkJobFloor * kWalkJobFloor &&
                               e.first <= kWalkJobCeil * kWalkJobCeil;
                    });
                const std::size_t far =
                    static_cast<std::size_t>(split - jobDist.begin());
                const std::size_t takeFar = std::min<std::size_t>(8, far);
                std::partial_sort(jobDist.begin(), jobDist.begin() + takeFar, split, byDist);
                for (std::size_t c = 0; c < takeFar && pick == kNoPlace; ++c)
                    if (commutable(hn, nodeOf(jobDist[c].second))) pick = jobDist[c].second;
                if (pick == kNoPlace) {   // nothing beyond the floor routes
                    const std::size_t near =
                        static_cast<std::size_t>(jobDist.end() - split);
                    const std::size_t takeNear = std::min<std::size_t>(8, near);
                    std::partial_sort(split, split + takeNear, jobDist.end(), byDist);
                    for (std::size_t c = 0; c < takeNear && pick == kNoPlace; ++c)
                        if (commutable(hn, nodeOf((split + c)->second)))
                            pick = (split + c)->second;
                }
            } else {
                const int kCandidates = 24;
                std::pair<Real, PlaceId> cands[kCandidates];
                int nc = 0;
                for (int c = 0; c < kCandidates; ++c) {
                    uint32_t h = a.brain + static_cast<uint32_t>(c) * 0x9E3779B9u;
                    h ^= h >> 16; h *= 0x7feb352dU; h ^= h >> 15;
                    PlaceId cand = jobs[h % jobs.size()];
                    const Vec2 d = places[cand].site - homePos;
                    cands[nc++] = {d.x * d.x + d.y * d.y, cand};
                }
                // Ties break on place id so the pick cannot depend on draw order.
                std::sort(cands, cands + nc, [](const std::pair<Real, PlaceId>& x,
                                                const std::pair<Real, PlaceId>& y) {
                    if (x.first != y.first) return x.first < y.first;
                    return x.second < y.second;
                });
                // CROSS-TOWN COMMUTERS (Glenn, 2026-09-22: "I'd like more traffic on the
                // freeway"). Nearest-of-24 keeps car commutes short — measured on
                // metro_lanes, 19 of 2279 drivers lived more than 1.5 km from work, and
                // a ring freeway only pays on a trip across town. A level-set share of
                // drivers (their own brain bits, no rng draw) instead takes the nearest
                // sampled job at least kLongCommute away — the outskirts-to-downtown
                // commute — and failing that the farthest one that routes.
                constexpr Real kLongCommute = 1800.0;
                uint32_t ct = a.brain * 0x9E3779B9u;   // own bits, decorrelated from the role roll
                ct ^= ct >> 15; ct *= 0x2c1b3c6dU; ct ^= ct >> 12;
                const bool crossTown =
                    longCommuteShare_ > 0 &&
                    static_cast<Real>(ct & 0x3FF) < longCommuteShare_ * 1024.0;
                if (crossTown) {
                    ++crossTownDrivers;
                    for (int c = 0; c < nc && pick == kNoPlace; ++c)
                        if (cands[c].first >= kLongCommute * kLongCommute &&
                            commutable(hn, nodeOf(cands[c].second)))
                            pick = cands[c].second;
                    for (int c = nc - 1; c >= 0 && pick == kNoPlace; --c)
                        if (commutable(hn, nodeOf(cands[c].second))) pick = cands[c].second;
                }
                for (int c = 0; c < nc && pick == kNoPlace; ++c)
                    if (commutable(hn, nodeOf(cands[c].second))) {
                        pick = cands[c].second;
                        break;
                    }
            }
            if (pick == kNoPlace) {   // none of the samples routed: widen out
                const std::size_t start =
                    (a.brain * 2654435761u) % jobs.size();
                for (std::size_t k = 0; k < jobs.size(); ++k) {
                    PlaceId cand = jobs[(start + k) % jobs.size()];
                    if (commutable(hn, nodeOf(cand))) { pick = cand; break; }
                }
            }
            if (pick != kNoPlace) {
                a.workPlace = pick;
                a.work = nodeOf(pick);
                a.workDoor = doorOf(pick);
                if (a.archetype == Agent::Mode::Driver) {
                    ++driversWithJobs;
                    driverCommute += (places[pick].site - homePos).length();
                }
                {   // An errand stop: the nearest shop to HOME that is routable
                    // from work, so the last leg home is short. Deterministic:
                    // nearest first, ties by place id.
                    // A store or a supermarket: the errand on the way home.
                    std::vector<PlaceId> shops = places.ofType(PlaceType::Shop);
                    for (PlaceId sp : places.ofType(PlaceType::Supermarket)) shops.push_back(sp);
                    Real bestD2 = 1e30;
                    for (PlaceId sp : shops) {
                        const Vec2 d = places[sp].site - places[hp].site;
                        const Real d2 = d.x * d.x + d.y * d.y;
                        if (d2 >= bestD2) continue;
                        const int sn = nodeOf(sp);
                        if (sn == a.work || sn == hn) continue;
                        // work -> shop -> home, with home -> work already there and back, closes a cycle: the
                        // shop is in their component, exactly. (A job the car graph does not join to home --
                        // a bus commuter's -- still asks the router.)
                        if (sameComp(a.work, hn)) {
                            if (!sameComp(a.work, sn)) continue;
                        } else {
                            if (!engine::findRoute(graph, a.work, sn).valid()) continue;
                            if (!engine::findRoute(graph, sn, hn).valid()) continue;
                        }
                        bestD2 = d2; a.shop = sn; a.shopPlace = sp; a.shopDoor = doorOf(sp);
                    }
                }
                {   // What this commute COSTS, in seconds (see Agent::commuteSeconds).
                    const bool onFoot = a.archetype == Agent::Mode::Pedestrian;
                    const engine::Route rt = engine::findRoute(graph, hn, a.work, onFoot);
                    Real secs = 0;
                    for (int li : rt.links) {
                        const engine::NavLink& L = graph.links[li];
                        const Real v = onFoot ? kWalkSpeed : engine::classSpeed(L.klass);
                        secs += L.length / std::max(Real(0.1), v * a.speedFactor);
                    }
                    a.commuteSeconds = secs;
                }
                const PlaceType wt = places[pick].type;
                if (wt == PlaceType::Shop || wt == PlaceType::Cafe ||
                    wt == PlaceType::Restaurant || wt == PlaceType::Supermarket) {
                    const Real open = places[pick].openHour;
                    const Real close = places[pick].closeHour;
                    // ONLY when the shop actually authored hours. A place minted
                    // from a LotBuilding carries the wide-open default (0..24,
                    // components.h:373), and running that through the formula
                    // below yields departWork 0.0 / departHome 24.0 — a window
                    // that is open every hour of the day. Such an agent departs
                    // for work once and is never again outside its work window,
                    // so it never goes home and its car never comes back. On
                    // piedmont, where every place is lot-derived, that was every
                    // shop worker in the city. Without authored hours it simply
                    // keeps the shift it drew at build.
                    const bool authored = (open > 0.0 || close < 24.0) &&
                                          places[pick].authoredHours;
                    const Real dw = open > 0.5 ? open - 0.5 : 0.0;
                    const Real dh = close < 23.5 ? close + 0.5 : 24.0;
                    if (authored && dw != dh) {
                        // Open the shop before it opens, close it after.
                        a.role = Agent::Role::Shopkeeper;
                        a.departWork = dw;
                        a.departHome = dh;
                    }
                }
                // else Commuter — keep the jittered office hours from build().
            }
        }
    }

    // STUDENTS are decided with everyone else, so a cache hit restores them too.
    assignStudents(places, graph);

    measureCommute(graph);
    PopTail tail;
    std::memset(&tail, 0, sizeof tail);
    tail.crossTownDrivers = crossTownDrivers; tail.driversWithJobs = driversWithJobs;
    tail.busCommuters = busCommuters; tail.busCommuteTried = busCommuteTried;
    tail.driverCommute = driverCommute; tail.medianAll = commuteSecondsMedian_;
    tail.medianDrive = commuteSecondsDrive_; tail.medianWalk = commuteSecondsWalk_;
    if (popCache_.hit && verify) {   // decided again: does the cache agree, field for field?
        int bad = 0;
        for (std::size_t i = 0; i < agents_.size(); ++i) {
            const PopRecord now = record(agents_[i]);
            if (std::memcmp(&cached[i], &now, sizeof(PopRecord)) != 0) ++bad;
        }
        if (std::memcmp(&cachedTail, &tail, sizeof tail) != 0) ++bad;
        popCache_.mismatches = bad;
        std::fprintf(stderr, "[citysim] population cache VERIFY: %d of %zu records differ\n", bad, agents_.size());
    } else if (popCache_.used) {
        std::error_code ec;
        std::filesystem::create_directories(populationCacheDir_, ec);
        const std::string tmp = popPath + ".tmp";
        std::ofstream out(tmp, std::ios::binary);
        const uint32_t fmt = kPopulationFormat, count = static_cast<uint32_t>(agents_.size());
        out.write("RTPOP1", 6);
        out.write(reinterpret_cast<const char*>(&fmt), 4);
        out.write(reinterpret_cast<const char*>(&popCache_.key), 8);
        out.write(reinterpret_cast<const char*>(&count), 4);
        for (const Agent& a : agents_) { const PopRecord r = record(a); out.write(reinterpret_cast<const char*>(&r), sizeof r); }
        out.write(reinterpret_cast<const char*>(&tail), sizeof tail);
        out.close();
        if (out) { std::filesystem::rename(tmp, popPath, ec); popCache_.saved = !ec; }
    }
    }   // decided (not read from the cache)
    students_ = 0;
    for (const Agent& a : agents_) students_ += a.role == Agent::Role::Student ? 1 : 0;
    tagSpots();   // the venues are known now: which spots are on the campus

    // Seed the surface-level social graph: agents sharing a workplace are
    // coworkers; those sharing a home are neighbors (housemates).
    //
    // Only agents who share a PLACE can be related, so bucket by place and pair
    // within a bucket instead of testing every pair in the city against every
    // other. The old double loop was ~11.5 M pair tests on piedmont — paid on
    // every single level load — to discover a comparatively tiny number of
    // bonds. Emission order is preserved exactly: for each agent in ascending
    // index, its partners in ascending index, with the same work-beats-home
    // precedence, so the resulting table is identical.
    {
        std::unordered_map<PlaceId, std::vector<int>> byWork, byHome;
        for (std::size_t i = 0; i < agents_.size(); ++i) {
            const Agent& A = agents_[i];
            if (A.workPlace != kNoPlace)
                byWork[A.workPlace].push_back(static_cast<int>(i));
            if (A.homePlace != kNoPlace)
                byHome[A.homePlace].push_back(static_cast<int>(i));
        }
        std::vector<int> partners;
        for (std::size_t i = 0; i < agents_.size(); ++i) {
            const Agent& A = agents_[i];
            partners.clear();
            // Buckets were filled in ascending index, so the tail past `i` is
            // already ascending; merging two such tails and sorting reproduces
            // the inner loop's visit order over exactly the pairs that matter.
            auto tailOf = [&](std::unordered_map<PlaceId, std::vector<int>>& m,
                              PlaceId key) {
                if (key == kNoPlace) return;
                auto it = m.find(key);
                if (it == m.end()) return;
                for (int j : it->second)
                    if (j > static_cast<int>(i)) partners.push_back(j);
            };
            tailOf(byWork, A.workPlace);
            tailOf(byHome, A.homePlace);
            std::sort(partners.begin(), partners.end());
            partners.erase(std::unique(partners.begin(), partners.end()),
                           partners.end());
            for (int j : partners) {
                const Agent& B = agents_[j];
                if (A.workPlace != kNoPlace && A.workPlace == B.workPlace)
                    relationships_.set(A.uid, B.uid, Relationship::Coworker);
                else if (A.homePlace != kNoPlace && A.homePlace == B.homePlace)
                    relationships_.set(A.uid, B.uid, Relationship::Neighbor);
            }
        }
    }
    commuteStats_.crossTownDrivers = crossTownDrivers;
    commuteStats_.busCommuters = busCommuters;
    commuteStats_.busCommuteTried = busCommuteTried;
    if (busCommuteTried) buses_.resetPlanStats();   // the choosing is not ridership
    commuteStats_.driversWithJobs = driversWithJobs;
    commuteStats_.meanDriverCommute = driversWithJobs ? driverCommute / driversWithJobs : 0;
}

// MEDIAN COMMUTE, in sim-seconds. Sampled (every 64th commuter) because each
// probe is a fresh uncached A*.
//
// Two things read it. The host multiplies it by hoursPerSecond to find what a
// commute COSTS in in-world hours — the number that decides whether an agent's
// day is livable at all, since a clock running 180x real time turns a 3 km
// drive into 12.5 in-world hours. And scheduleSnapshot uses it to decide how
// long after a departure boundary an agent is still on the road.
//
// Measured at build (where home/work are first assigned) and again after
// assignPlaces moves work to a real building, so it is never stale.
void CitySim::measureCommute(const NavGraph& graph) {
    // SPLIT BY MODE. Drivers occupy the low indices (build assigns
    // 0..driverCount-1), so a single stride-64 sample over the whole array is
    // driver-dominated and a single median hides the walkers completely. That
    // matters: at kWalkSpeed a kilometre-scale home/work pair is an hours-long
    // journey in world time, so a population can be perfectly livable for its
    // drivers and impossible for everyone on foot.
    std::vector<Real> driveSamples, walkSamples;
    for (std::size_t i = 0; i < agents_.size(); i += 16) {
        const Agent& a = agents_[i];
        if (a.home == a.work) continue;
        const bool onFoot = a.archetype == Agent::Mode::Pedestrian;
        const engine::Route r = engine::findRoute(graph, a.home, a.work, onFoot);
        if (!r.valid()) continue;
        Real secs = 0;
        for (int li : r.links) {
            const engine::NavLink& L = graph.links[li];
            const Real v = onFoot ? kWalkSpeed : engine::classSpeed(L.klass);
            secs += L.length / std::max(Real(0.1), v * a.speedFactor);
        }
        (onFoot ? walkSamples : driveSamples).push_back(secs);
    }
    auto median = [](std::vector<Real>& v) -> Real {
        if (v.empty()) return 0;
        std::sort(v.begin(), v.end());
        return v[v.size() / 2];
    };
    commuteSecondsDrive_ = median(driveSamples);
    commuteSecondsWalk_ = median(walkSamples);
    // The blended figure scheduleSnapshot uses stays the whole-population
    // median, since it places both kinds of agent.
    std::vector<Real> all;
    all.insert(all.end(), driveSamples.begin(), driveSamples.end());
    all.insert(all.end(), walkSamples.begin(), walkSamples.end());
    const Real m = median(all);
    if (m > 0) commuteSecondsMedian_ = m;
}

// Where an agent's own day puts it at in-world hour `clock`. See the header for
// why this takes the clock as a parameter instead of reading the sim's.
//
// The model is the one goalThink already runs, read forwards instead of ticked:
// inside [departWork, departHome) the day says "be at work", outside it "be at
// home", and the agent is TRAVELLING for as long as the journey takes after each
// boundary. Travel time comes from the measured median rather than this agent's
// own route, which is deliberate — the point is to place a population without
// paying an A* per agent. A seeded agent is approximately right immediately and
// exactly right as soon as it takes its next real trip.
CitySim::Snapshot CitySim::scheduleSnapshot(const Agent& a, Real clock) const {
    Snapshot s;
    // No commute: it is always home. Also the stranded case (work == home),
    // which goalThink short-circuits in exactly the same way.
    if (a.home == a.work) return s;

    const Real dwHour = departWorkHour(a);
    const bool atWorkWindow = inWindow(clock, dwHour, a.departHome);
    // Hours since the boundary that put us in this half of the day.
    const Real since = std::fmod(
        clock - (atWorkWindow ? dwHour : a.departHome) + 24.0, 24.0);
    const Real travelHours =
        commuteSecondsMedian_ * (hoursPerSecond_ > 0 ? hoursPerSecond_ : 0.0);

    if (since < travelHours) {
        // Still on the road: under way for `since`, heading to whichever end
        // this half of the day belongs to.
        s.where = atWorkWindow ? Snapshot::Where::ToWork : Snapshot::Where::ToHome;
        s.elapsedSeconds =
            hoursPerSecond_ > 0 ? since / hoursPerSecond_ : 0.0;
        return s;
    }
    s.where = atWorkWindow ? Snapshot::Where::AtWork : Snapshot::Where::AtHome;
    return s;
}

// SEED THE CITY AT AN HOUR instead of stepping up to it.
//
// The host used to run 400 full-fidelity steps purely to get the population
// moving before the level appeared. That is expensive at city scale, and it
// cannot express an arbitrary start: opening at 22:00 would mean simulating 27
// in-world hours. Here every agent is placed directly from its own schedule —
// at home, at work, or a measured distance along its commute.
//
// Ascending agent order, and every decision comes from the agent's own state or
// its own rng stream, so the result is deterministic and independent of how the
// population is ordered relative to anything else.
void CitySim::seedFromSchedule(Real clock) {
    if (!nav_ || agents_.empty()) return;
    clockHours_ = std::fmod(clock, 24.0);
    if (clockHours_ < 0) clockHours_ += 24.0;

    for (std::size_t i = 0; i < agents_.size(); ++i)
        placeFromSchedule(static_cast<int>(i));
    rehashAll_ = true;   // everyone moved
}

// Put ONE agent where its day says it should be, right now. Shared by the
// load-time seeding above and by waking a dormant agent, so a city that was
// seeded and a city that was simulated place an agent by identical rules —
// there is no second, subtly different reconstruction to drift apart.
void CitySim::placeFromSchedule(int idx) {
    if (!nav_ || idx < 0 || idx >= static_cast<int>(agents_.size())) return;
    {
        const std::size_t i = static_cast<std::size_t>(idx);
        Agent& a = agents_[i];
        if (a.playerControlled || a.released) return;
        const Snapshot s = scheduleSnapshot(a, clockHours_);
        const bool travelling = s.where == Snapshot::Where::ToWork ||
                                s.where == Snapshot::Where::ToHome;
        const Agent::Activity want =
            s.where == Snapshot::Where::AtHome   ? Agent::Activity::AtHome
            : s.where == Snapshot::Where::AtWork ? Agent::Activity::AtWork
            : s.where == Snapshot::Where::ToWork ? Agent::Activity::Commuting
                                                 : Agent::Activity::Returning;
        // A BUS HAS NO DAY. Re-seating works by finding the table state wearing
        // the schedule's activity label -- and a service vehicle's table has
        // states wearing those labels for its own reasons: OffDuty is
        // Activity::AtHome. So any clock jump that said "this agent is at home
        // now" parked the ENTIRE FLEET off-duty, permanently, and no bus moved
        // again (measured: 0% moving, 0.0 m over 91 samples, every bus goal=2).
        // A bus or a cab runs its own table from its own entry; a commuter's
        // schedule has nothing to say about it.
        {
            const int who = indexOf(a);
            if (isBus(who) || isTaxi(who)) {
                a.goal = tableFor(a).entry();
                a.goalHours = 0;
                a.wakeAt = -1;
                // A cab waiting for its first fare waits IN A BAY near where it
                // stands -- idle cabs were the heap left at the corners on load.
                if (isTaxi(who) && !a.moving && a.car >= 0 &&
                    a.car < static_cast<int>(vehicles_.size()) && a.restNode >= 0 &&
                    a.restNode < nav_->nodeCount()) {
                    releaseBays(a);
                    const int bay = claimBayNear(
                        nav_->nodes[static_cast<std::size_t>(a.restNode)], who, kParkSearch);
                    if (bay >= 0) {
                        const ParkingBay& b = bays_[static_cast<std::size_t>(bay)];
                        a.parkedBay = bay;
                        a.pos = b.pos;
                        a.heading = b.heading;
                        a.restNode = nav_->links[static_cast<std::size_t>(b.link)].to;
                        vehicles_[static_cast<std::size_t>(a.car)].pos = a.pos;
                        vehicles_[static_cast<std::size_t>(a.car)].heading = a.heading;
                        parkedGrid_.place(a.car, a.pos);
                        grid_.place(who, a.pos);
                    }
                }
                return;
            }
        }
        // Seat the agent on the table state wearing that label (the same
        // remapping installGoalTables uses), so its next transition is the one
        // its day actually calls for.
        const GoalTable& t = tableFor(a);
        int goal = -1;
        for (int st = 0; st < t.stateCount(); ++st)
            if (t.state(st).activity == want) { goal = st; break; }
        if (goal < 0) return;   // this table has no such state: leave as built
        a.goal = goal;
        a.goalHours = 0;
        a.activity = want;
        a.wakeAt = -1;
        // AT WORK SINCE WHEN: the morning at work ends in lunch (a dwell), so a
        // city opened at 12:30 must know its workers arrived hours ago -- with
        // the clock started at the seed, every lunch fell after the shift and
        // nobody went out.
        if (s.where == Snapshot::Where::AtWork) {
            Real since = std::fmod(clockHours_ - departWorkHour(a) + 48.0, 24.0) -
                         a.commuteSeconds * hoursPerSecond_;
            a.goalHours = std::max(Real(0), since);
            // Lunch long over (the morning's dwell plus the break): the
            // afternoon, not a lunch rush of everyone at once at load.
            const GoalState& gs = t.state(goal);
            if (gs.dwellHours > 0 && since > gs.dwellHours + 0.6) {
                for (int st = 0; st < t.stateCount(); ++st)
                    if (t.state(st).name == "AtWorkPM") { a.goal = st; a.goalHours = 0; break; }
            }
        }

        if (!travelling) {
            // At one end of the day. Rest there, and park the car with it.
            const int node = s.where == Snapshot::Where::AtWork ? a.work : a.home;
            a.restNode = node;
            a.moving = false;
            a.speed = 0;
            a.route.links.clear();
            a.leg = 0;
            a.distOnLeg = 0;
            a.elevation = 0;
            a.pos = idlePose(node, a.mode, a.brain);
            if (!nav_->outLinks[node].empty())
                a.heading = nav_->direction(nav_->outLinks[node][0]);
            // A WALKER AT ONE END OF ITS DAY IS INSIDE, at the door -- not
            // standing on the sidewalk by the node (the seeded crowd that stood
            // around all day). Someone whose "work" is outdoors (a stroller's
            // park) stays out.
            if (a.mode == Agent::Mode::Pedestrian) {
                const bool atWorkEnd = s.where == Snapshot::Where::AtWork;
                if (!atWorkEnd && a.homePlace != kNoPlace) {
                    a.pos = a.homeDoor;
                    a.indoors = true;
                } else if (atWorkEnd && a.workPlace != kNoPlace) {
                    a.pos = a.workDoor;
                    a.indoors = true;
                } else {
                    a.indoors = false;
                }
            }
            releaseBays(a);
            a.tripGoal = node;
            bool offStreet = false;
            if (a.mode == Agent::Mode::Driver && parksInBays(a)) {
                // In a MARKED BAY near home or work -- the one allocator every
                // parked car goes through -- or, with none free, off-street.
                const int bay = claimBayNear(nav_->nodes[static_cast<std::size_t>(node)],
                                             indexOf(a), kParkSearch);
                if (bay >= 0) {
                    const ParkingBay& b = bays_[static_cast<std::size_t>(bay)];
                    a.parkedBay = bay;
                    a.pos = b.pos;
                    a.heading = b.heading;
                    a.restNode = nav_->links[static_cast<std::size_t>(b.link)].to;
                } else if (!bays_.empty()) {
                    offStreet = true;   // a parking city with none free near
                }
            }
            if (a.mode == Agent::Mode::Driver && a.car >= 0 &&
                a.car < static_cast<int>(vehicles_.size())) {
                vehicles_[a.car].pos = a.pos;
                vehicles_[a.car].heading = a.heading;
                vehicles_[a.car].offStreet = offStreet;
                parkedGrid_.place(a.car, a.pos);
            }
        } else {
            // Under way. Launch the real trip, then run the far-tier advance for
            // as long as the schedule says it has been travelling — the same
            // primitive that catches a distant agent up when it is promoted, so
            // a seeded agent and a simulated one move by identical rules.
            const int from = s.where == Snapshot::Where::ToWork ? a.home : a.work;
            const int to = s.where == Snapshot::Where::ToWork ? a.work : a.home;
            releaseBays(a);   // under way: not pulling out of a bay it was seeded in
            a.restNode = from;
            startTrip(a, from, to, /*fromRest=*/true);
            if (a.moving && s.elapsedSeconds > 0) vAdvance(a, s.elapsedSeconds);
        }
        grid_.place(static_cast<int>(i), a.pos);
        a.vLastTick = simSeconds_;
        a.tickFromPos = a.pos;      // re-seeded: placed, not moved
        a.tickFromHeading = a.heading;
    }
}

// Bring a dormant agent back. Its stored position is however many world-hours
// stale, so it is not resumed — it is REBUILT from its schedule, by the same
// function that seeds the city at load.
//
// It wakes to V, never straight to K. Waking is an approximation (the snapshot
// places it from the population's median commute, not its own), and V is
// precisely the tier whose job is to be approximate; by the time the player is
// close enough for the difference to matter, the promote pass has already run
// its catch-up and handed over an exact lane pose.
void CitySim::pinAgent(int i, bool on) {
    if (i < 0 || i >= static_cast<int>(agents_.size())) return;
    agents_[static_cast<std::size_t>(i)].pinned = on;
    const auto it = std::lower_bound(pinned_.begin(), pinned_.end(), i);
    const bool has = it != pinned_.end() && *it == i;
    if (on && !has) pinned_.insert(it, i);
    if (!on && has) pinned_.erase(it);
}

void CitySim::rebuildTierLists() {
    kIdx_.clear();
    vIdx_.clear();
    for (int i = 0; i < static_cast<int>(agents_.size()); ++i) {
        const Agent::Tier t = agents_[static_cast<std::size_t>(i)].tier;
        if (t == Agent::Tier::K) kIdx_.push_back(i);
        else if (t == Agent::Tier::V) vIdx_.push_back(i);
    }
}

void CitySim::setTier(int i, Agent::Tier t) {
    Agent& a = agents_[static_cast<std::size_t>(i)];
    if (a.tier == t) return;
    auto drop = [&](std::vector<int>& v) {
        const auto it = std::lower_bound(v.begin(), v.end(), i);
        if (it != v.end() && *it == i) v.erase(it);
    };
    auto add = [&](std::vector<int>& v) { v.insert(std::lower_bound(v.begin(), v.end(), i), i); };
    if (a.tier == Agent::Tier::K) drop(kIdx_);
    else if (a.tier == Agent::Tier::V) { drop(vIdx_); vGrid_.remove(i); }
    else dGrid_.remove(i);
    a.tier = t;
    if (t == Agent::Tier::D) {
        releaseSeat(a);   // a bench, a place in a group: let go (it was holding them for ever)
        // DORMANT: its sensing memory goes back to the heap. Its TRIP stays as it was (frozen until it is rebuilt
        // from its schedule on waking): clearing it set a mid-trip driver "not moving" with its car left in the
        // road, which everything that counts parked cars then saw as parked in a heap
        // (metro_cars_park_in_spaces_not_heaps: 27 on the verge, 24 stacked).
        a.memory.release();
    }
    if (t == Agent::Tier::K) add(kIdx_);
    else if (t == Agent::Tier::V) { add(vIdx_); vGrid_.place(i, a.pos); }
    else dGrid_.place(i, a.pos);
    // entering the drawn and stepped tiers: interpolate from here, not from where it was last listed
    if (t != Agent::Tier::D) {
        a.tickFromPos = a.pos;
        a.tickFromHeading = a.heading;
    }
}

void CitySim::wakeDormant(int idx) {
    if (idx < 0 || idx >= static_cast<int>(agents_.size())) return;
    Agent& a = agents_[static_cast<std::size_t>(idx)];
    placeFromSchedule(idx);
    setTier(idx, Agent::Tier::V);
    a.vLastTick = simSeconds_;   // its next coarse tick advances from now
    a.vHold = 0;
    // Any wake time it was carrying belongs to a day that has since moved on.
    a.wakeAt = -1;
}

Vec2 CitySim::pushPoseClearOfLanes(Vec2 p, Real margin) const {
    if (!nav_) return p;
    for (int guard = 0; guard < 6; ++guard) {
        Real worst = 0;
        Vec2 away(0, 0);
        for (int li = 0; li < nav_->linkCount(); ++li) {
            const engine::NavLink& L = nav_->links[li];
            if (L.layer != 0 || L.elevAbsolute) continue;   // decks don't block ground
            const Vec2& a = nav_->nodes[L.from];
            const Vec2& b = nav_->nodes[L.to];
            const Vec2 ab = b - a;
            const Real len2 = ab.lengthSquared();
            Real t = len2 > 1e-12 ? dot(p - a, ab) / len2 : 0.0;
            t = t < 0 ? 0 : (t > 1 ? 1 : t);
            const Vec2 q(a.x + ab.x * t, a.y + ab.y * t);
            const Real d = (p - q).length();
            const Real need = L.width * 0.5 + margin;
            if (need - d > worst) {
                worst = need - d;
                away = d > 1e-6 ? (p - q) * (1.0 / d) : Vec2(1, 0);
            }
        }
        if (worst <= 0) break;
        p = p + away * (worst + 0.15);
    }
    return p;
}

Vec2 CitySim::idlePose(int node, Agent::Mode mode, uint32_t brain) const {
    if (!nav_ || node < 0 || node >= nav_->nodeCount()) return Vec2();
    const std::vector<int>& out = nav_->outLinks[node];
    if (out.empty()) return nav_->nodes[node];
    int li = out[0];
    Vec2 dir = nav_->direction(li);
    Vec2 right(dir.y, -dir.x);
    Real hw = nav_->links[li].width * 0.5;
    // Both idle OFF the carriageway: a pedestrian just beyond the kerb, a driver
    // parked on the verge (ADR-0062 — an idle car is a PHYSICAL body now; parked
    // in lane 0 it blocked the road until its first trip).
    Real off = (mode == Agent::Mode::Driver) ? hw + 2.8 : hw + 1.2;
    if (off < 0.5) off = 0.5;
    // Per-agent setback ALONG the road (mirrors arrival parking): several agents
    // sharing a home node must not stack in one spot — two cars occupying the
    // same verge pose spawned interpenetrating the moment both departed.
    Real back = 2.0 + parkSetbackSlot(brain) *
                          ((mode == Agent::Mode::Driver) ? 1.4 : 0.6);
    back = std::min(back, nav_->links[li].length * 0.45);
    Vec2 p = nav_->nodes[node] + dir * back + right * off;
    // An idle CAR is a physical body: keep it out of every carriageway, not
    // just this link's (short-link knots overlap their neighbours' verges).
    if (mode == Agent::Mode::Driver) p = pushPoseClearOfLanes(p, 1.3);
    return p;
}

// Pick a random routable wander goal from `from` and start the trip. Prefers a
// goal that doesn't begin by U-TURNING back up the link just arrived on (a
// standstill 180 flips the lane offset to the other side of the road — a
// visible pop); falls back to a U-turn when nothing else routes (dead ends).
// GET BACK IN. A car owner walks the last few metres of every arrival, so it is
// on foot when the next departure comes round; it drives again as soon as its
// own car is there to be driven. If the car is GONE — stolen by the player, or
// never owned in the first place — the agent stays on foot and walks the trip,
// which is the honest outcome rather than conjuring a car back under it.
//
// Called from BOTH trip starters. A wander trip builds its own route and would
// otherwise leave a dismounted driver walking the rest of the level.
void CitySim::remountOwnedCar(Agent& a) {
    if (a.archetype != Agent::Mode::Driver || a.vehicle >= 0) return;
    if (a.car < 0 || a.car >= static_cast<int>(vehicles_.size())) return;
    if (vehicles_[a.car].driver >= 0) return;   // someone else is in it
    a.vehicle = a.car;
    vehicles_[a.car].driver = static_cast<int>(&a - agents_.data());
    a.mode = Agent::Mode::Driver;
}

bool CitySim::startWanderTrip(Agent& a, int from, bool fromRest) {
    a.tripGoal = -1;   // a wander has no destination door
    const int n = nav_ ? nav_->nodeCount() : 0;
    if (n <= 1 || from < 0 || from >= n) return false;
    remountOwnedCar(a);
    auto reversesArrival = [&](const engine::Route& r) {
        if (a.arrivedLink < 0 || r.links.empty()) return false;
        const engine::NavLink& in = nav_->links[a.arrivedLink];
        const engine::NavLink& out = nav_->links[r.links.front()];
        return out.from == in.to && out.to == in.from;
    };
    // A DEAD END: the only way out is back along the link it arrived on, so every route reverses
    // it and the scan below would search the WHOLE graph (thousands of A* runs, a second of one
    // step in the morning rush) only to take its fallback -- the first valid goal. Take that
    // goal at once: the same trip, without the search.
    bool forcedUTurn = false;
    if (a.arrivedLink >= 0 && static_cast<std::size_t>(from) < nav_->outLinks.size()) {
        const engine::NavLink& in = nav_->links[static_cast<std::size_t>(a.arrivedLink)];
        forcedUTurn = true;
        for (int li : nav_->outLinks[static_cast<std::size_t>(from)]) {
            const engine::NavLink& L = nav_->links[static_cast<std::size_t>(li)];
            if (a.mode == Agent::Mode::Pedestrian &&
                (L.klass == engine::RoadClass::Freeway || L.klass == engine::RoadClass::Ramp || !L.walkable))
                continue;
            if (!(L.from == in.to && L.to == in.from)) { forcedUTurn = false; break; }
        }
    }
    // Scan EVERY node from a random start, so a non-reversing goal is found
    // whenever one exists — dice rolls occasionally picked only U-turn goals,
    // and each of those flipped the car to the other side of the road in place.
    // (A cap on the scan was tried: it brought those flips back.) The searches
    // price the U-turn link like startTrip's bay exit does, so a route takes
    // the way round the block whenever there is one and the first reachable
    // goal nearly always serves: one search, not thousands.
    int uturn = -1;
    if (a.arrivedLink >= 0 && static_cast<std::size_t>(a.arrivedLink) < twinOf_.size())
        uturn = twinOf_[static_cast<std::size_t>(a.arrivedLink)];
    if (uturn >= 0 && static_cast<std::size_t>(uturn) < departScale_.size()) departScale_[static_cast<std::size_t>(uturn)] = 50.0;
    else uturn = -1;
    struct RestoreScale {   // however the scan ends
        std::vector<Real>& s; int li; bool& priced;
        ~RestoreScale() { if (li >= 0) s[static_cast<std::size_t>(li)] = 1.0; priced = false; }
    } restore{departScale_, uturn, wanderPriced_};
    wanderPriced_ = uturn >= 0;
    int start = static_cast<int>(tripRnd(a) % static_cast<uint32_t>(std::max(1, nav_->streetNodeCount())));   // a street node
    int fallback = -1;
    // After the first unreachable goal, flood what IS reachable once and skip the rest without
    // searching: a failed A* explores everything reachable before it gives up, the costliest
    // search there is, and an agent on a scrap of network met dozens of them per trip.
    std::vector<char> reach;
    // ...and after a run of goals that are reachable only by turning back, flood once what lies AHEAD
    // (from `from` by any exit but the one it arrived on, never through `from` again) and search only
    // that: an agent whose other exit is a dead-end stub routed all 2313 nodes of island_8_saltwood
    // in one step (357 ms, the hitch Glenn felt driving the pass) to take its fallback.
    std::vector<char> ahead;
    int reversing = 0;
    auto floodAhead = [&] {
        std::vector<char> seen(static_cast<std::size_t>(n), 0);
        std::vector<int> stack;
        const bool walk = a.mode == Agent::Mode::Pedestrian;
        auto usable = [&](const engine::NavLink& L) {
            return !walk || (L.klass != engine::RoadClass::Freeway && L.klass != engine::RoadClass::Ramp && L.walkable);
        };
        const engine::NavLink* in = a.arrivedLink >= 0 ? &nav_->links[static_cast<std::size_t>(a.arrivedLink)] : nullptr;
        for (int li : nav_->outLinks[static_cast<std::size_t>(from)]) {
            const engine::NavLink& L = nav_->links[static_cast<std::size_t>(li)];
            if (!usable(L) || (in && L.from == in->to && L.to == in->from)) continue;
            if (!seen[static_cast<std::size_t>(L.to)]) { seen[static_cast<std::size_t>(L.to)] = 1; stack.push_back(L.to); }
        }
        seen[static_cast<std::size_t>(from)] = 1;   // never back through the start
        while (!stack.empty()) {
            const int v = stack.back(); stack.pop_back();
            for (int li : nav_->outLinks[static_cast<std::size_t>(v)]) {
                const engine::NavLink& L = nav_->links[static_cast<std::size_t>(li)];
                if (!usable(L) || seen[static_cast<std::size_t>(L.to)]) continue;
                seen[static_cast<std::size_t>(L.to)] = 1; stack.push_back(L.to);
            }
        }
        seen[static_cast<std::size_t>(from)] = 0;
        return seen;
    };
    const int nGoal = std::max(1, nav_->streetNodeCount());   // wander to street nodes, never onto a walk
    for (int k = 0; k < nGoal; ++k) {
        int goal = (start + k) % nGoal;
        if (goal == from) continue;
        if (!reach.empty() && !reach[static_cast<std::size_t>(goal)]) continue;
        if (!ahead.empty() && !ahead[static_cast<std::size_t>(goal)]) continue;
        engine::Route r = engine::findRoute(*nav_, from, goal,
                                            a.mode == Agent::Mode::Pedestrian,
                                            uturn >= 0 ? &departScale_ : nullptr);
        if (!r.valid()) {
            if (reach.empty()) reach = engine::reachableFrom(*nav_, from, a.mode == Agent::Mode::Pedestrian);
            continue;
        }
        if (reversesArrival(r) && !forcedUTurn) {
            if (fallback < 0) fallback = goal;
            if (++reversing == 16 && ahead.empty()) {
                ahead = floodAhead();
                if (std::find(ahead.begin(), ahead.end(), 1) == ahead.end()) break;   // nothing ahead: the fallback it is
                // every goal ahead HAS a way on without turning back, but a long one: at 50x the U-turn
                // still won A* each time (1176 searches in one step). Price it out; the next goal goes on.
                if (uturn >= 0) departScale_[static_cast<std::size_t>(uturn)] = 1e6;
            }
            if (reversing >= 64) break;   // bounded, whatever the graph: take the fallback
            continue;
        }
        startTrip(a, from, goal, fromRest);
        return a.moving;
    }
    if (fallback >= 0) {
        startTrip(a, from, fallback, fromRest);
        return a.moving;
    }
    return false;
}

// --- the GOAL layer (ADR-0064) ----------------------------------------------
// An agent's day is a GoalTable (city_goals.h): states that each bind one small
// C++ action, transitions on the events emitted below. The built-in tables
// mirror the historical schedule/wander control flow bit-exactly — same rng
// draws in the same order, same tick the transitions fire; a scripting build
// may replace them at load (setGoalTables), and every per-tick transition still
// executes right here, in C++.

// Where a rest departure starts from: the node the agent last arrived at, or
// its home before the first trip. Matches the historical schedule origins too —
// an agent resting AtHome has restNode == home, one AtWork has restNode == work.
int CitySim::departNode(const Agent& a) const {
    return (nav_ && a.restNode >= 0 && a.restNode < nav_->nodeCount()) ? a.restNode
                                                                       : a.home;
}

// Execute the agent's current (GoTo) goal state: start the trip toward its
// target. On failure the table's NoRoute row (if any) decides the fallback
// state — the historical "no path: fall back to the origin's resting state".
bool CitySim::startGoalTrip(Agent& a, int origin, bool fromRest) {
    const GoalState& s = tableFor(a).state(a.goal);
    bool started = false;
    // OUTING / LUNCH destinations are chosen HERE, before the bus check, so a
    // trip across town can take the bus like any other. A trip resumed after a
    // bus leg keeps the destination it chose (outingTo).
    int chosen = -1;
    const bool errandMenu = s.target == GoalTarget::Shop && catalog_.menu("errand");
    if (s.target == GoalTarget::Outing || s.target == GoalTarget::Lunch || s.target == GoalTarget::Campus ||
        s.target == GoalTarget::Activity || errandMenu) {
        if (a.outingTo >= 0) {
            chosen = a.outingTo;
        } else {
            chosen = s.target == GoalTarget::Outing ? pickOuting(a, origin)
                     : errandMenu ? pickFromMenu(a, origin, *catalog_.menu("errand"))
                     : s.target == GoalTarget::Lunch ? pickLunch(a, origin)
                     : s.target == GoalTarget::Campus ? pickCampusBreak(a, origin)
                     : !s.menu.empty() && catalog_.menu(s.menu) ? pickFromMenu(a, origin, *catalog_.menu(s.menu))
                                                       : pickActivity(a, origin, s);
            a.outingTo = chosen;
        }
    } else {
        a.tripVenue = -1;
        a.outingTo = -1;
    }

    // CATCH THE BUS. This lived in goalThink's "at a GoTo state and not moving"
    // branch, which MEASURED as the wrong place: walkers are never in that
    // state (1.46M goalThink entries, ZERO at a resting GoTo), because
    // tryGoalEvent transitions Rest -> GoTo and launches the trip in the same
    // tick. Only drivers linger there, blocked by launch clearance -- so the
    // bus was offered exclusively to agents who could not get their car out.
    //
    // startGoalTrip is the choke point EVERY trip passes through, however its
    // state was reached, which is where a mode choice belongs.
    const int busSelf = indexOf(a);
    if (a.mode == Agent::Mode::Pedestrian && nav_ && !buses_.empty() &&
        !isBus(busSelf) && !buses_.tripOf(busSelf) && !isTaxi(busSelf)) {
        const int to = chosen >= 0 ? chosen : goalNodeFor(a, s.target);
        if (to >= 0 && origin >= 0 && origin < nav_->nodeCount() &&
            to < nav_->nodeCount() && to != origin) {
            const Vec2 p0 = nav_->nodes[static_cast<std::size_t>(origin)];
            const Vec2 p1 = nav_->nodes[static_cast<std::size_t>(to)];
            BusTrip bt = buses_.planTrip(p0, p1, busMaxWalk_);
            if (bt.valid() && buses_.waitFor(busSelf, bt)) {
                const int stopNode =
                    buses_.route(bt.route)
                        .stops[static_cast<std::size_t>(bt.fromStop)].node;
                if (stopNode != origin) {
                    startTrip(a, origin, stopNode, fromRest);
                    if (a.moving) {
                        a.activity = s.activity;
                        return true;    // walking to the stop IS the trip now
                    }
                } else {
                    // ALREADY AT THE STOP -- the usual case after a change of bus at a
                    // shared stop. This fell through to walking the whole trip, which
                    // between towns there is no walk for: wait here (awaitingRide
                    // holds them).
                    a.indoors = false;
                    a.activity = s.activity;
                    return true;
                }
                buses_.stopWaiting(busSelf);   // could not reach the stop
            }
        }
    }
    switch (s.target) {
        case GoalTarget::Random:
            started = startWanderTrip(a, origin, fromRest);
            break;
        case GoalTarget::Outing:
        case GoalTarget::Lunch:
        case GoalTarget::Campus:
        case GoalTarget::Activity:
            if (chosen >= 0 && chosen != origin) {
                startTrip(a, origin, chosen, fromRest);
                started = a.moving;
            }
            if (!started) { a.outingTo = -1; a.tripVenue = -1; releaseSeat(a); }   // no trip: no spot held
            break;
        case GoalTarget::Work:
        case GoalTarget::Home:
            startTrip(a, origin, s.target == GoalTarget::Work ? a.work : a.home,
                      fromRest);
            started = a.moving;
            break;
        case GoalTarget::Shop:
            // the errand (the catalog's "errand" menu: a shop near home); the old fixed shop without one
            if (errandMenu) {
                if (chosen >= 0 && chosen != origin) {
                    startTrip(a, origin, chosen, fromRest);
                    started = a.moving;
                }
                if (!started) { a.outingTo = -1; a.tripVenue = -1; releaseSeat(a); }
            } else if (a.shop >= 0) {
                startTrip(a, origin, a.shop, fromRest);
                started = a.moving;
            }
            break;
        case GoalTarget::Depot: {
            const int self3 = indexOf(a);
            if (isBus(self3)) {
                const int r = busRoute_[static_cast<std::size_t>(self3)];
                if (r >= 0 && r < buses_.routeCount()) {
                    const int dn = buses_.route(r).depotNode;
                    if (dn >= 0) {
                        startTrip(a, origin, dn, fromRest);
                        started = a.moving;
                    }
                }
            }
            break;
        }
        case GoalTarget::Stop: {
            // The bus's NEXT stop. arriveOrChain advances the index before the
            // table is asked what comes next, so by the time this runs it
            // already names the stop after the one just served.
            const int self2 = indexOf(a);
            if (isBus(self2)) {
                const int r = busRoute_[static_cast<std::size_t>(self2)];
                const int si = busStop_[static_cast<std::size_t>(self2)];
                if (r >= 0 && r < buses_.routeCount()) {
                    const BusRoute& br = buses_.route(r);
                    if (si >= 0 && si < static_cast<int>(br.stops.size())) {
                        startTrip(a, origin, br.stops[static_cast<std::size_t>(si)].node,
                                  fromRest);
                        started = a.moving;
                        // A STOP IT CANNOT REACH IS SKIPPED. The index only
                        // advances on ARRIVAL, so without this a bus whose next
                        // stop is unroutable retries that same stop for ever:
                        // NoRoute -> Drive -> the same node -> NoRoute. It
                        // stalls mid-route and never serves the stops where
                        // people are waiting, which is exactly what the
                        // end-to-end test caught -- buses drove, riders waited,
                        // and nobody was ever picked up.
                        if (!started) {
                            busStop_[static_cast<std::size_t>(self2)] =
                                (si + 1) % static_cast<int>(br.stops.size());
                            ++busSkippedLegs_;
                        }
                    }
                }
            }
            break;
        }
        case GoalTarget::Fare:
        case GoalTarget::Drop: {
            // DYNAMIC targets: the node comes from the hail this driver was
            // assigned, not from a field on the agent. No fare -> no trip, and
            // the NoRoute row below puts the cab back on its cruise rather
            // than leaving it parked forever.
            const Fare* f = dispatch_.fareOf(indexOf(a));
            if (f) {
                const int node = s.target == GoalTarget::Fare ? f->pickup : f->dropoff;
                if (node >= 0) {
                    startTrip(a, origin, node, fromRest);
                    started = a.moving;
                }
            }
            break;
        }
        default:
            break;   // a GoTo with no target: nothing to do
    }
    if (started) {
        a.activity = s.activity;   // the day now reads as this state's label
        return true;
    }
    int next = tableFor(a).onEvent(a.goal, GoalEvent::NoRoute);
    if (next >= 0) {
        a.goal = next;
        a.goalHours = 0;
        a.activity = tableFor(a).state(next).activity;
    }
    return false;
}

// Deliver one event: take the first table row that matches the agent's state.
// A row into a GoTo state is launch-gated for drivers (a car departs only once
// its spawn area is clear of moving traffic — it waits out the traffic and the
// event simply fires again next tick, like anyone pulling out of a spot).
CitySim::GoalFire CitySim::tryGoalEvent(Agent& a, GoalEvent event) {
    const GoalTable& t = tableFor(a);
    int next = t.onEvent(a.goal, event);
    if (next < 0) return GoalFire::NoRow;
    const GoalState& to = t.state(next);
    if (to.action == GoalAction::GoTo) {
        int from = departNode(a);
        // A far (V) agent departs ungated: nobody is watching a spawn 600 m
        // out, and clearance against other far ghosts is meaningless.
        // ARCHETYPE, not mode. A driver that has parked is walking (mode ==
        // Pedestrian) right up until startTrip puts it back in its car, so
        // gating on `mode` skipped clearance for every driver after its first
        // arrival — which is to say, almost all of them. That reinstates the
        // "two cars spawn inside each other at a node" case launchClear exists
        // to prevent, and the fender-bender rule then freezes both in place.
        // A BUS IS NOT PULLING OUT OF A DRIVEWAY. launchClear stops a PARKED
        // car spawning into moving traffic; a bus leaving a stop is already on
        // the road, mid-route, and gating it there stalls the whole service --
        // measured: 24 buses served 13 stops in 400 s while 1404 riders stood
        // waiting, and the blocked-driver counter ticked 4187 times.
        if (a.archetype == Agent::Mode::Driver && !a.far() && !isBus(indexOf(a)) &&
            !launchClear(a, from))
            return GoalFire::Blocked;
        if (!isBus(indexOf(a)) && departuresThisStep_ >= kDeparturesPerStep)
            return GoalFire::Blocked;   // the step's routing budget is spent: leave next tick
        ++departuresThisStep_;
        a.goal = next;
        a.goalHours = 0;
        startGoalTrip(a, from, /*fromRest=*/true);
    } else {
        a.goal = next;
        a.goalHours = 0;
        a.activity = to.activity;
    }
    return GoalFire::Fired;
}

// step()'s pass 1 for one agent: run its current goal state. A GoTo state whose
// trip isn't running retries the departure each tick; a Rest state emits this
// tick's events in a fixed order — DwellDone, the clock-window depart, Idle —
// and the first row that fires (or blocks on launch clearance) wins.
void CitySim::goalThink(Agent& a, Real dtHours) {
    // WAKING. While asleep the agent was skipped entirely, so its dwell clock
    // stopped with it — catch it up before anything reads it.
    if (a.wakeAt >= 0) {
        a.goalHours += clockTotalHours_ - a.sleptAt;   // exact across rate changes
        a.wakeAt = -1;
    }
    const int self = indexOf(a);
    // Does a WALKER even get here? 1615 of them are moving, so they departed
    // somehow; if none of them passes through goalThink the departure happens
    // somewhere else entirely and the bus check is in the wrong place.
    if (a.archetype == Agent::Mode::Pedestrian) ++busGate_.thinkPed;
    else ++busGate_.thinkDrv;

    // THE SERVICE DAY opens and closes ONCE, not every tick: the edge is what
    // carries the event, so a bus that has already started for the yard is not
    // told again each frame (and one that failed to route there is not nagged).
    if (isBus(self) && busesInService() != busServiceWas_)
        tryGoalEvent(a, busesInService() ? GoalEvent::ServiceStart
                                         : GoalEvent::ServiceEnd);

    // A FREE CAB LOOKS FOR WORK. Costed on straight-line distance to the
    // pickup: cheap, and the real route is computed anyway the moment the trip
    // starts -- a pickup with no route fires NoRoute and the table puts the cab
    // back on its cruise, so a bad guess here costs one tick, not a stuck cab.
    if (isTaxi(self) && nav_ && dispatch_.waiting() > 0 && !dispatch_.fareOf(self)) {
        const Vec2 from = a.pos;
        const int nodeCount = static_cast<int>(nav_->nodes.size());
        const int took = dispatch_.assign(self, [&](int pickup) -> double {
            if (pickup < 0 || pickup >= nodeCount) return -1.0;
            const Vec2 p = nav_->nodes[static_cast<std::size_t>(pickup)];
            const Real dx = p.x - from.x, dy = p.y - from.y;
            return std::sqrt(dx * dx + dy * dy);
        });
        if (took >= 0 && tryGoalEvent(a, GoalEvent::GotFare) != GoalFire::NoRow) return;
    }
    const GoalTable& t = tableFor(a);
    if (a.goal < 0 || a.goal >= t.stateCount()) return;   // no table: inert
    const GoalState& s = t.state(a.goal);
    if (s.action == GoalAction::GoTo) {
        if (!a.moving) {
            int from = departNode(a);
            if (a.archetype == Agent::Mode::Pedestrian) ++busGate_.gotoPed;
            else ++busGate_.gotoDrv;
            // HAIL INSTEAD OF WALK. A walker facing a long trip sometimes takes
            // a cab -- which is the only thing that puts anyone in the back of
            // one without a host driving it. Decided ONCE, at the departure,
            // from the agent's own brain stream so it is reproducible; a hail
            // that finds no cab simply waits, and the rider is parked at the
            // kerb until one is free (hail(), awaitingRide()).
            if (hailChance_ > 0 && a.mode == Agent::Mode::Pedestrian && nav_ &&
                !isTaxi(self) && dispatch_.driverFor(self) < 0) {
                const int to = goalNodeFor(a, s.target);
                if (to >= 0 && from >= 0 && to != from &&
                    from < nav_->nodeCount() && to < nav_->nodeCount()) {
                    const Vec2 p0 = nav_->nodes[static_cast<std::size_t>(from)];
                    const Vec2 p1 = nav_->nodes[static_cast<std::size_t>(to)];
                    const Real dx = p1.x - p0.x, dy = p1.y - p0.y;
                    if (std::sqrt(dx * dx + dy * dy) >= hailMinMetres_ &&
                        brainUnit(a) < hailChance_ && hail(self, from, to))
                        return;
                }
            }
            // Archetype, for the same reason as tryGoalEvent above.
            if (a.archetype != Agent::Mode::Driver || a.far() ||
                isBus(indexOf(a)) || launchClear(a, from)) {
                if (!isBus(indexOf(a)) && departuresThisStep_ >= kDeparturesPerStep) return;   // retries next tick
                if (!isBus(indexOf(a))) ++departuresThisStep_;
                startGoalTrip(a, from, /*fromRest=*/true);
            }
        }
        return;
    }
    // Rest: the dwell clock runs on in-world hours, like the schedule windows.
    a.goalHours += dtHours;
    // ON A SEAT (M5): walking to or from it, nothing fires; sitting, the end of the dwell gets up and walks
    // back to the path first -- the next trip starts from there, not from the bench.
    if (a.seatPhase == 1 || a.seatPhase == 3) return;
    if (a.seatPhase == 2 && a.session >= 0) return;   // a group's player: the session says when it ends
    if (a.seatPhase == 2) {
        const Real sitFor = a.restDwell > 0 ? a.restDwell : s.dwellHours;   // a sit is short: the day's windows wait for it
        if (a.goalHours >= sitFor) a.seatPhase = 3;
        return;
    }
    // The stop's own length when it set one (a coffee, a browse), else the
    // state's.
    const Real dwell = a.restDwell > 0 ? a.restDwell : s.dwellHours;
    // The commute clock: inside the [departWork, departHome) window the day
    // says "be at work", outside it "be at home". Only meaningful for agents
    // with a real commute — a stranded pair (work == home, no route at build)
    // never departs, exactly as before.
    //
    // Checked BEFORE the dwell: a state that has both (an outing's pause, the
    // morning at work before lunch) ends the day when the window closes
    // instead of starting another stop -- else a stroller whose pauses kept
    // finishing never went home.
    if (a.home != a.work) {
        const bool atWorkNow = inWindow(clockHours_, departWorkHour(a), a.departHome);
        if (tryGoalEvent(a, atWorkNow ? GoalEvent::DepartWork
                                      : GoalEvent::DepartHome) != GoalFire::NoRow)
            return;
    }
    if (dwell > 0 && a.goalHours >= dwell)
        if (tryGoalEvent(a, GoalEvent::DwellDone) != GoalFire::NoRow) return;
    if (tryGoalEvent(a, GoalEvent::Idle) != GoalFire::NoRow) return;

    // NOTHING FIRED, AND NOTHING CAN UNTIL A KNOWN TIME. An agent resting at
    // home or at work is waiting for its own day, not reacting to the world, so
    // rather than asking it the same question sixty times a second it names the
    // moment it next has something to decide and leaves the active list until
    // then. Its car stays exactly where it is, drawn and collidable — cars are
    // state, agents are the thing that can be recomputed.
    //
    // Reaching here means no Idle row exists for this state, so the only things
    // that can move it are its dwell timer and its clock window; both are known
    // in advance. (A wander table always has an Idle row, so wanderers never
    // sleep — which is right: they are supposed to keep moving.)
    if (hoursPerSecond_ <= 0) return;   // clock stopped: nothing to wait for
    Real hoursUntil = kNeverWakes;
    if (dwell > 0)
        hoursUntil = std::max(Real(0), dwell - a.goalHours);
    if (a.home != a.work) {
        // The commute fires when the window predicate FLIPS, so the next
        // boundary is whichever end of the window is ahead of us.
        const bool atWorkNow = inWindow(clockHours_, departWorkHour(a), a.departHome);
        const Real boundary = atWorkNow ? a.departHome : a.departWork;
        Real delta = std::fmod(boundary - clockHours_, Real(24));
        if (delta < 0) delta += 24.0;
        hoursUntil = std::min(hoursUntil, delta);
    }
    // A stranded agent (work == home) with no dwell waits forever — and that is
    // the cheapest agent in the city, which is exactly right.
    a.sleptAt = clockTotalHours_;
    a.wakeAt = simSeconds_ + hoursUntil / hoursPerSecond_;
}

// THE CLOCK'S RATE CHANGED UNDER SLEEPING AGENTS (lockstep with the sky: the
// day/night loop was retuned, held, or released, and city_render hands the
// cycle's rate to every step). Wake times are sim-SECONDS derived from
// hours-until at the OLD rate, so the remaining wait is rescaled; a hold
// (rate 0) keeps sleepers under (the active-list gate) and the held seconds
// are handed back on release — an agent wakes on its own HOUR whatever the
// sky did in between (`sleepers_wake_on_their_hour_whatever_the_rate_did`).
void CitySim::rerateSleep(Real newRate) {
    if (newRate == hoursPerSecond_) return;
    if (newRate <= 0) return;   // entering a hold: the active-list gate keeps them under
    // The rate the wake times were computed at: the running rate, or — after
    // a hold — the last live one (a hold's seconds are handed back below, and
    // a hold that also retuned the loop rescales from THAT rate, not from 0:
    // the first cut only shifted, and a sleeper woke 6.4 hours late).
    const Real from = hoursPerSecond_ > 0 ? hoursPerSecond_ : lastLiveRate_;
    const Real k = from > 0 ? from / newRate : Real(1);
    for (Agent& a : agents_) {
        if (a.wakeAt < 0) continue;
        const Real remaining = (a.wakeAt - simSeconds_) + heldSeconds_;
        a.wakeAt = simSeconds_ + std::max(Real(0), remaining) * k;
    }
    heldSeconds_ = 0;
}

// Re-seat every agent onto the (new) tables: the first state wearing the
// agent's current activity label, else the entry state. Mid-trip agents keep
// driving — only their next transition consults the new table.
void CitySim::installGoalTables(GoalTable pedestrian, GoalTable driver) {
    goalPed_ = std::move(pedestrian);
    goalDriver_ = std::move(driver);
    for (Agent& a : agents_) {
        const GoalTable& t = tableFor(a);
        int mapped = t.entry();
        // Service vehicles keep their own entry, for the reason in
        // placeFromSchedule: matching on an activity LABEL would seat a bus in
        // whatever state happens to wear AtHome, which is its depot.
        const int who = indexOf(a);
        if (!isBus(who) && !isTaxi(who))
            for (int s = 0; s < t.stateCount(); ++s)
                if (t.state(s).activity == a.activity) { mapped = s; break; }
        a.goal = mapped;
        a.goalHours = 0;
    }
}

void CitySim::setGoalTables(GoalTable pedestrian, GoalTable driver) {
    installGoalTables(std::move(pedestrian), std::move(driver));
}

bool CitySim::boardRide(int passenger, int driver) {
    const int n = static_cast<int>(agents_.size());
    if (passenger < 0 || passenger >= n || driver < 0 || driver >= n) return false;
    if (!rides_.board(passenger, driver)) return false;
    Agent& p = agents_[static_cast<std::size_t>(passenger)];
    p.moving = false;   // the one flag; see the note on boardRide in the header
    p.speed = 0;
    return true;
}

void CitySim::alightRide(int passenger, int atNode) {
    rides_.alight(passenger);
    // Set down RESTING, wherever the vehicle stopped. The next goal tick sees a
    // GoTo state with !moving and launches a fresh trip from here -- which is
    // exactly "got out and walked the rest of the way", with no special case.
    //
    // "From here" is restNode, and nothing moved it during the ride: the next
    // trip departed from the stop the rider BOARDED at, so they stepped off and
    // reappeared back where they started (255 of 277 alightings in the rider
    // test, up to 1.7 km). The vehicle's stop is the new departure point.
    if (nav_ && atNode >= 0 && atNode < nav_->nodeCount() && passenger >= 0 &&
        passenger < static_cast<int>(agents_.size()))
        agents_[static_cast<std::size_t>(passenger)].restNode = atNode;
}

bool CitySim::pedVisible(int i) const {
    if (i < 0 || i >= static_cast<int>(agents_.size())) return false;
    const Agent& a = agents_[static_cast<std::size_t>(i)];
    if (a.mode != Agent::Mode::Pedestrian || a.far()) return false;
    if (riding(i)) return false;   // drawn in the vehicle, not on the pavement
    return a.moving || !a.indoors;
}

// A place to STAND near `want` that nobody else is standing in: `want` itself,
// else steps along the sidewalk (`along`) either way, then a second row a step
// back from the kerb. People waiting at a stop or pausing on a walk otherwise
// all end on the one sidewalk point their street leads to (measured on metro:
// 427 overlapping pairs among ~190 standing figures near the player).
engine::Vec2 CitySim::freeStandingSpot(const Agent& a, engine::Vec2 want,
                                       engine::Vec2 along) const {
    const int self = indexOf(a);
    const Real al = along.length();
    const Vec2 u = al > 1e-6 ? along * (1.0 / al) : Vec2(1, 0);
    const Vec2 side(u.y, -u.x);   // right of travel: away from the kerb on the right-hand walk
    std::vector<int> near;
    // the nearest body to p, squared (capped at the 1.5 m query)
    auto nearest2 = [&](Vec2 p) {
        grid_.query(p, 1.5, near);
        Real best = 1.5 * 1.5;
        for (int bi : near) {
            if (bi == self) continue;
            const Agent& b = agents_[static_cast<std::size_t>(bi)];
            if (b.mode != Agent::Mode::Pedestrian || b.far()) continue;
            if (!b.moving && b.indoors) continue;   // inside: not on the pavement
            if (riding(bi)) continue;
            best = std::min(best, (b.pos - p).lengthSquared());
        }
        return best;
    };
    constexpr Real kStep = 0.8;
    // A CROWD (a busy stop) can fill every spot tried; the fallback is then the roomiest of them, not `want`
    // itself -- which stood the newcomer inside whoever already stood there (two STILL people are never pushed
    // apart: macOS CI's one overlapping pair in metro_pedestrians_walk_and_keep_apart). Three rows now.
    Vec2 roomiest = want;
    Real roomiest2 = -1;
    for (int row = 0; row < 3; ++row) {
        const Vec2 base = want + side * (row * kStep);
        for (int k = 0; k < 9; ++k) {
            const Real off = kStep * static_cast<Real>((k + 1) / 2) * ((k & 1) ? 1.0 : -1.0);
            const Vec2 p = base + u * off;
            const Real d2 = nearest2(p);
            if (d2 >= 0.75 * 0.75) return p;
            if (d2 > roomiest2) { roomiest2 = d2; roomiest = p; }
        }
    }
    return roomiest;
}

// Somewhere NEAR to go next on an outing: an open park, cafe, store,
// supermarket, restaurant or civic building within a short walk -- or just a
// walk round the block (a random street corner 150-450 m off). Weighted so a
// park or a coffee is likelier than the town hall; never straight back to
// the stop just left. Sets a.tripVenue (-1 for the plain walk).
int CitySim::pickOuting(Agent& a, int origin) {
    const Menu* m = catalog_.menu("outing");
    return m ? pickFromMenu(a, origin, *m) : -1;
}

// ---- THE CATALOG'S CHOOSER (activities framework, step A) --------------------------------------------------------

std::string CitySim::venueKind(const Venue& v) const {
    switch (v.type) {
        case PlaceType::Cafe: return "cafe";
        case PlaceType::Restaurant: return "restaurant";
        case PlaceType::Shop: return "shop";
        case PlaceType::Supermarket: return "supermarket";
        case PlaceType::Office: return "office";
        case PlaceType::Home: return "home";
        case PlaceType::Park: return v.campus == 4 ? "quad" : v.campus == 5 ? "field" : "park";
        case PlaceType::Civic: return v.campus == 2 ? "library" : v.campus == 1 ? "teaching" : "civic";
        default: return "?";
    }
}

static const char* spotSiteKind(SpotKind k) {
    switch (k) {
        case SpotKind::Sit: return "seat";
        case SpotKind::Lie: return "bed";
        case SpotKind::Stand: return "stand";
        case SpotKind::Jog: return "loop";
        case SpotKind::Play: return "pitch";
        case SpotKind::Watch: return "watch";
        default: return "?";
    }
}

std::string CitySim::siteKindOf(const Agent& a) const {
    if (a.tripSeat >= 0 && a.tripSeat < static_cast<int>(seats_.size())) return spotSiteKind(seats_[static_cast<std::size_t>(a.tripSeat)].kind);
    if (a.tripVenue >= 0 && a.tripVenue < static_cast<int>(venues_.size())) return venueKind(venues_[static_cast<std::size_t>(a.tripVenue)]);
    if (a.tripActivity >= 0 && a.tripActivity < static_cast<int>(catalog_.defs.size())) {
        const ActivityDef& d = catalog_.defs[static_cast<std::size_t>(a.tripActivity)];
        if (!d.sites.empty()) return d.sites.front();
    }
    return "";
}

// One activity's site for this agent from `origin`: a spot (reserved), a place, or a street corner. countOnly: how
// many candidate sites there are (a per-site menu weight), nothing chosen.
int CitySim::tryActivity(Agent& a, int origin, int di, int prevVenue, bool countOnly, int* count) {
    if (count) *count = 0;
    if (di < 0 || di >= static_cast<int>(catalog_.defs.size()) || !nav_) return -1;
    const ActivityDef& d = catalog_.defs[static_cast<std::size_t>(di)];
    const Real h = clockHours_;
    if (!hourIn(h, d.hourLo, d.hourHi)) return -1;
    if (d.walkersOnly && (a.archetype != Agent::Mode::Pedestrian || a.mode != Agent::Mode::Pedestrian)) return -1;
    if (d.bringOwnEighths > 0 && static_cast<int>((a.brain >> 13) & 7u) < d.bringOwnEighths) return -1;
    if (d.inShift && !inWindow(clockHours_, departWorkHour(a), a.departHome)) return -1;
    const int fromNode = d.fromHome && a.home >= 0 && a.home < nav_->nodeCount() ? a.home : origin;
    const Vec2 here = nav_->nodes[static_cast<std::size_t>(fromNode)];
    // WATCHING: only while that activity is running within reach
    if (!d.during.empty()) {
        const int dd = catalog_.find(d.during);
        bool on = false;
        for (const Session& se : sessions_)
            if (se.def == dd && se.state == Session::State::Running &&
                (areas_[static_cast<std::size_t>(se.area)].center - here).lengthSquared() <= d.distHi * d.distHi)
                on = true;
        if (!on) return -1;
    }
    // AREAS (a group activity: a session on a pitch, a lawn)
    for (const std::string& k : d.sites)
        if (isAreaSiteKind(k)) {
            if (countOnly) {
                for (const ActivityArea& ar : areas_)
                    if (ar.kind == k && (ar.tags & d.tags) == d.tags &&
                        (ar.center - here).lengthSquared() <= d.distHi * d.distHi) { if (count) *count += 1; }
                return -1;
            }
            const int ai = joinSession(a, di, here, d.distLo, d.distHi, d.nearest);
            if (ai < 0) return -1;
            a.tripActivity = di;
            return areas_[static_cast<std::size_t>(ai)].node;
        }
    // SPOTS
    uint32_t kinds = 0;
    bool venuesToo = false, street = false;
    for (const std::string& k : d.sites) {
        bool found = false;
        for (int sk = 0; sk < static_cast<int>(SpotKind::Count); ++sk)
            if (k == spotSiteKind(static_cast<SpotKind>(sk))) { kinds |= spotKindBit(static_cast<SpotKind>(sk)); found = true; }
        if (k == "street") { street = true; found = true; }
        if (!found) venuesToo = true;
    }
    if (kinds) {
        ActivityQuery q;
        q.kinds = kinds; q.tags = d.tags; q.minDist = d.distLo; q.maxDist = d.distHi;
        q.nearest = d.nearest > 0 ? d.nearest : 1 << 30;
        if (countOnly) {
            int n = 0;
            for (const ActivitySpot& sp : seats_)
                if (sp.occupant < 0 && (q.kinds & spotKindBit(sp.kind)) && (sp.tags & q.tags) == q.tags) {
                    const Real d2 = (sp.pos - here).lengthSquared();
                    if (d2 >= q.minDist * q.minDist && d2 <= q.maxDist * q.maxDist) ++n;
                }
            if (count) *count += n;
        } else {
            const int sp = pickSpot(a, here, q);
            if (sp >= 0) {
                a.tripSeat = sp;
                a.tripActivity = di;
                return seats_[static_cast<std::size_t>(sp)].node;
            }
        }
    }
    // PLACES
    if (venuesToo) {
        std::vector<std::pair<Real, int>> cand;
        for (int i = 0; i < static_cast<int>(venues_.size()); ++i) {
            const Venue& v = venues_[static_cast<std::size_t>(i)];
            if (i == prevVenue || v.node == origin || !v.openAt(h)) continue;
            const std::string vk = venueKind(v);
            bool want = false;
            for (const std::string& k : d.sites) want = want || k == vk;
            if (!want) continue;
            const Real d2 = (nav_->nodes[static_cast<std::size_t>(v.node)] - here).lengthSquared();
            if (d2 < d.distLo * d.distLo || d2 > d.distHi * d.distHi) continue;
            cand.push_back({d2, i});
        }
        if (countOnly) { if (count) *count += static_cast<int>(cand.size()); }
        else if (!cand.empty()) {
            int pick;
            if (d.nearest > 0) {
                const std::size_t k = std::min<std::size_t>(cand.size(), static_cast<std::size_t>(d.nearest));
                std::partial_sort(cand.begin(), cand.begin() + static_cast<std::ptrdiff_t>(k), cand.end());
                pick = cand[tripRnd(a) % k].second;
            } else {
                pick = cand[rnd() % static_cast<uint32_t>(cand.size())].second;
            }
            a.tripVenue = pick;
            a.tripActivity = di;
            return venues_[static_cast<std::size_t>(pick)].node;
        }
    }
    // A STREET CORNER: somewhere in the distance band a walker leaves by (a walk round the block)
    if (street) {
        if (countOnly) { if (count) *count += 1; return -1; }
        const int n = nav_->streetNodeCount();
        for (int tries = 0; tries < 48; ++tries) {
            const int cand = static_cast<int>(rnd() % static_cast<uint32_t>(n));
            const Real d2 = (nav_->nodes[static_cast<std::size_t>(cand)] - here).lengthSquared();
            if (d2 < d.distLo * d.distLo || d2 > d.distHi * d.distHi) continue;
            for (int ol : nav_->outLinks[static_cast<std::size_t>(cand)])
                if (nav_->links[static_cast<std::size_t>(ol)].walkable) { a.tripActivity = di; return cand; }
        }
    }
    return -1;
}

int CitySim::pickFromMenu(Agent& a, int origin, const Menu& m) {
    const int prev = a.tripVenue;
    a.tripVenue = -1;
    a.tripActivity = -1;
    releaseSeat(a);
    if (!nav_ || origin < 0 || origin >= nav_->nodeCount()) return -1;
    const MenuBand* band = nullptr;
    for (const MenuBand& b : m.bands)
        if (hourIn(clockHours_, b.hourLo, b.hourHi)) { band = &b; break; }
    if (!band) return -1;
    // FIRST: tried in order, each with its chance
    for (const MenuEntry& e : band->first) {
        if (static_cast<Real>(rnd() % 1000u) >= e.chance * 1000.0) continue;
        const int node = tryActivity(a, origin, catalog_.find(e.activity), prev, false, nullptr);
        if (node >= 0) return node;
    }
    // THEN a weighted pick: a per-site activity weighs as much again for every site it could go to
    std::vector<std::pair<Real, int>> cum;   // cumulative weight -> entry
    Real total = 0;
    for (std::size_t k = 0; k < band->pick.size(); ++k) {
        const MenuEntry& e = band->pick[k];
        const int di = catalog_.find(e.activity);
        if (di < 0) continue;
        int n = 0;
        tryActivity(a, origin, di, prev, true, &n);
        if (n <= 0) continue;
        const Real w = e.weight * (catalog_.defs[static_cast<std::size_t>(di)].perSite ? n : 1);
        if (w <= 0) continue;
        total += w;
        cum.push_back({total, static_cast<int>(k)});
    }
    if (cum.empty()) return -1;
    const Real roll = static_cast<Real>(rnd() % 100000u) / 100000.0 * total;
    std::size_t at = 0;
    while (at + 1 < cum.size() && roll > cum[at].first) ++at;
    // the chosen one, then (if its site has gone) the others in order
    for (std::size_t t = 0; t < cum.size(); ++t) {
        const MenuEntry& e = band->pick[static_cast<std::size_t>(cum[(at + t) % cum.size()].second)];
        const int node = tryActivity(a, origin, catalog_.find(e.activity), prev, false, nullptr);
        if (node >= 0) return node;
    }
    return -1;
}

// A STUDENT'S BREAK between classes (campus milestone 4): a session in the
// library, a sit on a bench on the quad or a bleacher by the pitch, a walk
// across the quad -- and at midday, lunch out like anyone. Sets tripVenue or
// tripSeat; -1 = no stop (the table's NoRoute row: straight back to class).
int CitySim::pickCampusBreak(Agent& a, int origin) {
    const Menu* m = catalog_.menu("student_break");
    return m ? pickFromMenu(a, origin, *m) : -1;
}

int CitySim::pickCampusSeat(Agent& a, Vec2 here) {
    // a free seat on the campus (within 120 m of the quad or the field: tagSpots), one of the nearest six to here
    ActivityQuery q;
    q.kinds = spotKindBit(SpotKind::Sit);
    q.tags = spot_tag::kCampus;
    q.maxDist = 600.0; q.nearest = 6;
    return pickSpot(a, here, q);
}

void CitySim::assignStudents(const PlaceMap& places, const NavGraph& graph) {
    // Every hall of each kind: the quad block's residence hall and the dorm block's, both teaching halls.
    std::vector<PlaceId> halls, teach;
    PlaceId lib = kNoPlace;
    for (const Place& p : places.places()) {
        if (p.campus == 3) halls.push_back(p.id);
        if (p.campus == 1) teach.push_back(p.id);
        if (p.campus == 2 && lib == kNoPlace) lib = p.id;
    }
    if (halls.empty() || teach.empty()) return;
    auto doorOf = [&](PlaceId id) { const Place& p = places[id]; return p.entrance + (p.site - p.entrance) * 0.4; };
    // the halls a student can live in: routable to every teaching hall and back on foot
    struct Hall { PlaceId id; int node; int beds; };
    std::vector<Hall> live;
    std::vector<std::pair<PlaceId, int>> classes;
    for (PlaceId t : teach) {
        const int tn = graph.nearestNode(places[t].entrance);
        if (tn >= 0) classes.push_back({t, tn});
    }
    if (classes.empty()) return;
    for (PlaceId h : halls) {
        const int hn = graph.nearestNode(places[h].entrance);
        if (hn < 0) continue;
        bool ok = true;
        for (const auto& c : classes)
            ok = ok && c.second != hn && engine::findRoute(graph, hn, c.second, true).valid() &&
                 engine::findRoute(graph, c.second, hn, true).valid();
        if (ok) live.push_back({h, hn, places[h].capacity > 0 ? places[h].capacity : 150});
    }
    if (live.empty()) return;
    const int ln = lib != kNoPlace ? graph.nearestNode(places[lib].entrance) : -1;

    // WHO: walkers only (a student has no car), never a bus or a cab's driver, chosen by their own bits so the same
    // city always has the same students. As many as the halls have beds, never more than a quarter of the walkers.
    std::vector<std::pair<uint32_t, int>> cand;
    int walkers = 0;
    for (std::size_t i = 0; i < agents_.size(); ++i) {
        const Agent& a = agents_[i];
        if (a.archetype != Agent::Mode::Pedestrian || isBus(static_cast<int>(i)) || isTaxi(static_cast<int>(i))) continue;
        ++walkers;
        uint32_t k = a.brain * 0x27d4eb2fu;
        k ^= k >> 15; k *= 0x165667b1u; k ^= k >> 13;
        cand.push_back({k, static_cast<int>(i)});
    }
    int beds = 0;
    for (const Hall& h : live) beds += h.beds;
    const int want = std::min(beds, walkers / 4);
    if (want <= 0) return;
    std::partial_sort(cand.begin(), cand.begin() + want, cand.end());
    std::size_t hall = 0;
    int inHall = 0;
    for (int c = 0; c < want; ++c) {
        while (inHall >= live[hall].beds * want / std::max(1, beds) + 1 && hall + 1 < live.size()) { ++hall; inHall = 0; }   // halls fill in proportion
        ++inHall;
        const Hall& H = live[hall];
        Agent& a = agents_[static_cast<std::size_t>(cand[static_cast<std::size_t>(c)].second)];
        a.role = Agent::Role::Student;
        a.homePlace = H.id; a.home = H.node; a.homeDoor = doorOf(H.id);
        a.restNode = H.node; a.pos = a.homeDoor; a.indoors = true;
        if (!graph.outLinks[static_cast<std::size_t>(H.node)].empty())
            a.heading = graph.direction(graph.outLinks[static_cast<std::size_t>(H.node)][0]);
        const auto& cl = classes[(a.brain >> 9) % classes.size()];   // which hall their course is taught in
        a.workPlace = cl.first; a.work = cl.second; a.workDoor = doorOf(cl.first);
        a.shopPlace = lib; a.shop = ln; a.shopDoor = lib != kNoPlace ? doorOf(lib) : a.workDoor;
        // The first class at 8:30-10, the last out at 16:00-18:30.
        const Real u0 = static_cast<Real>((a.brain >> 5) & 0xFF) / 255.0, u1 = static_cast<Real>((a.brain >> 17) & 0xFF) / 255.0;
        a.departWork = 8.5 + 1.5 * u0;
        a.departHome = 16.0 + 2.5 * u1;
        const engine::Route there = engine::findRoute(graph, H.node, cl.second, true);
        Real walk = 0;
        for (int li : there.links) walk += graph.links[static_cast<std::size_t>(li)].length;
        a.commuteSeconds = walk / std::max(Real(0.1), kWalkSpeed * a.speedFactor);
    }
}

// Lunch OUT: a walker at work, in its shift, goes to one of the few nearest
// open cafes or restaurants. About three in eight brought lunch and stay in;
// a car commuter stays in too (its car is parked at work -- a lunch trip
// would drive it round the block). -1 = no trip (the table's NoRoute row).
int CitySim::pickLunch(Agent& a, int origin) {
    const Menu* m = catalog_.menu("lunch");
    return m ? pickFromMenu(a, origin, *m) : -1;
}

int CitySim::goalNodeFor(const Agent& a, GoalTarget target) const {
    switch (target) {
        case GoalTarget::Work: return a.work;
        case GoalTarget::Home: return a.home;
        case GoalTarget::Shop: return a.shop;
        default: return -1;
    }
}

bool CitySim::isBus(int i) const {
    return i >= 0 && i < static_cast<int>(busRoute_.size()) &&
           busRoute_[static_cast<std::size_t>(i)] >= 0;
}

void CitySim::setBuses(int routes, int stopsPerRoute, int busCount, Real maxWalk) {
    busMaxWalk_ = maxWalk;
    buses_.clear();
    busRoute_.assign(agents_.size(), -1);
    busStop_.assign(agents_.size(), -1);
    if (!nav_ || routes <= 0 || busCount <= 0) return;
    buses_.build(*nav_, routes, stopsPerRoute, rng_ ? rng_ : 1u);
    if (buses_.empty()) return;
    if (busTable_.stateCount() == 0) busTable_ = busGoals();
    // Buses come off the DRIVER pool, spread by index like the cabs, and are
    // dealt across the routes BY LAP TIME -- one each first, then to whichever
    // route is furthest short of its share -- so every route runs about the same
    // headway. Round-robin gave a town's 3 km loop as many buses as the 25 km
    // regional one once places out along the freeway had routes of their own.
    const int R = buses_.routeCount();
    std::vector<Real> lapShare(static_cast<std::size_t>(R), 0);
    {
        Real total = 0;
        for (int r = 0; r < R; ++r) {
            const BusRoute& br = buses_.route(r);
            lapShare[static_cast<std::size_t>(r)] = buses_.rideSeconds(r, 0, 0) > 0 ? buses_.rideSeconds(r, 0, 0) : br.loopLength;
            total += lapShare[static_cast<std::size_t>(r)];
        }
        for (Real& l : lapShare) l = total > 0 ? l / total : 1.0 / R;
    }
    std::vector<int> dealt(static_cast<std::size_t>(R), 0);
    int made = 0;
    for (std::size_t i = 0; i < agents_.size() && made < busCount; ++i) {
        if (agents_[i].archetype != Agent::Mode::Driver) continue;
        if (isTaxi(static_cast<int>(i))) continue;   // a cab is not also a bus
        int r = made < R ? made : 0;
        if (made >= R)
            for (int q = 1; q < R; ++q)
                if (lapShare[static_cast<std::size_t>(q)] * (made + 1) - dealt[static_cast<std::size_t>(q)] >
                    lapShare[static_cast<std::size_t>(r)] * (made + 1) - dealt[static_cast<std::size_t>(r)]) r = q;
        ++dealt[static_cast<std::size_t>(r)];
        busRoute_[i] = r;
        agents_[i].goal = busTable_.entry();
        agents_[i].goalHours = 0;
        // A BUS DRIVES A BUS. Without this the transit fleet was ordinary cars
        // -- a sedan carrying 24 people -- and car-following held them a
        // sedan's gap apart. The body comes from whichever fleet slot declares
        // itself a Bus, so a scripted fleet's own dimensions win over the
        // built-in fallback.
        // The OWNED car, not the one being driven this instant: a driver who
        // happened to be on foot at setup has vehicle -1, and its bus stayed a
        // sedan.
        if (agents_[i].car >= 0 &&
            agents_[i].car < static_cast<int>(vehicles_.size())) {
            VehicleBody bb{11.4, 2.55, 3.20, VehicleType::Bus};
            for (int fs = 0; fs < fleetSize(); ++fs)
                if (fleetBody(fs).type == VehicleType::Bus) { bb = fleetBody(fs); break; }
            SimVehicle& sv = vehicles_[static_cast<std::size_t>(agents_[i].car)];
            sv.length = bb.length;
            sv.width = bb.width;
            sv.height = bb.height;
            sv.type = bb.type;
        }
        ++made;
    }

    // Buses per route, for the wait a rider weighs (half a headway).
    {
        std::vector<int> perRoute(static_cast<std::size_t>(buses_.routeCount()), 0);
        for (std::size_t i = 0; i < agents_.size(); ++i)
            if (busRoute_[i] >= 0 && busRoute_[i] < buses_.routeCount())
                ++perRoute[static_cast<std::size_t>(busRoute_[i])];
        buses_.setFleet(std::move(perRoute));
    }
    // How many buses already stand at each stop node: the next queues behind.
    std::unordered_map<int, int> seatedAt;
    // START ON THE ROUTE, SPREAD ROUND IT. Each bus used to begin wherever its
    // driver happened to be -- often a kilometre from its own loop -- and bus
    // k of a route was aimed at stop k, so a route's six buses were bunched
    // over its first six stops (the "comes 689 m off" the HUD caught). Now
    // bus k of m starts AT stop k*N/m, standing at the kerb, so a route's
    // buses are evenly spaced from the first minute and a player who arrives
    // at a stop sees one within a headway, not after the fleet finds its way.
    for (int r = 0; r < buses_.routeCount(); ++r) {
        std::vector<int> fleet;
        for (std::size_t i = 0; i < agents_.size(); ++i)
            if (busRoute_[i] == r) fleet.push_back(static_cast<int>(i));
        const BusRoute& route = buses_.route(r);
        const int n = static_cast<int>(route.stops.size());
        const int m = static_cast<int>(fleet.size());
        for (int k = 0; k < m && n > 0; ++k) {
            const int at = static_cast<int>((static_cast<long long>(k) * n) / m);
            const int stopNode = route.stops[static_cast<std::size_t>(at)].node;
            seatBusAt(fleet[static_cast<std::size_t>(k)], stopNode, seatedAt[stopNode]++);
            busStop_[static_cast<std::size_t>(fleet[static_cast<std::size_t>(k)])] =
                (at + 1) % n;
        }
    }
    (void)stopsPerRoute;   // the network already has them; logging lives in city_render
}

// PARKING (Glenn, 2026-09-18: "They park at the corner in a pile, but they
// should be using the parallel parking spaces... they need to park in spaces
// and not in a heap on the corner"). Measured on metro at load: 4646 marked
// bays and 1259 drivers, yet 2 cars in bays and 857 of 1257 parked cars within
// 3 m of another -- every car was put 2-12 m from its home NODE, and an
// arrival only looked for a bay on the street it came in on, within 30 m of
// the corner. The spaces existed; nothing chose them.
int CitySim::claimBayNear(Vec2 target, int self, Real maxDist) {
    int best = -1;
    Real bestD2 = maxDist * maxDist;
    for (std::size_t i = 0; i < bays_.size(); ++i) {
        if (bays_[i].occupant != -1) continue;
        const Vec2 d = bays_[i].pos - target;
        const Real d2 = d.x * d.x + d.y * d.y;
        if (d2 < bestD2) { bestD2 = d2; best = static_cast<int>(i); }
    }
    if (best >= 0) bays_[static_cast<std::size_t>(best)].occupant = self;
    return best;
}

std::vector<int> CitySim::nearestFreeBays(Vec2 target, Real maxDist, int k) const {
    std::vector<std::pair<Real, int>> near;
    const Real r2 = maxDist * maxDist;
    for (std::size_t i = 0; i < bays_.size(); ++i) {
        if (bays_[i].occupant != -1) continue;
        const Vec2 d = bays_[i].pos - target;
        const Real d2 = d.x * d.x + d.y * d.y;
        if (d2 < r2) near.push_back({d2, static_cast<int>(i)});
    }
    const std::size_t keep = std::min(near.size(), static_cast<std::size_t>(std::max(k, 0)));
    std::partial_sort(near.begin(), near.begin() + static_cast<std::ptrdiff_t>(keep), near.end());
    std::vector<int> out;
    for (std::size_t i = 0; i < keep; ++i) out.push_back(near[i].second);
    return out;
}

// Metres a bus still has to drive to the point it stops at: the rest of its
// route, less the stand-back from the last junction (the arrival rule).
Real CitySim::busDistanceToStop(const Agent& a) const {
    const int legs = static_cast<int>(a.route.links.size());
    if (a.leg >= legs) return 0;
    Real rem = nav_->links[static_cast<std::size_t>(a.route.links[static_cast<std::size_t>(a.leg)])].length -
               a.distOnLeg;
    for (int k = a.leg + 1; k < legs && rem < 200.0; ++k)
        rem += nav_->links[static_cast<std::size_t>(a.route.links[static_cast<std::size_t>(k)])].length;
    return rem - busStandBack(a);
}

// Where a bus stands at its stop: behind the crosswalk on the street it came
// in on, exactly like a car held at the stop line -- the box, the zebra band, a
// margin, and half its own length. In ROUTE metres, because on metro's ~4 m
// links "short of the node on the last link" was 2.4 m: the bus dwelt up to
// 40 s inside the junction (Glenn, 2026-09-19: "That creates an instant
// traffic jam"). Never behind the previous junction on its route.
Real CitySim::busStandBackAt(int inLink) const {
    if (!nav_ || inLink < 0 || inLink >= nav_->linkCount()) return 0;
    const int node = nav_->links[static_cast<std::size_t>(inLink)].to;
    Real approach = 0;
    int li = inLink;
    for (int guard = 0; guard < 16 && li >= 0 && approach < 80.0; ++guard) {
        const engine::NavLink& L = nav_->links[static_cast<std::size_t>(li)];
        approach += L.length;
        if (nav_->isJunction(L.from)) break;
        // the single street link arriving at a plain node (not the way back)
        int prev = -1;
        for (int k = 0; k < nav_->linkCount() && prev < 0; ++k) {
            const engine::NavLink& P = nav_->links[static_cast<std::size_t>(k)];
            if (P.to == L.from && P.from != L.to) prev = k;
        }
        li = prev;
    }
    Real busLen = 12.0;
    for (int i = 0; i < static_cast<int>(agents_.size()); ++i)
        if (isBus(i)) { busLen = vehicleLength(i); break; }
    const Real want = junctionRadius(node) + kCrosswalkFarEdge + kStopLineMargin + 0.5 * busLen;
    return std::min(want, approach * 0.6);
}

Real CitySim::busStandBack(const Agent& a) const {
    if (a.route.links.empty()) return 0;
    const int legs = static_cast<int>(a.route.links.size());
    const int node = nav_->links[static_cast<std::size_t>(a.route.links.back())].to;
    Real approach = 0;
    for (int k = legs - 1; k >= 0 && approach < 80.0; --k) {
        const engine::NavLink& L = nav_->links[static_cast<std::size_t>(a.route.links[static_cast<std::size_t>(k)])];
        approach += L.length;
        if (nav_->isJunction(L.from)) break;
    }
    const Real want = junctionRadius(node) + kCrosswalkFarEdge + kStopLineMargin +
                      0.5 * vehicleLength(indexOf(a));
    return std::min(want, approach * 0.6);
}

void CitySim::releaseBays(Agent& a) {
    const int self = indexOf(a);
    for (int* b : {&a.parkedBay, &a.targetBay}) {
        if (*b >= 0 && *b < static_cast<int>(bays_.size()) &&
            bays_[static_cast<std::size_t>(*b)].occupant == self)
            bays_[static_cast<std::size_t>(*b)].occupant = -1;
        *b = -1;
    }
}

bool CitySim::parksInBays(const Agent& a) const {
    if (a.archetype != Agent::Mode::Driver || wander_) return false;
    const int self = indexOf(a);
    return !isBus(self) && !isTaxi(self);
}

// A bus standing at a stop, engine running: in its own vehicle, at rest on
// the node, with no trip -- so the goal table's Drive launches it toward its
// next stop on the next tick. The same fields placeFromSchedule resets for an
// agent seated at rest, plus the mount, because a bus is never on foot.
void CitySim::seatBusAt(int idx, int node, int queued) {
    if (!nav_ || idx < 0 || idx >= static_cast<int>(agents_.size())) return;
    if (node < 0 || node >= nav_->nodeCount()) return;
    Agent& a = agents_[static_cast<std::size_t>(idx)];
    remountOwnedCar(a);
    a.restNode = node;
    a.moving = false;
    a.speed = 0;
    a.route.links.clear();
    a.leg = 0;
    a.distOnLeg = 0;
    a.elevation = 0;
    a.pos = idlePose(node, Agent::Mode::Driver, a.brain);
    if (!nav_->outLinks[static_cast<std::size_t>(node)].empty())
        a.heading = nav_->direction(nav_->outLinks[static_cast<std::size_t>(node)][0]);
    // Routes share hub stops: a second bus at the same stop stands a bus
    // length further back instead of on the first one (two buses at a hub
    // were seated on the identical point).
    if (queued > 0) a.pos = a.pos - a.heading * (Real(14) * queued);
    a.tickFromPos = a.pos;
    a.tickFromHeading = a.heading;
    if (a.car >= 0 && a.car < static_cast<int>(vehicles_.size())) {
        vehicles_[static_cast<std::size_t>(a.car)].pos = a.pos;
        vehicles_[static_cast<std::size_t>(a.car)].heading = a.heading;
    }
    grid_.place(idx, a.pos);
}

void CitySim::setTaxiFraction(Real f) {
    taxiFraction_ = f < 0 ? 0 : (f > 1 ? 1 : f);
    applyTaxiFraction();
}

void CitySim::applyTaxiFraction() {
    if (agents_.empty()) return;
    taxi_.assign(agents_.size(), 0);
    if (taxiFraction_ <= 0) return;
    if (taxiTable_.stateCount() == 0) taxiTable_ = taxiGoals();
    int drivers = 0;
    for (const Agent& a : agents_)
        if (a.archetype == Agent::Mode::Driver) ++drivers;
    if (drivers <= 0) return;
    const int want = static_cast<int>(drivers * taxiFraction_ + 0.5);
    if (want <= 0) return;
    // Every Nth DRIVER by index: deterministic, and spread through the fleet
    // instead of clustered at the front where they would all spawn together.
    const int stride = std::max(1, drivers / want);
    int seen = 0, made = 0;
    for (std::size_t i = 0; i < agents_.size() && made < want; ++i) {
        if (agents_[i].archetype != Agent::Mode::Driver) continue;
        if (seen++ % stride == 0) {
            taxi_[i] = 1;
            agents_[i].goal = taxiTable_.entry();
            agents_[i].goalHours = 0;
            ++made;
        }
    }
}

bool CitySim::hail(int passenger, int pickup, int dropoff) {
    if (!dispatch_.hail(passenger, pickup, dropoff)) return false;
    // WAIT AT THE KERB. A rider who carries on walking is not where the cab was
    // sent, and boarding would snap them across the city to meet it. Stopping
    // here is what makes `pickup` mean anything -- and the goal pass leaves a
    // waiting agent alone (see awaitingRide), so they stand until collected.
    if (passenger >= 0 && passenger < static_cast<int>(agents_.size())) {
        Agent& p = agents_[static_cast<std::size_t>(passenger)];
        p.moving = false;
        p.speed = 0;
    }
    return true;
}

void CitySim::setTaxi(int i, bool on) {
    if (taxiTable_.stateCount() == 0) taxiTable_ = taxiGoals();
    if (i < 0 || i >= static_cast<int>(agents_.size())) return;
    if (taxi_.size() != agents_.size()) taxi_.assign(agents_.size(), 0);
    taxi_[static_cast<std::size_t>(i)] = on ? 1 : 0;
    if (on) {
        Agent& a = agents_[static_cast<std::size_t>(i)];
        a.goal = taxiTable_.entry();
        a.goalHours = 0;
    }
}

bool CitySim::isTaxi(int i) const {
    return i >= 0 && i < static_cast<int>(taxi_.size()) &&
           taxi_[static_cast<std::size_t>(i)] != 0;
}

void CitySim::setWander(bool on) {
    wander_ = on;
    installGoalTables(on ? wanderGoals(false) : defaultScheduleGoals(),
                      on ? wanderGoals(true) : defaultScheduleGoals());
}

// Is the spawn area at `node` free of moving cars? Agents departing from rest
// materialize on the lane there — launching under a crossing (or same-node)
// car spawned two bodies inside each other, which the crash rule then locked.
bool CitySim::launchClear(const Agent& a, int node) const {
    if (!nav_ || node < 0 || node >= nav_->nodeCount()) return true;
    // LEAVING A BAY: the car pulls out mid-street, into the kerb lane at its
    // bay's station -- not at the node. Wait for a GAP there: no moving car on
    // this street coming up from behind within 30 m, nor just ahead. Checking
    // only the node let cars pull out in front of passing traffic; the
    // follower crash-froze and the queue behind it stalled (metro: mean speed
    // 5.6 -> 4.1 m/s after 3 minutes once cars parked in bays).
    if (a.parkedBay >= 0 && a.parkedBay < static_cast<int>(bays_.size())) {
        const ParkingBay& bay = bays_[static_cast<std::size_t>(a.parkedBay)];
        if (nav_->links[static_cast<std::size_t>(bay.link)].to == node) {
            grid_.query(bay.pos, 40.0, queryScratch_);
            for (int bi : queryScratch_) {
                const Agent& b = agents_[static_cast<std::size_t>(bi)];
                if (&b == &a || b.mode != Agent::Mode::Driver) continue;
                if (b.far() || !b.moving || b.released) continue;
                if (b.leg < static_cast<int>(b.route.links.size()) &&
                    b.route.links[static_cast<std::size_t>(b.leg)] == bay.link) {
                    const Real rel = b.distOnLeg - bay.station;
                    if (rel > -30.0 && rel < 8.0) return false;
                    continue;
                }
                const Vec2 d = b.pos - bay.pos;
                if (d.x * d.x + d.y * d.y < 8.0 * 8.0) return false;
            }
            return true;
        }
    }
    const Vec2 p = nav_->nodes[node];
    // The radius must cover every place the launch could put the body: the lane
    // start at the node PLUS the junction-origin box skip (jr + 2 m down the
    // first link) — a 5 m check let a second car spawn exactly on a first one
    // that had been skipped 6 m out.
    Real clear = junctionRadius(node) + 2.0 + 6.0;
    // Grid candidates (P4.1): pass 1 re-hashes each agent right after its goal
    // tick, so positions are current here; the slack covers cell granularity.
    grid_.query(p, clear + 4.0, queryScratch_);
    for (int bi : queryScratch_) {
        const Agent& b = agents_[bi];
        if (&b == &a || b.mode != Agent::Mode::Driver) continue;
        if (b.far()) continue;   // far tier: no body here
        if (!b.moving || b.released) continue;
        if (std::fabs(b.elevation - a.elevation) > 3.0) continue;
        Real dx = b.pos.x - p.x, dy = b.pos.y - p.y;
        if (dx * dx + dy * dy < clear * clear) return false;
    }
    return true;
}

void CitySim::startTrip(Agent& a, int origin, int goal, bool fromRest) {
    // Where the CAR is before it pulls out: its own parked pose if it is
    // standing in the world unattended, else wherever the agent stands.
    Vec2 pulledFrom = a.pos;
    Vec2 pulledHeading = a.heading;
    if (a.car >= 0 && a.car < static_cast<int>(vehicles_.size()) &&
        vehicles_[static_cast<std::size_t>(a.car)].driver < 0) {
        pulledFrom = vehicles_[static_cast<std::size_t>(a.car)].pos;
        pulledHeading = vehicles_[static_cast<std::size_t>(a.car)].heading;
    }
    // Leaving a BAY: the trip starts from where the car stands on its street,
    // not from the node past it (see the prepend below).
    const int leavingBay =
        (fromRest && a.parkedBay >= 0 && a.parkedBay < static_cast<int>(bays_.size()) &&
         nav_->links[static_cast<std::size_t>(bays_[static_cast<std::size_t>(a.parkedBay)].link)].to == origin)
            ? a.parkedBay : -1;
    remountOwnedCar(a);
    if (a.car >= 0 && a.car < static_cast<int>(vehicles_.size()))
        vehicles_[static_cast<std::size_t>(a.car)].offStreet = false;   // out of the garage
    // A bay reserved for a previous plan is not this trip's.
    if (a.targetBay >= 0) {
        if (a.targetBay < static_cast<int>(bays_.size()) &&
            bays_[static_cast<std::size_t>(a.targetBay)].occupant == indexOf(a))
            bays_[static_cast<std::size_t>(a.targetBay)].occupant = -1;
        a.targetBay = -1;
    }
    a.tripGoal = goal;
    a.stopAtDist = -1;   // an ordinary trip ends at its node

    // DRIVE TO A SPACE NEAR THE DESTINATION: reserve the nearest free bay to
    // it and end the route AT that bay (its street, stopping at its station).
    // The nearest few free bays, tried in order: a bay whose approach would
    // U-TURN onto its own street (arriving along the far carriageway, then
    // coming back) is skipped for the next -- usually the one across the
    // road. Taking the nearest regardless put a U-turn at the end of 67 of 144
    // routes in metro, and U-turning cars stalled the streets behind them.
    int bay = -1;
    if (a.mode == Agent::Mode::Driver && parksInBays(a) && goal >= 0 &&
        goal < nav_->nodeCount() && goal != origin) {
        for (int cand : nearestFreeBays(nav_->nodes[static_cast<std::size_t>(goal)],
                                        kParkSearch, 8)) {
            const int bl = bays_[static_cast<std::size_t>(cand)].link;
            const engine::NavLink& BL = nav_->links[static_cast<std::size_t>(bl)];
            engine::Route r;
            if (BL.from != origin) {
                const int twin = (leavingBay >= 0 &&
                                  static_cast<std::size_t>(bays_[static_cast<std::size_t>(leavingBay)].link) < twinOf_.size())
                                     ? twinOf_[static_cast<std::size_t>(bays_[static_cast<std::size_t>(leavingBay)].link)]
                                     : -1;
                if (twin >= 0) departScale_[static_cast<std::size_t>(twin)] = 50.0;
                r = engine::findRoute(*nav_, origin, BL.from, false,
                                      (twin >= 0 || wanderPriced_) ? &departScale_ : nullptr);
                if (twin >= 0) departScale_[static_cast<std::size_t>(twin)] = 1.0;
                if (!r.valid()) continue;
                const engine::NavLink& last =
                    nav_->links[static_cast<std::size_t>(r.links.back())];
                if (last.from == BL.to && last.to == BL.from) continue;   // a U-turn
            }
            r.links.push_back(bl);
            a.route = r;
            a.targetBay = cand;
            bays_[static_cast<std::size_t>(cand)].occupant = indexOf(a);
            bay = cand;
            break;
        }
    }
    if (bay < 0) {
        // Leaving a bay the car faces along its street; turning straight back
        // down the other carriageway at the corner is a U-turn in traffic (67
        // of 144 metro routes did it). Price that twin so the route goes round
        // the block instead, unless nothing else gets there.
        const int twin = (leavingBay >= 0 && a.mode == Agent::Mode::Driver &&
                          static_cast<std::size_t>(bays_[static_cast<std::size_t>(leavingBay)].link) < twinOf_.size())
                             ? twinOf_[static_cast<std::size_t>(bays_[static_cast<std::size_t>(leavingBay)].link)]
                             : -1;
        if (twin >= 0) departScale_[static_cast<std::size_t>(twin)] = 50.0;
        a.route = engine::findRoute(*nav_, origin, goal,
                                    a.mode == Agent::Mode::Pedestrian,
                                    (twin >= 0 || wanderPriced_) ? &departScale_ : nullptr);
        if (twin >= 0) departScale_[static_cast<std::size_t>(twin)] = 1.0;
    }
    a.leg = 0;
    a.distOnLeg = 0;
    a.speed = 0;
    if (!a.route.valid()) {
        // No path: do NOT teleport to the goal (that was the "disappear/reappear"
        // bug). Stay parked at the ORIGIN so the agent simply doesn't take this
        // trip; the goal layer's NoRoute row decides where its day falls back to.
        a.moving = false;
        a.elevation = 0;
        // A car in a BAY stays in it: moving it to the verge left the bay
        // claimed by a car 80 m away (and put it back on the corner heap).
        if (a.parkedBay >= 0 && a.parkedBay < static_cast<int>(bays_.size()))
            a.pos = bays_[static_cast<std::size_t>(a.parkedBay)].pos;
        else
            a.pos = idlePose(origin, a.mode, a.brain);
        if (a.targetBay >= 0 && a.targetBay < static_cast<int>(bays_.size()) &&
            bays_[static_cast<std::size_t>(a.targetBay)].occupant == indexOf(a))
            bays_[static_cast<std::size_t>(a.targetBay)].occupant = -1;
        a.targetBay = -1;
        return;
    }
    int lanes = nav_->links[a.route.links.front()].lanes;
    a.lane = (a.mode == Agent::Mode::Driver && lanes > 1)
                 ? static_cast<int>(tripRnd(a) % static_cast<uint32_t>(lanes))
                 : 0;
    a.laneF = a.lane;
    a.laneTimer = 4.0 + (a.home % 11);
    ++a.trips;   // a new route: the pursuit bridge rebuilds its path off this
    a.moving = true;
    a.indoors = false;   // out of the door
    a.restDwell = 0;
    int leaveStation = -1;
    Real leaveAt = 0;
    if (a.parkedBay >= 0 && a.parkedBay < static_cast<int>(bays_.size())) {
        if (a.parkedBay == leavingBay) {
            leaveStation = bays_[static_cast<std::size_t>(a.parkedBay)].link;
            leaveAt = bays_[static_cast<std::size_t>(a.parkedBay)].station;
        }
        if (bays_[a.parkedBay].occupant == indexOf(a))
            bays_[a.parkedBay].occupant = -1;   // pulling out frees the bay
        a.parkedBay = -1;
    }
    a.crashTimer = 0;    // a fresh trip carries no wreck state: without this an
    a.crashCount = 0;    // escapee that PARKED near its wreck departed hours
                         // later still contact-immune and ghosted through traffic
    // A REST departure whose origin is a junction starts a few metres down the
    // first link, past the box — materializing a parked car among crossing
    // traffic spawned collisions the crash rule then locked in place.
    if (leaveStation >= 0 && a.mode == Agent::Mode::Driver) {
        // Out of the bay: the route begins on the bay's own street, at the
        // bay's station, so pulling out is a sideways step into the lane.
        a.route.links.insert(a.route.links.begin(), leaveStation);
        a.distOnLeg = leaveAt;
        const int lanes0 = std::max(1, nav_->links[static_cast<std::size_t>(leaveStation)].lanes);
        a.lane = lanes0 - 1;   // the kerb lane, beside the bay
        a.laneF = a.lane;
    } else if (fromRest && a.mode == Agent::Mode::Driver && nav_->isJunction(origin)) {
        // Start clear of the origin's box -- the WHOLE body past it. Walked
        // across legs: on metro's ~4 m links the old single-link clamp (40%
        // of the first link) started a car 1.6 m from the node, inside the
        // crossing it was leaving (every bus began its day parked in one).
        Real skip = junctionRadius(origin) + 0.5 * vehicleLength(indexOf(a)) + 0.5;
        const int legN = static_cast<int>(a.route.links.size());
        Real total = 0;
        for (int k = 0; k < legN; ++k) total += nav_->links[a.route.links[k]].length;
        skip = std::min(skip, total * 0.4);   // a short trip: most of it still ahead
        a.leg = 0;
        Real Lk = nav_->links[a.route.links.front()].length;
        while (skip > Lk && a.leg + 1 < legN) {
            skip -= Lk;
            ++a.leg;
            Lk = nav_->links[a.route.links[a.leg]].length;
        }
        a.distOnLeg = std::min(skip, Lk);
    }
    refreshPose(a);
    a.heading = nav_->direction(a.route.links.front());   // start pointed down leg 0
    a.tickFromPos = a.pos;          // a placement, not a motion (see tickFromPos)
    a.tickFromHeading = a.heading;
    a.tickFromPullS = a.pullS;
    // Pull out of the space rather than appear in the lane. Only for a car
    // leaving from rest in the drawn tier; a far agent is never on screen. A
    // CHAINED trip keeps a pull already under way -- cutting it short snapped
    // the car into its lane mid-merge (a 9 m jump and a 158-degree pivot).
    if (fromRest) a.pullLen = 0;
    if (fromRest && a.mode == Agent::Mode::Driver && !a.far()) {
        const Vec2 off = pulledFrom - a.pos;
        const Real d = off.length();
        if (d > 0.5 && d < 40.0) {
            a.pullOffset = off;
            a.pullYawOffset = 0;
            if (pulledHeading.lengthSquared() > 0.25) {
                Real dy = std::atan2(pulledHeading.x, pulledHeading.y) -
                          std::atan2(a.heading.x, a.heading.y);
                while (dy > engine::PI) dy -= 2 * engine::PI;
                while (dy < -engine::PI) dy += 2 * engine::PI;
                a.pullYawOffset = dy;
            }
            a.pullLen = std::min(std::max(d * 2.5, Real(8)), Real(30));
            a.pullS = 0;
        }
    }
}

// How far into the pull-out a car is drawn: 1 at its space, 0 in its lane.
// PURELY A DRAWING MATTER -- the sim drives the lane exactly as before. Moving
// the sim's own position/heading instead stalled the cars: a car that starts
// pointed away from its lane will not accelerate, and a stopped car cannot
// turn, so 84% of pulling-out samples stood still.
Real CitySim::pullOutWeight(const Agent& a, Real pullS) {
    if (a.pullLen <= 0) return 0;
    const Real t = std::min(pullS / a.pullLen, Real(1));
    return 1 - t * t * (3 - 2 * t);
}

void CitySim::refreshPose(Agent& a) {
    if (!a.moving || a.leg >= static_cast<int>(a.route.links.size())) return;
    int li = a.route.links[a.leg];
    Real L = nav_->links[li].length;
    Real s = a.distOnLeg;
    if (s < 0) s = 0; else if (s > L) s = L;

    // Sample this agent's own guide line (lane centre / sidewalk) on a link.
    auto sample = [&](int link, Real t) {
        if (t < 0) t = 0; else if (t > 1) t = 1;
        if (a.mode != Agent::Mode::Driver) return nav_->sidewalkPoint(link, t);
        const engine::NavLink& LL = nav_->links[link];
        const int lanes = std::max(1, LL.lanes);
        // FRACTIONAL lane (device: visible lane changes): laneF eases toward
        // a.lane, so the car glides across the dashes instead of teleporting.
        const Real flMax = Real(lanes - 1) +
            (LL.klass == engine::RoadClass::Freeway ? Real(1) : Real(0));
        const Real fl = std::min(std::max(a.laneF, Real(0)), flMax);
        const Real spacing = laneSpacingFor(link);
        Real off = (0.5 + fl) * spacing;
        if (LL.oneWay) off -= lanes * 0.5 * spacing;
        // WAVER (device): a human hand is never perfectly still — but only at
        // SPEED (device: "weaving from side to side when a car is going slow
        // shouldn't be... Only on the freeway going at a high speed does it
        // make sense"). The old gate min(1, speed/4) saturated at 14 km/h —
        // below EVERY cruise speed in the sim — so it was a no-op and the
        // weave ran full amplitude at all speeds. Amplitude now fades in
        // above city pace (~14 m/s; arterials barely, freeways fully). The
        // phase runs on a CONSTANT clock: the old speed-scaled frequency
        // teleported the phase whenever speed changed, reading as a buzz
        // exactly where speed changes most (accelerating up an on-ramp).
        const Real phase = (a.home * 2.399 + a.work * 1.117);
        const Real waverGain =
            std::min(Real(1), std::max(Real(0), (a.speed - 14.0) / 12.0));
        off += 0.12 * std::sin(simSeconds_ * 0.55 + phase) * waverGain;
        Vec2 cpt = nav_->pointOnLink(link, t);
        const Vec2 d = nav_->direction(link);
        return cpt + Vec2(d.y, -d.x) * off;
    };
    a.pos = sample(li, L > 1e-9 ? s / L : 0.0);
    {   // continuous carriageway height: corridor decks/ramps lerp their
        // node elevations along the link; layer keeps legacy bridges lifted
        const engine::NavLink& EL = nav_->links[li];
        const Real et = L > 1e-9 ? s / L : 0.0;
        // one convention for the same-level tests: height ABOVE THE GROUND on every link (NavLink::aboveA/B,
        // resolved at load); the absolute Y only draws the car on a deck
        a.elevation = EL.aboveA + (EL.aboveB - EL.aboveA) * et;
        a.deckY = EL.elevAbsolute ? EL.elevA + (EL.elevB - EL.elevA) * et : Real(-1e30);
        a.grade = (EL.elevB - EL.elevA) / std::max(Real(1), EL.length);
    }

    // Corner-cut blending (device fix): the lane/sidewalk offset direction
    // rotates with each leg, so sampling only the CURRENT leg makes the pose
    // JUMP sideways the tick a node is crossed — on a wide arterial several
    // metres in one frame, a car visibly blinking across the turn. Near an
    // interior node, trace a quadratic Bezier from the incoming guide line to
    // the outgoing one instead: continuous through the corner (u = 0.5 exactly
    // at the leg change, both halves sampling the same curve), and it reads as
    // the arc the rate-limited heading was already pretending to drive.
    auto blendSpan = [&](int la, int lb) {
        Real B = std::max(nav_->links[la].width, nav_->links[lb].width) * 0.5 + 1.0;
        // A DRIVER's corner must be traceable by a real car: the quadratic's
        // apex radius is ~B/2, so B ~= 10 gives ~5 m — the physical sedan's
        // actual lock radius. The old width-derived span produced ~2.5 m
        // apexes — the position curve was tighter than the sim's OWN heading
        // model (kCarMinTurnRadius) could steer. NEAR-REVERSALS keep the
        // tight span: a 180-degree "arc" with a 10 m control span sweeps a
        // 20 m loop across the whole carriageway (dead-end U-turn tips
        // ploughed the oncoming queue).
        if (a.mode == Agent::Mode::Driver) {
            const engine::Vec2 da = nav_->direction(la);
            const engine::Vec2 db = nav_->direction(lb);
            if (da.x * db.x + da.y * db.y > -0.5)
                B = std::max(B, Real(10.0));
        }
        B = std::min(B, nav_->links[la].length * 0.45);
        B = std::min(B, nav_->links[lb].length * 0.45);
        return B;
    };
    auto corner = [&](int la, int lb, Real B, Real u) {   // u in [0,1] across the node
        Real La = nav_->links[la].length, Lb = nav_->links[lb].length;
        engine::Vec2 p0 = sample(la, La > 1e-9 ? (La - B) / La : 0.0);
        engine::Vec2 c = (sample(la, 1.0) + sample(lb, 0.0)) * 0.5;
        if (a.mode == Agent::Mode::Pedestrian) {
            // MITER the walker's corner (roads-v2 S8): averaging the two
            // guide endpoints CHORDS the offset envelope's arc, so on the
            // inside of a bend the blended path dipped ~1.5 m into the
            // carriageway — measured by the road-walk gate as 738 asphalt
            // ticks, all at degree-2 bends. Push the control point out to
            // the miter (offset / cos(halfAngle), clamped 2x — the same rule
            // the mesher's rings use). A car keeps the cut: arcing INTO the
            // junction on the carriageway is what driving through it is.
            const engine::Vec2 nodeP = nav_->nodes[nav_->links[la].to];
            const engine::Vec2 m = c - nodeP;
            const Real m2 = m.lengthSquared();
            const Real off = std::max((sample(la, 1.0) - nodeP).length(),
                                      (sample(lb, 0.0) - nodeP).length());
            if (m2 > 1e-9) {
                Real f = std::min((off * off) / m2, Real(4.0));
                c = nodeP + m * f;
            }
        }
        engine::Vec2 p2 = sample(lb, Lb > 1e-9 ? B / Lb : 1.0);
        Real v = 1.0 - u;
        return p0 * (v * v) + c * (2.0 * u * v) + p2 * (u * u);
    };
    const int legCount = static_cast<int>(a.route.links.size());
    // The path's own direction where the pose lands: the corner curve's
    // tangent inside a blend, else the link's.
    Vec2 tangent = nav_->direction(li);
    auto cornerAt = [&](int la, int lb, Real B, Real u) {
        const Vec2 ahead = corner(la, lb, B, std::min(Real(1), u + 0.02));
        const Vec2 behind = corner(la, lb, B, std::max(Real(0), u - 0.02));
        const Vec2 t = ahead - behind;
        const Real tl = t.length();
        if (tl > 1e-6) tangent = t * (1.0 / tl);
        return corner(la, lb, B, u);
    };
    if (a.leg + 1 < legCount) {              // exit half: approaching the node
        int nli = a.route.links[a.leg + 1];
        Real B = blendSpan(li, nli);
        if (B > 1e-6 && L - s < B)
            a.pos = cornerAt(li, nli, B, (B - (L - s)) / (2.0 * B));
    }
    if (a.leg > 0) {                         // entry half: just past the node
        int pli = a.route.links[a.leg - 1];
        Real B = blendSpan(pli, li);
        if (B > 1e-6 && s < B)
            a.pos = cornerAt(pli, li, B, 0.5 + s / (2.0 * B));
    } else if (wander_ && a.arrivedLink >= 0 &&
               nav_->links[a.arrivedLink].to == nav_->links[li].from) {
        // A CHAINED wander trip (arrival rolled straight into the next route):
        // blend leg 0 from the link the agent arrived along, exactly like an
        // interior corner — without this the restart popped onto the new lane
        // line, stacking simultaneous chainers on the node point.
        Real B = blendSpan(a.arrivedLink, li);
        if (B > 1e-6 && s < B)
            a.pos = cornerAt(a.arrivedLink, li, B, 0.5 + s / (2.0 * B));
    }
    if (a.mode != Agent::Mode::Driver) return;
    a.pathDir = tangent;

    // JUNCTION TURNS ACROSS SHORT LINKS. The blend above caps its span at 45%
    // of each link, which on metro's ~4 m links turned a 90-degree junction
    // turn into a ~2 m kink: the position whipped round the corner and the
    // rate-limited nose trailed it, so a turning car CRABBED sideways (measured:
    // 4.5% of moving cars more than 40 degrees off their direction of travel),
    // its body swung across the next lane, and a car that stopped mid-turn
    // stayed stuck pointing at the kerb -- a stopped car cannot yaw. So a
    // TURN at a junction traces one curve over the full span, reaching back and
    // forward across as many links as it needs, and the path tangent there is
    // what steer() turns the nose toward.
    {
        const int legN = legCount;
        const Real kReach = 14.0;
        int kin = -1;
        Real d = 0;   // route metres from the car ahead to the node (< 0: past it)
        {
            Real ahead = L - s;
            for (int k = a.leg; k + 1 < legN && ahead <= kReach; ++k) {
                if (k > a.leg) ahead += nav_->links[a.route.links[k]].length;
                if (ahead > kReach) break;
                if (nav_->isJunction(nav_->links[a.route.links[k]].to)) { kin = k; d = ahead; break; }
            }
            Real behind = s;
            for (int k = a.leg; k > 0 && behind <= kReach; --k) {
                if (k < a.leg) behind += nav_->links[a.route.links[k]].length;
                if (behind > kReach) break;
                if (nav_->isJunction(nav_->links[a.route.links[k]].from)) {
                    if (kin < 0 || behind < d) { kin = k - 1; d = -behind; }
                    break;
                }
            }
        }
        if (kin >= 0) {
            const int lin = a.route.links[kin], lout = a.route.links[kin + 1];
            const Vec2 din = nav_->direction(lin), dout = nav_->direction(lout);
            const Real cosT = din.x * dout.x + din.y * dout.y;
            if (cosT < 0.94 && cosT > -0.5) {   // a real turn, not a reversal
                // Span: the per-link rule's ~10 m, bounded by the street run on
                // either side (to the neighbouring junction or the route's end)
                // so two corners' curves never overlap.
                Real B = std::max(Real(10.0), std::max(nav_->links[lin].width,
                                                       nav_->links[lout].width) * 0.5 + 1.0);
                Real runIn = 0, runOut = 0;
                for (int k = kin; k >= 0 && runIn < 2.5 * B; --k) {
                    runIn += nav_->links[a.route.links[k]].length;
                    if (nav_->isJunction(nav_->links[a.route.links[k]].from)) break;
                }
                for (int k = kin + 1; k < legN && runOut < 2.5 * B; ++k) {
                    runOut += nav_->links[a.route.links[k]].length;
                    if (nav_->isJunction(nav_->links[a.route.links[k]].to)) break;
                }
                B = std::min(B, std::min(runIn, runOut) * 0.45);
                if (B > 0.5 && std::fabs(d) < B) {
                    auto back = [&](Real m) {   // the guide line m before the node
                        int k = kin;
                        Real Lk = nav_->links[a.route.links[k]].length;
                        while (m > Lk && k > 0) { m -= Lk; --k; Lk = nav_->links[a.route.links[k]].length; }
                        return sample(a.route.links[k], Lk > 1e-9 ? (Lk - m) / Lk : 0.0);
                    };
                    auto fwd = [&](Real m) {    // ...and m after it
                        int k = kin + 1;
                        Real Lk = nav_->links[a.route.links[k]].length;
                        while (m > Lk && k + 1 < legN) { m -= Lk; ++k; Lk = nav_->links[a.route.links[k]].length; }
                        return sample(a.route.links[k], Lk > 1e-9 ? m / Lk : 1.0);
                    };
                    const Vec2 p0 = back(B), p2 = fwd(B);
                    const Vec2 c = (back(0) + fwd(0)) * 0.5;
                    const Real u = (B - d) / (2.0 * B);
                    const Real v = 1.0 - u;
                    a.pos = p0 * (v * v) + c * (2.0 * u * v) + p2 * (u * u);
                    const Vec2 tan = (c - p0) * (2.0 * v) + (p2 - c) * (2.0 * u);
                    const Real tl = tan.length();
                    if (tl > 1e-6) a.pathDir = tan * (1.0 / tl);
                }
            }
        }
    }
}

// Turn the heading toward the current leg's direction. A pedestrian pivots
// freely; a car is rate-limited so its path curvature never tightens past
// kCarMinTurnRadius — at speed v it may yaw at most v / radius rad/s, which is
// what traces a smooth arc through a junction instead of an instant snap.
void CitySim::steer(Agent& a, Real dt) {
    if (a.leg >= static_cast<int>(a.route.links.size())) return;
    Vec2 desired = nav_->direction(a.route.links[a.leg]);
    if (a.mode != Agent::Mode::Driver) { a.heading = desired; return; }
    if (a.pathDir.x != 0 || a.pathDir.y != 0) desired = a.pathDir;   // the arc's tangent
    // Yaw rate is proportional to speed: at v the tightest arc is kCarMinTurnRadius,
    // so the car may turn at most v / radius rad/s. A stopped car cannot change
    // heading at all (like a real car) — it holds until it rolls, which also means
    // a car halted at a light never snaps its heading. Trip start seeds the initial
    // heading directly (startTrip), so a just-launched car is already aligned.
    // CHANGING LANES the nose points where the car is going, the lane's way plus the sideways glide -- it used to
    // point straight down the lane and slide across it like a crab
    if (std::fabs(a.laneVel) > 1e-3 && a.speed > 1.0) {
        const Real lat = a.laneVel * laneSpacingFor(a.route.links[a.leg]);   // m/s, + = toward the kerb
        const Vec2 right(desired.y, -desired.x);
        Vec2 d = desired * a.speed + right * lat;
        const Real dlen = std::sqrt(d.x * d.x + d.y * d.y);
        if (dlen > 1e-6) desired = d * (1.0 / dlen);
    }
    Real rate = a.speed / kCarMinTurnRadius;
    a.heading = rotateToward(a.heading, desired, rate * dt);
}

// THE NEXT JUNCTION IS NOT ALWAYS AT THE END OF THIS LINK. The junction rules
// used to look only at the current link's end node -- fine when a link runs
// junction to junction, but metro's streets are chains of ~4 m links (the
// road polyline's own vertices). A 4 m link cannot hold a 9 m stop setback, so
// the line was clamped to 40% of the link: 2.4 m short of the node, INSIDE the
// box. And the car one link back saw no junction at all (Glenn, 2026-09-19:
// "they should not stop in the middle of the intersection ... They should
// stop before the cross walk"). So the rules now look ahead along the route to
// the next real junction and measure the line in route metres.
CitySim::JunctionAhead CitySim::junctionAhead(const Agent& a, Real horizon) const {
    JunctionAhead j;
    const int legN = static_cast<int>(a.route.links.size());
    if (a.leg >= legN) return j;
    Real ahead = nav_->links[a.route.links[a.leg]].length - a.distOnLeg;
    for (int k = a.leg; k < legN; ++k) {
        const engine::NavLink& lk = nav_->links[a.route.links[k]];
        if (k > a.leg) ahead += lk.length;
        if (nav_->isJunction(lk.to)) {
            j.leg = k;
            j.node = lk.to;
            j.toNode = ahead;
            // The approach runs back to the previous junction (or the route's
            // start); a line never sits behind it. 60 m is past any setback's
            // reach (the clamp below stops growing at setback / 0.6).
            Real app = lk.length;
            for (int b = k; b > 0 && app < 60.0 &&
                            !nav_->isJunction(nav_->links[a.route.links[b]].from);
                 --b)
                app += nav_->links[a.route.links[b - 1]].length;
            j.approach = app;
            return j;
        }
        if (ahead >= horizon) break;
    }
    return j;
}

// Where this agent's STOP LINE sits, in metres back from the junction node. A
// walker holds at the corner; a car holds with its front bumper short of the
// painted zebra band on its approach: junction box radius (the mouth) + the
// band + a margin + half its own body. Stopping at the node itself parked a
// legally-waiting car in the middle of the intersection (device round 3).
//
// The ONE effective stop line every junction rule agrees on: the setback short
// of the node, but never behind the start of the APPROACH — an approach shorter
// than the setback (junctions a few metres apart) gets a line partway down it,
// and the smooth brake, the yield scan, and the hard clamp all use THIS.
// (Gating the rules on the raw setback used to disable box occupancy + turn
// yield entirely on short approaches — exactly where junctions are densest.)
Real CitySim::stopLineBack(const Agent& a, const JunctionAhead& ja) const {
    Real stopSetback = 0.5;
    if (a.mode == Agent::Mode::Driver) {
        Real halfLen = 2.1;   // sedan fallback
        if (a.vehicle >= 0 && a.vehicle < static_cast<int>(vehicles_.size()))
            halfLen = vehicles_[a.vehicle].length * 0.5;
        stopSetback = junctionRadius(ja.node) + kCrosswalkFarEdge + kStopLineMargin +
                      halfLen;
    }
    return std::min(stopSetback, std::max(Real(0.5), ja.approach * 0.6));
}

// The junction rules of advance(): the stop-line geometry, the signal brake, and
// the box-occupancy / turn-yield / exit-room scan. Returns the speed target
// after those caps plus the stop line the hard clamp in advance() holds at.
// Pure query: no rng draws, no agent mutation.
Move CitySim::moveFor(const Agent& a, int li) const {
    for (std::size_t k = static_cast<std::size_t>(std::max(0, a.leg)); k + 1 < a.route.links.size(); ++k)
        if (a.route.links[k] == li) return moveOf(nav_->direction(li), nav_->direction(a.route.links[k + 1]));
    return Move::Straight;   // its last leg (or not on its route): straight on
}

SignalState CitySim::signalFor(const Agent& a, int li) const {
    return signals_.stateFor(li, moveFor(a, li));
}

CitySim::JunctionGate CitySim::junctionSpeedCap(const Agent& a, int li,
                                                Real target) const {
    JunctionGate gate;
    bool car = a.mode == Agent::Mode::Driver;
    // Cars look ahead along the route (see junctionAhead); walkers keep the
    // current link -- their kerb rules are written per link. The reach covers
    // the longest setback (a bus at a wide box, ~25 m) plus the signal
    // approach.
    const JunctionAhead ja = junctionAhead(a, car ? kSignalApproach + 34.0 : 0.0);
    if (ja.leg < 0) {
        gate.cap = target;
        return gate;
    }
    li = a.route.links[static_cast<std::size_t>(ja.leg)];   // the APPROACH link
    const int jLeg = ja.leg;
    const int toNode = ja.node;
    JunctionAhead jw = ja;
    if (!car) jw.approach = nav_->links[li].length;   // walkers: their link
    const Real lineBack = stopLineBack(a, jw);
    bool yieldAtLine = false;   // a TURNING car holding for oncoming traffic
    {
        Real distToEnd = ja.toNode;
        // Box pace, eased into: kJunctionSpeed by kJunctionApproach metres out,
        // braking comfortably (half the stop decel) before that. The cap used
        // to switch on AT 9 m -- a car at 13 m/s dropped to 4 in one tick.
        if (car)
            target = std::min(target,
                              std::sqrt(kJunctionSpeed * kJunctionSpeed +
                                        kCarDecel * std::max(Real(0), distToEnd - kJunctionApproach)));
        // Obey the stoplight. Cars go on THEIR approach's green; pedestrians
        // cross ONLY in the WALK window (roads-v2.1 R3) — the conflict-group
        // phases mean a "green approach" no longer guarantees the crossing
        // street is red, so the old ride-the-perpendicular-green rule would
        // walk peds into moving turns. WALK is everything-red by construction.
        Real distToLine = ja.toNode - lineBack;
        gate.distToLine = distToLine;
        // Only cars still BEFORE the line brake for the signal — a car already
        // past it is COMMITTED and must clear the box (the all-red clearance
        // exists for exactly that). Capping a committed car to zero pinned it
        // mid-intersection every time a green expired under it.
        // Cars AND peds ride their own approach's green — for peds this is
        // the PARALLEL-WALK rule, and under CONFLICT-GROUP phases it is
        // correct BY CONSTRUCTION: every arm a parallel-walking ped crosses
        // conflicts with their approach, so it is red during their green.
        // (Under the old bearing-parity bins that guarantee did not hold —
        // and fixed-time WALK gates could not cover variable crossing
        // widths: a 9 s gate left peds mid-box on arterials. The WALK window
        // remains as bonus all-red scramble time.) Turning cars crossing the
        // walkway on green brake for peds via the vision wedge, as before.
        bool signalHolds =
            car ? signalFor(a, li) != SignalState::Green
                : !(signals_.stateForLink(li) == SignalState::Green ||
                    signals_.walkRemainingAt(toNode) >= 6.5);
        if (distToLine >= 0 && distToLine < kSignalApproach && signals_.hasSignal(li) &&
            signalHolds) {
            target = std::min(target, approachStop(distToLine, car ? kCarDecel : kPedDecel,
                                                   target));
        }
        // HOLDS at the line (cars still before it, any signal state):
        //  1. BOX OCCUPANCY — never enter while a car on a CROSSING heading is
        //     inside the box (a lingerer from the last phase, a mid-turn car):
        //     entering anyway is where mid-box wrecks came from. Same-axis
        //     occupants (leaders ahead, opposing straight-through on the other
        //     side of the road) don't hold anyone.
        //  2. TURN YIELD — a TURN also waits for live ONCOMING approach traffic
        //     (opposing arms share a green; a left turn crosses it). Going
        //     straight never yields, so two held turners can't deadlock; two
        //     opposing stopped turners break the tie by agent order.
        if (car && distToLine >= 0 && distToLine < kSignalApproach &&
            a.leg < static_cast<int>(a.route.links.size())) {
            Vec2 d0 = nav_->direction(li);
            bool turning = false;
            if (jLeg + 1 < static_cast<int>(a.route.links.size())) {
                Vec2 d1 = nav_->direction(a.route.links[jLeg + 1]);
                turning = d0.x * d1.x + d0.y * d1.y < 0.85;   // a real bend
            } else if (wander_) {
                // Final leg in wander mode: arrival CHAINS straight through this
                // node onto an unknown next route — treat it as a turn so a
                // chained continuation yields to live oncoming traffic instead
                // of sweeping across it (the crash-at-the-junction generator).
                turning = true;
            }
            Vec2 jc = nav_->nodes[toNode];
            Real jr = junctionRadius(toNode);
            Real range = jr + 6.0;
            // on its green ARROW a left turn is protected: everything it would cross is red
            const bool protectedTurn = turning && signals_.protectedLeft(li) && moveFor(a, li) == Move::Left;
            // Gridlock escape: held this long by nothing but STALLED occupants, a
            // real driver inches through the box. Staggered per agent (brain bits)
            // so a ring of mutual waiters releases one at a time, deterministic.
            const bool gridlocked =
                a.holdTimer > 6.0 + static_cast<Real>(a.brain % 4u) * 1.5;
            // Grid candidates (P4.1): every rule below re-applies its exact
            // range/heading predicate, so the verdict matches the full scan.
            grid_.query(jc, range + 4.0, queryScratch_);
            for (int bi : queryScratch_) {
                const Agent& b = agents_[bi];
                if (&b == &a || b.mode != Agent::Mode::Driver) continue;
                if (b.far()) continue;   // far tier: no body
                if (!b.moving || b.released) continue;
                if (std::fabs(b.elevation - a.elevation) > 3.0) continue;
                Real dx = b.pos.x - jc.x, dy = b.pos.y - jc.y;
                Real db2 = dx * dx + dy * dy;
                if (db2 > range * range) continue;
                Real along = b.heading.x * d0.x + b.heading.y * d0.y;
                // 1: crossing-heading occupant INSIDE the box proper. A stalled
                // occupant stops counting once this car is gridlocked — waiting
                // on a car that is itself waiting is how the circular wait forms.
                if (db2 < jr * jr && std::fabs(along) < 0.7) {
                    if (gridlocked && b.speed < 0.3) continue;
                    yieldAtLine = true;
                    break;
                }
                // 2: turn yield against oncoming approach traffic.
                if (!turning || along >= -0.3 || protectedTurn) continue;
                if (b.speed > 0.5) { yieldAtLine = true; break; }  // live traffic
                // A gridlocked car stops waiting on anything STALLED — including
                // the stopped-turner tie-break below, whose cross-junction chains
                // (A waits on B waits on C...) were the 400-second holds. Only
                // live moving traffic holds it now; the staggered release means
                // opposing turners still don't sweep simultaneously.
                if (gridlocked) continue;
                if (&b < &a && b.leg + 1 < static_cast<int>(b.route.links.size())) {
                    const JunctionAhead jb = junctionAhead(b, range + 4.0);
                    if (jb.node == toNode &&
                        jb.leg + 1 < static_cast<int>(b.route.links.size())) {
                        Vec2 bd0 = nav_->direction(b.route.links[jb.leg]);
                        Vec2 bd1 = nav_->direction(b.route.links[jb.leg + 1]);
                        if (bd0.x * bd1.x + bd0.y * bd1.y < 0.85) {
                            yieldAtLine = true;
                            break;
                        }
                    }
                }
            }
            // 3: EXIT ROOM -- don't block the box (Glenn, 2026-09-19: "they
            //    should not stop in the middle of the intersection. That creates
            //    an instant traffic jam"). Rule 1 only holds for CROSSING
            //    occupants, so a car stopped in the box heading the same way --
            //    its exit full -- held nobody, and the cars behind followed it
            //    in: measured on metro, ~3 cars at any moment stopped in a box,
            //    and pile-ups there rolled back onto one another by the contact
            //    rule. Enter only if the WHOLE car fits past the box behind the
            //    car ahead in its lane (or that car is moving away). The lane
            //    chain's leader is exactly that car, wherever it stands: in the
            //    box short of the node, or queued on the street beyond it.
            //    No gridlock override: waiting at the line is always right.
            if (!yieldAtLine && jLeg + 1 < static_cast<int>(a.route.links.size())) {
                const std::size_t self = static_cast<std::size_t>(indexOf(a));
                if (self < gaps_.size() && gaps_[self] < 1e8 &&
                    leaderSpeeds_[self] < 3.0) {
                    const Real clearAt = ja.toNode + jr +
                                         0.5 * vehicleLength(static_cast<int>(self)) +
                                         minGaps_[self];
                    if (gaps_[self] < clearAt) yieldAtLine = true;
                }
            }
            if (yieldAtLine)
                target = std::min(target, approachStop(distToLine, kCarDecel, target));
        }
    }
    gate.cap = target;
    gate.yieldAtLine = yieldAtLine;
    gate.node = toNode;
    gate.approachLink = li;
    gate.distToNode = ja.toNode;
    return gate;
}

// The perception block of advance() (ADR-0063: sense -> remember -> predict ->
// decide -> act). Each tick the car SENSES pedestrians and the player through its
// 2.5D sensor — the forward wedge plus a height band, so a walker crossing the
// overpass is not a phantom to brake for — and REMEMBERS the sightings as tracks
// with velocity estimates. It then acts on the MEMORY, not the snapshot: a person
// who slipped out of the cone is still yielded to where they're HEADING for a
// few seconds (object permanence), and a crosser on a collision course is
// braked for BEFORE entering the lane (time-to-collision on the tracked
// velocity). We still do NOT sense other AI cars: car-vs-car is resolved by
// lanes, same-lane car-following (advance()'s gap cap) and the signals —
// braking for every car in a wide cone deadlocked junctions (see step()).
// Imperfect: with probability (1 - reliability) this step's sighting is
// missed (a fault) — memory makes a missed frame degrade gracefully instead
// of blinding the car outright. Returns the distance to the nearest person (or
// predicted collision point) ahead; infinity when nothing demands a brake.
Real CitySim::senseAhead(Agent& a) {
    Real seenAhead = std::numeric_limits<Real>::infinity();   // dist to a person ahead
    engine::VisionCone cone;
    cone.origin = a.pos;
    // Look where the car is GOING. The nose trails the path through a turn
    // (rate-limited yaw), so a cone on the nose missed a walker off the
    // front corner the car was turning toward.
    cone.forward = (a.pathDir.x != 0 || a.pathDir.y != 0) ? a.pathDir : a.heading;
    cone.range = 18.0;
    cone.halfAngleRad = 0.45;    // ~26 deg: a crosser in the lane ahead,
                                 // not someone standing on the far sidewalk
    // A CAR is looked for down the LANE, as far as it takes to stop from this speed plus both bodies: the 18 m
    // person cone saw a stopped car only once it was too late to stop from 50 km/h, and its 26 deg width at range
    // would brake for the next lane. A corridor a lane wide, straight along where we are going.
    const Vec2 dir = cone.forward;
    const Real ownHalf = a.vehicle >= 0 ? fleetBody(a.vehicle).length * 0.5 : Real(2.2);
    const Real carRange = std::max(Real(18), a.speed * a.speed / (2 * Real(4.5)) + ownHalf + 12);
    auto inLaneAhead = [&](const Vec2& p) {
        const Real dx = p.x - a.pos.x, dz = p.y - a.pos.y;
        const Real fwd = dx * dir.x + dz * dir.y, lat = std::fabs(dx * dir.y - dz * dir.x);
        return fwd > 0 && fwd < carRange && lat < kLaneHalfCorridor;
    };
    if (brainUnit(a) <= a.reliability) {
        engine::SensorVolume sensor;
        sensor.cone = cone;
        // Agent-backed ghosts come from the grid (P4.1) — candidates sorted
        // ascending, so sightings land in memory in the same order the full
        // sensed_ scan produced. The slack past the cone range covers a
        // TETHERED walker's ghost sitting at its body anchor (± its lead).
        grid_.query(a.pos, cone.range + 12.0, queryScratch_);
        for (int gi : queryScratch_) {
            if (sensedIndex_[gi] < 0) continue;   // not a sensable body this step
            const SensedGhost& g = sensed_[sensedIndex_[gi]];
            if (engine::sees(sensor, g.pos, g.elevation - a.elevation))
                a.memory.observe(g.id, g.pos, simSeconds_);
        }
        // External points (the live player) carry no elevation from the
        // host — skip the height gate rather than invent one for them.
        for (const SensedGhost& g : sensed_) {
            if (g.id >= 0) continue;   // agent ghosts handled via the grid above
            if (externalHalfOf(g.id) > 0 ? inLaneAhead(g.pos) : engine::sees(sensor, g.pos, 0.0))
                a.memory.observe(g.id, g.pos, simSeconds_);
        }
    } else {
        ++faultCount_;
    }
    a.memory.update(simSeconds_);
    for (const engine::TrackedBody& t : a.memory.tracks()) {
        if (t.confidence < kMemoryActConfidence) continue;
        // A CAR ahead (the player's): in the lane corridor, held short by both bodies, not a person's clearance
        const Real oh = externalHalfOf(t.id);
        if (oh > 0) {
            if (!inLaneAhead(t.pos)) continue;
            const Real extra = ownHalf + oh + kCarBuffer - kPedClearance;
            Real fd = (t.pos.x - a.pos.x) * dir.x + (t.pos.y - a.pos.y) * dir.y;
            seenAhead = std::min(seenAhead, std::max(Real(0), fd - extra));
            Real ttc = engine::timeToCollision(a.pos, a.heading * a.speed, t.pos, t.vel, ownHalf + oh);
            if (ttc < kTtcHorizon) seenAhead = std::min(seenAhead, std::max(Real(0), a.speed * ttc - extra));
            continue;
        }
        // Where memory says the body IS (extrapolated while unseen): yield
        // if that estimate sits in the corridor ahead.
        if (engine::sees(cone, t.pos)) {
            Real fd = engine::forwardDistance(cone, t.pos);
            if (fd > 0) seenAhead = std::min(seenAhead, fd);
        }
        // Where memory says it's GOING: if our courses collide within the
        // horizon, brake as if the collision point were an obstacle that
        // far ahead — anticipation, not reaction.
        Real ttc = engine::timeToCollision(a.pos, a.heading * a.speed,
                                           t.pos, t.vel, kCollisionRadius);
        if (ttc < kTtcHorizon)
            seenAhead = std::min(seenAhead, std::max(Real(0), a.speed * ttc));
    }
    return seenAhead;
}

void CitySim::advance(Agent& a, Real dt, Real gap, Real minGap) {
    if (!a.moving) return;
    if (a.busDwell > 0) {   // doors open at a stop: stand still
        a.busDwell = std::max<Real>(0, a.busDwell - dt);
        a.speed = 0;
        return;
    }
    int li = a.route.links[a.leg];
    bool car = a.mode == Agent::Mode::Driver;
    if (car) {
        // LANE LIFE (device: "see the cars change lanes ... signal with
        // their turn signals"): laneF EASES toward a.lane (~2 s per lane) so
        // the change is a visible glide the lamp bake can indicate; a paced
        // discretionary change picks a neighbouring lane now and then on any
        // multi-lane link. Deterministic: agent-keyed hash, sim-clock paced.
        // THE GLIDE (Glenn, 2026-10-02: "Lane changing for simulated cars is wonky"): a critically damped spring,
        // not a constant-rate slide -- the old glide started and stopped sideways at full rate, a kink at each
        // end. ~2.6 s a lane, peak ~0.55 lanes/s; steer() turns the nose into it.
        {
            constexpr Real w = 1.8, kMaxRate = 0.6;
            const Real e = Real(a.lane) - a.laneF;
            a.laneVel += (w * w * e - 2 * w * a.laneVel) * dt;
            a.laneVel = std::max(-kMaxRate, std::min(kMaxRate, a.laneVel));
            a.laneF += a.laneVel * dt;
            if (std::fabs(Real(a.lane) - a.laneF) < 0.002 && std::fabs(a.laneVel) < 0.01) {
                a.laneF = Real(a.lane);
                a.laneVel = 0;
            }
        }
        const Real dl = Real(a.lane) - a.laneF;
        a.laneTimer -= dt;
        if (a.laneTimer <= 0) {
            const engine::NavLink& LL = nav_->links[li];
            const int lanes = std::max(1, LL.lanes);
            uint32_t h = static_cast<uint32_t>(a.home * 73 + a.work * 131 +
                                               a.trips * 17 + a.leg) *
                         2654435761u;
            a.laneTimer = 1.0 + static_cast<Real>((h >> 8) % 10) * 0.1;   // a decision every 1-2 s
            // ROAD RULES (Glenn, 2026-10-02: "Should we mimic road rules? Faster cars are in the furthest lane?" --
            // "our cars and roads are left lane like the uk or japan. And I want to keep it that way"). Traffic
            // drives on the LEFT: lane 0 lies next to the centre line (the OFFSIDE, overtaking lane) and the last
            // lane is the NEARSIDE, the kerb. (NavGraph::rightOf is the kerb side: in world x/z it points left.)
            // Not at a junction: nobody changes lane in the box or on its threshold.
            if (lanes > 1 && a.speed > 3.0 && std::fabs(dl) < 0.05 && !nearJunction(a.pos, 6.0)) {
                const int kerb = lanes - 1;
                const int legCount = static_cast<int>(a.route.links.size());
                // 1. THE NEXT TURN within ~90 m decides first: a LEFT turn (to the nearside) from the kerb lane, a
                // RIGHT turn (across the oncoming traffic) from the offside lane, an exit from the kerb lane;
                // straight on through a junction leaves the lane free
                int turnLane = -1;
                Real ahead = LL.length - a.distOnLeg;
                for (int lg = a.leg; lg + 1 < legCount && ahead < 90.0; ++lg) {
                    const int c0 = a.route.links[lg], c1 = a.route.links[lg + 1];
                    const engine::NavLink& cur = nav_->links[c0];
                    const engine::NavLink& nxt = nav_->links[c1];
                    if (cur.klass == engine::RoadClass::Freeway && nxt.klass == engine::RoadClass::Ramp) { turnLane = kerb; break; }
                    if (nav_->isJunction(cur.to)) {
                        const Vec2 d0 = nav_->direction(c0), d1 = nav_->direction(c1);
                        const Vec2 nearside(d0.y, -d0.x);
                        const Real side = d1.x * nearside.x + d1.y * nearside.y;
                        if (side > 0.4) turnLane = kerb;
                        else if (side < -0.4) turnLane = 0;
                        break;
                    }
                    ahead += nxt.length;
                }
                // 2. KEEP LEFT EXCEPT TO OVERTAKE: held up by a slower car close ahead, pull out one lane toward the
                // offside; otherwise drift back to the nearside, one lane at a time, once there is room
                int preferred = a.lane;
                if (turnLane >= 0) {
                    preferred = turnLane;
                } else {
                    const int ai = indexOf(a);
                    const Real gap = gaps_[static_cast<std::size_t>(ai)];
                    const Real leader = leaderSpeeds_[static_cast<std::size_t>(ai)];
                    const Real cruise = engine::classSpeed(LL.klass) * a.speedFactor;
                    const bool heldUp = gap < a.speed * 3.0 + 12.0 && leader < cruise * 0.85;
                    if (heldUp && a.lane > 0) preferred = a.lane - 1;
                    else if (!heldUp && a.lane < kerb) preferred = a.lane + 1;
                }
                const bool returning = preferred > a.lane && turnLane < 0;
                const int dir = (preferred > a.lane) - (preferred < a.lane);
                if (dir != 0) {
                    const int want = a.lane + dir;
                    // room check: nobody in the target lane alongside.
                    // Grid candidates (P4.1): 13 m along + the widest
                    // carriageway's lane offsets fit well inside 26 m.
                    //
                    // By POSITION, not by link: "same link, distOnLeg within
                    // 13 m" saw nobody on metro's ~4 m links but the car's
                    // own link, and cars merged into bodies one link ahead.
                    bool room = true;
                    const Real spacing = laneSpacingFor(li);
                    const Real shift = (Real(want) - a.laneF) * spacing;   // + = toward the kerb (nearside)
                    grid_.query(a.pos, 60.0, queryScratch_);
                    for (int bi : queryScratch_) {
                        const Agent& b = agents_[bi];
                        // (a STOPPED car counts: merging into a queue is merging into bodies)
                        if (&b == &a || b.mode != Agent::Mode::Driver ||
                            b.far() || b.leg >= (int)b.route.links.size())
                            continue;
                        if (b.heading.x * a.heading.x + b.heading.y * a.heading.y < 0.7)
                            continue;   // not travelling my way
                        const Real dx = b.pos.x - a.pos.x, dy = b.pos.y - a.pos.y;
                        const Real along = a.heading.x * dx + a.heading.y * dy;
                        const Real right = a.heading.y * dx - a.heading.x * dy;
                        if (std::fabs(right - shift) > spacing * 0.6) continue;
                        // room ahead and behind, plus what the speed difference closes in ~3 s: a car coming up
                        // fast in the target lane is not "room" just because it is 20 m back
                        Real need = along >= 0 ? 13.0 + std::max(Real(0), a.speed - b.speed) * 2.0
                                               : 13.0 + std::max(Real(0), b.speed - a.speed) * 3.0;
                        // back to the nearside only past the slower traffic: not into a gap behind a car you'd be
                        // stuck behind again at once (that was the weave)
                        if (returning && along >= 0 && b.speed < a.speed - 1.0) need = std::max(need, Real(40));
                        if (std::fabs(along) < need) {
                            room = false;
                            break;
                        }
                    }
                    if (room) a.lane = want;
                }
            }
        }
    }
    // Nominal pace scaled by personality (ADR-0062): drivers and walkers each
    // hold their OWN fraction of the limit, so traffic doesn't move in lockstep.
    // Junction/signal caps are shared road rules and stay unscaled.
    Real target = (car ? engine::classSpeed(nav_->links[li].klass) : kWalkSpeed) *
                  a.speedFactor;
    Real accel = car ? kCarAccel : kPedAccel;
    // Junction rules (stop line, signal brake, box occupancy / turn yield) cap
    // the target; the gate carries the line the hard clamp below holds at.
    JunctionGate gate = junctionSpeedCap(a, li, target);
    target = gate.cap;
    // CORNER FEASIBILITY (roads-v2.1 R5): a real car cannot hold a junction
    // arc at cruise (90 degrees at 8 m/s on a ~7 m arc is ~1 g lateral) — and
    // the physical tier drives THIS plan, so an infeasible corner meant the
    // body understeered past the mouth and lost its ghost. Cap the approach
    // by the bend into the NEXT leg, easing down at comfortable decel, and
    // hold the corner pace while the nose is still rotating onto the leg.
    if (car) {
        const engine::Vec2 dl = nav_->direction(li);
        auto bendCap = [](Real cosB) {
            // Floor 2.8: a corner-apex car sits in the city's SLOW contact
            // class (< 3 m/s) — the ped kerb rule's "fast car" boundary.
            return std::max(Real(2.8), Real(8.5) * std::max(Real(0), cosB));
        };
        // Every real bend within braking reach, not just the one at the end
        // of this link: on ~4 m links that one was always 0-4 m away, so the
        // cap bound in a single tick.
        {
            Real rem = std::max(Real(0), nav_->links[li].length - a.distOnLeg);
            engine::Vec2 dPrev = dl;
            for (int k = a.leg + 1;
                 k < static_cast<int>(a.route.links.size()) && rem < 45.0; ++k) {
                const engine::Vec2 dn = nav_->direction(a.route.links[k]);
                const Real cosB = dPrev.x * dn.x + dPrev.y * dn.y;
                if (cosB < 0.94) {   // > ~20 degrees: a real bend
                    const Real vC = bendCap(cosB);
                    target = std::min(
                        target, std::sqrt(vC * vC + 2 * kCarDecel * 0.6 * rem));
                }
                rem += nav_->links[a.route.links[k]].length;
                dPrev = dn;
            }
        }
        const Real cosH = a.heading.x * dl.x + a.heading.y * dl.y;
        if (cosH < 0.94) target = std::min(target, bendCap(cosH));
    }
    // Perception: how far ahead the nearest person (or predicted collision) is;
    // ease to a stop kPedClearance short of it.
    Real seenAhead = std::numeric_limits<Real>::infinity();
    if (car) {
        seenAhead = senseAhead(a);
        if (seenAhead < 1e9)
            target = std::min(target, approachStop(seenAhead - kPedClearance, kCarDecel,
                                                   target));
    }

    // minGap is length-aware (computeGaps → pairMinGap): sedan traffic reproduces
    // the old 5.0 m, and a longer body keeps a bigger bumper gap so nothing packs
    // tighter than before. Peds fall back to the fixed footprint gap.
    if (car) {
        // IDM car-following (roads-v2 plan Â§3.1): one smooth law for free
        // acceleration toward the (signal/junction/perception-capped) target
        // and braking for the leader â the anti-pile-up backbone, replacing
        // the linear followCap ramp + instantaneous down-jump. The net gap is
        // bumper-to-bumper: centre gap minus the bodies (minGap carries them
        // plus kCarBumperGap, which doubles as IDM's jam gap s0).
        const int myIdx = static_cast<int>(&a - agents_.data());
        engine::IdmParams idm;
        idm.accelMax = accel;
        idm.decelComf = kCarDecel;
        idm.minGap = kCarBumperGap;
        idm.headway = kCarHeadway;
        Real netGap = gap < 1e8
            ? std::max(Real(0.05), gap - (minGap - kCarBumperGap))
            : std::numeric_limits<Real>::infinity();
        Real dv = gap < 1e8 ? a.speed - leaderSpeeds_[myIdx] : 0.0;
        // The vision wedge may know a NEARER body than the lane chain does
        // (a merger's nose, a wreck on another link). Whoever is closest is
        // the leader IDM brakes for. A gridlock-creeping car ignores the
        // wedge: the creep exists to unpick stalemates, and the fender-bender
        // rule arbitrates any resulting brush.
        if (myIdx < static_cast<int>(carAheadGap_.size()) &&
            carAheadGap_[myIdx] < netGap && a.holdTimer <= 6.0) {
            netGap = carAheadGap_[myIdx];
            dv = a.speed - carAheadSpeed_[myIdx];
        }
        // A BUS BRAKES FOR ITS STOP. It used to arrive at speed and stand
        // still the next tick -- 5.6 m/s to 0 in 33 ms, a jolt nobody rides
        // through (Glenn: "a lot of jitter from the bus movement"). The stop
        // point (short of the junction, see the arrival rule below) is a
        // stationary leader: IDM eases the bus to rest on it.
        if (isBus(myIdx) && a.busDwell <= 0 && !a.route.links.empty()) {
            const Real toStop = busDistanceToStop(a);
            if (toStop + kCarBumperGap < netGap) {
                netGap = std::max(Real(0.05), toStop + kCarBumperGap);
                dv = a.speed;
            }
        }
        // Any wedge-IGNORING drive (gridlock creep past 6 s, tow-truck
        // escape) happens at a CRAWL while bodies are near: a full-speed
        // escape brushing a moving passerby registered as a fast
        // (dangerous-class) contact; walking pace keeps any brush in the
        // slow, arbitrated class.
        if (myIdx < static_cast<int>(carAheadGap_.size()) &&
            carAheadGap_[myIdx] < 8.0 &&
            (a.holdTimer > 6.0 || a.crashCount > 5))
            target = std::min(target, Real(1.8));
        a.speed = engine::idmStep(a.speed, std::max(Real(0.1), target), netGap,
                                  dv, dt, kCarDecel * 2.0, idm);
        // The capped target is still a hard ceiling (signals/junction rules
        // must bind immediately, not on the IDM relaxation curve).
        a.speed = std::min(a.speed, std::max(Real(0), target));
    } else {
        Real slowZone = kPedSlowZone;
        target = followCap(target, gap, minGap, slowZone);
        a.speed = std::min(target, a.speed + accel * dt);
    }

    // Gridlock clock + escape. A jam's terminal form is a RING: cars pinned
    // nose-to-nose across adjacent junctions by the follow gap (each behind the
    // next stalled car), which no yield rule will ever release. So ANY car pinned
    // to a standstill near a junction — by a yield or by a stalled leader, but
    // never by a red (the light will change) — counts its hold; once gridlocked,
    // it stops yielding to stalled traffic (junctionSpeedCap) and CREEPS, so the
    // ring inches apart the way real drivers unpick a blocked box. If bodies
    // brush during the creep, the fender-bender freeze-and-recover arbitrates.
    if (car) {
        const int jli = gate.approachLink;
        const bool redAhead = jli >= 0 && signals_.hasSignal(jli) &&
                              signalFor(a, jli) != SignalState::Green;
        const bool nearJunc = gate.node >= 0 && gate.distToNode < kSignalApproach + 12.0;
        // The gridlock clock also runs for a WEDGE-PINNED car ANYWHERE on the
        // road — a wreck pile at a link ENTRANCE (post-crash bodies
        // overlapped, wedge gap ~0) held five cars at v=0 forever, because
        // the clock only ran near junctions and the creep never fired.
        const std::size_t selfIdx =
            static_cast<std::size_t>(&a - agents_.data());
        const bool wedgePinned =
            a.speed < 0.05 && a.crashTimer <= 0 && a.crashCount > 0 &&
            selfIdx < carAheadGap_.size() && carAheadGap_[selfIdx] < 0.4;
        if ((a.speed < 0.05 && nearJunc && !redAhead && a.crashTimer <= 0) ||
            wedgePinned)
            a.holdTimer += dt;
        else if (a.speed > 1.5)   // above creep: genuinely rolling again — a
            a.holdTimer = 0;      // reset at creep speed sawtoothed the escape
        const bool gridlocked =
            a.holdTimer > 6.0 + static_cast<Real>(a.brain % 4u) * 1.5;
        if (gridlocked && !redAhead && a.crashTimer <= 0 && a.speed < 0.7)
            a.speed = 0.7;   // creep — the hard line clamp below still applies
    }

    // Hard stop line at a red light: the smooth cap above slows the agent but
    // never to exactly zero, so without this the leftover motion would carry it
    // THROUGH the light. Clamp advance so it cannot pass the line while its
    // approach is not green; it waits here until the signal clears.
    //
    // The line is in ROUTE metres and may lie beyond this link: then the motion
    // below runs the normal leg-chaining loop, capped at the line (lineRoom).
    Real lineRoom = std::numeric_limits<Real>::infinity();
    if (gate.node >= 0) {
        const int toNode = gate.node;
        const int jli = gate.approachLink;
        bool redAhead = signals_.hasSignal(jli) &&
                        (car ? signalFor(a, jli) != SignalState::Green
                             : !(signals_.stateForLink(jli) == SignalState::Green ||
                                 signals_.walkRemainingAt(toNode) >= 6.5));
        // A car that the cap has just brought to the line can overshoot it by a
        // hair in one tick; it is still waiting AT the line, not committed.
        const Real kLineSlack = car ? 0.3 : 0.0;
        if ((redAhead || gate.yieldAtLine) && gate.distToLine >= -kLineSlack - 1e-6 &&
            car && gate.distToLine > nav_->links[li].length - a.distOnLeg) {
            lineRoom = std::max(Real(0), gate.distToLine);
            a.state = redAhead ? Agent::State::Waiting : Agent::State::Yielding;
        } else if (redAhead || gate.yieldAtLine) {
            // The hold applies only BEFORE the line. A car already past it is
            // committed to the box and drives on — clamping it there trapped it
            // mid-intersection whenever a green expired under it, and everything
            // arriving cross-phase piled into it.
            if (gate.distToLine >= -kLineSlack - 1e-6) {
                Real room = std::max(Real(0), gate.distToLine);
                Real motion = std::min(a.speed * dt, room);
                if (motion < a.speed * dt) a.speed = 0;   // held at the line
                a.distOnLeg += motion;
                // FSM: red = Waiting; holding a turn for oncoming = Yielding.
                // A pedestrian held at the kerb is Waiting too (honest red ring).
                a.state = car ? (redAhead ? Agent::State::Waiting : Agent::State::Yielding)
                              : Agent::State::Waiting;
                refreshPose(a);
                steer(a, dt);
                return;
            }
        }
    }

    // S8 KERB DISCIPLINE (roads-v2 plan §3.3): at an UNSIGNALLED junction a
    // walker GAP-ACCEPTS — it waits at the kerb while cars are inbound on the
    // crossing and steps off into a gap, instead of walking out and relying
    // on every driver's brakes (the signalled case is already the stop-line
    // hold above). Assertiveness cap: after ~12 s of no gap the walker
    // crosses anyway — traffic never fully starves a pedestrian, and the
    // cars' person-sense stays the safety net, exactly like a real kerb.
    if (!car) {
        const int toNode = nav_->links[li].to;
        const Real toEnd = nav_->links[li].length - a.distOnLeg;
        // The kerb line sits BEFORE the corner-cut blend window (half the
        // road width + margin): the sidewalk path cuts through the junction
        // interior around the node, so a hold any later parks the walker IN
        // the carriageway. Past the wait band the walker is committed —
        // stopping mid-box would be strictly worse than walking on.
        const Real kerb = nav_->links[li].width * 0.5 + 3.0;
        if (nav_->isJunction(toNode) && !signals_.hasSignal(li) &&
            toEnd < kerb && toEnd > kerb - 1.8) {
            const Vec2 nodeP = nav_->nodes[toNode];
            // Patience relaxes the ACCEPTED GAP, it never removes the check:
            // fresh at the kerb, a walker wants ~2.5 s of clear approach;
            // past ~12 s of waiting it asserts into traffic — but stepping
            // out UNDER a car arriving within ~a second is not assertive,
            // it is a contact (measured: the unconditional override walked a
            // ped 1.3 m in front of a braking car's bumper).
            const bool patient = a.holdTimer < 12.0;
            bool carInbound = false;
            // Grid candidates (P4.1): the widest accepted-gap radius is
            // speed * 2.5 + 4 at the fastest personality-scaled class speed
            // (~32 m/s -> ~85 m); 92 m covers it plus a step's drift.
            grid_.query(nodeP, 92.0, queryScratch_);
            for (int ci : queryScratch_) {
                const Agent& c = agents_[ci];
                if (c.mode != Agent::Mode::Driver || c.far() ||
                    !c.moving || c.speed < 0.5)
                    continue;
                if (std::fabs(c.elevation - a.elevation) > 2.5) continue;
                const Real cd = (c.pos - nodeP).length();
                const bool near = patient ? (cd < 7.0 || cd < c.speed * 2.5 + 4.0)
                                          : (cd < 7.0 || cd < c.speed * 1.2);
                if (near) { carInbound = true; break; }
            }
            if (carInbound) {
                a.holdTimer += dt;
                a.speed = 0;
                a.state = Agent::State::Waiting;
                refreshPose(a);
                steer(a, dt);
                return;
            }
        }
        if (toEnd >= kerb + 2.0) a.holdTimer = 0;   // clear of the kerb: reset patience
    }

    Real motion = a.speed * dt;
    if (motion > lineRoom) { motion = lineRoom; a.speed = 0; }   // held at a line further on

    // A pedestrian never walks INTO a car body (roads-v2.1 R3): a car
    // standing across the walkway — queue spillback over a crosswalk, a
    // mid-box lingerer draining on the all-red — is a WALL to a walker.
    // Hold a half-step short until it moves on. (Cars sense pedestrians via
    // the vision wedge; walkers get the mirror-image body check here.)
    if (!car && motion > 0) {
        const Vec2 probe(a.pos.x + a.heading.x * 1.0,
                         a.pos.y + a.heading.y * 1.0);
        // Vehicle bodies via their DRIVERS in the grid (P4.1): every vehicle
        // mirrors its driver's pose, so driver candidates near the probe find
        // every body the vehicles_ scan found. 26 m covers the 6 m body test
        // plus a launch-tick's ghost-vs-mirror gap.
        grid_.query(probe, 26.0, queryScratch_);
        for (int di : queryScratch_) {
            const Agent& drv = agents_[di];
            if (drv.mode != Agent::Mode::Driver || drv.far() ||
                drv.vehicle < 0 ||
                drv.vehicle >= static_cast<int>(vehicles_.size()))
                continue;
            const SimVehicle& v = vehicles_[drv.vehicle];
            const Real dx = probe.x - v.pos.x, dy = probe.y - v.pos.y;
            if (dx * dx + dy * dy > 36.0) continue;
            const Real lx = std::fabs(dx * v.heading.x + dy * v.heading.y);
            const Real ly = std::fabs(-v.heading.y * dx + v.heading.x * dy);
            const Real vSpeed =
                v.driver >= 0 && v.driver < static_cast<int>(agents_.size())
                    ? agents_[v.driver].speed
                    : 0.0;
            if (lx < v.length * 0.5 + 0.35 && ly < v.width * 0.5 + 0.35 &&
                vSpeed > 1.5) {
                // Only a genuinely SWEEPING body holds the walker — below
                // that the car's own ped-yield governs, and holding for
                // creeping queues interleaved car-yields-ped / ped-waits-car
                // into a 300 s junction crawl.
                // Hold only for a MOVING body (it will clear; a mutual hold
                // with the car's own ped-sense deadlocked whole junctions).
                // STANDING cars across walkways are removed at the source by
                // the don't-block-the-box rule below.
                a.speed = 0;
                motion = 0;
                a.state = Agent::State::Waiting;
                break;
            }
        }
    }

    // Hard stop for a pedestrian/player ahead: the smooth approachStop above eases
    // the car down but never to exactly zero, so on its own the car would still
    // creep forward and run the person over. Refuse to advance within kPedHardStop
    // of whatever it sees in its lane — the car holds until the path is clear.
    if (seenAhead < 1e9) {
        Real room = std::max(Real(0), seenAhead - kPedHardStop);
        if (motion > room) { motion = room; a.speed = 0; }
    }

    int legCount = static_cast<int>(a.route.links.size());
    while (motion > 0 && a.leg < legCount) {
        Real L = nav_->links[a.route.links[a.leg]].length;
        Real remain = L - a.distOnLeg;
        if (motion < remain) { a.distOnLeg += motion; motion = 0; }
        else {
            motion -= remain;
            // MISSED EXIT (§9.6 device rule): about to turn onto an exit ramp
            // but not in the slow lane — you miss it. Carry on with traffic
            // and re-plan from the freeway continuation.
            if (a.leg + 1 < legCount) {
                const engine::NavLink& cur = nav_->links[a.route.links[a.leg]];
                const engine::NavLink& nxt =
                    nav_->links[a.route.links[a.leg + 1]];
                if (cur.klass == engine::RoadClass::Freeway &&
                    nxt.klass == engine::RoadClass::Ramp &&
                    a.lane < cur.lanes - 1) {
                    int cont = -1;   // the freeway link straight ahead
                    for (int ol : nav_->outLinks[cur.to])
                        if (nav_->links[ol].klass ==
                                engine::RoadClass::Freeway &&
                            nav_->links[ol].to != cur.from)
                            cont = ol;
                    if (cont >= 0) {
                        const int goal = a.route.links.back();
                        engine::Route r2 = engine::findRoute(
                            *nav_, nav_->links[cont].to,
                            nav_->links[goal].to,
                            a.mode == Agent::Mode::Pedestrian);
                        if (r2.valid()) {
                            r2.links.insert(r2.links.begin(), cont);
                            a.route = r2;
                            a.leg = 0;
                            a.distOnLeg = 0;
                            legCount = static_cast<int>(a.route.links.size());
                        }
                    }
                }
            }
            ++a.leg;
            a.distOnLeg = 0;
            // Re-clamp the lane to the NEW leg's lane count: keeping lane 1 from
            // a two-lane arterial onto a one-lane local put the car on the kerb
            // and out of every same-link follower's gap key.
            if (a.leg < legCount) {
                const engine::NavLink& newL = nav_->links[a.route.links[a.leg]];
                int lanes = std::max(1, newL.lanes);
                if (a.lane >= lanes) a.lane = lanes - 1;
                // laneF must follow: it survived a narrow link untouched and
                // POPPED back two lanes on the next wide one (device: "not
                // changing to adjacent lanes, teleporting between lanes")
                if (a.laneF > Real(lanes - 1)) a.laneF = Real(lanes - 1);
                // Merging from a ramp: ENTER on the slow lane, gliding in
                // from the accel-lane side (device: "cars should start on
                // the lane they enter on").
                if (a.leg > 0 && newL.klass == engine::RoadClass::Freeway &&
                    nav_->links[a.route.links[a.leg - 1]].klass ==
                        engine::RoadClass::Ramp) {
                    a.lane = lanes - 1;
                    a.laneF = Real(lanes - 1);   // enter ON the slow lane
                }
            }
        }
    }
    // ARRIVE a few metres short of the exact endpoint (schedule mode): several
    // commuters can share a destination node, and with real collisions (the
    // fender-bender rule) cars converging on the same point crash-locked just
    // shy of it and never completed. Close enough is arrived; the park pose
    // takes over. Wander keeps exact arrival — its trips chain THROUGH the node.
    //
    // A BUS stops well short: clear of the junction box plus half its own
    // length, at the kerb of the street it came in on. Stopping ON the node
    // parked it in the middle of the junction for its whole dwell, and at a
    // hub the buses of three routes arrived on three different streets,
    // stood on the same point inside each other, and never untangled.
    const bool busHere = isBus(indexOf(a));
    if (busHere && car && a.leg < legCount) {
        // Route metres to the stop node; + a little on the stand-back so IDM
        // settles ON the point, not past it.
        Real rem = nav_->links[a.route.links[a.leg]].length - a.distOnLeg;
        for (int k = a.leg + 1; k < legCount && rem < 200.0; ++k)
            rem += nav_->links[a.route.links[k]].length;
        if (rem < busStandBack(a) + 0.6) {
            a.busStoodLeg = a.leg;
            a.leg = legCount;
        }
    } else if (!wander_ && car && a.leg == legCount - 1) {
        const int li = a.route.links[a.leg];
        Real L = nav_->links[li].length;
        Real shortBy = std::min(Real(3.0), L * 0.5);
        if (L - a.distOnLeg < shortBy) a.leg = legCount;
        // Driving to a reserved BAY: the trip ends AT the bay's station.
        if (a.targetBay >= 0 && a.targetBay < static_cast<int>(bays_.size()) &&
            bays_[static_cast<std::size_t>(a.targetBay)].link == li &&
            a.distOnLeg >= bays_[static_cast<std::size_t>(a.targetBay)].station - 0.3)
            a.leg = legCount;
    }

    // A DIRECTOR'S ERRAND ENDS WHERE IT WAS SENT (ADR-0091), part-way down the
    // last link rather than at its node — the difference between arriving at
    // the car and arriving twenty metres from it.
    if (a.stopAtDist >= 0 && a.leg == legCount - 1 && a.distOnLeg >= a.stopAtDist)
        a.leg = legCount;

    if (a.leg >= legCount) {
        arriveOrChain(a, a.speed);   // pass the still-rolling speed: a chain keeps it
    } else {
        refreshPose(a);
        steer(a, dt);
        if (car) labelDriverState(a, seenAhead, gap, legCount);
    }
}

// The arrival branch of advance(), table-driven (ADR-0064): the Arrived event
// asks the agent's GoalTable what comes next. A DRIVER whose next state is
// another trip (a GoTo state — wander's Roam self-loop) CHAINS straight into
// it; otherwise the agent rests — a driver parks on the verge, a walker rests
// at its sidewalk end — wearing the new state's activity label.
// `vArrive` is the speed still carried at the arrival tick — a chain keeps it.
void CitySim::arriveOrChain(Agent& a, Real vArrive) {
    a.moving = false;
    a.speed = 0;
    // A WALKER REACHING THE STOP THEY WERE HEADING FOR: stand and wait for the
    // bus rather than carrying on with the day. The goal state is left exactly
    // as it was, so once the bus sets them down the trip they were originally
    // making simply resumes from wherever that is.
    {
        const int self = indexOf(a);
        if (a.mode == Agent::Mode::Pedestrian) {
            const BusTrip* bt = buses_.tripOf(self);
            if (bt && !bt->aboard) {   // already stopped above
                // Waiting OUTSIDE, in a spot of their own: everyone bound for
                // this stop ended on the same sidewalk point, one inside the
                // next.
                a.indoors = false;
                if (!a.route.links.empty()) {
                    const int ll = a.route.links.back();
                    a.pos = freeStandingSpot(a, a.pos, nav_->direction(ll));
                    a.tickFromPos = a.pos;
                }
                return;
            }
        }
    }
    // A BUS AT A STOP: set down everyone whose stop this is, then pick up
    // everyone waiting here for this route, then move the index on. ALIGHT
    // FIRST -- a full bus that boards first would drive past the people at the
    // very stop where it just freed seats.
    {
        const int self = indexOf(a);
        if (isBus(self)) {
            const int r = busRoute_[static_cast<std::size_t>(self)];
            const int si = busStop_[static_cast<std::size_t>(self)];
            if (r >= 0 && r < buses_.routeCount()) {
                const int stops = static_cast<int>(buses_.route(r).stops.size());
                if (si >= 0 && si < stops) {
                    int people = 0;
                    // Riders step off where the bus STANDS (short of the stop
                    // node, see busStandBack): the nearer end of its link, if
                    // walkers can start there.
                    int setDown = buses_.route(r).stops[static_cast<std::size_t>(si)].node;
                    if (!a.far() && a.busStoodLeg >= 0 &&
                        a.busStoodLeg < static_cast<int>(a.route.links.size())) {
                        const engine::NavLink& SL =
                            nav_->links[static_cast<std::size_t>(a.route.links[static_cast<std::size_t>(a.busStoodLeg)])];
                        const int nearEnd =
                            (a.pos - nav_->nodes[static_cast<std::size_t>(SL.from)]).lengthSquared() <
                                    (a.pos - nav_->nodes[static_cast<std::size_t>(SL.to)]).lengthSquared()
                                ? SL.from : SL.to;
                        for (int ol : nav_->outLinks[static_cast<std::size_t>(nearEnd)])
                            if (nav_->links[static_cast<std::size_t>(ol)].walkable) { setDown = nearEnd; break; }
                    }
                    for (int p : buses_.alightingAt(r, si)) {
                        // Only riders on THIS bus. alightingAt lists everyone on
                        // the route bound for this stop, and a route runs six
                        // buses: whichever reached the stop first set down
                        // riders sitting on the others, hundreds of metres away,
                        // and they jumped to the stop (97 of 152 alightings).
                        if (rides_.driverOf(p) != self) continue;
                        alightRide(p, setDown);
                        buses_.forget(p);
                        ++people;
                    }
                    for (int p : buses_.waitingAt(r, si)) {
                        if (rides_.load(self) >= kBusSeats) break;
                        ++busBoardAttempts_;
                        if (boardRide(p, self)) { buses_.markAboard(p); ++people; }
                        else ++busBoardRefused_;
                    }
                    // THE DWELL. It used to chain straight into the next leg
                    // at speed, so a stop was a place the bus drove through:
                    // riders "boarded" in passing and a player could never get
                    // on at all (boarding needs a standing bus). Every stop is
                    // a stop -- a base wait, so someone walking up has time --
                    // plus a few seconds per person on or off.
                    a.busDwell = std::min(kBusDwellMax,
                                          kBusDwellBase + kBusDwellPerRider * people);
                    ++busStopsServed_;
                    busStop_[static_cast<std::size_t>(self)] = (si + 1) % stops;
                }
            }
        }
    }
    // A CAB AT ITS STOP. The passenger changes vehicle HERE, before the table
    // is asked what comes next, so the transition out of ToPickup already has
    // them aboard and the one out of ToDrop already has them on the pavement.
    // Which stop it is comes from the state's TARGET, not from a flag: the cab
    // is at a pickup if that is what it was driving to.
    {
        const int self = indexOf(a);
        const Fare* f = isTaxi(self) ? dispatch_.fareOf(self) : nullptr;
        const GoalTable& tt = tableFor(a);
        if (f && a.goal >= 0 && a.goal < tt.stateCount()) {
            const GoalTarget tgt = tt.state(a.goal).target;
            if (tgt == GoalTarget::Fare) {
                boardRide(f->passenger, self);
            } else if (tgt == GoalTarget::Drop) {
                alightRide(f->passenger,
                           a.route.links.empty()
                               ? -1
                               : nav_->links[static_cast<std::size_t>(a.route.links.back())].to);
                dispatch_.complete(self);   // free to take another hail
            }
        }
    }
    int lastLink = a.route.links.back();
    // Rest at the ARRIVAL link's elevation — zeroing it parked bridge-deck
    // arrivals at ground level, under their own road.
    a.elevation = nav_->links[lastLink].aboveB;
    // A DIRECTED AGENT ARRIVES AND WAITS (ADR-0091). Its plan belongs to the
    // director, so the goal table does not get to chain the next trip — it
    // stands where it was sent until told otherwise. restNode still moves, so
    // `direct off` hands the schedule an agent standing somewhere real.
    if (a.directed) {
        a.restNode = nav_->links[static_cast<std::size_t>(lastLink)].to;
        a.arrivedLink = lastLink;
        a.state = Agent::State::Resting;
        return;
    }
    const GoalTable& t = tableFor(a);
    int next = t.onEvent(a.goal, GoalEvent::Arrived);
    if (a.mode == Agent::Mode::Driver && next >= 0 &&
        t.state(next).action == GoalAction::GoTo) {
        // CHAIN straight into the next trip THIS tick (wander, the agent
        // lab) — no parking pose (which read on device as the car "jumping
        // backwards in time, then shooting forward") and no one-frame rest
        // INSIDE the junction box for cross traffic to overlap. The car
        // keeps its speed and heading through the node; steer() rotates it
        // onto the new route at the bounded yaw rate, like any other turn.
        a.restNode = nav_->links[lastLink].to;
        a.arrivedLink = lastLink;
        a.goal = next;
        a.goalHours = 0;
        Vec2 keepHeading = a.heading;
        // Where a bus actually stopped: short of the node, possibly a few links
        // back (see busStandBack). Its next trip starts FROM there, not from
        // the node, or it would jump the setback forward on pulling away.
        const int stoodLeg = a.busStoodLeg;
        a.busStoodLeg = -1;
        const bool busStopShort = isBus(indexOf(a)) && !a.far() && stoodLeg >= 0 &&
            stoodLeg < static_cast<int>(a.route.links.size());
        std::vector<int> stoodLinks;
        if (busStopShort)
            stoodLinks.assign(a.route.links.begin() + stoodLeg, a.route.links.end());
        const Real stoodAt = a.distOnLeg;
        const int keepLane = a.lane;
        const Real keepLaneF = a.laneF;
        if (startGoalTrip(a, a.restNode, /*fromRest=*/false)) {
            if (busStopShort && a.moving) {
                a.route.links.insert(a.route.links.begin(), stoodLinks.begin(), stoodLinks.end());
                a.leg = 0;
                a.distOnLeg = stoodAt;
                const int lanes = std::max(1, nav_->links[stoodLinks.front()].lanes);
                a.lane = std::min(keepLane, lanes - 1);
                a.laneF = std::min(keepLaneF, Real(lanes - 1));
                refreshPose(a);
            }
            // A bus standing at its stop leaves from rest when the dwell ends.
            a.speed = a.busDwell > 0 ? Real(0) : vArrive;
            a.heading = keepHeading;   // no snap: the yaw stays rate-limited
            return;
        }
        // No route anywhere (isolated node): pull onto the VERGE like a
        // schedule arrival — resting in-lane left a body no gap key, no box
        // rule, and no crash pass could see. The goal pass retries the trip.
        {
            Vec2 dir = nav_->direction(lastLink);
            Vec2 right(dir.y, -dir.x);
            Real hw = nav_->links[lastLink].width * 0.5;
            Real back = 4.0 + parkSetbackSlot(a.brain) * 1.3;
            back = std::min(back, nav_->links[lastLink].length * 0.5);
            a.pos = pushPoseClearOfLanes(
                nav_->nodes[nav_->links[lastLink].to] - dir * back +
                    right * (hw + 2.8),
                1.3);
            a.heading = dir;
        }
        a.route.links.clear();
        return;
    }
    if (a.mode == Agent::Mode::Driver) {
        // Park OFF the travel lanes (ADR-0062). Preferred: a marked curbside
        // BAY on the arrival link (R6b) — the stretch just behind the node,
        // the same range the old verge snap used. Fallback: the grass verge.
        const int myIdx = static_cast<int>(&a - agents_.data());
        int bay = -1;
        // The bay this trip was driven to (the route ended at it).
        if (a.targetBay >= 0 && a.targetBay < static_cast<int>(bays_.size()) &&
            bays_[static_cast<std::size_t>(a.targetBay)].link == lastLink)
            bay = a.targetBay;
        else if (a.targetBay >= 0 && a.targetBay < static_cast<int>(bays_.size()) &&
                 bays_[static_cast<std::size_t>(a.targetBay)].occupant == myIdx)
            bays_[static_cast<std::size_t>(a.targetBay)].occupant = -1;
        a.targetBay = -1;
        if (bay < 0 && lastLink < static_cast<int>(baysOnLink_.size())) {
            const Real L = nav_->links[lastLink].length;
            for (int bi : baysOnLink_[lastLink]) {
                if (bays_[bi].occupant != -1) continue;
                if (L - bays_[bi].station > 30.0) continue;   // near the node
                bay = bi;
                break;
            }
        }
        bool offStreet = false;
        if (bay >= 0) {
            bays_[bay].occupant = myIdx;
            a.parkedBay = bay;
            a.pos = bays_[bay].pos;
            a.heading = bays_[bay].heading;
        } else if (parksInBays(a) && !bays_.empty()) {
            // NO FREE SPACE NEAR: off-street (a garage, a driveway) rather
            // than onto the verge at the corner, where every such car used to
            // pile up. It is not drawn and is not a body until it leaves. A
            // city with no kerbside parking at all keeps the verge below.
            offStreet = true;
            a.pos = idlePose(nav_->links[lastLink].to, a.mode, a.brain);
            a.heading = nav_->direction(lastLink);
        } else {
            // Verge fallback: a few metres short of the node, per-agent
            // setback so arrivals at one destination don't stack. Pushed
            // clear of EVERY carriageway: a knot's tiny arrival link has no
            // bays, and its own verge can sit inside a neighbouring link's
            // lanes — the parked car then dammed the junction for minutes.
            Vec2 dir = nav_->direction(lastLink);
            Vec2 right(dir.y, -dir.x);
            Real hw = nav_->links[lastLink].width * 0.5;
            Real back = 4.0 + parkSetbackSlot(a.brain) * 1.3;   // 4..13 m
            back = std::min(back, nav_->links[lastLink].length * 0.5);
            a.pos = pushPoseClearOfLanes(
                nav_->nodes[nav_->links[lastLink].to] - dir * back +
                    right * (hw + 2.8),
                1.3);
            a.heading = dir;
        }
        // GET OUT. The leg that just ended was the DRIVING part of the day; the
        // rest of it happens on foot. The car is left exactly where it was
        // parked and stops following its owner around — from here it is an
        // object the world contains, which is what lets it be seen parked, be
        // missed when its owner comes back, and be stolen.
        if (a.car >= 0 && a.car < static_cast<int>(vehicles_.size())) {
            SimVehicle& v = vehicles_[a.car];
            v.pos = a.pos;          // the pose it keeps until someone drives it
            v.heading = a.heading;
            v.driver = -1;          // nobody is in it
            v.offStreet = offStreet;
            // Index it where it stands: from here it is a body in the world
            // that nothing can find through its owner.
            parkedGrid_.place(a.car, v.pos);
        }
        a.vehicle = -1;             // ...and the agent is no longer driving one
        a.mode = Agent::Mode::Pedestrian;
    } else {
        // A walker rests at the end of its sidewalk (continuous with the final
        // motion — not snapped across the node, which was the old visible jump).
        a.pos = nav_->sidewalkPoint(lastLink, 1.0);
    }
    // Living City: an arrival AT one of its own places rests at THAT DOOR — the
    // agent is semantically indoors. The corner pose could sit inside the
    // traffic corridor (and, sensed, froze every approaching car — device).
    // Applies to a driver too now that it finishes its trip on foot: it leaves
    // the car at the kerb and covers the last few metres to the door.
    {
        // The node the trip was FOR: a car parked in a bay a street away still
        // delivers its driver to the right door.
        const int node = a.tripGoal >= 0 ? a.tripGoal : nav_->links[lastLink].to;
        const bool atHome = a.homePlace != kNoPlace && node == a.home;
        const bool atWork =
            node == a.work &&
            (a.workPlace != kNoPlace ||
             (a.role == Agent::Role::Stroller && a.work != a.home));
        const bool atShop = a.shopPlace != kNoPlace && node == a.shop;
        // Where the agent is now: INSIDE at a door (not drawn), or outside
        // (drawn, standing somewhere of its own).
        bool inside = false;
        auto jitter = [&](Real lo, Real hi) {
            return lo + (hi - lo) * static_cast<Real>(tripRnd(a) % 1000u) / 999.0;
        };
        bool seated = false;
        // HOW LONG (the activity catalog): the activity's own minutes, else what its kind of site keeps a visitor
        auto stay = [&]() {
            double lo = 0, hi = 0;
            if (a.tripActivity >= 0 && a.tripActivity < static_cast<int>(catalog_.defs.size())) {
                const ActivityDef& d = catalog_.defs[static_cast<std::size_t>(a.tripActivity)];
                lo = d.minutesLo; hi = d.minutesHi;
            }
            if (hi <= 0) siteKindMinutes(siteKindOf(a), lo, hi);
            return jitter(lo / 60.0, hi / 60.0);
        };
        if (a.session >= 0 && a.session < static_cast<int>(sessions_.size()) && a.mode == Agent::Mode::Pedestrian &&
            areas_[static_cast<std::size_t>(sessions_[static_cast<std::size_t>(a.session)].area)].node == node) {
            // A GROUP'S PLAYER at its area's path: onto it, to its first point (the session runs it from here)
            const Session& se = sessions_[static_cast<std::size_t>(a.session)];
            a.restDwell = 1e9;
            a.seatPhase = 1;
            a.seatBack = a.pos;
            a.roamTarget = se.formation != Formation::Roam ? memberPlace(se, indexOf(a), nullptr) : zonePoint(se, a.sessionRole, tripRnd(a));
            seated = true;
        } else if (a.tripSeat >= 0 && a.tripSeat < static_cast<int>(seats_.size()) &&
            seats_[static_cast<std::size_t>(a.tripSeat)].node == node &&
            seats_[static_cast<std::size_t>(a.tripSeat)].occupant == indexOf(a) && a.mode == Agent::Mode::Pedestrian) {
            // AT THE SEAT'S PATH: off it to the seat, a sit of five to fifteen minutes, back (stepSeats) -- or onto the
            // track for a run of twenty to thirty-five
            a.restDwell = stay();
            a.seatPhase = 1;
            a.seatBack = a.pos;
            seated = true;
        } else if (a.tripSeat >= 0) {
            releaseSeat(a);
        }
        if (seated) {
        } else if (a.tripVenue >= 0 && a.tripVenue < static_cast<int>(venues_.size()) &&
            venues_[static_cast<std::size_t>(a.tripVenue)].node == node) {
            // An OUTING or LUNCH stop: in through the door, for as long as
            // that kind of place keeps people -- or, at a park, outside.
            const Venue& v = venues_[static_cast<std::size_t>(a.tripVenue)];
            a.restDwell = stay();   // a coffee, a meal, a session in the library, a while on the quad
            if (placeIsIndoors(v.type)) {
                a.pos = v.door;
                inside = true;
            }
        } else if (a.mode == Agent::Mode::Pedestrian && outingStroller(a) &&
                   node != a.home) {
            // A walk round the block: a pause to look about, outside.
            a.restDwell = a.tripActivity >= 0 ? stay() : jitter(0.01, 0.04);
        } else if (atHome) {
            a.pos = a.homeDoor;
            inside = true;
        } else if (atWork) {
            a.pos = a.workDoor;
            inside = a.role != Agent::Role::Stroller;   // the park is outside
        } else if (atShop) {
            a.pos = a.shopDoor;
            inside = true;
        }
        a.indoors = inside && a.mode == Agent::Mode::Pedestrian;
        a.outingTo = -1;   // arrived: the next outing chooses afresh
        a.atActivity = a.tripActivity;
        a.tripActivity = -1;
        // Outside: a spot of its own, not the sidewalk point everyone arriving
        // along this street ends on.
        if (!a.indoors && a.mode == Agent::Mode::Pedestrian && a.seatPhase == 0)
            a.pos = freeStandingSpot(a, a.pos, nav_->direction(lastLink));
        if (a.seatPhase == 1) a.seatBack = a.pos;
    }
    if (next >= 0) {
        a.goal = next;
        a.goalHours = 0;
        a.activity = t.state(next).activity;
    } else {
        // No Arrived row (a sparse custom table): keep the historical flip so
        // the day's label still reads sensibly.
        a.activity = (a.activity == Agent::Activity::Commuting)
                         ? Agent::Activity::AtWork
                         : Agent::Activity::AtHome;
    }
    a.restNode = nav_->links[lastLink].to;   // the next departure starts here
    a.arrivedLink = lastLink;                // ...and avoids U-turning back up this
    a.route.links.clear();
    a.tickFromPos = a.pos;          // parked / at the door: placed, not slid there
    a.tickFromHeading = a.heading;
}

// Driver FSM (ADR-0061): label what's governing the car this step, from what it
// sees, so the behaviour is legible (debug widgets read a.state). Precedence
// mirrors how advance() actually capped the speed: a ped/player in the cone
// (Yielding) dominates a same-lane leader (Following), which dominates a heading
// change at the coming node (Turning); otherwise the car runs free (Cruising).
// The Waiting-at-a-red case returned from advance() earlier.
void CitySim::labelDriverState(Agent& a, Real seenAhead, Real gap,
                               int legCount) const {
    bool bendAhead = false;
    if (a.leg + 1 < legCount) {
        Vec2 d0 = nav_->direction(a.route.links[a.leg]);
        Vec2 d1 = nav_->direction(a.route.links[a.leg + 1]);
        Real align = d0.x * d1.x + d0.y * d1.y;   // 1 = straight through
        Real distToEnd = nav_->links[a.route.links[a.leg]].length - a.distOnLeg;
        if (align < 0.98 && distToEnd < kJunctionApproach) bendAhead = true;
    }
    if (seenAhead < 1e9)          a.state = Agent::State::Yielding;
    else if (gap < kCarSlowZone)  a.state = Agent::State::Following;
    else if (bendAhead)           a.state = Agent::State::Turning;
    else                          a.state = Agent::State::Cruising;
}

void CitySim::computeGaps() {
    const Real INF = std::numeric_limits<Real>::infinity();
    // (read only for K agents: sized once, reset where they are -- not 2.4 MB of writes a tick at 100k)
    if (gaps_.size() != agents_.size()) {
        gaps_.assign(agents_.size(), INF);
        minGaps_.assign(agents_.size(), kCarMinGap);
        leaderSpeeds_.assign(agents_.size(), 0.0);
    } else {
        for (int k : kIdx_) {
            gaps_[static_cast<std::size_t>(k)] = INF;
            minGaps_[static_cast<std::size_t>(k)] = kCarMinGap;   // overwritten where a leader exists
            leaderSpeeds_[static_cast<std::size_t>(k)] = 0.0;     // valid only where gaps_ < INF
        }
    }
    auto laneKeyOf = [this](const Agent& a, int li) {
        // Clamp per link so a car keyed onto a narrower continuation matches the
        // followers/leaders actually driving that link's lanes.
        int laneKey = (a.mode == Agent::Mode::Driver)
                          ? std::min(a.lane, std::max(1, nav_->links[li].lanes) - 1)
                          : 1024;
        return static_cast<long long>(li) * 4096 + laneKey;
    };
    std::unordered_map<long long, std::vector<std::pair<Real, int>>> lanes;
    for (int i : kIdx_) {
        const Agent& a = agents_[i];
        if (a.far()) continue;   // far tier: not on the road
        if (!a.moving || a.leg >= static_cast<int>(a.route.links.size())) continue;
        const int li = a.route.links[a.leg];
        const long long key = laneKeyOf(a, li);
        lanes[key].push_back({a.distOnLeg, i});
        // MID-CHANGE a car is in BOTH lanes: the one it is leaving keeps it as a leader (its followers there
        // used to lose it the instant it decided, and drove into its tail), and it follows both lanes' leaders
        if (a.mode == Agent::Mode::Driver && std::fabs(Real(a.lane) - a.laneF) > 0.15) {
            // the lane it is leaving: laneF rounded back toward where it came from
            const int from = std::max(0, static_cast<int>(std::lround(a.laneF - (a.lane > a.laneF ? 0.35 : -0.35))));
            const long long old = static_cast<long long>(li) * 4096 + std::min(from, std::max(1, nav_->links[li].lanes) - 1);
            if (old != key) lanes[old].push_back({a.distOnLeg, i});
        }
    }
    // (link,lane) -> the car nearest the entry {distOnLeg, agentIndex}, so a
    // follower crossing a node can pick up the leader on its next link AND that
    // leader's length (for a length-aware gap).
    std::unordered_map<long long, std::pair<Real, int>> minEntry;
    for (auto& kv : lanes) {
        std::vector<std::pair<Real, int>>& v = kv.second;
        std::sort(v.begin(), v.end(), [](const std::pair<Real, int>& a,
                                          const std::pair<Real, int>& b) {
            return a.first != b.first ? a.first < b.first : a.second < b.second;
        });
        minEntry[kv.first] = { v.front().first, v.front().second };
        const bool walkers = (kv.first % 4096) == 1024;
        for (std::size_t k = 0; k + 1 < v.size(); ++k) {
            // WALKERS go side by side, not in file: one LEVEL with me (within a body's depth along the way) is not in
            // front of me -- the lean takes us past each other. Counted as a leader, a crowd that set off from one
            // door together (a class change) stood at the same distance, each held behind the next, for ever.
            std::size_t j = k + 1;
            if (walkers)
                while (j < v.size() && v[j].first - v[k].first < kPedSideBySide) ++j;
            if (j >= v.size()) continue;
            // the nearer leader wins: a changing car sits in two lanes' chains
            const Real g = std::max(Real(0), v[j].first - agents_[v[j].second].bodyLag - v[k].first);
            if (g >= gaps_[v[k].second]) continue;
            gaps_[v[k].second] = g;
            minGaps_[v[k].second] = pairMinGap(v[k].second, v[j].second);
            leaderSpeeds_[v[k].second] = agents_[v[j].second].speed;
        }
    }
    // Car-following ACROSS a node: the front car on a link (no leader ahead on its
    // own link) keeps its gap to the car just ahead on its NEXT link — the one it's
    // chasing through the intersection or around the bend. Without this, followers
    // overlap the leader as they cross a node (same-lane keys don't span it). We
    // chain across up to a couple of short links so a leader that has already moved
    // onto the link-after-next is still seen.
    for (int i : kIdx_) {
        const Agent& a = agents_[i];
        if (gaps_[i] != INF) continue;                 // has a same-link leader already
        if (a.far()) continue;        // far tier: no following
        if (!a.moving) continue;
        int legN = static_cast<int>(a.route.links.size());
        Real ahead = nav_->links[a.route.links[a.leg]].length - a.distOnLeg;  // to end of this link
        // Chain by DISTANCE, not by link count: metro's streets are ~4 m links,
        // so "two links" was ~8 m of sight -- a car at 15 m/s met a stopped
        // queue with a third of its braking distance (measured: the fast
        // rear-enders that seeded the junction pile-ups). 60 m covers IDM's
        // comfortable stop from the fastest street class.
        for (int step = 1; a.leg + step < legN && ahead < 60.0; ++step) {
            int nextLi = a.route.links[a.leg + step];
            auto it = minEntry.find(laneKeyOf(a, nextLi));
            if (it != minEntry.end()) {
                gaps_[i] = std::max(Real(0), ahead + it->second.first - agents_[it->second.second].bodyLag);
                minGaps_[i] = pairMinGap(i, it->second.second);
                leaderSpeeds_[i] = agents_[it->second.second].speed;
                break;
            }
            ahead += nav_->links[nextLi].length;       // no car on this link; look one further
        }
    }
}

// The car-vs-car VISION WEDGE (roads-v2 plan Â§3.1, sense 2 of the driver
// agent) â slice 3: the CORRIDOR sense. The lane-keyed gap logic only sees
// leaders on MY route's lane chain; a merging car's nose, a wreck on another
// link, or a body stalled inside the junction box is invisible to it â and
// the soak telemetry traced real contacts to exactly those. Each car senses
// bodies in its forward swept corridor and hands the nearest to IDM as a
// leader. Guard rails that keep the historic deadlock out (cars were
// deliberately blind to cars before this â braking for everything on the
// heading ray froze junctions):
//   - ROUTE-AWARE horizon: my corridor ends at my link's end node (+ a box
//     allowance) â past it my path turns; bodies there (cross-street
//     queues!) are not ahead of me whatever the ray says.
//   - ONCOMING cars in their own lane pass; only a truly head-on body binds.
//   - a STOPPED cross-oriented car beyond close range is a cross-street
//     queue â the signal owns that conflict, not mutual corridor braking.
void CitySim::computeCarWedge() {
    const Real INF = std::numeric_limits<Real>::infinity();
    if (carAheadGap_.size() != agents_.size()) {
        carAheadGap_.assign(agents_.size(), INF);
        carAheadSpeed_.assign(agents_.size(), 0.0);
    } else {
        for (int k : kIdx_) {
            carAheadGap_[static_cast<std::size_t>(k)] = INF;
            carAheadSpeed_[static_cast<std::size_t>(k)] = 0.0;
        }
    }
    constexpr Real kRange = 38.0;   // sense horizon (m)
    constexpr Real kLat = 2.15;     // corridor half-width: two bodies abreast
    for (int ai : active_) {
        Agent& a = agents_[ai];
        const std::size_t i = static_cast<std::size_t>(ai);
        if (a.mode != Agent::Mode::Driver || !a.moving) continue;
        const Real fx = a.heading.x, fy = a.heading.y;
        Real linkLeft = kRange;
        if (nav_ && a.leg < static_cast<int>(a.route.links.size())) {
            const int li = a.route.links[a.leg];
            linkLeft = nav_->links[li].length - a.distOnLeg + 10.0;
        }
        const Real horizon = std::min(kRange, linkLeft);
        // ONE grid fetch (P4.1) feeds both the corridor and the crossing
        // senses below — candidates ascending, exactly the order (and, after
        // the identical in-loop predicates, exactly the result) the full
        // agent scans produced.
        grid_.query(a.pos, kRange + 6.0, queryScratch_);
        for (int bi : queryScratch_) {
            const std::size_t j = static_cast<std::size_t>(bi);
            if (j == i) continue;
            const Agent& b = agents_[j];
            if (b.mode != Agent::Mode::Driver) continue;
            if (b.far()) continue;   // far tier: no body
            // A car standing in a marked BAY is out of the travel lanes by
            // construction (bays narrow the lanes). Its centre is ~2.1 m off
            // the kerb lane's heading line -- just inside the 2.15 m corridor
            // -- and every passing car braked behind it as a stopped leader
            // (metro: mean speed 5.6 -> 4.3 m/s once cars rested in bays).
            if (!b.moving && b.parkedBay >= 0) continue;
            // Different decks never conflict (viaduct vs the street below).
            if (std::fabs(b.elevation - a.elevation) > 2.5) continue;
            // A possessed leader is where its BODY is, up to a length behind its ghost (Agent::bodyLag).
            const Real dx = b.pos.x - b.heading.x * b.bodyLag - a.pos.x, dy = b.pos.y - b.heading.y * b.bodyLag - a.pos.y;
            if (dx * dx + dy * dy > kRange * kRange) continue;
            const Real along = fx * dx + fy * dy;
            const Real across = -fy * dx + fx * dy;   // + = b on my left
            if (along <= 0 || along >= horizon || std::fabs(across) >= kLat)
                continue;
            const Real hdot = b.heading.x * fx + b.heading.y * fy;
            // ONCOMING bodies are not the corridor's problem, in-lane or not:
            // a lane-bound car cannot dodge sideways, so braking converts a
            // (rare, U-turn-made) head-on into a nose-to-nose STANDOFF that
            // the creep/contact/escape machinery then has to grind through --
            // measured as escape time trebling on the dead-end wander map.
            // The contact rule arbitrates real head-on touches, as before.
            if (hdot < -0.6) continue;
            if (b.speed < 0.5 && std::fabs(hdot) < 0.5 && along > 9.0)
                continue;                                           // cross queue: signal's job
            const Real bodies = 0.5 * (vehicleLength(static_cast<int>(i)) +
                                       vehicleLength(static_cast<int>(j)));
            const Real gap = std::max(Real(0.05), along - bodies);
            if (gap < carAheadGap_[i]) {
                carAheadGap_[i] = gap;
                carAheadSpeed_[i] = std::max(Real(0), b.speed * hdot);
            }
        }
        // CROSSING sense (slice 4): a car on a predicted collision course —
        // closest approach inside the combined body envelope within my
        // stopping horizon. The conflicts the corridor can't see: two greens
        // turning across each other, unsignalled crossings, merge angles.
        // Anti-deadlock is ASYMMETRY: I yield only to a crosser approaching
        // from my RIGHT (right-hand traffic); it sees me on ITS left and
        // drives on — mutual yield cannot form. A crosser that a red signal
        // will stop before the box is exempt (a green driver assumes signal-
        // stopped traffic stays stopped); one already committed is not.
        for (int bi : queryScratch_) {
            const std::size_t j = static_cast<std::size_t>(bi);
            if (j == i) continue;
            const Agent& b = agents_[j];
            if (b.mode != Agent::Mode::Driver || !b.moving || b.speed < 0.3)
                continue;
            if (b.far()) continue;   // far tier: no body
            if (std::fabs(b.elevation - a.elevation) > 2.5) continue;
            const Real dx = b.pos.x - a.pos.x, dy = b.pos.y - a.pos.y;
            if (dx * dx + dy * dy > kRange * kRange) continue;
            const Real across = -fy * dx + fx * dy;
            if (across > 0) continue;             // my LEFT: their yield, not mine
            const Real hdot = b.heading.x * fx + b.heading.y * fy;
            if (hdot < -0.6 || hdot > 0.85) continue;   // oncoming/parallel: not a crossing
            if (nav_ && b.leg < static_cast<int>(b.route.links.size())) {
                const int bLi = b.route.links[b.leg];
                const bool redBound =
                    signals_.hasSignal(bLi) &&
                    signalFor(b, bLi) != SignalState::Green &&
                    b.distOnLeg < nav_->links[bLi].length - 3.0;
                if (redBound) continue;           // the signal will stop them
            }
            const Real rvx = b.heading.x * b.speed - fx * a.speed;
            const Real rvy = b.heading.y * b.speed - fy * a.speed;
            const Real rv2 = rvx * rvx + rvy * rvy;
            if (rv2 < 1e-6) continue;
            const Real tStar = -(dx * rvx + dy * rvy) / rv2;
            // Look ahead at least my own stopping time — a fast car needs the
            // conflict called early enough to brake for it.
            const Real myHorizon = std::max(Real(2.6), a.speed / kCarDecel + Real(1.0));
            if (tStar <= 0 || tStar > myHorizon) continue;
            const Real mx = dx + rvx * tStar, my = dy + rvy * tStar;
            if (mx * mx + my * my > (2.0 * kLat) * (2.0 * kLat)) continue;
            const Real bodies = 0.5 * (vehicleLength(static_cast<int>(i)) +
                                       vehicleLength(static_cast<int>(j)));
            // Brake as if a stopped body sat at my share of the closing run.
            const Real gap = std::max(Real(0.05), a.speed * tStar - bodies);
            if (gap < carAheadGap_[i]) {
                carAheadGap_[i] = gap;
                carAheadSpeed_[i] = 0.0;
            }
        }
    }
}

void CitySim::step(Real dt, Real hoursPerSecond) {
    departuresThisStep_ = 0;
    // Cadence gate. Traffic is not physics: agents follow lanes, so advancing
    // by a bigger dt costs precision, not correctness. Bank the time and tick
    // when the period is due; everything downstream (including the far tier's
    // own schedule) then measures in SIM SECONDS rather than in calls.
    tickAccum_ += dt;
    sinceTick_ += dt;
    if (tickPeriodSeconds_ > 0.0 && tickAccum_ + 1e-9 < tickPeriodSeconds_) return;
    const Real simDt = tickAccum_;
    tickAccum_ = 0.0;
    sinceTick_ = 0.0;
    // The interpolation origin for this tick: where everyone was before it.
    for (const std::vector<int>* tierList : {&kIdx_, &vIdx_})
        for (int ti : *tierList) {
            Agent& a = agents_[static_cast<std::size_t>(ti)];
            a.tickFromPos = a.pos;
            a.tickFromHeading = a.heading;
            a.tickFromPullS = a.pullS;
        }
    stepTick(simDt, hoursPerSecond);
    // RT_TRACE_AGENT=<id>: that agent's plan, every tick (with the speed watchdog: what a flickering one is doing)
    static const int traceId = std::getenv("RT_TRACE_AGENT") ? std::atoi(std::getenv("RT_TRACE_AGENT")) : -1;
    if (traceId >= 0 && traceId < static_cast<int>(agents_.size())) {
        const Agent& t = agents_[static_cast<std::size_t>(traceId)];
        std::fprintf(stderr, "[trace %d] dt %.3f from %.2f %.2f -> %.2f %.2f (%.2f m) heading %.2f %.2f speed %.2f lean %.2f -> %.2f leg %d dist %.2f state %d\n",
                     traceId, simDt, t.tickFromPos.x, t.tickFromPos.y, t.pos.x, t.pos.y, (t.pos - t.tickFromPos).length(),
                     t.heading.x, t.heading.y, t.speed, t.lateralOffset, t.leanTarget, t.leg, t.distOnLeg, static_cast<int>(t.state));
    }
}

// ---- SEATS (the furniture library, M5) ------------------------------------------------------------------------
void CitySim::setSpots(std::vector<ActivitySpot> seats) {
    seats_.clear();
    if (!nav_) return;
    // the nearest node a walker can leave by, within 60 m: where a sit starts and ends
    const int n = nav_->nodeCount();
    for (SeatSpot& s : seats) {
        // a seat is reached from the path within 60 m; a place on a track from as far as 150 (a field is wide, and a
        // runner crosses the grass to it)
        const Real reach = s.kind == SpotKind::Jog ? 150.0 : 60.0;
        Real best = reach * reach;
        s.node = -1;
        for (int i = 0; i < n; ++i) {
            const Vec2 d = nav_->nodes[static_cast<std::size_t>(i)] - s.pos;
            const Real d2 = d.x * d.x + d.y * d.y;
            if (d2 >= best) continue;
            bool walk = false;
            for (int ol : nav_->outLinks[static_cast<std::size_t>(i)]) walk = walk || nav_->links[static_cast<std::size_t>(ol)].walkable;
            if (!walk) continue;
            best = d2;
            s.node = i;
        }
        s.occupant = -1;
        if (s.node >= 0) seats_.push_back(s);
    }
    tagSpots();
}

void CitySim::tagSpots() {
    // IN A PARK: near a park's site (its door is its middle when it has no gate)
    auto nearPark = [&](Vec2 p, Real reach) {
        for (const Venue& v : venues_)
            if (v.type == PlaceType::Park && (v.door - p).lengthSquared() < reach * reach) return true;
        return false;
    };
    for (ActivityArea& ar : areas_) {
        ar.tags &= ~(spot_tag::kCampus | spot_tag::kSports | spot_tag::kPark);
        if (nearPark(ar.center, 60.0)) ar.tags |= spot_tag::kPark;
        for (const Venue& v : venues_) {
            if (v.campus != 4 && v.campus != 5) continue;
            if ((v.door - ar.center).lengthSquared() >= 200.0 * 200.0) continue;
            ar.tags |= spot_tag::kCampus;
            if (v.campus == 5) ar.tags |= spot_tag::kSports;
        }
    }
    for (ActivitySpot& s : seats_) {
        s.tags &= ~(spot_tag::kCampus | spot_tag::kSports | spot_tag::kPark);
        if (s.kind == SpotKind::Sit && nearPark(s.pos, 60.0)) s.tags |= spot_tag::kPark;
        for (const Venue& v : venues_) {
            if (v.campus != 4 && v.campus != 5) continue;
            // (a track's far side is a field's width from its door)
            const Real reach = s.kind == SpotKind::Jog ? 200.0 : 120.0;
            if ((v.door - s.pos).lengthSquared() >= reach * reach) continue;
            s.tags |= spot_tag::kCampus;
            if (v.campus == 5) s.tags |= spot_tag::kSports;
        }
    }
}

int CitySim::pickSpot(Agent& a, Vec2 here, const ActivityQuery& q) {
    std::vector<std::pair<Real, int>> c;
    for (int i = 0; i < static_cast<int>(seats_.size()); ++i) {
        const ActivitySpot& s = seats_[static_cast<std::size_t>(i)];
        if (s.occupant >= 0 || !(q.kinds & spotKindBit(s.kind)) || (s.tags & q.tags) != q.tags) continue;
        const Real d2 = (s.pos - here).lengthSquared();
        if (d2 < q.minDist * q.minDist || d2 > q.maxDist * q.maxDist) continue;
        c.push_back({d2, i});
    }
    if (c.empty()) return -1;
    const std::size_t k = std::min<std::size_t>(static_cast<std::size_t>(std::max(1, q.nearest)), c.size());
    std::partial_sort(c.begin(), c.begin() + static_cast<std::ptrdiff_t>(k), c.end());
    const int pick = c[rnd() % static_cast<uint32_t>(k)].second;
    seats_[static_cast<std::size_t>(pick)].occupant = indexOf(a);
    return pick;
}

int CitySim::pickActivity(Agent& a, int origin, const GoalState& st) {
    a.tripVenue = -1;
    releaseSeat(a);
    if (!nav_ || origin < 0 || origin >= nav_->nodeCount()) return -1;
    ActivityQuery q;
    q.kinds = st.spotKinds ? st.spotKinds : spotKindBit(SpotKind::Sit);
    q.tags = st.spotTags;
    const int s = pickSpot(a, nav_->nodes[static_cast<std::size_t>(origin)], q);
    if (s < 0) return -1;
    a.tripSeat = s;
    return seats_[static_cast<std::size_t>(s)].node;
}

const CitySim::ActivitySpot* CitySim::usingSpot(int i) const {
    if (i < 0 || i >= static_cast<int>(agents_.size())) return nullptr;
    const Agent& a = agents_[static_cast<std::size_t>(i)];
    if (a.seatPhase != 2 || a.tripSeat < 0 || a.tripSeat >= static_cast<int>(seats_.size())) return nullptr;
    return &seats_[static_cast<std::size_t>(a.tripSeat)];
}

std::string CitySim::describeAgent(int i) const {
    if (i < 0 || i >= static_cast<int>(agents_.size())) return "no such agent";
    const Agent& a = agents_[static_cast<std::size_t>(i)];
    static const char* kStates[] = {"resting", "walking", "avoiding", "waiting", "cruising", "following", "yielding", "turning"};
    const int st = static_cast<int>(a.state);
    auto actName = [&](int d) { return d >= 0 && d < static_cast<int>(catalog_.defs.size()) ? catalog_.defs[static_cast<std::size_t>(d)].name.c_str() : "-"; };
    int link = -1;
    bool foot = false;
    if (a.leg >= 0 && a.leg < static_cast<int>(a.route.links.size())) {
        link = a.route.links[static_cast<std::size_t>(a.leg)];
        if (nav_ && link >= 0 && link < nav_->linkCount()) foot = nav_->links[static_cast<std::size_t>(link)].footpath;
    }
    const char* sess = "-";
    if (a.session >= 0 && a.session < static_cast<int>(sessions_.size())) {
        static const char* kSes[] = {"gathering", "running", "ending", "dead"};
        sess = kSes[static_cast<int>(sessions_[static_cast<std::size_t>(a.session)].state)];
    }
    char b[640];
    std::snprintf(b, sizeof b,
                  "agent %d %s tier %c pos %.1f %.1f speed %.2f heading %.2f %.2f moving %d state %s indoors %d visible %d | trip venue %d "
                  "seat %d phase %d activity %s (at %s) session %d %s role %d | route leg %d/%zu link %d%s dist %.1f | rest %.3f h "
                  "goal %.3f h | tethered %d lag %.2f player %d | lean %.2f -> %.2f",
                  i, a.mode == Agent::Mode::Pedestrian ? "walker" : "driver", "DVK"[static_cast<int>(a.tier)], a.pos.x, a.pos.y,
                  a.speed, a.heading.x, a.heading.y, a.moving ? 1 : 0, st >= 0 && st < 8 ? kStates[st] : "?", a.indoors ? 1 : 0,
                  pedVisible(i) ? 1 : 0, a.tripVenue, a.tripSeat, a.seatPhase, actName(a.tripActivity), actName(a.atActivity),
                  a.session, sess, a.sessionRole, a.leg, a.route.links.size(), link, foot ? " (footpath)" : "", a.distOnLeg,
                  a.restDwell, a.goalHours, a.tethered ? 1 : 0, a.bodyLag, a.playerControlled ? 1 : 0, a.lateralOffset, a.leanTarget);
    return b;
}

bool CitySim::farDrawPose(int i, Vec2& pos, Vec2& heading) const {
    if (i < 0 || i >= static_cast<int>(agents_.size()) || !nav_) return false;
    const Agent& a = agents_[static_cast<std::size_t>(i)];
    if (a.tier != Agent::Tier::V || a.released || a.playerControlled) return false;
    if (a.mode == Agent::Mode::Pedestrian && (riding(i) || (!a.moving && a.indoors))) return false;
    if (a.mode == Agent::Mode::Driver && !a.moving) return false;   // (a parked car is the parked-car pass's)
    pos = a.pos;
    heading = a.heading;
    if (!a.moving || a.speed <= 0 || a.vHold > 0 || a.leg < 0 || a.leg >= static_cast<int>(a.route.links.size())) return true;
    const Real dt = std::clamp(simSeconds_ - a.vLastTick, Real(0), vRefreshSeconds * 1.5);
    const engine::NavLink& L = nav_->links[static_cast<std::size_t>(a.route.links[static_cast<std::size_t>(a.leg)])];
    const Real step = std::min(a.speed * dt, std::max(Real(0), L.length - a.distOnLeg));
    pos = a.pos + a.heading * step;
    return true;
}

bool CitySim::restPose(int i, RestPose& out) const {
    if (const SeatSpot* st = seatedOn(i)) {
        out.kind = RestPose::Kind::Seat;
        out.pos = st->pos;
        out.face = st->face;
        out.hip = st->hip;
        return true;
    }
    if (i < 0 || i >= static_cast<int>(agents_.size())) return false;
    const Agent& a = agents_[static_cast<std::size_t>(i)];
    if (a.seatPhase != 2 || a.speed > 0 || a.session < 0 || a.session >= static_cast<int>(sessions_.size())) return false;
    const Session& se = sessions_[static_cast<std::size_t>(a.session)];
    if (se.formation == Formation::Roam || se.def < 0) return false;
    const ActivityDef& d = catalog_.defs[static_cast<std::size_t>(se.def)];
    if (a.sessionRole < 0 || a.sessionRole >= static_cast<int>(d.roles.size())) return false;
    const Pose pose = d.roles[static_cast<std::size_t>(a.sessionRole)].pose;
    if (pose == Pose::Stand) return false;
    out.kind = pose == Pose::Lie ? RestPose::Kind::Lie : RestPose::Kind::SitGround;
    out.pos = a.pos;
    out.face = a.heading;
    out.hip = 0;
    return true;
}

const CitySim::SeatSpot* CitySim::seatedOn(int i) const {
    const ActivitySpot* s = usingSpot(i);
    return s && (s->kind == SpotKind::Sit || s->kind == SpotKind::Lie) ? s : nullptr;
}

void CitySim::setLoops(std::vector<std::vector<Vec2>> loops) {
    loops_.clear();
    for (std::vector<Vec2>& pts : loops) {
        if (pts.size() < 3) continue;
        Loop L;
        L.pts = std::move(pts);
        L.cum.push_back(0);
        for (std::size_t i = 0; i < L.pts.size(); ++i) {
            L.length += (L.pts[(i + 1) % L.pts.size()] - L.pts[i]).length();
            L.cum.push_back(L.length);
        }
        loops_.push_back(std::move(L));
    }
}

void CitySim::setAreas(std::vector<ActivityArea> areas) {
    areas_.clear();
    sessions_.clear();
    if (!nav_) return;
    const int n = nav_->streetNodeCount();
    for (ActivityArea& ar : areas) {
        Real best = 150.0 * 150.0;   // a field is wide: its players cross the grass to it
        ar.node = -1;
        for (int i = 0; i < n; ++i) {
            const Real d2 = (nav_->nodes[static_cast<std::size_t>(i)] - ar.center).lengthSquared();
            if (d2 >= best) continue;
            bool walk = false;
            for (int ol : nav_->outLinks[static_cast<std::size_t>(i)]) walk = walk || nav_->links[static_cast<std::size_t>(ol)].walkable;
            if (!walk) continue;
            best = d2;
            ar.node = i;
        }
        if (ar.node >= 0) areas_.push_back(ar);
    }
    tagSpots();
}

int CitySim::Session::roleCount(int role) const {
    int n = 0;
    for (int r : roles) n += r == role ? 1 : 0;
    return n;
}

Vec2 CitySim::zonePoint(const Session& s, int role, uint32_t bits) const {
    const ActivityArea& ar = areas_[static_cast<std::size_t>(s.area)];
    const ActivityDef& d = catalog_.defs[static_cast<std::size_t>(s.def)];
    int zone = role >= 0 && role < static_cast<int>(d.roles.size()) ? d.roles[static_cast<std::size_t>(role)].zone : -1;
    if (zone >= 0 && s.swapped) zone = 1 - zone;
    const Real inL = std::max(Real(0.5), ar.halfL - 2.0), inW = std::max(Real(0.5), ar.halfW - 1.5);
    const Real u0 = zone == 1 ? 1.0 : -inL, u1 = zone == 0 ? -1.0 : inL;
    const Real fu = static_cast<Real>(bits & 0xFFFF) / 65535.0, fv = static_cast<Real>((bits >> 16) & 0xFFFF) / 65535.0;
    const Vec2 side(ar.axis.y, -ar.axis.x);
    return ar.center + ar.axis * (u0 + (u1 - u0) * fu) + side * (-inW + 2 * inW * fv);
}

// Join a session of activity `di` on the nearest suitable area: one gathering or running with a free role, else a
// fresh one on an area with nothing on it. Returns the area, -1 for none.
int CitySim::joinSession(Agent& a, int di, Vec2 here, Real distLo, Real distHi, int nearest) {
    const ActivityDef& d = catalog_.defs[static_cast<std::size_t>(di)];
    std::vector<std::pair<Real, int>> cand;
    for (int i = 0; i < static_cast<int>(areas_.size()); ++i) {
        const ActivityArea& ar = areas_[static_cast<std::size_t>(i)];
        bool kind = false;
        for (const std::string& k : d.sites) kind = kind || k == ar.kind;
        if (!kind || (ar.tags & d.tags) != d.tags) continue;
        const Real d2 = (ar.center - here).lengthSquared();
        if (d2 < distLo * distLo || d2 > distHi * distHi) continue;
        cand.push_back({d2, i});
    }
    std::sort(cand.begin(), cand.end());
    if (nearest > 0 && static_cast<int>(cand.size()) > nearest) cand.resize(static_cast<std::size_t>(nearest));
    auto freeRole = [&](const Session& se) {
        int best = -1;
        Real bestFill = 1e9;
        for (int r = 0; r < static_cast<int>(d.roles.size()); ++r) {
            const int n = se.roleCount(r), cap = d.roles[static_cast<std::size_t>(r)].n;
            if (n >= cap) continue;
            const Real fill = static_cast<Real>(n) / std::max(1, cap);
            if (fill < bestFill) { bestFill = fill; best = r; }
        }
        return best;
    };
    // A SETTLED GROUP (a ring, a pair, sunbathers) shares its lawn: a chat anyone may join anywhere in reach first,
    // then a fresh one wherever a lawn has room for it
    if (d.formation != Formation::Roam) return joinSettled(a, di, cand, freeRole);
    for (const auto& c : cand) {
        // a session already here, of this activity, with room
        int si = -1, role = -1;
        bool busy = false;
        for (int k = 0; k < static_cast<int>(sessions_.size()); ++k) {
            Session& se = sessions_[static_cast<std::size_t>(k)];
            if (se.area != c.second || se.state == Session::State::Dead) continue;
            busy = true;
            if (se.def != di || (se.state != Session::State::Gathering && se.state != Session::State::Running)) continue;
            role = freeRole(se);
            if (role >= 0) { si = k; break; }
        }
        if (si < 0 && !busy) {   // nothing on this area: start one
            Session se;
            se.def = di;
            se.area = c.second;
            se.gatherUntil = clockTotalHours_ + d.gatherMinutes / 60.0;
            se.center = areas_[static_cast<std::size_t>(c.second)].center;
            se.axis = areas_[static_cast<std::size_t>(c.second)].axis;
            for (int k = 0; k <= static_cast<int>(sessions_.size()); ++k)
                if (k == static_cast<int>(sessions_.size()) || sessions_[static_cast<std::size_t>(k)].state == Session::State::Dead) {
                    if (k == static_cast<int>(sessions_.size())) sessions_.push_back(se); else sessions_[static_cast<std::size_t>(k)] = se;
                    si = k;
                    break;
                }
            role = freeRole(sessions_[static_cast<std::size_t>(si)]);
        }
        if (si < 0 || role < 0) continue;
        Session& se = sessions_[static_cast<std::size_t>(si)];
        se.members.push_back(indexOf(a));
        se.roles.push_back(role);
        // a group still forming waits a while longer for each new player (they are walking over)
        if (se.state == Session::State::Gathering)
            se.gatherUntil = std::max(se.gatherUntil, clockTotalHours_ + d.gatherMinutes / 60.0 * 0.5);
        a.session = si;
        a.sessionRole = role;
        return c.second;
    }
    return -1;
}

// a small integer hash (lowbias32): places and beats that must be the same whoever asks
static uint32_t hash32(uint32_t x) {
    x ^= x >> 16; x *= 0x7feb352du; x ^= x >> 15; x *= 0x846ca68bu; x ^= x >> 16;
    return x;
}

// Is `p` at least `r` from every footpath's line (a walk's half width counted in)?
bool CitySim::clearOfWalks(Vec2 p, Real r) const {
    if (!nav_) return true;
    for (const engine::NavLink& L : nav_->links) {
        if (!L.footpath) continue;
        const Vec2 ab = L.footB - L.footA;
        const Real l2 = ab.lengthSquared();
        const Real t = l2 > 1e-9 ? std::clamp(dot(p - L.footA, ab) / l2, Real(0), Real(1)) : 0;
        if ((L.footA + ab * t - p).lengthSquared() < (r + 1.0) * (r + 1.0)) return false;
    }
    return true;
}

int CitySim::joinSettled(Agent& a, int di, const std::vector<std::pair<Real, int>>& cand,
                         const std::function<int(const Session&)>& freeRole) {
    const ActivityDef& d = catalog_.defs[static_cast<std::size_t>(di)];
    auto enlist = [&](int si, int role) {
        Session& se = sessions_[static_cast<std::size_t>(si)];
        se.members.push_back(indexOf(a));
        se.roles.push_back(role);
        if (se.state == Session::State::Gathering)
            se.gatherUntil = std::max(se.gatherUntil, clockTotalHours_ + d.gatherMinutes / 60.0 * 0.5);
        a.session = si;
        a.sessionRole = role;
        return se.area;
    };
    // join one going (gathering or under way) with a place free, on the nearest area that has one
    for (const auto& c : cand)
        for (int k = 0; k < static_cast<int>(sessions_.size()); ++k) {
            const Session& se = sessions_[static_cast<std::size_t>(k)];
            if (se.area != c.second || se.def != di) continue;
            if (se.state != Session::State::Gathering && se.state != Session::State::Running) continue;
            const int role = freeRole(se);
            if (role >= 0) return enlist(k, role);
        }
    // start one: a spot on the area clear of the groups already on it (and of a game: a lawn with a kickabout on it is
    // the kickabout's), the farthest of a few tries from them
    const Real need = std::max(Real(2.0), static_cast<Real>(d.radius)) + 3.0;
    for (const auto& c : cand) {
        const ActivityArea& ar = areas_[static_cast<std::size_t>(c.second)];
        bool game = false;
        std::vector<Vec2> taken;
        for (const Session& se : sessions_) {
            if (se.area != c.second || se.state == Session::State::Dead) continue;
            if (se.formation == Formation::Roam) game = true;
            taken.push_back(se.center);
        }
        if (game) continue;
        const Vec2 side(ar.axis.y, -ar.axis.x);
        // a pair needs its length along the area, a ring its width
        const Real reachL = static_cast<Real>(d.formation == Formation::Pair ? d.radius * 0.5 + 1.0 : 2.0);
        const Real inL = ar.halfL - reachL, inW = ar.halfW - 2.0;
        if (inL <= 0 || inW <= 0) continue;
        Vec2 best;
        Real bestD = -1;
        uint32_t h = hash32(static_cast<uint32_t>(indexOf(a)) * 2654435761u ^ static_cast<uint32_t>(clockTotalHours_ * 3600.0));
        for (int t = 0; t < 16; ++t) {
            h = hash32(h + 0x9E37u);
            const Real fu = static_cast<Real>(h & 0xFFFF) / 65535.0 * 2 - 1, fv = static_cast<Real>(h >> 16) / 65535.0 * 2 - 1;
            const Vec2 p = ar.center + ar.axis * (inL * fu) + side * (inW * fv);
            // not on a walk (a lawn's box takes in the walks across it): a ring's edge, or each end of a pair
            if (d.formation == Formation::Pair) {
                if (!clearOfWalks(p - ar.axis * (d.radius * 0.5), 1.5) || !clearOfWalks(p + ar.axis * (d.radius * 0.5), 1.5)) continue;
            } else if (!clearOfWalks(p, std::max(Real(1.0), static_cast<Real>(d.radius)) + 1.2)) continue;
            Real dmin = 1e9;
            for (const Vec2& q : taken) dmin = std::min(dmin, (q - p).length());
            if (dmin > bestD) { bestD = dmin; best = p; }
        }
        if (bestD < 2 * need) continue;   // this lawn is full
        Session se;
        se.def = di;
        se.area = c.second;
        se.formation = d.formation;
        se.center = best;
        se.axis = ar.axis;
        se.gatherUntil = clockTotalHours_ + d.gatherMinutes / 60.0;
        int si = -1;
        for (int k = 0; k <= static_cast<int>(sessions_.size()); ++k)
            if (k == static_cast<int>(sessions_.size()) || sessions_[static_cast<std::size_t>(k)].state == Session::State::Dead) {
                if (k == static_cast<int>(sessions_.size())) sessions_.push_back(se); else sessions_[static_cast<std::size_t>(k)] = se;
                si = k;
                break;
            }
        const int role = freeRole(sessions_[static_cast<std::size_t>(si)]);
        if (role < 0) { sessions_[static_cast<std::size_t>(si)].state = Session::State::Dead; continue; }
        return enlist(si, role);
    }
    return -1;
}

// A SETTLED GROUP'S PLACES: a ring round its middle facing in (a body's shoulders apart, never tighter than the
// definition's radius); a pair `radius` apart along the session's axis, facing, each stepping a little to and fro
// every few seconds (a catch); sunbathers side by side, feet the same way.
Vec2 CitySim::memberPlace(const Session& se, int agentIndex, Vec2* face) const {
    const ActivityDef& d = catalog_.defs[static_cast<std::size_t>(se.def)];
    int k = 0;
    const int n = std::max(1, static_cast<int>(se.members.size()));
    for (int i = 0; i < static_cast<int>(se.members.size()); ++i)
        if (se.members[static_cast<std::size_t>(i)] == agentIndex) k = i;
    const Vec2 side(se.axis.y, -se.axis.x);
    Vec2 p = se.center, f = se.axis;
    switch (se.formation) {
        case Formation::Circle: {
            const Real r = std::max(static_cast<Real>(d.radius), Real(0.75) * n / Real(6.2831853));
            const Real ang = 6.2831853 * k / n + static_cast<Real>(se.def) * 0.7;
            const Vec2 out(std::cos(ang), std::sin(ang));
            p = se.center + out * (n == 1 ? 0.0 : r);
            f = n == 1 ? se.axis : out * -1.0;
            break;
        }
        case Formation::Pair: {
            const Real s = k == 0 ? -1.0 : 1.0;
            // a step this way or that every four seconds or so, as the throws come
            const uint32_t beat = static_cast<uint32_t>(clockTotalHours_ * 3600.0 / 4.0);
            const uint32_t h = hash32(static_cast<uint32_t>(agentIndex) * 2246822519u + beat);
            const Real lat = (static_cast<Real>(h & 0xFF) / 255.0 * 2 - 1) * 1.2;
            const Real lon = (static_cast<Real>((h >> 8) & 0xFF) / 255.0 * 2 - 1) * 0.8;
            p = se.center + se.axis * (s * (d.radius * 0.5 + lon)) + side * lat;
            f = se.axis * -s;
            break;
        }
        case Formation::Spread: {
            const Real gap = d.radius > 0 ? d.radius : 1.1;
            p = se.center + side * ((k - (n - 1) * 0.5) * gap);
            f = se.axis;
            break;
        }
        case Formation::Roam: break;
    }
    if (face) *face = f;
    return p;
}

void CitySim::leaveSession(Agent& a) {
    if (a.session < 0 || a.session >= static_cast<int>(sessions_.size())) { a.session = -1; return; }
    Session& se = sessions_[static_cast<std::size_t>(a.session)];
    const int me = indexOf(a);
    for (std::size_t k = 0; k < se.members.size(); ++k)
        if (se.members[k] == me) {
            se.members.erase(se.members.begin() + static_cast<std::ptrdiff_t>(k));
            se.roles.erase(se.roles.begin() + static_cast<std::ptrdiff_t>(k));
            break;
        }
    if (se.members.empty()) se.state = Session::State::Dead;
    a.session = -1;
    a.sessionRole = -1;
}

// THE SESSIONS' CLOCK: a gathering session starts once its players are on the area (or is given up when it has not
// filled in time); a running one swaps ends at its half time and ends at its length; an ending one sends its players
// off the area (stepSeats walks them back, and they leave it there).
void CitySim::stepSessions() {
    for (Session& se : sessions_) {
        if (se.state == Session::State::Dead) continue;
        const ActivityDef& d = catalog_.defs[static_cast<std::size_t>(se.def)];
        int present = 0;
        for (int m : se.members) present += agents_[static_cast<std::size_t>(m)].seatPhase == 2 ? 1 : 0;
        const int need = d.minPlayers > 0 ? d.minPlayers : 1;
        if (se.state == Session::State::Gathering) {
            if (present >= need) {
                se.state = Session::State::Running;
                se.startedAt = clockTotalHours_;
                double lo = d.minutesLo, hi = d.minutesHi;
                if (hi <= 0) siteKindMinutes(d.sites.empty() ? "pitch" : d.sites.front(), lo, hi);
                const Real f = static_cast<Real>(rnd() % 1000u) / 999.0;
                se.endAt = clockTotalHours_ + (lo + (hi - lo) * f) / 60.0;
            } else if (clockTotalHours_ >= se.gatherUntil) {
                se.state = Session::State::Ending;   // never filled: give it up
            }
        } else if (se.state == Session::State::Running) {
            if (d.swapAt > 0 && !se.swapped && clockTotalHours_ >= se.startedAt + (se.endAt - se.startedAt) * d.swapAt) {
                se.swapped = true;
                for (int m : se.members) {   // to the other end
                    Agent& p = agents_[static_cast<std::size_t>(m)];
                    p.roamTarget = zonePoint(se, p.sessionRole, tripRnd(p));
                }
            }
            if (clockTotalHours_ >= se.endAt) se.state = Session::State::Ending;
        }
        if (se.state == Session::State::Ending) {
            std::vector<int> onTheWay;   // still walking to it: let go (they arrive to nothing, and stand about)
            for (int m : se.members) {
                Agent& p = agents_[static_cast<std::size_t>(m)];
                if (p.seatPhase == 1 || p.seatPhase == 2) p.seatPhase = 3;
                else if (p.seatPhase == 0) onTheWay.push_back(m);
            }
            for (int m : onTheWay) leaveSession(agents_[static_cast<std::size_t>(m)]);
        }
    }
}

Real CitySim::loopLength(int loop) const {
    return loop >= 0 && loop < static_cast<int>(loops_.size()) ? loops_[static_cast<std::size_t>(loop)].length : 0;
}

Vec2 CitySim::loopPoint(int loop, Real s, Vec2* tangent) const {
    if (loop < 0 || loop >= static_cast<int>(loops_.size())) return Vec2(0, 0);
    const Loop& L = loops_[static_cast<std::size_t>(loop)];
    if (L.length <= 0) return L.pts.front();
    s = std::fmod(s, L.length);
    if (s < 0) s += L.length;
    const std::size_t i = static_cast<std::size_t>(std::upper_bound(L.cum.begin(), L.cum.end(), s) - L.cum.begin()) - 1;
    const std::size_t k = std::min(i, L.pts.size() - 1);
    const Vec2 a = L.pts[k], b = L.pts[(k + 1) % L.pts.size()];
    const Real seg = L.cum[k + 1] - L.cum[k];
    const Real t = seg > 1e-9 ? (s - L.cum[k]) / seg : 0;
    if (tangent) *tangent = seg > 1e-9 ? (b - a) * (1.0 / seg) : Vec2(1, 0);
    return a + (b - a) * t;
}

void CitySim::releaseSeat(Agent& a) {
    if (a.session >= 0) leaveSession(a);
    if (a.tripSeat >= 0 && a.tripSeat < static_cast<int>(seats_.size()) &&
        seats_[static_cast<std::size_t>(a.tripSeat)].occupant == indexOf(a))
        seats_[static_cast<std::size_t>(a.tripSeat)].occupant = -1;
    a.tripSeat = -1;
    a.seatPhase = 0;
}

int CitySim::pickSeat(Agent& a, Vec2 here) {
    // a free seat 30-500 m off, one of the nearest four (so neighbours do not all take the same bench)
    ActivityQuery q;
    q.kinds = spotKindBit(SpotKind::Sit);
    q.minDist = 30.0; q.maxDist = 500.0; q.nearest = 4;
    return pickSpot(a, here, q);
}

void CitySim::stepSeats(Real dt) {
    constexpr Real kWalk = 1.2;   // an amble across the grass
    constexpr Real kJog = 2.8;    // a steady run
    stepSessions();
    // the K and V tiers merged ascending (the old all-agents order): a dormant agent holds no seat (setTier)
    seatScan_.clear();
    std::merge(kIdx_.begin(), kIdx_.end(), vIdx_.begin(), vIdx_.end(), std::back_inserter(seatScan_));
    for (int si : seatScan_) {
        Agent& a = agents_[static_cast<std::size_t>(si)];
        if (a.seatPhase == 0) continue;
        // THE LEASH (ADR-0062) holds off the paths too: a plan that has got its lead ahead of its body -- across a
        // lawn, round a fountain -- waits for it. Unleashed, the body fell behind, stuck, and was teleported up to
        // its plan (a sunbather 9.8 m in one step, found by the speed watchdog).
        if (a.tethered && (a.pos - a.tetherAnchor).lengthSquared() > a.tetherLead * a.tetherLead) {
            a.speed = 0;
            continue;
        }
        // A GROUP'S PLAYER (a session): onto the area to its first point; there, standing by while the group gathers,
        // or running to point after point of its zone; and, the session over, back to where it left the path
        if (a.session >= 0 && a.session < static_cast<int>(sessions_.size())) {
            const Session& se = sessions_[static_cast<std::size_t>(a.session)];
            const ActivityDef& d = catalog_.defs[static_cast<std::size_t>(se.def)];
            const RoleDef* role = a.sessionRole >= 0 && a.sessionRole < static_cast<int>(d.roles.size())
                                      ? &d.roles[static_cast<std::size_t>(a.sessionRole)] : nullptr;
            Vec2 target = a.seatPhase == 3 ? a.seatBack : a.roamTarget;
            Real speed = kWalk;
            if (a.seatPhase == 2 && se.formation != Formation::Roam) {
                // A SETTLED GROUP: to its place (it moves as the ring grows, or with the catch), then held there,
                // facing its way -- gathering or under way alike
                Vec2 face;
                const Vec2 place = memberPlace(se, indexOf(a), &face);
                const Vec2 dd = place - a.pos;
                const Real L = dd.length();
                const Real pace = se.formation == Formation::Pair ? 1.6 : kWalk;
                if (L <= pace * dt || L < 0.02) {
                    a.pos = place;
                    a.heading = face;
                    a.speed = 0;
                } else {
                    a.pos = a.pos + dd * (std::min(L, pace * dt) / L);
                    a.heading = dd * (1.0 / L);
                    a.speed = pace;
                }
                grid_.place(indexOf(a), a.pos);
                continue;
            }
            if (a.seatPhase == 2) {
                if (se.state != Session::State::Running) {   // gathering: stand by, facing the middle
                    a.speed = 0;
                    const Vec2 to = areas_[static_cast<std::size_t>(se.area)].center - a.pos;
                    if (to.lengthSquared() > 1e-6) a.heading = normalize(to);
                    continue;
                }
                const Real f = static_cast<Real>((a.brain >> 7) & 0xFF) / 255.0;
                speed = role ? role->speedLo + (role->speedHi - role->speedLo) * f : 2.0;
            }
            const Vec2 dd = target - a.pos;
            const Real L = dd.length(), step = speed * dt;
            if (L <= step) {
                a.pos = target;
                if (a.seatPhase == 1) { a.seatPhase = 2; a.goalHours = 0; a.speed = 0; }
                else if (a.seatPhase == 2) a.roamTarget = zonePoint(se, a.sessionRole, tripRnd(a));
                else {   // off the area: out of the session, and the rest that held it ends now
                    leaveSession(a);
                    a.seatPhase = 0;
                    a.restDwell = 1e-3;
                    a.speed = 0;
                }
            } else {
                a.pos = a.pos + dd * (step / L);
                a.heading = dd * (1.0 / L);
                a.speed = speed;
            }
            grid_.place(indexOf(a), a.pos);
            continue;
        }
        if (a.tripSeat < 0 || a.tripSeat >= static_cast<int>(seats_.size())) { a.seatPhase = 0; continue; }
        const SeatSpot& s = seats_[static_cast<std::size_t>(a.tripSeat)];
        // RUNNING THE LOOP (a Jog spot): round it at a jogger's pace until the run's time is up (goalThink)
        if (a.seatPhase == 2) {
            if (s.kind != SpotKind::Jog || s.loop < 0) continue;
            a.loopS += kJog * dt;
            Vec2 tg(1, 0);
            a.pos = loopPoint(s.loop, a.loopS, &tg);
            a.heading = tg;
            a.speed = kJog;
            grid_.place(indexOf(a), a.pos);
            continue;
        }
        // to the seat, stopping in front of it (a jogger: onto the track at its place); back to where it left the path
        const Vec2 target = a.seatPhase == 1 ? (s.kind == SpotKind::Jog ? s.pos : s.pos + s.face * 0.35) : a.seatBack;
        const Vec2 d = target - a.pos;
        const Real L = d.length(), step = kWalk * dt;
        if (L <= step) {
            a.pos = target;
            a.speed = 0;
            if (a.seatPhase == 1) {
                a.seatPhase = 2;
                a.pos = s.pos;
                a.heading = s.face;
                a.loopS = s.loopS;
                a.goalHours = 0;   // the sit's own clock starts now, not on the walk over
            } else {
                releaseSeat(a);
            }
        } else {
            a.pos = a.pos + d * (step / L);
            a.heading = d * (1.0 / L);
            a.speed = kWalk;
        }
        grid_.place(indexOf(a), a.pos);
    }
}

void CitySim::stepTick(Real dt, Real hoursPerSecond) {
    stepSeats(dt);
    RT_PROFILE_ZONE_NAMED("CitySim step");
    if (!nav_ || agents_.empty()) return;
    // Print-only phase timing (CitySim::PhaseTimes): whole-population passes
    // vs per-active-agent passes. Stage 1 of the 100k plan needs a baseline.
    const auto phaseNow = std::chrono::steady_clock::now;
    auto phaseT0 = phaseNow();
    const auto stepBegin = phaseT0;
    const auto phaseMark = [&](double& into) {
        const auto n2 = phaseNow();
        into += std::chrono::duration<double, std::micro>(n2 - phaseT0).count();
        phaseT0 = n2;
    };

    // P4.1: re-hash the population. place() early-outs on an unchanged cell,
    // so a V agent (whose pose only moves on its coarse tick) costs a compare.
    // Only what can have moved: the K and V tiers (a dormant agent stands where it went under) -- unless a bulk
    // move put everyone somewhere new
    if (rehashAll_) {
        for (std::size_t i = 0; i < agents_.size(); ++i) {
            const Agent& a = agents_[i];
            grid_.place(static_cast<int>(i), a.pos);
            if (a.tier == Agent::Tier::V) vGrid_.place(static_cast<int>(i), a.pos);
            else if (a.tier == Agent::Tier::D) dGrid_.place(static_cast<int>(i), a.pos);
        }
        rehashAll_ = false;
    } else {
        for (int i : kIdx_) grid_.place(i, agents_[static_cast<std::size_t>(i)].pos);
        for (int i : vIdx_) {
            grid_.place(i, agents_[static_cast<std::size_t>(i)].pos);
            vGrid_.place(i, agents_[static_cast<std::size_t>(i)].pos);
        }
    }
    phaseMark(phase_.rehash);
    // Record the rate BEFORE tierPass: waking a dormant agent reconstructs it
    // through scheduleSnapshot, which converts commute seconds into a share of
    // the day and so needs the CURRENT rate, not last step's.
    rerateSleep(hoursPerSecond);
    hoursPerSecond_ = hoursPerSecond;
    if (hoursPerSecond > 0) lastLiveRate_ = hoursPerSecond;
    // P4.2: the tier bubble — demote K agents past the outer ring, promote V
    // agents inside the inner one (each with a catch-up tick, so the handoff
    // pose is the exact lane pose). BEFORE the clock advances: a promoted
    // agent joins this step's K passes with no double-advanced time.
    tierPass(hoursPerSecond);
    phaseMark(phase_.tierPass);
    // ACTIVE LIST (perf). Every pass below used to walk the WHOLE population
    // just to `continue` on the far tier: ~12 sweeps over thousands of
    // 432-byte agents per step, which is memory traffic, not simulation. The
    // tier bubble already knows who is live, so resolve that ONCE here and let
    // the passes iterate the survivors. Ascending indices keep the iteration
    // order (and therefore the results) bit-identical to the old sweeps.
    //
    // SLEEPERS drop out here too. An agent waiting out its day at a door has
    // nothing to contribute to any of these passes — it does not move, and a
    // resting pedestrian is already excluded from the sensed set as
    // semantically indoors — so skipping it removes it from all of them at
    // once. It reappears the instant its own schedule calls for it.
    active_.clear();
    sleeping_ = 0;
    for (int ki : kIdx_) {
        const std::size_t i = static_cast<std::size_t>(ki);
        const Agent& a = agents_[i];
        if (a.far()) continue;
        // A held clock (rate 0) keeps sleepers under: nothing on their
        // schedule can come due while the day stands still.
        if (a.wakeAt >= 0 && (hoursPerSecond_ <= 0 || simSeconds_ < a.wakeAt)) {
            ++sleeping_;
            continue;
        }
        active_.push_back(static_cast<int>(i));
    }
    phaseMark(phase_.activeList);

    clockHours_ += dt * hoursPerSecond;
    clockHours_ = std::fmod(clockHours_, 24.0);
    if (clockHours_ < 0) clockHours_ += 24.0;
    clockTotalHours_ += dt * hoursPerSecond;
    if (hoursPerSecond <= 0) heldSeconds_ += dt;
    simSeconds_ += dt;   // memory's time base: sightings age and fade against this
    signals_.update(dt);

    // Pass 1: the GOAL layer (ADR-0064) — AI agents only, the host drives
    // players. Each agent runs its current state in its archetype's GoalTable:
    // a resting agent emits this tick's events (clock windows, dwell, idle) and
    // takes the first matching transition; a GoTo agent whose trip isn't
    // running retries the departure (launch-clearance gated for drivers, so a
    // car waits out traffic near its spot and pulls out when clear). The
    // built-in tables reproduce the historical schedule and wander behaviour
    // exactly; see goalThink / city_goals.h.
    for (int ai : active_) {
        Agent& a = agents_[ai];
        const std::size_t i = static_cast<std::size_t>(ai);
        if (a.playerControlled || a.released) continue;
        // A RIDER does not re-plan: its GoTo state would see !moving and
        // relaunch the trip on foot every tick, walking it out of the car.
        // A DIRECTED agent's plan is the director's (ADR-0091): no schedule.
        if (!a.directed && !riding(ai) && !awaitingRide(ai))
            goalThink(a, dt * hoursPerSecond);
        // A departure moved the pose (idle verge -> lane start): re-hash NOW so
        // every later grid consumer this step sees current positions.
        grid_.place(static_cast<int>(i), a.pos);
    }

    // Snapshot the bodies agents may SENSE this step — pedestrians and the
    // player — so perception sees a consistent world. Each carries a STABLE id
    // (agent index; -(1+k) for the k-th host-injected point) so an observer's
    // memory can track it across steps, and its elevation so the 2.5D sensor can
    // ignore bodies a bridge deck away. AI cars are deliberately excluded (see
    // advance()): lanes + car-following + signals govern car-vs-car, and braking
    // for cross/oncoming cars in the cone deadlocked traffic.
    // The service edge is consumed now that every bus has seen it. Updated ONCE
    // per tick, after the goal pass -- flipping it inside the per-agent loop
    // would let the first bus swallow the edge and leave the other 23 in
    // service until the next boundary.
    busServiceWas_ = busesInService();

    sensed_.clear();
    // agent -> its ghost (grid lookups, which find any tier: every entry not set this tick must read -1, so the
    // ones set LAST tick are cleared rather than the whole array)
    if (sensedIndex_.size() != agents_.size()) sensedIndex_.assign(agents_.size(), -1);
    else for (int k : sensedSet_) sensedIndex_[static_cast<std::size_t>(k)] = -1;
    sensedSet_.clear();
    for (int ai : active_) {
        const Agent& a = agents_[ai];
        const std::size_t i = static_cast<std::size_t>(ai);
        // A RESTING pedestrian is INSIDE its place (home/shop/office) — not a
        // body on the street. Sensing it anyway had cars braking forever for a
        // person who is semantically indoors (a rest pose near the kerb sat in
        // the sensed corridor and froze traffic for good — device jam).
        if (a.mode == Agent::Mode::Pedestrian && !a.moving && !a.playerControlled)
            continue;
        if (a.mode == Agent::Mode::Pedestrian || a.playerControlled) {
            // A tethered walker has a physical BODY (the tether anchor the
            // bridge feeds every step) that may trail its ghost by metres —
            // cars must brake for where the person IS, not where the plan is.
            Vec2 sp = (a.mode == Agent::Mode::Pedestrian && a.tethered)
                          ? a.tetherAnchor : a.pos;
            sensedIndex_[i] = static_cast<int>(sensed_.size());
            sensedSet_.push_back(static_cast<int>(i));
            sensed_.push_back({sp, a.elevation, static_cast<int>(i)});
        }
    }
    // The live player (host-injected) is a body too — cars brake for it.
    for (std::size_t k = 0; k < externalObstacles_.size(); ++k)
        sensed_.push_back({externalObstacles_[k], 0.0, -(1 + static_cast<int>(k))});

    // Pass 2: leading gaps. Pass 3: advance. `advanced` records who really ran
    // advance() (and so had refreshPose re-anchor its base position) this tick —
    // the reactive lean below is only valid on a re-anchored pose.
    // (kept between ticks; the agents set last tick cleared -- read for any agent the grid finds)
    if (advancedFlags_.size() != agents_.size()) advancedFlags_.assign(agents_.size(), 0);
    else for (int k : advancedSet_) advancedFlags_[static_cast<std::size_t>(k)] = 0;
    advancedSet_.clear();
    std::vector<uint8_t>& advanced = advancedFlags_;
    phaseMark(phase_.goals);   // clock + signals + goal pass + sensed build
    computeGaps();
    computeCarWedge();   // S7 senses: bodies in the forward corridor
    phaseMark(phase_.gaps);
    int tetherHeldThisTick = 0;   // stranded by the leash, city-wide
    for (int ai : active_) {
        Agent& a = agents_[ai];
        const std::size_t i = static_cast<std::size_t>(ai);
        if (a.playerControlled || a.released) continue;
        // Crashed (fender-bender): the car sits where it hit until its hold
        // expires — a crash is a real stop, not a suggestion.
        if (a.crashTimer > 0) {
            a.crashTimer -= dt;
            a.speed = 0;
            a.state = Agent::State::Waiting;
            continue;
        }
        if (a.crashCount > 0) {
            Real dx = a.pos.x - a.crashAnchor.x, dy = a.pos.y - a.crashAnchor.y;
            if (dx * dx + dy * dy > 6.0 * 6.0) a.crashCount = 0;   // clear of the wreck
        }
        // Tethered ghost too far from its physical car: WAIT for it. The plan can
        // never outrun the physics — a car knocked back, climbing, or slow off the
        // line finds its ghost holding just ahead instead of gone (ADR-0062).
        if (a.moving && a.tethered) {
            Real dx = a.pos.x - a.tetherAnchor.x, dy = a.pos.y - a.tetherAnchor.y;
            if (std::sqrt(dx * dx + dy * dy) > a.tetherLead) {
                a.speed = 0;
                a.state = Agent::State::Waiting;
                ++tetherHeldThisTick;
                continue;
            }
        }
        if (a.moving) { advance(a, dt, gaps_[i], minGaps_[i]); advanced[i] = 1; advancedSet_.push_back(static_cast<int>(i)); }
    }
    // Keep the flags: "did this agent get stepped" is the question a director
    // asks when its agent stands still (ADR-0091).
    advancedLast_ = advanced;
    tetherHeld_ = tetherHeldThisTick;

    // FENDER-BENDERS (device: cars must collide, not ghost). Ambient cars are
    // planner-owned (ADR-0062) — no rigid bodies between them — so contact is
    // resolved here: when two moving cars' bodies meet on a CLOSING course, both
    // freeze on the spot (speed 0, red ring) and resume on staggered per-brain
    // holds, so a junction tangle crunches to a stop and unwinds car by car.
    // Triggering only on a closing approach lets the first resumer drive OUT
    // through the residual overlap without instantly re-freezing the pair.
    phaseMark(phase_.advMove);
    for (int ai : active_) {
        Agent& a = agents_[ai];
        const std::size_t i = static_cast<std::size_t>(ai);
        if (a.mode != Agent::Mode::Driver || !a.moving || a.released ||
            a.playerControlled)
            continue;
        // THE QUADRATIC. This was `for (j = i + 1; j < agents_.size(); ++j)`:
        // every agent in the city, per active driver, per tick, each loaded
        // from a 432-byte struct just to be rejected. O(drivers x TOTAL).
        // That is where the step went -- 96.8% of a 50k step (test_sim_scale
        // [phase]) -- and why cost per NEAR agent climbed 2.4 -> 29.5 us as the
        // TOTAL population grew, and why capping the sensing query changed
        // nothing: wrong loop.
        //
        // grid_ holds every agent and returns a SUPERSET SORTED ASCENDING, with
        // callers applying their own exact predicates -- precisely this case.
        // Skipping j <= i reproduces `j = i + 1`; ascending order keeps the
        // pair ORDER identical; every predicate below is untouched. So results
        // stay bit-identical. The radius must exceed the largest rs a pair can
        // have (0.35 * (lenA + lenB)); 30 m clears any fleet vehicle.
        constexpr Real kPairQueryRadius = 30.0;
        grid_.query(a.pos, kPairQueryRadius, pairScratch_);
        for (int gj : pairScratch_) {
            const std::size_t j = static_cast<std::size_t>(gj);
            if (j <= i) continue;
            Agent& b = agents_[j];
            if (b.mode != Agent::Mode::Driver || !b.moving || b.released ||
                b.playerControlled || b.far())
                continue;
            if (std::fabs(b.elevation - a.elevation) > 3.0) continue;  // bridge deck
            // ESCAPE VALVE: a car wedged through many consecutive freezes (a
            // wreck neither party can steer or reverse out of) stops being a
            // contact until it drives clear — the tow-truck resolution. Without
            // it a crossing-path contact never resolves and the junction dies.
            if (a.crashCount > 5 || b.crashCount > 5) continue;
            // Broad phase: the capsules below (half-length 0.5L - kHalfW, radius kHalfW) can touch out to a
            // centre distance of exactly 0.5 * (La + Lb). This was 0.35 -- the old isotropic disc -- and cut
            // the narrow phase short: two 6.4 m vans meeting at 37 degrees, centres 4.5 m apart, were
            // "no contact" while their bodies overlapped 1.8 m (#23, the packed-junction soak).
            Real rs = 0.5 * (vehicleLength(static_cast<int>(i)) +
                             vehicleLength(static_cast<int>(j)));
            Real dx = b.pos.x - a.pos.x, dy = b.pos.y - a.pos.y;
            Real d2 = dx * dx + dy * dy;
            if (d2 >= rs * rs) continue;              // broad phase (cheap reject)
            // ORIENTED narrow phase (roads-v2 S7 slice 2): a car is ~4.5 m
            // long but only ~2 m wide — the isotropic disc called every
            // adjacent-lane pass and turn-arc convergence a "crash" (measured
            // 115 phantom-heavy contacts in the 10-minute soak). Each body is
            // a CAPSULE (axis segment along the heading, half-width radius);
            // contact = exact segment-segment distance under the summed radii:
            // fires exactly at bumper touch nose-to-tail, at body overlap in a
            // side-swipe, and never for a lane-apart pass (>= 3 m centrelines).
            {
                const Real kHalfW = 0.92;   // body half-width + a small skin
                auto axis = [&](const Agent& c, int idx, Real& ex, Real& ey) {
                    const Real half =
                        std::max(Real(0), 0.5 * vehicleLength(idx) - kHalfW);
                    ex = c.heading.x * half;
                    ey = c.heading.y * half;
                };
                Real ax, ay, bx2, by2;
                axis(a, static_cast<int>(i), ax, ay);
                axis(b, static_cast<int>(j), bx2, by2);
                const Real A0x = -ax, A0y = -ay, A1x = ax, A1y = ay;
                const Real B0x = dx - bx2, B0y = dy - by2;
                const Real B1x = dx + bx2, B1y = dy + by2;
                auto ori = [](Real ox, Real oy, Real px, Real py,
                              Real qx, Real qy) {
                    const Real v = (px - ox) * (qy - oy) - (py - oy) * (qx - ox);
                    return v > 1e-9 ? 1 : (v < -1e-9 ? -1 : 0);
                };
                Real best2 = 1e30;
                if (ori(A0x, A0y, A1x, A1y, B0x, B0y) !=
                        ori(A0x, A0y, A1x, A1y, B1x, B1y) &&
                    ori(B0x, B0y, B1x, B1y, A0x, A0y) !=
                        ori(B0x, B0y, B1x, B1y, A1x, A1y)) {
                    best2 = 0;   // axes cross: bodies interpenetrate
                } else {
                    auto p2s2 = [](Real px, Real py, Real sx, Real sy,
                                   Real tx, Real ty) {
                        const Real ux = tx - sx, uy = ty - sy;
                        const Real l2 = ux * ux + uy * uy;
                        Real t = l2 > 1e-12
                                     ? ((px - sx) * ux + (py - sy) * uy) / l2
                                     : 0.0;
                        t = t < 0 ? 0 : (t > 1 ? 1 : t);
                        const Real cx = sx + ux * t - px, cy = sy + uy * t - py;
                        return cx * cx + cy * cy;
                    };
                    best2 = std::min(
                        std::min(p2s2(B0x, B0y, A0x, A0y, A1x, A1y),
                                 p2s2(B1x, B1y, A0x, A0y, A1x, A1y)),
                        std::min(p2s2(A0x, A0y, B0x, B0y, B1x, B1y),
                                 p2s2(A1x, A1y, B0x, B0y, B1x, B1y)));
                }
                if (best2 >= (2.0 * kHalfW) * (2.0 * kHalfW)) continue;
            }
            Real rvx = b.heading.x * b.speed - a.heading.x * a.speed;
            Real rvy = b.heading.y * b.speed - a.heading.y * a.speed;
            // Separating bodies part without a crash — EXCEPT when both are
            // still driving fast through each other: crossing/passing
            // geometry flips to "separating" the instant the centres pass,
            // while the bodies are still interpenetrating. That IS a crash
            // (it read as ghosting, observably), not a near miss.
            if (rvx * dx + rvy * dy >= 0 && !(a.speed > 2.0 && b.speed > 2.0))
                continue;
            // Undo the overshoot: whoever MOVED into the contact backs its path
            // progress out again, split by speed, so the pair rests touching —
            // without this a resumed car RATCHETED deeper each freeze cycle.
            Real dist = std::sqrt(d2);
            Real pen = rs - dist;
            Real vsum = a.speed + b.speed;
            if (pen > 0 && vsum > 1e-6) {
                // Returns whatever couldn't be applied (a car at the very start
                // of its leg can't back past it) so the PARTNER absorbs it —
                // otherwise a freshly-chained car at distOnLeg 0 left half the
                // penetration in place.
                auto rollBack = [&](Agent& c, Agent& other, Real amount) -> Real {
                    if (amount <= 0 || !c.moving ||
                        c.leg >= static_cast<int>(c.route.links.size()))
                        return amount;
                    // Backing up must SEPARATE the pair — a car that has already
                    // slightly passed its partner would back INTO it instead.
                    Real toOther = c.heading.x * (other.pos.x - c.pos.x) +
                                   c.heading.y * (other.pos.y - c.pos.y);
                    if (toOther <= 0) return amount;   // partner behind: hold still
                    Real applied = std::min(amount, c.distOnLeg);
                    c.distOnLeg -= applied;
                    refreshPose(c);
                    return amount - applied;
                };
                Real leftA = rollBack(a, b, pen * (a.speed / vsum));
                Real leftB = rollBack(b, a, pen * (b.speed / vsum));
                if (leftA > 0) rollBack(b, a, leftA);
                if (leftB > 0) rollBack(a, b, leftB);
            }
            if (std::getenv("RT_CRASH_DEBUG")) {
                const Real hdot = a.heading.x * b.heading.x + a.heading.y * b.heading.y;
                const Real along = a.heading.x * dx + a.heading.y * dy;
                const Real across = -a.heading.y * dx + a.heading.x * dy;
                const int la = a.leg < (int)a.route.links.size() ? a.route.links[a.leg] : -1;
                const int lb = b.leg < (int)b.route.links.size() ? b.route.links[b.leg] : -1;
                std::printf("[crash] t=%.1f pos(%.1f,%.1f) va=%.1f vb=%.1f hdot=%+.2f "
                            "along=%+.1f across=%+.1f la=%d(%.0f/%.0f) lb=%d(%.0f/%.0f) "
                            "cca=%d ccb=%d\n",
                            simSeconds_, a.pos.x, a.pos.y, a.speed, b.speed, hdot,
                            along, across,
                            la, a.distOnLeg, la >= 0 ? nav_->links[la].length : 0.0,
                            lb, b.distOnLeg, lb >= 0 ? nav_->links[lb].length : 0.0,
                            a.crashCount, b.crashCount);
                std::printf("        ids %zu/%zu gaps %.1f/%.1f wedge %.1f/%.1f lane %d/%d laneF %.2f/%.2f leg %d/%d state %d/%d hold %.1f/%.1f\n", i, j,
                            gaps_[i] > 1e8 ? -1.0 : gaps_[i], gaps_[j] > 1e8 ? -1.0 : gaps_[j],
                            carAheadGap_[i] > 1e8 ? -1.0 : carAheadGap_[i],
                            carAheadGap_[j] > 1e8 ? -1.0 : carAheadGap_[j], a.lane, b.lane, a.laneF, b.laneF,
                            a.leg, b.leg, (int)a.state, (int)b.state, a.holdTimer, b.holdTimer);
            }
            if (a.speed > 2.0 && b.speed > 2.0) ++fastCrashEvents_;
            auto crash = [this](Agent& c) {
                if (c.crashTimer <= 0) {
                    c.crashTimer = crashHoldSeconds(c.brain);
                    if (c.crashCount == 0) c.crashAnchor = c.pos;
                    ++c.crashCount;
                    ++crashEvents_;   // soak gate: total contacts over a run
                }
                // Contact means BODIES TOUCHING now (capsule), so the impact
                // kills the velocity WITH the event — without this, a pair
                // that cannot be rolled apart (both freshly chained at a
                // dead-end tip, distOnLeg ~0) spends the pre-freeze tick
                // interpenetrating at full speed: ghosting, observably.
                c.speed = 0;
                c.state = Agent::State::Waiting;
            };
            crash(a);
            crash(b);
        }
    }

    // Reactive pedestrian behaviour (ADR-0061): each walker acts on what it SEES
    // in its vision cone. It steers to one side to go AROUND the neighbours (and
    // the player) it sees ahead — and never reacts to what's behind it. A hard
    // body-overlap floor below is the physical backstop so two people can't occupy
    // the same spot. Deterministic (index order); resets to the sidewalk each
    // step, so the sidestep is a transient lean while someone is in view.
    phaseMark(phase_.advPairs);
    crowdBase_.resize(agents_.size());
    if (crowdHas_.size() != agents_.size()) crowdHas_.assign(agents_.size(), 0);
    else for (int k : kIdx_) crowdHas_[static_cast<std::size_t>(k)] = 0;   // (set only for K walkers)
    for (int ki : kIdx_) {
        Agent& a = agents_[static_cast<std::size_t>(ki)];
        if (!a.moving) { a.state = Agent::State::Resting; a.lateralOffset = 0; a.crowdOffset = Vec2(0, 0); }
    }
    for (int ki : kIdx_) {
        const std::size_t i = static_cast<std::size_t>(ki);
        Agent& a = agents_[i];
        if (!a.moving) continue;
        if (a.mode == Agent::Mode::Driver) continue;   // driver FSM is set in advance()
        // Only agents advance() re-anchored this tick may lean: the lean is
        // POS += offset on top of a fresh refreshPose. A tether-held (or host-
        // controlled) ghost skipped advance(), so leaning it again would stack
        // offset on offset — a pinned walker's ghost slid sideways without
        // bound, dragging its body after it through the tether feedback.
        if (!advanced[i]) continue;
        // `travel` is the walker's path direction this step (set by steer()); it
        // leans to `right` of that to go around what it SEES ahead.
        Vec2 travel = a.heading;
        Vec2 rightv(travel.y, -travel.x);

        // THINK on the slow clock, ACT every tick (ADR-0062). The reactive scan
        // (who's ahead, which side do I pass) runs only when this agent's think
        // timer expires — staggered per agent — and its answer is COMMITTED to
        // `leanTarget` + the FSM state until the next think. Re-deciding every
        // tick made walkers flip-flop ("wigging out"); a held decision reads as
        // intent, and the integration below still moves smoothly every tick.
        a.thinkTimer -= dt;
        if (a.thinkTimer <= 0) {
            a.thinkTimer += thinkPeriod_;
            engine::VisionCone cone;
            cone.origin = a.pos; cone.forward = travel;
            cone.range = kPedVisionRange; cone.halfAngleRad = kPedVisionHalfAngle;
            // SENSE -> REMEMBER (ADR-0063): sight the walking neighbours (and the
            // player) into memory at think cadence — successive sightings a think
            // apart are what give each track its velocity estimate.
            engine::SensorVolume sensor;
            sensor.cone = cone;
            // Agent-backed ghosts from the grid (P4.1), ascending, then the
            // host-injected externals — the order the full sensed_ scan used.
            // Slack past the cone range covers a tethered ghost at its anchor.
            grid_.query(a.pos, cone.range + 12.0, queryScratch_);
            for (int gi : queryScratch_) {
                if (gi == static_cast<int>(i)) continue;   // not myself
                if (sensedIndex_[gi] < 0) continue;        // no sensable body
                // Skip player-agent ghosts: the host already injects the live
                // player as an external point — seeing both would double it.
                if (agents_[gi].playerControlled) continue;
                const SensedGhost& g = sensed_[sensedIndex_[gi]];
                if (engine::sees(sensor, g.pos, g.elevation - a.elevation))
                    a.memory.observe(g.id, g.pos, simSeconds_);
            }
            for (const SensedGhost& g : sensed_) {
                if (g.id >= 0) continue;   // agent ghosts handled above
                if (engine::sees(sensor, g.pos, 0.0))
                    a.memory.observe(g.id, g.pos, simSeconds_);
            }
            a.memory.update(simSeconds_);
            // PREDICT -> DECIDE: lean away from where each remembered body will
            // BE shortly (its track velocity run a beat ahead), not where the
            // snapshot last had it — so two approaching walkers part early and
            // smoothly. The answer is COMMITTED (leanTarget) until the next
            // think. Poles are static and eternal: no memory needed.
            Real bias = 0; bool saw = false;
            auto consider = [&](const Vec2& p) {
                if (!engine::sees(cone, p)) return;
                Real fd = engine::forwardDistance(cone, p);
                if (fd <= 0.1 || fd >= kPedVisionRange) return;
                saw = true;
                Real side = (a.pos.x - p.x) * rightv.x + (a.pos.y - p.y) * rightv.y;
                Real w = (kPedVisionRange - fd) / kPedVisionRange;   // nearer -> stronger
                bias += (side >= 0 ? 1.0 : -1.0) * w;                // pass on the side I'm on
            };
            for (const engine::TrackedBody& t : a.memory.tracks()) {
                if (t.confidence < kMemoryActConfidence) continue;
                consider(t.pos + t.vel * kPedAnticipation);
            }
            for (const Vec2& o : staticObstacles_) consider(o);   // signal poles
            // Cars are bodies too (device: walkers pinned against a car): the
            // walker considers the CLOSEST POINT on each car's rectangle — not
            // its centre, which sits outside the short vision cone when you're
            // up against a long body — so the lean routes around the car. Both
            // driven and parked cars: vehicles_ carries every body's live pose.
            // Car bodies via their DRIVERS in the grid (P4.1): every vehicle
            // mirrors its driver's pose, and ascending driver order IS
            // ascending vehicle order (drivers are built first, one car each),
            // so the bias sum accumulates in the vehicles_ scan's order.
            grid_.query(a.pos, 26.0, queryScratch_);
            for (int di : queryScratch_) {
                const Agent& drv = agents_[di];
                if (drv.mode != Agent::Mode::Driver ||
                    drv.far() || drv.vehicle < 0 ||
                    drv.vehicle >= static_cast<int>(vehicles_.size()))
                    continue;
                const SimVehicle& v = vehicles_[drv.vehicle];
                const Real ve = (v.driver >= 0 &&
                                 v.driver < static_cast<int>(agents_.size()))
                                    ? agents_[v.driver].elevation : 0.0;
                if (std::fabs(ve - a.elevation) > 2.0) continue;   // a deck away
                const Vec2 f = v.heading, r(v.heading.y, -v.heading.x);
                const Vec2 rel(a.pos.x - v.pos.x, a.pos.y - v.pos.y);
                const Real hl = v.length * 0.5, hw = v.width * 0.5;
                const Real lx = std::max(-hl, std::min(hl, rel.x * f.x + rel.y * f.y));
                const Real ly = std::max(-hw, std::min(hw, rel.x * r.x + rel.y * r.y));
                consider(Vec2(v.pos.x + f.x * lx + r.x * ly,
                              v.pos.y + f.y * lx + r.y * ly));
            }
            a.leanTarget = std::max(Real(-1), std::min(Real(1), bias)) * kPedMaxLateral;
            // Held at a signal (advance() parked us at the kerb, speed 0,
            // Waiting): keep the honest red ring rather than repainting Walking.
            if (!(a.speed == 0 && a.state == Agent::State::Waiting))
                a.state = saw ? Agent::State::Avoiding : Agent::State::Walking;
        }

        // ACT: move the lean toward the COMMITTED target at a bounded RATE
        // (continuous, not a pop): grows while the decision says "step aside",
        // decays back to the path once a think says clear. This is what lets you
        // herd a walker like a boid.
        Real prev = a.lateralOffset;
        Real maxDelta = kPedLateralRate * dt;
        Real delta = std::max(-maxDelta, std::min(maxDelta, a.leanTarget - prev));
        a.lateralOffset = prev + delta;

        a.pos.x += rightv.x * a.lateralOffset;
        a.pos.y += rightv.y * a.lateralOffset;
        // ...and where the crowd had it last tick (kept below, after the overlap floor)
        crowdBase_[i] = a.pos;
        crowdHas_[i] = 1;
        a.pos = a.pos + a.crowdOffset;
        // Turn to face where it's actually moving (forward walk + the sideways
        // drift), so the body visibly rotates as it steers away. Uses the ped's
        // OWN speed (personality-scaled), not the nominal walk speed.
        Real walk = a.speed > 0.1 ? a.speed : kWalkSpeed * a.speedFactor;
        Vec2 vel(travel.x * walk + rightv.x * (delta / dt),
                 travel.y * walk + rightv.y * (delta / dt));
        Real vl = std::sqrt(vel.x * vel.x + vel.y * vel.y);
        if (vl > 1e-6) a.heading = Vec2(vel.x / vl, vel.y / vl);
    }
    // Hard body-overlap floor: several symmetric relaxation passes so two people
    // (whether or not they saw each other) never interpenetrate.
    phaseMark(phase_.advPop);
    for (int iter = 0; iter < 6; ++iter)
        for (int ai : active_) {
            Agent& a = agents_[ai];
            const std::size_t i = static_cast<std::size_t>(ai);
            if (a.mode != Agent::Mode::Pedestrian || !a.moving)
                continue;
            // a walker HELD by its leash this tick (not stepped) stands its ground like anyone standing: nothing
            // re-anchors it, so a shove would stay -- and the walkers round it now keep their spacing, so they press
            if (!advanced[i]) continue;
            // THE BIG ONE. This was `for (j = i + 1; j < agents_.size(); ++j)`
            // -- every agent in the city, per active walker, SIX TIMES a tick,
            // with a sqrt on most of them. Measured: 163,259 us of a 178,607 us
            // step at 50k, i.e. 91% of the whole simulation, and it scales as
            // n^2: 2,588 us at 10k, 163,259 at 50k -- 5x the agents, 63x the
            // cost. It ignores tiers entirely, which is why it got worse with
            // every agent added and why fixing the car pair scan barely moved.
            //
            // grid_ returns a superset sorted ascending and callers apply their
            // own predicates, so skipping j <= i keeps the same pairs in the
            // same order. RADIUS: this loop MOVES agents, so grid positions go
            // stale as it relaxes. Pushes are always apart, but an agent shoved
            // clear of one neighbour can drift toward another, so the radius
            // carries the body minimum plus room for six passes of drift. Pairs
            // beyond it could not have touched anyway -- the push only fires
            // below kPedBodyMin.
            grid_.query(a.pos, kPedBodyMin + 4.0, pairScratch_);
            for (int gj : pairScratch_) {
                const std::size_t j = static_cast<std::size_t>(gj);
                if (j == i) continue;
                Agent& b = agents_[j];
                if (b.mode != Agent::Mode::Pedestrian || b.far()) continue;
                // A person STANDING or SITTING out on the street holds their ground too: the walker steps round
                // them. (Only walker pairs were separated, so a walker could brush through someone standing at
                // a stop or sitting on a bench -- the one overlapping pair macOS CI caught.) Walker pairs are
                // visited once (j > i); a still body never runs this loop, so it is visited from every walker.
                const bool still = (!b.moving || !advanced[j]) && pedVisible(gj);
                if (!still && (!b.moving || j <= i)) continue;
                Real dx = a.pos.x - b.pos.x, dy = a.pos.y - b.pos.y;
                Real d = std::sqrt(dx * dx + dy * dy);
                if (d > 1e-4 && d < kPedBodyMin) {
                    if (still) {
                        const Real push = kPedBodyMin - d;
                        a.pos.x += dx / d * push; a.pos.y += dy / d * push;
                    } else {
                        Real push = (kPedBodyMin - d) * 0.5;
                        a.pos.x += dx / d * push; a.pos.y += dy / d * push;
                        b.pos.x -= dx / d * push; b.pos.y -= dy / d * push;
                    }
                } else if (d <= 1e-4) {
                    if (still) {
                        a.pos.x += kPedBodyMin;
                    } else {
                        a.pos.x += kPedBodyMin * 0.5;
                        b.pos.x -= kPedBodyMin * 0.5;
                    }
                }
            }
        }

    // KEEP THE CROWD'S SPACING: what the floor (and the offset carried in) moved each walker off its path this tick is
    // its crowd offset next tick -- at most 1.5 m, shrinking 0.4 m/s once nobody presses, so it drifts back to its
    // path as the crowd thins rather than hopping there.
    for (int ai : active_) {
        const std::size_t i = static_cast<std::size_t>(ai);
        if (i >= crowdHas_.size() || !crowdHas_[i]) continue;
        Agent& a = agents_[i];
        // not at a kerb (waiting to cross: the crowd's push must never carry it into the road), and sideways no
        // further than the lean's own limit, lean and push together
        if (a.state == Agent::State::Waiting) { a.crowdOffset = Vec2(0, 0); continue; }
        Vec2 off = a.pos - crowdBase_[i];
        const Vec2 rightv(a.heading.y, -a.heading.x);
        const Real side = dot(off, rightv), room = std::max(Real(0), kPedMaxLateral - std::fabs(a.lateralOffset));
        if (std::fabs(side) > room) off = off - rightv * (side - std::copysign(room, side));
        Real L = off.length();
        if (L > 1.5) { off = off * (1.5 / L); L = 1.5; }
        const Real shrink = 0.4 * dt;
        a.crowdOffset = L > shrink ? off * ((L - shrink) / L) : Vec2(0, 0);
    }

    // Hard radial push-out from static obstacles (signal poles) and the PLAYER: a
    // walker never ends up standing inside a pole — or brushing through the
    // player (a wider berth, so a near miss reads as a step-around). The cone
    // bias above makes it lean away in advance; this is the physical backstop.
    phaseMark(phase_.advSolver);
    for (int ki : kIdx_) {
        Agent& a = agents_[static_cast<std::size_t>(ki)];
        if (a.mode != Agent::Mode::Pedestrian || !a.moving ||
            a.far())
            continue;
        auto pushOut = [&](const Vec2& o, Real clearance) {
            Real dx = a.pos.x - o.x, dy = a.pos.y - o.y;
            Real d = std::sqrt(dx * dx + dy * dy);
            if (d > 1e-4 && d < clearance) {
                Real push = clearance - d;
                a.pos.x += dx / d * push;
                a.pos.y += dy / d * push;
            } else if (d <= 1e-4) {
                a.pos.x += clearance;        // dead-centre: pick a direction
            }
        };
        for (const Vec2& o : staticObstacles_) pushOut(o, kPoleClearance);
        for (const Vec2& o : externalObstacles_) pushOut(o, kPlayerClearance);
        // ...and never stand INSIDE a car's footprint (device: walkers stuck
        // bumping a car). The push resolves along the SHALLOWEST axis of the
        // car-local box, so a walker pressed against a door is squeezed out
        // sideways and SLIDES along the body until it clears the bumper —
        // combined with the cone lean above, it walks around the car.
        // Bodies via their drivers in the grid (P4.1), ascending = the
        // vehicles_ scan's order, so sequential pushes resolve identically.
        // Squeeze out of one car's oriented footprint along its shallowest axis.
        auto clearOfCar = [&](const SimVehicle& v, Real ve) {
            if (std::fabs(ve - a.elevation) > 2.0) return;
            const Vec2 f = v.heading, r(v.heading.y, -v.heading.x);
            const Vec2 rel(a.pos.x - v.pos.x, a.pos.y - v.pos.y);
            const Real cx = v.length * 0.5 + 0.35, cy = v.width * 0.5 + 0.35;
            const Real lx = rel.x * f.x + rel.y * f.y;
            const Real ly = rel.x * r.x + rel.y * r.y;
            if (std::fabs(lx) >= cx || std::fabs(ly) >= cy) return;
            const Real px = cx - std::fabs(lx), py = cy - std::fabs(ly);
            if (px < py) {
                const Real s = lx >= 0 ? 1.0 : -1.0;
                a.pos.x += f.x * s * px; a.pos.y += f.y * s * px;
            } else {
                const Real s = ly >= 0 ? 1.0 : -1.0;
                a.pos.x += r.x * s * py; a.pos.y += r.y * s * py;
            }
        };
        grid_.query(a.pos, 26.0, queryScratch_);
        for (int di : queryScratch_) {
            const Agent& drv = agents_[di];
            if (drv.mode != Agent::Mode::Driver || drv.far() ||
                drv.vehicle < 0 ||
                drv.vehicle >= static_cast<int>(vehicles_.size()))
                continue;
            const SimVehicle& v = vehicles_[drv.vehicle];
            const Real ve = (v.driver >= 0 &&
                             v.driver < static_cast<int>(agents_.size()))
                                ? agents_[v.driver].elevation : 0.0;
            clearOfCar(v, ve);
        }
        // PARKED cars are bodies too, and cannot be reached through a driver —
        // theirs got out and walked away. Before they were indexed separately,
        // a walker strolled straight through every car at the kerb.
        parkedGrid_.query(a.pos, 26.0, parkedScratch_);
        for (int vi : parkedScratch_) {
            if (vi < 0 || vi >= static_cast<int>(vehicles_.size())) continue;
            const SimVehicle& v = vehicles_[vi];
            if (v.driver >= 0) continue;   // driven: handled by the scan above
            if (v.offStreet) continue;     // in a garage: not in the street
            clearOfCar(v, a.elevation);    // parked at grade with the walker
        }
    }

    // P4.2: the far tier's coarse ticks — one uid-keyed bucket per fixed frame
    // (~1 Hz per agent at the 60 Hz step), each advancing by the sim time
    // accumulated since that agent's last tick. Deterministic: membership is
    // uid-derived (order-independent across promote/demote churn), processing
    // is agent-index order, and the phase never consults a wall clock.
    {
        // THE BUDGET IS IN SIM SECONDS, NOT TICKS. `vTickDivisor` (60) was
        // used as a duration — "one bucket per tick, 60 buckets" only means
        // "~1 Hz per agent" while 60 ticks happen to take a second. Slow the
        // sim and distant agents silently refreshed at half rate; their
        // positions went stale near the bubble edge, the tier pass promoted on
        // those stale samples, and K/P inflated 950 -> 2650 (K agents are the
        // DRAWN ones, so the cost landed twice). Deriving the bucket count
        // from the tick length keeps a true vRefreshSeconds refresh at ANY
        // rate: fewer buckets means more agents per tick but proportionally
        // fewer ticks — the same agents per second, the same amortised cost.
        const Real tickLen = dt > 1e-9 ? dt : (1.0 / 60.0);
        int div = static_cast<int>(vRefreshSeconds / tickLen + 0.5);
        div = std::max(1, std::min(div, vTickDivisor));
        const uint32_t bucket =
            static_cast<uint32_t>(frameIndex_ % static_cast<uint64_t>(div));
        tierScan_ = vIdx_;
        for (int vi : tierScan_) {
            const std::size_t i = static_cast<std::size_t>(vi);
            Agent& a = agents_[i];
            if (a.tier != Agent::Tier::V) continue;
            if (a.playerControlled || a.released) continue;
            if (a.uid % static_cast<uint32_t>(div) != bucket) continue;
            tickV(static_cast<int>(i), hoursPerSecond);
        }
    }
    ++frameIndex_;

    // A possessed car mirrors its driver; an unpossessed (parked) car stays put.
    for (const std::vector<int>* tierList : {&kIdx_, &vIdx_})
        for (int ti : *tierList) {
            Agent& a = agents_[static_cast<std::size_t>(ti)];
            if (a.mode == Agent::Mode::Driver && a.vehicle >= 0 &&
                a.vehicle < static_cast<int>(vehicles_.size())) {
                vehicles_[a.vehicle].pos = a.pos;
                vehicles_[a.vehicle].heading = a.heading;
            }
        }
    // RIDERS (city_transit.h) take their driver's pose. LAST, after every
    // driver has advanced, so a passenger lands on its driver's final position
    // for the tick rather than trailing it by one. rides() is sorted ascending
    // by passenger, so traversal order -- and the result -- never depends on
    // hash order (ADR-0002).
    for (const std::pair<int, int>& r : rides_.rides()) {
        const int n = static_cast<int>(agents_.size());
        if (r.first < 0 || r.first >= n || r.second < 0 || r.second >= n) continue;
        Agent& rider = agents_[static_cast<std::size_t>(r.first)];
        const Agent& drv = agents_[static_cast<std::size_t>(r.second)];
        rider.pos = drv.pos;
        rider.elevation = drv.elevation;
        rider.heading = drv.heading;
        rider.speed = drv.speed;
        grid_.place(r.first, rider.pos);
    }
    // PULL-OUT PROGRESS is the distance each car actually moved this tick,
    // whichever path moved it. Counting it inside advance() missed the
    // stop-line approach, which moves the car and returns early -- the pull
    // then advanced on alternate ticks only, a stutter in the easing.
    for (const std::vector<int>* tierList : {&kIdx_, &vIdx_})
        for (int ti : *tierList) {
            Agent& a = agents_[static_cast<std::size_t>(ti)];
            if (a.pullLen <= 0) continue;
            a.pullS += (a.pos - a.tickFromPos).length();
            if (a.pullS >= a.pullLen) a.pullLen = 0;
        }
    phaseMark(phase_.advance);
    phase_.total += std::chrono::duration<double, std::micro>(
                        std::chrono::steady_clock::now() - stepBegin).count();
    ++phase_.steps;
}

// --- three-tier traffic (P4.2) ----------------------------------------------

// The bubble: demotions swept in agent-index order, then promotions from a
// grid query around the centre (candidates ascending) — a deterministic
// sequence for a deterministic centre feed. Hysteresis (demote radius >
// promote radius) keeps a boundary agent from flapping tiers step to step.
void CitySim::tierPass(Real hoursPerSecond) {
    if (!tieringEnabled || !haveTierCenter_) return;   // no player: all K
    const Vec2 c = tierCenter_;
    tierScan_ = kIdx_;   // (a copy: demotions edit the list)
    for (int ki : tierScan_) {
        const std::size_t i = static_cast<std::size_t>(ki);
        Agent& a = agents_[i];
        if (a.tier != Agent::Tier::K) continue;
        // Player-adjacent agents never demote: host-driven, released to the
        // player, or tethered to a physical body another bridge owns -- nor one being followed.
        if (a.playerControlled || a.released || a.tethered || a.pinned) continue;
        const Real dr = a.mode == Agent::Mode::Driver ? carDemoteRadius
                                                      : pedDemoteRadius;
        const Real dx = a.pos.x - c.x, dy = a.pos.y - c.y;
        if (dx * dx + dy * dy <= dr * dr) {
            // near you now: remembered for a while after you go (only those really in range -- a fresh city starts
            // everyone K, and stamping the ones demoted this tick kept the whole population awake)
            a.rememberUntil = simSeconds_ + rememberSeconds;
            continue;
        }
        setTier(static_cast<int>(i), Agent::Tier::V);
        a.vLastTick = simSeconds_;   // its first coarse tick advances from here
        a.vHold = 0;
        // Strip K-only transients: wreck/hold state would be minutes stale at
        // re-promotion, and remembered tracks describe bodies long gone. The
        // route itself is KEPT — that is the far tier's whole point.
        a.crashTimer = 0;
        a.crashCount = 0;
        a.holdTimer = 0;
        a.memory.clear();
        ++demotions_;
    }
    // V -> D. A far agent still ticks once a second; past this range even that
    // is wasted, so it stops being simulated and will be rebuilt from its own
    // schedule when the player returns. Its car is deliberately left alone.
    if (dormancyEnabled) {
        tierScan_ = vIdx_;
        for (int vi : tierScan_) {
            const std::size_t i = static_cast<std::size_t>(vi);
            Agent& a = agents_[i];
            if (a.tier != Agent::Tier::V) continue;
            if (a.playerControlled || a.released || a.tethered) continue;
            // REMEMBERED (near you lately) or followed: stays far, on its exact trip, rather than rebuilt later
            if (a.pinned || a.rememberUntil > simSeconds_) continue;
            // NEVER A BUS, NOR ANYONE MID-JOURNEY ON ONE. A bus is the city's shared
            // state: frozen out past the bubble it stranded every rider waiting along
            // its loop, and the riders aboard with it -- measured on metro_planned, 38 of
            // 40 buses far, 5 moved in five minutes, and the regional loop (18 km, nearly
            // all of it far) carried nobody. A rider rebuilt from their schedule on waking
            // would also lose the ride. Forty buses ticking at 1 Hz cost nothing.
            if (isBus(static_cast<int>(i)) || buses_.tripOf(static_cast<int>(i))) continue;
            const Real dx = a.pos.x - c.x, dy = a.pos.y - c.y;
            if (dx * dx + dy * dy <= dormantRadius * dormantRadius) continue;
            setTier(static_cast<int>(i), Agent::Tier::D);
            a.dormantSince = simSeconds_;
            ++dormancies_;
        }
        // D -> V. Reconstruct from the schedule and hand to the far tier, never
        // straight to K: waking is an approximation, and V is the tier whose
        // whole job is being approximate until the player is close enough for
        // it to matter.
        dGrid_.query(c, dormantResumeRadius + 4.0, dormantScratch_);
        for (int idx : dormantScratch_) {
            Agent& a = agents_[static_cast<std::size_t>(idx)];
            if (a.tier != Agent::Tier::D) continue;
            const Real dx = a.pos.x - c.x, dy = a.pos.y - c.y;
            if (dx * dx + dy * dy >= dormantResumeRadius * dormantResumeRadius)
                continue;
            wakeDormant(idx);
            ++wakes_;
        }
    }

    const Real reach = std::max(carPromoteRadius, pedPromoteRadius);
    // (V agents only, at their positions as of this tick's re-hash -- grid_'s too; a woken one was placed on waking)
    vGrid_.query(c, reach + 4.0, tierScratch_);
    for (int idx : tierScratch_) {
        Agent& a = agents_[static_cast<std::size_t>(idx)];
        if (a.tier != Agent::Tier::V) continue;
        const Real pr = a.mode == Agent::Mode::Driver ? carPromoteRadius
                                                      : pedPromoteRadius;
        const Real dx = a.pos.x - c.x, dy = a.pos.y - c.y;
        if (dx * dx + dy * dy >= pr * pr) continue;
        // Catch the V state up to NOW, then hand it to K exactly where the
        // catch-up left it: the promotion pose IS the lane pose — no teleport,
        // no snap. Render + kinematic proxy pick it up on this step's bake.
        tickV(idx, hoursPerSecond);
        if (!clearPromotion(idx)) continue;   // no room behind: stays far, retries
        setTier(idx, Agent::Tier::K);
        grid_.place(idx, a.pos);
        ++promotions_;
    }
    // FOLLOWED agents are in the full sim wherever they are: woken, caught up, promoted
    for (int idx : pinned_) {
        Agent& a = agents_[static_cast<std::size_t>(idx)];
        if (a.tier == Agent::Tier::K) continue;
        if (a.tier == Agent::Tier::D) wakeDormant(idx);
        tickV(idx, hoursPerSecond);
        setTier(idx, Agent::Tier::K);
        grid_.place(idx, a.pos);
        ++promotions_;
    }
}

// A far car drives its route BLIND (vAdvance: no sensing, no queues), so the
// catch-up tick can leave it inside a live queue. Promoted as-is, it arrived at
// cruise on top of stopped cars: measured on metro, the worst junction heap sat
// exactly on the 500 m promotion ring -- a car at 15 m/s handed to the live
// sim 0.9 m behind a stopped one, the contact freeze, and the pile that grew
// from it. So seat it where a driver would be: behind the last body in its
// corridor, at that body's pace. Returns false when its route has no room
// behind it; it then stays far and tries again next pass.
bool CitySim::clearPromotion(int idx) {
    Agent& a = agents_[static_cast<std::size_t>(idx)];
    if (a.mode != Agent::Mode::Driver || !a.moving ||
        a.leg >= static_cast<int>(a.route.links.size()))
        return true;
    const int leg0 = a.leg;
    const Real dist0 = a.distOnLeg;
    const Real myLen = vehicleLength(idx);
    Real pace = a.speed;
    // The far tier's heading is its link's; the live pose is the lane curve.
    if (a.pathDir.x != 0 || a.pathDir.y != 0) a.heading = a.pathDir;
    for (int pass = 0; pass < 10; ++pass) {
        Real backBy = 0;
        grid_.query(a.pos, 30.0, queryScratch_);
        for (int bi : queryScratch_) {
            if (bi == idx) continue;
            const Agent& b = agents_[static_cast<std::size_t>(bi)];
            if (b.mode != Agent::Mode::Driver || b.far()) continue;
            if (!b.moving && b.parkedBay >= 0) continue;   // out of the lanes
            if (!b.moving && b.vehicle < 0) continue;
            if (std::fabs(b.elevation - a.elevation) > 2.5) continue;
            const Real dx = b.pos.x - a.pos.x, dy = b.pos.y - a.pos.y;
            const Real along = a.heading.x * dx + a.heading.y * dy;
            const Real across = -a.heading.y * dx + a.heading.x * dy;
            const Real hdot = a.heading.x * b.heading.x + a.heading.y * b.heading.y;
            const Real bodies = 0.5 * (myLen + vehicleLength(bi));
            if (std::fabs(across) < 2.15 && along > 0.5) {
                const Real need = bodies + kCarBumperGap + std::min(a.speed, Real(6.0)) * 0.6;
                if (along >= need) continue;
                // Ahead in my lane: stand behind it, at its pace. A body
                // CROSSING my path that close: not now.
                if (hdot <= 0.3) { a.leg = leg0; a.distOnLeg = dist0; refreshPose(a); return false; }
                backBy = std::max(backBy, need - along);
                pace = std::min(pace, std::max(Real(0), b.speed * hdot));
                continue;
            }
            // Any other body TOUCHING me (behind, beside, crossing a box):
            // backing up cannot fix that -- wait a pass; the far car moves on.
            if (dx * dx + dy * dy < bodies * bodies) {
                // Rectangle overlap, half-width 0.95 (separating axes).
                const Vec2 ax[4] = {a.heading, Vec2(-a.heading.y, a.heading.x), b.heading,
                                    Vec2(-b.heading.y, b.heading.x)};
                bool apart = false;
                for (const Vec2& n : ax) {
                    auto reach = [&](Vec2 h, Real len) {
                        return std::fabs(h.x * n.x + h.y * n.y) * len * 0.5 +
                               std::fabs(-h.y * n.x + h.x * n.y) * 0.95;
                    };
                    if (std::fabs(dx * n.x + dy * n.y) >
                        reach(a.heading, myLen) + reach(b.heading, vehicleLength(bi))) {
                        apart = true;
                        break;
                    }
                }
                if (!apart) { a.leg = leg0; a.distOnLeg = dist0; refreshPose(a); return false; }
            }
        }
        // Handed over INSIDE a junction (past the stop line, short of the
        // node): the far tier drives through boxes on a modelled dwell, not
        // the lights. Put it back at the line; the live rules take it from
        // there.
        // Past the node but still in the box: it is leaving; wait a pass.
        if (backBy <= 0) {
            const JunctionAhead ja = junctionAhead(a, 40.0);
            if (ja.leg >= 0) {
                const Real toLine = ja.toNode - stopLineBack(a, ja);
                if (toLine < 0 && ja.toNode > 0) backBy = -toLine + 0.2;
            }
            if (backBy <= 0) {
                Real behind = a.distOnLeg;
                for (int k = a.leg; k >= 0 && behind < 30.0; --k) {
                    const int from = nav_->links[a.route.links[static_cast<std::size_t>(k)]].from;
                    if (nav_->isJunction(from)) {
                        if (behind < junctionRadius(from) + 0.5 * myLen) {
                            a.leg = leg0; a.distOnLeg = dist0; refreshPose(a);
                            return false;
                        }
                        break;
                    }
                    if (k > 0) behind += nav_->links[a.route.links[static_cast<std::size_t>(k - 1)]].length;
                }
            }
        }
        if (backBy <= 0) {
            a.speed = std::min(a.speed, pace);
            return true;
        }
        pace = 0;   // backed up: it waits there, not rolls on at cruise
        Real m = backBy;
        while (m > 0) {
            if (a.distOnLeg >= m) { a.distOnLeg -= m; break; }
            m -= a.distOnLeg;
            if (a.leg == 0) { a.leg = leg0; a.distOnLeg = dist0; refreshPose(a); return false; }
            --a.leg;
            a.distOnLeg = nav_->links[a.route.links[static_cast<std::size_t>(a.leg)]].length;
        }
        refreshPose(a);
    }
    a.leg = leg0;
    a.distOnLeg = dist0;
    refreshPose(a);
    return false;
}

// One coarse tick: the goal layer at accumulated hours (departures, dwell,
// commute windows — same tables, coarser cadence), then the modelled route
// advance over the accumulated seconds. Also promotion's catch-up.
void CitySim::tickV(int i, Real hoursPerSecond) {
    Agent& a = agents_[static_cast<std::size_t>(i)];
    const Real dts = simSeconds_ - a.vLastTick;
    a.vLastTick = simSeconds_;
    if (dts <= 1e-9) return;
    if (a.playerControlled || a.released) return;
    if (!a.directed && !a.moving && !riding(i) && !awaitingRide(i))
        goalThink(a, dts * hoursPerSecond);
    if (a.moving) vAdvance(a, dts);
    grid_.place(i, a.pos);   // the far tier re-hashes on its tick, not per step
}

// Advance a V agent along its REAL A* route at the modelled pace: link class
// speed scaled by its personality, shaped by fixed average junction dwells
// (vHold). No sensing, no FSM, no live signal state — hasSignal() is static
// topology. Arrival runs the same arriveOrChain K uses (bays, goal table,
// wander chains), so a far agent's day stays the same day.
void CitySim::vAdvance(Agent& a, Real dt) {
    const bool car = a.mode == Agent::Mode::Driver;
    const int legCount = static_cast<int>(a.route.links.size());
    if (a.leg >= legCount) { a.moving = false; return; }
    Real t = dt;
    while (t > 1e-9 && a.leg < legCount) {
        if (a.busDwell > 0) {   // a bus standing at its stop
            const Real h = std::min(a.busDwell, t);
            a.busDwell -= h;
            t -= h;
            a.speed = 0;
            continue;
        }
        if (a.vHold > 0) {   // waiting out a modelled junction dwell
            const Real h = std::min(a.vHold, t);
            a.vHold -= h;
            t -= h;
            continue;
        }
        const int li = a.route.links[a.leg];
        const engine::NavLink& L = nav_->links[li];
        const Real v = (car ? engine::classSpeed(L.klass) : kWalkSpeed) *
                       a.speedFactor;
        a.speed = v;
        const Real remain = L.length - a.distOnLeg;
        if (remain > v * t) {
            a.distOnLeg += v * t;
            t = 0;
            break;
        }
        t -= v > 1e-9 ? remain / v : 0.0;
        const int toNode = L.to;
        ++a.leg;
        a.distOnLeg = 0;
        if (a.leg < legCount) {
            // The same lane re-clamp advance() applies at a leg change.
            const engine::NavLink& NL = nav_->links[a.route.links[a.leg]];
            const int lanes = std::max(1, NL.lanes);
            if (a.lane >= lanes) a.lane = lanes - 1;
            if (a.laneF > Real(lanes - 1)) a.laneF = Real(lanes - 1);
            if (nav_->isJunction(toNode))
                a.vHold += signals_.hasSignal(li) ? kVSignalDelay
                                                  : kVJunctionDelay;
        }
    }
    if (a.leg >= legCount) {
        // Arrived: park/rest/chain exactly like a K arrival. The tick's
        // leftover seconds (sub-tick) are dropped — modelled travel, not
        // integrated motion.
        a.vHold = 0;
        arriveOrChain(a, a.speed);
        return;
    }
    refreshPose(a);
    a.heading = nav_->direction(a.route.links[a.leg]);
}

}  // namespace citysim
