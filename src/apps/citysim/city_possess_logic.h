#ifndef RAYTRACER_APPS_CITYSIM_CITY_POSSESS_LOGIC_H
#define RAYTRACER_APPS_CITYSIM_CITY_POSSESS_LOGIC_H

#include "../../rt_math.h"

#include <cstdio>
#include <sstream>
#include <string>
#include <vector>

namespace citysim {

using engine::Real;
using engine::Vec3;

// The PURE half of possession (ADR-0079): parsing the control channel's
// `possess.cmd` one-shot, shaping the `possess.status` reply, and the speed
// policy. No Jolt, no ECS, no CitySim — testable in the Jolt-free suite; the
// system (city_possess.cpp) is the thin impure shell around this.

// One staged command. The channel writes a single string key; commands are
// sequential (one director), so last-writer-wins is the intended semantics.
struct PossessCmd {
    enum class Kind {
        None,        // empty/blank string: nothing staged
        Car,         // "car [x z]" — spawn an AI sedan (at x,z or near camera)
        Walker,      // "walker [x z]" — commandeer nearest pedestrian
        DriveTo,     // "driveto x z"
        WalkTo,      // "walkto x z"
        Stop,        // "stop" — hold the brakes / stand still
        Release,     // "release" — detach brain + camera
        Invalid,     // unparseable: status carries the reason
    };
    Kind kind = Kind::None;
    bool hasPos = false;   // Car/Walker: optional spawn point supplied
    Real x = 0, z = 0;
    std::string error;     // Invalid: what was wrong (surfaced in status)
};

inline PossessCmd parsePossessCmd(const std::string& line) {
    PossessCmd cmd;
    std::istringstream in(line);
    std::string word;
    if (!(in >> word)) return cmd;   // empty -> None
    auto twoNumbers = [&](bool required) {
        if (in >> cmd.x >> cmd.z) { cmd.hasPos = true; return true; }
        return !required;
    };
    if (word == "car" || word == "walker") {
        cmd.kind = word == "car" ? PossessCmd::Kind::Car
                                 : PossessCmd::Kind::Walker;
        twoNumbers(/*required=*/false);   // position is optional
        return cmd;
    }
    if (word == "driveto" || word == "walkto") {
        cmd.kind = word == "driveto" ? PossessCmd::Kind::DriveTo
                                     : PossessCmd::Kind::WalkTo;
        if (!twoNumbers(/*required=*/true)) {
            cmd.kind = PossessCmd::Kind::Invalid;
            cmd.error = word + " needs <x> <z>";
        }
        return cmd;
    }
    if (word == "stop") { cmd.kind = PossessCmd::Kind::Stop; return cmd; }
    if (word == "release") { cmd.kind = PossessCmd::Kind::Release; return cmd; }
    cmd.kind = PossessCmd::Kind::Invalid;
    cmd.error = "unknown possess command: " + word;
    return cmd;
}

// What the avatar is doing, for the status line (and the tests).
enum class PossessState {
    None,       // nothing possessed
    Idle,       // possessed, no destination
    Driving,    // en route (car)
    Walking,    // en route (walker)
    Arrived,    // reached the destination, holding
    Stuck,      // commanded but not moving (StuckDetector fired)
    NoRoute,    // destination didn't route from here
};

inline const char* possessStateName(PossessState s) {
    switch (s) {
        case PossessState::None: return "none";
        case PossessState::Idle: return "idle";
        case PossessState::Driving: return "driving";
        case PossessState::Walking: return "walking";
        case PossessState::Arrived: return "arrived";
        case PossessState::Stuck: return "stuck";
        case PossessState::NoRoute: return "no-route";
    }
    return "?";
}

// The `possess.status` line. One string, machine-parsable-ish but written for
// the reading agent: kind, state, pose, speed, and how much route remains.
inline std::string formatPossessStatus(const char* kind, PossessState state,
                                       const Vec3& pos, Real speed,
                                       Real remaining) {
    char buf[192];
    std::snprintf(buf, sizeof(buf),
                  "%s state=%s pos=%.1f,%.1f,%.1f speed=%.1f remaining=%.0f",
                  kind, possessStateName(state), pos.x, pos.y, pos.z, speed,
                  remaining);
    return buf;
}

// --- speed policy -----------------------------------------------------------
// The possessed car cruises at the road's class speed, capped to a city pace:
// this is a camera-carrying avatar, not a lap record. The per-point speeds
// come from routePolylineWithSpeeds; the follower looks up the entry for its
// current segment and the pursuit end-brake still owns the final approach.
constexpr Real kPossessCruiseCap = 14.0;   // m/s (~50 km/h)
constexpr Real kPossessArriveDist = 3.0;   // remaining metres that count as there

inline Real possessCruiseSpeed(Real classSpeed) {
    return classSpeed < kPossessCruiseCap ? classSpeed : kPossessCruiseCap;
}

// Pick the per-point speed for the follower's current arc station. `arcs` and
// `speeds` are index-aligned with the polyline (LaneFollower::arcs()).
inline Real speedAtStation(const std::vector<Real>& arcs,
                           const std::vector<Real>& speeds, Real station) {
    if (speeds.empty()) return 0;
    std::size_t i = 0;
    while (i + 1 < arcs.size() && arcs[i + 1] < station) ++i;
    return possessCruiseSpeed(speeds[i < speeds.size() ? i : speeds.size() - 1]);
}

}  // namespace citysim

#endif
