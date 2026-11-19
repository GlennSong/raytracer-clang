#ifndef RAYTRACER_APPS_CITYSIM_CITY_POSSESS_LOGIC_H
#define RAYTRACER_APPS_CITYSIM_CITY_POSSESS_LOGIC_H

#include "../../rt_math.h"

#include <cstdio>
#include <sstream>
#include <string>

namespace citysim {

using engine::Real;
using engine::Vec3;

// The PURE half of possession (ADR-0079): parsing the control channel's
// `possess.cmd` one-shot and shaping the `possess.status` reply. No ECS, no
// CitySim — testable in the Jolt-free suite; the system (city_possess.cpp) is
// the thin impure shell around this. (Possession's own car -- `car`,
// `driveto` and the bus `ride` -- went with the old driving, ADR-0459.)

// One staged command. The channel writes a single string key; commands are
// sequential (one director), so last-writer-wins is the intended semantics.
struct PossessCmd {
    enum class Kind {
        None,        // empty/blank string: nothing staged
        Walker,      // "walker [x z]" — commandeer nearest pedestrian
        WalkTo,      // "walkto x z"
        Stop,        // "stop" — stand still
        Release,     // "release" — detach brain + camera
        Direct,      // "direct on|off" — hold the goal layer off this agent
        Board,       // "board [x z]" — get into the nearest free car
        Alight,      // "alight" — get out again, on the kerb
        Enter,       // "enter" — go inside the place you are standing at
        Exit,        // "exit" — come back out
        Invalid,     // unparseable: status carries the reason
    };
    Kind kind = Kind::None;
    bool hasPos = false;   // Walker/Board: optional point supplied
    bool on = false;       // Direct: on|off
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
    if (word == "walker") {
        cmd.kind = PossessCmd::Kind::Walker;
        twoNumbers(/*required=*/false);   // position is optional
        return cmd;
    }
    if (word == "walkto") {
        cmd.kind = PossessCmd::Kind::WalkTo;
        if (!twoNumbers(/*required=*/true)) {
            cmd.kind = PossessCmd::Kind::Invalid;
            cmd.error = word + " needs <x> <z>";
        }
        return cmd;
    }
    if (word == "direct") {
        std::string sw;
        if (!(in >> sw) || (sw != "on" && sw != "off")) {
            cmd.kind = PossessCmd::Kind::Invalid;
            cmd.error = "direct needs on|off";
            return cmd;
        }
        cmd.kind = PossessCmd::Kind::Direct;
        cmd.on = sw == "on";
        return cmd;
    }
    if (word == "board") {
        cmd.kind = PossessCmd::Kind::Board;
        twoNumbers(/*required=*/false);   // optional: which car, by where it is
        return cmd;
    }
    if (word == "alight") { cmd.kind = PossessCmd::Kind::Alight; return cmd; }
    if (word == "enter") { cmd.kind = PossessCmd::Kind::Enter; return cmd; }
    if (word == "exit") { cmd.kind = PossessCmd::Kind::Exit; return cmd; }
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
    Walking,    // en route
    Arrived,    // reached the destination, holding
    NoRoute,    // destination didn't route from here
};

inline const char* possessStateName(PossessState s) {
    switch (s) {
        case PossessState::None: return "none";
        case PossessState::Idle: return "idle";
        case PossessState::Walking: return "walking";
        case PossessState::Arrived: return "arrived";
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

}  // namespace citysim

#endif
