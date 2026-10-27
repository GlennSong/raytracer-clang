#ifndef RAYTRACER_ENGINE_CONTROL_CITY_REQUESTS_H
#define RAYTRACER_ENGINE_CONTROL_CITY_REQUESTS_H

#include "../properties.h"

#include <string>

namespace engine {

// Requests the engine's control verbs send to the city sim (ADR-0253). Application is city-free (the engine_core
// charter), so these are the contract: the engine publishes them on the bus, the city's systems (apps/citysim)
// hear them, and a state without a city has nobody listening -- the verb then says so.

// PossessRequest: one possession command line, in the grammar parsePossessCmd reads (city_possess_logic.h):
// "car [x z]", "walker [x z]", "driveto x z", "walkto x z", "direct on|off", "board [x z]", "enter", "exit",
// "alight", "ride x z", "stop", "release". CityPossessSystem hears it; `possess?` answers.
struct PossessRequest {
    std::string command;
};
inline void describeProperties(PossessRequest& r, PropertyVisitor& v) {
    v.field(FieldMeta("command"), r.command);
}

// RideRequest: the player boards the nearest bus (1) or cab (2) or either (3), or gets off (0). The socket's
// `ride bus|taxi|any|off`; CityPlayerTransitSystem hears it.
struct RideRequest {
    int mode = 3;
};
inline void describeProperties(RideRequest& r, PropertyVisitor& v) {
    v.field(FieldMeta("mode").range(0, 3), r.mode);
}

// TrainBoardRequest: put the player aboard a train, standing mid-carriage (a probe's way in; on foot you walk in
// through a door). TrainSystem hears it; `trains?` answers.
struct TrainBoardRequest {
    int train = 0;
    int car = 1;
};
inline void describeProperties(TrainBoardRequest& r, PropertyVisitor& v) {
    v.field(FieldMeta("train"), r.train);
    v.field(FieldMeta("car"), r.car);
}

// KnockRequest: throw the walker nearest the player down, as a car would (the socket's `knock`, for trying the
// ragdolls without driving into someone). CityWalkerSystem hears it; `knock?` answers.
struct KnockRequest {};
inline void describeProperties(KnockRequest&, PropertyVisitor&) {}

// PinRequest: follow an agent (on) -- it stays in the full sim however far it goes -- or stop (off). The socket's
// `pin` / `unpin`; CityRenderSystem hears it, `pin?` answers.
struct PinRequest {
    int agent = -1;
    bool on = true;
};
inline void describeProperties(PinRequest& r, PropertyVisitor& v) {
    v.field(FieldMeta("agent"), r.agent);
    v.field(FieldMeta("on"), r.on);
}

// FindPlaceRequest: the places whose name contains `name` (a shop by its sign). CityRenderSystem hears it;
// `findplace?` answers: where, what, open now, and who is in at its door.
struct FindPlaceRequest {
    std::string name;
};
inline void describeProperties(FindPlaceRequest& r, PropertyVisitor& v) {
    v.field(FieldMeta("name"), r.name);
}

}  // namespace engine

#endif
