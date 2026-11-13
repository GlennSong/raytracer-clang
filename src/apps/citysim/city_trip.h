#ifndef RAYTRACER_APPS_CITYSIM_CITY_TRIP_H
#define RAYTRACER_APPS_CITYSIM_CITY_TRIP_H

// A PERSON SENT SOMEWHERE (ADR-0415, the one-agent lab; Glenn: "the agent should decide where to go and how to get
// there ... plan the trip and then we can see their path and follow the agent until they reach their destination").
//
// The person is the city's first resident with a home (on agent_lab, its only one). Told `trip near` or `trip far`
// they pick a building themselves -- a place a near or far distance off (travel.lua's bands), drawn from the city's
// places by their own seeded dice; told `trip to <place>` or `trip at <x> <z>`, they go where they are told. Then the
// trip planner (trip_plan.h) prices every way of travelling, they take the cheapest, and its legs are carried out in
// order with the sim's own verbs: walk (sendAgentTo on foot), get in (boardVehicle), drive to a space near the door
// (sendAgentTo in the car: the sim reserves the space), get out (alightVehicle leaves the car in it), walk to the
// door. A walk's last few steps off the pavement -- to the car, to a door -- and a walk short enough to need no route
// are walked out to the pavement, along it and in (stepAgentToward). A leg that cannot be started re-plans from where
// the person stands without that way of travelling, and the trip says so.
//
// Seen: the plan's legs drawn along the way they will be travelled (CitySim::walkPoses: the sim's own placing), in
// each way's colour, the destination marked, a few lines on screen
// (where to, the options and their minutes, the choice, the leg under way), and the spectate camera following the
// person from the moment the trip starts. `trip?` answers all of it in one line for a director.
//
// Keys (on a level that keeps no day schedule, citysim.schedule = false: the trip lab): Y = trip near, I = trip far,
// U = trip freeway (ADR-0416: the building whose drive from here uses the freeway furthest).
// Verbs (any city level): trip near|far|freeway|to <place name or id>|at <x> <z>|mark (the map's waypoint),
// and trip?.

#include "../../engine/control/control_commands.h"
#include "../../engine/system.h"
#include "../../engine/ui/ui_text.h"
#include "places.h"
#include "trip_plan.h"

#include <string>
#include <vector>

namespace citysim {

class CityRenderSystem;
class CitySpectateSystem;

class CityTripSystem : public engine::System {
public:
    CityTripSystem(CityRenderSystem& city, CitySpectateSystem& spectate) : city_(city), spectate_(spectate) {}

    void onStart(engine::FrameContext& ctx) override;      // keys, the two verbs, travel.lua
    void onStop(engine::FrameContext& ctx) override;       // the verbs go with the system
    void fixedUpdate(engine::FrameContext& ctx) override;  // the trip's clock
    void update(engine::FrameContext& ctx) override;       // requests, the legs, the plan drawn
    void render(engine::FrameContext& ctx) override;       // the lines on screen

private:
    std::string stage(const engine::ControlCommand& cmd);   // the `trip` verb: checked, then taken up in update
    std::string status() const;                             // the `trip?` line
    void take(engine::FrameContext& ctx, const std::string& request);
    int person() const;                                      // the agent who travels; -1 when the city has none
    bool pickPlace(bool far, PlaceId& out, std::string& why);
    bool pickFreeway(PlaceId& out, std::string& why);       // the building whose drive uses the freeway furthest
    void begin(engine::FrameContext& ctx, Vec2 dest, PlaceId place, const std::string& label);
    TripRequest requestTo(Vec2 dest, const std::vector<int>& ruledOut) const;
    bool plan(const std::string& note);                     // (re)plan from where the person stands
    void step();                                            // start the next leg, or see the current one through
    bool startLeg();
    void finishLeg();
    engine::Vec2 walkTarget(std::size_t k) const;           // where walk leg k ends: a door, or the car's
    std::vector<engine::Vec2> footSteps(engine::Vec2 from, engine::Vec2 to) const;   // off the paths: via the pavement
    void draw(engine::FrameContext& ctx);
    // leg k of the choice as it will be travelled, and each point's colour (the way's, or its kind of road's)
    std::vector<engine::Vec3> legLine(std::size_t k, std::vector<engine::Vec3>& colours) const;
    engine::Vec3 roadColour(int link, const engine::Vec3& way) const;
    Real metresLeft() const;
    std::string placeLabel(PlaceId id) const;

    CityRenderSystem& city_;
    CitySpectateSystem& spectate_;
    TravelCatalog travel_;
    bool haveTravel_ = false;
    std::string travelError_ = "travel.lua not read";
    std::vector<std::string> requests_;
    engine::UiText ui_;

    // THE TRIP
    bool active_ = false;
    int who_ = -1;
    PlaceId place_ = kNoPlace;
    std::string where_;          // "cafe \"Sumi\" (#120)", or "x z"
    Vec2 dest_;
    TripPlan plan_;
    std::vector<int> ruledOut_;  // ways a leg of which could not be done, this trip
    int leg_ = 0;                // in the chosen option
    bool legUnderWay_ = false;
    bool straight_ = false;      // the leg's last steps (or all of it), walked straight off the paths
    bool straightDone_ = false;  // ...and there
    std::vector<engine::Vec2> steps_;   // ...the points it goes by (footSteps), the next first
    bool arrived_ = false;
    std::string note_;           // what happened on the way (a re-plan, a failure)
    int trips_ = 0;              // trips begun: the person's dice for the next pick
    double simTime_ = 0;         // sim seconds (fixed steps), the trip's clock
    double startedAt_ = 0, legStartedAt_ = 0, arrivedAt_ = 0;
    Real plannedMinutes_ = 0;    // the choice's minutes when first planned
    std::vector<Real> legActual_; // the minutes each leg done so far took (trip? sets them against the plan's)
    std::vector<std::vector<engine::Vec3>> lines_;   // each leg's line as last drawn (a leg to come: drawn once)
    std::vector<std::vector<engine::Vec3>> lineColours_;   // ...each point's colour
    std::string freewayNote_;    // what the last freeway pick found (trip? and the box say it)
};

}  // namespace citysim

#endif
