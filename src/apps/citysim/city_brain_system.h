#ifndef RAYTRACER_APPS_CITYSIM_CITY_BRAIN_SYSTEM_H
#define RAYTRACER_APPS_CITYSIM_CITY_BRAIN_SYSTEM_H

// THE RESIDENTS WITH THE AGENT BRAIN, SEEN AND TOLD (ADR-0441, ADR-0443; city_brain.h does the work, inside the sim's
// tick). This system is the viewer's side: the verbs, the markers and the plan drawn in the world, a few lines on
// screen, and the spectate camera on one person.
//
// THE FOLLOWED RESIDENT is the one the camera is on and the panel is about; a verb given no name acts on them. `<who>`
// is a resident's name (`ana`) or number (`0`).
//
// Verbs (on a level whose citysim block states markers; each answers "err ..." otherwise):
//   brain? [who]                       a resident in one line: what they want, the plan, the leg, the state, since
//                                      when, what they wait for, where they and their household's cars are, why
//   residents?                         every resident in a few words: name, household, what they are doing, where
//   goto [who] <marker>                they want to be at that marker from now (their day table stops)
//   brain trait [who] <name> on|off    give or take one of travel.lua's traits (has_board), used from their next plan
//   brain day [who|all] on|off         a day table: the round of markers the level states for that person
//   brain follow                       the camera on the NEXT resident, in the level's order, round and round
//   brain follow <who>                 the camera on that resident
//   brain follow off                   the camera let go, so `camera <x> <y> <z> <pitch> <yaw>` can place it
//   markers?                           every marker: its name, street, station, kerb and where a person stands at it
//
// Drawn: each marker as a ring round the spot where a person at it stands, a post beside the ring and its name (the
// one the followed resident wants to be at, brighter). The followed resident's plan, each leg in its way's colour
// (travel.lua): a walk under way along the pavements it will take; a drive under way along the lanes and turns of its
// route, from the car's front bumper on; a leg with no route yet as a straight DASHED line between its ends. Each
// resident's name above their head. On screen: the followed resident's name and household, what they want, the plan,
// and what they are doing in the data's own words (brain.lua `label`).
//
// INSPECT BY LOOKING (ADR-0443): the resident, or the household car, nearest the line through the middle of the view
// (within kLookWithin of it, no farther than kLookRange from the camera; failing that, the resident within kLookNear of
// the camera) has its `brain?` line, or the car's own line, shown at the foot of the screen. The engine's own "what is
// the view's centre on" (`aim?`) answers with a physics body, and a walker or a car of the sim is an instance without
// one of its own, so the line is measured against the sim's positions here.

#include "../../engine/control/control_commands.h"
#include "../../engine/system.h"
#include "../../engine/ui/ui_text.h"

#include <string>
#include <vector>

namespace citysim {

class CityBrain;
class CityRenderSystem;
class CitySpectateSystem;

class CityBrainSystem : public engine::System {
public:
    CityBrainSystem(CityRenderSystem& city, CitySpectateSystem& spectate) : city_(city), spectate_(spectate) {}

    void onStart(engine::FrameContext& ctx) override;
    void onStop(engine::FrameContext& ctx) override;
    void update(engine::FrameContext& ctx) override;
    void render(engine::FrameContext& ctx) override;

private:
    std::string brainVerb(const engine::ControlCommand& cmd);
    std::string gotoVerb(const engine::ControlCommand& cmd);
    std::string markersLine() const;
    std::string residentsLine() const;
    void draw(engine::FrameContext& ctx);
    // What the view's centre is on: a resident (`who`), or a household's car (`car`, its number), or neither (both -1).
    void lookedAt(const engine::FrameContext& ctx, const CityBrain& b, int& who, int& car) const;

    CityRenderSystem& city_;
    CitySpectateSystem& spectate_;
    engine::UiText ui_;
    bool labels_ = true;           // `brain labels on|off`: the places' and residents' names drawn over the view (#299)
    int followed_ = 0;             // the followed resident: the camera's, the panel's, and a nameless verb's
    bool camOnce_ = false;         // the camera was put on them once, when the level came up
    bool followAsked_ = false;     // `brain follow`: put the camera on followed_ at the next update
};

}  // namespace citysim

#endif
