#ifndef RAYTRACER_APPS_CITYSIM_CITY_BRAIN_SYSTEM_H
#define RAYTRACER_APPS_CITYSIM_CITY_BRAIN_SYSTEM_H

// THE RESIDENT WITH THE AGENT BRAIN, SEEN AND TOLD (ADR-0441; city_brain.h does the work, inside the sim's tick). This
// system is the viewer's side: the verbs, the markers and the plan drawn in the world, a few lines on screen, and the
// spectate camera on the person.
//
// Verbs (on a level whose citysim block states markers; each answers "err ..." otherwise):
//   brain?                       the resident in one line: what they want, the plan, the leg, the state, since when,
//                                what they wait for, where they and their car are
//   goto <marker>                they want to be at that marker from now (the day table stops; `brain day on` resumes)
//   brain trait <name> on|off    give or take one of travel.lua's traits (has_board), used from their next plan
//   brain day on|off             the day table: the round of markers the level states
//   brain follow                 the spectate camera on the person
//   markers?                     every marker: its name, street, station, kerb and where a person stands at it
//
// Drawn: each marker as a post with a ring at its foot and its name (the one the resident wants to be at, brighter);
// the resident's plan, each leg in its way's colour (travel.lua): a walk under way along the pavements it will take
// (CitySim::walkPoses, as the trip lab draws it), a drive under way along the lanes of its route, a leg still to come
// as a straight line between its ends.

#include "../../engine/control/control_commands.h"
#include "../../engine/system.h"
#include "../../engine/ui/ui_text.h"

#include <string>
#include <vector>

namespace citysim {

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
    void draw(engine::FrameContext& ctx);

    CityRenderSystem& city_;
    CitySpectateSystem& spectate_;
    engine::UiText ui_;
    bool followed_ = false;        // the camera was put on the person once, when the level came up
    bool followAsked_ = false;     // `brain follow`
};

}  // namespace citysim

#endif
