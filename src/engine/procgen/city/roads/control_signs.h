#ifndef RAYTRACER_ENGINE_PROCGEN_CITY_ROADS_CONTROL_SIGNS_H
#define RAYTRACER_ENGINE_PROCGEN_CITY_ROADS_CONTROL_SIGNS_H

// THE SIGNS THAT FOLLOW FROM THE ROAD'S FACTS (ADR-0424; docs/road-model-design.md, "Placement policy", part two begun):
// a STOP sign at each arm the road file states must stop, a YIELD sign at each arm stated to give way, a DO NOT ENTER
// sign at the exit end of each one-way street, a ONE WAY sign facing each road that crosses one. Each is a ROW of data,
// the `sign` of its attachment kind in assets/data/road_attachments.json (that file's top says every part): what it
// means, when it stands (over stated facts only), whose traffic it faces, where exactly, what it keeps clear of, its
// priority, what it looks like (a request to the sign kit, sign_kit.h) and its source. Nothing here names a sign: a new
// one is an entry in that file.
//
// Words used here:
//   - an arm's MOUTH is where its street's centre line meets the junction; its line runs out along the arm's heading;
//   - its STOP LINE is the model's (ModelArm::stopLine): ADR-0419's rule from the stated junction, decided in one place
//     (deck_mesh.h junctionAreas) that the paint and the lane level read too;
//   - the KERB of a street is its travel lanes' half width and its shoulder out from the centre line;
//   - a sign's FACE points toward the traffic that reads it;
//   - KERB SIDE / OFF SIDE: the side of a lane toward the kerb, and toward the centre line (the world drives on the
//     left; engine::rightOf(direction) is the kerb side of traffic going that way).
//
// Pure: the road model, the rows, and two questions about the ground (how deep inside the drawn carriageway a point
// is, and what already stands there). Placement is not guessing: a sign whose spot is on the carriageway, or too near
// something already standing, is not placed and is reported, never moved.

#include "engine/procgen/city/roads/road_model.h"

#include <nlohmann/json.hpp>

#include <functional>
#include <map>
#include <string>
#include <vector>

namespace engine {

// ONE ROW (a kind's `sign`), read.
struct ControlSignRule {
    std::string kind;                    // the attachment kind it places (its key in road_attachments.json)
    std::string means;                   // the rule set's device it stands for (empty: none)
    std::string regulation;              // when: the arm states this sign (empty: not asked)
    std::string oneWayEnd;               // when: "exit" or "any" end of a one-way street (empty: not asked)
    std::vector<std::string> controls;   // when: the junction's stated control is one of these (empty: any)
    std::string faces;                   // "arriving", "entering" or "crossing"
    std::string side;                    // "kerb" or "near_kerb_or_far_off"
    std::string elseSide;                // "off": where the kerb side has no room, the same spot across the arm
    Real fromStopLine = 0, fromKerb = 0, keepsClear = 0, crossingAngle = 30;
    int priority = 0;
    nlohmann::json look;                             // the sign kit's request
    std::map<std::string, nlohmann::json> lookAt;    // a junction control's own look
    std::string source;
};

// Every kind with a `sign` row, by priority (then by name). A row that does not read (a word not listed at the top of
// road_attachments.json, a missing look) is left out and said in `problems`.
std::vector<ControlSignRule> controlSignRules(const AttachmentKinds& kinds, std::vector<std::string>* problems = nullptr);

// DOES AN ARM STATE WHAT A ROW ASKS? The one reading of a row's `when` over an arm's stated signs (`regulation`: one of
// the arm's regulations; empty: not asked) and its junction's stated control (`controls`: one of these; empty: any).
// The signs (placeControlSigns) and the road markings (markingsOnArm) both ask it.
bool armStates(const std::string& regulation, const std::vector<std::string>& controls, const ModelJunction& J, const ModelArm& a);

// ONE MARKING ROW (a kind's `marking` in road_attachments.json: that file's top says every part): what the paint draws
// across the lanes arriving by an arm, and when -- any one of its `when_any` entries holding, each read as a sign row's
// `when` is (armStates).
struct ArmMarkingRule {
    std::string kind;                    // the attachment kind (its key in road_attachments.json)
    std::string draws;                   // what the paint draws for it (the paint's own shapes: "bar")
    // the rule set's device it is the paint of (its `means`; empty: none). THE ONE TABLE saying which device paints what
    // (issue #292): the rule set (assets/scripts/rules/lab.lua `devices`) states no paint of its own.
    std::string means;
    struct When {
        std::string regulation;
        std::vector<std::string> controls;
        int arrivingLanes = 0;           // at least this many lanes arrive by the arm (`arriving_lanes`; 0: not asked)
    };
    std::vector<When> whenAny;
    // WHERE ALONG THE ARM a marking painted before the stop line goes (the lane arrows' layout, issue #292: these were
    // numbers inside the paint): each one's distance before the arm's stop line, nearest first (`before_stop_line`);
    // the ones after the first are painted only on a block at least `repeat_needs_block` long; and the arm's road must
    // run at least `needs_road` back from the stop line. Metres; empty / 0: the row does not say.
    std::vector<double> beforeStopLine;
    double repeatNeedsBlock = 0;
    double needsRoad = 0;
    std::string source;
};
// Every kind with a `marking` row, by name. A row that does not read is left out and said in `problems`.
std::vector<ArmMarkingRule> armMarkingRules(const AttachmentKinds& kinds, std::vector<std::string>* problems = nullptr);
// WHICH MARKINGS ARE PAINTED ON THE LANES ARRIVING BY AN ARM (is a stop bar painted here? lane arrows?): THE ONE PLACE it
// is decided, from the arm's stated signs, its junction's stated control and how many lanes arrive by it
// (`arrivingLanes`), by the rows -- never from the roads' classes. The rows that hold, in the rules' order.
std::vector<const ArmMarkingRule*> markingsOnArm(const ModelJunction& J, std::size_t arm, const std::vector<ArmMarkingRule>& rules,
                                                 std::size_t arrivingLanes);

// ONE SIGN PLACED.
struct PlacedSign {
    std::string kind;
    nlohmann::json look;                 // the request, with its arrow settled
    int junction = -1, arm = -1;         // where it is attached
    Vec2 at;                             // its post's foot, plan (= world x, z)
    Vec2 face;                           // unit: the way its face points (toward its readers)
    int reader = -1;                     // faces "crossing": the arm whose traffic reads it
    Real keepsClear = 0;                 // its row's: what is placed after it keeps this far off
};
struct ControlSignPlan {
    std::vector<PlacedSign> signs;
    std::map<std::string, int> placed;   // per kind
    std::map<std::string, int> refused;  // per kind: no room, or on the carriageway
    std::vector<std::string> notes;      // each refusal, with its place and why
};

// PLACES every row over the model: rows by priority (each over every junction, arms in order), so a lower priority never
// takes a higher one's room. `carriagewayDepth`: how far
// inside the drawn carriageway a plan point lies (0: not on it; null: not asked). `standing`: what already stands on the
// pavement (signal poles, sign posts), which every sign keeps clear of by its row's `keeps_clear`; each sign placed joins
// it.
ControlSignPlan placeControlSigns(const RoadModel& m, const std::vector<ControlSignRule>& rules,
                                  const std::function<Real(const Vec2&)>& carriagewayDepth, std::vector<Vec2> standing);

}  // namespace engine

#endif
