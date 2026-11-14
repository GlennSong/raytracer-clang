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
//   - its STOP LINE stands back from the mouth by ADR-0419's rule: the widest other street at the same node (half its
//     travel width) and kStopLineBack -- the rule the junctions were grouped by, read from the stated sections;
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

// WHERE AN ARM'S STOP LINE STANDS, in metres out along it from its mouth: the built layer's (the paint's) where the
// model has it, else ADR-0419's rule (see the top of this file). kNotStated when the arm has no street or line.
Real stopLineFromMouth(const RoadModel& m, const ModelJunction& J, std::size_t arm);
// How far a street's kerb stands from its centre line: its travel lanes' half width and its shoulder.
Real kerbOffset(const ModelStreet& s);

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
