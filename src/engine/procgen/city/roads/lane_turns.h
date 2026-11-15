#ifndef RAYTRACER_ENGINE_PROCGEN_CITY_ROADS_LANE_TURNS_H
#define RAYTRACER_ENGINE_PROCGEN_CITY_ROADS_LANE_TURNS_H

// WHICH TURNS A JUNCTION OFFERS, AND WHICH LANE TAKES EACH (docs/road-model-design.md step 8, ADR-0426): stated once, for
// the road model, from what the road file states -- the junction's arms in order round it (the list's own order: the
// stated ring, counter-clockwise, road_layout.h armRing, ADR-0427; a heading only says whether an arm can be turned into),
// its `through` pairs, each arriving lane's kind and slot, the junction's control -- by the rows of one data table,
// assets/data/lane_turns.json (its head says every word). No angle and no distance decides anything here.
//
// Words used here:
//   - an ARM is one street's way into and out of a junction; its arriving lanes are the street's lanes that run into the
//     junction there, its leaving lanes those that run out of it;
//   - a lane's SLOT is its place across its road among the lanes running its way, from the centre-line side (0) to the
//     kerb side (lane_expand.h Lane::slot; a pocket on the kerb side is one beyond the kerb lane, one on the centre-line
//     side -1);
//   - NEAR TURN: the turn that keeps to the kerb side; FAR TURN: the turn across the oncoming traffic. This world drives
//     on the left: the kerb side comes from one place (lane_graph_build.h kerbPerpSign), never a hand.

#include "engine/procgen/city/roads/lane_graph.h"   // Turn

#include <nlohmann/json.hpp>

#include <array>
#include <string>
#include <vector>

namespace engine {

// ONE ROW of the table (see assets/data/lane_turns.json).
struct LaneTurnRow {
    Turn turn = Turn::None;
    enum class When : uint8_t { Any, StraightOn, NoStraightOn } when = When::Any;
    std::vector<std::string> controls;   // empty: any control
    std::vector<std::string> kinds;      // the arriving lanes' kinds it applies to
    enum class Take : uint8_t { Every, Count, ShareMore, ShareLess } take = Take::Every;
    bool fromKerb = true;                // count from the kerb side (else from the centre-line side)
    uint32_t count = 1;                  // Take::Count
    enum class Into : uint8_t { SameSlot, SameSide } into = Into::SameSlot;
    std::string source;
};
struct LaneTurnRules {
    std::vector<LaneTurnRow> rows;
    uint64_t hash = 0;                   // of the file's bytes (what keys the caches that hold turns)
};
// The rows of a parsed table; a row that does not read is left out and said in `problems`.
LaneTurnRules parseLaneTurnRules(const nlohmann::json& doc, std::vector<std::string>* problems = nullptr);
// assets/data/lane_turns.json, read once. No built-in copy: a missing or unreadable file gives no rows (and says why in
// the log), so no junction offers a turn and the lane-level table reports it.
const LaneTurnRules& laneTurnRules();
// The hash of a table's file, its bytes as they are (a comment edit moves it too): LaneTurnRules::hash.
uint64_t laneTurnTableHash(const std::string& text);

// ONE ARM AS THE TABLE SEES IT: its stated heading, and its arriving and leaving lanes, each with its kind and slot (the
// caller's lane ids, in any order).
struct TurnArmLane {
    uint32_t lane = 0;
    std::string kind;
    int slot = 0;
    std::string name;   // for the notes (the builder's lane id); empty: its number
};
struct TurnArm {
    double heading = 0;                  // degrees, the way the arm leaves the junction, plan coordinates
    bool hasHeading = false;
    std::vector<TurnArmLane> arriving, leaving;
};
// One turn offered: from arriving lane `from` of arm `fromArm` into leaving lane `to` of arm `toArm`, its kind, and the
// table row that stated it.
struct StatedTurn {
    uint32_t fromArm = 0, toArm = 0;
    uint32_t from = 0, to = 0;
    Turn turn = Turn::None;
    uint32_t row = 0;
};

// THE KIND OF EACH TURN FROM ARM `from` (see the table's head): per arm, Turn::None for `from` itself and for an arm with
// no stated heading. `arms` IN RING ORDER, counter-clockwise (the junction's stated list: road_layout.h armRing); their
// headings are not read for the order. `kerbCcw`: whether the kerb side of arriving traffic lies counter-clockwise round the junction from
// its arm (in plan coordinates); `halfway` (optional, out): the arms taken as far turns only because they stood halfway
// round with no partner stated.
std::vector<Turn> turnKindsFrom(const std::vector<TurnArm>& arms, const std::vector<std::array<uint32_t, 2>>& through,
                                uint32_t from, bool kerbCcw, std::vector<uint32_t>* halfway = nullptr);
// Whether the kerb side of a junction's arriving traffic lies counter-clockwise from its arm, in plan coordinates: from
// lane_graph_build.h kerbPerpSign, the one statement of which side the kerb is.
bool kerbSideCounterClockwise();

// THE TURNS A JUNCTION OFFERS, every arriving lane into every leaving lane the rows give it, in the order (arriving arm,
// leaving arm, from lane, to lane). Every row whose conditions hold applies (a turn two rows state is kept once).
// `control`: the junction's stated control. `notes` (optional): what could not be settled, in words -- an arm with no
// heading; a side taken as far; an arriving lane that takes no turn, because no row covers its kind there or because the
// junction offers its arm no way out at all.
// `kerbCcw`: which side the kerb is (kerbSideCounterClockwise; a test of a mirrored world passes the other).
std::vector<StatedTurn> turnsAtJunction(const std::vector<TurnArm>& arms, const std::vector<std::array<uint32_t, 2>>& through,
                                        const std::string& control, const LaneTurnRules& rules,
                                        std::vector<std::string>* notes = nullptr, bool kerbCcw = kerbSideCounterClockwise());

const char* turnKey(Turn t);   // the table's word for a kind of turn ("straight", "near_turn", ...)

}  // namespace engine

#endif
