#include "engine/procgen/city/roads/road_problem.h"

#include "check.h"

#include <cstdio>
#include <cstring>

namespace engine {

namespace {

using K = RoadProblemKind;

// THE KINDS, in the enum's order: name, meaning, sentence.
constexpr RoadProblemKindRow kRows[kRoadProblemKindCount] = {
    {K::LaneNotBesideSpine, "lane not beside its spine",
     "A lane's points were not laid one beside each point of its road's centre line, so it cannot be cut at stations.",
     "lane {lane}: not laid beside its road's spine point for point: no lane path"},
    {K::LaneFoldTidied, "lane fold tidied",
     "A lane on the inside of a sharp corner doubled back on itself; the fold was cut out and the lane is shorter.",
     "lane {lane} runs backwards for {n0:2} m near ({x:1}, {y:1}): it lies on the inside of a corner of road {road} sharper than "
     "its offset from the road's spine; tidied (the fold cut out, {n1:2} m shorter)"},
    {K::LaneNothingLeft, "lane gone after tidying",
     "Cutting the folds out of a lane left nothing of it.",
     "lane {lane}: nothing of it is left once its folds are cut out: no lane path"},
    {K::ArmNoStopLine, "arm without a stop line",
     "A junction's arm names a street whose road is not in the road file, so it has no stop line.",
     "junction {junction}: arm {arm} (street {street}) has no stop line: its road is not in the file"},
    {K::StreetEndNoStopLine, "street end without a stop line",
     "A street ends at a junction that has no arm with a stop line for it; its lane runs to the street's end and stops.",
     "street {street}: its {w0} end names junction {junction}, which has no arm with a stop line for it: lane {lane} runs on to "
     "the street's end there, joined to nothing across the junction"},
    {K::NoRoomBetweenStopLines, "no room between stop lines",
     "The stop lines at the two ends of a street leave no length of lane between them.",
     "lane {lane} on street {street}: no room between its stop lines ({n0:2} to {n1:2} along road {road}): no lane path there"},
    {K::LaneOnNoStreet, "lane on no street",
     "A lane runs along none of the stated streets and inside no junction.",
     "lane {lane}: it runs along none of the road model's streets, nor inside a junction: no lane path"},
    {K::StreetEndUnstated, "street end with nothing stated",
     "A street ends where it meets another road (or part way along its own), and the road file states no junction or "
     "carry-on there.",
     "street {street}: its {w0} end at ({x:1}, {y:1}) {w1}, but the road file states no junction there and nothing it carries on "
     "into: its lanes stop there, joined to nothing (the planner's to state)"},
    {K::CarryOnNotNamedBack, "carry-on not named back",
     "A street says it carries on into another, which does not say the same of it.",
     "street {street} carries on into {street2}, which does not name it back: its lanes lead nowhere there"},
    {K::CarryOnAmbiguous, "carry-on with no one way on",
     "A street carries on into another where no lane leaves, or which names it back at both ends.",
     "street {street} carries on into {street2}, {w0}: lane {lane} leads nowhere there"},
    {K::CarryOnNoLaneLeaves, "carry-on with no lane going that way",
     "A street carries on into another that has no lane leaving the way this lane runs.",
     "street {street} carries on into {street2}, where no lane leaves its end the way lane {lane} runs"},
    {K::CarryOnOtherSlot, "carry-on into another slot",
     "A lane carries on into a street with no lane in its place across the road; it leads into the nearest.",
     "lane {lane} (slot {n0:0}) carries on from street {street} into {street2}, which has no lane in its slot: into lane {lane2} "
     "(slot {n1:0})"},
    {K::CarryOnGap, "carry-on joined across a gap",
     "Two lanes joined where one street carries on as another, their lines not meeting.",
     "lane {lane} carries on from street {street} into lane {lane2} of street {street2}, their lines {n0:3} m apart at ({x:2}, "
     "{y:2}): joined"},
    {K::ConnectionNotJoined, "leave or merge not joined",
     "A lane the model says leaves or merges into another could not be joined to it: one of the two has no lane path there.",
     "lane {lane} {w0} lane {lane2} at station {station:2} along {road}, where {w1} has no lane path to {w2}: not joined"},
    {K::ConnectionGap, "leave or merge joined across a gap",
     "A lane joined to the lane it leaves or merges into, their lines not meeting.",
     "lane {lane} {w0} lane {lane2}, their lines {n0:3} m apart at ({x:2}, {y:2}): joined"},
    {K::TurnNoJunction, "turn at no junction",
     "A stated turn names a junction the road model does not have.",
     "stated turn {n0:0} names no junction of the road model: not built"},
    {K::JunctionNoLanePath, "junction no lane path reaches",
     "Turns are stated at a junction that no lane path arrives at or leaves.",
     "junction {junction}: {n0:0} turns are stated, but no lane path reaches it: none built"},
    {K::ArmWaysCancel, "arm left out of the outline",
     "The lanes of one arm point opposite ways across it, so the arm's edge of the junction cannot be drawn.",
     "junction {junction}: arm {arm} (road {road}): its lanes' ways across the arm cancel out: the arm is left out of the "
     "junction's outline"},
    {K::OutlineFewCorners, "outline with too few corners",
     "A junction's outline has fewer than three corners.",
     "junction {junction} at ({x:1}, {y:1}): its outline has {n0:0} corners"},
    {K::OutlineCrossesItself, "outline crossing itself",
     "A junction's outline crosses itself: two arms' stop lines overlap.",
     "junction {junction} at ({x:1}, {y:1}): its outline crosses itself (two arms' stop lines overlap: a corner for step 9)"},
    {K::SignOnArmNoLane, "sign on an arm no lane arrives by",
     "An arm states a sign, and no lane arrives by that arm to obey it.",
     "junction {junction}: its arm on road {road} carries \"{w0}\", but no lane arrives by it"},
    {K::TurnNoLanePath, "turn with no lane path",
     "A stated turn has no lane path to start from or to end on, so no path through the junction is built for it.",
     "junction {junction}: the {w0} from lane {lane} into lane {lane2} has no lane path {w1}: not built (it stays stated)"},
    {K::TurnFitsNoVehicle, "fits no vehicle",
     "A stated turn's corner is too tight or too narrow for every vehicle: a corner fault.",
     "junction {junction}: the {w0} from lane {lane} into lane {lane2} fits no vehicle: {w1} (it stays stated; a corner fault)"},
    {K::TurnsNotAllPlaced, "turns not all placed",
     "Some stated turns were neither built nor refused (should never happen).",
     "of the {n0:0} stated turns, {n1:0} were placed at their junctions: the rest neither built nor refused"},
    {K::LaneHoleAtStations, "centimetre hole between stations",
     "A hole of about a centimetre between two of a lane's streets, from stations written to the centimetre.",
     "lane {lane}: {n0:3} m between two of its streets' stated stations, {n1:2} m along it, with no lane path and no junction "
     "there (the road file writes stations to the centimetre)"},
    {K::LaneUncovered, "lane stretch with no lane path",
     "A stretch of a lane has no lane path, and nothing that left it out says why.",
     "lane {lane}: from {n0:2} to {n1:2} m along it no lane path covers it, and nothing that left it out says why"},
    {K::LaneKink, "kink in a lane",
     "A lane's line turns sharply at one point, more than the bend around it.",
     "kink: the line of lane {lane} (lane path {path}) jumps {n0:1} degrees at one point, {n1:1} m along it at ({x:1}, {y:1}), "
     "beyond the bend around it (more than {n2:0}): kept as drawn, and the bend profile slows for it"},
    {K::ConnectorKink, "kink in a connector",
     "A path through a junction turns sharply at one point, more than the bend around it.",
     "kink: connector {path} (junction {n3:0}) jumps {n0:1} degrees at one point, {n1:1} m along it at ({x:1}, {y:1}), beyond the "
     "bend around it (more than {n2:0})"},
    {K::JoinKink, "kink where paths join",
     "Where one path leads into the next, the heading jumps.",
     "kink: {w0} {path} leads into {w1} {path2} with its heading jumping {n0:1} degrees at ({x:1}, {y:1}) (more than {n1:0})"},
    {K::MergeNotCarried, "merge with no join found",
     "The model states a merge, and no crossing point holds a join of the two lanes' paths, so who gives way is not carried.",
     "lane {lane} merges into lane {lane2} (the road model's connection {n0:0}), but the crossing points hold no join of their "
     "paths: who gives way there is not carried"},
    {K::LaneGraphCheck, "lane table check",
     "The finished lane table fails one of its own checks.",
     "lane graph: {w0}"},
    {K::ManyCrossingPoints, "very many crossing points",
     "A junction has very many places where two paths' bodies could touch: slow to plan and to drive.",
     "crossing points: junction {n0:0} has {n1:0}, more than {n2:0}: a slow junction for the junction plan and for every car there"},
    {K::PathsNearlyOverEachOther, "paths nearly over each other",
     "Two paths lie within touching distance of each other for nearly their whole length.",
     "crossing points: {w0} and {w1} are within touching distance for {n0:0}% and {n1:0}% of their lengths ({w2}, crossing point "
     "{n2:0}, at ({x:1}, {y:1})): two paths laid nearly over each other"},
    {K::CrossingOutsideJunction, "paths meeting outside a junction",
     "A path through a junction meets another path outside every junction's outline.",
     "crossing points: {w0} and {w1} meet at ({x:1}, {y:1}) ({w2}, crossing point {n0:0}), outside every junction's outline"},
};

std::string decimal(Real v, int decimals) {
    char b[64];
    std::snprintf(b, sizeof b, "%.*f", decimals < 0 ? 0 : decimals, v);
    return b;
}

// One {key} or {key:N} of a kind's sentence, from the record.
std::string field(const RoadProblem& p, const std::string& key, int decimals) {
    if (key == "junction") return p.junction;
    if (key == "street") return p.street;
    if (key == "street2") return p.street2;
    if (key == "lane") return p.lane;
    if (key == "lane2") return p.lane2;
    if (key == "road") return p.road;
    if (key == "arm") return std::to_string(p.arm);
    if (key == "path") return std::to_string(p.path);
    if (key == "path2") return std::to_string(p.path2);
    if (key == "x") return decimal(p.at.x, decimals);
    if (key == "y") return decimal(p.at.y, decimals);
    if (key == "station") return decimal(p.station, decimals);
    if (key.size() == 2 && key[0] == 'n' && key[1] >= '0' && key[1] <= '9') {
        const std::size_t i = static_cast<std::size_t>(key[1] - '0');
        CHECK(i < p.numbers.size());
        return i < p.numbers.size() ? decimal(p.numbers[i], decimals) : std::string("?");
    }
    if (key.size() == 2 && key[0] == 'w' && key[1] >= '0' && key[1] <= '9') {
        const std::size_t i = static_cast<std::size_t>(key[1] - '0');
        CHECK(i < p.words.size());
        return i < p.words.size() ? p.words[i] : std::string("?");
    }
    CHECK(false && "a problem kind's sentence names a field the record does not have");
    return "?";
}

}  // namespace

const RoadProblemKindRow& problemKindRow(RoadProblemKind kind) {
    const std::size_t i = static_cast<std::size_t>(kind);
    CHECK(i < kRoadProblemKindCount && kRows[i].kind == kind);
    return kRows[i < kRoadProblemKindCount ? i : 0];
}

std::string problemLine(const RoadProblem& p) {
    std::string out;
    for (const char* c = problemKindRow(p.kind).line; *c; ++c) {
        if (*c != '{') {
            out += *c;
            continue;
        }
        const char* end = std::strchr(c, '}');
        if (!end) break;
        std::string key(c + 1, end);
        int decimals = -1;
        const std::size_t colon = key.find(':');
        if (colon != std::string::npos) {
            decimals = std::atoi(key.c_str() + colon + 1);
            key.resize(colon);
        }
        out += field(p, key, decimals);
        c = end;
    }
    return out;
}

std::vector<std::string> problemLines(const std::vector<RoadProblem>& problems) {
    std::vector<std::string> out;
    out.reserve(problems.size());
    for (const RoadProblem& p : problems) out.push_back(problemLine(p));
    return out;
}

std::vector<std::pair<RoadProblemKind, uint32_t>> problemsByKind(const std::vector<RoadProblem>& problems) {
    uint32_t counts[kRoadProblemKindCount] = {};
    for (const RoadProblem& p : problems) ++counts[static_cast<std::size_t>(p.kind)];
    std::vector<std::pair<RoadProblemKind, uint32_t>> out;
    for (std::size_t k = 0; k < kRoadProblemKindCount; ++k)
        if (counts[k] > 0) out.emplace_back(static_cast<RoadProblemKind>(k), counts[k]);
    return out;
}

}  // namespace engine
