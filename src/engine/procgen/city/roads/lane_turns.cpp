#include "engine/procgen/city/roads/lane_turns.h"

#include "engine/asset_root.h"                                // assetPath: the table's file
#include "engine/bundle/bundle.h"                             // fnv1aStr: the table's hash
#include "engine/procgen/city/roads/lanes/lane_graph_build.h"   // kerbPerpSign: which side the kerb is
#include "log.h"

#include <algorithm>
#include <fstream>
#include <cstdlib>
#include <iterator>

namespace engine {

namespace {

bool turnOf(const std::string& word, Turn& out) {
    for (Turn t : {Turn::Straight, Turn::NearTurn, Turn::FarTurn, Turn::UTurn})
        if (word == turnKey(t)) {
            out = t;
            return true;
        }
    return false;
}

}  // namespace

const char* turnKey(Turn t) {
    switch (t) {
    case Turn::Straight: return "straight";
    case Turn::NearTurn: return "near_turn";
    case Turn::FarTurn: return "far_turn";
    case Turn::UTurn: return "u_turn";
    case Turn::None: break;
    }
    return "none";
}

LaneTurnRules parseLaneTurnRules(const nlohmann::json& doc, std::vector<std::string>* problems) {
    LaneTurnRules out;
    auto bad = [&](std::size_t i, const std::string& why) {
        if (problems) problems->push_back("lane_turns.json: row " + std::to_string(i) + ": " + why);
    };
    if (!doc.is_object() || !doc.contains("turns") || !doc["turns"].is_array()) {
        if (problems) problems->push_back("lane_turns.json: no \"turns\" list");
        return out;
    }
    const nlohmann::json& rows = doc["turns"];
    for (std::size_t i = 0; i < rows.size(); ++i) {
        const nlohmann::json& r = rows[i];
        LaneTurnRow row;
        if (!r.is_object() || !r.contains("turn") || !r["turn"].is_string() || !turnOf(r["turn"].get<std::string>(), row.turn)) {
            bad(i, "its \"turn\" is not one of straight, near_turn, far_turn");
            continue;
        }
        if (row.turn == Turn::UTurn) {   // turnKindsFrom never names an arm a U-turn: such a row could never apply
            bad(i, "a u_turn row could never apply: no junction offers a U-turn (turnKindsFrom)");
            continue;
        }
        const std::string when = r.value("when", std::string("any"));
        if (when == "straight_on") row.when = LaneTurnRow::When::StraightOn;
        else if (when == "no_straight_on") row.when = LaneTurnRow::When::NoStraightOn;
        else if (when != "any") {
            bad(i, "its \"when\" (" + when + ") is not straight_on, no_straight_on or any");
            continue;
        }
        if (r.contains("controls") && r["controls"].is_array())
            for (const nlohmann::json& c : r["controls"])
                if (c.is_string()) row.controls.push_back(c.get<std::string>());
        if (!r.contains("kinds") || !r["kinds"].is_array() || r["kinds"].empty()) {
            bad(i, "it names no lane \"kinds\"");
            continue;
        }
        for (const nlohmann::json& k : r["kinds"])
            if (k.is_string()) row.kinds.push_back(k.get<std::string>());
        const nlohmann::json lanes = r.value("lanes", nlohmann::json());
        if (lanes.is_string() && lanes.get<std::string>() == "every") {
            row.take = LaneTurnRow::Take::Every;
        } else if (lanes.is_object() && lanes.contains("from") && lanes["from"].is_string()) {
            const std::string from = lanes["from"].get<std::string>();
            if (from != "kerb" && from != "centre") {
                bad(i, "its lanes are counted \"from\" " + from + ", not kerb or centre");
                continue;
            }
            row.fromKerb = from == "kerb";
            if (lanes.contains("count") && lanes["count"].is_number_unsigned()) {
                row.take = LaneTurnRow::Take::Count;
                row.count = lanes["count"].get<uint32_t>();
            } else if (lanes.value("share", std::string()) == "more") {
                row.take = LaneTurnRow::Take::ShareMore;
            } else if (lanes.value("share", std::string()) == "less") {
                row.take = LaneTurnRow::Take::ShareLess;
            } else {
                bad(i, "its lanes give neither a \"count\" nor a \"share\" (more or less)");
                continue;
            }
        } else {
            bad(i, "its \"lanes\" is neither \"every\" nor {from, count | share}");
            continue;
        }
        const std::string into = r.value("into", std::string());
        if (into == "same_slot") row.into = LaneTurnRow::Into::SameSlot;
        else if (into == "same_side") row.into = LaneTurnRow::Into::SameSide;
        else {
            bad(i, "its \"into\" (" + into + ") is not same_slot or same_side");
            continue;
        }
        row.source = r.value("source", std::string());
        if (row.source.empty()) {
            bad(i, "it gives no \"source\"");
            continue;
        }
        out.rows.push_back(std::move(row));
    }
    return out;
}

const LaneTurnRules& laneTurnRules() {
    static const LaneTurnRules rules = [] {
        const std::string path = assetPath("assets/data/lane_turns.json");
        std::ifstream in(path, std::ios::binary);
        if (!in) {
            LOG_WARN << "[roads] " << path << " is missing: no junction offers a turn";
            return LaneTurnRules();
        }
        const std::string text((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
        nlohmann::json doc;
        try {
            doc = nlohmann::json::parse(text);
        } catch (const std::exception& e) {
            LOG_WARN << "[roads] " << path << ": " << e.what() << ": no junction offers a turn";
            return LaneTurnRules();
        }
        std::vector<std::string> problems;
        LaneTurnRules r = parseLaneTurnRules(doc, &problems);
        for (const std::string& p : problems) LOG_WARN << "[roads] " << p;
        r.hash = laneTurnTableHash(text);
        return r;
    }();
    return rules;
}

uint64_t laneTurnTableHash(const std::string& text) { return bundle::fnv1aStr(text); }

bool kerbSideCounterClockwise() {
    // Arriving traffic on an arm whose heading (the way it leaves the junction) is h runs the other way, -dir(h); its
    // kerb is perp(-dir(h)) * kerbPerpSign = -perp(dir(h)) * kerbPerpSign. perp turns a quarter counter-clockwise, so the
    // kerb lies a quarter turn counter-clockwise from the arm exactly when kerbPerpSign is negative.
    return roads::lanes::kerbPerpSign() < 0;
}

std::vector<Turn> turnKindsFrom(const std::vector<TurnArm>& arms, const std::vector<std::array<uint32_t, 2>>& through,
                                uint32_t from, bool kerbCcw, std::vector<uint32_t>* halfway) {
    std::vector<Turn> out(arms.size(), Turn::None);
    if (from >= arms.size() || !arms[from].hasHeading) return out;
    // the arms with a stated heading, counter-clockwise round the junction (by heading; equal headings by index)
    std::vector<uint32_t> ring;
    for (uint32_t a = 0; a < arms.size(); ++a)
        if (arms[a].hasHeading) ring.push_back(a);
    std::stable_sort(ring.begin(), ring.end(), [&](uint32_t a, uint32_t b) { return arms[a].heading < arms[b].heading; });
    const std::size_t n = ring.size();
    const std::size_t at = static_cast<std::size_t>(std::find(ring.begin(), ring.end(), from) - ring.begin());
    auto stepsTo = [&](uint32_t arm) {   // counter-clockwise steps round the ring from `from` to `arm`
        const std::size_t k = static_cast<std::size_t>(std::find(ring.begin(), ring.end(), arm) - ring.begin());
        return (k + n - at) % n;
    };
    int partner = -1;
    for (const std::array<uint32_t, 2>& t : through) {
        if (t[0] == from && t[1] < arms.size() && arms[t[1]].hasHeading) partner = static_cast<int>(t[1]);
        if (t[1] == from && t[0] < arms.size() && arms[t[0]].hasHeading) partner = static_cast<int>(t[0]);
    }
    const std::size_t partnerSteps = partner >= 0 ? stepsTo(static_cast<uint32_t>(partner)) : 0;
    for (uint32_t b : ring) {
        if (b == from) continue;
        if (static_cast<int>(b) == partner) {
            out[b] = Turn::Straight;
            continue;
        }
        const std::size_t p = stepsTo(b);
        bool ccwSide = false;
        if (partner >= 0) {
            ccwSide = p < partnerSteps;
        } else if (2 * p == n) {   // halfway round with no partner stated: the side is not stated
            out[b] = Turn::FarTurn;
            if (halfway) halfway->push_back(b);
            continue;
        } else {
            ccwSide = 2 * p < n;
        }
        out[b] = ccwSide == kerbCcw ? Turn::NearTurn : Turn::FarTurn;
    }
    return out;
}

std::vector<StatedTurn> turnsAtJunction(const std::vector<TurnArm>& arms, const std::vector<std::array<uint32_t, 2>>& through,
                                        const std::string& control, const LaneTurnRules& rules, std::vector<std::string>* notes,
                                        bool kerbCcw) {
    std::vector<StatedTurn> out;
    auto note = [&](const std::string& s) {
        if (notes) notes->push_back(s);
    };
    for (uint32_t a = 0; a < arms.size(); ++a)
        if (!arms[a].hasHeading && (!arms[a].arriving.empty() || !arms[a].leaving.empty()))
            note("arm " + std::to_string(a) + " states no heading: no turn into it or out of it");
    auto bySlot = [](std::vector<TurnArmLane> v) {
        std::stable_sort(v.begin(), v.end(), [](const TurnArmLane& x, const TurnArmLane& y) { return x.slot < y.slot; });
        return v;
    };
    for (uint32_t a = 0; a < arms.size(); ++a) {
        if (arms[a].arriving.empty() || !arms[a].hasHeading) continue;
        std::vector<uint32_t> halfway;
        const std::vector<Turn> kinds = turnKindsFrom(arms, through, a, kerbCcw, &halfway);
        for (uint32_t h : halfway)
            note("arm " + std::to_string(h) + " stands halfway round from arm " + std::to_string(a) +
                 " with no through pair stated: taken as a far turn");
        bool offers[kTurnCount] = {};
        for (uint32_t b = 0; b < arms.size(); ++b)
            if (kinds[b] != Turn::None && !arms[b].leaving.empty()) offers[static_cast<std::size_t>(kinds[b])] = true;
        const bool straightOn = offers[static_cast<std::size_t>(Turn::Straight)];
        const std::size_t before = out.size();
        for (uint32_t ri = 0; ri < rules.rows.size(); ++ri) {
            const LaneTurnRow& row = rules.rows[ri];
            if (row.when == LaneTurnRow::When::StraightOn && !straightOn) continue;
            if (row.when == LaneTurnRow::When::NoStraightOn && straightOn) continue;
            if (!row.controls.empty() && std::find(row.controls.begin(), row.controls.end(), control) == row.controls.end()) continue;
            if (!offers[static_cast<std::size_t>(row.turn)]) continue;
            // the arriving lanes of the row's kinds, centre-line side first
            std::vector<TurnArmLane> lanes;
            for (const TurnArmLane& l : bySlot(arms[a].arriving))
                if (std::find(row.kinds.begin(), row.kinds.end(), l.kind) != row.kinds.end()) lanes.push_back(l);
            if (lanes.empty()) continue;
            // which of them take it, in order from the row's side
            std::vector<TurnArmLane> take;
            std::size_t want = lanes.size();
            if (row.take == LaneTurnRow::Take::Count) want = std::min<std::size_t>(row.count, lanes.size());
            if (row.take == LaneTurnRow::Take::ShareMore || row.take == LaneTurnRow::Take::ShareLess) {
                const Turn other = row.turn == Turn::NearTurn ? Turn::FarTurn : row.turn == Turn::FarTurn ? Turn::NearTurn : row.turn;
                const bool shared = other != row.turn && offers[static_cast<std::size_t>(other)] && lanes.size() > 1;
                if (shared) want = row.take == LaneTurnRow::Take::ShareLess ? lanes.size() / 2 : lanes.size() - lanes.size() / 2;
            }
            for (std::size_t k = 0; k < want; ++k) take.push_back(row.fromKerb ? lanes[lanes.size() - 1 - k] : lanes[k]);
            for (uint32_t b = 0; b < arms.size(); ++b) {
                if (kinds[b] != row.turn || arms[b].leaving.empty()) continue;
                const std::vector<TurnArmLane> into = bySlot(arms[b].leaving);
                for (std::size_t k = 0; k < take.size(); ++k) {
                    std::size_t j = 0;
                    if (row.into == LaneTurnRow::Into::SameSlot) {
                        for (std::size_t c = 1; c < into.size(); ++c)
                            if (std::abs(into[c].slot - take[k].slot) < std::abs(into[j].slot - take[k].slot)) j = c;
                    } else {
                        const std::size_t kk = std::min(k, into.size() - 1);
                        j = row.fromKerb ? into.size() - 1 - kk : kk;
                    }
                    StatedTurn t;
                    t.fromArm = a;
                    t.toArm = b;
                    t.from = take[k].lane;
                    t.to = into[j].lane;
                    t.turn = row.turn;
                    t.row = ri;
                    out.push_back(t);
                }
            }
        }
        // every arriving lane that takes no turn here, and why: said, never dropped
        bool offersAny = false;
        for (bool o : offers) offersAny = offersAny || o;
        for (const TurnArmLane& l : arms[a].arriving) {
            bool takes = false;
            for (std::size_t k = before; k < out.size() && !takes; ++k) takes = out[k].from == l.lane;
            if (takes) continue;
            const std::string name = l.name.empty() ? "#" + std::to_string(l.lane) : l.name;
            note("arm " + std::to_string(a) + ": arriving lane " + name + " (kind " + l.kind + ", slot " + std::to_string(l.slot) +
                 ") takes no turn: " + (offersAny ? "no row of lane_turns.json gives a " + l.kind + " lane a turn here"
                                                  : std::string("no other arm of the junction has a lane leaving by it")));
        }
    }
    std::sort(out.begin(), out.end(), [](const StatedTurn& x, const StatedTurn& y) {
        if (x.fromArm != y.fromArm) return x.fromArm < y.fromArm;
        if (x.toArm != y.toArm) return x.toArm < y.toArm;
        if (x.from != y.from) return x.from < y.from;
        return x.to < y.to;
    });
    out.erase(std::unique(out.begin(), out.end(), [](const StatedTurn& x, const StatedTurn& y) {
                  return x.fromArm == y.fromArm && x.toArm == y.toArm && x.from == y.from && x.to == y.to;
              }), out.end());
    return out;
}

}  // namespace engine
