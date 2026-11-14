#include "engine/procgen/city/roads/control_signs.h"

#include "engine/ai/nav_graph.h"                       // rightOf: the kerb side of traffic going a way
#include "engine/procgen/city/roads/lanes/deck_mesh.h"  // kStopLineBack: where the paint's stop lines stand

#include <algorithm>
#include <cmath>
#include <cstdio>

namespace engine {

namespace {

// Road ends this close together are one node of a junction (the planner's ends meet at one point; the layout's end
// tolerance is well under it): the roads whose widths set an arm's stop line (ADR-0419) are the ones at its node.
constexpr Real kSameNode = 2.0;

const char* const kFaces[] = {"arriving", "entering", "crossing"};
const char* const kSides[] = {"kerb", "near_kerb_or_far_off"};
const char* const kOneWayEnds[] = {"exit", "any"};

template <std::size_t N>
bool oneOf(const std::string& w, const char* const (&words)[N]) {
    for (const char* x : words)
        if (w == x) return true;
    return false;
}

std::string text(const nlohmann::json& j) {
    if (j.is_string()) return j.get<std::string>();
    std::string out;
    if (j.is_array())
        for (const nlohmann::json& s : j)
            if (s.is_string()) out += (out.empty() ? "" : " ") + s.get<std::string>();
    return out;
}

Vec2 headingDir(Real degrees) { return Vec2(std::cos(degrees * DEG_TO_RAD), std::sin(degrees * DEG_TO_RAD)); }

}  // namespace

std::vector<ControlSignRule> controlSignRules(const AttachmentKinds& kinds, std::vector<std::string>* problems) {
    std::vector<ControlSignRule> out;
    auto fail = [&](const std::string& kind, const std::string& why) {
        if (problems) problems->push_back("road_attachments.json kinds." + kind + ".sign: " + why);
    };
    for (const AttachmentKind& k : kinds.kinds) {
        if (k.sign.is_null() || !k.sign.is_object()) continue;
        const nlohmann::json& s = k.sign;
        ControlSignRule r;
        r.kind = k.name;
        if (k.on != "arm") { fail(k.name, "a sign that follows from a fact attaches to an arm (\"on\": \"arm\")"); continue; }
        r.means = s.value("means", std::string());
        if (s.contains("when") && s["when"].is_object()) {
            const nlohmann::json& w = s["when"];
            r.regulation = w.value("regulation", std::string());
            r.oneWayEnd = w.value("one_way_end", std::string());
            if (w.contains("controls") && w["controls"].is_array())
                for (const nlohmann::json& c : w["controls"])
                    if (c.is_string()) r.controls.push_back(c.get<std::string>());
        }
        if (r.regulation.empty() && r.oneWayEnd.empty()) { fail(k.name, "its \"when\" asks neither \"regulation\" nor \"one_way_end\""); continue; }
        if (!r.oneWayEnd.empty() && !oneOf(r.oneWayEnd, kOneWayEnds)) { fail(k.name, "\"one_way_end\" is \"" + r.oneWayEnd + "\", expected exit or any"); continue; }
        r.faces = s.value("faces", std::string());
        if (!oneOf(r.faces, kFaces)) { fail(k.name, "\"faces\" is \"" + r.faces + "\", expected arriving, entering or crossing"); continue; }
        r.side = s.value("side", std::string());
        if (!oneOf(r.side, kSides)) { fail(k.name, "\"side\" is \"" + r.side + "\", expected kerb or near_kerb_or_far_off"); continue; }
        r.elseSide = s.value("else_side", std::string());
        if (!r.elseSide.empty() && (r.elseSide != "off" || r.side != "kerb")) {
            fail(k.name, "\"else_side\" is \"" + r.elseSide + "\"; a kerb-side sign may name \"off\", nothing else");
            continue;
        }
        if ((r.faces == "crossing") != (r.side == "near_kerb_or_far_off")) {
            fail(k.name, "a sign facing crossing traffic stands on the near kerb side or far off side corner, and only it");
            continue;
        }
        r.fromStopLine = s.value("from_stop_line", 0.0);
        r.fromKerb = s.value("from_kerb", 0.0);
        r.keepsClear = s.value("keeps_clear", 0.0);
        r.crossingAngle = s.value("crossing_angle", 30.0);
        r.priority = s.value("priority", 0);
        r.look = s.value("look", nlohmann::json());
        if (!r.look.is_object() || !r.look.contains("sign")) { fail(k.name, "its \"look\" names no sign of the sign kit"); continue; }
        if (s.contains("look_at") && s["look_at"].is_object())
            for (const auto& [control, look] : s["look_at"].items())
                if (look.is_object() && look.contains("sign")) r.lookAt[control] = look;
        r.source = text(s.value("source", nlohmann::json()));
        out.push_back(std::move(r));
    }
    std::stable_sort(out.begin(), out.end(), [](const ControlSignRule& a, const ControlSignRule& b) {
        return a.priority != b.priority ? a.priority < b.priority : a.kind < b.kind;
    });
    return out;
}

Real kerbOffset(const ModelStreet& s) { return s.travelWidth() * 0.5 + s.section.shoulder; }

Real stopLineFromMouth(const RoadModel& m, const ModelJunction& J, std::size_t arm) {
    if (arm >= J.arms.size()) return kNotStated;
    const ModelArm& a = J.arms[arm];
    Vec2 mouth;
    if (!armMouth(m, J, arm, mouth)) return kNotStated;
    if (stated(a.stopLine)) return std::fabs(a.stopLine - a.station);   // the paint's (the built layer)
    Real atNode = -1, anywhere = 0;
    for (std::size_t o = 0; o < J.arms.size(); ++o) {
        if (o == arm || !J.arms[o].street.valid() || J.arms[o].street.index >= m.streets.size()) continue;
        const Real hw = m.streets[J.arms[o].street.index].travelWidth() * 0.5;
        anywhere = std::max(anywhere, hw);
        Vec2 other;
        if (armMouth(m, J, o, other) && (other - mouth).length() <= kSameNode) atNode = std::max(atNode, hw);
    }
    return (atNode >= 0 ? atNode : anywhere) + roads::lanes::kStopLineBack;
}

ControlSignPlan placeControlSigns(const RoadModel& m, const std::vector<ControlSignRule>& rules,
                                  const std::function<Real(const Vec2&)>& carriagewayDepth, std::vector<Vec2> standing) {
    ControlSignPlan plan;
    char buf[256];
    // rows by priority over the whole model, so a sign of a lower priority never takes the room of a higher one's
    for (const ControlSignRule& r : rules) {
        for (std::size_t j = 0; j < m.junctions.size(); ++j) {
            const ModelJunction& J = m.junctions[j];
            if (!r.controls.empty() && std::find(r.controls.begin(), r.controls.end(), J.control) == r.controls.end()) continue;
            for (std::size_t k = 0; k < J.arms.size(); ++k) {
                const ModelArm& a = J.arms[k];
                if (!a.street.valid() || a.street.index >= m.streets.size() || !stated(a.heading)) continue;
                const ModelStreet& S = m.streets[a.street.index];
                // WHEN: over the stated facts
                if (!r.regulation.empty() && std::find(a.regulations.begin(), a.regulations.end(), r.regulation) == a.regulations.end())
                    continue;
                if (!r.oneWayEnd.empty()) {
                    if (!S.oneWay) continue;
                    // a one-way street's traffic runs along its road's spine: it leaves the street here where the spine
                    // runs into the junction
                    if (r.oneWayEnd == "exit" && !a.spineIn) continue;
                }
                Vec2 mouth;
                const Real stop = stopLineFromMouth(m, J, k);
                if (!armMouth(m, J, k, mouth) || !stated(stop)) continue;
                const Vec2 d = headingDir(a.heading);   // out along the arm, away from the junction
                const Vec2 along = mouth + d * (stop - r.fromStopLine);
                const Real out = kerbOffset(S) + r.fromKerb;
                nlohmann::json look = r.look;
                {
                    const std::map<std::string, nlohmann::json>::const_iterator la = r.lookAt.find(J.control);
                    if (la != r.lookAt.end()) look = la->second;
                }
                // WHERE, and facing whom: one spot per reader
                struct Spot {
                    Vec2 at, face;
                    int reader = -1;
                    bool hasElse = false;
                    Vec2 elseAt;   // the row's `else_side`: across the arm
                };
                std::vector<Spot> spots;
                const bool elseOff = r.elseSide == "off";
                if (r.faces == "arriving") {
                    // traffic arriving runs in along -d; the face points out along d, toward it; its kerb side
                    spots.push_back({along + rightOf(d * -1.0) * out, d, static_cast<int>(k), elseOff, along + rightOf(d) * out});
                } else if (r.faces == "entering") {
                    // traffic that would enter runs out along d; the face points back toward the junction
                    spots.push_back({along + rightOf(d) * out, d * -1.0, -1, elseOff, along + rightOf(d * -1.0) * out});
                } else {
                    // each other arm with traffic arriving that crosses this one (not along its line, either way)
                    const Real crossCos = std::cos(r.crossingAngle * DEG_TO_RAD);
                    for (std::size_t p = 0; p < J.arms.size(); ++p) {
                        const ModelArm& P = J.arms[p];
                        if (p == k || !P.street.valid() || P.street.index >= m.streets.size() || !stated(P.heading)) continue;
                        const ModelStreet& PS = m.streets[P.street.index];
                        const bool arrives = PS.oneWay ? P.spineIn : PS.section.lanesForward + PS.section.lanesBackward > 0;
                        if (!arrives) continue;
                        const Vec2 dp = headingDir(P.heading);
                        if (std::fabs(dot(dp, d)) >= crossCos) continue;
                        // this arm on the reader's kerb side: the near corner (toward the reader's arm); on its off side:
                        // the far corner (away from it)
                        const bool onKerbSide = dot(d, rightOf(dp * -1.0)) > 0;
                        const Vec2 toward = dot(rightOf(d), dp) > 0 ? rightOf(d) : rightOf(d) * -1.0;
                        // PARALLEL TO THE ONE-WAY STREET (2B.40 para 02): its face square to the street, toward the reader's
                        // side -- not along the reader's arm, which crosses at a slant where the junction is skewed
                        spots.push_back({along + (onKerbSide ? toward : toward * -1.0) * out, toward, static_cast<int>(p), false, Vec2()});
                    }
                }
                for (const Spot& sp : spots) {
                    nlohmann::json lk = look;
                    if (lk.value("arrow", std::string()) == "traffic") {
                        // the street's traffic, as its reader sees it: the reader's right is rightOf(face) (they look
                        // along -face, and rightOf of a heading is its kerb side, the left)
                        const Vec2 traffic = a.spineIn ? d * -1.0 : d;
                        lk["arrow"] = dot(traffic, rightOf(sp.face)) > 0 ? "right" : "left";
                    }
                    // BACK TO BACK: a sign of this kind at this arm already standing within keeps_clear and facing the
                    // other way is the same post (two crossing approaches whose corners are one: MUTCD 2B.40's near
                    // corner for one is the far corner for the other); this one is mounted on its back
                    // ...and one already there facing the SAME way already serves this reader (two arms of one road
                    // reaching a junction the plan spread over two nodes read the same corner): nothing more to stand
                    Vec2 at = sp.at;
                    bool shared = false, served = false;
                    for (const PlacedSign& o : plan.signs) {
                        if (o.kind != r.kind || o.junction != static_cast<int>(j) || o.arm != static_cast<int>(k) ||
                            (o.at - sp.at).length() >= r.keepsClear)
                            continue;
                        if (dot(o.face, sp.face) > 0.9) { served = true; break; }
                        if (dot(o.face, sp.face) < -0.9 && !shared) {
                            at = o.at;
                            shared = true;
                        }
                    }
                    if (served) continue;
                    // the spot, else (a row with `else_side`) the same spot across the arm
                    const char* why = nullptr;
                    if (!shared) {
                        std::vector<Vec2> tries{sp.at};
                        if (sp.hasElse) tries.push_back(sp.elseAt);
                        for (const Vec2& t : tries) {
                            why = nullptr;
                            if (carriagewayDepth && carriagewayDepth(t) > 0) why = "on the carriageway";
                            if (!why)
                                for (const Vec2& s : standing)
                                    if ((s - t).length() < r.keepsClear) { why = "no room (something stands within keeps_clear)"; break; }
                            if (!why) { at = t; break; }
                        }
                    }
                    if (why) {
                        ++plan.refused[r.kind];
                        std::snprintf(buf, sizeof buf, "%s at junction %s arm %zu (%s) at (%.1f, %.1f): %s", r.kind.c_str(), J.id.c_str(), k,
                                      a.road.c_str(), sp.at.x, sp.at.y, why);
                        plan.notes.push_back(buf);
                        continue;
                    }
                    PlacedSign ps;
                    ps.kind = r.kind;
                    ps.look = lk;
                    ps.junction = static_cast<int>(j);
                    ps.arm = static_cast<int>(k);
                    ps.at = at;
                    ps.face = sp.face;
                    ps.reader = sp.reader;
                    ps.keepsClear = r.keepsClear;
                    plan.signs.push_back(ps);
                    if (!shared) standing.push_back(at);
                    ++plan.placed[r.kind];
                }
            }
        }
    }
    return plan;
}

}  // namespace engine
