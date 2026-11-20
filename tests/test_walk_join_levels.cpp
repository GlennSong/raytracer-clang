// WHERE A PLACE JOINS THE WALKS: THE STREET MAP'S LOOK-UPS SET BESIDE THE ROAD MODEL'S ONE ANSWER (ADR-0468, issue #300;
// #306 task 4, piece 5). A comparison, not a switch-over: nobody walks differently, and nothing is stepped.
//
// Placing a place, starting a walk from it and ending a walk at it each find the walking network their own way today:
//   - placing (the crowd's CrowdUsable, city_render.cpp): the nearest link people walk within kBoardReach, both its ends
//     and the place's nearest node in the main part of the street map on foot;
//   - starting (CitySim::sendAgentTo): the nearest link, on the place's pavement (kerbLinkNear), and the node a walk
//     leaves from (walkStartNode: the nearest, or the first of the nearest few a walkable link leaves);
//   - finishing (sendAgentTo's destination): the same nearest link, and its stop beside the place when the place is
//     within kErrandOffLink of it (else the walk ends at a node).
// The road model's one answer (RoadModel::walkJoinOn, from the place's stated street, station and kerb) is asked for
// each place, and each look-up is counted as agreeing (the same street and kerb) or not, with why and how far apart.
// A lot's door states no street, so on river_town the old sim's places (each a building's door, snapped to the street
// map by PlaceMap::add) are set beside the fallback by position (RoadModel::walkJoinNear).
//
//   RT_TEST_FILTER=level_walk_join ./build-viewer/level_tests
//
// The scenes: brain_crowd (the traffic lab's roads: its 8 stated markers, which traffic_lab and brain_lab read too, and
// the crowd's places) and river_town (its crowd's places and its buildings' doors).

#include "test_framework.h"

#include "crowd_test_rig.h"
#include "../src/engine/ai/pathfind.h"
#include "../src/engine/procgen/city/polyline.h"
#include "../src/engine/procgen/city/roads/road_model.h"

#include <algorithm>
#include <cctype>
#include <cstdio>
#include <map>
#include <set>
#include <string>
#include <vector>

using namespace engine;

namespace {

// WHAT A LOOK-UP OF THE STREET MAP FOUND: a link, the street and kerb it runs beside, and where a walker on it stands
// level with the place (its pavement, a walker's first file in: verge 1 m).
struct OldAnswer {
    int link = -1;
    int street = -1;
    int kerb = 0;
    Vec2 point;
};
OldAnswer oldAnswer(const NavGraph& nav, int link, const Vec2& p) {
    OldAnswer o;
    o.link = link;
    if (link < 0 || link >= nav.linkCount()) return o;
    const NavLink& L = nav.links[static_cast<std::size_t>(link)];
    o.street = L.street;
    // (a link running along its road's spine carries the traffic whose kerb is +1; a walker walks it on that kerb)
    o.kerb = L.stationTo >= L.stationFrom ? 1 : -1;
    const Vec2 A = nav.nodes[static_cast<std::size_t>(L.from)], B = nav.nodes[static_cast<std::size_t>(L.to)];
    const Vec2 ab = B - A;
    const Real len2 = ab.lengthSquared();
    const Real t = len2 > 0 ? std::clamp(dot(p - A, ab) / len2, Real(0), Real(1)) : Real(0);
    o.point = L.footpath ? A + ab * t : nav.sidewalkPoint(link, t, 1.0);
    return o;
}
bool shareJunction(const RoadModel& m, int a, int b) {
    if (a < 0 || b < 0 || a >= static_cast<int>(m.streets.size()) || b >= static_cast<int>(m.streets.size())) return false;
    for (const ModelJunctionId& x : m.streets[static_cast<std::size_t>(a)].junctions)
        for (const ModelJunctionId& y : m.streets[static_cast<std::size_t>(b)].junctions)
            if (x.valid() && x == y) return true;
    return false;
}
// WHY A LOOK-UP AND THE ONE ANSWER DISAGREE (empty: they agree: the same street, and the same kerb where the look-up
// takes one: placing asks only whether a link people walk is near, either of a street's pair)
std::string causeOf(const RoadModel& m, const NavGraph& nav, const OldAnswer& o, const WalkJoin& j, bool kerbTaken = true) {
    if (!j.valid()) return std::string("the model joins none: ") + walkJoinWhyWord(j.why);
    if (o.link < 0) return "the street map finds no link people walk within 25 m";
    const NavLink& L = nav.links[static_cast<std::size_t>(o.link)];
    if (L.footpath) return "the street map's nearest is a park's or the quad's footpath";
    if (!L.walkable) return "the street map's nearest is a ramp's or freeway's link, which nobody walks";
    if (L.street < 0) return "the street map's nearest is a link inside a junction";
    if (L.street != static_cast<int>(j.street.index))
        return shareJunction(m, L.street, static_cast<int>(j.street.index)) ? "the street map's nearest is the cross street at a corner"
                                                                             : "the street map's nearest is another street";
    if (kerbTaken && o.kerb != j.dir) {
        if (j.across) return "the model joins across the road (its kerb has no walk)";
        // (a one-way street is one link, with no twin for the other pavement: its walkers keep to its traffic's kerb)
        return L.oneWay ? "a one-way street: the street map walks only its traffic's kerb" : "the street map's is the other kerb";
    }
    return std::string();
}
// One use's tally over the places.
struct Tally {
    int places = 0, agree = 0;
    std::vector<double> apart;                 // metres between the two answers, where they agree
    std::map<std::string, int> causes;
    std::vector<std::string> listed;           // the disagreements (and far-apart agreements), each named
    void add(const std::string& name, const std::string& cause, double metres) {
        ++places;
        if (cause.empty()) {
            ++agree;
            apart.push_back(metres);
            if (metres > 5.0) listed.push_back(name + ": agree, but " + std::to_string(static_cast<int>(metres + 0.5)) + " m apart");
            return;
        }
        ++causes[cause];
        listed.push_back(name + ": " + cause);
    }
    void print(const char* level, const char* use) const {
        std::vector<double> a = apart;
        std::sort(a.begin(), a.end());
        const auto q = [&](double f) { return a.empty() ? 0.0 : a[static_cast<std::size_t>(f * static_cast<double>(a.size() - 1))]; };
        std::printf("    [walk join] %s, %s: %d places, %d agree (%.1f%%); apart where they agree: median %.2f m, 95th %.2f m, worst %.2f m\n",
                    level, use, places, agree, places ? 100.0 * agree / places : 0.0, q(0.5), q(0.95), q(1.0));
        for (const std::pair<const std::string, int>& c : causes) std::printf("        %4d  %s\n", c.second, c.first.c_str());
    }
    void printListed(std::size_t most) const {
        for (std::size_t i = 0; i < listed.size() && i < most; ++i) std::printf("          - %s\n", listed[i].c_str());
        if (listed.size() > most) std::printf("          ... and %zu more\n", listed.size() - most);
    }
};
// A crowd's place is named <kind>_<n>; a level's own markers are not.
bool crowdPlace(const std::string& name) {
    const std::size_t u = name.rfind('_');
    return u != std::string::npos && u + 1 < name.size() && std::isdigit(static_cast<unsigned char>(name[u + 1]));
}
// The counts pinned per level, from the run looked at (2026-11-19, ADR-0468).
struct Pinned {
    int places, cutOff, placing, starting, finishing, fallbackSame, rampNodes, atNode, doors, doorsJoined, doorsAgree;
};
const Pinned& pinnedFor(const std::string& level) {
    static const Pinned kBrainCrowd{451, 4, 445, 405, 405, 445, 1, 0, 0, 0, 0};
    static const Pinned kRiverTown{799, 98, 751, 749, 746, 764, 0, 3, 913, 902, 746};
    return level == "brain_crowd.json" ? kBrainCrowd : kRiverTown;
}
}  // namespace

TEST_CASE(level_walk_join_the_street_maps_lookups_beside_the_one_query) {
    for (const char* level : {"brain_crowd.json", "river_town.json"}) {
        crowdtest::LoadedCrowd lc(level);
        CHECK(lc.ok);
        if (!lc.ok) continue;
        citysim::CitySim& sim = lc.city.simMutable();
        const citysim::CityBrain* b = sim.brain();
        CHECK(b != nullptr && sim.drivingV2() && sim.drivingV2()->model());
        if (!b || !sim.drivingV2() || !sim.drivingV2()->model()) continue;
        const RoadModel& m = *sim.drivingV2()->model();
        const NavGraph& nav = sim.nav();
        const std::vector<uint8_t> open = m.openTo("walker").walks;
        const ReachableParts parts = reachableWalks(m.laneLevel, open);
        // the street map's main part on foot, as the crowd's CrowdUsable takes it
        const std::vector<int> part = stronglyConnected(nav, true);
        std::vector<int> partSize;
        for (const int p : part)
            if (p >= 0) {
                if (p >= static_cast<int>(partSize.size())) partSize.resize(static_cast<std::size_t>(p) + 1, 0);
                ++partSize[static_cast<std::size_t>(p)];
            }
        const int mainPart = partSize.empty() ? -1 : static_cast<int>(std::max_element(partSize.begin(), partSize.end()) - partSize.begin());

        // ---- THE BRAIN'S PLACES (markers): stated street, station and kerb ----------------------------------------------
        Tally placing, starting, finishing, ownPlacing, ownStarting;
        int joined = 0, held = 0, across = 0, cutOff = 0, statedCount = 0, fallbackSame = 0, rampNodes = 0, atNode = 0;
        int usableOld = 0, usableBoth = 0, fallbackCorner = 0, fallbackOtherKerb = 0, fallbackOther = 0;
        std::vector<double> fallbackApart, startNodeApart;
        std::vector<std::string> fallbackDiffer, notes;
        struct CutOffPart {
            int places = 0;
            std::set<std::string> streets;
        };
        std::map<uint32_t, CutOffPart> cutOffParts;
        for (const citysim::BrainMarker& mk : b->markers()) {
            const WalkJoin j = m.walkJoinOn(mk.frontage, open, &parts);
            statedCount += mk.stated ? 1 : 0;
            joined += j.valid() ? 1 : 0;
            if (j.valid() && j.held > 1.0) {
                ++held;
                notes.push_back(mk.name + ": past where its kerb's sidewalk stops, joined " + std::to_string(static_cast<int>(j.held + 0.5)) +
                                " m along at its end");
            }
            across += j.valid() && j.across ? 1 : 0;
            if (j.valid() && j.cutOff) {
                ++cutOff;
                CutOffPart& c = cutOffParts[j.part];
                ++c.places;
                c.streets.insert(m.streets[j.street.index].id);
            }
            // ...and what the fallback by position would answer, were nothing stated (a lot's door today)
            const WalkJoin fb = m.walkJoinNear(mk.stand, open, &parts);
            if (j.valid() && fb.valid() && fb.at.walk == j.at.walk) {
                ++fallbackSame;
                fallbackApart.push_back(distance(fb.point, j.point));
            } else {
                const bool sameKerbStreet = j.valid() && fb.valid() && fb.street == j.street;
                const bool corner = j.valid() && fb.valid() && shareJunction(m, static_cast<int>(j.street.index), static_cast<int>(fb.street.index));
                ++(sameKerbStreet ? fallbackOtherKerb : corner ? fallbackCorner : fallbackOther);
                fallbackDiffer.push_back(mk.name + ": stated " + (j.valid() ? m.streets[j.street.index].id : std::string("none")) +
                                         ", by position " + (fb.valid() ? m.streets[fb.street.index].id : std::string("none")));
            }
            // PLACING: the crowd's look-up
            const int placeLink = lc.city.nearestWalkableLink(mk.stand, citysim::kBoardReach);
            // (its answer is a link of either kerb: measured from the pavement of the pair the place is on)
            OldAnswer po = oldAnswer(nav, placeLink, mk.stand);
            if (placeLink >= 0) po.point = oldAnswer(nav, sim.kerbSideLink(placeLink, mk.stand), mk.stand).point;
            bool usable = placeLink >= 0;
            if (usable) {
                const NavLink& L = nav.links[static_cast<std::size_t>(placeLink)];
                const int node = nav.nearestNode(mk.stand);
                usable = part[static_cast<std::size_t>(L.from)] == mainPart && part[static_cast<std::size_t>(L.to)] == mainPart && node >= 0 &&
                         part[static_cast<std::size_t>(node)] == mainPart;
            }
            usableOld += usable ? 1 : 0;
            usableBoth += usable && j.valid() && !j.cutOff ? 1 : 0;
            const std::string pc = causeOf(m, nav, po, j, false);
            placing.add(mk.name, pc, distance(po.point, j.point));
            // STARTING and FINISHING: sendAgentTo's look-ups
            const OldAnswer so = oldAnswer(nav, sim.kerbLinkNear(mk.stand), mk.stand);
            const std::string sc = causeOf(m, nav, so, j);
            starting.add(mk.name, sc, distance(so.point, j.point));
            const int startNode = sim.walkStartNode(mk.stand), nearestNode = nav.nearestNode(mk.stand);
            if (startNode != nearestNode) {
                ++rampNodes;
                notes.push_back(mk.name + ": its nearest node has no walkable link (a ramp's); a walk leaves from the next");
            }
            if (startNode >= 0 && j.valid()) startNodeApart.push_back(distance(nav.nodes[static_cast<std::size_t>(startNode)], j.point));
            std::string fc = sc;
            if (fc.empty() && so.link >= 0) {
                const NavLink& L = nav.links[static_cast<std::size_t>(so.link)];
                const Real off = distToSegment(mk.stand, nav.nodes[static_cast<std::size_t>(L.from)], nav.nodes[static_cast<std::size_t>(L.to)]);
                if (off > citysim::kErrandOffLink) {
                    fc = "the walk ends at a node, not beside the place (it lies over 15 m off its link)";
                    ++atNode;
                }
            }
            finishing.add(mk.name, fc, distance(so.point, j.point));
            if (!crowdPlace(mk.name)) {
                ownPlacing.add(mk.name, pc, distance(po.point, j.point));
                ownStarting.add(mk.name, sc, distance(so.point, j.point));
            }
        }
        std::printf("    [walk join] %s: %zu places (%d stated), %d joined by the model; held past a sidewalk's end %d, across the road %d, "
                    "cut off %d; the street map's placing takes %d (%d of them the model joins in its largest part)\n",
                    level, b->markers().size(), statedCount, joined, held, across, cutOff, usableOld, usableBoth);
        placing.print(level, "placing a place");
        placing.printListed(12);
        starting.print(level, "starting a walk");
        starting.printListed(12);
        std::printf("        a walk's start node is not the nearest (the nearest a ramp's): %d places\n", rampNodes);
        finishing.print(level, "finishing a walk");
        finishing.printListed(12);
        if (ownPlacing.places > 0) {
            ownPlacing.print(level, "the level's own markers (traffic_lab, brain_lab), placing");
            ownPlacing.printListed(8);
            ownStarting.print(level, "the level's own markers (traffic_lab, brain_lab), starting and finishing");
            ownStarting.printListed(8);
        }
        {
            std::sort(startNodeApart.begin(), startNodeApart.end());
            std::sort(fallbackApart.begin(), fallbackApart.end());
            const auto q = [](const std::vector<double>& a, double f) {
                return a.empty() ? 0.0 : a[static_cast<std::size_t>(f * static_cast<double>(a.size() - 1))];
            };
            std::printf("        a walk's start node from the model's point: median %.1f m, 95th %.1f m, worst %.1f m\n", q(startNodeApart, 0.5),
                        q(startNodeApart, 0.95), q(startNodeApart, 1.0));
            std::printf("    [walk join] %s: the fallback by position from where a person stands gives the stated walk for %d of %zu "
                        "(apart: median %.2f m, worst %.2f m)\n",
                        level, fallbackSame, b->markers().size(), q(fallbackApart, 0.5), q(fallbackApart, 1.0));
            std::printf("        the others: the cross street at a corner %d, the same street's other walk %d, another street %d\n", fallbackCorner,
                        fallbackOtherKerb, fallbackOther);
            for (std::size_t i = 0; i < fallbackDiffer.size() && i < 8; ++i) std::printf("          - %s\n", fallbackDiffer[i].c_str());
            for (std::size_t i = 0; i < notes.size() && i < 16; ++i) std::printf("          * %s\n", notes[i].c_str());
            if (notes.size() > 16) std::printf("          * ... and %zu more\n", notes.size() - 16);
            std::printf("    [walk join] %s: the walks' parts on foot: %zu, the largest %u walks of %zu; places in a cut-off part: %d, in %zu "
                        "parts\n",
                        level, parts.sizes.size(), parts.sizes.empty() ? 0u : parts.sizes[parts.largest], m.laneLevel.walks.size(), cutOff,
                        cutOffParts.size());
            for (const std::pair<const uint32_t, CutOffPart>& c : cutOffParts) {
                std::string streets;
                for (const std::string& s : c.second.streets) streets += " " + s;
                std::printf("          part %u (%u walks): %d places, on%s\n", c.first, parts.sizes[c.first], c.second.places, streets.c_str());
            }
        }
        CHECK(joined == static_cast<int>(b->markers().size()));   // every marker of the brain stands beside a sidewalk
        // THE COUNTS of the run looked at (ADR-0468's table): places, cut off, placing / starting / finishing agreeing, the
        // fallback giving the stated walk. They move when the walks or the street map do (piece 6 retakes or drops them).
        const Pinned& pin = pinnedFor(level);
        CHECK(static_cast<int>(b->markers().size()) == pin.places && cutOff == pin.cutOff);
        CHECK(placing.agree == pin.placing && starting.agree == pin.starting && finishing.agree == pin.finishing);
        CHECK(fallbackSame == pin.fallbackSame && rampNodes == pin.rampNodes && atNode == pin.atNode);

        // ---- THE OLD SIM'S PLACES (a building's door, river_town): nothing stated, so the fallback by position ----------
        const citysim::PlaceMap& pm = lc.city.places();
        if (pm.size() > 0) {
            Tally doors;
            int doorJoined = 0, doorCut = 0;
            std::vector<double> doorDistance;
            std::map<std::string, int> doorWhy;
            for (const citysim::Place& p : pm.places()) {
                const WalkJoin j = m.walkJoinNear(p.door, open, &parts);
                if (j.valid()) {
                    ++doorJoined;
                    doorDistance.push_back(j.distance);
                    doorCut += j.cutOff ? 1 : 0;
                } else {
                    ++doorWhy[walkJoinWhyWord(j.why)];
                }
                OldAnswer o = oldAnswer(nav, p.entranceLink, p.door);
                o.point = p.entrance;
                doors.add(p.name.empty() ? "place " + std::to_string(p.id) : p.name, causeOf(m, nav, o, j), distance(o.point, j.point));
            }
            std::sort(doorDistance.begin(), doorDistance.end());
            const auto q = [&](double f) {
                return doorDistance.empty() ? 0.0 : doorDistance[static_cast<std::size_t>(f * static_cast<double>(doorDistance.size() - 1))];
            };
            std::printf("    [walk join] %s: %d buildings' doors (nothing stated: the fallback), %d joined (%d in a cut-off part); door to "
                        "the point joined: median %.1f m, 95th %.1f m, worst %.1f m\n",
                        level, pm.size(), doorJoined, doorCut, q(0.5), q(0.95), q(1.0));
            for (const std::pair<const std::string, int>& w : doorWhy) std::printf("        %4d  not joined: %s\n", w.second, w.first.c_str());
            doors.print(level, "a building's door (PlaceMap::add's snap)");
            doors.printListed(10);
            CHECK(pm.size() == pin.doors && doorJoined == pin.doorsJoined && doors.agree == pin.doorsAgree);
        }
    }
}
