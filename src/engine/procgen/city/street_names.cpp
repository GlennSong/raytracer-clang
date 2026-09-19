#include "street_names.h"

#include <algorithm>
#include <cmath>
#include <unordered_set>

namespace engine {

namespace {

// A deterministic name bank. Real cities mix trees, people, places and
// numbers; the mix matters more than any one word for a map feeling
// navigable. (Kept in step with tools/city_street_names.py's spirit, larger:
// a metro has hundreds of streets and every name must be unique.)
const char* const kTrees[] = {"Oak", "Maple", "Elm", "Cedar", "Willow", "Birch", "Aspen",
    "Alder", "Chestnut", "Hawthorn", "Laurel", "Linden", "Magnolia", "Juniper", "Poplar",
    "Sycamore", "Hazel", "Rowan", "Spruce", "Walnut", "Cypress", "Hickory", "Locust",
    "Mulberry", "Pine", "Redwood", "Sequoia", "Tamarack", "Beech", "Dogwood"};
const char* const kPeople[] = {"Lincoln", "Jefferson", "Franklin", "Madison", "Monroe",
    "Jackson", "Harrison", "Sherman", "Grant", "Hamilton", "Adams", "Clay", "Kearny",
    "Bryant", "Folsom", "Larkin", "Geary", "Taylor", "Hayes", "Fulton", "Grove", "Page",
    "Haight", "Mason", "Powell", "Stockton", "Davis", "Drumm", "Howard", "Harrison"};
const char* const kPlaces[] = {"Harbour", "Market", "Mill", "Quarry", "Foundry", "Cannery",
    "Depot", "Union", "Commerce", "Exchange", "Granary", "Wharf", "Mission", "Station",
    "College", "Church", "Garden", "Orchard", "Meadow", "Ridge", "Summit", "Valley",
    "Lake", "River", "Bridge", "Park", "Hill", "Spring", "Forest", "Canal"};
const char* const kOrdinals[] = {"First", "Second", "Third", "Fourth", "Fifth", "Sixth",
    "Seventh", "Eighth", "Ninth", "Tenth", "Eleventh", "Twelfth", "Thirteenth",
    "Fourteenth", "Fifteenth", "Sixteenth", "Seventeenth", "Eighteenth", "Nineteenth",
    "Twentieth"};

struct Rng {
    uint32_t s;
    explicit Rng(uint32_t seed) : s(seed ? seed : 1u) {}
    uint32_t next() { s ^= s << 13; s ^= s >> 17; s ^= s << 5; return s; }
};

// Suffix by width (the only class signal the plan reliably carries), plus the
// graph's own class for alleys.
const char* suffixFor(double width, RoadClass k, Rng& r) {
    if (k == RoadClass::Alley) return "Alley";
    static const char* const wide[] = {"Boulevard", "Avenue", "Parkway"};
    static const char* const mid[] = {"Avenue", "Road", "Way"};
    static const char* const street[] = {"Street", "Road", "Drive"};
    static const char* const narrow[] = {"Lane", "Court", "Place"};
    if (width >= 16) return wide[r.next() % 3];
    if (width >= 13) return mid[r.next() % 3];
    if (width >= 9) return street[r.next() % 3];
    return narrow[r.next() % 3];
}

bool isStreet(RoadClass k) { return k != RoadClass::Freeway && k != RoadClass::Ramp; }

}  // namespace

StreetNaming nameStreets(const RoadGraph& g, const StreetNamingParams& p) {
    StreetNaming out;
    const int E = static_cast<int>(g.edges.size());
    const int N = static_cast<int>(g.nodes.size());
    out.streetOfEdge.assign(static_cast<std::size_t>(E), -1);
    if (E == 0 || N == 0) return out;

    std::vector<std::vector<int>> at(static_cast<std::size_t>(N));
    for (int e = 0; e < E; ++e) {
        const RoadEdge& ed = g.edges[static_cast<std::size_t>(e)];
        if (ed.a < 0 || ed.b < 0 || ed.a >= N || ed.b >= N || ed.a == ed.b) continue;
        at[static_cast<std::size_t>(ed.a)].push_back(e);
        at[static_cast<std::size_t>(ed.b)].push_back(e);
    }
    auto other = [&](int e, int n) {
        const RoadEdge& ed = g.edges[static_cast<std::size_t>(e)];
        return ed.a == n ? ed.b : ed.a;
    };
    auto dirFrom = [&](int e, int n) {   // unit direction leaving n along e
        const Vec2 d = g.nodes[static_cast<std::size_t>(other(e, n))].pos -
                       g.nodes[static_cast<std::size_t>(n)].pos;
        const Real l = d.length();
        return l > 1e-9 ? d * (1.0 / l) : Vec2(1, 0);
    };
    const double cosJunction = std::cos(p.junctionTurnDeg * 3.14159265358979 / 180.0);

    std::vector<char> used(static_cast<std::size_t>(E), 0);
    for (int start = 0; start < E; ++start) {
        const RoadEdge& se = g.edges[static_cast<std::size_t>(start)];
        if (used[static_cast<std::size_t>(start)] || !isStreet(se.klass) || se.a == se.b) continue;
        used[static_cast<std::size_t>(start)] = 1;
        std::vector<int> edges{start}, nodes{se.a, se.b};
        // Grow off each end: 0 = past b (append), 1 = past a (prepend).
        for (int side = 0; side < 2; ++side) {
            int cur = start;
            int node = side == 0 ? se.b : se.a;
            for (;;) {
                const Vec2 heading = dirFrom(cur, other(cur, node)) ;   // arriving direction
                const std::vector<int>& here = at[static_cast<std::size_t>(node)];
                const bool plain = here.size() == 2;
                int best = -1;
                double bestCos = -2;
                for (int c : here) {
                    if (c == cur || used[static_cast<std::size_t>(c)]) continue;
                    const RoadEdge& ce = g.edges[static_cast<std::size_t>(c)];
                    if (!isStreet(ce.klass)) continue;
                    if (std::fabs(ce.width - g.edges[static_cast<std::size_t>(cur)].width) >=
                        p.widthTolerance)
                        continue;
                    const Vec2 d = dirFrom(c, node);
                    const double cs = heading.x * d.x + heading.y * d.y;
                    if (cs > bestCos) { bestCos = cs; best = c; }
                }
                if (best < 0) break;
                if (!plain && bestCos < cosJunction) break;   // a turn: another street
                // A plain node: a sampled curve turns a few degrees per node;
                // a sharp kink AT one node is two streets meeting at a corner.
                if (plain && bestCos < p.cosPlainKink) break;
                used[static_cast<std::size_t>(best)] = 1;
                const int next = other(best, node);
                if (side == 0) { edges.push_back(best); nodes.push_back(next); }
                else { edges.insert(edges.begin(), best); nodes.insert(nodes.begin(), next); }
                cur = best;
                node = next;
                if (next == (side == 0 ? nodes.front() : nodes.back())) break;   // a loop closed
            }
        }
        Street s;
        s.edges = std::move(edges);
        s.nodes = std::move(nodes);
        double wsum = 0;
        for (int e : s.edges) {
            const RoadEdge& ed = g.edges[static_cast<std::size_t>(e)];
            wsum += ed.width;
            s.length += (g.nodes[static_cast<std::size_t>(ed.a)].pos -
                         g.nodes[static_cast<std::size_t>(ed.b)].pos).length();
        }
        s.width = wsum / static_cast<double>(s.edges.size());
        s.klass = g.edges[static_cast<std::size_t>(s.edges.front())].klass;
        const int idx = static_cast<int>(out.streets.size());
        for (int e : s.edges) out.streetOfEdge[static_cast<std::size_t>(e)] = idx;
        out.streets.push_back(std::move(s));
    }

    // Names: deterministic, unique. Longest streets name first so the main
    // roads draw from the full bank.
    std::vector<int> order(out.streets.size());
    for (std::size_t i = 0; i < order.size(); ++i) order[i] = static_cast<int>(i);
    std::stable_sort(order.begin(), order.end(), [&](int a, int b) {
        return out.streets[static_cast<std::size_t>(a)].length >
               out.streets[static_cast<std::size_t>(b)].length;
    });
    Rng rng(p.seed);
    // The WORD is unique while the bank lasts: "Hamilton Rd" and "Hamilton Dr"
    // in one city is a wrong turn waiting to happen. Past that, the full name.
    std::unordered_set<std::string> taken, takenWord;
    struct Bank { const char* const* words; int n; };
    const Bank banks[] = {{kTrees, static_cast<int>(sizeof(kTrees) / sizeof(*kTrees))},
                          {kPeople, static_cast<int>(sizeof(kPeople) / sizeof(*kPeople))},
                          {kPlaces, static_cast<int>(sizeof(kPlaces) / sizeof(*kPlaces))},
                          {kOrdinals, static_cast<int>(sizeof(kOrdinals) / sizeof(*kOrdinals))}};
    for (int si : order) {
        Street& s = out.streets[static_cast<std::size_t>(si)];
        std::string name;
        for (int attempt = 0; attempt < 96; ++attempt) {
            const Bank& b = banks[rng.next() % 4];
            const std::string word = b.words[rng.next() % static_cast<uint32_t>(b.n)];
            name = word + " " + suffixFor(s.width, s.klass, rng);
            const bool wordFree = !takenWord.count(word);
            if (!taken.count(name) && (wordFree || attempt >= 64)) {
                takenWord.insert(word);
                break;
            }
            name.clear();
        }
        if (name.empty()) {   // the bank ran dry: number it
            const Bank& b = banks[rng.next() % 4];
            name = std::string(b.words[rng.next() % static_cast<uint32_t>(b.n)]) + " " +
                   suffixFor(s.width, s.klass, rng) + " " + std::to_string(si);
        }
        taken.insert(name);
        s.name = std::move(name);
    }
    return out;
}

std::string abbreviateStreetName(const std::string& name) {
    static const std::pair<const char*, const char*> kShort[] = {
        {"Boulevard", "Blvd"}, {"Avenue", "Ave"}, {"Parkway", "Pkwy"}, {"Street", "St"},
        {"Road", "Rd"}, {"Drive", "Dr"}, {"Lane", "Ln"}, {"Court", "Ct"}, {"Place", "Pl"},
        {"Alley", "Aly"}};
    for (const auto& [full, abbr] : kShort) {
        const std::string suffix = std::string(" ") + full;
        const std::size_t at = name.rfind(suffix);
        if (at != std::string::npos &&
            (at + suffix.size() == name.size() || name[at + suffix.size()] == ' '))
            return name.substr(0, at) + " " + abbr + name.substr(at + suffix.size());
    }
    return name;
}

}  // namespace engine
