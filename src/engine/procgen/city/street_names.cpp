#include "street_names.h"

#include "../../asset_root.h"
#include "../../../log.h"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <unordered_set>

namespace engine {

namespace {

// The FALLBACK book: assets/data/streets.json is the content (see the header).
// This is only what a stripped install falls back to, so a city still has
// street names when the asset is missing.
StreetNameBook fallbackBook() {
    StreetNameBook b;
    b.banks = {{"Oak", "Maple", "Elm", "Cedar", "Willow", "Birch", "Aspen", "Alder"},
               {"Lincoln", "Jefferson", "Franklin", "Madison", "Monroe", "Jackson"},
               {"Market", "Mill", "Depot", "Union", "Commerce", "Exchange"},
               {"First", "Second", "Third", "Fourth", "Fifth", "Sixth"}};
    b.suffixes = {{16, {"Boulevard", "Avenue", "Parkway"}},
                  {13, {"Avenue", "Road", "Way"}},
                  {9, {"Street", "Road", "Drive"}},
                  {0, {"Lane", "Court", "Place"}}};
    b.classSuffix = {{"alley", "Alley"}};
    b.abbreviations = {{"Boulevard", "Blvd"}, {"Parkway", "Pkwy"}, {"Avenue", "Ave"},
                       {"Street", "St"}, {"Road", "Rd"}, {"Drive", "Dr"}, {"Lane", "Ln"},
                       {"Court", "Ct"}, {"Place", "Pl"}, {"Alley", "Aly"}};
    return b;
}

struct Rng {
    uint32_t s;
    explicit Rng(uint32_t seed) : s(seed ? seed : 1u) {}
    uint32_t next() { s ^= s << 13; s ^= s >> 17; s ^= s << 5; return s; }
};

const char* classTag(RoadClass k) {
    switch (k) {
        case RoadClass::Freeway: return "freeway";
        case RoadClass::Arterial: return "arterial";
        case RoadClass::Collector: return "collector";
        case RoadClass::Ramp: return "ramp";
        case RoadClass::Alley: return "alley";
        default: return "local";
    }
}

// The suffix for a road: its CLASS if the book names one (an alley is an
// Alley whatever its width), else the widest rule it meets.
std::string suffixFor(const StreetNameBook& b, double width, RoadClass k, Rng& r) {
    const auto byClass = b.classSuffix.find(classTag(k));
    if (byClass != b.classSuffix.end()) return byClass->second;
    for (const StreetNameBook::SuffixRule& rule : b.suffixes)
        if (width >= rule.minWidth && !rule.choices.empty())
            return rule.choices[r.next() % rule.choices.size()];
    return "Street";
}

bool isStreet(RoadClass k) { return k != RoadClass::Freeway && k != RoadClass::Ramp; }

}  // namespace

StreetNameBook parseStreetNameBook(const nlohmann::json& names) {
    StreetNameBook b;
    if (!names.is_object()) return b;
    b.seed = names.value("seed", 20260918u);
    if (names.contains("banks") && names["banks"].is_object())
        for (const auto& [tag, words] : names["banks"].items()) {
            std::vector<std::string> pool;
            for (const auto& w : words)
                if (w.is_string()) pool.push_back(w.get<std::string>());
            if (!pool.empty()) b.banks.push_back(std::move(pool));
        }
    if (names.contains("suffixes"))
        for (const auto& rule : names["suffixes"]) {
            StreetNameBook::SuffixRule sr;
            sr.minWidth = rule.value("minWidth", 0.0);
            for (const auto& c : rule.value("choices", nlohmann::json::array()))
                if (c.is_string()) sr.choices.push_back(c.get<std::string>());
            if (!sr.choices.empty()) b.suffixes.push_back(std::move(sr));
        }
    std::stable_sort(b.suffixes.begin(), b.suffixes.end(),
                     [](const StreetNameBook::SuffixRule& x, const StreetNameBook::SuffixRule& y) {
                         return x.minWidth > y.minWidth;   // widest rule first
                     });
    if (names.contains("classSuffix") && names["classSuffix"].is_object())
        for (const auto& [k, v] : names["classSuffix"].items())
            if (v.is_string()) b.classSuffix[k] = v.get<std::string>();
    if (names.contains("abbreviations") && names["abbreviations"].is_object())
        for (const auto& [k, v] : names["abbreviations"].items())
            if (v.is_string()) b.abbreviations[k] = v.get<std::string>();
    return b;
}

const StreetNameBook& streetNameBook() {
    static StreetNameBook book = [] {
        StreetNameBook b;
        const std::string path = assetPath("assets/data/streets.json");
        std::ifstream in(path);
        if (in) {
            nlohmann::json doc;
            try {
                in >> doc;
                b = parseStreetNameBook(doc.value("names", nlohmann::json::object()));
                b.fromAsset = b.usable();
            } catch (const std::exception& e) {
                LOG_WARN << "[streets] " << path << ": " << e.what();
            }
        }
        if (!b.usable()) {
            LOG_WARN << "[streets] no usable assets/data/streets.json: "
                        "falling back to the built-in name book";
            b = fallbackBook();
        }
        return b;
    }();
    return book;
}

StreetNaming nameStreets(const RoadGraph& g, const StreetNamingParams& p) {
    return nameStreets(g, streetNameBook(), p);
}

StreetNaming nameStreets(const RoadGraph& g, const StreetNameBook& book,
                         const StreetNamingParams& p) {
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
    Rng rng(p.seed ? p.seed : book.seed);
    // The WORD is unique while the bank lasts: "Hamilton Rd" and "Hamilton Dr"
    // in one city is a wrong turn waiting to happen. Past that, the full name.
    std::unordered_set<std::string> taken, takenWord;
    const std::size_t bankCount = book.banks.size();
    for (int si : order) {
        Street& s = out.streets[static_cast<std::size_t>(si)];
        std::string name;
        for (int attempt = 0; attempt < 96 && bankCount; ++attempt) {
            const std::vector<std::string>& bank = book.banks[rng.next() % bankCount];
            const std::string word = bank[rng.next() % bank.size()];
            name = word + " " + suffixFor(book, s.width, s.klass, rng);
            const bool wordFree = !takenWord.count(word);
            if (!taken.count(name) && (wordFree || attempt >= 64)) {
                takenWord.insert(word);
                break;
            }
            name.clear();
        }
        if (name.empty() && bankCount) {   // the banks ran dry: number it
            const std::vector<std::string>& bank = book.banks[rng.next() % bankCount];
            name = bank[rng.next() % bank.size()] + " " +
                   suffixFor(book, s.width, s.klass, rng) + " " + std::to_string(si);
        }
        taken.insert(name);
        s.name = std::move(name);
    }
    return out;
}

std::string abbreviateStreetName(const std::string& name) {
    return abbreviateStreetName(name, streetNameBook());
}

std::string abbreviateStreetName(const std::string& name, const StreetNameBook& book) {
    for (const auto& [full, abbr] : book.abbreviations) {
        const std::string suffix = std::string(" ") + full;
        const std::size_t at = name.rfind(suffix);
        if (at != std::string::npos &&
            (at + suffix.size() == name.size() || name[at + suffix.size()] == ' '))
            return name.substr(0, at) + " " + abbr + name.substr(at + suffix.size());
    }
    return name;
}

}  // namespace engine
