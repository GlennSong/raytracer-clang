#include "city_svg.h"

#include "../../../log.h"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <unordered_map>

namespace engine {

namespace {

const char* kLayerNames[] = {"roads", "curbs", "sidewalks", "gaps", "nav", "furniture",
                             "objects", "blocks", "lots", "buildings", "districts",
                             "places", "conflicts", "legend"};
constexpr int kLayerCount = 14;

bool* layerFlag(CityMapLayers& L, int i) {
    switch (i) {
        case 0: return &L.roads;     case 1: return &L.curbs;    case 2: return &L.sidewalks;
        case 3: return &L.gaps;      case 4: return &L.nav;      case 5: return &L.furniture;
        case 6: return &L.objects;   case 7: return &L.blocks;   case 8: return &L.lots;
        case 9: return &L.buildings; case 10: return &L.districts; case 11: return &L.places;
        case 12: return &L.conflicts;
        default: return &L.legend;
    }
}

const char* classColor(RoadClass k) {
    switch (k) {
        case RoadClass::Freeway:   return "#1f1f1f";
        case RoadClass::Ramp:      return "#3d3d3d";
        case RoadClass::Arterial:  return "#5a5a5a";
        case RoadClass::Collector: return "#7c7c7c";
        case RoadClass::Local:     return "#a3a3a3";
        case RoadClass::Alley:     return "#c9c4b8";
    }
    return "#a3a3a3";
}

// A stable colour per district name (the same district is the same colour
// on every map of every city).
std::string districtColor(const std::string& name) {
    static const char* palette[] = {"#e76f51", "#2a9d8f", "#e9c46a", "#8ab17d", "#6d597a",
                                    "#f4a261", "#457b9d", "#b5838d", "#606c38", "#bc6c25"};
    uint32_t h = 2166136261u;
    for (unsigned char c : name) { h ^= c; h *= 16777619u; }
    return palette[h % 10];
}

std::string esc(const std::string& s) {
    std::string o;
    for (char c : s) {
        if (c == '&') o += "&amp;";
        else if (c == '<') o += "&lt;";
        else if (c == '>') o += "&gt;";
        else if (c == '\'') o += "&apos;";
        else o += c;
    }
    return o;
}

void polyPoints(std::ostream& out, const Poly2& p) {
    for (const Vec2& v : p) out << v.x << "," << v.y << " ";
}

}  // namespace

const char* const* CityMapLayers::names(int* count) {
    if (count) *count = kLayerCount;
    return kLayerNames;
}

CityMapLayers CityMapLayers::fromList(const std::string& csv) {
    CityMapLayers L;
    std::string s = csv;
    s.erase(std::remove_if(s.begin(), s.end(), [](unsigned char c) { return std::isspace(c); }),
            s.end());
    if (s.empty() || s == "all") return L;
    for (int i = 0; i < kLayerCount; ++i) *layerFlag(L, i) = false;
    std::stringstream ss(s);
    std::string item;
    while (std::getline(ss, item, ',')) {
        bool known = false;
        for (int i = 0; i < kLayerCount; ++i)
            if (item == kLayerNames[i]) { *layerFlag(L, i) = true; known = true; }
        if (!known && !item.empty())
            LOG_WARN << "[citymap] unknown layer '" << item << "' (known: " << CityMapLayers().toList() << ")";
    }
    return L;
}

std::string CityMapLayers::toList() const {
    CityMapLayers copy = *this;
    std::string s;
    for (int i = 0; i < kLayerCount; ++i)
        if (*layerFlag(copy, i)) { if (!s.empty()) s += ","; s += kLayerNames[i]; }
    return s;
}

CityMapLayers furnitureMapLayers() {
    return CityMapLayers::fromList("roads,nav,furniture,legend");
}

std::vector<Poly2> sidewalkBandCentrelines(const std::vector<Poly2>& curbLoops,
                                           double sidewalkWidth) {
    std::vector<Poly2> out;
    const double d = sidewalkWidth * 0.5;
    for (const Poly2& L : curbLoops) {
        const std::size_t n = L.size();
        if (n < 3) continue;
        Poly2 c;
        c.reserve(n);
        for (std::size_t i = 0; i < n; ++i) {
            const Vec2& p = L[i];
            const Vec2& q = L[(i + 1) % n];
            const Vec2& o = L[(i + n - 1) % n];
            // Right normals of the incoming and outgoing edges (the band
            // rides the loop's right normal, outward from the asphalt).
            auto rightNormal = [](const Vec2& a, const Vec2& b) {
                Vec2 e = b - a;
                const double len = e.length();
                if (len < 1e-9) return Vec2(0, 0);
                e = e * (1.0 / len);
                return Vec2(e.y, -e.x);
            };
            const Vec2 n0 = rightNormal(o, p), n1 = rightNormal(p, q);
            Vec2 m = n0 + n1;
            const double ml = m.length();
            if (ml < 1e-6) { c.push_back(p + n1 * d); continue; }
            m = m * (1.0 / ml);
            // Mitre, clamped so a hairpin cannot throw the point to infinity.
            const double cosHalf = std::max(dot(m, n1), 0.35);
            c.push_back(p + m * (d / cosHalf));
        }
        out.push_back(std::move(c));
    }
    return out;
}

std::vector<SidewalkCrossing> findSidewalkRoadCrossings(
    const CityMapData& data, const std::vector<const RoadDeckField*>& decks,
    double tolerance, double mergeRadius) {
    std::vector<SidewalkCrossing> out;
    const RoadGraph& g = data.roads;
    const int N = static_cast<int>(g.nodes.size());
    if (decks.empty() || data.curbLoops.empty()) return out;
    // A spatial hash of the graph edges, only to LABEL a hit with its road.
    const double cell = 32.0;
    std::unordered_map<long long, std::vector<int>> cells;
    auto key = [](int cx, int cz) {
        return (static_cast<long long>(cx) << 32) ^ (static_cast<long long>(cz) & 0xffffffffLL);
    };
    auto cellOf = [&](double v) { return static_cast<int>(std::floor(v / cell)); };
    for (int ei = 0; ei < static_cast<int>(g.edges.size()); ++ei) {
        const RoadEdge& e = g.edges[ei];
        if (e.a < 0 || e.b < 0 || e.a >= N || e.b >= N) continue;
        const Vec2& A = g.nodes[e.a].pos;
        const Vec2& B = g.nodes[e.b].pos;
        const double hw = e.width;
        for (int cz = cellOf(std::min(A.y, B.y) - hw); cz <= cellOf(std::max(A.y, B.y) + hw); ++cz)
            for (int cx = cellOf(std::min(A.x, B.x) - hw); cx <= cellOf(std::max(A.x, B.x) + hw); ++cx)
                cells[key(cx, cz)].push_back(ei);
    }
    auto segDist = [](const Vec2& p, const Vec2& a, const Vec2& b) {
        const Vec2 ab = b - a;
        const double l2 = ab.lengthSquared();
        double t = l2 > 1e-12 ? dot(p - a, ab) / l2 : 0.0;
        t = std::max(0.0, std::min(1.0, t));
        return (a + ab * t - p).length();
    };
    auto nearestEdge = [&](const Vec2& p) {
        int best = -1;
        double bestD = 1e30;
        auto it = cells.find(key(cellOf(p.x), cellOf(p.y)));
        if (it == cells.end()) return best;
        for (int ei : it->second) {
            const double d = segDist(p, g.nodes[g.edges[ei].a].pos, g.nodes[g.edges[ei].b].pos);
            if (d < bestD) { bestD = d; best = ei; }
        }
        return best;
    };
    // Every band centreline, sampled each metre, against every deck.
    struct Hit { Vec2 pos; double depth; RoadClass klass; double width; };
    std::vector<Hit> hits;
    for (const Poly2& band : sidewalkBandCentrelines(data.curbLoops, data.sidewalkWidth)) {
        const std::size_t n = band.size();
        for (std::size_t i = 0; i < n; ++i) {
            const Vec2& a = band[i];
            const Vec2& b = band[(i + 1) % n];
            const double len = (b - a).length();
            const int steps = std::max(1, static_cast<int>(len));
            for (int k = 0; k < steps; ++k) {
                const Vec2 p = a + (b - a) * (static_cast<double>(k) / steps);
                double depth = 0.0;
                RoadClass klass = RoadClass::Local;
                double width = 0.0;
                for (const RoadDeckField* d : decks) {
                    if (!d) continue;
                    int spine = -1;
                    const double dd = d->depthInside(p.x, p.y, &spine);
                    if (dd > depth) {
                        depth = dd;
                        if (spine >= 0 && spine < static_cast<int>(d->spines.size())) {
                            klass = d->spines[spine].klass;
                            width = d->spines[spine].halfWidth * 2.0;
                        }
                    }
                }
                depth -= tolerance;
                if (depth > 0.0) hits.push_back({p, depth, klass, width});
            }
        }
    }
    // Cluster into places (greedy, around the deepest hit).
    std::vector<char> used(hits.size(), 0);
    for (std::size_t i = 0; i < hits.size(); ++i) {
        if (used[i]) continue;
        SidewalkCrossing c;
        c.pos = hits[i].pos;
        c.depth = hits[i].depth;
        c.deckClass = hits[i].klass;
        c.deckWidth = hits[i].width;
        double minX = 1e30, maxX = -1e30, minZ = 1e30, maxZ = -1e30;
        for (std::size_t j = i; j < hits.size(); ++j) {
            if (used[j] || (hits[j].pos - hits[i].pos).length() > mergeRadius) continue;
            used[j] = 1;
            ++c.samples;
            if (hits[j].depth > c.depth) {
                c.depth = hits[j].depth; c.pos = hits[j].pos;
                c.deckClass = hits[j].klass; c.deckWidth = hits[j].width;
            }
            minX = std::min(minX, hits[j].pos.x); maxX = std::max(maxX, hits[j].pos.x);
            minZ = std::min(minZ, hits[j].pos.y); maxZ = std::max(maxZ, hits[j].pos.y);
        }
        c.spanMetres = std::hypot(maxX - minX, maxZ - minZ);
        c.edge = nearestEdge(c.pos);
        if (c.edge >= 0) { c.klass = g.edges[c.edge].klass; c.width = g.edges[c.edge].width; }
        out.push_back(c);
    }
    // Dead-end stubs: degree from the edges, nearest degree-1 node per place.
    std::vector<int> degree(N, 0);
    for (const RoadEdge& e : g.edges)
        if (e.a >= 0 && e.b >= 0 && e.a < N && e.b < N) { ++degree[e.a]; ++degree[e.b]; }
    for (SidewalkCrossing& c : out) {
        double best = 25.0, nearest = 15.0;
        int nearestNode = -1;
        for (int v = 0; v < N; ++v) {
            const double d = (g.nodes[v].pos - c.pos).length();
            if (degree[v] == 1 && d < best) { best = d; c.deadEndDist = d; }
            if (d < nearest) { nearest = d; c.nodeDist = d; c.nodeDegree = degree[v]; nearestNode = v; }
        }
        if (nearestNode >= 0) {
            for (int v = 0; v < N; ++v) {
                if ((g.nodes[v].pos - g.nodes[nearestNode].pos).length() > 2.0) continue;
                ++c.coincident;
                if (!c.coincidentDegrees.empty()) c.coincidentDegrees += "+";
                c.coincidentDegrees += std::to_string(degree[v]);
            }
        }
    }
    std::sort(out.begin(), out.end(),
              [](const SidewalkCrossing& a, const SidewalkCrossing& b) { return a.depth > b.depth; });
    return out;
}

const char* roadClassName(RoadClass k) {
    switch (k) {
        case RoadClass::Freeway:   return "freeway";
        case RoadClass::Ramp:      return "ramp";
        case RoadClass::Arterial:  return "arterial";
        case RoadClass::Collector: return "collector";
        case RoadClass::Local:     return "local";
        case RoadClass::Alley:     return "alley";
    }
    return "?";
}

bool writeCityMapSvg(const std::string& path, const CityMapData& data,
                     const CityMapLayers& layers,
                     const std::vector<const RoadDeckField*>& decks) {
    const RoadGraph& g = data.roads;
    double minX = 1e30, minZ = 1e30, maxX = -1e30, maxZ = -1e30;
    auto grow = [&](double x, double z) {
        minX = std::min(minX, x); maxX = std::max(maxX, x);
        minZ = std::min(minZ, z); maxZ = std::max(maxZ, z);
    };
    for (const RoadNode& n : g.nodes) grow(n.pos.x, n.pos.y);
    for (const Poly2& p : data.blocks) for (const Vec2& v : p) grow(v.x, v.y);
    for (const CityMapData::Building& b : data.buildings) for (const Vec2& v : b.plan) grow(v.x, v.y);
    if (minX > maxX) { LOG_ERROR << "[citymap] nothing to draw"; return false; }
    const double pad = 60.0;
    minX -= pad; minZ -= pad; maxX += pad; maxZ += pad;
    const double w = maxX - minX, h = maxZ - minZ;

    std::ofstream out(path);
    if (!out) {
        LOG_ERROR << "[citymap] cannot write " << path;
        return false;
    }
    out << std::fixed << std::setprecision(1);
    out << "<svg xmlns='http://www.w3.org/2000/svg' viewBox='" << minX << " " << minZ << " " << w
        << " " << h << "' width='" << w << "' height='" << h
        << "' font-family='Helvetica, Arial, sans-serif'>\n";
    out << "<rect x='" << minX << "' y='" << minZ << "' width='" << w << "' height='" << h
        << "' fill='#f4f1ea'/>\n";
    // In-file toggles: the legend entries switch their layer when the file is
    // opened in a browser (scripts run in a top-level SVG document).
    out << "<script><![CDATA[\n"
           "function toggleLayer(id, el) {\n"
           "  var g = document.getElementById(id); if (!g) return;\n"
           "  var off = g.getAttribute('display') === 'none';\n"
           "  g.setAttribute('display', off ? 'inline' : 'none');\n"
           "  if (el) el.setAttribute('opacity', off ? '1' : '0.35');\n"
           "}\n]]></script>\n";

    const int N = static_cast<int>(g.nodes.size());
    int drawn = 0;

    // --- districts: lots filled by district, hub rings + labels ------------------
    if (layers.districts) {
        out << "<g id='layer-districts'>\n";
        for (const CityMapData::Building& b : data.buildings) {
            if (b.plan.size() < 3 || b.district.empty()) continue;
            out << "<polygon points='";
            polyPoints(out, b.plan);
            out << "' fill='" << districtColor(b.district) << "' fill-opacity='0.22' stroke='none'/>\n";
        }
        for (const CityMapData::Hub& hub : data.hubs) {
            const std::string col = districtColor(hub.name);
            out << "<circle cx='" << hub.pos.x << "' cy='" << hub.pos.y << "' r='" << data.hubRadius
                << "' fill='none' stroke='" << col << "' stroke-width='2' stroke-dasharray='12 8' opacity='0.8'/>\n";
            out << "<circle cx='" << hub.pos.x << "' cy='" << hub.pos.y << "' r='6' fill='" << col << "'/>\n";
            out << "<text x='" << hub.pos.x + 10 << "' y='" << hub.pos.y - 8 << "' font-size='26' fill='"
                << col << "' font-weight='bold'>" << esc(hub.name) << "</text>\n";
        }
        out << "</g>\n";
        ++drawn;
    }
    // --- blocks + lots ----------------------------------------------------------
    if (layers.blocks) {
        out << "<g id='layer-blocks' fill='#d9e4d2' fill-opacity='0.5' stroke='#7a9a6e' stroke-width='0.8'>\n";
        for (const Poly2& p : data.blocks) {
            if (p.size() < 3) continue;
            out << "<polygon points='";
            polyPoints(out, p);
            out << "'/>\n";
        }
        out << "</g>\n";
        ++drawn;
    }
    if (layers.lots) {
        out << "<g id='layer-lots' fill='none' stroke='#8c7b6b' stroke-width='0.5' stroke-dasharray='3 2'>\n";
        for (const Poly2& p : data.lots) {
            if (p.size() < 3) continue;
            out << "<polygon points='";
            polyPoints(out, p);
            out << "'/>\n";
        }
        out << "</g>\n";
        ++drawn;
    }
    // --- sidewalks: the band, then the curb line on top ---------------------------
    if (layers.sidewalks) {
        out << "<g id='layer-sidewalks' fill='none' stroke='#bfb8ad' stroke-width='" << data.sidewalkWidth
            << "' stroke-linejoin='round' stroke-linecap='round'>\n";
        for (const Poly2& c : sidewalkBandCentrelines(data.curbLoops, data.sidewalkWidth)) {
            out << "<polygon points='";
            polyPoints(out, c);
            out << "'/>\n";
        }
        out << "</g>\n";
        ++drawn;
    }
    // --- roads: one stroke per edge, width = carriageway, colour = class -----------
    if (layers.roads) {
        out << "<g id='layer-roads' stroke-linecap='round' fill='none'>\n";
        const RoadClass order[] = {RoadClass::Freeway, RoadClass::Ramp, RoadClass::Arterial,
                                   RoadClass::Collector, RoadClass::Local, RoadClass::Alley};
        for (RoadClass k : order)
            for (const RoadEdge& e : g.edges) {
                if (e.klass != k || e.a < 0 || e.b < 0 || e.a >= N || e.b >= N) continue;
                out << "<line x1='" << g.nodes[e.a].pos.x << "' y1='" << g.nodes[e.a].pos.y << "' x2='"
                    << g.nodes[e.b].pos.x << "' y2='" << g.nodes[e.b].pos.y << "' stroke='"
                    << classColor(k) << "' stroke-width='" << e.width << "'/>\n";
            }
        out << "</g>\n";
        ++drawn;
    }
    if (layers.curbs) {
        out << "<g id='layer-curbs' fill='none' stroke='#4a4036' stroke-width='0.5'>\n";
        for (const Poly2& L : data.curbLoops) {
            if (L.size() < 3) continue;
            out << "<polygon points='";
            polyPoints(out, L);
            out << "'/>\n";
        }
        out << "</g>\n";
        ++drawn;
    }
    if (layers.gaps) {
        out << "<g id='layer-gaps' stroke='#d81b60' stroke-width='1.2' fill='none'>\n";
        for (const auto& gp : data.mouthGaps)
            out << "<line x1='" << gp.first.x << "' y1='" << gp.first.y << "' x2='" << gp.second.x
                << "' y2='" << gp.second.y << "'/>\n";
        out << "</g>\n";
        ++drawn;
    }
    // --- buildings: plan polygons ---------------------------------------------------
    if (layers.buildings) {
        out << "<g id='layer-buildings' fill='#8f8378' fill-opacity='0.75' stroke='#3f3731' stroke-width='0.4'>\n";
        for (const CityMapData::Building& b : data.buildings) {
            if (b.plan.size() < 3) continue;
            out << "<polygon points='";
            polyPoints(out, b.plan);
            out << "'";
            if (!b.type.empty() || !b.district.empty())
                out << "><title>" << esc(b.type) << (b.district.empty() ? "" : " · ") << esc(b.district)
                    << "</title></polygon>\n";
            else out << "/>\n";
        }
        out << "</g>\n";
        ++drawn;
    }
    // --- nav graph -------------------------------------------------------------------
    if (layers.nav) {
        out << "<g id='layer-nav' stroke='#0aa2c7' stroke-width='0.6' fill='none' opacity='0.9'>\n";
        for (int li = 0; li < data.nav.linkCount(); ++li) {
            const NavLink& L = data.nav.links[li];
            if (L.from < 0 || L.to < 0 || L.from >= data.nav.nodeCount() || L.to >= data.nav.nodeCount())
                continue;
            out << "<line x1='" << data.nav.nodes[L.from].x << "' y1='" << data.nav.nodes[L.from].y
                << "' x2='" << data.nav.nodes[L.to].x << "' y2='" << data.nav.nodes[L.to].y << "'/>\n";
        }
        for (std::size_t ni = 0; ni < data.nav.nodes.size(); ++ni)
            if (ni < data.nav.outLinks.size() && data.nav.outLinks[ni].size() >= 3)
                out << "<circle cx='" << data.nav.nodes[ni].x << "' cy='" << data.nav.nodes[ni].y
                    << "' r='3'/>\n";
        out << "</g>\n";
        ++drawn;
    }
    // --- planted objects -------------------------------------------------------------
    if (layers.objects) {
        out << "<g id='layer-objects'>\n<g fill='#2e7d32' stroke='none'>\n";
        for (const CityMapData::Object& o : data.objects)
            if (o.kind == CityMapData::ObjectKind::Scenery)
                out << "<circle cx='" << o.pos.x << "' cy='" << o.pos.y << "' r='1.0'/>\n";
        out << "</g>\n<g fill='#6a1b9a' stroke='none'>\n";
        for (const CityMapData::Object& o : data.objects)
            if (o.kind == CityMapData::ObjectKind::Furniture)
                out << "<circle cx='" << o.pos.x << "' cy='" << o.pos.y << "' r='1.2'/>\n";
        out << "</g>\n</g>\n";
        ++drawn;
    }
    if (layers.furniture) {
        out << "<g id='layer-furniture'>\n<g fill='#f2a20c' stroke='none'>\n";
        for (const Vec3& b : data.furniture.lampBases)
            out << "<circle cx='" << b.x << "' cy='" << b.z << "' r='1.3'/>\n";
        out << "</g>\n<g stroke='#d62828' stroke-width='0.9' fill='#d62828'>\n";
        for (const SignalSpot& s : data.furniture.signals) {
            out << "<circle cx='" << s.base.x << "' cy='" << s.base.z << "' r='2.2'/>\n";
            out << "<line x1='" << s.base.x << "' y1='" << s.base.z << "' x2='" << s.base.x + s.face.x * 6.0
                << "' y2='" << s.base.z + s.face.y * 6.0 << "'/>\n";
        }
        out << "</g>\n</g>\n";
        ++drawn;
    }
    // --- conflicts: the sidewalk band inside a carriageway ---------------------------
    std::vector<SidewalkCrossing> crossings;
    if (layers.conflicts) {
        crossings = findSidewalkRoadCrossings(data, decks);
        out << "<g id='layer-conflicts' stroke='#ff1744' stroke-width='1.6' fill='#ff1744' font-size='14' font-weight='bold'>\n";
        int n = 1;
        for (const SidewalkCrossing& c : crossings) {
            const double r = 5.0;
            out << "<line x1='" << c.pos.x - r << "' y1='" << c.pos.y - r << "' x2='" << c.pos.x + r
                << "' y2='" << c.pos.y + r << "'/><line x1='" << c.pos.x - r << "' y1='" << c.pos.y + r
                << "' x2='" << c.pos.x + r << "' y2='" << c.pos.y - r << "'/>\n";
            out << "<circle cx='" << c.pos.x << "' cy='" << c.pos.y << "' r='" << r + 3
                << "' fill='none'/>\n";
            out << "<text x='" << c.pos.x + r + 4 << "' y='" << c.pos.y - 2 << "' stroke='none'>#" << n
                << " " << std::setprecision(1) << c.depth << " m into " << roadClassName(c.klass)
                << (c.deadEndDist >= 0 ? " (stub end " : "") << (c.deadEndDist >= 0 ? std::to_string(static_cast<int>(c.deadEndDist)) + " m away)" : "")
                << "</text>\n";
            ++n;
        }
        out << "</g>\n";
        ++drawn;
        // The census, deepest first (the first ten in the log; all in the map).
        if (decks.empty())
            LOG_WARN << "[citymap] conflicts layer: no road decks given — nothing measured";
        LOG_INFO << "[citymap] sidewalk band on built asphalt at " << crossings.size() << " places"
                 << " (" << decks.size() << " decks, tolerance 0.5 m)";
        for (std::size_t i = 0; i < crossings.size() && i < 10; ++i) {
            const SidewalkCrossing& c = crossings[i];
            LOG_INFO << "[citymap]   #" << i + 1 << " (" << c.pos.x << ", " << c.pos.y << ") "
                     << c.depth << " m onto the deck near a " << c.width << " m " << roadClassName(c.klass)
                     << " (edge " << c.edge << "), on the built " << c.deckWidth << " m "
                     << roadClassName(c.deckClass) << " chain; "
                     << (c.nodeDegree >= 0 ? "nearest node degree " + std::to_string(c.nodeDegree) + " at " + std::to_string(c.nodeDist).substr(0, 4) + " m (" + std::to_string(c.coincident) + " nodes within 2 m: degrees " + c.coincidentDegrees + "); " : "no node within 15 m; ")
                     << c.samples << " samples over " << c.spanMetres << " m"
                     << (c.deadEndDist >= 0 ? " — a dead-end stub " : "")
                     << (c.deadEndDist >= 0 ? std::to_string(c.deadEndDist).substr(0, 4) + " m away" : "");
        }
    }
    if (layers.places) {
        out << "<g id='layer-places' font-size='12' fill='#1b5e20'>\n";
        for (const CityMapData::Place& p : data.places) {
            out << "<rect x='" << p.pos.x - 3 << "' y='" << p.pos.y - 3 << "' width='6' height='6' fill='#1b5e20'/>\n";
            out << "<text x='" << p.pos.x + 5 << "' y='" << p.pos.y + 4 << "'>" << esc(p.type)
                << (p.name.empty() ? "" : " ") << esc(p.name) << "</text>\n";
        }
        out << "</g>\n";
        ++drawn;
    }

    // --- legend: one clickable entry per drawn layer, counts, scale bar ----------------
    if (layers.legend) {
        const double lx = minX + 24, ly = minZ + 40;
        out << "<g id='layer-legend' font-size='22' fill='#222'>\n";
        out << "<rect x='" << lx - 12 << "' y='" << ly - 30 << "' width='560' height='"
            << 40 + 30 * 14 << "' fill='white' fill-opacity='0.8' stroke='#999'/>\n";
        out << "<text x='" << lx << "' y='" << ly << "' font-weight='bold'>city map · 1 unit = 1 m · y = world z · click a row to toggle its layer</text>\n";
        int row = 1;
        auto entry = [&](bool on, const char* id, const std::string& swatch, const std::string& text) {
            if (!on) return;
            const double y = ly + 30 * row++;
            out << "<g onclick=\"toggleLayer('" << id << "', this)\" style='cursor:pointer'>"
                << swatch << "<text x='" << lx + 30 << "' y='" << y + 7 << "'>" << text << "</text></g>\n";
        };
        auto sw = [&](const std::string& fill, const std::string& stroke, int r) {
            std::ostringstream s;
            const double y = ly + 30 * row;
            s << "<circle cx='" << lx + 10 << "' cy='" << y << "' r='" << r << "' fill='" << fill
              << "' stroke='" << stroke << "' stroke-width='2'/>";
            return s.str();
        };
        int nSignals = static_cast<int>(data.furniture.signals.size());
        int nLamps = static_cast<int>(data.furniture.lampBases.size());
        int nTrees = 0, nFurn = 0;
        for (const CityMapData::Object& o : data.objects)
            (o.kind == CityMapData::ObjectKind::Scenery ? nTrees : nFurn)++;
        entry(layers.districts, "layer-districts", sw("#e76f51", "none", 8),
              "districts: lots tinted by district, hub rings (" + std::to_string(data.hubs.size()) + " hubs)");
        entry(layers.blocks, "layer-blocks", sw("#d9e4d2", "#7a9a6e", 8),
              "blocks (" + std::to_string(data.blocks.size()) + ")");
        entry(layers.lots, "layer-lots", sw("none", "#8c7b6b", 8),
              "lots (" + std::to_string(data.lots.size()) + ")");
        entry(layers.sidewalks, "layer-sidewalks", sw("#bfb8ad", "none", 8),
              "sidewalks: the mesher's curb band, " + std::to_string(static_cast<int>(data.sidewalkWidth * 10) / 10.0).substr(0, 4) +
                  " m wide, outside " + std::to_string(data.curbLoops.size()) + " curb loops");
        entry(layers.roads, "layer-roads", sw("#7c7c7c", "none", 8),
              "streets by class (stroke = carriageway width), " + std::to_string(g.edges.size()) + " edges");
        entry(layers.curbs, "layer-curbs", sw("none", "#4a4036", 8), "curb lines (asphalt union outline)");
        entry(layers.gaps, "layer-gaps", sw("none", "#d81b60", 8),
              "band gaps: non-street mouths (" + std::to_string(data.mouthGaps.size()) + ")");
        entry(layers.buildings, "layer-buildings", sw("#8f8378", "#3f3731", 8),
              "buildings: plan polygons (" + std::to_string(data.buildings.size()) + ")");
        entry(layers.nav, "layer-nav", sw("none", "#0aa2c7", 8),
              "nav graph: links + junction rings (" + std::to_string(data.nav.linkCount()) + " links)");
        entry(layers.objects, "layer-objects", sw("#2e7d32", "#6a1b9a", 8),
              "planted objects: trees/scenery " + std::to_string(nTrees) + " (green), furniture " +
                  std::to_string(nFurn) + " (purple)");
        entry(layers.furniture, "layer-furniture", sw("#d62828", "#f2a20c", 8),
              "poles: " + std::to_string(nSignals) + " signals (red, tick = facing), " +
                  std::to_string(nLamps) + " lamps (amber)");
        entry(layers.places, "layer-places", sw("#1b5e20", "none", 8),
              "places (" + std::to_string(data.places.size()) + ")");
        entry(layers.conflicts, "layer-conflicts", sw("none", "#ff1744", 8),
              "CONFLICTS: sidewalk band on built asphalt (" + std::to_string(crossings.size()) +
                  " places, deepest first" + (decks.empty() ? "; no decks measured" : "") + ")");
        const double sy = ly + 30 * row + 10;
        out << "<line x1='" << lx << "' y1='" << sy << "' x2='" << lx + 200 << "' y2='" << sy
            << "' stroke='#222' stroke-width='4'/><text x='" << lx + 210 << "' y='" << sy + 8 << "'>200 m</text>\n";
        out << "</g>\n";
    }
    out << "</svg>\n";
    LOG_INFO << "[citymap] wrote " << path << " (" << drawn << " layers: " << layers.toList() << "; "
             << g.edges.size() << " edges, " << data.curbLoops.size() << " curb loops, "
             << data.buildings.size() << " buildings, " << data.objects.size() << " objects)";
    return static_cast<bool>(out);
}

}  // namespace engine
