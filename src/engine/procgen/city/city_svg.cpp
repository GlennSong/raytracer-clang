#include "city_svg.h"

#include "../../../log.h"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <sstream>

namespace engine {

namespace {

const char* kLayerNames[] = {"roads", "curbs", "sidewalks", "gaps", "nav", "furniture",
                             "objects", "blocks", "lots", "buildings", "districts",
                             "places", "legend"};
constexpr int kLayerCount = 13;

bool* layerFlag(CityMapLayers& L, int i) {
    switch (i) {
        case 0: return &L.roads;     case 1: return &L.curbs;    case 2: return &L.sidewalks;
        case 3: return &L.gaps;      case 4: return &L.nav;      case 5: return &L.furniture;
        case 6: return &L.objects;   case 7: return &L.blocks;   case 8: return &L.lots;
        case 9: return &L.buildings; case 10: return &L.districts; case 11: return &L.places;
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

bool writeCityMapSvg(const std::string& path, const CityMapData& data,
                     const CityMapLayers& layers) {
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
