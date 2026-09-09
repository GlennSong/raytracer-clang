#include "engine/procgen/lanelab/lanelab_export.h"
#include "engine/procgen/lanelab/vertical_profile.h"

#include "engine/glb_export.h"
#include <nlohmann/json.hpp>

#include <cstring>
#include <fstream>

namespace engine {
namespace lanelab {

bool writeGlb(const std::vector<NamedMesh>& meshes, const std::string& path, std::string* error) {
    // The engine's writer (glb_export.h) does the glTF; it also keeps tangents and a varying vertex colour,
    // which the original lanelab-only writer dropped.
    std::vector<engine::GlbEntry> entries;
    for (const NamedMesh& nm : meshes) { engine::GlbEntry e; e.name = nm.name; e.mesh = &nm.mesh; e.color = nm.color; entries.push_back(std::move(e)); }
    return engine::writeGlb(entries, path, nlohmann::json::object(), error);
}

bool writePlanSvg(const Result& r, const std::string& path) {
    const RoadLabGraph& g = r.graph; std::array<double, 4> b = g.bounds(); double W = b[1] - b[0], Hh = b[3] - b[2], scale = 1600.0 / std::max(W, Hh);
    std::ofstream f(path); if (!f) return false;
    auto X = [&](double x) { return (x - b[0]) * scale; }; auto Y = [&](double y) { return (b[3] - y) * scale; };
    f << "<svg xmlns='http://www.w3.org/2000/svg' width='" << W * scale << "' height='" << Hh * scale << "' viewBox='0 0 " << W * scale << " " << Hh * scale << "'>\n";
    f << "<rect width='100%' height='100%' fill='#8fa36b'/>\n";
    auto poly = [&](const PolySet& set, const char* fill, const char* stroke, double sw) {
        for (const Polygon2& p : set) {
            f << "<path fill='" << fill << "' fill-rule='evenodd' stroke='" << stroke << "' stroke-width='" << sw << "' d='";
            auto ring = [&](const Ring& rg) { for (size_t i = 0; i < rg.size(); ++i) f << (i ? " L" : "M") << X(rg[i].x) << " " << Y(rg[i].y); f << " Z"; };
            ring(p.outer); for (const Ring& h : p.holes) ring(h); f << "'/>\n";
        }
    };
    poly(r.pavement.sidewalk, "#d9d4c7", "#6d6a63", 0.5); poly(r.pavement.shoulder, "#5f5d5a", "none", 0); poly(r.pavement.median, "#7f9a5c", "none", 0);
    std::map<std::string, const char*> fc = {{"freeway", "#2b2b30"}, {"ramp", "#38383e"}, {"arterial", "#3a3a3c"}, {"local", "#454547"}};
    for (size_t li = 0; li < r.lanes.lanes.size(); ++li) { auto it = fc.find(r.lanes.lanes[li].cls); poly(r.pavement.footprints[li], it == fc.end() ? "#3a3a3c" : it->second, "none", 0); }
    poly(r.pavement.surface, "none", "#111", 0.6);
    for (const EdgeSpec* e : g.paved()) {
        std::string d; bool open = false;
        for (size_t i = 0; i < e->xy.size(); ++i) {
            bool onb = e->z[i] - e->t[i] > g.rules.bridgeH;
            if (onb) { d += (open ? " L" : "M") + std::to_string(X(e->xy[i].x)) + " " + std::to_string(Y(e->xy[i].y)); open = true; } else open = false;
        }
        if (!d.empty()) f << "<path fill='none' stroke='#f2d16b' stroke-width='" << 2.4 * scale << "' stroke-opacity='0.8' d='" << d << "'/>\n";
    }
    for (const Pier& p : r.piers) f << "<rect x='" << X(p.xy.x) - 1.3 * scale << "' y='" << Y(p.xy.y) - 1.3 * scale << "' width='" << 2.6 * scale << "' height='" << 2.6 * scale << "' fill='#f2d16b' stroke='#000' stroke-width='0.3'/>\n";
    f << "</svg>\n"; return true;
}

bool writeStatsJson(const Result& r, const std::string& path) {
    nlohmann::json j; j["name"] = r.graph.name; j["lanes"] = r.lanes.lanes.size(); j["seconds"] = r.seconds; j["timings"] = r.timings;
    for (const EdgeSpec& e : r.graph.edges) {
        if (e.laneCount() == 0) continue; nlohmann::json ej; ej["class"] = e.cls; ej["length"] = e.length(); ej["max_grade"] = maxGrade(e); ej["bridge_len"] = r.bridgeLen.at(e.id);
        if (e.ramp.valid) ej["ramp"] = {{"depart_s", e.ramp.departS}, {"gore_s", e.ramp.touchS}, {"climb", e.ramp.climb}, {"free_len", e.ramp.freeLen}, {"required_len", e.ramp.requiredLen}, {"ok", e.ramp.ok}, {"anchors", e.anchorLanes}};
        j["edges"][e.id] = ej;
    }
    for (const Surface& s : r.pavement.decks) j["decks"].push_back({{"area", s.area}, {"verts", s.verts.size()}, {"tris", s.tris.size()}, {"non_manifold", s.nonManifold}, {"cracks", s.cracks}, {"boundary_odd", s.boundaryOdd}});
    j["layers"] = {{"surface", setArea(r.pavement.surface)}, {"shoulder", setArea(r.pavement.shoulder)}, {"sidewalk", setArea(r.pavement.sidewalk)}, {"median", setArea(r.pavement.median)}};
    j["islands"] = r.pavement.islands; j["enclosed_blocks"] = r.pavement.enclosedBlocks; j["node_mismatch_after"] = r.nodeMismatchAfter;
    if (r.hasTerrain) j["terrain"] = {{"cut_m3", r.conform.cutM3}, {"fill_m3", r.conform.fillM3}, {"above", r.conform.above}, {"samples", r.conform.samples}, {"max_excess", r.conform.maxExcess}};
    for (const Check& c : invariants(r)) j["invariants"].push_back({{"name", c.name}, {"ok", c.ok}, {"detail", c.detail}});
    // cameras: one per anchored gore, 45 m back along the ramp, 6 m up, looking at the gore (engine world x, height, z)
    j["cameras"] = nlohmann::json::array();
    for (const EdgeSpec& e : r.graph.edges) {
        for (int k = 0; k < 2; ++k) {
            int gi = e.anchorIdx ? e.anchorIdx[k] : -1; if (gi < 0 || !e.ramp.valid) continue;
            size_t g = static_cast<size_t>(gi); double back = e.s[g] - 45.0; size_t bi = g; while (bi > 0 && e.s[bi] > back) --bi;
            Vec2 eye = e.xy[bi], tgt = e.xy[g]; double ze = e.z[bi] + 6.0, zt = e.z[g] + 1.0;
            j["cameras"].push_back({{"name", e.id + (k == 0 ? "_from_gore" : "_to_gore")}, {"eye", {eye.x, ze, eye.y}}, {"target", {tgt.x, zt, tgt.y}}});
        }
    }
    std::ofstream f(path); if (!f) return false; f << j.dump(1); return true;
}

}  // namespace lanelab
}  // namespace engine
