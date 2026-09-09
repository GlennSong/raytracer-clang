#include "engine/procgen/lanelab/road_graph_spec.h"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <stdexcept>

namespace engine {
namespace lanelab {

namespace {

const std::map<std::string, RoadClassSpec>& defaultClasses() {
    static const std::map<std::string, RoadClassSpec> k = [] {
        std::map<std::string, RoadClassSpec> m;
        auto add = [&](const char* n, double w, int f, int b, double sh, double sw, double med, int rank, double g, double win, double th) {
            RoadClassSpec c; c.name = n; c.lanes.w = w; c.lanes.fwd = f; c.lanes.back = b; c.shoulder = sh; c.sidewalk = sw; c.median = med;
            c.rank = rank; c.gMax = g; c.window = win; c.thick = th; m[n] = c;
        };
        add("freeway", 3.6, 3, 0, 2.5, 0.0, 0.0, 3, 0.03, 440.0, 1.2);
        add("arterial", 3.0, 2, 2, 0.0, 2.6, 0.0, 2, 0.06, 120.0, 0.6);
        add("local", 3.5, 1, 1, 0.0, 1.8, 0.0, 1, 0.09, 40.0, 0.5);
        add("ramp", 3.6, 1, 0, 1.2, 0.0, 0.0, 0, 0.09, 40.0, 1.0);
        add("divider", 0.0, 0, 0, 0.0, 0.0, 1.5, -1, 1.0, 1.0, 0.0);
        return m;
    }();
    return k;
}

double num(const nlohmann::json& j, const char* key, double d) { return j.contains(key) ? j.at(key).get<double>() : d; }

RampAnchor readAnchor(const nlohmann::json& j, const LaneLayout& hostLanes) {
    RampAnchor a;
    if (j.is_null()) return a;
    if (j.is_string()) { a.edge = j.get<std::string>(); return a; }
    a.edge = j.at("edge").get<std::string>(); a.side = j.value("side", std::string("right"));
    if (j.contains("s")) { a.hasStation = true; a.s = j.at("s").get<double>(); a.set = true; }
    else if (j.contains("x")) { a.axis = 'x'; a.coord = j.at("x").get<double>(); a.set = true; }
    else if (j.contains("y")) { a.axis = 'y'; a.coord = j.at("y").get<double>(); a.set = true; }
    else if (j.contains("at")) { a.axis = 'p'; a.px = j.at("at").at(0).get<double>(); a.py = j.at("at").at(1).get<double>(); a.set = true; }
    a.aux = num(j, "aux", 120.0); a.decel = num(j, "decel", 100.0); a.taper = num(j, "taper", 30.0 * hostLanes.w); a.approach = num(j, "approach", 150.0);
    return a;
}

}  // namespace

std::vector<Vec2> bezier(const std::array<Vec2, 4>& p, int n) {
    std::vector<Vec2> out; out.reserve(n);
    for (int i = 0; i < n; ++i) {
        double t = static_cast<double>(i) / (n - 1), u = 1 - t;
        out.push_back(p[0] * (u*u*u) + p[1] * (3*u*u*t) + p[2] * (3*u*t*t) + p[3] * (t*t*t));
    }
    return out;
}

std::vector<double> stations(const std::vector<Vec2>& xy) {
    std::vector<double> s(xy.size(), 0.0);
    for (size_t i = 1; i < xy.size(); ++i) s[i] = s[i-1] + distance(xy[i-1], xy[i]);
    return s;
}

std::vector<Vec2> resample(const std::vector<Vec2>& pts, double step) {
    if (pts.size() < 2) return pts;
    std::vector<double> s = stations(pts); double L = s.back();
    int n = std::max(2, static_cast<int>(std::lround(L / step)) + 1);
    std::vector<Vec2> out; out.reserve(n); size_t k = 1;
    for (int i = 0; i < n; ++i) {
        double target = L * i / (n - 1);
        while (k + 1 < pts.size() && s[k] < target) ++k;
        double seg = s[k] - s[k-1]; double f = seg > 1e-12 ? (target - s[k-1]) / seg : 0.0;
        out.push_back(lerp(pts[k-1], pts[k], std::clamp(f, 0.0, 1.0)));
    }
    return out;
}

namespace {
Vec2 tangentAt(const std::vector<Vec2>& xy, size_t i) {
    size_t j = std::min(i + 1, xy.size() - 1), k = i == 0 ? 0 : i - 1; Vec2 d = xy[j] - xy[k];
    return d.length() > 0 ? normalize(d) : Vec2(1, 0);
}
Vec2 interpolateAt(const std::vector<Vec2>& xy, const std::vector<double>& s, double station) {
    if (station <= s.front()) return xy.front();
    if (station >= s.back()) return xy.back();
    size_t k = std::upper_bound(s.begin(), s.end(), station) - s.begin();
    double f = (station - s[k-1]) / std::max(s[k] - s[k-1], 1e-12); return lerp(xy[k-1], xy[k], f);
}
}  // namespace

Vec2 resolvePoint(const nlohmann::json& spec, const RoadLabGraph& g) {
    if (spec.is_array()) return {spec.at(0).get<double>(), spec.at(1).get<double>()};
    const EdgeSpec* e = g.find(spec.at("edge").get<std::string>());
    if (!e || e->xy.empty()) throw std::runtime_error("point references unknown or later edge " + spec.at("edge").get<std::string>());
    const std::vector<Vec2>& xy = e->xy; Vec2 base; size_t i = 0;
    if (spec.contains("s")) base = interpolateAt(xy, e->s, spec.at("s").get<double>());
    else if (spec.contains("t")) base = interpolateAt(xy, e->s, spec.at("t").get<double>() * e->s.back());
    else if (spec.contains("x") || spec.contains("y")) {
        bool ax = spec.contains("x"); double v = spec.at(ax ? "x" : "y").get<double>(); double best = 1e300;
        for (size_t k = 0; k < xy.size(); ++k) { double d = std::fabs((ax ? xy[k].x : xy[k].y) - v); if (d < best) { best = d; i = k; } }
        base = xy[i];
    } else throw std::runtime_error("point on edge " + e->id + " needs one of s, t, x, y");
    double best = 1e300;
    for (size_t k = 0; k < xy.size(); ++k) { double d = distance(xy[k], base); if (d < best) { best = d; i = k; } }
    Vec2 tan = tangentAt(xy, i), nrm = perp(tan);
    return base + tan * num(spec, "along", 0.0) + nrm * num(spec, "normal", 0.0) + Vec2(num(spec, "dx", 0.0), num(spec, "dy", 0.0));
}

std::vector<Vec2> resolvePath(const nlohmann::json& p, const RoadLabGraph& g, double step) {
    std::string kind = p.value("type", std::string("polyline")); std::vector<Vec2> pts;
    if (kind == "offset") {
        const EdgeSpec* e = g.find(p.at("edge").get<std::string>()); if (!e) throw std::runtime_error("offset of unknown edge");
        double d = p.at("d").get<double>();
        for (size_t i = 0; i < e->xy.size(); ++i) pts.push_back(e->xy[i] + perp(tangentAt(e->xy, i)) * d);
    } else if (kind == "arc") {
        double cx = p.at("center").at(0).get<double>(), cy = p.at("center").at(1).get<double>(), r = p.at("r").get<double>();
        double a0 = p.at("a0").get<double>() * M_PI / 180.0, a1 = p.at("a1").get<double>() * M_PI / 180.0;
        int n = std::max(8, static_cast<int>(std::fabs(a1 - a0) * r / 2) + 1);
        for (int i = 0; i < n; ++i) { double a = a0 + (a1 - a0) * i / (n - 1); pts.emplace_back(cx + r * std::cos(a), cy + r * std::sin(a)); }
    } else {
        for (const auto& q : p.at("points")) pts.push_back(resolvePoint(q, g));
        if (kind == "bezier") {
            if (pts.size() != 4) throw std::runtime_error("a bezier path needs exactly 4 points");
            pts = bezier({pts[0], pts[1], pts[2], pts[3]});
        } else if (kind != "polyline") throw std::runtime_error("unknown path type " + kind);
    }
    if (p.value("reverse", false)) std::reverse(pts.begin(), pts.end());
    return resample(pts, step);
}

const EdgeSpec* RoadLabGraph::find(const std::string& id) const { auto it = index.find(id); return it == index.end() ? nullptr : &edges[it->second]; }
EdgeSpec* RoadLabGraph::find(const std::string& id) { auto it = index.find(id); return it == index.end() ? nullptr : &edges[it->second]; }

double RoadLabGraph::hw(const EdgeSpec& e) const {
    if (e.laneCount() == 0) return 0.0;
    double m = 0; for (const LaneSlot& l : laneSlots(cls(e), e.lanes)) m = std::max(m, std::fabs(l.offset));
    return m + e.lanes.w / 2;
}

std::vector<const EdgeSpec*> RoadLabGraph::paved() const {
    std::vector<const EdgeSpec*> out; for (const EdgeSpec& e : edges) if (e.laneCount() > 0) out.push_back(&e); return out;
}

std::array<double, 4> RoadLabGraph::bounds() const {
    if (terrain.hasBounds) return terrain.bounds;
    double x0 = 1e300, x1 = -1e300, y0 = 1e300, y1 = -1e300, m = 0;
    for (const EdgeSpec& e : edges) {
        const RoadClassSpec& c = cls(e); m = std::max(m, 4 * (hw(e) + c.sidewalk + c.shoulder));
        for (const Vec2& p : e.xy) { x0 = std::min(x0, p.x); x1 = std::max(x1, p.x); y0 = std::min(y0, p.y); y1 = std::max(y1, p.y); }
    }
    return {x0 - m, x1 + m, y0 - m, y1 + m};
}

RoadSpec bandsFor(const RoadClassSpec& c, const LaneLayout& L) {
    RoadSpec spec; spec.name = c.name;
    auto band = [](BandKind k, double w, int dir) { RoadBand b; b.kind = k; b.width = static_cast<float>(w); b.dir = static_cast<int8_t>(dir); return b; };
    if (c.sidewalk > 0) spec.bands.push_back(band(BandKind::Sidewalk, c.sidewalk, 0));
    if (c.shoulder > 0) spec.bands.push_back(band(BandKind::Shoulder, c.shoulder, 0));
    for (int k = 0; k < L.back; ++k) spec.bands.push_back(band(BandKind::Travel, L.w, -1));   // left side: backward lanes, outer first
    if (L.back > 0 && L.gap > 0) spec.bands.push_back(band(BandKind::Median, L.gap, 0));
    for (int k = 0; k < L.fwd; ++k) spec.bands.push_back(band(BandKind::Travel, L.w, +1));    // right side: forward lanes, inner first
    if (c.shoulder > 0) spec.bands.push_back(band(BandKind::Shoulder, c.shoulder, 0));
    if (c.sidewalk > 0) spec.bands.push_back(band(BandKind::Sidewalk, c.sidewalk, 0));
    return spec;
}

std::vector<LaneSlot> laneSlots(const RoadClassSpec& c, const LaneLayout& L) {
    RoadSpec spec = bandsFor(c, L); double W = spec.totalWidth(); std::vector<LaneSlot> out; int nf = 0, nb = 0;
    for (size_t i = 0; i < spec.bands.size(); ++i) {
        const RoadBand& b = spec.bands[i]; if (b.kind != BandKind::Travel) continue;
        double x0, x1; spec.bandSpan(static_cast<int>(i), x0, x1);
        double offset = W / 2 - (x0 + x1) / 2;                       // left of the spine is +normal
        // names: f0 is the innermost forward lane (nearest the spine/median), b0 the innermost backward lane
        std::string name = b.dir > 0 ? "f" + std::to_string(nf++) : "b" + std::to_string(L.back - 1 - nb++);
        out.push_back({name, offset, b.dir});
    }
    return out;
}

RoadLabGraph RoadLabGraph::fromJson(const nlohmann::json& spec, const std::string& baseDir) {
    RoadLabGraph g; g.name = spec.value("name", std::string("roads")); g.classes = defaultClasses();
    if (spec.contains("classes")) for (auto it = spec["classes"].begin(); it != spec["classes"].end(); ++it) {
        RoadClassSpec& c = g.classes[it.key()]; if (c.name.empty()) { c = g.classes.at("local"); c.name = it.key(); }
        const nlohmann::json& j = it.value();
        c.lanes.w = num(j, "w", c.lanes.w); c.lanes.fwd = static_cast<int>(num(j, "fwd", c.lanes.fwd)); c.lanes.back = static_cast<int>(num(j, "back", c.lanes.back));
        c.lanes.gap = num(j, "gap", c.lanes.gap); c.shoulder = num(j, "shoulder", c.shoulder); c.sidewalk = num(j, "sidewalk", c.sidewalk); c.median = num(j, "median", c.median);
        c.rank = static_cast<int>(num(j, "rank", c.rank)); c.gMax = num(j, "g_max", c.gMax); c.window = num(j, "window", c.window); c.thick = num(j, "thick", c.thick);
        if (j.contains("edges") && j["edges"].is_object()) {
            static const char* kRoleKey[5] = {"seam", "median", "vs_street", "elevated", "at_grade"};
            for (size_t i = 0; i < 5; ++i) {
                if (!j["edges"].contains(kRoleKey[i])) continue;
                const nlohmann::json& e = j["edges"][kRoleKey[i]]; BarrierSpec b;
                const std::string kind = e.value("kind", std::string("none"));
                b.kind = kind == "wall" ? BarrierKind::Wall : kind == "guardrail" ? BarrierKind::Guardrail : BarrierKind::None;
                b.h = num(e, "h", kind == "guardrail" ? 0.75 : 0.9);
                b.offset = num(e, "offset", 0.0); b.thick = num(e, "thick", kind == "guardrail" ? 0.1 : 0.4); b.set = true;
                c.edges[i] = b;
            }
        }
    }
    if (spec.contains("rules")) {
        const nlohmann::json& r = spec["rules"]; Rules& R = g.rules;
        R.step = num(r, "step", R.step); R.sameLevelDz = num(r, "same_level_dz", R.sameLevelDz); R.blendLen = num(r, "blend_len", R.blendLen); R.closing = num(r, "closing", R.closing);
        R.bridgeH = num(r, "bridge_h", R.bridgeH); R.pierSpacing = num(r, "pier_spacing", R.pierSpacing); R.slope = num(r, "slope", R.slope); R.conformW = num(r, "conform_w", R.conformW); R.underClearance = num(r, "under_clearance", R.underClearance); R.structureDepth = num(r, "structure_depth", R.structureDepth);
        R.rampLevelDz = num(r, "ramp_level_dz", R.rampLevelDz);
        R.skirt = num(r, "skirt", R.skirt); R.skirtDrop = num(r, "skirt_drop", R.skirtDrop); R.endpointTol = num(r, "endpoint_tol", R.endpointTol);
    }
    if (spec.contains("terrain")) {
        const nlohmann::json& t = spec["terrain"]; TerrainSpec& T = g.terrain; T.type = t.value("type", std::string("flat"));
        if (t.contains("bounds")) { T.hasBounds = true; for (int i = 0; i < 4; ++i) T.bounds[static_cast<size_t>(i)] = t["bounds"].at(i).get<double>(); }
        T.res = num(t, "res", 2.0); T.seed = static_cast<int>(num(t, "seed", 1));
        if (t.contains("octaves")) for (const auto& o : t["octaves"]) T.octaves.emplace_back(o.at(0).get<double>(), o.at(1).get<double>());
        if (t.contains("valleys")) for (const auto& v : t["valleys"]) { TerrainSpec::Valley V; V.alongY = v.contains("y") && !v.contains("x"); V.c = V.alongY ? v.at("y").get<double>() : v.at("x").get<double>(); V.width = num(v, "width", 40.0); V.depth = num(v, "depth", 0.0); T.valleys.push_back(V); }
        if (t.contains("hills")) for (const auto& h : t["hills"]) T.hills.push_back({num(h, "x", 0), num(h, "y", 0), num(h, "r", 1), num(h, "h", 0)});
        if (t.contains("tilt")) { T.hasTilt = true; const auto& k = t["tilt"]; T.dzdx = num(k, "dzdx", 0); T.x0 = num(k, "x0", 0); T.dzdy = num(k, "dzdy", 0); T.y0 = num(k, "y0", 0); }
        if (t.contains("file")) { T.file = t["file"].get<std::string>(); if (!T.file.empty() && T.file[0] != '/') T.file = baseDir + "/" + T.file; }
    }
    for (const auto& ej : spec.at("edges")) {
        EdgeSpec e; e.id = ej.at("id").get<std::string>(); e.cls = ej.at("class").get<std::string>();
        if (g.index.count(e.id)) throw std::runtime_error("duplicate edge id " + e.id);
        if (!g.classes.count(e.cls)) throw std::runtime_error("edge " + e.id + ": unknown class " + e.cls);
        const RoadClassSpec& c = g.classes.at(e.cls); e.lanes = c.lanes;
        if (ej.contains("lanes")) {
            const auto& l = ej["lanes"]; e.lanes.w = num(l, "w", e.lanes.w); e.lanes.fwd = static_cast<int>(num(l, "fwd", e.lanes.fwd)); e.lanes.back = static_cast<int>(num(l, "back", e.lanes.back));
            e.lanes.gap = num(l, "gap", e.lanes.gap); e.lanes.dovetail = num(l, "dovetail", e.lanes.dovetail);
            if (l.contains("pockets")) for (const auto& p : l["pockets"]) {
                PocketSpec P; P.id = p.at("id").get<std::string>(); P.kind = p.value("kind", std::string("turn")); P.side = p.value("side", std::string("right"));
                P.dir = p.value("dir", std::string("fwd")) == "back" ? -1 : +1; P.s0 = p.at("s0").get<double>(); P.s1 = p.at("s1").get<double>();
                P.taper = num(p, "taper", 10 * e.lanes.w); P.taperOut = num(p, "taper_out", 0.0); e.pockets.push_back(P);
            }
        }
        if (ej.contains("z_min")) { e.hasZMin = true; e.zMin = ej["z_min"].get<double>(); }
        if (ej.contains("lots_range")) { e.lotsFrom = ej["lots_range"].at(0).get<double>(); e.lotsTo = ej["lots_range"].at(1).get<double>(); }
        if (ej.contains("floor")) for (const auto& fp : ej["floor"]) e.floorPts.push_back({fp.at(0).get<double>(), fp.at(1).get<double>(), fp.at(2).get<double>(), fp.size() > 3 ? fp.at(3).get<double>() : 0.0});
        auto hostLanes = [&](const nlohmann::json& a) { if (a.is_object()) { const EdgeSpec* h = g.find(a.at("edge").get<std::string>()); if (h) return h->lanes; } return e.lanes; };
        if (ej.contains("from")) e.from = readAnchor(ej["from"], hostLanes(ej["from"]));
        if (ej.contains("to")) e.to = readAnchor(ej["to"], hostLanes(ej["to"]));
        for (const std::string& ref : {e.from.edge, e.to.edge}) if (!ref.empty() && !g.index.count(ref)) throw std::runtime_error("edge " + e.id + ": from/to references unknown or later edge " + ref);
        e.path = ej.at("path"); e.xy = resolvePath(e.path, g, g.rules.step); e.s = stations(e.xy);
        g.index[e.id] = static_cast<int>(g.edges.size()); g.edges.push_back(std::move(e));
    }
    if (spec.contains("connectors")) for (const auto& c : spec["connectors"]) g.connectors.push_back({c.at("from").get<std::string>(), c.at("to").get<std::string>(), num(c, "to_s", 0.0)});
    if (g.edges.empty()) throw std::runtime_error("graph has no edges");
    return g;
}

RoadLabGraph RoadLabGraph::load(const std::string& path) {
    std::ifstream f(path); if (!f) throw std::runtime_error("cannot open " + path);
    nlohmann::json j; f >> j; std::string dir = path.find('/') == std::string::npos ? "." : path.substr(0, path.rfind('/'));
    return fromJson(j, dir);
}

}  // namespace lanelab
}  // namespace engine
