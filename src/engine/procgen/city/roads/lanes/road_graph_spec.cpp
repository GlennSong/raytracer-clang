#include "engine/procgen/city/roads/lanes/road_graph_spec.h"
#include "engine/procgen/city/roads/lanes/polyline_ops.h"   // pointAt, tangentAt
#include "engine/asset_root.h"                               // assetPath: the road class table's file
#include "engine/bundle/bundle.h"                            // fnv1aStr: the table's hash, for the bake key

#include <algorithm>
#include <cmath>
#include <fstream>
#include <functional>
#include <iterator>
#include <set>
#include <stdexcept>

namespace engine {
namespace roads::lanes {

// THE ROAD CLASS TABLE (ADR-0276, #117 item 18; a data file since ADR-0438): every class's section, grade, profile and
// the builder's treatment of it, once, in assets/data/road_classes.json (which says why each number is what it is). A
// graph names its classes from it (fromJson starts each class at its row), the city planner writes its rows into every
// graph it plans (plan_scene: the brief's widths over them), the island adds its mountain row, DesignRules reads the
// grades. Read here, once; no copy of the rows is kept in code.

namespace {

double num(const nlohmann::json& j, const char* key, double d) { return j.contains(key) ? j.at(key).get<double>() : d; }

// The keys of a class row's `edges`, by EdgeRole.
constexpr const char* kEdgeRoleKey[5] = {"seam", "median", "vs_street", "elevated", "at_grade"};

// The table as the file states it, its default row's name and the hash of its bytes: read once.
struct RoadClassTable {
    std::map<std::string, RoadClassSpec> rows;
    std::string defaultName;
    uint64_t hash = 0;
};
const RoadClassTable& roadClassTable() {
    static const RoadClassTable table = [] {
        const std::string path = assetPath(kRoadClassTableFile);
        std::ifstream in(path, std::ios::binary);
        if (!in) throw std::runtime_error("the road class table " + path + " is missing: no road can be built without it");
        const std::string text((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
        RoadClassTable t;
        try {
            t.rows = parseRoadClassTable(nlohmann::json::parse(text), &t.defaultName);
        } catch (const std::exception& e) {
            throw std::runtime_error("the road class table " + path + ": " + e.what());
        }
        t.hash = bundle::fnv1aStr(text);
        return t;
    }();
    return table;
}

}  // namespace

void readRoadClassRow(RoadClassSpec& c, const nlohmann::json& j) {
    c.lanes.w = num(j, "w", c.lanes.w); c.lanes.fwd = static_cast<int>(num(j, "fwd", c.lanes.fwd)); c.lanes.back = static_cast<int>(num(j, "back", c.lanes.back));
    c.lanes.gap = num(j, "gap", c.lanes.gap); c.shoulder = num(j, "shoulder", c.shoulder); c.sidewalk = num(j, "sidewalk", c.sidewalk); c.median = num(j, "median", c.median);
    c.lanes.parking = num(j, "parking", c.lanes.parking);
    c.rank = static_cast<int>(num(j, "rank", c.rank)); c.gMax = num(j, "g_max", c.gMax); c.window = num(j, "window", c.window); c.balance = num(j, "balance", c.balance); c.thick = num(j, "thick", c.thick);
    if (j.contains("track") && j["track"].is_boolean()) c.track = j["track"].get<bool>();
    if (j.contains("footway") && j["footway"].is_boolean()) c.footway = j["footway"].get<bool>();
    if (j.contains("junction_markings") && j["junction_markings"].is_boolean()) c.junctionMarkings = j["junction_markings"].get<bool>();
    if (j.contains("limited_access") && j["limited_access"].is_boolean()) c.limitedAccess = j["limited_access"].get<bool>();
    if (j.contains("default_barriers") && j["default_barriers"].is_boolean()) c.defaultBarriers = j["default_barriers"].get<bool>();
    if (j.contains("plan_colour") && j["plan_colour"].is_string()) c.planColour = j["plan_colour"].get<std::string>();
    if (j.contains("engine_class") && j["engine_class"].is_string()) {
        const std::string name = j["engine_class"].get<std::string>();
        if (!parseRoadClass(name, c.engineClass))
            throw std::runtime_error("road class " + c.name + ": engine_class \"" + name + "\" is none of the engine's classes "
                                     "(local, collector, arterial, freeway, ramp, alley)");
    }
    if (j.contains("edges") && j["edges"].is_object()) {
        for (size_t i = 0; i < 5; ++i) {
            if (!j["edges"].contains(kEdgeRoleKey[i])) continue;
            const nlohmann::json& e = j["edges"][kEdgeRoleKey[i]]; BarrierSpec b;
            const std::string kind = e.value("kind", std::string("none"));
            b.kind = kind == "wall" ? BarrierKind::Wall : kind == "guardrail" ? BarrierKind::Guardrail : BarrierKind::None;
            b.h = num(e, "h", kind == "guardrail" ? 0.75 : 0.9);
            b.offset = num(e, "offset", 0.0); b.thick = num(e, "thick", kind == "guardrail" ? 0.1 : 0.4); b.minDrop = num(e, "min_drop", 0.0); b.set = true;
            c.edges[i] = b;
        }
    }
}

std::map<std::string, RoadClassSpec> parseRoadClassTable(const nlohmann::json& doc, std::string* defaultName) {
    if (!doc.is_object() || !doc.contains("classes") || !doc["classes"].is_object())
        throw std::runtime_error("no \"classes\": {name: row} object");
    const nlohmann::json& rows = doc["classes"];
    std::map<std::string, RoadClassSpec> out;
    // a row that is `like` another is read after it, whatever order the file lists them in
    std::set<std::string> reading;
    std::function<void(const std::string&)> read = [&](const std::string& name) {
        if (out.count(name)) return;
        if (!rows.contains(name)) throw std::runtime_error("a row is like \"" + name + "\", which the table does not have");
        if (!reading.insert(name).second) throw std::runtime_error("row \"" + name + "\" is like itself, through its `like`");
        const nlohmann::json& row = rows[name];
        RoadClassSpec c;
        if (row.contains("like")) {
            const std::string base = row["like"].get<std::string>();
            read(base);
            c = out.at(base);
        }
        c.name = name;
        readRoadClassRow(c, row);
        out[name] = c;
    };
    for (nlohmann::json::const_iterator it = rows.begin(); it != rows.end(); ++it) read(it.key());
    const std::string fallback = doc.value("default", std::string());
    if (!out.count(fallback)) throw std::runtime_error("its \"default\" row \"" + fallback + "\" is not one of its classes");
    if (defaultName) *defaultName = fallback;
    return out;
}

const std::map<std::string, RoadClassSpec>& roadClasses() { return roadClassTable().rows; }
const std::string& defaultRoadClassName() { return roadClassTable().defaultName; }
uint64_t roadClassTableHash() { return roadClassTable().hash; }

const RoadClassSpec& roadClass(const std::string& name) {
    const std::map<std::string, RoadClassSpec>& t = roadClasses();
    const std::map<std::string, RoadClassSpec>::const_iterator it = t.find(name);
    return it != t.end() ? it->second : t.at(defaultRoadClassName());
}

nlohmann::json classJson(const RoadClassSpec& c) {
    nlohmann::json j = {{"w", c.lanes.w}, {"fwd", c.lanes.fwd}, {"back", c.lanes.back}, {"shoulder", c.shoulder},
                        {"sidewalk", c.sidewalk}, {"g_max", c.gMax}, {"rank", c.rank}, {"thick", c.thick},
                        {"window", c.window}};
    if (c.median != 0) j["median"] = c.median;
    if (c.lanes.gap != 0) j["gap"] = c.lanes.gap;
    if (c.lanes.parking != 0) j["parking"] = c.lanes.parking;
    if (c.balance != 0) j["balance"] = c.balance;
    if (c.track) j["track"] = true;
    if (c.footway) j["footway"] = true;
    {
        // each written only where it differs from what a file leaving it out is read with: the table's row of that
        // name, or (a class the table does not have) its default row's
        const RoadClassSpec& usual = roadClass(c.name);
        if (c.junctionMarkings != usual.junctionMarkings) j["junction_markings"] = c.junctionMarkings;
        if (c.engineClass != usual.engineClass) j["engine_class"] = roadClassName(c.engineClass);
        if (c.limitedAccess != usual.limitedAccess) j["limited_access"] = c.limitedAccess;
        if (c.defaultBarriers != usual.defaultBarriers) j["default_barriers"] = c.defaultBarriers;
        if (c.planColour != usual.planColour) j["plan_colour"] = c.planColour;
    }
    const char* const* kRoleKey = kEdgeRoleKey;
    for (size_t i = 0; i < 5; ++i) {
        const BarrierSpec& b = c.edges[i];
        if (!b.set) continue;
        nlohmann::json e = {{"kind", b.kind == BarrierKind::Wall ? "wall" : b.kind == BarrierKind::Guardrail ? "guardrail" : "none"}};
        if (b.kind != BarrierKind::None) { e["h"] = b.h; e["thick"] = b.thick; if (b.offset != 0) e["offset"] = b.offset; }
        if (b.minDrop != 0) e["min_drop"] = b.minDrop;
        j["edges"][kRoleKey[i]] = e;
    }
    return j;
}

namespace {

RampAnchor readAnchor(const nlohmann::json& j, const LaneLayout& hostLanes) {
    RampAnchor a;
    if (j.is_null()) return a;
    if (j.is_string()) { a.edge = j.get<std::string>(); return a; }
    a.edge = j.at("edge").get<std::string>(); a.side = j.value("side", std::string("right"));
    // the kerb lane by name (#282: a road side is the kerb or the offside, never left or right): "kerb" the forward
    // lanes' (on the spine's right in plan coordinates), "back-kerb" the backward lanes'; "offside" the forward lanes'
    // lane on the other side, by the centre line (on a one-way road, the lane nearest a freeway on that side: ADR-0413)
    if (a.side == "kerb") a.side = "right";
    else if (a.side == "back-kerb") a.side = "back-right";
    else if (a.side == "offside") a.side = "left";
    if (j.contains("s")) { a.hasStation = true; a.s = j.at("s").get<double>(); a.set = true; }
    else if (j.contains("x")) { a.axis = 'x'; a.coord = j.at("x").get<double>(); a.set = true; }
    else if (j.contains("y")) { a.axis = 'y'; a.coord = j.at("y").get<double>(); a.set = true; }
    else if (j.contains("at")) { a.axis = 'p'; a.px = j.at("at").at(0).get<double>(); a.py = j.at("at").at(1).get<double>(); a.set = true; }
    a.aux = num(j, "aux", 120.0); a.decel = num(j, "decel", 100.0); a.approach = num(j, "approach", 150.0);
    // the taper the file states; none stated, or zero: the default (the one place it is applied)
    a.taper = num(j, "taper", 0.0);
    if (!(a.taper > 0)) a.taper = kRampTaperLaneWidths * hostLanes.w;
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

Vec2 resolvePoint(const nlohmann::json& spec, const RoadLabGraph& g) {
    if (spec.is_array()) return {spec.at(0).get<double>(), spec.at(1).get<double>()};
    const EdgeSpec* e = g.find(spec.at("edge").get<std::string>());
    if (!e || e->xy.empty()) throw std::runtime_error("point references unknown or later edge " + spec.at("edge").get<std::string>());
    const std::vector<Vec2>& xy = e->xy; Vec2 base; size_t i = 0;
    if (spec.contains("s")) base = pointAt(xy, e->s, spec.at("s").get<double>());
    else if (spec.contains("t")) base = pointAt(xy, e->s, spec.at("t").get<double>() * e->s.back());
    else if (spec.contains("x") || spec.contains("y")) {
        bool ax = spec.contains("x"); double v = spec.at(ax ? "x" : "y").get<double>(); double best = 1e300;
        for (size_t k = 0; k < xy.size(); ++k) { double d = std::fabs((ax ? xy[k].x : xy[k].y) - v); if (d < best) { best = d; i = k; } }
        base = xy[i];
    } else throw std::runtime_error("point on edge " + e->id + " needs one of s, t, x, y");
    double best = 1e300;
    for (size_t k = 0; k < xy.size(); ++k) { double d = distance(xy[k], base); if (d < best) { best = d; i = k; } }
    Vec2 tan = tangentAt(xy, i, 0.0), nrm = perp(tan);
    return base + tan * num(spec, "along", 0.0) + nrm * num(spec, "normal", 0.0) + Vec2(num(spec, "dx", 0.0), num(spec, "dy", 0.0));
}

std::vector<Vec2> resolvePath(const nlohmann::json& p, const RoadLabGraph& g, double step) {
    std::string kind = p.value("type", std::string("polyline")); std::vector<Vec2> pts;
    if (kind == "offset") {
        const EdgeSpec* e = g.find(p.at("edge").get<std::string>()); if (!e) throw std::runtime_error("offset of unknown edge");
        double d = p.at("d").get<double>();
        for (size_t i = 0; i < e->xy.size(); ++i) pts.push_back(e->xy[i] + perp(tangentAt(e->xy, i, 0.0)) * d);
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
bool RoadLabGraph::designed() const {
    bool any = false;
    for (const EdgeSpec& e : edges) {
        if (e.xy.size() < 2 && !e.isRamp()) continue;
        if (e.profile.empty()) return false;
        any = true;
    }
    return any;
}
const CrossingSpec* RoadLabGraph::declaredCrossing(const std::string& a, const std::string& b, const Vec2& p, double within) const {
    const CrossingSpec* best = nullptr;
    double bd = within;
    for (const CrossingSpec& c : crossings) {
        if (!((c.a == a && c.b == b) || (c.a == b && c.b == a))) continue;
        const double d = std::hypot(c.at.x - p.x, c.at.y - p.y);
        if (d <= bd) { bd = d; best = &c; }
    }
    return best;
}

std::vector<const EdgeSpec*> RoadLabGraph::paved() const {
    std::vector<const EdgeSpec*> out; for (const EdgeSpec& e : edges) if (e.laneCount() > 0) out.push_back(&e); return out;
}

RoadClass classOf(const RoadLabGraph& g, const EdgeSpec& e) {
    return e.isRamp() ? RoadClass::Ramp : g.cls(e).engineClass;
}

RoadSpec streetSection(const RoadLabGraph& g, const EdgeSpec& e) {
    const RoadClassSpec& c = g.cls(e);
    const double carriageway = 2.0 * g.carriagewayHalfWidth(e);
    const int perDir = std::max(1, std::max(c.lanes.fwd, c.lanes.back));
    return roadSpecStreetParking(carriageway, perDir, c.sidewalk, 0.15, e.lanes.parking, kMinTravelLane);
}

DrivenLine drivenLine(const RoadLabGraph& g, const EdgeSpec& e, const LaneSlot& slot) {
    const double half = e.lanes.w * 0.5;
    double lo = slot.offset - half, hi = slot.offset + half;
    double band = 0, width = 0;
    // (the bands lie the same distance either side of the carriageway's middle: the one band's inner edge bounds both)
    if (g.cls(e).carriesCars() && e.lanes.parking > 0 && roadSpecParkingBand(streetSection(g, e), +1, band, width)) {
        const double inner = std::fabs(band) - width * 0.5;
        hi = std::min(hi, inner);
        lo = std::max(lo, -inner);
    }
    if (!(hi > lo)) return DrivenLine{slot.offset, e.lanes.w};
    return DrivenLine{(lo + hi) * 0.5, hi - lo};
}

double kerbParking(const RoadLabGraph& g, const EdgeSpec& e, bool* fits) {
    double offset = 0, width = 0;
    const bool has = e.lanes.parking > 0 && roadSpecParkingBand(streetSection(g, e), +1, offset, width);
    if (fits) *fits = has || e.lanes.parking <= 0;
    return has ? width : 0.0;
}

std::array<double, 4> RoadLabGraph::bounds() const {
    if (terrain.hasBounds) return terrain.bounds;
    double x0 = 1e300, x1 = -1e300, y0 = 1e300, y1 = -1e300, m = 0;
    for (const EdgeSpec& e : edges) {
        m = std::max(m, 4 * sidewalkOuterHalfWidth(e));
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
    if (lanesMedianGap(L.fwd, L.back, L.gap) > 0) spec.bands.push_back(band(BandKind::Median, L.gap, 0));
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
    RoadLabGraph g; g.name = spec.value("name", std::string("roads")); g.classes = roadClasses();
    if (spec.contains("classes")) for (auto it = spec["classes"].begin(); it != spec["classes"].end(); ++it) {
        // a class the table does not have starts from its default row (the local street's) -- but parks only where it
        // says so: that row's kerb parking belongs to that row's section (ADR-0418). Nor does it take that row's colour
        // on the plan picture: it is drawn as any class with none of its own.
        RoadClassSpec& c = g.classes[it.key()];
        if (c.name.empty()) { c = g.classes.at(defaultRoadClassName()); c.name = it.key(); c.lanes.parking = 0.0; c.planColour.clear(); }
        readRoadClassRow(c, it.value());   // the one reader of a class row (the table's file is read by it too)
    }
    if (spec.contains("rules")) {
        const nlohmann::json& r = spec["rules"]; Rules& R = g.rules;
        R.step = num(r, "step", R.step); R.sameLevelDz = num(r, "same_level_dz", R.sameLevelDz); R.blendLen = num(r, "blend_len", R.blendLen); R.closing = num(r, "closing", R.closing);
        R.bridgeH = num(r, "bridge_h", R.bridgeH); R.pierSpacing = num(r, "pier_spacing", R.pierSpacing); R.slope = num(r, "slope", R.slope); R.conformW = num(r, "conform_w", R.conformW); R.underClearance = num(r, "under_clearance", R.underClearance); R.structureDepth = num(r, "structure_depth", R.structureDepth);
        R.rampLevelDz = num(r, "ramp_level_dz", R.rampLevelDz);
        R.storage = num(r, "storage", R.storage);
        if (r.contains("freeway_separates") && r["freeway_separates"].is_boolean()) R.freewaySeparates = r["freeway_separates"].get<bool>();
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
        if (t.contains("base")) T.base = t["base"];
        if (t.contains("relief_fade")) { T.hasFade = true; const auto& k = t["relief_fade"]; T.fadeX = k.at("centre").at(0).get<double>(); T.fadeY = k.at("centre").at(1).get<double>(); T.fadeR0 = num(k, "r0", 0); T.fadeR1 = num(k, "r1", 1); }
        if (t.contains("relief_calm")) { T.hasCalm = true; const auto& k = t["relief_calm"]; T.calmDx = k.at("dir").at(0).get<double>(); T.calmDy = k.at("dir").at(1).get<double>(); T.calmFrom = num(k, "from", 0); T.calmTo = num(k, "to", 1); }
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
            e.lanes.parking = num(l, "parking", e.lanes.parking);
            if (l.contains("pockets")) for (const auto& p : l["pockets"]) {
                PocketSpec P; P.id = p.at("id").get<std::string>(); P.kind = p.value("kind", std::string("turn")); P.side = p.value("side", std::string("right"));
                P.dir = p.value("dir", std::string("fwd")) == "back" ? -1 : +1; P.s0 = p.at("s0").get<double>(); P.s1 = p.at("s1").get<double>();
                // the taper the file states; none stated, or zero: the default (the one place it is applied)
                P.taper = num(p, "taper", 0.0);
                if (!(P.taper > 0)) P.taper = kPocketTaperLaneWidths * e.lanes.w;
                P.taperOut = num(p, "taper_out", 0.0); e.pockets.push_back(P);
            }
        }
        if (ej.contains("platforms")) for (const auto& pj : ej["platforms"]) {
            PlatformSpec P; P.name = pj.value("name", std::string()); P.s0 = num(pj, "s0", 0.0); P.s1 = num(pj, "s1", 0.0);
            const std::string side = pj.value("side", std::string("both")); P.side = side == "left" ? +1 : side == "right" ? -1 : 0;
            P.w = num(pj, "w", P.w); P.h = num(pj, "h", P.h);
            if (P.s1 > P.s0) e.platforms.push_back(P);
        }
        if (ej.contains("z_min")) { e.hasZMin = true; e.zMin = ej["z_min"].get<double>(); }
        if (ej.contains("profile")) { e.profileStep = ej["profile"].value("step", 1.0); e.profile = ej["profile"].at("z").get<std::vector<double>>(); }
        if (ej.contains("lots_range")) { e.lotsFrom = ej["lots_range"].at(0).get<double>(); e.lotsTo = ej["lots_range"].at(1).get<double>(); }
        if (ej.contains("floor")) for (const auto& fp : ej["floor"]) e.floorPts.push_back({fp.at(0).get<double>(), fp.at(1).get<double>(), fp.at(2).get<double>(), fp.size() > 3 ? fp.at(3).get<double>() : 0.0});
        e.speedLimit = num(ej, "speed_limit", 0.0);
        e.speedLimitHand = ej.value("speed_limit_hand", false);
        e.frontageOf = ej.value("frontage_of", std::string());
        if (!(e.speedLimit >= 0.0)) throw std::runtime_error("edge " + e.id + ": speed_limit must be a speed in m/s, 0 or more");
        auto hostLanes = [&](const nlohmann::json& a) { if (a.is_object()) { const EdgeSpec* h = g.find(a.at("edge").get<std::string>()); if (h) return h->lanes; } return e.lanes; };
        if (ej.contains("landing")) {
            const nlohmann::json& lj = ej["landing"];
            RampLanding& L = e.landing;
            L.declared = true;
            L.street = lj.value("street", std::string());
            L.joins = lj.value("joins", std::string());
            L.carriageway = lj.value("carriageway", std::string());
            L.half = lj.value("half", std::string());
            if (lj.contains("crossing") && lj["crossing"].is_object()) {
                const nlohmann::json& cj = lj["crossing"];
                L.crossingKind = cj.value("kind", std::string());
                L.crossingStreet = cj.value("street", std::string());
                if (cj.contains("at")) L.crossingAt = Vec2(cj["at"].at(0).get<double>(), cj["at"].at(1).get<double>());
                L.clearance = num(cj, "clearance", 0.0);
            } else {
                L.crossingKind = "none";
            }
            if (L.joins != "merge" && L.joins != "junction") throw std::runtime_error("edge " + e.id + ": landing.joins must be \"merge\" or \"junction\", not \"" + L.joins + "\"");
        }
        if (ej.contains("from")) e.from = readAnchor(ej["from"], hostLanes(ej["from"]));
        if (ej.contains("to")) e.to = readAnchor(ej["to"], hostLanes(ej["to"]));
        for (const std::string& ref : {e.from.edge, e.to.edge}) if (!ref.empty() && !g.index.count(ref)) throw std::runtime_error("edge " + e.id + ": from/to references unknown or later edge " + ref);
        e.path = ej.at("path"); e.xy = resolvePath(e.path, g, g.rules.step); e.s = stations(e.xy);
        g.index[e.id] = static_cast<int>(g.edges.size()); g.edges.push_back(std::move(e));
    }
    if (spec.contains("connectors")) for (const auto& c : spec["connectors"]) g.connectors.push_back({c.at("from").get<std::string>(), c.at("to").get<std::string>(), num(c, "to_s", 0.0)});
    if (spec.contains("crossings")) {
        g.declaresCrossings = true;
        for (const auto& c : spec["crossings"]) {
            CrossingSpec x;
            x.a = c.at("a").get<std::string>(); x.b = c.at("b").get<std::string>();
            x.kind = c.value("kind", std::string("at_grade")); x.over = c.value("over", std::string());
            x.at = Vec2(c.at("at").at(0).get<double>(), c.at("at").at(1).get<double>());
            x.clearance = num(c, "clearance", -1.0);
            if (x.kind != "bridge" && x.kind != "at_grade") throw std::runtime_error("crossing " + x.a + "/" + x.b + ": unknown kind " + x.kind);
            // (a bridge may leave `over` empty: the design stage chooses by its own rule, and designedGraph writes the choice in)
            if (x.kind == "bridge" && !x.over.empty() && x.over != x.a && x.over != x.b) throw std::runtime_error("crossing " + x.a + "/" + x.b + ": over names neither road");
            g.crossings.push_back(x);
        }
    }
    // the file names its streets by id; a junction's arm and a street's carry-on hold the street's place in the list
    std::map<std::string, uint32_t> streetAt;
    if (spec.contains("streets"))
        for (std::size_t i = 0; i < spec["streets"].size(); ++i) streetAt[spec["streets"][i].at("id").get<std::string>()] = static_cast<uint32_t>(i);
    if (spec.contains("junctions")) {
        // THE CONTROL PLAN (ModelJunction): read as written; a road an arm names must exist, and where the list names
        // controls every junction must name one (a junction left without one is the plan's fault, never filled in here).
        // A list naming none is the road model's layout alone: it declares no control.
        g.listsJunctions = true;
        for (const nlohmann::json& jj : spec["junctions"]) g.declaresJunctions = g.declaresJunctions || jj.contains("control");
        for (const nlohmann::json& jj : spec["junctions"]) {
            ModelJunction pj;
            pj.id = jj.at("id").get<std::string>();
            pj.control = jj.value("control", std::string());
            pj.why = jj.value("why", std::string());
            pj.hand = jj.value("hand", false);
            if (jj.contains("at")) pj.at = Vec2(jj["at"].at(0).get<double>(), jj["at"].at(1).get<double>());
            if (g.declaresJunctions && pj.control.empty()) throw std::runtime_error("junction " + pj.id + ": names no control");
            for (const nlohmann::json& aj : jj.at("arms")) {
                ModelArm a;
                a.road = aj.at("road").get<std::string>();
                a.station = aj.at("at").get<double>();
                const std::string spine = aj.at("spine").get<std::string>();
                if (spine != "in" && spine != "out") throw std::runtime_error("junction " + pj.id + ": arm " + a.road + ": spine is \"" + spine + "\", expected \"in\" or \"out\"");
                a.spineIn = spine == "in";
                if (aj.contains("regulations")) a.regulations = aj["regulations"].get<std::vector<std::string>>();
                if (!g.index.count(a.road)) throw std::runtime_error("junction " + pj.id + ": arm names unknown road " + a.road);
                // the street it is, by id: one the file's "streets" list holds (an arm naming no street states none; one
                // naming a street the file does not have is refused, like any other unknown id)
                if (aj.contains("street")) {
                    const std::string streetId = aj["street"].get<std::string>();
                    const std::map<std::string, uint32_t>::const_iterator street = streetAt.find(streetId);
                    if (street == streetAt.end())
                        throw std::runtime_error("junction " + pj.id + ": arm " + a.road + " names unknown street " + streetId);
                    a.street = ModelStreetId(street->second);
                }
                if (aj.contains("heading")) a.heading = aj["heading"].get<double>();
                pj.arms.push_back(a);
            }
            // (each arm once: the list is an order of the junction's arms, ADR-0427 -- the same road end twice is no ring)
            for (std::size_t x = 0; x < pj.arms.size(); ++x)
                for (std::size_t y = x + 1; y < pj.arms.size(); ++y)
                    if (pj.arms[x].road == pj.arms[y].road && pj.arms[x].station == pj.arms[y].station && pj.arms[x].spineIn == pj.arms[y].spineIn)
                        throw std::runtime_error("junction " + pj.id + ": arm " + pj.arms[x].road + " at " + std::to_string(pj.arms[x].station) +
                                                 " is listed twice (arms " + std::to_string(x) + " and " + std::to_string(y) + ")");
            if (jj.contains("through"))
                for (const nlohmann::json& tj : jj["through"]) {
                    const std::array<int, 2> pair = {tj.at(0).get<int>(), tj.at(1).get<int>()};
                    for (int k : pair)
                        if (k < 0 || k >= static_cast<int>(pj.arms.size()))
                            throw std::runtime_error("junction " + pj.id + ": through names arm " + std::to_string(k) + ", which it does not have");
                    if (pair[0] == pair[1]) throw std::runtime_error("junction " + pj.id + ": through pairs arm " + std::to_string(pair[0]) + " with itself");
                    pj.through.push_back({static_cast<uint32_t>(pair[0]), static_cast<uint32_t>(pair[1])});
                }
            if (jj.contains("inside"))
                for (const nlohmann::json& sj : jj["inside"]) {
                    ModelStretch st;
                    st.road = sj.at("road").get<std::string>();
                    st.from = sj.at("from").get<double>();
                    st.to = sj.at("to").get<double>();
                    if (!g.index.count(st.road)) throw std::runtime_error("junction " + pj.id + ": inside names unknown road " + st.road);
                    if (!(st.from <= st.to)) throw std::runtime_error("junction " + pj.id + ": inside stretch of " + st.road + " runs backwards");
                    pj.inside.push_back(st);
                }
            if (jj.contains("nodes"))
                for (const nlohmann::json& nj : jj["nodes"]) {
                    ModelNode n;
                    n.at = Vec2(nj.at("at").at(0).get<double>(), nj.at("at").at(1).get<double>());
                    for (const nlohmann::json& pt : nj.at("points")) {
                        ModelNodePoint p;
                        p.road = pt.at("road").get<std::string>();
                        p.station = pt.at("at").get<double>();
                        if (!g.index.count(p.road)) throw std::runtime_error("junction " + pj.id + ": a node names unknown road " + p.road);
                        n.points.push_back(p);
                    }
                    pj.nodes.push_back(n);
                }
            g.junctions.push_back(pj);
        }
    }
    if (spec.contains("streets")) {
        // THE STREETS (ModelStreet): read as written; the road must exist, the stretch run forwards, and each end name a
        // junction of the list (or none)
        g.listsStreets = true;
        std::map<std::string, uint32_t> junctionAt;
        for (std::size_t j = 0; j < g.junctions.size(); ++j) junctionAt[g.junctions[j].id] = static_cast<uint32_t>(j);
        for (const nlohmann::json& sj : spec["streets"]) {
            ModelStreet st;
            st.id = sj.at("id").get<std::string>();
            st.road = sj.at("road").get<std::string>();
            st.from = sj.at("from").get<double>();
            st.to = sj.at("to").get<double>();
            if (!g.index.count(st.road)) throw std::runtime_error("street " + st.id + ": names unknown road " + st.road);
            if (!(st.from <= st.to)) throw std::runtime_error("street " + st.id + ": runs backwards along " + st.road);
            if (sj.contains("junctions"))
                for (std::size_t k = 0; k < 2 && k < sj["junctions"].size(); ++k) {
                    const nlohmann::json& v = sj["junctions"][k];
                    const std::string id = v.is_string() ? v.get<std::string>() : std::string();
                    if (id.empty()) continue;
                    if (!junctionAt.count(id)) throw std::runtime_error("street " + st.id + ": names unknown junction " + id);
                    st.junctions[k] = ModelJunctionId(junctionAt.at(id));
                }
            if (sj.contains("carries_on"))
                for (std::size_t k = 0; k < 2 && k < sj["carries_on"].size(); ++k) {
                    const nlohmann::json& v = sj["carries_on"][k];
                    const std::string id = v.is_string() ? v.get<std::string>() : std::string();
                    if (id.empty()) continue;
                    if (!streetAt.count(id)) throw std::runtime_error("street " + st.id + ": carries on into unknown street " + id);
                    st.carriesOn[k] = ModelStreetId(streetAt.at(id));
                }
            st.name = sj.value("name", std::string());
            g.streets.push_back(st);
        }
    }
    if (spec.contains("arms_in_ring")) {
        // THE JUNCTIONS' ARMS LISTED IN THEIR RING (ADR-0427): the file's word for it, true or absent
        if (!spec["arms_in_ring"].is_boolean()) throw std::runtime_error("arms_in_ring: wants true (every junction's arms listed counter-clockwise round it)");
        g.armsInRing = spec["arms_in_ring"].get<bool>();
    }
    if (spec.contains("carry_on_control")) g.carryOnControl = spec["carry_on_control"].get<std::string>();
    if (spec.contains("lane_rules")) {
        // THE LANE LEVEL'S RULES (ADR-0426): every rule it reads, stated
        const nlohmann::json& lr = spec["lane_rules"];
        if (!lr.is_object() || !lr.contains("solid_before_stop_line") || !lr["solid_before_stop_line"].is_number() ||
            lr["solid_before_stop_line"].get<double>() < 0)
            throw std::runtime_error("lane_rules: wants {\"solid_before_stop_line\": metres, zero or more}");
        g.laneRules.solidBeforeStopLine = lr["solid_before_stop_line"].get<double>();
        g.statesLaneRules = true;
    }
    if (spec.contains("divided")) {
        // THE DIVIDED ROADS (DividedPair): two roads that exist, each in one pair at most
        g.listsDivided = true;
        std::set<std::string> paired;
        for (const nlohmann::json& pj : spec["divided"]) {
            const DividedPair p{pj.at(0).get<std::string>(), pj.at(1).get<std::string>()};
            for (const std::string& r : p) {
                if (!g.index.count(r)) throw std::runtime_error("divided: names unknown road " + r);
                if (!paired.insert(r).second) throw std::runtime_error("divided: road " + r + " is in two pairs");
            }
            g.divided.push_back(p);
        }
    }
    if (g.edges.empty()) throw std::runtime_error("graph has no edges");
    return g;
}

RoadLabGraph RoadLabGraph::load(const std::string& path) {
    std::ifstream f(path); if (!f) throw std::runtime_error("cannot open " + path);
    nlohmann::json j; f >> j; std::string dir = path.find('/') == std::string::npos ? "." : path.substr(0, path.rfind('/'));
    return fromJson(j, dir);
}

}  // namespace roads::lanes
}  // namespace engine
