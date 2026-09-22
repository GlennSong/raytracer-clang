#include "engine/procgen/city/roads/lanes/city_producer.h"

#include "engine/mesh_builder.h"
#include "engine/procgen/city/roads/lanes/block_audit.h"
#include "engine/procgen/city/roads/lanes/deck_mesh.h"
#include "engine/procgen/city/roads/lanes/lots_producer.h"
#include "engine/procgen/city/roads/lanes/road_twin.h"
#include "engine/procgen/terrain_field.h"
#include "log.h"

#include <algorithm>
#include <chrono>
#include <cstdlib>
#include <fstream>
#include <map>
#include <memory>
#include <mutex>

namespace engine {
namespace roads::lanes {

const char* const kLanesBuildTag = "2026-09-21.5";   // .1: the terrain is clamped under every deck VERTEX, not by centreline reach   // .2/.3: ramp ends (elevated or at-grade runs) welded to the road they merge into (nav twin)   // .4: one-way carriageways and ramps; the nav gets a deck's travel width   // .5: blocks are the holes of the pavement WITH its sidewalks

namespace {
using bundle::BinReader;
using bundle::BinWriter;

std::string graphDir(const std::string& path) { const size_t s = path.find_last_of('/'); return s == std::string::npos ? "." : path.substr(0, s); }

// The terrain grid file a graph references, resolved the way RoadLabGraph::fromJson does (relative to the graph's directory).
std::string terrainFileOf(const nlohmann::json& spec, const std::string& baseDir) {
    if (!spec.contains("terrain") || !spec["terrain"].is_object() || !spec["terrain"].contains("file")) return std::string();
    std::string f = spec["terrain"]["file"].get<std::string>(); if (f.empty() || f[0] == '/') return f; return baseDir + "/" + f;
}

double secondsSince(const std::chrono::steady_clock::time_point& t) { return std::chrono::duration<double>(std::chrono::steady_clock::now() - t).count(); }

class CityProducer final : public bundle::BundleProducer {
public:
    std::string name() const override { return kCityProducerName; }
    bool applies(const bundle::LevelInputs& in) const override { return !cityEntities(in.level).empty(); }
    double weight() const override { return 36.0; }   // metro seconds today; lots will weigh in beside it

    bundle::ProducerIdentity identity(const bundle::LevelInputs& in) const override {
        bundle::ProducerIdentity id; id.tag = kLanesBuildTag;
        uint64_t k = bundle::fnv1aStr(std::string(kCityProducerName) + "|" + std::to_string(kCityFormatVersion) + "|" + kLanesBuildTag + "|real" + std::to_string(sizeof(Real)));
        const double cell = cityRenderCell(in.level); k = bundle::fnv1a(&cell, sizeof(cell), k);
        for (const CityEntity& e : cityEntities(in.level)) {
            k = bundle::fnv1aStr("entity" + std::to_string(e.ordinal), k);
            if (e.inlineGraph) { k = bundle::fnv1aStr(e.block.dump(), k); continue; }
            bundle::InputFile f; f.path = e.graphPath; uint64_t fk = bundle::kFnvOffset;
            if (bundle::fnv1aFile(e.graphPath, fk, &f.bytes, &f.mtime)) { f.fnv = fk; k = bundle::fnv1a(&fk, sizeof(fk), k); }
            else k = bundle::fnv1aStr("missing:" + e.graphPath, k);
            id.inputs.push_back(f);
            std::ifstream gf(e.graphPath); nlohmann::json spec; try { gf >> spec; } catch (const std::exception&) { spec = nlohmann::json(); }
            const std::string tf = terrainFileOf(spec, graphDir(e.graphPath));
            if (!tf.empty()) {
                bundle::InputFile t; t.path = tf; uint64_t tk = bundle::kFnvOffset;
                if (bundle::fnv1aFile(tf, tk, &t.bytes, &t.mtime)) { t.fnv = tk; k = bundle::fnv1a(&tk, sizeof(tk), k); } else k = bundle::fnv1aStr("missing:" + tf, k);
                id.inputs.push_back(t);
            }
        }
        id.key = k; return id;
    }

    bundle::ProducerReport produce(const bundle::LevelInputs& in, bundle::BundleWriter& out, const bundle::ProgressFn* progress) const override {
        bundle::ProducerReport rep; const auto t0 = std::chrono::steady_clock::now();
        const std::vector<CityEntity> ents = cityEntities(in.level); const double cell = cityRenderCell(in.level);
        nlohmann::json entities = nlohmann::json::array();
        auto report = [&](int ordinal, double local, const std::string& stage, const std::string& msg) {
            if (!progress) return true; bundle::Progress p; p.stage = stage; p.message = msg; p.fraction = (ordinal + local) / std::max<size_t>(1, ents.size()); return (*progress)(p);
        };
        for (const CityEntity& e : ents) {
            if (!report(e.ordinal, 0.02, "graph", e.inlineGraph ? "inline graph" : e.graphPath)) { rep.ok = false; rep.error = "cancelled"; return rep; }
            std::unique_ptr<Result> result;
            BuildProgressFn onBuild = [&](const BuildProgress& bp) { return report(e.ordinal, 0.02 + 0.78 * bp.fraction, bp.stage, ""); };
            BuildOptions opts; opts.progress = progress ? &onBuild : nullptr; opts.threads = in.threads;
            try { result = build(loadCityGraph(e), opts); }
            catch (const BuildCancelled&) { rep.ok = false; rep.error = "cancelled"; return rep; }
            catch (const std::exception& ex) { rep.ok = false; rep.error = (e.inlineGraph ? std::string("inline graph") : e.graphPath) + ": " + ex.what(); return rep; }
            if (!report(e.ordinal, 0.80, "meshes", summary(*result).substr(0, summary(*result).find('\n')))) { rep.ok = false; rep.error = "cancelled"; return rep; }
            const auto tp = std::chrono::steady_clock::now();
            static const bool check = std::getenv("RT_CITY_CHECK") != nullptr;   // rt_bake --check: run the invariant sweep into the report
            CityProducts p = cityProductsFromResult(*result, cell, check);
            const double packSeconds = secondsSince(tp);
            if (!report(e.ordinal, 0.95, "write", std::to_string(p.cells.size()) + " cell meshes")) { rep.ok = false; rep.error = "cancelled"; return rep; }
            writeCityProducts(out, e.ordinal, p);
            const nlohmann::json stageTimes = p.report.value("timings", nlohmann::json::object());   // a named object: items() on a temporary iterates nothing
            for (const auto& kv : stageTimes.items()) rep.timings["e" + std::to_string(e.ordinal) + "." + kv.key()] += kv.value().get<double>();
            rep.timings["e" + std::to_string(e.ordinal) + ".products"] += packSeconds;
            p.report["ordinal"] = e.ordinal; p.report["graph"] = e.inlineGraph ? "(inline)" : e.graphPath; entities.push_back(p.report);
            LOG_INFO << "[city] e" << e.ordinal << ": " << result->lanes.lanes.size() << " lanes, " << p.cells.size() << " cell meshes, " << p.holes.size() << " pavement holes, built in " << result->seconds << " s";
        }
        rep.report = {{"entities", entities}, {"cell", cell}}; rep.seconds = secondsSince(t0); return rep;
    }
};
}  // namespace

void registerCityProducer() { bundle::registerProducer(std::make_unique<CityProducer>()); }

std::vector<CityEntity> cityEntities(const nlohmann::json& level) {
    std::vector<CityEntity> out;
    if (!level.contains("entities") || !level["entities"].is_array()) return out;
    int i = 0;
    for (const nlohmann::json& ent : level["entities"]) {
        const int idx = i++;
        const std::string shape = ent.value("shape", std::string());
        // The lab's own entity, and the roads module's road entity that names this
        // builder: the same city, two spellings (docs/road-module-plan.md phase 3).
        nlohmann::json block;
        if (shape == "lanelab") block = ent.contains("lanelab") ? ent["lanelab"] : nlohmann::json::object();
        else if (shape == "road" && ent.contains("road") && ent["road"].is_object() &&
                 ent["road"].value("builder", std::string()) == "lanes") block = ent["road"];
        else continue;
        CityEntity e; e.ordinal = static_cast<int>(out.size()); e.entityIndex = idx; e.block = std::move(block);
        e.inlineGraph = e.block.contains("edges"); e.graphPath = e.inlineGraph ? std::string() : e.block.value("graph", std::string());
        out.push_back(std::move(e));
    }
    return out;
}

int cityOrdinalForEntity(const nlohmann::json& level, int entityIndex) {
    for (const CityEntity& e : cityEntities(level)) if (e.entityIndex == entityIndex) return e.ordinal;
    return -1;
}

namespace {
std::mutex g_cityBundleMu;
std::map<std::string, std::pair<std::shared_ptr<bundle::Bundle>, std::string>> g_cityBundles;
}  // namespace

std::shared_ptr<bundle::Bundle> levelCityBundle(const bundle::LevelInputs& in, std::string* status) {
    // Keyed by the level AND by what its inputs hash to, so an edited level in the same
    // process is a different city rather than a stale one.
    const std::string key = in.levelPath + "|" + bundle::bundleDirForLevel(in);
    std::lock_guard<std::mutex> lock(g_cityBundleMu);
    auto it = g_cityBundles.find(key);
    if (it == g_cityBundles.end()) {
        registerCityProducer();
        registerLotsProducer();   // both before the first obtain: the bundle is named by every producer that applies
        bundle::Obtained o = bundle::obtainForLevel(in, kCityProducerName);
        it = g_cityBundles.emplace(key, std::make_pair(std::shared_ptr<bundle::Bundle>(std::move(o.bundle)), o.status)).first;
    }
    if (status) *status = it->second.second;
    return it->second.first;
}

void forgetLevelCityBundles() {
    std::lock_guard<std::mutex> lock(g_cityBundleMu);
    g_cityBundles.clear();
}

double cityRenderCell(const nlohmann::json& level) {
    if (level.contains("citysim") && level["citysim"].is_object()) return level["citysim"].value("renderCell", 250.0);
    return 250.0;
}

std::string citySectionPrefix(int ordinal) { return "city/e" + std::to_string(ordinal) + "/"; }

RoadLabGraph loadCityGraph(const CityEntity& e) {
    RoadLabGraph g = e.inlineGraph ? RoadLabGraph::fromJson(e.block, ".") : RoadLabGraph::load(e.graphPath);
    // THE LEVEL SETS ITS SIDEWALKS (Glenn, 2026-09-21: "I think the sidewalks should be wider.
    // They're too skinny"). The importer copies the source level's one sidewalk number (3.5 m)
    // into every street class of the graph asset; a level can override it without regenerating
    // the asset: `"sidewalks": 5` (every class that has a sidewalk) or `"sidewalks": {"arterial": 6,
    // "local": 5}` (by class). Plural, because a road block's `sidewalk` is already the lattice's
    // one number and other readers take it as such. The level JSON keys the bundle, so an edit
    // rebuilds the city.
    if (e.block.contains("sidewalks")) {
        const nlohmann::json& sw = e.block["sidewalks"];
        for (auto& [name, cls] : g.classes) {
            if (sw.is_number()) { if (cls.sidewalk > 0) cls.sidewalk = sw.get<double>(); }
            else if (sw.is_object() && sw.contains(name) && sw[name].is_number()) cls.sidewalk = sw[name].get<double>();
        }
    }
    return g;
}

// THE DRIVING SURFACE THE LANES BUILT, as the field everything stands on. One spine per graph
// edge: the edge's own resampled centreline and the profile the pavement was laid to (EdgeSpec::z
// is the deck, not the terrain). The half-width is the CARRIAGEWAY's — lanes plus shoulder, not
// the sidewalk — because this answers "am I on the road", and a car on the pavement edge is not.
// No junction pads: a lane city has no separate pad surface, its ribbons simply overlap, and each
// edge's profile is defined right through the node.
static RoadDeckField deckFromResult(const Result& r) {
    RoadDeckField d;
    d.spines.reserve(r.graph.edges.size());
    for (std::size_t ei = 0; ei < r.graph.edges.size(); ++ei) {
        const EdgeSpec& e = r.graph.edges[ei];
        if (e.xy.size() < 2 || e.z.size() != e.xy.size()) continue;
        const auto it = r.graph.classes.find(e.cls);
        const double shoulder = it != r.graph.classes.end() ? it->second.shoulder : 0.0;
        const double lanes = std::max(1, e.laneCount()) * e.lanes.w + e.lanes.gap;
        UnionSpine s;
        s.points = e.xy;
        // THE HEIGHT THE PAVEMENT WAS DRAWN AT, not the height this edge alone wanted.
        // EdgeSpec::z is the edge's OWN solved profile; within blendLen of a higher-ranked
        // road the drawn deck is DeckHeight::deckRoad — the partner's blend — and the two
        // differ by the junction's mismatch. Measured on metro_lanes with the own profile:
        // 171 of 1639 traffic samples past 25 cm, worst 1.07 m, every one of them beside a
        // merge. A deck field that does not answer what was drawn is worse than none.
        s.yAbs.reserve(e.xy.size());
        for (std::size_t k = 0; k < e.xy.size(); ++k)
            s.yAbs.push_back(r.heights ? r.heights->deckRoad(static_cast<int>(ei), e.xy[k]) : e.z[k]);
        s.halfWidth = lanes * 0.5 + shoulder;
        s.klass = classOf(e);
        s.authoredDeck = e.isRamp() || s.klass == RoadClass::Freeway;
        // A road that bridges is a LAYER above the one it crosses, which is how a 2-D height
        // query tells them apart (RoadDeckField::heightAt). The build already measured it.
        const auto bl = r.bridgeLen.find(e.id);
        s.layer = bl != r.bridgeLen.end() && bl->second > 0.0 ? 1 : 0;
        d.spines.push_back(std::move(s));
    }
    d.buildIndex();
    return d;
}

// THE KERB LINE: the boundary of the driving surface itself, which is what the city map draws a
// sidewalk against and what street furniture stands clear of. The lanes pavement already holds it
// as a polygon set — outers and holes alike are kerb loops, an island's kerb is still a kerb.
static CurbBandAudit bandsFromResult(const Result& r) {
    CurbBandAudit b;
    for (const Polygon2& poly : r.pavement.surface) {
        if (poly.outer.size() >= 3) b.loops.push_back(poly.outer);
        for (const Ring& h : poly.holes)
            if (h.size() >= 3) b.loops.push_back(h);
    }
    // The widest sidewalk any class asks for: the band the loader falls back to, and what the
    // citysim reads as "how far from the kerb does the pavement reach".
    for (const auto& [name, spec] : r.graph.classes) {
        (void)name;
        b.sidewalkWidth = std::max(b.sidewalkWidth, spec.sidewalk);
    }
    b.curbHeight = lanesSidewalkRise();
    // Junctions, for the audits that ask how sharp a city's corners are: a node where three or
    // more edge ENDS meet, counted from the resolved spines rather than the authored graph.
    std::map<std::pair<long, long>, int> deg;
    auto key = [](const Vec2& p) { return std::make_pair(std::lround(p.x * 10), std::lround(p.y * 10)); };
    for (const EdgeSpec& e : r.graph.edges) {
        if (e.xy.size() < 2) continue;
        ++deg[key(e.xy.front())];
        ++deg[key(e.xy.back())];
    }
    for (const auto& [k, n] : deg) {
        if (n < 3) continue;
        b.junctions.push_back(Vec2(k.first / 10.0, k.second / 10.0));
        b.junctionDegree.push_back(n);
        b.junctionMinAngle.push_back(-1.0);       // not measured here; the mesher's own audit does
    }
    return b;
}

CityProducts cityProductsFromResult(const Result& r, double renderCell, bool withInvariants) {
    CityProducts p; p.cell = renderCell; const auto t0 = std::chrono::steady_clock::now();
    if (r.hasTerrain) { p.hasTerrain = true; p.ground.x0 = r.terrain.x0; p.ground.y0 = r.terrain.y0; p.ground.res = r.terrain.res; p.ground.nx = r.terrain.nx; p.ground.ny = r.terrain.ny; p.ground.z = r.terrain.z; }
    p.twin = roadTwin(r);
    HeightField ground;
    if (p.hasTerrain) { auto grid = std::make_shared<HeightGrid>(r.terrain); ground = [grid](double x, double z) { return grid->sample(x, z); }; }
    p.nav = navRoadGraph(p.twin, ground ? ground : HeightField());
    for (const RoadNode& n : p.twin.graph.nodes) p.row.nodes.push_back(n);
    for (const RoadEdge& e : p.twin.graph.edges)
        if (e.klass == RoadClass::Freeway || e.klass == RoadClass::Ramp) {
            RoadEdge w = e;
            w.width += twinRightOfWayPad(r, e.klass);   // the right-of-way, not the travel lanes
            p.row.edges.push_back(w);
        }
    p.holes = pavementHoles(r, 2000.0);
    p.deck = deckFromResult(r);
    p.bands = bandsFromResult(r);
    const double tTwin = secondsSince(t0); const auto t1 = std::chrono::steady_clock::now();
    size_t triangles = 0, bytes = 0; nlohmann::json materials = nlohmann::json::array();
    std::vector<NamedMesh> named = buildMeshes(r); const double tMeshes = secondsSince(t1); const auto t2 = std::chrono::steady_clock::now();
    for (const NamedMesh& nm : named) {
        if (nm.mesh.vertices.empty()) continue;
        const bool paint = nm.name.rfind("paint", 0) == 0; const bool terrain = nm.name == "terrain";
        const uint32_t flags = paint ? bundle::PackedMesh::kPaint : bundle::PackedMesh::kCollidable;
        std::vector<MeshBuilder::CellChunk> chunks = MeshBuilder::chunkByCell(nm.mesh, renderCell);
        std::sort(chunks.begin(), chunks.end(), [](const MeshBuilder::CellChunk& a, const MeshBuilder::CellChunk& b) { return a.cx != b.cx ? a.cx < b.cx : a.cz < b.cz; });
        size_t matTris = 0;
        for (MeshBuilder::CellChunk& c : chunks) {
            CityCellMesh cm; cm.cx = c.cx; cm.cz = c.cz;
            const bool steel = nm.name == "guardrail";   // galvanised: smoother than concrete, still matte
            cm.mesh = bundle::packMesh(c.mesh, nm.name, nm.color, flags, paint ? 0.7f : steel ? 0.42f : 0.93f, terrain ? 0.7 : 0.85);
            matTris += cm.mesh.triangleCount(); bytes += cm.mesh.bytes(); p.cells.push_back(std::move(cm));
        }
        triangles += matTris; materials.push_back({{"name", nm.name}, {"triangles", matTris}, {"cells", chunks.size()}, {"collidable", !paint}});
    }
    const double tPack = secondsSince(t2); const auto t3 = std::chrono::steady_clock::now();
    nlohmann::json inv = nlohmann::json::array();
    if (withInvariants) for (const Check& c : invariants(r)) inv.push_back({{"name", c.name}, {"ok", c.ok}, {"detail", c.detail}});
    std::map<std::string, double> timings = r.timings; timings["twin"] = tTwin; timings["meshes"] = tMeshes; timings["pack"] = tPack; if (withInvariants) timings["invariants"] = secondsSince(t3);
    p.report = {{"summary", summary(r)}, {"invariants", inv}, {"checked", withInvariants}, {"timings", timings}, {"seconds", r.seconds}, {"lanes", r.lanes.lanes.size()}, {"triangles", triangles},
                {"meshBytes", bytes}, {"materials", materials}, {"cells", p.cells.size()}, {"holes", p.holes.size()}, {"deckSpines", p.deck.spines.size()}, {"kerbLoops", p.bands.loops.size()}, {"twinNodes", p.twin.graph.nodes.size()}, {"twinEdges", p.twin.graph.edges.size()},
                {"navNodes", p.nav.nodes.size()}, {"navEdges", p.nav.edges.size()}, {"rowEdges", p.row.edges.size()}, {"hasTerrain", p.hasTerrain}};
    return p;
}

void writeCityProducts(bundle::BundleWriter& w, int ordinal, const CityProducts& p) {
    const std::string pre = citySectionPrefix(ordinal);
    w.addJson(pre + "meta", nlohmann::json{{"format", kCityFormatVersion}, {"tag", kLanesBuildTag}, {"hasTerrain", p.hasTerrain}, {"cell", p.cell}});
    if (p.hasTerrain) { BinWriter b; bundle::putHeightGrid(b, p.ground); w.add(pre + "ground", b.bytes); }
    { BinWriter b; bundle::putRoadEntity(b, p.twin); w.add(pre + "roads/twin", b.bytes); }
    { BinWriter b; bundle::putRoadGraph(b, p.nav); w.add(pre + "roads/nav", b.bytes); }
    { BinWriter b; bundle::putRoadGraph(b, p.row); w.add(pre + "roads/row", b.bytes); }
    { BinWriter b; bundle::putRings(b, p.holes); w.add(pre + "blocks/holes", b.bytes); }
    { BinWriter b; bundle::putDeckField(b, p.deck); w.add(pre + "roads/deck", b.bytes); }
    { BinWriter b; bundle::putCurbBands(b, p.bands); w.add(pre + "roads/bands", b.bytes); }
    std::map<std::pair<int, int>, nlohmann::json> cells;
    for (const CityCellMesh& c : p.cells) {
        BinWriter b; bundle::putPackedMesh(b, c.mesh);
        w.add(pre + "cell/" + std::to_string(c.cx) + "_" + std::to_string(c.cz) + "/mesh/" + c.mesh.name, b.bytes);
        nlohmann::json& cj = cells[{c.cx, c.cz}];
        if (cj.is_null()) cj = {{"cx", c.cx}, {"cz", c.cz}, {"materials", nlohmann::json::array()}, {"triangles", 0}};
        cj["materials"].push_back(c.mesh.name); cj["triangles"] = cj["triangles"].get<size_t>() + c.mesh.triangleCount();
    }
    nlohmann::json list = nlohmann::json::array(); for (const auto& kv : cells) list.push_back(kv.second);
    w.addJson(pre + "cells", nlohmann::json{{"cell", p.cell}, {"cells", list}});
    w.addJson(pre + "report", p.report);
}

bool readCityProducts(const bundle::Bundle& b, int ordinal, CityProducts& p, std::string* err) {
    const std::string pre = citySectionPrefix(ordinal);
    auto fail = [&](const std::string& why) { if (err) *err = pre + why; return false; };
    const nlohmann::json meta = b.json(pre + "meta"); if (meta.is_null()) return fail("meta: missing");
    if (meta.value("format", 0) != kCityFormatVersion) return fail("meta: city format " + std::to_string(meta.value("format", 0)));
    p.hasTerrain = meta.value("hasTerrain", false); p.cell = meta.value("cell", 250.0);
    auto section = [&](const std::string& name, bundle::Bundle::View& v) { v = b.section(pre + name); return !v.empty(); };
    bundle::Bundle::View v;
    if (p.hasTerrain) { if (!section("ground", v)) return fail("ground: missing"); BinReader r(v.data, v.size); if (!bundle::getHeightGrid(r, p.ground)) return fail("ground: unreadable"); }
    if (!section("roads/twin", v)) return fail("roads/twin: missing"); { BinReader r(v.data, v.size); if (!bundle::getRoadEntity(r, p.twin)) return fail("roads/twin: unreadable"); }
    if (!section("roads/nav", v)) return fail("roads/nav: missing"); { BinReader r(v.data, v.size); if (!bundle::getRoadGraph(r, p.nav)) return fail("roads/nav: unreadable"); }
    if (!section("roads/row", v)) return fail("roads/row: missing"); { BinReader r(v.data, v.size); if (!bundle::getRoadGraph(r, p.row)) return fail("roads/row: unreadable"); }
    if (!section("blocks/holes", v)) return fail("blocks/holes: missing"); { BinReader r(v.data, v.size); if (!bundle::getRings(r, p.holes)) return fail("blocks/holes: unreadable"); }
    // The deck and the kerb line (2026-09-20): a bundle baked before they existed has neither,
    // and the tag bump that introduced them means one cannot be loaded — so a missing section
    // here is a hard error, not a silent city the sim would sink into.
    if (!section("roads/deck", v)) return fail("roads/deck: missing"); { BinReader r(v.data, v.size); if (!bundle::getDeckField(r, p.deck)) return fail("roads/deck: unreadable"); }
    if (!section("roads/bands", v)) return fail("roads/bands: missing"); { BinReader r(v.data, v.size); if (!bundle::getCurbBands(r, p.bands)) return fail("roads/bands: unreadable"); }
    const nlohmann::json cells = b.json(pre + "cells"); if (cells.is_null()) return fail("cells: missing");
    p.cells.clear();
    for (const nlohmann::json& c : cells.value("cells", nlohmann::json::array())) {
        const int cx = c.value("cx", 0), cz = c.value("cz", 0);
        for (const nlohmann::json& m : c.value("materials", nlohmann::json::array())) {
            const std::string name = m.get<std::string>(); const std::string sec = "cell/" + std::to_string(cx) + "_" + std::to_string(cz) + "/mesh/" + name;
            if (!section(sec, v)) return fail(sec + ": missing");
            CityCellMesh cm; cm.cx = cx; cm.cz = cz; BinReader r(v.data, v.size); if (!bundle::getPackedMesh(r, cm.mesh)) return fail(sec + ": unreadable"); p.cells.push_back(std::move(cm));
        }
    }
    p.report = b.json(pre + "report");
    return true;
}

}  // namespace roads::lanes
}  // namespace engine
