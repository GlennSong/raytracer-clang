#include "engine/procgen/city/roads/lanes/city_producer.h"

#include "engine/mesh_builder.h"
#include "engine/procgen/city/roads/lanes/block_audit.h"
#include "engine/procgen/city/roads/lanes/deck_mesh.h"
#include "engine/procgen/city/roads/lanes/road_twin.h"
#include "engine/procgen/terrain_field.h"
#include "log.h"

#include <algorithm>
#include <chrono>
#include <cstdlib>
#include <fstream>
#include <map>

namespace engine {
namespace roads::lanes {

const char* const kLanesBuildTag = "2026-09-20.1";

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
        if (ent.value("shape", std::string()) != "lanelab") continue;
        CityEntity e; e.ordinal = static_cast<int>(out.size()); e.entityIndex = idx; e.block = ent.contains("lanelab") ? ent["lanelab"] : nlohmann::json::object();
        e.inlineGraph = e.block.contains("edges"); e.graphPath = e.inlineGraph ? std::string() : e.block.value("graph", std::string());
        out.push_back(std::move(e));
    }
    return out;
}

double cityRenderCell(const nlohmann::json& level) {
    if (level.contains("citysim") && level["citysim"].is_object()) return level["citysim"].value("renderCell", 250.0);
    return 250.0;
}

std::string citySectionPrefix(int ordinal) { return "city/e" + std::to_string(ordinal) + "/"; }

RoadLabGraph loadCityGraph(const CityEntity& e) { return e.inlineGraph ? RoadLabGraph::fromJson(e.block, ".") : RoadLabGraph::load(e.graphPath); }

CityProducts cityProductsFromResult(const Result& r, double renderCell, bool withInvariants) {
    CityProducts p; p.cell = renderCell; const auto t0 = std::chrono::steady_clock::now();
    if (r.hasTerrain) { p.hasTerrain = true; p.ground.x0 = r.terrain.x0; p.ground.y0 = r.terrain.y0; p.ground.res = r.terrain.res; p.ground.nx = r.terrain.nx; p.ground.ny = r.terrain.ny; p.ground.z = r.terrain.z; }
    p.twin = roadTwin(r);
    HeightField ground;
    if (p.hasTerrain) { auto grid = std::make_shared<HeightGrid>(r.terrain); ground = [grid](double x, double z) { return grid->sample(x, z); }; }
    p.nav = navRoadGraph(p.twin, ground ? ground : HeightField());
    for (const RoadNode& n : p.twin.graph.nodes) p.row.nodes.push_back(n);
    for (const RoadEdge& e : p.twin.graph.edges) if (e.klass == RoadClass::Freeway || e.klass == RoadClass::Ramp) p.row.edges.push_back(e);
    p.holes = pavementHoles(r, 2000.0);
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
                {"meshBytes", bytes}, {"materials", materials}, {"cells", p.cells.size()}, {"holes", p.holes.size()}, {"twinNodes", p.twin.graph.nodes.size()}, {"twinEdges", p.twin.graph.edges.size()},
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
