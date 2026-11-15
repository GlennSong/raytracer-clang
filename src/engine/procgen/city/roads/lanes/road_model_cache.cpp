#include "engine/procgen/city/roads/lanes/road_model_cache.h"

#include "engine/bundle/bundle.h"
#include "engine/bundle/cache_use.h"
#include "engine/procgen/city/roads/lanes/city_producer.h"   // the roads' key, the city entities, loadCityGraph
#include "engine/procgen/city/roads/lanes/road_layout.h"     // laneLevelKey: the layout and lane rules, keyed here
#include "engine/procgen/city/roads/lane_turns.h"            // laneTurnRules: the turns' table, keyed here

#include <nlohmann/json.hpp>

#include <chrono>
#include <cstdio>
#include <exception>
#include <filesystem>
#include <fstream>
#include <utility>

#ifndef _WIN32
#include <unistd.h>
#endif

namespace engine {
namespace roads::lanes {

namespace {

namespace fs = std::filesystem;

// The file's two sections and the manifest field that names what it is.
constexpr const char* kModelSection = "roadmodel/model";
constexpr const char* kSummarySection = "roadmodel/summary";
constexpr const char* kManifestField = "roadmodel";

double secondsSince(const std::chrono::steady_clock::time_point& t) {
    return std::chrono::duration<double>(std::chrono::steady_clock::now() - t).count();
}

// A process id for a temporary file's name, so two processes writing one key never write into each other's file.
std::string processTag() {
#ifndef _WIN32
    return std::to_string(static_cast<long>(::getpid()));
#else
    return "0";
#endif
}

bool fail(std::string* err, const std::string& why) {
    if (err) *err = why;
    return false;
}

nlohmann::json summaryJson(const LaneGraphSummary& s) {
    return nlohmann::json{{"paths", s.paths},           {"lanes", s.lanes},
                          {"connectors", s.connectors}, {"sections", s.sections},
                          {"junctions", s.junctions},   {"refusedTurns", s.refusedTurns},
                          {"problems", s.problems},     {"reachableShare", s.reachableShare}};
}

// Reads one whole number of the summary; false when it is missing or not one.
bool countField(const nlohmann::json& j, const char* name, uint32_t& out) {
    if (!j.contains(name) || !j[name].is_number_unsigned()) return false;
    out = j[name].get<uint32_t>();
    return true;
}

bool summaryFromJson(const nlohmann::json& j, LaneGraphSummary& s) {
    if (!j.is_object()) return false;
    if (!countField(j, "paths", s.paths) || !countField(j, "lanes", s.lanes) ||
        !countField(j, "connectors", s.connectors) || !countField(j, "sections", s.sections) ||
        !countField(j, "junctions", s.junctions) || !countField(j, "refusedTurns", s.refusedTurns) ||
        !countField(j, "problems", s.problems))
        return false;
    if (!j.contains("reachableShare") || !j["reachableShare"].is_number()) return false;
    s.reachableShare = j["reachableShare"].get<double>();
    return true;
}

// The counts a graph itself can say: its paths by kind, sections and junctions.
void countGraph(const LaneGraph& g, LaneGraphSummary& s) {
    s.paths = static_cast<uint32_t>(g.paths.size());
    s.lanes = 0;
    for (const Path& p : g.paths) s.lanes += p.kind == PathKind::Lane ? 1 : 0;
    s.connectors = s.paths - s.lanes;
    s.sections = static_cast<uint32_t>(g.sections.size());
    s.junctions = static_cast<uint32_t>(g.junctions.size());
}

}  // namespace

bool operator==(const LaneGraphSummary& a, const LaneGraphSummary& b) {
    return a.paths == b.paths && a.lanes == b.lanes && a.connectors == b.connectors && a.sections == b.sections &&
           a.junctions == b.junctions && a.refusedTurns == b.refusedTurns && a.problems == b.problems &&
           a.reachableShare == b.reachableShare;
}

LaneGraphSummary summarizeLaneGraph(const LaneGraph& graph, const LaneGraphBuild& built) {
    LaneGraphSummary s;
    countGraph(graph, s);
    for (const JunctionNote& n : built.junctionNotes) s.refusedTurns += n.turnsRefused;
    s.problems = static_cast<uint32_t>(built.problems.size());
    s.reachableShare = built.reachableShare;
    return s;
}

std::string describeLaneGraphSummary(const LaneGraphSummary& s) {
    char b[320];
    std::snprintf(b, sizeof b,
                  "%u paths (%u lanes, %u connectors), %u sections, %u junctions, %u turns refused, a car can get "
                  "round %.1f%% of the lane paths",
                  s.paths, s.lanes, s.connectors, s.sections, s.junctions, s.refusedTurns, 100.0 * s.reachableShare);
    return b;
}

uint64_t roadModelCacheKey(uint64_t roadsKey, uint64_t laneFacts, uint64_t turnTable, const LaneGraphOptions& opts,
                           uint32_t builderVersion) {
    bundle::Fnv k;
    k.str("roadmodel");
    k.pod(kRoadModelFormat);
    k.pod(builderVersion);
    k.pod(roadsKey);
    k.pod(laneFacts);
    k.pod(turnTable);
    // Each string's length goes in before it, so ("ab", "c") and ("a", "bc") are different keys.
    k.pod(static_cast<uint64_t>(opts.vehicles.size()));
    for (const VehicleSize& v : opts.vehicles) {
        k.pod(static_cast<uint64_t>(v.tag.size()));
        k.str(v.tag);
        k.pod(v.turningRadius);
        k.pod(v.width);
        k.pod(v.length);   // (since builder version 6: bodies swept at their length, ADR-0412)
    }
    return k.h;
}

std::string roadModelCachePath(const std::string& dir, uint64_t key) {
    return dir + "/" + bundle::hex16(key) + ".roadmodel";
}

bool writeRoadModelCache(const std::string& path, uint64_t key, const RoadModel& model, const LaneGraphSummary& summary,
                         std::string* err) {
    std::error_code ec;
    const fs::path parent = fs::path(path).parent_path();
    if (!parent.empty()) fs::create_directories(parent, ec);
    const std::string tmp = path + ".tmp-" + processTag();
    {
        bundle::BundleWriter w;
        std::string why;
        if (!w.openFile(tmp, &why)) return fail(err, "road model cache: " + why);
        nlohmann::json manifest;
        manifest[kManifestField] = {{"key", bundle::hex16(key)}};
        bundle::BinWriter bytes;
        putRoadModel(bytes, model);
        if (!w.add(kModelSection, bytes.bytes) || !w.addJson(kSummarySection, summaryJson(summary)) ||
            !w.finish(manifest, &why)) {
            fs::remove(tmp, ec);
            return fail(err, "road model cache: cannot write " + tmp + (why.empty() ? std::string() : ": " + why));
        }
    }
    fs::rename(tmp, path, ec);
    if (ec) {
        fs::remove(tmp, ec);
        return fail(err, "road model cache: cannot move " + tmp + " into place");
    }
    return true;
}

bool readRoadModelCache(const std::string& path, uint64_t key, RoadModel& model, LaneGraphSummary& summary,
                        std::string* err) {
    model = RoadModel();
    summary = LaneGraphSummary();
    std::error_code ec;
    if (!fs::is_regular_file(path, ec)) return fail(err, path + ": no such file");
    std::string why;
    const std::unique_ptr<bundle::Bundle> b = bundle::Bundle::open(path, &why);
    if (!b) return fail(err, why);
    const nlohmann::json& m = b->manifest();
    if (!m.contains(kManifestField) || !m[kManifestField].is_object())
        return fail(err, path + ": not a road model cache (its table of contents names no road model)");
    const nlohmann::json& rm = m[kManifestField];
    if (rm.value("key", std::string()) != bundle::hex16(key))
        return fail(err, path + ": made for key " + rm.value("key", std::string("(none)")) + ", not " +
                             bundle::hex16(key));
    for (const char* section : {kModelSection, kSummarySection})
        if (!b->checkSection(section))
            return fail(err, path + ": section " + section + " is missing or its bytes no longer match their hash");
    const bundle::Bundle::View v = b->section(kModelSection);
    bundle::BinReader r(v.data, v.size);
    RoadModel read;
    if (!getRoadModel(r, read, &why)) return fail(err, path + ": " + why);   // (another layout of the bytes is refused here)
    LaneGraphSummary s;
    if (!summaryFromJson(b->json(kSummarySection), s)) return fail(err, path + ": its summary is unreadable");
    LaneGraphSummary counted = s;
    countGraph(read.laneLevel, counted);
    if (!(counted == s)) return fail(err, path + ": its summary does not agree with its lane level");
    model = std::move(read);
    summary = s;
    return true;
}

uint64_t levelRoadsKey(const bundle::LevelInputs& in) {
    registerCityProducer();
    const bundle::BundleProducer* p = bundle::findProducer(kCityProducerName);
    if (p == nullptr || !p->applies(in)) return 0;
    return p->identity(in).key;
}

LevelBuiltRoads obtainLevelRoadModel(const bundle::LevelInputs& in, const LaneGraphOptions& opts,
                                     const std::string& cacheDir) {
    const std::chrono::steady_clock::time_point t0 = std::chrono::steady_clock::now();
    LevelBuiltRoads out;
    const std::vector<CityEntity> cities = cityEntities(in.level);
    if (cities.empty()) {
        out.error = "the level has no lane-built city";
        out.seconds = secondsSince(t0);
        return out;
    }
    for (std::size_t i = 1; i < cities.size(); ++i)
        out.notes.push_back("city " + std::to_string(cities[i].ordinal) + " (" +
                            (cities[i].inlineGraph ? std::string("an inline graph") : cities[i].graphPath) +
                            ") has no lane graph: the driving model drives the first city only");
    // The roads' key, and the first city's control plan (the roads' key no longer sees it: keyedRoadFileText, ADR-0421)
    {
        nlohmann::json spec;
        if (cities.front().inlineGraph) spec = cities.front().block;
        else {
            std::ifstream gf(cities.front().graphPath);
            try { gf >> spec; } catch (const std::exception&) { spec = nlohmann::json(); }
        }
        // the layout and the lane rules the lane level reads by id (the roads' key leaves them out), and the table the
        // turns are stated by (ADR-0426)
        out.key = roadModelCacheKey(levelRoadsKey(in), laneLevelKey(spec), laneTurnRules().hash, opts);
    }

    if (!cacheDir.empty()) {
        out.path = roadModelCachePath(cacheDir, out.key);
        std::error_code ec;
        if (fs::exists(out.path, ec)) {
            RoadModel read;
            std::string why;
            if (readRoadModelCache(out.path, out.key, read, out.summary, &why)) {
                out.model = std::make_shared<RoadModel>(std::move(read));
                out.fromCache = true;
                bundle::noteCacheUse(out.path, kRoadModelCacheKind);
                out.seconds = secondsSince(t0);
                return out;
            }
            out.notes.push_back("the saved road model was refused and is built again: " + why);
        }
    }

    const CityEntity& city = cities.front();
    std::unique_ptr<Result> roads;
    try {
        BuildOptions bo;
        bo.threads = in.threads;
        roads = build(loadCityGraph(city), bo);
    } catch (const std::exception& ex) {
        out.error = "the road build failed: " + std::string(ex.what());
        out.seconds = secondsSince(t0);
        return out;
    }
    out.roadSeconds = secondsSince(t0);
    const std::chrono::steady_clock::time_point tg = std::chrono::steady_clock::now();
    // THE BUILT MODEL of this build, its lane level with it (ADR-0429)
    LaneGraphBuild built;
    std::shared_ptr<RoadModel> model = std::make_shared<RoadModel>(assembleRoadModel(*roads, opts, &built));
    roads.reset();
    out.graphSeconds = secondsSince(tg);
    if (!built.error.empty()) {
        out.error = built.error;
        out.seconds = secondsSince(t0);
        return out;
    }
    out.summary = summarizeLaneGraph(model->laneLevel, built);
    // A lane level that fails its own check would be refused by the reader on every later load and rebuilt each time: say
    // so now, and drive on it (the check is about the file as much as the graph).
    const std::vector<std::string> problems = validate(model->laneLevel);
    if (!problems.empty())
        out.notes.push_back("the lane graph built fails its check (" + std::to_string(problems.size()) +
                            " problems, the first: " + problems.front() + "); it is not saved");
    out.model = std::move(model);
    if (!out.path.empty() && problems.empty()) {
        std::string why;
        if (writeRoadModelCache(out.path, out.key, *out.model, out.summary, &why)) {
            out.saved = true;
            bundle::noteCacheUse(out.path, kRoadModelCacheKind);
        } else {
            out.notes.push_back(why);
        }
    }
    out.seconds = secondsSince(t0);
    return out;
}

std::string describeLevelRoadModel(const LevelBuiltRoads& lg) {
    if (!lg.model) return "no lane graph: " + lg.error;
    char how[512];
    if (lg.fromCache)
        std::snprintf(how, sizeof how, "; read from the cache %s in %.2f s (no road build)", lg.path.c_str(), lg.seconds);
    else
        std::snprintf(how, sizeof how, "; built in %.1f s (roads %.1f s, lane graph %.1f s)%s%s", lg.seconds,
                      lg.roadSeconds, lg.graphSeconds, lg.saved ? ", saved to " : lg.path.empty() ? ", not cached" : ", NOT saved",
                      lg.saved ? lg.path.c_str() : "");
    return describeLaneGraphSummary(lg.summary) + how;
}

}  // namespace roads::lanes
}  // namespace engine
