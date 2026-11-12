#include "engine/procgen/city/roads/lanes/lane_graph_cache.h"

#include "engine/bundle/bundle.h"
#include "engine/bundle/cache_use.h"
#include "engine/procgen/city/roads/lanes/city_producer.h"   // the roads' key, the city entities, loadCityGraph

#include <nlohmann/json.hpp>

#include <chrono>
#include <cstdio>
#include <exception>
#include <filesystem>
#include <utility>

#ifndef _WIN32
#include <unistd.h>
#endif

namespace engine {
namespace roads::lanes {

namespace {

namespace fs = std::filesystem;

// The file's two sections and the manifest field that names what it is.
constexpr const char* kGraphSection = "lanegraph/graph";
constexpr const char* kSummarySection = "lanegraph/summary";
constexpr const char* kManifestField = "lanegraph";

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

LaneGraphSummary summarizeLaneGraph(const LaneGraphBuild& built) {
    LaneGraphSummary s;
    countGraph(built.graph, s);
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

uint64_t laneGraphCacheKey(uint64_t roadsKey, const LaneGraphOptions& opts, uint32_t builderVersion) {
    bundle::Fnv k;
    k.str("lanegraph");
    k.pod(kLaneGraphCacheFormat);
    k.pod(builderVersion);
    k.pod(roadsKey);
    // Each string's length goes in before it, so ("ab", "c") and ("a", "bc") are different keys.
    k.pod(static_cast<uint64_t>(opts.control.size()));
    k.str(opts.control);
    k.pod(static_cast<uint64_t>(opts.vehicles.size()));
    for (const VehicleSize& v : opts.vehicles) {
        k.pod(static_cast<uint64_t>(v.tag.size()));
        k.str(v.tag);
        k.pod(v.turningRadius);
        k.pod(v.width);
    }
    return k.h;
}

std::string laneGraphCachePath(const std::string& dir, uint64_t key) {
    return dir + "/" + bundle::hex16(key) + ".lanegraph";
}

bool writeLaneGraphCache(const std::string& path, uint64_t key, const LaneGraph& graph, const LaneGraphSummary& summary,
                         std::string* err) {
    std::error_code ec;
    const fs::path parent = fs::path(path).parent_path();
    if (!parent.empty()) fs::create_directories(parent, ec);
    const std::string tmp = path + ".tmp-" + processTag();
    {
        bundle::BundleWriter w;
        std::string why;
        if (!w.openFile(tmp, &why)) return fail(err, "lane graph cache: " + why);
        nlohmann::json manifest;
        manifest[kManifestField] = {{"format", kLaneGraphCacheFormat}, {"key", bundle::hex16(key)}};
        if (!writeLaneGraph(w, kGraphSection, graph) || !w.addJson(kSummarySection, summaryJson(summary)) ||
            !w.finish(manifest, &why)) {
            fs::remove(tmp, ec);
            return fail(err, "lane graph cache: cannot write " + tmp + (why.empty() ? std::string() : ": " + why));
        }
    }
    fs::rename(tmp, path, ec);
    if (ec) {
        fs::remove(tmp, ec);
        return fail(err, "lane graph cache: cannot move " + tmp + " into place");
    }
    return true;
}

bool readLaneGraphCache(const std::string& path, uint64_t key, LaneGraph& graph, LaneGraphSummary& summary,
                        std::string* err) {
    graph = LaneGraph();
    summary = LaneGraphSummary();
    std::error_code ec;
    if (!fs::is_regular_file(path, ec)) return fail(err, path + ": no such file");
    std::string why;
    const std::unique_ptr<bundle::Bundle> b = bundle::Bundle::open(path, &why);
    if (!b) return fail(err, why);
    const nlohmann::json& m = b->manifest();
    if (!m.contains(kManifestField) || !m[kManifestField].is_object())
        return fail(err, path + ": not a lane graph cache (its table of contents names no lane graph)");
    const nlohmann::json& lg = m[kManifestField];
    if (lg.value("format", 0u) != kLaneGraphCacheFormat)
        return fail(err, path + ": layout " + std::to_string(lg.value("format", 0u)) + ", this build reads " +
                             std::to_string(kLaneGraphCacheFormat));
    if (lg.value("key", std::string()) != bundle::hex16(key))
        return fail(err, path + ": made for key " + lg.value("key", std::string("(none)")) + ", not " +
                             bundle::hex16(key));
    for (const char* section : {kGraphSection, kSummarySection})
        if (!b->checkSection(section))
            return fail(err, path + ": section " + section + " is missing or its bytes no longer match their hash");
    LaneGraph g;
    if (!readLaneGraph(*b, kGraphSection, g, &why)) return fail(err, path + ": " + why);
    LaneGraphSummary s;
    if (!summaryFromJson(b->json(kSummarySection), s)) return fail(err, path + ": its summary is unreadable");
    LaneGraphSummary counted = s;
    countGraph(g, counted);
    if (!(counted == s)) return fail(err, path + ": its summary does not agree with its graph");
    graph = std::move(g);
    summary = s;
    return true;
}

uint64_t levelRoadsKey(const bundle::LevelInputs& in) {
    registerCityProducer();
    const bundle::BundleProducer* p = bundle::findProducer(kCityProducerName);
    if (p == nullptr || !p->applies(in)) return 0;
    return p->identity(in).key;
}

LevelLaneGraph obtainLevelLaneGraph(const bundle::LevelInputs& in, const LaneGraphOptions& opts,
                                    const std::string& cacheDir) {
    const std::chrono::steady_clock::time_point t0 = std::chrono::steady_clock::now();
    LevelLaneGraph out;
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
    out.key = laneGraphCacheKey(levelRoadsKey(in), opts);

    if (!cacheDir.empty()) {
        out.path = laneGraphCachePath(cacheDir, out.key);
        std::error_code ec;
        if (fs::exists(out.path, ec)) {
            LaneGraph g;
            std::string why;
            if (readLaneGraphCache(out.path, out.key, g, out.summary, &why)) {
                out.graph = std::make_shared<const LaneGraph>(std::move(g));
                out.fromCache = true;
                bundle::noteCacheUse(out.path, kLaneGraphCacheKind);
                out.seconds = secondsSince(t0);
                return out;
            }
            out.notes.push_back("the saved lane graph was refused and is built again: " + why);
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
    LaneGraphBuild built = buildLaneGraph(*roads, opts);
    roads.reset();
    out.graphSeconds = secondsSince(tg);
    out.summary = summarizeLaneGraph(built);
    // A graph that fails its own check would be refused by the reader on every later load and rebuilt each time: say so
    // now, and drive on it (the check is about the file as much as the graph).
    const std::vector<std::string> problems = validate(built.graph);
    if (!problems.empty())
        out.notes.push_back("the lane graph built fails its check (" + std::to_string(problems.size()) +
                            " problems, the first: " + problems.front() + "); it is not saved");
    out.graph = std::make_shared<const LaneGraph>(std::move(built.graph));
    if (!out.path.empty() && problems.empty()) {
        std::string why;
        if (writeLaneGraphCache(out.path, out.key, *out.graph, out.summary, &why)) {
            out.saved = true;
            bundle::noteCacheUse(out.path, kLaneGraphCacheKind);
        } else {
            out.notes.push_back(why);
        }
    }
    out.seconds = secondsSince(t0);
    return out;
}

std::string describeLevelLaneGraph(const LevelLaneGraph& lg) {
    if (!lg.graph) return "no lane graph: " + lg.error;
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
