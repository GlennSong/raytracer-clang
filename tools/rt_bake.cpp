// rt_bake — prebuild a level's bundle (ADR-0084) from the command line.
//
//   rt_bake <level.json> [--out <root>] [--producer a,b] [--glb] [--glb-split cell] [--force] [--threads N] [--require]
//   rt_bake --inspect <bundle-dir>
//   rt_bake --prune [--yes] [--out <root>] [level.json ...]
//
// Runs every registered producer that applies to the level (the procedural city, then its lots), copying unchanged
// producers' sections forward from earlier bundles, and writes <root>/<key>/level.bundle + manifest.json.
// The same bakeLevel() the editor's "Bake level cache" button calls. Run from the repo root: level files
// name their graphs relative to it, like the viewer.

#include "engine/bundle/bake.h"
#include "engine/procgen/city/citylots_producer.h"
#include "engine/bundle/bundle_glb.h"
#ifdef RT_ENABLE_LANELAB
#include "engine/procgen/lanelab/city_producer.h"
#include "engine/procgen/lanelab/lots_producer.h"
#endif

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <string>
#include <vector>

#ifndef _WIN32
#include <unistd.h>
#endif

using namespace engine::bundle;

namespace {
int usage() {
    std::fprintf(stderr,
        "usage: rt_bake <level.json> [--out <root>] [--producer a,b] [--glb] [--glb-split cell] [--force] [--threads N] [--require]\n"
        "       rt_bake --inspect <bundle-dir>\n"
        "       rt_bake --prune [--yes] [--out <root>] [level.json ...]   list stale bundles under the root; --yes deletes them\n"
        "                    (stale = not the newest for its level path, and not the current bundle of a listed level)\n"
        "  --out        bundle root (default: $RT_BUNDLE_DIR or cache/levels)\n"
        "  --producer   only these producers (default: every one that applies)\n"
        "  --glb        also write city.glb (one object per material) into the bundle dir\n"
        "  --glb-split cell   also write cells/<cx>_<cz>.glb + index.json (one file per render cell)\n"
        "  --force      rebuild even when the keys match\n"
        "  --threads N  worker threads for the builders (default: every hardware thread)\n"
        "  --require    exit 1 instead of building when the bundle is missing or stale\n"
        "  --check      also run the city's invariant sweep into the report (minutes on metro)\n");
    return 2;
}

bool tty() {
#ifndef _WIN32
    return isatty(2) != 0;
#else
    return false;
#endif
}

std::string humanBytes(uint64_t b) { char s[32]; if (b >= (1ull << 30)) std::snprintf(s, sizeof(s), "%.2f GB", b / 1073741824.0); else if (b >= (1ull << 20)) std::snprintf(s, sizeof(s), "%.1f MB", b / 1048576.0); else std::snprintf(s, sizeof(s), "%.0f KB", b / 1024.0); return s; }

int inspect(const std::string& dir) {
    std::string err; std::unique_ptr<Bundle> b = Bundle::open(dir + "/" + kBundleFile, &err);
    if (!b) { std::fprintf(stderr, "rt_bake: %s\n", err.c_str()); return 1; }
    const nlohmann::json& m = b->manifest();
    std::printf("%s: %s, format %u.%u, key %s, created %s\n", b->path().c_str(), humanBytes(b->bytes()).c_str(), m["format"].value("major", 0u), m["format"].value("minor", 0u), m.value("key", std::string("?")).c_str(), m.value("created", std::string("?")).c_str());
    const nlohmann::json eng = m.value("engine", nlohmann::json::object());
    std::printf("engine %s (%s, %s, %s, Real %d)\n", eng.value("version", std::string("?")).c_str(), eng.value("buildType", std::string("?")).c_str(), eng.value("compiler", std::string("?")).c_str(), eng.value("platform", std::string("?")).c_str(), eng.value("real", 0));
    std::printf("level %s\n", m.value("level", nlohmann::json::object()).value("path", std::string("?")).c_str());
    for (const nlohmann::json& p : m.value("producers", nlohmann::json::array())) {
        std::printf("producer %-8s key %s tag %s %s %.1f s%s\n", p.value("name", std::string("?")).c_str(), p.value("key", std::string("?")).c_str(), p.value("tag", std::string("?")).c_str(),
                    p.contains("copiedFrom") ? "copied" : "built", p.value("seconds", 0.0), p.contains("copiedFrom") ? (" from " + p["copiedFrom"].get<std::string>()).c_str() : "");
        for (const nlohmann::json& in : p.value("inputs", nlohmann::json::array())) std::printf("  input %s (%s, fnv %s)\n", in.value("path", std::string("?")).c_str(), humanBytes(in.value("bytes", 0ull)).c_str(), in.value("fnv", std::string("?")).c_str());
    }
    uint64_t total = 0; std::vector<SectionInfo> secs = b->sectionTable(); for (const SectionInfo& s : secs) total += s.size;
    std::printf("%zu sections, %s; largest:\n", secs.size(), humanBytes(total).c_str());
    std::sort(secs.begin(), secs.end(), [](const SectionInfo& a, const SectionInfo& c) { return a.size > c.size; });
    for (size_t i = 0; i < secs.size() && i < 8; ++i) std::printf("  %-48s %s\n", secs[i].name.c_str(), humanBytes(secs[i].size).c_str());
    size_t cells = 0; for (const std::string& s : b->sections("")) if (s.find("/cell/") != std::string::npos && s.find("/mesh/") != std::string::npos) ++cells;
    std::printf("%zu cell mesh sections\n", cells);
    return 0;
}
}  // namespace

int main(int argc, char** argv) {
    if (argc < 2) return usage();
    engine::registerCityLotsProducer();
#ifdef RT_ENABLE_LANELAB
    engine::lanelab::registerCityProducer();
    engine::lanelab::registerLotsProducer();
#endif
    if (std::strcmp(argv[1], "--inspect") == 0) { if (argc < 3) return usage(); return inspect(argv[2]); }
    if (std::strcmp(argv[1], "--prune") == 0) {
        bool yes = false; std::string outRoot; std::vector<std::string> levels;
        for (int i = 2; i < argc; ++i) { const std::string a = argv[i]; if (a == "--yes") yes = true; else if (a == "--out" && i + 1 < argc) outRoot = argv[++i]; else if (!a.empty() && a[0] == '-') return usage(); else levels.push_back(a); }
        const std::string root = outRoot.empty() ? bundleRoot() : outRoot;
        std::vector<std::string> keep;
        for (const std::string& lv : levels) {   // a listed level's CURRENT bundle is kept even when an older one is newer by clock
            LevelInputs in; std::string err; if (!loadLevelInputs(lv, in, &err)) { std::fprintf(stderr, "rt_bake: %s\n", err.c_str()); return 1; }
            const std::string d = bundleDirForLevel(in, root); if (!d.empty()) keep.push_back(d);
        }
        const PruneReport rep = pruneBundles(root, keep, yes);
        for (const PruneEntry& e : rep.entries) {
            std::string extras; for (const std::string& x : e.extras) extras += (extras.empty() ? "  + " : ", ") + x;
            std::printf("%-7s %9s  %s  %s  %s%s\n", e.stale ? "stale" : (e.current ? "current" : "keep"), humanBytes(e.bytes).c_str(), e.dir.c_str(), e.created.empty() ? "-" : e.created.c_str(), e.level.c_str(), extras.c_str());
        }
        size_t stale = 0; for (const PruneEntry& e : rep.entries) stale += e.stale;
        std::printf("%zu bundles under %s, %zu stale (%s)\n", rep.entries.size(), root.c_str(), stale, humanBytes(rep.staleBytes).c_str());
        if (!yes && stale) std::printf("dry run: pass --yes to delete the stale bundles\n"); else if (yes) std::printf("removed %zu\n", rep.removed);
        return 0;
    }
    BakeRequest req; req.levelPath = argv[1]; bool glb = false, split = false, require = false;
    for (int i = 2; i < argc; ++i) {
        const std::string a = argv[i];
        if (a == "--out" && i + 1 < argc) req.outRoot = argv[++i];
        else if (a == "--producer" && i + 1 < argc) { std::string list = argv[++i]; size_t p; while ((p = list.find(',')) != std::string::npos) { req.only.push_back(list.substr(0, p)); list.erase(0, p + 1); } if (!list.empty()) req.only.push_back(list); }
        else if (a == "--glb") glb = true;
        else if (a == "--glb-split" && i + 1 < argc) { if (std::string(argv[++i]) != "cell") return usage(); split = true; }
        else if (a == "--force") req.force = true;
        else if (a == "--threads" && i + 1 < argc) req.threads = static_cast<unsigned>(std::max(0, std::atoi(argv[++i])));
        else if (a == "--require") require = true;
        else if (a == "--check") setenv("RT_CITY_CHECK", "1", 1);   // run the lab's invariant sweep into the report (slow on metro)
        else return usage();
    }
    if (producers().empty()) { std::fprintf(stderr, "rt_bake: no producers are registered in this build (RT_ENABLE_LANELAB off?)\n"); return 1; }
    if (require) {
        LevelInputs in; std::string err; if (!loadLevelInputs(req.levelPath, in, &err)) { std::fprintf(stderr, "rt_bake: %s\n", err.c_str()); return 1; }
        setenv("RT_BUNDLE_REQUIRE", "1", 1); if (!req.outRoot.empty()) setBundleRoot(req.outRoot);
        for (const BundleProducer* p : producers()) { if (!p->applies(in) || (!req.only.empty() && std::find(req.only.begin(), req.only.end(), p->name()) == req.only.end())) continue;
            Obtained o = obtainForLevel(in, p->name()); std::printf("%s: %s\n", p->name().c_str(), o.status.c_str()); if (!o.bundle) return 1; }
        return 0;
    }
    const bool live = tty(); std::string lastStage; int lastPct = -1;
    ProgressFn progress = [&](const Progress& p) {
        const std::string stage = p.producer + "/" + p.stage;
        if (stage != lastStage) { if (live && lastPct >= 0) std::fprintf(stderr, "\r%*s\r", 78, ""); std::fprintf(stderr, "[%s]%s%s\n", stage.c_str(), p.message.empty() ? "" : " ", p.message.c_str()); lastStage = stage; }
        const int pct = static_cast<int>(p.fraction * 100.0 + 0.5);
        if (live && pct != lastPct) { std::fprintf(stderr, "\r%3d%%  %s", pct, stage.c_str()); std::fflush(stderr); lastPct = pct; }
        return true;
    };
    const BakeReport rep = bakeLevel(req, &progress);
    if (live && lastPct >= 0) std::fprintf(stderr, "\r%*s\r", 78, "");
    if (!rep.ok) { std::fprintf(stderr, "rt_bake: %s\n", rep.error.c_str()); return 1; }
    for (const auto& kv : rep.reports) {
        const nlohmann::json& r = kv.second.report;
        for (const nlohmann::json& e : r.value("entities", nlohmann::json::array())) {
            const std::string sum = e.value("summary", std::string()); std::printf("%s: %s\n", kv.first.c_str(), sum.substr(0, sum.find('\n')).c_str());
            for (const nlohmann::json& inv : e.value("invariants", nlohmann::json::array())) if (!inv.value("ok", true)) std::printf("  FAIL %s: %s\n", inv.value("name", std::string()).c_str(), inv.value("detail", std::string()).substr(0, 160).c_str());
        }
    }
    if (rep.upToDate) std::printf("up to date: %s\n", rep.dir.c_str());
    else {
        uint64_t bytes = 0; std::error_code ec; bytes = std::filesystem::file_size(rep.dir + "/" + kBundleFile, ec);
        std::string built; for (const std::string& n : rep.built) built += (built.empty() ? "" : ", ") + n; std::string reused; for (const std::string& n : rep.reused) reused += (reused.empty() ? "" : ", ") + n;
        std::printf("wrote %s (%s, %.1f s; built %s%s%s)\n", rep.dir.c_str(), humanBytes(bytes).c_str(), rep.seconds, built.empty() ? "nothing" : built.c_str(), reused.empty() ? "" : "; reused ", reused.c_str());
    }
    if (glb || split) {
        std::string err; std::unique_ptr<Bundle> b = Bundle::open(rep.dir + "/" + kBundleFile, &err);
        if (!b) { std::fprintf(stderr, "rt_bake: %s\n", err.c_str()); return 1; }
        GlbProgressFn glbProgress = [&](double f) { Progress p; p.producer = "glb"; p.stage = split && glb ? "export" : "export"; p.fraction = f; return progress(p); };
        if (glb) { lastStage.clear(); lastPct = -1; const std::string path = rep.dir + "/city.glb"; if (!writeBundleGlb(*b, path, &err, &glbProgress)) { std::fprintf(stderr, "rt_bake: glb: %s\n", err.c_str()); return 1; } if (live) std::fprintf(stderr, "\r%*s\r", 78, ""); std::error_code ec; std::printf("wrote %s (%s)\n", path.c_str(), humanBytes(std::filesystem::file_size(path, ec)).c_str()); }
        if (split) { lastStage.clear(); lastPct = -1; std::vector<std::string> files; if (!writeBundleGlbCells(*b, rep.dir + "/cells", &err, &files, &glbProgress)) { std::fprintf(stderr, "rt_bake: glb cells: %s\n", err.c_str()); return 1; } if (live) std::fprintf(stderr, "\r%*s\r", 78, ""); std::printf("wrote %zu cell GLBs under %s/cells (index.json lists them)\n", files.size(), rep.dir.c_str()); }
    }
    return 0;
}
