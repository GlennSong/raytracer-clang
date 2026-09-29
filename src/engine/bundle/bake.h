#ifndef RAYTRACER_ENGINE_BUNDLE_BAKE_H
#define RAYTRACER_ENGINE_BUNDLE_BAKE_H

// Baking level bundles (ADR-0084): producers, content keys, the bundle root, and the two entry points —
// bakeLevel() for the CLI and the editor, obtainForLevel() for the loader.
//
// A PRODUCER turns a level's inputs into named sections. It states its identity — a content key over the
// bytes of the inputs it reads plus a developer-bumped code tag — and the bake copies a producer's sections
// forward from any existing bundle whose manifest carries the same key, so changing one producer's inputs
// rebuilds that producer alone. The bundle directory is named by the combined key of every producer that
// applies to the level: <root>/<hex16>/level.bundle + manifest.json (the manifest is written last and is
// the commit marker).
//
// Root precedence: setBundleRoot() > $RT_BUNDLE_DIR > "cache/levels" (CWD-relative, like cache/terrain).
// Knobs the loader honours: RT_NOCACHE=1 (bake in memory, never read or write the disk), RT_BUNDLE_WRITE=0
// (read, but bake misses in memory), RT_BUNDLE_REQUIRE=1 (a miss is an error), RT_BUNDLE_REQUIRE_ENGINE=1
// (a bundle built by another engine version counts as a miss).

#include "engine/bundle/bundle.h"
#include <cstdint>
#include <functional>
#include <map>
#include <memory>
#include <string>
#include <vector>

namespace engine {
namespace bundle {

struct LevelInputs {
    std::string levelPath, levelDir;
    nlohmann::json level;
    unsigned threads = 0;   // 0 = the producer's default
};
bool loadLevelInputs(const std::string& levelPath, LevelInputs& out, std::string* err);

struct InputFile { std::string path; uint64_t bytes = 0; int64_t mtime = 0; uint64_t fnv = 0; };
struct ProducerIdentity { uint64_t key = 0; std::string tag; std::vector<InputFile> inputs; };

struct Progress { std::string producer, stage, message; double fraction = 0; };   // fraction: 0..1 of the whole bake
using ProgressFn = std::function<bool(const Progress&)>;                           // false = cancel

struct ProducerReport {
    bool ok = true;
    std::string error;
    double seconds = 0;
    std::map<std::string, double> timings;
    nlohmann::json report;   // free-form: summary text, invariants, counts — printed by rt_bake and kept in the manifest
};

class BundleProducer {
public:
    virtual ~BundleProducer() = default;
    virtual std::string name() const = 0;
    virtual bool applies(const LevelInputs& in) const = 0;
    virtual ProducerIdentity identity(const LevelInputs& in) const = 0;
    // Writes every section under "<name()>/...". The progress fraction it reports is LOCAL (0..1 of this producer).
    virtual ProducerReport produce(const LevelInputs& in, BundleWriter& out, const ProgressFn* progress) const = 0;
    virtual double weight() const { return 1.0; }   // relative cost, for the overall progress fraction
};

void registerProducer(std::unique_ptr<BundleProducer> p);
std::vector<const BundleProducer*> producers();
const BundleProducer* findProducer(const std::string& name);

std::string bundleRoot();
void setBundleRoot(const std::string& root);   // empty = back to env/default
uint64_t combinedKey(const std::vector<std::pair<std::string, uint64_t>>& producerKeys);
std::string bundleDirFor(const std::string& root, uint64_t combined);
// The manifest's entry for a producer, or null.
nlohmann::json manifestProducer(const nlohmann::json& manifest, const std::string& name);

struct BakeRequest {
    std::string levelPath;
    std::string outRoot;              // empty = bundleRoot()
    std::vector<std::string> only;    // producer names; empty = every producer that applies
    bool force = false;               // rebuild even when keys match
    bool toMemory = false;            // never touch the disk (RT_NOCACHE); the report carries the bytes
    unsigned threads = 0;
};
struct BakeReport {
    bool ok = false;
    std::string error, dir, keyHex;
    double seconds = 0;
    std::vector<std::string> built, reused;              // producer names
    std::map<std::string, ProducerReport> reports;       // per producer built this run
    std::shared_ptr<std::vector<uint8_t>> memory;        // toMemory only
    bool upToDate = false;                               // nothing had to be built or copied
};
BakeReport bakeLevel(const BakeRequest& req, const ProgressFn* progress = nullptr);

// The loader's side: the bundle holding the current sections of `producer` for this level, from disk when
// it is there and current, else baked now (per the knobs above). `status` is one line for logs and the
// `bundle?` control verb.
struct Obtained {
    std::shared_ptr<Bundle> bundle;
    std::string dir, status;
    bool hit = false, built = false;
    double seconds = 0;
};
Obtained obtainForLevel(const LevelInputs& in, const std::string& producer, const ProgressFn* progress = nullptr);

std::string lastBundleStatus();
void setLastBundleStatus(const std::string& s);

// The directory a level's bundle lives in today: <root>/<combined key of every registered producer that
// applies>. Empty when no producer applies. `root` empty = bundleRoot().
std::string bundleDirForLevel(const LevelInputs& in, const std::string& root = std::string());

// Housekeeping: every bundle under `root` that is not the newest for its level path (manifest "level.path",
// by "created") and not in `keep` (directories the caller resolved as current, e.g. bundleDirForLevel) is
// stale; a directory without a manifest is stale too (an aborted write). In-flight `*.tmp-<pid>` directories
// are never touched. `apply` deletes the stale ones; otherwise the report is a dry run.
// `level` is the manifest's level path made absolute (a test bakes with an absolute path, rt_bake with a
// repo-relative one; both are the same level). `extras` names what the directory holds besides the bundle
// itself — GLB exports — which a prune deletes along with it.
struct PruneEntry { std::string dir, level, created, key; uint64_t bytes = 0; bool stale = false, current = false; std::vector<std::string> extras; };
struct PruneReport { std::vector<PruneEntry> entries; uint64_t staleBytes = 0; size_t removed = 0; };
PruneReport pruneBundles(const std::string& root, const std::vector<std::string>& keep, bool apply);

// ONE LEVEL'S CACHE, for the editor's Level cache panel (Glenn: "the editor should show if there is a bake for a
// level, how current it is, the size and have options to build it, rebuild it, delete it"). `currentDir` is
// where today's bake of the level lives (bundleDirForLevel); `current` says whether it is there. `older` are
// the level's other bundles: all out of date, since the level, its inputs or the engine changed since they
// were written. The cache totals are the whole root's, as a dry-run prune sees it (the level's current bundle
// counted as kept). Sizes are file sizes; a compressing filesystem stores them in less.
struct LevelCacheStatus {
    std::string root, levelAbs, currentDir, currentCreated, error;
    bool applies = false;   // a producer applies to this level (it has a city to bake)
    bool current = false;   // today's bake is on disk
    uint64_t currentBytes = 0;
    std::vector<PruneEntry> older;
    uint64_t olderBytes = 0;
    std::size_t cacheBundles = 0, cacheStale = 0;
    uint64_t cacheBytes = 0, cacheStaleBytes = 0;
};
LevelCacheStatus levelCacheStatus(const std::string& levelPath, const std::string& root = std::string());
// Delete these bundle directories -- each must sit DIRECTLY under `root` (anything else is refused, as is an
// in-flight `*.tmp-<pid>` write). Returns how many went; `err` names the first refusal or failure.
// EVERY BAKE in the root, for the editor's All bakes window (Glenn: "it would be nice to see all of the bakes
// somewhere"): each bundle's directory, the level it was baked from, when and how big, and -- when `classify`
// -- whether it is that level's current bake (the level as it is today would load from it), out of date, or
// an orphan (its level file is gone). classify reads each distinct level's inputs once (~0.2 s a level);
// without it every bake is Unknown and the listing is manifest reads only. Newest first within a level.
struct BakeListing {
    enum class State { Unknown, Current, OutOfDate, Orphan, Broken };   // Broken: no manifest or no bundle file
    std::string dir, level, created;
    uint64_t bytes = 0;
    State state = State::Unknown;
};
std::vector<BakeListing> listBakes(const std::string& root = std::string(), bool classify = true);
const char* bakeStateName(BakeListing::State s);

std::size_t deleteBundleDirs(const std::string& root, const std::vector<std::string>& dirs, std::string* err = nullptr);

}  // namespace bundle
}  // namespace engine

#endif
