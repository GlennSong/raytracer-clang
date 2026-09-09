#include "engine/bundle/bake.h"

#include "log.h"

#include <algorithm>
#include <chrono>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <mutex>

#ifndef _WIN32
#include <unistd.h>
#endif

namespace engine {
namespace bundle {

namespace fs = std::filesystem;

namespace {
std::vector<std::unique_ptr<BundleProducer>>& registry() { static std::vector<std::unique_ptr<BundleProducer>> r; return r; }
std::string& rootOverride() { static std::string s; return s; }
std::mutex& statusMutex() { static std::mutex m; return m; }
std::string& statusLine() { static std::string s = "none"; return s; }

double secondsSince(const std::chrono::steady_clock::time_point& t) { return std::chrono::duration<double>(std::chrono::steady_clock::now() - t).count(); }
bool envFlag(const char* name, bool defaultValue = false) {
    const char* v = std::getenv(name); if (!v || !*v) return defaultValue; return !(v[0] == '0' && v[1] == '\0');
}
int pid() {
#ifndef _WIN32
    return static_cast<int>(::getpid());
#else
    return 0;
#endif
}

nlohmann::json readManifest(const std::string& dir) {
    std::ifstream f(dir + "/" + kManifestFile); if (!f) return nlohmann::json();
    try { nlohmann::json j; f >> j; return j; } catch (const std::exception&) { return nlohmann::json(); }
}

nlohmann::json inputsJson(const std::vector<InputFile>& inputs) {
    nlohmann::json a = nlohmann::json::array();
    for (const InputFile& f : inputs) a.push_back({{"path", f.path}, {"bytes", f.bytes}, {"mtime", f.mtime}, {"fnv", hex16(f.fnv)}});
    return a;
}

struct Applicable { const BundleProducer* p; ProducerIdentity id; };
std::vector<Applicable> applicableProducers(const LevelInputs& in, const std::vector<std::string>& only) {
    std::vector<Applicable> out;
    for (const BundleProducer* p : producers()) {
        if (!only.empty() && std::find(only.begin(), only.end(), p->name()) == only.end()) continue;
        if (!p->applies(in)) continue;
        out.push_back({p, p->identity(in)});
    }
    std::sort(out.begin(), out.end(), [](const Applicable& a, const Applicable& b) { return a.p->name() < b.p->name(); });
    return out;
}
uint64_t keyOf(const std::vector<Applicable>& aps) {
    std::vector<std::pair<std::string, uint64_t>> ks; for (const Applicable& a : aps) ks.emplace_back(a.p->name(), a.id.key); return combinedKey(ks);
}

// A bundle under `root` whose manifest carries `name` with `key`: its directory, or empty.
std::string findDonor(const std::string& root, const std::string& name, uint64_t key, const std::string& preferDir) {
    auto matches = [&](const std::string& dir) {
        const nlohmann::json m = readManifest(dir); if (m.is_null()) return false;
        if (m.value("format", nlohmann::json::object()).value("major", 0u) != kFormatMajor) return false;
        const nlohmann::json p = manifestProducer(m, name); return !p.is_null() && p.value("key", std::string()) == hex16(key) && fs::exists(dir + "/" + kBundleFile);
    };
    if (!preferDir.empty() && matches(preferDir)) return preferDir;
    std::error_code ec;
    for (const fs::directory_entry& e : fs::directory_iterator(root, ec)) {
        if (!e.is_directory()) continue; const std::string dir = e.path().string(); if (dir == preferDir) continue;
        if (matches(dir)) return dir;
    }
    return std::string();
}
}  // namespace

bool loadLevelInputs(const std::string& levelPath, LevelInputs& out, std::string* err) {
    std::ifstream f(levelPath); if (!f) { if (err) *err = "cannot open " + levelPath; return false; }
    try { f >> out.level; } catch (const std::exception& ex) { if (err) *err = levelPath + ": " + ex.what(); return false; }
    out.levelPath = levelPath; const size_t slash = levelPath.find_last_of('/'); out.levelDir = slash == std::string::npos ? "." : levelPath.substr(0, slash);
    return true;
}

void registerProducer(std::unique_ptr<BundleProducer> p) {
    if (!p) return; for (const auto& q : registry()) if (q->name() == p->name()) return;   // idempotent per name
    registry().push_back(std::move(p));
}
std::vector<const BundleProducer*> producers() { std::vector<const BundleProducer*> v; for (const auto& p : registry()) v.push_back(p.get()); return v; }
const BundleProducer* findProducer(const std::string& name) { for (const auto& p : registry()) if (p->name() == name) return p.get(); return nullptr; }

std::string bundleRoot() {
    if (!rootOverride().empty()) return rootOverride();
    if (const char* e = std::getenv("RT_BUNDLE_DIR")) if (*e) return e;
    return "cache/levels";
}
void setBundleRoot(const std::string& root) { rootOverride() = root; }

uint64_t combinedKey(const std::vector<std::pair<std::string, uint64_t>>& producerKeys) {
    std::vector<std::pair<std::string, uint64_t>> ks = producerKeys; std::sort(ks.begin(), ks.end());
    uint64_t h = fnv1aStr("rt-level-bundle"); h = fnv1a(&kFormatMajor, sizeof(kFormatMajor), h);
    for (const auto& k : ks) { h = fnv1aStr(k.first, h); h = fnv1a(&k.second, sizeof(k.second), h); }
    return h;
}
std::string bundleDirFor(const std::string& root, uint64_t combined) { return root + "/" + hex16(combined); }

nlohmann::json manifestProducer(const nlohmann::json& manifest, const std::string& name) {
    if (!manifest.contains("producers") || !manifest["producers"].is_array()) return nlohmann::json();
    for (const nlohmann::json& p : manifest["producers"]) if (p.value("name", std::string()) == name) return p;
    return nlohmann::json();
}

BakeReport bakeLevel(const BakeRequest& req, const ProgressFn* progress) {
    BakeReport rep; const auto t0 = std::chrono::steady_clock::now();
    LevelInputs in; if (!loadLevelInputs(req.levelPath, in, &rep.error)) return rep;
    in.threads = req.threads;
    const std::vector<Applicable> aps = applicableProducers(in, req.only);
    if (aps.empty()) { rep.error = "no registered producer applies to " + req.levelPath; return rep; }
    const std::string root = req.outRoot.empty() ? bundleRoot() : req.outRoot;
    const uint64_t key = keyOf(aps); rep.keyHex = hex16(key); rep.dir = bundleDirFor(root, key);

    // Already there and current?
    if (!req.force && !req.toMemory) {
        const nlohmann::json m = readManifest(rep.dir);
        bool all = !m.is_null() && fs::exists(rep.dir + "/" + kBundleFile);
        for (const Applicable& a : aps) { if (!all) break; const nlohmann::json p = manifestProducer(m, a.p->name()); all = !p.is_null() && p.value("key", std::string()) == hex16(a.id.key); }
        if (all) { rep.ok = true; rep.upToDate = true; for (const Applicable& a : aps) rep.reused.push_back(a.p->name()); rep.seconds = secondsSince(t0); return rep; }
    }

    // Write into a temp dir (or memory), producer by producer, copying current sections forward.
    std::string tmpDir; BundleWriter w;
    if (req.toMemory) w.openMemory();
    else {
        std::error_code ec; fs::create_directories(root, ec);
        tmpDir = rep.dir + ".tmp-" + std::to_string(pid()); fs::remove_all(tmpDir, ec); fs::create_directories(tmpDir, ec);
        if (!w.openFile(tmpDir + "/" + kBundleFile, &rep.error)) return rep;
    }
    double totalWeight = 0; for (const Applicable& a : aps) totalWeight += std::max(1e-6, a.p->weight());
    double doneWeight = 0; nlohmann::json producersJson = nlohmann::json::array();
    for (const Applicable& a : aps) {
        const std::string name = a.p->name(); const double wgt = std::max(1e-6, a.p->weight());
        std::string donor = req.force || req.toMemory ? std::string() : findDonor(root, name, a.id.key, rep.dir);
        nlohmann::json entry = {{"name", name}, {"key", hex16(a.id.key)}, {"tag", a.id.tag}, {"inputs", inputsJson(a.id.inputs)}};
        if (!donor.empty()) {
            std::string err; std::unique_ptr<Bundle> from = Bundle::open(donor + "/" + kBundleFile, &err);
            if (from) {
                for (const std::string& s : from->sections(name + "/")) { const Bundle::View v = from->section(s); w.add(s, v.data, v.size); }
                const nlohmann::json old = manifestProducer(from->manifest(), name);
                for (const char* k : {"seconds", "timings", "report", "built"}) if (old.contains(k)) entry[k] = old[k];
                entry["copiedFrom"] = donor; rep.reused.push_back(name); producersJson.push_back(entry); doneWeight += wgt;
                if (progress) { Progress p; p.producer = name; p.stage = "reused"; p.fraction = doneWeight / totalWeight; (*progress)(p); }
                continue;
            }
            LOG_WARN << "[bundle] donor " << donor << " unreadable (" << err << "); rebuilding " << name;
        }
        ProgressFn local; const ProgressFn* localPtr = nullptr;
        if (progress) { local = [&](const Progress& p) { Progress q = p; q.producer = name; q.fraction = (doneWeight + wgt * std::clamp(p.fraction, 0.0, 1.0)) / totalWeight; return (*progress)(q); }; localPtr = &local; }
        const auto tp = std::chrono::steady_clock::now();
        ProducerReport pr;
        try { pr = a.p->produce(in, w, localPtr); } catch (const std::exception& ex) { pr.ok = false; pr.error = ex.what(); }
        pr.seconds = secondsSince(tp);
        if (!pr.ok) {
            rep.error = name + ": " + (pr.error.empty() ? "failed" : pr.error);
            if (!tmpDir.empty()) { std::error_code ec; fs::remove_all(tmpDir, ec); }
            rep.seconds = secondsSince(t0); return rep;
        }
        entry["seconds"] = pr.seconds; entry["timings"] = pr.timings; entry["report"] = pr.report; entry["built"] = isoNowUtc();
        rep.reports[name] = pr; rep.built.push_back(name); producersJson.push_back(entry); doneWeight += wgt;
    }
    uint64_t levelFnv = kFnvOffset; uint64_t levelBytes = 0; int64_t levelMtime = 0; fnv1aFile(req.levelPath, levelFnv, &levelBytes, &levelMtime);
    nlohmann::json manifest = {{"kind", "rt-level-bundle"}, {"created", isoNowUtc()}, {"engine", engineIdentityJson()}, {"key", rep.keyHex},
                               {"level", {{"path", req.levelPath}, {"bytes", levelBytes}, {"mtime", levelMtime}, {"fnv", hex16(levelFnv)}}},
                               {"producers", producersJson}, {"threads", req.threads}};
    if (!w.finish(manifest, &rep.error)) { if (!tmpDir.empty()) { std::error_code ec; fs::remove_all(tmpDir, ec); } return rep; }
    if (req.toMemory) { rep.memory = w.memory(); rep.ok = true; rep.seconds = secondsSince(t0); return rep; }
    {   // manifest.json last (the commit marker), then rename into place
        std::ofstream mf(tmpDir + "/" + kManifestFile); mf << manifest.dump(1) << "\n";
        if (!mf) { rep.error = "cannot write manifest"; std::error_code ec; fs::remove_all(tmpDir, ec); return rep; }
    }
    std::error_code ec;
    if (fs::exists(rep.dir)) fs::remove_all(rep.dir, ec);   // an older or partial bundle under this key
    fs::rename(tmpDir, rep.dir, ec);
    if (ec) {   // another writer finished first: keep theirs
        fs::remove_all(tmpDir, ec);
        if (!fs::exists(rep.dir + "/" + kManifestFile)) { rep.error = "cannot move bundle into place"; return rep; }
    }
    rep.ok = true; rep.seconds = secondsSince(t0); return rep;
}

Obtained obtainForLevel(const LevelInputs& in, const std::string& producer, const ProgressFn* progress) {
    Obtained o; const auto t0 = std::chrono::steady_clock::now(); std::string err;
    const bool nocache = envFlag("RT_NOCACHE"), require = envFlag("RT_BUNDLE_REQUIRE"), writeAllowed = envFlag("RT_BUNDLE_WRITE", true), sameEngine = envFlag("RT_BUNDLE_REQUIRE_ENGINE");
    const BundleProducer* p = findProducer(producer);
    if (!p) { o.status = "no producer named " + producer; setLastBundleStatus(o.status); return o; }
    const std::vector<Applicable> aps = applicableProducers(in, {});
    const uint64_t key = keyOf(aps); const std::string root = bundleRoot(); o.dir = bundleDirFor(root, key);
    uint64_t myKey = 0; for (const Applicable& a : aps) if (a.p == p) myKey = a.id.key;
    if (!nocache) {
        const nlohmann::json m = readManifest(o.dir); const nlohmann::json pe = manifestProducer(m, producer);
        if (!pe.is_null() && pe.value("key", std::string()) == hex16(myKey)) {
            const std::string ver = m.value("engine", nlohmann::json::object()).value("version", std::string());
            if (sameEngine && ver != engineIdentity().version) LOG_WARN << "[bundle] " << o.dir << " was built by engine " << ver << " (this is " << engineIdentity().version << "); RT_BUNDLE_REQUIRE_ENGINE set, treating as a miss";
            else {
                std::unique_ptr<Bundle> b = Bundle::open(o.dir + "/" + kBundleFile, &err);
                if (b) { o.bundle = std::move(b); o.hit = true; o.seconds = secondsSince(t0); o.status = "hit " + o.dir + " (open " + std::to_string(o.seconds) + " s)"; setLastBundleStatus(o.status); return o; }
                LOG_WARN << "[bundle] " << err << "; rebuilding";
            }
        }
        if (require) { o.status = "miss " + o.dir + " and RT_BUNDLE_REQUIRE is set"; setLastBundleStatus(o.status); return o; }
    }
    BakeRequest req; req.levelPath = in.levelPath; req.threads = in.threads; req.toMemory = nocache || !writeAllowed; req.force = nocache;
    const BakeReport rep = bakeLevel(req, progress);
    if (!rep.ok) { o.status = "bake failed: " + rep.error; setLastBundleStatus(o.status); return o; }
    std::unique_ptr<Bundle> b = rep.memory ? Bundle::fromMemory(rep.memory, &err) : Bundle::open(rep.dir + "/" + kBundleFile, &err);
    if (!b) { o.status = "bake wrote an unreadable bundle: " + err; setLastBundleStatus(o.status); return o; }
    o.bundle = std::move(b); o.built = true; o.seconds = secondsSince(t0);
    o.status = std::string(nocache ? "nocache built in memory " : (rep.memory ? "miss built in memory " : "miss built and wrote ")) + (rep.memory ? "" : rep.dir) + " (" + std::to_string(rep.seconds) + " s)";
    setLastBundleStatus(o.status); return o;
}

std::string bundleDirForLevel(const LevelInputs& in, const std::string& root) {
    const std::vector<Applicable> aps = applicableProducers(in, {});
    if (aps.empty()) return std::string();
    return bundleDirFor(root.empty() ? bundleRoot() : root, keyOf(aps));
}

PruneReport pruneBundles(const std::string& root, const std::vector<std::string>& keep, bool apply) {
    PruneReport rep; std::error_code ec;
    auto norm = [](const std::string& p) { std::error_code e; fs::path a = fs::absolute(p, e); return a.lexically_normal().string(); };
    std::vector<std::string> keepNorm; for (const std::string& k : keep) keepNorm.push_back(norm(k));
    for (const fs::directory_entry& e : fs::directory_iterator(root, ec)) {
        if (!e.is_directory()) continue;
        const std::string name = e.path().filename().string();
        if (name.find(".tmp-") != std::string::npos) continue;   // a bake in flight (or its leftovers): not ours to judge
        PruneEntry pe; pe.dir = e.path().string();
        for (const fs::directory_entry& f : fs::recursive_directory_iterator(pe.dir, ec)) if (f.is_regular_file(ec)) pe.bytes += f.file_size(ec);
        for (const fs::directory_entry& f : fs::directory_iterator(pe.dir, ec)) { const std::string fn = f.path().filename().string(); if (fn != kBundleFile && fn != kManifestFile) pe.extras.push_back(fn + (f.is_directory(ec) ? "/" : "")); }
        std::sort(pe.extras.begin(), pe.extras.end());
        const nlohmann::json m = readManifest(pe.dir);
        if (m.is_null() || !fs::exists(pe.dir + "/" + kBundleFile)) { pe.level = "(no manifest)"; pe.stale = true; rep.entries.push_back(pe); continue; }
        pe.level = m.value("level", nlohmann::json::object()).value("path", std::string()); if (!pe.level.empty()) pe.level = norm(pe.level);
        pe.created = m.value("created", std::string()); pe.key = m.value("key", std::string());
        rep.entries.push_back(pe);
    }
    std::map<std::string, size_t> newest;   // level path -> entry index (ISO-8601 "created" sorts as text)
    for (size_t i = 0; i < rep.entries.size(); ++i) { const PruneEntry& pe = rep.entries[i]; if (pe.stale) continue; auto it = newest.find(pe.level); if (it == newest.end() || pe.created > rep.entries[it->second].created) newest[pe.level] = i; }
    for (size_t i = 0; i < rep.entries.size(); ++i) {
        PruneEntry& pe = rep.entries[i]; if (pe.stale) continue;
        pe.current = std::find(keepNorm.begin(), keepNorm.end(), norm(pe.dir)) != keepNorm.end();
        pe.stale = !pe.current && newest[pe.level] != i;
    }
    std::sort(rep.entries.begin(), rep.entries.end(), [](const PruneEntry& a, const PruneEntry& b) { return a.level != b.level ? a.level < b.level : a.created > b.created; });
    for (const PruneEntry& pe : rep.entries) {
        if (!pe.stale) continue;
        rep.staleBytes += pe.bytes;
        if (apply) { fs::remove_all(pe.dir, ec); if (!ec) ++rep.removed; else LOG_WARN << "[bundle] prune: cannot remove " << pe.dir << ": " << ec.message(); }
    }
    return rep;
}

std::string lastBundleStatus() { std::lock_guard<std::mutex> l(statusMutex()); return statusLine(); }
void setLastBundleStatus(const std::string& s) { std::lock_guard<std::mutex> l(statusMutex()); statusLine() = s; }

}  // namespace bundle
}  // namespace engine
