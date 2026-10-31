#include "engine/bundle/cache_use.h"

#include <nlohmann/json.hpp>

#include <algorithm>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <mutex>

namespace engine {
namespace bundle {

namespace fs = std::filesystem;

namespace {
std::mutex g_mu;
std::string g_level;   // absolute; empty = no load under way
int64_t g_stamp = 0;

std::string recordOf(const std::string& file) { return file + ".uses"; }
}  // namespace

void setCacheLevel(const std::string& levelPath) {
    std::lock_guard<std::mutex> lock(g_mu);
    if (levelPath.empty()) { g_level.clear(); return; }
    std::error_code ec;
    g_level = fs::absolute(levelPath, ec).lexically_normal().string();
    g_stamp = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now().time_since_epoch()).count();
}

bool readCacheUse(const std::string& file, CacheUse& out) {
    std::ifstream in(recordOf(file));
    if (!in) return false;
    try {
        const nlohmann::json j = nlohmann::json::parse(in);
        out.kind = j.value("kind", std::string());
        out.levels.clear();
        if (j.contains("levels") && j["levels"].is_object())
            for (auto it = j["levels"].begin(); it != j["levels"].end(); ++it) out.levels[it.key()] = it.value().get<int64_t>();
        return true;
    } catch (const std::exception&) {
        return false;
    }
}

void noteCacheUse(const std::string& file, const std::string& kind) {
    std::lock_guard<std::mutex> lock(g_mu);
    if (g_level.empty()) return;
    CacheUse use;
    readCacheUse(file, use);
    use.kind = kind;
    use.levels[g_level] = g_stamp;
    nlohmann::json j;
    j["kind"] = use.kind;
    j["levels"] = nlohmann::json::object();
    for (const auto& [level, stamp] : use.levels) j["levels"][level] = stamp;
    // Written aside and renamed, so a viewer reading it meanwhile never sees half a record.
    const std::string tmp = recordOf(file) + ".tmp";
    {
        std::ofstream out(tmp);
        if (!out) return;
        out << j.dump(1) << "\n";
        if (!out) return;
    }
    std::error_code ec;
    fs::rename(tmp, recordOf(file), ec);
}

CachePruneReport pruneCacheFiles(const std::vector<std::string>& dirs, bool apply) {
    CachePruneReport rep;
    std::vector<CacheUse> uses;
    std::vector<std::string> orphanRecords;
    for (const std::string& dir : dirs) {
        std::error_code ec;
        for (const fs::directory_entry& e : fs::directory_iterator(dir, ec)) {
            if (!e.is_regular_file(ec)) continue;
            const std::string path = e.path().string();
            const std::string name = e.path().filename().string();
            if (name.find(".tmp") != std::string::npos) continue;   // a write in flight
            if (name.size() > 5 && name.compare(name.size() - 5, 5, ".uses") == 0) {
                if (!fs::exists(path.substr(0, path.size() - 5), ec)) orphanRecords.push_back(path);
                continue;
            }
            CacheFileEntry f;
            f.path = path;
            f.bytes = e.file_size(ec);
            CacheUse u;
            f.owned = readCacheUse(path, u);
            f.kind = u.kind;
            for (const auto& lv : u.levels) f.levels.push_back(lv.first);
            rep.files.push_back(f);
            uses.push_back(u);
        }
    }
    // The newest load of each (level, kind) that used any file.
    std::map<std::pair<std::string, std::string>, int64_t> newest;
    for (std::size_t i = 0; i < rep.files.size(); ++i)
        for (const auto& [level, stamp] : uses[i].levels) {
            int64_t& n = newest[{level, uses[i].kind}];
            n = std::max(n, stamp);
        }
    std::map<std::string, bool> levelExists;   // a level file since deleted (a test's) keeps nothing
    for (std::size_t i = 0; i < rep.files.size(); ++i) {
        CacheFileEntry& f = rep.files[i];
        if (!f.owned) { rep.unownedBytes += f.bytes; continue; }
        f.stale = true;
        for (const auto& [level, stamp] : uses[i].levels) {
            auto it = levelExists.find(level);
            if (it == levelExists.end()) { std::error_code ec; it = levelExists.emplace(level, fs::exists(level, ec)).first; }
            if (it->second && stamp >= newest[{level, uses[i].kind}]) f.stale = false;   // still this level's current one
        }
        if (!f.stale) continue;
        rep.staleBytes += f.bytes;
        if (apply) {
            std::error_code ec;
            if (fs::remove(f.path, ec)) ++rep.removed;
            fs::remove(recordOf(f.path), ec);
        }
    }
    if (apply)
        for (const std::string& r : orphanRecords) { std::error_code ec; fs::remove(r, ec); }
    std::sort(rep.files.begin(), rep.files.end(), [](const CacheFileEntry& a, const CacheFileEntry& b) { return a.path < b.path; });
    return rep;
}

}  // namespace bundle
}  // namespace engine
