#include "recent_scenes.h"

#include "../renderer/settings.h"

#include <algorithm>

namespace engine {

namespace {
constexpr char kRecentKey[] = "recentScenes";

std::vector<std::string> split(const std::string& joined) {
    std::vector<std::string> out;
    std::string cur;
    for (char c : joined) {
        if (c == '\n') {
            if (!cur.empty()) out.push_back(cur);
            cur.clear();
        } else {
            cur.push_back(c);
        }
    }
    if (!cur.empty()) out.push_back(cur);
    return out;
}

std::string join(const std::vector<std::string>& items) {
    std::string out;
    for (size_t i = 0; i < items.size(); i++) {
        if (i) out.push_back('\n');
        out += items[i];
    }
    return out;
}
}  // namespace

std::vector<std::string> loadRecentScenes(const Settings& settings) {
    return split(settings.getString(kRecentKey, ""));
}

void recordRecentScene(Settings& settings, const std::string& path) {
    if (path.empty()) return;

    std::vector<std::string> list = loadRecentScenes(settings);
    // Drop any earlier occurrence so the path appears exactly once, then put
    // it at the front as the newest.
    list.erase(std::remove(list.begin(), list.end(), path), list.end());
    list.insert(list.begin(), path);
    if (list.size() > static_cast<size_t>(kMaxRecentScenes))
        list.resize(kMaxRecentScenes);

    settings.setString(kRecentKey, join(list));
}

void clearRecentScenes(Settings& settings) {
    settings.setString(kRecentKey, "");
}

}  // namespace engine
