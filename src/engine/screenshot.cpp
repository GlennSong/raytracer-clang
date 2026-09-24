#include "screenshot.h"
#include "../renderer/settings.h"

#include <cstdlib>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <string>

namespace engine {

namespace {

std::string homeDir() {
#ifdef _WIN32
    if (const char* p = std::getenv("USERPROFILE")) return p;
#endif
    if (const char* h = std::getenv("HOME")) return h;
    return ".";
}

// XDG_PICTURES_DIR from ~/.config/user-dirs.dirs, e.g. XDG_PICTURES_DIR="$HOME/Pictures".
std::string xdgPictures(const std::string& home) {
    if (const char* env = std::getenv("XDG_PICTURES_DIR"); env && *env) return env;
    const char* cfg = std::getenv("XDG_CONFIG_HOME");
    std::ifstream in((cfg && *cfg ? std::string(cfg) : home + "/.config") + "/user-dirs.dirs");
    std::string line;
    while (std::getline(in, line)) {
        const std::string key = "XDG_PICTURES_DIR=";
        if (line.rfind(key, 0) != 0) continue;
        std::string v = line.substr(key.size());
        if (v.size() >= 2 && v.front() == '"' && v.back() == '"') v = v.substr(1, v.size() - 2);
        if (v.rfind("$HOME", 0) == 0) v = home + v.substr(5);
        return v;
    }
    return "";
}

}  // namespace

std::string defaultScreenshotFolder() {
    const std::string home = homeDir();
    std::string pictures;
#if defined(__linux__)
    pictures = xdgPictures(home);
#endif
    if (pictures.empty()) pictures = home + "/Pictures";
    return (std::filesystem::path(pictures) / "Raytracer").string();
}

std::string screenshotFolder(const Settings& settings) {
    const std::string s = settings.getString(kScreenshotFolderKey, "");
    return s.empty() ? defaultScreenshotFolder() : s;
}

std::string nextScreenshotPath(const std::string& folder) {
    std::error_code ec;
    std::filesystem::create_directories(folder, ec);
    if (!std::filesystem::is_directory(folder, ec)) return "";
    const std::time_t now = std::time(nullptr);
    std::tm local{};
#ifdef _WIN32
    localtime_s(&local, &now);
#else
    localtime_r(&now, &local);
#endif
    char stamp[32];
    std::strftime(stamp, sizeof(stamp), "rt_%Y-%m-%d_%H-%M-%S", &local);
    // The file is written a frame later, so two presses in one second must not
    // both see the name free: count within the second as well as on disk.
    static std::string lastStamp;
    static int inSecond = 0;
    inSecond = lastStamp == stamp ? inSecond + 1 : 1;
    lastStamp = stamp;
    const std::filesystem::path dir(folder);
    auto named = [&](int n) { return dir / (std::string(stamp) + (n > 1 ? "_" + std::to_string(n) : "") + ".png"); };
    int n = inSecond;
    while (std::filesystem::exists(named(n), ec) && n < 1000) ++n;
    inSecond = n;
    return named(n).string();
}

}  // namespace engine
