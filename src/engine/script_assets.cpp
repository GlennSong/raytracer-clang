#include "script_assets.h"

#include "../log.h"

#include <fstream>
#include <sstream>

namespace engine {

namespace {

// The union of every candidate list this engine used to carry. Order matters:
// an explicit path wins, then the asset root, then the run-from-build/ variant,
// then level-relative.
std::vector<std::string> candidates(const std::string& file,
                                    const std::string& levelDir) {
    std::vector<std::string> out{file,
                                 "assets/scripts/" + file,
                                 "../assets/scripts/" + file};
    if (!levelDir.empty()) out.push_back(levelDir + "/" + file);
    return out;
}

}  // namespace

std::string resolveScriptPath(const std::string& file, const std::string& levelDir) {
    if (file.empty()) return {};
    for (const std::string& path : candidates(file, levelDir)) {
        std::ifstream f(path);
        if (f) return path;
    }
    return {};
}

std::string loadScriptCode(const std::string& file, const std::string& levelDir) {
    const std::string path = resolveScriptPath(file, levelDir);
    if (path.empty()) return {};
    std::ifstream f(path);
    if (!f) return {};
    std::stringstream ss;
    ss << f.rdbuf();
    return ss.str();
}

#ifdef RT_ENABLE_SCRIPTING
ScriptModuleSource makeModuleSource(const std::string& levelDir,
                                    std::vector<std::string>* touched) {
    return [levelDir, touched](const std::string& name, std::string& code,
                               std::string& resolvedPath) {
        // `name` arrives already validated as an identifier by the module loader
        // (script_modules.cpp), so appending ".lua" cannot escape the roots.
        resolvedPath = resolveScriptPath(name + ".lua", levelDir);
        if (resolvedPath.empty()) return false;
        code = loadScriptCode(name + ".lua", levelDir);
        if (code.empty()) return false;
        // Which candidate won depends on the working directory, so say it once —
        // a module resolving differently under the test binary than under the app
        // is otherwise invisible.
        LOG_INFO << "require '" << name << "' -> " << resolvedPath;
        if (touched != nullptr) touched->push_back(resolvedPath);
        return true;
    };
}
#endif

}  // namespace engine
