#ifndef RAYTRACER_TESTS_INCLUDE_WALK_H
#define RAYTRACER_TESTS_INCLUDE_WALK_H

// A LIBRARY'S BOUNDARY, CHECKED BY FOLLOWING ITS INCLUDES (first written for the driving library, issue #284; shared
// with the agent brain's, ADR-0440). A library -- every file under its own folder -- may include, outside itself, only
// the headers on an ALLOW-LIST, and those are FOLLOWED: everything they include in turn must be on the list too, so
// nothing comes in through a header the library does include. Angle includes must be the standard library's (no folder,
// no ".h") or the JSON reader the bundle code uses.

#include <algorithm>
#include <filesystem>
#include <fstream>
#include <string>
#include <vector>

struct IncludeWalk {
    std::filesystem::path src;
    std::string library;                     // the library's own folder, as a path under src/ ending in '/'
    // Other libraries this one is built on, as folders under src/ ending in '/': their files are followed like the
    // library's own (each has its own boundary test).
    std::vector<std::string> builtOn;
    std::vector<std::string> allowed;        // headers outside the library, as paths under src/
    std::vector<std::string> external;       // headers from outside src/ (a third-party include path), by name
    std::vector<std::string> seen;           // every file reached, as paths under src/
    std::vector<std::string> breaches;

    std::string underSrc(const std::filesystem::path& p) const {
        return std::filesystem::relative(std::filesystem::weakly_canonical(p), src).generic_string();
    }
    void walk(const std::filesystem::path& file) {
        const std::string me = underSrc(file);
        if (std::find(seen.begin(), seen.end(), me) != seen.end()) return;
        seen.push_back(me);
        std::ifstream in(file);
        std::string line;
        while (std::getline(in, line)) {
            const std::size_t inc = line.find("#include");
            if (inc == std::string::npos || line.find_first_not_of(" \t") != inc) continue;
            const std::size_t q = line.find('"', inc), a = line.find('<', inc);
            if (q != std::string::npos) {
                const std::string name = line.substr(q + 1, line.find('"', q + 1) - q - 1);
                std::filesystem::path found = file.parent_path() / name;
                if (!std::filesystem::exists(found)) found = src / name;
                if (!std::filesystem::exists(found)) {
                    if (std::find(external.begin(), external.end(), name) == external.end())
                        breaches.push_back(me + " includes \"" + name + "\", which is not under src/ nor allowed");
                    continue;
                }
                const std::string there = underSrc(found);
                bool inLibrary = there.rfind(library, 0) == 0;
                for (const std::string& folder : builtOn) inLibrary = inLibrary || there.rfind(folder, 0) == 0;
                if (!inLibrary && std::find(allowed.begin(), allowed.end(), there) == allowed.end())
                    breaches.push_back(me + " includes " + there + ", which is not on the allow-list");
                else
                    walk(found);
            } else if (a != std::string::npos) {
                const std::string name = line.substr(a + 1, line.find('>', a + 1) - a - 1);
                const bool standard = name.find('/') == std::string::npos && name.find(".h") == std::string::npos;
                if (!standard && name != "nlohmann/json.hpp")
                    breaches.push_back(me + " includes <" + name + ">, which is not the standard library's");
            }
        }
    }
};

// Every breach found from the files directly in `dir` (not its sub-folders), and how many files were reached.
inline std::vector<std::string> includeBreachesOf(const std::filesystem::path& src, const std::string& library,
                                                  const std::filesystem::path& dir,
                                                  const std::vector<std::string>& allowed,
                                                  const std::vector<std::string>& external, std::size_t* files) {
    IncludeWalk w;
    w.src = std::filesystem::weakly_canonical(src);
    w.library = library;
    w.allowed = allowed;
    w.external = external;
    std::vector<std::filesystem::path> roots;
    for (const std::filesystem::directory_entry& e : std::filesystem::directory_iterator(dir))
        if (e.is_regular_file()) roots.push_back(e.path());
    std::sort(roots.begin(), roots.end());
    for (const std::filesystem::path& f : roots) w.walk(f);
    if (files) *files = w.seen.size();
    return w.breaches;
}

#endif
