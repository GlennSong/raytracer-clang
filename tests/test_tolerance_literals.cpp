#include "test_framework.h"

// NO BARE TOLERANCES IN THE ROAD AND DRIVING CODE (AGENTS.md "Coding Standards"; rt_math.h "Comparing computed numbers").
//
// A small number written straight into the code (`if (d < 1e-9)`, `s <= to + 1e-6`) is a tolerance nobody named: the
// next file picks another value for the same idea, and two of them then disagree about whether two points are one.
// Tolerances are named once (rt_math.h kSamePoint, kWrittenStation; or a named `constexpr` beside its reason) and
// numbers are compared through approxEqual and its ordered companions.
//
// THE RULE THIS TEST HOLDS, on the text of every .h and .cpp under src/apps/driving and
// src/engine/procgen/city/roads: a BARE TOLERANCE LITERAL is a number written with a negative exponent (1e-6, 2.5e-3)
// outside a comment, outside a string, and not on a line that says `constexpr` (a definition with a name).
//   - A file NOT on the list below that holds one FAILS: name the tolerance, or use a shared one.
//   - A file ON the list that no longer holds any FAILS too, so that it is taken off and the list only shrinks.
//   - A listed file may hold any number of them: the list is by file, and the count beside each name is a comment (what
//     it held on 2026-11-14), so work going on inside a listed file never makes this test flap.
//
// WHAT IT DOES NOT CATCH (say so, rather than pretend): a raw `<=` or `==` between two computed distances with no
// literal at all (the fault that started this: announce.cpp `heldStop <= startAt`); a tolerance written without an
// exponent (0.001, 0.01); a tolerance named locally with a value that disagrees with the shared ones; a literal on a
// line that also happens to say constexpr; a literal split from its exponent by a macro. It also counts small numbers
// that are not tolerances (a guard against dividing by zero, a default argument): those want a name as well.
// (A sharper check would need the compiler's view of the code -- a clang-tidy rule flagging relational operators whose
// two sides are both floating-point expressions, with an allow-comment for the stated-fact cases. Proposed, not built.)

#include <algorithm>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <map>
#include <sstream>
#include <string>
#include <vector>

namespace {

// THE FILES THAT HOLD BARE TOLERANCE LITERALS TODAY (2026-11-14), paths under src/, each with how many it held then
// (a comment, not checked). 22 files, 102 literals, all in the road code; the driving code holds none. Take a file off
// when its last one is named.
const std::vector<std::string> kHoldsBareTolerances = {
    "engine/procgen/city/roads/lanes/block_audit.cpp",          // 9
    "engine/procgen/city/roads/lanes/city_producer.cpp",        // 1
    "engine/procgen/city/roads/lanes/control_plan.cpp",         // 1
    "engine/procgen/city/roads/lanes/deck_height.cpp",          // 2
    "engine/procgen/city/roads/lanes/deck_mesh.cpp",            // 30
    "engine/procgen/city/roads/lanes/geom2d.cpp",               // 4
    "engine/procgen/city/roads/lanes/geom2d.h",                 // 1
    "engine/procgen/city/roads/lanes/interchange.cpp",          // 1
    "engine/procgen/city/roads/lanes/lane_expand.cpp",          // 1
    "engine/procgen/city/roads/lanes/pavement.cpp",             // 6
    "engine/procgen/city/roads/lanes/polyline_ops.cpp",         // 3
    "engine/procgen/city/roads/lanes/polyline_ops.h",           // 2
    "engine/procgen/city/roads/lanes/road_design.cpp",          // 3
    "engine/procgen/city/roads/lanes/road_graph_spec.cpp",      // 1
    "engine/procgen/city/roads/lanes/road_layout.cpp",          // 4
    "engine/procgen/city/roads/lanes/road_model_compare.cpp",   // 7
    "engine/procgen/city/roads/lanes/road_twin.cpp",            // 6
    "engine/procgen/city/roads/lanes/terrain_recipe.cpp",       // 2
    "engine/procgen/city/roads/lanes/vertical_profile.cpp",     // 7
    "engine/procgen/city/roads/road_entity.cpp",                // 2
    "engine/procgen/city/roads/road_model.cpp",                 // 8
    "engine/procgen/city/roads/street_map.cpp",                 // 1
};

// The folders scanned, under src/.
const char* const kScanned[] = {"apps/driving", "engine/procgen/city/roads"};

bool isDigit(char c) { return c >= '0' && c <= '9'; }
bool isWordChar(char c) { return isDigit(c) || (c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z') || c == '_'; }

// One line of code with its comments and the insides of its strings taken out. `inBlock` carries a /* ... */ comment
// from line to line.
std::string codeOf(const std::string& line, bool& inBlock) {
    std::string out;
    std::size_t i = 0;
    while (i < line.size()) {
        if (inBlock) {
            const std::size_t end = line.find("*/", i);
            if (end == std::string::npos) break;
            inBlock = false;
            i = end + 2;
        } else if (line.compare(i, 2, "//") == 0) {
            break;
        } else if (line.compare(i, 2, "/*") == 0) {
            inBlock = true;
            i += 2;
        } else if (line[i] == '"') {
            std::size_t j = i + 1;
            while (j < line.size() && line[j] != '"') j += line[j] == '\\' ? 2 : 1;
            i = j + 1;
            out += "\"\"";
        } else {
            out += line[i++];
        }
    }
    return out;
}

// How many numbers with a negative exponent `code` holds: digits, an optional point and more digits, e or E, a minus
// sign, digits; not part of a longer word on either side.
int tolerancesIn(const std::string& code) {
    int found = 0;
    for (std::size_t e = 1; e + 2 < code.size(); ++e) {
        if ((code[e] != 'e' && code[e] != 'E') || code[e + 1] != '-' || !isDigit(code[e + 2]) || !isDigit(code[e - 1])) continue;
        std::size_t start = e;   // walk back over the mantissa
        while (start > 0 && (isDigit(code[start - 1]) || code[start - 1] == '.')) --start;
        if (start > 0 && isWordChar(code[start - 1])) continue;   // the tail of a name (x1e-3 is not a number)
        std::size_t end = e + 2;
        while (end < code.size() && isDigit(code[end])) ++end;
        if (end < code.size() && isWordChar(code[end]) && code[end] != 'f' && code[end] != 'F') continue;
        ++found;
    }
    return found;
}

// The bare tolerance literals in a file's text: every line's code that does not say constexpr.
int bareTolerancesIn(std::istream& text) {
    int found = 0;
    bool inBlock = false;
    std::string line;
    while (std::getline(text, line)) {
        const std::string code = codeOf(line, inBlock);
        if (code.find("constexpr") != std::string::npos) continue;
        found += tolerancesIn(code);
    }
    return found;
}

int bareTolerancesInText(const std::string& text) {
    std::istringstream in(text);
    return bareTolerancesIn(in);
}

}  // namespace

TEST_CASE(tolerance_literals_the_scanner_counts_what_it_says) {
    // The rule on hand-written lines, so a change to the scanner cannot quietly stop it seeing anything.
    CHECK(bareTolerancesInText("if (d.length() < 1e-9) continue;\n") == 1);
    CHECK(bareTolerancesInText("if (s > from + 1e-6 && s < to - 2.5E-3) x();\n") == 2);
    CHECK(bareTolerancesInText("const float t = 1e-4f;\n") == 1);
    CHECK(bareTolerancesInText("constexpr double kCutTolerance = 1e-3;\n") == 0);              // named
    CHECK(bareTolerancesInText("// snaps at 1e-6 m, then 1e-4\n") == 0);                       // a comment
    CHECK(bareTolerancesInText("x = 1;   // within 1e-6\n") == 0);
    CHECK(bareTolerancesInText("/* 1e-6\n   1e-9 */ y = 2;\n") == 0);                          // a block comment
    CHECK(bareTolerancesInText("printf(\"%f 1e-6\\n\", v);\n") == 0);                         // inside a string
    CHECK(bareTolerancesInText("double far = 1e300, big = 2e+5, n = 1e6;\n") == 0);            // not small numbers
    CHECK(bareTolerancesInText("double v = scale1e-3;\n") == 0);                               // a name minus a number
    CHECK(bareTolerancesInText("if (a <= b) go();\n") == 0);   // NOT caught: a raw comparison of two computed numbers
    CHECK(bareTolerancesInText("if (d < 0.001) go();\n") == 0);   // NOT caught: a tolerance written without an exponent
}

TEST_CASE(tolerance_literals_no_new_bare_tolerance_in_roads_or_driving) {
    namespace fs = std::filesystem;
    const fs::path src = fs::path(RT_SOURCE_DIR) / "src";
    std::map<std::string, int> found;   // every scanned file holding at least one, by its path under src/
    std::size_t files = 0;
    for (const char* folder : kScanned) {
        CHECK(fs::is_directory(src / folder));
        if (!fs::is_directory(src / folder)) continue;
        for (const fs::directory_entry& e : fs::recursive_directory_iterator(src / folder)) {
            const std::string ext = e.path().extension().string();
            if (!e.is_regular_file() || (ext != ".h" && ext != ".cpp")) continue;
            ++files;
            std::ifstream in(e.path());
            const int n = bareTolerancesIn(in);
            if (n > 0) found[fs::relative(e.path(), src).generic_string()] = n;
        }
    }
    CHECK(files > 100u);   // both folders were really read (about 190 files on 2026-11-14)
    int total = 0, fresh = 0, cleared = 0;
    for (const std::pair<const std::string, int>& f : found) {
        total += f.second;
        if (std::find(kHoldsBareTolerances.begin(), kHoldsBareTolerances.end(), f.first) != kHoldsBareTolerances.end()) continue;
        ++fresh;
        std::printf("    NEW: src/%s holds %d bare tolerance literal(s) (a number like 1e-6 outside a named constexpr). Use "
                    "rt_math.h kSamePoint / kWrittenStation with approxEqual, atOrBefore, strictlyPast ..., or name the "
                    "tolerance once beside its reason.\n",
                    f.first.c_str(), f.second);
    }
    for (const std::string& listed : kHoldsBareTolerances) {
        if (found.count(listed)) continue;
        ++cleared;
        std::printf("    CAN COME OFF THE LIST: src/%s holds no bare tolerance literal any more (or is gone); remove it from "
                    "kHoldsBareTolerances in tests/test_tolerance_literals.cpp.\n",
                    listed.c_str());
    }
    std::printf("    bare tolerance literals: %d in %zu files of %zu scanned (the list allows %zu files)\n", total, found.size(),
                files, kHoldsBareTolerances.size());
    CHECK(fresh == 0);
    CHECK(cleared == 0);
}
