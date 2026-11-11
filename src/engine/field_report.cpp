#include "field_report.h"
#include "screenshot.h"
#include "../renderer/settings.h"

#include <nlohmann/json.hpp>

#include <cstdio>
#include <ctime>
#include <filesystem>
#include <fstream>

namespace engine {

std::string fieldReportFolder(const Settings& settings) {
    const std::string s = settings.getString(kFieldReportFolderKey, "");
    if (!s.empty()) return s;
    return (std::filesystem::path(screenshotFolder(settings)) / "reports").string();
}

std::string nextFieldReportDir(const std::string& base, std::string& id) {
    std::error_code ec;
    std::filesystem::create_directories(base, ec);
    if (!std::filesystem::is_directory(base, ec)) return "";
    const std::time_t now = std::time(nullptr);
    std::tm local{};
#ifdef _WIN32
    localtime_s(&local, &now);
#else
    localtime_r(&now, &local);
#endif
    char stamp[32];
    std::strftime(stamp, sizeof(stamp), "%Y-%m-%d_%H-%M-%S", &local);
    for (int n = 1; n < 1000; ++n) {
        const std::string name = std::string(stamp) + (n > 1 ? "_" + std::to_string(n) : "");
        const std::filesystem::path dir = std::filesystem::path(base) / name;
        // create_directory says whether IT made the folder: two reports in one second never share one
        if (std::filesystem::create_directory(dir, ec)) {
            id = name;
            return dir.string();
        }
    }
    return "";
}

const std::vector<std::string>& fieldReportVerbsAtPress() {
    // Where, what at, what state the world is in. Each is an existing question (ADR-0253); a verb this state
    // has no answerer for replies err and is dropped.
    static const std::vector<std::string> verbs = {
        "info",      "where?",    "aim?",      "lot? {aim} 30", "ground? {aim}", "vehicle?", "possess?",
        "agent?",    "look?",     "daynight?", "weather?",      "fog?",          "bundle?",  "mem?",
    };
    return verbs;
}

const std::vector<std::string>& fieldReportVerbsAfter() {
    // the read-back of the questions staged at the press (answered on a later frame)
    static const std::vector<std::string> verbs = {"ground?"};
    return verbs;
}

std::string fieldReportVerbLine(const std::string& pattern, const FieldReport& r) {
    const std::size_t at = pattern.find("{aim}");
    if (at == std::string::npos) return pattern;
    if (!r.aimHit) return "";
    char xz[64];
    std::snprintf(xz, sizeof(xz), "%.2f %.2f", r.aim[0], r.aim[2]);
    return pattern.substr(0, at) + xz + pattern.substr(at + 5);
}

bool fieldReportKeep(const std::string& reply, std::string& kept) {
    if (reply.rfind("ok", 0) != 0) return false;   // err ..., or nothing
    kept = reply.size() > 3 ? reply.substr(3) : "";
    while (!kept.empty() && (kept.back() == '\n' || kept.back() == ' ')) kept.pop_back();
    return !kept.empty() && kept != "none" && kept != "pending";
}

std::string fieldReportTitle(const FieldReport& r) {
    if (!r.title.empty()) return r.title;
    std::string first = r.note.substr(0, r.note.find('\n'));
    while (!first.empty() && first.back() == ' ') first.pop_back();
    if (first.empty()) return "Field report " + r.id;
    if (first.size() > 80) {
        // cut on a character boundary: half a multi-byte character is not UTF-8, and the JSON dump refuses it
        std::size_t cut = 77;
        while (cut > 0 && (static_cast<unsigned char>(first[cut]) & 0xC0) == 0x80) --cut;
        first = first.substr(0, cut) + "...";
    }
    return first;
}

std::string fieldReportJson(const FieldReport& r) {
    nlohmann::ordered_json j;
    j["id"] = r.id;
    j["title"] = fieldReportTitle(r);
    j["note"] = r.note;
    j["time"] = r.wallTime;
    j["level"] = r.level;
    j["build"] = r.build;
    j["pose"] = r.pose;
    j["teleport"] = r.pose.empty() ? "" : "teleport " + r.pose;
    if (r.aimHit) j["aim"] = {r.aim[0], r.aim[1], r.aim[2]};
    else j["aim"] = nullptr;
    j["shot"] = std::filesystem::exists(r.shot) ? "shot.png" : "";
    j["file_issue"] = r.fileIssue;
    nlohmann::ordered_json facts = nlohmann::ordered_json::object();
    for (const FieldReportFact& f : r.facts) facts[f.verb] = f.reply;
    j["facts"] = facts;
    // replace, not throw: a reply with a stray byte must not take the game down on Save
    return j.dump(2, ' ', false, nlohmann::ordered_json::error_handler_t::replace) + "\n";
}

std::string fieldReportMarkdown(const FieldReport& r) {
    std::string md;
    md += r.note.empty() ? "(no note)\n" : r.note + (r.note.back() == '\n' ? "" : "\n");
    md += "\n---\n\n";
    md += "- **Level:** `" + r.level + "`\n";
    if (!r.pose.empty()) md += "- **Go there:** `teleport " + r.pose + "`\n";
    if (r.aimHit) {
        char buf[96];
        std::snprintf(buf, sizeof(buf), "- **Looking at:** (%.2f, %.2f, %.2f)\n", r.aim[0], r.aim[1], r.aim[2]);
        md += buf;
    }
    md += "- **Build:** `" + r.build + "`\n";
    md += "- **Time:** " + r.wallTime + "\n";
    md += "- **Report:** `" + r.id + "`\n";
    if (!r.facts.empty()) {
        md += "\n<details><summary>What the game said</summary>\n\n";
        for (const FieldReportFact& f : r.facts) md += "- `" + f.verb + "`: " + f.reply + "\n";
        md += "\n</details>\n";
    }
    return md;
}

bool writeFieldReport(const FieldReport& r, std::string& why) {
    const std::filesystem::path dir(r.dir);
    {
        std::ofstream out(dir / "report.json");
        out << fieldReportJson(r);
        if (!out) { why = "cannot write " + (dir / "report.json").string(); return false; }
    }
    {
        std::ofstream out(dir / "note.md");
        out << "# " << fieldReportTitle(r) << "\n\n" << fieldReportMarkdown(r);
        if (!out) { why = "cannot write " + (dir / "note.md").string(); return false; }
    }
    return true;
}

}  // namespace engine
