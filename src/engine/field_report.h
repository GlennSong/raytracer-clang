#ifndef RAYTRACER_ENGINE_FIELD_REPORT_H
#define RAYTRACER_ENGINE_FIELD_REPORT_H

// FIELD REPORTS (Glenn, 2026-11-10: "take a screenshot and write some text and send it to you ... with a bunch
// of metadata attached like where I am in the game world and what I'm looking at ... or maybe so I can send it to
// GitHub as an issue to track"). F9 (the "report" action, bind.report) in a dev build (RT_DEV_TOOLS):
//   1. the frame is dumped to <folder>/<id>/shot.png, the way F12's screenshot is;
//   2. the world freezes under a note box (FieldReportState);
//   3. Save writes report.json (the facts) and note.md (the note, the teleport line, the facts as text) beside
//      the shot; "File a GitHub issue" also runs tools/file_report.py on the folder.
// The facts are the control channel's own answers (ADR-0253), asked in-process: `where?`, `aim?` (what the
// crosshair is on), `lot?` at that spot, `vehicle?`, `daynight?` ... -- one source for what the game knows about
// itself, the same lines the MCP reads. A verb nothing answers in this state is left out.
//
// The folder is the `report.folder` setting, else the screenshot folder's reports/ (~/Pictures/Raytracer/reports).

#include <string>
#include <vector>

namespace engine {

class Settings;

constexpr const char* kFieldReportFolderKey = "report.folder";

struct FieldReportFact {
    std::string verb;    // the line asked: "lot? 12.0 -40.5 30"
    std::string reply;   // its answer, without the "ok " prefix
};

struct FieldReport {
    std::string id;          // the folder's name: 2026-11-10_21-04-33
    std::string dir;         // the folder (created)
    std::string shot;        // dir/shot.png (may be missing if the renderer cannot capture)
    std::string title;       // one line; the issue's title
    std::string note;        // what Glenn wrote
    bool fileIssue = false;
    std::string level;       // the level file
    std::string build;       // RT_BUILD_GIT_DESCRIBE + build type
    std::string pose;        // `x y z pitch yaw`, what `teleport` takes
    bool aimHit = false;     // the crosshair hit something within range
    double aim[3] = {0, 0, 0};
    std::string wallTime;    // local time of the press, ISO 8601
    std::vector<FieldReportFact> facts;
};

// The base folder: the setting, else <screenshot folder>/reports.
std::string fieldReportFolder(const Settings& settings);
// A fresh folder under `base`, named for the local time (with _2, _3 ... when taken). Empty if it cannot be made.
// `id` gets the folder's name.
std::string nextFieldReportDir(const std::string& base, std::string& id);

// The verbs a report asks, in order, before and after the frame is captured. `{aim}` in a line is replaced by the
// crosshair's "x z" (a line needing it is skipped when the crosshair hit nothing). The `ask` lines stage a question
// a system answers on a later frame (ground? x z); their read-back is in `after`.
const std::vector<std::string>& fieldReportVerbsAtPress();
const std::vector<std::string>& fieldReportVerbsAfter();

// A verb line with {aim} filled; empty when it needs the aim and there is none.
std::string fieldReportVerbLine(const std::string& pattern, const FieldReport& r);
// Keep a reply as a fact? ("ok ..." kept without the prefix; err / none / empty dropped.)
bool fieldReportKeep(const std::string& reply, std::string& kept);

// The title from the note when none was typed: its first line, cut to 80 characters; "Field report <id>" when
// the note is empty too.
std::string fieldReportTitle(const FieldReport& r);

std::string fieldReportJson(const FieldReport& r);
// note.md, also the issue's body: the note, the teleport line, then every fact as a list.
std::string fieldReportMarkdown(const FieldReport& r);
// Writes report.json and note.md into r.dir. False (with why) when a file cannot be written.
bool writeFieldReport(const FieldReport& r, std::string& why);

}  // namespace engine

#endif
