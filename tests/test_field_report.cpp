// Field reports (engine/field_report.h): the folder, the facts, the files a report writes.
#include "test_framework.h"

#include "../src/engine/field_report.h"
#include "../src/engine/application.h"

#include <nlohmann/json.hpp>

#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <string>

using namespace engine;

namespace {

std::string slurp(const std::filesystem::path& p) {
    std::ifstream in(p);
    std::stringstream ss;
    ss << in.rdbuf();
    return ss.str();
}

std::filesystem::path scratchDir(const char* name) {
    const std::filesystem::path d = std::filesystem::temp_directory_path() / ("rt_field_report_" + std::string(name));
    std::filesystem::remove_all(d);
    return d;
}

}  // namespace

// Every question a report asks is a verb the table lists: a renamed verb would otherwise drop out of every report
// without a word (its reply would be "err unknown command" and filtered as nothing-to-say).
TEST_CASE(field_report_asks_only_listed_verbs) {
    Application app;
    for (const std::string& line : fieldReportVerbsAtPress())
        CHECK(app.commands().has(line.substr(0, line.find(' '))));
    for (const std::string& line : fieldReportVerbsAfter())
        CHECK(app.commands().has(line.substr(0, line.find(' '))));
    CHECK(app.commands().has("aim?"));
}

TEST_CASE(field_report_verb_lines_take_the_aim_or_drop_out) {
    FieldReport r;
    CHECK(fieldReportVerbLine("where?", r) == "where?");
    CHECK(fieldReportVerbLine("lot? {aim} 30", r).empty());   // nothing under the crosshair
    r.aimHit = true;
    r.aim[0] = 12.5; r.aim[1] = 3; r.aim[2] = -40.25;
    CHECK(fieldReportVerbLine("lot? {aim} 30", r) == "lot? 12.50 -40.25 30");

    std::string kept;
    CHECK(fieldReportKeep("ok 1.00 2.00 3.00 -10.00 90.00", kept) && kept == "1.00 2.00 3.00 -10.00 90.00");
    CHECK(!fieldReportKeep("err vehicle?: nothing answers it here", kept));
    CHECK(!fieldReportKeep("ok none", kept));
    CHECK(!fieldReportKeep("ok", kept));
    CHECK(!fieldReportKeep("", kept));
}

TEST_CASE(field_report_title_falls_back_to_the_note) {
    FieldReport r;
    r.id = "2026-11-10_21-04-33";
    CHECK(fieldReportTitle(r) == "Field report 2026-11-10_21-04-33");
    r.note = "Car falls through the ramp\nright after the bend";
    CHECK(fieldReportTitle(r) == "Car falls through the ramp");
    r.note = std::string(120, 'x');
    CHECK(fieldReportTitle(r).size() == 80);
    r.title = "Typed title";
    CHECK(fieldReportTitle(r) == "Typed title");
}

// Two presses in one second get two folders; the files hold the note, the teleport line and every fact.
TEST_CASE(field_report_writes_its_folder) {
    const std::filesystem::path base = scratchDir("write");
    std::string id1, id2;
    const std::string d1 = nextFieldReportDir(base.string(), id1);
    const std::string d2 = nextFieldReportDir(base.string(), id2);
    CHECK(!d1.empty() && !d2.empty() && d1 != d2 && id1 != id2);
    CHECK(std::filesystem::is_directory(d1) && std::filesystem::is_directory(d2));

    FieldReport r;
    r.id = id1;
    r.dir = d1;
    r.shot = (std::filesystem::path(d1) / "shot.png").string();
    r.note = "The sofa blocks the door";
    r.level = "assets/levels/metro_v2_test.json";
    r.build = "abc123 Release";
    r.pose = "10.00 2.00 -5.00 -12.00 90.00";
    r.aimHit = true;
    r.aim[0] = 11; r.aim[1] = 1.5; r.aim[2] = -9;
    r.facts.push_back({"aim?", "at 11.00 1.50 -9.00 dist 4.2 surface concrete"});
    r.facts.push_back({"lot? 11.00 -9.00 30", "at (11, -9) | 1 record(s)"});
    std::string why;
    CHECK(writeFieldReport(r, why));

    const nlohmann::json j = nlohmann::json::parse(slurp(std::filesystem::path(d1) / "report.json"), nullptr, false);
    CHECK(!j.is_discarded());
    CHECK(j["title"] == "The sofa blocks the door");
    CHECK(j["teleport"] == "teleport 10.00 2.00 -5.00 -12.00 90.00");
    CHECK(j["aim"].is_array() && j["aim"].size() == 3);
    CHECK(j["shot"] == "");   // no frame was dumped in a test
    CHECK(j["facts"]["lot? 11.00 -9.00 30"] == "at (11, -9) | 1 record(s)");

    const std::string md = slurp(std::filesystem::path(d1) / "note.md");
    CHECK(md.rfind("# The sofa blocks the door\n", 0) == 0);
    CHECK(md.find("`teleport 10.00 2.00 -5.00 -12.00 90.00`") != std::string::npos);
    CHECK(md.find("`aim?`: at 11.00") != std::string::npos);
    std::filesystem::remove_all(base);
}
