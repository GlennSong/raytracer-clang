// Field reports (field_report.h): F9 -> the shot, the facts, a note box over the frozen world -> a folder, and an
// issue when asked. Here because only the Application holds all of it: the window, the renderer's frame dump,
// the command table the facts are asked of, and the state stack the box is pushed on.
#include "application.h"
#include "engine/build_info.h"
#include "states/field_report_state.h"
#include "../log.h"

#include <algorithm>
#include <cstdio>
#include <cstdlib>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <memory>
#include <sstream>

#ifdef RT_ENABLE_IMGUI
#include <imgui.h>
#endif

namespace engine {

namespace {

std::string isoNow() {
    const std::time_t now = std::time(nullptr);
    std::tm local{};
#ifdef _WIN32
    localtime_s(&local, &now);
#else
    localtime_r(&now, &local);
#endif
    char buf[40];
    std::strftime(buf, sizeof(buf), "%Y-%m-%dT%H:%M:%S%z", &local);
    return buf;
}

std::string firstWord(const std::string& line) { return line.substr(0, line.find(' ')); }

#ifndef _WIN32
// one word for sh: in single quotes, a quote inside closed, escaped and reopened
std::string shellQuote(const std::string& s) {
    std::string q = "'";
    for (const char c : s) q += c == '\'' ? std::string("'\\''") : std::string(1, c);
    return q + "'";
}
#endif

// the tool that files a report folder as an issue (tools/file_report.py), from the working directory the viewer
// was started in (the repo root, as for assets/), else the source tree it was built from
std::string fileReportTool() {
    std::error_code ec;
    if (std::filesystem::exists("tools/file_report.py", ec)) return std::filesystem::absolute("tools/file_report.py", ec).string();
#ifdef RT_SOURCE_DIR
    const std::string src = std::string(RT_SOURCE_DIR) + "/tools/file_report.py";
    if (std::filesystem::exists(src, ec)) return src;
#endif
    return "";
}

}  // namespace

void Application::stepFieldReport() {
#ifdef RT_DEV_TOOLS
    if (reportToastLeft_ > 0) reportToastLeft_ -= frameDelta;
    switch (reportPhase_) {
        case ReportPhase::Idle:
            // a focused text field owns the keys: F9 typed into the teleport box is not a report
            if (inputMap.pressed("report") && !window->getInput().uiWantsKeyboard) startFieldReport();
            break;
        case ReportPhase::Capturing: {
            // The dump is written a frame or two after it is asked for; open the box once the file has stopped
            // growing (the box must not be IN the shot). A renderer that cannot capture never writes it: give up
            // waiting after a few seconds.
            ++reportFrames_;
            std::error_code ec;
            const long long size = std::filesystem::exists(report_.shot, ec)
                                       ? static_cast<long long>(std::filesystem::file_size(report_.shot, ec))
                                       : -1;
            const bool landed = size > 0 && size == reportShotSize_;
            reportShotSize_ = size;
            if (!landed && reportFrames_ <= 240) break;
            openFieldReport();
            if (reportHeadless_) {   // the `report` verb: no box to wait for
                reportSave_ = true;
                closeFieldReport();
            }
            break;
        }
        case ReportPhase::Open:
            if (reportClosing_) closeFieldReport();
            break;
    }
#endif
}

void Application::startFieldReport(const std::string& headlessNote, bool headless) {
    report_ = FieldReport{};
    report_.note = headlessNote;
    reportHeadless_ = headless;
    report_.dir = nextFieldReportDir(fieldReportFolder(settingsStore), report_.id);
    if (report_.dir.empty()) {
        reportToast_ = "Report: cannot make a folder in " + fieldReportFolder(settingsStore) + " (set " +
                       kFieldReportFolderKey + ")";
        reportToastLeft_ = 5.0;
        LOG_WARN << reportToast_;
        return;
    }
    report_.shot = (std::filesystem::path(report_.dir) / "shot.png").string();
    report_.level = settingsStore.getString("levelPath", "?");
    report_.build = std::string(RT_BUILD_GIT_DESCRIBE) + " " + RT_BUILD_TYPE + " " + RT_BUILD_PLATFORM;
    report_.wallTime = isoNow();
    reportStaged_.clear();

    // THE FACTS, asked now -- the moment of the press, before anything moves
    const std::vector<std::string>& after = fieldReportVerbsAfter();
    for (const std::string& pattern : fieldReportVerbsAtPress()) {
        const std::string line = fieldReportVerbLine(pattern, report_);
        if (line.empty()) continue;
        const std::string reply = commandTable.run(parseControlCommand(line));
        std::string kept;
        if (!fieldReportKeep(reply, kept)) continue;
        if (line == "where?") report_.pose = kept;
        if (line == "aim?") {
            double x = 0, y = 0, z = 0;
            if (std::sscanf(kept.c_str(), "at %lf %lf %lf", &x, &y, &z) == 3) {
                report_.aimHit = true;
                report_.aim[0] = x; report_.aim[1] = y; report_.aim[2] = z;
            }
        }
        // a staged question ("ground? x z") is answered later; its read-back is the fact
        if (line != firstWord(line) &&
            std::find(after.begin(), after.end(), firstWord(line)) != after.end()) {
            reportStaged_.push_back(firstWord(line));
            continue;
        }
        report_.facts.push_back({line, kept});
    }

    if (!rendererPtr->requestFrameDump(report_.shot)) LOG_WARN << "Report: this renderer cannot capture frames";
    reportPhase_ = ReportPhase::Capturing;
    reportFrames_ = 0;
    reportShotSize_ = -1;
    LOG_INFO << "Report: " << report_.dir;
}

void Application::openFieldReport() {
    // the questions staged at the press have had their frames: read them back -- only those staged, or the
    // read-back is whatever was last asked there (another report's spot, an MCP query)
    for (const std::string& line : fieldReportVerbsAfter()) {
        if (std::find(reportStaged_.begin(), reportStaged_.end(), line) == reportStaged_.end()) continue;
        std::string kept;
        if (fieldReportKeep(commandTable.run(parseControlCommand(line)), kept)) report_.facts.push_back({line, kept});
    }
    reportClosing_ = false;
    reportPhase_ = ReportPhase::Open;
    if (reportHeadless_) return;
    stateStack.pushState(std::make_unique<FieldReportState>(*window, report_, [this](const FieldReport& r, bool save) {
        report_ = r;
        reportSave_ = save;
        reportClosing_ = true;   // next frame: the key that closed the box is spent by then
    }));
}

void Application::closeFieldReport() {
    if (!reportHeadless_) stateStack.popState();
    reportPhase_ = ReportPhase::Idle;
    reportClosing_ = false;
    std::error_code ec;
    if (!reportSave_) {
        std::filesystem::remove_all(report_.dir, ec);   // the folder this press made, nothing else
        reportToast_ = "Report cancelled";
        reportToastLeft_ = 2.0;
        return;
    }
    std::string why;
    if (!writeFieldReport(report_, why)) {
        reportToast_ = "Report NOT saved: " + why;
        reportToastLeft_ = 6.0;
        LOG_WARN << reportToast_;
        return;
    }
    reportToast_ = "Report saved: " + report_.dir;
    LOG_INFO << reportToast_;
    if (report_.fileIssue) {
#ifndef _WIN32
        const std::string tool = fileReportTool();
        if (tool.empty()) {
            reportToast_ += "\n(no tools/file_report.py here: file it with `python3 tools/file_report.py " + report_.dir + "`)";
        } else {
            // in the background: the push and the issue take seconds; its log and the issue's link land in the folder
            const std::string cmd = "python3 " + shellQuote(tool) + " " + shellQuote(report_.dir) + " > " +
                                    shellQuote(report_.dir + "/issue.log") + " 2>&1 &";
            if (std::system(cmd.c_str()) == 0) reportToast_ += "\nFiling the GitHub issue (issue.log in the folder)";
            else reportToast_ += "\nCould not start the issue filer";
        }
#endif
    }
    reportToastLeft_ = 5.0;
}

void Application::drawFieldReportToast(const FrameContext& ctx) {
#if defined(RT_DEV_TOOLS) && defined(RT_ENABLE_IMGUI)
    // not while a shot is being taken: the last report's toast is not part of the next one's picture
    if (reportToastLeft_ <= 0 || reportPhase_ == ReportPhase::Capturing || ImGui::GetCurrentContext() == nullptr) return;
    ImGui::SetNextWindowPos(ImVec2(static_cast<float>(ctx.windowWidth) * 0.5f,
                                   static_cast<float>(ctx.windowHeight) - 24.0f),
                            ImGuiCond_Always, ImVec2(0.5f, 1.0f));
    ImGui::SetNextWindowBgAlpha(0.7f);
    ImGui::Begin("##fieldreporttoast", nullptr,
                 ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_AlwaysAutoResize | ImGuiWindowFlags_NoInputs |
                     ImGuiWindowFlags_NoSavedSettings | ImGuiWindowFlags_NoFocusOnAppearing | ImGuiWindowFlags_NoNav);
    ImGui::TextUnformatted(reportToast_.c_str());
    ImGui::End();
#else
    (void)ctx;
#endif
}

}  // namespace engine
