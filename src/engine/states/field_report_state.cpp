#include "field_report_state.h"
#include "../../renderer/window.h"

#include <algorithm>
#include <cstring>
#include <filesystem>
#include <utility>

#ifdef RT_ENABLE_IMGUI
#include <imgui.h>
#endif

namespace engine {

FieldReportState::FieldReportState(Window& window, FieldReport report, Done done)
    : window_(window), report_(std::move(report)), done_(std::move(done)) {}

void FieldReportState::onEnter(FrameContext&) {
    window_.setCursorMode(CursorMode::Normal);
}

void FieldReportState::finish(bool save) {
    if (finished_) return;
    finished_ = true;
    report_.note = note_;
    report_.title = title_;
    if (done_) done_(report_, save);
}

void FieldReportState::render(FrameContext& ctx) {
#ifdef RT_ENABLE_IMGUI
    if (ImGui::GetCurrentContext() == nullptr || finished_) return;
    const ImGuiIO& io = ImGui::GetIO();
    const float w = static_cast<float>(ctx.windowWidth), h = static_cast<float>(ctx.windowHeight);
    ImGui::SetNextWindowPos(ImVec2(w * 0.5f, h * 0.5f), ImGuiCond_Always, ImVec2(0.5f, 0.5f));
    ImGui::SetNextWindowSize(ImVec2(std::min(620.0f, w - 32.0f), 0.0f), ImGuiCond_Always);
    ImGui::SetNextWindowBgAlpha(0.92f);
    ImGui::Begin("Field report", nullptr,
                 ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_NoSavedSettings | ImGuiWindowFlags_NoMove |
                     ImGuiWindowFlags_AlwaysAutoResize);
    ImGui::TextDisabled("%s", report_.level.c_str());
    if (!report_.pose.empty()) ImGui::TextDisabled("teleport %s", report_.pose.c_str());
    if (report_.aimHit)
        ImGui::TextDisabled("looking at (%.1f, %.1f, %.1f)", report_.aim[0], report_.aim[1], report_.aim[2]);
    else
        ImGui::TextDisabled("looking at nothing within range");
    std::error_code ec;
    ImGui::TextDisabled("%s", std::filesystem::exists(report_.shot, ec) ? "screenshot taken" : "no screenshot");
    ImGui::Separator();

    ImGui::TextUnformatted("What's wrong, or worth a look?");
    if (focusNote_) {
        ImGui::SetKeyboardFocusHere();
        focusNote_ = false;
    }
    ImGui::InputTextMultiline("##note", note_, sizeof(note_), ImVec2(-1.0f, ImGui::GetTextLineHeight() * 7.0f));
    ImGui::TextUnformatted("Title (blank: the note's first line)");
    ImGui::SetNextItemWidth(-1.0f);
    ImGui::InputText("##title", title_, sizeof(title_));
    ImGui::Checkbox("File a GitHub issue too (public: the repo is public)", &report_.fileIssue);
    ImGui::Separator();

    bool save = ImGui::Button("Save  (Ctrl+Enter)");
    ImGui::SameLine();
    bool cancel = ImGui::Button("Cancel  (Esc)");
    if (io.KeyCtrl && (ImGui::IsKeyPressed(ImGuiKey_Enter) || ImGui::IsKeyPressed(ImGuiKey_KeypadEnter))) save = true;
    if (ImGui::IsKeyPressed(ImGuiKey_Escape)) cancel = true;
    ImGui::TextDisabled("into %s", report_.dir.c_str());
    ImGui::End();
    if (save) finish(true);
    else if (cancel) finish(false);
#else
    (void)ctx;
    // No UI to type in: keep the shot and the facts, with no note.
    finish(true);
#endif
}

}  // namespace engine
