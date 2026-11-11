#ifndef RAYTRACER_ENGINE_STATES_FIELD_REPORT_STATE_H
#define RAYTRACER_ENGINE_STATES_FIELD_REPORT_STATE_H

#include "../app_state.h"
#include "../field_report.h"

#include <functional>

namespace engine {

class Window;

// The note box of a field report (field_report.h): over the frozen world (it blocks input, so nothing under it
// updates), a box for the note, a title, and "File a GitHub issue". Save (Ctrl+Enter) or Cancel (Esc) hands the
// report back through `done`; the Application closes the box on the NEXT frame, so the Esc that cancelled is
// never also the game's quit.
class FieldReportState : public AppState {
public:
    using Done = std::function<void(const FieldReport& report, bool save)>;
    FieldReportState(Window& window, FieldReport report, Done done);

    bool isTransparent() const override { return true; }
    bool blocksInput() const override { return true; }

    void onEnter(FrameContext& ctx) override;
    void render(FrameContext& ctx) override;

private:
    Window& window_;
    FieldReport report_;
    Done done_;
    bool finished_ = false;
    bool focusNote_ = true;
    char note_[4096] = {};
    char title_[160] = {};
    void finish(bool save);
};

}  // namespace engine

#endif
