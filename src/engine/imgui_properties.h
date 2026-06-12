#ifndef RAYTRACER_ENGINE_IMGUI_PROPERTIES_H
#define RAYTRACER_ENGINE_IMGUI_PROPERTIES_H

#ifdef RT_ENABLE_IMGUI

#include "properties.h"

namespace engine {

// PropertyVisitor that renders Dear ImGui widgets — the in-viewport twin of
// the Qt inspector's build/sync/write passes, collapsed into one immediate-
// mode pass. FieldMeta semantics map to widgets here and nowhere else:
// bounded scalar -> slider (logScale -> logarithmic), unbounded -> drag,
// color Vec3 -> color edit, choices -> combo, readOnly -> plain text.
//
// Edits apply to the live component immediately. After the visit the caller
// can ask what happened:
//  - editedLabel(): the field written this frame (null if none) — feed it to
//    the registry's post-edit hook.
//  - interactionStarted()/interactionFinished(): a continuous interaction
//    (drag, typing) began / committed on some widget this visit. Instant
//    widgets (checkbox, combo) report both on the same frame. Bracket these
//    with component snapshots to record drag-granular undo entries.
class ImGuiPropertyVisitor : public PropertyVisitor {
public:
    void field(const FieldMeta& meta, Real& value) override;
    void field(const FieldMeta& meta, float& value) override;
    void field(const FieldMeta& meta, Vec3& value) override;
    void field(const FieldMeta& meta, bool& value) override;
    void field(const FieldMeta& meta, std::string& value) override;
    void bitFlag(const FieldMeta& meta, uint32_t& flags, uint32_t mask) override;

    bool edited() const { return editedField != nullptr; }
    const char* editedLabel() const { return editedField; }
    bool interactionStarted() const { return started; }
    bool interactionFinished() const { return finished; }

private:
    bool scalarWidget(const FieldMeta& meta, float& value);
    void noteEdited(const FieldMeta& meta) { editedField = meta.label; }
    // Call directly after a widget: records activation/commit transitions.
    // `instant` widgets commit on the same frame they change.
    void trackItem(bool instant, bool changed);

    const char* editedField = nullptr;   // FieldMeta labels are literals
    bool started = false;
    bool finished = false;
};

}  // namespace engine

#endif  // RT_ENABLE_IMGUI
#endif
