#ifndef RAYTRACER_EDITOR_PROPERTY_INSPECTOR_H
#define RAYTRACER_EDITOR_PROPERTY_INSPECTOR_H

#include "../engine/editor_bridge.h"

#include <QWidget>
#include <memory>
#include <vector>

class QVBoxLayout;

// The Unity-style inspector: one collapsible section per component the
// selected entity carries, every row generated from the engine's property
// descriptions (describeProperties + ComponentRegistry) — this file knows
// widgets, never fields. Three visitor passes share one field ordering:
// build (create widgets), sync (engine -> widgets, skipping focused ones),
// write (one widget -> engine). Widgets never hold references into the ECS;
// every pass re-resolves through the registry, so component-storage moves
// and entity replacement are safe by construction. Moc-free.
class PropertyInspector : public QWidget {
public:
    explicit PropertyInspector(engine::EditorBridge& bridge);
    ~PropertyInspector() override;

    // Call on a UI timer: rebuilds sections when the selection changed,
    // otherwise mirrors engine values into unfocused widgets.
    void refresh();

private:
    struct Impl;
    void rebuild(engine::Entity entity);
    void writeField(int componentIndex, int fieldIndex);

    engine::EditorBridge& bridge;
    QVBoxLayout* sectionsLayout = nullptr;
    std::unique_ptr<Impl> impl;
    engine::Entity shown;
};

#endif
