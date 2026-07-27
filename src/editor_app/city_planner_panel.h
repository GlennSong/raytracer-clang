#ifndef RAYTRACER_EDITOR_CITY_PLANNER_PANEL_H
#define RAYTRACER_EDITOR_CITY_PLANNER_PANEL_H

#include "../engine/editor_bridge.h"

#include <QWidget>
#include <string>

class QCheckBox;
class QComboBox;
class QDoubleSpinBox;
class QFormLayout;
class QLabel;
class QPushButton;
class QSpinBox;

// The City Planner dock (P7.3 phase 1): camera presets (Top/Iso/Free), the
// "Bones" recipe knobs, Regenerate (graph-only + overlay rebuild, ~1s) and
// Bake mesh (the one explicit carriageway build), layer toggles, and the
// stats readout — so road-network tuning is measured in the editor instead
// of regenerate-and-screenshot loops.
//
// The panel owns a copy of the road's WHOLE generate block: the spin boxes
// edit only the keys they display and composeRecipe() splices them back, so
// keys the panel doesn't show survive the round-trip. Signals call the
// bridge directly (editor_main's single-threaded engine pump — the same
// calling pattern as the property inspector). Moc-free, like the shell.
class CityPlannerPanel : public QWidget {
public:
    explicit CityPlannerPanel(engine::EditorBridge& bridge);

    // Call on the UI timer: grays out while the bridge can't edit, and
    // (re)loads the recipe when an edit session (re)attaches.
    void refresh();

private:
    void loadRecipe();
    void regenerate();
    void bake();
    std::string composeRecipe() const;
    void showStats(const engine::CityPlannerStats& stats);

    engine::EditorBridge& bridge;
    std::string recipeJson;   // the whole generate block, as last synced
    uint64_t loadedGen = 0;   // bridge.attachGeneration() the recipe came from

    QSpinBox* seed = nullptr;
    QDoubleSpinBox* siteRadius = nullptr;
    QSpinBox* hotspots = nullptr;
    QDoubleSpinBox* arterialSpan = nullptr;
    bool footprintMode = false;   // recipe carries skeleton:"footprint"
    QFormLayout* bonesForm = nullptr;
    QDoubleSpinBox* districtLen = nullptr;
    QDoubleSpinBox* gateSpacing = nullptr;
    QDoubleSpinBox* sway = nullptr;
    QComboBox* stage = nullptr;
    QDoubleSpinBox* loopMin = nullptr;
    QDoubleSpinBox* loopMax = nullptr;
    QDoubleSpinBox* killRadius = nullptr;
    QDoubleSpinBox* corridorSpacing = nullptr;
    QPushButton* regenButton = nullptr;
    QPushButton* bakeButton = nullptr;
    QPushButton* clearButton = nullptr;
    QCheckBox* layerFootprint = nullptr;
    QCheckBox* layerHubs = nullptr;
    QCheckBox* layerArterials = nullptr;
    QCheckBox* layerNodes = nullptr;
    QLabel* stats = nullptr;
};

#endif
