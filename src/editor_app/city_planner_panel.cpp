#include "city_planner_panel.h"

#include <QCheckBox>
#include <QDoubleSpinBox>
#include <QFormLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QLabel>
#include <QPushButton>
#include <QSpinBox>
#include <QToolButton>
#include <QVBoxLayout>

#include <nlohmann/json.hpp>

using nlohmann::json;
using engine::CityPlannerStats;

namespace {

// Bones spin-box factory: metres, wide sane range, no live signal wiring —
// values are read on [Regenerate], never pushed per keystroke.
QDoubleSpinBox* meterSpin(double lo, double hi, const char* objectName) {
    auto* spin = new QDoubleSpinBox;
    spin->setRange(lo, hi);
    spin->setDecimals(0);
    spin->setSuffix(" m");
    spin->setObjectName(objectName);
    return spin;
}

}  // namespace

CityPlannerPanel::CityPlannerPanel(engine::EditorBridge& bridge)
    : bridge(bridge) {
    auto* column = new QVBoxLayout(this);

    // Camera presets: plan views on the orbit-ortho camera, Free restores fly.
    auto* cameraRow = new QHBoxLayout;
    auto addPreset = [&](const char* label, int preset, const char* tip) {
        auto* button = new QToolButton(this);
        button->setText(label);
        button->setToolTip(tip);
        QObject::connect(button, &QToolButton::clicked,
                         [this, preset]() { this->bridge.plannerCameraPreset(preset); });
        cameraRow->addWidget(button);
    };
    addPreset("Top", 0, "Top-down orthographic plan view of the city site");
    addPreset("Iso", 1, "Isometric orthographic view (yaw 45, pitch 35.26)");
    addPreset("Free", 2, "Free perspective fly camera");
    cameraRow->addStretch();
    column->addLayout(cameraRow);

    // Bones: the generate-block knobs the phase-1 iteration loop turns.
    auto* bones = new QGroupBox("Bones", this);
    auto* form = new QFormLayout(bones);
    seed = new QSpinBox;
    seed->setRange(1, 999999);
    seed->setObjectName("plannerSeed");
    form->addRow("Seed", seed);
    siteRadius = meterSpin(40, 6000, "plannerSiteRadius");
    form->addRow("Site radius", siteRadius);
    hotspots = new QSpinBox;
    hotspots->setRange(1, 64);
    hotspots->setObjectName("plannerHotspots");
    form->addRow("Hotspots cap", hotspots);
    arterialSpan = meterSpin(0, 3000, "plannerArterialSpan");
    arterialSpan->setSpecialValueText("(min_block_edge)");   // 0 = derive
    form->addRow("Arterial span", arterialSpan);
    loopMin = meterSpin(0, 10000, "plannerLoopMin");
    form->addRow("Loop min", loopMin);
    loopMax = meterSpin(0, 10000, "plannerLoopMax");
    form->addRow("Loop max", loopMax);
    killRadius = meterSpin(0, 5000, "plannerKillRadius");
    form->addRow("Kill radius", killRadius);
    corridorSpacing = meterSpin(0, 5000, "plannerCorridorSpacing");
    form->addRow("Corridor spacing", corridorSpacing);
    column->addWidget(bones);

    // Regenerate (graph + overlays, fast) | Bake mesh (the slow explicit one).
    auto* buttonRow = new QHBoxLayout;
    regenButton = new QPushButton("Regenerate", this);
    regenButton->setObjectName("plannerRegenerate");
    regenButton->setToolTip(
        "Re-run the recipe: graph + schematic overlays only (no road mesh)");
    QObject::connect(regenButton, &QPushButton::clicked,
                     [this]() { regenerate(); });
    buttonRow->addWidget(regenButton);
    bakeButton = new QPushButton("Bake mesh", this);
    bakeButton->setObjectName("plannerBake");
    bakeButton->setToolTip("Build the full carriageway mesh from the graph");
    QObject::connect(bakeButton, &QPushButton::clicked, [this]() { bake(); });
    buttonRow->addWidget(bakeButton);
    column->addLayout(buttonRow);

    // Layers: overlay visibility (rebuilt only on Regenerate).
    auto* layers = new QGroupBox("Layers", this);
    auto* layerColumn = new QVBoxLayout(layers);
    auto addLayer = [&](const char* label, const char* key) {
        auto* box = new QCheckBox(label, layers);
        box->setChecked(true);
        box->setObjectName(QString("plannerLayer_") + key);
        std::string name = key;
        QObject::connect(box, &QCheckBox::toggled, [this, name](bool on) {
            this->bridge.plannerSetLayer(name, on);
        });
        layerColumn->addWidget(box);
        return box;
    };
    layerHubs = addLayer("Hubs", "hubs");
    layerArterials = addLayer("Arterials", "arterials");
    layerNodes = addLayer("Nodes", "nodes");
    column->addWidget(layers);

    stats = new QLabel("—", this);
    stats->setObjectName("plannerStats");
    stats->setWordWrap(true);
    column->addWidget(stats);
    column->addStretch();
}

void CityPlannerPanel::refresh() {
    const bool editing = bridge.editable();
    setEnabled(editing);
    if (!editing) return;
    // Compare the ATTACH GENERATION, not an editable() dip: opening a level
    // from the asset browser swaps editor states within one engine frame, so
    // a 150ms poll never sees the panel detached — a boolean latch kept the
    // previous level's recipe forever ("No recipe to regenerate").
    if (loadedGen != bridge.attachGeneration()) loadRecipe();
}

void CityPlannerPanel::loadRecipe() {
    loadedGen = bridge.attachGeneration();
    recipeJson = bridge.plannerRecipe();
    if (recipeJson.empty()) {
        stats->setText("No generated road in this level");
        return;
    }
    json gen = json::parse(recipeJson, nullptr, false);
    if (!gen.is_object()) {
        stats->setText("Unreadable generate recipe");
        return;
    }
    // The primary site's radius/hotspots live in sites[0] when the recipe is
    // multi-site; single-site recipes keep them at the top level.
    const json* site = &gen;
    if (gen.contains("sites") && gen["sites"].is_array() && !gen["sites"].empty())
        site = &gen["sites"][0];

    seed->setValue(gen.value("seed", 5));
    siteRadius->setValue(site->value("radius", 700.0));
    hotspots->setValue(site->value("hotspots", 6));
    arterialSpan->setValue(gen.value("arterial_span", 0.0));
    // Growth-dial defaults mirror MetroParams (metro.h).
    loopMin->setValue(gen.value("loop_min", 80.0));
    loopMax->setValue(gen.value("loop_max", 190.0));
    killRadius->setValue(gen.value("kill_radius", 48.0));
    corridorSpacing->setValue(gen.value("corridor_spacing", 55.0));
    stats->setText("Recipe loaded — Regenerate to grow the bones");
}

std::string CityPlannerPanel::composeRecipe() const {
    json gen = json::parse(recipeJson, nullptr, false);
    if (!gen.is_object()) return "";
    gen["seed"] = seed->value();
    // Write the panel's keys back into the WHOLE block (everything it does
    // not display — fabric dials, widths, sites 1.. — passes through intact).
    if (gen.contains("sites") && gen["sites"].is_array() &&
        !gen["sites"].empty()) {
        gen["sites"][0]["radius"] = siteRadius->value();
        gen["sites"][0]["hotspots"] = hotspots->value();
    } else {
        gen["radius"] = siteRadius->value();
        gen["hotspots"] = hotspots->value();
    }
    // arterial_span 0 = "derive from min_block_edge"; only persist the key
    // once it is set (or was already present) so recipes aren't polluted.
    if (arterialSpan->value() > 0.0 || gen.contains("arterial_span"))
        gen["arterial_span"] = arterialSpan->value();
    gen["loop_min"] = loopMin->value();
    gen["loop_max"] = loopMax->value();
    gen["kill_radius"] = killRadius->value();
    gen["corridor_spacing"] = corridorSpacing->value();
    return gen.dump();
}

void CityPlannerPanel::showStats(const CityPlannerStats& s) {
    stats->setText(QString("%1 nodes · %2 edges · %3 spans · %4 hubs · %5 ms")
                       .arg(s.nodes)
                       .arg(s.edges)
                       .arg(s.spans)
                       .arg(s.hubs)
                       .arg(static_cast<int>(s.regenMs + 0.5)));
}

void CityPlannerPanel::regenerate() {
    const std::string gen = composeRecipe();
    if (gen.empty()) {
        stats->setText("No recipe to regenerate (open a level with a "
                       "generated road)");
        return;
    }
    CityPlannerStats result = bridge.plannerApplyRecipe(gen);
    if (!result.ok) {
        stats->setText("Regenerate failed (see console)");
        return;
    }
    recipeJson = bridge.plannerRecipe();   // re-sync from the saved truth
    showStats(result);
}

void CityPlannerPanel::bake() {
    stats->setText(bridge.plannerBakeMesh()
                       ? "Baked the carriageway mesh"
                       : "Bake failed (regenerate first?)");
}
