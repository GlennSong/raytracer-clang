// "Bake level cache" dialog (ADR-0084): defaults land in the widgets, choices come back out. Runs in
// editor_qt_tests' offscreen QApplication; returns its failure count.
#include "../src/editor_app/bake_dialog.h"

#include <QCheckBox>
#include <QLineEdit>
#include <cstdio>

namespace {
int bakeFailures = 0;
#define BAKE_REQUIRE(cond) do { if (!(cond)) { std::printf("    FAIL %s:%d  %s\n", __FILE__, __LINE__, #cond); ++bakeFailures; } } while (0)
}  // namespace

int runBakeDialogQtTests() {
    BakeOptions defaults; defaults.outRoot = "cache/levels"; defaults.glb = true; defaults.splitCells = false; defaults.force = false;
    BakeDialog d(nullptr, defaults, "lanelab_ring.json");
    BAKE_REQUIRE(d.rootEdit->text() == "cache/levels");
    BAKE_REQUIRE(d.glbBox->isChecked() && !d.splitBox->isChecked() && !d.forceBox->isChecked());
    d.rootEdit->setText("  /tmp/bundles  "); d.splitBox->setChecked(true); d.forceBox->setChecked(true); d.glbBox->setChecked(false);
    const BakeOptions o = d.options();
    BAKE_REQUIRE(o.outRoot == "/tmp/bundles" && !o.glb && o.splitCells && o.force);
    BAKE_REQUIRE(d.windowTitle() == "Bake level cache");
    std::printf("[bake dialog] %s\n", bakeFailures == 0 ? "ok" : "FAILED");
    return bakeFailures;
}
