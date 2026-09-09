#ifndef RAYTRACER_EDITOR_APP_BAKE_DIALOG_H
#define RAYTRACER_EDITOR_APP_BAKE_DIALOG_H

// "Bake level cache" options (ADR-0084): the output root, whether to write the Blender GLB (per material,
// per cell), and whether to force a rebuild. Pure Qt and Q_OBJECT-free like the rest of the editor shell,
// so it builds into editor_qt_tests without lanelab or a running engine.

#include <QDialog>
#include <QString>
#include <optional>

class QCheckBox;
class QLineEdit;

struct BakeOptions {
    QString outRoot;        // bundle root; empty = the engine's default (cache/levels)
    bool glb = false;
    bool splitCells = false;
    bool force = false;
};

class BakeDialog : public QDialog {
public:
    BakeDialog(QWidget* parent, const BakeOptions& defaults, const QString& levelName);
    BakeOptions options() const;

    QLineEdit* rootEdit = nullptr;
    QCheckBox* glbBox = nullptr;
    QCheckBox* splitBox = nullptr;
    QCheckBox* forceBox = nullptr;
};

// Modal; nullopt on Cancel.
std::optional<BakeOptions> showBakeDialog(QWidget* parent, const BakeOptions& defaults, const QString& levelName);

#endif
