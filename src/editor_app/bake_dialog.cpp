#include "bake_dialog.h"

#include <QCheckBox>
#include <QDialogButtonBox>
#include <QFileDialog>
#include <QFormLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QPushButton>

BakeDialog::BakeDialog(QWidget* parent, const BakeOptions& defaults, const QString& levelName) : QDialog(parent) {
    setWindowTitle("Bake level cache");
    auto* form = new QFormLayout(this);
    auto* what = new QLabel(QString("Prebuild <b>%1</b> into a bundle the engine loads instead of rebuilding the city.").arg(levelName.toHtmlEscaped()), this);
    what->setWordWrap(true);
    form->addRow(what);

    auto* rootRow = new QWidget(this); auto* rootLayout = new QHBoxLayout(rootRow); rootLayout->setContentsMargins(0, 0, 0, 0);
    rootEdit = new QLineEdit(defaults.outRoot, rootRow); rootEdit->setPlaceholderText("cache/levels (default)");
    auto* browse = new QPushButton("…", rootRow); browse->setFixedWidth(32);
    QObject::connect(browse, &QPushButton::clicked, [this]() {
        const QString start = rootEdit->text().isEmpty() ? QString("cache/levels") : rootEdit->text();
        const QString dir = QFileDialog::getExistingDirectory(this, "Bundle output folder", start);
        if (!dir.isEmpty()) rootEdit->setText(dir);
    });
    rootLayout->addWidget(rootEdit, 1); rootLayout->addWidget(browse);
    form->addRow("Output folder", rootRow);

    glbBox = new QCheckBox("Write city.glb for Blender (one object per material)", this); glbBox->setChecked(defaults.glb);
    splitBox = new QCheckBox("Also one GLB per 250 m cell (cells/ + index.json)", this); splitBox->setChecked(defaults.splitCells);
    forceBox = new QCheckBox("Force rebuild even when the cache is current", this); forceBox->setChecked(defaults.force);
    form->addRow(glbBox); form->addRow(splitBox); form->addRow(forceBox);

    auto* buttons = new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel, this);
    buttons->button(QDialogButtonBox::Ok)->setText("Bake");
    QObject::connect(buttons, &QDialogButtonBox::accepted, this, &QDialog::accept);
    QObject::connect(buttons, &QDialogButtonBox::rejected, this, &QDialog::reject);
    form->addRow(buttons);
}

BakeOptions BakeDialog::options() const {
    BakeOptions o; o.outRoot = rootEdit->text().trimmed(); o.glb = glbBox->isChecked(); o.splitCells = splitBox->isChecked(); o.force = forceBox->isChecked(); return o;
}

std::optional<BakeOptions> showBakeDialog(QWidget* parent, const BakeOptions& defaults, const QString& levelName) {
    BakeDialog d(parent, defaults, levelName);
    if (d.exec() != QDialog::Accepted) return std::nullopt;
    return d.options();
}
