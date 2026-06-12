#include "property_inspector.h"

#include "../engine/components.h"
#include "../engine/undo_stack.h"

#include <QCheckBox>
#include <QComboBox>
#include <QDoubleSpinBox>
#include <QFormLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QMenu>
#include <QPushButton>
#include <QVBoxLayout>

#include <cmath>

using namespace engine;

namespace {

// One described field's widgets. Which pointers are set depends on the field
// type the build pass saw; sync/write passes see the same field order, so the
// row index is the address.
struct FieldRow {
    bool readOnly = false;
    QDoubleSpinBox* nums[3] = {};   // Real/float: [0]; Vec3: [0..2]
    QCheckBox* check = nullptr;     // bool / bitFlag
    QLineEdit* text = nullptr;      // string
    QComboBox* combo = nullptr;     // string with choices
    QLabel* lockedLabel = nullptr;  // read-only display

    bool hasFocus() const {
        for (auto* n : nums)
            if (n && n->hasFocus()) return true;
        return (text && text->hasFocus()) || (combo && combo->hasFocus());
    }
};

QString vecToText(const Vec3& v) {
    return QString("%1, %2, %3").arg(v.x, 0, 'g', 4).arg(v.y, 0, 'g', 4)
        .arg(v.z, 0, 'g', 4);
}

// Creates a row of widgets per field and records it. `onEdit(fieldIndex)` is
// wired into every editable widget.
class BuildVisitor : public PropertyVisitor {
public:
    BuildVisitor(QFormLayout* form, std::vector<FieldRow>& rows,
                 std::function<void(int)> onEdit)
        : form(form), rows(rows), onEdit(std::move(onEdit)) {}

    void field(const FieldMeta& meta, Real& value) override {
        scalarRow(meta, value);
    }
    void field(const FieldMeta& meta, float& value) override {
        scalarRow(meta, static_cast<Real>(value));
    }
    void field(const FieldMeta& meta, Vec3& value) override {
        if (meta.readOnly) return lockedRow(meta, vecToText(value));
        FieldRow row;
        auto* holder = new QWidget;
        auto* h = new QHBoxLayout(holder);
        h->setContentsMargins(0, 0, 0, 0);
        for (int i = 0; i < 3; i++) {
            row.nums[i] = makeSpin(meta);
            Real start = (i == 0) ? value.x : (i == 1) ? value.y : value.z;
            row.nums[i]->setValue(start);
            connectSpin(row.nums[i]);
            h->addWidget(row.nums[i]);
        }
        addRow(meta, holder, row);
    }
    void field(const FieldMeta& meta, bool& value) override {
        FieldRow row;
        row.check = new QCheckBox;
        row.check->setChecked(value);
        row.check->setEnabled(!meta.readOnly);
        auto fire = edit(static_cast<int>(rows.size()));
        QObject::connect(row.check, &QCheckBox::toggled,
                         [fire](bool) { fire(); });
        addRow(meta, row.check, row);
    }
    void field(const FieldMeta& meta, std::string& value) override {
        if (meta.readOnly)
            return lockedRow(meta, QString::fromStdString(value));
        FieldRow row;
        int index = static_cast<int>(rows.size());
        if (!meta.choices.empty()) {
            row.combo = new QComboBox;
            for (const char* opt : meta.choices)
                row.combo->addItem(opt);
            row.combo->setCurrentText(QString::fromStdString(value));
            auto fire = edit(index);
            QObject::connect(row.combo, &QComboBox::currentTextChanged,
                             [fire](const QString&) { fire(); });
            addRow(meta, row.combo, row);
        } else {
            row.text = new QLineEdit(QString::fromStdString(value));
            auto fire = edit(index);
            QObject::connect(row.text, &QLineEdit::editingFinished,
                             [fire]() { fire(); });
            addRow(meta, row.text, row);
        }
    }
    void bitFlag(const FieldMeta& meta, uint32_t& flags, uint32_t mask) override {
        bool set = (flags & mask) != 0;
        field(meta, set);   // builds the same checkbox row
    }

private:
    void scalarRow(const FieldMeta& meta, Real value) {
        if (meta.readOnly)
            return lockedRow(meta, QString::number(value, 'g', 5));
        FieldRow row;
        row.nums[0] = makeSpin(meta);
        row.nums[0]->setValue(value);
        connectSpin(row.nums[0]);
        addRow(meta, row.nums[0], row);
    }
    void lockedRow(const FieldMeta& meta, const QString& textValue) {
        FieldRow row;
        row.readOnly = true;
        row.lockedLabel = new QLabel(textValue);
        row.lockedLabel->setStyleSheet("color: gray");
        addRow(meta, row.lockedLabel, row);
    }
    // The connected lambdas MUST NOT capture this visitor — it's stack-local
    // to rebuild() and long dead by the time a widget fires. They capture a
    // copy of the callback (which holds the long-lived PropertyInspector).
    std::function<void()> edit(int index) const {
        auto fn = onEdit;
        return [fn, index]() { fn(index); };
    }

    QDoubleSpinBox* makeSpin(const FieldMeta& meta) const {
        auto* spin = new QDoubleSpinBox;
        bool bounded = meta.minValue != 0.0 || meta.maxValue != 0.0;
        spin->setRange(bounded ? meta.minValue : -1e9,
                       bounded ? meta.maxValue : 1e9);
        spin->setSingleStep(meta.step > 0.0 ? meta.step : 0.1);
        spin->setDecimals(3);
        spin->setKeyboardTracking(false);
        if (meta.unit[0]) spin->setSuffix(QString(" %1").arg(meta.unit));
        return spin;
    }
    void connectSpin(QDoubleSpinBox* spin) {
        auto fire = edit(static_cast<int>(rows.size()));
        QObject::connect(spin, qOverload<double>(&QDoubleSpinBox::valueChanged),
                         [fire](double) { fire(); });
    }
    void addRow(const FieldMeta& meta, QWidget* widget, FieldRow& row) {
        form->addRow(meta.label, widget);
        rows.push_back(row);
    }

    QFormLayout* form;
    std::vector<FieldRow>& rows;
    std::function<void(int)> onEdit;
};

// Engine -> widgets, same field order as the build pass. Focused widgets are
// left alone so the refresh loop never fights typing; `muted` suppresses the
// valueChanged -> writeField echo.
class SyncVisitor : public PropertyVisitor {
public:
    SyncVisitor(std::vector<FieldRow>& rows, bool& muted)
        : rows(rows), muted(muted) {}

    void field(const FieldMeta&, Real& value) override { scalar(value); }
    void field(const FieldMeta&, float& value) override {
        scalar(static_cast<Real>(value));
    }
    void field(const FieldMeta&, Vec3& value) override {
        FieldRow* row = next();
        if (!row || row->hasFocus()) return;
        if (row->readOnly) {
            if (row->lockedLabel) row->lockedLabel->setText(vecToText(value));
            return;
        }
        const Real parts[3] = {value.x, value.y, value.z};
        for (int i = 0; i < 3; i++)
            if (row->nums[i] && row->nums[i]->value() != parts[i])
                row->nums[i]->setValue(parts[i]);
    }
    void field(const FieldMeta&, bool& value) override {
        FieldRow* row = next();
        if (row && row->check && row->check->isChecked() != value)
            row->check->setChecked(value);
    }
    void field(const FieldMeta&, std::string& value) override {
        FieldRow* row = next();
        if (!row || row->hasFocus()) return;
        QString v = QString::fromStdString(value);
        if (row->lockedLabel) row->lockedLabel->setText(v);
        if (row->text && row->text->text() != v) row->text->setText(v);
        if (row->combo && row->combo->currentText() != v)
            row->combo->setCurrentText(v);
    }
    void bitFlag(const FieldMeta& meta, uint32_t& flags, uint32_t mask) override {
        bool set = (flags & mask) != 0;
        field(meta, set);
    }

private:
    void scalar(Real value) {
        FieldRow* row = next();
        if (!row || row->hasFocus()) return;
        if (row->readOnly) {
            if (row->lockedLabel)
                row->lockedLabel->setText(QString::number(value, 'g', 5));
            return;
        }
        if (row->nums[0] && row->nums[0]->value() != value)
            row->nums[0]->setValue(value);
    }
    FieldRow* next() {
        (void)muted;
        return cursor < rows.size() ? &rows[cursor++] : nullptr;
    }

    std::vector<FieldRow>& rows;
    bool& muted;
    std::size_t cursor = 0;
};

// One widget -> engine, addressed by field index within the component.
// `editedLabel()` reports which described field the write landed on (null if
// the target was out of range or read-only) — the key for post-edit hooks
// and the undo log.
class WriteVisitor : public PropertyVisitor {
public:
    WriteVisitor(std::vector<FieldRow>& rows, int target)
        : rows(rows), target(target) {}

    const char* editedLabel() const { return edited; }

    void field(const FieldMeta& meta, Real& value) override {
        if (FieldRow* row = match(meta))
            if (row->nums[0]) value = row->nums[0]->value();
    }
    void field(const FieldMeta& meta, float& value) override {
        if (FieldRow* row = match(meta))
            if (row->nums[0]) value = static_cast<float>(row->nums[0]->value());
    }
    void field(const FieldMeta& meta, Vec3& value) override {
        if (FieldRow* row = match(meta))
            if (row->nums[0] && row->nums[1] && row->nums[2])
                value = Vec3(row->nums[0]->value(), row->nums[1]->value(),
                             row->nums[2]->value());
    }
    void field(const FieldMeta& meta, bool& value) override {
        if (FieldRow* row = match(meta))
            if (row->check) value = row->check->isChecked();
    }
    void field(const FieldMeta& meta, std::string& value) override {
        FieldRow* row = match(meta);
        if (!row) return;
        if (row->combo) value = row->combo->currentText().toStdString();
        else if (row->text) value = row->text->text().toStdString();
    }
    void bitFlag(const FieldMeta& meta, uint32_t& flags, uint32_t mask) override {
        if (FieldRow* row = match(meta)) {
            if (!row->check) return;
            flags = row->check->isChecked() ? (flags | mask) : (flags & ~mask);
        }
    }

private:
    FieldRow* match(const FieldMeta& meta) {
        int index = cursor++;
        if (index != target || meta.readOnly) return nullptr;
        if (static_cast<std::size_t>(index) >= rows.size()) return nullptr;
        edited = meta.label;   // FieldMeta labels are string literals
        return &rows[index];
    }

    std::vector<FieldRow>& rows;
    int target;
    int cursor = 0;
    const char* edited = nullptr;
};

}  // namespace

struct PropertyInspector::Impl {
    // Registry entry index -> that component's field rows (build order).
    struct Section {
        int entryIndex = -1;
        std::vector<FieldRow> rows;
    };
    std::vector<Section> sections;
    bool muted = false;            // suppress write-back while syncing
    // Add/remove component happened (menu click, or engine-side/undo): the
    // section list is stale. Rebuilt on the NEXT refresh — never mid-signal,
    // so a button's own click can't delete it.
    bool structureDirty = false;
};

PropertyInspector::PropertyInspector(engine::EditorBridge& bridge)
    : bridge(bridge), impl(std::make_unique<Impl>()) {
    sectionsLayout = new QVBoxLayout(this);
    sectionsLayout->setAlignment(Qt::AlignTop);
}

PropertyInspector::~PropertyInspector() = default;

void PropertyInspector::rebuild(engine::Entity entity) {
    // Drop the old sections wholesale; widget lifetimes stay trivial.
    while (QLayoutItem* item = sectionsLayout->takeAt(0)) {
        delete item->widget();
        delete item;
    }
    impl->sections.clear();
    shown = entity;

    World* world = bridge.world();
    if (!world || !world->alive(entity)) return;

    const auto& entries = bridge.registry().entries();
    for (std::size_t i = 0; i < entries.size(); i++) {
        if (!entries[i].has(*world, entity)) continue;

        auto* box = new QGroupBox(QString::fromStdString(entries[i].name));
        auto* form = new QFormLayout(box);

        Impl::Section section;
        section.entryIndex = static_cast<int>(i);
        int componentIndex = static_cast<int>(impl->sections.size());
        impl->sections.push_back(std::move(section));

        BuildVisitor builder(form, impl->sections.back().rows,
                             [this, componentIndex](int fieldIndex) {
                                 writeField(componentIndex, fieldIndex);
                             });
        entries[i].visit(*world, entity, builder);

        if (entries[i].removeFrom) {
            auto* removeButton = new QPushButton("Remove Component");
            QObject::connect(removeButton, &QPushButton::clicked,
                             [this, name = entries[i].name]() {
                                 if (!bridge.attached()) return;
                                 bridge.removeComponent(bridge.selected(), name);
                                 impl->structureDirty = true;
                             });
            form->addRow(QString(), removeButton);
        }
        sectionsLayout->addWidget(box);
    }

    // Add Component: registered types the entity lacks that allow menu
    // attachment (the registry's addTo thunks).
    auto* addMenu = new QMenu;
    for (const auto& entry : entries) {
        if (!entry.addTo || entry.has(*world, entity)) continue;
        addMenu->addAction(QString::fromStdString(entry.name),
                           [this, name = entry.name]() {
                               if (!bridge.attached()) return;
                               bridge.addComponent(bridge.selected(), name);
                               impl->structureDirty = true;
                           });
    }
    if (addMenu->isEmpty()) {
        delete addMenu;
    } else {
        auto* addButton = new QPushButton("Add Component");
        addMenu->setParent(addButton);
        addButton->setMenu(addMenu);
        sectionsLayout->addWidget(addButton);
    }
}

void PropertyInspector::writeField(int componentIndex, int fieldIndex) {
    if (impl->muted || !bridge.attached()) return;
    World* world = bridge.world();
    Entity entity = bridge.selected();
    if (!world || !world->alive(entity)) return;
    if (componentIndex < 0 ||
        componentIndex >= static_cast<int>(impl->sections.size()))
        return;

    auto& section = impl->sections[componentIndex];
    const auto& entries = bridge.registry().entries();
    const auto& entry = entries[section.entryIndex];

    UndoStack* undo = bridge.undoStack();
    nlohmann::json before;
    if (undo) before = undo->componentState(*world, entity, entry.name);

    WriteVisitor writer(section.rows, fieldIndex);
    entry.visit(*world, entity, writer);

    if (const char* label = writer.editedLabel()) {
        // Post-edit hook (e.g. Shape size -> mesh rebuild), then the undo
        // log: one command per committed widget edit, consecutive edits to
        // the same field coalesced by the stack.
        if (entry.onEdited) entry.onEdited(*world, entity, label);
        if (undo)
            undo->recordFieldEdit(
                entity, entry.name, label, std::move(before),
                undo->componentState(*world, entity, entry.name));
    }

    // Editor objects are stationary: keep the interpolation pair in step so
    // edits don't smear across frames.
    if (Transform* t = world->get<Transform>(entity))
        if (auto* prev = world->get<PrevTransform>(entity)) prev->value = *t;
}

void PropertyInspector::refresh() {
    if (!bridge.attached()) {
        if (shown.valid()) rebuild(engine::Entity{});
        return;
    }
    Entity selected = bridge.selected();
    if (!(selected == shown) ||
        (shown.valid() && !bridge.world()->alive(shown))) {
        impl->structureDirty = false;
        rebuild(selected);
        return;
    }
    if (!shown.valid()) return;

    // The entity's component set can change under us — the Add Component
    // menu, the in-viewport inspector, or an undo. Compare what's present
    // against the sections we built and rebuild on mismatch.
    if (!impl->structureDirty) {
        const auto& entries = bridge.registry().entries();
        std::size_t sectionCursor = 0;
        for (std::size_t i = 0; i < entries.size(); i++) {
            if (!entries[i].has(*bridge.world(), shown)) continue;
            if (sectionCursor >= impl->sections.size() ||
                impl->sections[sectionCursor].entryIndex != static_cast<int>(i)) {
                impl->structureDirty = true;
                break;
            }
            sectionCursor++;
        }
        if (sectionCursor != impl->sections.size()) impl->structureDirty = true;
    }
    if (impl->structureDirty) {
        impl->structureDirty = false;
        rebuild(shown);
        return;
    }

    impl->muted = true;
    World* world = bridge.world();
    const auto& entries = bridge.registry().entries();
    for (auto& section : impl->sections) {
        SyncVisitor sync(section.rows, impl->muted);
        entries[section.entryIndex].visit(*world, shown, sync);
    }
    impl->muted = false;
}
