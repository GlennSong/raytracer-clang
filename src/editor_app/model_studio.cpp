// The model studio (#191). See model_studio.h.
#include "model_studio.h"

#include <QAction>
#include <QApplication>
#include <QCheckBox>
#include <QClipboard>
#include <QComboBox>
#include <QFormLayout>
#include <QDoubleSpinBox>
#include <QHeaderView>
#include <QSplitter>
#include <QTreeWidget>
#include <QPainter>
#include <QSyntaxHighlighter>
#include <QTextCharFormat>
#include <QDir>
#include <QFile>
#include <QFileDialog>
#include <QFileInfo>
#include <QFileSystemWatcher>
#include <QFontDatabase>
#include <QHBoxLayout>
#include <QInputDialog>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QLineEdit>
#include <QKeySequence>
#include <QLabel>
#include <QMessageBox>
#include <QPlainTextEdit>
#include <QPushButton>
#include <QRegularExpression>
#include <QScrollArea>
#include <QShortcut>
#include <QTextBlock>
#include <QTextCursor>
#include <QTextStream>
#include <QVBoxLayout>

namespace {

const char* kScripts = "assets/scripts";
const char* kLevels = "cache/model_studio";      // (cache/ is not tracked: these levels are the studio's own)

// What a new model starts as: enough to see something, with the words a model is usually made of.
const char* kStarter = R"LUA(-- %1: a model (docs/model-record-design.md). Save to rebuild it in the viewport.
local m = model.new()
m:material("paint", material.new{ roughness = 0.5, metallic = 0.1 })

-- a block with its edges cut back, standing on the ground
local body = poly.box{ 1, 1, 1 }
body:translate{ 0, 0.5, 0 }
body:bevel(0.05, 30, nil, 2)
for _, part in pairs(body:to_parts(30)) do
  m:add(mesh.paint(part, { 0.8, 0.35, 0.2 }), "paint")
end

-- what a copy of it collides with
m:shape("body", { box = { 1, 1, 1 }, pos = { 0, 0.5, 0 } })
return m
)LUA";

// The studio's level: daylight, a ground to stand on, and the model at the origin.
const char* kLevel = R"JSON({
  "version": 1,
  "watch_scripts": true,
  "editor_view": "models",
  "name": "model_studio",
  "_note": "Written by the editor's model studio. One prefab straight from a recipe; edit the recipe, not this.",
  "_studio_args": %3,
  "player": { "position": [0, 1.2, -8], "collider": { "shape": "capsule", "radius": 0.3, "halfHeight": 0.6 }, "friction": 0.5 },
  "environment": { "skyColor": [0.55, 0.68, 0.86] },
  "lighting": { "ambientMultiplier": 0.7, "exposure": 0.7,
    "sun": { "castsShadow": true, "color": [1, 0.98, 0.94], "direction": [0.4, 0.72, -0.32], "intensity": 6.0 } },
  "entities": [
    { "id": 1, "name": "ground", "shape": "box", "position": [0, -0.2, 0], "size": [40, 0.4, 40],
      "material": { "albedo": [0.5, 0.5, 0.48], "metallic": 0.0, "roughness": 1.0, "opacity": 1.0, "emission": [0, 0, 0], "checkerboard": true },
      "physics": { "motion": "static", "friction": 0.9, "restitution": 0.0 } },
    { "id": 2, "name": "model", "shape": "prefab", "prefab": { "recipe": "%1"%2 }, "position": [0, 0, 0] }
  ]
}
)JSON";

}  // namespace

namespace {

// The script pane: a plain text edit with line numbers down its left side (an error names a line; so does "what
// made this").
class CodePane : public QPlainTextEdit {
public:
    explicit CodePane(QWidget* parent) : QPlainTextEdit(parent), gutter_(new Gutter(this)) {
        QObject::connect(this, &QPlainTextEdit::blockCountChanged, [this](int) { layoutGutter(); });
        QObject::connect(this, &QPlainTextEdit::updateRequest, [this](const QRect& rect, int dy) {
            if (dy != 0) gutter_->scroll(0, dy);
            else gutter_->update(0, rect.y(), gutter_->width(), rect.height());
        });
        layoutGutter();
    }

protected:
    void resizeEvent(QResizeEvent* e) override {
        QPlainTextEdit::resizeEvent(e);
        layoutGutter();
    }

private:
    class Gutter : public QWidget {
    public:
        explicit Gutter(CodePane* pane) : QWidget(pane), pane_(pane) {}
    protected:
        void paintEvent(QPaintEvent* e) override { pane_->paintGutter(e); }
    private:
        CodePane* pane_;
    };
    int gutterWidth() const {
        int digits = 1;
        for (int n = std::max(1, blockCount()); n >= 10; n /= 10) ++digits;
        return 10 + fontMetrics().horizontalAdvance(QLatin1Char('9')) * std::max(3, digits);
    }
    void layoutGutter() {
        setViewportMargins(gutterWidth(), 0, 0, 0);
        const QRect r = contentsRect();
        gutter_->setGeometry(r.left(), r.top(), gutterWidth(), r.height());
    }
    void paintGutter(QPaintEvent* e) {
        QPainter painter(gutter_);
        painter.fillRect(e->rect(), palette().color(QPalette::AlternateBase));
        painter.setPen(palette().color(QPalette::PlaceholderText));
        painter.setFont(font());
        QTextBlock block = firstVisibleBlock();
        int top = qRound(blockBoundingGeometry(block).translated(contentOffset()).top());
        const int current = textCursor().blockNumber();
        while (block.isValid() && top <= e->rect().bottom()) {
            const int height = qRound(blockBoundingRect(block).height());
            if (block.isVisible() && top + height >= e->rect().top()) {
                painter.setPen(palette().color(block.blockNumber() == current ? QPalette::Text : QPalette::PlaceholderText));
                painter.drawText(0, top, gutter_->width() - 6, fontMetrics().height(), Qt::AlignRight, QString::number(block.blockNumber() + 1));
            }
            block = block.next();
            top += height;
        }
    }
    Gutter* gutter_;
};

// Lua, coloured: comments, strings, numbers, the language's own words, and the toolkit's tables (mesh, poly, model...).
class LuaColours : public QSyntaxHighlighter {
public:
    explicit LuaColours(QTextDocument* doc) : QSyntaxHighlighter(doc) {
        keyword_.setForeground(QColor("#c586c0"));
        toolkit_.setForeground(QColor("#4ec9b0"));
        number_.setForeground(QColor("#b5cea8"));
        string_.setForeground(QColor("#ce9178"));
        comment_.setForeground(QColor("#6a9955"));
    }

protected:
    void highlightBlock(const QString& text) override {
        static const QRegularExpression words("\\b(and|break|do|else|elseif|end|false|for|function|goto|if|in|local|nil|not|or|repeat|return|then|true|until|while)\\b");
        static const QRegularExpression tables("\\b(mesh|poly|model|material|profile|path|rng|tex|sdf|terrain|building|furniture|stylized|real|streetfurniture|args)\\b");
        static const QRegularExpression numbers("\\b\\d+(\\.\\d+)?([eE][-+]?\\d+)?\\b");
        auto each = [&](const QRegularExpression& re, const QTextCharFormat& format) {
            QRegularExpressionMatchIterator it = re.globalMatch(text);
            while (it.hasNext()) {
                const QRegularExpressionMatch m = it.next();
                setFormat(static_cast<int>(m.capturedStart()), static_cast<int>(m.capturedLength()), format);
            }
        };
        each(numbers, number_);
        each(words, keyword_);
        each(tables, toolkit_);
        // strings and a comment to the line's end, in the order they come (a -- inside a string is not a comment)
        for (int i = 0; i < text.size(); ++i) {
            const QChar c = text[i];
            if (c == '"' || c == '\'') {
                int j = i + 1;
                while (j < text.size() && text[j] != c) j += text[j] == '\\' ? 2 : 1;
                setFormat(i, std::min<int>(j, text.size() - 1) - i + 1, string_);
                i = j;
            } else if (c == '-' && i + 1 < text.size() && text[i + 1] == '-') {
                setFormat(i, text.size() - i, comment_);
                break;
            }
        }
    }

private:
    QTextCharFormat keyword_, toolkit_, number_, string_, comment_;
};

// A line added to a list script (models.lua, prefabs.lua) just before the closing brace of the table it returns.
// Returns what happened, for the status: empty when it was added.
QString addListLine(const QString& file, const QString& name, const QString& line) {
    QFile f(file);
    if (!f.open(QIODevice::ReadOnly | QIODevice::Text)) return "cannot read " + file;
    QString text = QString::fromUtf8(f.readAll());
    f.close();
    if (QRegularExpression(QString("^\\s*%1\\s*=").arg(QRegularExpression::escape(name)), QRegularExpression::MultilineOption).match(text).hasMatch())
        return QString();   // already there: left as it is
    const int brace = text.lastIndexOf(QRegularExpression("^\\}\\s*$", QRegularExpression::MultilineOption));
    if (brace < 0) return file + " does not end with the closing } of its table on a line of its own";
    text.insert(brace, line + "\n");
    if (!f.open(QIODevice::WriteOnly | QIODevice::Text | QIODevice::Truncate)) return "cannot write " + file;
    f.write(text.toUtf8());
    return QString();
}

}  // namespace

ModelStudioPanel::ModelStudioPanel(QWidget* parent) : QWidget(parent) {
    auto* column = new QVBoxLayout(this);
    column->setContentsMargins(4, 4, 4, 4);
    auto* row = new QHBoxLayout();
    auto* newButton = new QPushButton("New...", this);
    auto* openButton = new QPushButton("Open...", this);
    auto* saveButton = new QPushButton("Save", this);
    auto* reloadButton = new QPushButton("Reload", this);
    saveButton->setToolTip("Write the script (Ctrl+S in the pane). The model is rebuilt in the viewport.");
    reloadButton->setToolTip("Take the script from disk again, dropping what is in the pane.");
    auto* copyButton = new QPushButton("Copy view", this);
    copyButton->setToolTip("Copy what you are looking at (the script, the piece last clicked and its line, the camera, a screenshot's path)\nto paste into a message.");
    auto* listButton = new QPushButton("Add to lists", this);
    listButton->setToolTip("Give this script its line in models.lua and prefabs.lua: the engine can then build it by name and a level can place it.");
    file_ = new QLabel("no script open", this);
    row->addWidget(newButton);
    row->addWidget(openButton);
    row->addWidget(saveButton);
    row->addWidget(reloadButton);
    row->addWidget(copyButton);
    row->addWidget(listButton);
    column->addLayout(row);
    column->addWidget(file_);
    auto* argsRow = new QHBoxLayout();
    argsRow->addWidget(new QLabel("Arguments", this));
    args_ = new QLineEdit(this);
    args_->setFont(QFontDatabase::systemFont(QFontDatabase::FixedFont));
    args_->setPlaceholderText("none.  A Lua table the script reads as `args`:  { kind = \"tree\", seed = 3 }   (Enter rebuilds)");
    args_->setToolTip("What the script is run with, as a recipe's `args`. Enter applies them and rebuilds the model.\nKept with this script for the next time it is opened here.");
    argsRow->addWidget(args_, 1);
    column->addLayout(argsRow);
    QObject::connect(args_, &QLineEdit::returnPressed, [this]() { setArguments(args_->text()); });
    // the form: a control for each argument the script says it takes (filled by refreshForm after a build)
    form_ = new QWidget(this);
    formRows_ = new QFormLayout(form_);
    formRows_->setContentsMargins(0, 0, 0, 0);
    formRows_->setLabelAlignment(Qt::AlignRight);
    // (in a scrolling strip of its own, so a script with many arguments does not push the script out of sight)
    auto* formScroll = new QScrollArea(this);
    formScroll->setWidget(form_);
    formScroll->setWidgetResizable(true);
    formScroll->setFrameShape(QFrame::NoFrame);
    formScroll->setMaximumHeight(118);
    formScroll->hide();
    form_->setProperty("strip", QVariant::fromValue<QObject*>(formScroll));
    column->addWidget(formScroll);
    QObject::connect(copyButton, &QPushButton::clicked, [this]() { copyView(); });
    QObject::connect(listButton, &QPushButton::clicked, [this]() { addToLists(); });

    edit_ = new CodePane(this);
    new LuaColours(edit_->document());
    QFont mono = QFontDatabase::systemFont(QFontDatabase::FixedFont);
    mono.setPointSize(10);
    edit_->setFont(mono);
    edit_->setLineWrapMode(QPlainTextEdit::NoWrap);
    edit_->setTabStopDistance(edit_->fontMetrics().horizontalAdvance(' ') * 2);
    edit_->setPlaceholderText("Open a model script (a .lua under assets/scripts), or start a new one.");
    // the script above, the model's tree below, the bar between them draggable
    tree_ = new QTreeWidget(this);
    tree_->setColumnCount(3);
    tree_->setHeaderLabels({"Model", "Placed / what it is", "Line"});
    tree_->header()->setStretchLastSection(false);
    tree_->header()->setSectionResizeMode(0, QHeaderView::ResizeToContents);
    tree_->header()->setSectionResizeMode(1, QHeaderView::Stretch);
    tree_->header()->setSectionResizeMode(2, QHeaderView::ResizeToContents);
    tree_->setToolTip("The model's nodes, each under its parent, and what hangs on each. Click a row to go to the script line that made it;\n"
                      "a part is also selected in the viewport. A node that can move has a box to move it.");
    auto* split = new QSplitter(Qt::Vertical, this);
    split->addWidget(edit_);
    split->addWidget(tree_);
    split->setStretchFactor(0, 3);
    split->setStretchFactor(1, 2);
    column->addWidget(split, 1);
    QObject::connect(tree_, &QTreeWidget::itemClicked, [this](QTreeWidgetItem* item, int) {
        const int line = item->data(0, Qt::UserRole).toInt();
        const int piece = item->data(0, Qt::UserRole + 1).toInt();
        if (line > 0) gotoLine(line);
        if (piece >= 0 && selectPiece) selectPiece(piece);
        pickedLine_ = line;
        pickedWhat_ = item->text(0) + ": " + item->text(1);
    });

    status_ = new QLabel(this);
    status_->setWordWrap(true);
    status_->setTextInteractionFlags(Qt::TextSelectableByMouse);
    column->addWidget(status_);
    numbers_ = new QLabel(this);
    numbers_->setWordWrap(true);
    numbers_->setTextInteractionFlags(Qt::TextSelectableByMouse);
    numbers_->setStyleSheet("color: palette(placeholder-text);");
    column->addWidget(numbers_);

    watcher_ = new QFileSystemWatcher(this);
    QObject::connect(newButton, &QPushButton::clicked, [this]() { newScript(); });
    QObject::connect(openButton, &QPushButton::clicked, [this]() {
        const QString chosen = QFileDialog::getOpenFileName(this, "Open a model script", kScripts, "Lua scripts (*.lua)");
        if (!chosen.isEmpty()) openScript(chosen);
    });
    QObject::connect(saveButton, &QPushButton::clicked, [this]() { save(); });
    QObject::connect(reloadButton, &QPushButton::clicked, [this]() {
        if (!path_.isEmpty()) loadText();
    });
    auto* saveKey = new QShortcut(QKeySequence::Save, edit_);
    saveKey->setContext(Qt::WidgetShortcut);     // only while the pane has the keyboard: elsewhere Ctrl+S saves the level
    QObject::connect(saveKey, &QShortcut::activated, [this]() { save(); });
    QObject::connect(edit_, &QPlainTextEdit::cursorPositionChanged, [this]() {
        if (path_.isEmpty() || status_->property("bad").toBool()) return;
        const QTextCursor c = edit_->textCursor();
        setStatus(QString("line %1, column %2").arg(c.blockNumber() + 1).arg(c.positionInBlock() + 1), false);
    });
    QObject::connect(edit_->document(), &QTextDocument::modificationChanged, [this](bool changed) {
        if (path_.isEmpty()) return;
        file_->setText(QDir::current().relativeFilePath(path_) + (changed ? "  (not saved)" : ""));
    });
    // The file changed on disk: someone else wrote it (an assistant, another editor). Unedited, the pane follows;
    // edited, it keeps the user's text and says so.
    QObject::connect(watcher_, &QFileSystemWatcher::fileChanged, [this](const QString& changed) {
        if (changed != path_) return;
        if (!watcher_->files().contains(path_) && QFile::exists(path_)) watcher_->addPath(path_);   // (a save by rename drops the watch)
        QFile f(path_);
        if (!f.open(QIODevice::ReadOnly | QIODevice::Text)) return;
        const QString onDisk = QString::fromUtf8(f.readAll());
        if (onDisk == edit_->toPlainText()) return;                 // our own save
        if (edit_->document()->isModified()) {
            changedOnDisk_ = true;
            setStatus("The script changed on disk while you were editing. Save keeps yours; Reload takes the other.", true);
            return;
        }
        loadText();
    });
}

QString ModelStudioPanel::recipeName() const { return QFileInfo(path_).completeBaseName(); }
QString ModelStudioPanel::levelPath() const { return QString("%1/%2.json").arg(kLevels, recipeName()); }
bool ModelStudioPanel::dirty() const { return !path_.isEmpty() && edit_->document()->isModified(); }

void ModelStudioPanel::setStatus(const QString& text, bool bad) {
    status_->setProperty("bad", bad);
    status_->setStyleSheet(bad ? "color: #d05040;" : "");
    status_->setText(text);
}

void ModelStudioPanel::gotoLine(int line) {
    const QTextBlock block = edit_->document()->findBlockByNumber(line - 1);
    if (!block.isValid()) return;
    QTextCursor c(block);
    c.select(QTextCursor::LineUnderCursor);
    edit_->setTextCursor(c);
    edit_->centerCursor();
}

void ModelStudioPanel::loadText() {
    QFile f(path_);
    if (!f.open(QIODevice::ReadOnly | QIODevice::Text)) {
        setStatus("Cannot read " + path_, true);
        return;
    }
    const QString text = QString::fromUtf8(f.readAll());
    // keep the place: another hand may have changed a line far from where the user is reading
    const int line = edit_->textCursor().blockNumber(), column = edit_->textCursor().positionInBlock();
    loading_ = true;
    edit_->setPlainText(text);
    edit_->document()->setModified(false);
    loading_ = false;
    changedOnDisk_ = false;
    const QTextBlock block = edit_->document()->findBlockByNumber(line);
    if (block.isValid()) {
        QTextCursor c(block);
        c.movePosition(QTextCursor::Right, QTextCursor::MoveAnchor, std::min(column, static_cast<int>(block.length()) - 1));
        edit_->setTextCursor(c);
    }
    file_->setText(QDir::current().relativeFilePath(path_));
}

bool ModelStudioPanel::writeLevel() {
    // the arguments as the level carries them
    QString json, why;
    const QString typed = args_->text().trimmed();
    if (!typed.isEmpty()) {
        if (!argsToJson || !argsToJson(typed, json, why)) {
            setStatus("The arguments are not a Lua table: " + (why.isEmpty() ? QString("{ name = value, ... }") : why), true);
            return false;
        }
    }
    // (the typed text is kept beside them, as a JSON string, to put back in the box next time)
    QString kept = QString::fromUtf8(QJsonDocument(QJsonArray{typed}).toJson(QJsonDocument::Compact));
    kept = kept.mid(1, kept.size() - 2);
    QDir().mkpath(kLevels);
    QFile f(levelPath());
    if (!f.open(QIODevice::WriteOnly | QIODevice::Text | QIODevice::Truncate)) {
        setStatus("Cannot write " + levelPath(), true);
        return false;
    }
    f.write(QString(kLevel).arg(recipeName(), json.isEmpty() ? QString() : ", \"args\": " + json, kept).toUtf8());
    argsJson_ = json;
    return true;
}

bool ModelStudioPanel::setArguments(const QString& luaTable) {
    if (path_.isEmpty()) { setStatus("Open a script first.", true); return false; }
    args_->setText(luaTable);
    if (!writeLevel()) return false;
    // (the level's own file is watched: writing it rebuilds the model with the new arguments, the camera left alone)
    setStatus("Rebuilding " + recipeName() + " with those arguments ...", false);
    return true;
}

void ModelStudioPanel::openScript(const QString& chosen) {
    if (dirty()) {
        const QMessageBox::StandardButton choice = QMessageBox::warning(
            this, "Save the script?", "The script in the pane has changes that are not saved.", QMessageBox::Save | QMessageBox::Discard | QMessageBox::Cancel,
            QMessageBox::Save);
        if (choice == QMessageBox::Cancel) return;
        if (choice == QMessageBox::Save && !save()) return;
    }
    const QString abs = QFileInfo(chosen).absoluteFilePath();
    // A recipe is found by its name under assets/scripts (or the level's own folder): one outside would open in the
    // pane and build nothing.
    const QString scripts = QFileInfo(kScripts).absoluteFilePath();
    if (QFileInfo(abs).absolutePath() != scripts) {
        setStatus("A model script lives in " + QString(kScripts) + " (the engine finds a recipe by its name there).", true);
        return;
    }
    if (!path_.isEmpty()) watcher_->removePath(path_);
    path_ = abs;
    watcher_->addPath(path_);
    edit_->setTextCursor(QTextCursor(edit_->document()));
    loadText();
    // the arguments this script was last run with here, from its studio level
    args_->clear();
    {
        QFile was(levelPath());
        if (was.open(QIODevice::ReadOnly | QIODevice::Text)) args_->setText(QJsonDocument::fromJson(was.readAll()).object().value("_studio_args").toString());
    }
    if (!writeLevel()) { args_->clear(); writeLevel(); }
    setStatus("Building " + recipeName() + " ...", false);
    if (openLevel) openLevel(levelPath().toStdString());
}

void ModelStudioPanel::newScript() {
    bool ok = false;
    const QString name = QInputDialog::getText(this, "New model", "Its name (letters, digits and _; it becomes assets/scripts/<name>.lua):", QLineEdit::Normal,
                                               "my_model", &ok).trimmed();
    if (!ok || name.isEmpty()) return;
    if (!QRegularExpression("^[A-Za-z_][A-Za-z0-9_]*$").match(name).hasMatch()) {
        setStatus("A model's name is letters, digits and _, and does not start with a digit.", true);
        return;
    }
    const QString file = QString("%1/%2.lua").arg(kScripts, name);
    if (QFile::exists(file)) {
        setStatus(file + " is already there: Open it, or choose another name.", true);
        return;
    }
    QFile f(file);
    if (!f.open(QIODevice::WriteOnly | QIODevice::Text)) {
        setStatus("Cannot write " + file, true);
        return;
    }
    f.write(QString(kStarter).arg(name).toUtf8());
    f.close();
    openScript(file);
}

bool ModelStudioPanel::save() {
    if (path_.isEmpty()) return false;
    QFile f(path_);
    if (!f.open(QIODevice::WriteOnly | QIODevice::Text | QIODevice::Truncate)) {
        setStatus("Cannot write " + path_, true);
        return false;
    }
    f.write(edit_->toPlainText().toUtf8());
    f.close();
    edit_->document()->setModified(false);
    changedOnDisk_ = false;
    if (!watcher_->files().contains(path_)) watcher_->addPath(path_);
    setStatus("Saved. Rebuilding " + recipeName() + " ...", false);
    return true;
}

void ModelStudioPanel::showPiece(int line, const QString& what) {
    if (path_.isEmpty()) return;
    pickedLine_ = line;
    pickedWhat_ = what;
    if (line > 0) {
        gotoLine(line);
        setStatus(QString("line %1 made this: %2").arg(line).arg(what) + (dirty() ? "  (the pane has unsaved edits: the line may have moved)" : ""), false);
    } else {
        setStatus("This is " + what + " (no script line is recorded for it).", false);
    }
}

void ModelStudioPanel::refreshForm() {
    const QString json = recipeParams && !path_.isEmpty() ? recipeParams(recipeName()) : QString();
    const QJsonArray declared = QJsonDocument::fromJson(json.toUtf8()).array();
    const QJsonObject now = QJsonDocument::fromJson(argsJson_.toUtf8()).object();
    // the same arguments as before: only put the controls where the arguments now in force say (the user may have
    // typed in the box); a different set: build the form again
    if (json != formJson_) {
        formJson_ = json;
        while (formRows_->rowCount() > 0) formRows_->removeRow(0);
        // the choices first (they are usually what decides the rest), then the others, each lot by name
        QList<QJsonObject> ordered;
        for (const QJsonValue& v : declared) if (v.toObject().value("type").toString() == "choice") ordered.push_back(v.toObject());
        for (const QJsonValue& v : declared) if (v.toObject().value("type").toString() != "choice") ordered.push_back(v.toObject());
        for (const QJsonObject& d : ordered) {
            const QString name = d.value("name").toString(), type = d.value("type").toString();
            QWidget* control = nullptr;
            if (type == "choice") {
                auto* box = new QComboBox(form_);
                for (const QJsonValue& c : d.value("choices").toArray()) box->addItem(c.toString());
                QObject::connect(box, &QComboBox::currentTextChanged, [this](const QString&) { formChanged(); });
                control = box;
            } else if (type == "number") {
                auto* box = new QDoubleSpinBox(form_);
                const bool whole = d.value("whole").toBool();
                box->setDecimals(whole ? 0 : 3);
                box->setSingleStep(whole ? 1.0 : 0.1);
                if (d.contains("min") && d.contains("max")) box->setRange(d.value("min").toDouble(), d.value("max").toDouble());
                else box->setRange(-1e9, 1e9);
                box->setKeyboardTracking(false);
                QObject::connect(box, &QDoubleSpinBox::valueChanged, [this](double) { formChanged(); });
                control = box;
            } else if (type == "bool") {
                auto* box = new QCheckBox(form_);
                QObject::connect(box, &QCheckBox::toggled, [this](bool) { formChanged(); });
                control = box;
            } else {
                auto* box = new QLineEdit(form_);
                QObject::connect(box, &QLineEdit::editingFinished, [this]() { formChanged(); });
                control = box;
            }
            control->setObjectName(name);
            control->setProperty("type", type);
            control->setProperty("default", d.value("default").toVariant());
            control->setProperty("whole", d.value("whole").toBool());
            formRows_->addRow(name, control);
        }
        if (QWidget* strip = qobject_cast<QWidget*>(form_->property("strip").value<QObject*>())) strip->setVisible(!declared.isEmpty());
    }
    // each control at the value in force: the argument given, or the default
    loading_ = true;
    for (int row = 0; row < formRows_->rowCount(); ++row) {
        QLayoutItem* item = formRows_->itemAt(row, QFormLayout::FieldRole);
        QWidget* control = item != nullptr ? item->widget() : nullptr;
        if (control == nullptr) continue;
        const QString name = control->objectName();
        const QVariant value = now.contains(name) ? now.value(name).toVariant() : control->property("default");
        if (auto* combo = qobject_cast<QComboBox*>(control)) combo->setCurrentText(value.toString());
        else if (auto* number = qobject_cast<QDoubleSpinBox*>(control)) number->setValue(value.toDouble());
        else if (auto* check = qobject_cast<QCheckBox*>(control)) check->setChecked(value.toBool());
        else if (auto* line = qobject_cast<QLineEdit*>(control)) line->setText(value.toString());
    }
    loading_ = false;
}

void ModelStudioPanel::formChanged() {
    if (loading_ || path_.isEmpty()) return;
    // the arguments as a Lua table: only what differs from its default (the script's defaults say the rest)
    QStringList parts;
    for (int row = 0; row < formRows_->rowCount(); ++row) {
        QLayoutItem* item = formRows_->itemAt(row, QFormLayout::FieldRole);
        QWidget* control = item != nullptr ? item->widget() : nullptr;
        if (control == nullptr) continue;
        const QString name = control->objectName();
        const QVariant def = control->property("default");
        auto quoted = [](QString text) { return "\"" + text.replace("\\", "\\\\").replace("\"", "\\\"") + "\""; };
        if (auto* combo = qobject_cast<QComboBox*>(control)) {
            if (combo->currentText() != def.toString()) parts << name + " = " + quoted(combo->currentText());
        } else if (auto* number = qobject_cast<QDoubleSpinBox*>(control)) {
            if (std::abs(number->value() - def.toDouble()) > 1e-9)
                parts << name + " = " + (control->property("whole").toBool() ? QString::number(static_cast<qlonglong>(number->value())) : QString::number(number->value(), 'g', 10));
        } else if (auto* check = qobject_cast<QCheckBox*>(control)) {
            if (check->isChecked() != def.toBool()) parts << name + " = " + (check->isChecked() ? "true" : "false");
        } else if (auto* line = qobject_cast<QLineEdit*>(control)) {
            if (line->text() != def.toString()) parts << name + " = " + quoted(line->text());
        }
    }
    const QString text = parts.isEmpty() ? QString() : "{ " + parts.join(", ") + " }";
    if (text == args_->text().trimmed()) return;
    setArguments(text);
}

void ModelStudioPanel::refreshTree() {
    tree_->clear();
    if (!modelTree) return;
    std::vector<QTreeWidgetItem*> parents;     // the last row at each depth
    for (const StudioRow& row : modelTree()) {
        QTreeWidgetItem* parent = row.depth > 0 && row.depth - 1 < static_cast<int>(parents.size()) ? parents[static_cast<std::size_t>(row.depth - 1)] : nullptr;
        QTreeWidgetItem* item = parent != nullptr ? new QTreeWidgetItem(parent) : new QTreeWidgetItem(tree_);
        item->setText(0, row.kind == "node" ? row.name : row.kind + "  " + row.name);
        item->setText(1, row.detail);
        item->setText(2, row.line > 0 ? QString::number(row.line) : QString());
        item->setData(0, Qt::UserRole, row.line);
        item->setData(0, Qt::UserRole + 1, row.piece);
        if (row.kind == "node") {
            QFont bold = item->font(0);
            bold.setBold(true);
            item->setFont(0, bold);
        }
        parents.resize(static_cast<std::size_t>(row.depth) + 1);
        parents[static_cast<std::size_t>(row.depth)] = item;
        // a node that can move: a box to move it (degrees for a turn, metres for a slide), held to its limits
        if (!row.joint.isEmpty()) {
            auto* box = new QDoubleSpinBox(tree_);
            const bool limited = row.hi > row.lo;
            box->setRange(limited ? row.lo : -100000.0, limited ? row.hi : 100000.0);
            box->setDecimals(row.turn ? 1 : 3);
            box->setSingleStep(row.turn ? 5.0 : 0.05);
            box->setSuffix(row.turn ? " deg" : " m");
            box->setValue(row.value);
            box->setKeyboardTracking(false);
            const QString joint = row.joint;
            QObject::connect(box, &QDoubleSpinBox::valueChanged, [this, joint](double value) {
                if (setJoint) setJoint(joint, value);
            });
            tree_->setItemWidget(item, 2, box);
        }
    }
    tree_->expandAll();
}

void ModelStudioPanel::copyView() {
    if (path_.isEmpty()) { setStatus("Open a script first.", true); return; }
    QString text = "Model studio: " + QDir::current().relativeFilePath(path_) + (dirty() ? " (the pane has edits not yet saved)" : "") + "\n";
    if (pickedLine_ > 0) {
        text += QString("Picked: line %1: %2\n").arg(pickedLine_).arg(pickedWhat_);
        const QTextBlock block = edit_->document()->findBlockByNumber(pickedLine_ - 1);
        if (block.isValid()) text += "    > " + block.text().trimmed() + "\n";
    } else if (!pickedWhat_.isEmpty()) {
        text += "Picked: " + pickedWhat_ + "\n";
    } else {
        text += "Nothing picked (click a piece of the model to name it).\n";
    }
    if (!args_->text().trimmed().isEmpty()) text += "Arguments: " + args_->text().trimmed() + "\n";
    const QTextCursor c = edit_->textCursor();
    text += QString("Cursor: line %1\n").arg(c.blockNumber() + 1);
    if (status_->property("bad").toBool()) text += "Error shown: " + status_->text().section('\n', 0, 0) + "\n";
    if (!numbers_->text().isEmpty()) text += "Model: " + numbers_->text() + "\n";
    if (viewNote) text += viewNote();
    QApplication::clipboard()->setText(text);
    setStatus("Copied what you are looking at. Paste it into your message.", false);
}

void ModelStudioPanel::addToLists() {
    if (path_.isEmpty()) { setStatus("Open a script first.", true); return; }
    if (status_->property("bad").toBool()) { setStatus("Mend the script first: a model that does not build should not be on the lists.", true); return; }
    const QString name = recipeName();
    const QString models = QString("%1/models.lua").arg(kScripts), prefabs = QString("%1/prefabs.lua").arg(kScripts);
    const QString typed = args_->text().trimmed();
    QString why = addListLine(models, name, typed.isEmpty() ? QString("  %1 = { recipe = \"%1\" },").arg(name)
                                                             : QString("  %1 = { recipe = \"%1\", args = %2 },").arg(name, typed));
    if (why.isEmpty()) why = addListLine(prefabs, name, QString("  %1 = { model = \"%1\", physics = { body = \"static\" } },").arg(name));
    if (!why.isEmpty()) { setStatus("Not added: " + why, true); return; }
    setStatus(QString("'%1' is in models.lua and prefabs.lua. A level places it with { \"shape\": \"prefab\", \"prefab\": \"%1\", \"position\": [x, y, z] }.").arg(name), false);
}

void ModelStudioPanel::noteLog(bool isError, const QString& line) {
    if (path_.isEmpty()) return;
    // The loader's line when this level's model cannot be built:  prefab entity 'model': prefab '<recipe>': ...
    if (isError && line.contains("prefab entity 'model'")) {
        QString why = line.mid(line.indexOf("prefab entity 'model'") + 23).trimmed();
        // a Lua error names its line as  [string "..."]:LINE:  or  <file>.lua:LINE:
        int errorLine = 0;
        QRegularExpressionMatch m = QRegularExpression(QString("%1\\.lua: \\[string [^\\]]*\\]:(\\d+):").arg(QRegularExpression::escape(recipeName()))).match(why);
        if (m.hasMatch()) errorLine = m.captured(1).toInt();
        why.replace(QRegularExpression("\\[string \"[^\\]]*\\]:(\\d+):"), "line \\1:");
        // (said once: "prefab 'x': the model 'recipe:x': x.lua: line 3: ..." is "line 3: ...")
        const int own = why.indexOf(recipeName() + ".lua: ");
        if (own >= 0) why = why.mid(own + recipeName().size() + 6);
        const bool old = why.endsWith("(showing the last one that built)");
        if (old) why = why.left(why.size() - 33).trimmed() + "\nThe viewport shows the model as it last built.";
        setStatus(why, true);
        refreshForm();      // (a script that failed on a wrong argument still said what it takes)
        if (errorLine > 0 && !dirty()) gotoLine(errorLine);
        return;
    }
    // ...and when the level has loaded (again): built, unless an error for this load came first.
    if (!isError && line.contains("Loaded level:") && line.contains(recipeName() + ".json")) {
        if (!status_->property("bad").toBool() || status_->text().startsWith("Saved") || status_->text().startsWith("Building"))
            setStatus("Built " + recipeName() + ".", false);
        if (modelNumbers) numbers_->setText(modelNumbers());
        refreshTree();
        refreshForm();
        return;
    }
    if (!isError && line.contains("watch_scripts: change detected")) {
        if (!changedOnDisk_) setStatus("Rebuilding " + recipeName() + " ...", false);
    }
}
