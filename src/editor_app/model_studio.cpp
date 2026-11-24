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
  "_note": "Written by the editor's model studio. One prefab: a recipe run directly, or a line of prefabs.lua. Edit those, not this.",
  "_studio_args": %2,
  "player": { "position": [0, 1.2, -8], "collider": { "shape": "capsule", "radius": 0.3, "halfHeight": 0.6 }, "friction": 0.5 },
  "environment": { "skyColor": [0.55, 0.68, 0.86] },
  "lighting": { "ambientMultiplier": 0.7, "exposure": 0.7,
    "sun": { "castsShadow": true, "color": [1, 0.98, 0.94], "direction": [0.4, 0.72, -0.32], "intensity": 6.0 } },
  "entities": [
    { "id": 1, "name": "ground", "shape": "box", "position": [0, -0.2, 0], "size": [40, 0.4, 40],
      "material": { "albedo": [0.5, 0.5, 0.48], "metallic": 0.0, "roughness": 1.0, "opacity": 1.0, "emission": [0, 0, 0], "checkerboard": true },
      "physics": { "motion": "static", "friction": 0.9, "restitution": 0.0 } },
    { "id": 2, "name": "model", "shape": "prefab", "prefab": %1, "position": [0, 0, 0] }
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
    auto* listButton = new QPushButton("Make prefab", this);
    listButton->setToolTip("Give this script its line in models.lua and in prefabs.lua (the engine can then build it by name and a level can place it),\nand view that prefab.");
    auto* prefabButton = new QPushButton("Open prefab...", this);
    prefabButton->setToolTip("View a prefab from prefabs.lua: its model's script here, the prefab itself in the viewport, its settings in the Inspector.");
    file_ = new QLabel("no script open", this);
    row->addWidget(newButton);
    row->addWidget(openButton);
    row->addWidget(prefabButton);
    row->addWidget(saveButton);
    row->addWidget(reloadButton);
    row->addWidget(copyButton);
    row->addWidget(listButton);
    column->addLayout(row);
    column->addWidget(file_);
    // what the viewport shows: the script's model alone, or a prefab made of it
    auto* showRow = new QHBoxLayout();
    showRow->addWidget(new QLabel("Showing", this));
    showing_ = new QComboBox(this);
    showing_->setToolTip("The model alone: the script run directly (shape only).\nA prefab: a line of prefabs.lua that shows this script's model, as a level would place it,\nwith its physics and behaviour script. Its settings are in the Inspector.");
    showRow->addWidget(showing_, 1);
    showRow->addWidget(new QLabel("  Lines", this));
    const char* overlayNames[3] = {"shapes", "markers", "joints"};
    const char* overlayHints[3] = {"The collision shapes, as lines. Green: it stands still. Orange: it is on a node that moves.",
                                   "The markers: three short axes each (x red, y green, z blue; +z is the way it faces).",
                                   "The moving nodes: what each turns about or slides along, its limits (white) and where it is now (yellow)."};
    for (int i = 0; i < 3; ++i) {
        overlay_[i] = new QCheckBox(overlayNames[i], this);
        overlay_[i]->setToolTip(overlayHints[i]);
        showRow->addWidget(overlay_[i]);
        QObject::connect(overlay_[i], &QCheckBox::toggled, [this](bool) {
            if (!loading_ && setOverlay) setOverlay(overlay_[0]->isChecked(), overlay_[1]->isChecked(), overlay_[2]->isChecked());
        });
    }
    column->addLayout(showRow);
    QObject::connect(showing_, &QComboBox::activated, [this](int index) {
        const QString name = showing_->itemData(index).toString();
        if (name != prefab_) openScriptAs(path_, name);
    });
    QObject::connect(prefabButton, &QPushButton::clicked, [this]() {
        const QStringList names = prefabNames ? prefabNames() : QStringList();
        if (names.isEmpty()) { setStatus("prefabs.lua has no prefabs (or cannot be read).", true); return; }
        bool ok = false;
        const QString name = QInputDialog::getItem(this, "Open a prefab", "A prefab in prefabs.lua:", names, std::max(0, static_cast<int>(names.indexOf(prefab_))), false, &ok);
        if (ok && !name.isEmpty()) openPrefab(name);
    });
    // the prefab's form (it lives in the editor's Inspector dock: editor_main puts it there)
    prefabForm_ = new QWidget();
    {
        auto* box = new QVBoxLayout(prefabForm_);
        box->setContentsMargins(4, 4, 4, 8);
        prefabTitle_ = new QLabel(prefabForm_);
        QFont bold = prefabTitle_->font();
        bold.setBold(true);
        prefabTitle_->setFont(bold);
        prefabTitle_->setWordWrap(true);
        box->addWidget(prefabTitle_);
        auto* rows = new QWidget(prefabForm_);
        prefabRows_ = new QFormLayout(rows);
        prefabRows_->setContentsMargins(0, 0, 0, 0);
        box->addWidget(rows);
        prefabError_ = new QLabel(prefabForm_);
        prefabError_->setWordWrap(true);
        prefabError_->setStyleSheet("color: #d05040;");
        prefabError_->setTextInteractionFlags(Qt::TextSelectableByMouse);
        box->addWidget(prefabError_);
        auto* note = new QLabel("A change here rewrites this prefab's entry in prefabs.lua (comments inside the entry are not kept). The model's shape is the script's.", prefabForm_);
        note->setWordWrap(true);
        note->setStyleSheet("color: palette(placeholder-text);");
        box->addWidget(note);
        prefabForm_->hide();
    }
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
    tree_->setHeaderLabels({"Prefab / model", "Placed / what it is", "Line"});
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
        // a row of the prefab: its line is in prefabs.lua, not in the pane; its settings are in the Inspector
        if (item->data(0, Qt::UserRole + 2).toBool()) {
            if (showInspector) showInspector();
            pickedLine_ = 0;
            pickedWhat_ = "prefab " + prefab_ + ": " + item->text(0).trimmed() + " (" + item->text(1) + ")";
            return;
        }
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
QString ModelStudioPanel::levelPath() const {
    return prefab_.isEmpty() ? QString("%1/%2.json").arg(kLevels, recipeName()) : QString("%1/%2.prefab.json").arg(kLevels, prefab_);
}
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
    const QString typed = prefab_.isEmpty() ? args_->text().trimmed() : QString();    // (a prefab's model has its arguments in models.lua)
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
    const QString what = prefab_.isEmpty() ? QString("{ \"recipe\": \"%1\"%2 }").arg(recipeName(), json.isEmpty() ? QString() : ", \"args\": " + json)
                                           : "\"" + prefab_ + "\"";
    f.write(QString(kLevel).arg(what, kept).toUtf8());
    argsJson_ = json;
    return true;
}

bool ModelStudioPanel::setArguments(const QString& luaTable) {
    if (path_.isEmpty()) { setStatus("Open a script first.", true); return false; }
    if (!prefab_.isEmpty()) {
        setStatus("A prefab's model takes its arguments from its line in models.lua. Show the model alone to try others.", true);
        return false;
    }
    args_->setText(luaTable);
    if (!writeLevel()) return false;
    // (the level's own file is watched: writing it rebuilds the model with the new arguments, the camera left alone)
    setStatus("Rebuilding " + recipeName() + " with those arguments ...", false);
    return true;
}

void ModelStudioPanel::openScript(const QString& chosen) { openScriptAs(chosen, QString()); }

void ModelStudioPanel::openPrefab(const QString& name) {
    if (name.isEmpty()) {
        if (!path_.isEmpty()) openScriptAs(path_, QString());
        return;
    }
    const QJsonObject data = QJsonDocument::fromJson((prefabData ? prefabData(name) : QString()).toUtf8()).object();
    if (data.contains("error") || data.isEmpty()) {
        setStatus("The prefab '" + name + "' cannot be viewed: " + data.value("error").toString("nothing is known of it"), true);
        return;
    }
    const QString recipe = data.value("recipe").toString();
    if (recipe.isEmpty()) {
        setStatus("The prefab '" + name + "' shows the model '" + data.value("model").toString() + "', which is read from files: there is no script to show.", true);
        return;
    }
    openScriptAs(QString("%1/%2.lua").arg(kScripts, recipe), name);
}

void ModelStudioPanel::openScriptAs(const QString& chosen, const QString& prefab) {
    const QString abs = QFileInfo(chosen).absoluteFilePath();
    const bool sameScript = abs == path_;        // only what is shown changes: the pane and its edits stay
    if (!sameScript && dirty()) {
        const QMessageBox::StandardButton choice = QMessageBox::warning(
            this, "Save the script?", "The script in the pane has changes that are not saved.", QMessageBox::Save | QMessageBox::Discard | QMessageBox::Cancel,
            QMessageBox::Save);
        if (choice == QMessageBox::Cancel) return;
        if (choice == QMessageBox::Save && !save()) return;
    }
    // A recipe is found by its name under assets/scripts (or the level's own folder): one outside would open in the
    // pane and build nothing.
    const QString scripts = QFileInfo(kScripts).absoluteFilePath();
    if (QFileInfo(abs).absolutePath() != scripts || !QFile::exists(abs)) {
        setStatus(QFile::exists(abs) ? "A model script lives in " + QString(kScripts) + " (the engine finds a recipe by its name there)."
                                     : "There is no script " + QDir::current().relativeFilePath(abs) + ".", true);
        return;
    }
    if (!sameScript) {
        if (!path_.isEmpty()) watcher_->removePath(path_);
        path_ = abs;
        watcher_->addPath(path_);
        edit_->setTextCursor(QTextCursor(edit_->document()));
    }
    prefab_ = prefab;
    pickedLine_ = 0;            // what was picked belonged to what was shown before
    pickedWhat_.clear();
    numbers_->clear();
    if (!sameScript) loadText();
    // the arguments: this script's as last run here (from its studio level), or the prefab's model's (not to change here)
    args_->clear();
    args_->setEnabled(prefab_.isEmpty());
    if (prefab_.isEmpty()) {
        QFile was(levelPath());
        if (was.open(QIODevice::ReadOnly | QIODevice::Text)) args_->setText(QJsonDocument::fromJson(was.readAll()).object().value("_studio_args").toString());
    }
    if (!writeLevel()) { args_->clear(); writeLevel(); }
    if (!prefab_.isEmpty()) {
        const QJsonObject data = QJsonDocument::fromJson((prefabData ? prefabData(prefab_) : QString()).toUtf8()).object();
        const QJsonValue modelArgs = data.value("args");
        args_->setText(modelArgs.isObject() ? "models.lua: " + QString::fromUtf8(QJsonDocument(modelArgs.toObject()).toJson(QJsonDocument::Compact)) : QString("models.lua: none"));
    }
    refreshPrefab();
    setStatus("Building " + (prefab_.isEmpty() ? recipeName() : "the prefab " + prefab_) + " ...", false);
    if (openLevel) openLevel(levelPath().toStdString());
}

void ModelStudioPanel::refreshOverlayBoxes() {
    if (!getOverlay) return;
    bool on[3] = {false, false, false};
    getOverlay(on[0], on[1], on[2]);
    const bool was = loading_;
    loading_ = true;
    for (int i = 0; i < 3; ++i) overlay_[i]->setChecked(on[i]);
    loading_ = was;
}

void ModelStudioPanel::refreshPrefab() {
    if (path_.isEmpty()) return;
    // SHOWING: the model alone, then each prefab whose model is built by this script
    loading_ = true;
    showing_->clear();
    showing_->addItem("the model alone  (" + recipeName() + ".lua run directly)", QString());
    for (const QString& name : prefabNames ? prefabNames() : QStringList()) {
        const QJsonObject data = QJsonDocument::fromJson((prefabData ? prefabData(name) : QString()).toUtf8()).object();
        if (data.value("recipe").toString() != recipeName() && name != prefab_) continue;
        const QJsonValue modelArgs = data.value("args");
        showing_->addItem("prefab  " + name + "   (model " + data.value("model").toString() +
                              (modelArgs.isObject() ? " " + QString::fromUtf8(QJsonDocument(modelArgs.toObject()).toJson(QJsonDocument::Compact)) : QString()) + ")", name);
    }
    showing_->setCurrentIndex(std::max(0, showing_->findData(prefab_)));
    loading_ = false;

    // THE FORM: a control for each thing a prefab's line can say, at what this prefab's line says
    prefabForm_->setVisible(!prefab_.isEmpty());
    if (prefab_.isEmpty()) return;
    const QString fieldsJson = prefabFields ? prefabFields() : QString();
    if (fieldsJson != prefabFieldsJson_) {
        prefabFieldsJson_ = fieldsJson;
        while (prefabRows_->rowCount() > 0) prefabRows_->removeRow(0);
        for (const QJsonValue& v : QJsonDocument::fromJson(fieldsJson.toUtf8()).array()) {
            const QJsonObject d = v.toObject();
            const QString part = d.value("part").toString(), name = d.value("name").toString(), type = d.value("type").toString();
            const bool optional = d.value("optional").toBool();
            QWidget* control = nullptr;
            if (type == "choice") {
                auto* box = new QComboBox(prefabForm_);
                for (const QJsonValue& c : d.value("choices").toArray()) box->addItem(c.toString());
                QObject::connect(box, &QComboBox::activated, [this](int) { prefabFormChanged(); });
                control = box;
            } else if (type == "number") {
                // (one step under its least value is "not said": the key is left out of the line)
                auto* box = new QDoubleSpinBox(prefabForm_);
                const double least = d.value("min").toDouble(), step = d.value("step").toDouble(0.1);
                box->setDecimals(step < 1 ? 3 : 1);
                box->setSingleStep(step);
                box->setRange(optional ? least - step : least, 1e9);
                if (optional) box->setSpecialValueText("not set");
                box->setKeyboardTracking(false);
                QObject::connect(box, &QDoubleSpinBox::valueChanged, [this](double) { prefabFormChanged(); });
                control = box;
            } else {
                auto* box = new QLineEdit(prefabForm_);
                if (optional) box->setPlaceholderText("none");
                QObject::connect(box, &QLineEdit::editingFinished, [this]() { prefabFormChanged(); });
                control = box;
            }
            control->setProperty("part", part);
            control->setProperty("key", name);
            control->setToolTip(d.value("hint").toString());
            auto* label = new QLabel(name.isEmpty() ? part : part + " " + name, prefabForm_);
            label->setToolTip(d.value("hint").toString());
            prefabRows_->addRow(label, control);
        }
    }
    const QJsonObject data = QJsonDocument::fromJson((prefabData ? prefabData(prefab_) : QString()).toUtf8()).object();
    const QString list = QDir::current().relativeFilePath(data.value("list").toString());
    prefabTitle_->setText("Prefab  " + prefab_ + (data.value("line").toInt() > 0 ? QString("   (%1, line %2)").arg(list).arg(data.value("line").toInt()) : QString()));
    if (data.contains("error")) {
        // (its line is wrong, by another hand: the controls keep what they had, and the mistake is said)
        prefabError_->setText(data.value("error").toString());
        return;
    }
    prefabError_->clear();
    loading_ = true;
    for (int row = 0; row < prefabRows_->rowCount(); ++row) {
        QLayoutItem* item = prefabRows_->itemAt(row, QFormLayout::FieldRole);
        QWidget* control = item != nullptr ? item->widget() : nullptr;
        if (control == nullptr) continue;
        const QString part = control->property("part").toString(), key = control->property("key").toString();
        const QJsonValue value = key.isEmpty() ? data.value(part) : data.value(part).toObject().value(key);
        if (auto* combo = qobject_cast<QComboBox*>(control)) combo->setCurrentText(value.toString());
        else if (auto* number = qobject_cast<QDoubleSpinBox*>(control)) number->setValue(value.isDouble() ? value.toDouble() : number->minimum());
        else if (auto* line = qobject_cast<QLineEdit*>(control)) line->setText(value.toString());
    }
    loading_ = false;
}

QString ModelStudioPanel::setPrefabField(const QString& part, const QString& key, const QString& value) {
    if (prefab_.isEmpty()) return "no prefab is viewed (studio_prefab <name>)";
    for (int row = 0; row < prefabRows_->rowCount(); ++row) {
        QLayoutItem* item = prefabRows_->itemAt(row, QFormLayout::FieldRole);
        QWidget* control = item != nullptr ? item->widget() : nullptr;
        if (control == nullptr || control->property("part").toString() != part || control->property("key").toString() != key) continue;
        loading_ = true;
        QString why;
        if (auto* combo = qobject_cast<QComboBox*>(control)) {
            if (combo->findText(value) < 0) {
                QStringList choices;
                for (int i = 0; i < combo->count(); ++i) choices << combo->itemText(i);
                why = "it is one of: " + choices.join(", ");
            } else {
                combo->setCurrentText(value);
            }
        } else if (auto* number = qobject_cast<QDoubleSpinBox*>(control)) {
            bool ok = false;
            const double v = value.toDouble(&ok);
            if (value.isEmpty() && !number->specialValueText().isEmpty()) number->setValue(number->minimum());
            else if (ok) number->setValue(v);
            else why = "it is a number";
        } else if (auto* text = qobject_cast<QLineEdit*>(control)) {
            text->setText(value);
        }
        loading_ = false;
        if (!why.isEmpty()) return why;
        prefabFormChanged();
        return prefabError_->text();
    }
    return "a prefab has no `" + part + (key.isEmpty() ? QString() : " " + key) + "` (model; physics body, friction; draw class, distance; script)";
}

void ModelStudioPanel::prefabFormChanged() {
    if (loading_ || prefab_.isEmpty() || !savePrefab) return;
    // the line as the list would give it: model = ..., physics = { ... }, draw = { ... }, script = ...
    QJsonObject line;
    for (int row = 0; row < prefabRows_->rowCount(); ++row) {
        QLayoutItem* item = prefabRows_->itemAt(row, QFormLayout::FieldRole);
        QWidget* control = item != nullptr ? item->widget() : nullptr;
        if (control == nullptr) continue;
        const QString part = control->property("part").toString(), key = control->property("key").toString();
        QJsonValue value;
        if (auto* combo = qobject_cast<QComboBox*>(control)) value = combo->currentText();
        else if (auto* number = qobject_cast<QDoubleSpinBox*>(control)) { if (number->specialValueText().isEmpty() || number->value() > number->minimum()) value = number->value(); }
        else if (auto* text = qobject_cast<QLineEdit*>(control)) { if (!text->text().trimmed().isEmpty()) value = text->text().trimmed(); }
        if (value.isNull() || value.isUndefined()) continue;
        if (key.isEmpty()) { line[part] = value; continue; }
        QJsonObject table = line.value(part).toObject();
        table[key] = value;
        line[part] = table;
    }
    const QJsonObject before = QJsonDocument::fromJson((prefabData ? prefabData(prefab_) : QString()).toUtf8()).object();
    const QString why = savePrefab(prefab_, QString::fromUtf8(QJsonDocument(line).toJson(QJsonDocument::Compact)));
    if (!why.isEmpty()) {
        prefabError_->setText("Not written: " + why);
        return;
    }
    prefabError_->clear();
    setStatus("Wrote the prefab " + prefab_ + " to prefabs.lua. Placing it again ...", false);
    // another model: its recipe goes in the pane (the list's change places the prefab again either way)
    if (line.value("model").toString() != before.value("model").toString()) openPrefab(prefab_);
    else refreshPrefab();
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
    // (viewing a prefab, the arguments are its model's line's: nothing to set here)
    const QString json = recipeParams && !path_.isEmpty() && prefab_.isEmpty() ? recipeParams(recipeName()) : QString();
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
    if (loading_ || path_.isEmpty() || !prefab_.isEmpty()) return;
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
    // THE PREFAB'S OWN ROWS, when one is viewed: what a copy carries besides the model's shape
    if (!prefab_.isEmpty()) {
        const QJsonObject data = QJsonDocument::fromJson((prefabData ? prefabData(prefab_) : QString()).toUtf8()).object();
        const int line = data.value("line").toInt();
        auto* top = new QTreeWidgetItem(tree_);
        top->setText(0, "prefab  " + prefab_);
        top->setText(1, data.contains("error") ? data.value("error").toString() : QString("prefabs.lua: what a copy carries (the Inspector changes these)"));
        top->setText(2, line > 0 ? QString::number(line) : QString());
        QFont bold = top->font(0);
        bold.setBold(true);
        top->setFont(0, bold);
        top->setData(0, Qt::UserRole + 2, true);
        top->setData(0, Qt::UserRole + 1, -1);
        auto add = [&](const QString& what, const QString& detail) {
            auto* item = new QTreeWidgetItem(top);
            item->setText(0, what);
            item->setText(1, detail);
            item->setData(0, Qt::UserRole + 2, true);
            item->setData(0, Qt::UserRole + 1, -1);
        };
        if (!data.contains("error")) {
            const QJsonObject physics = data.value("physics").toObject(), draw = data.value("draw").toObject();
            const QJsonValue modelArgs = data.value("args");
            add("model  " + data.value("model").toString(),
                data.contains("recipe") ? "models.lua: " + data.value("recipe").toString() + ".lua" +
                                              (modelArgs.isObject() ? " with " + QString::fromUtf8(QJsonDocument(modelArgs.toObject()).toJson(QJsonDocument::Compact)) : QString())
                                        : QString("models.lua: read from files"));
            add("physics", physics.value("body").toString() == "static"
                               ? "solid: the model's collision shapes are bodies; friction " + (physics.contains("friction") ? QString::number(physics.value("friction").toDouble()) : QString("each shape's own"))
                               : QString("not solid"));
            add("draw", "class " + draw.value("class").toString() + ", " + (draw.contains("distance") ? QString("to %1 m").arg(draw.value("distance").toDouble()) : QString("to the class's distance")));
            add("script", data.value("script").toString().isEmpty() ? QString("none: nothing moves it") : data.value("script").toString() + ": the behaviour every copy runs");
        }
    }
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

QString ModelStudioPanel::describe() const {
    QJsonObject o;
    o["script"] = path_.isEmpty() ? QString() : QDir::current().relativeFilePath(path_);
    o["showing"] = prefab_.isEmpty() ? QString("model") : QString("prefab");
    if (!prefab_.isEmpty()) o["prefab"] = QJsonDocument::fromJson((prefabData ? prefabData(prefab_) : QString()).toUtf8()).object();
    o["lines"] = QString(overlay_[0]->isChecked() ? "shapes " : "") + (overlay_[1]->isChecked() ? "markers " : "") + (overlay_[2]->isChecked() ? "joints" : "");
    o["arguments"] = args_->text().trimmed();
    o["unsaved"] = dirty();
    o["status"] = status_->text();
    o["error"] = status_->property("bad").toBool();
    o["model"] = numbers_->text();
    o["cursor_line"] = edit_->textCursor().blockNumber() + 1;
    o["picked_line"] = pickedLine_;
    o["picked"] = pickedWhat_;
    if (pickedLine_ > 0) o["picked_text"] = edit_->document()->findBlockByNumber(pickedLine_ - 1).text().trimmed();
    return QString::fromUtf8(QJsonDocument(o).toJson(QJsonDocument::Compact));
}

void ModelStudioPanel::copyView() {
    if (path_.isEmpty()) { setStatus("Open a script first.", true); return; }
    QString text = "Model studio: " + QDir::current().relativeFilePath(path_) + (dirty() ? " (the pane has edits not yet saved)" : "") + "\n";
    if (!prefab_.isEmpty()) text += "Showing the prefab: " + prefab_ + " (prefabs.lua)\n";
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
    openPrefab(name);
    setStatus(QString("'%1' is in models.lua and prefabs.lua, and is what the viewport now shows. A level places it with { \"shape\": \"prefab\", \"prefab\": \"%1\", \"position\": [x, y, z] }.").arg(name), false);
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
    if (!isError && line.contains("Loaded level:") && line.contains("/" + QFileInfo(levelPath()).fileName())) {
        if (!status_->property("bad").toBool() || status_->text().startsWith("Saved") || status_->text().startsWith("Building"))
            setStatus(prefab_.isEmpty() ? "Built " + recipeName() + "." : "Built " + recipeName() + ", placed as the prefab " + prefab_ + ".", false);
        refreshPrefab();
        if (modelNumbers) numbers_->setText(modelNumbers());
        refreshTree();
        refreshForm();
        return;
    }
    if (!isError && line.contains("watch_scripts: change detected")) {
        if (!changedOnDisk_ && !status_->text().startsWith("Wrote the prefab")) setStatus("Rebuilding " + recipeName() + " ...", false);
    }
}
