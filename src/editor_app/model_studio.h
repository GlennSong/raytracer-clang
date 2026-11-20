#pragma once

// THE MODEL STUDIO (#191; Glenn, 2026-11-20: "interactively build a Lua script right there in that tool and see the
// output in a 3-D panel next to it"). A dock with a script pane: open or start a model recipe (a Lua script under
// assets/scripts), and the viewport beside it shows that script's model, stamped as a prefab on a ground plane in
// the real engine. Saving the script rebuilds the model in place; so does anything else that changes the file (an
// assistant editing it, an editor outside), and so does an edit to a library the script requires.
//
// How: the panel writes a small level (cache/model_studio/<name>.json: light, ground, and one prefab entity that
// names the recipe directly) and asks the editor to open it. The level sets watch_scripts, so the engine's own
// script watcher does the rebuilding; the panel adds no second mechanism. A script error is shown under the pane
// with its line, and the cursor is put on that line.
//
// Pure Qt and Q_OBJECT-free like the rest of the editor shell: editor_main gives it `openLevel` and feeds it the
// engine's log lines.

#include <QString>
#include <QWidget>
#include <functional>
#include <string>
#include <vector>

class QFileSystemWatcher;
class QFormLayout;
class QLabel;
class QLineEdit;
class QPlainTextEdit;
class QTreeWidget;

// One row of the studio's tree of the model (editor_main fills these from the engine's model; the panel only shows
// them). `depth` nests it: a node under its parent, and what hangs on a node under the node.
struct StudioRow {
    int depth = 0;
    QString kind;          // "node", "part", "marker", "shape", "use"
    QString name;          // the node's or marker's or shape's name; what a part is made of
    QString detail;        // where it was placed and what it is, in words
    int line = 0;          // the script line that made it (0: not known)
    int piece = -1;        // for a part: which drawn piece it is, to select it in the viewport (-1: none)
    // a node that can move: its name as a pose names it, how (a turn in degrees, or a slide in metres), its limits
    // (lo == hi: none) and where it is now
    QString joint;
    bool turn = true;
    double lo = 0, hi = 0, value = 0;
};

class ModelStudioPanel : public QWidget {
public:
    explicit ModelStudioPanel(QWidget* parent = nullptr);

    // Open a level by its path (editor_main's openScene).
    std::function<void(const std::string&)> openLevel;
    // Where the viewport's camera is and a screenshot just asked for, as lines of text (editor_main knows both).
    std::function<QString()> viewNote;
    // A Lua table as JSON (editor_main asks the engine): the Arguments box's text into what the level carries.
    // Returns false with `error` when the text is not a table.
    std::function<bool(const QString& lua, QString& json, QString& error)> argsToJson;
    // ARGUMENTS: run the script with these (a Lua table as text, as a recipe's `args`), and rebuild. Kept with the
    // script's studio level, so they are there the next time the script is opened.
    bool setArguments(const QString& luaTable);
    // WHAT THE SCRIPT TAKES: its `params{ ... }` as JSON (editor_main asks the engine: recipeParams), for the
    // form. `recipe` is the script's name. Empty: it declared nothing, and the Arguments box is all there is.
    std::function<QString(const QString& recipe)> recipeParams;
    void refreshForm();
    // THE TREE OF THE MODEL: every node under its parent with where it was placed, and under each node what hangs on
    // it. editor_main gives the rows, selects a part's piece in the viewport, and moves a moving node.
    std::function<std::vector<StudioRow>()> modelTree;
    std::function<void(int piece)> selectPiece;
    std::function<void(const QString& joint, double value)> setJoint;
    void refreshTree();
    // The model in the viewport in numbers, one line: its size, triangles, pieces, shapes (editor_main reads the world).
    std::function<QString()> modelNumbers;

    // Show the script at `path` (a .lua under assets/scripts): its text in the pane, its model in the viewport.
    void openScript(const QString& path);
    // A new script from the starter, named by the user, then opened.
    void newScript();
    // Write the pane's text to its file. The engine sees the change and rebuilds.
    bool save();
    // One line of the engine's log (from the console's drain): the studio reads its own model's errors and its
    // rebuilds out of it.
    void noteLog(bool isError, const QString& line);
    // WHAT MADE THIS: something of the model was clicked in the viewport. `line` is the script line that put it
    // there (0: not known), `what` says what it is. The pane goes to the line.
    void showPiece(int line, const QString& what);
    // COPY WHAT I AM LOOKING AT: the script, what was last picked (its line and what it is), where the camera is
    // and a screenshot's path, onto the clipboard, to paste into a message to whoever is editing the script.
    void copyView();
    // SAVE AS A MODEL AND A PREFAB: give the script its line in models.lua and in prefabs.lua, so the engine can
    // build it by name and a level can place it. Says what it did (or why not) under the pane.
    void addToLists();
    // Is a script open with changes not yet written?
    bool dirty() const;
    QString scriptPath() const { return path_; }

private:
    void loadText();                 // the file's text into the pane
    bool writeLevel();               // the studio level for this script, with the Arguments box's arguments
    QString levelPath() const;
    QString recipeName() const;      // the script's name as `require` names it
    void setStatus(const QString& text, bool bad);
    void gotoLine(int line);

    QPlainTextEdit* edit_ = nullptr;
    QLabel* file_ = nullptr;
    QLineEdit* args_ = nullptr;      // the recipe's arguments, a Lua table
    QWidget* form_ = nullptr;        // ...and, when the script says what it takes, a control for each
    QFormLayout* formRows_ = nullptr;
    QString formJson_;               // what the form was built from
    QString argsJson_;               // the arguments now in force, as the level carries them
    void formChanged();              // a control moved: the arguments from the form, applied
    QLabel* status_ = nullptr;
    QFileSystemWatcher* watcher_ = nullptr;
    QString path_;
    QTreeWidget* tree_ = nullptr;    // the model's nodes and what hangs on them
    QLabel* numbers_ = nullptr;      // the model in numbers, under the status
    int pickedLine_ = 0;             // what was last picked in the viewport
    QString pickedWhat_;
    bool loading_ = false;           // the pane is being filled: not the user's edit
    bool changedOnDisk_ = false;     // the file changed while the pane had unsaved edits
};
