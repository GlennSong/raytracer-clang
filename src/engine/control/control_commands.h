#ifndef RAYTRACER_ENGINE_CONTROL_COMMANDS_H
#define RAYTRACER_ENGINE_CONTROL_COMMANDS_H

#include "control_channel.h"

#include <functional>
#include <map>
#include <memory>
#include <utility>
#include <string>
#include <vector>

namespace engine {

// The control channel's command table (ADR-0253): every socket verb is registered once, by name, with
// its usage line and a one-line summary. It replaces the one long if-chain in Application, where a verb
// could be handled twice without anyone knowing (`ride` was: the second handler never ran). A second
// add() of a name is refused and logged, so that cannot happen again.
//
// The table is also the one list of what the viewer can be told: `commands?` prints it, `commands? json`
// gives it to the MCP server, and Lua scripts can add their own verbs to it.
//
// Main thread only, like the channel's drain(): handlers run where Settings and the world are touchable.
struct ControlCommandInfo {
    std::string name;      // lowercased, as parseControlCommand gives it
    std::string usage;     // "teleport <x y z [pitch yaw] | x z>"
    std::string summary;   // one line: what it does
    std::string owner;     // who registered it: "engine", a system's name, or a Lua script
};

class ControlCommands {
public:
    using Handler = std::function<std::string(const ControlCommand& cmd)>;

    // Register a verb, lowercased (as incoming names are). Refused (false, logged) when the name is taken, so two
    // handlers can never share one, or when it is not one word.
    bool add(const std::string& name, const std::string& usage, const std::string& summary, Handler handler,
             const std::string& owner = "engine");
    // Remove a verb (a script unloading, a system shutting down). False when it was not there.
    bool remove(const std::string& name);
    // Remove every verb one owner registered (a Lua script reloading). Returns how many went.
    int removeOwnedBy(const std::string& owner);

    bool has(const std::string& name) const { return commands_.count(name) != 0; }
    std::size_t size() const { return commands_.size(); }
    // How many add() calls were refused for a taken name: a test holds the built-in set to zero.
    int refused() const { return refused_; }

    // A system's answer to a listed verb (ADR-0253): the engine lists the verb (usage, summary, a default reply for
    // when nothing answers it), and the system that owns what it asks about binds the real handler while it
    // runs. One binding per verb: a second is refused (an empty Binding, logged). Unbinds when destroyed, so a
    // system keeps its Bindings as members; the table must outlive them.
    class Binding {
    public:
        Binding() = default;
        Binding(ControlCommands* table, std::string name) : table_(table), name_(std::move(name)) {}
        ~Binding() { reset(); }
        Binding(Binding&& o) noexcept : table_(o.table_), name_(std::move(o.name_)) { o.table_ = nullptr; }
        Binding& operator=(Binding&& o) noexcept {
            if (this != &o) { reset(); table_ = o.table_; name_ = std::move(o.name_); o.table_ = nullptr; }
            return *this;
        }
        Binding(const Binding&) = delete;
        Binding& operator=(const Binding&) = delete;
        explicit operator bool() const { return table_ != nullptr; }
        void reset();

    private:
        ControlCommands* table_ = nullptr;
        std::string name_;
    };
    Binding bind(const std::string& name, const std::string& owner, Handler handler);
    // Who answers a verb now: its binder, or "" when the default reply stands.
    std::string boundBy(const std::string& name) const;

    // Run one parsed command: its handler's reply, or "err unknown command: <name> (try commands?)".
    std::string run(const ControlCommand& cmd) const;

    // Every verb, sorted by name.
    std::vector<ControlCommandInfo> list() const;
    // The listing as one line: `ok <n> commands: <usage>; <usage>; ...` (each usage starts with its verb).
    std::string listingText() const;
    // The listing as JSON for tools: `[{"name":..,"usage":..,"summary":..,"owner":..}, ...]`, on one line, or
    // pretty-printed with `indent` spaces (the site's commands.json). No "ok " prefix: `commands? json` adds it.
    std::string listingJson(int indent = -1) const;

private:
    struct Entry {
        ControlCommandInfo info;
        Handler handler;
        Handler bound;          // a system's answer, while it is bound (empty = `handler` stands)
        std::string boundBy;
    };
    void unbind(const std::string& name);
    std::map<std::string, Entry> commands_;
    int refused_ = 0;
};

// A system's answers to the channel's questions (ADR-0253): the system writes each answer as it measures it
// (set), and the question verb -- listed by the engine, bound here while the system runs -- replies "ok <latest>".
// It also knows when it was last asked, so a count too costly to keep every frame (the island's census) runs only
// while someone is asking.
class ControlAnswers {
public:
    ControlAnswers() = default;
    ControlAnswers(const ControlAnswers&) = delete;   // the bindings point back at this
    ControlAnswers& operator=(const ControlAnswers&) = delete;
    // Answer `verb` (it must be listed) with `initial` until the first set(). A null table binds nothing (a test).
    void bind(ControlCommands* table, const std::string& owner, const std::string& verb,
              const std::string& initial = "none");
    void set(const std::string& verb, std::string text);
    const std::string& get(const std::string& verb) const;
    // Seconds (steady clock) since any of these verbs was last asked; huge when never.
    double secondsSinceAsked() const;

private:
    struct Answer {
        std::string verb;
        std::string text;
        ControlCommands::Binding binding;
    };
    std::vector<std::unique_ptr<Answer>> answers_;   // stable addresses: each binding's handler points at its text
    double askedAt_ = -1e30;
};

}  // namespace engine

#endif
