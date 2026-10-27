#ifndef RAYTRACER_ENGINE_CONTROL_COMMANDS_H
#define RAYTRACER_ENGINE_CONTROL_COMMANDS_H

#include "control_channel.h"

#include <functional>
#include <map>
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

    // Register a verb. Refused (false, logged) when the name is taken, so two handlers can never share one.
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

    // Run one parsed command: its handler's reply, or "err unknown command: <name> (try commands?)".
    std::string run(const ControlCommand& cmd) const;

    // Every verb, sorted by name.
    std::vector<ControlCommandInfo> list() const;
    // The listing as one line: `ok <n> commands: name -- usage; ...`, or JSON for tools
    // (`ok [{"name":..,"usage":..,"summary":..,"owner":..}, ...]`).
    std::string listingText() const;
    std::string listingJson() const;

private:
    struct Entry {
        ControlCommandInfo info;
        Handler handler;
    };
    std::map<std::string, Entry> commands_;
    int refused_ = 0;
};

}  // namespace engine

#endif
