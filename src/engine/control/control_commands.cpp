#include "control_commands.h"
#include "../../log.h"

#include <nlohmann/json.hpp>

#include <cctype>

namespace engine {

bool ControlCommands::add(const std::string& verb, const std::string& usage, const std::string& summary,
                          Handler handler, const std::string& owner) {
    // Incoming names are lowercased (parseControlCommand), so a verb is stored that way too, or it could be
    // listed and never called. A name with whitespace could never arrive as one word.
    std::string name = verb;
    for (char& c : name) c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
    if (name.empty() || name.find_first_of(" \t\r\n") != std::string::npos) {
        LOG_ERROR << "[control] refused verb \"" << verb << "\" from " << owner << ": a verb is one word";
        ++refused_;
        return false;
    }
    auto it = commands_.find(name);
    if (it != commands_.end()) {
        LOG_ERROR << "[control] `" << name << "` is already registered by " << it->second.info.owner
                  << "; refused the second one from " << owner;
        ++refused_;
        return false;
    }
    commands_[name] = Entry{{name, usage.empty() ? name : usage, summary, owner}, std::move(handler)};
    return true;
}

bool ControlCommands::remove(const std::string& name) { return commands_.erase(name) != 0; }

int ControlCommands::removeOwnedBy(const std::string& owner) {
    int n = 0;
    for (auto it = commands_.begin(); it != commands_.end();) {
        if (it->second.info.owner == owner) { it = commands_.erase(it); ++n; }
        else ++it;
    }
    return n;
}

std::string ControlCommands::run(const ControlCommand& cmd) const {
    auto it = commands_.find(cmd.name);
    if (it == commands_.end()) return "err unknown command: " + cmd.name + " (try commands?)";
    // A copy: a handler may remove verbs (a Lua script reloading drops its own), which would destroy the
    // std::function in the map while it runs.
    const Handler handler = it->second.handler;
    return handler(cmd);
}

std::vector<ControlCommandInfo> ControlCommands::list() const {
    std::vector<ControlCommandInfo> out;
    out.reserve(commands_.size());
    for (const auto& kv : commands_) out.push_back(kv.second.info);
    return out;
}

std::string ControlCommands::listingText() const {
    std::string out = "ok " + std::to_string(commands_.size()) + " commands:";
    for (const auto& kv : commands_) out += " " + kv.second.info.usage + ";";
    return out;
}

std::string ControlCommands::listingJson(int indent) const {
    nlohmann::json arr = nlohmann::json::array();
    for (const auto& kv : commands_) {
        const ControlCommandInfo& c = kv.second.info;
        arr.push_back({{"name", c.name}, {"usage", c.usage}, {"summary", c.summary}, {"owner", c.owner}});
    }
    return arr.dump(indent);
}

}  // namespace engine
