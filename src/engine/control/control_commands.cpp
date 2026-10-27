#include "control_commands.h"
#include "../../log.h"

#include <nlohmann/json.hpp>

namespace engine {

bool ControlCommands::add(const std::string& name, const std::string& usage, const std::string& summary,
                          Handler handler, const std::string& owner) {
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
    return it->second.handler(cmd);
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

std::string ControlCommands::listingJson() const {
    nlohmann::json arr = nlohmann::json::array();
    for (const auto& kv : commands_) {
        const ControlCommandInfo& c = kv.second.info;
        arr.push_back({{"name", c.name}, {"usage", c.usage}, {"summary", c.summary}, {"owner", c.owner}});
    }
    return "ok " + arr.dump();
}

}  // namespace engine
