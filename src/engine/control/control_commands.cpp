#include "control_commands.h"
#include "../../log.h"

#include <nlohmann/json.hpp>

#include <cctype>
#include <chrono>

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
    std::map<std::string, Entry>::iterator it = commands_.find(name);
    if (it != commands_.end()) {
        LOG_ERROR << "[control] `" << name << "` is already registered by " << it->second.info.owner
                  << "; refused the second one from " << owner;
        ++refused_;
        return false;
    }
    commands_[name] = Entry{{name, usage.empty() ? name : usage, summary, owner}, std::move(handler), {}, {}};
    return true;
}

bool ControlCommands::remove(const std::string& name) { return commands_.erase(name) != 0; }

int ControlCommands::removeOwnedBy(const std::string& owner) {
    int n = 0;
    for (std::map<std::string, Entry>::iterator it = commands_.begin(); it != commands_.end();) {
        if (it->second.info.owner == owner) { it = commands_.erase(it); ++n; }
        else ++it;
    }
    return n;
}

std::string ControlCommands::run(const ControlCommand& cmd) const {
    std::map<std::string, Entry>::const_iterator it = commands_.find(cmd.name);
    if (it == commands_.end()) return "err unknown command: " + cmd.name + " (try commands?)";
    // A copy: a handler may remove verbs (a Lua script reloading drops its own), which would destroy the
    // std::function in the map while it runs.
    const Handler handler = it->second.bound ? it->second.bound : it->second.handler;
    return handler(cmd);
}

ControlCommands::Binding ControlCommands::bind(const std::string& name, const std::string& owner, Handler handler) {
    std::map<std::string, Entry>::iterator it = commands_.find(name);
    if (it == commands_.end()) {
        LOG_ERROR << "[control] " << owner << " cannot answer `" << name << "`: no such verb is listed";
        return {};
    }
    if (it->second.bound) {
        LOG_ERROR << "[control] `" << name << "` is already answered by " << it->second.boundBy << "; refused "
                  << owner;
        return {};
    }
    it->second.bound = std::move(handler);
    it->second.boundBy = owner;
    return Binding(this, name);
}

std::string ControlCommands::boundBy(const std::string& name) const {
    std::map<std::string, Entry>::const_iterator it = commands_.find(name);
    return it == commands_.end() ? std::string() : it->second.boundBy;
}

void ControlCommands::unbind(const std::string& name) {
    std::map<std::string, Entry>::iterator it = commands_.find(name);
    if (it == commands_.end()) return;
    it->second.bound = nullptr;
    it->second.boundBy.clear();
}

void ControlCommands::Binding::reset() {
    if (table_ != nullptr) table_->unbind(name_);
    table_ = nullptr;
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

namespace {
double steadySeconds() {
    return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
}
}  // namespace

void ControlAnswers::bind(ControlCommands* table, const std::string& owner, const std::string& verb,
                          const std::string& initial) {
    for (const std::unique_ptr<Answer>& a : answers_)
        if (a->verb == verb) return;   // bound already (a state entered twice)
    std::unique_ptr<Answer> a = std::make_unique<Answer>();
    a->verb = verb;
    a->text = initial;
    if (table != nullptr) {
        const Answer* answer = a.get();
        a->binding = table->bind(verb, owner, [this, answer](const ControlCommand&) {
            askedAt_ = steadySeconds();
            return "ok " + answer->text;
        });
    }
    answers_.push_back(std::move(a));
}

void ControlAnswers::set(const std::string& verb, std::string text) {
    for (const std::unique_ptr<Answer>& a : answers_)
        if (a->verb == verb) { a->text = std::move(text); return; }
}

const std::string& ControlAnswers::get(const std::string& verb) const {
    static const std::string kNone = "none";
    for (const std::unique_ptr<Answer>& a : answers_)
        if (a->verb == verb) return a->text;
    return kNone;
}

double ControlAnswers::secondsSinceAsked() const { return steadySeconds() - askedAt_; }

}  // namespace engine
