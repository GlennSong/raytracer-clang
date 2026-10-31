#include "settings.h"
#include <nlohmann/json.hpp>
#include <fstream>
#include <sstream>

using json = nlohmann::json;

namespace engine {
namespace {

// Never saved: the keys the bus retired (ADR-0253), so a settings.json written before it loses them -- a stale
// `teleport.request` in one fired on the next play start.
const char* const kNotSaved[] = {
    // retired: Play Here is the play factory's fromHere argument now
    "playFromHere",
    // retired: commands that are typed requests now
    "teleport.open", "clouds.apply", "jitter.request", "lamps.lights", "citysim.master", "citysim.agents",
    "citysim.cones", "citysim.nav", "citysim.plan",
    "possess.cmd", "teleport.request", "cameraApply", "camera.setDetached", "player.setThirdPerson", "player.limp",
    "player.limpVelocity", "player.rideRequest", "vehicle.spawn", "vehicle.spawnRecipe", "interact.request",
    "elevator.call", "elevator.pick", "train.board", "walkers.knockNearest", "pin.request", "findplace.request",
    "ground.query", "who.query", "input.tap", "input.hold", "weather.set", "daynight.set", "daynight.setPaused",
    "daynight.setMinutes", "daynight.setDay", "daynight.setTerrain", "daynight.setHud", "daynight.setMoon",
    "daynight.chart",
    // retired: answers the owning systems give themselves now
    "teleport.result", "possess.status", "look.status", "agent.status", "daynight.status", "weather.status",
    "citysim.hour", "vehicle.telemetry", "player.telemetry", "interact.status", "elevator.status",
    "walkers.telemetry", "walkers.ragdolls", "walkers.flicker", "activities.telemetry", "speeders.telemetry",
    "parking.telemetry", "traffic.telemetry", "opensigns.telemetry", "trains.telemetry", "depots.telemetry",
    "depots.stalled", "jitter.result", "pin.status", "findplace.result", "who.result", "ground.result", "telemetry.askedAt",
};

}  // namespace

bool Settings::persists(const std::string& key) {
    for (const char* k : kNotSaved)
        if (key == k) return false;
    return true;
}

void Settings::load(const std::string& filepath) {
    std::ifstream file(filepath);
    if (!file.is_open()) return;

    json root;
    try {
        root = json::parse(file);
    } catch (const json::parse_error&) {
        return;
    }

    for (auto& [key, val] : root.items()) {
        if (!persists(key)) continue;   // a one-shot or a retired key left in an old file
        if (val.is_string())
            values[key] = val.get<std::string>();
        else if (val.is_boolean())
            values[key] = val.get<bool>() ? "true" : "false";
        else if (val.is_number())
            values[key] = std::to_string(val.get<double>());
    }
}

void Settings::save(const std::string& filepath) const {
    json root = json::object();
    for (const auto& [key, value] : values) {
        if (!persists(key)) continue;
        if (value == "true" || value == "false") {
            root[key] = (value == "true");
            continue;
        }
        bool isNumber = !value.empty();
        bool hasDot = false;
        for (size_t i = 0; i < value.size(); i++) {
            char c = value[i];
            if (c == '-' && i == 0) continue;
            if (c == '.' && !hasDot) { hasDot = true; continue; }
            if (!std::isdigit(c)) { isNumber = false; break; }
        }
        if (isNumber) {
            root[key] = std::stod(value);
        } else {
            root[key] = value;
        }
    }

    std::ofstream file(filepath);
    if (!file.is_open()) return;
    file << root.dump(2) << "\n";
}

double Settings::getDouble(const std::string& key, double defaultValue) const {
    auto it = values.find(key);
    if (it == values.end()) return defaultValue;
    try { return std::stod(it->second); }
    catch (...) { return defaultValue; }
}

void Settings::setDouble(const std::string& key, double value) {
    std::ostringstream ss;
    ss << value;
    values[key] = ss.str();
}

bool Settings::getBool(const std::string& key, bool defaultValue) const {
    auto it = values.find(key);
    if (it == values.end()) return defaultValue;
    return it->second == "true";
}

void Settings::setBool(const std::string& key, bool value) {
    values[key] = value ? "true" : "false";
}

std::string Settings::getString(const std::string& key, const std::string& defaultValue) const {
    auto it = values.find(key);
    if (it == values.end()) return defaultValue;
    return it->second;
}

void Settings::setString(const std::string& key, const std::string& value) {
    values[key] = value;
}

}  // namespace engine
