#include "settings.h"
#include <nlohmann/json.hpp>
#include <fstream>
#include <sstream>

using json = nlohmann::json;

namespace engine {

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
