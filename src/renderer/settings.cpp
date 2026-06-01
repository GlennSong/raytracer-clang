#include "settings.h"
#include <fstream>
#include <sstream>
#include <algorithm>

static std::string trim(const std::string& s) {
    size_t start = s.find_first_not_of(" \t\n\r");
    if (start == std::string::npos) return "";
    size_t end = s.find_last_not_of(" \t\n\r");
    return s.substr(start, end - start + 1);
}

void Settings::load(const std::string& filepath) {
    std::ifstream file(filepath);
    if (!file.is_open()) return;

    std::string content((std::istreambuf_iterator<char>(file)),
                         std::istreambuf_iterator<char>());

    size_t pos = 0;
    while (pos < content.size()) {
        size_t keyStart = content.find('"', pos);
        if (keyStart == std::string::npos) break;
        size_t keyEnd = content.find('"', keyStart + 1);
        if (keyEnd == std::string::npos) break;

        std::string key = content.substr(keyStart + 1, keyEnd - keyStart - 1);

        size_t colon = content.find(':', keyEnd);
        if (colon == std::string::npos) break;

        size_t valStart = colon + 1;
        while (valStart < content.size() && content[valStart] == ' ') valStart++;

        std::string value;
        if (valStart < content.size() && content[valStart] == '"') {
            size_t valEnd = content.find('"', valStart + 1);
            if (valEnd == std::string::npos) break;
            value = content.substr(valStart + 1, valEnd - valStart - 1);
            pos = valEnd + 1;
        } else {
            size_t valEnd = content.find_first_of(",}\n", valStart);
            if (valEnd == std::string::npos) valEnd = content.size();
            value = trim(content.substr(valStart, valEnd - valStart));
            pos = valEnd + 1;
        }

        values[key] = value;
    }
}

void Settings::save(const std::string& filepath) const {
    std::ofstream file(filepath);
    if (!file.is_open()) return;

    file << "{\n";
    size_t count = 0;
    for (const auto& [key, value] : values) {
        file << "  \"" << key << "\": ";

        bool isNumber = !value.empty();
        bool hasDot = false;
        for (size_t i = 0; i < value.size(); i++) {
            char c = value[i];
            if (c == '-' && i == 0) continue;
            if (c == '.' && !hasDot) { hasDot = true; continue; }
            if (!std::isdigit(c)) { isNumber = false; break; }
        }

        bool isBool = (value == "true" || value == "false");

        if (isNumber || isBool) {
            file << value;
        } else {
            file << "\"" << value << "\"";
        }

        if (++count < values.size()) file << ",";
        file << "\n";
    }
    file << "}\n";
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
