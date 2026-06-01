#ifndef RAYTRACER_SETTINGS_H
#define RAYTRACER_SETTINGS_H

#include <string>
#include <unordered_map>

class Settings {
public:
    void load(const std::string& filepath);
    void save(const std::string& filepath) const;

    double getDouble(const std::string& key, double defaultValue) const;
    void setDouble(const std::string& key, double value);

    bool getBool(const std::string& key, bool defaultValue) const;
    void setBool(const std::string& key, bool value);

    std::string getString(const std::string& key, const std::string& defaultValue) const;
    void setString(const std::string& key, const std::string& value);

private:
    std::unordered_map<std::string, std::string> values;
};

#endif
