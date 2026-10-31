#ifndef RAYTRACER_SETTINGS_H
#define RAYTRACER_SETTINGS_H

#include <string>
#include <unordered_map>

namespace engine {

// The engine's preferences: a string map saved to settings.json at exit (window size, camera pose, render
// knobs, the day/night cycle...). Preferences only (ADR-0253): commands and answers between systems travel on the
// EventBus and the control channel's command table, not through here. The keys the bus retired are never saved,
// and are dropped from an old file when it loads.
class Settings {
public:
    // Is `key` saved? False for the retired keys.
    static bool persists(const std::string& key);

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


}  // namespace engine

#endif
