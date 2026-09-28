#ifndef RAYTRACER_ENGINE_SCREENSHOT_H
#define RAYTRACER_ENGINE_SCREENSHOT_H

// IN-WORLD SCREENSHOTS (Glenn, 2026-09-24: "a screenshot button ... so I can take
// an in world screenshot without using the operating system's. The screenshots
// should be stored in a user's folder of choice ... but having a default is good
// too"). F12 (the "screenshot" action, rebindable with bind.screenshot) writes the
// next frame as a PNG through Renderer::requestFrameDump.
//
// The folder is the `screenshot.folder` setting (settings.json; the editor's
// File > Screenshot Folder... sets it). Unset or blank, it is the user's
// Pictures folder (XDG_PICTURES_DIR on Linux) + "/Raytracer".

#include <string>

namespace engine {

class Settings;

constexpr const char* kScreenshotFolderKey = "screenshot.folder";

// ~/Pictures/Raytracer, or the platform's equivalent.
std::string defaultScreenshotFolder();
// The setting, else the default.
std::string screenshotFolder(const Settings& settings);
// A fresh file in `folder` (created if missing), named for the local time:
// rt_2026-09-24_14-03-22.png, with _2, _3 ... for several in one second.
// Empty if the folder cannot be created.
std::string nextScreenshotPath(const std::string& folder);

}  // namespace engine

#endif
