#ifndef RAYTRACER_ENGINE_SCRIPTING_SIGN_KIT_LUA_H
#define RAYTRACER_ENGINE_SCRIPTING_SIGN_KIT_LUA_H

// THE SIGN KIT'S CATALOGUE IN LUA (procgen/world/sign_kit.h). A catalogue file (the shipped one is
// assets/scripts/signs_us.lua, over the recipes in sign_kit.lua) returns a table with
//   names()              every sign's name, in the catalogue's order
//   examples()           requests that show every sign once or more (the gallery's first rows)
//   faces(request)       the faces a request needs, in order: the sign's own, then its plaques'. Each a SignFace as a
//                        table {w, h, background, elems, shine = {background, elems}, wear, seed, label}
//   model(request, slots)  the sign built: {label, info, parts = {{name, face = i | nil, mesh}, ...}}, its face
//                        parts' texture coordinates taken from slots[i] = {page, u0, v0, u1, v1, tab_v0}
//   wear_look, sheeting  tables of numbers (SignWearLook; the night-shine table)
// A REQUEST is a plain table: { sign = "speed_limit", value = 35, wear = 0.4, seed = 7, far = true, mount = "mast_arm",
// plaques = { { sign = "all_way" } }, ... } -- what each sign reads from it is the catalogue's business. Nothing in
// C++ names a sign.
//
// The catalogue runs with the mesh words (procgen_bindings.h), `require` (script_modules.h), and `sign_text`: the sign
// font's measure, so a layout can size a panel to its words (sign_text.width(text, cap_metres, x_scale) -> metres).

#include "../procgen/world/sign_kit.h"

#include <nlohmann/json.hpp>
#include <memory>
#include <string>
#include <vector>

namespace engine {

class Font;

class SignKit {
public:
    SignKit();
    ~SignKit();
    SignKit(const SignKit&) = delete;
    SignKit& operator=(const SignKit&) = delete;

    // Run catalogue `file` (resolved as any level script is). False, with `err`, when it is missing or fails.
    bool load(const std::string& file = "signs_us.lua", const std::string& levelDir = std::string(), std::string* err = nullptr);
    // ...or a catalogue's source text (its `require`s still resolve from `levelDir` and the script roots).
    bool loadSource(const std::string& source, const std::string& levelDir = std::string(), std::string* err = nullptr);

    std::vector<std::string> names();
    std::vector<nlohmann::json> examples();

    // Build every request: faces laid out, deduplicated, rasterized, worn and packed into pages (colour and shine), then
    // each model built on its faces' slots. False, with `err`, when the catalogue fails on a request.
    bool build(const std::vector<nlohmann::json>& requests, const Font& font, SignKitBuild& out, std::string* err = nullptr,
               const SignKitOptions& opt = SignKitOptions());

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

// A face as the catalogue writes it (a table read through lua_json) -> the SignFace the rasterizer draws.
SignFace signFaceFromJson(const nlohmann::json& j);

}  // namespace engine

#endif
