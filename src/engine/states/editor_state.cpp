#include "editor_state.h"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <nlohmann/json.hpp>

#include "editor_home_view.h"
#include "../../renderer/window.h"
#include "../level_loader.h"
#include "../asset_manager.h"
#include "../systems/dev_control_system.h"
#include "../systems/camera_system.h"
#include "../systems/camera_panel_system.h"
#include "../systems/planet_lab_system.h"
#include "../systems/terrain_lod_system.h"
#include "../systems/residency_system.h"
#include "../systems/grass_system.h"
#include "../systems/render_system.h"
#include "../systems/light_system.h"
#include "../systems/probe_system.h"
#ifdef __EMSCRIPTEN__
#include "../systems/day_night_system.h"
#include "../systems/cloud_map_system.h"
#include "../systems/weather_sim_system.h"
#endif
#include "../../log.h"

namespace engine {

EditorState::EditorState(Window& window, Renderer& renderer,
                         std::string levelFile,
                         EditorSystem::PlayFactory makePlayState,
                         EditorBridge* bridge)
    : PlayingState(window), editorRenderer(renderer),
      levelFile(std::move(levelFile)) {
    addSystem<DevControlSystem>();   // Esc quits the app from the editor
    auto& camSys = addSystem<CameraSystem>();
    camera_ = &camSys;
#ifdef __EMSCRIPTEN__
    // The web editor has no ImGui day/night panel, so run the cycle here and
    // let the page's debug panel drive it through settings (rt_web_env). Native
    // editors keep a static sky and reach the cycle via play mode's ImGui.
    addSystem<DayNightSystem>();
    addSystem<WeatherSimSystem>();   // (ADR-0390)
    addSystem<CloudMapSystem>();   // (ADR-0386)
#endif
    addSystem<GrassSystem>();
    addSystem<ResidencySystem>();    // what is on the GPU, by where the camera is (ADR-0095)
    addSystem<TerrainLodSystem>();   // CDLOD terrain draws (ADR-0036)
    addSystem<LightSystem>();        // the street lamps (ADR-0362)
    addSystem<ProbeSystem>();        // the reflection probes (ADR-0374)
    addSystem<RenderSystem>();
    addSystem<CameraPanelSystem>(camSys);
    addSystem<PlanetLabSystem>();     // Debug > Planet Lab (RT_ENABLE_IMGUI)
    addSystem<EditorSystem>(camSys, this->levelFile, std::move(makePlayState),
                            bridge);
}

void EditorState::onEnter(FrameContext& ctx) {
    // The document is the only truth: drop whatever the previous mode left
    // (play-session spawns, physics aftermath) and load fresh.
    ctx.world.destroyAll();
    ctx.assets.clear();   // free the previous level's deduped GPU meshes

    // Editor camera: scene-view style — right-drag looks, WASD/QE fly,
    // scroll dollies, left click is selection. (Buttonless free-look is the
    // in-game freecam's behavior, not the editor's: it fights the mouse.)
    ctx.settings.setString("cameraMode", "fly");
    ctx.settings.setBool("cameraFreeLook", false);
    ctx.settings.setBool("cameraDetachEnabled", false);   // F = frame selected
    ctx.settings.setString("cameraStorePath", levelFile + ".cameras.json");

    // Editor startup / level switch: snap the fly camera to the HOME view (an
    // aerial framing of the origin) instead of wherever settings.json says the
    // last session wandered. Same-level re-entry — the Esc-out-of-a-playtest
    // round trip — keeps the pose (see EditorHomeViewPolicy). The write lands
    // BEFORE PlayingState::onEnter below, so CameraSystem::onStart seeds from
    // it like any other settings-driven pose.
    static EditorHomeViewPolicy sHomeView;   // process-lifetime; states aren't
    const bool freshLevel = sHomeView.shouldReset(levelFile);
    if (freshLevel) {
        ctx.settings.setDouble("flyEyeX", kEditorHomeEyeX);
        ctx.settings.setDouble("flyEyeY", kEditorHomeEyeY);
        ctx.settings.setDouble("flyEyeZ", kEditorHomeEyeZ);
        ctx.settings.setDouble("flyYaw", kEditorHomeYawDeg);
        ctx.settings.setDouble("flyPitch", kEditorHomePitchDeg);
        // A persisted top-down ortho would make the home view read as a map;
        // home is always the perspective framing.
        ctx.settings.setBool("flyOrtho", false);
    }
    // The control channel's `info` (and the camera panel's offline render)
    // read this; ArenaState already sets it, and the editor not setting it
    // left `info` reporting whatever level the LAST play session saved.
    ctx.settings.setString("levelPath", levelFile);

    reloadDocument(ctx);
    // A level that asks for it ("editor_view": "models": the model studio's) opens LOOKING AT ITS MODELS: the camera
    // stands back from everything its stamped prefabs draw, three-quarter on, the whole of it in view. Only when the
    // level is newly opened, so a rebuild after an edit leaves the camera where the user put it.
    if (freshLevel) framedRadius_ = frameModelsIfAsked(ctx, /*moveCameraNow=*/false);
    PlayingState::onEnter(ctx);
    // PlayingState captures the pointer for first-person play; the editor is
    // a pointing UI — picking, gizmos, panels — so the cursor stays visible.
    window.setCursorMode(CursorMode::Normal);
    LOG_INFO << "Edit mode: click selects, 1/2/3 move/rotate/scale, "
                "Play runs the level";
}

Real EditorState::frameModelsIfAsked(FrameContext& ctx, bool moveCameraNow, bool onlyIfChanged) {
    std::ifstream f(levelFile);
    if (!f) return 0;
    const nlohmann::json root = nlohmann::json::parse(f, nullptr, false);
    if (!root.is_object() || root.value("editor_view", std::string()) != "models") return 0;
    Vec3 lo(1e30, 1e30, 1e30), hi(-1e30, -1e30, -1e30);
    bool any = false;
    ctx.world.each<Transform, ModelPiece>([&](Entity e, Transform& t, ModelPiece& piece) {
        if (piece.from > 0) return;                       // the full level only
        const Renderable* r = ctx.world.get<Renderable>(e);
        if (r == nullptr) return;
        const BoundingSphere b = ctx.renderer.getMeshBounds(r->mesh);
        const Vec3 c = t.matrix().transformPoint(b.center);
        const Vec3 reach(b.radius, b.radius, b.radius);
        lo = Vec3(std::min(lo.x, c.x - reach.x), std::min(lo.y, c.y - reach.y), std::min(lo.z, c.z - reach.z));
        hi = Vec3(std::max(hi.x, c.x + reach.x), std::max(hi.y, c.y + reach.y), std::max(hi.z, c.z + reach.z));
        any = true;
    });
    // nothing built (a recipe with a mistake in it): stand where a small model would be seen from
    const Vec3 centre = any ? (lo + hi) * 0.5 : Vec3(0, 1, 0);
    const Real radius = any ? std::max<Real>(0.5, (hi - lo).length() * 0.5) : 2.0;
    // After a rebuild the camera is the user's, unless what is there now is nothing like what it was framed on:
    // a model where there was none (a mistake mended, arguments given), or one over twice or under half the size.
    const Real size = any ? radius : Real(0);
    if (onlyIfChanged) {
        const bool alike = framedRadius_ > 0 && size > 0 && size < framedRadius_ * 2.0 && size > framedRadius_ * 0.5;
        if (alike || size == 0) return framedRadius_;
    }
    const Vec3 from = normalize(Vec3(-0.55, 0.38, -1.0));   // over the left shoulder of something that faces +z
    const Vec3 eye = centre + from * (radius * 2.4);
    const Vec3 look = normalize(centre - eye);
    // (fly camera: forward = (sin yaw cos pitch, sin pitch, -cos yaw cos pitch))
    ctx.settings.setDouble("flyEyeX", eye.x);
    ctx.settings.setDouble("flyEyeY", std::max<Real>(eye.y, 0.3));
    ctx.settings.setDouble("flyEyeZ", eye.z);
    ctx.settings.setDouble("flyPitch", std::asin(std::clamp<Real>(look.y, -1, 1)) * 180.0 / 3.14159265358979323846);
    ctx.settings.setDouble("flyYaw", std::atan2(look.x, -look.z) * 180.0 / 3.14159265358979323846);
    if (moveCameraNow && camera_ != nullptr) {
        FlyCameraController& fly = camera_->flyController();
        fly.eye = Vec3(eye.x, std::max<Real>(eye.y, 0.3), eye.z);
        fly.pitch = ctx.settings.getDouble("flyPitch", 0.0);
        fly.yaw = ctx.settings.getDouble("flyYaw", 0.0);
    }
    return size;
}

void EditorState::reloadDocument(FrameContext& ctx) {
    if (!LevelLoader::load(levelFile, ctx.world, editorRenderer, ctx.view,
                           ctx.assets, /*editorMode=*/true)) {
        LOG_ERROR << "Failed to load level: " << levelFile;
    }
    // Seeded AFTER the load, from the files the load actually opened, so a
    // module reached through `require` is watched even though the level JSON
    // never names it.
    watch.collect(levelFile);
}

void EditorState::update(FrameContext& ctx) {
    PlayingState::update(ctx);
    if (!watch.tick()) return;
    // Rebuild in place rather than swapping states: the editor owns camera,
    // selection and panel state that a state swap would discard.
    LOG_INFO << "watch_scripts: change detected, rebuilding scene";
    ctx.world.destroyAll();
    ctx.assets.clear();
    reloadDocument(ctx);
    framedRadius_ = frameModelsIfAsked(ctx, /*moveCameraNow=*/true, /*onlyIfChanged=*/true);
}

void EditorState::onResume(FrameContext& ctx) {
    PlayingState::onResume(ctx);
    window.setCursorMode(CursorMode::Normal);   // see onEnter
}

}  // namespace engine
