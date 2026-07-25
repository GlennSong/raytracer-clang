// Web entry point (ADR-0058). The browser build can't use Application::run()'s
// blocking `while (running()) runFrame()` loop — the page would hang and never
// yield to the event loop. Instead we run begin() once and hand runFrame() to
// emscripten_set_main_loop, which drives it from requestAnimationFrame, exactly
// the per-frame decomposition the editor host already uses (application.h).
//
// The WebGPU device is acquired by the backend itself (RequestAdapter/Device,
// awaited via ASYNCIFY) so Renderer::initialize() stays synchronous. Single-
// threaded: JobSystem runs in synchronous mode, so the build needs no -pthread /
// SharedArrayBuffer / cross-origin isolation.
//
// Starts in editor mode (views any level, no gameplay/pointer-lock) and reads
// the level from the URL: ?level=<name> loads assets/levels/<name>.json. Default
// is a self-contained procedural scene (no external glTF/HDR to 404).

#include "engine/application.h"
#include "engine/states/editor_state.h"
#include "game/arena_state.h"
#include "log.h"

#include <emscripten/emscripten.h>
#include <cstdlib>
#include <functional>
#include <memory>
#include <string>

using namespace engine;

namespace {

// File-scope so it outlives main() — the browser keeps calling frame() long
// after main() has returned control to the event loop.
Application g_app;

// State factories, file-scope so the rt_web_play hook can swap the running state
// long after main() has unwound. Editor = free look, no gameplay; Play =
// ArenaState (Jolt physics, gun, shooting). Set up in main().
std::function<std::unique_ptr<AppState>()> g_makePlay;
std::function<std::unique_ptr<AppState>()> g_makeEditor;
bool g_inPlay = false;

void frame() {
    g_app.runFrame();
}

// Resolve the level path from ?level=<name> (default: a self-contained scene).
std::string levelFromUrl() {
    char* c = static_cast<char*>(EM_ASM_PTR({
        var p = new URLSearchParams(location.search).get('level') || 'showcase';
        if (!p.endsWith('.json')) p += '.json';
        if (p.indexOf('/') < 0) p = 'assets/levels/' + p;
        return stringToNewUTF8(p);
    }));
    std::string path = c ? c : "assets/levels/showcase.json";
    std::free(c);
    return path;
}

}  // namespace

// Debug-panel hooks called from the page (web/index.html). Renderer toggles map
// to the same public flags the ImGui overlay drives on desktop; stats mirror the
// HUD. Camera fly/orbit toggle goes through the gamepad Back button (window.cpp).
extern "C" {

// Fly toggle: free != 0 → free 6-DOF flight; 0 → grounded FPS walk.
EMSCRIPTEN_KEEPALIVE void rt_web_camera(int free) {
    g_app.settings().setBool("cameraGrounded", free == 0);
}

// Pause/resume the simulation clock. The page pauses while the debug panel is
// open or the mouse isn't captured (not actively playing), so physics/gameplay
// freeze instead of running unattended. Rendering continues either way.
EMSCRIPTEN_KEEPALIVE void rt_web_pause(int paused) {
    g_app.simClock().setPaused(paused != 0);
}

EMSCRIPTEN_KEEPALIVE void rt_web_flag(int id, int val) {
    Renderer& r = g_app.renderer();
    switch (id) {
        case 0: r.debugView = val; break;        // 0=normal..8=cascades
        case 1: r.wireframe = val; break;        // 0=off,1=only,2=overlay
        case 2: r.ssaoEnabled = val != 0; break;
        case 3: r.ssrEnabled = val != 0; break;
        case 4: r.bloomEnabled = val != 0; break;
        case 5: r.lensEffectsEnabled = val != 0; break;
        case 6: r.showHud = val != 0; break;
        default: break;
    }
}

// Play/Edit toggle from the page. play != 0 → enter ArenaState (gameplay: the
// gun, shooting, Jolt-driven player); 0 → back to the free-look editor. Swaps
// the running state via the same deferred replace the editor toolbar uses.
EMSCRIPTEN_KEEPALIVE void rt_web_play(int play) {
    bool wantPlay = (play != 0);
    if (wantPlay == g_inPlay) return;
    g_inPlay = wantPlay;
    if (wantPlay && g_makePlay) {
        // Gameplay walks the player on the ground; force grounded FPS controls.
        g_app.settings().setBool("cameraGrounded", true);
        // Spawn the player at the editor's current view rather than the level's
        // document spawn (undefined / far above the showcase scene). The editor's
        // onStop persists flyEye* during this swap, just before ArenaState reads
        // it — so the capsule drops in where the camera is looking.
        g_app.settings().setBool("playFromHere", true);
        g_app.requestState(g_makePlay());
    } else if (g_makeEditor) {
        g_app.requestState(g_makeEditor());
    }
}

// Day/night + world controls from the page (the web build has no ImGui). These
// write settings keys that DayNightSystem re-reads each frame (editor + play).
// id: 0=timeOfDay [0,1] 1=speed (days/sec) 2=paused(0/1) 3=cycle enabled(0/1).
EMSCRIPTEN_KEEPALIVE void rt_web_env(int id, float v) {
    Settings& s = g_app.settings();
    switch (id) {
        case 0: s.setDouble("daynight.timeOfDay", v); break;
        case 1: s.setDouble("daynight.speed", v); break;
        case 2: s.setBool("daynight.paused", v != 0.0f); break;
        case 3: s.setBool("daynight.enabled", v != 0.0f); break;
        default: break;
    }
}

// Read back the live sun direction (0=x 1=y 2=z) so the page — and headless
// checks — can confirm time-of-day actually moves the sun.
EMSCRIPTEN_KEEPALIVE float rt_web_sun(int i) {
    const Vec3& d = g_app.renderView().lighting.sun.direction;
    return static_cast<float>(i == 0 ? d.x : i == 1 ? d.y : d.z);
}

// View-transform controls (the forward shader's output stage) + effect tuning.
// id: 0=tonemap op (0 ACES, 1 AgX) 1=exposure 2=contrast 3=saturation
// 4=postEffectScale (SSAO/SSR buffer resolution, 0.25–1.0). Exposure lives on
// the view's lighting (the renderer reads it each frame); the rest are live
// Renderer knobs.
EMSCRIPTEN_KEEPALIVE void rt_web_post(int id, float v) {
    Renderer& r = g_app.renderer();
    switch (id) {
        case 0: r.tonemapOperator = static_cast<int>(v); break;
        case 1: g_app.renderView().lighting.exposure = v; break;
        case 2: r.gradeParams.contrast = v; break;
        case 3: r.gradeParams.saturation = v; break;
        case 4: r.postEffectScale = v; break;
        default: break;
    }
}

// Living City debug layers from the page (the web build has no ImGui panel).
// One-shot writes: each lands in a settings key the citysim render bridge polls
// and CLEARS every frame, so the J/L keyboard toggles keep working between
// panel writes. id: 0=master widgets 1=agent rings 2=vision cones 3=nav graph
// 4=city plan (blocks + lots).
EMSCRIPTEN_KEEPALIVE void rt_web_city(int id, int val) {
    static const char* keys[] = {"citysim.master", "citysim.agents",
                                 "citysim.cones", "citysim.nav", "citysim.plan"};
    if (id >= 0 && id < 5)
        g_app.settings().setDouble(keys[id], val != 0 ? 1.0 : 0.0);
}

// Planet Lab controls from the page (the web has no ImGui, so its HTML panel
// drives PlanetLabSystem through settings). Each id writes one param key; the
// system reads them and rebuilds when rt_web_planet_generate bumps the counter.
// ids: 0 mode(0 rocky/1 gas) 1 seed 2 radius 3 faceRes 4 rockyPreset 5 gasPreset
// 6 seaLevel 7 hasOcean 8 continents 9 mountains 10 craters 11 relief 12 bands
// 13 turbulence 14 cloudDetail 15 storms 16 spin.
EMSCRIPTEN_KEEPALIVE void rt_web_planet(int id, float v) {
    static const char* keys[] = {
        "planetLab.mode", "planetLab.seed", "planetLab.radius", "planetLab.faceRes",
        "planetLab.rockyPreset", "planetLab.gasPreset", "planetLab.seaLevel",
        "planetLab.hasOcean", "planetLab.continentAmp", "planetLab.mountainAmp",
        "planetLab.craterAmp", "planetLab.relief", "planetLab.bandFreq",
        "planetLab.flowWarp", "planetLab.detailAmp", "planetLab.stormCount",
        "planetLab.spin"};
    if (id >= 0 && id < static_cast<int>(sizeof(keys) / sizeof(keys[0])))
        g_app.settings().setDouble(keys[id], static_cast<double>(v));
}

// Commit the current params: bump the counter PlanetLabSystem watches, so it
// regenerates + re-uploads the planet mesh on its next update.
EMSCRIPTEN_KEEPALIVE void rt_web_planet_generate(void) {
    Settings& s = g_app.settings();
    s.setDouble("planetLab.gen", s.getDouble("planetLab.gen", 0) + 1.0);
}

// Per-frame render stats for the panel readout. 0=drawCalls 1=instancedDraws
// 2=instances 3=triangles 4=entities.
EMSCRIPTEN_KEEPALIVE int rt_web_stat(int which) {
    RenderStats s = g_app.renderer().getRenderStats();
    switch (which) {
        case 0: return static_cast<int>(s.drawCalls);
        case 1: return static_cast<int>(s.instancedDrawCalls);
        case 2: return static_cast<int>(s.totalInstances);
        case 3: return static_cast<int>(s.trianglesDrawn);
        case 4: return static_cast<int>(s.entitiesSubmitted);
        default: return 0;
    }
}

}  // extern "C"

int main() {
    // Size the surface to the canvas's device-pixel size so the aspect ratio is
    // correct and the image isn't stretched (the WebGPU surface config drives the
    // canvas backing size). Fall back to the window if the canvas isn't laid out.
    int cw = EM_ASM_INT({
        return Math.max(1, Math.floor((document.getElementById('canvas').clientWidth
            || window.innerWidth) * (window.devicePixelRatio || 1)));
    });
    int ch = EM_ASM_INT({
        return Math.max(1, Math.floor((document.getElementById('canvas').clientHeight
            || window.innerHeight) * (window.devicePixelRatio || 1)));
    });
    if (!g_app.initialize({cw, ch, "Raytracer Engine (Web)", "settings.json"},
                          createPlatformWindow())) {
        LOG_ERROR << "Failed to initialize web application";
        return 1;
    }

    const std::string levelPath = levelFromUrl();
    LOG_INFO << "Web level: " << levelPath;

    // File-scope factories (see rt_web_play). Each captures the other so the
    // editor's "Play" and the arena's "Stop" both have a target to swap to.
    g_makeEditor = [levelPath]() -> std::unique_ptr<AppState> {
        return std::make_unique<EditorState>(g_app.windowRef(), g_app.renderer(),
                                             levelPath, g_makePlay);
    };
    g_makePlay = [levelPath]() -> std::unique_ptr<AppState> {
        return std::make_unique<ArenaState>(g_app.windowRef(), g_app.renderer(),
                                            levelPath, g_makeEditor);
    };

    // Fly camera + free-look so the on-screen sticks give FPS move+look out of
    // the box, but grounded (FPS walk: horizontal move, fixed height) by default.
    // The debug panel's Fly toggle clears cameraGrounded for free 6-DOF flight.
    g_app.settings().setString("cameraMode", "fly");
    g_app.settings().setBool("cameraFreeLook", true);
    g_app.settings().setBool("cameraGrounded", true);
    g_app.pushState(g_makeEditor());

    g_app.begin();

    // fps=0 → drive from requestAnimationFrame; simulate_infinite_loop=true so
    // main() unwinds cleanly while the loop keeps running (the standard
    // Emscripten pattern). Control returns to the browser here.
    emscripten_set_main_loop(frame, 0, true);
    return 0;
}
