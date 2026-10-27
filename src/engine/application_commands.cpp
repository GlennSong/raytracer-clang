// The control channel's verbs (ADR-0078), registered in the command table (ADR-0253). Moved here verbatim
// from the if-chain that was Application::handleControlCommand; each verb now says its usage and what it
// does, which is what `commands?` lists and the MCP server reads.
#include "components.h"                 // CityMap (citymap)
#ifdef RT_ENABLE_IMGUI
#include <imgui.h>                       // clip/clip?: the debug UI clipboard from the socket
#endif
#include "procgen/city/city_svg.h"     // the layered city map
#include "procgen/city/building_records.h"   // `lot?`: what is actually here
#include "application.h"
#include "event_types.h"
#include "systems/camera_system.h"   // CameraPoseRequest, CameraDetachRequest
#include "systems/day_night_system.h"   // DayNightRequest, WeatherRequest
#include "systems/player_system.h"   // PersonRequest, LimpRequest
#include "systems/teleport_pose.h"   // TeleportRequest, TeleportPanelRequest
#include "systems/render_system.h"   // LampLightsRequest
#include "systems/vehicle_system.h"  // VehicleSpawnRequest
#include "systems/interact_broker_system.h"   // InteractRequest
#include "systems/elevator_system.h"   // ElevatorRequest
#include "control/city_requests.h"     // PossessRequest, RideRequest, CityLayerRequest, JitterRequest, ...
#include "systems/debug_overlay_system.h"   // static settings<->renderer mapping (control `render`)
#include "../log.h"
#include "engine/bundle/bundle.h"
#include "engine/bundle/bake.h"
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <map>

namespace engine {

namespace {

bool num(const std::string& s, double& out) {
    try { out = std::stod(s); return true; } catch (...) { return false; }
}

// A command verb hands its typed request to whichever system hears it (ADR-0253). With nobody listening (this
// state has no such system) it says so, instead of staging a request nothing will take.
template <typename E>
std::string publishRequest(EventBus& bus, const E& request, const std::string& verb, const std::string& ok) {
    if (bus.subscriberCount<E>() == 0) return "err " + verb + ": nothing here takes it (no system for it in this state)";
    bus.publish(request);
    return ok;
}

// The reply of a question no running system answers: the engine lists it, the system that owns the answer binds
// it while it runs (ControlCommands::bind).
ControlCommands::Handler unanswered(const std::string& who) {
    return [who](const ControlCommand& cmd) -> std::string {
        return "err " + cmd.name + ": nothing answers it here (it needs " + who + ")";
    };
}

}  // namespace

void Application::registerControlCommands() {
    // One function per family, the families of the site's list (tools/commands_page.py). As each family's
    // verbs move onto typed events (ADR-0253), they move into the systems that own them.
    registerConnectionVerbs();
    registerCameraVerbs();
    registerSkyVerbs();
    registerPlayerVerbs();
    registerPossessionVerbs();
    registerCityVerbs();
    registerGroundVerbs();
    registerEngineVerbs();
    registerEventVerbs();
}

// Connection
void Application::registerConnectionVerbs() {
    commandTable.add("commands?", "commands? [json]",
                     "Every command the viewer takes, with its usage; `json` adds the summaries, for tools.",
                     [this](const ControlCommand& cmd) -> std::string {
        if (!cmd.args.empty() && cmd.args[0] == "json") return "ok " + commandTable.listingJson();
        return commandTable.listingText();
    });

    commandTable.add("ping", "ping",
                     "Is the viewer answering? Replies pong.",
                     [](const ControlCommand&) -> std::string {
        return "ok pong";
    });

    commandTable.add("info", "info",
                     "The level, frame number, paused/overlay/hud/ui flags and the engine version.",
                     [this](const ControlCommand&) -> std::string {
        char buf[512];
        std::snprintf(buf, sizeof(buf),
                      "ok level=%s frame=%llu paused=%d overlay=%d hud=%d ui=%d engine=%s",
                      settingsStore.getString("levelPath", "?").c_str(),
                      static_cast<unsigned long long>(frameCounter),
                      clock.paused() ? 1 : 0, debugOverlayActive ? 1 : 0,
                      rendererPtr->showHud ? 1 : 0,
                      rendererPtr->uiHidden ? 1 : 0,
                      engine::bundle::engineIdentity().version.c_str());
        return buf;
    });
}

// Camera and pictures
void Application::registerCameraVerbs() {
    commandTable.add("camera", "camera <x> <y> <z> <pitchDeg> <yawDeg>",
                     "Put the free camera at a pose (detaches it from the player, even in play mode).",
                     [this](const ControlCommand& cmd) -> std::string {
        // A CameraPoseRequest; CameraSystem takes it next update, seeds the fly
        // pose, and detaches so the pose wins even in play mode.
        double v[5];
        if (cmd.args.size() < 5 || !num(cmd.args[0], v[0]) ||
            !num(cmd.args[1], v[1]) || !num(cmd.args[2], v[2]) ||
            !num(cmd.args[3], v[3]) || !num(cmd.args[4], v[4]))
            return "err usage: camera <x> <y> <z> <pitchDeg> <yawDeg>";
        CameraPoseRequest r;
        r.eye = Vec3(v[0], v[1], v[2]);
        r.pitch = v[3];
        r.yaw = v[4];
        return publishRequest(eventBus, r, "camera", "ok camera staged");
    });

    commandTable.add("camera?", "camera?",
                     "The camera's eye, pitch and yaw now, stamped with the frame number.",
                     [this](const ControlCommand&) -> std::string {
        const CameraState& c = view.camera;
        Vec3 fwd = c.target - c.position;
        const Real len = fwd.length();
        if (len > 1e-9) fwd = fwd * (1.0 / len);
        constexpr Real kRadToDeg = 57.29577951308232;
        char buf[256];
        // frame= stamps the reply with SERVER time: judder measurements divide
        // eye deltas by frame-count deltas, not by client wall clock — socket
        // round-trip jitter was masquerading as 39% camera noise.
        std::snprintf(buf, sizeof(buf),
                      "ok eye=%.2f,%.2f,%.2f pitch=%.2f yaw=%.2f frame=%llu",
                      c.position.x, c.position.y, c.position.z,
                      std::asin(std::clamp(fwd.y, Real(-1), Real(1))) * kRadToDeg,
                      std::atan2(fwd.x, -fwd.z) * kRadToDeg,
                      static_cast<unsigned long long>(frameCounter));
        return buf;
    });

    commandTable.add("shot", "shot <path.png>",
                     "Save the next composited frame as a PNG; poll for the file.",
                     [this](const ControlCommand& cmd) -> std::string {
        if (cmd.args.empty()) return "err usage: shot <path.png>";
        // Fires on the next composited frame; the caller polls for the file
        // (the MCP shim does). Replying "written" would stall the frame.
        return rendererPtr->requestFrameDump(cmd.args[0])
                   ? "ok armed " + cmd.args[0]
                   : "err this backend cannot capture frames";
    });

    commandTable.add("shots", "shots <prefix> <frames>",
                     "Save the next N frames as <prefix>_000.png, _001.png, ...",
                     [this](const ControlCommand& cmd) -> std::string {
        double n = 0;
        if (cmd.args.size() < 2 || !num(cmd.args[1], n) || n < 1)
            return "err usage: shots <prefix> <frames>  (writes <prefix>_000.png ...)";
        burstPrefix_ = cmd.args[0]; burstLeft_ = static_cast<int>(n); burstIndex_ = 0;
        return "ok shots armed for " + std::to_string(burstLeft_) + " frames";
    });

    commandTable.add("overlay", "overlay <hud|ui|debug|lamplights|master|agents|cones|nav|plan> <on|off>",
                     "Show or hide an overlay: the HUD, the UI, the debug overlay, street-lamp lights, or a city sim layer.",
                     [this](const ControlCommand& cmd) -> std::string {
        if (cmd.args.size() < 2) return "err usage: overlay <name> <on|off>";
        const std::string& what = cmd.args[0];
        const bool on = cmd.args[1] == "on" || cmd.args[1] == "1";
        if (what == "hud") { rendererPtr->showHud = on; return "ok hud"; }
        if (what == "ui") { rendererPtr->uiHidden = !on; return "ok ui"; }
        if (what == "debug") { setDebugOverlay(on); return "ok debug"; }
        // Street-lamp POINT LIGHTS, live (the emissive heads stay lit): a
        // LampLightsRequest for RenderSystem.
        if (what == "lamplights") {
            LampLightsRequest r;
            r.on = on;
            return publishRequest(eventBus, r, "overlay lamplights", "ok lamplights");
        }
        // The city sim's debug layers: a CityLayerRequest for CityRenderSystem,
        // the same one the web build's panel (rt_web_city) sends.
        if (what == "master" || what == "agents" || what == "cones" ||
            what == "nav" || what == "plan") {
            CityLayerRequest r;
            r.layer = what;
            r.on = on;
            return publishRequest(eventBus, r, "overlay " + what, "ok " + what);
        }
        return "err unknown overlay (hud|ui|debug|lamplights|master|agents|cones|nav|plan)";
    });

    commandTable.add("person", "person first|third|toggle",
                     "On-foot camera: first or third person (the V key).",
                     [this](const ControlCommand& cmd) -> std::string {
        // On-foot camera: first or third person (the V key, explicit and
        // guard-free). A PersonRequest for PlayerSystem; persisted as playerThirdPerson.
        if (cmd.args.empty() || (cmd.args[0] != "first" && cmd.args[0] != "third" && cmd.args[0] != "toggle"))
            return "err usage: person first|third|toggle";
        PersonRequest r;
        r.mode = cmd.args[0] == "toggle" ? 2 : (cmd.args[0] == "third" ? 1 : 0);
        return publishRequest(eventBus, r, "person", "ok person staged");
    });

    commandTable.add("where?", "where?",
                     "The viewpoint as x y z pitch yaw, the format teleport takes.",
                     [this](const ControlCommand&) -> std::string {
        // The viewpoint in the paste format: x y z pitch yaw.
        const CameraState& c = view.camera;
        Vec3 fwd = c.target - c.position;
        const Real len = fwd.length();
        if (len > 1e-9) fwd = fwd * (1.0 / len);
        constexpr Real kRadToDeg = 57.29577951308232;
        char buf[128];
        std::snprintf(buf, sizeof(buf), "ok %.2f %.2f %.2f %.2f %.2f", c.position.x, c.position.y,
                      c.position.z, std::asin(std::clamp(fwd.y, Real(-1), Real(1))) * kRadToDeg,
                      std::atan2(fwd.x, -fwd.z) * kRadToDeg);
        return buf;
    });

    commandTable.add("detach", "detach on|off|toggle",
                     "Detach the camera from the player for fast travel (the F key).",
                     [this](const ControlCommand& cmd) -> std::string {
        // Drive the F toggle from a script so the fast-travel path is testable
        // without a keyboard (it was reported fixed twice while broken).
        const std::string mode = cmd.args.empty() ? "toggle" : cmd.args[0];
        CameraDetachRequest r;
        if (mode == "on") r.mode = 1;
        else if (mode == "off") r.mode = 0;
        else if (mode != "toggle") return "err usage: detach on|off|toggle";
        return publishRequest(eventBus, r, "detach", "ok detach " + mode + " staged");
    });

    commandTable.add("view", "view <0-8> [wire 0-2]",
                     "Debug view: 0 normal, 1 AO, 2 SSR, 3 depth, 4 normals, 5 shadow, 6 albedo, 7 facing, 8 cascades; plus wireframe.",
                     [this](const ControlCommand& cmd) -> std::string {
        // Debug view + wireframe are plain public renderer ints (the
        // RT_DEBUG_VIEW / RT_WIREFRAME env knobs, now runtime).
        double v;
        if (cmd.args.empty() || !num(cmd.args[0], v) || v < 0 || v > 8)
            return "err usage: view <0-8> [wire0-2]  (0=normal 1=AO 2=SSR "
                   "3=depth 4=normals 5=shadow 6=albedo 7=facing 8=cascades)";
        rendererPtr->debugView = static_cast<int>(v);
        if (cmd.args.size() > 1 && num(cmd.args[1], v))
            rendererPtr->wireframe = std::clamp(static_cast<int>(v), 0, 2);
        return "ok view";
    });
}

// Time, sky and weather
void Application::registerSkyVerbs() {
    commandTable.add("sim", "sim pause|resume|step|speed <x>",
                     "Pause, resume or single-step the sim clock, or set its speed.",
                     [this](const ControlCommand& cmd) -> std::string {
        if (cmd.args.empty()) return "err usage: sim pause|resume|step|speed <x>";
        const std::string& what = cmd.args[0];
        if (what == "pause") { clock.setPaused(true); return "ok paused"; }
        if (what == "resume") { clock.setPaused(false); return "ok resumed"; }
        if (what == "step") { clock.requestStep(); return "ok stepped"; }
        if (what == "speed") {
            double s;
            if (cmd.args.size() < 2 || !num(cmd.args[1], s) || s <= 0.0)
                return "err usage: sim speed <multiplier>";
            clock.setTimeScale(s);
            return "ok speed";
        }
        return "err unknown sim action";
    });

    commandTable.add("daynight", "daynight <hour 0-24> | hold | run | minutes <n> | day <1-365> | moon <age|auto> | terrain on|off | hud on|off | chart <path.svg>",
                     "Drive the day/night cycle: set the hour, hold or run it, its length, the calendar day, the moon, terrain shadows, the sky HUD, or write the sky chart.",
                     [this](const ControlCommand& cmd) -> std::string {
        // A DayNightRequest for DayNightSystem — it works while the sim clock
        // is paused (update still runs), which is exactly the "freeze golden
        // hour, keep framing" workflow.
        DayNightRequest r;
        auto send = [&](const std::string& ok) { return publishRequest(eventBus, r, "daynight", ok); };
        if (cmd.args.empty())
            return "err usage: daynight <hour0-24>|hold|run|minutes <n>|day <1-365>|moon <age|auto>";
        if (cmd.args[0] == "terrain") {
            // Terrain horizon shadows on/off (the before/after switch).
            if (cmd.args.size() < 2 || (cmd.args[1] != "on" && cmd.args[1] != "off"))
                return "err usage: daynight terrain on|off";
            r.terrain = cmd.args[1] == "on" ? 1 : 0;
            return send("ok terrain shadows staged");
        }
        if (cmd.args[0] == "hud") {
            // The sky HUD (compass strip + polar chart), debug overlay only.
            if (cmd.args.size() < 2 || (cmd.args[1] != "on" && cmd.args[1] != "off"))
                return "err usage: daynight hud on|off";
            r.hud = cmd.args[1] == "on" ? 1 : 0;
            return send("ok sky hud staged");
        }
        if (cmd.args[0] == "chart") {
            // The day's sky chart as SVG: sun and moon arcs with hour ticks,
            // rise/set marks, both bodies now (sky_chart.h).
            if (cmd.args.size() < 2) return "err usage: daynight chart <path.svg>";
            std::string path = cmd.args[1];
            for (size_t i = 2; i < cmd.args.size(); ++i) path += " " + cmd.args[i];
            r.chart = path;
            return send("ok chart staged");
        }
        if (cmd.args[0] == "day") {
            // The calendar day: season and the moon's phase follow it.
            double d;
            if (cmd.args.size() < 2 || !num(cmd.args[1], d) || d < 1.0 || d > 365.0)
                return "err usage: daynight day <1-365>";
            r.day = static_cast<int>(d);
            return send("ok calendar day staged");
        }
        if (cmd.args[0] == "moon") {
            // Hold the moon at an age in days (0 new, 14.77 full), or `auto`.
            if (cmd.args.size() < 2) return "err usage: daynight moon <age-days|auto>";
            double age;
            if (cmd.args[1] == "auto") {
                r.moonAuto = true;
                return send("ok moon follows the calendar");
            }
            if (!num(cmd.args[1], age) || age < 0.0)
                return "err usage: daynight moon <age-days|auto>";
            r.moon = age;
            return send("ok moon age staged");
        }
        if (cmd.args[0] == "minutes") {
            // The loop length in real minutes (0 freezes the sun).
            double m;
            if (cmd.args.size() < 2 || !num(cmd.args[1], m) || m < 0.0)
                return "err usage: daynight minutes <real-minutes-per-day>";
            r.minutes = m;
            return send("ok day length staged");
        }
        if (cmd.args[0] == "hold") {
            r.hold = 1;
            return send("ok cycle held");
        }
        if (cmd.args[0] == "run") {
            r.hold = 0;
            return send("ok cycle running");
        }
        double hour;
        if (!num(cmd.args[0], hour) || hour < 0.0 || hour >= 24.0)
            return "err usage: daynight <hour0-24>|hold|run|minutes <n>|day <1-365>|moon <age|auto>";
        r.hour = hour;
        return send("ok time staged");
    });

    commandTable.add("daynight?", "daynight?",
                     "The clock: the sky's hour, the loop length, sunrise and sunset, and the city sim's hour beside it.",
                     unanswered("a DayNightSystem"));

    commandTable.add("weather", "weather clear|fair|overcast|storm|auto|off [now]",
                     "Set the weather (eased in, or `now` to snap); auto walks between states; off hands the clouds back to the panel.",
                     [this](const ControlCommand& cmd) -> std::string {
        // Weather states over the volumetric deck (weather_cycle.h); consumed
        // by DayNightSystem. `auto` starts the seeded neighbor walk; `off`
        // hands the cloud knobs back to the panel and `clouds`.
        if (cmd.args.empty()) return "err usage: weather clear|fair|overcast|storm|auto|off";
        const std::string& w = cmd.args[0];
        if (w != "clear" && w != "fair" && w != "overcast" && w != "storm" &&
            w != "auto" && w != "off")
            return "err usage: weather clear|fair|overcast|storm|auto|off";
        // A trailing `now` snaps the state instead of easing the front in
        // (screenshots, the editor's compare); DayNightSystem parses it.
        WeatherRequest r;
        r.state = w;
        r.now = cmd.args.size() > 1 && cmd.args[1] == "now";
        return publishRequest(eventBus, r, "weather", r.now ? "ok weather set" : "ok weather staged");
    });

    commandTable.add("clouds", "clouds [on|off] [coverage <0-1>] [density <v>] [scale <v>] [wind <m/s>]",
                     "Set the cloud deck live: on or off, coverage, density, scale, wind. Kept with the deck's settings.",
                     [this](const ControlCommand& cmd) -> std::string {
        // A CloudsRequest for DayNightSystem; a knob not named is left alone.
        // Under `weather` other than off the weather drives the deck instead.
        const char* usage = "err usage: clouds [on|off] [coverage <0-1>] [density <v>] [scale <v>] [wind <m/s>]";
        if (cmd.args.empty()) return usage;
        CloudsRequest r;
        for (size_t i = 0; i < cmd.args.size(); ++i) {
            const std::string& k = cmd.args[i];
            if (k == "on" || k == "off") { r.enabled = k == "on" ? 1 : 0; continue; }
            double v;
            if (i + 1 >= cmd.args.size() || !num(cmd.args[i + 1], v) || v < 0) return usage;
            ++i;
            if (k == "coverage") r.coverage = v;
            else if (k == "density") r.density = v;
            else if (k == "scale") r.scale = v;
            else if (k == "wind") r.wind = v;
            else return usage;
        }
        return publishRequest(eventBus, r, "clouds", "ok clouds");
    });

    commandTable.add("weather?", "weather?",
                     "The weather state, and whether it is on auto.",
                     unanswered("a DayNightSystem"));

    commandTable.add("sun", "sun <x> <y> <z> [intensity] [r g b]",
                     "Set the sun's direction (toward the sun), intensity and colour, for levels with a static sun.",
                     [this](const ControlCommand& cmd) -> std::string {
        // Direct sun set — the ImGui Lighting panel's slider, remotely. This
        // is the golden-hour knob for levels that author a STATIC sun (no
        // day/night cycle to steer, e.g. metropolis_sky in edit mode); with a
        // live cycle the next applyLighting overwrites it — use `daynight`.
        // Sky disc follows the sun so the scattering sky agrees with shadows;
        // an optional colour is what actually makes an hour GOLDEN (direction
        // alone reads as grey daylight from a warm-white authored sun).
        double v[7] = {0, 0, 0, -1, -1, -1, -1};
        if (cmd.args.size() < 3 || !num(cmd.args[0], v[0]) ||
            !num(cmd.args[1], v[1]) || !num(cmd.args[2], v[2]))
            return "err usage: sun <x> <y> <z> [intensity] [r g b] "
                   "(direction TOWARD sun)";
        for (size_t i = 3; i < cmd.args.size() && i < 7; ++i)
            num(cmd.args[i], v[i]);
        Vec3 dir(v[0], v[1], v[2]);
        const Real len = dir.length();
        if (len < 1e-6) return "err zero sun direction";
        dir = dir * (1.0 / len);
        auto& lit = view.lighting;
        lit.sun.direction = dir;
        lit.sky.sunDirection = dir;
        if (v[3] > 0) lit.sun.intensity = static_cast<float>(v[3]);
        if (v[4] >= 0 && v[5] >= 0 && v[6] >= 0) {
            const Vec3 col(v[4], v[5], v[6]);
            lit.sun.color = col;
            lit.sky.sunColor = col;
        }
        return "ok sun set";
    });

    commandTable.add("sun?", "sun?",
                     "The solar elevation, the active light's height and intensity, and whether it is dark.",
                     [this](const ControlCommand&) -> std::string {
        // The lit truth, for numeric light probes (find golden hour without
        // eyeballing screenshots). sunY is the SOLAR elevation — at night the
        // active light in slot 0 is the moon (WS2), so lightY/intensity
        // describe what's lighting the world while sunY/dark keep the solar
        // truth the lamp predicates use.
        const auto& lit = view.lighting;
        char buf[200];
        std::snprintf(buf, sizeof(buf),
                      "ok sunY=%.3f lightY=%.3f intensity=%.3f dark=%d",
                      lit.solarElevation, lit.sun.direction.y,
                      lit.sun.intensity,
                      lit.solarElevation < 0.06 ? 1 : 0);
        return buf;
    });

    commandTable.add("fog", "fog <density> [heightFalloff] [r g b]",
                     "Tune the level's fog live (nothing is saved).",
                     [this](const ControlCommand& cmd) -> std::string {
        // The atmosphere tuning knob (WS: "turn fog back on and help me tune
        // it"): density [heightFalloff] [r g b], live on the level's lighting.
        // Like `sun`, this edits the LEVEL-OWNED lighting — nothing persists;
        // bake the values you like into the level JSON.
        double v[5] = {-1, -1, -1, -1, -1};
        if (cmd.args.empty() || !num(cmd.args[0], v[0]) || v[0] < 0)
            return "err usage: fog <density> [heightFalloff] [r g b]";
        for (size_t i = 1; i < cmd.args.size() && i < 5; ++i)
            num(cmd.args[i], v[i]);
        auto& fog = view.lighting.fog;
        fog.enabled = v[0] > 0.0;
        fog.density = static_cast<float>(v[0]);
        if (v[1] >= 0) fog.heightFalloff = static_cast<float>(v[1]);
        if (v[2] >= 0 && v[3] >= 0 && v[4] >= 0)
            fog.color = Vec3(v[2], v[3], v[4]);
        return "ok fog set";
    });

    commandTable.add("fog?", "fog?",
                     "The fog's settings now.",
                     [this](const ControlCommand&) -> std::string {
        const auto& fog = view.lighting.fog;
        char buf[160];
        std::snprintf(buf, sizeof(buf),
                      "ok enabled=%d density=%.5f heightFalloff=%.4f "
                      "color=%.2f,%.2f,%.2f",
                      fog.enabled ? 1 : 0, fog.density, fog.heightFalloff,
                      fog.color.x, fog.color.y, fog.color.z);
        return buf;
    });
}

// The player
void Application::registerPlayerVerbs() {
    commandTable.add("teleport", "teleport <x y z [pitch yaw] | x z> | open",
                     "Move the player (the camera when there is none); `x z` lands on the ground. Read the result with teleport?. `open` unfolds the Teleport panel.",
                     [this](const ControlCommand& cmd) -> std::string {
        // Move the PLAYER (the camera when there is none): "x y z [pitch yaw]"
        // or "x z" (on the ground) — the Teleport panel's paste format, so a
        // location goes between the panel, the log and a session by paste.
        // A TeleportRequest for TeleportSystem; `teleport?` (bound by it) answers.
        if (cmd.args.empty()) return "err usage: teleport <x y z [pitch yaw] | x z> | open";
        // `open` unfolds the Debug window's Teleport section (a probe can shoot it).
        if (cmd.args[0] == "open")
            return publishRequest(eventBus, TeleportPanelRequest{}, "teleport open", "ok teleport panel");
        std::string pose = cmd.args[0];
        for (size_t i = 1; i < cmd.args.size(); ++i) pose += " " + cmd.args[i];
        TeleportRequest r;
        if (parseTeleportPose(pose, r.pose) < 2) return "err unreadable pose: " + pose;
        return publishRequest(eventBus, r, "teleport", "ok teleport staged (poll `teleport?`)");
    });

    commandTable.add("teleport?", "teleport?",
                     "Where the last teleport landed, or (pending).",
                     unanswered("a TeleportSystem: play mode"));

    commandTable.add("tap", "tap <KeyName>",
                     "Press and release a key through the real input path (e.g. tap F).",
                     [this](const ControlCommand& cmd) -> std::string {
        // Press and release a key through the real input path (see the frame
        // loop). Tests the KEY, not a verb that imitates what the key does.
        if (cmd.args.empty()) return "err usage: tap <KeyName>  (e.g. tap F)";
        if (keyCodeFromName(cmd.args[0]) == KeyCode::Unknown)
            return "err unknown key: " + cmd.args[0];
        eventBus.publish(KeyTapRequest{cmd.args[0]});
        return "ok tap " + cmd.args[0] + " staged";
    });

    commandTable.add("hold", "hold <KeyName|MouseLeft> <seconds>",
                     "Hold a key down through the real input path for that much sim time.",
                     [this](const ControlCommand& cmd) -> std::string {
        // Hold a key down through the real input path for <seconds> of sim time.
        double secs = 0.0;
        if (cmd.args.size() < 2 || !num(cmd.args[1], secs) || secs <= 0.0)
            return "err usage: hold <KeyName> <seconds>";
        if (keyCodeFromName(cmd.args[0]) == KeyCode::Unknown && cmd.args[0] != "MouseLeft")
            return "err unknown key: " + cmd.args[0] + " (keys, or MouseLeft)";
        KeyHoldRequest r;
        r.key = cmd.args[0];
        r.seconds = secs;
        eventBus.publish(r);
        return "ok hold " + cmd.args[0] + " staged";
    });

    // drop a car in front of the player (the N key); `vehicle_spawn moped`
    commandTable.add("vehicle_spawn", "vehicle_spawn [recipe]",
                     "Drop a vehicle in front of the player (the N key); e.g. vehicle_spawn moped.",
                     [this](const ControlCommand& cmd) -> std::string {
        VehicleSpawnRequest r;
        if (!cmd.args.empty()) r.recipe = cmd.args[0];
        return publishRequest(eventBus, r, "vehicle_spawn", "ok staged");
    });

    commandTable.add("vehicle?", "vehicle?",
                     "The player's vehicle: speed, rpm, gear, drive mode, position, and the sim speed.",
                     unanswered("a VehicleSystem: play mode"));   // measured by it each frame

    // FURNITURE INTERACTIONS (InteractionSystem): `interact` taps the interact key, `interact hold` holds it,
    // `interact stand` stands up, `interact next` moves the focus (Tab); `interact?` reports the focus.
    commandTable.add("interact", "interact [hold|stand|next]",
                     "Furniture interaction: tap or hold the interact key, stand up, or move the focus (Tab).",
                     [this](const ControlCommand& cmd) -> std::string {
        const std::string a = cmd.args.empty() ? "" : cmd.args[0];
        InteractRequest r;
        r.action = a == "hold" ? 2 : a == "stand" ? 3 : a == "next" ? 4 : 1;
        return publishRequest(eventBus, r, "interact", "ok staged");
    });

    // THE PLAYER'S MOVEMENT (engine/locomotion.h): mode (walk|mantle|climb|swim|ladder), ground, position,
    // velocity, stamina and the last change ("jump (off steep ground)", "pull up onto a ledge 1.20 m ...").
    commandTable.add("move?", "move?",
                     "The player's movement: mode, ground, position, velocity, stamina and the last change.",
                     unanswered("a PlayerSystem: play mode"));

    // go limp (the G key); jump, or lying still a moment, gets up
    commandTable.add("ragdoll", "ragdoll",
                     "The player goes limp (the G key); jump, or lying still a moment, gets up.",
                     [this](const ControlCommand&) -> std::string {
        return publishRequest(eventBus, LimpRequest{}, "ragdoll", "ok staged");
    });

    // `train_board <train> [car]`: aboard, standing mid-carriage
    commandTable.add("train_board", "train_board [train] [car]",
                     "Put the player aboard a train, standing mid-carriage (defaults: train 0, car 1).",
                     [this](const ControlCommand& cmd) -> std::string {
        TrainBoardRequest r;
        double v;
        if (!cmd.args.empty()) {
            if (!num(cmd.args[0], v) || v < 0) return "err usage: train_board [train] [car]";
            r.train = static_cast<int>(v);
        }
        if (cmd.args.size() > 1) {
            if (!num(cmd.args[1], v) || v < 1) return "err usage: train_board [train] [car]";
            r.car = static_cast<int>(v);
        }
        return publishRequest(eventBus, r, "train_board", "ok staged");
    });

    commandTable.add("interact?", "interact?",
                     "What the interaction focus is on.",
                     unanswered("an InteractBrokerSystem: play mode"));

    commandTable.add("ride", "ride bus|taxi|any|off | ride <x> <z>",
                     "The player boards the nearest bus or taxi, or gets off; with x z, the possessed agent rides a bus there.",
                     [this](const ControlCommand& cmd) -> std::string {
        // One verb, two riders. `ride <x> <z>` sends the possessed agent by bus to a
        // place (ADR-0091); it was a second handler further down that this one always
        // returned before, so it never ran.
        double x, z;
        if (cmd.args.size() >= 2 && num(cmd.args[0], x) && num(cmd.args[1], z)) {
            return publishRequest(eventBus, PossessRequest{"ride " + cmd.args[0] + " " + cmd.args[1]}, cmd.name, "ok staged (poll possess?)");
        }
        // Otherwise the player boards the nearest bus or cab, or gets off
        // (city_player_transit.h). NPCs have been riding since RideBook; this is the player's seat.
        const std::string what = cmd.args.empty() ? "any" : cmd.args[0];
        RideRequest r;
        if (what == "off") r.mode = 0;
        else if (what == "bus") r.mode = 1;
        else if (what == "taxi" || what == "cab") r.mode = 2;
        else if (what == "any") r.mode = 3;
        else return "err usage: ride bus|taxi|any|off  (or ride <x> <z> for the possessed agent)";
        return publishRequest(eventBus, r, "ride", "ok ride " + what + " staged");
    });

    commandTable.add("elevator", "elevator call | pick <delta>",
                     "Press the elevator call key, or move the panel's floor selection by delta.",
                     [this](const ControlCommand& cmd) -> std::string {
        // The elevator's verbs, remotely (an ElevatorRequest; ElevatorSystem
        // takes it next frame, exactly like the E / Up / Down keys): `elevator
        // call` presses E, `elevator pick <n>` moves the panel's selection by n
        // storeys. Drives a headless ride for frames (tools/elevator_ride.py).
        if (cmd.args.empty()) return "err usage: elevator call | pick <delta>";
        ElevatorRequest r;
        if (cmd.args[0] == "call") {
            r.call = true;
            return publishRequest(eventBus, r, "elevator", "ok elevator call staged");
        }
        double pick;
        if (cmd.args[0] == "pick" && cmd.args.size() > 1 && num(cmd.args[1], pick)) {
            r.pick = static_cast<int>(pick);
            return publishRequest(eventBus, r, "elevator", "ok elevator pick staged");
        }
        return "err usage: elevator call | pick <delta>";
    });

    commandTable.add("elevator?", "elevator?",
                     "The elevator: in the cab or at a door, moving, the floor, the selection and the cab's floor.",
                     unanswered("an ElevatorSystem: play mode in a building with lifts"));
}

// Possession
void Application::registerPossessionVerbs() {
    // --- Possession (ADR-0079): transient play acts ----------------------
    // Application stays city-free (engine_core charter): these verbs only
    // publish a PossessRequest (control/city_requests.h); CityPossessSystem
    // (registered by city-running states) hears it and answers possess?,
    // look? and agent?. In a state without that system the verbs say so.
    commandTable.add("possess", "possess car|walker [x z]",
                     "Take over a city car or walker (the nearest, or the one nearest x z). Poll possess?.",
                     [this](const ControlCommand& cmd) -> std::string {
        if (cmd.args.empty() ||
            (cmd.args[0] != "car" && cmd.args[0] != "walker"))
            return "err usage: possess <car|walker> [x z]";
        std::string staged = cmd.args[0];
        for (size_t i = 1; i < cmd.args.size() && i < 3; ++i)
            staged += " " + cmd.args[i];
        return publishRequest(eventBus, PossessRequest{staged}, cmd.name, "ok staged (poll possess?)");
    });

    {
        const ControlCommands::Handler h = [this](const ControlCommand& cmd) -> std::string {
            double x, z;
            if (cmd.args.size() < 2 || !num(cmd.args[0], x) || !num(cmd.args[1], z))
                return "err usage: " + cmd.name + " <x> <z>";
            const std::string line = (cmd.name == "drive_to" ? "driveto " : "walkto ") + cmd.args[0] + " " + cmd.args[1];
            return publishRequest(eventBus, PossessRequest{line}, cmd.name, "ok staged (poll possess?)");
        };
        commandTable.add("drive_to", "drive_to <x> <z>",
                         "Drive the possessed car to a point.", h);
        commandTable.add("walk_to", "walk_to <x> <z>",
                         "Walk the possessed walker to a point.", h);
    }

    // Hold the goal layer off the possessed agent so a director's plan is not
    // overwritten by its schedule seconds later (ADR-0091).
    commandTable.add("direct", "direct on|off",
                     "Hold the possessed agent's own schedule off, so a director's plan is not overwritten.",
                     [this](const ControlCommand& cmd) -> std::string {
        if (cmd.args.size() != 1 || (cmd.args[0] != "on" && cmd.args[0] != "off"))
            return "err usage: direct on|off";
        return publishRequest(eventBus, PossessRequest{"direct " + cmd.args[0]}, cmd.name, "ok staged (poll possess?)");
    });

    // Get into the nearest free car / get out again (ADR-0091).
    commandTable.add("board", "board [x z]",
                     "The possessed walker gets into the nearest free car (or the one nearest x z).",
                     [this](const ControlCommand& cmd) -> std::string {
        std::string arg;
        if (cmd.args.size() >= 2) arg = " " + cmd.args[0] + " " + cmd.args[1];
        return publishRequest(eventBus, PossessRequest{"board" + arg}, cmd.name, "ok staged (poll possess?)");
    });

    {
        const ControlCommands::Handler h = [this](const ControlCommand& cmd) -> std::string {
            return publishRequest(eventBus, PossessRequest{cmd.name}, cmd.name, "ok staged (poll possess?)");
        };
        commandTable.add("enter", "enter",
                         "The possessed walker goes into the building whose door it is standing at.", h);
        commandTable.add("exit", "exit",
                         "The possessed walker comes back out to the street.", h);
    }

    commandTable.add("alight", "alight",
                     "The possessed walker gets out of the car it is in.",
                     [this](const ControlCommand&) -> std::string {
        return publishRequest(eventBus, PossessRequest{"alight"}, "alight", "ok staged (poll possess?)");
    });

    commandTable.add("possess_stop", "possess_stop",
                     "Stop the possessed agent where it is.",
                     [this](const ControlCommand&) -> std::string {
        return publishRequest(eventBus, PossessRequest{"stop"}, "possess_stop", "ok staged");
    });

    commandTable.add("release", "release",
                     "Hand the possessed agent back to the city sim.",
                     [this](const ControlCommand&) -> std::string {
        return publishRequest(eventBus, PossessRequest{"release"}, "release", "ok staged");
    });

    commandTable.add("possess?", "possess?",
                     "The possessed agent: kind, state, position, speed and distance left.",
                     unanswered("a CityPossessSystem: play mode on a city level"));

    // What the agent can SEE: the places, people and transit around it, each
    // with a point walk_to accepts (ADR-0091). Written every frame by
    // CityPossessSystem; "none" when nothing is possessed.
    commandTable.add("look?", "look?",
                     "What the possessed agent can see: places, people and transit nearby, each with a point walk_to accepts.",
                     unanswered("a CityPossessSystem: play mode on a city level"));

    // The possessed agent's own mind: activity, goal state, and the fields that
    // decide whether it walks (ADR-0091). Written every frame by
    // CityPossessSystem; "none" when nothing is possessed.
    commandTable.add("agent?", "agent?",
                     "The possessed agent's mind: activity, goal state, and what decides whether it walks.",
                     unanswered("a CityPossessSystem: play mode on a city level"));
}

// Asking the city
void Application::registerCityVerbs() {
    commandTable.add("walkers?", "walkers?",
                     "The city's walkers: how many, the fastest, step reversals, seated, indoors, students, on walks, joggers and games.",
                     unanswered("a CityWalkerSystem: play mode on a city level"));

    // throw the townsperson nearest the player as if a car hit it (the log says where)
    commandTable.add("knock", "knock",
                     "Throw the townsperson nearest the player as if a car hit them.",
                     [this](const ControlCommand&) -> std::string {
        return publishRequest(eventBus, KnockRequest{}, "knock", "ok staged");
    });

    commandTable.add("knock?", "knock?",
                     "Townspeople thrown so far, and the ragdolls in play.",
                     unanswered("a CityWalkerSystem: play mode on a city level"));

    commandTable.add("flicker?", "flicker?",
                     "Walker flickers (a body gone and back within 1.5 s): the count and the latest few.",
                     unanswered("a CityWalkerSystem: play mode on a city level"));

    // what the city's people are out doing (citysim activities): per activity, at it / coming, groups, the nearest
    commandTable.add("activities?", "activities?",
                     "What the city's people are out doing: per activity, who is at it or coming, groups, the nearest.",
                     unanswered("a CityWalkerSystem: play mode on a city level"));

    // people drawn moving faster than a sprint (citysim walkers): how many, and the latest with their state
    commandTable.add("speeders?", "speeders?",
                     "People drawn moving faster than a sprint: how many, and the latest with their state.",
                     unanswered("a CityWalkerSystem: play mode on a city level"));

    commandTable.add("parking?", "parking?",
                     "Parking: kerb bays and garage stalls taken, and the nearest taken stall.",
                     unanswered("a CityRenderSystem: play mode on a city level"));

    commandTable.add("traffic?", "traffic?",
                     "City traffic: cars driving island-wide and any pile-ups (more with RT_TRAFFIC_DEBUG).",
                     unanswered("a CityRenderSystem: play mode on a city level"));

    commandTable.add("opensigns?", "opensigns?",
                     "Shops' OPEN signs: lit, closed for the night, open with nobody on shift, and the nearest lit one.",
                     unanswered("a CityRenderSystem: play mode on a city level"));

    // the trains (citysim train_system): lines, stops, every train's place, speed and doors, and whether you are aboard
    commandTable.add("trains?", "trains?",
                     "The trains: lines, stops, every train's place, speed and doors, and whether you are aboard.",
                     unanswered("a TrainSystem: play mode on a level with rail"));

    commandTable.add("depots?", "depots?",
                     "Buses: on route, heading in, off duty and parked in depot bays, the nearest bus and bay, and any stalled heading in.",
                     unanswered("a CityRenderSystem: play mode on a city level"));

    // `pin <agent>` / `unpin <agent>`: FOLLOW an agent -- it stays in the full sim however far it goes
    {
        const ControlCommands::Handler h = [this](const ControlCommand& cmd) -> std::string {
            double id;
            if (cmd.args.empty() || !num(cmd.args[0], id) || id < 0) return "err usage: pin|unpin <agent id>";
            PinRequest r;
            r.agent = static_cast<int>(id);
            r.on = cmd.name == "pin";
            return publishRequest(eventBus, r, cmd.name, "ok staged (`pin?` for the count)");
        };
        commandTable.add("pin", "pin <agent id>",
                         "Follow an agent: it stays in the full sim however far it goes.", h);
        commandTable.add("unpin", "unpin <agent id>",
                         "Stop following an agent.", h);
    }

    commandTable.add("jitter", "jitter [agent id|-1] [frames]",
                     "Record how evenly a vehicle is drawn for some frames (default 300): -1 the bus or cab you ride, else the nearest moving one. jitter? answers.",
                     [this](const ControlCommand& cmd) -> std::string {
        JitterRequest r;
        double v;
        if (!cmd.args.empty()) {
            if (!num(cmd.args[0], v)) return "err usage: jitter [agent id|-1] [frames]";
            r.agent = static_cast<int>(v);
        }
        if (cmd.args.size() > 1) {
            if (!num(cmd.args[1], v) || v < 10) return "err usage: jitter [agent id|-1] [frames >= 10]";
            r.frames = static_cast<int>(v);
        }
        return publishRequest(eventBus, r, "jitter", "ok recording (read `jitter?` when it is done)");
    });

    commandTable.add("jitter?", "jitter?",
                     "The jitter measurement: which vehicle is being recorded, then how evenly it and the camera moved.",
                     unanswered("a CityRenderSystem: play mode on a city level"));

    commandTable.add("pin?", "pin?",
                     "The agents being followed.",
                     unanswered("a CityRenderSystem: play mode on a city level"));

    // `findplace <name...>`: where a named place is (a shop by its sign), its type, hours and who is in; `findplace?` reads it
    commandTable.add("findplace", "findplace <name...>",
                     "Find a named place (a shop by its sign); read the answer with findplace?.",
                     [this](const ControlCommand& cmd) -> std::string {
        FindPlaceRequest r;
        for (const std::string& w : cmd.args) r.name += (r.name.empty() ? "" : " ") + w;
        return publishRequest(eventBus, r, "findplace", "ok");
    });

    commandTable.add("findplace?", "findplace?",
                     "The last findplace answer: where, its type, hours and who is in.",
                     unanswered("a CityRenderSystem: play mode on a city level"));

    // `who? <agent>`: stage a question about any agent; `who?` alone: the answer (refreshed every step)
    commandTable.add("who?", "who? [agent id]",
                     "With an id, ask about any agent; alone, read the answer (refreshed every step).",
                     unanswered("a CityWalkerSystem: play mode on a city level"));
}

// Asking the ground and the buildings
void Application::registerGroundVerbs() {
    // `recipes?`: the level's buildings counted by architect recipe, each with its tallest one's centre, base and
    // height -- whether a recipe the tables roll actually stands in this city, and where to go and look at it
    commandTable.add("recipes?", "recipes?",
                     "The level's buildings counted by architect recipe, each with its tallest one's place and height.",
                     [this](const ControlCommand&) -> std::string {
        const CityBuildings* cb = nullptr;
        worldState.each<CityBuildings>([&](Entity, CityBuildings& b) { if (!cb) cb = &b; });
        if (!cb) return "ok none (no city buildings on this level)";
        struct Tally { int n = 0; const BuildingRecord* tallest = nullptr; };
        std::map<std::string, Tally> by;
        for (const BuildingRecord& r : cb->records) {
            Tally& t = by[r.recipe.empty() ? "?" : r.recipe]; ++t.n;
            if (!t.tallest || r.height > t.tallest->height) t.tallest = &r;
        }
        std::string out = "ok " + std::to_string(cb->records.size()) + " buildings, " + std::to_string(by.size()) + " recipes";
        char buf[160];
        for (const auto& [name, t] : by) {
            const engine::Vec2 c = t.tallest->plan.size() >= 3 ? centroid(t.tallest->plan) : engine::Vec2(0, 0);
            std::snprintf(buf, sizeof buf, " | %s %d tallest (%.0f, %.0f) base %.1f h %.0f", name.c_str(), t.n, c.x, c.y, t.tallest->baseY, t.tallest->height);
            out += buf;
        }
        return out;
    });

    // THE TALL BUILDINGS round a point, for a probe to aim at (#114: a swing probe that hooked empty sky never
    // swept a tower's storeys): `towers? <x> <z> [radius] [min height]` -> up to 24, tallest first, each
    // "cx cz groundY height" with its plan's centre.
    commandTable.add("towers?", "towers? <x> <z> [radius 300] [min height 40]",
                     "The tallest buildings round a point (up to 24): centre, ground height and height.",
                     [this](const ControlCommand& cmd) -> std::string {
        if (cmd.args.size() < 2) return "err usage: towers? <x> <z> [radius 300] [min height 40]";
        double qx = 0, qz = 0, rad = 300.0, minH = 40.0;
        if (!num(cmd.args[0], qx) || !num(cmd.args[1], qz)) return "err usage: towers? <x> <z> [radius] [min height]";
        if (cmd.args.size() > 2) num(cmd.args[2], rad);
        if (cmd.args.size() > 3) num(cmd.args[3], minH);
        const CityBuildings* cb = nullptr;
        worldState.each<CityBuildings>([&](Entity, CityBuildings& c) { if (!cb) cb = &c; });
        if (!cb) return "ok no city";
        std::vector<const BuildingRecord*> near;
        cb->near(engine::Vec2(qx, qz), rad, near);
        std::vector<std::pair<double, const BuildingRecord*>> tall;
        for (const BuildingRecord* r : near)
            if (r->height >= minH && r->plan.size() >= 3) tall.push_back({r->height, r});
        std::sort(tall.begin(), tall.end(), [](const auto& a, const auto& b) { return a.first > b.first; });
        std::string out = "ok " + std::to_string(tall.size());
        for (std::size_t i = 0; i < tall.size() && i < 24; ++i) {
            const BuildingRecord& r = *tall[i].second;
            engine::Vec2 c(0, 0);
            for (const engine::Vec2& v : r.plan) c = c + v * (1.0 / static_cast<double>(r.plan.size()));
            char b[120];
            std::snprintf(b, sizeof b, " | %.1f %.1f %.1f %.1f", c.x, c.y, r.groundY, r.height);
            out += b;
        }
        return out;
    });

    commandTable.add("lot?", "lot? <x> <z> [radius 40]",
                     "What is actually here: the nearest road edge (and whether the point is in it), and the building records around, with their party walls.",
                     [this](const ControlCommand& cmd) -> std::string {
        // WHAT IS ACTUALLY HERE (Glenn: "Are you able to pull the city block
        // and lot details while the game is running? If not that would be a
        // useful mcp diagnostic").
        //
        // `citymap` draws the whole city and the censuses count aggregates, but
        // there was no way to stand somewhere wrong and ask WHY. Chasing the
        // lot-in-a-street bug cost a day of measuring populations to find four
        // bad pads, when the one question worth asking was "what is the lot
        // under my feet, and what does it think it is".
        if (cmd.args.size() < 2)
            return "err usage: lot? <x> <z> [radius]  (radius default 40)";
        double qx = 0, qz = 0, rad = 40.0;
        if (!num(cmd.args[0], qx) || !num(cmd.args[1], qz))
            return "err usage: lot? <x> <z> [radius]";
        if (cmd.args.size() > 2) num(cmd.args[2], rad);
        const engine::Vec2 q(qx, qz);
        std::string out = "at (" + std::to_string(qx) + ", " + std::to_string(qz) + ")";

        // THE ROAD under/near the point: the thing a lot must not be standing in.
        const CityMap* map = nullptr;
        worldState.each<CityMap>([&](Entity, CityMap& m) { if (!map) map = &m; });
        if (map && map->data) {
            const engine::RoadGraph& g = map->data->roads;
            double bestD = 1e30, bestW = 0;
            int bestE = -1;
            for (std::size_t e = 0; e < g.edges.size(); ++e) {
                const engine::RoadEdge& ed = g.edges[e];
                if (ed.a < 0 || ed.b < 0 ||
                    ed.a >= static_cast<int>(g.nodes.size()) ||
                    ed.b >= static_cast<int>(g.nodes.size())) continue;
                const engine::Vec2& A = g.nodes[ed.a].pos;
                const engine::Vec2& B = g.nodes[ed.b].pos;
                engine::Vec2 ab = B - A;
                const double l2 = ab.x * ab.x + ab.y * ab.y;
                double t = l2 > 1e-12 ? ((q.x - A.x) * ab.x + (q.y - A.y) * ab.y) / l2 : 0.0;
                t = t < 0 ? 0 : (t > 1 ? 1 : t);
                const engine::Vec2 P(A.x + ab.x * t, A.y + ab.y * t);
                const double d = std::sqrt((q.x - P.x) * (q.x - P.x) + (q.y - P.y) * (q.y - P.y));
                if (d < bestD) { bestD = d; bestW = ed.width; bestE = static_cast<int>(e); }
            }
            if (bestE >= 0) {
                const double into = bestW * 0.5 - bestD;
                out += " | road edge " + std::to_string(bestE) + " width " +
                       std::to_string(bestW) + " centreline " + std::to_string(bestD) + " m";
                out += into > 0 ? (" -- IN THE CARRIAGEWAY by " + std::to_string(into) + " m")
                                : (" (clear by " + std::to_string(-into) + " m)");
            }
        } else {
            out += " | no city map on this level";
        }

        // THE BUILDINGS here: what stands on this ground and what it thinks it is.
        const CityBuildings* cb = nullptr;
        worldState.each<CityBuildings>([&](Entity, CityBuildings& b) { if (!cb) cb = &b; });
        if (!cb) return "ok " + out + " | no city buildings on this level";
        std::vector<const BuildingRecord*> hits;
        cb->near(q, rad, hits);
        // its party walls (ADR-0227): the line each stands on, and the last storey it is blank to (0 = the roof)
        auto partyOf = [](const BuildingParams& p) {
            std::string s;
            for (int k = 0; k < std::min<int>(p.partyWalls, 4); ++k) {
                char buf[96];
                std::snprintf(buf, sizeof buf, " party%d=(n %.2f,%.2f at %.1f top %d)", k, p.partyN[k].x, p.partyN[k].y,
                              p.partyAt[k], p.partyTop[k]);
                s += buf;
            }
            return s;
        };
        out += " | " + std::to_string(hits.size()) + " record(s) within " +
               std::to_string(rad) + " m";
        int shown = 0;
        for (const BuildingRecord* r : hits) {
            if (shown++ >= 6) { out += " | ..."; break; }
            const bool inside = r->plan.size() >= 3 && pointInPolygon(r->plan, q);
            const engine::Vec2 c = r->plan.size() >= 3 ? centroid(r->plan) : engine::Vec2(0, 0);
            out += " | #" + std::to_string(static_cast<int>(r - cb->records.data())) +
                   " " + (r->type.empty() ? "?" : r->type) +
                   "/" + (r->recipe.empty() ? "?" : r->recipe) +
                   " district=" + (r->district.empty() ? "?" : r->district) +
                   " verts=" + std::to_string(r->plan.size()) +
                   " baseY=" + std::to_string(r->baseY) +
                   " h=" + std::to_string(r->height) +
                   " floors=" + std::to_string(r->params.floors) +
                   partyOf(r->params) +
                   (r->enterable ? " enterable" : "") +
                   " centroid=(" + std::to_string(c.x) + ", " + std::to_string(c.y) + ")" +
                   (inside ? " <-- YOU ARE INSIDE THIS PLAN" : "");
        }
        return "ok " + out;
    });

    commandTable.add("citymap", "citymap <path.svg> [layer,layer,...|all]",
                     "Write the running level's layered city map as SVG.",
                     [this](const ControlCommand& cmd) -> std::string {
        // The layered city map, from the running level (city_svg.h):
        // `citymap <path.svg> [roads,sidewalks,furniture,...|all]`.
        if (cmd.args.empty()) return "err usage: citymap <path.svg> [layer,layer,...|all]";
        const CityMap* map = nullptr;
        worldState.each<CityMap>([&](Entity, CityMap& m) { if (!map) map = &m; });
        if (!map || !map->data) return "err no city map on this level (no roads?)";
        const CityMapLayers layers =
            cmd.args.size() > 1 ? CityMapLayers::fromList(cmd.args[1]) : CityMapLayers();
        std::vector<const RoadDeckField*> decks;
        worldState.each<RoadDeck>([&](Entity, RoadDeck& d) { decks.push_back(&d.field); });
        return writeCityMapSvg(cmd.args[0], *map->data, layers, decks)
                   ? "ok city map written (" + layers.toList() + ")"
                   : "err cannot write " + cmd.args[0];
    });

    // Ground probe (the planting/seam diagnosis tool): `ground? x z` stages a
    // query CityRenderSystem answers next frame (carved field, road lift, and
    // the nearest nav link's deck height at that point). Poll until the reply
    // echoes your coordinates — the same ask-again idiom as possess?.
    commandTable.add("ground?", "ground? [x z [r]]",
                     "With x z, ask about the ground there (carved field, road lift, nearest deck height); alone, read the answer.",
                     unanswered("a CityRenderSystem: play mode on a city level"));
}

// Engine knobs and health
void Application::registerEngineVerbs() {
    commandTable.add("reload", "reload",
                     "Reload the level (regrows the city).",
                     [this](const ControlCommand&) -> std::string {
        if (!controlReloadFactory) return "err no reload factory registered";
        requestState(controlReloadFactory());
        return "ok reloading";
    });

    // --- Tier 2 (device: "how deep can the controls go?"): everything that
    // is already a Settings key or a public Renderer field. `set`/`get` are
    // the generic escape hatch — every settings-driven knob, current and
    // future, is reachable without another verb. The typed verbs below are
    // the ones that need more than a bare settings write.
    commandTable.add("set", "set <key> <value...>",
                     "Set any Settings key (the generic knob; `render apply` pushes render keys live).",
                     [this](const ControlCommand& cmd) -> std::string {
        if (cmd.args.size() < 2) return "err usage: set <key> <value...>";
        // Re-join: values may carry spaces (paths). Stored as a string —
        // Settings coerces on read, so numeric/bool consumers still work.
        std::string value = cmd.args[1];
        for (size_t i = 2; i < cmd.args.size(); ++i) value += " " + cmd.args[i];
        settingsStore.setString(cmd.args[0], value);
        return "ok set " + cmd.args[0];
    });

    // WHAT IS PLAYING (Glenn: "how come river town has no audio?"): the mixer's state and its voices now
    commandTable.add("audio?", "audio?",
                     "Is the audio mixer up, and how many voices are playing.",
                     [this](const ControlCommand&) -> std::string {
        char b[160];
        std::snprintf(b, sizeof b, "ok ready %d voices %zu", audioEngine.ready() ? 1 : 0, audioEngine.activeVoiceCount());
        return b;
    });

    commandTable.add("get", "get <key>",
                     "Read any Settings key.",
                     [this](const ControlCommand& cmd) -> std::string {
        if (cmd.args.empty()) return "err usage: get <key>";
        const std::string v = settingsStore.getString(cmd.args[0], "");
        return v.empty() ? "ok (unset)" : "ok " + v;
    });

    {
        const ControlCommands::Handler h = [](const ControlCommand& cmd) -> std::string {
            // The debug UI's clipboard, from outside: `clip <text>` sets it,
            // `clip?` reads it — the way to tell "ImGui's copy does nothing"
            // from "the window manager's clipboard bridge is not there".
#ifdef RT_ENABLE_IMGUI
            if (ImGui::GetCurrentContext() == nullptr) return "err no debug UI context";
            if (cmd.name == "clip?") {
                const char* t = ImGui::GetClipboardText();
                return std::string("ok ") + (t && *t ? t : "(empty)");
            }
            if (cmd.args.empty()) return "err usage: clip <text>";
            std::string text = cmd.args[0];
            for (size_t i = 1; i < cmd.args.size(); ++i) text += " " + cmd.args[i];
            ImGui::SetClipboardText(text.c_str());
            return "ok clipboard set";
#else
            return "err built without the debug UI";
#endif
        };
        commandTable.add("clip", "clip <text>",
                         "Set the debug UI's clipboard.", h);
        commandTable.add("clip?", "clip?",
                         "Read the debug UI's clipboard.", h);
    }

    commandTable.add("mem?", "mem?",
                     "GPU memory: the backend's allocations, then mesh data grouped by name, largest first.",
                     [this](const ControlCommand&) -> std::string {
        // WHAT IS ON THE GPU: the backend's live allocations, then the meshes grouped by
        // name, largest first (AssetManager::meshBytesByPrefix). The rest of what the
        // driver reports -- render targets, shadow maps, instance and uniform buffers --
        // is the difference against nvidia-smi.
        std::string out = "ok " + rendererPtr->memoryReport();
        std::size_t total = 0;
        const auto groups = assetManager->meshBytesByPrefix();
        for (const auto& g : groups) total += g.bytes;
        char head[96];
        std::snprintf(head, sizeof(head), " || mesh data by name, %.0f MB in all:", total / 1048576.0);
        out += head;
        for (std::size_t i = 0; i < groups.size() && i < 16; ++i) {
            char b[160];
            std::snprintf(b, sizeof(b), "  %s %zu meshes %.1f MB", groups[i].prefix.c_str(), groups[i].meshes, groups[i].bytes / 1048576.0);
            out += b;
        }
        return out;
    });

    commandTable.add("keys?", "keys?",
                     "Every key binding and the mode it works in, then any real clashes.",
                     [this](const ControlCommand&) -> std::string {
        // WHAT DOES THIS KEY DO. Bindings live in a dozen systems' onStart, so
        // the only way to answer that was grep. A shared key is legitimate --
        // several here are deliberate -- but it should be readable.
        std::string out;
        for (const auto& kv : inputMap.bindingReport()) {
            out += (out.empty() ? "" : "  |  ") + kv.first + ": ";
            for (std::size_t i = 0; i < kv.second.size(); ++i)
                out += std::string(i ? ", " : "") + kv.second[i] + " [" +
                       inputContextName(inputMap.actionContext(kv.second[i])) + "]";
        }
        const std::vector<std::string> clash = inputMap.collisions();
        out += "  ||  REAL CLASHES (same mode): " +
               (clash.empty() ? std::string("none") : std::to_string(clash.size()));
        for (const std::string& c : clash) out += "  " + c;
        return "ok " + (out.empty() ? std::string("no bindings") : out);
    });

    commandTable.add("render", "render apply|save",
                     "Push the render Settings keys (ssao.*, ssr.*, shadow.*, bloom.*, grade.*) to the renderer, or save them.",
                     [this](const ControlCommand& cmd) -> std::string {
        // The ImGui panel's knob families (ssao.*, ssr.*, shadow.*, bloom.*,
        // tonemap.op, grade.*, hud.show) ride the EXISTING static
        // settings<->renderer mapping the visionOS panel uses. Flow: `set
        // ssao.radius 1.2` ... then `render apply`.
        if (cmd.args.empty()) return "err usage: render apply|save";
        if (cmd.args[0] == "apply") {
            DebugOverlaySystem::loadSettings(settingsStore, *rendererPtr);
            return "ok applied";
        }
        if (cmd.args[0] == "save") {
            DebugOverlaySystem::saveSettings(settingsStore, *rendererPtr);
            settingsStore.save("settings.json");
            return "ok saved";
        }
        return "err usage: render apply|save";
    });

    commandTable.add("ledger", "ledger start <csv> | stop | summary",
                     "The frame-time ledger: capture to CSV, stop, or a summary now.",
                     [this](const ControlCommand& cmd) -> std::string {
        // The always-on frame ledger (ADR-0077), remotely: capture to CSV for
        // tools/frame-report.py, or a one-line summary right now.
        if (cmd.args.empty()) return "err usage: ledger start <csv>|stop|summary";
        if (cmd.args[0] == "start") {
            if (cmd.args.size() < 2) return "err usage: ledger start <csv>";
            frameStats.startCapture(cmd.args[1], "control-channel capture");
            return "ok capturing " + cmd.args[1];
        }
        if (cmd.args[0] == "stop") {
            frameStats.stopCapture();
            return "ok capture stopped";
        }
        if (cmd.args[0] == "summary") {
            FrameStats::Summary s = frameStats.summarize();
            char buf[200];
            std::snprintf(buf, sizeof(buf),
                          "ok frame avg=%.2fms p95=%.2fms max=%.2fms "
                          "update=%.2f fixed=%.2f render=%.2f wait=%.2f",
                          s.avgTotalMs, s.p95TotalMs, s.maxTotalMs,
                          s.avgUpdateMs, s.avgFixedMs, s.avgRenderMs,
                          s.avgWaitMs);
            return buf;
        }
        return "err usage: ledger start <csv>|stop|summary";
    });

    // the level bundle (ADR-0084): hit/miss and how long it took
    commandTable.add("bundle?", "bundle?",
                     "The level bundle: hit or miss, how long it took, and where it lives.",
                     [](const ControlCommand&) -> std::string {
        return "ok " + engine::bundle::lastBundleStatus() + " root=" + engine::bundle::bundleRoot();
    });
}

// Events: the bus's named types (ADR-0253), from the socket
void Application::registerEventVerbs() {
    commandTable.add("events?", "events? [json]",
                     "Every named event on the bus, with its fields; `json` adds the summaries, defaults and subscribers.",
                     [this](const ControlCommand& cmd) -> std::string {
        const std::vector<const EventBus::NamedType*> types = eventBus.namedTypes();
        if (!cmd.args.empty() && cmd.args[0] == "json") {
            nlohmann::json out = nlohmann::json::array();
            for (const EventBus::NamedType* t : types)
                out.push_back({{"name", t->name}, {"summary", t->summary}, {"fields", nlohmann::json::parse(t->fields())},
                               {"subscribers", eventBus.namedSubscriberCount(t->name)}});
            return "ok " + out.dump();
        }
        std::string out = "ok " + std::to_string(types.size()) + " events:";
        for (std::size_t i = 0; i < types.size(); ++i) {
            out += (i == 0 ? " " : "; ") + types[i]->name + " {";
            const nlohmann::json fields = nlohmann::json::parse(types[i]->fields());
            bool first = true;
            for (nlohmann::json::const_iterator f = fields.begin(); f != fields.end(); ++f) {
                out += (first ? "" : ", ") + f.key();
                first = false;
            }
            out += "}";
        }
        return out;
    });

    commandTable.add("publish", "publish <Event> [{json fields}]",
                     "Publish a named event now, its fields as JSON (missing ones keep their defaults).",
                     [this](const ControlCommand& cmd) -> std::string {
        if (cmd.args.empty()) return "err usage: publish <Event> [{json fields}]";
        // The fields are the raw text after the event's name (whitespace inside a JSON string kept).
        std::string text;
        const std::size_t from = cmd.rest.find_first_not_of(" \t", cmd.args[0].size());
        if (from != std::string::npos) text = cmd.rest.substr(from);
        const std::string& name = cmd.args[0];
        std::string error;
        if (!eventBus.publishNamed(name, text, /*queued=*/false, &error)) return "err " + error;
        return "ok published " + name + " to " + std::to_string(eventBus.namedSubscriberCount(name)) + " subscribers";
    });

    commandTable.add("watch", "watch <Event> [Event ...]",
                     "Stream these events to this connection as `event <Name> {json}` lines, until unwatch or it closes.",
                     [this](const ControlCommand& cmd) -> std::string {
        if (cmd.args.empty()) return "err usage: watch <Event> [Event ...]";
        const uint64_t client = controlChannel.client();
        if (client == 0) return "err watch: no connection to stream to";
        for (const std::string& name : cmd.args)
            if (eventBus.namedType(name) == nullptr) return "err unknown event: " + name + " (try events?)";
        for (const std::string& name : cmd.args) {
            bool already = false;
            for (const EventWatch& w : eventWatches) already = already || (w.name == name && w.client == client);
            if (already) continue;
            const EventBus::SubscriptionId id = eventBus.subscribeNamed(name, [this, name, client](const std::string& f) {
                controlChannel.post(client, "event " + name + " " + f);
            });
            eventWatches.push_back({name, id, client});
        }
        int mine = 0;   // this connection's: an earlier client's are swept after the drain
        for (const EventWatch& w : eventWatches) mine += w.client == client ? 1 : 0;
        return "ok watching " + std::to_string(mine) + " events";
    });

    commandTable.add("unwatch", "unwatch [Event ...]",
                     "Stop streaming these events (all of them when none is named).",
                     [this](const ControlCommand& cmd) -> std::string {
        int stopped = 0;
        for (std::size_t i = eventWatches.size(); i-- > 0;) {
            const EventWatch& w = eventWatches[i];
            if (!cmd.args.empty() && std::find(cmd.args.begin(), cmd.args.end(), w.name) == cmd.args.end()) continue;
            eventBus.unsubscribe(w.id);
            eventWatches.erase(eventWatches.begin() + static_cast<std::ptrdiff_t>(i));
            ++stopped;
        }
        return "ok stopped " + std::to_string(stopped) + " watches";
    });
}

}  // namespace engine
