#include "watch_system.h"

#include "../components.h"
#include "../interact_broker.h"   // projectToScreen
#include "../player_pose.h"
#include "../waypoint.h"
#include "../wrist_watch.h"

#include <algorithm>
#include <cmath>
#include <cstdio>

namespace engine {

namespace {
constexpr Real kDeg = 3.14159265358979323846 / 180.0;
constexpr float kMark[4] = {1.00f, 0.78f, 0.16f, 1.0f};   // the waypoint's gold (the map's too)

Entity thePlayer(World& world) {
    Entity out;
    world.each<Transform, CharacterController, ControlledBy>(
        [&](Entity e, Transform&, CharacterController&, ControlledBy&) { if (!out.valid()) out = e; });
    return out;
}
}  // namespace

void WatchSystem::onStart(FrameContext& ctx) {
    ctx.actions.bindButton("watch", KeyCode::Q);
    ctx.actions.bindButton("watch", GamepadButton::DpadDown);
    ctx.actions.setActionContext("watch", InputContext::OnFoot);
}

void WatchSystem::update(FrameContext& ctx) {
    const Entity player = thePlayer(ctx.world);
    if (!player.valid()) return;
    if (!ctx.world.has<WristWatch>(player)) ctx.world.add<WristWatch>(player, WristWatch{});
    WristWatch* w = ctx.world.get<WristWatch>(player);
    // A HAND FREE to raise it: the body's mode says (engine/player); the gun's support hand comes off to look
    const PlayerPose* pose = ctx.world.get<PlayerPose>(player);
    const bool free = pose && pose->mode != PlayerPose::Mode::Hidden && pose->handsFree >= 1 &&
                      !ctx.world.has<InVehicle>(player);
    const Real want = free && ctx.actions.held("watch") ? 1.0 : 0.0;
    const Real dt = std::clamp(ctx.frameDelta, 0.0, 0.1);
    w->raise += (want - w->raise) * (1.0 - std::exp(-dt * 9.0));
    if (w->raise < 0.002) w->raise = 0;
}

std::string WatchSystem::distanceText(Real metres) {
    char buf[32];
    if (metres < 995.0) std::snprintf(buf, sizeof(buf), "%.0f m", std::max(Real(0), metres));
    else if (metres < 9950.0) std::snprintf(buf, sizeof(buf), "%.1f km", metres / 1000.0);
    else std::snprintf(buf, sizeof(buf), "%.0f km", metres / 1000.0);
    return buf;
}

std::string WatchSystem::clockText(Real hours) {
    const int minutes = static_cast<int>(std::floor(std::fmod(std::fmod(hours, 24.0) + 24.0, 24.0) * 60.0));
    char buf[16];
    std::snprintf(buf, sizeof(buf), "%02d:%02d", minutes / 60, minutes % 60);
    return buf;
}

void WatchSystem::drawDial(Renderer& r, std::vector<Renderer::UiQuad>& out, float size, const Face& f) {
    const float c = size * 0.5f, s = size / 200.0f;   // laid out on a 200 px dial
    const float bezel[4] = {0.16f, 0.16f, 0.18f, 1.0f}, glass[4] = {0.02f, 0.025f, 0.035f, 1.0f};
    const float tick[4] = {0.55f, 0.58f, 0.62f, 1.0f}, white[4] = {0.95f, 0.96f, 0.98f, 1.0f};
    const float north[4] = {0.95f, 0.32f, 0.25f, 1.0f}, dim[4] = {0.6f, 0.63f, 0.67f, 1.0f};
    ui_.disc(r, out, c, c, 100.0f * s, bezel);
    ui_.disc(r, out, c, c, 93.0f * s, glass);
    // a point on the dial at `deg` (the world's bearing, clockwise from north) and radius `rad`: up is ahead
    auto at = [&](Real deg, float rad, float& x, float& y) {
        const Real a = (deg - f.headingDeg) * kDeg;
        x = c + static_cast<float>(std::sin(a)) * rad * s;
        y = c - static_cast<float>(std::cos(a)) * rad * s;
    };
    // THE COMPASS: a tick every 30 degrees, the four points lettered, north red
    for (int k = 0; k < 12; ++k) {
        float x, y;
        at(k * 30.0, 84.0f, x, y);
        ui_.disc(r, out, x, y, (k % 3 ? 2.2f : 3.4f) * s, tick);
    }
    static const char* points[4] = {"N", "E", "S", "W"};
    for (int k = 0; k < 4; ++k) {
        float x, y;
        at(k * 90.0, 66.0f, x, y);
        const float fs = 22.0f * s;
        const float w = ui_.measure(points[k], fs), asc = ui_.ascent(fs);
        ui_.text(r, out, x - w * 0.5f, y + asc * 0.36f, points[k], fs, k == 0 ? north : white);
    }
    // ahead: a small notch at 12 o'clock
    UiText::rect(out, c - 1.5f * s, c - 97.0f * s, c + 1.5f * s, c - 88.0f * s, white);
    // THE MIDDLE: how far the waypoint is (with the time under it), or the time
    auto centred = [&](const std::string& t, float fs, float baseline, const float* col) {
        ui_.text(r, out, c - ui_.measure(t, fs) * 0.5f, c + baseline * s, t, fs, col);
    };
    if (f.distance >= 0) {
        float x, y;
        at(f.bearingDeg, 84.0f, x, y);
        const float shade[4] = {0, 0, 0, 0.9f};
        ui_.disc(r, out, x, y, 10.0f * s, shade);
        ui_.disc(r, out, x, y, 7.5f * s, kMark);
        centred(f.distance < 12.0 ? std::string("here") : distanceText(f.distance), 30.0f * s, 10.0f, kMark);
        if (f.clockHours >= 0) centred(clockText(f.clockHours), 15.0f * s, 34.0f, dim);
    } else if (f.clockHours >= 0) {
        centred(clockText(f.clockHours), 32.0f * s, 11.0f, white);
    }
}

void WatchSystem::render(FrameContext& ctx) {
    const Entity player = thePlayer(ctx.world);
    const WristWatch* w = player.valid() ? ctx.world.get<WristWatch>(player) : nullptr;
    if (!w || w->raise < 0.6 || ctx.actions.layerUp()) return;
    const Transform* t = ctx.world.get<Transform>(player);
    if (!t) return;
    // what it shows
    const CameraState& cam = ctx.view.camera;
    Vec3 ahead = cam.target - cam.position;
    ahead.y = 0;
    Face f;
    f.headingDeg = ahead.lengthSquared() > 1e-10 ? std::atan2(ahead.x, -ahead.z) / kDeg : 0.0;
    f.clockHours = ctx.view.lighting.clockHours;
    if (const Waypoint* wp = ctx.world.get<Waypoint>(player)) {
        const Real dx = wp->at.x - t->position.x, dz = wp->at.z - t->position.z;
        f.distance = std::sqrt(dx * dx + dz * dz);
        f.bearingDeg = std::atan2(dx, -dz) / kDeg;
    }
    // drawn in its own square, then each quad's corners taken onto the screen
    const float W = static_cast<float>(ctx.framebufferWidth), H = static_cast<float>(ctx.framebufferHeight);
    const float k = ctx.windowWidth > 0 ? W / static_cast<float>(ctx.windowWidth) : 1.0f;
    const float D = 200.0f;
    std::vector<Renderer::UiQuad> dial;
    std::vector<Renderer::UiQuad> q;
    const bool third = ctx.settings.getBool("playerThirdPerson", false);
    if (!third && w->faceShown) {
        // FIRST PERSON: on the face, through the camera (a lit screen on the wrist)
        drawDial(ctx.renderer, dial, D, f);
        for (Renderer::UiQuad d : dial) {
            bool ok = true;
            for (int i = 0; i < 4 && ok; ++i) {
                const Real u = (d.x[i] / D) * 2.0 - 1.0, v = 1.0 - (d.y[i] / D) * 2.0;
                const Vec3 p = w->centre + w->right * (u * w->radius) + w->up * (v * w->radius);
                Real sx = 0, sy = 0;
                ok = projectToScreen(p, cam.position, cam.target, cam.up, cam.fovDegrees, cam.aspectRatio, W, H, sx, sy);
                d.x[i] = static_cast<float>(sx);
                d.y[i] = static_cast<float>(sy);
            }
            if (ok) q.push_back(d);
        }
    } else if (third) {
        // THIRD PERSON: the same dial by the character -- left of the body as drawn, kept on the screen
        const float size = 190.0f * k;
        drawDial(ctx.renderer, dial, size, f);
        Real sx = W * 0.5f, sy = H * 0.5f;
        projectToScreen(t->position + Vec3(0, 0.3, 0), cam.position, cam.target, cam.up, cam.fovDegrees,
                        cam.aspectRatio, W, H, sx, sy);
        const float x0 = std::clamp(static_cast<float>(sx) - size - 90.0f * k, 16.0f * k, W - size - 16.0f * k);
        const float y0 = std::clamp(static_cast<float>(sy) - size * 0.5f, 16.0f * k, H - size - 16.0f * k);
        for (Renderer::UiQuad d : dial) {
            for (int i = 0; i < 4; ++i) {
                d.x[i] += x0;
                d.y[i] += y0;
            }
            q.push_back(d);
        }
    }
    if (!q.empty()) ctx.renderer.submitUi(q);
}

}  // namespace engine
