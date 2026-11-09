#include "cloud_map_system.h"

#include "../../log.h"
#include <tinygltf/stb_image_write.h>   // (the implementation is model_importer's)
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>

namespace engine {

namespace {
// RT_WIND_TIME=<s> pins the drift as it pins the renderer's sway (the cloud lab: every run's clouds in one place)
double pinnedWindTime() {
    static const double t = [] { const char* e = std::getenv("RT_WIND_TIME"); return e ? std::atof(e) : -1.0; }();
    return t;
}
constexpr double kMinSecondsBetweenBakes = 1.5;
constexpr double kMinSecondsForField = 10.0;   // (a new simulated weather: the sky moves slowly, a bake is seconds)
constexpr int kOldTexFrames = 4;   // past the frames in flight
}  // namespace

void CloudMapSystem::startBake(const CloudMapInputs& in) {
    const CloudBook book = cloudBook();
    pending_ = std::async(std::launch::async, [book, in] {
        const auto t0 = std::chrono::steady_clock::now();
        CloudMapImage m;
        bakeCloudMap(book, in, m);
        m.bakeMs = std::chrono::duration<float, std::milli>(std::chrono::steady_clock::now() - t0).count();
        return m;
    });
    baking_ = true;
    sinceBake_ = 0.0;
}

void CloudMapSystem::onStart(FrameContext& ctx) {
    if (ctx.commands != nullptr)
        pictureCmd_ = ctx.commands->bind("cloudmap", "CloudMapSystem", [this](const ControlCommand& cmd) -> std::string {
            if (cmd.args.empty()) return "err cloudmap <path.png>";
            picturePath_ = cmd.args[0];
            return "ok staged " + picturePath_;
        });
}

// THE MAP AS A PICTURE (`cloudmap`): from above, north up (-z), each texel a cloud's footprint over the sea-blue of
// clear air, tinted by its type -- stratus grey, stratocumulus pale, cumulus white, cumulonimbus red -- the cirrus
// a faint blue wash, the camera a red dot
void CloudMapSystem::writePicture(const std::string& path, const Vec3& camera) const {
    const int n = last_.size;
    std::vector<uint8_t> img(static_cast<std::size_t>(n) * n * 3);
    auto put = [&](int i, int j, const Vec3& c) {
        uint8_t* q = &img[(static_cast<std::size_t>(j) * n + i) * 3];
        q[0] = static_cast<uint8_t>(std::lround(std::clamp(c.x, 0.0, 1.0) * 255.0));
        q[1] = static_cast<uint8_t>(std::lround(std::clamp(c.y, 0.0, 1.0) * 255.0));
        q[2] = static_cast<uint8_t>(std::lround(std::clamp(c.z, 0.0, 1.0) * 255.0));
    };
    const Vec3 tints[4] = {Vec3(0.62, 0.62, 0.66), Vec3(0.85, 0.85, 0.80), Vec3(1.0, 1.0, 1.0), Vec3(1.0, 0.55, 0.50)};
    for (int j = 0; j < n; ++j)
        for (int i = 0; i < n; ++i) {
            const uint8_t* p = &last_.rgba[(static_cast<std::size_t>(j) * n + i) * 4];
            const double f = p[0] / 255.0, slot = p[1] / 255.0 * 3.0, cirrus = p[3] / 255.0;
            const int a = std::min(static_cast<int>(slot), 2);
            const Vec3 tint = tints[a] * (1.0 - (slot - a)) + tints[a + 1] * (slot - a);
            // (the ground under clear air, where the map was baked with the simulation: land green, the sea blue --
            // as the satellite references read)
            Vec3 ground(0.12, 0.25, 0.48);
            if (last_.inputs.field) {
                const double x = last_.originX + (i + 0.5) * last_.extent / n - last_.inputs.driftX;
                const double z = last_.originZ + (j + 0.5) * last_.extent / n - last_.inputs.driftZ;
                if (last_.inputs.field->landAt(x, z)) ground = Vec3(0.22, 0.30, 0.14);
            }
            const Vec3 air = ground + Vec3(0.25, 0.3, 0.4) * cirrus;
            put(i, j, air * (1.0 - f) + tint * f);
        }
    // the camera, in the air
    const double texel = last_.extent / n;
    const int ci = static_cast<int>((camera.x + driftX_ - last_.originX) / texel);
    const int cj = static_cast<int>((camera.z + driftZ_ - last_.originZ) / texel);
    for (int dj = -3; dj <= 3; ++dj)
        for (int di = -3; di <= 3; ++di)
            if (ci + di >= 0 && cj + dj >= 0 && ci + di < n && cj + dj < n) put(ci + di, cj + dj, Vec3(1.0, 0.1, 0.1));
    if (!stbi_write_png(path.c_str(), n, n, 3, img.data(), n * 3)) LOG_WARN << "[clouds] could not write " << path;
}

void CloudMapSystem::update(FrameContext& ctx) {
    VolumetricCloudParams& vc = ctx.view.lighting.volumetricClouds;
    if (oldTexFrames_ > 0 && --oldTexFrames_ == 0 && oldTex_.valid()) {
        ctx.renderer.removeTexture(oldTex_);
        oldTex_ = {};
    }
    if (!vc.enabled) return;
    const CloudBook& book = cloudBook();
    if (!picturePath_.empty() && haveMap_) {
        writePicture(picturePath_, ctx.view.camera.position);
        LOG_INFO << "[clouds] the map written to " << picturePath_;
        picturePath_.clear();
    }

    // THE DRIFT: the deck's own `wind` is its speed in a 6 m/s ground wind. Every reader takes the air at a world
    // point as the world PLUS the drift (the march, the map's bake, the shadow), so a cloud fixed in the air stands
    // over the world at the air MINUS the drift: for it to go where the wind goes (windDir: where the air goes), the
    // drift runs against the wind. (Until 2026-11-09 it ran with it, and every cloud sailed upwind -- since #59.)
    const double speed = static_cast<double>(vc.wind) * (std::max(ctx.renderer.windSpeed, 0.0f) / 6.0);
    const double dirX = -ctx.renderer.windDir[0], dirZ = -ctx.renderer.windDir[1];
    if (pinnedWindTime() >= 0.0) {
        driftX_ = dirX * speed * pinnedWindTime();
        driftZ_ = dirZ * speed * pinnedWindTime();
    } else {
        const double dt = std::clamp(ctx.frameDelta, 0.0, 0.25);
        driftX_ += dirX * speed * dt;
        driftZ_ += dirZ * speed * dt;
    }
    vc.driftX = driftX_;
    vc.driftZ = driftZ_;
    if (!book.loaded) return;   // (no clouds.lua: the noise places the clouds, as before)

    // THE TYPES, and the deck's span: the lowest base to the highest top of the types the weather spreads over
    const std::array<float, 4> w = cloudTypeWeights(vc.typeLo, vc.typeHi);
    float lo = 1e9f, hi = -1e9f;
    for (std::size_t k = 0; k < 4; ++k) {
        const CloudTypeSpec& t = book.types[k];
        VolumetricCloudParams::CloudType& ct = vc.types[k];
        for (int i = 0; i < 4; ++i) ct.profile[i] = t.profile[static_cast<std::size_t>(i)];
        ct.density = t.density;
        ct.erosion = t.erosion;
        ct.anvil = t.anvil;
        ct.base = t.base;
        ct.top = t.top;
        if (w[k] > 0.0f) { lo = std::min(lo, t.base); hi = std::max(hi, t.top); }
    }
    if (hi > lo) { vc.bottom = lo; vc.top = hi; }
    vc.shapeScale = book.shapeScale;
    vc.shadow = book.shadow;
    cloudCirrusSweep(static_cast<uint32_t>(ctx.settings.getDouble("weather.seed", 7.0)), vc.cirrusSweepX, vc.cirrusSweepZ);

    // THE MAP: handed over when baked; rebaked when the camera has drifted far through the air, or the weather moved
    sinceBake_ += ctx.frameDelta;
    if (baking_ && pending_.wait_for(std::chrono::seconds(0)) == std::future_status::ready) {
        CloudMapImage m = pending_.get();
        baking_ = false;
        const TextureHandle t = ctx.renderer.uploadTexture(m.size, m.size, 4, m.rgba.data());
        if (t.valid()) {
            ctx.renderer.setCloudMap(t, m.originX, m.originZ, m.extent);
            if (oldTex_.valid()) ctx.renderer.removeTexture(oldTex_);
            oldTex_ = tex_;
            oldTexFrames_ = kOldTexFrames;
            tex_ = t;
            built_ = m.inputs;
            haveMap_ = true;
            last_ = m;
            LOG_INFO << "[clouds] map baked in " << m.bakeMs << " ms (coverage " << m.inputs.coverage << ", types "
                     << m.inputs.typeLo << ".." << m.inputs.typeHi << ")";
        }
    }
    CloudMapInputs in;
    in.coverage = vc.coverage;
    in.typeLo = vc.typeLo;
    in.typeHi = vc.typeHi;
    in.cirrus = vc.cirrus * vc.wisps > 0.0f ? vc.cirrus : 0.0f;
    in.seed = static_cast<uint32_t>(ctx.settings.getDouble("weather.seed", 7.0));
    ctx.world.each<WeatherField>([&](Entity, WeatherField& f) {
        in.field = f.sim;
        in.fieldVersion = f.version;
    });
    in.driftX = driftX_;
    in.driftZ = driftZ_;
    in.centreX = ctx.view.camera.position.x + driftX_;
    in.centreZ = ctx.view.camera.position.z + driftZ_;
    const double moved = std::hypot(in.centreX - built_.centreX, in.centreZ - built_.centreZ);
    const bool weatherMoved = std::fabs(in.coverage - built_.coverage) > 0.01f ||
                              std::fabs(in.typeLo - built_.typeLo) > 0.01f || std::fabs(in.typeHi - built_.typeHi) > 0.01f ||
                              std::fabs(in.cirrus - built_.cirrus) > 0.01f ||
                              in.seed != built_.seed;
    const bool fieldMoved = in.fieldVersion != built_.fieldVersion && sinceBake_ > kMinSecondsForField;
    if (!baking_ && (!haveMap_ || moved > book.rebuildMetres || fieldMoved ||
                     (weatherMoved && sinceBake_ > kMinSecondsBetweenBakes)))
        startBake(in);
}

void CloudMapSystem::onStop(FrameContext& ctx) {
    if (baking_) pending_.wait();
    baking_ = false;
    ctx.renderer.setCloudMap(TextureHandle{}, 0.0, 0.0, 0.0f);
    if (tex_.valid()) ctx.renderer.removeTexture(tex_);
    if (oldTex_.valid()) ctx.renderer.removeTexture(oldTex_);
    tex_ = oldTex_ = {};
    haveMap_ = false;
}

}  // namespace engine
