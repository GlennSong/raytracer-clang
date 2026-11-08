#include "cloud_map_system.h"

#include "../../log.h"
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

void CloudMapSystem::update(FrameContext& ctx) {
    VolumetricCloudParams& vc = ctx.view.lighting.volumetricClouds;
    if (oldTexFrames_ > 0 && --oldTexFrames_ == 0 && oldTex_.valid()) {
        ctx.renderer.removeTexture(oldTex_);
        oldTex_ = {};
    }
    if (!vc.enabled) return;
    const CloudBook& book = cloudBook();

    // THE DRIFT: the deck's own `wind` is its speed in a 6 m/s ground wind
    const double speed = static_cast<double>(vc.wind) * (std::max(ctx.renderer.windSpeed, 0.0f) / 6.0);
    const double dirX = ctx.renderer.windDir[0], dirZ = ctx.renderer.windDir[1];
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
    in.centreX = ctx.view.camera.position.x + driftX_;
    in.centreZ = ctx.view.camera.position.z + driftZ_;
    const double moved = std::hypot(in.centreX - built_.centreX, in.centreZ - built_.centreZ);
    const bool weatherMoved = std::fabs(in.coverage - built_.coverage) > 0.01f ||
                              std::fabs(in.typeLo - built_.typeLo) > 0.01f || std::fabs(in.typeHi - built_.typeHi) > 0.01f ||
                              std::fabs(in.cirrus - built_.cirrus) > 0.01f ||
                              in.seed != built_.seed;
    if (!baking_ && (!haveMap_ || moved > book.rebuildMetres ||
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
