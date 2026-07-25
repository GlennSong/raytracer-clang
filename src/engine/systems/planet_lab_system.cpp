#include "planet_lab_system.h"

#include "../components.h"
#include "../asset_manager.h"
#include "../procgen/planet.h"
#include "../../renderer/renderer.h"
#include "../../renderer/settings.h"
#include "../../rt_math.h"

#include <cstdint>

#ifdef RT_ENABLE_IMGUI
#include <imgui.h>
#endif

namespace engine {

void PlanetLabSystem::rebuild(FrameContext& ctx) {
    // Build the mesh from the current UI state (preset + slider overrides).
    RenderMesh mesh;
    if (mode_ == 0) {
        PlanetParams p = rockyPreset_ == 1 ? planetMars()
                       : rockyPreset_ == 2 ? planetMoon() : planetEarthlike();
        p.radius = radius_;
        p.faceRes = faceRes_;
        p.seaLevel = seaLevel_;
        p.hasOcean = hasOcean_;
        p.continentAmp = continentAmp_;
        p.mountainAmp = mountainAmp_;
        p.craterAmp = craterAmp_;
        p.reliefFraction = relief_;
        mesh = generatePlanet(p, static_cast<uint32_t>(seed_));
    } else {
        GasGiantParams p = gasPreset_ == 1 ? gasGiantNeptune() : gasGiantJupiter();
        p.radius = radius_;
        p.faceRes = faceRes_;
        p.bandFreq = bandFreq_;
        p.flowWarp = flowWarp_;
        p.detailAmp = detailAmp_;
        p.stormCount = stormCount_;
        mesh = generateGasGiantColoredMesh(p, static_cast<uint32_t>(seed_));
    }
    if (mesh.vertices.empty()) return;

    MeshHandle handle = ctx.assets.acquireMesh(mesh);   // one-off (unshared) upload

    if (!haveEntity_) {
        // Spawn in front of the camera so it lands in view wherever you are.
        const CameraState& cam = ctx.view.camera;
        Vec3 fwd = cam.target - cam.position;
        fwd = fwd.lengthSquared() > 1e-9 ? normalize(fwd) : Vec3(0, 0, -1);
        Transform t;
        t.position = cam.position + fwd * (radius_ * 3.0);

        planet_ = ctx.world.create();
        ctx.world.add<Transform>(planet_, t);
        ctx.world.add<PrevTransform>(planet_, PrevTransform{t});
        Renderable r;
        r.material.albedo = Vec3(1, 1, 1);   // hue is carried in the vertex colour
        r.material.roughness = 0.9;
        r.material.metallic = 0.0;
        r.mesh = handle;
        ctx.world.add<Renderable>(planet_, r);
        haveEntity_ = true;
    } else if (Renderable* r = ctx.world.get<Renderable>(planet_)) {
        MeshHandle old = r->mesh;
        r->mesh = handle;
        ctx.assets.releaseMesh(old);         // free the previous mesh (refcount 0)
    }

    // Axial spin (MotionSystem integrates Velocity.angular on non-physics bodies).
    Vec3 spin = spin_ ? Vec3(0, 0.15, 0) : Vec3();
    if (Velocity* v = ctx.world.get<Velocity>(planet_)) v->angular = spin;
    else ctx.world.add<Velocity>(planet_, Velocity{Vec3(), spin});
}

void PlanetLabSystem::removePlanet(FrameContext& ctx) {
    if (!haveEntity_) return;
    if (Renderable* r = ctx.world.get<Renderable>(planet_)) ctx.assets.releaseMesh(r->mesh);
    ctx.world.destroy(planet_);
    haveEntity_ = false;
}

bool PlanetLabSystem::sceneEnabled(FrameContext& ctx) const {
    bool enabled = false;
    ctx.world.each<ScenePlanetLab>([&](Entity, ScenePlanetLab&) { enabled = true; });
    return enabled;
}

// Web front-end path: the browser has no ImGui, so its HTML panel writes params
// into Settings and bumps "planetLab.gen" through rt_web_planet* (web_main.cpp).
// Regenerate when that counter changes. On native nothing sets these keys, so this
// is inert and the ImGui panel in render() drives everything instead.
void PlanetLabSystem::update(FrameContext& ctx) {
    if (!sceneEnabled(ctx)) return;
    Settings& s = ctx.settings;
    int gen = static_cast<int>(s.getDouble("planetLab.gen", 0));
    if (gen == lastGen_) return;
    if (lastGen_ < 0) { lastGen_ = gen; return; }   // sync on first observe; don't fire
    lastGen_ = gen;

    mode_ = static_cast<int>(s.getDouble("planetLab.mode", mode_));
    seed_ = static_cast<int>(s.getDouble("planetLab.seed", seed_));
    radius_ = static_cast<float>(s.getDouble("planetLab.radius", radius_));
    faceRes_ = static_cast<int>(s.getDouble("planetLab.faceRes", faceRes_));
    rockyPreset_ = static_cast<int>(s.getDouble("planetLab.rockyPreset", rockyPreset_));
    gasPreset_ = static_cast<int>(s.getDouble("planetLab.gasPreset", gasPreset_));
    seaLevel_ = static_cast<float>(s.getDouble("planetLab.seaLevel", seaLevel_));
    hasOcean_ = s.getDouble("planetLab.hasOcean", hasOcean_ ? 1.0 : 0.0) != 0.0;
    continentAmp_ = static_cast<float>(s.getDouble("planetLab.continentAmp", continentAmp_));
    mountainAmp_ = static_cast<float>(s.getDouble("planetLab.mountainAmp", mountainAmp_));
    craterAmp_ = static_cast<float>(s.getDouble("planetLab.craterAmp", craterAmp_));
    relief_ = static_cast<float>(s.getDouble("planetLab.relief", relief_));
    bandFreq_ = static_cast<float>(s.getDouble("planetLab.bandFreq", bandFreq_));
    flowWarp_ = static_cast<float>(s.getDouble("planetLab.flowWarp", flowWarp_));
    detailAmp_ = static_cast<float>(s.getDouble("planetLab.detailAmp", detailAmp_));
    stormCount_ = static_cast<int>(s.getDouble("planetLab.stormCount", stormCount_));
    spin_ = s.getDouble("planetLab.spin", spin_ ? 1.0 : 0.0) != 0.0;
    rebuild(ctx);
}

void PlanetLabSystem::render(FrameContext& ctx) {
#ifdef RT_ENABLE_IMGUI
    if (ImGui::GetCurrentContext() == nullptr) return;
    if (!sceneEnabled(ctx)) return;   // scene-scoped: only a level that opted in

    // Its own floating, draggable window (a "sidebar" seeded top-left on first use).
    ImGui::SetNextWindowPos(ImVec2(20, 20), ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowSize(ImVec2(320, 0), ImGuiCond_FirstUseEver);   // 0 = auto-fit
    if (ImGui::Begin("Planet Lab")) {
        ImGui::Combo("Type", &mode_, "Rocky\0Gas Giant\0");

        if (mode_ == 0) ImGui::Combo("Preset", &rockyPreset_, "Earth\0Mars\0Moon\0");
        else            ImGui::Combo("Preset", &gasPreset_, "Jupiter\0Neptune\0");

        ImGui::InputInt("Seed", &seed_);
        ImGui::SameLine();
        if (ImGui::Button("Random"))            // deterministic LCG shuffle of the seed
            seed_ = static_cast<int>((static_cast<uint32_t>(seed_) * 1103515245u + 12345u)
                                     & 0x7fffffffu);

        ImGui::SliderFloat("Radius", &radius_, 5.0f, 40.0f);
        ImGui::SliderInt("Detail", &faceRes_, 16, 160);

        if (mode_ == 0) {
            ImGui::SeparatorText("Rocky");
            ImGui::SliderFloat("Sea Level", &seaLevel_, -0.3f, 0.4f);
            ImGui::Checkbox("Ocean", &hasOcean_);
            ImGui::SliderFloat("Continents", &continentAmp_, 0.2f, 2.0f);
            ImGui::SliderFloat("Mountains", &mountainAmp_, 0.0f, 1.5f);
            ImGui::SliderFloat("Craters", &craterAmp_, 0.0f, 0.6f);
            ImGui::SliderFloat("Relief", &relief_, 0.005f, 0.12f, "%.3f");
        } else {
            ImGui::SeparatorText("Gas Giant");
            ImGui::SliderFloat("Bands", &bandFreq_, 3.0f, 30.0f);
            ImGui::SliderFloat("Turbulence", &flowWarp_, 0.0f, 1.0f);
            ImGui::SliderFloat("Cloud Detail", &detailAmp_, 0.0f, 0.5f);
            ImGui::SliderInt("Storms", &stormCount_, 0, 8);
        }

        ImGui::Checkbox("Spin", &spin_);
        ImGui::Separator();

        if (ImGui::Button(haveEntity_ ? "Regenerate" : "Generate Planet"))
            rebuild(ctx);
        if (haveEntity_) {
            ImGui::SameLine();
            if (ImGui::Button("Remove")) removePlanet(ctx);
            ImGui::TextDisabled("Editing the live planet — tweak and Regenerate.");
        }
    }
    ImGui::End();
#else
    (void)ctx;
#endif
}

}  // namespace engine
