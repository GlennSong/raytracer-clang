#include "test_framework.h"

#include "../src/engine/component_registry.h"
#include "../src/engine/property_json.h"
#include "../src/engine/components.h"
#include "../src/engine/camera/scene_camera.h"
#include <nlohmann/json.hpp>

using namespace engine;  // namespace migration (ADR-0015)
using json = nlohmann::json;

// The property layer (editor-app plan): components describe their fields once;
// inspectors and serializers are visitors over that single description.

namespace {
// Counts what a describe walk reports — the cheapest coverage check.
struct CountVisitor : PropertyVisitor {
    int fields = 0;
    void field(const FieldMeta&, Real&) override { fields++; }
    void field(const FieldMeta&, float&) override { fields++; }
    void field(const FieldMeta&, Vec3&) override { fields++; }
    void field(const FieldMeta&, bool&) override { fields++; }
    void field(const FieldMeta&, std::string&) override { fields++; }
    void bitFlag(const FieldMeta&, uint32_t&, uint32_t) override { fields++; }
};
}  // namespace

TEST_CASE(properties_json_round_trips_a_lens) {
    LensParams lens;
    lens.focalLength = 35.0;
    lens.fStop = 2.8;
    lens.vignette = 0.4;

    json j;
    JsonWriteVisitor writer(j);
    describeProperties(lens, writer);
    CHECK_APPROX(j["Focal Length"].get<double>(), 35.0, 1e-9);
    CHECK_APPROX(j["f-stop"].get<double>(), 2.8, 1e-9);

    LensParams restored;   // defaults differ from lens
    JsonReadVisitor reader(j);
    describeProperties(restored, reader);
    CHECK_APPROX(restored.focalLength, 35.0, 1e-9);
    CHECK_APPROX(restored.fStop, 2.8, 1e-9);
    CHECK_APPROX(restored.vignette, 0.4, 1e-9);
}

TEST_CASE(properties_json_handles_colors_flags_and_readonly) {
    RenderMaterial mat;
    mat.albedo = Vec3(0.9, 0.1, 0.2);
    mat.flags = RenderMaterial::FLAG_CHECKERBOARD;

    json j;
    JsonWriteVisitor writer(j);
    describeProperties(mat, writer);
    CHECK(j["Checkerboard"].get<bool>());
    CHECK_APPROX(j["Albedo"][0].get<double>(), 0.9, 1e-6);

    RenderMaterial restored;
    JsonReadVisitor reader(j);
    describeProperties(restored, reader);
    CHECK((restored.flags & RenderMaterial::FLAG_CHECKERBOARD) != 0);
    CHECK_APPROX(restored.albedo.x, 0.9, 1e-6);

    // Read-only fields are written out (display/diff value) but never read
    // back — they're derived or owned elsewhere.
    SourceSpec spec;
    spec.shape = "box";
    json sj;
    JsonWriteVisitor specWriter(sj);
    describeProperties(spec, specWriter);
    sj["Shape"] = "torus";   // hostile/hand-edited input
    SourceSpec specRestored;
    JsonReadVisitor specReader(sj);
    describeProperties(specRestored, specReader);
    CHECK(specRestored.shape == "box");   // locked field untouched
}

TEST_CASE(component_registry_enumerates_an_entity) {
    World world;
    Entity e = world.create();
    world.add<Transform>(e);
    world.add<Renderable>(e);
    SceneCamera cam;
    cam.name = "Crane";
    world.add<SceneCamera>(e, cam);

    ComponentRegistry registry;
    registerEngineComponents(registry);

    int present = 0;
    int totalFields = 0;
    for (const auto& entry : registry.entries()) {
        if (!entry.has(world, e)) continue;
        present++;
        CountVisitor counter;
        entry.visit(world, e, counter);
        totalFields += counter.fields;
    }
    CHECK(present == 3);        // Transform, Material, Camera
    CHECK(totalFields > 10);    // fields actually walked, not empty stubs

    // The walk reaches the live component: edit through a visit, observe it.
    for (const auto& entry : registry.entries()) {
        if (entry.name != "Camera" || !entry.has(world, e)) continue;
        json j;
        JsonWriteVisitor writer(j);
        entry.visit(world, e, writer);
        CHECK(j["Name"] == "Crane");
    }
}
