#include "properties.h"

#include "components.h"
#include "camera/scene_camera.h"

namespace engine {

// Serialization ids (FieldMeta::id) follow the level/camera JSON formats —
// the JSON visitors ARE those formats now, so the ids must match what the
// loaders historically read ("focalLength", "albedo", "near", ...).

void describeProperties(Transform& t, PropertyVisitor& v) {
    v.field(FieldMeta("Position").id("position").increment(0.1), t.position);
    v.field(FieldMeta("Scale").id("scale").range(0.01, 1000.0).increment(0.05),
            t.scale);
    // Orientation is deliberately absent: the gizmo and yaw/pitch tools own
    // rotation; a raw quaternion in a panel helps no one.
}

void describeProperties(RenderMaterial& m, PropertyVisitor& v) {
    v.field(FieldMeta("Albedo").id("albedo").asColor(), m.albedo);
    v.field(FieldMeta("Metallic").id("metallic").range(0.0, 1.0).increment(0.01),
            m.metallic);
    v.field(FieldMeta("Roughness").id("roughness").range(0.02, 1.0).increment(0.01),
            m.roughness);
    v.field(FieldMeta("Opacity").id("opacity").range(0.0, 1.0).increment(0.01),
            m.opacity);
    v.field(FieldMeta("Emission").id("emission").asColor(), m.emission);
    v.bitFlag(FieldMeta("Checkerboard").id("checkerboard"), m.flags,
              RenderMaterial::FLAG_CHECKERBOARD);
}

void describeProperties(LensParams& lens, PropertyVisitor& v) {
    v.field(FieldMeta("Focal Length").id("focalLength")
                .range(8.0, 300.0).log().units("mm"),
            lens.focalLength);
    v.field(FieldMeta("Sensor Height").id("sensorHeight")
                .range(4.0, 70.0).units("mm"),
            lens.sensorHeight);
    v.field(FieldMeta("f-stop").id("fStop").range(0.7, 22.0).log(), lens.fStop);
    v.field(FieldMeta("Focus Distance").id("focusDistance")
                .range(0.1, 1000.0).units("m"),
            lens.focusDistance);
    v.field(FieldMeta("Near Plane").id("near").range(0.01, 10.0).increment(0.01),
            lens.nearPlane);
    v.field(FieldMeta("Far Plane").id("far").range(10.0, 10000.0).increment(10.0),
            lens.farPlane);
    v.field(FieldMeta("Distortion K1").id("k1").range(-0.5, 0.5).increment(0.01),
            lens.distortionK1);
    v.field(FieldMeta("Distortion K2").id("k2").range(-0.5, 0.5).increment(0.01),
            lens.distortionK2);
    v.field(FieldMeta("Chromatic Ab.").id("chromaticAberration")
                .range(0.0, 0.05).increment(0.001),
            lens.chromaticAberration);
    v.field(FieldMeta("Vignette").id("vignette").range(0.0, 1.0).increment(0.01),
            lens.vignette);
}

void describeProperties(SceneCamera& cam, PropertyVisitor& v) {
    v.field(FieldMeta("Name").id("name"), cam.name);
    describeProperties(cam.lens, v);
}

void describeProperties(SourceSpec& spec, PropertyVisitor& v) {
    v.field(FieldMeta("Name").id("name"), spec.name);
    if (!spec.meshFile.empty()) {
        v.field(FieldMeta("Mesh").id("mesh").locked(), spec.meshFile);
    } else {
        v.field(FieldMeta("Shape").id("shape").locked(), spec.shape);
        // Editable since the registry grew post-edit hooks: the editor installs
        // a hook on this component that rebuilds the mesh after a Size write.
        v.field(FieldMeta("Size").id("size").range(0.05, 100.0).increment(0.05),
                spec.size);
    }
    v.field(FieldMeta("Physics").id("hasPhysics"), spec.hasPhysics);
    v.field(FieldMeta("Motion").id("motion")
                .options({"static", "dynamic", "kinematic"}),
            spec.motion);
    v.field(FieldMeta("Friction").id("friction").range(0.0, 2.0).increment(0.05),
            spec.friction);
    v.field(FieldMeta("Restitution").id("restitution")
                .range(0.0, 1.0).increment(0.05),
            spec.restitution);
}

void describeProperties(Velocity& vel, PropertyVisitor& v) {
    // Runtime motion state — interesting to WATCH during a playtest
    // (observer mode shows it live); document entities never carry it.
    v.field(FieldMeta("Linear").id("linear").increment(0.1), vel.linear);
    v.field(FieldMeta("Angular").id("angular").increment(0.1), vel.angular);
}

}  // namespace engine
