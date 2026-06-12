#include "properties.h"

#include "components.h"
#include "camera/scene_camera.h"

namespace engine {

void describeProperties(Transform& t, PropertyVisitor& v) {
    v.field(FieldMeta("Position").increment(0.1), t.position);
    v.field(FieldMeta("Scale").range(0.01, 1000.0).increment(0.05), t.scale);
    // Orientation is deliberately absent: the gizmo and yaw/pitch tools own
    // rotation; a raw quaternion in a panel helps no one.
}

void describeProperties(RenderMaterial& m, PropertyVisitor& v) {
    v.field(FieldMeta("Albedo").asColor(), m.albedo);
    v.field(FieldMeta("Metallic").range(0.0, 1.0).increment(0.01), m.metallic);
    v.field(FieldMeta("Roughness").range(0.02, 1.0).increment(0.01), m.roughness);
    v.field(FieldMeta("Opacity").range(0.0, 1.0).increment(0.01), m.opacity);
    v.field(FieldMeta("Emission").asColor(), m.emission);
    v.bitFlag(FieldMeta("Checkerboard"), m.flags,
              RenderMaterial::FLAG_CHECKERBOARD);
}

void describeProperties(LensParams& lens, PropertyVisitor& v) {
    v.field(FieldMeta("Focal Length").range(8.0, 300.0).log().units("mm"),
            lens.focalLength);
    v.field(FieldMeta("Sensor Height").range(4.0, 70.0).units("mm"),
            lens.sensorHeight);
    v.field(FieldMeta("f-stop").range(0.7, 22.0).log(), lens.fStop);
    v.field(FieldMeta("Focus Distance").range(0.1, 1000.0).units("m"),
            lens.focusDistance);
    v.field(FieldMeta("Near Plane").range(0.01, 10.0).increment(0.01),
            lens.nearPlane);
    v.field(FieldMeta("Far Plane").range(10.0, 10000.0).increment(10.0),
            lens.farPlane);
    v.field(FieldMeta("Distortion K1").range(-0.5, 0.5).increment(0.01),
            lens.distortionK1);
    v.field(FieldMeta("Distortion K2").range(-0.5, 0.5).increment(0.01),
            lens.distortionK2);
    v.field(FieldMeta("Chromatic Ab.").range(0.0, 0.05).increment(0.001),
            lens.chromaticAberration);
    v.field(FieldMeta("Vignette").range(0.0, 1.0).increment(0.01),
            lens.vignette);
}

void describeProperties(SceneCamera& cam, PropertyVisitor& v) {
    v.field(FieldMeta("Name"), cam.name);
    describeProperties(cam.lens, v);
}

void describeProperties(SourceSpec& spec, PropertyVisitor& v) {
    if (!spec.meshFile.empty())
        v.field(FieldMeta("Mesh").locked(), spec.meshFile);
    else
        v.field(FieldMeta("Shape").locked(), spec.shape);
    // Size edits require a mesh rebuild (renderer access) — read-only here
    // until the registry grows a post-edit hook; the in-viewport panel still
    // rebuilds live.
    v.field(FieldMeta("Size").locked(), spec.size);
    v.field(FieldMeta("Physics"), spec.hasPhysics);
    v.field(FieldMeta("Motion").options({"static", "dynamic", "kinematic"}),
            spec.motion);
    v.field(FieldMeta("Friction").range(0.0, 2.0).increment(0.05),
            spec.friction);
    v.field(FieldMeta("Restitution").range(0.0, 1.0).increment(0.05),
            spec.restitution);
}

}  // namespace engine
