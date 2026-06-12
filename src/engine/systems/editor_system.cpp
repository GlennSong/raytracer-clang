#include "editor_system.h"

#include "../components.h"
#include "../mesh_builder.h"
#include "../level_writer.h"
#include "../model_importer.h"
#include "../../log.h"
#include <cstdio>

#ifdef RT_ENABLE_IMGUI
#include <imgui.h>
// Gizmos degrade gracefully when the ImGuizmo submodule isn't fetched (the
// build warns); everything else in the editor still works.
#if __has_include(<ImGuizmo.h>)
#include <ImGuizmo.h>
#define RT_HAS_IMGUIZMO 1
#endif
#endif

namespace engine {

namespace {

// Mouse (window coords) -> world-space pick ray. NDC depth follows ADR-0009:
// near plane maps to 1, far to 0, so the ray runs from z=1 toward z=0.
struct PickRay {
    Vec3 origin;
    Vec3 direction;
};

PickRay rayThroughCursor(const FrameContext& ctx) {
    const CameraState& cam = ctx.view.camera;
    Mat4 view = Mat4::lookAt(cam.position, cam.target, cam.up);
    Mat4 proj = (cam.projection == CameraProjection::Perspective)
        ? Mat4::perspective(degreesToRadians(cam.fovDegrees), cam.aspectRatio,
                            cam.nearPlane, cam.farPlane)
        : Mat4::orthographic(cam.orthoHeight, cam.aspectRatio,
                             cam.nearPlane, cam.farPlane);
    Mat4 invVP = (proj * view).inverse();

    double w = std::max(ctx.windowWidth, 1);
    double h = std::max(ctx.windowHeight, 1);
    double ndcX = 2.0 * ctx.input.mouseX / w - 1.0;
    double ndcY = 1.0 - 2.0 * ctx.input.mouseY / h;

    Vec3 nearP = invVP.transformPoint(Vec3(ndcX, ndcY, 1.0));
    Vec3 farP = invVP.transformPoint(Vec3(ndcX, ndcY, 0.0));
    return {nearP, normalize(farP - nearP)};
}

// Smallest positive t where the ray enters the sphere; negative on miss.
double raySphere(const PickRay& ray, const Vec3& center, double radius) {
    Vec3 oc = ray.origin - center;
    double b = dot(oc, ray.direction);
    double c = oc.lengthSquared() - radius * radius;
    double disc = b * b - c;
    if (disc < 0.0) return -1.0;
    double t = -b - std::sqrt(disc);
    if (t < 0.0) t = -b + std::sqrt(disc);   // origin inside the sphere
    return t;
}

#ifdef RT_ENABLE_IMGUI
// Our Mat4 is row-major with column-vector convention; ImGuizmo wants
// GL-style float16 column-major. The conversion is a transpose.
void toGizmo(const Mat4& m, float* out) {
    for (int c = 0; c < 4; c++)
        for (int r = 0; r < 4; r++)
            out[c * 4 + r] = static_cast<float>(m.m[r][c]);
}

Mat4 fromGizmo(const float* in) {
    Mat4 m;
    for (int c = 0; c < 4; c++)
        for (int r = 0; r < 4; r++)
            m.m[r][c] = in[c * 4 + r];
    return m;
}

// Decompose a manipulated matrix back into our Transform (M = T*R*S): the
// translation is the last column, scale the rotation columns' lengths, and
// the orientation the normalized rotation part.
void matrixToTransform(const Mat4& m, Transform& t) {
    t.position = Vec3(m.m[0][3], m.m[1][3], m.m[2][3]);
    Vec3 cx(m.m[0][0], m.m[1][0], m.m[2][0]);
    Vec3 cy(m.m[0][1], m.m[1][1], m.m[2][1]);
    Vec3 cz(m.m[0][2], m.m[1][2], m.m[2][2]);
    t.scale = Vec3(cx.length(), cy.length(), cz.length());
    if (t.scale.x < 1e-9 || t.scale.y < 1e-9 || t.scale.z < 1e-9) return;

    Mat4 pure;
    Vec3 nx = cx / t.scale.x, ny = cy / t.scale.y, nz = cz / t.scale.z;
    pure.m[0][0] = nx.x; pure.m[1][0] = nx.y; pure.m[2][0] = nx.z;
    pure.m[0][1] = ny.x; pure.m[1][1] = ny.y; pure.m[2][1] = ny.z;
    pure.m[0][2] = nz.x; pure.m[1][2] = nz.y; pure.m[2][2] = nz.z;
    t.orientation = Quat::fromRotationMatrix(pure);
}
#endif

// Sensible default sizes per shape (loader semantics: x = radius, y = height).
Vec3 defaultShapeSize(const std::string& shape) {
    if (shape == "sphere")   return Vec3(0.5, 0, 0);
    if (shape == "cylinder") return Vec3(0.5, 1, 0);
    if (shape == "cone")     return Vec3(0.5, 1, 0);
    if (shape == "capsule")  return Vec3(0.4, 1, 0);
    if (shape == "torus")    return Vec3(0.6, 0.2, 0);
    if (shape == "plane")    return Vec3(2, 2, 0);
    return Vec3(1, 1, 1);   // box, wedge
}

}  // namespace

EditorSystem::EditorSystem(CameraSystem& cameras, std::string levelFile,
                           PlayFactory makePlayState, EditorBridge* bridge)
    : cameras(cameras), levelFile(std::move(levelFile)),
      makePlayState(std::move(makePlayState)), bridge(bridge) {}

void EditorSystem::onStart(FrameContext& ctx) {
    ctx.actions.bindButton("gizmo_translate", KeyCode::Num1);
    ctx.actions.bindButton("gizmo_rotate", KeyCode::Num2);
    ctx.actions.bindButton("gizmo_scale", KeyCode::Num3);
    if (bridge) bridge->attach(&ctx.world, this, levelFile);
}

void EditorSystem::onStop(FrameContext&) {
    // Play (or app shutdown) tears the editor state down: the shell's panels
    // see a detached bridge and gray out rather than touching gameplay state.
    if (bridge) bridge->detach();
}

void EditorSystem::update(FrameContext& ctx) {
    if (ctx.actions.pressed("gizmo_translate")) gizmoOp = 0;
    if (ctx.actions.pressed("gizmo_rotate")) gizmoOp = 1;
    if (ctx.actions.pressed("gizmo_scale")) gizmoOp = 2;

    bool click = ctx.input.mouseLeftDown && !prevMouseLeft;
    prevMouseLeft = ctx.input.mouseLeftDown;
    if (click && !ctx.input.uiWantsMouse && !gizmoBusy)
        pickAtCursor(ctx);

    if (!ctx.world.alive(selected)) selected = Entity{};
}

void EditorSystem::pickAtCursor(FrameContext& ctx) {
    PickRay ray = rayThroughCursor(ctx);

    Entity best;
    double bestT = 1e30;
    ctx.world.each<Transform, Renderable>(
        [&](Entity e, Transform& t, Renderable& r) {
            BoundingSphere bounds = ctx.renderer.getMeshBounds(r.mesh);
            Vec3 center = t.matrix().transformPoint(bounds.center);
            Real maxScale = std::max({std::abs(t.scale.x), std::abs(t.scale.y),
                                      std::abs(t.scale.z)});
            double hit = raySphere(ray, center, bounds.radius * maxScale);
            if (hit > 0.0 && hit < bestT) {
                bestT = hit;
                best = e;
            }
        });

    selected = best;   // clicking empty space deselects
}

Vec3 EditorSystem::spawnPoint(FrameContext& ctx) const {
    // A few meters in front of the view, so new objects appear where you look.
    const CameraState& cam = ctx.view.camera;
    return cam.position + normalize(cam.target - cam.position) * 5.0;
}

Entity EditorSystem::addPrimitive(FrameContext& ctx, const std::string& shape) {
    SourceSpec spec;
    spec.shape = shape;
    spec.size = defaultShapeSize(shape);

    RenderMesh mesh = MeshBuilder::shape(shape, spec.size);
    if (mesh.vertices.empty()) return Entity{};

    Entity e = ctx.world.create();
    Transform t;
    t.position = spawnPoint(ctx);
    ctx.world.add<Transform>(e, t);
    ctx.world.add<PrevTransform>(e, {t});
    ctx.world.add<SourceSpec>(e, spec);

    Renderable r;
    r.mesh = ctx.renderer.uploadMesh(mesh);
    r.material.albedo = Vec3(0.7, 0.7, 0.7);
    r.material.roughness = 0.6f;
    ctx.world.add<Renderable>(e, r);
    return e;
}

Entity EditorSystem::duplicateSelected(FrameContext& ctx) {
    Transform* t = ctx.world.get<Transform>(selected);
    SourceSpec* spec = ctx.world.get<SourceSpec>(selected);
    Renderable* r = ctx.world.get<Renderable>(selected);
    if (!t || !spec || !r) return Entity{};

    Entity e = ctx.world.create();
    Transform copy = *t;
    copy.position += Vec3(spec->size.x + 0.5, 0, 0);   // beside the original
    ctx.world.add<Transform>(e, copy);
    ctx.world.add<PrevTransform>(e, {copy});
    ctx.world.add<SourceSpec>(e, *spec);
    ctx.world.add<Renderable>(e, *r);   // shares the uploaded mesh
    return e;
}

#ifdef RT_ENABLE_IMGUI

void EditorSystem::render(FrameContext& ctx) {
    // No ImGui context (e.g. a backend without debug-UI support): stay inert.
    if (ImGui::GetCurrentContext() == nullptr) return;
#ifdef RT_HAS_IMGUIZMO
    ImGuizmo::BeginFrame();
#endif
    drawToolbar(ctx);
    drawInspector(ctx);
    drawGizmo(ctx);
    drawSelectionMarker(ctx);
}

void EditorSystem::drawToolbar(FrameContext& ctx) {
    ImGui::Begin("Editor");

    if (ImGui::Button("  Play  ")) {
        // Save first: Play loads the document fresh, so what you built is
        // exactly what runs (and nothing is lost if the game crashes).
        LevelWriter::save(levelFile, ctx.world);
        ctx.transition.replaceWith(makePlayState());
    }
    ImGui::SameLine();
    if (ImGui::Button("Save")) LevelWriter::save(levelFile, ctx.world);
    ImGui::SameLine();
    ImGui::TextDisabled("%s", levelFile.c_str());

    ImGui::SeparatorText("Add");
    static const char* SHAPES[] = {"box", "sphere", "cylinder", "plane",
                                   "cone", "wedge", "torus", "capsule"};
    for (int i = 0; i < 8; i++) {
        if (ImGui::Button(SHAPES[i])) selected = addPrimitive(ctx, SHAPES[i]);
        if (i % 4 != 3) ImGui::SameLine();
    }
    if (ImGui::Button("camera"))
        selected = cameras.placeCameraAtView(ctx);

    ImGui::SetNextItemWidth(180);
    ImGui::InputText("##gltf", modelPath, sizeof(modelPath));
    ImGui::SameLine();
    if (ImGui::Button("Add glTF")) {
        // Resolve relative to the level's directory, like the loader does.
        std::string dir = levelFile;
        std::size_t slash = dir.find_last_of("/\\");
        dir = (slash == std::string::npos) ? "" : dir.substr(0, slash + 1);
        ImportedModel model = ModelImporter::load(dir + modelPath, ctx.renderer);
        if (model.meshes.empty()) {
            LOG_WARN << "No meshes in " << modelPath;
        } else {
            Transform t;
            t.position = spawnPoint(ctx);
            for (size_t i = 0; i < model.meshes.size(); i++) {
                Entity e = ctx.world.create();
                ctx.world.add<Transform>(e, t);
                ctx.world.add<PrevTransform>(e, {t});
                Renderable r;
                r.mesh = model.meshes[i].meshHandle;
                r.material = model.meshes[i].material;
                ctx.world.add<Renderable>(e, r);
                if (i == 0) {
                    SourceSpec spec;
                    spec.shape = "";
                    spec.meshFile = modelPath;
                    ctx.world.add<SourceSpec>(e, spec);
                    selected = e;
                }
            }
        }
    }

    ImGui::SeparatorText("Gizmo");
    ImGui::RadioButton("Move (1)", &gizmoOp, 0);
    ImGui::SameLine();
    ImGui::RadioButton("Rotate (2)", &gizmoOp, 1);
    ImGui::SameLine();
    ImGui::RadioButton("Scale (3)", &gizmoOp, 2);

    ImGui::End();
}

void EditorSystem::drawInspector(FrameContext& ctx) {
    Transform* t = ctx.world.get<Transform>(selected);
    if (!t) return;

    ImGui::Begin("Editor");
    ImGui::SeparatorText("Selected");

    SourceSpec* spec = ctx.world.get<SourceSpec>(selected);
    SceneCamera* cam = ctx.world.get<SceneCamera>(selected);
    if (cam)
        ImGui::Text("Camera: %s (lens in Debug > Cameras)", cam->name.c_str());
    else if (spec)
        ImGui::Text("%s", spec->meshFile.empty() ? spec->shape.c_str()
                                                 : spec->meshFile.c_str());

    float pos[3] = {static_cast<float>(t->position.x),
                    static_cast<float>(t->position.y),
                    static_cast<float>(t->position.z)};
    if (ImGui::DragFloat3("Position", pos, 0.05f))
        t->position = Vec3(pos[0], pos[1], pos[2]);
    float scl[3] = {static_cast<float>(t->scale.x),
                    static_cast<float>(t->scale.y),
                    static_cast<float>(t->scale.z)};
    if (ImGui::DragFloat3("Scale", scl, 0.02f, 0.01f, 100.0f))
        t->scale = Vec3(scl[0], scl[1], scl[2]);

    Renderable* r = ctx.world.get<Renderable>(selected);
    if (r && !cam) {
        auto& m = r->material;
        float albedo[3] = {static_cast<float>(m.albedo.x),
                           static_cast<float>(m.albedo.y),
                           static_cast<float>(m.albedo.z)};
        if (ImGui::ColorEdit3("Albedo", albedo))
            m.albedo = Vec3(albedo[0], albedo[1], albedo[2]);
        ImGui::SliderFloat("Metallic", &m.metallic, 0.0f, 1.0f);
        ImGui::SliderFloat("Roughness", &m.roughness, 0.02f, 1.0f);
        bool checker = (m.flags & RenderMaterial::FLAG_CHECKERBOARD) != 0;
        if (ImGui::Checkbox("Checkerboard", &checker))
            m.flags = checker ? (m.flags | RenderMaterial::FLAG_CHECKERBOARD)
                              : (m.flags & ~RenderMaterial::FLAG_CHECKERBOARD);
    }

    if (spec && spec->meshFile.empty() && !cam) {
        float size[3] = {static_cast<float>(spec->size.x),
                         static_cast<float>(spec->size.y),
                         static_cast<float>(spec->size.z)};
        if (ImGui::DragFloat3("Size", size, 0.05f, 0.05f, 100.0f)) {
            spec->size = Vec3(size[0], size[1], size[2]);
            // Rebuild the mesh at the new size (the old upload leaks — see
            // docs/TECH_DEBT.md mesh-cache item).
            RenderMesh mesh = MeshBuilder::shape(spec->shape, spec->size);
            if (!mesh.vertices.empty() && r)
                r->mesh = ctx.renderer.uploadMesh(mesh);
        }
    }

    if (spec) {
        const char* MOTIONS[] = {"none", "static", "dynamic", "kinematic"};
        int motion = !spec->hasPhysics ? 0
                     : spec->motion == "dynamic"   ? 2
                     : spec->motion == "kinematic" ? 3 : 1;
        if (ImGui::Combo("Physics", &motion, MOTIONS, 4)) {
            spec->hasPhysics = motion != 0;
            if (motion != 0) spec->motion = MOTIONS[motion];
        }
    }

    if (ImGui::Button("Duplicate")) selected = duplicateSelected(ctx);
    ImGui::SameLine();
    if (ImGui::Button("Delete")) {
        ctx.world.destroy(selected);
        selected = Entity{};
    }

    // Cameras are stationary; keep the interpolation pair in sync so edits
    // don't smear across frames (true for all editor objects: nothing moves).
    if (ctx.world.alive(selected))
        if (auto* prev = ctx.world.get<PrevTransform>(selected))
            prev->value = *ctx.world.get<Transform>(selected);

    ImGui::End();
}

void EditorSystem::drawGizmo(FrameContext& ctx) {
#ifndef RT_HAS_IMGUIZMO
    (void)ctx;
    gizmoBusy = false;
    return;
#else
    Transform* t = ctx.world.get<Transform>(selected);
    if (!t) {
        gizmoBusy = false;
        return;
    }

    const CameraState& cam = ctx.view.camera;
    Mat4 view = Mat4::lookAt(cam.position, cam.target, cam.up);
    Mat4 proj = Mat4::perspective(degreesToRadians(cam.fovDegrees),
                                  cam.aspectRatio, cam.nearPlane, cam.farPlane);

    float viewF[16], projF[16], modelF[16];
    toGizmo(view, viewF);
    toGizmo(proj, projF);
    toGizmo(t->matrix(), modelF);

    ImGuiIO& io = ImGui::GetIO();
    ImGuizmo::SetOrthographic(false);
    ImGuizmo::SetRect(0, 0, io.DisplaySize.x, io.DisplaySize.y);

    ImGuizmo::OPERATION op = (gizmoOp == 1)   ? ImGuizmo::ROTATE
                             : (gizmoOp == 2) ? ImGuizmo::SCALE
                                              : ImGuizmo::TRANSLATE;
    ImGuizmo::Manipulate(viewF, projF, op, ImGuizmo::WORLD, modelF);
    gizmoBusy = ImGuizmo::IsOver() || ImGuizmo::IsUsing();

    if (ImGuizmo::IsUsing()) {
        matrixToTransform(fromGizmo(modelF), *t);
        if (auto* prev = ctx.world.get<PrevTransform>(selected))
            prev->value = *t;
    }
#endif
}

void EditorSystem::drawSelectionMarker(FrameContext& ctx) const {
    Transform* t = ctx.world.get<Transform>(selected);
    Renderable* r = ctx.world.get<Renderable>(selected);
    if (!t || !r) return;

    // Project the bounding-sphere center to the screen and ring it — selection
    // feedback without touching the entity's material.
    const CameraState& cam = ctx.view.camera;
    Mat4 view = Mat4::lookAt(cam.position, cam.target, cam.up);
    Mat4 proj = Mat4::perspective(degreesToRadians(cam.fovDegrees),
                                  cam.aspectRatio, cam.nearPlane, cam.farPlane);
    BoundingSphere bounds = ctx.renderer.getMeshBounds(r->mesh);
    Vec3 center = t->matrix().transformPoint(bounds.center);

    Vec3 viewPos = view.transformPoint(center);
    if (viewPos.z > -1e-3) return;   // behind the camera (-Z is forward)
    Vec3 ndc = proj.transformPoint(viewPos);

    const ImGuiViewport* vp = ImGui::GetMainViewport();
    float sx = vp->Pos.x + (static_cast<float>(ndc.x) * 0.5f + 0.5f) * vp->Size.x;
    float sy = vp->Pos.y + (0.5f - static_cast<float>(ndc.y) * 0.5f) * vp->Size.y;

    // Apparent radius: world radius over distance, scaled into pixels.
    Real maxScale = std::max({std::abs(t->scale.x), std::abs(t->scale.y),
                              std::abs(t->scale.z)});
    float dist = static_cast<float>(-viewPos.z);
    float fovScale = vp->Size.y / (2.0f * std::tan(degreesToRadians(cam.fovDegrees) * 0.5f));
    float radius = static_cast<float>(bounds.radius * maxScale) / dist * fovScale;
    radius = std::clamp(radius * 1.15f, 12.0f, 0.6f * vp->Size.y);

    ImGui::GetBackgroundDrawList()->AddCircle(
        ImVec2(sx, sy), radius, IM_COL32(255, 170, 40, 200), 0, 2.0f);
}

#else  // !RT_ENABLE_IMGUI

void EditorSystem::render(FrameContext& ctx) {
    // No UI without ImGui; picking still works (selection marker doesn't draw).
    (void)ctx;
    (void)cameras;
    (void)gizmoOp;
    (void)modelPath;
}
void EditorSystem::drawToolbar(FrameContext&) {}
void EditorSystem::drawInspector(FrameContext&) {}
void EditorSystem::drawGizmo(FrameContext&) {}
void EditorSystem::drawSelectionMarker(FrameContext&) const {}

#endif

}  // namespace engine
