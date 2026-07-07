#ifndef RAYTRACER_ENGINE_DEBUG_DRAW_H
#define RAYTRACER_ENGINE_DEBUG_DRAW_H

#include "../renderer/renderer.h"
#include "../rt_math.h"
#include <cstddef>
#include <vector>

namespace engine {

// Immediate-mode debug line drawing (ADR-0067): any system scribbles lines /
// wire shapes into FrameContext::debug each frame; DebugDrawSystem turns the
// accumulated list into camera-facing ribbon quads and draws them through the
// ordinary AssetManager/Renderer path (FLAG_OVERLAY, on top of the scene) —
// no per-backend line pipeline needed, so it works on Metal/Vulkan/WebGPU/Null
// alike. The engine-side model and the mesh bake are pure data, tested
// headless.
//
// Lifetime: shapes default to ONE frame (re-add every frame, classic
// immediate mode). A positive `seconds` keeps a shape alive across frames —
// the tool for fixed-step producers (physics contacts: pass the fixed step so
// the shape survives until the next step) and one-shot markers ("where did
// the ray hit", 2 s).
class DebugDraw {
public:
    struct Line {
        Vec3 a, b;
        Vec3 color;
        Real ttl;   // seconds left; <= 0 means "this frame only"
    };

    void line(const Vec3& a, const Vec3& b, const Vec3& color,
              Real seconds = 0);

    // Shaft plus four head fins at `to`.
    void arrow(const Vec3& from, const Vec3& to, const Vec3& color,
               Real seconds = 0);

    // Axis-aligned box from min/max corners (12 edges).
    void aabb(const Vec3& min, const Vec3& max, const Vec3& color,
              Real seconds = 0);

    // Oriented box: center + half extents, rotated by `orientation`.
    void box(const Vec3& center, const Vec3& halfExtent,
             const Quat& orientation, const Vec3& color, Real seconds = 0);

    // Wire sphere: three great circles (XY / XZ / YZ).
    void sphere(const Vec3& center, Real radius, const Vec3& color,
                Real seconds = 0, int segments = 24);

    // Circle around `normal` at `center`.
    void circle(const Vec3& center, const Vec3& normal, Real radius,
                const Vec3& color, Real seconds = 0, int segments = 32);

    // RGB basis gizmo of a transform: X red, Y green, Z blue.
    void axes(const Mat4& transform, Real size = 1.0, Real seconds = 0);

    const std::vector<Line>& lines() const { return entries; }
    std::size_t lineCount() const { return entries.size(); }
    bool empty() const { return entries.empty(); }

    // Frame lifecycle, driven by Application: age timed shapes by the frame
    // delta at the top of the frame, drop everything expired after render.
    void update(Real dt);
    void endFrame();
    void clear() { entries.clear(); }

private:
    std::vector<Line> entries;
};

// Bake a line list into camera-facing ribbon quads (two triangles per line),
// wound to front-face the camera per the engine's clockwise convention, line
// color in Vertex::color. Ribbon half-width scales with distance to the
// camera so lines keep a roughly constant on-screen thickness;
// `pixelScale` ~ halfWidth per meter of camera distance. Degenerate
// (zero-length) lines are skipped.
RenderMesh buildDebugLineMesh(const std::vector<DebugDraw::Line>& lines,
                              const Vec3& cameraPosition,
                              Real pixelScale = 0.002);


}  // namespace engine

#endif
