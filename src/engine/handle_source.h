#ifndef RAYTRACER_ENGINE_HANDLE_SOURCE_H
#define RAYTRACER_ENGINE_HANDLE_SOURCE_H

#include "curve_edit.h"        // DragPlane (+ Vec3 via rt_math)
#include "editable_curve.h"
#include <functional>
#include <utility>
#include <vector>

namespace engine {

struct RoadEntity;   // src/engine/procgen/city/road_net.h

enum class HandleKind { Knot, TangentIn, TangentOut };

struct EditHandle {
    Vec3       position;
    HandleKind kind;
    int        index;          // which knot / node
};

// The seam the path-edit tool talks to (ADR-0050): a thing with draggable handles.
// One tool, many sources — an EditableCurve (animation paths), a RoadEntity (the road
// editor), or anything else with control points. Pure data: no World/Renderer here,
// so it's unit-testable; the editor regenerates whatever a moveHandle mutated.
class HandleSource {
public:
    virtual ~HandleSource() = default;
    virtual std::vector<EditHandle> handles() const = 0;
    virtual void moveHandle(const EditHandle& h, const Vec3& worldPos) = 0;
    // The path drawn as discrete segments (so a branching road works, not just a line).
    virtual std::vector<std::pair<Vec3, Vec3>> previewSegments() const = 0;
    // The constraint plane that fits this source by default (roads: Ground; free curves:
    // Screen). The tool starts here; the user can override per drag.
    virtual DragPlane defaultPlane() const = 0;
};

// Drive a 3D EditableCurve: a knot handle plus its in/out tangent handles, dragged
// freely in view space.
class CurveHandleSource : public HandleSource {
public:
    explicit CurveHandleSource(EditableCurve& c) : curve(&c) {}
    std::vector<EditHandle> handles() const override;
    void moveHandle(const EditHandle& h, const Vec3& worldPos) override;
    std::vector<std::pair<Vec3, Vec3>> previewSegments() const override;
    DragPlane defaultPlane() const override { return DragPlane::Screen; }
private:
    EditableCurve* curve;
};

// Drive a RoadEntity: a node handle plus one through-tangent handle per node (every road is a
// spline), on the ground plane, seated on the road surface via the level's terrain
// sampler (`ground` — the entity no longer stores one; pass what the editor knows,
// null = flat).
class RoadHandleSource : public HandleSource {
public:
    RoadHandleSource(RoadEntity& n, std::function<double(double, double)> ground)
        : net(&n), ground(std::move(ground)) {}
    std::vector<EditHandle> handles() const override;
    void moveHandle(const EditHandle& h, const Vec3& worldPos) override;
    std::vector<std::pair<Vec3, Vec3>> previewSegments() const override;
    DragPlane defaultPlane() const override { return DragPlane::Ground; }
private:
    RoadEntity* net;
    std::function<double(double, double)> ground;
};

}  // namespace engine

#endif
