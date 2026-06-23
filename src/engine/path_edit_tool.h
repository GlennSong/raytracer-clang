#ifndef RAYTRACER_ENGINE_PATH_EDIT_TOOL_H
#define RAYTRACER_ENGINE_PATH_EDIT_TOOL_H

#include "curve_edit.h"        // EditRay, DragPlane, projectDrag, nearestHandle
#include "handle_source.h"     // HandleSource, EditHandle, HandleKind
#include <functional>
#include <utility>
#include <vector>

namespace engine {

// The viewport path-edit tool (ADR-0050, Layer 3): the click-drag state machine that
// turns mouse rays into handle edits, on top of the HandleSource binding and the
// curve_edit manipulator math. Renderer-free by design — the editor feeds it the pick
// ray and the camera forward, draws handles()/previewSegments(), and supplies live
// regen / undo bracketing through callbacks — so the whole interaction is
// unit-testable without a viewport, and one tool serves curves and roads alike.
class PathEditTool {
public:
    // Bind to the thing being edited (a CurveHandleSource, a RoadHandleSource, ...).
    // Null unbinds. Always clears any drag in progress.
    void bind(HandleSource* source);
    HandleSource* source() const { return source_; }
    bool bound() const { return source_ != nullptr; }

    // Live mesh regen after each applied move (e.g. rebuild the road carriageway).
    void onEdit(std::function<void()> cb) { onEdit_ = std::move(cb); }
    // Fired once, when a drag first grabs a handle — the editor snapshots for undo here,
    // so one drag becomes one undo entry.
    void onGrab(std::function<void()> cb) { onGrab_ = std::move(cb); }

    // Constraint plane: the source's default (roads: Ground; curves: Screen) unless the
    // user overrides it for this drag with the G/X/Y/Z keys.
    void setPlaneOverride(DragPlane plane) { planeOverride_ = plane; hasOverride_ = true; }
    void clearPlaneOverride() { hasOverride_ = false; }
    DragPlane activePlane() const;

    // Grab the handle under `ray` (within pickRadius perpendicular world units). True if
    // one was grabbed — the editor then knows the tool, not the object picker, owns this
    // click. A miss leaves the tool idle so a plain click still selects.
    bool beginDrag(const EditRay& ray, double pickRadius);
    // Move the grabbed handle to where `ray` meets the constraint plane through it.
    // `viewDir` is the camera forward (only the Screen plane uses it). No-op when idle.
    void drag(const EditRay& ray, const Vec3& viewDir);
    // Release. True if the drag actually moved the handle (so a click that grabbed but
    // didn't move commits no undo entry). Always returns to idle.
    bool endDrag();

    bool dragging() const { return grabbed_ >= 0; }
    int grabbedIndex() const { return grabbed_; }
    HandleKind grabbedKind() const { return grabbedHandle_.kind; }

    // Drawing pass-throughs for the render pass (empty when unbound).
    std::vector<EditHandle> handles() const;
    std::vector<std::pair<Vec3, Vec3>> previewSegments() const;

private:
    HandleSource* source_ = nullptr;
    DragPlane planeOverride_ = DragPlane::Screen;
    bool hasOverride_ = false;
    int grabbed_ = -1;            // index into handles() this drag; -1 = idle
    EditHandle grabbedHandle_{};  // the grabbed handle; its anchor is refreshed per drag
    bool moved_ = false;          // did this drag apply at least one move?
    std::function<void()> onEdit_;
    std::function<void()> onGrab_;
};

}  // namespace engine

#endif
