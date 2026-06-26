#include "path_edit_tool.h"

namespace engine {

void PathEditTool::bind(HandleSource* source) {
    source_ = source;
    grabbed_ = -1;
    hovered_ = -1;
    selectedNode_ = -1;
    moved_ = false;
}

// The handle under `ray` within pickRadius, or -1. Shared by hover and grab so the dot
// you see highlighted is exactly the one a click will take.
int PathEditTool::pickHandle(const EditRay& ray, double pickRadius) const {
    if (!source_) return -1;
    std::vector<EditHandle> hs = source_->handles();
    // Prefer KNOTS: a click on a node grabs the node even when a neighbour's tangent handle overlaps
    // it (an auto-tangent on a through-node can land right on the next knot). Try knots first, then
    // fall back to the tangents.
    auto pick = [&](bool knotsOnly) -> int {
        std::vector<Vec3> pts;
        std::vector<int> map;
        for (int i = 0; i < static_cast<int>(hs.size()); ++i) {
            if (knotsOnly && hs[i].kind != HandleKind::Knot) continue;
            pts.push_back(hs[i].position);
            map.push_back(i);
        }
        int local = nearestHandle(pts, ray, pickRadius);
        return local < 0 ? -1 : map[local];
    };
    int knot = pick(true);
    return knot >= 0 ? knot : pick(false);
}

int PathEditTool::updateHover(const EditRay& ray, double pickRadius) {
    hovered_ = pickHandle(ray, pickRadius);
    return hovered_;
}

DragPlane PathEditTool::activePlane() const {
    if (hasOverride_) return planeOverride_;
    return source_ ? source_->defaultPlane() : DragPlane::Screen;
}

bool PathEditTool::beginDrag(const EditRay& ray, double pickRadius) {
    grabbed_ = -1;
    moved_ = false;
    int idx = pickHandle(ray, pickRadius);
    if (idx < 0) return false;
    grabbed_ = idx;
    grabbedHandle_ = source_->handles()[idx];
    selectedNode_ = grabbedHandle_.index;   // light up this node and target topology ops at it
    if (onGrab_) onGrab_();
    return true;
}

void PathEditTool::drag(const EditRay& ray, const Vec3& viewDir) {
    if (grabbed_ < 0 || !source_) return;
    Vec3 target = projectDrag(ray, grabbedHandle_.position, activePlane(), viewDir);
    source_->moveHandle(grabbedHandle_, target);
    moved_ = true;
    if (onEdit_) onEdit_();
    // Re-read the grabbed handle's position: the move may be constrained (a road node
    // snaps its Y to the terrain), so the next drag must project through the new anchor.
    for (const EditHandle& h : source_->handles()) {
        if (h.kind == grabbedHandle_.kind && h.index == grabbedHandle_.index) {
            grabbedHandle_ = h;
            break;
        }
    }
}

bool PathEditTool::endDrag() {
    bool moved = moved_;
    grabbed_ = -1;
    moved_ = false;
    return moved;
}

std::vector<EditHandle> PathEditTool::handles() const {
    return source_ ? source_->handles() : std::vector<EditHandle>{};
}

std::vector<std::pair<Vec3, Vec3>> PathEditTool::previewSegments() const {
    return source_ ? source_->previewSegments() : std::vector<std::pair<Vec3, Vec3>>{};
}

}  // namespace engine
