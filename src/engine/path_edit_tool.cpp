#include "path_edit_tool.h"

namespace engine {

void PathEditTool::bind(HandleSource* source) {
    source_ = source;
    grabbed_ = -1;
    moved_ = false;
}

DragPlane PathEditTool::activePlane() const {
    if (hasOverride_) return planeOverride_;
    return source_ ? source_->defaultPlane() : DragPlane::Screen;
}

bool PathEditTool::beginDrag(const EditRay& ray, double pickRadius) {
    grabbed_ = -1;
    moved_ = false;
    if (!source_) return false;
    std::vector<EditHandle> hs = source_->handles();
    std::vector<Vec3> pts;
    pts.reserve(hs.size());
    for (const EditHandle& h : hs) pts.push_back(h.position);
    int idx = nearestHandle(pts, ray, pickRadius);
    if (idx < 0) return false;
    grabbed_ = idx;
    grabbedHandle_ = hs[idx];
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
