#include "handle_source.h"

#include "procgen/city/road_net.h"   // RoadNet + its edit ops

namespace engine {

// --- EditableCurve -----------------------------------------------------------
std::vector<EditHandle> CurveHandleSource::handles() const {
    std::vector<EditHandle> out;
    out.reserve(curve->knots.size() * 3);
    for (int i = 0; i < static_cast<int>(curve->knots.size()); ++i) {
        out.push_back({curve->knots[i].position, HandleKind::Knot, i});
        out.push_back({curve->inHandleWorld(i), HandleKind::TangentIn, i});
        out.push_back({curve->outHandleWorld(i), HandleKind::TangentOut, i});
    }
    return out;
}

void CurveHandleSource::moveHandle(const EditHandle& h, const Vec3& worldPos) {
    switch (h.kind) {
        case HandleKind::Knot:       curve->moveKnot(h.index, worldPos); break;
        case HandleKind::TangentIn:  curve->setInHandle(h.index, worldPos); break;
        case HandleKind::TangentOut: curve->setOutHandle(h.index, worldPos); break;
    }
}

std::vector<std::pair<Vec3, Vec3>> CurveHandleSource::previewSegments() const {
    std::vector<std::pair<Vec3, Vec3>> segs;
    std::vector<Vec3> pts = curve->sample(16);
    for (std::size_t i = 1; i < pts.size(); ++i) segs.push_back({pts[i - 1], pts[i]});
    return segs;
}

// --- RoadNet -----------------------------------------------------------------
namespace {
Vec3 roadHandlePos(const RoadNet& net, const Vec2& p) {
    double y = (net.heightAt ? net.heightAt(p.x, p.y) : 0.0) + net.lift + 0.1;
    return Vec3(p.x, y, p.y);                       // road is an XZ network; y from terrain
}
}  // namespace

std::vector<EditHandle> RoadHandleSource::handles() const {
    std::vector<EditHandle> out;
    for (int i = 0; i < static_cast<int>(net->nodes.size()); ++i) {
        out.push_back({roadHandlePos(*net, net->nodes[i]), HandleKind::Knot, i});
        Vec2 t = roadNetTangentAt(*net, i);         // every road is a spline: one tangent handle per node
        out.push_back({roadHandlePos(*net, net->nodes[i] + t), HandleKind::TangentOut, i});
    }
    return out;
}

void RoadHandleSource::moveHandle(const EditHandle& h, const Vec3& worldPos) {
    Vec2 xz(worldPos.x, worldPos.z);               // roads live on the ground
    if (h.kind == HandleKind::Knot) {
        roadNetMoveNode(*net, h.index, xz);
    } else if (h.kind == HandleKind::TangentOut) {
        if (h.index >= 0 && h.index < static_cast<int>(net->nodes.size()))
            roadNetSetTangent(*net, h.index, xz - net->nodes[h.index]);
    }
}

std::vector<std::pair<Vec3, Vec3>> RoadHandleSource::previewSegments() const {
    std::vector<std::pair<Vec3, Vec3>> segs;
    const int n = static_cast<int>(net->nodes.size());
    for (const std::array<int, 2>& e : net->edges) {
        if (e[0] < 0 || e[1] < 0 || e[0] >= n || e[1] >= n) continue;
        segs.push_back({roadHandlePos(*net, net->nodes[e[0]]),
                        roadHandlePos(*net, net->nodes[e[1]])});
    }
    return segs;
}

}  // namespace engine
