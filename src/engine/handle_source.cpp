#include "handle_source.h"

#include "procgen/city/road_net.h"   // RoadEntity + its edit ops

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

// --- RoadEntity -----------------------------------------------------------------
namespace {
Vec3 roadHandlePos(const RoadEntity& road,
                   const std::function<double(double, double)>& ground,
                   const Vec2& p) {
    double y = (ground ? ground(p.x, p.y) : 0.0) + road.look.lift + 0.1;
    return Vec3(p.x, y, p.y);                       // road is an XZ network; y from terrain
}
}  // namespace

std::vector<EditHandle> RoadHandleSource::handles() const {
    std::vector<EditHandle> out;
    const RoadGraph& g = net->graph;
    for (int i = 0; i < static_cast<int>(g.nodes.size()); ++i) {
        out.push_back({roadHandlePos(*net, ground, g.nodes[i].pos), HandleKind::Knot, i});
        Vec2 t = roadNetTangentAt(*net, i);         // every road is a spline: one tangent handle per node
        out.push_back({roadHandlePos(*net, ground, g.nodes[i].pos + t),
                       HandleKind::TangentOut, i});
    }
    return out;
}

void RoadHandleSource::moveHandle(const EditHandle& h, const Vec3& worldPos) {
    Vec2 xz(worldPos.x, worldPos.z);               // roads live on the ground
    if (h.kind == HandleKind::Knot) {
        roadNetMoveNode(*net, h.index, xz);
    } else if (h.kind == HandleKind::TangentOut) {
        if (h.index >= 0 && h.index < static_cast<int>(net->graph.nodes.size()))
            roadNetSetTangent(*net, h.index, xz - net->graph.nodes[h.index].pos);
    }
}

std::vector<std::pair<Vec3, Vec3>> RoadHandleSource::previewSegments() const {
    std::vector<std::pair<Vec3, Vec3>> segs;
    const RoadGraph& g = net->graph;
    const int n = static_cast<int>(g.nodes.size());
    for (const RoadEdge& e : g.edges) {
        if (e.a < 0 || e.b < 0 || e.a >= n || e.b >= n) continue;
        segs.push_back({roadHandlePos(*net, ground, g.nodes[e.a].pos),
                        roadHandlePos(*net, ground, g.nodes[e.b].pos)});
    }
    return segs;
}

}  // namespace engine
