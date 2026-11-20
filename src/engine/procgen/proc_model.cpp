// The model record's walks (#191): what is too long for the header. See proc_model.h.
#include "proc_model.h"

#include "../mesh_builder.h"

#include <map>

namespace engine {

namespace {

// `outer` is every copy of the piece `m` is, in the asked model's space; null when `m` is the asked model.
void walkUses(const ProcModel& m, const std::vector<Mat4>* outer, std::vector<ProcPlacedUse>& out) {
    for (const ProcUse& use : m.uses) {
        if (use.piece == nullptr || use.placements.empty()) continue;
        // The root's frame is no move and no turn, so a use on it has its placements' matrices as they are.
        const bool onRoot = use.node == 0;
        const Mat4 node = onRoot ? Mat4::identity() : m.nodeFrame(use.node).matrix();
        std::vector<Mat4> transforms;
        transforms.reserve(use.placements.size() * (outer != nullptr ? outer->size() : 1));
        if (outer == nullptr) {
            for (const ProcPlacement& p : use.placements) transforms.push_back(onRoot ? p.matrix() : node * p.matrix());
        } else {
            for (const Mat4& o : *outer)
                for (const ProcPlacement& p : use.placements) transforms.push_back(onRoot ? o * p.matrix() : o * node * p.matrix());
        }
        out.push_back(ProcPlacedUse{&use, transforms});
        walkUses(*use.piece, &transforms, out);
    }
}

}  // namespace

std::vector<ProcPlacedUse> placedUses(const ProcModel& m) {
    std::vector<ProcPlacedUse> out;
    walkUses(m, nullptr, out);
    return out;
}

int ProcModel::instanceCount() const {
    int n = 0;
    for (const ProcPlacedUse& placed : placedUses(*this)) n += static_cast<int>(placed.transforms.size());
    return n;
}

bool ProcModel::markerFrame(const std::string& path, ProcFrame& out) const {
    const std::size_t slash = path.find('/');
    if (slash == std::string::npos) {
        int node = 0;
        const ProcMarker* mk = findMarker(path, &node);
        if (mk == nullptr) return false;
        out = nodeFrame(node) * mk->frame;
        return true;
    }
    // "node/rest": the rest is asked of the piece used on that node, then carried out through the copy and the node.
    const int node = findNode(path.substr(0, slash));
    if (node < 0) return false;
    for (const ProcUse& use : uses) {
        if (use.node != node || use.piece == nullptr || use.placements.empty()) continue;
        ProcFrame inner;
        if (!use.piece->markerFrame(path.substr(slash + 1), inner)) continue;
        const ProcPlacement& copy = use.placements[0];
        const ProcFrame inCopy{copy.position + copy.rotation.rotate(inner.position * copy.scale), copy.rotation * inner.rotation};
        out = nodeFrame(node) * inCopy;
        return true;
    }
    return false;
}

std::vector<ProcDrawnPiece> drawnPieces(const ProcModel& m) {
    std::vector<ProcDrawnPiece> out;
    std::map<const ProcPart*, std::size_t> known;
    for (const ProcPlacedUse& placed : placedUses(m)) {
        const ProcModel& piece = *placed.use->piece;
        for (const ProcPart& part : piece.parts) {
            if (part.mesh.indices.empty()) continue;
            std::map<const ProcPart*, std::size_t>::iterator slot = known.find(&part);
            if (slot == known.end()) {
                slot = known.emplace(&part, out.size()).first;
                out.push_back(ProcDrawnPiece{&piece, &part, {}});
            }
            std::vector<Mat4>& transforms = out[slot->second].transforms;
            // The piece's root is its own space: a part on it stands exactly where the copy does.
            if (part.node == 0) {
                transforms.insert(transforms.end(), placed.transforms.begin(), placed.transforms.end());
            } else {
                const Mat4 node = piece.nodeFrame(part.node).matrix();
                for (const Mat4& xf : placed.transforms) transforms.push_back(xf * node);
            }
        }
    }
    return out;
}

RenderMesh flattenModel(const ProcModel& m) {
    RenderMesh out;
    for (const ProcPart& p : m.parts) {
        if (p.node == 0) MeshBuilder::append(out, p.mesh);   // the root is the model's own space: no move to apply
        else MeshBuilder::appendTransformed(out, p.mesh, m.nodeFrame(p.node).matrix());
    }
    for (const ProcDrawnPiece& d : drawnPieces(m))
        for (const Mat4& xf : d.transforms) MeshBuilder::appendTransformed(out, d.part->mesh, xf);
    return out;
}

}  // namespace engine
