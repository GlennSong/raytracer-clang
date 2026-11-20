// The model record's walks (#191): what is too long for the header. See proc_model.h.
#include "proc_model.h"

#include "../mesh_builder.h"

#include <algorithm>
#include <cmath>
#include <map>

namespace engine {

namespace {

// `outer` is every copy of the piece `m` is, in the asked model's space; null when `m` is the asked model.
void walkUses(const ProcModel& m, const std::vector<Mat4>* outer, Real distance, std::vector<ProcPlacedUse>& out) {
    const int level = m.levelAt(distance);
    for (const ProcUse& use : m.uses) {
        if (use.piece == nullptr || use.placements.empty() || use.level != level) continue;
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
        walkUses(*use.piece, &transforms, distance, out);
    }
}

}  // namespace

std::vector<ProcPlacedUse> placedUses(const ProcModel& m, Real distance) {
    std::vector<ProcPlacedUse> out;
    walkUses(m, nullptr, distance, out);
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

std::vector<ProcDrawnPiece> drawnPieces(const ProcModel& m, Real distance) {
    std::vector<ProcDrawnPiece> out;
    std::map<const ProcPart*, std::size_t> known;
    for (const ProcPlacedUse& placed : placedUses(m, distance)) {
        const ProcModel& piece = *placed.use->piece;
        const int level = piece.levelAt(distance);
        for (const ProcPart& part : piece.parts) {
            if (part.mesh.indices.empty() || part.level != level) continue;
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

const ProcNode* ProcModel::jointAt(const std::string& path) const {
    const std::size_t slash = path.find('/');
    const int node = findNode(path.substr(0, slash));
    if (node < 0) return nullptr;
    if (slash == std::string::npos)
        return nodes[static_cast<std::size_t>(node)].motion.kind == ProcMotion::Kind::None ? nullptr : &nodes[static_cast<std::size_t>(node)];
    for (const ProcUse& use : uses) {
        if (use.node != node || use.piece == nullptr) continue;
        if (const ProcNode* inner = use.piece->jointAt(path.substr(slash + 1))) return inner;
    }
    return nullptr;
}

namespace {

std::shared_ptr<ProcModel> posedInto(const ProcModel& m, const ProcPose& pose, const std::string& prefix) {
    std::shared_ptr<ProcModel> out = std::make_shared<ProcModel>(m);
    for (ProcNode& n : out->nodes) {
        if (n.motion.kind == ProcMotion::Kind::None) continue;
        n.frame = n.frame * n.motion.frame(pose.of(prefix + n.name));
        n.motion = ProcMotion{};   // held: a still does not move
    }
    for (ProcUse& use : out->uses) {
        if (use.node == 0 || use.piece == nullptr) continue;   // a piece on the root has no node's name to be reached by
        const std::string inside = prefix + m.nodes[static_cast<std::size_t>(use.node)].name + "/";
        if (pose.anyUnder(inside)) use.piece = posedInto(*use.piece, pose, inside);
    }
    return out;
}

}  // namespace

std::shared_ptr<ProcModel> posedModel(const ProcModel& m, const ProcPose& pose, std::string* why) {
    for (const std::pair<const std::string, Real>& value : pose.values)
        if (m.jointAt(value.first) == nullptr) {
            if (why != nullptr) *why = "the model has no moving node named '" + value.first + "'";
            return nullptr;
        }
    return posedInto(m, pose, std::string());
}

int addSimplifiedLevel(ProcModel& m, Real distance, double ratio, double maxError) {
    const int level = m.addLevel(distance);
    if (level < 0) return -1;
    const std::size_t parts = m.parts.size(), uses = m.uses.size();
    for (std::size_t i = 0; i < parts; ++i) {
        if (m.parts[i].level != 0) continue;
        ProcPart simple = m.parts[i];
        const std::size_t have = simple.mesh.indices.size() / 3;
        const std::size_t target = static_cast<std::size_t>(std::max(0.0, std::floor(static_cast<double>(have) * ratio)));
        simple.mesh = MeshBuilder::simplifiedSurface(m.parts[i].mesh, target, maxError >= 0.0 ? maxError * maxError : 1e30);
        simple.level = level;
        m.parts.push_back(std::move(simple));
    }
    for (std::size_t i = 0; i < uses; ++i) {
        if (m.uses[i].level != 0) continue;
        ProcUse again = m.uses[i];
        again.level = level;
        m.uses.push_back(std::move(again));
    }
    return level;
}

RenderMesh flattenModel(const ProcModel& m, Real distance) {
    RenderMesh out;
    const int level = m.levelAt(distance);
    for (const ProcPart& p : m.parts) {
        if (p.level != level) continue;
        if (p.node == 0) MeshBuilder::append(out, p.mesh);   // the root is the model's own space: no move to apply
        else MeshBuilder::appendTransformed(out, p.mesh, m.nodeFrame(p.node).matrix());
    }
    for (const ProcDrawnPiece& d : drawnPieces(m, distance))
        for (const Mat4& xf : d.transforms) MeshBuilder::appendTransformed(out, d.part->mesh, xf);
    return out;
}

namespace {

const Vec3 kLeftRight(1, 0, 0);   // the mirror plane's normal

// A frame as the mirror shows it. The position is reflected; the rotation is the mirror's image of the turn (the
// same angle about the reflected axis, the other way round), which is again a plain rotation. Its y and z come out
// reflected; its x reflected and reversed.
ProcFrame reflectFrame(const ProcFrame& f) {
    const Vec3 axis = reflect(Vec3(f.rotation.x, f.rotation.y, f.rotation.z), kLeftRight);
    return ProcFrame{reflect(f.position, kLeftRight), Quat(-axis.x, -axis.y, -axis.z, f.rotation.w)};
}

using MirrorMemo = std::map<const ProcModel*, std::shared_ptr<ProcModel>>;

std::shared_ptr<ProcModel> mirrorInto(const ProcModel& m, MirrorMemo& memo, std::string* why) {
    const MirrorMemo::iterator known = memo.find(&m);
    if (known != memo.end()) return known->second;
    if (!m.flatten.empty()) {
        if (why != nullptr) *why = "a model that grades terrain has no mirrored twin";
        return nullptr;
    }
    std::shared_ptr<ProcModel> out = std::make_shared<ProcModel>();
    out->materials = m.materials;
    out->nodes = m.nodes;
    for (ProcNode& node : out->nodes) {
        node.frame = reflectFrame(node.frame);
        for (ProcMarker& mk : node.markers) mk.frame = reflectFrame(mk.frame);
        // The mirror image of a move, in the node's own (mirrored) space: a slide runs along the reflected axis; a
        // turn by the same amount runs the other way round the reflected axis, which is a turn about its opposite.
        const Vec3 axis = reflect(node.motion.axis, kLeftRight);
        node.motion.axis = node.motion.kind == ProcMotion::Kind::Turn ? axis * -1.0 : axis;
    }
    for (const ProcPart& part : m.parts) {
        ProcPart twin = part;
        twin.mesh = MeshBuilder::mirrored(part.mesh, kLeftRight);   // in its node's space, whose frame is mirrored with it
        out->parts.push_back(std::move(twin));
    }
    for (const ProcCollider& c : m.colliders) {
        ProcCollider twin = c;
        if (c.kind == ProcCollider::Kind::Mesh) {
            twin.mesh = MeshBuilder::mirrored(c.mesh, kLeftRight);
        } else {
            twin.center = reflect(c.center, kLeftRight);
            twin.yaw = -c.yaw;   // a turn about the vertical runs the other way in the mirror
        }
        out->colliders.push_back(std::move(twin));
    }
    for (const ProcUse& use : m.uses) {
        ProcUse twin = use;
        if (use.piece != nullptr) {
            twin.piece = mirrorInto(*use.piece, memo, why);
            if (twin.piece == nullptr) return nullptr;
        }
        for (ProcPlacement& p : twin.placements) {
            const ProcFrame f = reflectFrame(ProcFrame{p.position, p.rotation});
            p.position = f.position;
            p.rotation = f.rotation;
        }
        if (!use.collisionProto.vertices.empty()) twin.collisionProto = MeshBuilder::mirrored(use.collisionProto, kLeftRight);
        out->uses.push_back(std::move(twin));
    }
    memo[&m] = out;
    return out;
}

}  // namespace

std::shared_ptr<ProcModel> mirroredModel(const ProcModel& m, std::string* why) {
    MirrorMemo memo;
    return mirrorInto(m, memo, why);
}

}  // namespace engine
