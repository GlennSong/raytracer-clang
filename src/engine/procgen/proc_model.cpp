// The model record's walks (#191): what is too long for the header. See proc_model.h.
#include "proc_model.h"

#include "../mesh_builder.h"

#include <algorithm>
#include <cmath>
#include <map>
#include <set>

namespace engine {

namespace {

// `outer` is every copy of the piece `m` is, in the asked model's space; null when `m` is the asked model. `pose`
// (may be null: at rest) moves each moving node; `prefix` is the path `m`'s nodes are named by in it ("door_fl/").
void walkUses(const ProcModel& m, const std::vector<Mat4>* outer, Real distance, const ProcPose* pose, const std::string& prefix,
              std::vector<ProcPlacedUse>& out) {
    const int level = m.levelAt(distance);
    for (const ProcUse& use : m.uses) {
        if (use.piece == nullptr || use.placements.empty() || use.level != level) continue;
        // The root's frame is no move and no turn, so a use on it has its placements' matrices as they are.
        const bool onRoot = use.node == 0;
        const Mat4 node = onRoot ? Mat4::identity() : (pose != nullptr ? m.nodeFrame(use.node, *pose, prefix) : m.nodeFrame(use.node)).matrix();
        std::vector<Mat4> transforms;
        transforms.reserve(use.placements.size() * (outer != nullptr ? outer->size() : 1));
        if (outer == nullptr) {
            for (const ProcPlacement& p : use.placements) transforms.push_back(onRoot ? p.matrix() : node * p.matrix());
        } else {
            for (const Mat4& o : *outer)
                for (const ProcPlacement& p : use.placements) transforms.push_back(onRoot ? o * p.matrix() : o * node * p.matrix());
        }
        const std::string inside = prefix + m.nodes[static_cast<std::size_t>(use.node)].name + "/";
        out.push_back(ProcPlacedUse{&use, transforms, inside});
        walkUses(*use.piece, &transforms, distance, pose, inside, out);
    }
}

}  // namespace

std::vector<ProcPlacedUse> placedUses(const ProcModel& m, Real distance, const ProcPose* pose) {
    std::vector<ProcPlacedUse> out;
    walkUses(m, nullptr, distance, pose, std::string(), out);
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

std::vector<ProcDrawnPiece> drawnPieces(const ProcModel& m, Real distance, const ProcPose* pose) {
    std::vector<ProcDrawnPiece> out;
    std::map<const ProcPart*, std::size_t> known;
    for (const ProcPlacedUse& placed : placedUses(m, distance, pose)) {
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
                const Mat4 node = (pose != nullptr ? piece.nodeFrame(part.node, *pose, placed.prefix) : piece.nodeFrame(part.node)).matrix();
                for (const Mat4& xf : placed.transforms) transforms.push_back(xf * node);
            }
        }
    }
    return out;
}

RenderMesh shapeMesh(const ProcCollider& shape) {
    RenderMesh out;
    switch (shape.kind) {
        case ProcCollider::Kind::Mesh: return shape.mesh;   // exact triangles, already in the node's space
        case ProcCollider::Kind::Box: out = MeshBuilder::box(shape.halfExtent * 2.0); break;
        case ProcCollider::Kind::Sphere: out = MeshBuilder::sphere(static_cast<float>(shape.radius), 12, 16); break;
        case ProcCollider::Kind::Capsule:
            out = MeshBuilder::capsule(static_cast<float>(shape.radius), static_cast<float>(shape.halfHeight * 2.0), 8, 16);
            break;
        case ProcCollider::Kind::Hull: {
            std::vector<Vec3> points;
            for (const Vertex& v : shape.mesh.vertices) points.push_back(v.position);
            out = MeshBuilder::hullMesh(points);
            break;
        }
    }
    MeshBuilder::transform(out, shape.frame().matrix());
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
    out->frozen = false;   // a new model, whatever the one it was posed from is
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

namespace {

void levelDistances(const ProcModel& m, std::set<Real>& out) {
    out.insert(m.levelFrom.begin(), m.levelFrom.end());
    for (const ProcUse& use : m.uses)
        if (use.piece != nullptr) levelDistances(*use.piece, out);
}

bool sameMatrix(const Mat4& a, const Mat4& b) {
    for (int r = 0; r < 4; ++r)
        for (int c = 0; c < 4; ++c)
            if (a.m[r][c] != b.m[r][c]) return false;
    return true;
}

}  // namespace

std::vector<ProcModelDraw> modelDraws(const ProcModel& m, const ProcPose* pose) {
    std::set<Real> cuts;
    levelDistances(m, cuts);
    const std::vector<Real> bands(cuts.begin(), cuts.end());   // the first is 0, the full level's
    std::vector<ProcModelDraw> out;
    std::map<const ProcPart*, std::vector<std::size_t>> drawsOf;
    for (std::size_t b = 0; b < bands.size(); ++b) {
        const Real from = bands[b];
        const Real until = b + 1 < bands.size() ? bands[b + 1] : Real(0);
        // The same part at the same place in the band before: that draw goes on through this one.
        auto emit = [&](const ProcPart& part, const ProcModel& owner, const Mat4& local) {
            std::vector<std::size_t>& known = drawsOf[&part];
            for (std::size_t i : known) {
                if (out[i].until != from || from == 0 || !sameMatrix(out[i].local, local)) continue;
                out[i].until = until;
                return;
            }
            known.push_back(out.size());
            out.push_back(ProcModelDraw{&part, &owner, local, from, until});
        };
        const int level = m.levelAt(from);
        for (const ProcPart& p : m.parts) {
            if (p.level != level || p.mesh.indices.empty()) continue;
            emit(p, m, p.node == 0 ? Mat4::identity() : (pose != nullptr ? m.nodeFrame(p.node, *pose) : m.nodeFrame(p.node)).matrix());
        }
        for (const ProcDrawnPiece& d : drawnPieces(m, from, pose))
            for (const Mat4& xf : d.transforms) emit(*d.part, *d.piece, xf);
    }
    return out;
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
            if (c.kind == ProcCollider::Kind::Hull) twin.mesh = MeshBuilder::mirrored(c.mesh, kLeftRight);
            const ProcFrame f = reflectFrame(c.frame());   // a box, a sphere and a capsule are their own mirror images
            twin.center = f.position;
            twin.rotation = f.rotation;
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
