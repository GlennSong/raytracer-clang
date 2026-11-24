// A model as a glTF file, and back (#191, model step 6). See model_file.h for the convention.
#include "model_file.h"

#include "../glb_export.h"
#include "../mesh_builder.h"
#include "../model_importer.h"

#include <nlohmann/json.hpp>

#include <cmath>
#include <deque>
#include <map>

namespace engine {

using json = nlohmann::json;

namespace {

const char* kindWord(ProcCollider::Kind k) {
    switch (k) {
        case ProcCollider::Kind::Box: return "box";
        case ProcCollider::Kind::Sphere: return "sphere";
        case ProcCollider::Kind::Capsule: return "capsule";
        case ProcCollider::Kind::Hull: return "hull";
        case ProcCollider::Kind::Mesh: return "mesh";
    }
    return "mesh";
}

json vec3Json(const Vec3& v) { return json::array({v.x, v.y, v.z}); }

// ---------------------------------------------------------------------------------------------------------------
// WRITING
// ---------------------------------------------------------------------------------------------------------------
struct Writer {
    std::vector<GlbTreeNode> nodes;
    std::vector<GlbTreeMaterial> materials;
    std::map<std::pair<const ProcMaterial*, std::string>, int> materialOf;
    std::deque<RenderMesh> solids;                                   // collision shapes' meshes, kept alive to the write
    std::map<const ProcModel*, int> pieceNumber;                     // a used piece's `use` number
    std::map<const ProcModel*, std::vector<int>> pieceNodes;         // the nodes of its first copy, in the order written
    std::string error;

    int material(const ProcModelMaterial& mm) {
        const std::pair<const ProcMaterial*, std::string> key{mm.material.get(), mm.name};
        const std::map<std::pair<const ProcMaterial*, std::string>, int>::iterator known = materialOf.find(key);
        if (known != materialOf.end()) return known->second;
        const ProcMaterial& pm = *mm.material;
        GlbTreeMaterial out;
        out.name = mm.name;
        out.metallic = pm.metallic;
        out.roughness = pm.roughness;
        out.opacity = pm.opacity;
        // glTF holds emission to 0..1; a lamp here may be brighter. The file's own field gets what it can hold, for
        // any other tool, and the true value rides as a property this reader takes first.
        out.emission = Vec3(std::min(pm.emission.x, Real(1)), std::min(pm.emission.y, Real(1)), std::min(pm.emission.z, Real(1)));
        out.twoSided = pm.twoSided;
        if (pm.textured && !pm.albedo.pixels.empty()) {
            out.albedoW = pm.albedo.width; out.albedoH = pm.albedo.height; out.albedoChannels = pm.albedo.channels;
            out.albedo = &pm.albedo.pixels;
        }
        if (pm.textured && !pm.normal.pixels.empty()) {
            out.normalW = pm.normal.width; out.normalH = pm.normal.height; out.normalChannels = pm.normal.channels;
            out.normal = &pm.normal.pixels;
        }
        json extras = json::object();
        if (pm.textured) extras["tile"] = pm.tile;
        if (pm.surface != 0) extras["surface"] = pm.surface;
        if (pm.alphaFoliage) extras["alpha_foliage"] = true;
        if (pm.emission.x > 1 || pm.emission.y > 1 || pm.emission.z > 1) extras["emission"] = vec3Json(pm.emission);
        out.extras = extras;
        materials.push_back(out);
        materialOf[key] = static_cast<int>(materials.size()) - 1;
        return static_cast<int>(materials.size()) - 1;
    }

    // One node written. A second copy of a used piece draws the meshes of the first: `reuse` is the first copy's
    // nodes in the order they were written, `written` counts this copy's, and the two run in step.
    int push(GlbTreeNode node, const std::vector<int>* reuse, std::vector<int>* record, std::size_t& written) {
        if (reuse != nullptr) {
            node.primitives.clear();
            node.meshOf = written < reuse->size() ? (*reuse)[written] : -1;
        }
        ++written;
        nodes.push_back(std::move(node));
        if (record != nullptr) record->push_back(static_cast<int>(nodes.size()) - 1);
        return static_cast<int>(nodes.size()) - 1;
    }

    // Write `m` as a subtree: its root as `top` (the caller's name, parent, placement and properties), everything
    // else under it.
    bool model(const ProcModel& m, GlbTreeNode top, const std::vector<int>* reuse, std::vector<int>* record, std::size_t& written) {
        if (!m.flatten.empty()) { error = "a model that grades terrain cannot be written to a file"; return false; }
        std::vector<int> glbOf(m.nodes.size(), -1);
        auto primitives = [&](int node, int level) {
            std::vector<GlbTreePrimitive> out;
            for (const ProcPart& part : m.parts)
                if (part.node == node && part.level == level && !part.mesh.indices.empty())
                    out.push_back(GlbTreePrimitive{&part.mesh, part.material == 0 ? -1 : material(m.materials[static_cast<std::size_t>(part.material)])});
            return out;
        };
        for (std::size_t i = 0; i < m.nodes.size(); ++i) {
            const ProcNode& n = m.nodes[i];
            GlbTreeNode g;
            if (i == 0) {
                g = top;
                if (!g.extras.is_object()) g.extras = json::object();
                if (m.levelFrom.size() > 1) g.extras["levels"] = m.levelFrom;
            } else {
                g.name = n.name;
                g.parent = glbOf[static_cast<std::size_t>(n.parent)];
                g.translation = n.frame.position;
                g.rotation = n.frame.rotation;
                g.extras = json::object();
            }
            if (n.motion.kind != ProcMotion::Kind::None) {
                const bool turn = n.motion.kind == ProcMotion::Kind::Turn;
                json motion = {{"kind", turn ? "turn" : "slide"}, {"axis", vec3Json(n.motion.axis)}};
                if (n.motion.limited) {   // degrees for a turn and metres for a slide, as a recipe writes them
                    motion["min"] = n.motion.min * (turn ? RAD_TO_DEG : 1.0);
                    motion["max"] = n.motion.max * (turn ? RAD_TO_DEG : 1.0);
                }
                g.extras["motion"] = motion;
            }
            g.primitives = primitives(static_cast<int>(i), 0);
            if (g.primitives.empty()) g.extras["node"] = true;   // an object with no mesh would read as a marker
            glbOf[i] = push(g, reuse, record, written);
            for (std::size_t level = 1; level < m.levelFrom.size(); ++level) {
                GlbTreeNode lod;
                lod.primitives = primitives(static_cast<int>(i), static_cast<int>(level));
                if (lod.primitives.empty()) continue;
                lod.name = g.name + "_lod" + std::to_string(level);   // by the object's name: a piece's copy is called by the piece's
                lod.parent = glbOf[i];
                push(lod, reuse, record, written);
            }
            for (const ProcMarker& mk : n.markers) {
                GlbTreeNode e;
                e.name = mk.name;
                e.parent = glbOf[i];
                e.translation = mk.frame.position;
                e.rotation = mk.frame.rotation;
                e.extras = json::object();
                if (mk.role != "point") e.extras["role"] = mk.role;
                if (mk.size > 0) e.extras["size"] = mk.size;
                push(e, reuse, record, written);
            }
        }
        for (const ProcCollider& c : m.colliders) {
            GlbTreeNode g;
            g.name = std::string("col_") + kindWord(c.kind) + "_" + c.name;
            g.parent = glbOf[static_cast<std::size_t>(c.node)];
            ProcCollider local = c;                      // the solid about its own origin; the object carries its place
            if (c.kind != ProcCollider::Kind::Mesh) {
                local.center = Vec3(0, 0, 0);
                local.rotation = Quat();
                g.translation = c.center;
                g.rotation = c.rotation;
            }
            solids.push_back(shapeMesh(local));
            g.primitives.push_back(GlbTreePrimitive{&solids.back(), -1});
            g.extras = json::object();
            if (c.friction != ProcCollider{}.friction) g.extras["friction"] = c.friction;
            if (c.dynamic) g.extras["dynamic"] = true;
            push(g, reuse, record, written);
        }
        for (const ProcUse& use : m.uses) {
            if (use.piece == nullptr) continue;
            if (!use.collisionProto.vertices.empty()) { error = "a use with a collision mesh of its own cannot be written to a file yet"; return false; }
            if (pieceNumber.count(use.piece.get()) == 0) pieceNumber[use.piece.get()] = static_cast<int>(pieceNumber.size()) + 1;
            for (const ProcPlacement& p : use.placements) {
                GlbTreeNode copy;
                copy.name = use.piece->name.empty() ? "piece" + std::to_string(pieceNumber[use.piece.get()]) : use.piece->name;
                copy.parent = glbOf[static_cast<std::size_t>(use.node)];
                copy.translation = p.position;
                copy.rotation = p.rotation;
                copy.scale = p.scale;
                copy.extras = json::object();
                copy.extras["use"] = pieceNumber[use.piece.get()];
                if (use.level > 0) copy.extras["level"] = use.level;
                if (use.collision != InstanceCollision::None) {
                    json collide = json::object();
                    collide["shape"] = use.collision == InstanceCollision::Mesh ? "mesh" : use.collision == InstanceCollision::Box ? "box"
                                     : use.collision == InstanceCollision::Sphere ? "sphere" : "capsule";
                    collide["size"] = vec3Json(use.colliderHalfExtent * 2.0);
                    collide["radius"] = use.colliderRadius;
                    collide["height"] = use.colliderHalfHeight * 2.0;
                    collide["friction"] = use.colliderFriction;
                    copy.extras["collide"] = collide;
                }
                // The first copy written holds the piece's meshes; every later one, anywhere in the file, draws them.
                const std::map<const ProcModel*, std::vector<int>>::iterator first = pieceNodes.find(use.piece.get());
                if (first == pieceNodes.end() && reuse == nullptr) {
                    std::vector<int> mine;
                    std::size_t count = 0;
                    if (!model(*use.piece, copy, nullptr, &mine, count)) return false;
                    pieceNodes[use.piece.get()] = mine;
                    if (record != nullptr) record->insert(record->end(), mine.begin(), mine.end());
                    written += count;
                } else if (first != pieceNodes.end()) {
                    std::size_t count = 0;
                    std::vector<int> mine;
                    if (!model(*use.piece, copy, &first->second, &mine, count)) return false;
                    if (record != nullptr) record->insert(record->end(), mine.begin(), mine.end());
                    written += count;
                } else {
                    // inside a later copy of an outer piece, a piece met for the first time there: it draws the
                    // outer first copy's nodes, in step
                    if (!model(*use.piece, copy, reuse, record, written)) return false;
                }
            }
        }
        return true;
    }
};

// ---------------------------------------------------------------------------------------------------------------
// READING
// ---------------------------------------------------------------------------------------------------------------
json extrasOf(const std::string& text) {
    if (text.empty()) return json::object();
    json j = json::parse(text, nullptr, false);
    return j.is_object() ? j : json::object();
}

// A matrix as the rigid frame it holds and what is left over (scale). `uniform`: the one scale, when all three agree.
struct Split {
    ProcFrame frame;
    Mat4 rest;            // inverse(frame) * the matrix: applied to a mesh's points, they are in the frame's space
    bool plain = true;    // rest is (to a hair) the identity
    Real scale = 1;
    bool even = true;     // the three axes are scaled alike
    bool mirrored = false;
};
Split split(const Mat4& m) {
    Split s;
    Vec3 c[3];
    Real len[3];
    for (int k = 0; k < 3; ++k) {
        c[k] = Vec3(m.m[0][k], m.m[1][k], m.m[2][k]);
        len[k] = c[k].length();
        if (len[k] > 1e-12) c[k] = c[k] / len[k];
    }
    s.mirrored = dot(cross(c[0], c[1]), c[2]) < 0;
    if (s.mirrored) c[0] = c[0] * -1.0;   // keep the frame a rotation; the flip stays in `rest`
    // The three directions may not be quite square to each other (a sheared object): square them up, z kept.
    c[0] = normalize(cross(c[1], c[2]));
    c[1] = normalize(cross(c[2], c[0]));
    Mat4 r;
    for (int k = 0; k < 3; ++k) { r.m[0][k] = c[k].x; r.m[1][k] = c[k].y; r.m[2][k] = c[k].z; }
    s.frame.rotation = Quat::fromRotationMatrix(r);
    s.frame.position = Vec3(m.m[0][3], m.m[1][3], m.m[2][3]);
    s.rest = s.frame.matrix().inverse() * m;
    s.scale = len[0];
    s.even = std::abs(len[0] - len[1]) < 1e-6 * std::max(Real(1), len[0]) && std::abs(len[0] - len[2]) < 1e-6 * std::max(Real(1), len[0]);
    const Mat4 id = Mat4::identity();
    for (int i = 0; i < 4 && s.plain; ++i)
        for (int j = 0; j < 4; ++j)
            if (std::abs(s.rest.m[i][j] - id.m[i][j]) > 1e-6) { s.plain = false; break; }
    return s;
}

// `name` as "<base>_lod<N>": the base and N, or false.
bool lodName(const std::string& name, std::string& base, int& level) {
    const std::size_t at = name.rfind("_lod");
    if (at == std::string::npos || at == 0 || at + 4 >= name.size()) return false;
    int n = 0;
    for (std::size_t i = at + 4; i < name.size(); ++i) {
        if (name[i] < '0' || name[i] > '9') return false;
        n = n * 10 + (name[i] - '0');
    }
    base = name.substr(0, at);
    level = n;
    return true;
}

struct Reader {
    const CpuTree& tree;
    std::vector<std::string>* notes;
    std::string error;
    std::map<int, std::shared_ptr<ProcMaterial>> materials;           // by the file's material number
    std::map<int, std::shared_ptr<ProcModel>> pieces;                 // by `use` number
    int maxLevel = 0;

    void note(const std::string& text) { if (notes != nullptr) notes->push_back(text); }

    int materialFor(ProcModel& m, const CpuMaterial& cm) {
        if (cm.sourceIndex < 0) return 0;
        std::map<int, std::shared_ptr<ProcMaterial>>::iterator known = materials.find(cm.sourceIndex);
        if (known == materials.end()) {
            std::shared_ptr<ProcMaterial> pm = std::make_shared<ProcMaterial>();
            pm->metallic = cm.metallic;
            pm->roughness = cm.roughness;
            pm->opacity = cm.opacity;
            pm->emission = cm.emission;
            pm->twoSided = cm.doubleSided;
            const json extras = extrasOf(cm.extrasJson);
            auto image = [](const CpuImage& in, TextureData& out) {
                if (!in.valid()) return;
                out.width = in.width; out.height = in.height; out.channels = in.channels; out.pixels = in.pixels;
            };
            image(cm.baseColorTex, pm->albedo);
            image(cm.normalTex, pm->normal);
            pm->textured = !pm->albedo.pixels.empty() || !pm->normal.pixels.empty();
            pm->tile = extras.value("tile", pm->tile);
            pm->surface = extras.value("surface", 0);
            pm->alphaFoliage = extras.value("alpha_foliage", false);
            if (extras.contains("emission") && extras["emission"].is_array() && extras["emission"].size() == 3)
                pm->emission = Vec3(extras["emission"][0].get<double>(), extras["emission"][1].get<double>(), extras["emission"][2].get<double>());
            known = materials.emplace(cm.sourceIndex, pm).first;
        }
        int index = m.addMaterial(cm.name, known->second);
        if (index < 0) index = m.addMaterial("", known->second);   // two of the file's materials share a name
        return index;
    }

    // A node's mesh as parts of `m` on `node` at `level`, its points carried into the node's space by `rest`.
    void parts(ProcModel& m, int mesh, int node, int level, const Split& s) {
        if (mesh < 0) return;
        for (const CpuMesh& prim : tree.meshes[static_cast<std::size_t>(mesh)]) {
            ProcPart part;
            part.mesh = prim.geometry;
            if (!s.plain) MeshBuilder::transform(part.mesh, s.rest);
            part.material = materialFor(m, prim.material);
            part.node = node;
            part.level = level;
            m.parts.push_back(std::move(part));
        }
    }

    // The mesh of a `col_` object as one mesh in the object's own rigid frame.
    RenderMesh solid(int mesh, const Split& s) {
        RenderMesh out;
        if (mesh < 0) return out;
        for (const CpuMesh& prim : tree.meshes[static_cast<std::size_t>(mesh)]) MeshBuilder::append(out, prim.geometry);
        if (!s.plain) MeshBuilder::transform(out, s.rest);
        return out;
    }

    // Read the object `gi` and what is under it into `m`, whose node `parent` it hangs on. `toModel` carries the
    // file's world into `m`'s own space; `parentWorld` is the parent node's frame there.
    bool object(ProcModel& m, int gi, int parent, const Mat4& toModel, const Mat4& fileParent, const ProcFrame& parentFrame, bool isRoot,
                const std::string& parentObject = std::string()) {
        const CpuTreeNode& g = tree.nodes[static_cast<std::size_t>(gi)];
        const Mat4 fileWorld = fileParent * g.local;
        const Split s = split(toModel * fileWorld);                        // in the model's space
        const ProcFrame under{parentFrame.rotation.conjugate().rotate(s.frame.position - parentFrame.position),
                              parentFrame.rotation.conjugate() * s.frame.rotation};   // under the parent node
        const json extras = extrasOf(g.extrasJson);
        std::string base;
        int level = 0;

        // one copy of a used piece
        if (!isRoot && extras.contains("use") && extras["use"].is_number_integer()) {
            const int number = extras["use"].get<int>();
            if (!s.even) note("'" + g.name + "': a copy of a used piece is scaled unevenly; the largest scale is kept");
            if (pieces.count(number) == 0) {
                std::shared_ptr<ProcModel> piece = std::make_shared<ProcModel>();
                pieces[number] = piece;
                if (g.name.compare(0, 5, "piece") != 0) piece->name = g.name;
                // the piece's own space is this object's, scale and all taken off
                if (!object(*piece, gi, 0, (toModel * fileWorld).inverse() * toModel, fileParent, ProcFrame{}, true)) return false;
            }
            ProcUse* use = nullptr;
            const int useLevel = extras.value("level", 0);
            for (ProcUse& u : m.uses)
                if (u.piece == pieces[number] && u.node == parent && u.level == useLevel) use = &u;
            if (use == nullptr) {
                ProcUse fresh;
                fresh.piece = pieces[number];
                fresh.node = parent;
                fresh.level = useLevel;
                if (extras.contains("collide") && extras["collide"].is_object()) {
                    const json& c = extras["collide"];
                    const std::string shape = c.value("shape", std::string("mesh"));
                    fresh.collision = shape == "box" ? InstanceCollision::Box : shape == "sphere" ? InstanceCollision::Sphere
                                    : shape == "capsule" ? InstanceCollision::Capsule : InstanceCollision::Mesh;
                    if (c.contains("size") && c["size"].is_array() && c["size"].size() == 3)
                        fresh.colliderHalfExtent = Vec3(c["size"][0].get<double>(), c["size"][1].get<double>(), c["size"][2].get<double>()) * 0.5;
                    fresh.colliderRadius = c.value("radius", fresh.colliderRadius);
                    fresh.colliderHalfHeight = c.value("height", fresh.colliderHalfHeight * 2.0) * 0.5;
                    fresh.colliderFriction = c.value("friction", fresh.colliderFriction);
                }
                m.uses.push_back(fresh);
                use = &m.uses.back();
            }
            use->placements.push_back(ProcPlacement{under.position, under.rotation, s.scale});
            return true;
        }

        // a collision shape
        if (!isRoot && g.name.compare(0, 4, "col_") == 0) {
            const std::size_t bar = g.name.find('_', 4);
            const std::string kind = g.name.substr(4, bar == std::string::npos ? std::string::npos : bar - 4);
            std::string name = bar == std::string::npos ? std::string() : g.name.substr(bar + 1);
            if (!name.empty() && name[0] == '.') name.clear();             // Blender's ".001" on a shape with no name
            const RenderMesh mesh = solid(g.mesh, s);
            if (mesh.vertices.empty()) { note("'" + g.name + "': a collision shape with no mesh is left out"); return true; }
            Vec3 lo(1e30, 1e30, 1e30), hi(-1e30, -1e30, -1e30);
            for (const Vertex& v : mesh.vertices) {
                lo = Vec3(std::min(lo.x, v.position.x), std::min(lo.y, v.position.y), std::min(lo.z, v.position.z));
                hi = Vec3(std::max(hi.x, v.position.x), std::max(hi.y, v.position.y), std::max(hi.z, v.position.z));
            }
            const Vec3 size = hi - lo, mid = (lo + hi) * 0.5;
            ProcCollider c;
            c.name = name;
            c.node = parent;
            c.center = under.point(mid);
            c.rotation = under.rotation;
            c.friction = extras.value("friction", c.friction);
            c.dynamic = extras.value("dynamic", false);
            if (kind == "box") {
                c.kind = ProcCollider::Kind::Box;
                c.halfExtent = size * 0.5;
            } else if (kind == "sphere") {
                c.kind = ProcCollider::Kind::Sphere;
                c.radius = std::max({size.x, size.y, size.z}) * 0.5;
            } else if (kind == "capsule") {
                c.kind = ProcCollider::Kind::Capsule;                      // its length along the object's own y
                c.radius = std::max(size.x, size.z) * 0.5;
                c.halfHeight = std::max(Real(0), size.y * 0.5 - c.radius);
            } else {
                c.kind = kind == "hull" ? ProcCollider::Kind::Hull : ProcCollider::Kind::Mesh;
                if (kind != "hull" && kind != "mesh") note("'" + g.name + "': '" + kind + "' is not a kind of collision shape; read as a mesh");
                c.mesh = mesh;
                if (c.kind == ProcCollider::Kind::Hull) {                  // the points about the object's origin, the object placing them
                    c.center = under.position;
                } else {                                                   // a mesh shape is in its node's space
                    MeshBuilder::transform(c.mesh, under.matrix());
                    c.center = Vec3(0, 0, 0);
                    c.rotation = Quat();
                }
            }
            if (!m.addShape(c)) { error = "two collision shapes named '" + name + "'"; return false; }
            return true;
        }

        // a level of detail of a node
        if (!isRoot && lodName(g.name, base, level) && g.mesh >= 0) {
            // the object it is under, by the name the FILE gives that object (a model's root may be called anything)
            int node = base == parentObject || m.nodes[static_cast<std::size_t>(parent)].name == base ? parent : m.findNode(base);
            if (node < 0) {
                note("'" + g.name + "' names a node the model lacks; it hangs on '" + m.nodes[static_cast<std::size_t>(parent)].name + "'");
                node = parent;
            }
            while (static_cast<int>(m.levelFrom.size()) <= level) m.levelFrom.push_back(-1);   // distances are set at the end
            // its points into the node's space: through this object's own place under its parent
            Split inNode = s;
            if (node != parent || !near(under)) {
                const Mat4 nodeWorld = m.nodeFrame(node).matrix();
                inNode.rest = nodeWorld.inverse() * (toModel * fileWorld);
                inNode.plain = false;
            }
            parts(m, g.mesh, node, level, inNode);
            return true;
        }

        // a marker: an object with no mesh and nothing under it
        if (!isRoot && g.mesh < 0 && g.children.empty() && !extras.value("node", false) && !extras.contains("motion")) {
            ProcMarker mk;
            mk.name = g.name;
            mk.role = extras.value("role", std::string("point"));
            mk.size = extras.contains("size") ? extras.value("size", 0.0) : (std::abs(s.scale - 1) > 1e-6 ? s.scale : 0.0);
            mk.frame = under;
            if (!m.addMarker(parent, mk)) { error = "two markers named '" + g.name + "' (or a marker with no name)"; return false; }
            return true;
        }

        // a node
        int node = 0;
        ProcFrame frame = parentFrame;
        if (isRoot) {
            // the root is the model's own space: what the file gives it is carried into its parts and everything under it
            Split whole;
            whole.rest = toModel * fileWorld;
            whole.plain = split(whole.rest).plain && near(split(whole.rest).frame);
            parts(m, g.mesh, 0, 0, whole);
            frame = ProcFrame{};
        } else {
            if (!s.plain) note("'" + g.name + "' is scaled in the file; its scale is put into its mesh");
            if (s.mirrored) note("'" + g.name + "' is mirrored in the file; its mesh is read as it stands (a mirrored twin is m:mirrored())");
            node = m.addNode(g.name, parent, under);
            if (node < 0) { error = g.name.empty() ? "an object with no name" : "two nodes named '" + g.name + "'"; return false; }
            frame = s.frame;
            parts(m, g.mesh, node, 0, s);
        }
        if (extras.contains("motion") && extras["motion"].is_object() && node != 0) {
            const json& j = extras["motion"];
            ProcMotion motion;
            const bool turn = j.value("kind", std::string("turn")) != "slide";
            motion.kind = turn ? ProcMotion::Kind::Turn : ProcMotion::Kind::Slide;
            if (j.contains("axis") && j["axis"].is_array() && j["axis"].size() == 3)
                motion.axis = normalize(Vec3(j["axis"][0].get<double>(), j["axis"][1].get<double>(), j["axis"][2].get<double>()));
            if (j.contains("min") && j.contains("max")) {
                motion.limited = true;
                motion.min = j["min"].get<double>() * (turn ? DEG_TO_RAD : 1.0);
                motion.max = j["max"].get<double>() * (turn ? DEG_TO_RAD : 1.0);
            }
            m.nodes[static_cast<std::size_t>(node)].motion = motion;
        }
        if (isRoot && extras.contains("levels") && extras["levels"].is_array()) {
            m.levelFrom.clear();
            for (const json& d : extras["levels"]) m.levelFrom.push_back(d.get<double>());
            if (m.levelFrom.empty() || m.levelFrom[0] != 0) { m.levelFrom = {0}; note("the root's `levels` must start at 0; ignored"); }
        }
        for (int child : g.children)
            if (!object(m, child, node, toModel, fileWorld, frame, false, g.name)) return false;
        return true;
    }

    static bool near(const ProcFrame& f) {
        return f.position.length() < 1e-9 && std::abs(f.rotation.x) < 1e-9 && std::abs(f.rotation.y) < 1e-9 && std::abs(f.rotation.z) < 1e-9;
    }
};

// Levels that a file named (by `_lod<N>`) but gave no distance for: each twice as far as the last, the first at 40 m.
void fillLevelDistances(ProcModel& m, std::vector<std::string>* notes) {
    bool filled = false;
    for (std::size_t i = 1; i < m.levelFrom.size(); ++i)
        if (!(m.levelFrom[i] > m.levelFrom[i - 1])) {
            m.levelFrom[i] = i == 1 ? 40.0 : m.levelFrom[i - 1] * 2.0;
            filled = true;
        }
    if (filled && notes != nullptr) notes->push_back("the file gives no distance for a level of detail; 40 m and doubling were used (say `distances`)");
    for (const ProcUse& use : m.uses)
        if (use.piece != nullptr) fillLevelDistances(const_cast<ProcModel&>(*use.piece), nullptr);
}

}  // namespace

bool writeModelGlb(const ProcModel& m, const std::string& path, std::string* error) {
    Writer w;
    GlbTreeNode top;
    top.name = "root";
    std::size_t written = 0;
    if (!w.model(m, top, nullptr, nullptr, written)) {
        if (error != nullptr) *error = w.error;
        return false;
    }
    json scene = json::object();
    scene["what"] = "a model (raytracer-clang, AGENTS.md \"Model file names\")";
    if (!m.name.empty()) scene["name"] = m.name;
    return writeGlbTree(w.nodes, w.materials, path, scene, error);
}

std::shared_ptr<ProcModel> readModelGlb(const std::string& path, std::string* error, std::vector<std::string>* notes) {
    const CpuTree tree = ModelImporter::loadCpuTree(path);
    if (!tree.ok) {
        if (error != nullptr) *error = tree.error;
        return nullptr;
    }
    Reader r{tree, notes, std::string(), {}, {}, 0};
    std::shared_ptr<ProcModel> m = std::make_shared<ProcModel>();
    m->name = extrasOf(tree.sceneExtrasJson).value("name", std::string());
    bool ok = true;
    const bool ownRoot = tree.roots.size() == 1 && tree.nodes[static_cast<std::size_t>(tree.roots[0])].name == "root";
    if (ownRoot) {
        ok = r.object(*m, tree.roots[0], 0, Mat4::identity(), Mat4::identity(), ProcFrame{}, true);
    } else {
        for (int top : tree.roots)
            if (ok) ok = r.object(*m, top, 0, Mat4::identity(), Mat4::identity(), ProcFrame{}, false);
    }
    if (!ok) {
        if (error != nullptr) *error = path + ": " + r.error;
        return nullptr;
    }
    fillLevelDistances(*m, notes);
    return m;
}

std::shared_ptr<ProcModel> loadModelFiles(const std::vector<std::string>& paths, const std::vector<Real>& distances, std::string* error,
                                          std::vector<std::string>* notes) {
    if (paths.empty()) {
        if (error != nullptr) *error = "no files given";
        return nullptr;
    }
    std::shared_ptr<ProcModel> m = readModelGlb(paths[0], error, notes);
    if (m == nullptr) return nullptr;
    for (std::size_t i = 1; i < paths.size(); ++i) {
        const std::shared_ptr<ProcModel> lower = readModelGlb(paths[i], error, notes);
        if (lower == nullptr) return nullptr;
        if (m->levelFrom.size() != i) {
            if (error != nullptr) *error = paths[i] + ": level " + std::to_string(i + 1) + " is given as a file, but the files before it already hold " +
                                           std::to_string(m->levelFrom.size()) + " levels";
            return nullptr;
        }
        m->levelFrom.push_back(-1);
        const int level = static_cast<int>(i);
        std::vector<int> materialOf(lower->materials.size(), 0);
        for (std::size_t k = 1; k < lower->materials.size(); ++k) {
            materialOf[k] = m->addMaterial(lower->materials[k].name, lower->materials[k].material);
            if (materialOf[k] < 0) materialOf[k] = m->addMaterial("", lower->materials[k].material);   // the name means another material here
        }
        auto nodeFor = [&](int theirs, bool& moved) {
            moved = false;
            if (theirs == 0) return 0;
            const std::string& name = lower->nodes[static_cast<std::size_t>(theirs)].name;
            const int mine = m->findNode(name);
            if (mine >= 0) return mine;
            if (notes != nullptr) notes->push_back(paths[i] + ": '" + name + "' is not a node of " + paths[0] + "; what is on it hangs on the root");
            moved = true;
            return 0;
        };
        for (const ProcPart& part : lower->parts) {
            if (part.level != 0) continue;
            ProcPart mine = part;
            bool moved = false;
            mine.node = nodeFor(part.node, moved);
            if (moved) MeshBuilder::transform(mine.mesh, lower->nodeFrame(part.node).matrix());   // where it stood in its own file
            mine.material = materialOf[static_cast<std::size_t>(part.material)];
            mine.level = level;
            m->parts.push_back(std::move(mine));
        }
        for (const ProcUse& use : lower->uses) {
            if (use.level != 0) continue;
            ProcUse mine = use;
            bool moved = false;
            mine.node = nodeFor(use.node, moved);
            if (moved) {
                const ProcFrame at = lower->nodeFrame(use.node);
                for (ProcPlacement& p : mine.placements) { p.position = at.point(p.position); p.rotation = at.rotation * p.rotation; }
            }
            mine.level = level;
            m->uses.push_back(std::move(mine));
        }
    }
    if (!distances.empty()) {
        bool rising = distances.size() == m->levelFrom.size() && distances[0] == 0;
        for (std::size_t i = 1; rising && i < distances.size(); ++i) rising = distances[i] > distances[i - 1];
        if (!rising) {
            if (error != nullptr) *error = "`distances` must have one entry per level (" + std::to_string(m->levelFrom.size()) + "), start at 0 and rise";
            return nullptr;
        }
        m->levelFrom = distances;
    }
    fillLevelDistances(*m, notes);
    return m;
}

}  // namespace engine
