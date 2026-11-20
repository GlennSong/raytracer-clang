// A model into the world (#191, step 7). See model_spawn.h.
#include "model_spawn.h"

#include "../log.h"
#include "asset_manager.h"
#include "procgen/surface_maps.h"
#include "procgen/tree.h"   // leafTexture

#include <cmath>

namespace engine {

void applyWorldPlanarUVs(RenderMesh& mesh, double scale) {
    for (Vertex& vert : mesh.vertices) {
        const Vec3& p = vert.position;
        const Vec3& n = vert.normal;
        double u, v; Vec3 T;
        if (std::fabs(n.y) > 0.5) {
            u = p.x * scale; v = p.z * scale; T = Vec3(1, 0, 0);
        } else {
            double tx = n.z, tz = -n.x, tl = std::sqrt(tx * tx + tz * tz);
            if (tl < 1e-6) { tx = 1; tz = 0; tl = 1; }
            tx /= tl; tz /= tl;
            u = (p.x * tx + p.z * tz) * scale; v = p.y * scale; T = Vec3(tx, 0, tz);
        }
        vert.u = static_cast<float>(u);
        vert.v = static_cast<float>(v);
        vert.tangent = T;
    }
}

std::array<TextureHandle, 4> bakeSurfaceTextures(Renderer& renderer, RenderMaterial::Surface surf, SurfaceTexCache& cache) {
    int id = static_cast<int>(surf);
    SurfaceTexCache::iterator it = cache.find(id);
    if (it != cache.end()) return it->second;
    SurfaceMaps mp = surfaceMaps(surf, 256, 1337u);
    auto up = [&](const TextureData& td) {
        return renderer.uploadTexture(td.width, td.height, td.channels, td.pixels.data());
    };
    std::array<TextureHandle, 4> h{up(mp.albedo), up(mp.normal), up(mp.mr), up(mp.ao)};
    cache[id] = h;
    return h;
}

void bindSurfaceMaps(RenderMaterial& mat, const std::array<TextureHandle, 4>& h) {
    mat.albedoMap = h[0];
    mat.normalMap = h[1];
    mat.metallicRoughnessMap = h[2];
    mat.aoMap = h[3];
}

bool surfaceIsBakedOnMeshUVs(RenderMaterial::Surface s) {
    return s == RenderMaterial::Surface::WoodGrain || s == RenderMaterial::Surface::Fabric ||
           s == RenderMaterial::Surface::Marble || s == RenderMaterial::Surface::Carpet;
}

MeshHandle ModelDresser::dress(const ProcMaterial& made, const RenderMesh& mesh, const std::string& meshKey, RenderMaterial& out) {
    auto upload = [&](const TextureData& td) -> TextureHandle {
        return renderer_.uploadTexture(td.width, td.height, td.channels, td.pixels.data());
    };
    out.albedo = Vec3(1, 1, 1);    // hue carried in vertex colour
    out.metallic = made.metallic;
    out.roughness = made.roughness;
    out.opacity = made.opacity;
    out.emission = made.emission;
    if (made.twoSided)
        out.flags |= RenderMaterial::FLAG_TWO_SIDED;
    // Analytic surface library (renderer.h Surface): an id evaluated in-shader
    // from the mesh's own UVs — e.g. RoadMarkings paints lane lines from the
    // road-local u/v baked onto the carriageway. Independent of texture maps,
    // so it rides whether or not the part is `textured`.
    if (made.surface != 0)
        out.setSurface(static_cast<RenderMaterial::Surface>(made.surface));
    if (!made.textured && surfaceIsBakedOnMeshUVs(out.surface()))
        bindSurfaceMaps(out, bakeSurfaceTextures(renderer_, out.surface(), surfaceTex_));
    const bool known = uploaded_.count(&made) != 0;
    MaterialMaps& maps = uploaded_[&made];
    if (!known) {
        if (made.textured && !made.albedo.pixels.empty()) maps.albedo = upload(made.albedo);
        if (made.textured && !made.normal.pixels.empty()) maps.normal = upload(made.normal);
        if (made.alphaFoliage) maps.leaf = upload(leafTexture(128));
    }
    if (made.alphaFoliage) {              // alpha-cut leaf cards
        out.albedoMap = maps.leaf;
        out.flags |= RenderMaterial::FLAG_ALPHA_TEST;
    }
    if (!made.textured) return assets_.acquireMesh(mesh, meshKey);
    RenderMesh tiled = mesh;
    double tile = made.tile > 1e-6 ? made.tile : 1.0;
    applyWorldPlanarUVs(tiled, 1.0 / tile);
    if (!made.albedo.pixels.empty()) out.albedoMap = maps.albedo;
    if (!made.normal.pixels.empty()) out.normalMap = maps.normal;
    return assets_.acquireMesh(tiled, meshKey);
}

std::vector<Entity> spawnModelColliders(World& world, const ProcModel& model, const ProcFrame* at, Real frictionAll, bool movingAreKinematic,
                                        std::vector<int>* colliderOf) {
    std::vector<Entity> made;
    if (colliderOf != nullptr) colliderOf->clear();
    auto note = [&](Entity e, int collider) {
        made.push_back(e);
        if (colliderOf != nullptr) colliderOf->push_back(collider);
    };
    const Mat4 atMatrix = at != nullptr ? at->matrix() : Mat4::identity();
    auto frictionOf = [&](Real own) { return frictionAll >= 0 ? frictionAll : own; };
    auto appendTris = [](MeshCollider& mc, const RenderMesh& rm, const Mat4* xf) {
        for (std::size_t i = 0; i + 2 < rm.indices.size(); i += 3) {
            Vec3 a = rm.vertices[rm.indices[i]].position;
            Vec3 b = rm.vertices[rm.indices[i + 1]].position;
            Vec3 c = rm.vertices[rm.indices[i + 2]].position;
            if (xf) { a = xf->transformPoint(a); b = xf->transformPoint(b); c = xf->transformPoint(c); }
            if (cross(b - a, c - a).length() < 1e-5) continue;   // skip slivers
            uint32_t base = static_cast<uint32_t>(mc.vertices.size());
            mc.vertices.push_back(a); mc.vertices.push_back(b); mc.vertices.push_back(c);
            mc.indices.push_back(base); mc.indices.push_back(base + 1); mc.indices.push_back(base + 2);
        }
    };
    auto spawnMeshCollider = [&](MeshCollider&& mc) {
        if (mc.indices.empty()) return;
        Entity e = world.create();
        Transform t;
        world.add<Transform>(e, t);
        world.add<PrevTransform>(e, PrevTransform{t});
        world.add<MeshCollider>(e, std::move(mc));
        note(e, -1);
    };
    auto spawnPrimitive = [&](ColliderShape shape, const Vec3& center, const Quat& turn, const ProcCollider& src, int collider = -1,
                              bool kinematic = false) {
        Entity e = world.create();
        Transform t;
        t.position = center;
        t.orientation = turn;
        world.add<Transform>(e, t);
        world.add<PrevTransform>(e, PrevTransform{t});
        Collider c;
        c.shape = shape;
        c.halfExtent = src.halfExtent;
        c.radius = src.radius;
        c.halfHeight = src.halfHeight;
        c.friction = frictionOf(src.friction);
        world.add<Collider>(e, c);
        RigidBody rb;
        rb.motion = src.dynamic ? BodyMotion::Dynamic : (kinematic ? BodyMotion::Kinematic : BodyMotion::Static);
        world.add<RigidBody>(e, rb);
        note(e, collider);
    };
    // Static triangle-mesh colliders are MERGED by friction into one body each:
    // a city is hundreds of solids (every building, plinth, lawn, the terrain, the
    // roads), and one static mesh body per surface keeps the physics broadphase
    // cheap (a few bodies, each with its own BVH) instead of hundreds of bodies.
    // Primitives (a round tower, a lamp post) stay per-entity — there are few.
    std::vector<std::pair<Real, MeshCollider>> meshBuckets;
    auto bucketFor = [&](Real friction) -> MeshCollider& {
        for (std::pair<Real, MeshCollider>& b : meshBuckets)
            if (std::abs(b.first - friction) < 1e-4) return b.second;
        meshBuckets.emplace_back(friction, MeshCollider{});
        meshBuckets.back().second.friction = friction;
        return meshBuckets.back().second;
    };
    // A mesh's triangles in the world: through its node (when it is not on the root) and through `at` (when given).
    // With neither there is no matrix at all, so a recipe entity's root shapes are exactly the recipe's triangles.
    auto appendPlaced = [&](Real friction, const RenderMesh& tris, int node) {
        const bool onRoot = node == 0;
        if (onRoot && at == nullptr) { appendTris(bucketFor(friction), tris, nullptr); return; }
        const Mat4 xf = onRoot ? atMatrix : (at != nullptr ? atMatrix * model.nodeFrame(node).matrix() : model.nodeFrame(node).matrix());
        appendTris(bucketFor(friction), tris, &xf);
    };
    for (std::size_t ci = 0; ci < model.colliders.size(); ++ci) {
        const ProcCollider& pc = model.colliders[ci];
        const int index = static_cast<int>(ci);
        const bool moves = movingAreKinematic && model.nodeMoves(pc.node);
        if (moves && (pc.kind == ProcCollider::Kind::Mesh || pc.kind == ProcCollider::Kind::Hull))
            LOG_WARN << "collision shape '" << pc.name << "' is a mesh or hull on a node that moves: it stays where it is at rest (no moving mesh or hull shape yet)";
        // A shape stands at the node it is on. The root's frame is no move and no turn, so a shape on it is where
        // the recipe put it, as every collider was before nodes.
        const bool onRoot = pc.node == 0;
        ProcFrame placed = onRoot ? pc.frame() : model.nodeFrame(pc.node) * pc.frame();
        if (at != nullptr) placed = *at * placed;
        switch (pc.kind) {
            case ProcCollider::Kind::Mesh:
                appendPlaced(frictionOf(pc.friction), pc.mesh, pc.node);
                break;
            case ProcCollider::Kind::Hull:
                // The physics has no hull shape yet: a hull on static scenery is its faces as triangles, which
                // collide the same. A hull on a body that moves waits for bodies that move (#191 step 8).
                appendPlaced(frictionOf(pc.friction), shapeMesh(pc), pc.node);
                break;
            case ProcCollider::Kind::Box:
                spawnPrimitive(ColliderShape::Box, placed.position, placed.rotation, pc, index, moves); break;
            case ProcCollider::Kind::Sphere:
                spawnPrimitive(ColliderShape::Sphere, placed.position, placed.rotation, pc, index, moves); break;
            case ProcCollider::Kind::Capsule:
                spawnPrimitive(ColliderShape::Capsule, placed.position, placed.rotation, pc, index, moves); break;
        }
    }

    // Colliders of used pieces: stamp a collider at each copy (a lamp per verge).
    // Mesh stamps merge into the friction buckets; primitives are one body each.
    for (const ProcPlacedUse& pu : placedUses(model)) {
        const ProcUse& cg = *pu.use;
        if (cg.collision == InstanceCollision::None || pu.transforms.empty()) continue;
        if (cg.collision == InstanceCollision::Mesh) {
            const RenderMesh whole = cg.collisionProto.vertices.empty() ? flattenModel(*cg.piece) : RenderMesh();
            const RenderMesh& proto = cg.collisionProto.vertices.empty() ? whole : cg.collisionProto;
            MeshCollider& mc = bucketFor(frictionOf(cg.colliderFriction));
            for (const Mat4& xf : pu.transforms) {
                if (at == nullptr) { appendTris(mc, proto, &xf); continue; }
                const Mat4 world = atMatrix * xf;
                appendTris(mc, proto, &world);
            }
            continue;
        }
        ColliderShape shape = cg.collision == InstanceCollision::Box ? ColliderShape::Box
                            : cg.collision == InstanceCollision::Sphere ? ColliderShape::Sphere
                                                                        : ColliderShape::Capsule;
        ProcCollider tmpl;
        tmpl.halfExtent = cg.colliderHalfExtent;
        tmpl.radius = cg.colliderRadius;
        tmpl.halfHeight = cg.colliderHalfHeight;
        tmpl.friction = cg.colliderFriction;
        // The proto stands on its origin, so lift the collider centre to wrap a
        // prop sitting on the ground rather than sinking it half-under.
        Real lift = shape == ColliderShape::Box ? cg.colliderHalfExtent.y
                  : shape == ColliderShape::Sphere ? cg.colliderRadius
                                                   : cg.colliderHalfHeight + cg.colliderRadius;
        for (const Mat4& xf : pu.transforms) {
            Vec3 pos(xf.m[0][3], xf.m[1][3], xf.m[2][3]);
            if (at == nullptr) { spawnPrimitive(shape, pos + Vec3(0, lift, 0), Quat(), tmpl); continue; }
            const ProcFrame placed = *at * ProcFrame{pos + Vec3(0, lift, 0), Quat()};
            spawnPrimitive(shape, placed.position, placed.rotation, tmpl);
        }
    }
    for (std::pair<Real, MeshCollider>& b : meshBuckets) spawnMeshCollider(std::move(b.second));
    return made;
}

}  // namespace engine
