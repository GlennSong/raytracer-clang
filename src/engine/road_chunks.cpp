#include "road_chunks.h"

#include "asset_manager.h"
#include "components.h"
#include "mesh_builder.h"
#include "procgen/city/road_net.h"

namespace engine {

int rebuildRoadRenderChunks(World& world, AssetManager& assets, Entity road,
                            const RoadNet& net, double cell,
                            const RenderMesh* prebuilt) {
    // Tear down the previous set first: destroy the companions and release
    // their meshes, so a regenerate is idempotent rather than accumulative.
    if (RenderChunks* prev = world.get<RenderChunks>(road)) {
        for (Entity e : prev->entities) {
            if (Renderable* r = world.get<Renderable>(e))
                if (r->mesh.valid()) assets.releaseMesh(r->mesh);
            world.destroy(e);
        }
        prev->entities.clear();
    }

    RenderMesh built;
    const RenderMesh* mesh = prebuilt;
    if (!mesh) {
        built = buildRoadNetMesh(net);
        mesh = &built;
    }
    if (mesh->vertices.empty()) {
        if (!world.get<RenderChunks>(road)) world.add<RenderChunks>(road, {});
        return 0;
    }

    // The one place the carriageway's material is configured: hue rides in
    // vertex colour; lane paint is the RoadMarkings surface shader.
    RenderMaterial mat;
    mat.albedo = Vec3(1, 1, 1);
    mat.roughness = 0.93f;
    if (net.markings) mat.setSurface(RenderMaterial::Surface::RoadMarkings);

    RenderChunks chunks;
    for (RenderMesh& piece : MeshBuilder::chunkByCell(*mesh, cell)) {
        if (piece.vertices.empty()) continue;
        Entity e = world.create();
        Transform t;                       // identity — the mesh is world-space
        world.add<Transform>(e, t);
        world.add<PrevTransform>(e, PrevTransform{t});
        Renderable r;
        r.renderLayer = LayerRoads;
        r.material = mat;
        r.mesh = assets.acquireMesh(piece, "");   // unkeyed: released on rebuild
        world.add<Renderable>(e, r);
        world.add<PickTarget>(e, PickTarget{road});
        chunks.entities.push_back(e);
    }
    const int n = static_cast<int>(chunks.entities.size());
    if (RenderChunks* rc = world.get<RenderChunks>(road)) *rc = std::move(chunks);
    else world.add<RenderChunks>(road, std::move(chunks));
    return n;
}

}  // namespace engine
