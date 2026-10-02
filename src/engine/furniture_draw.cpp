#include "furniture_draw.h"
#include "asset_manager.h"
#include "components.h"
#include "procgen/furniture_kit.h"
#include "procgen/surface_maps.h"
#include <algorithm>
#include <map>

namespace engine {

std::size_t spawnFurnitureGroups(World& world, AssetManager& assets, Renderer* renderer, FurnitureDrawCache& cache,
                                 const std::vector<PlacedPiece>& pieces, const Vec3& tint, double drawDistance,
                                 std::vector<Entity>& out) {
    std::size_t tris = 0;
    std::map<uint64_t, std::vector<Mat4>> byPiece;
    for (const PlacedPiece& pp : pieces)
        byPiece[(static_cast<uint64_t>(pp.piece) << 32) | pp.variant].push_back(pp.xform);
    static const PartId kFinishPart[kFurnMatCount] = {PartId::FurnitureWood, PartId::FurnitureFabric,
                                                      PartId::Furniture, PartId::FurnitureMetal,
                                                      PartId::FurnitureCeramic};
    for (const auto& [key, xforms] : byPiece) {
        const Piece pc = static_cast<Piece>(key >> 32);
        const uint32_t variant = static_cast<uint32_t>(key & 0xffffffffu);
        const FurniturePiece& kit = furniturePiece(pc, variant);
        for (int f = 0; f < kFurnMatCount; ++f) {
            const RenderMesh& pm = kit.mesh[static_cast<std::size_t>(f)];
            if (pm.vertices.empty()) continue;
            const uint64_t mk = (key << 4) | static_cast<uint64_t>(f);
            auto mit = cache.pieceMesh.find(mk);
            if (mit == cache.pieceMesh.end()) mit = cache.pieceMesh.emplace(mk, assets.acquireMesh(pm, "")).first;
            InstanceGroup g;
            g.mesh = mit->second;
            g.material = materialFor(kFinishPart[f], tint);
            const RenderMaterial::Surface surf = g.material.surface();
            if (renderer && surf != RenderMaterial::Surface::None) {
                const int sid = static_cast<int>(surf);
                auto it = cache.surfTex.find(sid);
                if (it == cache.surfTex.end()) {
                    SurfaceMaps mp = surfaceMaps(surf, 256, 1337u);
                    auto up = [&](const TextureData& td) {
                        return renderer->uploadTexture(td.width, td.height, td.channels, td.pixels.data());
                    };
                    it = cache.surfTex.emplace(sid, std::array<TextureHandle, 4>{up(mp.albedo), up(mp.normal),
                                                                            up(mp.mr), up(mp.ao)}).first;
                }
                g.material.albedoMap = it->second[0];
                g.material.normalMap = it->second[1];
                g.material.metallicRoughnessMap = it->second[2];
                g.material.aoMap = it->second[3];
            }
            g.transforms = xforms;
            Vec3 lo(1e30, 1e30, 1e30), hi(-1e30, -1e30, -1e30);
            for (const Mat4& t : xforms) {
                const Vec3 o(t.m[0][3], t.m[1][3], t.m[2][3]);
                lo = Vec3(std::min(lo.x, o.x), std::min(lo.y, o.y), std::min(lo.z, o.z));
                hi = Vec3(std::max(hi.x, o.x), std::max(hi.y, o.y), std::max(hi.z, o.z));
            }
            g.boundsCenter = (lo + hi) * 0.5;
            g.boundsRadius = (hi - lo).length() * 0.5 + 3.0;
            g.drawDistance = drawDistance;
            g.drawClass = DrawClass::Structure;
            Entity ge = world.create();
            world.add<InstanceGroup>(ge, std::move(g));
            out.push_back(ge);
            tris += pm.indices.size() / 3 * xforms.size();
        }
    }
    return tris;
}

}  // namespace engine
