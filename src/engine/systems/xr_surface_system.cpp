#include "xr_surface_system.h"

#include <cstdlib>

#include "../../log.h"

namespace engine {

namespace {

// MeshHandle <-> the ledger's opaque token. The ledger stays renderer-free
// (and therefore host-testable) by never seeing a MeshHandle; a Handle is two
// uint32s, so it packs losslessly. generation != 0 for any valid handle, so a
// packed token is never the ledger's "no mesh" zero.
uint64_t packMesh(MeshHandle handle) {
    return (static_cast<uint64_t>(handle.generation) << 32) | handle.index;
}
MeshHandle unpackMesh(uint64_t token) {
    MeshHandle handle;
    handle.index = static_cast<uint32_t>(token & 0xffffffffu);
    handle.generation = static_cast<uint32_t>(token >> 32);
    return handle;
}

// An anchor's geometry as the renderer wants it: classification tint in the
// vertex color (the debug material's albedo stays white), flat normals
// accumulated from faces when the runtime sent none.
RenderMesh buildSurfaceMesh(const XrSurfaceUpdate& update) {
    RenderMesh mesh;
    const Vec3 tint = xrSurfaceClassColor(update.cls);
    const bool haveNormals = update.normals.size() == update.positions.size();

    mesh.vertices.reserve(update.positions.size());
    for (size_t i = 0; i < update.positions.size(); i++) {
        Vertex v(update.positions[i],
                 haveNormals ? update.normals[i] : Vec3(0, 0, 0));
        v.color = tint;
        mesh.vertices.push_back(v);
    }
    mesh.indices = update.indices;

    if (!haveNormals) {
        for (size_t t = 0; t + 2 < mesh.indices.size(); t += 3) {
            Vertex& a = mesh.vertices[mesh.indices[t]];
            Vertex& b = mesh.vertices[mesh.indices[t + 1]];
            Vertex& c = mesh.vertices[mesh.indices[t + 2]];
            const Vec3 n = cross(b.position - a.position,
                                 c.position - a.position);
            a.normal += n;
            b.normal += n;
            c.normal += n;
        }
        for (Vertex& v : mesh.vertices) {
            const Real len = v.normal.length();
            v.normal = (len > 1e-12) ? v.normal / len : Vec3(0, 1, 0);
        }
    }
    return mesh;
}

}  // namespace

XrSurfaceLedger::MeshOps XrSurfaceSystem::meshOps(FrameContext& ctx) {
    return {
        [&ctx](const XrSurfaceUpdate& update) {
            return packMesh(ctx.renderer.uploadMesh(buildSurfaceMesh(update)));
        },
        [&ctx](uint64_t token) { ctx.renderer.removeMesh(unpackMesh(token)); },
    };
}

void XrSurfaceSystem::onStart(FrameContext&) {
    const char* env = std::getenv("RT_XR_SURFACES");
    visible_ = !(env && env[0] == '0');
}

void XrSurfaceSystem::update(FrameContext& ctx) {
    XrSurfaceStore* store = ctx.renderer.xrSurfaceStore();
    if (!store) return;

    drainScratch_.clear();
    store->drain(drainScratch_);
    if (!drainScratch_.empty()) {
        // Cache plane outlines before the ledger consumes the updates. Only
        // classified planes get outlines — the room mesh's boundary is chunk
        // seams, which read as noise, not structure.
        for (const XrSurfaceUpdate& u : drainScratch_) {
            if (u.op == XrSurfaceUpdate::Op::Removed) {
                outlines_.erase(u.anchorId);
                continue;
            }
            if (u.cls == XrSurfaceClass::Mesh || u.indices.empty()) continue;
            auto& segments = outlines_[u.anchorId];
            segments.clear();
            for (const auto& [ia, ib] : xrSurfaceBoundaryEdges(u.indices))
                segments.emplace_back(u.positions[ia], u.positions[ib]);
        }
        ledger_.apply(drainScratch_, meshOps(ctx));
    }

    // The numeric readout — how surface mapping is verified without seeing the
    // render. Logged when the census changes shape and as a slow heartbeat.
    const auto census = ledger_.census();
    const bool beat = (++frame_ % 450) == 0;   // ~5 s at 90 Hz
    if (census.total != lastLoggedTotal_ || (beat && census.total > 0)) {
        lastLoggedTotal_ = census.total;
        auto line = LOG_INFO;
        line << "[xr] surfaces: total=" << census.total;
        for (int i = 0; i < XR_SURFACE_CLASS_COUNT; i++) {
            if (census.countByClass[i] == 0) continue;
            line << " " << xrSurfaceClassName(static_cast<XrSurfaceClass>(i))
                 << "=" << census.countByClass[i];
        }
        line << " tris=" << census.triangles;
        if (census.floorValid) line << " floorY=" << census.floorY << "m";
    }
}

void XrSurfaceSystem::render(FrameContext& ctx) {
    if (!visible_ || ledger_.surfaces().empty()) return;
    if (!ctx.xr.active || !ctx.xr.originBaseValid) return;

    const Real scale = (ctx.renderer.xrWorldScale > 0.01f)
        ? static_cast<Real>(ctx.renderer.xrWorldScale) : 1.0;

    RenderMaterial material;
    material.albedo = Vec3(1, 1, 1);   // tint rides Vertex::color
    material.metallic = 0.0f;
    material.roughness = 1.0f;
    material.flags = RenderMaterial::FLAG_TWO_SIDED;

    for (const auto& [anchorId, entry] : ledger_.surfaces()) {
        const Mat4 world = xrSurfaceWorldTransform(ctx.xr.originBase, scale,
                                                   entry.originFromAnchor);
        if (entry.meshToken)
            ctx.renderer.drawMesh(unpackMesh(entry.meshToken), world, material);

        auto outline = outlines_.find(anchorId);
        if (outline != outlines_.end()) {
            const Vec3 color = xrSurfaceClassColor(entry.cls);
            for (const auto& [a, b] : outline->second)
                ctx.debug.line(world.transformPoint(a), world.transformPoint(b),
                               color);
            // Normal tick: ARKit planes face +Y in anchor space. 10 real cm.
            const Vec3 foot = world.transformPoint(Vec3(0, 0, 0));
            const Vec3 tip = world.transformPoint(Vec3(0, 0.1, 0));
            ctx.debug.line(foot, tip, color);
        }
    }
}

void XrSurfaceSystem::onStop(FrameContext& ctx) {
    ledger_.clear(meshOps(ctx));
    outlines_.clear();
    lastLoggedTotal_ = -1;
}

}  // namespace engine
