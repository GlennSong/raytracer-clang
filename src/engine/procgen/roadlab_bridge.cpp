#include "roadlab_bridge.h"

// Explicit paths: src/scene.h and src/engine/.../tessellate would otherwise
// shadow these, since -Isrc precedes -Iproto/roadlab. roadlab's own headers
// include each other unqualified, which resolves relative to their own
// directory, so only this first hop needs spelling out.
#include "../../../proto/roadlab/scene.h"
#include "../../../proto/roadlab/tessellate.h"

#include <chrono>

namespace roadlab_bridge {

namespace {

// roadlab works in doubles throughout — it is an offline prototype and the
// precision is free there. The engine wants floats, and this is the only place
// the narrowing happens.
engine::Vec3 toEngine(const roadlab::Vec3& v) {
    return engine::Vec3(static_cast<float>(v.x), static_cast<float>(v.y), static_cast<float>(v.z));
}

// Split one roadlab mesh into per-material RenderMeshes, remapping indices as we
// go. A triangle belongs to whichever material its first vertex carries; the
// tessellator never mixes materials within a triangle, and the invariant is
// worth stating because a silent mix would show up as one stray asphalt tri in
// the middle of a field.
// Which roads are CONNECTORS — the per-turning-movement curves a junction
// generates. roadlab draws them as full road ribbons lying on the junction pad,
// which its own software rasterizer tolerates but a depth buffer does not: two
// coplanar opaque surfaces tear against each other, and a four-arm junction has
// twelve of them stacked on one pad. The pad is the junction's visible surface;
// the connectors are its topology, and the lane graph still has them.
std::vector<char> connectorRoads(const roadlab::Network& net) {
    std::vector<char> isConn(size_t(net.roadCount()), 0);
    for (const roadlab::Road& r : net.roads())
        if (r.kind == roadlab::RoadKind::Connector && r.id >= 0 &&
            size_t(r.id) < isConn.size())
            isConn[size_t(r.id)] = 1;
    return isConn;
}

void split(const roadlab::Mesh& src, Baked& out, const std::vector<char>& isConn) {
    struct Bucket {
        engine::RenderMesh* mesh;
        std::vector<int> remap;
    };
    Bucket buckets[3] = {{&out.roadSurface, {}}, {&out.junctionPads, {}}, {&out.flat, {}}};
    for (Bucket& b : buckets) b.remap.assign(src.verts.size(), -1);

    for (size_t i = 0; i + 2 < src.indices.size(); i += 3) {
        const roadlab::Vertex& first = src.verts[src.indices[i]];
        if (first.road >= 0 && size_t(first.road) < isConn.size() &&
            isConn[size_t(first.road)])
            continue;
        int kind = static_cast<int>(first.material);
        if (kind < 0 || kind > 2) kind = 2;
        Bucket& b = buckets[kind];
        for (int k = 0; k < 3; ++k) {
            uint32_t vi = src.indices[i + k];
            int& slot = b.remap[vi];
            if (slot < 0) {
                const roadlab::Vertex& rv = src.verts[vi];
                engine::Vertex v;
                v.position = toEngine(rv.pos);
                v.normal = toEngine(rv.normal);
                // (t, s) in METRES. The engine's own road mesh bakes a
                // half-width-normalised U instead, which is exactly the
                // limitation this prototype exists to remove — paint that
                // stretches when a road widens, and a lane that appears having
                // nowhere to be drawn.
                v.u = static_cast<float>(rv.t);
                v.v = static_cast<float>(rv.s);
                v.color = toEngine(rv.color);
                slot = static_cast<int>(b.mesh->vertices.size());
                b.mesh->vertices.push_back(v);
            }
            b.mesh->indices.push_back(static_cast<uint32_t>(slot));
        }
    }
}

}  // namespace

bool build(const Recipe& recipe, Baked& out, std::string& error) {
    auto t0 = std::chrono::steady_clock::now();
    roadlab::Scene sc;

    if (recipe.generator == "metro") {
        roadlab::MetroParams mp;
        mp.seed = recipe.seed;
        roadlab::generateMetro(sc, mp);
    } else if (recipe.generator == "city") {
        roadlab::CityParams cp;
        cp.seed = recipe.seed;
        roadlab::generateCity(sc, cp);
    } else if (!roadlab::buildDemo(recipe.generator, sc)) {
        error = "roadlab: unknown generator '" + recipe.generator + "'";
        return false;
    }

    // AFTER the builders, which each set their own terrain amplitude — an
    // override applied first is silently discarded. (Learned the hard way in the
    // prototype's own CLI.)
    if (recipe.terrainAmp >= 0) sc.terrain.amplitude = recipe.terrainAmp;
    if (recipe.terrainWave > 0) sc.terrain.frequency = 1.0 / recipe.terrainWave;

    roadlab::finalizeScene(sc, recipe.terrain, recipe.props);
    split(sc.mesh, out, connectorRoads(sc.net));
    out.roads = sc.net.roadCount();
    out.junctions = sc.net.junctionCount();
    out.buildMs = std::chrono::duration<double, std::milli>(
                      std::chrono::steady_clock::now() - t0)
                      .count();
    return true;
}

}  // namespace roadlab_bridge
