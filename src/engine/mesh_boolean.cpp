// MeshBuilder::boolean -- join, cut and overlap of closed solids (#191 step 3), over the Manifold library
// (third_party/manifold, Apache-2.0). This is the ONLY file that includes Manifold: everything else, C++ or Lua, says
// MeshBuilder::boolean / mesh.union / mesh.subtract / mesh.intersect, so the library could be swapped behind them.
//
// What crosses the border, each way:
//   - A RenderMesh's vertex becomes fourteen numbers: position, normal, uv, colour, tangent. Manifold blends them
//     along every edge it cuts. It does not turn the normals of a subtracted solid's faces (asked to, with the
//     normals' slot named, v3.5.5 left them as they were: the mesh audit counted every cavity face as facing
//     inward), so that is done here.
//   - A RenderMesh draws a hard edge as two vertices at one place. Manifold needs to know they are one point of the
//     solid; its own `Merge` finds them.
//   - The engine winds a front face clockwise seen from outside, Manifold counter-clockwise. Going in, the last two
//     corners swap; coming out, every face goes through triFacing with the outward direction Manifold's order gives.
//   - Each input carries an id; every triangle of the result says which input it came from.
#include "mesh_builder.h"

#include "mesh_audit.h"

#include <manifold/manifold.h>

#include <cmath>
#include <unordered_map>

namespace engine {

namespace {

constexpr int kProps = 14;       // position 3, normal 3, uv 2, colour 3, tangent 3
constexpr int kNormalAt = 3;

// Two vertices nearer than this are one point of the solid. The engine's round primitives close their seam with a
// second vertex computed in single precision (a cylinder's last column is its first, turned through a float's 2 pi),
// which lands about a twentieth of a micron from the first: far outside the library's own "same point", far inside
// this. The same distance the weld calls one place.
constexpr double kSamePlace = 1e-5;   // metres

// Tell the library which vertices are one point: every vertex is paired with the first one found at its place.
// A grid of kSamePlace cells, each vertex looked for in its own cell and the 26 round it.
void samePlaces(const RenderMesh& mesh, manifold::MeshGL64& gl) {
    struct Cell {
        long long x, y, z;
        bool operator==(const Cell& o) const { return x == o.x && y == o.y && z == o.z; }
    };
    struct CellHash {
        std::size_t operator()(const Cell& c) const {
            return static_cast<std::size_t>(c.x * 73856093LL ^ c.y * 19349663LL ^ c.z * 83492791LL);
        }
    };
    std::unordered_map<Cell, std::vector<uint32_t>, CellHash> grid;
    for (uint32_t i = 0; i < mesh.vertices.size(); ++i) {
        const Vec3& p = mesh.vertices[i].position;
        const Cell home{static_cast<long long>(std::floor(p.x / kSamePlace)), static_cast<long long>(std::floor(p.y / kSamePlace)),
                        static_cast<long long>(std::floor(p.z / kSamePlace))};
        bool found = false;
        for (long long dx = -1; dx <= 1 && !found; ++dx)
            for (long long dy = -1; dy <= 1 && !found; ++dy)
                for (long long dz = -1; dz <= 1 && !found; ++dz) {
                    const auto cell = grid.find(Cell{home.x + dx, home.y + dy, home.z + dz});
                    if (cell == grid.end()) continue;
                    for (uint32_t other : cell->second) {
                        if ((mesh.vertices[other].position - p).length() > kSamePlace) continue;
                        gl.mergeFromVert.push_back(i);
                        gl.mergeToVert.push_back(other);
                        found = true;
                        break;
                    }
                }
        if (!found) grid[home].push_back(i);
    }
}

manifold::MeshGL64 toManifold(const RenderMesh& mesh, uint32_t id) {
    manifold::MeshGL64 gl;
    gl.numProp = kProps;
    gl.vertProperties.reserve(mesh.vertices.size() * kProps);
    for (const Vertex& v : mesh.vertices) {
        for (double x : {v.position.x, v.position.y, v.position.z, v.normal.x, v.normal.y, v.normal.z,
                         static_cast<double>(v.u), static_cast<double>(v.v), v.color.x, v.color.y, v.color.z,
                         v.tangent.x, v.tangent.y, v.tangent.z})
            gl.vertProperties.push_back(x);
    }
    gl.triVerts.reserve(mesh.indices.size());
    for (std::size_t t = 0; t + 2 < mesh.indices.size(); t += 3) {
        gl.triVerts.push_back(mesh.indices[t]);
        gl.triVerts.push_back(mesh.indices[t + 2]);   // clockwise to counter-clockwise
        gl.triVerts.push_back(mesh.indices[t + 1]);
    }
    gl.runIndex = {0, static_cast<uint64_t>(gl.triVerts.size())};
    gl.runOriginalID = {id};
    samePlaces(mesh, gl);
    return gl;
}

const char* statusText(manifold::Manifold::Error e) {
    using E = manifold::Manifold::Error;
    switch (e) {
        case E::NoError: return "";
        case E::NonFiniteVertex: return "has a vertex that is not a finite number";
        case E::NotManifold: return "is not a closed solid";
        case E::VertexOutOfBounds: return "has a triangle naming a vertex it does not have";
        case E::ResultTooLarge: return "gives a result too large to hold";
        default: return "was refused by the boolean library";
    }
}

// "the first mesh is not a closed solid (12 open edges)" -- or "" when the solid was taken.
std::string refusal(const char* which, const RenderMesh& mesh, const manifold::Manifold& solid) {
    if (mesh.indices.empty()) return std::string("the ") + which + " mesh is empty";
    const manifold::Manifold::Error e = solid.Status();
    if (e == manifold::Manifold::Error::NoError) return "";
    std::string text = std::string("the ") + which + " mesh " + statusText(e);
    if (e == manifold::Manifold::Error::NotManifold) text += " (" + std::to_string(openEdges(mesh)) + " open edges)";
    return text;
}

Vec3 at(const manifold::MeshGL64& gl, uint64_t vert, int prop) {
    const std::size_t i = static_cast<std::size_t>(vert) * gl.numProp + static_cast<std::size_t>(prop);
    return Vec3(gl.vertProperties[i], gl.vertProperties[i + 1], gl.vertProperties[i + 2]);
}

Vec3 unitOr(const Vec3& v, const Vec3& fallback) {
    const double length = v.length();
    return length > 0.0 ? v * (1.0 / length) : fallback;
}

}  // namespace

MeshBuilder::BooleanResult MeshBuilder::boolean(const RenderMesh& a, const RenderMesh& b, BooleanOp op) {
    BooleanResult result;
    result.fromA.materialIndex = a.materialIndex;
    result.fromB.materialIndex = b.materialIndex;

    const uint32_t firstId = manifold::Manifold::ReserveIDs(2);
    const manifold::Manifold solidA(toManifold(a, firstId));
    const manifold::Manifold solidB(toManifold(b, firstId + 1));
    result.error = refusal("first", a, solidA);
    if (result.error.empty()) result.error = refusal("second", b, solidB);
    if (!result.error.empty()) return result;

    const manifold::OpType type = op == BooleanOp::Union ? manifold::OpType::Add
                                : op == BooleanOp::Subtract ? manifold::OpType::Subtract
                                : manifold::OpType::Intersect;
    const manifold::Manifold cut = solidA.Boolean(solidB, type);
    if (cut.Status() != manifold::Manifold::Error::NoError) {
        result.error = std::string("the result ") + statusText(cut.Status());
        return result;
    }
    const manifold::MeshGL64 gl = cut.GetMeshGL64();   // the normals are turned below, by one rule of our own
    if (gl.numProp < static_cast<uint64_t>(kProps)) return result;   // nothing left: an empty result has no properties

    // For each vertex of each output mesh: the sum, over its faces, of how far its normal agrees with the face's facing.
    std::vector<double> agreeA, agreeB;
    for (std::size_t run = 0; run < gl.runOriginalID.size(); ++run) {
        const bool isA = gl.runOriginalID[run] == firstId;
        RenderMesh& out = isA ? result.fromA : result.fromB;
        std::vector<double>& agree = isA ? agreeA : agreeB;
        std::unordered_map<uint64_t, uint32_t> placed;   // Manifold's vertex -> this mesh's
        auto vertexOf = [&](uint64_t v) {
            const auto found = placed.find(v);
            if (found != placed.end()) return found->second;
            Vertex made;
            made.position = at(gl, v, 0);
            made.normal = unitOr(at(gl, v, kNormalAt), Vec3(0, 1, 0));
            const std::size_t i = static_cast<std::size_t>(v) * gl.numProp;
            made.u = static_cast<float>(gl.vertProperties[i + 6]);
            made.v = static_cast<float>(gl.vertProperties[i + 7]);
            made.color = at(gl, v, 8);
            made.tangent = unitOr(at(gl, v, 11), Vec3(0, 0, 0));
            const uint32_t index = static_cast<uint32_t>(out.vertices.size());
            out.vertices.push_back(made);
            placed.emplace(v, index);
            return index;
        };
        for (uint64_t k = gl.runIndex[run]; k + 2 < gl.runIndex[run + 1]; k += 3) {
            const uint64_t v0 = gl.triVerts[k], v1 = gl.triVerts[k + 1], v2 = gl.triVerts[k + 2];
            const Vec3 p0 = at(gl, v0, 0), p1 = at(gl, v1, 0), p2 = at(gl, v2, 0);
            const Vec3 outward = cross(p1 - p0, p2 - p0);   // counter-clockwise seen from outside
            const uint32_t i0 = vertexOf(v0), i1 = vertexOf(v1), i2 = vertexOf(v2);
            triFacing(out, i0, i1, i2, outward);
            const Vec3 facing = unitOr(outward, Vec3(0, 0, 0));
            agree.resize(out.vertices.size(), 0.0);
            for (uint32_t i : {i0, i1, i2}) agree[i] += dot(out.vertices[i].normal, facing);
        }
    }
    // A subtracted solid's faces are now the walls of a cavity and face into it, but each vertex still carries the
    // normal it had on the solid, pointing the other way. A vertex whose normal looks away from its own faces is
    // turned round.
    auto turnNormals = [](RenderMesh& mesh, const std::vector<double>& sums) {
        for (std::size_t i = 0; i < mesh.vertices.size() && i < sums.size(); ++i)
            if (sums[i] < 0.0) mesh.vertices[i].normal = mesh.vertices[i].normal * -1.0;
    };
    turnNormals(result.fromA, agreeA);
    turnNormals(result.fromB, agreeB);
    return result;
}

}  // namespace engine
