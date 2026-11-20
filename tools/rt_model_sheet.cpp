// rt_model_sheet -- THE MODEL SHEET (#191 step 1): one command that shows what a Lua recipe builds.
//
//   rt_model_sheet RECIPE.lua [--out sheet.png] [--args JSON] [--size 512] [--spp 48]
//
// Runs the recipe (a chunk that returns a Mesh or a `model`; a model's nodes are drawn where the tree puts them and
// listed in the text), and writes
//   sheet.png   four pictures in one: front and side on the top row, top and three-quarter on the bottom row,
//               the model standing on a floor of 1 m squares so its size reads in every picture;
//   sheet.txt   the numbers: size in metres, triangles and vertices per part, and the mesh audit's findings.
//
// Nothing here is new machinery. The recipe runs through the same `runProcgenModelValue` the level loader uses, the
// model goes into the offline path tracer through the same `bakeProcModel`, and the checks are `auditMesh`. No GPU
// and no viewer, so it runs beside a playing game.
//
// Known limits, on purpose for the first cut: the tracer draws both sides of a face (a wrongly wound face looks
// right here; the audit's numbers are what catch it), and `bakeProcModel` carries no opacity or emission, so glass
// and lamps draw as plain surfaces.
#include "camera.h"
#include "engine/bundle/bundle.h"   // fnv1a
#include "engine/mesh_audit.h"
#include "engine/procgen/proc_model.h"
#include "engine/script_assets.h"
#include "engine/scripting/procgen_bindings.h"
#include "engine/scripting/script_modules.h"
#include "engine/scripting/script_vm.h"
#include "image.h"
#include "job_system.h"
#include "level_scene.h"
#include "material.h"
#include "path_tracer.h"
#include "scene.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

using namespace engine;

namespace {

struct Options {
    std::string recipe;
    std::string out = "model_sheet.png";
    std::string argsJson;
    int size = 512;
    int samples = 48;
};

const char* kUsage =
    "usage: rt_model_sheet RECIPE.lua [--out sheet.png] [--args JSON] [--size N] [--spp N]\n"
    "  runs a Lua recipe that returns a mesh or a model and writes four views of it in one PNG,\n"
    "  with its size, triangle counts and mesh audit beside it as a .txt\n";

bool parse(int argc, char** argv, Options& o) {
    for (int i = 1; i < argc; ++i) {
        const std::string a = argv[i];
        auto value = [&](std::string& into) { if (i + 1 >= argc) return false; into = argv[++i]; return true; };
        std::string v;
        if (a == "-h" || a == "--help") return false;
        else if (a == "--out") { if (!value(o.out)) return false; }
        else if (a == "--args") { if (!value(o.argsJson)) return false; }
        else if (a == "--size") { if (!value(v)) return false; o.size = std::atoi(v.c_str()); }
        else if (a == "--spp") { if (!value(v)) return false; o.samples = std::atoi(v.c_str()); }
        else if (!a.empty() && a[0] == '-') return false;
        else if (o.recipe.empty()) o.recipe = a;
        else return false;
    }
    return !o.recipe.empty() && o.size >= 64 && o.samples >= 1;
}

struct Bounds {
    Vec3 lo{1e30, 1e30, 1e30}, hi{-1e30, -1e30, -1e30};
    void add(const Vec3& p) {
        lo = Vec3(std::min(lo.x, p.x), std::min(lo.y, p.y), std::min(lo.z, p.z));
        hi = Vec3(std::max(hi.x, p.x), std::max(hi.y, p.y), std::max(hi.z, p.z));
    }
    bool valid() const { return lo.x <= hi.x; }
    Vec3 centre() const { return (lo + hi) * 0.5; }
    Vec3 size() const { return hi - lo; }
};

Bounds boundsOf(const ProcModel& m) {
    Bounds b;
    for (const ProcPart& part : m.parts) {
        const ProcFrame at = m.nodeFrame(part.node);   // a part stands at the node it hangs on
        for (const Vertex& v : part.mesh.vertices) b.add(at.point(v.position));
    }
    for (const ProcInstanceGroup& g : m.instances)
        for (const Mat4& t : g.transforms)
            for (const Vertex& v : g.proto.vertices) b.add(t.transformPoint(v.position));
    return b;
}

// One number for "the same triangles": every drawn triangle as it stands in the scene (corners, normals, colours),
// then every placed copy. Two runs that print the same number built the same shape, so a change to the toolkit or to
// the model record can be checked against every recipe without looking at pictures. Each triangle is hashed alone and
// the hashes are added, so the ORDER of triangles and parts does not count: a recipe that walks a Lua table with
// `pairs` adds its parts in a different order every run, and it is still the same shape.
uint64_t fingerprint(const Scene& scene, const ProcModel& m) {
    uint64_t sum = 0;
    uint64_t h = 0;
    auto vec = [&](const Vec3& v) { const double d[3] = {v.x, v.y, v.z}; h = bundle::fnv1a(d, sizeof d, h); };
    for (const Triangle& t : scene.triangles) {
        h = bundle::fnv1aStr("triangle");
        vec(t.v0); vec(t.v1); vec(t.v2);
        vec(t.n0); vec(t.n1); vec(t.n2);
        vec(t.c0); vec(t.c1); vec(t.c2);
        sum += h;
    }
    for (const ProcInstanceGroup& g : m.instances) {
        h = bundle::fnv1aStr("placed");
        for (const Vertex& v : g.proto.vertices) { vec(v.position); vec(v.normal); vec(v.color); }
        h = bundle::fnv1a(g.proto.indices.data(), g.proto.indices.size() * sizeof(g.proto.indices[0]), h);
        for (const Mat4& t : g.transforms)
            for (int r = 0; r < 4; ++r) { const double row[4] = {t.m[r][0], t.m[r][1], t.m[r][2], t.m[r][3]}; h = bundle::fnv1a(row, sizeof row, h); }
        sum += h;
    }
    return sum;
}

// The floor and the light every sheet is taken under: a floor of 1 m squares at the model's foot, a pale sky and
// one sun over the viewer's left shoulder.
void addStage(Scene& scene, const Bounds& b) {
    Material floor = Material::pbr(Vec3(0.62, 0.62, 0.60), 0.0, 0.9);
    floor.checkerboard = true;
    const int fm = scene.addMaterial(floor);
    const Real reach = std::max({b.size().x, b.size().y, b.size().z, Real(1)}) * 6;
    const Vec3 c = b.centre();
    scene.addQuad(Vec3(c.x - reach, b.lo.y, c.z - reach), Vec3(2 * reach, 0, 0), Vec3(0, 0, 2 * reach), fm);
    scene.environment.enabled = true;
    scene.environment.skyHorizon = Vec3(0.80, 0.82, 0.85);
    scene.environment.skyZenith = Vec3(0.55, 0.62, 0.72);
    SceneLight sun;
    sun.type = SceneLight::Type::Directional;
    sun.direction = normalize(Vec3(-0.45, 0.80, 0.40));
    sun.intensity = 2.2;
    sun.angularRadius = 0.05;
    scene.lights.push_back(sun);
}

struct View { const char* name; Vec3 from; Vec3 up; };

Image renderView(const Scene& scene, const Bounds& b, const View& v, const Options& o, JobSystem& jobs) {
    // A long lens from far off, so the picture is close to a drawing's flat projection.
    const double fovDeg = 16.0;
    const Vec3 half = b.size() * 0.5;
    const double radius = std::max(half.length(), 0.25);
    const double distance = radius / std::sin(fovDeg * 0.5 * M_PI / 180.0) * 1.12;
    const Vec3 target = b.centre();
    Camera camera(target + normalize(v.from) * distance, target, v.up, fovDeg, 1.0);
    RenderConfig cfg;
    cfg.imageSize = o.size;
    cfg.samplesPerPixel = o.samples;
    cfg.maxBounces = 4;
    return renderImage(scene, camera, cfg, jobs);
}

std::string report(const Options& o, const ProcModel& m, const Bounds& b, uint64_t print) {
    std::ostringstream s;
    char line[512];
    s << "model sheet: " << o.recipe << "\n";
    if (!o.argsJson.empty()) s << "args: " << o.argsJson << "\n";
    const Vec3 size = b.size();
    std::snprintf(line, sizeof line, "size (m): %.3f wide (x)  %.3f tall (y)  %.3f deep (z)\n", size.x, size.y, size.z);
    s << line;
    std::snprintf(line, sizeof line, "lowest point y = %.3f, centre = (%.3f, %.3f, %.3f)\n", b.lo.y, b.centre().x, b.centre().y, b.centre().z);
    s << line;
    // The tree of nodes, when the model has more than its root: each node, what it hangs under, and where it ends
    // up in the model's space. A flat model prints nothing here.
    const bool tree = m.nodes.size() > 1;
    if (tree) {
        std::snprintf(line, sizeof line, "nodes: %zu\n", m.nodes.size());
        s << line;
        for (std::size_t i = 0; i < m.nodes.size(); ++i) {
            const ProcNode& n = m.nodes[i];
            int depth = 0;
            for (int p = n.parent; p >= 0; p = m.nodes[static_cast<std::size_t>(p)].parent) ++depth;
            const Vec3 at = m.nodeFrame(static_cast<int>(i)).position;
            long parts = 0;
            for (const ProcPart& part : m.parts) parts += part.node == static_cast<int>(i) ? 1 : 0;
            std::snprintf(line, sizeof line, "  %*s%s: at (%.3f, %.3f, %.3f) in the model, %ld part%s\n", depth * 2, "", n.name.c_str(),
                          at.x, at.y, at.z, parts, parts == 1 ? "" : "s");
            s << line;
        }
    }
    MeshAudit all;
    long vertices = 0;
    for (std::size_t i = 0; i < m.parts.size(); ++i) {
        const RenderMesh& mesh = m.parts[i].mesh;
        const MeshAudit a = auditMesh(mesh);
        all.add(a);
        vertices += static_cast<long>(mesh.vertices.size());
        const std::string on = tree ? " on " + m.nodes[static_cast<std::size_t>(m.parts[i].node)].name : std::string();
        std::snprintf(line, sizeof line, "part %zu%s: %ld triangles, %zu vertices%s\n", i + 1, on.c_str(), a.tris,
                      mesh.vertices.size(), a.clean() ? "" : "  <- audit findings");
        s << line;
    }
    for (std::size_t i = 0; i < m.instances.size(); ++i) {
        const ProcInstanceGroup& g = m.instances[i];
        const MeshAudit a = auditMesh(g.proto);
        all.add(a);
        std::snprintf(line, sizeof line, "instanced piece %zu: %ld triangles, placed %zu times%s\n", i + 1, a.tris,
                      g.transforms.size(), a.clean() ? "" : "  <- audit findings");
        s << line;
    }
    std::snprintf(line, sizeof line, "total: %ld triangles, %ld vertices in parts, %d placed copies, %d colliders\n", all.tris,
                  vertices, m.instanceCount(), m.colliderCount());
    s << line;
    std::snprintf(line, sizeof line, "fingerprint: %016llx (the same number means the same triangles)\n",
                  static_cast<unsigned long long>(print));
    s << line;
    if (all.clean()) {
        s << "mesh audit: clean\n";
    } else {
        std::snprintf(line, sizeof line,
                      "mesh audit: %ld facing inward (%ld of them fully reversed, %ld thin slivers), %ld with no area, "
                      "%ld not finite, %ld bad normals\n",
                      all.inward, all.flipped, all.inwardSliver, all.degenerate, all.nonFinite, all.badNormals);
        s << line;
        if (all.inward > 0) {
            std::snprintf(line, sizeof line, "  first inward face at (%.3f, %.3f, %.3f), area %.5f m2\n", all.where.x, all.where.y,
                          all.where.z, all.whereArea);
            s << line;
        }
        if (all.degenerate > 0) {
            std::snprintf(line, sizeof line, "  first face with no area at (%.3f, %.3f, %.3f)\n", all.degenWhere.x, all.degenWhere.y,
                          all.degenWhere.z);
            s << line;
        }
    }
    return s.str();
}

}  // namespace

int main(int argc, char** argv) {
    Options o;
    if (!parse(argc, argv, o)) { std::fputs(kUsage, stderr); return 2; }

    std::ifstream in(o.recipe);
    if (!in) { std::fprintf(stderr, "rt_model_sheet: cannot read %s\n", o.recipe.c_str()); return 1; }
    std::stringstream code;
    code << in.rdbuf();

    ScriptVM vm;
    openProcgenLibrary(vm);
    openModuleLoader(vm, makeModuleSource(""));
    if (!o.argsJson.empty()) setRecipeArgs(vm, o.argsJson);

    ProcModel model;
    std::string error;
    if (!runProcgenModelValue(vm, code.str(), model, &error)) {
        std::fprintf(stderr, "rt_model_sheet: %s: %s\n", o.recipe.c_str(), error.c_str());
        return 1;
    }
    const Bounds bounds = boundsOf(model);
    if (!bounds.valid()) { std::fprintf(stderr, "rt_model_sheet: %s built no geometry\n", o.recipe.c_str()); return 1; }

    Scene scene;
    bakeProcModel(model, scene);
    const uint64_t print = fingerprint(scene, model);
    addStage(scene, bounds);
    scene.buildAccelerator();

    // +z is the front of a piece (the furniture's convention: z runs out from the wall it backs onto).
    const View views[4] = {
        {"front", Vec3(0, 0.12, 1), Vec3(0, 1, 0)},
        {"side", Vec3(1, 0.12, 0), Vec3(0, 1, 0)},
        {"top", Vec3(0, 1, 0.001), Vec3(0, 0, -1)},
        {"three-quarter", Vec3(0.75, 0.55, 1), Vec3(0, 1, 0)},
    };
    JobSystem jobs;
    Image sheet(o.size * 2, o.size * 2);
    for (int i = 0; i < 4; ++i) {
        const Image tile = renderView(scene, bounds, views[i], o, jobs);
        const int ox = (i % 2) * o.size, oy = (i / 2) * o.size;
        for (int y = 0; y < o.size; ++y)
            for (int x = 0; x < o.size; ++x) sheet.setPixel(ox + x, oy + y, tile.getPixel(x, y));
    }
    // A thin dark cross between the four pictures.
    for (int k = 0; k < o.size * 2; ++k) {
        sheet.setPixel(o.size, k, Vec3(0.05, 0.05, 0.05));
        sheet.setPixel(k, o.size, Vec3(0.05, 0.05, 0.05));
    }
    sheet.writePng(o.out);

    const std::string text = report(o, model, bounds, print) + "pictures: front | side  over  top | three-quarter; floor squares are 1 m\n";
    std::string textPath = o.out;
    const std::size_t dot = textPath.rfind('.');
    textPath = (dot == std::string::npos ? textPath : textPath.substr(0, dot)) + ".txt";
    std::ofstream(textPath) << text;
    std::fputs(text.c_str(), stdout);
    std::printf("wrote %s and %s\n", o.out.c_str(), textPath.c_str());
    return 0;
}
