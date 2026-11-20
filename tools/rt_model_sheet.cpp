// rt_model_sheet -- THE MODEL SHEET (#191 step 1): one command that shows what a Lua recipe builds.
//
//   rt_model_sheet RECIPE.lua [--out sheet.png] [--args JSON] [--set NODE=VALUE]... [--size 512] [--spp 48]
//
// `--set` draws the model in a pose: a moving node's value, in degrees for a node that turns and metres for one
// that slides (`--set boom=40 --set door_fl/window=-0.2`). The text lists every moving node and its limits.
//
// Runs the recipe (a chunk that returns a Mesh or a `model`; a model's nodes are drawn where the tree puts them and
// listed in the text, with its named materials, its markers (also drawn) and the pieces it uses), and writes
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
#include "engine/mesh_builder.h"
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
    std::vector<std::pair<std::string, double>> set;   // --set name=value: the pose to draw
};

const char* kUsage =
    "usage: rt_model_sheet RECIPE.lua [--out sheet.png] [--args JSON] [--set NODE=VALUE]... [--size N] [--spp N]\n"
    "  runs a Lua recipe that returns a mesh or a model and writes four views of it in one PNG,\n"
    "  with its size, triangle counts and mesh audit beside it as a .txt\n"
    "  --set draws a pose: a moving node's value, degrees for a turn and metres for a slide (--set boom=40)\n";

bool parse(int argc, char** argv, Options& o) {
    for (int i = 1; i < argc; ++i) {
        const std::string a = argv[i];
        auto value = [&](std::string& into) { if (i + 1 >= argc) return false; into = argv[++i]; return true; };
        std::string v;
        if (a == "-h" || a == "--help") return false;
        else if (a == "--out") { if (!value(o.out)) return false; }
        else if (a == "--args") { if (!value(o.argsJson)) return false; }
        else if (a == "--set") {
            if (!value(v)) return false;
            const std::size_t eq = v.find('=');
            if (eq == std::string::npos || eq == 0) return false;
            o.set.emplace_back(v.substr(0, eq), std::atof(v.c_str() + eq + 1));
        }
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
    for (const ProcDrawnPiece& d : drawnPieces(m))
        for (const Mat4& t : d.transforms)
            for (const Vertex& v : d.part->mesh.vertices) b.add(t.transformPoint(v.position));
    return b;
}

// Every marker of the model with where it is in the model: its own, and those inside a piece used on a node, under
// the name they are reached by ("wheel_fl/valve").
struct PlacedMarker { std::string name; const ProcMarker* marker; std::string on; ProcFrame at; };
std::vector<PlacedMarker> markersOf(const ProcModel& m) {
    std::vector<PlacedMarker> out;
    for (std::size_t n = 0; n < m.nodes.size(); ++n) {
        const ProcFrame node = m.nodeFrame(static_cast<int>(n));
        for (const ProcMarker& mk : m.nodes[n].markers) out.push_back(PlacedMarker{mk.name, &mk, m.nodes[n].name, node * mk.frame});
    }
    for (const ProcUse& use : m.uses) {
        if (use.node == 0 || use.piece == nullptr) continue;   // a piece on the root has no node's name to be reached by
        const std::string& node = m.nodes[static_cast<std::size_t>(use.node)].name;
        for (const PlacedMarker& inner : markersOf(*use.piece)) {
            PlacedMarker pm{node + "/" + inner.name, inner.marker, node + ", in the piece used there", ProcFrame{}};
            if (m.markerFrame(pm.name, pm.at)) out.push_back(pm);
        }
    }
    return out;
}

// Draw each marker as three short bars from its point: red along its x, green along its y, and a longer blue one
// along its z, the way it faces. The bars are a small model of their own (a node at each marker, a bar on each
// node), baked like any other; markers are not part of the shape, so this is added after the fingerprint is taken.
// A marker with no size is drawn at a size that reads on this model.
void addMarkers(Scene& scene, const std::vector<PlacedMarker>& markers, const Bounds& b) {
    if (markers.empty()) return;
    const Real reach = std::max({b.size().x, b.size().y, b.size().z, Real(0.2)});
    const Vec3 colours[3] = {Vec3(0.9, 0.05, 0.05), Vec3(0.05, 0.7, 0.05), Vec3(0.05, 0.2, 0.95)};
    ProcModel bars;
    for (const PlacedMarker& pm : markers) {
        const int node = bars.addNode(pm.name, 0, pm.at);
        const Real len = pm.marker->size > 0 ? pm.marker->size : reach * 0.06;
        const Real thick = std::max(len * 0.08, reach * 0.004);
        for (int a = 0; a < 3; ++a) {
            const Real along = a == 2 ? len * 1.6 : len;
            Vec3 size(thick, thick, thick), centre(0, 0, 0);
            (a == 0 ? size.x : a == 1 ? size.y : size.z) = along;
            (a == 0 ? centre.x : a == 1 ? centre.y : centre.z) = along * 0.5;
            ProcPart bar;
            bar.mesh = MeshBuilder::box(size);
            MeshBuilder::transform(bar.mesh, Mat4::translate(centre.x, centre.y, centre.z));
            for (Vertex& v : bar.mesh.vertices) v.color = colours[a];
            bar.node = node;
            bars.parts.push_back(std::move(bar));
        }
    }
    bakeProcModel(bars, scene);
}

// One number for "the same triangles": every drawn triangle as it stands in the scene (corners, normals, colours,
// and what it is made of), then every placed copy. Two runs that print the same number built the same shape, so a change to the toolkit or to
// the model record can be checked against every recipe without looking at pictures. Each triangle is hashed alone and
// the hashes are added, so the ORDER of triangles and parts does not count: a recipe that walks a Lua table with
// `pairs` adds its parts in a different order every run, and it is still the same shape.
uint64_t fingerprint(const Scene& scene, const ProcModel& m) {
    uint64_t sum = 0;
    uint64_t h = 0;
    auto vec = [&](const Vec3& v) { const double d[3] = {v.x, v.y, v.z}; h = bundle::fnv1a(d, sizeof d, h); };
    // What each triangle is made of counts too: its material's numbers and the pixels of its maps, not which slot
    // of the scene the material happens to sit in.
    auto pixels = [&](int tex) -> uint64_t {
        if (tex < 0) return 0;
        const Texture& t = scene.textures[static_cast<std::size_t>(tex)];
        const int dims[3] = {t.width, t.height, t.channels};
        return bundle::fnv1a(t.pixels.data(), t.pixels.size(), bundle::fnv1a(dims, sizeof dims));
    };
    std::vector<uint64_t> made(scene.materials.size());
    for (std::size_t i = 0; i < scene.materials.size(); ++i) {
        const Material& mat = scene.materials[i];
        const double nums[4] = {mat.metallic, mat.roughness, mat.texScale, mat.meshUV ? 1.0 : 0.0};
        const uint64_t maps[3] = {pixels(mat.albedoTex), pixels(mat.normalTex), static_cast<uint64_t>(mat.surface)};
        made[i] = bundle::fnv1a(maps, sizeof maps, bundle::fnv1a(nums, sizeof nums));
    }
    for (const Triangle& t : scene.triangles) {
        h = bundle::fnv1aStr("triangle");
        vec(t.v0); vec(t.v1); vec(t.v2);
        vec(t.n0); vec(t.n1); vec(t.n2);
        vec(t.c0); vec(t.c1); vec(t.c2);
        h = bundle::fnv1a(&made[static_cast<std::size_t>(t.materialIndex)], sizeof(uint64_t), h);
        sum += h;
    }
    for (const ProcDrawnPiece& d : drawnPieces(m)) {
        const ProcMaterial& made = d.piece->materialOf(*d.part);
        const RenderMesh& proto = d.part->mesh;
        h = bundle::fnv1aStr("placed");
        const double look[3] = {made.metallic, made.roughness, made.alphaFoliage ? 1.0 : 0.0};
        h = bundle::fnv1a(look, sizeof look, h);
        for (const Vertex& v : proto.vertices) { vec(v.position); vec(v.normal); vec(v.color); }
        h = bundle::fnv1a(proto.indices.data(), proto.indices.size() * sizeof(proto.indices[0]), h);
        for (const Mat4& t : d.transforms)
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

// The model's moving nodes, through used pieces, each under the name a pose calls it by.
void listJoints(const ProcModel& m, const std::string& prefix, const Options& o, std::ostringstream& s) {
    char line[512];
    for (const ProcNode& n : m.nodes) {
        if (n.motion.kind == ProcMotion::Kind::None) continue;
        const bool turn = n.motion.kind == ProcMotion::Kind::Turn;
        const double unit = turn ? RAD_TO_DEG : 1.0;
        const std::string name = prefix + n.name;
        std::snprintf(line, sizeof line, "  %s: %s (%.2f, %.2f, %.2f)", name.c_str(), turn ? "turns about" : "slides along", n.motion.axis.x,
                      n.motion.axis.y, n.motion.axis.z);
        s << line;
        if (n.motion.limited) std::snprintf(line, sizeof line, ", %.3g to %.3g %s", n.motion.min * unit, n.motion.max * unit, turn ? "degrees" : "m");
        else std::snprintf(line, sizeof line, ", no limits");
        s << line;
        for (const std::pair<std::string, double>& set : o.set) {
            if (set.first != name) continue;
            const double held = n.motion.clamped(set.second / unit) * unit;
            std::snprintf(line, sizeof line, "; drawn at %.3g%s", held, std::abs(held - set.second) > 1e-9 ? " (stopped at its limit)" : "");
            s << line;
        }
        s << "\n";
    }
    for (const ProcUse& use : m.uses)
        if (use.node != 0 && use.piece != nullptr) listJoints(*use.piece, prefix + m.nodes[static_cast<std::size_t>(use.node)].name + "/", o, s);
}

// `m` is the model as drawn (held in the pose asked for); `built` is the recipe's own, which still says what moves.
std::string report(const Options& o, const ProcModel& m, const ProcModel& built, const Bounds& b, uint64_t print) {
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
        // In the tree's own order (a node, then everything under it), not the order the recipe made them in.
        std::vector<std::size_t> order;
        std::vector<std::size_t> open = {0};
        while (!open.empty()) {
            const std::size_t at = open.back();
            open.pop_back();
            order.push_back(at);
            for (std::size_t c = m.nodes.size(); c-- > 1;)
                if (m.nodes[c].parent == static_cast<int>(at)) open.push_back(c);
        }
        for (const std::size_t i : order) {
            const ProcNode& n = m.nodes[i];
            int depth = 0;
            for (int p = n.parent; p >= 0; p = m.nodes[static_cast<std::size_t>(p)].parent) ++depth;
            const Vec3 at = m.nodeFrame(static_cast<int>(i)).position;
            long parts = 0;
            for (const ProcPart& part : m.parts) parts += part.node == static_cast<int>(i) ? 1 : 0;
            long copies = 0;
            for (const ProcUse& use : m.uses) copies += use.node == static_cast<int>(i) ? static_cast<long>(use.placements.size()) : 0;
            std::snprintf(line, sizeof line, "  %*s%s: at (%.3f, %.3f, %.3f) in the model, %ld part%s", depth * 2, "", n.name.c_str(),
                          at.x, at.y, at.z, parts, parts == 1 ? "" : "s");
            s << line;
            if (copies > 0) { std::snprintf(line, sizeof line, ", %ld cop%s of a used piece", copies, copies == 1 ? "y" : "ies"); s << line; }
            s << "\n";
        }
    }
    {
        std::ostringstream joints;
        listJoints(built, std::string(), o, joints);
        if (!joints.str().empty()) s << "moving nodes (a pose names them: --set NAME=VALUE):\n" << joints.str();
    }
    // The materials, when the recipe named any, and the markers.
    bool named = false;
    for (const ProcModelMaterial& mm : m.materials) named = named || !mm.name.empty();
    if (named) {
        for (std::size_t i = 0; i < m.materials.size(); ++i) {
            const ProcModelMaterial& mm = m.materials[i];
            long parts = 0;
            for (const ProcPart& part : m.parts) parts += part.material == static_cast<int>(i) ? 1 : 0;
            if (parts == 0 && mm.name.empty()) continue;
            std::snprintf(line, sizeof line, "material %s: roughness %.2f, metallic %.2f%s%s, %ld part%s\n",
                          mm.name.empty() ? (i == 0 ? "(plain)" : "(no name)") : mm.name.c_str(), mm.material->roughness,
                          mm.material->metallic, mm.material->textured ? ", with maps" : "",
                          mm.material->opacity < 1.0f ? ", see-through" : "", parts, parts == 1 ? "" : "s");
            s << line;
        }
    }
    const std::vector<PlacedMarker> markers = markersOf(m);
    if (!markers.empty()) {
        std::snprintf(line, sizeof line, "markers: %zu (drawn as red x, green y, blue z; blue is the way it faces)\n", markers.size());
        s << line;
        for (const PlacedMarker& pm : markers) {
            const Vec3 face = pm.at.rotation.rotate(Vec3(0, 0, 1));
            std::snprintf(line, sizeof line, "  %s (%s) on %s: at (%.3f, %.3f, %.3f) in the model, facing (%.2f, %.2f, %.2f)", pm.name.c_str(),
                          pm.marker->role.c_str(), pm.on.c_str(), pm.at.position.x, pm.at.position.y, pm.at.position.z, face.x, face.y, face.z);
            s << line;
            if (pm.marker->size > 0) { std::snprintf(line, sizeof line, ", size %.3f m", pm.marker->size); s << line; }
            s << "\n";
        }
    }
    MeshAudit all;
    long vertices = 0;
    for (std::size_t i = 0; i < m.parts.size(); ++i) {
        const RenderMesh& mesh = m.parts[i].mesh;
        const MeshAudit a = auditMesh(mesh);
        all.add(a);
        vertices += static_cast<long>(mesh.vertices.size());
        std::string on = tree ? " on " + m.nodes[static_cast<std::size_t>(m.parts[i].node)].name : std::string();
        const std::string& made = m.materials[static_cast<std::size_t>(m.parts[i].material)].name;
        if (!made.empty()) on += " in " + made;
        std::snprintf(line, sizeof line, "part %zu%s: %ld triangles, %zu vertices%s\n", i + 1, on.c_str(), a.tris,
                      mesh.vertices.size(), a.clean() ? "" : "  <- audit findings");
        s << line;
    }
    const std::vector<ProcDrawnPiece> drawn = drawnPieces(m);
    for (std::size_t i = 0; i < drawn.size(); ++i) {
        const MeshAudit a = auditMesh(drawn[i].part->mesh);
        all.add(a);
        const std::string& made = drawn[i].piece->materials[static_cast<std::size_t>(drawn[i].part->material)].name;
        std::snprintf(line, sizeof line, "instanced piece %zu%s%s: %ld triangles, placed %zu times%s\n", i + 1, made.empty() ? "" : " in ",
                      made.c_str(), a.tris, drawn[i].transforms.size(), a.clean() ? "" : "  <- audit findings");
        s << line;
    }
    // (Triangles are counted as stored: a used piece once, however many copies of it are drawn.)
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
    // The recipe's model as built is what the text lists (its moving nodes and their limits); what is measured and
    // drawn is that model held in the pose asked for.
    const ProcModel built = model;
    if (!o.set.empty()) {
        ProcPose pose;
        for (const std::pair<std::string, double>& set : o.set) {
            const ProcNode* joint = built.jointAt(set.first);
            if (joint == nullptr) {
                std::fprintf(stderr, "rt_model_sheet: --set %s: the model has no moving node of that name\n", set.first.c_str());
                return 1;
            }
            pose.values[set.first] = set.second * (joint->motion.kind == ProcMotion::Kind::Turn ? DEG_TO_RAD : 1.0);
        }
        model = *posedModel(built, pose);
    }
    const Bounds bounds = boundsOf(model);
    if (!bounds.valid()) { std::fprintf(stderr, "rt_model_sheet: %s built no geometry\n", o.recipe.c_str()); return 1; }

    Scene scene;
    bakeProcModel(model, scene);
    const uint64_t print = fingerprint(scene, model);
    addMarkers(scene, markersOf(model), bounds);
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

    const std::string text = report(o, model, built, bounds, print) + "pictures: front | side  over  top | three-quarter; floor squares are 1 m\n";
    std::string textPath = o.out;
    const std::size_t dot = textPath.rfind('.');
    textPath = (dot == std::string::npos ? textPath : textPath.substr(0, dot)) + ".txt";
    std::ofstream(textPath) << text;
    std::fputs(text.c_str(), stdout);
    std::printf("wrote %s and %s\n", o.out.c_str(), textPath.c_str());
    return 0;
}
