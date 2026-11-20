// A stamped model, described (#191). See model_inspect.h.
#include "model_inspect.h"

#include <nlohmann/json.hpp>

#include <cmath>
#include <cstdio>
#include <functional>

namespace engine {

namespace {

std::string num(Real v) {
    char b[32];
    std::snprintf(b, sizeof b, "%.4g", static_cast<double>(v));
    return b;
}
std::string xyz(const Vec3& v) { return num(v.x) + ", " + num(v.y) + ", " + num(v.z); }
std::string thousands(std::size_t n) {
    std::string s = std::to_string(n);
    for (int i = static_cast<int>(s.size()) - 3; i > 0; i -= 3) s.insert(static_cast<std::size_t>(i), ",");
    return s;
}
std::string placed(const ProcFrame& f) {
    std::string text = "at " + xyz(f.position);
    Vec3 axis;
    Real angle = 0;
    f.rotation.toAxisAngle(axis, angle);
    if (std::abs(angle) > 1e-6) {
        char b[48];
        std::snprintf(b, sizeof b, ", turned %.1f deg about ", static_cast<double>(angle * 180.0 / 3.14159265358979323846));
        text += b + xyz(axis);
    }
    return text;
}

}  // namespace

std::vector<Entity> modelInstances(World& world) {
    std::vector<Entity> out;
    world.each<ModelInstance>([&](Entity e, ModelInstance&) { out.push_back(e); });
    return out;
}

std::string modelsListing(World& world) {
    std::string out;
    int n = 0;
    for (Entity e : modelInstances(world)) {
        const ModelInstance* mi = world.get<ModelInstance>(e);
        const Transform* t = world.get<Transform>(e);
        if (mi == nullptr) continue;
        out += (out.empty() ? "" : "; ") + std::to_string(n++) + " " + mi->prefab;
        if (t != nullptr) out += " at " + num(t->position.x) + " " + num(t->position.y) + " " + num(t->position.z);
        out += ", " + std::to_string(mi->pieces.size()) + " pieces, " + std::to_string(mi->bodies.size()) + " bodies";
    }
    return out.empty() ? "none" : out;
}

std::string modelTreeJson(World& world, Entity instance) {
    nlohmann::json rows = nlohmann::json::array();
    const ModelInstance* mi = world.get<ModelInstance>(instance);
    if (mi == nullptr || mi->model == nullptr) return rows.dump();
    const ProcModel& model = *mi->model;
    const std::vector<ProcModelDraw> draws = modelDraws(model);
    const double deg = 180.0 / 3.14159265358979323846;
    auto row = [](int depth, const char* kind, const std::string& name, const std::string& detail, int line) {
        nlohmann::json r;
        r["depth"] = depth;
        r["kind"] = kind;
        r["name"] = name;
        r["detail"] = detail;
        r["line"] = line;
        r["piece"] = -1;
        return r;
    };
    std::function<void(int, int)> walk = [&](int node, int depth) {
        const ProcNode& n = model.nodes[static_cast<std::size_t>(node)];
        std::string detail = node == 0 ? std::string("the model's own space") : placed(n.frame) + " under " + model.nodes[static_cast<std::size_t>(n.parent)].name;
        nlohmann::json r = row(depth, "node", n.name, std::string(), n.line);
        if (n.motion.kind != ProcMotion::Kind::None) {
            const bool turn = n.motion.kind == ProcMotion::Kind::Turn;
            const double unit = turn ? deg : 1.0;
            const double lo = n.motion.limited ? n.motion.min * unit : 0.0, hi = n.motion.limited ? n.motion.max * unit : 0.0;
            r["joint"] = n.name;
            r["turn"] = turn;
            r["lo"] = lo;
            r["hi"] = hi;
            r["value"] = mi->pose.of(n.name) * unit;
            detail += std::string(turn ? "; turns about " : "; slides along ") + xyz(n.motion.axis);
            detail += n.motion.limited ? ", " + num(lo) + " to " + num(hi) + (turn ? " deg" : " m") : std::string(", without end");
        }
        r["detail"] = detail;
        rows.push_back(r);
        for (const ProcPart& part : model.parts) {
            if (part.node != node || part.mesh.indices.empty()) continue;
            const std::string& material = model.materials[static_cast<std::size_t>(part.material)].name;
            std::string what = thousands(part.mesh.indices.size() / 3) + " triangles";
            if (part.level > 0) what += ", the level from " + num(model.levelFrom[static_cast<std::size_t>(part.level)]) + " m";
            nlohmann::json p = row(depth + 1, "part", material.empty() ? "plain" : material, what, part.line);
            for (std::size_t d = 0; d < draws.size(); ++d)
                if (draws[d].part == &part) { p["piece"] = static_cast<int>(d); break; }
            rows.push_back(p);
        }
        for (const ProcUse& use : model.uses) {
            if (use.node != node || use.piece == nullptr) continue;
            std::string what = std::to_string(use.placements.size()) + (use.placements.size() == 1 ? " copy, " : " copies, ") +
                               thousands(flattenModel(*use.piece).indices.size() / 3) + " triangles each";
            if (use.placements.size() == 1) what += ", " + placed(ProcFrame{use.placements[0].position, use.placements[0].rotation});
            if (use.level > 0) what += ", the level from " + num(model.levelFrom[static_cast<std::size_t>(use.level)]) + " m";
            rows.push_back(row(depth + 1, "use", use.piece->name.empty() ? "a piece" : use.piece->name, what, use.line));
        }
        for (const ProcMarker& mk : n.markers) rows.push_back(row(depth + 1, "marker", mk.name, mk.role + ", " + placed(mk.frame), mk.line));
        for (const ProcCollider& c : model.colliders) {
            if (c.node != node) continue;
            std::string what;
            switch (c.kind) {
                case ProcCollider::Kind::Box: what = "box " + xyz(c.halfExtent * 2.0) + " m"; break;
                case ProcCollider::Kind::Sphere: what = "sphere, radius " + num(c.radius) + " m"; break;
                case ProcCollider::Kind::Capsule: what = "capsule, radius " + num(c.radius) + " m, " + num(c.halfHeight * 2.0) + " m between its ends"; break;
                case ProcCollider::Kind::Hull: what = "hull"; break;
                case ProcCollider::Kind::Mesh: what = "mesh, " + thousands(c.mesh.indices.size() / 3) + " triangles"; break;
            }
            rows.push_back(row(depth + 1, "shape", c.name.empty() ? "(unnamed)" : c.name, what + ", " + placed(c.frame()), c.line));
        }
        for (std::size_t child = 1; child < model.nodes.size(); ++child)
            if (model.nodes[child].parent == node) walk(static_cast<int>(child), depth + 1);
    };
    walk(0, 0);
    return rows.dump();
}

}  // namespace engine
