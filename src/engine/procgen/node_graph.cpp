#include "node_graph.h"
#include "rock.h"

#include <nlohmann/json.hpp>
#include <algorithm>
#include <functional>
#include <optional>

using json = nlohmann::json;

namespace engine {

bool valueIsType(const GraphValue& v, GraphType t) {
    switch (t) {
        case GraphType::Scalar: return std::holds_alternative<double>(v);
        case GraphType::Vec3:   return std::holds_alternative<Vec3>(v);
        case GraphType::Field:  return std::holds_alternative<Sdf>(v);
        case GraphType::Mesh:   return std::holds_alternative<MeshPtr>(v);
    }
    return false;
}

const NodeType* NodeRegistry::find(const std::string& name) const {
    auto it = types_.find(name);
    return it != types_.end() ? &it->second : nullptr;
}

std::vector<std::string> NodeRegistry::typeNames() const {
    std::vector<std::string> names;
    names.reserve(types_.size());
    for (const auto& kv : types_) names.push_back(kv.first);
    std::sort(names.begin(), names.end());
    return names;
}

namespace {
// Safe input extraction (a mis-typed or missing input yields a default).
double scalarOf(const std::vector<GraphValue>& in, size_t i, double def = 0.0) {
    if (i < in.size()) if (auto* p = std::get_if<double>(&in[i])) return *p;
    return def;
}
Vec3 vec3Of(const std::vector<GraphValue>& in, size_t i, Vec3 def = Vec3()) {
    if (i < in.size()) if (auto* p = std::get_if<Vec3>(&in[i])) return *p;
    return def;
}
Sdf fieldOf(const std::vector<GraphValue>& in, size_t i) {
    if (i < in.size()) if (auto* p = std::get_if<Sdf>(&in[i])) return *p;
    return [](const Vec3&) { return 1e30; };   // empty field (everything outside)
}
}  // namespace

void registerBuiltinNodes(NodeRegistry& reg) {
    using V = std::vector<GraphValue>;
    using C = NodeContext;

    // A graph input ("knob"): returns params[attr] if the caller set it, else
    // the wired default. The mechanism behind "one graph, many seeds".
    reg.add({"Param",
             {{"default", GraphType::Scalar}},
             GraphType::Scalar,
             [](const V& in, const C& ctx) -> GraphValue {
                 if (ctx.params) {
                     auto it = ctx.params->find(ctx.attr);
                     if (it != ctx.params->end()) return it->second;
                 }
                 return in.empty() ? GraphValue(std::monostate{}) : in[0];
             }});

    reg.add({"SdfSphere",
             {{"center", GraphType::Vec3}, {"radius", GraphType::Scalar}},
             GraphType::Field,
             [](const V& in, const C&) -> GraphValue {
                 return sdfSphere(vec3Of(in, 0), scalarOf(in, 1, 1.0));
             }});

    reg.add({"SdfBox",
             {{"center", GraphType::Vec3}, {"halfExtent", GraphType::Vec3}},
             GraphType::Field,
             [](const V& in, const C&) -> GraphValue {
                 return sdfBox(vec3Of(in, 0), vec3Of(in, 1, Vec3(0.5, 0.5, 0.5)));
             }});

    reg.add({"SdfCapsule",
             {{"a", GraphType::Vec3}, {"b", GraphType::Vec3}, {"radius", GraphType::Scalar}},
             GraphType::Field,
             [](const V& in, const C&) -> GraphValue {
                 return sdfCapsule(vec3Of(in, 0), vec3Of(in, 1, Vec3(0, 1, 0)),
                                   scalarOf(in, 2, 0.3));
             }});

    reg.add({"Union",
             {{"a", GraphType::Field}, {"b", GraphType::Field}},
             GraphType::Field,
             [](const V& in, const C&) -> GraphValue {
                 return sdfUnion(fieldOf(in, 0), fieldOf(in, 1));
             }});

    reg.add({"SmoothUnion",
             {{"a", GraphType::Field}, {"b", GraphType::Field}, {"k", GraphType::Scalar}},
             GraphType::Field,
             [](const V& in, const C&) -> GraphValue {
                 return sdfSmoothUnion(fieldOf(in, 0), fieldOf(in, 1), scalarOf(in, 2, 0.3));
             }});

    reg.add({"Subtract",
             {{"a", GraphType::Field}, {"b", GraphType::Field}},
             GraphType::Field,
             [](const V& in, const C&) -> GraphValue {
                 return sdfSubtract(fieldOf(in, 0), fieldOf(in, 1));
             }});

    reg.add({"Intersect",
             {{"a", GraphType::Field}, {"b", GraphType::Field}},
             GraphType::Field,
             [](const V& in, const C&) -> GraphValue {
                 return sdfIntersect(fieldOf(in, 0), fieldOf(in, 1));
             }});

    reg.add({"Polygonize",
             {{"field", GraphType::Field}, {"min", GraphType::Vec3},
              {"max", GraphType::Vec3}, {"resolution", GraphType::Scalar}},
             GraphType::Mesh,
             [](const V& in, const C&) -> GraphValue {
                 SdfBounds b{vec3Of(in, 1, Vec3(-1, -1, -1)), vec3Of(in, 2, Vec3(1, 1, 1))};
                 int res = static_cast<int>(scalarOf(in, 3, 32.0));
                 return std::make_shared<RenderMesh>(polygonizeSdf(fieldOf(in, 0), b, res));
             }});

    // High-level generator node: a whole rock (base + seeded lumps - cuts,
    // polygonized) as one node — a graph can stay coarse (Param->Rock->out) or
    // drop to primitives. Mirrors generateRockSdf.
    reg.add({"Rock",
             {{"radius", GraphType::Scalar}, {"lumps", GraphType::Scalar},
              {"cuts", GraphType::Scalar}, {"lumpScale", GraphType::Scalar},
              {"smoothness", GraphType::Scalar}, {"resolution", GraphType::Scalar},
              {"seed", GraphType::Scalar}},
             GraphType::Mesh,
             [](const V& in, const C&) -> GraphValue {
                 RockSdfParams p;
                 p.baseRadius = scalarOf(in, 0, p.baseRadius);
                 p.lumps      = static_cast<int>(scalarOf(in, 1, p.lumps));
                 p.cuts       = static_cast<int>(scalarOf(in, 2, p.cuts));
                 p.lumpScale  = scalarOf(in, 3, p.lumpScale);
                 p.smoothness = scalarOf(in, 4, p.smoothness);
                 p.resolution = static_cast<int>(scalarOf(in, 5, p.resolution));
                 uint32_t seed = static_cast<uint32_t>(scalarOf(in, 6, 0.0));
                 return std::make_shared<RenderMesh>(generateRockSdf(p, seed));
             }});
}

GraphValue Graph::evaluate(const NodeRegistry& registry,
                           const std::unordered_map<std::string, GraphValue>& params) const {
    const int n = static_cast<int>(nodes.size());
    if (outputNode < 0 || outputNode >= n) return std::monostate{};

    std::vector<std::optional<GraphValue>> memo(n);
    std::vector<bool> visiting(n, false);

    std::function<GraphValue(int)> evalNode = [&](int idx) -> GraphValue {
        if (idx < 0 || idx >= n) return std::monostate{};
        if (memo[idx]) return *memo[idx];
        if (visiting[idx]) return std::monostate{};   // cycle guard
        visiting[idx] = true;

        const Node& node = nodes[idx];
        const NodeType* type = registry.find(node.type);
        GraphValue result = std::monostate{};
        if (type) {
            std::vector<GraphValue> inputs(type->inputs.size());
            for (size_t i = 0; i < inputs.size(); i++) {
                if (i < node.inputs.size() && node.inputs[i].source >= 0)
                    inputs[i] = evalNode(node.inputs[i].source);
                else if (i < node.inputs.size())
                    inputs[i] = node.inputs[i].literal;
            }
            NodeContext ctx{node.attr, &params};
            result = type->eval(inputs, ctx);
        }
        visiting[idx] = false;
        memo[idx] = result;
        return result;
    };
    return evalNode(outputNode);
}

// --- serialization ---

namespace {
json valueToJson(const GraphValue& v) {
    if (auto* d = std::get_if<double>(&v)) return *d;
    if (auto* p = std::get_if<Vec3>(&v)) return json::array({p->x, p->y, p->z});
    return nullptr;   // Field/Mesh/none are not literals
}
GraphValue jsonToValue(const json& j) {
    if (j.is_number()) return j.get<double>();
    if (j.is_array() && j.size() >= 3)
        return Vec3(j[0].get<double>(), j[1].get<double>(), j[2].get<double>());
    return std::monostate{};
}
}  // namespace

std::string graphToJson(const Graph& graph) {
    json root;
    root["output"] = graph.outputNode;
    json nodes = json::array();
    for (const Graph::Node& node : graph.nodes) {
        json jn;
        jn["type"] = node.type;
        if (!node.attr.empty()) jn["attr"] = node.attr;
        json inputs = json::array();
        for (const Graph::Input& in : node.inputs) {
            json ji;
            if (in.source >= 0) ji["source"] = in.source;
            else                ji["value"] = valueToJson(in.literal);
            inputs.push_back(ji);
        }
        jn["inputs"] = inputs;
        nodes.push_back(jn);
    }
    root["nodes"] = nodes;
    return root.dump(2);
}

// --- editing operations ---

namespace {
GraphValue defaultLiteral(GraphType t) {
    if (t == GraphType::Scalar) return 0.0;
    if (t == GraphType::Vec3) return Vec3();
    return std::monostate{};   // Field/Mesh must be connected
}
// Does `node` transitively depend on `target` via its input connections?
bool dependsOn(const Graph& g, int node, int target) {
    if (node == target) return true;
    if (node < 0 || node >= static_cast<int>(g.nodes.size())) return false;
    for (const Graph::Input& in : g.nodes[node].inputs)
        if (in.source >= 0 && dependsOn(g, in.source, target)) return true;
    return false;
}
}  // namespace

int graphAddNode(Graph& graph, const NodeRegistry& registry, const std::string& type) {
    const NodeType* t = registry.find(type);
    if (!t) return -1;
    Graph::Node node;
    node.type = type;
    for (const NodeSocket& sock : t->inputs) {
        Graph::Input in;
        in.source = -1;
        in.literal = defaultLiteral(sock.type);
        node.inputs.push_back(in);
    }
    graph.nodes.push_back(std::move(node));
    return static_cast<int>(graph.nodes.size()) - 1;
}

bool graphConnect(Graph& graph, const NodeRegistry& registry,
                  int srcNode, int dstNode, int socket) {
    const int n = static_cast<int>(graph.nodes.size());
    if (srcNode < 0 || srcNode >= n || dstNode < 0 || dstNode >= n || srcNode == dstNode)
        return false;
    const NodeType* st = registry.find(graph.nodes[srcNode].type);
    const NodeType* dt = registry.find(graph.nodes[dstNode].type);
    if (!st || !dt) return false;
    if (socket < 0 || socket >= static_cast<int>(dt->inputs.size())) return false;
    if (st->output != dt->inputs[socket].type) return false;        // type mismatch
    if (dependsOn(graph, srcNode, dstNode)) return false;           // would form a cycle
    if (socket >= static_cast<int>(graph.nodes[dstNode].inputs.size()))
        graph.nodes[dstNode].inputs.resize(socket + 1);
    graph.nodes[dstNode].inputs[socket].source = srcNode;
    return true;
}

void graphDisconnect(Graph& graph, int dstNode, int socket) {
    if (dstNode < 0 || dstNode >= static_cast<int>(graph.nodes.size())) return;
    if (socket < 0 || socket >= static_cast<int>(graph.nodes[dstNode].inputs.size())) return;
    graph.nodes[dstNode].inputs[socket].source = -1;
}

Graph graphFromJson(const std::string& text) {
    Graph graph;
    json root = json::parse(text, nullptr, /*allow_exceptions=*/false);
    if (root.is_discarded()) return graph;
    graph.outputNode = root.value("output", -1);
    if (!root.contains("nodes") || !root["nodes"].is_array()) return graph;
    for (const json& jn : root["nodes"]) {
        Graph::Node node;
        node.type = jn.value("type", std::string());
        node.attr = jn.value("attr", std::string());
        if (jn.contains("inputs"))
            for (const json& ji : jn["inputs"]) {
                Graph::Input in;
                if (ji.contains("source")) in.source = ji["source"].get<int>();
                else if (ji.contains("value")) in.literal = jsonToValue(ji["value"]);
                node.inputs.push_back(in);
            }
        graph.nodes.push_back(node);
    }
    return graph;
}

}  // namespace engine
