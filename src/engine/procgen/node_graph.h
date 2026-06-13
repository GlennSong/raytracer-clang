#ifndef RAYTRACER_ENGINE_PROCGEN_NODE_GRAPH_H
#define RAYTRACER_ENGINE_PROCGEN_NODE_GRAPH_H

#include "sdf.h"                       // Sdf, polygonizeSdf
#include "../../rt_math.h"
#include "../../renderer/renderer.h"   // RenderMesh
#include <functional>
#include <memory>
#include <string>
#include <unordered_map>
#include <variant>
#include <vector>

namespace engine {

// The procgen node graph (ADR-0021 Phase C, docs/node-graph-plan.md): generators
// authored as a graph of composable nodes — data, not C++. This is the headless
// engine half (value types + node registry + evaluation + serialization); the
// visual editor sits on top later. Geometry graph only (Mesh/Field/...); shader
// and particle graphs are separate types (ADR-0021).

// A value flowing on a wire. Meshes are shared (cheap to pass); monostate is
// "none" (a default / error result).
using MeshPtr = std::shared_ptr<RenderMesh>;
using GraphValue = std::variant<std::monostate, double, Vec3, Sdf, MeshPtr>;

enum class GraphType { Scalar, Vec3, Field, Mesh };
bool valueIsType(const GraphValue& v, GraphType t);

// A node type: typed input sockets, an output type, and an evaluation thunk
// that maps resolved inputs (in socket order) + a context to its output.
// Registered once per type — the registry is the single place node types are
// named (mirrors ComponentRegistry / the property layer).
struct NodeSocket {
    std::string name;
    GraphType type;
};
// Evaluation context: a node's config string (e.g. a Param's name) and the
// graph's external parameters (the exposed knobs a host/loader sets per call —
// "one graph, many seeds").
struct NodeContext {
    std::string attr;
    const std::unordered_map<std::string, GraphValue>* params = nullptr;
};
struct NodeType {
    std::string name;
    std::vector<NodeSocket> inputs;
    GraphType output;
    std::function<GraphValue(const std::vector<GraphValue>&, const NodeContext&)> eval;
};

class NodeRegistry {
public:
    void add(NodeType type) { types_[type.name] = std::move(type); }
    const NodeType* find(const std::string& name) const;

private:
    std::unordered_map<std::string, NodeType> types_;
};

// The built-in geometry nodes: SDF primitives (Sphere/Box/Capsule), booleans
// (Union/SmoothUnion/Subtract/Intersect), and Polygonize (Field -> Mesh). Each
// is a thin wrapper over sdf.h.
void registerBuiltinNodes(NodeRegistry& registry);

// A graph instance: nodes plus, per node input socket, either a connection to
// another node's output or a literal (Scalar/Vec3 only — Field/Mesh come from
// connections). Nodes have a single output.
struct Graph {
    struct Input {
        int source = -1;        // node index, or < 0 for a literal
        GraphValue literal;     // used when source < 0
    };
    struct Node {
        std::string type;
        std::vector<Input> inputs;
        std::string attr;        // optional per-node config (e.g. a Param's name)
    };
    std::vector<Node> nodes;
    int outputNode = -1;

    // Pull-evaluate the output node, memoizing each node. `params` are the
    // graph's exposed inputs (read by Param nodes). Returns monostate on any
    // error (unknown type, cycle, missing output).
    GraphValue evaluate(const NodeRegistry& registry,
                        const std::unordered_map<std::string, GraphValue>& params = {}) const;
};

// The graph is the generator asset: round-trips through JSON.
std::string graphToJson(const Graph& graph);
Graph graphFromJson(const std::string& json);

}  // namespace engine

#endif
