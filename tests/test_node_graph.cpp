#include "test_framework.h"

#include "../src/engine/procgen/node_graph.h"
#include "../src/engine/procgen/sdf.h"
#include <variant>

using namespace engine;  // namespace migration (ADR-0015)

namespace {

// rock-ish blob: two spheres smooth-unioned, polygonized.
Graph makeBlobGraph() {
    auto lit = [](GraphValue v) { Graph::Input i; i.source = -1; i.literal = std::move(v); return i; };
    auto src = [](int s) { Graph::Input i; i.source = s; return i; };
    Graph g;
    g.nodes.push_back({"SdfSphere", {lit(Vec3(-0.5, 0, 0)), lit(0.8)}});
    g.nodes.push_back({"SdfSphere", {lit(Vec3(0.5, 0, 0)), lit(0.8)}});
    g.nodes.push_back({"SmoothUnion", {src(0), src(1), lit(0.3)}});
    g.nodes.push_back({"Polygonize",
                       {src(2), lit(Vec3(-2, -2, -2)), lit(Vec3(2, 2, 2)), lit(28.0)}});
    g.outputNode = 3;
    return g;
}

MeshPtr meshOf(const GraphValue& v) {
    auto* p = std::get_if<MeshPtr>(&v);
    return p ? *p : nullptr;
}

}  // namespace

TEST_CASE(node_graph_output_matches_direct_calls) {
    NodeRegistry reg;
    registerBuiltinNodes(reg);
    MeshPtr graphMesh = meshOf(makeBlobGraph().evaluate(reg));
    CHECK(graphMesh != nullptr);

    // The same composition done by calling the substrate functions directly.
    Sdf field = sdfSmoothUnion(sdfSphere(Vec3(-0.5, 0, 0), 0.8),
                               sdfSphere(Vec3(0.5, 0, 0), 0.8), 0.3);
    RenderMesh direct = polygonizeSdf(field, {Vec3(-2, -2, -2), Vec3(2, 2, 2)}, 28);

    CHECK(graphMesh->vertices.size() > 0);
    CHECK(graphMesh->vertices.size() == direct.vertices.size());
    CHECK(graphMesh->indices.size() == direct.indices.size());
    bool same = graphMesh->vertices.size() == direct.vertices.size();
    for (size_t i = 0; same && i < direct.vertices.size(); i++)
        if ((graphMesh->vertices[i].position - direct.vertices[i].position).lengthSquared() > 1e-12)
            same = false;
    CHECK(same);   // the graph engine reproduces the hand-written result exactly
}

TEST_CASE(node_graph_round_trips_through_json) {
    NodeRegistry reg;
    registerBuiltinNodes(reg);
    Graph g = makeBlobGraph();
    Graph reloaded = graphFromJson(graphToJson(g));

    CHECK(reloaded.nodes.size() == g.nodes.size());
    CHECK(reloaded.outputNode == g.outputNode);
    MeshPtr before = meshOf(g.evaluate(reg));
    MeshPtr after = meshOf(reloaded.evaluate(reg));
    CHECK(before && after);
    CHECK(after->vertices.size() == before->vertices.size());
}

TEST_CASE(node_graph_errors_return_none_not_crash) {
    NodeRegistry reg;
    registerBuiltinNodes(reg);

    Graph unknown;
    unknown.nodes.push_back({"NoSuchNode", {}});
    unknown.outputNode = 0;
    CHECK(std::holds_alternative<std::monostate>(unknown.evaluate(reg)));

    Graph noOutput;   // outputNode stays -1
    CHECK(std::holds_alternative<std::monostate>(noOutput.evaluate(reg)));

    // A self-referential input must terminate (cycle guard), not hang.
    Graph cyclic;
    cyclic.nodes.push_back({"SmoothUnion", {{0, {}}, {0, {}}, {-1, GraphValue(0.3)}}});
    cyclic.outputNode = 0;
    cyclic.evaluate(reg);   // returns (any value); the test completing proves it terminated
    CHECK(true);
}

TEST_CASE(node_graph_value_type_check) {
    CHECK(valueIsType(GraphValue(1.5), GraphType::Scalar));
    CHECK(valueIsType(GraphValue(Vec3(1, 2, 3)), GraphType::Vec3));
    CHECK(!valueIsType(GraphValue(1.5), GraphType::Vec3));
    CHECK(!valueIsType(GraphValue(std::monostate{}), GraphType::Mesh));
}
