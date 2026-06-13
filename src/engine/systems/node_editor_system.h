#ifndef RAYTRACER_ENGINE_NODE_EDITOR_SYSTEM_H
#define RAYTRACER_ENGINE_NODE_EDITOR_SYSTEM_H

#include "../system.h"
#include "../procgen/node_graph.h"
#include <string>

namespace engine {

// In-viewport ImGui editor for procgen generator graphs (ADR-0021 Phase C,
// docs/node-graph-plan.md). A v1 list-style editor: add nodes from the registry,
// wire/disconnect typed sockets, edit literal params, set the output node, and
// load/save the graph asset — with a live preview mesh evaluated into the world.
// Inert without RT_ENABLE_IMGUI (like the other panels, ADR-0011). The visual
// node *canvas* (boxes + bezier wires) is a follow-up; this drives the same
// editing model (graphConnect/graphAddNode/...).
class NodeEditorSystem : public System {
public:
    NodeEditorSystem();
    void render(FrameContext& ctx) override;

private:
    void evaluatePreview(FrameContext& ctx);

    NodeRegistry registry_;
    Graph graph_;
    Entity preview_;            // the live-evaluated mesh entity
    MeshHandle previewMesh_;    // freed/replaced on each re-evaluate
    char pathBuf_[256];
    int seed_ = 1;
    bool show_ = false;         // toggled in the panel; gated by the debug overlay
};

}  // namespace engine

#endif
