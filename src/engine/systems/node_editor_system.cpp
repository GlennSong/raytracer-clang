#include "node_editor_system.h"

#include "../components.h"
#include "../asset_manager.h"
#include "../../log.h"
#include <cstring>
#include <fstream>
#include <variant>

#ifdef RT_ENABLE_IMGUI
#include <imgui.h>
#endif

namespace engine {

NodeEditorSystem::NodeEditorSystem() {
    registerBuiltinNodes(registry_);
    std::strncpy(pathBuf_, "assets/generators/rock.graph.json", sizeof(pathBuf_) - 1);
    pathBuf_[sizeof(pathBuf_) - 1] = '\0';
}

#ifdef RT_ENABLE_IMGUI

namespace {
const char* typeLabel(GraphType t) {
    switch (t) {
        case GraphType::Scalar: return "scalar";
        case GraphType::Vec3:   return "vec3";
        case GraphType::Field:  return "field";
        case GraphType::Mesh:   return "mesh";
    }
    return "?";
}
}  // namespace

void NodeEditorSystem::render(FrameContext& ctx) {
    if (!ctx.debugOverlayActive) return;   // an authoring tool, behind the overlay

    ImGui::Begin("Node Editor");

    // --- asset row ---
    ImGui::InputText("path", pathBuf_, sizeof(pathBuf_));
    ImGui::SameLine();
    if (ImGui::Button("Load")) {
        std::ifstream f(pathBuf_);
        if (f) {
            std::string text((std::istreambuf_iterator<char>(f)),
                             std::istreambuf_iterator<char>());
            graph_ = graphFromJson(text);
        } else {
            LOG_WARN << "Node editor: cannot open " << pathBuf_;
        }
    }
    ImGui::SameLine();
    if (ImGui::Button("Save")) {
        std::ofstream f(pathBuf_);
        if (f) f << graphToJson(graph_);
    }

    // --- evaluate / preview ---
    ImGui::DragInt("seed", &seed_, 1.0f, 0, 1 << 20);
    ImGui::SameLine();
    if (ImGui::Button("Evaluate")) evaluatePreview(ctx);

    // --- add node ---
    static int addChoice = 0;
    std::vector<std::string> names = registry_.typeNames();
    if (!names.empty()) {
        if (addChoice >= static_cast<int>(names.size())) addChoice = 0;
        if (ImGui::BeginCombo("##addtype", names[addChoice].c_str())) {
            for (int i = 0; i < static_cast<int>(names.size()); i++)
                if (ImGui::Selectable(names[i].c_str(), addChoice == i)) addChoice = i;
            ImGui::EndCombo();
        }
        ImGui::SameLine();
        if (ImGui::Button("Add Node")) graphAddNode(graph_, registry_, names[addChoice]);
    }

    ImGui::Separator();

    // --- node list ---
    for (int n = 0; n < static_cast<int>(graph_.nodes.size()); n++) {
        ImGui::PushID(n);
        Graph::Node& node = graph_.nodes[n];
        const NodeType* type = registry_.find(node.type);

        bool isOutput = (graph_.outputNode == n);
        if (ImGui::RadioButton("out", isOutput)) graph_.outputNode = n;
        ImGui::SameLine();
        ImGui::Text("[%d] %s", n, node.type.c_str());

        if (!node.attr.empty()) ImGui::Text("  attr: %s", node.attr.c_str());

        if (type) {
            node.inputs.resize(type->inputs.size());   // keep in sync with the type
            for (int s = 0; s < static_cast<int>(type->inputs.size()); s++) {
                ImGui::PushID(s);
                const NodeSocket& sock = type->inputs[s];
                Graph::Input& in = node.inputs[s];

                // Source picker: "(literal)" + every node whose output matches.
                std::string current = in.source >= 0 ? ("#" + std::to_string(in.source))
                                                     : "(literal)";
                ImGui::Text("%s [%s]", sock.name.c_str(), typeLabel(sock.type));
                ImGui::SameLine();
                if (ImGui::BeginCombo("##src", current.c_str())) {
                    if (ImGui::Selectable("(literal)", in.source < 0))
                        graphDisconnect(graph_, n, s);
                    for (int m = 0; m < static_cast<int>(graph_.nodes.size()); m++) {
                        const NodeType* mt = registry_.find(graph_.nodes[m].type);
                        if (!mt || mt->output != sock.type || m == n) continue;
                        std::string lbl = "#" + std::to_string(m) + " " + graph_.nodes[m].type;
                        if (ImGui::Selectable(lbl.c_str(), in.source == m))
                            graphConnect(graph_, registry_, m, n, s);
                    }
                    ImGui::EndCombo();
                }

                // Literal editor for unconnected scalar/vec3 sockets.
                if (in.source < 0) {
                    if (sock.type == GraphType::Scalar) {
                        float v = static_cast<float>(
                            std::holds_alternative<double>(in.literal) ? std::get<double>(in.literal) : 0.0);
                        if (ImGui::DragFloat("##lit", &v, 0.05f)) in.literal = static_cast<double>(v);
                    } else if (sock.type == GraphType::Vec3) {
                        Vec3 vv = std::holds_alternative<Vec3>(in.literal) ? std::get<Vec3>(in.literal) : Vec3();
                        float a[3] = {static_cast<float>(vv.x), static_cast<float>(vv.y),
                                      static_cast<float>(vv.z)};
                        if (ImGui::DragFloat3("##litv", a, 0.05f))
                            in.literal = Vec3(a[0], a[1], a[2]);
                    }
                }
                ImGui::PopID();
            }
        } else {
            ImGui::TextColored(ImVec4(1, 0.5f, 0.3f, 1), "  unknown node type");
        }
        ImGui::Separator();
        ImGui::PopID();
    }

    ImGui::End();
}

void NodeEditorSystem::evaluatePreview(FrameContext& ctx) {
    std::unordered_map<std::string, GraphValue> params{
        {"seed", GraphValue(static_cast<double>(seed_))}};
    GraphValue out = graph_.evaluate(registry_, params);
    auto* mp = std::get_if<MeshPtr>(&out);
    if (!mp || !*mp || (*mp)->vertices.empty()) {
        LOG_WARN << "Node editor: graph did not produce a mesh";
        return;
    }

    // Replace the previous preview mesh (un-keyed = a fresh upload; release the
    // old so the AssetManager frees it).
    if (previewMesh_.valid()) ctx.assets.releaseMesh(previewMesh_);
    previewMesh_ = ctx.assets.acquireMesh(**mp);

    if (!preview_.valid() || !ctx.world.has<Renderable>(preview_)) {
        preview_ = ctx.world.create();
        Transform t;
        t.position = Vec3(0, 2, 0);
        ctx.world.add<Transform>(preview_, t);
        ctx.world.add<PrevTransform>(preview_, PrevTransform{t});
        Renderable r;
        r.material.albedo = Vec3(0.7, 0.7, 0.7);
        r.material.roughness = 0.9f;
        ctx.world.add<Renderable>(preview_, r);
    }
    ctx.world.get<Renderable>(preview_)->mesh = previewMesh_;
}

#else   // !RT_ENABLE_IMGUI

void NodeEditorSystem::render(FrameContext&) {}
void NodeEditorSystem::evaluatePreview(FrameContext&) {}

#endif

}  // namespace engine
