#include "road_net.h"

#include "road_network.h"       // RoadGraph, RoadEdge
#include <algorithm>

namespace engine {

using json = nlohmann::json;

RenderMesh buildRoadNetMesh(const RoadNet& net) {
    RoadGraph g;
    g.nodes.resize(net.nodes.size());
    for (std::size_t i = 0; i < net.nodes.size(); ++i) g.nodes[i].pos = net.nodes[i];
    const int n = static_cast<int>(net.nodes.size());
    for (const std::array<int, 2>& e : net.edges) {
        if (e[0] < 0 || e[1] < 0 || e[0] >= n || e[1] >= n || e[0] == e[1]) continue;
        RoadEdge re;
        re.a = e[0]; re.b = e[1]; re.width = static_cast<Real>(net.width);
        g.edges.push_back(re);
    }
    RoadMeshParams p;
    p.lift = net.lift;
    p.color = net.color;
    p.sidewalkWidth = net.sidewalk;
    p.curbHeight = net.curb;
    p.cornerRadius = net.cornerRadius;
    p.laneMarkings = net.markings;
    p.crosswalks = net.crosswalks;
    p.minSetback = net.width * 0.5 + 0.5;        // pad clears the curb corners
    p.heightAt = net.heightAt;
    return buildRoadMesh(g, p);
}

void roadNetSetWidth(RoadNet& net, double width) {
    net.width = std::max(0.5, width);
}

bool roadNetMoveNode(RoadNet& net, int i, const Vec2& pos) {
    if (i < 0 || i >= static_cast<int>(net.nodes.size())) return false;
    net.nodes[i] = pos;
    return true;
}

RoadNet roadNetFromJson(const json& j) {
    RoadNet net;
    if (j.contains("nodes") && j["nodes"].is_array())
        for (const json& p : j["nodes"])
            net.nodes.push_back(Vec2(p.value("x", 0.0), p.value("z", 0.0)));
    if (j.contains("edges") && j["edges"].is_array())
        for (const json& e : j["edges"]) {
            if (e.is_array() && e.size() >= 2)
                net.edges.push_back({e[0].get<int>(), e[1].get<int>()});
            else if (e.is_object())
                net.edges.push_back({e.value("a", 0), e.value("b", 0)});
        }
    net.width = j.value("width", net.width);
    net.sidewalk = j.value("sidewalk", net.sidewalk);
    net.curb = j.value("curb", net.curb);
    net.cornerRadius = j.value("corner_radius", net.cornerRadius);
    net.lift = j.value("lift", net.lift);
    net.markings = j.value("markings", net.markings);
    net.crosswalks = j.value("crosswalks", net.crosswalks);
    if (j.contains("color") && j["color"].is_array() && j["color"].size() == 3)
        net.color = Vec3(j["color"][0].get<double>(), j["color"][1].get<double>(),
                         j["color"][2].get<double>());
    return net;
}

json roadNetToJson(const RoadNet& net) {
    json j;
    json nodes = json::array();
    for (const Vec2& p : net.nodes) nodes.push_back({{"x", p.x}, {"z", p.y}});
    j["nodes"] = std::move(nodes);
    json edges = json::array();
    for (const std::array<int, 2>& e : net.edges) edges.push_back(json::array({e[0], e[1]}));
    j["edges"] = std::move(edges);
    j["width"] = net.width;
    j["sidewalk"] = net.sidewalk;
    j["curb"] = net.curb;
    j["corner_radius"] = net.cornerRadius;
    j["lift"] = net.lift;
    j["markings"] = net.markings;
    j["crosswalks"] = net.crosswalks;
    j["color"] = json::array({net.color.x, net.color.y, net.color.z});
    return j;
}

}  // namespace engine
