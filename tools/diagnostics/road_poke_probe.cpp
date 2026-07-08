// Terrain poke-through diagnostic.
// Grows the living_city road net on the living_city terrain, computes the road
// conform (flatten), then for every top deck vertex compares the deck height to
// (a) the EXACT carved terrain (what the offline mesh / collider see) and
// (b) the CDLOD-sampled carved terrain (what the device draws) — so we can tell
// a conform bug from a LOD coarse-sampling artifact. Buckets poke-through by
// distance-to-junction and by local cross-slope.
#include "../../src/engine/procgen/city/road_net.h"
#include "../../src/engine/procgen/city/road_network.h"
#include "../../src/engine/procgen/terrain.h"
#include "../../src/engine/procgen/terrain_lod.h"
#include <nlohmann/json.hpp>
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <vector>
using namespace engine;

int main(int argc, char** argv) {
    TerrainParams tp;
    tp.size = 620; tp.resolution = 380; tp.heightScale = 13;
    tp.noiseScale = 0.008; tp.octaves = 4; tp.warp = 0.3; tp.mountainScale = 0.01;
    tp.rangeSpine = sampleRangeSpine({Vec3(-300,0,-255),Vec3(-150,0,-288),Vec3(20,0,-284),
                                      Vec3(190,0,-268),Vec3(310,0,-244)});
    tp.rangeWidth = 135; tp.rangeHeight = 76; tp.rangeVariation = 0.6;
    Noise noise(7u);
    auto natural = [&](double x, double z){ return terrainHeight(tp, noise, x, z); };

    RoadNet net; net.width = 7.0; net.sidewalk = 3.5; net.curb = 0.16;
    net.cornerRadius = 3.0; net.lift = 0.08;
    nlohmann::json gen = {{"kind","district"},{"radius",170},{"arterials",3},
        {"artery_width",13},{"street_width",7},{"block_size",82},{"curviness",0.22},{"seed",5}};
    applyGenerateRecipe(net, gen);
    net.heightAt = natural;

    // conform -> carved terrain (exact)
    TerrainParams carved = tp;
    carved.flatten = roadNetConformRegions(net);
    auto carvedH = [&](double x, double z){ return terrainHeight(carved, noise, x, z); };
    // CDLOD-sampled carved terrain: approximate the device by sampling the same
    // carved field with the half-cell FLATTEN DILATION coarse tiles use.
    // A representative coarse leaf cell on this world is ~ worldHalf/2^numLods.
    double dilate = argc > 1 ? atof(argv[1]) : 0.0;   // 0 = exact; try 1.0-3.0 for coarse LOD
    auto cdlodH = [&](double x, double z){ return terrainHeight(carved, noise, x, z, dilate); };

    RenderMesh mesh = buildRoadNetMesh(net);
    RoadGraph g = navRoadGraph(net);
    std::vector<int> deg(g.nodes.size(), 0);
    for (const RoadEdge& e : g.edges) { ++deg[e.a]; ++deg[e.b]; }
    auto nearestJunction = [&](double x, double z){
        double best = 1e30;
        for (int v = 0; v < (int)g.nodes.size(); ++v)
            if (deg[v] >= 3) best = std::min(best, (Vec2(x,z) - g.nodes[v].pos).length());
        return best;
    };

    int deck = 0, pokeExact = 0, pokeCdlod = 0;
    double worstExact = 0, worstCdlod = 0;
    double wx = 0, wz = 0;
    int proudN[4] = {0,0,0,0};                 // <0.45 | 0.45-0.7 | 0.7-1.0 | >1.0 m
    double worstProud = 0, px = 0, pz = 0;
    // buckets by distance to junction
    int bJ[3] = {0,0,0}, bJn[3] = {0,0,0};   // <8m, 8-20m, >20m : poke count / total
    // buckets by cross-slope (natural gradient magnitude)
    int bS[3] = {0,0,0}, bSn[3] = {0,0,0};
    for (const Vertex& v : mesh.vertices) {
        if (v.normal.y < 0.5) continue;   // deck/sidewalk tops only
        ++deck;
        double x = v.position.x, z = v.position.z, y = v.position.y;
        double dE = carvedH(x, z) - y;
        double dC = cdlodH(x, z) - y;
        double jd = nearestJunction(x, z);
        // local slope of NATURAL ground (metres per metre)
        double e = 2.0;
        double gx = (natural(x+e,z)-natural(x-e,z))/(2*e);
        double gz = (natural(x,z+e)-natural(x,z-e))/(2*e);
        double slope = std::sqrt(gx*gx+gz*gz);
        int jb = jd < 8 ? 0 : (jd < 20 ? 1 : 2);
        int sb = slope < 0.05 ? 0 : (slope < 0.15 ? 1 : 2);
        ++bJn[jb]; ++bSn[sb];
        if (dE > 0.02) { ++pokeExact; ++bJ[jb]; ++bS[sb];
            if (dE > worstExact) { worstExact = dE; wx = x; wz = z; } }
        if (dC > 0.02) { ++pokeCdlod; if (dC > worstCdlod) worstCdlod = dC; }
        // PROUD (plinth): how far the CDLOD terrain sits BELOW this deck/sidewalk
        // vertex. Normal curb+carve ~0.38 m; a big value = an exposed skirt cliff.
        double proud = -dC;   // deck - terrain
        if (proud > 0.0) {
            ++proudN[proud < 0.45 ? 0 : (proud < 0.7 ? 1 : (proud < 1.0 ? 2 : 3))];
            if (proud > worstProud) { worstProud = proud; px = x; pz = z; }
        }
    }
    printf("# POKE-THROUGH — living_city. deck verts=%d  (dilate=%.1f)\n", deck, dilate);
    printf("EXACT carved terrain : poke verts=%d (%.2f%%)  worst=%.2f m @ (%.0f,%.0f)\n",
           pokeExact, 100.0*pokeExact/deck, worstExact, wx, wz);
    printf("CDLOD-sampled        : poke verts=%d (%.2f%%)  worst=%.2f m\n",
           pokeCdlod, 100.0*pokeCdlod/deck, worstCdlod);
    printf("# PROUD (apron above CDLOD terrain — exposed skirt): worst=%.2f m @ (%.0f,%.0f)\n",
           worstProud, px, pz);
    printf("  0.00-0.45m: %d   0.45-0.70m: %d   0.70-1.0m: %d   >1.0m: %d  (of %d top verts)\n",
           proudN[0], proudN[1], proudN[2], proudN[3], deck);
    printf("# poke rate by distance-to-junction:\n");
    const char* jl[3] = {"<8m (in junction)","8-20m","  >20m (mid-span)"};
    for (int b = 0; b < 3; ++b)
        printf("  %-18s: %d/%d = %.2f%%\n", jl[b], bJ[b], bJn[b], bJn[b]?100.0*bJ[b]/bJn[b]:0);
    printf("# poke rate by natural cross-slope:\n");
    const char* sl[3] = {"gentle (<3deg)","moderate (3-8.5deg)","steep (>8.5deg)"};
    for (int b = 0; b < 3; ++b)
        printf("  %-20s: %d/%d = %.2f%%\n", sl[b], bS[b], bSn[b], bSn[b]?100.0*bS[b]/bSn[b]:0);
    return 0;
}
