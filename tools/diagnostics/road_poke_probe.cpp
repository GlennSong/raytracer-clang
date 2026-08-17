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
#include "../../src/engine/procgen/erosion.h"
#include "../../src/engine/procgen/terrain_lod.h"
#include <nlohmann/json.hpp>
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <string>
#include <vector>
using namespace engine;

int main(int argc, char** argv) {
    // Mode: default = the small living_city study; "metropolis" = the real 2.6km
    // ERODED coastal shelf + radius-1000 metro (plan P3.1 evidence pass).
    const bool metro = argc > 2 && std::string(argv[2]) == "metropolis";
    TerrainParams tp;
    nlohmann::json gen;
    if (metro) {
        tp.size = 2600; tp.resolution = 900; tp.heightScale = 13;
        tp.noiseScale = 0.0026; tp.octaves = 6; tp.warp = 0.35;
        tp.mountainHeight = 180; tp.mountainScale = 0.002;
        tp.mountainMaskScale = 0.001; tp.mountainMaskLo = -0.1f; tp.mountainMaskHi = 0.4f;
        tp.mountainAlongRange = true; tp.tiltX = -0.045; tp.seaLevel = -5.0;
        tp.rangeSpine = sampleRangeSpine({Vec3(-1180,0,-900),Vec3(-1120,0,-300),
                                          Vec3(-1180,0,300),Vec3(-1120,0,900)});
        tp.rangeWidth = 300; tp.rangeHeight = 260; tp.rangeVariation = 0.6;
        gen = {{"kind","metro"},{"radius",1000},{"hotspots",9},{"block_size",150},
               {"artery_width",13},{"street_width",7},{"freeway_width",22},
               {"collector_width",9.5},{"interchange_spacing",700},{"seed",9},
               {"seg_length",32},{"influence",340},{"kill_radius",70},
               {"merge_radius",52},{"corridor_spacing",90},{"ambient_per_500",40},
               {"loop_min",140},{"loop_max",330},{"min_road_len",24},
               {"sea_level",-5.0}};
    } else {
        tp.size = 620; tp.resolution = 380; tp.heightScale = 13;
        tp.noiseScale = 0.008; tp.octaves = 4; tp.warp = 0.3; tp.mountainScale = 0.01;
        tp.rangeSpine = sampleRangeSpine({Vec3(-300,0,-255),Vec3(-150,0,-288),Vec3(20,0,-284),
                                          Vec3(190,0,-268),Vec3(310,0,-244)});
        tp.rangeWidth = 135; tp.rangeHeight = 76; tp.rangeVariation = 0.6;
        gen = {{"kind","district"},{"radius",170},{"arterials",3},
            {"artery_width",13},{"street_width",7},{"block_size",82},{"curviness",0.22},{"seed",5}};
    }
    Noise noise(metro ? 11u : 7u);
    if (metro) {
        // The device bakes EROSION into the shared height path — conform evidence
        // must run on the same ground (erosion re-carving corridors is hypothesis
        // #4 in the plan). Same knobs as metropolis.json.
        ErosionParams ep;
        ep.droplets = 500000; ep.thermalIterations = 14; ep.talus = 1.0f;
        ep.erodeRadius = 4;
        bakeErodedTerrain(tp, noise, tp.size, 800, ep);
    }
    auto natural = [&](double x, double z){ return terrainHeight(tp, noise, x, z); };
    const std::function<double(double, double)> ground = natural;

    RoadEntity net; net.look.defaultWidth = 7.0; net.look.sidewalk = 3.5; net.look.curb = 0.16;
    net.look.cornerRadius = 3.0; net.look.lift = 0.08;
    // ground is passed BEFORE generate: the metro recipe gates on terrain
    applyGenerateRecipe(net, gen, ground);

    // conform -> carved terrain (exact)
    TerrainParams carved = tp;
    carved.flatten = roadNetConformRegions(net, ground);
    auto carvedH = [&](double x, double z){ return terrainHeight(carved, noise, x, z); };
    // CDLOD-sampled carved terrain: approximate the device by sampling the same
    // carved field with the half-cell FLATTEN DILATION coarse tiles use.
    // A representative coarse leaf cell on this world is ~ worldHalf/2^numLods.
    double dilate = argc > 1 ? atof(argv[1]) : 0.0;   // 0 = exact; try 1.0-3.0 for coarse LOD
    auto cdlodH = [&](double x, double z){ return terrainHeight(carved, noise, x, z, dilate); };

    RenderMesh mesh = buildRoadNetMesh(net, ground);
    RoadGraph g = navRoadGraph(net, ground);
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
    int proudIn = 0, proudOut = 0, proudOutDumped = 0;
    struct Site { double proud, x, z; };
    std::vector<Site> worstSites;
    std::vector<UnionSpine> cSpines = roadNetWeldSpines(navRoadGraph(net, ground));
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
            // P3.2 round 2 evidence: is a badly-proud vertex INSIDE some chain's
            // carve corridor (plane disagreement) or OUTSIDE every corridor
            // (coverage hole — junction corner / sidewalk wrap)?
            if (proud > 1.0) {
                bool inside = false;
                double minRel = 1e30;
                for (std::size_t sj = 0; sj < cSpines.size() && !inside; ++sj) {
                    const auto& pts = cSpines[sj].points;
                    const double reach = cSpines[sj].halfWidth + 3.5 + 2.0;
                    for (std::size_t i = 0; i + 1 < pts.size(); ++i) {
                        Vec2 ab = pts[i + 1] - pts[i];
                        double L2 = ab.lengthSquared();
                        double t = L2 < 1e-12 ? 0.0
                                              : std::max(0.0, std::min(1.0, dot(Vec2(x, z) - pts[i], ab) / L2));
                        double d = (Vec2(x, z) - (pts[i] + ab * t)).length();
                        minRel = std::min(minRel, d - reach);
                        if (d <= reach) { inside = true; break; }
                    }
                }
                if (inside) ++proudIn; else ++proudOut;
                worstSites.push_back({proud, x, z});
                if (!inside && proudOutDumped < 5) {
                    printf("  proud>1m OUTSIDE corridors: %.2f m @ (%.0f,%.0f), %.2f m past reach\n",
                           proud, x, z, minRel);
                    ++proudOutDumped;
                }
            }
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
    printf("  >1m proud: %d INSIDE a carve corridor (plane disagreement), %d OUTSIDE all (coverage hole)\n",
           proudIn, proudOut);
    // Top offender SITES (clustered to 30 m so one junction = one line), with
    // junction distance — the forensics pointer for the next fix round.
    std::sort(worstSites.begin(), worstSites.end(),
              [](const Site& a, const Site& b) { return a.proud > b.proud; });
    std::vector<Site> clustered;
    for (const Site& s : worstSites) {
        bool dup = false;
        for (const Site& c : clustered)
            if (std::hypot(s.x - c.x, s.z - c.z) < 30.0) { dup = true; break; }
        if (!dup) clustered.push_back(s);
        if (clustered.size() >= 6) break;
    }
    for (const Site& s : clustered)
        printf("  site: %.2f m proud @ (%.0f,%.0f), %.1f m to nearest junction\n",
               s.proud, s.x, s.z, nearestJunction(s.x, s.z));
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
