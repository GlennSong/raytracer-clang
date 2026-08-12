// roadlab — a clean-room prototype of a parametric road system.
//
// Headless by design: it renders to PNG, prints reports, and runs the traffic
// model without a window, so the whole thing is testable on any machine.
//
//   roadlab --demo showcase --out shot.png
//   roadlab --demo lanes --view top --debug strips
//   roadlab --scene my.json --sim 45 --cars 160 --view persp
//   roadlab --city --seed 3 --report --lint
//   roadlab --shots out/            (renders the whole demo set)
//   roadlab --import town.xodr --view top --report

#include "odr.h"
#include "scene.h"
#include "sim.h"
#include <cstdio>
#include <cstring>
#include <string>
#include <vector>

using namespace roadlab;

namespace {

struct Options {
    std::string demo = "showcase";
    std::string scenePath;
    std::string out = "roadlab.png";
    std::string view = "top";
    std::string shotsDir;
    std::string dumpJson;
    std::string xodrPath;
    std::string importPath;
    bool city = false;
    int width = 1600, height = 1000;
    int cars = 0, peds = 0;
    double simSeconds = 0;
    double wear = -1, grime = -1;
    int debugMode = 0;
    bool props = true, terrain = true, markings = true, decals = true;
    bool report = false, lint = false, list = false, quiet = false;
    uint32_t seed = 7;
    int threads = 0;
    double camX = 0, camY = 0, camZ = 0, camDist = 0;
    bool haveCam = false;
    double focusX = 0, focusZ = 0, focusSpan = 0;
    bool haveFocus = false;
    int driverRoad = -1, driverLane = -1;
    double driverS = -1;
};

bool matchArg(int& i, int argc, char** argv, const char* name, std::string& out) {
    if (std::strcmp(argv[i], name) != 0) return false;
    if (i + 1 >= argc) return false;
    out = argv[++i];
    return true;
}

bool matchNum(int& i, int argc, char** argv, const char* name, double& out) {
    std::string s;
    if (!matchArg(i, argc, argv, name, s)) return false;
    out = std::atof(s.c_str());
    return true;
}

void usage() {
    std::printf(
        "roadlab — parametric road prototype\n"
        "  --demo <name>       one of: showcase lanes roundabout interchange urban tiers\n"
        "  --scene <file.json> load an authored scene\n"
        "  --city              generate a network procedurally\n"
        "  --seed <n>          generator / shading seed\n"
        "  --out <file.png>    output image (default roadlab.png)\n"
        "  --shots <dir>       render every demo into <dir>\n"
        "  --view top|persp|driver\n"
        "  --width/--height    image size\n"
        "  --cam x y z         perspective eye point\n"
        "  --focus x z span    frame the top-down view on a region\n"
        "  --driver r l s      driver's eye on road r, lane l, station s\n"
        "  --sim <seconds>     run the traffic model before rendering\n"
        "  --cars/--peds <n>   agents to seed\n"
        "  --wear/--grime <0..1>\n"
        "  --debug strips|grid|none\n"
        "  --no-props --no-terrain --no-markings --no-decals\n"
        "  --report            print a structural report\n"
        "  --lint              print design-rule violations\n"
        "  --dump <file.json>  write a summary of the built network\n"
        "  --xodr <file.xodr>  export the network as OpenDRIVE\n"
        "  --import <f.xodr>   load an OpenDRIVE file as the scene\n"
        "  --list              list presets and demos\n");
}

void printReport(const Scene& sc, const Simulation* sim) {
    const Network& net = sc.net;
    int normal = 0, ramps = 0, rings = 0, connectors = 0;
    double totalLength = 0, laneKm = 0;
    for (const Road& r : net.roads()) {
        switch (r.kind) {
            case RoadKind::Normal: ++normal; break;
            case RoadKind::Ramp: ++ramps; break;
            case RoadKind::RoundaboutRing: ++rings; break;
            case RoadKind::Connector: ++connectors; break;
        }
        totalLength += r.activeLength();
        laneKm += r.activeLength() * r.xs.sectionAt(r.begin()).laneCount();
    }
    std::printf("\n=== %s ===\n", sc.name.c_str());
    std::printf("roads          %d  (%d normal, %d ramp, %d ring, %d connector)\n",
                net.roadCount(), normal, ramps, rings, connectors);
    std::printf("centreline     %.0f m     lane-km %.1f\n", totalLength, laneKm / 1000.0);
    std::printf("lane sections  %d\n", [&] {
        int n = 0;
        for (const Road& r : net.roads()) n += int(r.xs.sections.size());
        return n;
    }());
    std::printf("lane graph     %d nodes\n", int(net.lanes().nodes.size()));

    int conns = 0, conflicts = 0, phases = 0;
    std::printf("junctions      %d\n", net.junctionCount());
    for (const Junction& j : net.junctions()) {
        conns += int(j.connections.size());
        conflicts += int(j.conflicts.size());
        phases += int(j.phases.size());
    }
    std::printf("  connectors   %d\n  conflicts    %d\n  signal phases %d\n", conns, conflicts,
                phases);
    for (const Junction& j : net.junctions()) {
        if (j.connections.empty()) continue;
        std::printf("  - %-16s %s, %zu arms, %zu movements, %zu conflicts",
                    j.name.c_str(), controlName(j.control), j.arms.size(),
                    j.connections.size(), j.conflicts.size());
        if (!j.phases.empty()) std::printf(", %zu phases (%.0fs cycle)", j.phases.size(),
                                           j.cycleLength());
        std::printf("\n");
    }

    int decals = 0;
    for (const RoadPaint& rp : sc.paint.roads) decals += int(rp.decals.size());
    std::printf("paint decals   %d\n", decals);
    std::printf("props          %zu\n", sc.props.size());
    std::printf("mesh           %zu triangles, %zu vertices\n", sc.mesh.triangleCount(),
                sc.mesh.verts.size());

    if (sim) {
        SimStats st = sim->stats();
        std::printf("\nsim after %.0fs: %d vehicles (%d moving, %d stopped), mean %.1f km/h,\n"
                    "  mean wait %.1fs, %d lane changes (%d mandatory), %d trips completed,\n"
                    "  %d routes planned, %d currently needing a lane change for their route,\n"
                    "  %d pedestrians (%d crossing now, %d waiting at a kerb, %d crossings made,\n"
                    "    mean kerb wait %.1fs),\n"
                    "  %d parked cars\n",
                    sim->time(), st.vehicles, st.moving, st.stopped, st.meanSpeedKph, st.meanWait,
                    st.laneChanges, st.mandatoryChanges, st.completedTrips, st.replans,
                    st.offRoute, st.pedestrians, st.pedsCrossing, st.pedsWaiting,
                    st.crossingsMade, st.meanPedWait, st.parkedCars);
    }
}

Camera makeCamera(const Scene& sc, const Options& opt) {
    double aspect = double(opt.width) / double(opt.height);
    Vec2 lo, hi;
    sc.net.planBounds(lo, hi);
    if (opt.haveFocus) {
        double h = opt.focusSpan * 0.5;
        lo = {opt.focusX - h * aspect, opt.focusZ - h};
        hi = {opt.focusX + h * aspect, opt.focusZ + h};
    }
    if (opt.view == "driver") {
        int roadId = opt.driverRoad;
        if (roadId < 0) {
            // Pick the longest ordinary road and sit in its outermost lane.
            double best = 0;
            for (const Road& r : sc.net.roads()) {
                if (r.kind == RoadKind::Connector) continue;
                if (r.activeLength() > best) {
                    best = r.activeLength();
                    roadId = r.id;
                }
            }
        }
        if (roadId >= 0) {
            const Road& r = sc.net.road(roadId);
            int lane = opt.driverLane;
            if (lane == 0 || lane == -1) {
                EndLanes el = endLanes(r, false);
                lane = el.outgoing.empty() ? -1 : el.outgoing.front();
            }
            double s = opt.driverS >= 0 ? opt.driverS : r.begin() + r.activeLength() * 0.12;
            return cameraDriver(r, clampd(s, r.begin() + 1, r.end() - 1), lane);
        }
    }
    if (opt.view == "persp") {
        Vec2 centre = (lo + hi) * 0.5;
        double span = std::max(hi.x - lo.x, hi.y - lo.y);
        Vec3 target{centre.x, 4.0, centre.y};
        if (opt.haveCam) return cameraLook({opt.camX, opt.camY, opt.camZ}, target, 42.0);
        double d = span * 0.62;
        return cameraLook({centre.x - d * 0.75, span * 0.42 + 40.0, centre.y - d * 0.85}, target,
                          40.0);
    }
    return cameraTopDown(lo, hi, aspect);
}

}  // namespace

int main(int argc, char** argv) {
    Options opt;
    for (int i = 1; i < argc; ++i) {
        std::string s;
        double d = 0;
        if (matchArg(i, argc, argv, "--demo", opt.demo)) continue;
        if (matchArg(i, argc, argv, "--scene", opt.scenePath)) continue;
        if (matchArg(i, argc, argv, "--out", opt.out)) continue;
        if (matchArg(i, argc, argv, "--shots", opt.shotsDir)) continue;
        if (matchArg(i, argc, argv, "--view", opt.view)) continue;
        if (matchArg(i, argc, argv, "--dump", opt.dumpJson)) continue;
        if (matchArg(i, argc, argv, "--xodr", opt.xodrPath)) continue;
        if (matchArg(i, argc, argv, "--import", opt.importPath)) continue;
        if (matchNum(i, argc, argv, "--width", d)) { opt.width = int(d); continue; }
        if (matchNum(i, argc, argv, "--height", d)) { opt.height = int(d); continue; }
        if (matchNum(i, argc, argv, "--cars", d)) { opt.cars = int(d); continue; }
        if (matchNum(i, argc, argv, "--peds", d)) { opt.peds = int(d); continue; }
        if (matchNum(i, argc, argv, "--sim", d)) { opt.simSeconds = d; continue; }
        if (matchNum(i, argc, argv, "--wear", d)) { opt.wear = d; continue; }
        if (matchNum(i, argc, argv, "--grime", d)) { opt.grime = d; continue; }
        if (matchNum(i, argc, argv, "--seed", d)) { opt.seed = uint32_t(d); continue; }
        if (matchNum(i, argc, argv, "--threads", d)) { opt.threads = int(d); continue; }
        if (std::strcmp(argv[i], "--cam") == 0 && i + 3 < argc) {
            opt.camX = std::atof(argv[i + 1]);
            opt.camY = std::atof(argv[i + 2]);
            opt.camZ = std::atof(argv[i + 3]);
            opt.haveCam = true;
            i += 3;
            continue;
        }
        if (std::strcmp(argv[i], "--focus") == 0 && i + 3 < argc) {
            opt.focusX = std::atof(argv[i + 1]);
            opt.focusZ = std::atof(argv[i + 2]);
            opt.focusSpan = std::atof(argv[i + 3]);
            opt.haveFocus = true;
            i += 3;
            continue;
        }
        if (std::strcmp(argv[i], "--driver") == 0 && i + 3 < argc) {
            opt.driverRoad = std::atoi(argv[i + 1]);
            opt.driverLane = std::atoi(argv[i + 2]);
            opt.driverS = std::atof(argv[i + 3]);
            opt.view = "driver";
            i += 3;
            continue;
        }
        if (matchArg(i, argc, argv, "--debug", s)) {
            opt.debugMode = s == "strips" ? 1 : (s == "grid" ? 3 : 0);
            continue;
        }
        if (std::strcmp(argv[i], "--city") == 0) { opt.city = true; continue; }
        if (std::strcmp(argv[i], "--no-props") == 0) { opt.props = false; continue; }
        if (std::strcmp(argv[i], "--no-terrain") == 0) { opt.terrain = false; continue; }
        if (std::strcmp(argv[i], "--no-markings") == 0) { opt.markings = false; continue; }
        if (std::strcmp(argv[i], "--no-decals") == 0) { opt.decals = false; continue; }
        if (std::strcmp(argv[i], "--report") == 0) { opt.report = true; continue; }
        if (std::strcmp(argv[i], "--lint") == 0) { opt.lint = true; continue; }
        if (std::strcmp(argv[i], "--list") == 0) { opt.list = true; continue; }
        if (std::strcmp(argv[i], "--quiet") == 0) { opt.quiet = true; continue; }
        if (std::strcmp(argv[i], "--help") == 0 || std::strcmp(argv[i], "-h") == 0) {
            usage();
            return 0;
        }
        std::fprintf(stderr, "unknown argument: %s\n", argv[i]);
        usage();
        return 1;
    }

    if (opt.list) {
        std::printf("demos:   ");
        for (const std::string& n : demoNames()) std::printf("%s ", n.c_str());
        std::printf("\npresets: ");
        for (const std::string& n : roadPresetNames()) std::printf("%s ", n.c_str());
        std::printf("\n");
        return 0;
    }

    auto buildOne = [&](Scene& sc, const std::string& demo) -> bool {
        sc.shade.seed = opt.seed;
        sc.terrain.seed = opt.seed + 3;
        sc.propRules.seed = opt.seed + 11;
        sc.terrain.enabled = opt.terrain;
        if (opt.wear >= 0) sc.shade.wear = opt.wear;
        if (opt.grime >= 0) sc.shade.grime = opt.grime;
        sc.shade.markings = opt.markings;
        sc.shade.decals = opt.decals;
        sc.shade.debugMode = opt.debugMode;

        if (!opt.importPath.empty()) {
            std::string err;
            OdrImportReport rep;
            if (!readOpenDrive(opt.importPath, sc.net, err, &rep)) {
                std::fprintf(stderr, "roadlab: %s\n", err.c_str());
                return false;
            }
            if (!rep.name.empty()) sc.name = rep.name;
            if (!opt.quiet) {
                std::printf("imported %s: %d roads, %d junctions, %d lane sections, %d lanes\n",
                            opt.importPath.c_str(), rep.roads, rep.junctions, rep.laneSections,
                            rep.lanes);
                for (const std::string& note : rep.notes)
                    std::printf("  note: %s\n", note.c_str());
            }
        } else if (!opt.scenePath.empty()) {
            std::string err;
            if (!loadSceneJson(opt.scenePath, sc, err)) {
                std::fprintf(stderr, "roadlab: %s\n", err.c_str());
                return false;
            }
        } else if (opt.city) {
            CityParams cp;
            cp.seed = opt.seed;
            generateCity(sc, cp);
        } else if (!buildDemo(demo, sc)) {
            std::fprintf(stderr, "roadlab: unknown demo '%s'\n", demo.c_str());
            return false;
        }
        finalizeScene(sc, opt.terrain, opt.props);
        return true;
    };

    auto renderOne = [&](Scene& sc, const std::string& path) {
        Mesh renderMesh = sc.mesh;
        Simulation sim(sc.net, [&] {
            SimParams sp;
            sp.seed = opt.seed + 5;
            return sp;
        }());
        bool haveSim = opt.cars > 0 || opt.peds > 0 || opt.simSeconds > 0;
        if (haveSim) {
            sim.seedParking(0.55);
            if (opt.cars > 0) sim.seedVehicles(opt.cars);
            if (opt.peds > 0) sim.seedPedestrians(opt.peds);
            if (opt.simSeconds > 0) sim.run(opt.simSeconds);
            tessellateAgents(sim, renderMesh);
        }

        RenderOptions ro;
        ro.width = opt.width;
        ro.height = opt.height;
        ro.shade = sc.shade;
        ro.threads = opt.threads;
        Camera cam = makeCamera(sc, opt);
        Framebuffer fb;
        renderScene(renderMesh, sc.net, sc.paint, cam, ro, fb);
        if (!writePng(fb, path)) {
            std::fprintf(stderr, "roadlab: failed to write %s\n", path.c_str());
            return;
        }
        if (!opt.quiet) std::printf("wrote %s (%dx%d)\n", path.c_str(), opt.width, opt.height);
        if (opt.report) printReport(sc, haveSim ? &sim : nullptr);
        if (opt.lint) {
            if (sc.lint.empty()) {
                std::printf("lint: clean\n");
            } else {
                std::printf("lint: %zu finding(s)\n", sc.lint.size());
                for (const std::string& m : sc.lint) std::printf("  ! %s\n", m.c_str());
            }
        }
        if (!opt.xodrPath.empty()) {
            std::string err;
            OdrOptions oo;
            oo.name = sc.name;
            if (writeOpenDrive(sc.net, opt.xodrPath, err, oo))
                std::printf("wrote %s\n", opt.xodrPath.c_str());
            else
                std::fprintf(stderr, "roadlab: %s\n", err.c_str());
        }
        if (!opt.dumpJson.empty()) {
            std::string err;
            if (saveSceneJson(sc, opt.dumpJson, err))
                std::printf("dumped %s\n", opt.dumpJson.c_str());
            else
                std::fprintf(stderr, "roadlab: %s\n", err.c_str());
        }
    };

    if (!opt.shotsDir.empty()) {
        for (const std::string& demo : demoNames()) {
            Scene sc;
            if (!buildOne(sc, demo)) continue;
            renderOne(sc, opt.shotsDir + "/" + demo + ".png");
        }
        return 0;
    }

    Scene sc;
    if (!buildOne(sc, opt.demo)) return 1;
    renderOne(sc, opt.out);
    return 0;
}
