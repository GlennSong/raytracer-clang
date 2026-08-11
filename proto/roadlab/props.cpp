#include "props.h"

#include <cstdio>
#include <cstring>

namespace roadlab {

const char* propKindName(PropKind k) {
    switch (k) {
        case PropKind::StreetLight: return "street-light";
        case PropKind::TrafficSignal: return "traffic-signal";
        case PropKind::PedSignal: return "ped-signal";
        case PropKind::SignStop: return "sign-stop";
        case PropKind::SignYield: return "sign-yield";
        case PropKind::SignSpeed: return "sign-speed";
        case PropKind::SignAdvisory: return "sign-advisory";
        case PropKind::SignGuide: return "sign-guide";
        case PropKind::SignMerge: return "sign-merge";
        case PropKind::SignLaneEnds: return "sign-lane-ends";
        case PropKind::SignCurve: return "sign-curve";
        case PropKind::SignNoEntry: return "sign-no-entry";
        case PropKind::Tree: return "tree";
        case PropKind::Bench: return "bench";
        case PropKind::Hydrant: return "hydrant";
        case PropKind::ParkingMeter: return "parking-meter";
        case PropKind::BusStop: return "bus-stop";
        case PropKind::Bollard: return "bollard";
        case PropKind::CrashAttenuator: return "crash-attenuator";
        case PropKind::MilePost: return "mile-post";
    }
    return "?";
}

namespace {

struct Placer {
    const Network& net;
    const PropRules& rules;
    std::vector<Prop>& out;
    Rng rng;

    Placer(const Network& n, const PropRules& r, std::vector<Prop>& o)
        : net(n), rules(r), out(o), rng(r.seed) {}

    void add(PropKind kind, const Road& road, double s, double t, double yawOffset = 0.0,
             double value = 0.0, const char* label = nullptr) {
        Prop p;
        p.kind = kind;
        p.road = road.id;
        p.s = s;
        p.t = t;
        p.pos = road.surfacePoint(s, t);
        Frame f = road.spine.frameAt(s);
        p.yaw = wrapPi(f.heading + yawOffset);
        p.value = value;
        if (label) std::snprintf(p.label, sizeof p.label, "%s", label);
        out.push_back(p);
    }
};

// Stations on a road that a junction owns, so furniture keeps out of the way.
std::vector<double> junctionStationsOf(const Network& net, int roadId) {
    std::vector<double> v;
    for (const Junction& j : net.junctions())
        for (const JunctionArm& a : j.arms)
            if (a.road == roadId) v.push_back(a.sContact);
    return v;
}

// The furthest-out kerb-side offset a prop can stand at without being in the
// carriageway: just outside the footway, or just outside the shoulder.
bool vergeOffset(const Road& r, double s, int side, double& tOut) {
    const LaneSection& sec = r.xs.sectionAt(s);
    const std::vector<Strip>& stack = side > 0 ? sec.left : sec.right;
    if (stack.empty()) return false;
    double ds = s - sec.s0, acc = 0, best = -1;
    for (const Strip& st : stack) {
        double w = std::max(0.0, st.width.eval(ds));
        if (st.kind == StripKind::Sidewalk) {
            // On the outer third of the footway: clear of the kerb, clear of the
            // property line.
            best = acc + w * 0.72;
        } else if (st.kind == StripKind::Verge && best < 0) {
            best = acc + w * 0.5;
        } else if ((st.kind == StripKind::Shoulder || st.kind == StripKind::Slope) && best < 0) {
            best = acc + std::min(w, 1.6) * 0.95;
        }
        acc += w;
    }
    if (best < 0) best = acc + 0.8;
    tOut = side > 0 ? best : -best;
    return true;
}

bool stripOffset(const Road& r, double s, StripKind kind, int side, double& tOut) {
    const LaneSection& sec = r.xs.sectionAt(s);
    const std::vector<Strip>& stack = side > 0 ? sec.left : sec.right;
    double ds = s - sec.s0, acc = 0;
    for (const Strip& st : stack) {
        double w = std::max(0.0, st.width.eval(ds));
        if (st.kind == kind && w > 0.3) {
            tOut = side > 0 ? acc + w * 0.5 : -(acc + w * 0.5);
            return true;
        }
        acc += w;
    }
    return false;
}

}  // namespace

void generateProps(const Network& net, std::vector<Prop>& out, const PropRules& rules) {
    out.clear();
    Placer P(net, rules, out);

    for (const Road& r : net.roads()) {
        if (r.kind == RoadKind::Connector) continue;
        std::vector<double> jstations = junctionStationsOf(net, r.id);
        auto nearJunction = [&](double s, double d) {
            for (double js : jstations)
                if (std::fabs(s - js) < d) return true;
            return false;
        };

        // --- lighting -----------------------------------------------------
        if (rules.lights) {
            int side = 1;
            for (double s = r.begin() + 6.0; s < r.end() - 6.0; s += rules.lightSpacing) {
                if (nearJunction(s, 8.0)) continue;
                double t = 0;
                if (!vergeOffset(r, s, side, t)) continue;
                // A lamp on a bridge stands on the parapet, not in mid-air off
                // the edge of the deck.
                CarrierKind carrier = CarrierKind::AtGrade;
                for (const StructureSpan& sp : r.structures)
                    if (sp.contains(s)) carrier = sp.kind;
                if (carrier == CarrierKind::Tunnel) continue;
                if (carrier == CarrierKind::Bridge || carrier == CarrierKind::Viaduct) {
                    t = side > 0 ? r.xs.leftExtentAt(s) - 0.3 : r.xs.rightExtentAt(s) + 0.3;
                }
                P.add(PropKind::StreetLight, r, s, t, side > 0 ? -kPi * 0.5 : kPi * 0.5);
                if (rules.alternateLights) side = -side;
            }
        }

        // --- vegetation and street furniture -------------------------------
        if (rules.vegetation) {
            for (int side : {-1, 1}) {
                double t = 0;
                for (double s = r.begin() + 5.0; s < r.end() - 5.0;
                     s += rules.treeSpacing * P.rng.range(0.75, 1.35)) {
                    if (nearJunction(s, rules.junctionClearance)) continue;
                    if (!stripOffset(r, s, StripKind::Verge, side, t)) continue;
                    P.add(PropKind::Tree, r, s, t + P.rng.range(-0.3, 0.3), P.rng.range(-kPi, kPi));
                }
            }
        }
        if (rules.streetFurniture) {
            for (int side : {-1, 1}) {
                double tPark = 0;
                if (stripOffset(r, r.begin() + 1.0, StripKind::Parking, side, tPark)) {
                    // Meters live on the footway beside the bays they serve.
                    for (double s = r.begin() + 4.0; s < r.end() - 4.0; s += rules.meterSpacing) {
                        if (nearJunction(s, 12.0)) continue;
                        double t = 0;
                        if (!vergeOffset(r, s, side, t)) continue;
                        P.add(PropKind::ParkingMeter, r, s, t, side > 0 ? -kPi * 0.5 : kPi * 0.5);
                    }
                }
                double t = 0;
                for (double s = r.begin() + 20.0; s < r.end() - 10.0; s += rules.hydrantSpacing) {
                    if (nearJunction(s, 10.0)) continue;
                    if (!vergeOffset(r, s, side, t)) continue;
                    const LaneSection& sec = r.xs.sectionAt(s);
                    bool hasFootway = false;
                    for (const std::vector<Strip>* st : {&sec.left, &sec.right})
                        for (const Strip& x : *st)
                            if (x.kind == StripKind::Sidewalk) hasFootway = true;
                    if (!hasFootway) continue;
                    P.add(PropKind::Hydrant, r, s, t, 0.0);
                }
            }
        }

        // --- signs from the cross-section ----------------------------------
        if (!rules.signs) continue;

        for (size_t si = 0; si < r.xs.sections.size(); ++si) {
            const LaneSection& sec = r.xs.sections[si];
            double s0 = std::max(sec.s0, r.begin());
            double s1 = std::min(sec.s0 + sec.length, r.end());
            if (s1 - s0 < 1.0) continue;
            for (int side = 0; side < 2; ++side) {
                const std::vector<Strip>& stack = side == 0 ? sec.right : sec.left;
                for (const Strip& st : stack) {
                    if (!st.isLane()) continue;
                    double wStart = std::max(0.0, st.width.eval(s0 - sec.s0));
                    double wEnd = std::max(0.0, st.width.eval(s1 - sec.s0));
                    // LANE ENDS stands one taper length before the taper, which
                    // is the distance the taper itself was computed from.
                    if (wStart > 1.5 && wEnd < 0.6) {
                        double warn = taperLength(wStart, r.designSpeed);
                        double s = (st.dir >= 0) ? s0 - warn : s1 + warn;
                        s = clampd(s, r.begin() + 2.0, r.end() - 2.0);
                        double t = 0;
                        int postSide = st.dir >= 0 ? -1 : +1;
                        if (vergeOffset(r, s, postSide, t))
                            P.add(PropKind::SignLaneEnds, r, s, t,
                                  st.dir >= 0 ? kPi : 0.0, 0, "LANE ENDS");
                    }
                    // A lane growing out of nothing is an entrance; the MERGE
                    // sign belongs where the mainline driver first sees it.
                    if (wStart < 0.6 && wEnd > 1.5) {
                        double s = clampd(s0 - 60.0, r.begin() + 2.0, r.end() - 2.0);
                        double t = 0;
                        int postSide = st.dir >= 0 ? -1 : +1;
                        if (vergeOffset(r, s, postSide, t))
                            P.add(PropKind::SignMerge, r, s, t, st.dir >= 0 ? kPi : 0.0, 0,
                                  "MERGE");
                    }
                }
            }
        }

        // Speed limit where it changes, and a curve warning where the geometry
        // demands a speed the road otherwise implies it can carry.
        bool speedChange = r.pred.type != LinkType::Road;
        if (r.pred.type == LinkType::Road && r.pred.id >= 0)
            speedChange = std::fabs(net.road(r.pred.id).designSpeed - r.designSpeed) > 5.0;
        if (speedChange && r.activeLength() > 25.0) {
            double s = r.begin() + 12.0;
            double t = 0;
            if (vergeOffset(r, s, -1, t))
                P.add(PropKind::SignSpeed, r, s, t, kPi, r.designSpeed, "SPEED");
        }
        for (const GeomPrim& g : r.spine.prims()) {
            double k = std::max(std::fabs(g.curv0), std::fabs(g.curv1));
            if (k < 1e-6) continue;
            double vSafe = designSpeedForRadius(1.0 / k);
            if (vSafe > r.designSpeed * 0.85) continue;
            double s = clampd(g.s0 - 45.0, r.begin() + 2.0, r.end() - 2.0);
            if (nearJunction(s, 12.0)) continue;
            double t = 0;
            if (vergeOffset(r, s, -1, t))
                P.add(PropKind::SignCurve, r, s, t, kPi, std::floor(vSafe / 5.0) * 5.0, "CURVE");
            break;
        }
    }

    // --- junction-driven furniture ----------------------------------------
    for (const Junction& j : net.junctions()) {
        for (size_t ai = 0; ai < j.arms.size(); ++ai) {
            const JunctionArm& arm = j.arms[ai];
            const Road& r = net.road(arm.road);
            if (r.kind == RoadKind::Connector) continue;
            EndLanes el = endLanes(r, arm.atEnd);
            if (el.incoming.empty()) continue;
            // The near-right kerb of the approach, which is where a driver
            // arriving at this arm looks.
            int postSide = arm.atEnd ? -1 : +1;
            double up = arm.atEnd ? -1.0 : +1.0;
            double sPost = clampd(arm.sContact + up * 3.0, r.begin() + 0.5, r.end() - 0.5);
            double t = 0;
            if (!vergeOffset(r, sPost, postSide, t)) continue;
            double facing = arm.atEnd ? kPi : 0.0;

            bool yields = false;
            for (const Connection& c : j.connections)
                if (c.fromArm == int(ai) && c.yields) yields = true;

            if (rules.signals && j.control == JunctionControl::Signalized) {
                P.add(PropKind::TrafficSignal, r, sPost, t, facing, 0, "SIGNAL");
                if (r.allowsPedestrians) P.add(PropKind::PedSignal, r, sPost, t * 0.92, facing);
            } else if (rules.signs) {
                switch (j.control) {
                    case JunctionControl::AllWayStop:
                        P.add(PropKind::SignStop, r, sPost, t, facing, 0, "STOP");
                        break;
                    case JunctionControl::PriorityStop:
                        if (yields) P.add(PropKind::SignStop, r, sPost, t, facing, 0, "STOP");
                        break;
                    case JunctionControl::Yield:
                        if (yields) P.add(PropKind::SignYield, r, sPost, t, facing, 0, "YIELD");
                        break;
                    case JunctionControl::RoundaboutEntry:
                        if (r.kind != RoadKind::RoundaboutRing)
                            P.add(PropKind::SignYield, r, sPost, t, facing, 0, "YIELD");
                        break;
                    default:
                        break;
                }
            }
        }

        // Advisory plaque on a connector whose own curvature caps its speed well
        // below the roads it joins — the number comes from the geometry.
        if (rules.signs) {
            for (const Connection& c : j.connections) {
                if (c.connectorRoad < 0) continue;
                const Road& conn = net.road(c.connectorRoad);
                const Road& from = net.road(j.arms[size_t(c.fromArm)].road);
                if (conn.designSpeed > from.designSpeed * 0.6) continue;
                double t = 0;
                if (!vergeOffset(conn, conn.begin() + 2.0, -1, t)) t = -4.0;
                P.add(PropKind::SignAdvisory, conn, conn.begin() + 2.0, t, kPi,
                      std::floor(conn.designSpeed / 5.0) * 5.0, "ADVISORY");
            }
        }
    }

    // Crash attenuators at ramp gores: where a ramp's carriageway parts from the
    // mainline there is a nose, and a nose gets protected.
    for (const Road& r : net.roads()) {
        if (r.kind != RoadKind::Ramp) continue;
        double s = r.begin() + 1.0;
        Prop p;
        p.kind = PropKind::CrashAttenuator;
        p.road = r.id;
        p.s = s;
        p.t = r.xs.laneOffset.eval(s);
        p.pos = r.surfacePoint(s, p.t);
        p.yaw = r.spine.frameAt(s).heading;
        std::snprintf(p.label, sizeof p.label, "%s", "GORE");
        out.push_back(p);
    }
}

// --- geometry -------------------------------------------------------------

namespace {

const Vec3 kPole{0.30, 0.31, 0.32};
const Vec3 kSignWhite{0.80, 0.80, 0.78};
const Vec3 kSignRed{0.58, 0.09, 0.10};
const Vec3 kSignGreen{0.07, 0.30, 0.16};
const Vec3 kSignYellow{0.78, 0.62, 0.06};
const Vec3 kBark{0.14, 0.10, 0.07};
const Vec3 kLeaf{0.09, 0.20, 0.07};

void emitCone(Mesh& m, Vec3 base, double radius, double height, int sides, Vec3 color) {
    sides = std::max(3, sides);
    Vertex apex;
    apex.pos = base + Vec3{0, height, 0};
    apex.normal = {0, 1, 0};
    apex.color = color;
    apex.roughness = 0.9;
    apex.material = MatKind::Flat;
    for (int i = 0; i < sides; ++i) {
        double a0 = kTwoPi * double(i) / double(sides);
        double a1 = kTwoPi * double(i + 1) / double(sides);
        Vec3 p0 = base + Vec3{std::cos(a0) * radius, 0, std::sin(a0) * radius};
        Vec3 p1 = base + Vec3{std::cos(a1) * radius, 0, std::sin(a1) * radius};
        Vec3 n = normalize(cross(p1 - p0, apex.pos - p0));
        Vertex v0 = apex, v1 = apex, v2 = apex;
        v0.pos = p0;
        v1.pos = p1;
        v0.normal = v1.normal = v2.normal = n;
        m.tri(m.push(v0), m.push(v1), m.push(v2));
    }
}

// A sign: a post plus a face of the right shape and colour. The shape is the
// message — an octagon means stop whether or not you can read the legend.
void emitSign(Mesh& m, const Prop& p, int sides, double startAngle, double radius, Vec3 face,
              Vec3 border, double height) {
    emitCylinder(m, p.pos, 0.055, height, 8, kPole, 0.6);
    Vec3 centre = p.pos + Vec3{0, height + radius * 0.85, 0};
    Vec3 right{std::cos(p.yaw + kPi * 0.5) * radius, 0, std::sin(p.yaw + kPi * 0.5) * radius};
    Vec3 up{0, radius, 0};
    emitPolygonFace(m, centre, right * 1.06, up * 1.06, sides, startAngle, border);
    emitPolygonFace(m, centre + Vec3{0, 0, 0}, right, up, sides, startAngle, face);
}

}  // namespace

void tessellateProps(const std::vector<Prop>& props, Mesh& out) {
    for (const Prop& p : props) {
        switch (p.kind) {
            case PropKind::StreetLight: {
                double h = 8.5;
                emitCylinder(out, p.pos, 0.10, h, 8, kPole, 0.6);
                Vec3 armDir{std::cos(p.yaw), 0, std::sin(p.yaw)};
                Vec3 top = p.pos + Vec3{0, h, 0};
                emitBox(out, top + armDir * 1.1 + Vec3{0, 0.15, 0}, {1.2, 0.07, 0.07}, p.yaw,
                        kPole, 0.6);
                emitBox(out, top + armDir * 2.2, {0.42, 0.10, 0.20}, p.yaw, {0.62, 0.60, 0.50},
                        0.35);
                break;
            }
            case PropKind::TrafficSignal: {
                double h = 6.4;
                emitCylinder(out, p.pos, 0.11, h, 8, kPole, 0.6);
                Vec3 armDir{std::cos(p.yaw), 0, std::sin(p.yaw)};
                Vec3 top = p.pos + Vec3{0, h, 0};
                emitBox(out, top + armDir * 2.6, {2.7, 0.08, 0.08}, p.yaw, kPole, 0.6);
                Vec3 head = top + armDir * 4.6 - Vec3{0, 0.6, 0};
                emitBox(out, head, {0.20, 0.62, 0.22}, p.yaw, {0.09, 0.09, 0.09}, 0.5);
                const Vec3 lamps[3] = {{0.62, 0.06, 0.05}, {0.62, 0.45, 0.05}, {0.08, 0.52, 0.14}};
                for (int i = 0; i < 3; ++i) {
                    emitBox(out, head + Vec3{0, 0.38 - 0.38 * i, 0}, {0.21, 0.13, 0.13}, p.yaw,
                            lamps[i], 0.35);
                }
                break;
            }
            case PropKind::PedSignal: {
                emitCylinder(out, p.pos, 0.07, 2.9, 6, kPole, 0.6);
                emitBox(out, p.pos + Vec3{0, 2.95, 0}, {0.16, 0.22, 0.16}, p.yaw,
                        {0.09, 0.09, 0.09}, 0.5);
                break;
            }
            case PropKind::SignStop:
                emitSign(out, p, 8, kPi / 8.0, 0.42, kSignRed, kSignWhite, 2.1);
                break;
            case PropKind::SignYield:
                emitSign(out, p, 3, -kPi * 0.5, 0.52, kSignWhite, kSignRed, 2.1);
                break;
            case PropKind::SignSpeed:
                emitSign(out, p, 4, kPi * 0.25, 0.42, kSignWhite, {0.15, 0.15, 0.15}, 2.1);
                break;
            case PropKind::SignAdvisory:
            case PropKind::SignCurve:
            case PropKind::SignMerge:
            case PropKind::SignLaneEnds:
                emitSign(out, p, 4, 0.0, 0.46, kSignYellow, {0.12, 0.12, 0.12}, 2.1);
                break;
            case PropKind::SignGuide:
                emitSign(out, p, 4, kPi * 0.25, 0.9, kSignGreen, kSignWhite, 2.6);
                break;
            case PropKind::SignNoEntry:
                emitSign(out, p, 12, 0.0, 0.40, kSignRed, kSignWhite, 2.1);
                break;
            case PropKind::Tree: {
                double h = 2.4 + 1.4 * std::fabs(std::sin(p.yaw * 3.1));
                emitCylinder(out, p.pos, 0.16, h, 6, kBark, 0.95);
                emitCone(out, p.pos + Vec3{0, h * 0.6, 0}, 1.9, 3.4, 7, kLeaf);
                emitCone(out, p.pos + Vec3{0, h * 1.25, 0}, 1.35, 2.6, 7,
                         kLeaf * 1.15);
                break;
            }
            case PropKind::Bench:
                emitBox(out, p.pos + Vec3{0, 0.42, 0}, {0.85, 0.05, 0.25}, p.yaw,
                        {0.20, 0.13, 0.08}, 0.85);
                emitBox(out, p.pos + Vec3{0, 0.2, 0}, {0.75, 0.2, 0.05}, p.yaw, kPole, 0.7);
                break;
            case PropKind::Hydrant:
                emitCylinder(out, p.pos, 0.12, 0.62, 8, {0.52, 0.10, 0.08}, 0.7);
                emitBox(out, p.pos + Vec3{0, 0.70, 0}, {0.16, 0.10, 0.16}, p.yaw,
                        {0.52, 0.10, 0.08}, 0.7);
                break;
            case PropKind::ParkingMeter:
                emitCylinder(out, p.pos, 0.045, 1.05, 6, kPole, 0.6);
                emitBox(out, p.pos + Vec3{0, 1.18, 0}, {0.09, 0.16, 0.07}, p.yaw,
                        {0.16, 0.17, 0.18}, 0.5);
                break;
            case PropKind::BusStop: {
                for (double dx : {-1.4, 1.4})
                    emitCylinder(out, p.pos + Vec3{std::cos(p.yaw) * dx, 0, std::sin(p.yaw) * dx},
                                 0.05, 2.5, 6, kPole, 0.6);
                emitBox(out, p.pos + Vec3{0, 2.55, 0}, {1.6, 0.06, 0.75}, p.yaw,
                        {0.25, 0.26, 0.28}, 0.45);
                break;
            }
            case PropKind::Bollard:
                emitCylinder(out, p.pos, 0.08, 0.95, 8, {0.35, 0.30, 0.10}, 0.7);
                break;
            case PropKind::CrashAttenuator:
                emitBox(out, p.pos + Vec3{0, 0.45, 0}, {1.8, 0.45, 0.55}, p.yaw,
                        {0.66, 0.55, 0.10}, 0.7);
                break;
            case PropKind::MilePost:
                emitCylinder(out, p.pos, 0.04, 1.1, 4, kPole, 0.6);
                emitBox(out, p.pos + Vec3{0, 1.2, 0}, {0.02, 0.16, 0.10}, p.yaw, kSignGreen, 0.5);
                break;
        }
    }
}

}  // namespace roadlab
