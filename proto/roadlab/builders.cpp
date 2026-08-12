#include "builders.h"

#include <cstdio>

namespace roadlab {

int buildRoad(Network& net, const RoadDesc& desc) {
    Road r;
    r.name = desc.name;
    r.kind = desc.kind;
    RoadPreset preset = roadPreset(desc.preset);
    r.designSpeed = desc.designSpeed > 0 ? desc.designSpeed : preset.designSpeedKph;
    r.allowsPedestrians = preset.allowsPedestrians;
    r.spine = spineFromPolyline(desc.points, desc.cornerRadius, desc.spirals);
    r.spine.setCrossfall(desc.crossfall);
    if (desc.elevationKnots.empty()) {
        r.spine.setFlatElevation(desc.baseHeight);
    } else {
        r.spine.setElevationKnots(desc.elevationKnots);
    }
    if (desc.autoSuperelevation) r.spine.applySuperelevation(r.designSpeed);
    r.spine.finalize();

    LaneSection sec = preset.section;
    sec.s0 = 0;
    sec.assignIds();
    r.xs.sections.push_back(sec);
    r.xs.finalize(r.spine.length());
    return net.addRoad(std::move(r));
}

int buildIntersection(Network& net, const std::string& name,
                      const std::vector<std::pair<int, bool>>& arms, JunctionControl control,
                      double cornerRadius) {
    Junction j;
    j.name = name;
    j.control = control;
    j.cornerRadius = cornerRadius;
    for (const auto& a : arms) {
        JunctionArm arm;
        arm.road = a.first;
        arm.atEnd = a.second;
        j.arms.push_back(arm);
    }
    int id = net.addJunction(j);
    // Tell each arm road that this end belongs to a junction, so the lane graph
    // does not also try to link it road-to-road.
    for (const auto& a : arms) {
        Road& r = net.road(a.first);
        RoadLink link;
        link.type = LinkType::Junction;
        link.id = id;
        if (a.second)
            r.succ = link;
        else
            r.pred = link;
    }
    return id;
}

void buildRoundabout(Network& net, const RoundaboutDesc& desc) {
    if (desc.arms.size() < 2) return;

    // Bearing of each arm as seen from the centre. That ordering is the only
    // thing that makes a roundabout a roundabout rather than a pile of roads.
    struct ArmInfo {
        int road;
        bool atEnd;
        double bearing;
    };
    std::vector<ArmInfo> arms;
    for (const auto& a : desc.arms) {
        const Road& r = net.road(a.first);
        double s = a.second ? r.end() : r.begin();
        Vec2 p = r.spine.toPlan(s, 0);
        arms.push_back({a.first, a.second, std::atan2(p.y - desc.center.y, p.x - desc.center.x)});
    }
    std::sort(arms.begin(), arms.end(),
              [](const ArmInfo& a, const ArmInfo& b) { return a.bearing < b.bearing; });

    RoadPreset ring = roadPreset(desc.ringPreset);

    // One arc per gap. Arc i runs from arm i to arm i+1, counter-clockwise.
    std::vector<int> arcIds;
    for (size_t i = 0; i < arms.size(); ++i) {
        double a0 = arms[i].bearing;
        double a1 = arms[(i + 1) % arms.size()].bearing;
        Road r;
        char buf[96];
        std::snprintf(buf, sizeof buf, "%s.ring%zu", desc.name.c_str(), i);
        r.name = buf;
        r.kind = RoadKind::RoundaboutRing;
        r.designSpeed = ring.designSpeedKph;
        r.allowsPedestrians = false;
        r.spine = spineArc(desc.center, desc.radius, a0, a1, true);
        r.spine.setCrossfall(0.02);
        r.spine.setFlatElevation(desc.height);
        r.spine.finalize();
        LaneSection sec = ring.section;
        sec.s0 = 0;
        sec.assignIds();
        r.xs.sections.push_back(sec);
        r.xs.finalize(r.spine.length());
        arcIds.push_back(net.addRoad(std::move(r)));
    }

    // A junction at each arm: the ring arriving, the ring leaving, and the
    // approach. Circulating traffic wins because the ring roads are of kind
    // RoundaboutRing, which buildJunction already knows how to read.
    for (size_t i = 0; i < arms.size(); ++i) {
        int incoming = arcIds[(i + arcIds.size() - 1) % arcIds.size()];
        int outgoing = arcIds[i];
        char buf[96];
        std::snprintf(buf, sizeof buf, "%s.arm%zu", desc.name.c_str(), i);
        buildIntersection(net, buf,
                          {{incoming, true}, {outgoing, false}, {arms[i].road, arms[i].atEnd}},
                          JunctionControl::RoundaboutEntry, 5.0);
    }
}

// --- cross-section edits --------------------------------------------------

namespace {

// The base (uniform) section a road was built from. These edits assume the road
// still has the single section buildRoad gave it — an editor would instead layer
// sparse overrides onto the timeline, which is the right long-term answer.
LaneSection baseSectionOf(const Road& r) {
    LaneSection s = r.xs.sections.front();
    s.s0 = 0;
    for (std::vector<Strip>* stack : {&s.left, &s.right})
        for (Strip& st : *stack) st.width = Poly3::constant(st.width.eval(0.0));
    return s;
}

}  // namespace

void rebuildProfile(Network& net, int roadId) {
    Road& r = net.road(roadId);
    if (r.profileEdits.empty()) return;
    LaneSection base = baseSectionOf(r);
    std::vector<Road::ProfileEdit> edits = r.profileEdits;
    std::sort(edits.begin(), edits.end(),
              [](const Road::ProfileEdit& a, const Road::ProfileEdit& b) { return a.s < b.s; });

    ProfileTimeline tl;
    LaneSection current = base;
    tl.at(0.0, current);
    double len = r.spineLength();

    for (const Road::ProfileEdit& e : edits) {
        LaneSection next = current;
        double taper = e.taper;
        switch (e.kind) {
            case Road::ProfileEdit::Kind::Lanes:
            case Road::ProfileEdit::Kind::Auxiliary:
                next = sectionWithLanesChanged(current, e.side, e.delta);
                if (taper < 0) taper = taperLength(3.6 * std::fabs(double(e.delta)), r.designSpeed);
                break;
            case Road::ProfileEdit::Kind::TurnBay:
                next = sectionWithTurnBay(current, e.side, e.width);
                if (taper < 0) taper = clampd(e.width * 8.0, 10.0, 60.0);
                break;
            case Road::ProfileEdit::Kind::Twltl:
                next = sectionWithTwltl(current, e.width);
                if (taper < 0) taper = taperLength(e.width, r.designSpeed) * 0.5;
                break;
        }
        double start = clampd(e.s, 1.0, std::max(2.0, len - 2.0));
        tl.at(start, next, taper);
        if (e.runLength > 0) {
            double back = clampd(start + taper + e.runLength, start + taper + 1.0, len - 1.0);
            tl.at(back, current, taper);
        } else {
            current = next;
        }
    }
    r.xs = tl.build(len, r.designSpeed);
}

void applyLaneChange(Network& net, int roadId, double s, int side, int delta, double runLength) {
    Road::ProfileEdit e;
    e.kind = Road::ProfileEdit::Kind::Lanes;
    e.s = s;
    e.side = side;
    e.delta = delta;
    e.runLength = runLength;
    net.road(roadId).profileEdits.push_back(e);
    rebuildProfile(net, roadId);
}

void applyTurnBay(Network& net, int roadId, double sJunction, bool atEnd, int side, double length,
                  double width) {
    Road& r = net.road(roadId);
    Road::ProfileEdit e;
    e.kind = Road::ProfileEdit::Kind::TurnBay;
    // A bay opens over a taper and then runs to the stop line, so the edit is
    // authored from the junction backwards.
    e.taper = clampd(width * 8.0, 10.0, length * 0.55);
    e.side = side;
    e.width = width;
    e.atEnd = atEnd;
    if (atEnd) {
        e.s = clampd(sJunction - length, 1.0, r.spineLength() - 3.0);
        e.runLength = -1;
    } else {
        e.s = 1.0;
        e.runLength = clampd(sJunction + length, 2.0, r.spineLength() - 1.0);
    }
    r.profileEdits.push_back(e);
    rebuildProfile(net, roadId);
}

void applyTwltl(Network& net, int roadId) {
    Road::ProfileEdit e;
    e.kind = Road::ProfileEdit::Kind::Twltl;
    e.s = 1.0;
    e.width = 3.6;
    e.taper = 12.0;
    e.runLength = -1;
    net.road(roadId).profileEdits.push_back(e);
    rebuildProfile(net, roadId);
}

// --- ramps ----------------------------------------------------------------

namespace {

// Add an auxiliary lane on `side` running from sStart, at full width for
// `runLength`, then tapering away. Returns the lane id of the auxiliary lane
// while it is at full width, and the station where it reaches it.
int addAuxiliaryLane(Network& net, int roadId, double sStart, double runLength, int side,
                     bool taperOut, double& sFull) {
    Road& r = net.road(roadId);
    double taper = taperLength(3.6, r.designSpeed);
    sStart = clampd(sStart, 1.0, std::max(2.0, r.spineLength() - taper - 5.0));
    Road::ProfileEdit e;
    e.kind = Road::ProfileEdit::Kind::Auxiliary;
    e.s = sStart;
    e.side = side;
    e.delta = +1;
    e.taper = taper;
    e.runLength = taperOut ? runLength : -1;
    r.profileEdits.push_back(e);
    rebuildProfile(net, roadId);
    sFull = sStart + taper;
    // The auxiliary lane is the outermost lane of the widened stack.
    int sec = r.xs.sectionIndexAt(sFull + 0.5);
    const std::vector<Strip>& stack =
        side >= 0 ? r.xs.sections[size_t(sec)].left : r.xs.sections[size_t(sec)].right;
    int laneId = 0;
    for (const Strip& st : stack)
        if (st.isLane()) laneId = st.id;
    return laneId;
}

int makeRampRoad(Network& net, const RampDesc& desc, Vec2 innerPoint, double innerHeading,
                 double innerHeight, bool rampLeadsIn) {
    Road r;
    r.name = desc.name;
    r.kind = RoadKind::Ramp;
    RoadPreset preset = roadPreset(desc.preset);
    r.designSpeed = desc.designSpeed > 0 ? desc.designSpeed : preset.designSpeedKph;
    r.allowsPedestrians = false;
    if (rampLeadsIn) {
        r.spine = spineConnector(desc.outerPoint, desc.outerHeading, innerPoint, innerHeading);
        r.spine.setElevationKnots({{0.0, desc.outerHeight}, {r.spine.length(), innerHeight}});
    } else {
        r.spine = spineConnector(innerPoint, innerHeading, desc.outerPoint, desc.outerHeading);
        r.spine.setElevationKnots({{0.0, innerHeight}, {r.spine.length(), desc.outerHeight}});
    }
    r.spine.setCrossfall(0.02);
    r.spine.applySuperelevation(r.designSpeed, 0.07);
    r.spine.finalize();
    // A ramp's design speed is a property of the curve it turned out to be, not
    // a number typed next to it. Deriving it here means the advisory plaque, the
    // superelevation and the simulator's speed all come from one place.
    double kmax = 0;
    for (const GeomPrim& g : r.spine.prims())
        kmax = std::max(kmax, std::max(std::fabs(g.curv0), std::fabs(g.curv1)));
    if (kmax > 1e-6) {
        double vSafe = designSpeedForRadius(1.0 / kmax);
        r.designSpeed = clampd(std::min(r.designSpeed, vSafe), 25.0, r.designSpeed);
        r.spine.applySuperelevation(r.designSpeed, 0.07);
        r.spine.finalize();
    }
    LaneSection sec = preset.section;
    sec.s0 = 0;
    sec.assignIds();
    r.xs.sections.push_back(sec);
    r.xs.finalize(r.spine.length());
    return net.addRoad(std::move(r));
}

}  // namespace

int buildOnRamp(Network& net, int mainRoad, double sMerge, const RampDesc& desc,
                int* mainlineTail) {
    double sFull = 0;
    int auxLane = addAuxiliaryLane(net, mainRoad, sMerge, desc.auxLength, desc.side, true, sFull);
    if (auxLane == 0) return -1;

    // Where the ramp lane becomes the mainline's lane: the point the auxiliary
    // lane first exists at full width. Splitting there lets the ramp attach to
    // the downstream mainline as a plain lane-level link.
    Vec2 mergePos;
    double mergeHdg = 0;
    {
        const Road& m = net.road(mainRoad);
        if (!m.lanePose(auxLane, sFull, mergePos, mergeHdg)) return -1;
    }
    double mergeHeight = net.road(mainRoad).surfacePoint(sFull, 0).y;

    int tail = net.splitRoad(mainRoad, sFull);
    if (mainlineTail) *mainlineTail = tail;

    int ramp = makeRampRoad(net, desc, mergePos, mergeHdg, mergeHeight, true);
    // The ramp's single lane; ramp presets put it on the right of the reference
    // line so it is lane -1.
    int rampLane = -1;
    {
        const Road& rr = net.road(ramp);
        int sec = rr.xs.sectionIndexAt(rr.end() - 1e-3);
        for (const Strip& st : rr.xs.sections[size_t(sec)].right)
            if (st.isLane()) rampLane = st.id;
    }
    ExtraLaneLink link;
    link.fromRoad = ramp;
    link.fromAtEnd = true;
    link.fromLane = rampLane;
    link.toRoad = tail >= 0 ? tail : mainRoad;
    link.toAtStart = true;
    link.toLane = auxLane;
    net.extraLinks.push_back(link);
    return ramp;
}

int buildOffRamp(Network& net, int mainRoad, double sExit, const RampDesc& desc,
                 int* mainlineTail) {
    double decel = std::max(60.0, desc.auxLength);
    double taper = taperLength(3.6, net.road(mainRoad).designSpeed);
    double sStart = std::max(2.0, sExit - decel - taper);
    double sFull = 0;
    int auxLane = addAuxiliaryLane(net, mainRoad, sStart, decel, desc.side, false, sFull);
    if (auxLane == 0) return -1;

    Vec2 divergePos;
    double divergeHdg = 0;
    {
        const Road& m = net.road(mainRoad);
        double s = clampd(sExit, sFull + 1.0, m.end() - 1.0);
        if (!m.lanePose(auxLane, s, divergePos, divergeHdg)) return -1;
        sExit = s;
    }
    double divergeHeight = net.road(mainRoad).surfacePoint(sExit, 0).y;

    // The deceleration lane exists only UPSTREAM of the exit: at the diverge it
    // becomes the ramp. Splitting copies the whole profile to the tail, so the
    // edit has to be dropped there or the mainline keeps a phantom lane for the
    // rest of its length — which is exactly what it was doing.
    size_t auxEdit = net.road(mainRoad).profileEdits.empty()
                         ? 0
                         : net.road(mainRoad).profileEdits.size() - 1;
    int tail = net.splitRoad(mainRoad, sExit);
    if (mainlineTail) *mainlineTail = tail;
    if (tail >= 0) {
        Road& t = net.road(tail);
        if (auxEdit < t.profileEdits.size())
            t.profileEdits.erase(t.profileEdits.begin() + long(auxEdit));
        rebuildProfile(net, tail);
    }

    int ramp = makeRampRoad(net, desc, divergePos, divergeHdg, divergeHeight, false);
    int rampLane = -1;
    {
        const Road& rr = net.road(ramp);
        int sec = rr.xs.sectionIndexAt(rr.begin() + 1e-3);
        for (const Strip& st : rr.xs.sections[size_t(sec)].right)
            if (st.isLane()) rampLane = st.id;
    }
    ExtraLaneLink link;
    link.fromRoad = mainRoad;
    link.fromAtEnd = true;
    link.fromLane = auxLane;
    link.toRoad = ramp;
    link.toAtStart = true;
    link.toLane = rampLane;
    net.extraLinks.push_back(link);
    return ramp;
}

// --- vertical -------------------------------------------------------------

void makeOverpass(Network& net, int roadId, double s0, double s1, double rise, double approach,
                  double pierSpacing) {
    Road& r = net.road(roadId);
    double len = r.spineLength();
    s0 = clampd(s0, 1.0, len - 2.0);
    s1 = clampd(s1, s0 + 1.0, len - 1.0);
    double a = std::max(0.0, s0 - approach);
    double b = std::min(len, s1 + approach);
    double base = r.spine.elevationConst().eval(0.0);
    // Knots either side of the deck give the vertical curve somewhere to live;
    // the Hermite fit keeps the grade continuous so the approach does not kink.
    r.spine.setElevationKnots({{0.0, base},
                               {a, base},
                               {s0, base + rise},
                               {s1, base + rise},
                               {b, base},
                               {len, base}});
    r.spine.finalize();
    r.structures.push_back(bridgeSpan(s0, s1, pierSpacing));
    if (a < s0) r.structures.push_back(embankmentSpan(a, s0));
    if (b > s1) r.structures.push_back(embankmentSpan(s1, b));
}

void makeTunnel(Network& net, int roadId, double s0, double s1, double drop, double approach) {
    Road& r = net.road(roadId);
    double len = r.spineLength();
    s0 = clampd(s0, 1.0, len - 2.0);
    s1 = clampd(s1, s0 + 1.0, len - 1.0);
    double a = std::max(0.0, s0 - approach);
    double b = std::min(len, s1 + approach);
    double base = r.spine.elevationConst().eval(0.0);
    r.spine.setElevationKnots({{0.0, base},
                               {a, base},
                               {s0, base - drop},
                               {s1, base - drop},
                               {b, base},
                               {len, base}});
    r.spine.finalize();
    r.structures.push_back(tunnelSpan(s0, s1));
    if (a < s0) r.structures.push_back(cutSpan(a, s0));
    if (b > s1) r.structures.push_back(cutSpan(s1, b));
}

void makeEmbankment(Network& net, int roadId, double s0, double s1) {
    net.road(roadId).structures.push_back(embankmentSpan(s0, s1));
}

}  // namespace roadlab
