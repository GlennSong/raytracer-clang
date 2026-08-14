#include "builders.h"

#include "diag.h"

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
        // Measured a little way BACK along the arm, not at its endpoint.
        //
        // A roundabout dropped onto a lattice node has arms that end exactly at
        // the node, and the node is the roundabout's centre — so asking atan2
        // for the bearing of the endpoint asks it for the bearing of the zero
        // vector, and every arm answers zero. Equal bearings make every gap a
        // zero sweep, which spineArc widens to a full turn: the metro generator
        // was drawing three complete rings stacked on each other, plus arms
        // running through the middle of all three. Standing off by a couple of
        // radii asks the question the ordering actually cares about — which
        // direction does this arm come from — and is unchanged for an arm that
        // already stops clear of the centre.
        double back = std::max(desc.radius * 2.0, 10.0);
        double sBack = a.second ? std::max(r.begin(), s - back) : std::min(r.end(), s + back);
        Vec2 d = r.spine.toPlan(sBack, 0) - desc.center;
        double bearing = length(d) > 1e-3
                             ? std::atan2(d.y, d.x)
                             : wrapPi(r.spine.frameAt(s).heading + (a.second ? kPi : 0.0));
        arms.push_back({a.first, a.second, bearing});
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

// How long a mainline runs without its outer shoulder either side of a gore
// nose. Both halves use it, because both halves are the same fact seen from
// different ends: the shoulder is missing for as long as the ramp is close
// enough to be standing on it. Floored well above the shoulder's own taper
// requirement so the restoration never reads as a kink to the width lint.
double goreShoulderRun(const Road& r) {
    return clampd(taperLength(3.0, r.designSpeed), 45.0, 120.0);
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
    double len = r.spineLength();

    // The opening key is always the UNMODIFIED base. Every edit below is a
    // transition away from it and back, including the ones that land on the
    // road's own first station: the timeline covers the whole spine while a road
    // is only a window over it, so a gore at the window's start still has spine
    // to taper in over, upstream of anything anyone sees.
    //
    // Rewriting this key instead is a trap. baseSectionOf reads section 0 back as
    // the base the NEXT rebuild replays onto, and splitRoad copies the result to
    // every downstream piece — so one exit gore whose nose fell on a road
    // boundary retracted the shoulder for the rest of the ring, permanently, and
    // the earthwork with it.
    tl.at(0.0, current);

    // Where the outer shoulder is ABSENT, as merged intervals rather than as one
    // taper per gore.
    //
    // Each gore wants the shoulder gone over a hold and tapered at its far end,
    // and the taper is 120 m at motorway speed. Two interchanges 300 m apart
    // therefore ask for 350 m of shoulder work in 300 m of road, and
    // ProfileTimeline — correctly — refuses to let transitions overlap, so the
    // second one is pushed downstream until it is doing nothing where it matters
    // and the shoulder is back at full width under the next ramp.
    //
    // The answer is not a shorter taper. It is that between an exit and the
    // entrance behind it there is no shoulder to restore: that stretch is a
    // collector-distributor, and its outer edge belongs to the ramps the whole
    // way. Merging the intervals says exactly that, and leaves the taper alone.
    struct Gore {
        int side = -1;
        double s0 = 0, s1 = 0;
        bool startsAtNose = false;   // a ramp took the shoulder here, instantly
        bool endsAtNose = false;     // the auxiliary lane's own key restores it
        size_t startEdit = 0, endEdit = 0;
    };
    const double goreRun = goreShoulderRun(r);
    std::vector<Gore> gores;
    for (size_t i = 0; i < edits.size(); ++i) {
        const Road::ProfileEdit& e = edits[i];
        if (e.kind == Road::ProfileEdit::Kind::GoreShoulder)
            gores.push_back({e.side, e.s, e.s + e.goreHold, true, false, i, i});
        else if (e.kind == Road::ProfileEdit::Kind::Auxiliary && e.mergeNose)
            gores.push_back({e.side, e.s - e.goreHold, e.s, false, true, i, i});
    }
    std::sort(gores.begin(), gores.end(),
              [](const Gore& a, const Gore& b) { return a.s0 < b.s0; });
    std::vector<Gore> windows;
    for (const Gore& g : gores) {
        // Merge unless there is room to bring the shoulder back and take it away
        // again in between.
        if (!windows.empty() && windows.back().side == g.side &&
            g.s0 - windows.back().s1 < 2.0 * goreRun) {
            windows.back().s1 = std::max(windows.back().s1, g.s1);
            windows.back().endsAtNose = g.endsAtNose;
            windows.back().endEdit = g.endEdit;
            continue;
        }
        windows.push_back(g);
    }

    // Any key landing inside a window is emitted without its outer shoulder.
    auto emit = [&](double s, LaneSection sec, double transition) {
        for (const Gore& w : windows)
            if (s >= w.s0 - 1e-6 && s < w.s1 - 1e-6)
                sec = sectionWithShoulderRetracted(sec, w.side);
        tl.at(s, sec, transition);
    };

    for (size_t i = 0; i < edits.size(); ++i) {
        const Road::ProfileEdit& e = edits[i];
        // A window's own boundary keys are emitted by the edit that owns them,
        // so they see the same running section that edit does.
        for (const Gore& w : windows) {
            if (w.startEdit == i) {
                // A gore that starts at an exit nose has no run-in: the ramp
                // leaves carrying the shoulder, so it is gone from that station.
                double in = w.startsAtNose ? 1.0 : goreRun;
                double from = clampd(w.s0 - in, 1e-3, std::max(1.0, len - in - 2.0));
                tl.at(from, sectionWithShoulderRetracted(current, w.side), in);
            }
            if (w.endEdit == i && !w.endsAtNose)
                tl.at(std::min(len - 1.0, w.s1), current, goreRun);
        }
        if (e.kind == Road::ProfileEdit::Kind::GoreShoulder) continue;

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
            case Road::ProfileEdit::Kind::GoreShoulder:
                break;   // handled above
        }
        double start = clampd(e.s, 1.0, std::max(2.0, len - 2.0));
        emit(start, next, taper);
        if (e.runLength > 0) {
            double out = e.taperEnd >= 0 ? e.taperEnd : taper;
            double back = clampd(start + taper + e.runLength, start + taper + 1.0, len - 1.0);
            emit(back, current, out);
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
                     bool taperOut, double& sFull, double taperIn = -1, double taperOutLen = -1,
                     bool mergeNose = false, double goreHold = 0) {
    Road& r = net.road(roadId);
    double taper = taperIn >= 0 ? taperIn : taperLength(3.6, r.designSpeed);
    sStart = clampd(sStart, 1.0, std::max(2.0, r.spineLength() - taper - 5.0));
    auto laneCount = [&](double s) {
        int si = r.xs.sectionIndexAt(s);
        const std::vector<Strip>& stack =
            side >= 0 ? r.xs.sections[size_t(si)].left : r.xs.sections[size_t(si)].right;
        int n = 0;
        for (const Strip& st : stack)
            if (st.isLane()) ++n;
        return n;
    };
    int before = laneCount(sStart);

    Road::ProfileEdit e;
    e.kind = Road::ProfileEdit::Kind::Auxiliary;
    e.s = sStart;
    e.side = side;
    e.delta = +1;
    e.taper = taper;
    e.taperEnd = taperOutLen;
    e.mergeNose = mergeNose;
    e.goreHold = goreHold;
    e.runLength = taperOut ? runLength : -1;
    r.profileEdits.push_back(e);
    rebuildProfile(net, roadId);

    // Where the lane ACTUALLY reaches full width, read back off the profile
    // rather than assumed to be sStart + taper.
    //
    // ProfileTimeline serialises transitions that would otherwise overlap, so an
    // interchange packed behind another one gets its whole gore pushed
    // downstream — 60 m, in the metro generator, where an exit's shoulder
    // restoration ran into the next entrance's retraction. A ramp aimed at the
    // arithmetic then ends 60 m short of its own nose and 3.6 m inside the
    // outside lane, which is exactly the artefact this was. The profile is the
    // authority on what the profile did.
    sFull = -1;
    int laneId = 0;
    for (const LaneSection& sec : r.xs.sections) {
        if (sec.s0 < sStart - 1e-6) continue;
        const std::vector<Strip>& stack = side >= 0 ? sec.left : sec.right;
        int n = 0, last = 0;
        double w = 0;
        for (const Strip& st : stack)
            if (st.isLane()) {
                ++n;
                last = st.id;
                w = st.width.eval(0.0);
            }
        // The taper-in section also carries the extra lane, at zero width where
        // it starts; the one we want is where it is already there.
        if (n > before && w > 3.0) {
            sFull = sec.s0;
            laneId = last;
            break;
        }
    }
    if (sFull < 0) {
        // Fall back to the arithmetic rather than to nothing: a ramp that cannot
        // find its lane is a ramp that does not get built, which is a worse
        // failure than one built where the taper says.
        sFull = clampd(sStart + taper, 0.0, r.spineLength());
        int si = r.xs.sectionIndexAt(sFull + 0.5);
        const std::vector<Strip>& stack =
            side >= 0 ? r.xs.sections[size_t(si)].left : r.xs.sections[size_t(si)].right;
        for (const Strip& st : stack)
            if (st.isLane()) laneId = st.id;
    }
    return laneId;
}

// The mainline's cross-section from `laneId` OUTWARD, as a section in its own
// right, plus the offset of its reference line in the mainline's frame.
//
// This is what a ramp has to look like where it reaches the gore nose, and it is
// DERIVED rather than authored for the same reason a junction's arms snap to the
// pad's boundary points: cracks come from two pieces computing the same edge
// slightly differently, so only one piece is ever allowed to compute it. Here
// the mainline computes it and the ramp adopts it.
//
// Everything inboard of the lane is dropped — which is what removes the ramp's
// inner shoulder and its concrete barrier, both of which were being planted in
// the middle of the freeway.
LaneSection sectionFromLaneOutward(const Road& m, double s, int laneId, int side,
                                   double& refT) {
    int secIdx = m.xs.sectionIndexAt(s);
    const LaneSection& src = m.xs.sections[size_t(secIdx)];
    const std::vector<Strip>& stack = side >= 0 ? src.left : src.right;

    double inner = 0, outer = 0;
    if (!m.xs.laneSpanAt(secIdx, laneId, s, inner, outer)) {
        refT = 0;
        return src;
    }
    // The ramp's reference line sits on the lane's INNER edge, so its own strip
    // stack — which grows outward from t = 0 — lands exactly on the mainline's.
    refT = inner;

    LaneSection out;
    out.s0 = 0;
    bool reached = false;
    for (const Strip& st : stack) {
        if (st.id == laneId) reached = true;
        if (!reached) continue;
        // The earthwork comes too, and only because the mainline gives it up.
        //
        // A Slope runs from the outermost pavement out to natural ground. Through
        // a gore the outermost pavement is the RAMP's, and the mainline's batter
        // is retracted along with the shoulder it started from (see
        // sectionWithShoulderRetracted) precisely so it is not drawn under the
        // ramp. Something still has to carry the ground away from the edge, and
        // taking the mainline's own band means the two agree exactly where they
        // meet at the nose instead of stepping 3.5 m sideways.
        Strip copy = st;
        copy.width = Poly3::constant(st.width.eval(s - src.s0));
        (side >= 0 ? out.left : out.right).push_back(copy);
    }
    if ((side >= 0 ? out.left : out.right).empty()) {
        refT = 0;
        return src;
    }
    out.assignIds();
    return out;
}

// Which side of `main` the ramp's far end lies on. See RampDesc::side.
int rampSide(const Road& main, const RampDesc& desc) {
    if (desc.side != 0) return desc.side < 0 ? -1 : +1;
    double s = 0, t = 0;
    if (!main.spine.toST(desc.outerPoint, s, t)) return -1;
    return t >= 0 ? +1 : -1;
}

// A ramp preset, laid out for the side of the mainline it is actually on.
//
// makeRamp puts the reference line on the ramp's LEFT, because that is where a
// one-way carriageway's yellow line belongs. A ramp on the mainline's left has
// its reference line — the mainline lane's inner edge — on its RIGHT instead, so
// the whole section mirrors. Swapping the two stacks reverses the boundary
// sequence, which is exactly what a mirror does to geometry; paint is not
// symmetric under it, so the reference line's marking trades places with the
// outermost lane's. (True for a one-way carriageway only, which is all a ramp
// ever is — on a two-way section the centre line stays put under a mirror.)
LaneSection rampSectionFor(const RoadPreset& preset, int side) {
    LaneSection sec = preset.section;
    sec.s0 = 0;
    if (side < 0) {
        sec.assignIds();
        return sec;
    }
    std::swap(sec.left, sec.right);
    Strip* outermostLane = nullptr;
    for (Strip& st : sec.left)
        if (st.isLane()) outermostLane = &st;
    if (outermostLane) std::swap(sec.centerMark, outermostLane->outerMark);
    sec.assignIds();
    return sec;
}

// The stretch at the gore end of a ramp over which it is simply RUNNING
// ALONGSIDE the mainline rather than still converging on it.
//
// A biarc hits both end poses exactly, and arriving exactly is not the same as
// arriving well: tangent continuity at a single point says nothing about the
// metres before it, and the biarc's last arc is free to be as tight as it needs.
// On a short interchange that means the ramp crosses the outside lane at a
// shallow angle and straightens out on top of it — 3 m inside the freeway for
// the last 40 m, which is exactly what the metro generator was drawing after the
// nose itself had been fixed.
//
// A real entrance has a stretch where the two carriageways are simply parallel;
// the gore is the END of that stretch, not the whole manoeuvre. So the parallel
// stretch is built FIRST, backwards from the nose, as an offset of the
// mainline's own curve — the offset of a curve with curvature k at lateral t has
// curvature k/(1 - k·t), which is exact for the straight and circular mainlines
// this system actually builds — and the biarc is then only asked to reach the
// far end of it.
struct RampRunIn {
    Vec2 pos{0, 0};
    double heading = 0;
    double length = 0;      // along the RAMP
    double curvature = 0;
};

RampRunIn solveRunIn(const Road& main, double sNose, double refT, Vec2 nosePos, double noseHdg,
                     double want, Vec2 outerPoint, bool downstream) {
    RampRunIn out;
    out.pos = nosePos;
    out.heading = noseHdg;
    double kMain = main.spine.curvatureAt(sNose);
    double scale = 1.0 - kMain * refT;
    if (scale < 0.2) return out;                    // refT past the curve's own centre
    out.curvature = kMain / scale;
    double dir = downstream ? 1.0 : -1.0;
    // The parallel stretch comes out of the ramp's total run, and what is left is
    // what the biarc has to turn in. Take a fixed share rather than a fixed
    // length: a short ramp that gives up 60 m to run alongside is a short ramp
    // with a 25 m radius in it, which the design lint is right to reject.
    want = std::min(want, 0.22 * length(nosePos - outerPoint));
    // Shorten rather than give up if the biarc would be left with no room: a
    // ramp whose whole length is parallel has nowhere left to turn.
    for (double run = want; run >= 15.0; run *= 0.5) {
        double L = run * scale;
        double hEnd = noseHdg + dir * out.curvature * L;
        Vec2 pEnd;
        if (std::fabs(out.curvature) < 1e-9) {
            pEnd = {nosePos.x + dir * L * std::cos(noseHdg),
                    nosePos.y + dir * L * std::sin(noseHdg)};
        } else {
            double R = 1.0 / out.curvature;
            Vec2 c{nosePos.x - R * std::sin(noseHdg), nosePos.y + R * std::cos(noseHdg)};
            pEnd = {c.x + R * std::sin(hEnd), c.y - R * std::cos(hEnd)};
        }
        if (length(pEnd - outerPoint) < 60.0) continue;
        out.pos = pEnd;
        out.heading = hEnd;
        out.length = L;
        return out;
    }
    return out;
}

// The ramp's PLAN geometry, on its own, before anything is committed to the
// network — so it can be measured against the mainline first. See fitGoreHold.
Spine rampSpine(const RampDesc& desc, Vec2 innerPoint, double innerHeading, bool rampLeadsIn,
                const RampRunIn& in) {
    Spine sp;
    if (rampLeadsIn) {
        sp = spineConnector(desc.outerPoint, desc.outerHeading,
                            in.length > 0 ? in.pos : innerPoint,
                            in.length > 0 ? in.heading : innerHeading);
        if (in.length > 0) {
            sp.addArc(in.length, in.curvature);
            sp.finalize();
        }
    } else if (in.length > 0) {
        // Mirror image: the parallel stretch is at the START of an exit ramp, so
        // the arc is laid first and the biarc leaves from its far end.
        sp.setStart(innerPoint, innerHeading);
        sp.addArc(in.length, in.curvature);
        Spine away = spineConnector(in.pos, in.heading, desc.outerPoint, desc.outerHeading);
        for (const GeomPrim& g : away.prims()) sp.addArc(g.length, g.curv0);
        sp.finalize();
    } else {
        sp = spineConnector(innerPoint, innerHeading, desc.outerPoint, desc.outerHeading);
    }
    return sp;
}

// How far back from the nose the mainline has to go without its outer shoulder.
//
// Not a constant, and not the length of the parallel stretch either: a biarc
// whose last arc is nearly as flat as the mainline runs alongside for far longer
// than the stretch that was asked for — 160 m in the interchange demo, against a
// 63 m parallel run — and every metre of that has the mainline's shoulder under
// the ramp's lane. Two coplanar asphalt surfaces at the same height is the one
// thing a depth buffer cannot draw. So the window is MEASURED off the ramp's own
// plan geometry, which is why rampSpine exists separately from the road.
double fitGoreHold(const Road& main, const Spine& ramp, int side, double sNose, double refT,
                   double shoulderWidth, bool downstream) {
    double outward = double(side >= 0 ? 1 : -1);
    double edge = refT + outward * shoulderWidth;
    double hold = 0;
    double len = ramp.length();
    for (int k = 0; k <= 80; ++k) {
        double s = len * double(k) / 80.0;
        double os = 0, ot = 0;
        if (!main.spine.toST(ramp.toPlan(s, 0), os, ot)) continue;
        double along = downstream ? os - sNose : sNose - os;
        if (along < -10.0 || along > 500.0) continue;      // wrapped, or far away
        if (outward * (ot - edge) > 0.3) continue;         // clear of the pavement
        hold = std::max(hold, along);
    }
    return clampd(hold, 0.0, 400.0);
}

// The outermost shoulder's width on one side, which is what a gore has to
// vacate.
double outerShoulderWidth(const Road& r, double s, int side) {
    int si = r.xs.sectionIndexAt(s);
    const LaneSection& sec = r.xs.sections[size_t(si)];
    const std::vector<Strip>& stack = side >= 0 ? sec.left : sec.right;
    double w = 0;
    for (const Strip& st : stack)
        if (st.kind == StripKind::Shoulder) w = st.width.eval(s - sec.s0);
    return w;
}

// A ramp can only feed a lane that travels the way the mainline's stations run.
//
// On a two-way carriageway the far side runs the other way, so a ramp on it is
// the same construction with s reversed: the deceleration lane grows toward
// LOWER stations, the split's downstream piece is the one before the nose, and
// the lane link's ends swap. None of that is expressed here yet, and building it
// anyway produces a ramp that renders correctly and that the lane graph cannot
// route through — which is exactly what it did, silently, the first time the
// side was derived rather than assumed. Refuse instead, loudly enough for the
// fallback census and the generator's own reachability test to see it.
bool laneRunsWithStations(const Road& r, int laneId, double s) {
    int si = r.xs.sectionIndexAt(s);
    const LaneSection& sec = r.xs.sections[size_t(si)];
    const Strip* st = sec.strip(laneId);
    return st && st->dir > 0;
}

int makeRampRoad(Network& net, const RampDesc& desc, int side, Vec2 innerPoint,
                 double innerHeading, double innerHeight, bool rampLeadsIn,
                 const LaneSection* terminal = nullptr, const RampRunIn* runIn = nullptr) {
    Road r;
    r.name = desc.name;
    r.kind = RoadKind::Ramp;
    RoadPreset preset = roadPreset(desc.preset);
    r.designSpeed = desc.designSpeed > 0 ? desc.designSpeed : preset.designSpeedKph;
    r.allowsPedestrians = false;
    RampRunIn none;
    const RampRunIn& in = runIn ? *runIn : none;
    r.spine = rampSpine(desc, innerPoint, innerHeading, rampLeadsIn, in);
    if (rampLeadsIn) {
        r.spine.setElevationKnots({{0.0, desc.outerHeight}, {r.spine.length(), innerHeight}});
    } else {
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
    LaneSection sec = rampSectionFor(preset, side);
    if (terminal) {
        // Morph into the mainline's own slice over the approach to the nose, so
        // the ramp arrives as the lane it becomes rather than as a wider road
        // parked on top of one. blendSection is the same Needleman-Wunsch
        // alignment used for road-type transitions: matched strips blend width,
        // unmatched ones taper away, and the inner shoulder and barrier are
        // unmatched by construction.
        double len = r.spine.length();
        // The blend has to be COMPLETE before the parallel stretch starts. The
        // strips it drops are the ramp's inner shoulder and barrier, which sit
        // between the ramp's reference line and the mainline's centre — still
        // tapering them away while the ramp runs alongside puts a metre and a
        // half of concrete in the outside lane.
        double parallel = std::max(0.0, in.length);
        double room = std::max(2.0, len - parallel);
        double blend = clampd(std::min(len * 0.45, room - 1.0), 20.0, 90.0);
        ProfileTimeline tl;
        if (rampLeadsIn) {
            tl.at(0.0, sec);
            tl.at(std::max(1.0, room - blend), *terminal, blend);
        } else {
            tl.at(0.0, *terminal);
            tl.at(clampd(parallel, 1.0, len - 2.0), sec, blend);
        }
        r.xs = tl.build(len, r.designSpeed);
    } else {
        r.xs.sections.push_back(sec);
        r.xs.finalize(r.spine.length());
    }
    return net.addRoad(std::move(r));
}

}  // namespace

int buildOnRamp(Network& net, int mainRoad, double sMerge, const RampDesc& desc,
                int* mainlineTail) {
    // sMerge is the GORE NOSE: where the ramp's pavement and the mainline's
    // become one surface.
    //
    // Parallel type puts the auxiliary lane at full width from the nose, because
    // that is exactly what the ramp delivers there; only the downstream end
    // tapers. Taper type has no lane to deliver, so it grows one the usual way.
    // The old code did neither — it tapered IN over 150 m and then flew the ramp
    // across the carriageway to the far end of that taper, which is why the ramp
    // ran up to 11 m inside the freeway with a concrete barrier attached.
    int side = rampSide(net.road(mainRoad), desc);
    double outTaper = taperLength(3.6, net.road(mainRoad).designSpeed);
    double inTaper = desc.entry == RampEntry::Parallel ? kGoreNoseTaper : outTaper;

    // `sMerge` is where the caller wants the merge to happen, and it has always
    // been the station the RAMP ARRIVES at plus a taper's run. Shortening the
    // taper for a parallel entrance must not silently drag the ramp's endpoint
    // 85 m upstream with it — the ramp's outer end is fixed, so that would just
    // force its curve tighter until it trips the minimum-radius lint. Put the
    // nose where the arrival used to be.
    double sNose = sMerge;

    double sFull = 0;
    double parallelRun = clampd(desc.auxLength * 0.35, 40.0, 80.0);
    // Two passes. The first lays the auxiliary lane to find out where the lane's
    // inner edge actually is; the ramp's plan geometry follows from that, and how
    // far back the mainline has to vacate its shoulder follows from the ramp. The
    // second lays it again knowing the answer. Nothing is committed to the
    // network in between — rampSpine is plan geometry, not a road — so this is
    // two evaluations of the same construction, not a build and a repair.
    int auxLane = 0;
    double hold = parallelRun;
    for (int pass = 0; pass < 2; ++pass) {
        if (pass == 1) {
            net.road(mainRoad).profileEdits.pop_back();
            // Replay without it, or the second pass reads the FIRST pass's lane
            // count as the baseline and never sees a lane appear.
            rebuildProfile(net, mainRoad);
        }
        auxLane = addAuxiliaryLane(net, mainRoad, sNose, desc.auxLength, side, true, sFull,
                                   inTaper, outTaper, desc.entry == RampEntry::Parallel, hold);
        if (auxLane == 0) return -1;
        if (!laneRunsWithStations(net.road(mainRoad), auxLane, sFull)) {
            RL_FALLBACK("on-ramp onto a lane running against the mainline's stations");
            Road& m = net.road(mainRoad);
            m.profileEdits.pop_back();
            rebuildProfile(net, mainRoad);
            return -1;
        }
        if (pass == 1) break;
        const Road& m = net.road(mainRoad);
        double refT = 0;
        LaneSection probe = sectionFromLaneOutward(m, sFull, auxLane, side, refT);
        (void)probe;
        Vec2 pos = m.planPoint(sFull, refT);
        double hdg = m.spine.frameAt(sFull).heading;
        RampRunIn guess = solveRunIn(m, sFull, refT, pos, hdg, parallelRun, desc.outerPoint,
                                     false);
        Spine plan = rampSpine(desc, pos, hdg, true, guess);
        hold = std::max(parallelRun,
                        fitGoreHold(m, plan, side, sFull, refT,
                                    outerShoulderWidth(m, std::max(0.0, sNose - 5.0), side),
                                    false));
    }

    // The pose the ramp must arrive at, taken from the auxiliary lane's own
    // geometry: its inner edge, the mainline's heading, and the SURFACE height
    // there. Not the centreline height — on a superelevated carriageway that is
    // a third of a metre out, which is a visible lip even when the plan geometry
    // is right.
    Vec2 mergePos;
    double mergeHdg = 0, mergeHeight = 0;
    LaneSection terminal;
    RampRunIn runIn;
    {
        const Road& m = net.road(mainRoad);
        double refT = 0;
        terminal = sectionFromLaneOutward(m, sFull, auxLane, side, refT);
        mergePos = m.planPoint(sFull, refT);
        mergeHdg = m.spine.frameAt(sFull).heading;
        mergeHeight = m.surfacePoint(sFull, refT).y;
        runIn = solveRunIn(m, sFull, refT, mergePos, mergeHdg,
                           parallelRun, desc.outerPoint, false);
    }
    int ramp =
        makeRampRoad(net, desc, side, mergePos, mergeHdg, mergeHeight, true, &terminal, &runIn);

    int tail = net.splitRoad(mainRoad, sFull);
    if (mainlineTail) *mainlineTail = tail;
    // The ramp's single lane. It sits on whichever side of the ramp's own
    // reference line the mainline is NOT — see rampSectionFor.
    int rampLane = 0;
    {
        const Road& rr = net.road(ramp);
        int sec = rr.xs.sectionIndexAt(rr.end() - 1e-3);
        const LaneSection& ls = rr.xs.sections[size_t(sec)];
        for (const Strip& st : side >= 0 ? ls.left : ls.right)
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
    int side = rampSide(net.road(mainRoad), desc);
    double decel = std::max(60.0, desc.auxLength);
    double taper = taperLength(3.6, net.road(mainRoad).designSpeed);
    double sStart = std::max(2.0, sExit - decel - taper);
    double sFull = 0;
    double parallelRun = clampd(decel * 0.35, 40.0, 80.0);
    int auxLane = addAuxiliaryLane(net, mainRoad, sStart, decel, side, false, sFull);
    if (auxLane == 0) return -1;
    if (!laneRunsWithStations(net.road(mainRoad), auxLane, sFull)) {
        RL_FALLBACK("off-ramp from a lane running against the mainline's stations");
        Road& m = net.road(mainRoad);
        m.profileEdits.pop_back();
        rebuildProfile(net, mainRoad);
        return -1;
    }

    // The pose the ramp leaves from, taken from the deceleration lane's own
    // geometry rather than from its centreline: the inner edge, the mainline's
    // heading, and the surface height THERE. Aiming a ramp at a lane's centre
    // buries half its width in the carriageway before it has turned at all, and
    // taking the centreline height puts a lip across the nose on any
    // superelevated curve. Same three corrections as the entrance; the exit was
    // deferred at the time and shipped in the metro generator with all three.
    Vec2 divergePos;
    double divergeHdg = 0, divergeHeight = 0;
    LaneSection terminal;
    RampRunIn runOut;
    {
        const Road& m = net.road(mainRoad);
        double s = clampd(sExit, sFull + 1.0, m.end() - 1.0);
        double refT = 0;
        terminal = sectionFromLaneOutward(m, s, auxLane, side, refT);
        divergePos = m.planPoint(s, refT);
        divergeHdg = m.spine.frameAt(s).heading;
        divergeHeight = m.surfacePoint(s, refT).y;
        runOut = solveRunIn(m, s, refT, divergePos, divergeHdg,
                            parallelRun, desc.outerPoint, true);
        sExit = s;
    }

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
        // The tail begins AT the nose, and the shoulder it would otherwise begin
        // with is the strip the ramp is still standing on. Give it back once the
        // ramp has curved clear.
        Road::ProfileEdit gore;
        gore.kind = Road::ProfileEdit::Kind::GoreShoulder;
        gore.s = sExit;
        gore.side = side;
        gore.goreHold = parallelRun;
        t.profileEdits.push_back(gore);
        rebuildProfile(net, tail);
    }

    int ramp = makeRampRoad(net, desc, side, divergePos, divergeHdg, divergeHeight, false,
                            &terminal, &runOut);
    int rampLane = 0;
    {
        const Road& rr = net.road(ramp);
        int sec = rr.xs.sectionIndexAt(rr.begin() + 1e-3);
        const LaneSection& ls = rr.xs.sections[size_t(sec)];
        for (const Strip& st : side >= 0 ? ls.left : ls.right)
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
