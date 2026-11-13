#ifndef RAYTRACER_ENGINE_PROCGEN_CITY_ROADS_LANES_LANE_GRAPH_CROSSINGS_H
#define RAYTRACER_ENGINE_PROCGEN_CITY_ROADS_LANES_LANE_GRAPH_CROSSINGS_H

// WHERE PATHS THROUGH A JUNCTION CROSS, JOIN AND SPLIT (issue #239, docs/driving-briefs/B3-crossing-points.md,
// ADR-0400). For every pair of paths whose cars could touch, the stretch of each path where they could: the table the
// junction tasks (who gives way, first in goes first, room beyond) read. Shape only: it does not say who gives way.
//
// HOW "COULD TOUCH" IS MEASURED. Both centre lines are walked in steps of kCrossingStep. Two points can touch when they
// are closer, in plan, than half the widest vehicle allowed on the one path, plus half the widest allowed on the other,
// plus kCrossingMargin -- and their heights are within the road builder's same-level figure (a bridge over a street is
// not a crossing). Vehicle widths, not lane widths: the caller passes each vehicle tag's width, as it does for the
// connectors. Every pair of samples of the two paths is tested, so the touching pairs form patches in "distance along
// the one path, distance along the other"; each patch is one meeting, and its reach along each path is that path's
// stretch, widened by one step each way (the true edge lies within a step of the last sample that touches).
//
// LONG BODIES (issue #284 item 9, ADR-0412). A body with a length lies on the chord between its bumpers (lane_graph.h
// bodyOnPlan), and through a bend that cuts inside the line, beyond half its width of the centre line, so two paths
// whose centre lines stay apart can still have bodies that touch. RECORDS PER SIZE: on each path the vehicle tags whose
// swing round its tightest bend is more than kSwingCoveredByMargin are sorted into sizes (tags within that of each other's
// reach make one), the rest are its short part. For each pair of parts, one on each path, the pair's records are the
// first pass's widened by those two bodies' meetings (below), and each concerns only those parts' tags
// (CrossingPoint::aTags / otherTags); pairs of parts whose records come out the same are recorded once, and tags that
// cover a path's every part concern every vehicle. So two vehicles give way only where their own two bodies could touch.
// A path whose tags all stay short meets another such path in the first pass's records alone, which concern everyone.
// When the caller gives the vehicles lengths, the second
// pass places the BODY of each part's size (a short part's: a point) with its front every kCrossingStep, from the path's
// start to its length past the path's end (its rear on whichever path leads in, its front on whichever it leads to:
// swept_body.h), and finds the pairs of fronts whose bodies' middle lines come within the reach above (half of each
// width -- a body is within half its width of its middle line -- and the margin). Each patch of such fronts is turned
// into a stretch of each path that every touching body is on (front past its start, rear short of its end: what the
// driving model reads): from the hindmost touching front to the foremost touching rear, or, where those do not overlap
// (a short touch by part of a long body), the place they touch (lanes/lane_graph_crossings.cpp bodyStretchOn). That
// stretch widens the part pair's copy of the first pass's record it overlaps on both paths, and is a cross of its own
// where it overlaps none. Read from the touching bodies, not from every body over a
// place: a bus standing at its line with its body on the straight lane is not on a stretch its middle only reaches once
// it is half round the turn. A connector's bodies lie back along the lane it leaves and on along the lane it joins; a
// lane's lie straight on past its own ends (a turning bus's tail is its connector's, so a car that came onto the lane
// straight never inherits the swing). Two paths one of which leads into the other (a car following another) get no
// record from it. Crosswalks are not swept so.
//
// WHICH KIND. A patch holding both paths' starts, of two paths one path leads into, is a SPLIT (from the start of both
// to where they are clear). One holding both ends, of two paths that lead into one path, is a JOIN (from where they first
// touch to the end of both). One holding the end of one path and the start of a path that follows it (directly, or
// through paths shorter than touching distance) is no meeting at all: one car following another, dropped. Any other is
// a CROSS. Two paths that cross twice make two records.
//
// WHICH PAIRS. Every connector against every path near it: the other connectors of its junction, the connectors of
// other junctions, and lanes. A pair that is not two connectors of one junction (a large merged junction or a carry-on
// bringing two junctions' paths, or a connector and a lane, within touching distance) is filed under no junction and
// counted in the report. Two lanes are never paired: lanes meeting outside junctions are the builder's own check
// (LaneGraphBuild::crossingsOutside). A CROSSWALK gives one record for each path whose body could be on it: the stretch
// of the path whose points come within the vehicle's half width and the margin of the walk's area, and the stretch of
// the walk (from the middle of its outline's first edge to the middle of the edge halfway round, the way people walk
// across) the body covers. Rail tracks are owed: nothing fills the graph's tracks yet, and a track's width is not in it.
//
// It fills g.crossings and files them (indexCrossings), replacing what was there. Same graph, same table, every run.

#include "engine/procgen/city/roads/lane_graph.h"
#include "engine/procgen/city/roads/lanes/lane_graph_build.h"   // VehicleSize, CrossingReport

#include <cstdint>
#include <vector>

namespace engine {
namespace roads::lanes {

// How far apart the samples along each centre line are (metres).
constexpr double kCrossingStep = 0.25;
// How close two bodies may come, side to side, and still count as clear of each other (metres): room for a body's sway
// in its lane and the sampling. Two widest vehicles side by side in the narrowest lanes the builder lays (3.7 m apart)
// stay clear with it; turn lanes that do not are a builder fault, reported, not hidden by a smaller margin.
constexpr double kCrossingMargin = 0.5;
// A body whose swing round a path's tightest bend (how far its middle cuts inside the line, its chord's sagitta) is no
// more than this is met by the first pass, as a point on the line, the margin covering its swing as it did before
// bodies had lengths (a car's on the lab's corners: no body ever touched outside a record there); and sizes whose reach
// differs by no more than this are swept as one (metres). Longer bodies get the second pass, size by size (ADR-0412).
// (Half the margin made three times the records on the lab, every car a size of its own on most turns, and changed
// nothing the unit scenes see.)
constexpr double kSwingCoveredByMargin = kCrossingMargin;
// A meeting covering at least this share of both paths' lengths is reported as suspicious: two paths that run within
// touching distance nearly all the way (two turn lanes too close together, or connectors laid over each other).
constexpr double kNearlyWholeShare = 0.9;
// A junction with more records than this is reported: the junction plan and every car's checks there grow with it.
// (A four-way of one lane each way with every turn has 40.)
constexpr uint32_t kManyCrossingPoints = 400;

struct CrossingOptions {
    // Each vehicle tag's width and length (its turning radius is not read here). A path is as wide as the widest vehicle
    // allowed on it (a path open to no tag with a width counts its own width) and swept at the length of the longest.
    std::vector<VehicleSize> vehicles;
    // Two points are at one level when their heights differ by no more than this (metres): the road builder's same-level
    // figure, Rules::sameLevelDz.
    double clearance = 1.0;
};

// THE CROSSING POINTS OF A GRAPH whose paths, junctions and next lists are made: fills g.crossings, each junction's
// range and each path's list, and says how it went. Its problems: meetings covering nearly the whole of both paths, a
// junction with more than kManyCrossingPoints records, and a meeting of two paths that lies outside every junction's
// outline (more than kJunctionEndReach out).
CrossingReport findCrossingPoints(LaneGraph& g, const CrossingOptions& opts);

// WHERE A FAR TURN MAY WAIT INSIDE ITS JUNCTION (issue #261; docs/driving-briefs/D4-inside-the-junction.md; design
// 5.2's Path::waitInsideAt). A driver turning across oncoming traffic on a plain green may pull forward into the
// junction and wait there for a gap. Shape decides where: its front just short of the first stretch where its path
// meets a path from an approach facing it (an arriving lane whose traffic comes head on: kOncomingDot), so its body
// stands across the cross street's paths (which have a red while it may wait) and short of every oncoming one. A far
// turn that meets nothing from a facing approach (a T, or a junction with no oncoming road) gets none, and so does one
// whose first such stretch begins at its very start. Whether waiting there is allowed, and on which lights, is the rule
// set's: the driving model checks each point against its give-way table at load and drops one that would leave a body
// in the way of something that may move with it (driving/signal_programme.h). Needs the crossing points
// (findCrossingPoints first). Returns how many connectors got a point; every other path is left with none.
uint32_t placeWaitInsidePoints(LaneGraph& g);

}  // namespace roads::lanes
}  // namespace engine

#endif
