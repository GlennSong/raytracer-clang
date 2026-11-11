# B2 — A drivable path for every turn at every junction

**Read first:** design 5.1–5.3. `src/engine/procgen/city/roads/lanes/README.md`, `lane_expand.h`,
`road_graph_spec.h` (`ConnectorSpec`). ADR-0397. The memory of the rule "roads: no guessing" is in design 5.1.

## The goal

A **connector** is the path a car follows through a junction from the lane it arrives in to the lane it leaves by.
Today the builder can make them, but only one hand-made test road has any. This task gives every junction in the lab
one connector for every turn it allows, each a curve a real vehicle can drive.

## Decisions already made

**Which connectors exist is stated, not guessed.** One shared function decides, for a junction, which arriving lane
may make which turn into which leaving lane. The planner runs it and writes the result into the road graph; the
builder only gives each one a shape and checks it. The pairing rules:

| Turn | From | Into |
|---|---|---|
| Near turn (stays by the kerb) | the kerb-most lane that allows it | the kerb-most travel lane |
| Far turn (crosses oncoming traffic) | the offside-most lane that allows it | the offside-most travel lane |
| Straight on | each lane that allows it | the lane that lines up with it sideways; if the road beyond has fewer lanes, the nearest |
| Two turn lanes side by side | lane *k* | lane *k* |
| U-turn | only where the planner states one (a gap in a median) | never added by default |

Never into a parking lane, a bus lane or a bike lane unless the vehicle class may use it.

**The shape.** Straight, then a circular arc, then straight, with the arc's radius as large as fits between the two
lane ends. Not a free-form spline: its tightest point can't be controlled, and the tightest point is what matters.
The ends must meet the lanes in position (within 1 cm) and direction (within 1 degree). Height comes from the deck.

**Where it starts and ends.** It starts at the approach's stop line (which is before the crosswalk) and ends where
the leaving lane begins beyond the far crosswalk.

**Refusing.** For each vehicle class allowed on both lanes, compare the connector's tightest radius with the class's
turning circle (vehicle data, task C1; until then a table of the standard design figures with their source):

- Too tight for some classes: the connector exists, closed to those classes.
- Too tight for every class, or it leaves the paved area when swept at the widest allowed vehicle's width: **refused**.
  A refused connector is not built, the lane's arrow doesn't show it, and a line goes into a report file with the
  junction's position and the reason.

**Ramps are not connectors.** Where a ramp joins or leaves a road, the lanes run side by side and one ends. That is
a section boundary (task B1), handled by lane changing.

## Traps

- **Junctions close together.** The road builder merges junctions that sit near each other, and the old model broke
  on roads cut into 4 m pieces. A connector may be shorter than a car. Nothing here may assume a minimum length.
- **Skewed and five-way junctions** make "straight on" unclear. Classify each turn by the angle between arriving and
  leaving directions, with the thresholds in data. When two leaving roads both look "straight", the planner's stated
  list decides.
- **Driving on the left.** Work out near and far from the sign of the turn and the rule set's side. Don't write
  "left" anywhere.
- **Wide roads meeting narrow ones:** the largest arc that fits may still cut the corner of the kerb. The swept-width
  check catches it; don't loosen the check.
- **Two connectors that share a from-lane or a to-lane** must share that end exactly, or task B3 finds false gaps.

## Not in this task

Crossing points (B3). Signs and control (B4). The arrows' paint (B8), though the data they'll read is this.

## Done when

- In the lab and the four `assets/lanelab` scenes: every stated turn has exactly one connector or one line in the
  refusal report.
- Tests: ends meet in position and direction; tightest radius is no tighter than every allowed class's turning
  circle; no connector leaves the paved area; every lane can be reached and left.
- `lanes_tests` still passes.
- The road rebuild this needs is agreed with Glenn and lands with the other B tasks.

## Show Glenn

The plan picture (`lanes_tool` writes an SVG) of the lab's downtown grid with connectors drawn, coloured by turn, and
the refusal report.
