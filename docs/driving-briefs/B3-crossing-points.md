# B3 — Where paths through a junction cross, join and split

**Read first:** design 5.2 (the `CrossingPoint` type), 4 (announcers), 9. Brief B2.

## The goal

For every junction, find every pair of paths whose cars could touch, and record *where along each path* that is.
Everything the driving model does at junctions (who gives way, first in goes first, room beyond) reads this table.
This task is shape only: it does not decide who gives way.

## Decisions already made

**Three kinds of meeting:**

| Kind | When | The stretch recorded |
|---|---|---|
| Cross | two connectors' paths pass through each other | on each path, from where the bodies could first touch to where they no longer can |
| Join | two connectors end in the same lane | from where they come within touching distance, to the end of both |
| Split | two connectors start from the same lane | from the start of both, to where they are far enough apart |

Splits matter: a car turning off still has its tail in front of the car going straight on behind it.

**How "could touch" is measured.** Walk both centre lines in steps of 0.25 m. Two points can touch when they are
closer than half the widest allowed vehicle's width on one path, plus half on the other, plus a margin. Use
*vehicle* widths from the vehicle data, not lane widths. The margin is one named number.

**More than one meeting.** Two paths can cross twice. Each is its own record.

**Height.** Two paths only meet if their decks are within a clearance height of each other at that place. A bridge
over a street is not a crossing. The clearance is the road builder's existing figure, not a new one.

**Other things that cross a lane** are recorded the same way, with a different kind of "other path":

- a crosswalk: the stretch of each lane or connector that lies in its area, widened by the vehicle width;
- a rail track at street level: likewise.

These are what announcers (design 4) write against.

**Order.** Pairs are stored once, lower path id first, and every list is in id order, so the output is the same on
every run.

## Traps

- **Two turn lanes side by side** run close together all the way round. Measured with vehicle widths they should
  *not* count as touching. If they do, the connectors are too close: report it as a builder fault at that junction.
  Don't raise the margin to hide it.
- **A join's stretch must reach the end of both paths**, or there is a gap between "giving way at the junction" and
  "following in the lane" in which nobody is looking.
- **Long shallow crossings** (a skewed junction) give stretches many metres long. That is right, and it makes the
  junction slow. Don't shorten them.
- **Paths that only touch at their shared end point** (two connectors leaving one lane for different roads) are a
  split, not a cross.

## Not in this task

Who gives way (D1). Anything at run time.

## Done when

- Counts match the textbook figures on hand-made junctions, as tests:
  - a four-way junction, one lane each way, every turn allowed: **16 crosses, 8 joins, 8 splits**;
  - a T-junction, one lane each way: **3 crosses, 3 joins, 3 splits**.
- A bridge over a street gives no crossing.
- A crosswalk gives one record for each path that passes over it.
- Mirroring a junction's shape gives the mirrored table.
- The text dump of the lab's junctions is in the ADR for two junctions checked by hand.

## Show Glenn

The plan picture of one lab junction with each stretch drawn on its paths, coloured by kind.
