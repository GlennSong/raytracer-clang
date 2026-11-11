# E3a — Paths driven backwards

**Read first:** design 5.2 (parking), 15 (edge cases: angled bays), 10. Briefs B2 and B3. Task E3 must be done.

## The goal

Cars back out of angled and square parking bays and out of driveways, along real paths. Glenn, 2026-11-11: "we
should handle reversing". Three-point turns come after and are outlined at the end.

## Decisions already made

**A reversing move is a path with a flag.** The road builder gives each bay two short paths:

- *in:* from the lane (or car-park aisle) forward into the bay;
- *out:* from the bay, **driven backwards**, curving into the lane so that the car ends up lying along the lane and
  facing the way the lane goes. Then it drives off forward.

A square bay that can be driven straight through has an ordinary forward path out instead, and no reversing.

**On a path driven backwards:**

- the car's direction is the path's direction turned round;
- "the front" for gaps and stop points is the rear bumper: distance along the path is measured to the end of the car
  that leads;
- top speed is a low figure from the rule set;
- the same speed and following formulas apply. Nothing else changes in `act`.

**Giving way.** Where the out path meets the lane is a *join* (brief B3) recorded by the builder. The rule set's
control table gets an entry for it: the car backing out gives way to everything in the lane. It announces and decides
exactly as at a junction (D3).

**Lamps and the card.** Reversing lamps on. The card says "backing out of a space".

**Being seen.** For the whole move the car's body is laid onto the lane as it sweeps across it, the same way a car
in a junction blocks crossing points (D4 rule 1), so a car coming along the lane stops for it.

**The physical tier.** A car part-way through a reversing move is not handed to physics, and a physical car parks
only after becoming kinematic again. Steering a physics body backwards along a path is not attempted in this task.

## Traps

- **The out path's shape.** A car backing out of a bay angled at 60 degrees swings its nose across the next bay. The
  builder checks the swept shape against the bays either side being occupied by the widest car allowed, and refuses
  the bay (it isn't offered for parking) if it can't be done.
- **Which way it ends up facing.** Backing out must leave the car facing the lane's direction. On a two-way aisle
  either direction is possible: the route decides, and the builder provides both out paths.
- **A queue in the lane.** With stopped traffic across the back of the bay, the car waits in the bay, by rule, and
  relies on a gap. It does not nose out. That can be a long wait; the reason is recorded, so it is not a deadlock.
- **People walking behind** are announcers or obstacles as anywhere else, but a reversing driver notices less. That
  belongs to personality (E6); for now reversing drivers notice everything.

## Not in this task

Three-point turns. Parallel parking by reversing in (kerbside parking stays a forward lane change, design 5.2).

## Done when

1. A car backs out of an angled bay along its path, ends facing the lane's direction and drives away; its direction
   never jumps and it never turns tighter than its turning circle.
2. It waits for a car coming along the lane and goes after.
3. A car coming along the lane stops for one already backing out.
4. A bay whose out path can't be made is not offered, and is in the builder's report.
5. A lab car park with 40 bays filling and emptying for ten minutes: no overlaps, no deadlock loops.

## Show Glenn

A lab car park (or a street of angled bays): cars nosing in and backing out with reversing lamps.

## Afterwards: three-point turns

At a dead end with no turnaround, the builder lays three short paths: forward toward one kerb, backwards toward the
other, forward away. The same flag and formulas. It needs a road at least as wide as the class's figure allows and
no parked cars in the swept area, so the builder decides per dead end and per class. Brief to be written when wanted.
