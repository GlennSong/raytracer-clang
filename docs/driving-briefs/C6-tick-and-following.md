# C6 — The three-step tick, speed and following

**Read first:** design 7 and 8, and 4 (answers).

## The goal

Cars move along paths, speed up, follow each other and stop at stop points, and can never run into the car ahead.
The same start gives exactly the same result every time. One thread only; C12 adds threads.

## Decisions already made

**Deciding and moving are separate.**

- *Deciding* (10 times a second near, staggered: a car decides on the ticks where `car id mod N` matches) runs the
  facts and rules and leaves a set of **held answers**: stop points as positions on paths, top speeds, intentions.
- *Moving* (every tick) works out acceleration from the held answers **and a fresh look at the car ahead**. Finding
  the car ahead is cheap (the next entry in the path's list), so it is never held.

**The formulas** are in design 8. Fixed choices:

- Each stop point is treated as a stopped car at that place. Acceleration is the lowest over the car ahead and all
  stop points.
- Update order: new speed = old speed + acceleration × step, then capped by the hard "could stop in time" limit,
  never below zero. New position = old position + **new** speed × step.
- The reaction time in the hard limit is the car's decide interval, or the step if that is longer.
- Comfortable braking may be exceeded (up to the class's hardest braking) only when the hard limit demands it.

**Two copies of the state.** Every car reads the state as it was at the start of the tick and writes the new one.
After moving, the copies swap. No car ever reads a value another car wrote in the same tick.

**Running off the end of a path:** carry the leftover distance onto the next path of the route, as many times as
needed in one tick. Update the tail records (design 7.3) for every path the body still covers.

**Each path's list** is put back in order after moving. Cars rarely pass each other, so an insertion sort per path.

**Randomness** comes only from a hash of (person id, tick, a number naming the question), using `src/hash.h`.

**Events** for the host are sorted by car id before they are handed over.

## Traps

- **Division by a tiny gap.** Put a floor on the gap in the formula, and treat a gap at or below zero (a car placed
  too close) as "brake as hard as allowed", not as infinity or NaN.
- **A top speed of zero** (a held car): don't divide by it.
- **Creeping.** A car stopped behind another at the minimum gap must stay at rest, not shuffle forward each tick.
  A car at rest starts moving only when its acceleration is positive by a small named amount.
- **Overshooting a stop point.** The hard limit against a standing obstacle guarantees none. Test it; don't add a
  clamp that hides a wrong formula.
- **The tail.** The gap to the car ahead is to its *rear*, which may be on a different path from its front.
- **Sameness.** No unordered containers walked in a loop. No dependence on the order cars happen to be stored in:
  one test shuffles the storage and expects the same result.
- **The first tick** after loading: there are no held answers yet. Every car decides on its first tick.

## Not in this task

Junctions (every junction is an all-way stop in a later task). Lane changes. Threads.

## Done when

All as tests in the `driving` test program, on small hand-made roads:

1. One car on an empty road reaches its top speed along the curve the formula predicts.
2. A car stops at a stop point within 0.3 m short of it and never past it.
3. Twenty cars follow a leader that brakes as hard as it can to a stop. No two overlap, at steps of 1/60, 1/30, 0.1
   and 0.5 seconds and at decide intervals of 0.1 and 0.5 seconds.
4. A thousand seeded runs with a leader whose speed changes at random: the smallest gap is always above zero.
5. The same seed run twice gives the same state hash at every one of 10,000 ticks.
6. Shuffling the order cars are stored in gives the same hash.
7. A car crosses ten paths of 0.5 m each in one tick and ends in the right place, with the right tail records.
8. A queue released from a stop point empties at a steady interval per car; record the interval (it is tuned later
   to about 1.9 seconds).

## Show Glenn

Not visible until C8 draws it. Report the test results and the interval from test 8.
