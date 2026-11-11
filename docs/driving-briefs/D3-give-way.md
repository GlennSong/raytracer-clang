# D3 — Announcements, the give-way decision, patience

**Read first:** design 4 (announcers), 9.2, 9.3. Briefs D1 and C6.

## The goal

A driver nearing a junction says when it expects to be on each crossing point. A driver who must give way reads what
the others said and decides: go now, or stop and wait. This is the part of the old model that produced standoffs, so
the choices below are tight.

## Decisions already made

**Who announces.** Every car with a junction within its look-ahead (a time, from the rule set; about 8 seconds),
not only the first in each queue. A driver judging a gap needs to see the second and third car coming.

**The estimate of "when".** One shared function, from the car's own state only:

- not required to stop: time to cover the distance at its speed, speeding up or slowing toward the speed it will
  hold through the turn;
- required to stop first (a stop sign, a red): time to stop at the line, plus time from rest to the crossing point;
- never earlier than the car ahead of it in the same lane plus a following interval.

"Clear" is when its tail has passed the end of the stretch.

**"Not going".** A car that is stopping for any reason other than this junction's traffic (no room beyond, a red
light, a hold) announces that it is not going. Cross traffic then doesn't wait for it. Without this, a car that has
right of way but can't use it blocks everyone.

**The decision,** for each crossing point on my path and each announcement I must give way to (D1):

- I may go **ahead of** them if I will be clear before they arrive, by the gap the rule set asks for;
- or **behind** them if I arrive after they are clear, by the margin;
- otherwise I must give way. The fact records to whom, and by how many seconds I fell short.

**No dithering.** Two margins from the rule set, a comfortable one and a smaller hard one. A driver *decides* to go
by the comfortable one. Having decided, it keeps going unless the hard one is broken. Once it could no longer stop
before the line at comfortable braking, it is going. Without the two margins, a driver flips between go and stop
every tenth of a second.

**Patience.** The comfortable gap shrinks the longer the driver has waited, down to a floor, both from the rule set,
scaled by the person's own patience. The floor is never below the hard margin.

**"Ask while driving"** pairs: compare when each reached its line. Equal to within a tick: the fallback from D1.

**All-way stops:** a remembered fact, "I came to rest at this line", must be true before the driver may go. Then
order is by who reached the line first, among those whose paths meet. Cars whose paths don't meet go together.

**Where to stop:** the stop line. (Waiting inside the junction is D4.)

## Traps

- **The one with right of way never checks the one giving way.** If both check each other, both wait.
- **A far-off slow car** announces a time far in the future; it must not hold anyone. The look-ahead is a time, not
  a distance.
- **A queue:** the fifth car's estimate must include the four in front, or a waiting driver sees five cars arriving
  at once and then a false gap.
- **Estimates are guesses.** The other car may brake. The hard margin plus first-in-goes-first (D4) is what keeps a
  wrong guess safe; don't try to make the estimate perfect.
- **Several junctions in view:** decide for the nearest first. A driver doesn't enter a junction it has decided to
  go through if it has already decided to stop at the next and there is no room between (D5 completes this).

## Done when

Tests with two to four scripted cars, for each control kind in D1's list:

1. The one that should give way does, and stops at the line; the other does not slow.
2. Four cars reaching an all-way stop in the same tick leave one at a time, in an order that is the same every run.
3. A waiting driver takes a gap that is big enough and refuses one that is not; over many seeded runs, every gap
   taken lies between the floor and the full gap.
4. No driver changes its mind more than once on one approach.
5. A car with right of way but no room beyond does not hold up cross traffic.
6. Ten minutes of random traffic at one junction of each kind: no loop in the wait-for graph, no two cars overlapping.

## Show Glenn

In the lab (once C8 draws cars): a side street with a stop sign meeting a busy road. Cars wait at the line, take
real gaps, and the driver card says "giving way: car from the east, 2.1 seconds off; I need 4.1".
