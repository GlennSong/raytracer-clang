# E1 — Lane changes, letting in, missed turns

**Read first:** design 10, 7.3 (both-lane listing), 5.2 (sections and lane lines), 6.1 (e) modes. Brief C6.

## The goal

One mechanism for every sideways move: getting into the turn lane, passing a slow car, leaving a lane that ends,
joining from a ramp. (Pulling out and parking reuse it in E3.)

## Decisions already made

**Two reasons to change, as intentions:**

- *Must:* the route needs a different lane, or this lane ends. Urgency rises from 0 to 1 as the last chance nears:
  distance left, divided by (lanes still to cross × the distance one change takes at this speed).
- *Would like to:* the lane beside is faster. Worth it when what I gain in acceleration is more than what the car I
  would cut in front of loses, times a politeness number, plus a threshold (the plan's MOBIL). All from the rule set
  and the person. Off at middle distance.

**The gap test,** against the car ahead and the car behind in the target lane:

- after the move, I can still stop in time behind the new car ahead (the hard limit from C6);
- the car behind would not have to brake harder than a limit. That limit rises with my urgency, up to a ceiling in
  the rule set.

**The move:**

1. Indicator on. Hold it for a minimum time from the rule set before moving, unless urgency is near 1.
2. When the gap test passes, ask for the gap. Asks are granted in the settle step, per section, front to back, each
   checked against the ones already granted (two cars wanting one gap from either side: one gets it).
3. Move across over 3–4 seconds (longer when slower), along a smooth curve of sideways distance against time. The
   car's direction turns by the slope of that curve.
4. From the grant until the move ends, the car is in both lanes' lists. At the halfway point it becomes a car *of*
   the new lane (its place along is carried over as the same fraction of the lane's length).
5. Before halfway, if the gap test fails by the hard margin, ease back. After halfway, finish.
6. Indicator off.

**Lines.** No change across a line the lane data says can't be crossed from this side. No change on a connector.

**Finding a gap.** A driver with a must-change and no gap adjusts its speed toward the nearest gap in the target
lane (a top-speed answer a little under that lane's speed). It does not stop to wait, except at a lane's end.

**Letting in.** A new fact, "a car beside and ahead of me is indicating into my lane, urgently". A rule gives the
driver a larger gap to keep from the car ahead, which opens a space. How readily is a personality setting.

**Missing the turn.** At the last chance, a driver not in a lane that leads to its turn carries on, the route is
worked out again from where it is, and an event is recorded and counted. It never stops in a through lane to wait.

**A lane that ends** is the exception: its end is a stop point. A driver stopped there with its indicator on relies
on letting in.

## Traps

- **Lanes of different lengths** on a curve: carry position across by fraction, and compare cars in two lanes by
  where they are side by side, not by raw distance along.
- **Section boundaries:** a change must finish before the section ends, or not start. The lane beside may not exist
  in the next section.
- **Room claims (D5):** a change into a stretch where junction room has been granted to someone else is refused.
- **A long vehicle** changing lanes occupies both for its whole length and for longer.
- **Changing back and forth:** after a would-like-to change, none again for a time from the rule set.
- **Indicators are decisions:** they come from the intention, and stay on through the move. Never from how the road
  bends.

## Done when

1. A car needing the far-turn lane gets into it before the turn, indicating first, in 3–4 seconds per lane.
2. Two cars either side of one gap: one goes, the other waits; the same every run.
3. A change started and then cut off by a braking car eases back without touching anyone.
4. Heavy traffic with one car needing to cross three lanes: cars let it in; if it still can't, it misses the turn,
   re-routes and arrives. It never stops in a through lane.
5. A lane that ends, heavy traffic beside: the queue at the lane's end empties by being let in; no deadlock.
6. Ten minutes on the lab's freeway and downtown: no overlaps; every change had its indicator on first; gaps
   accepted within the rule set's range; the measurements table rows for lane changes pass.

## Show Glenn

The lab's boulevard: cars sorting into turn lanes a block before the junction, with indicators, and one car missing
its turn and going round the block. The driver card on it reads "missed the turn at … ; re-routed".
