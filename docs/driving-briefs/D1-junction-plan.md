# D1 — Who gives way to whom, worked out at load

**Read first:** design 6.1 (b), 9.1. Brief B3. `src/apps/citysim/traffic_signal.h` for how lights are grouped today
(read only; it is not changed).

## The goal

When a level loads, combine each junction's crossing table (B3), its control (from the road data) and the rule set's
`control` table into a lookup: for every pair of turns that meet, who gives way. Drivers then look the answer up;
they never work it out.

## Decisions already made

**The answer for a pair, at one signal phase, is one of:**

- *A gives way*, or *B gives way*;
- *ask while driving*, with the answer to fall back on if that ties.

**How a `settle` list is worked through,** left to right. The first question that tells the two apart decides:

| Question | Answered | How |
|---|---|---|
| `light` | at load, per phase | One has a green and the other a red: red gives way. Both green: no answer, go on. A turn on red allowed by the control gives way to everything. |
| `sign_rank` | at load | The approach's sign: none or priority road goes before give-way, which goes before stop. Equal: go on. |
| `movement` | at load | By the rule set's `movement_rank` list. Equal: go on. |
| `kerb_side` | at load | It is shape: is A's approach on B's kerb side? For approaches facing each other, neither: go on. |
| `arrived_first` | while driving | Whoever reached their line first. |

When the list reaches `arrived_first`, keep going through the rest of the list to find the fallback, and store both.
If the list runs out with no answer, the fallback is "lower path id gives way", and the loader notes it.

**Kinds of meeting:**

- *Cross* and *join*: as above.
- *Split* (two turns from one lane): nobody gives way. The car behind follows the car ahead.

**Lights.** Phases group *turns*, not whole approaches: two turns can be in one phase only if they don't meet, or
the control allows them to meet with one giving way (a far turn on a plain green gives way to oncoming traffic). The
grouping method is the existing one (colouring what conflicts, ADR-0060 and ADR-0109) applied to turns. Timing comes
from the rule set. This task makes the phases and the table for each; D2 runs the clock.

**Not here:** people and trains. Their right of way comes from the `announcers` table (design 4).

## Traps

- **Near and far.** Work out each turn's kind from its shape and the rule set's side. The existing `Move::Left`
  means the driver's right; don't carry it over.
- **Is it straight on?** At skewed and five-way junctions use the connector's own stated kind from B2. Don't work it
  out again.
- **`kerb_side` with five roads:** "on my kerb side" means the first approach found turning from mine toward the
  kerb. Test it at a five-way.
- **All-way stops** (`everyone_stops`): the table says "ask while driving" for every pair. That everyone stops first
  is a rule on the driver (D3), not part of this table.
- **Sameness:** tables are built in path id order.

## Done when

Tests compare the built table with one worked by hand for:

1. a four-way with lights, two phases;
2. the same with a far-turn arrow phase;
3. an all-way stop;
4. a side road with a stop sign meeting a main road (a T);
5. a four-way with no signs.

And: mirroring the junction's shape and flipping the rule set's side gives the same table. That one test proves the
near/far wording works and that no left or right slipped in.

The loader prints each junction's table as text, for the driver card and for checking.

## Show Glenn

The printed table for one lab junction, in words: "the far turn from the north gives way to straight-on from the
south".
