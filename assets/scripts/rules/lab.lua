-- THE TRAFFIC LAB'S RULE SET: the first rules of the road the new driving model drives by (driving-model-design.md 6;
-- issues #255 and #258). Read once when a level names it (citysim.drivingRules = "rules/lab.lua"), checked, and turned
-- into flat tables; no Lua runs while cars drive.
--
-- The smallest set that makes cars drive sensibly: keep to the road's limit (and the car's own top speed, which the
-- model applies by itself), slow for bends, slow before a lower limit, stop where the lane ends, stop at the
-- destination, and GIVE WAY at junctions (issue #262): a driver that must give way stops at the junction's line until
-- the gap is big enough. Who gives way to whom is the right-of-way table below (D1); the gaps are the numbers below.
-- There are no signals and no lane changes yet; waiting inside a junction (D4) and room beyond it (D5) come next.
--
-- Every number says where it came from. "[~]" marks a guess to tune against the lab's measurements, not a figure
-- from a source.

rule_set = {
  name = "lab",
  -- Traffic keeps to the left, as the engine's roads are built (the one place the word is allowed).
  drives_on = "left",
  -- No signs stand on the lab's roads yet, so there is nothing to rank and nothing for a sign to mean.
  ranks = {},
  devices = {},

  -- WHO GIVES WAY TO WHOM at each kind of junction control (issue #260): the questions asked, in order, until one tells
  -- two turns apart. The model works out a table from these when the level loads; the "give way" rule below acts on it.
  control = {
    -- The lab's junctions carry no planned control yet (the control plan, task B4, will put one at each), so every one
    -- is settled as an uncontrolled junction: who got there first; at the same moment, the one coming from the other's
    -- kerb side goes first; then by movement (a far turn gives way to the traffic it crosses).
    -- Source: California Vehicle Code 21800 (a), (b) (first to arrive; at the same time, yield to the vehicle on the
    -- right, which is the kerb side where traffic keeps right) and 21801 (a) (a left turn, the far turn there, yields to
    -- oncoming traffic); read with near/far and kerb side so it holds on either side of the road.
    not_planned = { settle = { "arrived_first", "kerb_side", "movement" } },
  },
  -- The kinds of turn, the one that goes first first. Straight on goes before the far turn (CVC 21801 (a)); the near
  -- turn between them and the U-turn last are [~] judgements: the near turn crosses no oncoming stream, the U-turn
  -- crosses every one.
  movement_rank = { "straight", "near_turn", "far_turn", "u_turn" },

  numbers = {
    -- The gap kept to the car ahead when both stand still: the Intelligent Driver Model's s0.
    standstill_gap = { 2.0, unit = "m",
      source = "Treiber, Hennecke & Helbing 2000, Congested traffic states in empirical observations and microscopic simulations, table 1 (s0 = 2 m)" },
    -- How hard a driver lets the car be pushed sideways round a bend before slowing for it (speed squared over the
    -- bend's radius). About a fifth of g: a 10 m corner at 10 mph, a 100 m curve at 30 mph.
    comfortable_side_accel = { 2.0, unit = "m/s2",
      source = "[~] a guess to tune: about 0.2 g, a comfortable sideways pull for an ordinary driver" },

    -- GIVING WAY AT JUNCTIONS (issue #262). A GAP is the time between one car's tail leaving the stretch where two paths
    -- touch and the other's front reaching it (traffic engineers' "post-encroachment time").
    -- How far ahead in time a driver says when it will be at a junction: who announces. A far-off slow car says nothing,
    -- so it holds nobody up. It must be longer than the slowest crossing plus the comfortable gap below (a far turn from
    -- rest takes about 5.5 s to clear, and 2.5 s more is 8): at the brief's 8 s a driver decided to go and then saw a car
    -- come into view already inside its gap, and changed its mind.
    junction_look_ahead = { 10.0, unit = "s",
      source = "[~] the driving-model brief D3's 'about 8 s' (to see the second and third car coming), raised to cover the slowest crossing and its gap" },
    -- The gap a driver waits for before it decides to go. A car from rest needs about 4 s to cross and clear a lane;
    -- with this gap after it, the oncoming car is about 6.5 s off when it starts, the critical headway the HCM gives for
    -- minor-street movements.
    gap_comfortable = { 2.5, unit = "s",
      source = "[~] derived from Highway Capacity Manual 6th ed., exhibit 20-11 (base critical headways 6.2-7.1 s for minor-street movements, less ~4 s to clear)" },
    -- Having decided to go, a driver stops again only when the gap falls below this.
    gap_hard = { 1.0, unit = "s",
      source = "[~] a post-encroachment time under about 1 s is the usual mark of a serious conflict (after Allen, Shin & Cooper 1978)" },
    -- Patience: the comfortable gap shrinks, the longer a driver has stood at the line, down to this (never below gap_hard).
    gap_floor = { 1.5, unit = "s", source = "[~] a guess to tune: between the hard gap and the comfortable one" },
    -- ... at this many seconds of gap per second waited (a usual driver reaches the floor after 20 s at the line).
    gap_shrink = { 0.05, unit = "ratio", source = "[~] a guess to tune" },
    -- A car at rest this near its line has reached it ("I came to rest at this line").
    at_line_within = { 1.0, unit = "m", source = "[~] a guess to tune: about a fifth of a car's length" },
  },

  -- THE LIMIT ON A ROAD WITH NONE POSTED, by the road's class (the lane graph's road classes). The lab's roads post no
  -- limits yet; these stand in until the control plan (task B4) plants speed-limit signs.
  default_limits = {
    ["local"]  = { 25, unit = "mph", source = "California Vehicle Code 22352(a)(2)(B): 25 mph in a business or residence district" },
    collector  = { 30, unit = "mph", source = "[~] a guess to tune: a typical posted limit on an urban collector street" },
    arterial   = { 35, unit = "mph", source = "[~] a guess to tune: a typical posted limit on an urban arterial" },
    boulevard  = { 35, unit = "mph", source = "[~] a guess to tune: as the arterials it is built like" },
    freeway    = { 65, unit = "mph", source = "California Vehicle Code 22349(a): 65 mph on a highway" },
    ramp       = { 35, unit = "mph", source = "[~] a guess to tune: a typical advisory speed on a freeway ramp; the bends slow cars further" },
    rural      = { 55, unit = "mph", source = "California Vehicle Code 22349(b): 55 mph on a two-lane undivided highway" },
  },

  -- EACH DRIVER'S OWN HABITS: a person's value is worked out from who they are (a hash of their id), the same every
  -- time, bunched round the usual value.
  personality = {
    -- How far above or below the limit this person drives.
    speed_habit = { usual = 1.0, low = 0.92, high = 1.08, unit = "ratio",
      source = "[~] a guess to tune: drivers spread a few miles an hour either side of the limit" },
    -- The seconds of travel this person keeps behind the car ahead: the Intelligent Driver Model's T.
    gap_kept = { usual = 1.5, low = 1.1, high = 2.0, unit = "s",
      source = "[~] round Treiber, Hennecke & Helbing 2000, table 1 (T = 1.6 s); the spread a guess to tune" },
    -- How slowly this person's comfortable gap shrinks while waiting at a line (gap_shrink is divided by it): a patient
    -- driver (2) waits twice as long before taking a smaller gap.
    patience = { usual = 1.0, low = 0.5, high = 2.0, unit = "ratio", source = "[~] a guess to tune" },
  },

  -- WHAT A DRIVER DOES with what it notices. Answers combine by fixed arithmetic (the nearest place to stop by, the
  -- lowest top speed), so the order of the rules changes nothing.
  rules = {
    { name = "speed limit",       give = { top_speed = "my_limit" } },
    { name = "lower limit ahead", give = { top_speed = "slower_ahead" } },
    { name = "bend ahead",        give = { top_speed = "corner_speed" } },
    { name = "lane ends",         when = "lane_ends_ahead", give = { stop_at = "lane_end" } },
    { name = "arrive",            give = { stop_at = "destination" } },
    -- Stop at the junction's line while the gap is too small (or until come to rest, where everyone stops first).
    { name = "give way",          when = "must_give_way", give = { stop_at = "give_way_line" } },
  },
}
