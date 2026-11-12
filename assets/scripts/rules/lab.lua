-- THE TRAFFIC LAB'S RULE SET: the first rules of the road the new driving model drives by (driving-model-design.md 6;
-- issues #255 and #258). Read once when a level names it (citysim.drivingRules = "rules/lab.lua"), checked, and turned
-- into flat tables; no Lua runs while cars drive.
--
-- The smallest set that makes cars drive sensibly: keep to the road's limit (and the car's own top speed, which the
-- model applies by itself), slow for bends, slow before a lower limit, stop where the lane ends, stop at the
-- destination, and GIVE WAY at junctions (issue #262): a driver that must give way stops at the junction's line until
-- the gap is big enough. Who gives way to whom is the right-of-way table below (D1); the gaps are the numbers below.
-- Every junction has a control the city planner chose by the PLACEMENT POLICY at the end of this file (issue #240):
-- traffic lights, an all-way stop, stop or give-way signs on the minor road, or none. The lights have no clock yet
-- (task D2, #261), so a junction with lights is treated as its row's `when_dark` says. No lane changes yet; waiting
-- inside a junction (D4) and room beyond it (D5) come next.
--
-- Every number says where it came from. "[~]" marks a guess to tune against the lab's measurements, not a figure
-- from a source. The California Vehicle Code (CVC) is read with near turn / far turn and kerb side in place of its
-- left and right, so each rule holds on either side of the road.

rule_set = {
  name = "lab",
  -- Traffic keeps to the left, as the engine's roads are built (the one place the word is allowed).
  drives_on = "left",
  -- WHICH SIGN GOES FIRST where two approaches carry different ones: an approach with no sign shares the first rank
  -- ("priority", the through road), then the give-way sign, then the stop sign (CVC 21802 (a), 21803 (a): the driver at
  -- a stop or a yield sign gives way to traffic on the other road).
  ranks = { "priority", "give_way", "stop" },
  -- WHAT STANDS ON THE ROAD and what it means. The names are the control plan's regulation kinds, the same words as the
  -- sign kit's `means.regulation` (assets/scripts/signs_us.lua), so the planting (#241) finds the sign for each; `sign`
  -- is the US manual's code for it (MUTCD 2009, R1-1 and R1-2).
  devices = {
    stop = { means = { stop_line = true, rank = "stop" }, sign = "R1-1", paint = { "stop_bar" } },
    give_way = { means = { rank = "give_way" }, sign = "R1-2", paint = { "give_way_teeth" } },
  },

  -- WHO GIVES WAY TO WHOM at each kind of junction control (issue #260): the questions asked, in order, until one tells
  -- two turns apart. The model works out a table from these when the level loads; the "give way" rule below acts on it.
  -- The control names are the control plan's (the placement policy below chooses them), the same words as the sign
  -- kit's `means.control` (stop, all_way_stop, give_way); `signal` and `unmarked` have no sign of their own.
  control = {
    -- TRAFFIC LIGHTS: a green goes before a red; on a green, a far turn gives way to the oncoming traffic it crosses.
    -- Source: CVC 21451 (a) (on a green, go, giving way to traffic lawfully in the junction), 21801 (a) (a left turn --
    -- the far turn here -- yields to oncoming traffic) and 21453 (a) (stop at a red). WHILE DARK: there is no light clock yet (#261), and lights stuck on one phase would
    -- hold half their approaches at a red for ever, so a junction with lights is treated as an all-way stop, as the
    -- law treats a signal that is out (CVC 21800 (d)(1): at inoperative signals, stop, then go on when it is safe) or
    -- flashing red (CVC 21457 (a)).
    signal = { settle = { "light", "movement" }, gives_way_on_green = { "far_turn" }, when_dark = "all_way_stop" },
    -- AN ALL-WAY STOP: everyone stops at the line; then whoever got there first goes; of two arriving together, the one
    -- coming from the other's kerb side; then by movement. Source: CVC 22450 (a) (stop at the limit line of a stop
    -- sign), 21800 (a), (c) (the first to arrive; at an all-way stop, of two arriving together, yield to the vehicle on
    -- the right, the kerb side where traffic keeps right), 21801 (a) (the far turn yields to oncoming traffic).
    all_way_stop = { settle = { "arrived_first", "kerb_side", "movement" }, everyone_stops = true },
    -- STOP SIGNS ON THE MINOR ROAD (a two-way stop): the major road goes first; among turns of one rank, by movement,
    -- then who arrived first. Source: CVC 21802 (a) (stop, then give way to traffic on the other road close enough to be
    -- a hazard), 21801 (a).
    stop = { settle = { "sign_rank", "movement", "arrived_first" } },
    -- GIVE-WAY SIGNS ON THE MINOR ROAD: as the stop, without the full stop. Source: CVC 21803 (a) (at a yield sign, give
    -- way to traffic on the other road close enough to be a hazard), 21801 (a).
    give_way = { settle = { "sign_rank", "movement", "arrived_first" } },
    -- NO CONTROL: who got there first; at the same moment, the one from the other's kerb side; then by movement.
    -- Source: CVC 21800 (a), (b) and 21801 (a), as above.
    unmarked = { settle = { "arrived_first", "kerb_side", "movement" } },
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

  -- THE LIMIT ON A ROAD WITH NONE POSTED, by the road's class (the lane graph's road classes). The control plan posts
  -- limits on most roads (placement.speed_limits below); these hold where it posts none (the lab's ramps).
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
    -- A full stop at the line where the control makes everyone stop (an all-way stop), and where the approach carries a
    -- stop sign (CVC 22450 (a)); then the give-way rule decides when to go.
    { name = "all-way stop",      when = { all = { "everyone_stops", { "not", "stopped_at_line" } } },
      give = { stop_at = "next_line" } },
    { name = "stop sign",         when = { all = { { "sign_rank", "is", "stop" }, { "not", "stopped_at_line" } } },
      give = { stop_at = "next_line" } },
  },

  -- THE PLACEMENT POLICY (issue #240): how the city planner chooses each junction's control and each road's posted
  -- limit (`city_plan controls GRAPH.json rules/lab.lua --write` writes them into the road graph; the lane graph builder
  -- carries them through and checks them). Rows are read top to bottom; the first whose conditions hold decides. The
  -- MAJOR road at a junction is the most important class there (class_order); among equals, the one road running
  -- straight through a T. Signs stand where the named approaches arrive ("minor": every other approach; "all": every
  -- approach). The US manual's own test for lights and all-way stops is traffic counted at the junction (MUTCD 2009,
  -- chapter 4C warrants and section 2B.07); the planner has no counts, so the road's class stands in for its traffic,
  -- and every class boundary below is a [~] judgement to tune.
  placement = {
    -- The road classes, most important first. A ramp is last: where it ends on a street, the street is the major road.
    class_order = { "freeway", "boulevard", "arterial", "collector", "local", "rural", "ramp" },
    -- The limit posted on each class. Ramps post none (in the US a ramp carries an advisory speed, not a limit), so
    -- the default above holds there.
    speed_limits = {
      ["local"]  = { 25, unit = "mph", source = "California Vehicle Code 22352(a)(2)(B): 25 mph in a business or residence district" },
      collector  = { 30, unit = "mph", source = "[~] a guess to tune: a typical posted limit on an urban collector street" },
      arterial   = { 35, unit = "mph", source = "[~] a guess to tune: a typical posted limit on an urban arterial" },
      boulevard  = { 35, unit = "mph", source = "[~] a guess to tune: as the arterials it is built like" },
      freeway    = { 65, unit = "mph", source = "California Vehicle Code 22349(a): 65 mph on a highway" },
      rural      = { 55, unit = "mph", source = "California Vehicle Code 22349(b): 55 mph on a two-lane undivided highway" },
    },
    junctions = {
      -- Two roads meeting end to end (a bend, a change of class): nothing to control.
      { name = "one road running on as another", when = { arms = { max = 2 } }, control = "unmarked" },
      -- A ramp ending on a street: the ramp's traffic stops and gives way to the street's (MUTCD 2009 section 2B.05:
      -- a STOP sign where a less important road meets a main road; a diamond interchange's ramp terminal, [~] stop
      -- rather than lights at the lab's traffic).
      { name = "a ramp ends on a street: the ramp stops", when = { ramp_end = true, has_minor = true, minor = "ramp" },
        control = "stop", signs = { minor = "stop" } },
      -- A ramp leaving a street with nothing arriving from it: the street's own traffic, turning onto the ramp across
      -- the oncoming lanes, gives way by movement; no sign.
      { name = "a ramp leaves a street", when = { ramp_end = true, has_minor = false }, control = "unmarked" },
      -- Five or more roads meeting (the midtown ring's corners, where streets meet a few metres apart and make one
      -- junction): lights, as such a junction cannot be worked by signs on its minor roads alone ([~] a judgement: the
      -- manual's warrants count traffic, and a junction of many approaches gathers it).
      { name = "five or more roads meet: lights", when = { arms = { min = 5 } }, control = "signal" },
      -- Arterials (and boulevards) crossing each other: traffic lights (MUTCD 2009 chapter 4C: the volume warrants,
      -- which two busy multi-lane roads crossing meet; [~] taken as met by the class).
      { name = "arterials cross: lights", when = { major = { "boulevard", "arterial" }, minor = { "boulevard", "arterial" }, approaches = { min = 3 } },
        control = "signal" },
      -- An arterial meeting a collector: lights too ([~] by the class, as above).
      { name = "an arterial meets a collector: lights", when = { major = { "boulevard", "arterial" }, minor = "collector", approaches = { min = 3 } },
        control = "signal" },
      -- A local street meeting a busier road: a stop sign on the local street (MUTCD 2009 section 2B.05: "intersection
      -- of a less important road with a main road").
      { name = "a local street meets a busier road: the local street stops",
        when = { major = { "boulevard", "arterial", "collector" }, minor = "local", has_minor = true },
        control = "stop", signs = { minor = "stop" } },
      -- Collectors crossing each other: an all-way stop (MUTCD 2009 section 2B.07: where the roads carry about the same
      -- traffic; [~] by the class).
      { name = "collectors cross: an all-way stop", when = { major = "collector", minor = "collector", has_minor = false, approaches = { min = 4 } },
        control = "all_way_stop", signs = { all = "stop" } },
      -- A collector ending on a collector (a T): the stem stops.
      { name = "a collector ends on a collector: it stops", when = { major = "collector", minor = "collector", has_minor = true },
        control = "stop", signs = { minor = "stop" } },
      -- Local streets crossing each other (downtown's grid): an all-way stop (MUTCD 2009 section 2B.07; [~] by the
      -- class: downtown's crossings are many and busy with people on foot).
      { name = "local streets cross: an all-way stop", when = { major = "local", minor = "local", approaches = { min = 4 } },
        control = "all_way_stop", signs = { all = "stop" } },
      -- A local street ending on another (a T): the stem gives way (MUTCD 2009 section 2B.09, YIELD sign applications;
      -- CVC 21800 (b)(2) gives the through road the right of way at a T in any case).
      { name = "a local street ends on another: it gives way", when = { major = "local", minor = "local", has_minor = true },
        control = "give_way", signs = { minor = "give_way" } },
      -- Anything else: no control (the uncontrolled rules above).
      { name = "nothing else applies: no control", control = "unmarked" },
    },
  },
}
