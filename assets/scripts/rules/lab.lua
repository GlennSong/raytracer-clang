-- THE TRAFFIC LAB'S RULE SET: the first rules of the road the new driving model drives by (driving-model-design.md 6;
-- issues #255 and #258). Read once when a level names it (citysim.drivingRules = "rules/lab.lua"), checked, and turned
-- into flat tables; no Lua runs while cars drive.
--
-- The smallest set that makes cars drive sensibly on roads WITHOUT junction rules: keep to the road's limit (and the
-- car's own top speed, which the model applies by itself), slow for bends, slow before a lower limit, stop where the
-- lane ends, stop at the destination. There is no giving way, no signal and no lane change yet: where two paths cross
-- or join inside a junction, cars pass through each other. Those rules come with the junction tasks (D1-D5).
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

  numbers = {
    -- The gap kept to the car ahead when both stand still: the Intelligent Driver Model's s0.
    standstill_gap = { 2.0, unit = "m",
      source = "Treiber, Hennecke & Helbing 2000, Congested traffic states in empirical observations and microscopic simulations, table 1 (s0 = 2 m)" },
    -- How hard a driver lets the car be pushed sideways round a bend before slowing for it (speed squared over the
    -- bend's radius). About a fifth of g: a 10 m corner at 10 mph, a 100 m curve at 30 mph.
    comfortable_side_accel = { 2.0, unit = "m/s2",
      source = "[~] a guess to tune: about 0.2 g, a comfortable sideways pull for an ordinary driver" },
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
  },

  -- WHAT A DRIVER DOES with what it notices. Answers combine by fixed arithmetic (the nearest place to stop by, the
  -- lowest top speed), so the order of the rules changes nothing.
  rules = {
    { name = "speed limit",       give = { top_speed = "my_limit" } },
    { name = "lower limit ahead", give = { top_speed = "slower_ahead" } },
    { name = "bend ahead",        give = { top_speed = "corner_speed" } },
    { name = "lane ends",         when = "lane_ends_ahead", give = { stop_at = "lane_end" } },
    { name = "arrive",            give = { stop_at = "destination" } },
  },
}
