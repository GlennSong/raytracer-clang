-- THE CONTROL PLACEMENT POLICY, SHARED BY EVERY CITY (ADR-0422; issue #240): how the city planner chooses each
-- junction's control and each road's posted limit. Moved here from the traffic lab's rule set (rules/lab.lua), which now
-- refers to it (`placement = { policy = "rules/control_policy.lua" }`); the planner of every new city reads it (through
-- the rule set that names it: plan_scene.h planToLanesScene, island_scene.h islandLanesScene), and `city_plan controls
-- GRAPH.json rules/lab.lua` plans a road file with it. Its words (the controls, the signs) are the rule set's: the reader
-- checks every row against the rule set that names this file, so a control or a sign the rule set has no row or device
-- for is refused. Nothing here is the lab's own: no row names a place.
--
-- Rows are read top to bottom; the first whose conditions hold decides, and its name is written into the road file as
-- the junction's `why`. The MAJOR road at a junction is the most important class there (class_order); among equals, the
-- one road running straight through a T. Signs stand where the named approaches arrive ("minor": every other approach;
-- "all": every approach). The US manual's own test for lights and all-way stops is traffic counted at the junction
-- (MUTCD 2009, chapter 4C warrants and section 2B.07); the planner has no counts, so the road's class stands in for its
-- traffic, and every class boundary below is a [~] judgement to tune.
control_policy = {
  -- The road classes, most important first. A ramp is last: where it ends on a street, the street is the major road.
  class_order = { "freeway", "boulevard", "arterial", "collector", "local", "rural", "ramp" },
  -- The limit posted on each class. Ramps post none (in the US a ramp carries an advisory speed, not a limit), so the
  -- rule set's default_limits hold there.
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
    -- Five or more roads meeting (a ring's corners, where streets meet a few metres apart and make one junction):
    -- lights, as such a junction cannot be worked by signs on its minor roads alone ([~] a judgement: the manual's
    -- warrants count traffic, and a junction of many approaches gathers it).
    { name = "five or more roads meet: lights", when = { arms = { min = 5 } }, control = "signal" },
    -- Arterials (and boulevards) crossing each other: traffic lights (MUTCD 2009 chapter 4C: the volume warrants,
    -- which two busy multi-lane roads crossing meet; [~] taken as met by the class).
    { name = "arterials cross: lights", when = { major = { "boulevard", "arterial" }, minor = { "boulevard", "arterial" }, approaches = { min = 3 } },
      control = "signal" },
    -- An arterial meeting a collector: lights too ([~] by the class, as above). Two approaches are enough when the
    -- collector is one-way: a frontage road running past the T where a spoke ends on it (ADR-0413), its traffic and the
    -- arterial's both arriving there; two-way roads of three arms or more always bring three.
    { name = "an arterial meets a collector: lights", when = { major = { "boulevard", "arterial" }, minor = "collector", approaches = { min = 2 } },
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
    -- Local streets crossing each other (a downtown grid): an all-way stop (MUTCD 2009 section 2B.07; [~] by the
    -- class: such crossings are many and busy with people on foot).
    { name = "local streets cross: an all-way stop", when = { major = "local", minor = "local", approaches = { min = 4 } },
      control = "all_way_stop", signs = { all = "stop" } },
    -- A local street ending on another (a T): the stem gives way (MUTCD 2009 section 2B.09, YIELD sign applications;
    -- CVC 21800 (b)(2) gives the through road the right of way at a T in any case).
    { name = "a local street ends on another: it gives way", when = { major = "local", minor = "local", has_minor = true },
      control = "give_way", signs = { minor = "give_way" } },
    -- Anything else: no control (the uncontrolled rules above).
    { name = "nothing else applies: no control", control = "unmarked" },
  },
}
