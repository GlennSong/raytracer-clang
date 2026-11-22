-- THE AGENT BRAIN'S STATES (docs/agent-brain-design.md section 5; ADR-0440; src/apps/citysim/brain/)
--
-- What a person can be doing at one moment, the events each state accepts, and what each event leads to. Read once at
-- load into flat tables; the machine in C++ runs them and names no state itself. What a state DOES (ask a skill to walk
-- this person, take the car, start the clock) is a small set of C++ functions registered under the state's name
-- (brain_states.cpp). Loading refuses a state here with no functions there, and functions there with no state here.
--
-- Words: an EVENT is a report delivered to one person ("arrived", "your car is gone"). A SKILL is a system that moves a
-- body (walking, driving); a state asks one and waits for its report. A LEG is one part of a trip made one way. NEAR
-- people have skills running; FAR people (most of the island) have none, and the clock stands in for the skill.
--
-- intents: what a person can want.
--   name      what it is called
--   arrive    the state they are in once the intent's place is reached
--
-- events: every report and interruption.
--   name       what it is called
--   otherwise  what it does in a state whose row does not list it: a state's name, or a verb (below). A state that
--              HOLDS answers only what its own row lists; `otherwise` never moves a held person.
--
-- states:
--   name    what it is called
--   label   what a person in it is said to be doing, on screen ("getting into the car"); left out: the name
--   group   for the reader: travelling, transit, at_place, resting, body, held
--   skill   the skill it asks: "walk", "drive", "ride", or "none"
--   ends    "never": only an event ends it.
--           "skill": for a near person the skill's report ends it; for a far person the clock does.
--           "clock": the clock ends it for everyone.
--   clock   the event raised when the state's planned time is up (needed unless ends = "never")
--   where   where the person is while in it: "place" (where they last arrived or set out from), "leg_start",
--           "along_leg" (between the leg's ends, by how much of the planned time has gone), "leg_end"
--   holds   true: entering it interrupts what the person was doing. The interrupted state and how much of its time was
--           left are kept, for `resume`.
--   seconds how long it lasts, for a state the clock ends whose time is its own and not a leg's (`no_way`)
--   on      event = what it leads to: a state's name, or a verb
--
-- THE VERBS (the machine's own mechanisms):
--   next_leg        this leg is done: begin the plan's next leg, or, after the last, arrive
--   replan          drop the plan and plan again from where the person stands
--   replan_without  the same, first ruling out the way of the leg that failed, for as long as this intent lasts
--   replan_afresh   the same, first allowing again every way that was ruled out (ADR-0443): how a person with no way
--                   there tries again
--   resume         leave a state that holds and carry on with what was interrupted, from where it had got to
--   redirect        stay in this state and in the vehicle of the leg under way, and go somewhere else in it: the plan
--                   is made again as "in this vehicle from here to the space nearest the place now wanted, then on
--                   foot", the old space is let go and the new one held, and the skill is told the new end (ADR-0442).
--                   Only for a state C++ can send elsewhere (driving). When no such plan can be made, nothing changes
--   ignore          nothing happens
--   refuse          nothing happens, and it is counted as an event that should not have come
--
-- A NEW INTENT THAT A STATE IGNORES IS NOT LOST: it is delivered again on entering each later state of the same trip.
-- So a driver told to be somewhere else while pulling out is redirected as the drive begins, and one told while parking
-- finishes parking and plans again as they step out of the car (the walking row's `new_intent = "replan"`).
--
-- THE RULE FOR A LEG THAT FAILS is the events' `otherwise`: plan again from here, ruling out what failed.
--
-- Which state a leg of each way of travelling begins in is the way's own row (travel.lua `begins_in`).
-- States marked NOT BUILT are registered in C++ as such: nobody can enter one yet, and trying says so.

brain = {
  -- a new person is where they want to be, in their intent's `arrive` state; this one only when that is not built
  start = "indoors",
  no_plan = "no_way",           -- wants to be somewhere and has no way there
  intent_event = "new_intent",  -- how a new intent arrives
  -- sent to a person when something they plan with has changed: their household's car became free or was moved, they
  -- were given or lost a board (ADR-0443)
  changed_event = "things_changed",

  intents = {
    { name = "be_at", arrive = "indoors" },
    -- at a place with no building to go into (a marker beside a street, ADR-0441): there, they stand outside
    { name = "be_outside_at", arrive = "standing" },
  },

  events = {
    -- the layers above and below
    -- acted on where a row says; else delivered again to the trip's next state, and at the latest on arriving
    { name = "new_intent", otherwise = "ignore" },
    { name = "resume_intent", otherwise = "refuse" },   -- raised by `released`: the intent takes over again
    { name = "done", otherwise = "refuse" },            -- a state the clock ends has run its time
    -- ADR-0443: a person with no way there has waited their `seconds` (`try_again`); something a person plans with
    -- has changed (`things_changed`: only a person with no way there acts on it; a trip under way carries on)
    { name = "try_again", otherwise = "refuse" },
    { name = "things_changed", otherwise = "ignore" },
    -- what the skills report
    { name = "arrived", otherwise = "refuse" },         -- walking: at the leg's end
    { name = "parked", otherwise = "refuse" },          -- driving: stopped at the destination, beside the space
    { name = "boarded", otherwise = "refuse" },         -- the bus they waited for stands at their stop: they are on it
    { name = "alighted", otherwise = "refuse" },        -- the bus stands at the leg's last stop: they are off it
    { name = "no_route", otherwise = "replan_without" },
    { name = "cannot_park", otherwise = "replan_without" },
    -- raised by `parking`: the space is taken and the world named another near the door (ADR-0441)
    { name = "space_taken", otherwise = "refuse" },
    { name = "missed_turn", otherwise = "ignore" },     -- the driving routes again by itself
    { name = "crashed", otherwise = "refuse" },
    { name = "ended_early", otherwise = "replan" },     -- something else ended the drive
    -- interruptions
    { name = "vehicle_gone", otherwise = "replan" },    -- the planner is told the car is not free, so prices without it
    { name = "missed_ride", otherwise = "replan_without" },
    { name = "blocked", otherwise = "replan_without" },
    { name = "knocked_down", otherwise = "ignore" },
    { name = "got_up", otherwise = "refuse" },
    { name = "stuck", otherwise = "ignore" },
    { name = "freed", otherwise = "refuse" },
    { name = "possessed", otherwise = "possessed" },    -- anyone can be taken over, whatever they are doing
    { name = "released", otherwise = "refuse" },
  },

  states = {
    -- TRAVELLING
    { name = "walking", label = "walking", group = "travelling", skill = "walk", ends = "skill", clock = "arrived",
      where = "along_leg",
      on = { arrived = "next_leg", new_intent = "replan", knocked_down = "knocked_down", stuck = "stuck" } },
    { name = "on_small_wheels", label = "riding small wheels", group = "travelling", skill = "walk", ends = "skill",
      clock = "arrived", where = "along_leg",
      on = { arrived = "next_leg", new_intent = "replan", knocked_down = "knocked_down", stuck = "stuck" } },
    -- at the car: take it if it is still there, then the way's start minutes. (A new intent waits for the drive to
    -- begin: it is delivered again there.)
    { name = "pulling_out", label = "getting into the car", group = "travelling", skill = "drive", ends = "clock",
      clock = "done", where = "leg_start",
      on = { done = "driving" } },
    -- `parked` is the driving's word for "stopped at the destination, in the lane beside the space". A new intent at
    -- the wheel redirects the drive. `blocked` (raised here when the car has waited its way's set_off_wait_minutes
    -- for a clear lane to leave its space into) is answered by the failed-leg rule: plan again without the car
    { name = "driving", label = "driving", group = "travelling", skill = "drive", ends = "skill", clock = "parked",
      where = "along_leg",
      on = { parked = "parking", crashed = "crashed", new_intent = "redirect" } },
    -- beside the space: take it if it is still free, then the way's end minutes. Taken, and another space near the
    -- door is free: drive on to that one (the leg now ends there). Taken, and none: `cannot_park`, the failed-leg rule.
    -- (A new intent waits until they are out of the car: it is delivered again to the walk that follows.)
    { name = "parking", label = "parking and getting out", group = "travelling", skill = "drive", ends = "clock",
      clock = "done", where = "leg_end",
      on = { done = "next_leg", space_taken = "driving" } },

    -- USING TRANSIT (ADR-0484): a bus leg begins waiting at the stop the walk before it ended at (travel.lua `bus`:
    -- its start minutes are the mean wait), boards when a bus of the line stands there, and rides to the leg's last
    -- stop. A new intent while waiting plans again from the stop; one while riding waits until they are off (it is
    -- delivered again to the walk that follows). Getting on and off are the skill's, inside these two states, so
    -- `boarding` and `alighting` stay NOT BUILT.
    { name = "waiting", label = "waiting at a stop", group = "transit", skill = "ride", ends = "skill", clock = "boarded",
      where = "leg_start",
      on = { boarded = "riding", new_intent = "replan" } },
    { name = "riding", label = "riding the bus", group = "transit", skill = "ride", ends = "skill", clock = "alighted",
      where = "along_leg",
      on = { alighted = "next_leg" } },
    { name = "boarding", label = "boarding", group = "transit" },     -- NOT BUILT (inside `waiting`)
    { name = "alighting", label = "getting off", group = "transit" },   -- NOT BUILT (inside `riding`)
    { name = "hailing", label = "hailing a cab", group = "transit" },   -- NOT BUILT

    -- AT A PLACE
    { name = "indoors", label = "indoors", group = "at_place", on = { new_intent = "replan" } },
    { name = "standing", label = "standing outside", group = "at_place", on = { new_intent = "replan" } },
    -- WANTS TO BE SOMEWHERE AND HAS NO WAY THERE (ADR-0443; before, they stood for ever). They stand where they are
    -- and try again every `seconds` (an ESTIMATE: a minute, long enough for a car to come back or a lane to clear),
    -- and at once when something they plan with changes. Trying again allows every way that had been ruled out: what
    -- failed a minute ago may work now. Why there was no way is kept and shown (`brain?`: "no way there (...)")
    { name = "no_way", label = "no way there, waiting to try again", group = "at_place", ends = "clock",
      clock = "try_again", seconds = 60,
      on = { try_again = "replan_afresh", things_changed = "replan_afresh", new_intent = "replan" } },
    { name = "using_spot", label = "using a spot", group = "at_place" },       -- NOT BUILT: sit, lie, lean
    { name = "group_activity", label = "in a group activity", group = "at_place" },   -- NOT BUILT
    { name = "working_post", label = "working a post", group = "at_place" },     -- NOT BUILT

    -- RESTING (NOT BUILT)
    { name = "asleep", label = "asleep", group = "resting" },

    -- BODY TROUBLE: each holds what the person was doing
    { name = "knocked_down", label = "knocked down", group = "body", holds = true,
      on = { got_up = "resume", possessed = "possessed" } },
    { name = "stuck", label = "stuck", group = "body", holds = true, on = { freed = "resume", possessed = "possessed" } },
    -- after a crash the person goes on without the car
    { name = "crashed", label = "in a crash", group = "body", holds = true,
      on = { got_up = "replan_without", possessed = "possessed" } },

    -- HELD: the player or a director decides. The intent is kept and nothing moves the person on until released.
    { name = "possessed", label = "held by the player", group = "held", holds = true, on = { released = "released" } },
    -- let go: the intent takes over again at once, planning from where the person now stands. (It holds too, so what
    -- was interrupted is still known for the moment it lasts: that is how a far person's place is worked out.)
    { name = "released", label = "let go", group = "held", holds = true, on = { resume_intent = "replan" } },
  },
}
