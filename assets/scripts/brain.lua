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
--   group   for the reader: travelling, transit, at_place, resting, body, held
--   skill   the skill it asks: "walk", "drive", or "none"
--   ends    "never": only an event ends it.
--           "skill": for a near person the skill's report ends it; for a far person the clock does.
--           "clock": the clock ends it for everyone.
--   clock   the event raised when the state's planned time is up (needed unless ends = "never")
--   where   where the person is while in it: "place" (where they last arrived or set out from), "leg_start",
--           "along_leg" (between the leg's ends, by how much of the planned time has gone), "leg_end"
--   holds   true: entering it interrupts what the person was doing. The interrupted state and how much of its time was
--           left are kept, for `resume`.
--   on      event = what it leads to: a state's name, or a verb
--
-- THE VERBS (the machine's own mechanisms):
--   next_leg        this leg is done: begin the plan's next leg, or, after the last, arrive
--   replan          drop the plan and plan again from where the person stands
--   replan_without  the same, first ruling out the way of the leg that failed, for as long as this intent lasts
--   resume          leave a state that holds and carry on with what was interrupted, from where it had got to
--   ignore          nothing happens
--   refuse          nothing happens, and it is counted as an event that should not have come
--
-- THE RULE FOR A LEG THAT FAILS is the events' `otherwise`: plan again from here, ruling out what failed.
--
-- Which state a leg of each way of travelling begins in is the way's own row (travel.lua `begins_in`).
-- States marked NOT BUILT are registered in C++ as such: nobody can enter one yet, and trying says so.

brain = {
  start = "indoors",            -- a new person
  no_plan = "standing",         -- wants to be somewhere and has no way there
  intent_event = "new_intent",  -- how a new intent arrives

  intents = {
    { name = "be_at", arrive = "indoors" },
  },

  events = {
    -- the layers above and below
    { name = "new_intent", otherwise = "ignore" },      -- acted on where a row says; else on arriving
    { name = "resume_intent", otherwise = "refuse" },   -- raised by `released`: the intent takes over again
    { name = "done", otherwise = "refuse" },            -- a state the clock ends has run its time
    -- what the skills report
    { name = "arrived", otherwise = "refuse" },         -- walking: at the leg's end
    { name = "parked", otherwise = "refuse" },          -- driving: stopped at the destination, beside the space
    { name = "no_route", otherwise = "replan_without" },
    { name = "cannot_park", otherwise = "replan_without" },
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
    { name = "walking", group = "travelling", skill = "walk", ends = "skill", clock = "arrived", where = "along_leg",
      on = { arrived = "next_leg", new_intent = "replan", knocked_down = "knocked_down", stuck = "stuck" } },
    { name = "on_small_wheels", group = "travelling", skill = "walk", ends = "skill", clock = "arrived",
      where = "along_leg",
      on = { arrived = "next_leg", new_intent = "replan", knocked_down = "knocked_down", stuck = "stuck" } },
    -- at the car: take it if it is still there, then the way's start minutes
    { name = "pulling_out", group = "travelling", skill = "drive", ends = "clock", clock = "done", where = "leg_start",
      on = { done = "driving" } },
    -- `parked` is the driving's word for "stopped at the destination, in the lane beside the space"
    { name = "driving", group = "travelling", skill = "drive", ends = "skill", clock = "parked", where = "along_leg",
      on = { parked = "parking", crashed = "crashed" } },
    -- beside the space: take it if it is still free, then the way's end minutes
    { name = "parking", group = "travelling", skill = "drive", ends = "clock", clock = "done", where = "leg_end",
      on = { done = "next_leg" } },

    -- USING TRANSIT (NOT BUILT)
    { name = "waiting", group = "transit" },
    { name = "boarding", group = "transit" },
    { name = "riding", group = "transit" },
    { name = "alighting", group = "transit" },
    { name = "hailing", group = "transit" },

    -- AT A PLACE
    { name = "indoors", group = "at_place", on = { new_intent = "replan" } },
    { name = "standing", group = "at_place", on = { new_intent = "replan" } },
    { name = "using_spot", group = "at_place" },       -- NOT BUILT: sit, lie, lean
    { name = "group_activity", group = "at_place" },   -- NOT BUILT
    { name = "working_post", group = "at_place" },     -- NOT BUILT

    -- RESTING (NOT BUILT)
    { name = "asleep", group = "resting" },

    -- BODY TROUBLE: each holds what the person was doing
    { name = "knocked_down", group = "body", holds = true, on = { got_up = "resume", possessed = "possessed" } },
    { name = "stuck", group = "body", holds = true, on = { freed = "resume", possessed = "possessed" } },
    -- after a crash the person goes on without the car
    { name = "crashed", group = "body", holds = true, on = { got_up = "replan_without", possessed = "possessed" } },

    -- HELD: the player or a director decides. The intent is kept and nothing moves the person on until released.
    { name = "possessed", group = "held", holds = true, on = { released = "released" } },
    -- let go: the intent takes over again at once, planning from where the person now stands. (It holds too, so what
    -- was interrupted is still known for the moment it lasts: that is how a far person's place is worked out.)
    { name = "released", group = "held", holds = true, on = { resume_intent = "replan" } },
  },
}
