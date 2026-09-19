-- Agent GOAL tables (ADR-0064) — the per-agent goal layer as data, authored in
-- Lua like vehicles.lua: this file defines a global `agents` table of archetype
-- goal tables the C++ reader (engine/scripting/agent_goals.cpp) turns into
-- citysim::GoalTable at LEVEL LOAD. Lua runs only at load; every per-tick
-- transition executes in C++ from the table — so a new behaviour (a shopper's
-- errand loop, an animal's graze/flee) is a script edit, not a C++ change.
--
-- A table is:
--   entry       = "StateName"          -- where every agent's day starts
--   states      = { { name=, action=("rest"|"goto"), target=("work"|"home"|
--                     "random"|"shop"|"lunch"|"outing"), activity=("AtHome"|
--                     "Commuting"|"AtWork"|"Returning"|"Shopping"|"Lunch"|
--                     "Outing"), dwell=<hours> }, ... }
--   transitions = { { from=, event=, to= }, ... }   -- first matching row wins
--
-- Actions are the small C++ vocabulary: `rest` stays put until an event
-- (`dwell` emits dwellDone after that many in-world hours); `goto` travels to
-- its target, retrying the departure while blocked. Events are what C++ emits:
--   departWork  resting, clock inside [departWork, departHome)
--   departHome  resting, clock outside that window
--   arrived     a trip completed (a DRIVER whose `to` state is a goto CHAINS
--               straight through the node — wander's Roam self-loop)
--   noRoute     a departure found no route
--   idle        resting, every tick (perpetual loops)
--   dwellDone   resting, `dwell` hours elapsed in this state
-- `activity` is the label the debug HUD (and tests) read while in the state.
--
-- These tables mirror the engine's built-in defaults exactly (pinned by
-- tests/test_agent_goals.cpp): the daily schedule, and wander's perpetual
-- random trips — chained by drivers, kerb-rested one tick by walkers.

agents = {}

agents.schedule = {
    entry = "AtHome",
    states = {
        { name = "AtHome",        action = "rest",                   activity = "AtHome" },
        { name = "CommuteToWork", action = "goto", target = "work",  activity = "Commuting" },
        { name = "AtWork",        action = "rest",                   activity = "AtWork", dwell = 4.0 },
        { name = "GoLunch",       action = "goto", target = "lunch", activity = "Lunch" },
        { name = "AtLunch",       action = "rest",                   activity = "Lunch", dwell = 0.6 },
        { name = "BackToWork",    action = "goto", target = "work",  activity = "Commuting" },
        { name = "AtWorkPM",      action = "rest",                   activity = "AtWork" },
        { name = "GoShopping",    action = "goto", target = "shop",  activity = "Shopping" },
        { name = "AtShop",        action = "rest",                   activity = "Shopping", dwell = 0.5 },
        { name = "ReturnHome",    action = "goto", target = "home",  activity = "Returning" },
    },
    transitions = {
        { from = "AtHome",        event = "departWork", to = "CommuteToWork" },
        { from = "CommuteToWork", event = "arrived",    to = "AtWork" },
        { from = "CommuteToWork", event = "noRoute",    to = "AtHome" },
        { from = "AtWork",        event = "departHome", to = "GoShopping" },
        -- Lunch, about four hours into the shift (noRoute: brought lunch).
        { from = "AtWork",        event = "dwellDone",  to = "GoLunch" },
        { from = "GoLunch",       event = "arrived",    to = "AtLunch" },
        { from = "GoLunch",       event = "noRoute",    to = "AtWorkPM" },
        { from = "AtLunch",       event = "dwellDone",  to = "BackToWork" },
        { from = "BackToWork",    event = "arrived",    to = "AtWorkPM" },
        { from = "BackToWork",    event = "noRoute",    to = "AtWorkPM" },
        { from = "AtWorkPM",      event = "departHome", to = "GoShopping" },
        { from = "GoShopping",    event = "arrived",    to = "AtShop" },
        { from = "GoShopping",    event = "noRoute",    to = "ReturnHome" },
        { from = "AtShop",        event = "dwellDone",  to = "ReturnHome" },
        { from = "ReturnHome",    event = "arrived",    to = "AtHome" },
        { from = "ReturnHome",    event = "noRoute",    to = "AtWorkPM" },
    },
}

agents.wander_driver = {
    entry = "Roam",
    states = {
        { name = "Roam", action = "goto", target = "random", activity = "Commuting" },
    },
    transitions = {
        -- Arrived onto a goto state: the car chains straight through the node.
        { from = "Roam", event = "arrived", to = "Roam" },
    },
}

agents.wander_pedestrian = {
    entry = "Roam",
    states = {
        { name = "Roam",     action = "goto", target = "random", activity = "Commuting" },
        { name = "RoamRest", action = "rest",                    activity = "AtWork" },
    },
    transitions = {
        -- A walker turns around AT the kerb: rest a tick, then idle relaunches.
        { from = "Roam",     event = "arrived", to = "RoamRest" },
        { from = "RoamRest", event = "idle",    to = "Roam" },
    },
}

-- A day OUT: the Stroller role (non-workers). From home, a chain of nearby
-- stops -- park, cafe, store, a walk round the block -- each with a pause
-- (the stop sets its own length; `dwell` is the fallback), until the outing
-- window closes. departHome is checked before dwellDone, so an outing ends
-- at the next pause after the window shuts.
agents.stroller = {
    entry = "AtHome",
    states = {
        { name = "AtHome",      action = "rest",                    activity = "AtHome" },
        { name = "Outing",      action = "goto", target = "outing", activity = "Outing" },
        { name = "OutAndAbout", action = "rest",                    activity = "Outing", dwell = 0.05 },
        { name = "ReturnHome",  action = "goto", target = "home",   activity = "Returning" },
    },
    transitions = {
        { from = "AtHome",      event = "departWork", to = "Outing" },
        { from = "Outing",      event = "arrived",    to = "OutAndAbout" },
        { from = "Outing",      event = "noRoute",    to = "ReturnHome" },
        { from = "OutAndAbout", event = "departHome", to = "ReturnHome" },
        { from = "OutAndAbout", event = "dwellDone",  to = "Outing" },
        { from = "ReturnHome",  event = "arrived",    to = "AtHome" },
        { from = "ReturnHome",  event = "noRoute",    to = "OutAndAbout" },
    },
}
