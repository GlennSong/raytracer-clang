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
--                     "random"|"shop"|"lunch"|"outing"|"activity"), menu=, activity=("AtHome"|
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
--   evening     resting, tonight is this agent's night out and its evening has begun
--   eveningOver resting, its evening is over (or no night out tonight)
-- `activity` is the label the debug HUD (and tests) read while in the state.
--
-- These tables mirror the engine's built-in defaults exactly (pinned by
-- tests/test_agent_goals.cpp): the daily schedule, and wander's perpetual
-- random trips — chained by drivers, kerb-rested one tick by walkers.

agents = {}

-- The RESIDENTS' days (commuter, shopkeeper, stroller, student) are not tables here: each is built from its role's
-- parts in roles.lua (src/apps/citysim/city_roles.h, buildDayTable). A level's own agents script may still define
-- `schedule` (the working roles' day) and `stroller` to replace them outright.

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
