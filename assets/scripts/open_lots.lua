-- WHAT STANDS ON AN OPEN LOT (#315, ADR-0524): the prefabs (prefabs.lua) the lot pass places on a playground, and
-- the room each needs. The lot pass decides where; the level loader stands each one as a prefab, so a swing swings
-- and a roundabout turns. This file is read when the lots are baked: changing it re-bakes them.
--
--   prefab   the prefab that stands there, its own origin at the middle of its space
--   along    metres of the play area it needs along the street
--   deep     metres it needs back from the street
--   turn     degrees it is turned from facing the gate (its model's +z), anticlockwise from above
--
-- `furniture`: the prefab that stands wherever the lot pass places a piece of outdoor furniture, by the piece's name
-- in the furniture kit. `back` is how far the prefab's own origin is in front of its back edge (the lot pass places
-- a piece by the middle of its back). `prefab` may be a list: each place takes one of them, by where it is, so a
-- beach's loungers and towels come in their colours. Every piece a lot stands outdoors is listed here; one that is
-- not is not stood (the loader says so).
return {
  furniture = {
    bench        = { prefab = "park_bench", back = 0.36 },
    plaza_bench  = { prefab = "stone_bench", back = 0.26 },
    bleacher     = { prefab = "bleachers", back = 3.03 },
    bistro_chair = { prefab = "bistro_chair", back = 0.22 },
    bar_stool    = { prefab = "bar_stool_swivel", back = 0.21 },
    bistro_table = { prefab = "bistro_table", back = 0.35 },
    high_table   = { prefab = "high_table", back = 0.35 },
    beach_umbrella = { prefab = { "beach_umbrella", "beach_umbrella_2" }, back = 1.1 },
    sun_lounger  = { prefab = { "sun_lounger", "sun_lounger_2", "sun_lounger_3" }, back = 0.975 },
    beach_towel  = { prefab = { "beach_towel", "beach_towel_2", "beach_towel_3", "beach_towel_4" }, back = 0.875 },
  },
  playground = {
    -- the pieces a playground chooses from; it stands as many as its rectangle takes, in a shuffled order
    equipment = {
      { prefab = "playground_swing", along = 4.2, deep = 5.6 },              -- (it swings forward and back)
      { prefab = "playground_slide", along = 7.4, deep = 3.0, turn = 90 },   -- (its chute along the street)
      { prefab = "playground_climbing_frame", along = 4.8, deep = 4.8 },
      { prefab = "playground_seesaw", along = 4.6, deep = 2.4 },
      { prefab = "playground_sandpit", along = 4.6, deep = 4.6 },
      { prefab = "playground_roundabout", along = 4.0, deep = 4.0 },
    },
  },
}
