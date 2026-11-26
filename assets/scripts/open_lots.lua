-- WHAT STANDS ON AN OPEN LOT (#315, ADR-0524): the prefabs (prefabs.lua) the lot pass places on a playground, and
-- the room each needs. The lot pass decides where; the level loader stands each one as a prefab, so a swing swings
-- and a roundabout turns. This file is read when the lots are baked: changing it re-bakes them.
--
--   prefab   the prefab that stands there, its own origin at the middle of its space
--   along    metres of the play area it needs along the street
--   deep     metres it needs back from the street
--   turn     degrees it is turned from facing the gate (its model's +z), anticlockwise from above
return {
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
