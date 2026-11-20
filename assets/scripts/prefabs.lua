-- PREFABS: the things the world can be given a copy of, by name, and what each is made of (#191, step 7;
-- src/engine/procgen/prefab.h).
--
-- A line is a prefab: its name, the model it shows (a name in models.lua), and what a copy carries.
--
--     name = {
--       model = "crane",
--       physics = { body = "static", friction = 0.7 },   -- solid: the model's collision shapes become bodies
--       draw = { class = "structure", distance = 400 },      -- how far it is drawn: a draw class, or metres
--       script = "crane_swing.lua",                         -- a behaviour every copy runs (it moves the model's nodes)
--     }
--
-- A level places one with an entity: { "shape": "prefab", "prefab": "crane", "position": [x, y, z] }.
-- Every copy shares the one model. A new kind of thing is a new line. No C++ changes.
return {
  crane = { model = "crane", physics = { body = "static" }, draw = { class = "structure" } },
  -- the same crane at work: its script swings the cab, lifts the boom and runs the hook line
  crane_working = { model = "crane", physics = { body = "static" }, draw = { class = "structure" }, script = "crane_swing.lua" },
  -- the kit cars as parked, solid scenery: models with used pieces (the tires) and three levels of detail
  parked_sedan = { model = "sedan", physics = { body = "static" } },
  parked_pickup = { model = "pickup", physics = { body = "static" } },
}
