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
  -- furniture, kerb props, a tree and a rock (prop_models.lua): each solid where its model's shape says
  office_chair = { model = "office_chair", physics = { body = "static" } },
  sofa = { model = "sofa", physics = { body = "static" } },
  hydrant = { model = "hydrant", physics = { body = "static" } },
  street_lamp = { model = "street_lamp", physics = { body = "static" } },
  oak = { model = "oak", physics = { body = "static" }, draw = { class = "scenery" } },
  boulder = { model = "boulder", physics = { body = "static" }, draw = { class = "scenery" } },
  -- park furniture: a sculpture on a plinth to sit on; the second turns slowly, and is another finish and shape
  park_sculpture = { model = "park_sculpture", physics = { body = "static" } },
  park_sculpture_turning = { model = "park_sculpture_steel", physics = { body = "static" }, script = "park_sculpture_turn.lua" },
  -- playground pieces, each solid and still, and the three that move as they are when children are on them
  -- (a script moves them for now; riders will, once prefabs carry interactions)
  playground_swing = { model = "playground_swing", physics = { body = "static" } },
  playground_swing_in_use = { model = "playground_swing", physics = { body = "static" }, script = "playground_swing_sway.lua" },
  playground_swing_four = { model = "playground_swing_four", physics = { body = "static" }, script = "playground_swing_sway.lua" },
  playground_seesaw = { model = "playground_seesaw", physics = { body = "static" } },
  playground_seesaw_in_use = { model = "playground_seesaw", physics = { body = "static" }, script = "playground_seesaw_rock.lua" },
  playground_roundabout = { model = "playground_roundabout", physics = { body = "static" } },
  playground_roundabout_in_use = { model = "playground_roundabout", physics = { body = "static" }, script = "playground_roundabout_spin.lua" },
  playground_slide = { model = "playground_slide", physics = { body = "static" } },
  playground_slide_tall = { model = "playground_slide_tall", physics = { body = "static" } },
}
