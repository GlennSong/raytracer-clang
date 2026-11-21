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
  -- furniture and street pieces with moving parts (models.lua, model_kit.lua). Each has a still prefab, solid where
  -- its model's shapes say, and an `_in_use` one whose script moves it as people using it would (model_moves.lua),
  -- until prefabs carry interactions and people do. The clock always runs, and a turning barber's pole is the pole.
  office_chair_adjustable = { model = "office_chair_adjustable", physics = { body = "static" } },
  office_chair_adjustable_in_use = { model = "office_chair_adjustable", physics = { body = "static" }, script = "swivel_chair_fidget.lua" },
  standing_desk = { model = "standing_desk", physics = { body = "static" } },
  standing_desk_in_use = { model = "standing_desk", physics = { body = "static" }, script = "standing_desk_lift.lua" },
  filing_cabinet = { model = "filing_cabinet", physics = { body = "static" } },
  filing_cabinet_in_use = { model = "filing_cabinet", physics = { body = "static" }, script = "drawers_open.lua" },
  desk_pedestal = { model = "desk_pedestal", physics = { body = "static" } },
  desk_pedestal_in_use = { model = "desk_pedestal", physics = { body = "static" }, script = "drawers_open.lua" },
  monitor_arm = { model = "monitor_arm", physics = { body = "static" } },
  monitor_arm_in_use = { model = "monitor_arm", physics = { body = "static" }, script = "monitor_arm_adjust.lua" },
  desk_lamp = { model = "desk_lamp", physics = { body = "static" } },
  desk_lamp_in_use = { model = "desk_lamp", physics = { body = "static" }, script = "desk_lamp_nod.lua" },
  whiteboard = { model = "whiteboard", physics = { body = "static" } },
  whiteboard_in_use = { model = "whiteboard", physics = { body = "static" }, script = "whiteboard_flip.lua" },
  desk_fan = { model = "desk_fan", physics = { body = "static" } },
  desk_fan_in_use = { model = "desk_fan", physics = { body = "static" }, script = "fan_run.lua" },
  stand_fan = { model = "stand_fan", physics = { body = "static" } },
  stand_fan_in_use = { model = "stand_fan", physics = { body = "static" }, script = "fan_run.lua" },
  ceiling_fan = { model = "ceiling_fan", physics = { body = "static" } },
  ceiling_fan_in_use = { model = "ceiling_fan", physics = { body = "static" }, script = "ceiling_fan_run.lua" },
  wardrobe = { model = "wardrobe", physics = { body = "static" } },
  wardrobe_in_use = { model = "wardrobe", physics = { body = "static" }, script = "doors_open.lua" },
  kitchen_cupboard = { model = "kitchen_cupboard", physics = { body = "static" } },
  kitchen_cupboard_in_use = { model = "kitchen_cupboard", physics = { body = "static" }, script = "doors_open.lua" },
  fridge = { model = "fridge", physics = { body = "static" } },
  fridge_in_use = { model = "fridge", physics = { body = "static" }, script = "fridge_open.lua" },
  rocking_chair = { model = "rocking_chair", physics = { body = "static" } },
  rocking_chair_in_use = { model = "rocking_chair", physics = { body = "static" }, script = "rocking_chair_rock.lua" },
  recliner = { model = "recliner", physics = { body = "static" } },
  recliner_in_use = { model = "recliner", physics = { body = "static" }, script = "recliner_recline.lua" },
  washing_machine = { model = "washing_machine", physics = { body = "static" } },
  washing_machine_in_use = { model = "washing_machine", physics = { body = "static" }, script = "washer_run.lua" },
  dryer = { model = "dryer", physics = { body = "static" } },
  dryer_in_use = { model = "dryer", physics = { body = "static" }, script = "washer_run.lua" },
  parking_barrier = { model = "parking_barrier", physics = { body = "static" }, draw = { class = "structure" } },
  parking_barrier_in_use = { model = "parking_barrier", physics = { body = "static" }, draw = { class = "structure" }, script = "parking_barrier_lift.lua" },
  rising_bollard = { model = "rising_bollard", physics = { body = "static" } },
  rising_bollard_in_use = { model = "rising_bollard", physics = { body = "static" }, script = "rising_bollard_lower.lua" },
  turnstile = { model = "turnstile", physics = { body = "static" } },
  turnstile_in_use = { model = "turnstile", physics = { body = "static" }, script = "turnstile_turn.lua" },
  litter_bin = { model = "litter_bin", physics = { body = "static" } },
  litter_bin_in_use = { model = "litter_bin", physics = { body = "static" }, script = "flap_swing.lua" },
  mailbox = { model = "mailbox", physics = { body = "static" } },
  mailbox_in_use = { model = "mailbox", physics = { body = "static" }, script = "flap_swing.lua" },
  newspaper_box = { model = "newspaper_box", physics = { body = "static" } },
  newspaper_box_in_use = { model = "newspaper_box", physics = { body = "static" }, script = "flap_swing.lua" },
  patio_umbrella = { model = "patio_umbrella", physics = { body = "static" } },
  patio_umbrella_in_use = { model = "patio_umbrella", physics = { body = "static" }, script = "patio_umbrella_fold.lua" },
  awning = { model = "awning", physics = { body = "static" }, draw = { class = "structure" } },
  awning_in_use = { model = "awning", physics = { body = "static" }, draw = { class = "structure" }, script = "awning_roll.lua" },
  barber_pole = { model = "barber_pole", physics = { body = "static" }, script = "barber_pole_turn.lua" },
  revolving_door = { model = "revolving_door", physics = { body = "static" }, draw = { class = "structure" } },
  revolving_door_in_use = { model = "revolving_door", physics = { body = "static" }, draw = { class = "structure" }, script = "revolving_door_turn.lua" },
  roller_shutter = { model = "roller_shutter", physics = { body = "static" }, draw = { class = "structure" } },
  roller_shutter_in_use = { model = "roller_shutter", physics = { body = "static" }, draw = { class = "structure" }, script = "roller_shutter_run.lua" },
  lift_doors = { model = "lift_doors", physics = { body = "static" }, draw = { class = "structure" } },
  lift_doors_in_use = { model = "lift_doors", physics = { body = "static" }, draw = { class = "structure" }, script = "lift_doors_open.lua" },
  wall_clock = { model = "wall_clock", physics = { body = "static" }, script = "wall_clock_run.lua" },
  -- more of the same in other sizes and finishes, and the park pieces that stand still
  office_chair_adjustable_navy = { model = "office_chair_adjustable_navy", physics = { body = "static" } },
  standing_desk_wide = { model = "standing_desk_wide", physics = { body = "static" } },
  kitchen_wall_cupboard = { model = "kitchen_wall_cupboard", physics = { body = "static" } },
  store_cupboard = { model = "store_cupboard", physics = { body = "static" } },
  picnic_table = { model = "picnic_table", physics = { body = "static" } },
  park_bench = { model = "park_bench", physics = { body = "static" } },
  drinking_fountain = { model = "drinking_fountain", physics = { body = "static" } },
  bike_rack = { model = "bike_rack", physics = { body = "static" } },
}
