-- NAMED MODELS: the models the world can ask for by name, and how each is built (#191, model step 6;
-- src/engine/scripting/model_recipes.h).
--
-- A line is a model: its name, and either a recipe to run (a script here, named as `require` names one) with its
-- arguments, or model files to read (under assets/models, the full one first; AGENTS.md "Model file names").
--
--     name = { recipe = "script", args = { ... } }
--     name = { files = { "high.glb", "low.glb" }, distances = { 0, 80 } }
--
-- A model is built the first time it is asked for and kept in the model store, where everything that shows it shares
-- it. It is built again only when what it was built from has changed: its line here, its recipe, a library the
-- recipe requires, a file it reads, or another model it uses (`model.get "tire"`). A new kind of model is a new line.
return {
  -- the kit cars on the car rig (rigs.lua): the first models with joints, markers, shapes and levels
  sedan = { recipe = "car_rig_lab", args = { name = "sedan" } },
  pickup = { recipe = "car_rig_lab", args = { name = "pickup" } },
  -- a small crane on the crane rig: the first model stamped as a prefab (prefabs.lua)
  crane = { recipe = "crane" },
  -- things the toolkit already made, as models with a collision shape each (prop_models.lua)
  office_chair = { recipe = "prop_models", args = { kind = "furniture", name = "office_chair" } },
  sofa = { recipe = "prop_models", args = { kind = "furniture", name = "sofa", variant = 41 } },
  hydrant = { recipe = "prop_models", args = { kind = "street_prop", name = "hydrant" } },
  street_lamp = { recipe = "prop_models", args = { kind = "lamp", height = 6 } },
  oak = { recipe = "prop_models", args = { kind = "tree", shape = "oak", seed = 3 } },
  boulder = { recipe = "prop_models", args = { kind = "rock", seed = 5 } },
  -- a sculpture for a park, on a plinth to sit on: written in the model studio (park_sculpture.lua)
  park_sculpture = { recipe = "park_sculpture" },
  park_sculpture_steel = { recipe = "park_sculpture", args = { finish = "steel", height = 3.4, twist = -110, taper = 0.45, strings = 17 } },
  -- playground pieces (playground_kit.lua): each a model of its own; a playground is the place they are set out in
  playground_swing = { recipe = "playground_swing" },
  playground_swing_four = { recipe = "playground_swing", args = { seats = 4, height = 2.7, paint = "blue" } },
  playground_seesaw = { recipe = "playground_seesaw" },
  playground_roundabout = { recipe = "playground_roundabout" },
  playground_slide = { recipe = "playground_slide" },
  playground_slide_tall = { recipe = "playground_slide", args = { height = 2.2, paint = "orange" } },
  -- furniture and street pieces with moving parts (model_kit.lua): each a model of its own, built from the furniture
  -- kit's shape words so it matches what the interiors place
  office_chair_adjustable = { recipe = "swivel_chair" },
  office_chair_adjustable_navy = { recipe = "swivel_chair", args = { fabric = "navy", arms = "none" } },
  standing_desk = { recipe = "standing_desk" },
  standing_desk_wide = { recipe = "standing_desk", args = { width = 2.0, wood = "walnut", frame = "black" } },
  filing_cabinet = { recipe = "drawer_cabinet" },
  desk_pedestal = { recipe = "drawer_cabinet", args = { drawers = 3, width = 0.42, depth = 0.55, drawer = 0.18, finish = "white", casters = "casters" } },
  monitor_arm = { recipe = "monitor_arm" },
  desk_lamp = { recipe = "desk_lamp" },
  whiteboard = { recipe = "whiteboard" },
  desk_fan = { recipe = "fan" },
  stand_fan = { recipe = "fan", args = { kind = "stand", size = 0.22, finish = "black" } },
  ceiling_fan = { recipe = "fan", args = { kind = "ceiling", size = 0.6, blades = 4, finish = "walnut" } },
  wall_clock = { recipe = "wall_clock" },
  wardrobe = { recipe = "cupboard", args = { rail = "rail", shelves = 1 } },
  kitchen_cupboard = { recipe = "cupboard", args = { width = 0.8, height = 0.88, depth = 0.58, shelves = 1, finish = "white" } },
  kitchen_wall_cupboard = { recipe = "cupboard", args = { width = 0.8, height = 0.7, depth = 0.33, shelves = 1, plinth = 0, finish = "white" } },
  store_cupboard = { recipe = "cupboard", args = { width = 0.9, height = 1.8, depth = 0.45, shelves = 4, finish = "steel" } },
  fridge = { recipe = "fridge" },
  rocking_chair = { recipe = "rocking_chair" },
  recliner = { recipe = "recliner" },
  washing_machine = { recipe = "washer" },
  dryer = { recipe = "washer", args = { kind = "dryer", finish = "steel" } },
  parking_barrier = { recipe = "parking_barrier" },
  rising_bollard = { recipe = "rising_bollard" },
  turnstile = { recipe = "turnstile" },
  litter_bin = { recipe = "flap_box" },
  mailbox = { recipe = "flap_box", args = { kind = "mailbox", paint = "blue" } },
  newspaper_box = { recipe = "flap_box", args = { kind = "newspaper", paint = "red" } },
  patio_umbrella = { recipe = "patio_umbrella" },
  awning = { recipe = "awning" },
  barber_pole = { recipe = "barber_pole" },
  picnic_table = { recipe = "park_pieces" },
  park_bench = { recipe = "park_pieces", args = { kind = "bench" } },
  drinking_fountain = { recipe = "park_pieces", args = { kind = "drinking_fountain" } },
  bike_rack = { recipe = "park_pieces", args = { kind = "bike_rack", length = 2.4, paint = "black" } },
  revolving_door = { recipe = "revolving_door" },
  roller_shutter = { recipe = "roller_shutter" },
  lift_doors = { recipe = "lift_doors" },
}
