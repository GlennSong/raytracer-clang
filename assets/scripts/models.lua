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
}
