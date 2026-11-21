-- A take-a-number stand feeding out tickets, one after another (#191; a prefab's script, prefabs.lua).
-- What each of the model's moving parts does is the list below (model_moves.lua).
return require("model_moves").script{
  { node = "ticket", loop = 0.14, speed = 0.05 },
}
