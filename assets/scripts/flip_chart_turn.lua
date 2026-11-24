-- A flip chart's top sheet turned over the back and brought forward again (#191; a prefab's script, prefabs.lua).
-- What each of the model's moving parts does is the list below (model_moves.lua).
return require("model_moves").script{
  { node = "sheet", from = 0, to = -170, speed = 120, wait = 6, wait_back = 6, delay = 1 },
}
