-- A toy chest opened and shut (#191; a prefab's script, prefabs.lua).
-- What each of the model's moving parts does is the list below (model_moves.lua).
return require("model_moves").script{
  { node = "lid", from = 0, to = 90, speed = 90, wait = 6, wait_back = 8, delay = 1 },
}
