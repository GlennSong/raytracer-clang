-- A paper trimmer's blade lifted and brought down (#191; a prefab's script, prefabs.lua).
-- What each of the model's moving parts does is the list below (model_moves.lua).
return require("model_moves").script{
  { node = "arm", from = 0, to = 60, speed = 90, wait = 1.5, wait_back = 3, delay = 1 },
}
