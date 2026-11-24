-- A library ladder rolled along its rail to another shelf and back; it is sent further than the rail is long and stops at its end (#191; a prefab's script, prefabs.lua).
-- What each of the model's moving parts does is the list below (model_moves.lua).
return require("model_moves").script{
  { node = "ladder", from = 0, to = 10, speed = 0.5, wait = 6, wait_back = 6, delay = 1 },
}
