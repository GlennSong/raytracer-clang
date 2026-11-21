-- A casement window opened and shut: both leaves swing out (#191; a prefab's script, prefabs.lua).
-- What each of the model's moving parts does is the list below (model_moves.lua).
return require("model_moves").script{
  { node = "leaf_left", from = 0, to = 80, speed = 50, travel = 2, wait = 8, wait_back = 8, delay = 1 },
  { node = "leaf_right", from = 0, to = 80, speed = 50, travel = 2, wait = 8, wait_back = 8, delay = 2 },
}
