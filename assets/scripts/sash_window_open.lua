-- A sash window opened and shut: the lower half is sent up further than it can go, and stops at its limit (#191; a prefab's script, prefabs.lua).
-- What each of the model's moving parts does is the list below (model_moves.lua).
return require("model_moves").script{
  { node = "lower_sash", from = 0, to = 2, speed = 0.3, wait = 8, wait_back = 8, delay = 1 },
}
