-- A bollard that sinks into the road to let a vehicle over it, and rises again (#191; a prefab's script, prefabs.lua).
-- What each of the model's moving parts does is the list below (model_moves.lua).
return require("model_moves").script{
  { node = "post", from = 0, to = -2, speed = 0.3, wait = 5, wait_back = 8, delay = 2 },
}
