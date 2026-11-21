-- A wall bed let down for the night and folded away again: the bed and its legs turn together (#191; a prefab's script, prefabs.lua).
-- What each of the model's moving parts does is the list below (model_moves.lua).
return require("model_moves").script{
  { node = "bed", from = 0, to = 90, speed = 30, travel = 3.5, wait = 12, wait_back = 6, delay = 2 },
  { node = "legs", from = 0, to = 90, speed = 30, travel = 3.5, wait = 12, wait_back = 6, delay = 2 },
}
