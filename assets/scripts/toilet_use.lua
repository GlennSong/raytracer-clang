-- A toilet in use: its lid is lifted, it is flushed, and the lid is put down (#191; a prefab's script, prefabs.lua).
-- What each of the model's moving parts does is the list below (model_moves.lua).
return require("model_moves").script{
  { node = "lid", from = 0, to = 98, speed = 140, travel = 1, wait = 9, wait_back = 6, delay = 1 },
  { node = "flush", from = 0, to = 35, speed = 200, travel = 0.3, wait = 0.6, wait_back = 15.8, delay = 9 },
}
