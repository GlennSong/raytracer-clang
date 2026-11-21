-- A weight machine in use: the bar is pulled down and the weights rise, then both go back (#191; a prefab's script, prefabs.lua).
-- What each of the model's moving parts does is the list below (model_moves.lua).
return require("model_moves").script{
  { node = "bar", from = 0, to = 0.6, speed = 0.6, travel = 1.1, wait = 0.4, wait_back = 0.8 },
  { node = "stack", from = 0, to = 0.6, speed = 0.6, travel = 1.1, wait = 0.4, wait_back = 0.8 },
}
