-- A grocer's scales weighing: the pan sinks and the needle swings round, then both go back (#191; a prefab's script, prefabs.lua).
-- What each of the model's moving parts does is the list below (model_moves.lua).
return require("model_moves").script{
  { node = "pan", from = 0, to = 0.018, speed = 0.06, travel = 1.6, wait = 4, wait_back = 3, delay = 1 },
  { node = "needle", from = 0, to = 190, speed = 130, travel = 1.6, wait = 4, wait_back = 3, delay = 1 },
}
