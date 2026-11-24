-- A toaster making toast: the lever and the bread go down together, and come up when it is done (#191; a prefab's script, prefabs.lua).
-- What each of the model's moving parts does is the list below (model_moves.lua).
return require("model_moves").script{
  { node = "lever", from = 0, to = 0.075, speed = 0.3, travel = 0.4, wait = 6, wait_back = 4, delay = 1 },
  { node = "toast", from = 0, to = 0.075, speed = 0.3, travel = 0.4, wait = 6, wait_back = 4, delay = 1 },
}
