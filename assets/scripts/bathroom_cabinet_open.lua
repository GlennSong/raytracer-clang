-- A bathroom cabinet in use: one door is opened and shut, then the other (#191; a prefab's script, prefabs.lua).
-- What each of the model's moving parts does is the list below (model_moves.lua).
return require("model_moves").script{
  { node = "door_left", from = 0, to = 100, speed = 120, travel = 1, wait = 4, wait_back = 10, delay = 1 },
  { node = "door_right", from = 0, to = 100, speed = 120, travel = 1, wait = 4, wait_back = 10, delay = 8 },
}
