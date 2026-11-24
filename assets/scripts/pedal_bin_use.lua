-- A pedal bin in use: a foot on the pedal lifts the lid (#191; a prefab's script, prefabs.lua).
-- What each of the model's moving parts does is the list below (model_moves.lua).
return require("model_moves").script{
  { node = "pedal", from = 0, to = 18, speed = 90, travel = 0.3, wait = 2, wait_back = 6, delay = 1 },
  { node = "lid", from = 0, to = 70, speed = 350, travel = 0.3, wait = 2, wait_back = 6, delay = 1 },
}
