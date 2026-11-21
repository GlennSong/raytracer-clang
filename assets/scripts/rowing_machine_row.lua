-- A rowing machine in use: the seat runs back as the handle is pulled, and both return; the fan spins (#191; a prefab's script, prefabs.lua).
-- What each of the model's moving parts does is the list below (model_moves.lua).
return require("model_moves").script{
  { node = "seat", from = 0, to = 0.7, speed = 0.9, travel = 0.9, wait = 0.2, wait_back = 0.3 },
  { node = "handle", from = 0, to = 0.75, speed = 0.95, travel = 0.9, wait = 0.2, wait_back = 0.3 },
  { node = "fan", spin = 500 },
}
