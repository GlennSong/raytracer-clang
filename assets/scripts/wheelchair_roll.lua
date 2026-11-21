-- A wheelchair's wheels turning as it is pushed (it does not travel: nothing a prefab does moves it along yet) (#191; a prefab's script, prefabs.lua).
-- What each of the model's moving parts does is the list below (model_moves.lua).
return require("model_moves").script{
  { node = "wheel_left", spin = 120 },
  { node = "wheel_right", spin = 120 },
  { node = "footrest_left", from = 0, to = 90, speed = 90, wait = 3, wait_back = 12, delay = 2 },
}
