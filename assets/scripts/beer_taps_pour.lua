-- A bar's taps in use: a handle pulled while a glass fills, then let back (#191; a prefab's script, prefabs.lua).
-- What each of the model's moving parts does is the list below (model_moves.lua).
return require("model_moves").script{
  { node = "tap_1", from = 0, to = 48, speed = 120, wait = 5, wait_back = 9, delay = 6 },
  { node = "tap_2", from = 0, to = 48, speed = 120, wait = 5, wait_back = 9, delay = 1 },
  { node = "tap_3", from = 0, to = 48, speed = 120, wait = 5, wait_back = 9, delay = 11 },
  { node = "tap_4", from = 0, to = 48, speed = 120, wait = 5, wait_back = 9, delay = 3 },
}
