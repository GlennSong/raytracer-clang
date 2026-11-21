-- A desk raised to standing height and lowered again: its two leg stages slide by the same amount (#191; a prefab's script, prefabs.lua).
-- What each of the model's moving parts does is the list below (model_moves.lua).
return require("model_moves").script{
  { node = "stage", from = 0, to = 0.3, speed = 0.035, wait = 6, delay = 2 },
  { node = "top", from = 0, to = 0.3, speed = 0.035, wait = 6, delay = 2 },
}
