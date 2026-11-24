-- A patio umbrella folded down and opened again: every rib turns by the same amount (#191; a prefab's script, prefabs.lua).
-- What each of the model's moving parts does is the list below (model_moves.lua).
return require("model_moves").script{
  { node = "rib_1", from = 0, to = 66, speed = 11, wait = 5 }, { node = "rib_2", from = 0, to = 66, speed = 11, wait = 5 },
  { node = "rib_3", from = 0, to = 66, speed = 11, wait = 5 }, { node = "rib_4", from = 0, to = 66, speed = 11, wait = 5 },
  { node = "rib_5", from = 0, to = 66, speed = 11, wait = 5 }, { node = "rib_6", from = 0, to = 66, speed = 11, wait = 5 },
  { node = "rib_7", from = 0, to = 66, speed = 11, wait = 5 }, { node = "rib_8", from = 0, to = 66, speed = 11, wait = 5 },
}
