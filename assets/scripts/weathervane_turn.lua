-- A weathervane hunting about the wind's direction (#191; a prefab's script, prefabs.lua).
-- What each of the model's moving parts does is the list below (model_moves.lua).
return require("model_moves").script{
  { node = "vane", sway = 35, period = 9 },
}
