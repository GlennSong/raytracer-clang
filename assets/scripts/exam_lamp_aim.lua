-- An examination lamp swung round and tipped (#191; a prefab's script, prefabs.lua).
-- What each of the model's moving parts does is the list below (model_moves.lua).
return require("model_moves").script{
  { node = "arm", sway = 60, period = 12 },
  { node = "head", sway = 25, period = 7 },
}
