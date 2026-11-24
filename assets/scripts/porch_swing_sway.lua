-- A porch swing swinging gently (#191; a prefab's script, prefabs.lua).
-- What each of the model's moving parts does is the list below (model_moves.lua).
return require("model_moves").script{
  { node = "seat", sway = 14, period = 3.4 },
}
