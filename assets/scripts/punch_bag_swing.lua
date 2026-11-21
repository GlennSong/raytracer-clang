-- A punch bag swinging after a blow (#191; a prefab's script, prefabs.lua).
-- What each of the model's moving parts does is the list below (model_moves.lua).
return require("model_moves").script{
  { node = "bag", sway = 12, period = 2.4 },
}
