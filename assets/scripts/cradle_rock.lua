-- A cradle rocking (#191; a prefab's script, prefabs.lua).
-- What each of the model's moving parts does is the list below (model_moves.lua).
return require("model_moves").script{
  { node = "rock", sway = 12, period = 2.6 },
}
