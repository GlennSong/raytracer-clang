-- A metronome ticking (#191; a prefab's script, prefabs.lua).
-- What each of the model's moving parts does is the list below (model_moves.lua).
return require("model_moves").script{
  { node = "arm", sway = 28, period = 1.0 },
}
