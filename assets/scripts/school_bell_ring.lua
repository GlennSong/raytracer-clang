-- A school bell rung: the bell swings and the clapper against it (#191; a prefab's script, prefabs.lua).
-- What each of the model's moving parts does is the list below (model_moves.lua).
return require("model_moves").script{
  { node = "bell", sway = 28, period = 1.1 },
  { node = "clapper", sway = 22, period = 1.1, phase = 0.3 },
}
