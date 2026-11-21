-- A turnstile with people going through: its arms turn a third of a turn for each (#191; a prefab's script, prefabs.lua).
-- What each of the model's moving parts does is the list below (model_moves.lua).
return require("model_moves").script{
  { node = "arms", step = 120, speed = 150, wait = 2.5 },
}
