-- A full-height turnstile letting people through, a third of a turn each (#191; a prefab's script, prefabs.lua).
-- What each of the model's moving parts does is the list below (model_moves.lua).
return require("model_moves").script{
  { node = "rotor", step = 120, speed = 80, wait = 3 },
}
