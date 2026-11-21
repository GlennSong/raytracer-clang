-- A ballot box in use: folded papers drop in through the slot, one after another (#191; a prefab's script, prefabs.lua).
-- What each of the model's moving parts does is the list below (model_moves.lua).
return require("model_moves").script{
  { node = "paper", loop = 0.5, speed = 0.35 },
}
