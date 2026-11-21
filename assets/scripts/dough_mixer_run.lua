-- A dough mixer kneading: the bowl turns one way and the hook the other (#191; a prefab's script, prefabs.lua).
-- What each of the model's moving parts does is the list below (model_moves.lua).
return require("model_moves").script{
  { node = "bowl", spin = 30 },
  { node = "hook", spin = 150 },
}
