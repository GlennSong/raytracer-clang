-- A piano opened to play: the cover over the keys is folded back, and later closed (#191; a prefab's script, prefabs.lua).
-- What each of the model's moving parts does is the list below (model_moves.lua).
return require("model_moves").script{
  { node = "fallboard", from = 0, to = 92, speed = 60, wait = 15, wait_back = 5, delay = 1 },
  { node = "lid", from = 0, to = 25, speed = 20, wait = 12, wait_back = 9, delay = 3 },
}
