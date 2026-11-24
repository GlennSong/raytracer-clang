-- A drip stand raised and lowered (#191; a prefab's script, prefabs.lua).
-- What each of the model's moving parts does is the list below (model_moves.lua).
return require("model_moves").script{
  { node = "pole", from = 0, to = 0.45, speed = 0.2, wait = 8, wait_back = 5, delay = 1 },
}
