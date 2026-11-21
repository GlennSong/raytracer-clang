-- A desk lamp being turned and aimed (#191; a prefab's script, prefabs.lua).
-- What each of the model's moving parts does is the list below (model_moves.lua).
return require("model_moves").script{
  { node = "swing", from = -40, to = 40, speed = 25, wait = 4 },
  { node = "lower", from = -15, to = 20, speed = 12, wait = 5, delay = 1 },
  { node = "head", from = -20, to = 25, speed = 20, wait = 3, delay = 2 },
}
