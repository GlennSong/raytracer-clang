-- A laptop opened to work at and shut again (#191; a prefab's script, prefabs.lua).
-- What each of the model's moving parts does is the list below (model_moves.lua).
return require("model_moves").script{
  { node = "lid", from = 0, to = 110, speed = 90, wait = 12, wait_back = 5, delay = 1 },
}
