-- A rice cooker opened to serve from and shut again (#191; a prefab's script, prefabs.lua).
-- What each of the model's moving parts does is the list below (model_moves.lua).
return require("model_moves").script{
  { node = "lid", from = 0, to = 80, speed = 90, wait = 4, wait_back = 10, delay = 1 },
}
