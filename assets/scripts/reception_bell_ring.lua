-- A desk bell rung now and then (#191; a prefab's script, prefabs.lua).
-- What each of the model's moving parts does is the list below (model_moves.lua).
return require("model_moves").script{
  { node = "button", from = 0, to = 0.012, speed = 0.2, travel = 0.1, wait = 0.05, wait_back = 5, delay = 1 },
}
