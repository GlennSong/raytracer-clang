-- Sliding paper screens opened and shut, first one and then the other; each is sent further than it can go and stops at its limit (#191; a prefab's script, prefabs.lua).
-- What each of the model's moving parts does is the list below (model_moves.lua).
return require("model_moves").script{
  { node = "screen_1", from = 0, to = 3, speed = 0.5, travel = 3, wait = 4, wait_back = 12, delay = 1 },
  { node = "screen_2", from = 0, to = 3, speed = 0.5, travel = 3, wait = 4, wait_back = 12, delay = 11 },
}
