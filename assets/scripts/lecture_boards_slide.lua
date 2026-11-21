-- A lecturer changing boards: the front one goes up as the back one comes down, and back again (#191; a prefab's script, prefabs.lua).
-- What each of the model's moving parts does is the list below (model_moves.lua).
return require("model_moves").script{
  { node = "board_front", from = 0, to = 3, speed = 0.45, wait = 8, wait_back = 8, delay = 2 },
  { node = "board_back", from = 0, to = -3, speed = 0.45, wait = 8, wait_back = 8, delay = 2 },
}
