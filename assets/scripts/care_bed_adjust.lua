-- A care bed being adjusted: it rises, the head end tilts up, and one rail is dropped and raised (#191; a prefab's script, prefabs.lua).
-- What each of the model's moving parts does is the list below (model_moves.lua).
return require("model_moves").script{
  { node = "lift", from = 0, to = 0.28, speed = 0.06, travel = 5, wait = 12, wait_back = 4, delay = 1 },
  { node = "back", from = 0, to = 55, speed = 12, travel = 5, wait = 9, wait_back = 7, delay = 4 },
  { node = "rail_left", from = 0, to = 0.3, speed = 0.3, travel = 1, wait = 5, wait_back = 19, delay = 8 },
}
