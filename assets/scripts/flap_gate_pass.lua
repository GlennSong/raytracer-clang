-- A ticket gate letting people through: both paddles swing away and close behind (#191; a prefab's script, prefabs.lua).
-- What each of the model's moving parts does is the list below (model_moves.lua).
return require("model_moves").script{
  { node = "paddle_left", from = 0, to = 85, speed = 250, travel = 0.5, wait = 1.5, wait_back = 2.5, delay = 1 },
  { node = "paddle_right", from = 0, to = 85, speed = 250, travel = 0.5, wait = 1.5, wait_back = 2.5, delay = 1 },
}
