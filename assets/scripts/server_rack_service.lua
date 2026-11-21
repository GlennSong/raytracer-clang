-- A server rack being serviced: the door is opened, one server is slid out on its rails and back, and the door is shut (#191; a prefab's script, prefabs.lua).
-- What each of the model's moving parts does is the list below (model_moves.lua).
return require("model_moves").script{
  { node = "door", from = 0, to = 115, speed = 80, travel = 2, wait = 12, wait_back = 6, delay = 1 },
  { node = "server", from = 0, to = 0.5, speed = 0.3, travel = 2, wait = 5, wait_back = 13, delay = 4 },
}
