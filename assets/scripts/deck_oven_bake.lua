-- A baker's oven in use: one deck's door after another is dropped open and shut (#191; a prefab's script, prefabs.lua).
-- What each of the model's moving parts does is the list below (model_moves.lua).
return require("model_moves").script{
  { node = "door_1", from = 0, to = 88, speed = 80, travel = 1.5, wait = 4, wait_back = 18, delay = 1 },
  { node = "door_2", from = 0, to = 88, speed = 80, travel = 1.5, wait = 4, wait_back = 18, delay = 9.5 },
  { node = "door_3", from = 0, to = 88, speed = 80, travel = 1.5, wait = 4, wait_back = 18, delay = 18 },
}
