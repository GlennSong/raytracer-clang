-- Lockers in use: a few doors opened, left a while and shut (#191; a prefab's script, prefabs.lua).
-- What each of the model's moving parts does is the list below (model_moves.lua).
return require("model_moves").script{
  { node = "door_1_1", from = 0, to = 100, speed = 120, wait = 5, wait_back = 9, delay = 1 },
  { node = "door_3_1", from = 0, to = 100, speed = 120, wait = 4, wait_back = 12, delay = 4 },
  { node = "door_2_2", from = 0, to = 100, speed = 120, wait = 4, wait_back = 10, delay = 2 },
  { node = "door_4_3", from = 0, to = 100, speed = 120, wait = 3, wait_back = 14, delay = 7 },
}
