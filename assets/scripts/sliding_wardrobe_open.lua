-- A sliding wardrobe in use: one door is slid across and back, then the other; each is sent further than it can go (#191; a prefab's script, prefabs.lua).
-- What each of the model's moving parts does is the list below (model_moves.lua).
return require("model_moves").script{
  { node = "door_1", from = 0, to = 3, speed = 0.5, travel = 3, wait = 5, wait_back = 13, delay = 1 },
  { node = "door_2", from = 0, to = 3, speed = 0.5, travel = 3, wait = 5, wait_back = 13, delay = 12 },
}
