-- A door in use: opened, held a moment and let shut; a second leaf follows the first (#191; a prefab's script, prefabs.lua).
-- What each of the model's moving parts does is the list below (model_moves.lua).
return require("model_moves").script{
  { node = "door_1", from = 0, to = 95, speed = 110, wait = 2.5, wait_back = 6, delay = 1.5 },
  { node = "door_2", from = 0, to = 95, speed = 110, wait = 2, wait_back = 6.5, delay = 2 },
}
