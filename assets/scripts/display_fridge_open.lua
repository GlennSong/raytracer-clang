-- A shop's drinks fridge with customers at it: a door is opened, a drink taken, the door let shut (#191; a prefab's script, prefabs.lua).
-- What each of the model's moving parts does is the list below (model_moves.lua).
return require("model_moves").script{
  { node = "door_1", from = 0, to = 85, speed = 120, wait = 2.5, wait_back = 9, delay = 5 },
  { node = "door_2", from = 0, to = 85, speed = 120, wait = 2.5, wait_back = 9, delay = 1.5 },
  { node = "door_3", from = 0, to = 85, speed = 120, wait = 2.5, wait_back = 9, delay = 8 },
  { node = "door_4", from = 0, to = 85, speed = 120, wait = 2.5, wait_back = 9, delay = 11 },
  { node = "door_5", from = 0, to = 85, speed = 120, wait = 2.5, wait_back = 9, delay = 3 },
}
