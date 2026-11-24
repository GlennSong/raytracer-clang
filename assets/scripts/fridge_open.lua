-- A fridge in use: its door is opened and shut, and now and then the freezer's (#191; a prefab's script, prefabs.lua).
-- What each of the model's moving parts does is the list below (model_moves.lua).
return require("model_moves").script{
  { node = "fridge_door", from = 0, to = 105, speed = 110, wait = 4, wait_back = 8, delay = 1 },
  { node = "freezer_door", from = 0, to = 90, speed = 110, wait = 3, wait_back = 20, delay = 9 },
}
