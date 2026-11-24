-- An oven in use: its door is opened and shut, and two of its knobs are turned up and down (#191; a prefab's script, prefabs.lua).
-- What each of the model's moving parts does is the list below (model_moves.lua).
return require("model_moves").script{
  { node = "door", from = 0, to = 85, speed = 70, wait = 4, wait_back = 9, delay = 2 },
  { node = "knob_1", from = 0, to = 200, speed = 200, wait = 12, wait_back = 3 },
  { node = "knob_4", from = 0, to = 120, speed = 200, wait = 7, wait_back = 5, delay = 4 },
}
