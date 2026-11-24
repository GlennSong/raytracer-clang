-- Capsule machines with customers at them: a knob is turned right round for each capsule (#191; a prefab's script, prefabs.lua).
-- What each of the model's moving parts does is the list below (model_moves.lua).
return require("model_moves").script{
  { node = "knob_1_1", step = 360, speed = 240, wait = 7, delay = 1 },
  { node = "knob_2_2", step = 360, speed = 240, wait = 9, delay = 4 },
  { node = "knob_3_1", step = 360, speed = 240, wait = 11, delay = 6 },
  { node = "knob_2_1", step = 360, speed = 240, wait = 13, delay = 9 },
}
