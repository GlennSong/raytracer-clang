-- A flag raised in the morning and lowered at night, many times a day so it can be seen (#191; a prefab's script, prefabs.lua).
-- What each of the model's moving parts does is the list below (model_moves.lua).
return require("model_moves").script{
  { node = "flag", from = 0, to = 30, speed = 0.6, wait = 15, wait_back = 5, delay = 2 },
}
