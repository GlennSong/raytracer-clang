-- A kitchen's swing doors with staff going through: one leaf pushed out, the other pushed in (#191; a prefab's script, prefabs.lua).
-- What each of the model's moving parts does is the list below (model_moves.lua).
return require("model_moves").script{
  { node = "door_1", from = 0, to = 75, speed = 220, wait = 0.5, wait_back = 5, delay = 1 },
  { node = "door_2", from = 0, to = -75, speed = 220, wait = 0.5, wait_back = 5, delay = 3.5 },
}
