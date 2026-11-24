-- A whiteboard turned over to its other side, and back (#191; a prefab's script, prefabs.lua).
-- What each of the model's moving parts does is the list below (model_moves.lua).
return require("model_moves").script{
  { node = "board", from = 0, to = 180, speed = 120, wait = 6, delay = 2 },
}
