-- A bin's, mailbox's or newspaper box's flap, pushed or pulled open and let go (#191; a prefab's script, prefabs.lua).
-- What each of the model's moving parts does is the list below (model_moves.lua).
return require("model_moves").script{
  { node = "flap", from = 0, to = 55, speed = 140, wait = 1.2, wait_back = 6, delay = 2 },
}
