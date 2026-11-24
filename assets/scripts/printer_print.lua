-- A copier at work: sheets come out one after another, the lid is lifted for a copy, and a paper drawer is pulled out to fill (#191; a prefab's script, prefabs.lua).
-- What each of the model's moving parts does is the list below (model_moves.lua).
return require("model_moves").script{
  { node = "sheet", loop = 0.22, speed = 0.25 },
  { node = "lid", from = 0, to = 65, speed = 70, travel = 1.2, wait = 4, wait_back = 18, delay = 3 },
  { node = "tray", from = 0, to = 0.38, speed = 0.4, travel = 1.2, wait = 4, wait_back = 18, delay = 14 },
}
