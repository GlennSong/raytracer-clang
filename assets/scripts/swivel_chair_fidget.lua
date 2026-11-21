-- An office chair with someone in it: it turns a little from side to side, leans back, and now and then is raised or lowered (#191; a prefab's script, prefabs.lua).
-- What each of the model's moving parts does is the list below (model_moves.lua).
return require("model_moves").script{
  { node = "swivel", from = -28, to = 24, speed = 30, wait = 2.5, wait_back = 4 },
  { node = "recline", from = 0, to = 14, speed = 10, wait = 5, wait_back = 3, delay = 1.5 },
  { node = "lift", from = 0, to = 0.1, speed = 0.06, wait = 9, wait_back = 9, delay = 4 },
  { node = "caster_1", sway = 30, period = 7 }, { node = "caster_3", sway = 40, period = 5, phase = 1 },
}
