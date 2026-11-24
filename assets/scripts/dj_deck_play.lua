-- A DJ's decks playing: both records turn at 33 turns a minute, and the crossfader goes from one to the other (#191; a prefab's script, prefabs.lua).
-- What each of the model's moving parts does is the list below (model_moves.lua).
return require("model_moves").script{
  { node = "record_1", spin = 200 },
  { node = "record_2", spin = 200 },
  { node = "fader", from = -0.05, to = 0.05, speed = 0.03, wait = 12 },
}
