-- PREFAB SOUNDS: what a use of a prefab sounds like (#191, step 9; Glenn, 2026-11-21: "a motor whirr for the
-- standing desk? Cabinet open and close sounds?"). A use in prefabs.lua says `sound = "<a name here>"`; this file
-- says what is heard for each thing that can happen to it. Read once by PrefabUseSystem.
--
--   done      when the use is done (a door starts to open, a switch is pressed)
--   undone    when it is undone (a door starts to close)
--   arrived   when its parts stop moving, the use on (a drawer reaches the end of its runners)
--   shut      when its parts stop moving, the use off (a door meets its frame). No `shut`: `arrived` is heard
--   moving    a loop for as long as its parts are on their way (a motor)
--   running   a loop for as long as it runs (`run`), slowing with it as it coasts
--   playing   music for as long as the use is on: a tune's name, or a list of them taken in turn ("tune:<a name in tunes.lua>")
--   volume, pitch, range (metres)
--
-- A clip is one of the engine's procedural sounds, made from nothing (src/engine/audio/sfx.h), by these names:
--   latch_open, door_shut, lid_shut, heavy_shut, knock, tick, ding, sliding_doors (a loop of a motor and a thunk),
--   hum (a motor's hum, a loop), winch (a ratchet and motor, a loop), roll (small wheels, a loop), freewheel (a loop)
-- There are about a dozen kinds here for some 130 prefabs. A new kind is a line; a new clip is a sound to write.
return {
  door     = { done = "latch_open", shut = "door_shut", volume = 0.8 },                 -- a hinged door, a gate
  heavy    = { done = "latch_open", shut = "heavy_shut", volume = 0.9, pitch = 0.85 },  -- a safe, an oven, a fridge
  cupboard = { done = "latch_open", shut = "lid_shut", volume = 0.5, pitch = 1.25 },    -- a light door, a lid, a flap
  drawer   = { moving = "roll", arrived = "knock", shut = "lid_shut", volume = 0.35, pitch = 1.3, range = 8 },
  sliding  = { moving = "roll", shut = "knock", volume = 0.4, pitch = 0.8 },            -- a door or a screen on a track
  powered  = { moving = "hum", done = "tick", volume = 0.5, pitch = 1.1, range = 14 },  -- automatic doors, a barrier, a shutter
  motor    = { moving = "hum", arrived = "tick", volume = 0.35, pitch = 1.7, range = 8 },   -- a desk's or a bed's lift
  machine  = { done = "tick", running = "hum", undone = "tick", volume = 0.6, pitch = 1.35, range = 14 },   -- a washer, a mixer, a fan
  wheel    = { running = "freewheel", volume = 0.4, pitch = 0.9, range = 10 },          -- pedals, a flywheel, a roundabout
  winch    = { moving = "winch", arrived = "knock", volume = 0.4, range = 12 },         -- a blind, a screen, a flag run up
  click    = { done = "tick", undone = "tick", volume = 0.7, range = 6 },               -- a switch, a button, a lever
  bell     = { done = "ding", volume = 0.9, range = 30 },
  knock    = { done = "knock", volume = 0.6 },                                          -- a thing struck or pushed
  -- MUSIC (tunes.lua): `playing` is tunes, one after another each time the use is started, each a loop while it is on
  piano    = { playing = { "tune:parlour_waltz", "tune:rainy_window" }, volume = 0.9, range = 22 },
  record   = { done = "tick", undone = "tick", playing = { "tune:sunday_record" }, volume = 0.7, range = 18 },
}
