-- TUNES (#191; Glenn, 2026-11-23: "would be fun to have musical functionality ... to play a song"). Music a prefab
-- plays, written as notes and made into sound by the engine (src/engine/audio/sfx.h `tune`): nothing is recorded.
-- A kind of sound in prefab_sounds.lua names tunes to play (`playing = { "tune:parlour_waltz", ... }`), one after
-- another each time the use is started, each looping for as long as the use is on.
--
--   voice   "piano" (struck strings) or "record" (a mellow keyboard through a worn record, with crackle)
--   bpm     beats a minute
--   parts   lines of notes, all starting together; the first is the tune, the rest play under it, quieter.
--           A note is a pitch and its beats: `C4:1` (C4 is middle C; `F#3`, `Bb4`), a chord `C4+E4+G4:2`, a rest
--           `-:1`. A `|` is only for the eye (a bar line). Every part should add up to the same number of beats.
--
-- These three were written for this game. Do not put a real song here: a tune is somebody's.
return {
  -- a waltz for the upright piano: sixteen bars in C, the left hand a bass note and two chords to the bar
  parlour_waltz = { voice = "piano", bpm = 138, parts = {
    "E5:2 G5:1 | F5:2 D5:1 | E5:1 D5:1 C5:1 | D5:3 | C5:2 E5:1 | D5:2 B4:1 | C5:1 B4:1 A4:1 | G4:3 | " ..
    "A4:2 C5:1 | B4:2 G4:1 | A4:1 G4:1 F4:1 | E4:3 | F4:1 A4:1 C5:1 | E5:2 D5:1 | C5:3 | C5:2 -:1",
    "C3:1 E3+G3:1 E3+G3:1 | G2:1 F3+B3:1 F3+B3:1 | C3:1 E3+G3:1 E3+G3:1 | G2:1 D3+G3:1 D3+G3:1 | " ..
    "A2:1 E3+A3:1 E3+A3:1 | G2:1 D3+G3:1 D3+G3:1 | F2:1 C3+F3:1 C3+F3:1 | C3:1 E3+G3:1 E3+G3:1 | " ..
    "F2:1 C3+F3:1 C3+F3:1 | G2:1 D3+G3:1 D3+G3:1 | F2:1 C3+F3:1 C3+F3:1 | C3:1 E3+G3:1 E3+G3:1 | " ..
    "F2:1 C3+F3:1 C3+F3:1 | G2:1 F3+B3:1 F3+B3:1 | C3:1 E3+G3:1 E3+G3:1 | C3:1 E3+G3:1 -:1",
  } },
  -- a slow piece in A minor: broken chords under a plain tune, eight bars of four
  rainy_window = { voice = "piano", bpm = 76, parts = {
    "E5:2 C5:1 D5:1 | E5:3 -:1 | D5:2 B4:1 C5:1 | D5:3 -:1 | C5:2 A4:1 B4:1 | C5:2 E5:2 | D5:1 C5:1 B4:2 | A4:4",
    "A2:1 E3:1 A3:1 E3:1 | F2:1 C3:1 F3:1 C3:1 | G2:1 D3:1 G3:1 D3:1 | E2:1 B2:1 E3:1 B2:1 | " ..
    "F2:1 C3:1 F3:1 C3:1 | C3:1 G3:1 C4:1 G3:1 | E2:1 B2:1 E3:1 G#3:1 | A2:1 E3:1 A3:2",
  } },
  -- for the record player: a lazy swing in F, a walking bass under held chords, eight bars of four, twice through
  -- with the tune turned the second time
  sunday_record = { voice = "record", bpm = 104, parts = {
    "A4:1.5 C5:0.5 D5:1 C5:1 | A4:2 F4:2 | G4:1.5 Bb4:0.5 C5:1 Bb4:1 | G4:3 -:1 | " ..
    "A4:1.5 C5:0.5 F5:1 E5:1 | D5:2 Bb4:2 | C5:1 A4:1 G4:1 E4:1 | F4:3 -:1 | " ..
    "C5:1.5 D5:0.5 F5:1 D5:1 | C5:2 A4:2 | Bb4:1.5 C5:0.5 D5:1 C5:1 | Bb4:3 -:1 | " ..
    "A4:1 C5:1 F5:1.5 E5:0.5 | D5:2 C5:2 | A4:1 G4:1 F4:1 E4:1 | F4:4",
    "F2:1 A2:1 C3:1 D3:1 | D2:1 F2:1 A2:1 C3:1 | G2:1 Bb2:1 D3:1 E3:1 | C2:1 E2:1 G2:1 Bb2:1 | " ..
    "F2:1 A2:1 C3:1 Eb3:1 | Bb2:1 D3:1 F3:1 D3:1 | C3:1 Bb2:1 A2:1 G2:1 | F2:1 C3:1 F2:1 C2:1 | " ..
    "F2:1 A2:1 C3:1 D3:1 | D2:1 F2:1 A2:1 C3:1 | G2:1 Bb2:1 D3:1 E3:1 | C2:1 E2:1 G2:1 Bb2:1 | " ..
    "F2:1 A2:1 C3:1 Eb3:1 | Bb2:1 D3:1 F3:1 D3:1 | C3:1 Bb2:1 A2:1 G2:1 | F2:2 C2:2",
    "A3+C4+E4:4 | F3+A3+C4:4 | Bb3+D4+F4:4 | Bb3+C4+E4:4 | A3+C4+Eb4:4 | Bb3+D4+F4:4 | Bb3+C4+E4:4 | A3+C4+F4:4 | " ..
    "A3+C4+E4:4 | F3+A3+C4:4 | Bb3+D4+F4:4 | Bb3+C4+E4:4 | A3+C4+Eb4:4 | Bb3+D4+F4:4 | Bb3+C4+E4:4 | A3+C4+F4:4",
  } },
}
