-- furniture_library.lua -- THE FURNITURE LIBRARY's descriptions (Glenn, 2026-10-01: "tag the furniture for use in
-- different places ... the furniture could have interaction points like sitting"). One entry per kit piece, by its
-- name (furniture.pieces()). Loaded once by the engine (procgen/furniture_library.h); a piece with no entry is
-- scenery.
--
-- PIECE SPACE: x across the piece, y up from the floor, z out from the wall it backs onto. Every seat faces +z.
--   spots  the places a body goes, each {id, at}.
--   verbs  how a body uses them, the FIRST verb the primary (tap E), another the secondary (hold E). Each:
--            verb   "sit" | "lie"
--            spots  the spot ids it takes -- a verb is offered only while all of them are free, so a sofa
--                   seats three, or lets one lie down
--            eye    where the user's eye goes; look the way they face
--            exit   where they stand up, on the floor
--          A piece may list one verb several times (a sofa's three seats): the nearest free one is offered.

local function seat(x, y, z) return { x, y, z } end

furniture_library = {
  office_chair = {
    family = "seating", tags = { "office", "study" },
    spots = { { id = "seat", at = seat(0, 0.52, 0.35) } },
    verbs = { { verb = "sit", spots = { "seat" }, eye = { 0, 1.24, 0.28 }, look = { 0, -0.08, 1 }, exit = { 0, 0, 1.05 } } },
  },
  dining_chair = {
    family = "seating", tags = { "kitchen", "dining", "living" },
    spots = { { id = "seat", at = seat(0, 0.49, 0.28) } },
    verbs = { { verb = "sit", spots = { "seat" }, eye = { 0, 1.21, 0.22 }, look = { 0, -0.08, 1 }, exit = { 0, 0, 0.9 } } },
  },
  lounge_chair = {
    family = "seating", tags = { "living", "lobby", "study" },
    spots = { { id = "seat", at = seat(0, 0.52, 0.50) } },
    verbs = { { verb = "sit", spots = { "seat" }, eye = { 0, 1.16, 0.36 }, look = { 0, -0.05, 1 }, exit = { 0, 0, 1.25 } } },
  },
  sofa = {
    family = "seating", tags = { "living", "lobby" },
    spots = {
      { id = "l", at = seat(-0.62, 0.50, 0.55) },
      { id = "m", at = seat(0.0, 0.50, 0.55) },
      { id = "r", at = seat(0.62, 0.50, 0.55) },
    },
    verbs = {
      { verb = "sit", spots = { "l" }, eye = { -0.62, 1.20, 0.45 }, look = { 0, -0.06, 1 }, exit = { -0.62, 0, 1.35 } },
      { verb = "sit", spots = { "m" }, eye = { 0.0, 1.20, 0.45 }, look = { 0, -0.06, 1 }, exit = { 0.0, 0, 1.35 } },
      { verb = "sit", spots = { "r" }, eye = { 0.62, 1.20, 0.45 }, look = { 0, -0.06, 1 }, exit = { 0.62, 0, 1.35 } },
      -- lying full length, the head on the left arm, looking up and along the sofa
      { verb = "lie", label = "lie down", spots = { "l", "m", "r" }, eye = { -0.78, 0.78, 0.52 }, look = { 1, 1.3, 0 },
        exit = { 0, 0, 1.35 } },
    },
  },
  bed = {
    family = "sleeping", tags = { "bedroom" },
    spots = {
      { id = "bed", at = seat(0, 0.58, 1.06) },
      { id = "edge_l", at = seat(-0.55, 0.58, 1.25) },
      { id = "edge_r", at = seat(0.55, 0.58, 1.25) },
    },
    verbs = {
      -- on the back, the head on the pillows at the headboard, looking up
      { verb = "lie", label = "lie down", spots = { "bed", "edge_l", "edge_r" }, eye = { 0, 0.82, 0.38 },
        look = { 0, 1.6, 1 }, exit = { 1.3, 0, 1.06 } },
      -- sitting on either edge, feet on the floor beside it
      { verb = "sit", label = "sit on the edge", spots = { "edge_l" }, eye = { -0.66, 1.28, 1.25 }, look = { -1, -0.08, 0 },
        exit = { -1.3, 0, 1.25 } },
      { verb = "sit", label = "sit on the edge", spots = { "edge_r" }, eye = { 0.66, 1.28, 1.25 }, look = { 1, -0.08, 0 },
        exit = { 1.3, 0, 1.25 } },
    },
  },
  -- OUTDOOR SEATING (M2): parks, plazas, forecourts, café terraces
  bench = {
    family = "seating", tags = { "outdoor", "public", "park" },
    spots = {
      { id = "l", at = seat(-0.55, 0.47, 0.35) },
      { id = "m", at = seat(0.0, 0.47, 0.35) },
      { id = "r", at = seat(0.55, 0.47, 0.35) },
    },
    verbs = {
      { verb = "sit", spots = { "l" }, eye = { -0.55, 1.17, 0.27 }, look = { 0, -0.05, 1 }, exit = { -0.55, 0, 1.1 } },
      { verb = "sit", spots = { "m" }, eye = { 0.0, 1.17, 0.27 }, look = { 0, -0.05, 1 }, exit = { 0.0, 0, 1.1 } },
      { verb = "sit", spots = { "r" }, eye = { 0.55, 1.17, 0.27 }, look = { 0, -0.05, 1 }, exit = { 0.55, 0, 1.1 } },
      { verb = "lie", label = "lie down", spots = { "l", "m", "r" }, eye = { -0.68, 0.70, 0.33 }, look = { 1, 1.4, 0 },
        exit = { 0, 0, 1.1 } },
    },
  },
  plaza_bench = {
    family = "seating", tags = { "outdoor", "public", "plaza" },
    spots = {
      { id = "l", at = seat(-0.55, 0.46, 0.27) },
      { id = "m", at = seat(0.0, 0.46, 0.27) },
      { id = "r", at = seat(0.55, 0.46, 0.27) },
    },
    verbs = {
      { verb = "sit", spots = { "l" }, eye = { -0.55, 1.16, 0.22 }, look = { 0, -0.05, 1 }, exit = { -0.55, 0, 1.0 } },
      { verb = "sit", spots = { "m" }, eye = { 0.0, 1.16, 0.22 }, look = { 0, -0.05, 1 }, exit = { 0.0, 0, 1.0 } },
      { verb = "sit", spots = { "r" }, eye = { 0.55, 1.16, 0.22 }, look = { 0, -0.05, 1 }, exit = { 0.55, 0, 1.0 } },
      { verb = "lie", label = "lie down", spots = { "l", "m", "r" }, eye = { -0.75, 0.66, 0.27 }, look = { 1, 1.4, 0 },
        exit = { 0, 0, 1.0 } },
    },
  },
  bistro_chair = {
    family = "seating", tags = { "outdoor", "cafe", "kitchen" },
    spots = { { id = "seat", at = seat(0, 0.48, 0.26) } },
    verbs = { { verb = "sit", spots = { "seat" }, eye = { 0, 1.19, 0.20 }, look = { 0, -0.08, 1 }, exit = { 0, 0, 0.9 } } },
  },
  toilet = {
    family = "fixture", tags = { "bathroom" },
    spots = { { id = "seat", at = seat(0, 0.42, 0.42) } },
    verbs = { { verb = "sit", spots = { "seat" }, eye = { 0, 1.12, 0.36 }, look = { 0, -0.1, 1 }, exit = { 0, 0, 1.05 } } },
  },
}
