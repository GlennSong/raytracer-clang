-- furniture_rooms.lua -- ROOM PROGRAMS (the furniture library, M3b; Glenn, 2026-10-03: "go ahead with the furniture
-- tags"). What a room of each kind holds, as steps run in order. Loaded with furniture_library.lua; the floor plans
-- (room_plan.cpp) name the kinds: office, bedroom, flat, living, kitchen, open_plan, meeting, kitchenette, closet,
-- bath. Shops are still furnished in code, by their trade.
--
-- A PICK is a piece name ("monitor") or what the piece is -- { family = "...", tags = { ... } }: every piece in
-- furniture_library.lua of that family carrying ALL those tags and fitting the slot, the largest that fits.
-- Describe a new piece with the right family and tags and the rooms it suits start using it.
--
-- STEPS
--   wall = { w, d }   a w x d footprint against a wall (w along it, d out into the room). With `pick`, that one piece
--                     centred; with `set`, several pieces, each { pick, x, z [, y] [, faces = "wall"] [, fit = {w, d}] }:
--                     x along the wall from the footprint's left end, z out from it, `faces = "wall"` turned back
--                     toward the wall (a chair at its desk), `fit` the slot a pick must fit (default: the footprint).
--   sides             the walls to try, in order: 0 the window wall, 2 the one opposite, 1 and 3 the ends; "long" =
--                     the room's long walls first; "opposite" = across from the previous step (only if it placed).
--   ring_sides        the walls to try in a ring floor's room (offices round a core), when different.
--   tall              it stands up the wall (no picture over it).   clear  floor kept free in front (metres; a
--                     described clearance wins).   count  a number, or { per = m, min, max }: one per m of width.
--   min_w             only in a room wider than this.
--   grid = { w, d }   cells in rows out from the window wall: aisle = { x, z }; short = a shallower cell where a full
--                     one will not fit, built with short_style (the piece's style bits).
--   counter = "kitchen" | "kitchenette"   a run of 0.6 m modules: base, sink, hob, tall_unit, wall_unit picks.
--   hang = pick       on the first clear wall of `sides`.
--   one_of = { step, step }   one per building (its hash): offices are pods or cubicles, a building at a time.

local function office_kind(tags) return { family = "seating", tags = tags } end

furniture_rooms = {
  office = {
    -- the desk backs onto the window, its chair in front facing it, the monitor on top; two in a wide office
    { wall = { 1.6, 1.6 }, sides = "long", ring_sides = { 0, 1, 3 }, count = { per = 3.4, min = 1, max = 2 },
      set = {
        { pick = { family = "surface", tags = { "office" } }, x = 0.8 },
        { pick = "monitor", x = 0.8, z = 0.12, y = 0.75 },
        { pick = office_kind({ "office" }), x = 0.8, z = 1.55, faces = "wall", fit = { 0.7, 0.7 } },
      } },
    { wall = { 0.46, 0.6 }, sides = { 1, 3, 2 }, tall = true, clear = 0.6, pick = { family = "storage", tags = { "office" } } },
    { wall = { 0.9, 0.9 }, sides = { 2, 1, 3 }, min_w = 5.5, pick = { family = "decor", tags = { "plant" } } },
  },

  bedroom = {
    -- the bed, head to a wall, between two nightstands; a wardrobe on another wall
    { wall = { 2.6, 2.12 }, sides = "long", ring_sides = { 1, 3, 0 },
      set = {
        { pick = { family = "storage", tags = { "bedroom" } }, x = 0.25, fit = { 0.5, 0.5 } },
        { pick = { family = "sleeping", tags = { "bedroom" } }, x = 1.3 },
        { pick = { family = "storage", tags = { "bedroom" } }, x = 2.35, fit = { 0.5, 0.5 } },
      } },
    { wall = { 1.2, 0.6 }, sides = { 1, 3, 0, 2 }, tall = true, clear = 0.7, pick = { family = "storage", tags = { "bedroom" } } },
  },

  flat = {   -- a studio: the bed, and a sofa in place of the wardrobe
    { wall = { 2.6, 2.12 }, sides = "long", ring_sides = { 1, 3, 0 },
      set = {
        { pick = { family = "storage", tags = { "bedroom" } }, x = 0.25, fit = { 0.5, 0.5 } },
        { pick = { family = "sleeping", tags = { "bedroom" } }, x = 1.3 },
        { pick = { family = "storage", tags = { "bedroom" } }, x = 2.35, fit = { 0.5, 0.5 } },
      } },
    { wall = { 2.16, 1.75 }, sides = { 2, 0, 1, 3 },
      set = {
        { pick = { family = "seating", tags = { "living" } }, x = 1.08, fit = { 2.16, 1.0 } },
        { pick = { family = "surface", tags = { "living" } }, x = 1.08, z = 1.15, fit = { 1.2, 0.7 } },
      } },
  },

  living = {
    -- the sofa on a long wall with a rug and a coffee table before it, the TV across the room, an armchair
    { wall = { 2.16, 1.75 }, sides = "long",
      set = {
        { pick = { family = "seating", tags = { "living" } }, x = 1.08, fit = { 2.16, 1.0 } },
        { pick = { family = "decor", tags = { "living", "floor" } }, x = 1.08, z = 0.75 },
        { pick = { family = "surface", tags = { "living" } }, x = 1.08, z = 1.15, fit = { 1.2, 0.7 } },
      } },
    { wall = { 1.6, 0.42 }, sides = "opposite", tall = true, clear = 0.8, pick = { family = "storage", tags = { "living" } } },
    { wall = { 0.84, 0.84 }, sides = { 1, 3, 0, 2 }, pick = { family = "seating", tags = { "living" } } },
  },

  kitchen = {
    -- the counter run on the longest wall (fridge tower, sink, hob, a wall cupboard over each base), a table for two
    { counter = "kitchen", wall = { 0, 0.64 }, sides = "long", tall = true, clear = 0.9,
      base = "kitchen_base", sink = "kitchen_sink", hob = "kitchen_hob", tall_unit = "kitchen_tall", wall_unit = "kitchen_wall" },
    { wall = { 1.4, 1.85 }, sides = { 0, 1, 2, 3 },
      set = {
        { pick = { family = "surface", tags = { "dining" } }, x = 0.7, z = 0.5, fit = { 1.4, 0.9 } },
        { pick = { family = "seating", tags = { "dining" } }, x = 0.7, z = 0.0, fit = { 0.6, 0.6 } },
        { pick = { family = "seating", tags = { "dining" } }, x = 0.7, z = 1.85, faces = "wall", fit = { 0.6, 0.6 } },
      } },
  },

  open_plan = {
    -- benching pods of six (four where six will not go), or cubicles: one or the other, a building at a time
    { one_of = {
        { grid = { 3.1, 4.8 }, aisle = { 1.3, 1.6 }, short = 3.2, short_style = 4, pick = "desk_pod" },
        { grid = { 2.4, 2.4 }, aisle = { 0.2, 1.6 }, pick = "cubicle" },
    } },
  },

  meeting = {
    { wall = { 3.0, 2.4 }, sides = { 0, 2 }, pick = { family = "surface", tags = { "meeting" } } },
    { hang = "whiteboard", sides = { 1, 3 } },
  },

  kitchenette = {
    { counter = "kitchenette", wall = { 0, 0.64 }, sides = { 2, 0 }, tall = true,
      base = "kitchen_base", sink = "kitchen_sink", tall_unit = "kitchen_tall" },
    { wall = { 1.4, 1.85 }, sides = { 0, 1, 3 },
      set = {
        { pick = { family = "surface", tags = { "dining" } }, x = 0.7, z = 0.5, fit = { 1.4, 0.9 } },
        { pick = { family = "seating", tags = { "dining" } }, x = 0.7, z = 0.0, fit = { 0.6, 0.6 } },
        { pick = { family = "seating", tags = { "dining" } }, x = 0.7, z = 1.85, faces = "wall", fit = { 0.6, 0.6 } },
      } },
  },

  closet = {
    { wall = { 1.2, 0.5 }, sides = "long", tall = true, pick = { family = "storage", tags = { "closet" } } },
  },

  bath = {
    { wall = { 1.7, 0.75 }, sides = "long", clear = 0.6, pick = { family = "fixture", tags = { "bathroom" } } },
    { wall = { 0.4, 0.7 }, sides = { 1, 3, 0, 2 }, clear = 0.6, pick = "toilet" },
    { wall = { 0.8, 0.5 }, sides = { 0, 1, 2, 3 }, clear = 0.6, pick = "vanity" },
  },
}
