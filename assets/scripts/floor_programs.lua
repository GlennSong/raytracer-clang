-- FLOOR PROGRAMS (src/engine/procgen/city/floor_zones.h). A storey's plate is cut like a city block into lots
-- (window lots along the glass, corner lots where it turns, interior lots in the court round the core); a program
-- then tags each lot, the way the archetype book picks a lot's building.
--
-- lots       how the plate is cut (metres; m2 for interior_lot; min_access: a window lot with less than this onto
--            circulation joins its neighbour)
-- fill       the tag every lot no zone takes gets ...
-- shallow    ... or this one, where the lot's frame is shallower than shallow_depth
-- zones      tried IN ORDER; a lot one takes is not offered to the next. Each:
--   tag        its name (styles refer to it)
--   kind       the room it builds (a RoomKind: open_plan, meeting, office, kitchenette, reception, lounge, desk_row, ...;
--              defaults to the tag); the kind picks the furniture program (furniture_rooms.lua)
--   sites      which lots may take it: "window", "corner", "interior" (all three when absent)
--   count | per_m2 | share   how many: exactly count a floor; one per per_m2 of floor; that share of the lots it may take
--   area       {min, max} m2; min_depth (m) of the furniture's frame
--   enclosed   partitions round it and a door onto circulation (else an open zone, no walls)
--   walls      "glass", "drywall", or "facade" (glass behind a curtain wall, drywall in masonry; the default)
--   near       "lobby" (closest to the lifts first), "core", "far" (farthest from the lifts first); else spread out
--   lifts      only on a floor with lifts (a stair-only office has no reception)
--   min_floor  only on a floor of at least this many m2 (a small floor keeps its lots for desks)
-- styles     a building's habit, picked per building by weight: `set` overrides zones' numbers by tag.
-- residential  homes: each `flat` lot is laid out inside as a flat (living room and bedrooms on the glass, hall,
--              bath and kitchen behind; a studio where that will not walk), and a floor that will not walk is left to
--              the older apartment floors rather than opened up
--
-- Keep the programs alike with builtinOfficeProgram() and builtinApartmentProgram() (floor_zones.cpp): they stand
-- where this file is not loaded (procgen without the scripting layer, most tests).

floor_programs = {
  office = {
    lots = { window_depth = 4.5, aisle = 1.6, street = 1.8, lot_width = 4.8, interior_lot = 24, min_side = 2.6 },
    fill = "open_plan",
    shallow = "desk_row", shallow_depth = 3.2,
    zones = {
      { tag = "reception", kind = "reception", sites = {"interior", "window"}, count = 1, near = "lobby", lifts = true,
        area = {12, 60} },
      -- the corner office is the whole corner (a rounded corner's arc too)
      { tag = "corner_office", kind = "office", sites = {"corner"}, share = 1.0, enclosed = true, area = {8, 1e9} },
      { tag = "pantry", kind = "kitchenette", sites = {"interior"}, count = 1, near = "core", area = {10, 50} },
      { tag = "copy", kind = "copy_room", sites = {"interior"}, count = 1, near = "core", enclosed = true,
        walls = "drywall", area = {6, 28}, min_depth = 2.0, min_floor = 700 },
      { tag = "meeting", kind = "meeting", sites = {"interior", "window"}, per_m2 = 320, enclosed = true, walls = "glass",
        area = {10, 45}, min_depth = 3.0 },
      { tag = "office", kind = "office", sites = {"window"}, share = 0.0, enclosed = true, near = "far",
        area = {8, 30}, min_depth = 3.0 },
      -- the floor's points: a water cooler with a high table to stand at, phone booths (one row a 900 m2)
      { tag = "water_cooler", kind = "water_cooler", sites = {"interior", "window"}, count = 1, area = {6, 30},
        min_depth = 2.0, min_floor = 500 },
      { tag = "phone_booths", kind = "phone_booths", sites = {"interior"}, per_m2 = 900, area = {6, 30},
        min_depth = 2.2, min_floor = 900 },
      { tag = "lounge", kind = "lounge", sites = {"window", "interior"}, count = 1, area = {12, 1e9}, min_floor = 800 },
      { tag = "open_plan", kind = "open_plan" },
      { tag = "desk_row", kind = "desk_row" },
    },
    styles = {
      { name = "corner_offices", weight = 2 },
      { name = "perimeter_offices", weight = 1, set = { office = { share = 0.5 } } },
      { name = "open_floor", weight = 1, set = { meeting = { per_m2 = 500 } } },
    },
  },

  -- AN APARTMENT FLOOR: flats along the glass, each a lot deep enough to reach the corridor round the core; what a
  -- deep plate leaves in the middle is the residents' storage rooms
  apartments = {
    residential = true,
    -- min_access: a flat needs its hall and its bath on the corridor, so a lot narrower there joins its neighbour;
    -- min_lot_area: so does a lot too small for a flat
    lots = { window_depth = 12.0, aisle = 1.6, street = 1.8, lot_width = 8.5, interior_lot = 20, min_side = 2.6,
             min_access = 4.0, min_lot_area = 30.0 },
    fill = "landing",
    shallow = "landing", shallow_depth = 0,
    zones = {
      { tag = "flat", kind = "flat", sites = {"window", "corner"}, share = 1.0, enclosed = true, walls = "drywall",
        area = {30, 1e9}, min_depth = 4.5 },
      -- a deep plate's middle: the residents' storage rooms
      { tag = "storage", kind = "closet", sites = {"interior"}, share = 1.0, enclosed = true, walls = "drywall",
        area = {4, 40} },
      -- what is left (a storage room with no door, a lot shut off): landing, part of the corridor
      { tag = "landing", kind = "hall", walls = "drywall" },
    },
  },
}
