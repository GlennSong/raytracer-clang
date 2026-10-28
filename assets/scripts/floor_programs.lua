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
--   departments  only on a floor of one of these departments (below)
--   trade      its rooms are of this trade (a restaurant, a bar: trades.cpp's names), furnished as one
--   ensuite    an enclosed room with its own bathroom, carved beside its door (a hotel room)
-- styles     a building's habit, picked per building by weight: `set` overrides zones' numbers by tag, and the
--            tag `lots` how the plate is cut (a budget hotel's narrow rooms)
-- workplace  false: not lit as offices, its baths bathrooms rather than restrooms (a hotel)
-- departments  a shop floor's: the department each storey is, by turns from the first storey over the ground (trade
--              names, src/engine/procgen/city/trades.cpp); its rooms take the trade (a sales floor furnishes as it)
-- residential  homes: each `flat` lot is laid out inside as a flat (living room and bedrooms on the glass, hall,
--              bath and kitchen behind; a studio where that will not walk), and a floor that will not walk is not
--              opened up: it is laid as flats round the core off its lobby, or as the `residents` floor
--
-- Keep the programs alike with builtinOfficeProgram(), builtinApartmentProgram(), builtinDepartmentStoreProgram(),
-- builtinDiningProgram() and builtinHotelProgram() (floor_zones.cpp): they stand
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

  -- A DEPARTMENT STORE'S FLOOR (a tower's first storeFloors storeys, a mall's anchor): one department a floor, its
  -- sales floor open over the glass and the court; the tills by the lifts, the fitting rooms (fashion floors) and the
  -- stockroom against the core
  department_store = {
    departments = { "boutique", "boutique", "electronics", "bookshop", "bakery" },   -- the bakery: the food hall
    lots = { window_depth = 6.0, aisle = 2.4, street = 2.4, lot_width = 7.2, interior_lot = 36, min_side = 2.6 },
    fill = "sales",
    shallow = "sales", shallow_depth = 0,
    zones = {
      { tag = "cash_desk", kind = "cash_desk", sites = {"interior", "window"}, count = 1, near = "lobby",
        area = {8, 60}, min_depth = 2.6 },
      -- (on the glass where the plate leaves no court: a tower's window lots reach the corridor round its core)
      { tag = "fitting_rooms", kind = "fitting_rooms", sites = {"interior", "window"}, count = 1, near = "core",
        enclosed = true, walls = "drywall", area = {8, 50}, min_depth = 2.4, departments = { "boutique" } },
      { tag = "stockroom", kind = "stockroom", sites = {"interior", "window"}, count = 1, near = "core", enclosed = true,
        walls = "drywall", area = {8, 50}, min_depth = 2.0, min_floor = 300 },
      { tag = "sales", kind = "sales_floor" },
    },
  },

  -- A RESTAURANT FLOOR (a tower's top dineFloors): restaurants and bars by turns along the glass, each its own room
  -- off the corridor round the core (its kitchen inside, at the back); the restrooms by the core, a lounge to wait in
  dining = {
    lots = { window_depth = 7.0, aisle = 1.6, street = 1.8, lot_width = 9.0, interior_lot = 30, min_side = 2.6,
             min_access = 2.0 },
    fill = "lounge",
    shallow = "lounge", shallow_depth = 0,
    zones = {
      { tag = "restrooms", kind = "bath", sites = {"interior", "window"}, count = 1, near = "core", enclosed = true,
        walls = "drywall", area = {6, 30}, min_depth = 2.0, min_floor = 250 },
      { tag = "restaurant", kind = "shop", trade = "restaurant", sites = {"window", "corner"}, share = 0.5,
        enclosed = true, walls = "drywall", area = {25, 1e9}, min_depth = 4.5 },
      { tag = "bar", kind = "shop", trade = "bar", sites = {"window", "corner"}, share = 1.0, enclosed = true,
        walls = "drywall", area = {20, 1e9}, min_depth = 3.5 },
      { tag = "lounge", kind = "lounge" },
    },
  },

  -- A HOTEL'S FLOOR: rooms along the glass off the corridor round the core, each with its own bathroom by its door
  -- (ensuite), suites at the corners, housekeeping's linen room by the lifts. Its class is the building's style: a
  -- budget hotel's rooms narrow and no suites, a business hotel's as set here, a luxury one's wide with more suites.
  hotel = {
    workplace = false,
    lots = { window_depth = 6.4, aisle = 1.6, street = 1.8, lot_width = 4.2, interior_lot = 24, min_side = 2.6,
             min_access = 1.6, min_lot_area = 14 },
    fill = "hall",
    shallow = "hall", shallow_depth = 0,
    zones = {
      { tag = "linen", kind = "linen_room", sites = {"interior", "window"}, count = 1, near = "lobby", enclosed = true,
        walls = "drywall", area = {6, 30}, min_depth = 2.0 },
      { tag = "suite", kind = "hotel_suite", sites = {"corner"}, share = 1.0, enclosed = true, walls = "drywall",
        area = {28, 1e9}, min_depth = 4.8, ensuite = true },
      { tag = "window_suite", kind = "hotel_suite", sites = {"window"}, share = 0.0, enclosed = true, walls = "drywall",
        area = {26, 1e9}, min_depth = 4.8, ensuite = true },
      { tag = "room", kind = "hotel_room", sites = {"window", "corner"}, share = 1.0, enclosed = true, walls = "drywall",
        area = {14, 1e9}, min_depth = 4.6, ensuite = true },
      { tag = "hall", kind = "hall" },
    },
    styles = {
      { name = "budget", weight = 1, set = { lots = { lot_width = 3.6, window_depth = 5.6 }, suite = { share = 0 } } },
      { name = "business", weight = 2 },
      { name = "luxury", weight = 1, set = { lots = { lot_width = 5.2, window_depth = 7.2 }, window_suite = { share = 0.25 } } },
    },
  },

  -- A RESIDENTS' FLOOR: an apartment tower's floor too small for flats even round the core off its lobby (a mid-rise
  -- whose lift core leaves 2-3 m of floor) is the residents' lounge, open round the core, reached from the lifts
  residents = {
    fill = "lounge",
    shallow = "lounge", shallow_depth = 0,
    zones = {
      { tag = "lounge", kind = "lounge" },
    },
  },
}
