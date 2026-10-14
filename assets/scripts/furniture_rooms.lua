-- furniture_rooms.lua -- ROOM PROGRAMS (the furniture library, M3b; Glenn, 2026-10-03: "go ahead with the furniture
-- tags"). What a room of each kind holds, as steps run in order. Loaded with furniture_library.lua; the floor plans
-- (room_plan.cpp) name the kinds: office, bedroom, flat, living, kitchen, open_plan, meeting, kitchenette, closet,
-- bath (restroom outside a home), desk_row, lounge, reception, gallery, ward, exam, nurses_station, waiting, nave,
-- foyer, auditorium, market, cell, courtroom, council_chamber, engine_bay, shop_floor, warehouse. Shops are still furnished in code, by their trade.
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
--   grid = { w, d }   cells in rows out from the window wall (the first `from` metres off it, default 0.6; the last
--                     `to` short of the far wall, default 0.8; at most `rows` rows):
--                     aisle = { x, z }; short = a shallower cell where a full one will not fit, built with
--                     short_style (the piece's style bits).
--   counter = "kitchen" | "kitchenette"   a run of 0.6 m modules: base, sink, hob, tall_unit, wall_unit picks.
--   hang = pick       on the first clear wall of `sides` (`any_wall`: an outside wall too, over its windows).
--   one_of = { step, step }   one per building (its hash): offices are pods or cubicles, a building at a time -- the
--                     others in turn where the building's choice fits nothing; `fallback = step` beside it where
--                     none of them does.

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
      },
      -- a zone too shallow for either: rows of desks at the glass
      fallback = { grid = { 1.6, 1.6 }, from = 0.15, aisle = { 0.2, 1.2 },
        set = {
          { pick = "desk_screen", x = 0.03 },
          { pick = { family = "surface", tags = { "office" } }, x = 0.8 },
          { pick = "monitor", x = 0.8, z = 0.12, y = 0.75 },
          { pick = office_kind({ "office" }), x = 0.8, z = 1.55, faces = "wall", fit = { 0.7, 0.7 } },
        } } },
  },

  desk_row = {
    -- an office floor too shallow for rooms (a setback tier round its base's core): a desk at every window, the chair
    -- on the room side facing out, the walk to the core behind; a low screen between each desk and the next
    { grid = { 1.6, 1.6 }, from = 0.15, aisle = { 0.2, 9.0 },
      set = {
        { pick = "desk_screen", x = 0.03 },
        { pick = { family = "surface", tags = { "office" } }, x = 0.8 },
        { pick = "monitor", x = 0.8, z = 0.12, y = 0.75 },
        { pick = office_kind({ "office" }), x = 0.8, z = 1.55, faces = "wall", fit = { 0.7, 0.7 } },
      } },
  },

  lounge = {
    -- a residential tower's floor too shallow for flats: the residents' lounge, a sofa and its table at every few
    -- windows, facing in, the walk to the core behind
    { grid = { 2.4, 1.9 }, from = 0.15, aisle = { 1.4, 9.0 },
      set = {
        { pick = { family = "seating", tags = { "living" } }, x = 1.2, fit = { 2.16, 1.0 } },
        { pick = { family = "surface", tags = { "living" } }, x = 1.2, z = 1.15, fit = { 1.2, 0.7 } },
      } },
  },

  reception = {
    -- an office floor's front desk, its back to the screen, facing the lifts across the corridor: the receptionist's
    -- chair behind it; a chair for visitors at each end of the room, a plant
    { wall = { 2.4, 1.6 }, sides = { 0 },
      set = {
        { pick = "office_chair", x = 1.2, z = 0.05, fit = { 0.7, 0.7 } },
        { pick = "reception_desk", x = 1.2, z = 0.75 },
      } },
    { wall = { 0.84, 0.84 }, sides = { 1 }, pick = "lounge_chair" },
    { wall = { 0.84, 0.84 }, sides = { 3 }, pick = "lounge_chair" },
    { wall = { 0.9, 0.9 }, sides = { 3, 1 }, pick = { family = "decor", tags = { "plant" } } },
  },

  gallery = {
    -- a museum's room: pictures round the walls (never over a window), glass display cases down the middle, a bench
    { grid = { 1.6, 0.7 }, from = 2.2, aisle = { 1.8, 2.0 }, pick = "display_case" },
    { wall = { 1.8, 0.6 }, sides = { 2, 1, 3 }, clear = 1.0, pick = "bench" },
    { hang = "picture", sides = { 1, 3, 2, 0 } },
    { hang = "picture", sides = { 3, 1, 2, 0 } },
    { hang = "picture", sides = { 2, 1, 3, 0 } },
    { hang = "picture", sides = { 1, 3, 2, 0 } },
    { hang = "picture", sides = { 3, 1, 2, 0 } },
    { hang = "picture", sides = { 2, 0, 1, 3 } },
  },

  -- THE HOSPITAL (hospitalPlan, room_plan.cpp)
  ward = {
    -- a patients' room: a hospital bed head to a side wall with its bedside cabinet, a second across from it in a room
    -- wide enough; a chair for visitors by the window
    { wall = { 1.6, 2.3 }, sides = { 1, 3 }, clear = 0.8,
      set = {
        { pick = "hospital_bed", x = 0.55 },
        { pick = "nightstand", x = 1.33, fit = { 0.5, 0.5 } },
      } },
    { wall = { 1.6, 2.3 }, sides = "opposite", clear = 0.8, min_w = 4.4,
      set = {
        { pick = "hospital_bed", x = 0.55 },
        { pick = "nightstand", x = 1.33, fit = { 0.5, 0.5 } },
      } },
    { wall = { 0.84, 0.84 }, sides = { 0, 2 }, pick = "lounge_chair" },
  },

  exam = {
    -- an examination room: the couch along a side wall, the doctor's desk at the window, a basin by the door
    { wall = { 2.0, 0.8 }, sides = { 1, 3 }, clear = 0.7, pick = "exam_couch" },
    { wall = { 1.6, 1.6 }, sides = { 0, 2 },
      set = {
        { pick = { family = "surface", tags = { "office" } }, x = 0.8 },
        { pick = "monitor", x = 0.8, z = 0.12, y = 0.75 },
        { pick = office_kind({ "office" }), x = 0.8, z = 1.55, faces = "wall", fit = { 0.7, 0.7 } },
      } },
    { wall = { 0.8, 0.5 }, sides = { 2, 3, 1 }, clear = 0.6, pick = "vanity" },
  },

  nurses_station = {
    -- the counter facing the ward corridor, two nurses' chairs behind it, the records cabinets on the end walls
    { wall = { 2.4, 1.6 }, sides = { 2 },
      set = {
        { pick = "office_chair", x = 0.6, z = 0.05, fit = { 0.7, 0.7 } },
        { pick = "office_chair", x = 1.8, z = 0.05, fit = { 0.7, 0.7 } },
        { pick = "reception_desk", x = 1.2, z = 0.75 },
      } },
    { wall = { 0.46, 0.6 }, sides = { 1, 3 }, tall = true, clear = 0.6, pick = { family = "storage", tags = { "office" } } },
  },

  waiting = {
    -- the entrance lobby: the information desk facing the doors, rows of waiting seats between, plants
    { wall = { 2.4, 1.6 }, sides = { 2 },
      set = {
        { pick = "office_chair", x = 1.2, z = 0.05, fit = { 0.7, 0.7 } },
        { pick = "reception_desk", x = 1.2, z = 0.75 },
      } },
    { grid = { 2.4, 0.8 }, from = 3.0, aisle = { 1.4, 1.0 }, pick = "waiting_seats" },
    { wall = { 0.9, 0.9 }, sides = { 1, 3 }, pick = { family = "decor", tags = { "plant" } } },
    { wall = { 0.9, 0.9 }, sides = { 3, 1 }, pick = { family = "decor", tags = { "plant" } } },
  },

  -- THE CIVIC HALLS (hallPlan / campusPlan, room_plan.cpp): side 0 is the street side or the public end, side 2 the
  -- far wall -- the altar's, the screen's, the bench's
  nave = {
    -- a church: the altar and the lectern on the far wall, pews in blocks either side of the aisles facing them
    { wall = { 2.4, 1.4 }, sides = { 2 }, pick = "altar" },
    { wall = { 0.8, 0.7 }, sides = { 2 }, pick = "lectern" },
    { grid = { 3.0, 0.75 }, from = 2.2, to = 4.5, aisle = { 2.2, 0.3 }, pick = "pew" },
    { wall = { 0.9, 0.9 }, sides = { 2, 1, 3 }, pick = { family = "decor", tags = { "plant" } } },
  },

  foyer = {
    -- a cinema's: the ticket desk facing the doors, the concession (a counter and its fridges), seats to wait on
    { wall = { 2.4, 1.6 }, sides = { 2, 1 },
      set = {
        { pick = "office_chair", x = 1.2, z = 0.05, fit = { 0.7, 0.7 } },
        { pick = "reception_desk", x = 1.2, z = 0.75 },
      } },
    { wall = { 1.8, 0.7 }, sides = { 1, 3, 2 }, clear = 1.0, pick = "shop_counter" },
    { wall = { 0.9, 0.7 }, sides = "opposite", tall = true, clear = 0.8, pick = "drinks_fridge" },
    { grid = { 2.4, 0.8 }, from = 2.0, aisle = { 1.6, 1.4 }, rows = 2, pick = "waiting_seats" },
    { hang = "picture", sides = { 1, 3, 2 } },
    { hang = "picture", sides = { 3, 1, 2 } },
  },

  auditorium = {
    -- the screen high on the far wall; rows of six on their risers raking up toward the door (the back), an aisle
    -- stair beside every block of rows
    { hang = "cinema_screen", sides = { 2 }, any_wall = true },
    { grid = { 4.3, 1.0 }, from = 0.6, to = 4.0, aisle = { 0.0, 0.0 }, tiers = true,
      set = {
        { pick = "aisle_step", x = 0.45 },
        { pick = "cinema_row", x = 2.6 },
      } },
  },

  market = {
    -- a market hall: stalls in rows, aisles between to walk and shop
    { grid = { 2.4, 1.4 }, from = 2.0, aisle = { 1.2, 2.4 }, pick = "market_stall" },
  },

  cell = {
    -- a police cell: a bunk along a side wall, a steel toilet in the corner
    { wall = { 0.9, 2.0 }, sides = { 1, 3 }, pick = "single_bed" },
    { wall = { 0.5, 0.7 }, sides = { 0, 3, 1 }, pick = "toilet" },
  },

  courtroom = {
    -- the bench across the far end (the judge, the witness box, the clerk), the jury box on a side wall, counsel's
    -- tables facing the bench, the public's benches at the back by the door
    { wall = { 4.0, 2.2 }, sides = { 2 }, pick = "judge_bench" },
    { wall = { 3.6, 2.0 }, sides = { 3, 1 }, pick = "jury_box" },
    { grid = { 3.0, 0.75 }, from = 0.6, aisle = { 1.2, 0.3 }, rows = 4, pick = "pew" },
    { grid = { 2.4, 1.6 }, from = 5.2, aisle = { 1.6, 0.0 }, rows = 1,
      set = {
        { pick = { family = "surface", tags = { "office" } }, x = 1.2, z = 0.75, fit = { 1.6, 0.8 } },
        { pick = office_kind({ "office" }), x = 0.7, z = 0.05, fit = { 0.7, 0.7 } },
        { pick = office_kind({ "office" }), x = 1.7, z = 0.05, fit = { 0.7, 0.7 } },
      } },
  },

  council_chamber = {
    -- the dais across the far end, the councillors' desks in rows facing it, the public at the back
    { wall = { 4.0, 2.2 }, sides = { 2 }, pick = "judge_bench" },
    { grid = { 3.0, 0.75 }, from = 0.6, aisle = { 1.2, 0.3 }, rows = 3, pick = "pew" },
    { grid = { 1.6, 1.6 }, from = 4.6, aisle = { 0.4, 0.6 },
      set = {
        { pick = { family = "surface", tags = { "office" } }, x = 0.8, z = 0.75, fit = { 1.6, 0.8 } },
        { pick = office_kind({ "office" }), x = 0.8, z = 0.05, fit = { 0.7, 0.7 } },
      } },
  },

  engine_bay = {
    -- a fire station's: the crew's gear lockers along the side walls, a workbench and a locker at the back; the floor
    -- behind each open bay door is its engine's (the city sim parks it there) and stays clear
    { grid = { 0.9, 0.6 }, from = 11.0, aisle = { 0.05, 1.8 }, rows = 1, run = { 6, 2.0 }, pick = "gear_locker" },
    { wall = { 0.9, 0.6 }, sides = { 1, 3 }, tall = true, clear = 0.8, count = { per = 4.0, min = 1, max = 4 }, pick = "gear_locker" },
    { wall = { 2.4, 0.75 }, sides = { 2 }, clear = 0.8, pick = { family = "surface", tags = { "lab" } } },
    { wall = { 0.46, 0.6 }, sides = { 2 }, tall = true, clear = 0.6, pick = { family = "storage", tags = { "office" } } },
  },

  shop_floor = {
    -- a factory: workbenches along the walls, the machines in rows down the floor (aisles to walk and wheel between),
    -- pallets of stock and the forklift by the loading bays
    { wall = { 1.3, 3.0 }, sides = { 3, 1 }, clear = 1.5, pick = "forklift" },
    { wall = { 1.2, 1.0 }, sides = { 1, 3 }, clear = 1.2, count = { per = 6.0, min = 1, max = 3 }, pick = "pallet_stack" },
    { wall = { 2.0, 0.8 }, sides = { 1, 3, 2 }, clear = 1.0, count = { per = 3.0, min = 2, max = 6 }, pick = "workbench" },
    { grid = { 2.6, 1.6 }, from = 6.0, aisle = { 1.6, 2.6 }, run = { 3, 3.0 }, pick = "machine" },
  },

  warehouse = {
    -- a warehouse: double-sided pallet racking in runs down the floor, wide aisles for the forklift, pallets waiting
    -- by the loading bays
    { grid = { 2.8, 2.2 }, from = 6.0, aisle = { 0.0, 3.2 }, run = { 4, 3.2 }, pick = "pallet_rack" },
    { wall = { 1.2, 1.0 }, sides = { 1, 3 }, clear = 1.2, count = { per = 4.0, min = 2, max = 5 }, pick = "pallet_stack" },
    { wall = { 1.3, 3.0 }, sides = { 3, 1 }, clear = 1.5, pick = "forklift" },
    { wall = { 2.0, 0.8 }, sides = { 2, 1, 3 }, clear = 1.0, pick = "workbench" },
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

  restroom = {
    -- a bath room in a school, a museum, an office: a row of toilets down a long wall, basins across from them
    { wall = { 0.9, 0.7 }, sides = "long", clear = 0.6, count = { per = 1.0, min = 2, max = 5 }, pick = "toilet" },
    { wall = { 0.9, 0.5 }, sides = "opposite", clear = 0.6, count = { per = 1.2, min = 1, max = 3 }, pick = "vanity" },
  },

  bath = {
    { wall = { 1.7, 0.75 }, sides = "long", clear = 0.6, pick = { family = "fixture", tags = { "bathroom" } } },
    { wall = { 0.4, 0.7 }, sides = { 1, 3, 0, 2 }, clear = 0.6, pick = "toilet" },
    { wall = { 0.8, 0.5 }, sides = { 0, 1, 2, 3 }, clear = 0.6, pick = "vanity" },
  },
}

-- THE UNIVERSITY (campusPlan, room_plan.cpp): the corridor wall (side 2) is each room's front -- the door, the
-- whiteboard, the lectern -- and the seats face it, rows out from the windows.
furniture_rooms.classroom = {
  { wall = { 1.6, 0.8 }, sides = { 2 }, pick = { family = "surface", tags = { "office" } } },   -- the teacher's desk
  { hang = "whiteboard", sides = { 2, 1, 3 } },
  -- a desk and its chair a cell: the pupil sits on the window side of the desk, facing the front
  { grid = { 0.9, 1.25 }, aisle = { 0.35, 0.25 },
    set = {
      { pick = { family = "seating", tags = { "classroom" } }, x = 0.45, z = 0.0 },
      { pick = { family = "surface", tags = { "classroom" } }, x = 0.45, z = 1.05, faces = "wall" },
    } },
}

furniture_rooms.lecture_hall = {
  -- THE STAGE (Glenn: "more space for the lecturn and maybe a table next to the lecturn"): a 6.4 x 3.2 m front kept
  -- clear of seats, the lectern off-centre, a table beside it; the big board behind (style 4: 4 m wide)
  { wall = { 6.4, 3.2 }, sides = { 2 },
    set = {
      { pick = "lectern", x = 2.0, z = 0.9 },
      { pick = { family = "surface", tags = { "office" } }, x = 4.3, z = 0.7, fit = { 1.6, 0.8 } },
    } },
  { hang = "whiteboard", style = 4, sides = { 2, 1, 3 } },
  -- rows of six on their risers, stepping up toward the windows (the back); beside every block of rows its aisle
  -- STAIR, two steps a tier, so each row is reached on foot
  { grid = { 4.3, 1.0 }, aisle = { 0.0, 0.0 }, tiers = true,
    set = {
      { pick = "aisle_step", x = 0.45 },
      { pick = { family = "seating", tags = { "lecture" } }, x = 2.6 },
    } },
}

furniture_rooms.lab = {
  { hang = "whiteboard", sides = { 2, 1, 3 } },
  { grid = { 2.4, 0.75 }, aisle = { 1.0, 1.5 }, pick = { family = "surface", tags = { "lab" } } },
}

furniture_rooms.reading_room = {
  { wall = { 1.2, 0.35 }, sides = { 1, 3 }, tall = true, count = 6, pick = { family = "storage", tags = { "books" } } },
  { grid = { 2.6, 2.0 }, aisle = { 1.4, 1.2 }, pick = { family = "surface", tags = { "library" } } },
}

furniture_rooms.stacks = {
  -- shelving round the walls, then long rows of bookcases BACK TO BACK (books out to both aisles), a cross aisle
  -- every five
  { wall = { 1.2, 0.35 }, sides = { 1, 3, 0 }, tall = true, count = 8, pick = { family = "storage", tags = { "books" } } },
  { grid = { 1.2, 0.72 }, aisle = { 0.0, 1.2 }, run = { 5, 1.4 },
    set = {
      { pick = { family = "storage", tags = { "books" } }, x = 0.6, z = 0.36, faces = "wall", fit = { 1.2, 0.36 } },
      { pick = { family = "storage", tags = { "books" } }, x = 0.6, z = 0.36, fit = { 1.2, 0.36 } },
    } },
}

furniture_rooms.dorm = {
  -- two singles, their heads to the window wall, a desk and chair on each side wall, wardrobes by the door
  { wall = { 3.2, 2.1 }, sides = { 0 },
    set = {
      { pick = { family = "sleeping", tags = { "dorm" } }, x = 0.5, fit = { 1.0, 2.1 } },
      { pick = { family = "sleeping", tags = { "dorm" } }, x = 2.7, fit = { 1.0, 2.1 } },
    } },
  { wall = { 0.9, 1.2 }, sides = { 1, 3 }, count = 2,
    set = {
      { pick = { family = "surface", tags = { "classroom" } }, x = 0.45, z = 0.0 },
      { pick = { family = "seating", tags = { "classroom" } }, x = 0.45, z = 1.15, faces = "wall" },
    } },
  { wall = { 1.2, 0.6 }, sides = { 1, 3 }, count = 2, tall = true, clear = 0.7, pick = { family = "storage", tags = { "dorm" } } },
}
-- (a DORM is its own room kind and program, apart from a home's bedroom: its pieces are picked by the "dorm" tag --
-- singles, not the double; a wardrobe each -- so the two can change independently)
