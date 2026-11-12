-- signs_us.lua -- THE US ROAD SIGNS, as data (src/engine/procgen/world/sign_kit.h; the recipes are sign_kit.lua).
--
-- Every sign the driving model's rules will plant (#240, #241): its name, the US manual's code for it, what regulation
-- or control it stands for (`means`, in the control plan's words, so the planting can find the sign for a
-- regulation), its plate (shape, size, colours, border) and its legend (lettering and symbols drawn with the kit).
-- A NEW SIGN IS A NEW ENTRY HERE: no C++ changes.
--
-- SOURCE. The US sign manual: "Manual on Uniform Traffic Control Devices for Streets and Highways" (the MUTCD), 2009
-- edition with Revisions 1 and 2, US Federal Highway Administration -- a US government work, public domain. Used:
--   Table 2B-1 (regulatory sign sizes, conventional road column), Table 2C-2 (warning sign sizes), Section 2A.18
--   (mounting height), Section 2A.11 (sign colours by use), Section 2D.43 (street name signs).
-- The OUTER sizes and the colours' uses are the manual's. The INSIDE of each face (letter heights, border widths,
-- where a symbol sits) follows the proportions of the FHWA "Standard Highway Signs" drawings as best read, to about an
-- inch, and is not checked against them line by line. The symbols are our own drawings of the manual's figures. The
-- lettering is Overpass Bold (SIL OFL, already credited), a typeface based on the US highway alphabets; the manual's
-- letter series B..F are approximated by narrowing it (`series`).
--
-- SIDES. The engine drives on the LEFT (`kerb_side = "left"`: the kerb is on the driver's left). A request names a
-- turn as "near" (toward the kerb) or "far", never left or right; the sign shows whichever that is here.

local kit = require "sign_kit"
local IN, FT = kit.IN, kit.FT

local C = {}
C.kit = kit
C.kerb_side = "left"

-- COLOURS: sRGB stand-ins for the manual's sign colours (their exact limits are chromaticity boxes in 23 CFR 655
-- Subpart F, Appendix; these are picked inside them by eye, not measured).
C.palette = {
  white = "#f5f5f2", red = "#b81c22", yellow = "#f5c400", fyg = "#c9e01a",   -- fyg: fluorescent yellow-green
  green = "#00693f", blue = "#1d4a8f", black = "#141414", orange = "#f06a1d", brown = "#5a3a22",
}

-- NIGHT SHINE: how strongly each colour's sheeting returns headlight to the driver, as a share of white's. The numbers
-- are the minimum new-sheeting coefficients of ASTM D4956 Type III (cd/lx/m2 at 0.2 deg observation, -4 deg entrance:
-- white 250, yellow 170, orange 100, green 45, red 45, blue 20, brown 12) over white's; fluorescent yellow-green is an
-- estimate; black legends are not reflective. `ra_white` is white's own, for a renderer that wants the unit.
C.sheeting = {
  ra_white = 250,
  white = 1.0, yellow = 0.68, orange = 0.40, green = 0.18, red = 0.18, blue = 0.08, brown = 0.05, fyg = 0.80, black = 0.0,
}

-- LETTER SERIES: the manual's alphabets B (narrowest) to F, as a share of the sign font's width. `squeeze_floor`: a line
-- too long for its sign is set no narrower than this share of its series before it is set smaller.
C.series = { B = 0.70, C = 0.80, D = 0.90, E = 1.0, F = 1.08, squeeze_floor = 0.8 }

-- WEAR: what wear 1 does (sign_kit.h SignWearLook, and the plate's bend and the post's lean here).
C.wear_look = {
  fade = 0.55, fade_grey = 0.5, fade_lift = 0.35, fade_cell = 0.25,   -- sunlight: colours go greyer and paler, in patches
  dirt = 0.55, dirt_cell = 0.05, streak = 4.0, dirt_low = 0.6,        -- grime: streaks, more toward the bottom
  dirt_color = { 0.32, 0.29, 0.24 },
  shine_loss = 0.5,                                                     -- old sheeting returns half the light
  bend_bow = 0.03, fold_deg = 28, fold_chance = 0.6,                    -- metres of bow; a corner folded (toward the reader)
  hold_margin = 0.02,                                                   -- metres either side of a bolted strip that stay flat
  lean_deg = 6,                                                         -- a knocked post
  metal_rough = 0.12, metal_darken = 0.25,   -- [~] old metal: this much rougher, and this share darker (oxide, grime)
}

-- MATERIALS: what each part of a sign is made of, in the engine's own material terms (the numbers a part carries to
-- the renderer: proc_model.h ProcMaterial's metallic and roughness, with the base colour, which procgen parts carry as
-- their vertex colour). Every part of a model names one of these. A dull metal is fully metallic with a HIGH roughness:
-- the colour comes from the metal's own reflectance, and the roughness spreads the highlight wide and soft.
C.materials = {
  -- the reflective sheeting the face is printed on: a plastic film, so not a metal; a soft sheen. Its colours are the
  -- face texture (base colour white = the texture as drawn)
  face = { color = { 1, 1, 1 }, metallic = 0.0, roughness = 0.55,
           source = "[~] sheeting is a polymer film (non-metal); roughness by eye for a satin sheen" },
  -- the plain back and edge of the plate: mill-finish aluminium sheet, dull grey, not brushed or polished
  plate_back = { color = { 0.80, 0.81, 0.82 }, metallic = 1.0, roughness = 0.62,
                 source = "colour: aluminium's reflectance (~0.91 linear, published PBR tables) dulled toward oxide grey [~];"
                          .. " roughness [~] by eye for mill finish" },
  -- the posts, clamps, mast arms and spans: hot-dip galvanized steel, a darker matte zinc grey
  post = { color = { 0.62, 0.63, 0.64 }, metallic = 1.0, roughness = 0.72,
           source = "colour: zinc's reflectance (~0.66-0.85 linear, published PBR tables) weathered toward grey [~];"
                    .. " roughness [~] by eye for aged galvanizing" },
}

-- THE PLATE: aluminium sheet, its corners rounded
C.plate = {
  thick = 0.002,           -- metres (0.080 in sheet)
  corner = 1.5,            -- inches: corner radius where an entry gives none
  segs = 3, far_segs = 1,  -- steps per rounded corner, near and far
  bleed = 0.3,             -- inches: the face's colour drawn past the plate's edge, so filtering never shows its edge
}

-- MOUNTING (MUTCD 2A.18): a roadside sign's bottom 7 ft over the road where people park or walk (urban), a plaque below
-- another may hang 1 ft lower; overhead signs (on a mast arm or a span) clear the road by 17 ft.
C.mounts = {
  post = { bottom = 7 * FT, plaque_drop = 1 * FT, size = 2 * IN, clamps = true },          -- 2 in square steel tube
  two_post = { bottom = 7 * FT, size = 4 * IN, at = 0.28, from_width = 1.6 },             -- panels wider than 1.6 m
  mast_arm = { clearance = 17 * FT, reach = 3.2, pole_r = 0.13, arm_r = 0.07, sides = 16, far_sides = 8 },
  overhead = { clearance = 17 * FT, span = 12.0, upright_r = 0.16, chord_r = 0.07, sides = 16, far_sides = 8 },
}

-- ===================================================================================================================
-- THE SIGNS
--   name, code, title, means      what it is and the regulation it stands for
--   shape, size (inches), corner  its plate (sign_kit.lua outlines); size may be a function(req, C) -> {w, h}
--   plate, border                 the plate's colour; a border band {color, width, inset} inside its edge
--   legend(f, req, C)             what is drawn on it (f: the face builder, inches round the centre, y up)
--   label(req, C)                 its name with its parameters (default: title (code))
--   mount                         how it stands unless the request says ("post", "two_post", "mast_arm", "overhead")
--   plaque                        a small plate hung under another sign
--   examples                      requests that show it (the gallery)
-- ===================================================================================================================

local function turnWord(dir) return dir == "left" and "LEFT" or "RIGHT" end

C.signs = {
  {
    name = "stop", code = "R1-1", title = "STOP", means = { control = "stop", regulation = "stop" },
    shape = "octagon", size = { 30 }, corner = 1.0, plate = "red", border = { color = "white", width = 0.75, inset = 0.5 },
    legend = function(f) f:text("STOP", { cap = 10, series = "C", color = "white", fit = 22 }) end,
    examples = { {}, { plaques = { { sign = "all_way" } } } },
  },
  {
    name = "all_way", code = "R1-3P", title = "ALL WAY", means = { control = "all_way_stop" }, plaque = true,
    shape = "rect", size = { 18, 6 }, corner = 0.75, plate = "red", border = { color = "white", width = 0.375, inset = 0.375 },
    legend = function(f) f:text("ALL WAY", { cap = 3.5, series = "C", color = "white", fit = 15 }) end,
  },
  {
    name = "yield", code = "R1-2", title = "YIELD", means = { control = "give_way", regulation = "give_way" },
    shape = "triangle_down", size = { 36 }, corner = 2.0, plate = "red",
    legend = function(f)
      local ring = kit.shapes.triangle_down(36)
      f:poly({ kit.inset(ring, 4.0, 2.0, 3) }, "white")
      f:text("YIELD", { y = 6.0, cap = 5, series = "C", color = "red", fit = 13 })
    end,
  },
  {
    name = "speed_limit", code = "R2-1", title = "SPEED LIMIT", means = { regulation = "speed_limit", value = "mph" },
    shape = "rect", size = { 24, 30 }, plate = "white", border = { color = "black", width = 0.625, inset = 0.375 },
    legend = function(f, req)
      f:text("SPEED", { y = 10.0, cap = 4, series = "D" })
      f:text("LIMIT", { y = 4.6, cap = 4, series = "D" })
      f:text(tostring(req.value or 25), { y = -5.6, cap = 10, series = "E", fit = 20 })
    end,
    label = function(req) return "SPEED LIMIT " .. tostring(req.value or 25) .. " (R2-1)" end,
    examples = { { value = 25 }, { value = 65 } },
  },
  {
    name = "one_way", code = "R6-1", title = "ONE WAY", means = { regulation = "one_way" },
    shape = "rect", size = { 36, 12 }, corner = 0.75, plate = "black", border = { color = "white", width = 0.375, inset = 0.25 },
    -- `arrow`: which way the traffic goes as the reader sees it, "left" or "right" (it is the street's direction, not a side)
    legend = function(f, req)
      local d = kit.sx(req.arrow or "right")
      kit.symbols.arrow_path(f, { { -d * 15.5, 0 }, { d * 15.5, 0 } }, { shaft = 5.6, head_w = 9.6, head_l = 7.0, color = "white" })
      f:text("ONE WAY", { x = -d * 3.4, cap = 3.6, series = "C", fit = 19 })
    end,
    label = function(req) return "ONE WAY " .. string.upper(req.arrow or "right") .. " (R6-1)" end,
    examples = { { arrow = "left" }, { arrow = "right" } },
  },
  {
    name = "do_not_enter", code = "R5-1", title = "DO NOT ENTER", means = { regulation = "no_entry" },
    shape = "rect", size = { 30, 30 }, corner = 1.875, plate = "white",
    legend = function(f)
      f:disc(0, 0, 14, "red")
      f:poly({ kit.shapes.rect(22, 4.5) }, "white")
      f:text("DO NOT", { y = 6.4, cap = 3.8, series = "C", color = "white" })
      f:text("ENTER", { y = -6.4, cap = 3.8, series = "C", color = "white" })
    end,
    mount = "post",
  },
  {
    name = "wrong_way", code = "R5-1a", title = "WRONG WAY", means = { regulation = "no_entry", warns = "wrong_way" },
    shape = "rect", size = { 36, 24 }, plate = "red", border = { color = "white", width = 0.75, inset = 0.5 },
    legend = function(f)
      f:text("WRONG", { y = 4.8, cap = 6, series = "C", color = "white" })
      f:text("WAY", { y = -4.8, cap = 6, series = "C", color = "white" })
    end,
  },
  {
    -- NO LEFT TURN (R3-2), NO RIGHT TURN (R3-1), NO U-TURN (R3-4): `turn` = "near", "far" or "u"; `toward` = which way a
    -- U-turn goes round ("far" by default: across the traffic)
    name = "no_turn", title = "NO TURN", means = { regulation = "no_turn", movement = "turn" },
    shape = "rect", size = { 24, 24 }, plate = "white", border = { color = "black", width = 0.625, inset = 0.375 },
    legend = function(f, req, C)
      local o = { shaft = 3.0, head_w = 7.4, head_l = 4.4, color = "black" }
      if (req.turn or "near") == "u" then
        local dir = kit.side(req.toward or "far", C.kerb_side)
        local d = kit.sx(dir)
        o.x, o.y0, o.y, o.r, o.y_tip, o.dir = -d * 3.6, -7.5, 1.2, 3.6, -3.6, dir
        kit.symbols.arrow_uturn(f, o)
      else
        local dir = kit.side(req.turn or "near", C.kerb_side)
        local d = kit.sx(dir)
        o.x, o.y0, o.y, o.r, o.reach, o.dir = -d * 3.8, -8.0, -0.4, 3.4, 11.0, dir
        kit.symbols.arrow_turn(f, o)
      end
      kit.symbols.prohibit(f, { d = 21, width = 2.4 })
    end,
    label = function(req, C)
      if (req.turn or "near") == "u" then return "NO U-TURN (R3-4)" end
      local dir = kit.side(req.turn or "near", C.kerb_side)
      return "NO " .. turnWord(dir) .. " TURN (" .. (dir == "left" and "R3-2" or "R3-1") .. ")"
    end,
    examples = { { turn = "near" }, { turn = "far" }, { turn = "u" } },
  },
  {
    -- LANE-USE (R3-5: one movement, with ONLY; R3-6: two from one lane). `moves`: a list of "straight", "near", "far"
    name = "lane_use", title = "LANE USE", means = { regulation = "lane_use" },
    shape = "rect", size = { 30, 36 }, plate = "white", border = { color = "black", width = 0.75, inset = 0.5 },
    legend = function(f, req, C)
      local moves = req.moves or { "near" }
      local o = { shaft = 4.2, head_w = 10.0, head_l = 6.0, color = "black" }
      local turn
      for _, m in ipairs(moves) do if m ~= "straight" then turn = kit.side(m, C.kerb_side) end end
      local single = #moves == 1
      local top = single and 13.5 or 14.5
      local base = single and -3.5 or -13.0
      local x = turn and -kit.sx(turn) * (single and 5.0 or 4.0) or 0
      for _, m in ipairs(moves) do
        if m == "straight" then
          kit.symbols.arrow_straight(f, { x = x, y0 = base, y1 = top, shaft = o.shaft, head_w = o.head_w, head_l = o.head_l, color = "black" })
        else
          local t = { shaft = o.shaft, head_w = o.head_w, head_l = o.head_l, color = "black", x = x, y0 = base,
                      y = single and 3.5 or -1.0, r = 4.5, reach = 13.5, dir = turn }
          kit.symbols.arrow_turn(f, t)
        end
      end
      if single then f:text("ONLY", { y = -10.5, cap = 6, series = "C" }) end
    end,
    label = function(req, C)
      local moves = req.moves or { "near" }
      local words = {}
      for _, m in ipairs(moves) do words[#words + 1] = m == "straight" and "STRAIGHT" or turnWord(kit.side(m, C.kerb_side)) end
      return table.concat(words, " + ") .. (#moves == 1 and " ONLY (R3-5)" or " (R3-6)")
    end,
    examples = { { moves = { "near" } }, { moves = { "far" } }, { moves = { "straight", "near" } } },
  },
  {
    -- KEEP LEFT (R4-8) / KEEP RIGHT (R4-7): pass the island on the `side` side ("near" by default)
    name = "keep_side", title = "KEEP", means = { regulation = "keep_side" },
    shape = "rect", size = { 24, 30 }, plate = "white", border = { color = "black", width = 0.625, inset = 0.375 },
    legend = function(f, req, C)
      kit.symbols.keep_side(f, { dir = kit.side(req.side or "near", C.kerb_side), h = 22 })
    end,
    label = function(req, C)
      local dir = kit.side(req.side or "near", C.kerb_side)
      return "KEEP " .. turnWord(dir) .. " (" .. (dir == "left" and "R4-8" or "R4-7") .. ")"
    end,
    examples = { { side = "near" }, { side = "far" } },
  },
  {
    name = "no_turn_on_red", code = "R10-11a", title = "NO TURN ON RED", means = { regulation = "no_turn_on_red" },
    shape = "rect", size = { 24, 30 }, plate = "white", border = { color = "black", width = 0.625, inset = 0.375 },
    legend = function(f)
      f:text("NO", { y = 8.5, cap = 5, series = "C" })
      f:text("TURN", { y = 0, cap = 5, series = "C", fit = 20 })
      f:text("ON RED", { y = -8.5, cap = 5, series = "C", fit = 20 })
    end,
  },
  {
    name = "stop_ahead", code = "W3-1", title = "STOP AHEAD", means = { warns = "stop" },
    shape = "diamond", size = { 30 }, corner = 1.875, plate = "yellow", border = { color = "black", width = 0.75, inset = 0.5 },
    legend = function(f)
      local oct = kit.shapes.octagon(11)
      local moved = {}
      for i, p in ipairs(oct) do moved[i] = { p[1], p[2] + 6.0 } end
      f:poly({ kit.round(moved, 0.6, 2) }, "red")
      local inner = {}
      for i, p in ipairs(kit.inset(oct, 0.5, 0.6, 2)) do inner[i] = { p[1], p[2] + 6.0 } end
      local inner2 = {}
      for i, p in ipairs(kit.inset(oct, 0.9, 0.6, 2)) do inner2[i] = { p[1], p[2] + 6.0 } end
      f:poly({ inner, inner2 }, "white")
      kit.symbols.arrow_straight(f, { y0 = -13.0, y1 = -0.6, shaft = 3.4, head_w = 8.0, head_l = 4.6, color = "black" })
    end,
  },
  {
    name = "yield_ahead", code = "W3-2", title = "YIELD AHEAD", means = { warns = "give_way" },
    shape = "diamond", size = { 30 }, corner = 1.875, plate = "yellow", border = { color = "black", width = 0.75, inset = 0.5 },
    legend = function(f)
      local tri = kit.shapes.triangle_down(13)
      local moved, inner = {}, {}
      for i, p in ipairs(tri) do moved[i] = { p[1], p[2] + 6.0 } end
      for i, p in ipairs(kit.inset(tri, 2.2, 0.5, 2)) do inner[i] = { p[1], p[2] + 6.0 } end
      f:poly({ kit.round(moved, 0.6, 2) }, "red")
      f:poly({ inner }, "white")
      kit.symbols.arrow_straight(f, { y0 = -13.0, y1 = -1.4, shaft = 3.4, head_w = 8.0, head_l = 4.6, color = "black" })
    end,
  },
  {
    name = "signal_ahead", code = "W3-3", title = "SIGNAL AHEAD", means = { warns = "signal" },
    shape = "diamond", size = { 30 }, corner = 1.875, plate = "yellow", border = { color = "black", width = 0.75, inset = 0.5 },
    legend = function(f) kit.symbols.signal(f, { h = 20 }) end,
  },
  {
    -- `color`: "fyg" (fluorescent yellow-green, the manual allows it for pedestrian and school signs) or "yellow"
    name = "ped_crossing", code = "W11-2", title = "PEDESTRIAN CROSSING", means = { warns = "crosswalk" },
    shape = "diamond", size = { 30 }, corner = 1.875, plate = function(req) return req.color or "fyg" end,
    border = { color = "black", width = 0.75, inset = 0.5 },
    legend = function(f) kit.symbols.walker(f, { y = -11.0, h = 22 }) end,
    examples = { {}, { color = "yellow" } },
  },
  {
    name = "school", code = "S1-1", title = "SCHOOL", means = { warns = "school_zone" },
    shape = "pentagon", size = { 36, 36, shoulder = 0.5 }, corner = 1.875, plate = function(req) return req.color or "fyg" end,
    border = { color = "black", width = 0.875, inset = 0.625 },
    legend = function(f)
      kit.symbols.walker(f, { x = -4.2, y = -13.0, h = 19 })
      kit.symbols.walker(f, { x = 5.4, y = -13.0, h = 15 })
    end,
  },
  {
    name = "bike_lane", code = "R3-17", title = "BIKE LANE", means = { regulation = "lane_use", open_to = { "bike" } },
    shape = "rect", size = { 24, 18 }, plate = "white", border = { color = "black", width = 0.5, inset = 0.375 },
    legend = function(f, req, C)
      kit.symbols.bicycle(f, { y = -2.4, w = 15, rider = true, dir = kit.side("far", C.kerb_side) })
      f:text("LANE", { y = -5.4, cap = 3.0, series = "C" })
    end,
  },
  {
    -- a kerb lane for buses (and taxis): `who` lists who may use it. Text sign; no manual code checked for this layout
    name = "bus_lane", title = "BUS LANE", means = { regulation = "lane_use", open_to = { "bus", "taxi" } },
    shape = "rect", size = { 24, 30 }, plate = "white", border = { color = "black", width = 0.625, inset = 0.375 },
    legend = function(f, req)
      local lines = {}
      for _, w in ipairs(req.who or { "BUS", "TAXI" }) do lines[#lines + 1] = w end
      lines[#lines + 1] = "ONLY"
      local cap, gap = 4.5, 3.0
      local total = #lines * cap + (#lines - 1) * gap
      for i, s in ipairs(lines) do
        f:text(s, { y = total / 2 - cap / 2 - (i - 1) * (cap + gap), cap = cap, series = "C", fit = 20 })
      end
    end,
    label = function(req) return table.concat(req.who or { "BUS", "TAXI" }, " / ") .. " ONLY" end,
    examples = { {}, { who = { "BUS" } } },
  },
  {
    name = "no_parking", code = "R8-3", title = "NO PARKING", means = { regulation = "no_parking" },
    shape = "rect", size = { 24, 24 }, plate = "white", border = { color = "black", width = 0.625, inset = 0.375 },
    legend = function(f)
      f:text("P", { cap = 13, series = "D" })
      kit.symbols.prohibit(f, { d = 21, width = 2.4 })
    end,
  },
  {
    -- time-limited parking: `hours`, and `times` (the hours it applies, any text)
    name = "parking_limit", code = "R7-108", title = "TIME LIMIT PARKING", means = { regulation = "parking_limit", value = "hours" },
    shape = "rect", size = { 12, 18 }, corner = 0.75, plate = "white", border = { color = "green", width = 0.375, inset = 0.25 },
    legend = function(f, req)
      f:text(tostring(req.hours or 2) .. " HOUR", { y = 5.0, cap = 2.4, series = "C", color = "green", fit = 10 })
      f:text("PARKING", { y = 1.4, cap = 2.0, series = "C", color = "green", fit = 10 })
      f:text(req.times or "8 AM - 6 PM", { y = -2.6, cap = 1.6, series = "C", color = "green", fit = 10 })
      f:text(req.days or "EXCEPT SUNDAY", { y = -5.6, cap = 1.3, series = "C", color = "green", fit = 10 })
    end,
    label = function(req) return tostring(req.hours or 2) .. " HOUR PARKING (R7-108)" end,
  },
  {
    -- a street-name blade: any `name`; 6 in capitals on an 8 in blade (2D.43), as long as the name needs
    name = "street_name", code = "D3-1", title = "STREET NAME", means = { names = "street" },
    shape = "rect", plate = "green", border = { color = "white", width = 0.375, inset = 0.375 }, corner = 1.0,
    size = function(req, C)
      local w = sign_text.width(req.name or "MAIN ST", 6 * IN, C.series.C) / IN + 6
      return { math.max(24, math.min(72, math.ceil(w))), 8 }
    end,
    legend = function(f, req)
      f:text(req.name or "MAIN ST", { cap = 6, series = "C", color = "white", fit = f.w - 4 })
    end,
    label = function(req) return "STREET NAME " .. (req.name or "MAIN ST") .. " (D3-1)" end,
    examples = { { name = "HARBOR RD" }, { name = "MARTIN LUTHER KING JR BLVD" } },
  },
  {
    -- a freeway guide sign: `lines` of destinations (any text), an `exit` number (a tab on the exit's side), an `arrow`
    -- ("up" straight on, "exit" up and toward the exit side, or none); `exit_side` "near" by default
    name = "guide", title = "GUIDE", means = { guides = "destination" },
    shape = "rect", plate = "green", border = { color = "white", width = 1.25, inset = 0.75 }, corner = 6,
    size = function(req, C)
      local lines = req.lines or { "SALTWOOD", "HARBOR RD" }
      local w = 0
      for _, s in ipairs(lines) do w = math.max(w, sign_text.width(s, 13.3 * IN, C.series.E) / IN) end
      local h = 10 + #lines * 13.3 + (#lines - 1) * 8 + 10 + (req.arrow and 24 or 0)
      return { math.ceil(w + 24), math.ceil(h) }
    end,
    legend = function(f, req, C)
      local lines = req.lines or { "SALTWOOD", "HARBOR RD" }
      local y = f.h / 2 - 10 - 13.3 / 2
      for _, s in ipairs(lines) do
        f:text(s, { y = y, cap = 13.3, series = "E", color = "white", fit = f.w - 16 })
        y = y - 13.3 - 8
      end
      local d = kit.sx(kit.side(req.exit_side or "near", C.kerb_side))
      if req.arrow then
        local p = f:pt(0, -f.h / 2 + 15)
        local ang = req.arrow == "exit" and d * 45 or 0
        f.elems[#f.elems + 1] = { kind = "arrow", x = p[1] - 9 * IN, y = p[2] - 9 * IN, w = 18 * IN, h = 18 * IN, angle = ang,
                                  color = f:color("white") }
      end
      if req.exit then
        local s = "EXIT " .. tostring(req.exit)
        local tw = f:text_width(s, 9, "E") + 10
        f:tab(s, d * (f.w / 2 - tw / 2), tw, 13, 9)
      end
    end,
    label = function(req) return "GUIDE " .. table.concat(req.lines or { "SALTWOOD", "HARBOR RD" }, " / ") end,
    mount = "two_post",
    examples = { { lines = { "SALTWOOD", "HARBOR RD" }, exit = 12, arrow = "exit" }, { lines = { "DUNWYN" }, arrow = "up", mount = "overhead" } },
  },
}

-- ===================================================================================================================
-- THE CATALOGUE'S FUNCTIONS (read by sign_kit_lua.h)
-- ===================================================================================================================

function C.find(name)
  for _, e in ipairs(C.signs) do if e.name == name then return e end end
  error("no sign named " .. tostring(name))
end

function C.names()
  local out = {}
  for i, e in ipairs(C.signs) do out[i] = e.name end
  return out
end

function C.examples()
  local out = {}
  for _, e in ipairs(C.signs) do
    if not e.plaque then
      for _, ex in ipairs(e.examples or { {} }) do
        local r = { sign = e.name }
        for k, v in pairs(ex) do r[k] = v end
        out[#out + 1] = r
      end
    end
  end
  return out
end

local function valueOf(v, req) if type(v) == "function" then return v(req, C) end return v end

function C.label(entry, req)
  if entry.label then return entry.label(req, C) end
  return entry.title .. (entry.code and (" (" .. entry.code .. ")") or "")
end

-- One sign's face, and what its model needs: the outline (inches), its size, corner, tab.
local function layout(entry, req, wear, seed)
  local size = valueOf(entry.size, req)
  local ring, w, h = kit.outline(entry.shape, size)
  local r = entry.corner or C.plate.corner
  local f = kit.face(w, h, C.palette, C.series)
  -- the plate's colour, a little past its edge (the plate's mesh is the outline itself: no texel outside it shows)
  f:poly({ kit.round(mesh.profile.offset(ring, C.plate.bleed), r + C.plate.bleed, C.plate.segs) }, valueOf(entry.plate, req))
  if entry.border then
    local b = entry.border
    f:poly({ kit.inset(ring, b.inset, r, C.plate.segs), kit.inset(ring, b.inset + b.width, r, C.plate.segs) }, b.color)
  end
  entry.legend(f, req, C)
  return f:finish(C.sheeting, C.label(entry, req), wear, seed), { ring = ring, w = w, h = h, r = r, tab = f.tab_in }
end

-- The faces of a request: the sign's, then each plaque's (worn alike, each its own seed).
local function allFaces(req)
  local wear, seed = req.wear or 0, req.seed or 0
  local faces, geo, entries = {}, {}, {}
  local list = { req }
  for _, p in ipairs(req.plaques or {}) do list[#list + 1] = p end
  for i, q in ipairs(list) do
    local entry = C.find(q.sign)
    faces[i], geo[i] = layout(entry, q, wear, seed + (i - 1) * 7919)
    entries[i] = entry
  end
  return faces, geo, entries
end

function C.faces(req)
  local faces = allFaces(req)
  return faces
end

-- THE MODEL: the plates stacked (the sign, its plaques under it), on their mount; worn when asked.
function C.model(req, slots)
  local faces, geo, entries = allFaces(req)
  local entry = entries[1]
  local far = req.far and true or false
  local wear, seed = req.wear or 0, req.seed or 0
  local r = rng.new(math.floor(seed) * 2654435761 % 4294967296 + 1)
  local look = C.wear_look
  -- each part's material from C.materials; metal ages with the sign (rougher and darker by the same wear number)
  local function material(name)
    local m = C.materials[name]
    if not m then error("no material named " .. tostring(name)) end
    local aged = (m.metallic or 0) > 0 and wear or 0
    local k = 1 - aged * look.metal_darken
    return { name = name, metallic = m.metallic, roughness = math.min(1, m.roughness + aged * look.metal_rough),
             color = { m.color[1] * k, m.color[2] * k, m.color[3] * k } }
  end
  local faceMat, backMat, postMat = material("face"), material("plate_back"), material("post")
  local plateParts, metal, posts = {}, {}, {}   -- metal: the plates' backs and rims; posts: posts, clamps, arms, spans
  -- the mount, chosen before the plates are built: a worn plate bends away from what it is bolted to
  local W0 = 0
  for _, g in ipairs(geo) do W0 = math.max(W0, g.w * IN) end
  local mount = req.mount or entry.mount or (W0 >= C.mounts.two_post.from_width and "two_post" or "post")
  local holds, hold   -- the vertical lines on the plate (x from its centre) where it is held, and the held strip's half-width
  if mount == "post" or mount == "two_post" then
    local mt = C.mounts[mount]
    holds = mount == "post" and { 0 } or { -W0 * (0.5 - mt.at), W0 * (0.5 - mt.at) }
    hold = mt.size / 2 + 0.008 + look.hold_margin            -- the post, its clamp's strap, and a margin
  elseif mount == "mast_arm" then
    holds, hold = { -W0 / 3, W0 / 3 }, 0.015 + look.hold_margin   -- the two hangers
  else
    holds, hold = { 0 }, look.hold_margin                       -- an overhead span holds it across its whole back
  end
  -- the plates, each centred on x = 0, stacked down from y = 0 (the stack's top)
  local y, W = 0, 0
  for i, g in ipairs(geo) do
    local outline = kit.round(g.ring, g.r, far and C.plate.far_segs or C.plate.segs)
    local front, back = kit.plate(outline, g.w, g.h, slots[i], C.plate.thick, backMat.color)
    local pieces = { front, back }
    if g.tab then
      -- an exit tab: its own small plate standing on the panel's top edge, the same texture mapping carried up
      local t = g.tab
      local tring = kit.round(kit.shapes.rect(t.w, t.h + 2 * g.r), 2, far and C.plate.far_segs or C.plate.segs)
      for k, p in ipairs(tring) do tring[k] = { p[1] + t.x, p[2] + g.h / 2 + t.h / 2 - g.r } end
      local tf, tb = kit.plate(tring, g.w, g.h, slots[i], C.plate.thick, backMat.color)
      pieces[#pieces + 1] = tf
      pieces[#pieces + 1] = tb
    end
    if i == 1 and wear > 0 then
      local fold = r:next() < wear * look.fold_chance and wear * look.fold_deg * math.pi / 180 or 0
      local side = r:next() < 0.5 and -1 or 1
      local ymax = (g.h / 2 + (g.tab and g.tab.h or 0)) * IN
      pieces = kit.bend(pieces, g.w * IN, ymax, wear * look.bend_bow * r:range(0.3, 1.0), fold, side, holds, hold)
    end
    local cy = y - g.h * IN / 2
    for k, m in ipairs(pieces) do pieces[k] = mesh.translate(m, { 0, cy, 0 }) end
    plateParts[#plateParts + 1] = { name = "face", face = i, mesh = pieces[1] }
    metal[#metal + 1] = pieces[2]
    if g.tab then
      plateParts[#plateParts + 1] = { name = "face", face = i, mesh = pieces[3] }
      metal[#metal + 1] = pieces[4]
    end
    geo[i].cy = cy
    y = y - g.h * IN - 1 * IN   -- an inch between plates
    W = math.max(W, g.w * IN)
  end
  local stackH = -(y + 1 * IN)   -- the stack's height
  local steel = postMat.color
  local thick = C.plate.thick
  local bottom, x0, origin
  local function place(dx, dy, dz)
    for _, p in ipairs(plateParts) do p.mesh = mesh.translate(p.mesh, { dx, dy, dz }) end
    for k, m in ipairs(metal) do metal[k] = mesh.translate(m, { dx, dy, dz }) end
  end
  if mount == "post" or mount == "two_post" then
    local mt = C.mounts[mount]
    bottom = mt.bottom - ((#geo > 1 and C.mounts.post.plaque_drop) or 0)
    local top = bottom + stackH
    place(0, top, 0)
    local s = mt.size
    local zp = -thick - s / 2
    local xs = mount == "post" and { 0 } or { -W * (0.5 - mt.at), W * (0.5 - mt.at) }
    for _, x in ipairs(xs) do
      posts[#posts + 1] = kit.box({ s, top - 0.03, s }, { x, (top - 0.03) / 2, zp }, steel)
      if not far and (mt.clamps or mount == "two_post") then
        for _, g in ipairs(geo) do
          for _, dy in ipairs({ -0.25, 0.25 }) do
            -- a strap round the post, wholly behind the plate's back
            posts[#posts + 1] = kit.box({ s + 0.016, 0.025, s + 0.012 }, { x, top + g.cy + dy * g.h * IN, zp - 0.008 }, steel)
          end
        end
      end
    end
    origin = "the post's foot (two posts: midway between them), on the ground"
    x0 = 0
  elseif mount == "mast_arm" then
    local mt = C.mounts.mast_arm
    local over = kit.sx(kit.side("far", C.kerb_side))   -- the arm reaches from the kerb out over the road
    bottom = mt.clearance
    local top = bottom + stackH
    local armY = top + 0.18
    local zArm = -thick - mt.arm_r
    x0 = over * mt.reach
    place(x0, top, 0)
    local sides = far and mt.far_sides or mt.sides
    posts[#posts + 1] = kit.bar_y(0, 0, armY + 0.35, zArm, mt.pole_r, sides, steel)
    posts[#posts + 1] = kit.bar_x(0, over * (mt.reach + W / 2 + 0.4), armY, zArm, mt.arm_r, sides, steel)
    for _, dx in ipairs({ -W / 3, W / 3 }) do
      posts[#posts + 1] = kit.box({ 0.03, armY - top + 0.04, 0.02 }, { x0 + dx, (armY + top) / 2, -thick - 0.012 }, steel)
    end
    origin = "the pole's foot, on the kerb; the sign hangs over the road on the far side"
  elseif mount == "overhead" then
    local mt = C.mounts.overhead
    bottom = mt.clearance
    local top = bottom + stackH
    place(0, top, 0)
    local sides = far and mt.far_sides or mt.sides
    local zc = -thick - mt.chord_r
    for _, x in ipairs({ -mt.span / 2, mt.span / 2 }) do
      posts[#posts + 1] = kit.bar_y(x, 0, top + 0.3, zc, mt.upright_r, sides, steel)
    end
    for _, cy in ipairs({ top - 0.12, bottom + 0.12 }) do
      posts[#posts + 1] = kit.bar_x(-mt.span / 2, mt.span / 2, cy, zc, mt.chord_r, sides, steel)
    end
    origin = "the middle of the span, on the ground"
    x0 = 0
  else
    error("unknown mount " .. tostring(mount))
  end
  -- a knocked post leans (a post mount only), from its foot
  local parts = {}
  local leanZ, leanX = 0, 0
  if wear > 0 and (mount == "post" or mount == "two_post") then
    leanZ = wear * look.lean_deg * math.pi / 180 * r:range(-1, 1)
    leanX = wear * look.lean_deg * 0.5 * math.pi / 180 * r:range(-1, 1)
  end
  local function lean(m)
    if leanZ == 0 and leanX == 0 then return m end
    return mesh.rotate_x(mesh.rotate_z(m, leanZ), leanX)
  end
  for _, p in ipairs(plateParts) do
    parts[#parts + 1] = { name = p.name, face = p.face, material = faceMat, mesh = lean(p.mesh) }
  end
  -- two metal parts, because their materials differ (no triangle is added by the split)
  parts[#parts + 1] = { name = "plate_back", material = backMat, mesh = lean(mesh.merge(metal)) }
  parts[#parts + 1] = { name = "post", material = postMat, mesh = lean(mesh.merge(posts)) }
  local sizes = {}
  for i, g in ipairs(geo) do sizes[i] = { g.w, g.h } end
  return {
    label = C.label(entry, req),
    info = {
      sign = entry.name, code = entry.code, title = entry.title, means = entry.means, shape = entry.shape,
      size_in = sizes, mount = mount, bottom_m = bottom, kerb_side = C.kerb_side, origin = origin,
      plaques = #geo - 1, far = far, wear = wear, seed = seed,
      manual = "MUTCD 2009 (FHWA), Tables 2B-1 / 2C-2, Section 2A.18",
    },
    parts = parts,
  }
end

return C
