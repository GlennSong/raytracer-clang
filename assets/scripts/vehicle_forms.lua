-- vehicle_forms.lua — the FORM layer: class package data -> mesh.car parameters.
--
-- vehicle_classes.lua answers "how big is a jeep, and what features does it
-- have"; car_mesh.cpp answers "given curves and window bands, build a shell".
-- Neither answers "what curves does a jeep have", and that is this file.
--
-- Everything here keys off `c.form` (coupe | suv | pickup) and `c.box_count`,
-- the two things Orsborn et al. found actually distinguish vehicle classes —
-- rather than off the class NAME, so a new class picks up a sensible shape by
-- declaring its form instead of needing a new branch.
--
-- The curves are fractions: `roof`/`sill` are fractions of overall height,
-- `plan` is a fraction of half-width, and z runs +0.5 at the nose to -0.5 at the
-- tail. All three are lists of {z, value} keys, smoothstepped between.

local forms = {}

-- The cabin roof plateau, as a fraction of height. Not a free knob: a taller
-- greenhouse has to start lower on the body, so it comes off the class's own
-- `greenhouse_frac` (1/3 of height is the textbook car; vans run higher).
local function plateau_of(c)
  local g = c.greenhouse_frac or 0.33
  -- 0.33 -> 0.94, rising slowly. Clamped so a wild override cannot invert the
  -- roof through the belt line.
  return math.max(0.80, math.min(0.99, 0.94 + (g - 0.33) * 0.45))
end

-- ROOFLINE. The single most recognisable curve on the car: where the bonnet
-- ends, how raked the screen is, whether the roof falls to a separate boot
-- (three-box) or runs back to a tailgate (two-box) or never falls at all (van).
function forms.roof(c)
  local top = plateau_of(c)
  local boxes = c.box_count or 3
  if c.form == "suv" then
    -- Taller and flatter, with an upright screen and a squared-off tail. The
    -- roof barely falls at the back — that is what reads as "utility".
    return { {0.50, 0.50}, {0.44, 0.56}, {0.32, 0.64}, {0.26, 0.69},
             {0.15, top},  {-0.30, top}, {-0.44, top - 0.03}, {-0.50, 0.74} }
  elseif c.form == "pickup" then
    -- Cab roof plateau ends early; behind it the bed sides are far lower, which
    -- is the silhouette. (The bed is still a hole in one volume, not a separate
    -- module — see the note in vehicles.lua.)
    return { {0.50, 0.52}, {0.44, 0.58}, {0.34, 0.66}, {0.28, 0.70},
             {0.18, top},  {-0.06, top}, {-0.14, 0.60}, {-0.46, 0.58},
             {-0.50, 0.54} }
  elseif boxes == 1 then
    -- One box: the screen starts almost at the bumper and the roof runs flat
    -- all the way to the tail. Short nose is the whole read.
    return { {0.50, 0.56}, {0.46, 0.72}, {0.38, 0.90}, {0.30, top},
             {-0.40, top}, {-0.47, top - 0.04}, {-0.50, 0.76} }
  elseif boxes == 2 then
    -- Two box: sedan's nose, but the roof carries back to a steep tailgate
    -- instead of dropping to a boot lid.
    return { {0.50, 0.44}, {0.44, 0.48}, {0.30, 0.55}, {0.24, 0.59},
             {0.13, top},  {-0.26, top}, {-0.40, top - 0.05}, {-0.47, 0.80},
             {-0.50, 0.64} }
  end
  -- Three box: bonnet, raked screen, cabin, then a distinct boot deck.
  return { {0.50, 0.42}, {0.44, 0.46}, {0.30, 0.52}, {0.25, 0.55},
           {0.125, top}, {-0.24, top + 0.01}, {-0.40, 0.62}, {-0.46, 0.58},
           {-0.50, 0.54} }
end

-- SILL — the underside. Higher and flatter on anything with ground clearance,
-- because a tall body with a car's low rocker reads as a lifted car, not a truck.
function forms.sill(c)
  if c.form == "suv" or c.form == "pickup" then
    return { {0.50, 0.26}, {0.44, 0.20}, {0.30, 0.17}, {-0.40, 0.17},
             {-0.46, 0.20}, {-0.50, 0.26} }
  elseif (c.box_count or 3) == 1 then
    return { {0.50, 0.24}, {0.44, 0.16}, {0.30, 0.13}, {-0.40, 0.13},
             {-0.46, 0.16}, {-0.50, 0.24} }
  end
  return { {0.50, 0.22}, {0.44, 0.14}, {0.30, 0.10}, {-0.40, 0.10},
           {-0.46, 0.14}, {-0.50, 0.22} }
end

-- PLAN — half-width along the car. Boxier classes hold full width closer to
-- both ends; a coupe pinches in hard at the nose and tail.
function forms.plan(c)
  if c.form == "suv" or c.form == "pickup" or (c.box_count or 3) == 1 then
    return { {0.50, 0.74}, {0.44, 0.90}, {0.32, 0.99}, {0.02, 1.00},
             {-0.30, 1.00}, {-0.42, 0.98}, {-0.47, 0.92}, {-0.50, 0.80} }
  end
  return { {0.50, 0.62}, {0.44, 0.82}, {0.30, 0.97}, {0.02, 1.00},
           {-0.22, 1.00}, {-0.40, 0.97}, {-0.46, 0.88}, {-0.50, 0.70} }
end

-- GLASS BANDS. The windscreen sits between the cowl and the roof plateau, so it
-- moves with the roofline rather than being authored twice; the backlight is
-- placed off the tail treatment.
function forms.windshield(c)
  if c.form == "suv" then return { 0.15, 0.27 } end
  if c.form == "pickup" then return { 0.18, 0.29 } end
  if (c.box_count or 3) == 1 then return { 0.30, 0.44 } end
  return { 0.125, 0.25 }
end

function forms.backlight(c)
  local boxes = c.box_count or 3
  if c.form == "pickup" then return { -0.14, -0.07 } end     -- small cab rear window
  if c.form == "suv" then return { -0.44, -0.31 } end
  if boxes == 1 then return { -0.47, -0.40 } end
  if boxes == 2 then return { -0.47, -0.33 } end
  return { -0.40, -0.24 }
end

-- SIDE GLASS. Row count drives pane count: two rows of seats means two doors of
-- glass per side, one row means a single cab pane. Quarter lights are kept for
-- the car forms (see the DLO note in car_mesh.cpp) and dropped on the utility
-- forms, whose real counterparts largely do without them.
function forms.side_windows(c)
  local rows = c.rows or 2
  if c.form == "pickup" or rows < 2 then
    return { { 0.02, 0.16 } }                       -- single cab pane
  end
  if c.form == "suv" then
    return { { -0.08, 0.13 }, { -0.30, -0.12 } }    -- two big upright panes
  end
  if (c.box_count or 3) == 1 then
    return { { 0.10, 0.26 }, { -0.34, 0.06 } }      -- cab pane + long cargo window
  end
  -- Car: four panes, each door's main glass plus its quarter light.
  return { { 0.082, 0.110 }, { -0.086, 0.072 },
           { -0.235, -0.121 }, { -0.273, -0.245 } }
end

-- Assemble the full mesh.car parameter table for a class.
function forms.car_params(c, d, opts)
  opts = opts or {}
  return {
    width = d.width, height = d.height, length = d.length,
    wheel_diameter = c.wheel_diameter,
    front_overhang = d.front_overhang, rear_overhang = d.rear_overhang,
    ground_clearance = c.h156,
    tumblehome = c.tumblehome,
    roof = forms.roof(c), sill = forms.sill(c), plan = forms.plan(c),
    windshield = forms.windshield(c), backlight = forms.backlight(c),
    side_windows = forms.side_windows(c),
    paint = opts.color, lod = opts.lod or "high",
  }
end

return forms
