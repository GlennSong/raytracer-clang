-- Vehicle CLASS BEHAVIOUR as data — the reference layer the car generator's
-- recipes are built on. Defines a global `vehicle_classes` table.
--
-- This file holds the two things that make a generated vehicle read as a real
-- one, kept separate from the recipes themselves so the sourcing stays legible:
--
--   1. FEATURE GATES — which features a class has AT ALL. Features switch on and
--      off by class; they are not scaled to zero. A pickup has a bed rule, a
--      coupe does not, and no amount of parameter tweaking turns one into the
--      other. Transcribed from Orsborn, Cagan, Pawlicki & Smith (2006),
--      "Creating cross-over vehicles", AIEDAM 20(3) 217-246, Table 2 (p. 237).
--
--   2. PACKAGE RANGES — the proportions. Car designers work from a package (the
--      occupant and wheel layout) and let form follow, which is why deriving
--      proportions here makes a vehicle plausible by construction rather than by
--      tuning. Codes are SAE J1100 "Motor Vehicle Dimensions" so a generated
--      vehicle is cross-checkable against any real spec sheet.
--
-- SOURCING. Numbers marked [J] are verified from a primary standard text; those
-- marked [~] are plausible defaults from secondary sources — good enough to
-- generate from, not good enough to cite. J1100 and J826 are readable because US
-- federal regulation incorporates them by reference:
--   J1100: https://law.resource.org/pub/us/cfr/ibr/005/sae.j1100.2001.html
--   J826:  https://archive.org/download/gov.law.sae.j826.1995/sae.j826.1995.html
--
-- NOT transcribed: Orsborn's numeric parameter envelopes (Figs. 29-38). They
-- bound Bezier control points of five named 2D curves in a coordinate frame the
-- paper never pins down (Fig. 30's "global" axis runs Y 20-70 in, which cannot be
-- lateral distance from the stated origin), and our generator sweeps 3D station
-- rings rather than fitting 2D curves. Converting would validate an
-- interpretation, not the paper. The class RELATIONS below are taken instead —
-- those are robust to the datum question.
--
-- Units are METRES and DEGREES (engine convention). SAE publishes mm; converted.

local vehicle_classes = {}


-- ---------------------------------------------------------------------------
-- SAE J1100 / J1516 / J826 constants shared by every class.

vehicle_classes.sae = {
    -- Chair height H30 (SgRP to accelerator heel point) is THE class axis: one
    -- continuous number spanning sports car to bus. The A/B split is a real
    -- standardised boundary, not a stylistic one. [J] J1516/J1517.
    h30_class_a = { 0.127, 0.405 },   -- cars, vans, light trucks
    h30_class_b = { 0.405, 0.530 },   -- heavy trucks, buses

    -- Back angle A40 correlates with H30 and is why truck drivers visibly sit
    -- upright while sports-car drivers recline. [J]
    a40_class_a = { 5, 40 },
    a40_class_b = { 11, 18 },
    a40_default = 25,                 -- [J] J826 default when unspecified

    -- Steering wheel diameter W9. [J] Class A/B bounds; [~] the car figure.
    w9_class_a_max = 0.450,
    w9_class_b     = { 0.450, 0.560 },
    w9_car_typical = 0.370,

    -- J826 H-point manikin leg segments, 50th percentile. [J] These are the
    -- numbers every car interior in the world is packaged around, so a seated
    -- driver built from them lands knees/hands/head where a real one's are.
    thigh_50    = 0.4315,
    lower_leg_50 = 0.4175,
    foot_angle_min = 87,              -- [J] degrees
    sgrp_outboard  = 0.381,           -- [J] driver SgRP from centreline

    -- US regulatory hardpoints. Every real vehicle obeys these, which is why a
    -- generated one that doesn't looks subtly wrong. [J]
    bumper_zone   = { 0.406, 0.508 }, -- 49 CFR 581 impact zone above ground
    headlamp_h125 = { 0.559, 1.372 }, -- FMVSS 108 lamp centre above ground
    plate_us      = { 0.305, 0.152 }, -- 6 x 12 in
    plate_eu      = { 0.520, 0.110 },
}


-- ---------------------------------------------------------------------------
-- FEATURE GATES — Orsborn et al. Table 2, mapped onto our feature names.
--
-- The paper's three classes are coupe / SUV / pickup. Our sedan and hatchback
-- both inherit the coupe gates (it is a front/side/rear-view grammar, and a
-- sedan differs from a coupe in door count, which the grammar handles by rule 41
-- rather than by class). Van, box truck and bus are ours, not the paper's —
-- their gates are marked [ours] and follow the same on/off discipline.
--
-- Each flag names a FEATURE, not a rule number, so the table stays readable; the
-- originating Table 2 rule is in the trailing comment.

local COUPE_GATES = {
    rear_fender_coupe   = true,   -- rule 18
    grill_own_rules     = true,   -- rules 23, 24 (grill rules only used on coupes)
    grill_in_bumper     = true,   -- rule 30 (grill inserted into the front bumper)
    trunk               = true,   -- rules 36, 37
    rear_side_window    = true,   -- rule 43 (coupes only)
    headlight_coupe     = true,   -- rule 59
    taillight_coupe     = true,   -- rule 60
    rear_door           = true,   -- rule 41 (sedans; a true coupe omits it)
}

local SUV_GATES = {
    rear_fender_utility = true,   -- rule 50 (SUVs and pickups)
    front_bumper_tall   = true,   -- rule 52 (pickups and SUVs only)
    rear_windshield_alt = true,   -- rule 48 (pickups and some SUVs)
    trunk               = true,   -- rule 37 (coupes and SUVs)
    hatch               = true,   -- rule 56 (trunk/hatch rule for SUVs)
    rear_bumper_step    = true,   -- rule 54 (step on SUV rear bumper)
    rear_bumper_utility = true,   -- rules 51, 57
    beltline_door_curve = true,   -- rule 55 (curve in beltline in front door, SUVs only)
    rear_door           = true,   -- rule 41
    rear_side_window    = true,   -- rule 44 (pickups and SUVs)
    rear_window_suv     = true,   -- rule 45 (SUVs only)
    taillight_utility   = true,   -- rule 53 (SUVs and pickups)
}

local PICKUP_GATES = {
    rear_fender_utility = true,   -- rule 50
    front_bumper_tall   = true,   -- rule 52
    rear_windshield_alt = true,   -- rule 48
    bed                 = true,   -- rule 49 (bed rule for pickups only)
    rear_bumper_utility = true,   -- rules 51, 57
    rear_side_window    = true,   -- rule 44
    taillight_utility   = true,   -- rule 53
    -- rule 40 "does not work with pickups": no wrap-around door glass.
    door_wrap_glass     = false,
}


-- ---------------------------------------------------------------------------
-- QUALITATIVE CLASS RELATIONS — Orsborn et al. pp. 237-240, prose + figures.
--
-- These survive the datum ambiguity that made the numeric envelopes unusable,
-- and they are what actually separates the classes at a glance:
--
--   * Coupe grills sit LOWER and come FARTHER FORWARD than SUV/pickup grills,
--     because of nose length and the grill's placement within the bumper.
--   * SUV and pickup grills are similar in height range and horizontal average;
--     SUV grills have the broader horizontal range. Both sit farther BACK than a
--     coupe's, because of shorter noses.
--   * SUV/pickup grills curve CONSISTENTLY along their length; coupe grills
--     curve SHARPLY at one end or the other. The hood does the inverse. So the
--     vertical-to-horizontal transition in the front end happens in the GRILL on
--     a coupe and in the HOOD on an SUV/pickup — one line, and it is most of
--     what makes a front end read as one class or the other.
--   * Coupe and SUV side curves do NOT overlap at all — the single cleanest
--     class separator in the paper.
--   * SUV hoods fall almost entirely INSIDE the coupe hood range; pickup fenders
--     nearly contain the SUV range. Classes are nested, not disjoint, except at
--     the side curve.

vehicle_classes.relations = {
    grill_drop     = { coupe = 0.00, suv = 0.34, pickup = 0.34 },  -- up from lowest
    grill_setback  = { coupe = 0.00, suv = 0.30, pickup = 0.26 },  -- back from nose
    grill_curve    = { coupe = "sharp_end", suv = "even", pickup = "even" },
    hood_curve     = { coupe = "even", suv = "sharp_end", pickup = "sharp_end" },
}


-- ---------------------------------------------------------------------------
-- THE CLASSES.
--
-- `wheelbase_wd` is in WHEEL DIAMETERS, the proportion designers actually reason
-- in: Peter Stevens (McLaren F1) gives ~4.5 as his rule of thumb, the Wikipedia
-- glossary says ~4; 4.0-4.7 is the honest band for a normal car. [~]
--
-- `height_wd` is authored per class rather than derived. The widely repeated
-- "height ~= 3x wheel diameter" rule does NOT reproduce real cars: 3 x 0.62 m
-- gives a 1.86 m sedan, and a real one is ~1.45 m. Measured off actual class
-- dimensions the ratio runs ~2.3 (sedan, SUV) to ~3.0 (bus). Stevens is likely
-- measuring something other than overall height; we don't guess at what.
--
-- Overhangs are likewise real: a sedan carries ~0.9 m front and ~1.0 m rear on a
-- 2.7 m wheelbase for a 4.6 m car. Deriving them from too-small multiples was
-- caught immediately by the wb/length bound below, which is the point of having
-- it.
--
-- `drivetrain` drives dash-to-axle, which is the single strongest cue for what a
-- car COSTS: FWD puts the engine over the front axle (long front overhang, short
-- dash-to-axle, front wheel close to the A-pillar), RWD sets it behind (short
-- overhang, long dash-to-axle — the BMW/Jaguar look). One enum, no other change.
--
-- `greenhouse_frac` defaults to 1/3 of total height, body 2/3 — a widely repeated
-- and reliable starting proportion. The belt line is NOT authored: per Orsborn it
-- has no rules of its own and EMERGES from hood + side windows + trunk.

vehicle_classes.classes = {
    sedan = {                                  -- ~4.60 x 1.82 x 1.45 m
        gates = COUPE_GATES, form = "coupe",
        box_count = 3, drivetrain = "fwd", sae_class = "a",
        wheel_diameter = 0.62, wheelbase_wd = 4.35, height_wd = 2.34,
        front_overhang_wd = 1.45, rear_overhang_wd = 1.61,
        track = 1.56, h156 = 0.14, h30 = 0.28, a40 = 25, w9 = 0.370,
        greenhouse_frac = 0.33, tumblehome = 7,
        rows = 2, l50 = 0.87,
    },
    hatchback = {                              -- ~3.95 x 1.75 x 1.48 m
        gates = COUPE_GATES, form = "coupe",
        box_count = 2, drivetrain = "fwd", sae_class = "a",
        wheel_diameter = 0.60, wheelbase_wd = 4.17, height_wd = 2.47,
        front_overhang_wd = 1.33, rear_overhang_wd = 1.08,
        track = 1.52, h156 = 0.15, h30 = 0.30, a40 = 25, w9 = 0.370,
        greenhouse_frac = 0.35, tumblehome = 8,
        rows = 2, l50 = 0.83,
    },
    jeep = {                                   -- ~4.40 x 1.88 x 1.75 m
        gates = SUV_GATES, form = "suv",
        box_count = 2, drivetrain = "rwd", sae_class = "a",
        wheel_diameter = 0.74, wheelbase_wd = 3.65, height_wd = 2.36,
        front_overhang_wd = 1.15, rear_overhang_wd = 1.15,
        track = 1.60, h156 = 0.22, h30 = 0.36, a40 = 22, w9 = 0.390,
        greenhouse_frac = 0.38, tumblehome = 3,
        rows = 2, l50 = 0.88,
    },
    pickup = {                                 -- ~5.70 x 2.00 x 1.95 m
        gates = PICKUP_GATES, form = "pickup",
        box_count = 2, modules = { "bed", "frame" },
        drivetrain = "rwd", sae_class = "a",
        wheel_diameter = 0.78, wheelbase_wd = 4.62, height_wd = 2.50,
        front_overhang_wd = 1.15, rear_overhang_wd = 1.54,
        track = 1.70, h156 = 0.22, h30 = 0.38, a40 = 20, w9 = 0.400,
        greenhouse_frac = 0.34, tumblehome = 2,
        rows = 2, l50 = 0.90,
    },
    van = {                                    -- ~5.20 x 1.98 x 1.90 m  [ours]
        gates = SUV_GATES, form = "suv",
        box_count = 1, drivetrain = "fwd", sae_class = "a",
        wheel_diameter = 0.68, wheelbase_wd = 4.41, height_wd = 2.79,
        front_overhang_wd = 1.32, rear_overhang_wd = 1.91,
        track = 1.68, h156 = 0.18, h30 = 0.38, a40 = 20, w9 = 0.420,
        greenhouse_frac = 0.30, tumblehome = 2,
        rows = 2, l50 = 0.90,
    },
    box_truck = {                              -- ~6.60 x 2.35 x 2.90 m  [ours]
        gates = PICKUP_GATES, form = "pickup",
        box_count = 1, modules = { "cargo_box" },
        drivetrain = "rwd", sae_class = "b",
        wheel_diameter = 0.90, wheelbase_wd = 4.22, height_wd = 3.22,
        front_overhang_wd = 1.22, rear_overhang_wd = 1.89,
        track = 1.90, h156 = 0.24, h30 = 0.45, a40 = 15, w9 = 0.500,
        greenhouse_frac = 0.22, tumblehome = 0,
        rows = 1, l50 = 0.90,
    },
    bus = {                                    -- ~11.4 x 2.55 x 3.20 m  [ours]
        gates = SUV_GATES, form = "suv",
        box_count = 1, modules = { "cargo_box", "seat_rows" },
        drivetrain = "rwd", sae_class = "b",
        wheel_diameter = 1.05, wheelbase_wd = 5.62, height_wd = 3.05,
        front_overhang_wd = 2.19, rear_overhang_wd = 2.95,
        track = 2.10, h156 = 0.28, h30 = 0.48, a40 = 15, w9 = 0.520,
        greenhouse_frac = 0.30, tumblehome = 0,
        -- Bus seating is a repeat, not a layout: city 0.61-0.71 m pitch, school
        -- 0.71-0.76, coach 0.86-1.02. [~] One repeat op fills the whole cabin.
        rows = 0, seat_pitch = 0.66, seat_width = 0.46,
    },
}


-- ---------------------------------------------------------------------------
-- Plausibility bounds. A recipe edit that leaves these has produced a cartoon —
-- this is what the generator's tests assert against, and it is why the numbers
-- above are sourced rather than invented.

vehicle_classes.bounds = {
    wheelbase_wd    = { 2.0, 6.0 },
    wb_over_length  = { 0.50, 0.70 },
    height_wd       = { 2.2, 3.3 },
    greenhouse_frac = { 0.20, 0.45 },
}


-- Derive the overall dimensions a class implies. Length is the sum of the
-- package, not an independent knob — which is the whole point of packaging
-- first: you cannot author a car whose parts don't add up.
function vehicle_classes.dims(c)
    local wd = c.wheel_diameter
    local wb = c.wheelbase_wd * wd
    return {
        wheelbase = wb,
        length = wb + (c.front_overhang_wd + c.rear_overhang_wd) * wd,
        height = c.height_wd * wd,
        -- Body sits ~13 cm proud of each tyre. [~] Right for cars; it undershoots
        -- the box truck and bus by ~8%, because on those the body width is set by
        -- the CARGO MODULE (which is wider than the cab), not by the track. The
        -- module overrides this when it lands.
        width  = c.track + 0.26,
        front_overhang = c.front_overhang_wd * wd,
        rear_overhang  = c.rear_overhang_wd * wd,
    }
end


-- Check a class (or a recipe merged onto one) against `bounds` and its SAE
-- class. Returns nil when clean, else a list of human-readable failures. The
-- generator's tests call this; it also caught four bad overhang values the first
-- time this file was written, which is why it exists.
function vehicle_classes.validate(name, c)
    local b, s, d = vehicle_classes.bounds, vehicle_classes.sae, vehicle_classes.dims(c)
    local bad = {}
    local function chk(label, v, r)
        if v < r[1] or v > r[2] then
            bad[#bad + 1] = string.format("%s: %s %.3f not in [%.2f, %.2f]",
                                          name, label, v, r[1], r[2])
        end
    end
    chk("wheelbase_wd", c.wheelbase_wd, b.wheelbase_wd)
    chk("wb/length", d.wheelbase / d.length, b.wb_over_length)
    chk("height_wd", c.height_wd, b.height_wd)
    chk("greenhouse_frac", c.greenhouse_frac, b.greenhouse_frac)
    chk("h30", c.h30, (c.sae_class == "b") and s.h30_class_b or s.h30_class_a)
    chk("a40", c.a40, (c.sae_class == "b") and s.a40_class_b or s.a40_class_a)
    if c.sae_class == "a" and c.w9 > s.w9_class_a_max then
        bad[#bad + 1] = name .. ": w9 exceeds Class A maximum"
    end
    return (#bad > 0) and bad or nil
end


-- Merge a class's defaults under a recipe's overrides. Recipes stay short: they
-- say what makes THEM different, and inherit the rest of the class.
function vehicle_classes.apply(class_name, opts)
    local c = vehicle_classes.classes[class_name]
    if not c then error("unknown vehicle class: " .. tostring(class_name)) end
    local p = {}
    for k, v in pairs(c) do p[k] = v end
    for k, v in pairs(opts or {}) do p[k] = v end
    return p
end

return vehicle_classes
