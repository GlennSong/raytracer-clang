-- tree_recipes.lua -- TREE SPECIES AS DATA (procgen/tree_recipe.h; Glenn, 2026-10-17: "Could lua be used to define
-- recipes and then the C++ could ingest those recipes and build out those trees?"). C++ expands the grammar, runs the
-- turtle, sizes the branches by the pipe model, bends and gnarls them and skins the bark; nothing here runs per branch.
--
-- A recipe:
--   axiom, iterations   the grammar's start and how many rewrites
--   rules               { predecessor, successor [, weight] }: a parametric L-system (ABoP). A predecessor may carry
--                       a guard ("B(l):l>0.25"); several rules for one predecessor are weighted random choices.
--                       Turtle: F(len) a segment; +(a) -(a) yaw, &(a) ^(a) pitch, /(a) \(a) roll (degrees);
--                       [ ] a branch; A or L a tip (where leaves go). Any other letter is a growing part with no
--                       mark of its own (T trunk, C leader, B branch here).
--                       $name is one of the recipe's vars, drawn per tree.
--   vars                the grammar's own numbers: a value, or {lo, hi} drawn per seed
--   params              the grower's knobs (tree.h TreeParams, by name), each a value or {lo, hi}:
--     shape    droop light crookChance crookAngle meander wander lean angleJitter
--     trunk    flare lobes lobeDepth lobeHeight nodeSwell barkNoise twist rootCount rootSpread
--     joints   collar (a side branch welds into its parent, flaring by this; 0 = overlapping tubes)
--     girth    tipRadius pipeExponent radiusScale ringSegments
--     leaves   leaves leafSize leavesPerTip leafThickness leafClump (how far back along a twig its cards spread)
--              leafPitch (deg a card tilts out from its twig) maxLeafCards
--   bark                the bark (procgen/material_recipes.h): "furrowed" (oak), "plated" (pine, spruce), "smooth"
--                       (beech), "papery" (birch: white, dark dashes and patches), "ringed" (palms, cherry)
--   bark_color, leaf_color   {r, g, b} linear, or {{lo}, {hi}} drawn per tree
--   leaf_palette        colour by patches across the crown (params.paletteScale m across): a list of colours the
--                       cards blend along, or a list of such lists, each seed picking one (red maples, orange ones)
--   card                THE LEAF CARD the tree's foliage wears (procgen/leaf_card.h): a small 2D twig grammar drawn by a
--                       2D turtle -- F(len) a stem, +(a) -(a) turns, [ ] a side shoot, L(scale) a leaf -- and the leaf:
--     axiom, iterations, rules, vars   as the tree's (vars drawn per card variant)
--     leaf = { shape = "oval" | "lobed" | "serrate" | "needle" | "leaflet" | "palmate", length, width, lobes, teeth,
--              color (a tint: the card's colour is brightness x tint x the tree's vertex colour; default white) }
--     flower = { shape = "blossom" | "daisy" | "floret", size, petals, notch, eye, color, eye_color }: drawn at W(s)
--     stem_color, stem_width, cup (a leaf's curl about its midrib, 0 flat), shade_jitter, tip_bright (lighter new
--     growth toward the card's far end), variants (cards a side of the atlas)
--
-- The tree lab (assets/levels/tree_lab.json) grows each in a row of seeds; this file hot-reloads there.

tree_recipes = {

  -- OAK: a short clear bole, then the crown FORKS -- the leader keeps dying back and two laterals take over (the
  -- sympodial zig-zag that makes an oak's limbs crooked) -- out to sprays of twigs. Gnarled: crooks, a fluted foot,
  -- knotty limbs.
  oak = {
    axiom = "T(1.3,2)",
    iterations = 13,
    rules = {
      { "T(l,n):n>0",   "F(l)/(97)T(l*0.95,n-1)" },
      { "T(l,n):n<=0",  "B(l*0.9)" },
      { "B(l):l>0.22",  "F(l)/($roll)[&($pitch)B(l*$side)]/(180)[&($pitch2)B(l*$side2)]", 3 },
      { "B(l):l>0.22",  "F(l)/($roll)[&($pitch)B(l*$side)]^(10)B(l*$lead)", 2 },
      { "B(l):l<=0.22", "F(l)/(137.5)[&(55)A(l)]/(137.5)[&(55)A(l)]/(137.5)[&(40)A(l)]A(l)" },
    },
    vars = { roll = {70, 110}, pitch = {42, 58}, pitch2 = {30, 46}, side = {0.80, 0.86}, side2 = {0.76, 0.84},
             lead = {0.80, 0.86} },
    params = {
      droop = {0.10, 0.20}, light = {0.15, 0.30}, crookChance = {0.15, 0.30}, crookAngle = {15, 30},
      meander = {0.08, 0.15}, lean = {0, 6}, angleJitter = {8, 14},
      flare = {0.5, 0.9}, lobes = {4, 7}, lobeDepth = {0.12, 0.22}, lobeHeight = {1.2, 2.2}, nodeSwell = {0.08, 0.15},
      barkNoise = {0.04, 0.08}, twist = {4, 12}, rootCount = 5, rootSpread = {2.5, 3.5}, collar = {0.5, 0.7},
      tipRadius = 0.012, pipeExponent = {2.2, 2.5}, radiusScale = 1.8, ringSegments = 10,
      leafSize = {0.36, 0.46}, leavesPerTip = 4, leafThickness = 0.05, leafClump = 0.9, maxLeafCards = 6000,
    },
    bark = "furrowed",
    bark_color = { {0.26, 0.21, 0.16}, {0.34, 0.27, 0.20} },
    leaf_color = { {0.14, 0.30, 0.08}, {0.20, 0.38, 0.11} },
    card = {
      -- oak leaves crowd at the end of each shoot: a couple along it, a rosette at its tip
      axiom = "S(2)", iterations = 3, variants = 2,
      rules = {
        { "S(n):n>0",  "F(0.13)+($bend)[+($a)L($s*0.8)]F(0.11)-($bend)[-($a)L($s*0.85)]S(n-1)" },
        { "S(n):n<=0", "F(0.06)[+($b)L($s*0.8)][-($b)L($s*0.8)][+($b*0.5)L($s*0.85)]L(0.95)" },
      },
      vars = { a = {48, 66}, b = {58, 76}, s = {0.85, 1.05}, bend = {-8, 8} },
      leaf = { shape = "lobed", length = 0.32, width = 0.13, lobes = 3 },
      stem_width = 0.014, cup = 0.5,
    },
  },

  -- BIRCH: one slim leader whorling light branches up its length, their twigs drooping; white bark, hardly fluted.
  birch = {
    axiom = "T(1.3,2)",
    iterations = 16,
    rules = {
      { "T(l,n):n>0",   "F(l)/(60)T(l*0.97,n-1)" },
      { "T(l,n):n<=0",  "C(l,9)" },
      { "C(l,n):n>0",   "F(l)/(137.5)[&($pitch)B(l*$side)]/(137.5)[&($pitch)B(l*$side)]C(l*$lead,n-1)" },
      { "C(l,n):n<=0",  "F(l)A(l)" },
      { "B(l):l>0.22",  "F(l)/(137.5)[&(35)B(l*0.70)]B(l*0.82)" },
      { "B(l):l<=0.22", "F(l)[&(55)A(l)][^(40)A(l)]A(l)" },
    },
    vars = { pitch = {35, 50}, side = {0.55, 0.65}, lead = {0.88, 0.92} },
    params = {
      droop = {0.25, 0.40}, light = {0.05, 0.15}, crookChance = {0.05, 0.12}, crookAngle = {8, 18},
      meander = {0.05, 0.10}, lean = {0, 8}, angleJitter = {8, 12},
      flare = 0.3, nodeSwell = 0.05, barkNoise = 0.03, twist = 0, rootCount = 3, rootSpread = 2.0, collar = 0.4,
      tipRadius = 0.010, pipeExponent = 2.3, radiusScale = 1.2, ringSegments = 8,
      leafSize = {0.38, 0.46}, leavesPerTip = 7, leafThickness = 0.035, leafClump = 0.9, maxLeafCards = 9000,
    },
    bark = "papery",
    bark_color = { {0.74, 0.72, 0.68}, {0.82, 0.80, 0.76} },
    leaf_color = { {0.20, 0.40, 0.10}, {0.27, 0.48, 0.12} },
    card = {
      -- small toothed leaves, alternate, well apart along a slender drooping twig
      axiom = "S(5)", iterations = 6, variants = 2,
      rules = {
        { "S(n):n>0",  "F(0.10)+($bend)[+($a)L($s)]F(0.10)+($bend)[-($a)L($s)]S(n-1)" },
        { "S(n):n<=0", "F(0.05)L(0.9)" },
      },
      vars = { a = {42, 56}, s = {0.85, 1.0}, bend = {-3, 2} },
      leaf = { shape = "serrate", length = 0.15, width = 0.10, teeth = 8 },
      stem_width = 0.008, cup = 0.4,
    },
  },

  -- SPRUCE: a straight whorled leader; each whorl's branches longest at the bottom (n counts down the tree), sagging
  -- and turning their tips back up; flat sprays of twigs off them.
  spruce = {
    axiom = "C($len,$whorls)",
    iterations = 38,
    rules = {
      { "C(l,n):n>0",   "F(l)/(137.5)[&($pitch)B(l*($reach*n/$whorls+0.30))]/(90)[&($pitch)B(l*($reach*n/$whorls+0.30))]/(90)[&($pitch)B(l*($reach*n/$whorls+0.30))]/(90)[&($pitch)B(l*($reach*n/$whorls+0.30))]C(l*0.99,n-1)" },
      { "C(l,n):n<=0",  "F(l)A(l)" },
      { "B(l):l>0.22",  "F(l)[+(50)A(l*0.6)][-(50)A(l*0.6)]/(90)[&(30)A(l*0.5)]B(l*0.82)" },
      { "B(l):l<=0.22", "F(l)A(l)" },
    },
    vars = { pitch = {58, 68}, reach = {1.6, 2.0}, len = {0.36, 0.48}, whorls = {26, 36} },
    params = {
      droop = {0.06, 0.12}, light = {0.10, 0.18}, crookChance = 0.0, crookAngle = 10,
      meander = 0.012, lean = {0, 2}, angleJitter = {6, 10},
      flare = 0.6, lobes = {5, 8}, lobeDepth = 0.10, lobeHeight = 1.0, nodeSwell = 0.04, barkNoise = 0.05,
      twist = 0, rootCount = 4, rootSpread = 2.0, collar = 0.3,
      tipRadius = 0.010, pipeExponent = 2.4, radiusScale = 1.4, ringSegments = 8,
      leafSize = {0.40, 0.50}, leavesPerTip = 4, leafThickness = 0.07, leafClump = 1.0, leafPitch = {18, 30},
      maxLeafCards = 16000,
    },
    bark = "plated",
    bark_color = { {0.22, 0.16, 0.12}, {0.28, 0.20, 0.15} },
    leaf_color = { {0.06, 0.16, 0.06}, {0.09, 0.21, 0.08} },
    -- a SPRAY: a shoot with side shoots off it, each a bottlebrush -- needles all round the twig, so seen flat they
    -- point every way off it, forward and back -- lighter at the tips where this year's growth is
    card = {
      axiom = "M(4)", iterations = 14, variants = 2,
      rules = {
        { "M(n):n>0",  "K(3)/(0)[+($side)K(6)][-($side)K(6)]M(n-1)" },
        { "M(n):n<=0", "K(5)" },
        { "K(n):n>0",  "F(0.026)+($bend)[+(78)L(1)][-(78)L(1)][+(52)L(0.95)][-(52)L(0.95)][+(108)L(0.75)][-(108)L(0.75)][+(26)L(0.85)][-(26)L(0.85)]K(n-1)" },
        { "K(n):n<=0", "F(0.012)[+(20)L(0.7)][-(20)L(0.7)]L(0.75)" },
      },
      vars = { side = {32, 46}, bend = {-3, 3} },
      leaf = { shape = "needle", length = 0.07, width = 0.008 },
      stem_width = 0.008, cup = 0.3, shade_jitter = 0.25, tip_bright = 0.35,
    },
  },

  -- SAKURA (Somei-yoshino): a short trunk splits into three scaffold limbs that climb and then spread, an umbrella
  -- wider than it is tall; dark bark banded with lenticels; in bloom the twigs are clusters of pale pink blossoms
  -- (each a deeper pink at its eye) with a few bronze young leaves.
  sakura = {
    axiom = "T(0.6,2)",
    iterations = 13,
    rules = {
      { "T(l,n):n>0",   "F(l)/(97)T(l*0.95,n-1)" },
      { "T(l,n):n<=0",  "[&($p0)B(l*1.6)]/(120)[&($p0)B(l*1.55)]/(120)[&($p0)B(l*1.5)]" },
      { "B(l):l>0.2",   "F(l)/($roll)[&($pitch)B(l*$side)]^($up)B(l*$lead)", 3 },
      { "B(l):l>0.2",   "F(l)/($roll)[&($pitch)B(l*$side)]/(180)[&($pitch)B(l*$side)]", 1 },
      { "B(l):l<=0.2",  "F(l)/(137.5)[&(50)A(l)]/(137.5)[&(45)A(l)]A(l)" },
    },
    vars = { p0 = {32, 44}, roll = {80, 110}, pitch = {35, 50}, side = {0.76, 0.84}, lead = {0.80, 0.86}, up = {4, 10} },
    params = {
      droop = {0.18, 0.28}, light = {0.15, 0.25}, crookChance = {0.10, 0.20}, crookAngle = {12, 22},
      meander = {0.08, 0.12}, lean = {0, 5}, angleJitter = {8, 12},
      flare = 0.45, nodeSwell = 0.08, barkNoise = 0.04, twist = 0, rootCount = 4, rootSpread = 2.5, collar = 0.5,
      tipRadius = 0.010, pipeExponent = 2.3, radiusScale = 1.6, ringSegments = 10,
      leafSize = {0.46, 0.54}, leavesPerTip = 4, leafThickness = 0.06, leafClump = 1.0, leafPitch = 50,
      maxLeafCards = 9000, paletteScale = 2.0,
    },
    bark = "ringed",
    bark_color = { {0.20, 0.13, 0.12}, {0.26, 0.17, 0.15} },
    leaf_color = {1, 1, 1},
    leaf_palette = { {1.0, 1.0, 1.0}, {1.0, 0.93, 0.96}, {0.96, 0.86, 0.90} },
    card = {
      -- in bloom the twigs vanish under BALLS of blossom: umbels of 5-7 flowers on short stalks, packed along the
      -- shoot, a bronze leaf showing here and there
      axiom = "U(3)", iterations = 5, variants = 2,
      rules = {
        { "U(n):n>0",  "F(0.055)+($bend)[+($a)G(1)][-($a)G(0.95)]U(n-1)", 3 },
        { "U(n):n>0",  "F(0.055)+($bend)[+($a)G(1)][-($a)G(0.95)][-(85)L(0.8)]U(n-1)", 1 },
        { "U(n):n<=0", "F(0.03)G(1.15)" },
        { "G(s)",      "[+(55)F(0.028*s)W(s)][+(22)F(0.04*s)W(s)][-(18)F(0.04*s)W(s*0.95)][-(52)F(0.028*s)W(s)][+(95)F(0.018*s)W(s*0.9)][-(95)F(0.018*s)W(s*0.9)][F(0.05*s)W(s)]" },
      },
      vars = { a = {40, 60}, bend = {-6, 6} },
      leaf = { shape = "oval", length = 0.07, width = 0.02, color = {0.55, 0.50, 0.28} },
      flower = { shape = "blossom", size = 0.05, petals = 5, notch = 0.22, eye = 0.22,
                 color = {1.0, 0.56, 0.70}, eye_color = {0.80, 0.16, 0.36} },
      stem_color = {0.62, 0.40, 0.42}, stem_width = 0.006, cup = 0.4,
    },
  },

  -- JAPANESE MAPLE (Acer palmatum): stems from low down, branches in OPPOSITE pairs (each pair turned 90 deg from
  -- the last), reaching out level in layered tiers with the tips turned up; seven-lobed palmate leaves. Each tree
  -- picks a palette -- crimson, orange, turning, or still green -- blended limb by limb across its crown.
  maple = {
    axiom = "T(0.5,1)",
    iterations = 10,
    rules = {
      { "T(l,n):n>0",   "F(l)T(l,n-1)" },
      { "T(l,n):n<=0",  "[&($s0)B(0.85)]/(120)[&($s0)B(0.8)]/(120)[&($s0)B(0.82)]" },
      { "B(l):l>0.18",  "F(l)/(90)[+($pitch)B(l*$side)][-($pitch)B(l*$side)]B(l*$lead)", 3 },
      { "B(l):l>0.18",  "F(l)/(90)[+($pitch)B(l*$side)]B(l*$lead)", 1 },
      { "B(l):l<=0.18", "F(l)/(90)[+(45)A(l)][-(45)A(l)]A(l)" },
    },
    vars = { s0 = {25, 38}, pitch = {50, 66}, side = {0.70, 0.76}, lead = {0.78, 0.84} },
    params = {
      droop = {0.12, 0.20}, light = {0.25, 0.35}, crookChance = {0.10, 0.20}, crookAngle = {10, 20},
      meander = {0.06, 0.10}, lean = {0, 6}, angleJitter = {8, 12},
      flare = 0.35, nodeSwell = 0.06, barkNoise = 0.03, twist = 0, rootCount = 3, rootSpread = 2.0, collar = 0.5,
      tipRadius = 0.008, pipeExponent = 2.4, radiusScale = 1.3, ringSegments = 8,
      leafSize = {0.34, 0.40}, leavesPerTip = 3, leafThickness = 0.025, leafClump = 1.0, leafPitch = 50,
      maxLeafCards = 9000, paletteScale = 1.8,
    },
    bark = "smooth",
    bark_color = { {0.26, 0.24, 0.22}, {0.32, 0.29, 0.26} },
    leaf_color = {1, 1, 1},
    leaf_palette = {
      { {0.42, 0.04, 0.03}, {0.58, 0.08, 0.04}, {0.34, 0.03, 0.05} },   -- crimson
      { {0.62, 0.22, 0.03}, {0.72, 0.38, 0.05}, {0.52, 0.10, 0.03} },   -- orange
      { {0.20, 0.34, 0.07}, {0.55, 0.38, 0.05}, {0.60, 0.14, 0.03} },   -- turning
      { {0.13, 0.30, 0.06}, {0.20, 0.38, 0.08}, {0.28, 0.40, 0.09} },   -- summer green
    },
    card = {
      -- leaves in opposite pairs on long stalks along the shoot, a pair and a single at its end
      axiom = "S(3)", iterations = 4, variants = 2,
      rules = {
        { "S(n):n>0",  "F(0.10)+($bend)[+($a)F(0.04)L($s)][-($a)F(0.04)L($s*0.95)]S(n-1)" },
        { "S(n):n<=0", "F(0.05)[+(40)F(0.03)L($s*0.9)][-(40)F(0.03)L($s*0.9)]F(0.03)L($s)" },
      },
      vars = { a = {50, 70}, s = {0.9, 1.05}, bend = {-5, 5} },
      leaf = { shape = "palmate", length = 0.17, lobes = 7, teeth = 3 },
      stem_color = {0.55, 0.30, 0.25}, stem_width = 0.008, cup = 0.35, shade_jitter = 0.22,
    },
  },

  -- JAPANESE BLACK PINE (Pinus thunbergii): a crooked, leaning trunk, bare below; a few long limbs fanning out
  -- level from its upper half, their ends gathered into dense, upturned clouds of long paired needles.
  pine = {
    axiom = "F($bare)C($len,$whorls)",
    iterations = 16,
    rules = {
      { "C(l,n):n>0",   "F(l)/(137.5)[&($pitch)B(l*$reach)]C(l*0.97,n-1)", 2 },
      { "C(l,n):n>0",   "F(l)/(137.5)C(l*0.97,n-1)", 1 },
      { "C(l,n):n<=0",  "F(l)B(l*0.6)" },
      { "B(l):l>0.25",  "F(l)/($roll)[+($fan)B(l*0.70)][-($fan)B(l*0.68)]B(l*0.80)", 2 },
      { "B(l):l>0.25",  "F(l)/($roll)[+($fan)B(l*0.70)]B(l*0.82)", 1 },
      { "B(l):l<=0.25", "F(l)[+(30)A(l)][-(30)A(l)][&(25)A(l)][^(25)A(l)]A(l)" },
    },
    vars = { bare = {1.6, 2.6}, len = {0.45, 0.6}, whorls = {7, 10}, pitch = {62, 80}, reach = {2.2, 2.8},
             roll = {70, 110}, fan = {30, 45} },
    params = {
      droop = {0.04, 0.08}, light = {0.25, 0.35}, crookChance = {0.25, 0.40}, crookAngle = {14, 24},
      meander = {0.10, 0.16}, lean = {4, 14}, angleJitter = {10, 16},
      flare = 0.5, lobes = {4, 6}, lobeDepth = 0.08, lobeHeight = 1.0, nodeSwell = 0.06, barkNoise = 0.06,
      twist = 6, rootCount = 4, rootSpread = 2.2, collar = 0.4,
      tipRadius = 0.010, pipeExponent = 2.7, radiusScale = 1.0, ringSegments = 9,
      leafSize = {0.40, 0.48}, leavesPerTip = 6, leafThickness = 0.04, leafClump = 0.8, leafPitch = {25, 40},
      maxLeafCards = 9000,
    },
    bark = "plated",
    bark_color = { {0.17, 0.14, 0.12}, {0.22, 0.18, 0.15} },
    leaf_color = { {0.07, 0.17, 0.07}, {0.10, 0.22, 0.09} },
    card = {
      -- shoots ending in brushes of long needles, pointing forward, lighter where this year's candle grew
      axiom = "M(2)", iterations = 10, variants = 2,
      rules = {
        { "M(n):n>0",  "K(2)[+($side)K(4)][-($side)K(4)]M(n-1)" },
        { "M(n):n<=0", "K(5)" },
        { "K(n):n>0",  "F(0.03)+($bend)[+($a)L(1)][-($a)L(1)][+($a*0.55)L(0.95)][-($a*0.55)L(0.95)][+($a*1.5)L(0.8)][-($a*1.5)L(0.8)]K(n-1)" },
        { "K(n):n<=0", "F(0.012)[+(12)L(0.85)][-(12)L(0.85)][+(26)L(0.8)][-(26)L(0.8)]L(0.9)" },
      },
      vars = { side = {28, 40}, a = {24, 34}, bend = {-3, 3} },
      leaf = { shape = "needle", length = 0.13, width = 0.007 },
      stem_color = {0.45, 0.35, 0.28}, stem_width = 0.009, cup = 0.3, shade_jitter = 0.25, tip_bright = 0.25,
    },
  },
}
