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
--     girth    tipRadius pipeExponent radiusScale ringSegments
--     leaves   leaves leafSize leavesPerTip leafThickness leafClump maxLeafCards
--   bark                the bark texture: "oak" (furrows), "birch" (pale, lenticels), "pine" (plates)
--   bark_color, leaf_color   {r, g, b} linear, or {{lo}, {hi}} drawn per tree
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
      barkNoise = {0.04, 0.08}, twist = {4, 12}, rootCount = 5, rootSpread = {2.5, 3.5},
      tipRadius = 0.012, pipeExponent = {2.2, 2.5}, radiusScale = 1.8, ringSegments = 10,
      leafSize = {0.36, 0.46}, leavesPerTip = 4, leafThickness = 0.05, leafClump = 0.9, maxLeafCards = 6000,
    },
    bark = "oak",
    bark_color = { {0.26, 0.21, 0.16}, {0.34, 0.27, 0.20} },
    leaf_color = { {0.14, 0.30, 0.08}, {0.20, 0.38, 0.11} },
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
      flare = 0.3, nodeSwell = 0.05, barkNoise = 0.03, twist = 0, rootCount = 3, rootSpread = 2.0,
      tipRadius = 0.010, pipeExponent = 2.3, radiusScale = 1.2, ringSegments = 8,
      leafSize = {0.28, 0.34}, leavesPerTip = 4, leafThickness = 0.035, leafClump = 0.8, maxLeafCards = 5000,
    },
    bark = "birch",
    bark_color = { {0.74, 0.72, 0.68}, {0.82, 0.80, 0.76} },
    leaf_color = { {0.20, 0.40, 0.10}, {0.27, 0.48, 0.12} },
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
      twist = 0, rootCount = 4, rootSpread = 2.0,
      tipRadius = 0.010, pipeExponent = 2.4, radiusScale = 1.4, ringSegments = 8,
      leafSize = {0.28, 0.34}, leavesPerTip = 3, leafThickness = 0.05, leafClump = 0.5, maxLeafCards = 9000,
    },
    bark = "pine",
    bark_color = { {0.22, 0.16, 0.12}, {0.28, 0.20, 0.15} },
    leaf_color = { {0.06, 0.16, 0.06}, {0.09, 0.21, 0.08} },
  },
}
