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
--     joints   joints = 1: every fork of wood (jointMinRadius on) one surface -- each limb cut back and the end rings
--              joined by their hull (ADR-0181); collar (a twig still welds into its parent, flaring by this);
--              trunkZone (m): the wood at least this thick from the foot, and the roots, ONE implicit surface instead
--              (ADR-0183), remeshed to trunkZoneTris even triangles every trunkZoneCell (m) -- the hull joins then off
--     girth    tipRadius pipeExponent radiusScale ringSegments
--     bark     barkMottle (soft dark patches) barkFootDark barkFootHeight (a dark base rising in tongues: a birch)
--     leaves   leaves leafSize leavesPerTip leafThickness leafClump (how far back along a twig its cards spread)
--              leafPitch (deg a card tilts out from its twig) maxLeafCards
--   bark                the bark (procgen/material_recipes.h): "furrowed" (oak), "plated" (pine, spruce), "smooth"
--                       (beech), "papery" (birch: white, dark dashes and patches), "ringed" (palms, cherry)
--   bark_color, leaf_color   {r, g, b} linear, or {{lo}, {hi}} drawn per tree
--   leaf_palette        colour by patches across the crown (params.paletteScale m across): a list of colours the
--                       cards blend along, or a list of such lists, each seed picking one (red maples, orange ones)
--   SHRUBS AND PERENNIALS are recipes too: the axiom fans many stems out of the ground (R(n) below), collar 0 and no
--   roots. params.tipCards puts the card atlas's first row at the stems' tips (flower spikes, daisies, berries) and
--   the other rows along them (leaves); the card grammar reads $row to know which it is drawing. clipX/Y/Z prune
--   the growth to a box (a clipped hedge: the cut twigs' leaves crowd its faces).
--   leaves              NEAR LEAVES (foliage = 2; procgen/foliage_kit.h): every leaf its own small blade wearing one leaf of
--                       the card's shape, folded on its midrib and curled at the tip, riding along leafy SHOOTS each leafy
--                       twig puts out -- { size (m), shoots, shoot_length, shoot_angle, lift, sag, per_shoot, pairs (1:
--                       opposite), angle, fold, curl, droop, face, budget, blossom = { size, per_cluster, clusters (per
--                       shoot), cup, budget } } (blossoms: small cups of the card's flower, in clusters along the shoots)
--   fronds              A PALM'S CROWN (params.foliage = 4; procgen/foliage_kit.h FrondSpec): { fan (0 pinnate, 1 fan),
--                       count, length, arch, rachis, leaflets, leaflet_length, leaflet_width, leaflet_angle,
--                       leaflet_droop, fan_spread, petiole, pitch_young, pitch_old, skirt, color, rachis_color,
--                       skirt_color }: fronds at the top of the stem, the young upright, the old hanging
--   kind = "blades"     A BLADE PLANT -- no grammar, no wood: strap leaves from a crown (procgen/foliage_kit.h BladeSpec),
--                       blades = { count, length, length_jitter, width, rise, spread (deg off upright, inner .. outer),
--                       fan (0 a round clump .. 1 a flat fan), arch, fold, twist, taper, crown, color, edge_color }
--   branch_cards        THE BRANCH CARDS (procgen/branch_cards.h): { sprig (m), wood_edge (m), ring_angle (deg), pictures,
--                       cell (px), fold (deg), bend } -- the wood modelled down to sprigs of `sprig` reach, its sides about
--                       wood_edge apart and a ring every ring_angle of bend; each sprig a card wearing a picture baked from
--                       the tree's own sprigs grown whole (`pictures` of them), its halves folded `fold` up off flat about
--                       its spine, its normals bent `bend` toward out of the crown. The hero's wood is held to ~8000 tris.
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
    -- FORM in a stand (forest.h ForestSpecies::form; the island's forests): the lower limbs shaded off, a clear bole
    forms = { stand = { axiom_prefix = "F(3.0)", scale = { params = { crownRadius = 0.6, crownBottom = 2.2 } }, params = { pruneBelow = 0.45 } } },
    axiom = "T(1.3,2)",
    iterations = 13,
    rules = {
      { "T(l,n):n>0",   "F(l)/(97)T(l*0.95,n-1)" },
      { "T(l,n):n<=0",  "B(l*$crown)" },
      { "B(l):l>$tip",  "F(l)/($roll)[&($pitch)B(l*$side)]/(180)[&($pitch2)B(l*$side2)]", 3 },
      { "B(l):l>$tip",  "F(l)/($roll)[&($pitch)B(l*$side)]^(10)B(l*$lead)", 2 },
      { "B(l):l<=$tip", "F(l)/(137.5)[&(55)A(l)]/(137.5)[&(55)A(l)]/(137.5)[&(40)A(l)]A(l)" },
    },
    vars = { roll = {70, 110}, pitch = {42, 58}, pitch2 = {30, 46}, side = {0.80, 0.86}, side2 = {0.76, 0.84},
             lead = {0.80, 0.86}, tip = 0.22, crown = 0.9 },
    params = {
      joints = 1, trunkZone = 0.10, trunkZoneTris = 2800,   -- (the trunk, its first limbs, its roots: one surface)
      droop = {0.10, 0.20}, light = {0.15, 0.30}, crookChance = {0.15, 0.30}, crookAngle = {15, 30},
      meander = {0.08, 0.15}, lean = {0, 6}, angleJitter = {8, 14},
      flare = {0.5, 0.9}, lobes = {4, 7}, lobeDepth = {0.12, 0.22}, lobeHeight = {1.2, 2.2}, nodeSwell = {0.08, 0.15},
      barkNoise = {0.04, 0.08}, twist = {4, 12}, rootCount = 5, rootSpread = {2.5, 3.5}, collar = {0.5, 0.7},
      tipRadius = 0.012, pipeExponent = {2.2, 2.5}, radiusScale = 1.8, ringSegments = 10,
      leafSize = {0.36, 0.46}, leavesPerTip = 4, leafThickness = 0.05, leafClump = 0.9, maxLeafCards = 6000,
    },
    -- AGES (tree_recipe.h TreeAge): the oak grown older -- a longer first stretch and a longer twig ($tip: the
    -- branching stops there -- so a bigger tree is a few more orders, not thousands more twigs), a girthier trunk, its
    -- limbs spreading and sagging lower, crooked, and some dead; an ancient one a short massive bole under a crown
    -- twice as wide as it is tall
    ages = {
      -- (REAL SIZES -- Glenn, 2026-10-18, against the scale lab's street: "Let's get the scale right": an open-grown
      -- English oak is ~9 m at thirty years, ~20 m mature, ~24 m old and broad, ~26 m and twice as wide ancient; the
      -- city's storey is 3.2 m, so a mature oak tops a five-storey walk-up)
      young   = { axiom = "T(1.9,2)", vars = { crown = 0.85, pitch = {36, 50}, tip = 0.17 }, sprig = 1.2,
                  near_leaves = 10000, near_twig_tris = 12000,   -- (a street tree's crown full, not tufts on bare forks)
                  params = { radiusScale = 2.0, deadChance = 0.01 } },
      mature  = { axiom = "T(1.5,2)", iterations = 12, vars = { tip = 0.9, crown = 1.2, pitch = {44, 58}, side = {0.82, 0.87} },
                  sprig = 1.8, near_leaves = 12000, near_twig_tris = 14000,
                  params = { colonize = 3, colonizePoints = 1500, colonizeStep = 0.50, colonizeShell = 0.35, lobeOverlap = 0.15,
                             crownRadius = 7.5, crownBottom = 4.0, crownTop = 18.0, lobeSize = 0.32,
                             crownWiden = 1.2, crownHeighten = 1.1, pipeExponent = 2.6,
                             radiusScale = 2.5, droop = {0.08, 0.14}, crookChance = {0.2, 0.35}, flare = {0.4, 0.6},
                             lobeDepth = {0.14, 0.22}, rootSpread = {3.0, 4.0}, deadChance = 0.03, trunkZone = 0.11,
                             trunkZoneTris = 3200 } },
      -- OLD and ANCIENT: the grammar grows only the SCAFFOLD (a long $tip: the trunk and a few massive limbs), and the
      -- rest of the crown is grown TO ITS SHAPE (colonize: toward points in a dome round it, most in its outer shell)
      -- -- the inside open to the big limbs, the fine branching and the leaves out in the shell (Glenn: "skip the
      -- tangle of branches ... I would expect more plumage and foliage")
      old     = { axiom = "T(1.6,2)", iterations = 12, vars = { tip = 1.0, crown = 1.3, pitch = {50, 66}, side = {0.83, 0.88} },
                  sprig = 2.0, near_leaves = 14000, near_twig_tris = 16000,
                  params = { colonize = 3, colonizePoints = 1500, colonizeStep = 0.50, colonizeShell = 0.35, lobeOverlap = 0.15,
                             crownRadius = 10.0, crownBottom = 3.8, crownTop = 21.5, lobeSize = 0.30,
                             crownWiden = 1.25, crownHeighten = 1.1, pipeExponent = 2.7,
                             radiusScale = 4.5, droop = {0.10, 0.18}, crookChance = {0.25, 0.40}, flare = {0.45, 0.65},
                             lobeDepth = {0.18, 0.28}, rootSpread = {3.5, 4.5}, deadChance = 0.06, trunkZone = 0.12,
                             trunkZoneTris = 3600 } },
      ancient = { axiom = "T(1.9,2)", iterations = 12, vars = { tip = 2.4, crown = 1.7, pitch = {58, 74}, pitch2 = {42, 58},
                                                               side = {0.85, 0.90}, side2 = {0.82, 0.88} },
                  sprig = 2.6, near_leaves = 20000, near_twig_tris = 22000,
                  params = { colonize = 3, colonizePoints = 3200, colonizeStep = 0.55, colonizeShell = 0.30, lobeOverlap = 0.15,
                             crownRadius = 15.0, crownBottom = 3.5, crownTop = 23.0, lobeSize = 0.26,
                             crownWiden = 1.3, crownHeighten = 1.05, pipeExponent = 2.7,
                             radiusScale = 6.0, droop = {0.12, 0.20}, meander = {0.04, 0.08}, crookChance = {0.10, 0.18}, flare = {0.55, 0.75},
                             lobes = {6, 9}, lobeDepth = {0.22, 0.34}, rootSpread = {3.0, 4.0}, rootCount = 6,
                             deadChance = 0.10, trunkZone = 0.14, trunkZoneTris = 4500, barkNoise = {0.07, 0.10} } },
    },
    bark = "furrowed",
    bark_color = { {0.26, 0.21, 0.16}, {0.34, 0.27, 0.20} },
    leaf_color = { {0.14, 0.30, 0.08}, {0.20, 0.38, 0.11} },
    leaves = { size = 0.11, shoots = 4, shoot_length = 0.30, per_shoot = 7, angle = 55, fold = 15, curl = 0.3, droop = 0.3,
               face = 0.6, budget = 30000 },
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
    -- (leafy shoots along the limbs up to 7 cm through, not only at their ends: a young oak's crown is leaf all
    -- through, not tufts at the tips of bare forks)
    branch_cards = { sprig = 0.95, fill_radius = 0.07, fill_spacing = 0.22 },
  },

  -- BIRCH: one slim leader whorling light branches up its length, their twigs drooping; white bark, hardly fluted.
  birch = {
    -- FORM in a stand (forest.h ForestSpecies::form; the island's forests): the lower limbs shaded off, a clear bole
    forms = { stand = { axiom_prefix = "F(3.0)", params = { pruneBelow = 0.45 } } },
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
      trunkZone = 0.06, trunkZoneTris = 2400,   -- (the trunk, its first limbs, its roots: one surface)
      joints = 1,
      droop = {0.25, 0.40}, light = {0.05, 0.15}, crookChance = {0.05, 0.12}, crookAngle = {8, 18},
      meander = {0.05, 0.10}, lean = {0, 8}, angleJitter = {8, 12},
      flare = 0.3, nodeSwell = 0.05, barkNoise = 0.03, barkMottle = 0.55, barkFootDark = 0.92, barkFootHeight = 1.8, twist = 0, rootCount = 3, rootSpread = 2.0, collar = 0.4,
      tipRadius = 0.010, pipeExponent = 2.3, radiusScale = 1.2, ringSegments = 8,
      leafSize = {0.38, 0.46}, leavesPerTip = 7, leafThickness = 0.035, leafClump = 0.9, maxLeafCards = 9000,
    },
    bark = "papery",
    bark_color = { {0.74, 0.72, 0.68}, {0.82, 0.80, 0.76} },
    leaf_color = { {0.20, 0.40, 0.10}, {0.27, 0.48, 0.12} },
    leaves = { size = 0.06, shoots = 4, shoot_length = 0.30, shoot_angle = 40, lift = 0.0, sag = 0.25, per_shoot = 9,
               angle = 50, fold = 10, curl = 0.25, droop = 0.55, face = 0.4, budget = 34000 },
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
    branch_cards = { sprig = 0.8 },
  },

  -- SPRUCE: a straight whorled leader; each whorl's branches longest at the bottom (n counts down the tree), sagging
  -- and turning their tips back up; flat sprays of twigs off them.
  spruce = {
    -- FORM in a stand (forest.h ForestSpecies::form; the island's forests): the lower limbs shaded off, a clear bole
    forms = { stand = { scale = { vars = { reach = 0.8 } }, params = { pruneBelow = 0.30 } } },
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
      trunkZone = 0.08, trunkZoneTris = 2400,   -- (the trunk, its first limbs, its roots: one surface)
      joints = 1,
      droop = {0.06, 0.12}, light = {0.10, 0.18}, crookChance = 0.0, crookAngle = 10,
      meander = 0.012, lean = {0, 2}, angleJitter = {6, 10},
      flare = 0.6, lobes = {5, 8}, lobeDepth = 0.10, lobeHeight = 1.0, nodeSwell = 0.04, barkNoise = 0.05,
      twist = 0, rootCount = 4, rootSpread = 2.0, collar = 0.3,
      tipRadius = 0.010, pipeExponent = 2.4, radiusScale = 1.4, ringSegments = 8,
      leafSize = {0.40, 0.50}, leavesPerTip = 4, leafThickness = 0.07, leafClump = 1.0, leafPitch = {18, 30},
      maxLeafCards = 16000,
    },
    -- AGES: a young spruce a narrow spire of few whorls; an old one tall and broad, full to the ground, its lower
    -- branches long and sweeping, the odd dead one low down
    ages = {
      -- (REAL SIZES -- Glenn, 2026-10-18: "Let's get the scale right": a Norway spruce is ~8 m at twenty years,
      -- ~25 m mature -- an eight-storey office -- and 32-40 m old, its crown staying narrow: ~8 m across at 25 m)
      young   = { vars = { whorls = {20, 24}, len = {0.40, 0.45}, reach = {1.4, 1.7} }, params = { radiusScale = 1.1 } },
      mature  = { iterations = 62, vars = { whorls = {50, 56}, len = {0.58, 0.64}, reach = {1.3, 1.6} }, sprig = 2.2,
                  near_leaves = 20000, near_twig_tris = 11000, params = { radiusScale = 3.0, droop = {0.08, 0.14}, deadChance = 0.03,
                                                     trunkZoneTris = 2800 } },
      old     = { iterations = 70, vars = { whorls = {58, 64}, len = {0.70, 0.76}, reach = {1.6, 1.9} }, sprig = 2.6, near_leaves = 22000,
                  near_twig_tris = 12000,
                  params = { radiusScale = 4.0, droop = {0.10, 0.16}, flare = 0.9, rootSpread = 2.6, deadChance = 0.05,
                             trunkZoneTris = 3000 } },
      ancient = { iterations = 76, vars = { whorls = {64, 70}, len = {0.80, 0.86}, reach = {1.8, 2.2} }, sprig = 3.4, near_leaves = 26000,
                  near_twig_tris = 14000,
                  params = { radiusScale = 5.5, droop = {0.12, 0.18}, flare = 1.1, rootSpread = 3.0, deadChance = 0.08,
                             trunkZoneTris = 3600, barkNoise = 0.07 } },
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
    -- NEAR FOLIAGE (foliage = 1; foliage_kit.h): a STRAND is one shoot drawn flat -- the orange-brown twig, needles
    -- angled forward off both sides (short ones where they point at you), a bud at the tip, lighter new growth --
    -- and a SPRAY is shoots built from it: three strand fins through each shoot's axis, side shoots fanned off a
    -- main one
    strand = {
      axiom = "K(13)", iterations = 15, variants = 2,
      rules = {
        -- three ways a stretch of twig carries its needles, drawn at random: never a regular comb
        { "K(n):n>0",  "F(0.018)[+($a)L(1)][-($a+6)L(0.95)][+($a*0.6)L(0.8)][-($a*0.55)L(0.8)][+(16)L(0.5)][-(12)L(0.45)]K(n-1)", 2 },
        { "K(n):n>0",  "F(0.018)[+($a+8)L(0.9)][-($a)L(1)][+($a*0.7)L(0.85)][-($a*0.7)L(0.7)][+($a*1.25)L(0.7)][-(20)L(0.5)]K(n-1)", 2 },
        { "K(n):n>0",  "F(0.018)[+($a*0.8)L(1)][-($a*1.2)L(0.85)][+($a*0.45)L(0.75)][-($a*0.5)L(0.9)][+(10)L(0.4)]K(n-1)", 1 },
        { "K(n):n<=0", "F(0.006)[+(24)L(0.65)][-(22)L(0.6)][+(10)L(0.5)]W(1)" },
      },
      vars = { a = {44, 56} },
      leaf = { shape = "needle", length = 0.075, width = 0.007, color = {0.26, 0.44, 0.32} },
      flower = { shape = "blossom", size = 0.022, petals = 7, notch = 0, eye = 0.4,
                 color = {0.50, 0.30, 0.16}, eye_color = {0.40, 0.22, 0.12} },
      stem_color = {0.80, 0.50, 0.28}, stem_width = 0.011, cup = 0.3, shade_jitter = 0.35, tip_bright = 0.3,
    },
    spray = {
      side_shoots = 9, side_angle = 50, side_length = 0.5, side_falloff = 0.9, lift = 20, fins = 2,
      aspect = 0.38, droop = 0.25, variants = 8, per_node = 2, budget = 3200,
      color = { {0.82, 0.82, 0.82}, {1.0, 1.0, 1.0} },
    },
    -- NEEDLES (Glenn's spruce sprig: a bottlebrush -- short stiff needles all round the twig, swept forward): every
    -- twig's needles and its side shoots in a fishbone in the branch's plane, this year's growth lighter at the ends
    needles = { kind = "brush", length = 0.025, width = 0.0019, density = 14, forward = 55,
                shoot_length = 0.26, side_shoots = 8, side_angle = 55, limb_radius = 0.035, hang = 0.5,
                -- (fronds along the thick inner branch: no seeing through the middle)
                frond_spacing = 0.3, frond_length = 0.55,
                color = { {0.08, 0.17, 0.12}, {0.12, 0.22, 0.16} }, tip_color = {0.22, 0.36, 0.20},
                twig_color = {0.30, 0.19, 0.11} },
    branch_cards = { sprig = 1.6, near_leaves = 16000 },   -- (needle card planes: two a "leaf")
  },

  -- SAKURA (Somei-yoshino): a short trunk splits into three scaffold limbs that climb and then spread, an umbrella
  -- wider than it is tall; dark bark banded with lenticels; in bloom the twigs are clusters of pale pink blossoms
  -- (each a deeper pink at its eye) with a few bronze young leaves.
  sakura = {
    -- (each seed its own size -- $trunk and $reach -- and AGES below: Glenn, 2026-10-18, "sakura trees should have
    -- different sizes right?")
    axiom = "T($trunk,2)",
    iterations = 13,
    rules = {
      { "T(l,n):n>0",   "F(l)/(97)T(l*0.95,n-1)" },
      { "T(l,n):n<=0",  "[&($p0)B(l*$reach)]/(120)[&($p0)B(l*$reach*0.97)]/(120)[&($p0)B(l*$reach*0.94)]" },
      { "B(l):l>$tip",  "F(l)/($roll)[&($pitch)B(l*$side)]^($up)B(l*$lead)", 3 },
      { "B(l):l>$tip",  "F(l)/($roll)[&($pitch)B(l*$side)]/(180)[&($pitch)B(l*$side)]", 1 },
      { "B(l):l<=$tip", "F(l)/(137.5)[&(50)A(l)]/(137.5)[&(45)A(l)]A(l)" },
    },
    vars = { trunk = {0.50, 0.72}, reach = {1.4, 1.8}, tip = 0.2,
             p0 = {32, 44}, roll = {80, 110}, pitch = {35, 50}, side = {0.76, 0.84}, lead = {0.80, 0.86}, up = {4, 10} },
    params = {
      trunkZone = 0.07, trunkZoneTris = 1600,   -- (the trunk, its first limbs, its roots: one surface)
      joints = 1,
      droop = {0.18, 0.28}, light = {0.15, 0.25}, crookChance = {0.10, 0.20}, crookAngle = {12, 22},
      meander = {0.08, 0.12}, lean = {0, 5}, angleJitter = {8, 12},
      flare = 0.45, nodeSwell = 0.08, barkNoise = 0.04, twist = 0, rootCount = 4, rootSpread = 2.5, collar = 0.5,
      tipRadius = 0.008, pipeExponent = 2.4, radiusScale = 1.15, ringSegments = 10,
      leafSize = {0.46, 0.54}, leavesPerTip = 4, leafThickness = 0.06, leafClump = 1.0, leafPitch = 50,
      maxLeafCards = 9000, paletteScale = 2.0,
    },
    -- AGES: a young sakura a slim sapling, its few limbs climbing; an old one a broad low umbrella on a girthy, crooked
    -- trunk, the limbs sagging toward the ground; an ancient one (Miharu's Takizakura, a thousand years) a massive
    -- fluted bole under a dome far wider than it is tall, its limbs sweeping down, some dead. The twig ($tip) grows
    -- with the tree, so a bigger one is a few more orders of branching, not thousands more twigs
    ages = {
      young   = { vars = { trunk = {0.38, 0.46}, reach = {1.1, 1.3}, p0 = {22, 32}, tip = 0.16 },
                  params = { radiusScale = 0.8, droop = {0.10, 0.16}, deadChance = 0 } },
      mature  = { params = { deadChance = 0.02 } },
      old     = { vars = { trunk = {0.80, 0.90}, reach = {2.1, 2.4}, p0 = {46, 58}, tip = 0.28 }, sprig = 2.2,
                  near_leaves = 12000, near_twig_tris = 14000,
                  params = { radiusScale = 2.0, droop = {0.20, 0.28}, crookChance = {0.2, 0.3}, flare = 0.65,
                             rootSpread = 3.0, deadChance = 0.05, trunkZoneTris = 2400 } },
      ancient = { vars = { trunk = {1.00, 1.10}, reach = {2.7, 3.0}, p0 = {52, 64}, tip = 0.32 }, sprig = 3.0,
                  near_leaves = 18000, near_twig_tris = 18000,
                  params = { radiusScale = 3.2, droop = {0.22, 0.30}, crookChance = {0.25, 0.35}, flare = 0.85,
                             lobes = {5, 8}, lobeDepth = 0.16, barkNoise = 0.06, rootSpread = 3.2, rootCount = 6,
                             deadChance = 0.08, trunkZoneTris = 3200 } },
    },
    bark = "ringed",
    bark_color = { {0.20, 0.13, 0.12}, {0.26, 0.17, 0.15} },
    leaf_color = {1, 1, 1},
    leaf_palette = { {1.0, 1.0, 1.0}, {1.0, 0.93, 0.96}, {0.96, 0.86, 0.90} },
    leaves = { size = 0.05, shoots = 6, shoot_length = 0.22, per_shoot = 1, angle = 50, fold = 12, curl = 0.2, droop = 0.3,
               face = 0.5, budget = 8000,
               blossom = { size = 0.035, per_cluster = 5, clusters = 5, cup = 0.35, budget = 32000 } },
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
    branch_cards = { sprig = 1.6, wood_edge = 0.065 },
  },

  -- JAPANESE MAPLE (Acer palmatum): stems from low down, branches in OPPOSITE pairs (each pair turned 90 deg from
  -- the last), reaching out level in layered tiers with the tips turned up; seven-lobed palmate leaves. Each tree
  -- picks a palette -- crimson, orange, turning, or still green -- blended limb by limb across its crown.
  maple = {
    -- FORM in a stand (forest.h ForestSpecies::form; the island's forests): the lower limbs shaded off, a clear bole
    forms = { stand = { axiom_prefix = "F(2.0)", params = { pruneBelow = 0.4 } } },
    -- (Glenn, 2026-10-18: "What about the maple tree? It looks pretty bad" -- it forked at half a metre and its
    -- limbs sagged to the ground: a heap of leaves. A Japanese maple: a clear trunk to head height, a few limbs
    -- rising into a broad vase wider than it is tall, the sprays held out in layers, the branching seen under them)
    axiom = "T(0.75,2)",
    iterations = 12,
    rules = {
      { "T(l,n):n>0",   "F(l)/(97)T(l*0.95,n-1)" },
      -- (each limb a bare stretch before it branches: the crown lifted off the ground, the limbs seen under it)
      { "T(l,n):n<=0",  "[&($s0)F($bare)B(1.1)]/(95)[&($s0)F($bare)B(1.0)]/(115)[&($s0)F($bare)B(1.05)]/(90)[&($s1)F($bare*0.6)B(1.0)]" },
      { "B(l):l>0.3",  "F(l)/(90)[+($pitch)B(l*$side)][-($pitch)B(l*$side)]^(4)B(l*$lead)", 3 },
      { "B(l):l>0.3",  "F(l)/(90)[+($pitch)B(l*$side)]^(4)B(l*$lead)", 1 },
      { "B(l):l<=0.3", "F(l)/(90)[+(45)A(l)][-(45)A(l)]A(l)" },
    },
    vars = { s0 = {44, 56}, s1 = {10, 18}, bare = {0.6, 0.9}, pitch = {42, 56}, side = {0.70, 0.76}, lead = {0.80, 0.85} },
    params = {
      trunkZone = 0.07, trunkZoneTris = 2400,   -- (the trunk, its first limbs, its roots: one surface)
      joints = 1,
      droop = {0.04, 0.08}, light = {0.22, 0.30}, crookChance = {0.10, 0.20}, crookAngle = {10, 20},
      meander = {0.06, 0.10}, lean = {0, 6}, angleJitter = {8, 12},
      flare = 0.35, nodeSwell = 0.06, barkNoise = 0.03, twist = 0, rootCount = 3, rootSpread = 2.0, collar = 0.5,
      tipRadius = 0.008, pipeExponent = 2.6, radiusScale = 0.95, ringSegments = 8,
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
    leaves = { size = 0.08, shoots = 3, shoot_length = 0.28, per_shoot = 8, pairs = 1, angle = 65, fold = 8, curl = 0.2,
               droop = 0.3, face = 0.7, budget = 30000 },
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
    branch_cards = { sprig = 2.2, near_twig_tris = 18000 },   -- (a maple's fine branching is dense: 3600 twigs),
  },

  -- JAPANESE BLACK PINE (Pinus thunbergii): a crooked, leaning trunk, bare below; a few long limbs fanning out
  -- level from its upper half, their ends gathered into dense, upturned clouds of long paired needles.
  pine = {
    -- FORM in a stand (forest.h ForestSpecies::form; the island's forests): the lower limbs shaded off, a clear bole
    forms = { stand = { params = { pruneBelow = 0.55 } } },
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
      trunkZone = 0.08, trunkZoneTris = 2000,   -- (the trunk, its first limbs, its roots: one surface)
      joints = 1,
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
    -- NEEDLES (Glenn's pine sprig: long needles in bundles of two or three, crowded round the twig ends -- the inner
    -- twig bare): a tuft at every twig's end, needles along its last 45%
    needles = { kind = "tuft", length = 0.12, width = 0.0022, density = 3, forward = 35, side_shoots = 0, reach = 0.45,
                frond_spacing = 0.45, frond_length = 0.5,   -- (tufted side twigs along the bare inner branch)
                color = { {0.06, 0.15, 0.06}, {0.09, 0.19, 0.08} }, tip_color = {0.14, 0.26, 0.10},
                twig_color = {0.26, 0.20, 0.15} },
    branch_cards = { sprig = 1.0 },
  },

  ---------------------------------------------------------------------------------------------------------------
  -- SHRUBS AND PERENNIALS (Glenn's street-planting photos, 2026-10-17)

  -- INDIAN HAWTHORN (Rhaphiolepis): a dense rounded mound of glossy oval leaves, the new growth red-bronze,
  -- clusters of blue-black berries at the shoot tips.
  hawthorn = {
    kind = "shrub",
    axiom = "R(9)",
    iterations = 9,
    rules = {
      { "R(n):n>0",     "/(137.5)[&($spread*(0.35+0.65*n/9))S($len)]R(n-1)" },
      { "R(n):n<=0",    "S($len)" },
      { "S(l):l>0.1",   "F(l)/(90)[+($fork)S(l*$side)][-($fork)S(l*$side)]S(l*$lead)", 2 },
      { "S(l):l>0.1",   "F(l)/(137.5)[&($fork)S(l*$side)]S(l*$lead)", 1 },
      { "S(l):l<=0.1",  "F(l)A(l)" },
    },
    vars = { spread = {45, 70}, len = {0.26, 0.32}, fork = {30, 42}, side = {0.72, 0.80}, lead = {0.80, 0.86} },
    params = {
      droop = {0.08, 0.14}, light = 0.2, meander = 0.10, angleJitter = 10, collar = 0, rootCount = 0, flare = 0,
      tipRadius = 0.004, pipeExponent = 2.5, radiusScale = 1.0, ringSegments = 5,
      leafSize = {0.20, 0.24}, leavesPerTip = 4, leafThickness = 0.012, leafClump = 1.0, leafPitch = 50,
      maxLeafCards = 7000, paletteScale = 0.45, tipCards = 1,
    },
    bark = "smooth",
    bark_color = {0.30, 0.25, 0.20},
    leaf_color = { {0.92, 0.92, 0.92}, {1.0, 1.0, 1.0} },
    leaf_palette = { {0.62, 0.72, 0.55}, {0.75, 0.85, 0.62}, {1.0, 0.95, 0.60}, {1.0, 0.55, 0.45} },
    -- near (a forest's understory, 2026-10-17): glossy leaves a few centimetres long in rosettes, berry umbels
    leaves = { size = 0.055, shoots = 3, shoot_length = 0.10, per_shoot = 6, angle = 55, fold = 10, curl = 0.15,
               droop = 0.15, face = 0.6, budget = 9000,
               blossom = { size = 0.012, per_cluster = 6, clusters = 1, cup = 0.6, budget = 3000 } },
    card = {
      axiom = "X($row)", iterations = 2, variants = 2,
      rules = {
        -- the tips: a rosette with a cluster of berries
        { "X(r):r<0.5",  "F(0.05)[+(55)L(0.85)][-(55)L(0.85)]F(0.04)[+(28)L(1)][-(28)L(1)]F(0.03)[+(30)F(0.025)W(1)][-(22)F(0.03)W(1)][F(0.035)W(1)][+(65)F(0.02)W(0.9)][-(60)F(0.02)W(0.9)]" },
        -- along the stems: alternate glossy leaves
        { "X(r):r>=0.5", "F(0.05)[+(55)L(0.9)][-(55)L(0.85)]F(0.05)[+(45)L(1)][-(45)L(1)]F(0.04)[+(25)L(1)][-(25)L(1)]L(1.05)" },
      },
      leaf = { shape = "oval", length = 0.12, width = 0.05, color = {0.22, 0.40, 0.10} },
      flower = { shape = "blossom", size = 0.028, petals = 8, notch = 0, eye = 0.3,
                 color = {0.10, 0.10, 0.22}, eye_color = {0.06, 0.06, 0.14} },
      stem_color = {0.40, 0.30, 0.25}, stem_width = 0.008, cup = 0.55, shade_jitter = 0.25,
    },
  },

  -- MEXICAN SAGE (Salvia leucantha): a fountain of long arching stems, grey-green lance leaves, each stem ending in
  -- a long velvety purple spike.
  mexican_sage = {
    kind = "shrub",
    axiom = "R(16)",
    iterations = 10,
    rules = {
      { "R(n):n>0",     "/(137.5)[&($spread*(0.25+0.75*n/16))S($len)]R(n-1)" },
      { "R(n):n<=0",    "S($len)" },
      { "S(l):l>0.12",  "F(l)S(l*0.92)", 3 },
      { "S(l):l>0.12",  "F(l)/(137.5)[&(35)S(l*0.65)]S(l*0.9)", 1 },
      { "S(l):l<=0.12", "F(l)A(l)" },
    },
    vars = { spread = {30, 55}, len = {0.20, 0.25} },
    params = {
      droop = {0.12, 0.18}, light = 0.25, meander = 0.10, angleJitter = 10, collar = 0, rootCount = 0, flare = 0,
      tipRadius = 0.005, pipeExponent = 2.6, radiusScale = 1.0, ringSegments = 5,
      leafSize = {0.26, 0.32}, leavesPerTip = 3, leafThickness = 0.010, leafClump = 1.0, leafPitch = 30,
      maxLeafCards = 4000, tipCards = 1,
    },
    bark = "smooth",
    bark_color = {0.35, 0.38, 0.30},
    leaf_color = { {0.90, 0.90, 0.90}, {1.0, 1.0, 1.0} },
    card = {
      axiom = "X($row)", iterations = 18, variants = 2,
      rules = {
        { "X(r):r<0.5",  "F(0.03)P(14)" },
        { "P(n):n>0",    "F(0.022)+($bend)[+(38)W(0.55+0.03*n)][-(38)W(0.55+0.03*n)]P(n-1)" },
        { "P(n):n<=0",   "W(0.5)" },
        { "X(r):r>=0.5", "F(0.06)[+(28)L(1)][-(28)L(1)]F(0.06)[+(24)L(0.95)][-(24)L(0.95)]F(0.05)[+(20)L(0.85)][-(20)L(0.85)]L(0.8)" },
      },
      vars = { bend = {-3, 3} },
      leaf = { shape = "leaflet", length = 0.22, width = 0.035, color = {0.30, 0.40, 0.24} },
      flower = { shape = "floret", size = 0.045, color = {0.48, 0.16, 0.68}, eye_color = {0.60, 0.30, 0.80} },
      stem_color = {0.40, 0.20, 0.52}, stem_width = 0.014, cup = 0.3,
    },
  },

  -- BLUE SALVIA (Salvia farinacea): a compact clump of upright stems, each a slim violet-blue spike over pale
  -- mealy stalks, broad green leaves low down.
  blue_salvia = {
    kind = "shrub",
    axiom = "R(12)",
    iterations = 8,
    rules = {
      { "R(n):n>0",     "/(137.5)[&($spread*(0.2+0.8*n/12))S($len)]R(n-1)" },
      { "R(n):n<=0",    "S($len)" },
      { "S(l):l>0.08",  "F(l)S(l*0.9)", 3 },
      { "S(l):l>0.08",  "F(l)/(137.5)[&(25)S(l*0.8)]S(l*0.9)", 1 },
      { "S(l):l<=0.08", "F(l)A(l)" },
    },
    vars = { spread = {14, 30}, len = {0.11, 0.14} },
    params = {
      droop = 0.04, light = 0.15, meander = 0.08, angleJitter = 8, collar = 0, rootCount = 0, flare = 0,
      tipRadius = 0.004, pipeExponent = 2.6, radiusScale = 1.0, ringSegments = 5,
      leafSize = {0.18, 0.22}, leavesPerTip = 2, leafThickness = 0.0, leafClump = 0.3, leafPitch = 8,
      maxLeafCards = 2000, tipCards = 1,
    },
    bark = "smooth",
    bark_color = {0.30, 0.36, 0.24},
    leaf_color = { {0.90, 0.90, 0.90}, {1.0, 1.0, 1.0} },
    card = {
      axiom = "X($row)", iterations = 16, variants = 2,
      rules = {
        { "X(r):r<0.5",  "F(0.10)P(12)" },
        { "P(n):n>0",    "F(0.02)+($bend)[+(30)W(0.6+0.03*n)][-(30)W(0.6+0.03*n)]P(n-1)" },
        { "P(n):n<=0",   "W(0.5)" },
        { "X(r):r>=0.5", "F(0.03)[+(50)L(1)][-(50)L(1)][+(25)L(0.9)][-(25)L(0.9)]F(0.04)[+(35)L(0.8)][-(35)L(0.8)]" },
      },
      vars = { bend = {-2, 2} },
      leaf = { shape = "oval", length = 0.16, width = 0.05, color = {0.20, 0.36, 0.12} },
      flower = { shape = "floret", size = 0.04, color = {0.20, 0.20, 0.78}, eye_color = {0.32, 0.30, 0.85} },
      stem_color = {0.55, 0.58, 0.72}, stem_width = 0.012, cup = 0.35,
    },
  },

  -- EURYOPS (golden daisy bush): a dense low mound of fine bright-green leaves, dotted all over with yellow daisies
  -- held just above it.
  euryops = {
    kind = "shrub",
    axiom = "R(10)",
    iterations = 9,
    rules = {
      { "R(n):n>0",     "/(137.5)[&($spread*(0.35+0.65*n/10))S($len)]R(n-1)" },
      { "R(n):n<=0",    "S($len)" },
      { "S(l):l>0.09",  "F(l)/(90)[+($fork)S(l*0.76)][-($fork)S(l*0.76)]S(l*0.84)", 2 },
      { "S(l):l>0.09",  "F(l)/(90)[+($fork)S(l*0.76)]S(l*0.86)", 1 },
      { "S(l):l<=0.09", "F(l)A(l)" },
    },
    vars = { spread = {55, 75}, len = {0.22, 0.27}, fork = {28, 38} },
    params = {
      droop = 0.10, light = 0.25, meander = 0.08, angleJitter = 10, collar = 0, rootCount = 0, flare = 0,
      tipRadius = 0.004, pipeExponent = 2.5, radiusScale = 1.0, ringSegments = 5,
      leafSize = {0.20, 0.24}, leavesPerTip = 3, leafThickness = 0.010, leafClump = 1.0, leafPitch = 40,
      maxLeafCards = 7000, tipCards = 1, paletteScale = 0.6,
    },
    bark = "smooth",
    bark_color = {0.32, 0.30, 0.22},
    leaf_color = { {0.92, 0.92, 0.92}, {1.0, 1.0, 1.0} },
    leaf_palette = { {0.80, 0.90, 0.70}, {1.0, 1.0, 1.0} },
    -- near: fine grey-green leaflets up the shoots, a daisy over each
    leaves = { size = 0.05, shoots = 3, shoot_length = 0.12, per_shoot = 8, angle = 40, fold = 20, curl = 0.1,
               droop = 0.05, face = 0.3, budget = 9000,
               blossom = { size = 0.035, per_cluster = 1, clusters = 1, cup = 0.12, budget = 2500 } },
    card = {
      axiom = "X($row)", iterations = 2, variants = 2,
      rules = {
        { "X(r):r<0.5",  "F(0.05)[+(45)L(1)][-(45)L(1)][+(20)L(1)][-(20)L(1)]F(0.04)[+(30)L(0.9)][-(30)L(0.9)]F(0.10)W(1)" },
        { "X(r):r>=0.5", "F(0.04)[+(50)L(1)][-(50)L(1)][+(25)L(1)][-(25)L(1)]F(0.04)[+(40)L(0.9)][-(40)L(0.9)][+(15)L(0.9)][-(15)L(0.9)]L(0.9)" },
      },
      leaf = { shape = "leaflet", length = 0.12, width = 0.018, color = {0.20, 0.42, 0.08} },
      flower = { shape = "daisy", size = 0.10, petals = 13, eye = 0.28,
                 color = {1.0, 0.80, 0.05}, eye_color = {0.88, 0.55, 0.03} },
      stem_color = {0.25, 0.40, 0.12}, stem_width = 0.008, cup = 0.3,
    },
  },

  -- 'HOT LIPS' SALVIA (Salvia microphylla): an open, twiggy shrub of small round leaves; at the shoot tips little
  -- two-lipped flowers, white hoods over red lips.
  hot_lips = {
    kind = "shrub",
    axiom = "R(8)",
    iterations = 9,
    rules = {
      { "R(n):n>0",     "/(137.5)[&($spread*(0.3+0.7*n/8))S($len)]R(n-1)" },
      { "R(n):n<=0",    "S($len)" },
      { "S(l):l>0.1",   "F(l)/(90)[+($fork)S(l*0.74)]S(l*0.86)", 2 },
      { "S(l):l>0.1",   "F(l)/(90)[+($fork)S(l*0.74)][-($fork)S(l*0.74)]S(l*0.84)", 1 },
      { "S(l):l<=0.1",  "F(l)A(l)" },
    },
    vars = { spread = {35, 60}, len = {0.26, 0.32}, fork = {25, 35} },
    params = {
      droop = 0.10, light = 0.2, meander = 0.12, angleJitter = 12, collar = 0, rootCount = 0, flare = 0,
      tipRadius = 0.003, pipeExponent = 2.5, radiusScale = 1.0, ringSegments = 5,
      leafSize = {0.20, 0.24}, leavesPerTip = 3, leafThickness = 0.008, leafClump = 1.0, leafPitch = 35,
      maxLeafCards = 5000, tipCards = 1,
    },
    bark = "smooth",
    bark_color = {0.30, 0.28, 0.20},
    leaf_color = { {0.90, 0.90, 0.90}, {1.0, 1.0, 1.0} },
    card = {
      axiom = "X($row)", iterations = 2, variants = 2,
      rules = {
        { "X(r):r<0.5",  "F(0.06)[+(50)L(0.8)][-(50)L(0.8)]F(0.07)[+(40)F(0.02)W(1)][-(35)F(0.02)W(1)]F(0.05)[+(30)F(0.02)W(0.95)][-(30)F(0.02)W(0.9)]F(0.03)W(0.85)" },
        { "X(r):r>=0.5", "F(0.05)[+(55)L(1)][-(55)L(1)]F(0.05)[+(50)L(0.95)][-(50)L(0.95)]F(0.05)[+(45)L(0.9)][-(45)L(0.9)]L(0.85)" },
      },
      leaf = { shape = "oval", length = 0.06, width = 0.035, color = {0.26, 0.44, 0.12} },
      flower = { shape = "floret", size = 0.06, color = {0.98, 0.97, 0.96}, eye_color = {0.88, 0.04, 0.10} },
      stem_color = {0.35, 0.30, 0.22}, stem_width = 0.006, cup = 0.4,
    },
  },

  -- A CLIPPED HEDGE (pittosporum / xylosma): a dense shrub grown past its box and sheared back to it -- the cut
  -- twigs' leaves crowd the faces. One plant fills 1.8 m of a row.
  hedge = {
    kind = "shrub",
    axiom = "R(10)",
    iterations = 10,
    rules = {
      { "R(n):n>0",     "/(137.5)[&($spread*(0.3+0.7*n/10))S($len)]R(n-1)" },
      { "R(n):n<=0",    "S($len)" },
      { "S(l):l>0.08",  "F(l)/(90)[+($fork)S(l*0.78)][-($fork)S(l*0.78)]S(l*0.86)" },
      { "S(l):l<=0.08", "F(l)A(l)" },
    },
    vars = { spread = {35, 60}, len = {0.34, 0.40}, fork = {30, 40} },
    params = {
      droop = 0.05, light = 0.15, meander = 0.08, angleJitter = 12, collar = 0, rootCount = 0, flare = 0,
      tipRadius = 0.004, pipeExponent = 2.5, radiusScale = 1.0, ringSegments = 5,
      leafSize = {0.20, 0.24}, leavesPerTip = 3, leafThickness = 0.010, leafClump = 1.0, leafPitch = 55,
      maxLeafCards = 9000, paletteScale = 0.5, clipX = 0.95, clipY = 1.6, clipZ = 0.6,
    },
    bark = "smooth",
    bark_color = {0.30, 0.26, 0.20},
    leaf_color = { {0.92, 0.92, 0.92}, {1.0, 1.0, 1.0} },
    leaf_palette = { {0.55, 0.65, 0.50}, {0.70, 0.80, 0.55}, {1.0, 0.95, 0.55} },
    card = {
      axiom = "F(0.04)S(4)", iterations = 5, variants = 2,
      rules = {
        { "S(n):n>0",  "F(0.035)+($bend)[+($a)L(1)][-($a)L(0.95)]S(n-1)" },
        { "S(n):n<=0", "F(0.02)[+(30)L(0.9)][-(30)L(0.9)]L(1)" },
      },
      vars = { a = {45, 65}, bend = {-8, 8} },
      leaf = { shape = "oval", length = 0.08, width = 0.04, color = {0.16, 0.34, 0.07} },
      stem_color = {0.35, 0.28, 0.20}, stem_width = 0.006, cup = 0.5, shade_jitter = 0.3,
    },
  },

  ---------------------------------------------------------------------------------------------------------------
  -- BLADE PLANTS (Glenn's street-planting photos, 2026-10-17)

  -- NEW ZEALAND FLAX (Phormium): a fan of long stiff strap leaves, keeled, the outer ones arching over at the tip;
  -- olive with bronze-red margins.
  flax = {
    kind = "blades",
    blades = { count = 46, length = 1.6, length_jitter = 0.3, width = 0.08, rise = 4, spread = 42, fan = 0.8,
               arch = 0.45, fold = 34, twist = 0.25, taper = 0.65, crown = 0.18,
               color = { {0.19, 0.26, 0.09}, {0.25, 0.31, 0.12} }, edge_color = {0.30, 0.17, 0.09} },
  },

  -- CAPE RUSH (Elegia / Chondropetalum): a dense upright clump of fine, nearly round stems, a few bending out.
  rush = {
    kind = "blades",
    blades = { count = 130, length = 1.25, length_jitter = 0.3, width = 0.012, rise = 2, spread = 16, fan = 0,
               arch = 0.25, fold = 65, twist = 0, taper = 0.92, crown = 0.2,
               color = { {0.20, 0.27, 0.12}, {0.27, 0.32, 0.15} } },
  },

  -- BLUE FESCUE: a low round tuft of fine blue-grey blades fountaining out.
  fescue = {
    kind = "blades",
    blades = { count = 90, length = 0.32, length_jitter = 0.3, width = 0.007, rise = 8, spread = 60, fan = 0,
               arch = 0.7, fold = 50, twist = 0.1, taper = 0.8, crown = 0.08,
               color = { {0.28, 0.36, 0.37}, {0.36, 0.42, 0.42} } },
  },

  ---------------------------------------------------------------------------------------------------------------
  -- PALMS (Glenn: "the palm leaves ... don't quite look like palm leaves or trunks")

  -- COCONUT PALM: a slim ringed trunk curving up out of a lean, a round crown of long arching pinnate fronds,
  -- their leaflets hanging.
  coconut_palm = {
    axiom = "T($len,$segs)",
    iterations = 24,
    rules = {
      { "T(l,n):n>0",  "F(l)&($bend)/(7)T(l*0.995,n-1)" },
      { "T(l,n):n<=0", "F(l*0.5)A(l)" },
    },
    vars = { len = {0.45, 0.55}, segs = {13, 17}, bend = {1.0, 2.6} },
    params = {
      droop = 0, light = 0.2, meander = 0.04, lean = {6, 16}, angleJitter = 1,
      flare = 0.45, nodeSwell = 0.03, barkNoise = 0.02, twist = 0, rootCount = 0, collar = 0, joints = 0,
      tipRadius = 0.13, pipeExponent = 2.0, radiusScale = 1.0, ringSegments = 12, barkVScale = 2.5,
      foliage = 4,
    },
    bark = "ringed",
    bark_color = { {0.34, 0.30, 0.25}, {0.40, 0.35, 0.29} },
    fronds = { count = 22, length = 3.8, arch = 1.0, rachis = 0.03, leaflets = 44, leaflet_length = 0.8,
               leaflet_width = 0.045, leaflet_angle = 58, leaflet_droop = 0.7, pitch_young = 22, pitch_old = 112,
               color = { {0.17, 0.31, 0.08}, {0.24, 0.38, 0.10} }, rachis_color = {0.48, 0.44, 0.26} },
  },

  -- FAN PALM (Washingtonia): a tall straight trunk, a crown of round fan leaves on long petioles, and a skirt of
  -- dead brown fronds hanging under it.
  fan_palm = {
    axiom = "T($len,$segs)",
    iterations = 34,
    rules = {
      { "T(l,n):n>0",  "F(l)&($bend)/(11)T(l,n-1)" },
      { "T(l,n):n<=0", "F(l*0.5)A(l)" },
    },
    vars = { len = {0.45, 0.55}, segs = {20, 28}, bend = {0.0, 0.6} },
    params = {
      droop = 0, light = 0.1, meander = 0.02, lean = {0, 4}, angleJitter = 0.5,
      flare = 0.6, nodeSwell = 0.02, barkNoise = 0.03, twist = 0, rootCount = 0, collar = 0, joints = 0,
      tipRadius = 0.16, pipeExponent = 2.0, radiusScale = 1.0, ringSegments = 12, barkVScale = 2.0,
      foliage = 4,
    },
    bark = "ringed",
    bark_color = { {0.30, 0.26, 0.22}, {0.36, 0.31, 0.26} },
    fronds = { fan = 1, count = 26, length = 2.4, arch = 0.35, rachis = 0.025, leaflets = 46, petiole = 0.55,
               leaflet_width = 0.05, leaflet_droop = 0.9, fan_spread = 210, pitch_young = 20, pitch_old = 100,
               skirt = 18, color = { {0.18, 0.33, 0.12}, {0.24, 0.38, 0.14} }, rachis_color = {0.40, 0.36, 0.22},
               skirt_color = {0.36, 0.27, 0.17} },
  },

  ---------------------------------------------------------------------------------------------------------------
  -- STREET AND PARK TREES (Glenn's walking photos, 2026-10-17)

  -- THE MULTI-STEM STREET TREE (an Australian willow / peppermint kind): a short pale fluted bole splitting low into
  -- three or four leaning, sinuous leaders, the crown open -- narrow grey-green leaves in sparse tufts at the ends,
  -- the limbs showing through.
  multistem = {
    -- FORM in a stand (forest.h ForestSpecies::form; the island's forests): the lower limbs shaded off, a clear bole
    forms = { stand = { axiom_prefix = "F(1.5)", params = { pruneBelow = 0.25 } } },
    axiom = "T(0.7,2)",
    iterations = 13,
    rules = {
      { "T(l,n):n>0",   "F(l)/(83)T(l*0.95,n-1)" },
      { "T(l,n):n<=0",  "[&($s0)E(1.25)]/(95)[&($s0)E(1.2)]/(110)[&($s1)E(1.3)]/(90)[&($s1*0.6)E(1.1)]" },
      { "E(l):l>0.25", "F(l)/($roll)[&($pitch)E(l*$side)]^(6)E(l*$lead)", 3 },
      { "E(l):l>0.25", "F(l)/($roll)E(l*0.9)", 1 },
      { "E(l):l<=0.25","F(l)/(137.5)[&(40)A(l)]A(l)" },
    },
    vars = { s0 = {18, 30}, s1 = {28, 40}, roll = {80, 120}, pitch = {28, 40}, side = {0.64, 0.72}, lead = {0.80, 0.86} },
    params = {
      trunkZone = 0.06, trunkZoneTris = 2400,   -- (the trunk, its first limbs, its roots: one surface)
      joints = 1,
      droop = {0.10, 0.18}, light = {0.25, 0.35}, crookChance = {0.20, 0.35}, crookAngle = {14, 26},
      meander = {0.14, 0.22}, lean = {2, 10}, angleJitter = {10, 16},
      flare = 0.6, lobes = {5, 8}, lobeDepth = {0.16, 0.26}, lobeHeight = {1.4, 2.2}, nodeSwell = 0.08, barkNoise = 0.07,
      twist = {20, 40}, rootCount = 5, rootSpread = 2.6, collar = 0.5,
      tipRadius = 0.009, pipeExponent = 2.4, radiusScale = 1.4, ringSegments = 10, barkMottle = 0.25,
      leafSize = {0.34, 0.40}, leavesPerTip = 5, leafThickness = 0.02, leafClump = 0.6, maxLeafCards = 5000,
    },
    bark = "smooth",
    bark_color = { {0.50, 0.47, 0.42}, {0.58, 0.55, 0.50} },
    leaf_color = { {0.22, 0.30, 0.18}, {0.28, 0.35, 0.22} },
    leaves = { size = 0.10, shoots = 3, shoot_length = 0.35, shoot_angle = 35, lift = 0.4, sag = 0.12, per_shoot = 9,
               angle = 35, fold = 18, curl = 0.25, droop = 0.35, face = 0.3, budget = 22000 },
    card = {
      axiom = "S(4)", iterations = 5, variants = 2,
      rules = {
        { "S(n):n>0",  "F(0.08)+($bend)[+($a)L($s)][-($a)L($s*0.95)]S(n-1)" },
        { "S(n):n<=0", "F(0.04)[+(20)L(0.9)][-(20)L(0.9)]L(1)" },
      },
      vars = { a = {25, 40}, s = {0.9, 1.05}, bend = {-6, 6} },
      leaf = { shape = "leaflet", length = 0.20, width = 0.03 },
      stem_width = 0.007, cup = 0.4,
    },
    branch_cards = { sprig = 1.3 },
  },

  -- RIVER BIRCH (the street row's tree): one trunk, a dense oval crown with drooping edges, small toothed leaves
  -- turning yellow here and there; the bark tan and peeling.
  river_birch = {
    -- FORM in a stand (forest.h ForestSpecies::form; the island's forests): the lower limbs shaded off, a clear bole
    forms = { stand = { axiom_prefix = "F(2.5)", params = { pruneBelow = 0.4 } } },
    axiom = "T(1.2,2)",
    iterations = 16,
    rules = {
      { "T(l,n):n>0",   "F(l)/(70)T(l*0.96,n-1)" },
      { "T(l,n):n<=0",  "C(l,8)" },
      { "C(l,n):n>0",   "F(l)/(137.5)[&($pitch)B(l*$side)]/(137.5)[&($pitch)B(l*$side*0.9)]C(l*$lead,n-1)" },
      { "C(l,n):n<=0",  "F(l)A(l)" },
      { "B(l):l>0.2",   "F(l)/(137.5)[&(38)B(l*0.72)]B(l*0.84)" },
      { "B(l):l<=0.2",  "F(l)[&(55)A(l)][^(35)A(l)]A(l)" },
    },
    vars = { pitch = {40, 55}, side = {0.66, 0.74}, lead = {0.86, 0.90} },
    params = {
      trunkZone = 0.06, trunkZoneTris = 2400,   -- (the trunk, its first limbs, its roots: one surface)
      joints = 1,
      droop = {0.28, 0.40}, light = {0.08, 0.15}, crookChance = {0.05, 0.12}, crookAngle = {8, 16},
      meander = {0.05, 0.10}, lean = {0, 5}, angleJitter = {8, 12},
      flare = 0.35, nodeSwell = 0.05, barkNoise = 0.05, rootCount = 3, rootSpread = 2.0, collar = 0.4,
      tipRadius = 0.009, pipeExponent = 2.3, radiusScale = 1.25, ringSegments = 9, barkMottle = 0.45,
      leafSize = {0.38, 0.46}, leavesPerTip = 7, leafThickness = 0.035, leafClump = 0.9, maxLeafCards = 9000,
      paletteScale = 1.2,
    },
    bark = "papery",
    bark_color = { {0.52, 0.42, 0.33}, {0.60, 0.49, 0.38} },
    leaf_color = { {0.20, 0.33, 0.08}, {0.26, 0.38, 0.10} },
    leaf_palette = { {0.18, 0.31, 0.07}, {0.24, 0.36, 0.09}, {0.40, 0.42, 0.08}, {0.24, 0.36, 0.09} },
    leaves = { size = 0.07, shoots = 4, shoot_length = 0.32, shoot_angle = 40, lift = 0.0, sag = 0.25, per_shoot = 9,
               angle = 50, fold = 12, curl = 0.25, droop = 0.5, face = 0.4, budget = 34000 },
    card = {
      axiom = "S(5)", iterations = 6, variants = 2,
      rules = {
        { "S(n):n>0",  "F(0.10)+($bend)[+($a)L($s)]F(0.10)+($bend)[-($a)L($s)]S(n-1)" },
        { "S(n):n<=0", "F(0.05)L(0.9)" },
      },
      vars = { a = {42, 56}, s = {0.85, 1.0}, bend = {-3, 2} },
      leaf = { shape = "serrate", length = 0.16, width = 0.12, teeth = 9 },
      stem_width = 0.008, cup = 0.45,
    },
    branch_cards = { sprig = 0.8, wood_edge = 0.065 },
  },

  -- MAGNOLIA (the park's lakeside tree): a short trunk branching low into a dense oval crown of big glossy dark
  -- leaves; smooth grey bark.
  magnolia = {
    axiom = "T(0.8,2)",
    iterations = 12,
    rules = {
      { "T(l,n):n>0",   "F(l)/(97)T(l*0.95,n-1)" },
      { "T(l,n):n<=0",  "[&($s0)B(l*1.3)]/(120)[&($s0)B(l*1.25)]/(120)[&($s0*0.5)B(l*1.35)]" },
      { "B(l):l>0.2",   "F(l)/($roll)[&($pitch)B(l*$side)]^(8)B(l*$lead)", 3 },
      { "B(l):l>0.2",   "F(l)/($roll)[&($pitch)B(l*$side)]/(180)[&($pitch)B(l*$side)]", 1 },
      { "B(l):l<=0.2",  "F(l)/(137.5)[&(45)A(l)]/(137.5)[&(40)A(l)]A(l)" },
    },
    vars = { s0 = {22, 34}, roll = {80, 110}, pitch = {30, 42}, side = {0.76, 0.82}, lead = {0.82, 0.88} },
    params = {
      trunkZone = 0.07, trunkZoneTris = 2400,   -- (the trunk, its first limbs, its roots: one surface)
      joints = 1,
      droop = {0.08, 0.14}, light = {0.25, 0.35}, crookChance = {0.08, 0.15}, crookAngle = {10, 18},
      meander = {0.06, 0.10}, lean = {0, 5}, angleJitter = {8, 12},
      flare = 0.4, nodeSwell = 0.06, barkNoise = 0.03, rootCount = 4, rootSpread = 2.2, collar = 0.5,
      tipRadius = 0.010, pipeExponent = 2.3, radiusScale = 1.3, ringSegments = 10,
      leafSize = {0.46, 0.54}, leavesPerTip = 6, leafThickness = 0.04, leafClump = 1.0, maxLeafCards = 9000,
    },
    bark = "smooth",
    bark_color = { {0.40, 0.39, 0.36}, {0.46, 0.45, 0.42} },
    leaf_color = { {0.08, 0.19, 0.05}, {0.11, 0.24, 0.07} },
    leaves = { size = 0.16, shoots = 3, shoot_length = 0.25, shoot_angle = 45, lift = 0.4, sag = 0.06, per_shoot = 6,
               angle = 55, fold = 20, curl = 0.15, droop = 0.25, face = 0.65, budget = 18000 },
    card = {
      axiom = "S(3)", iterations = 4, variants = 2,
      rules = {
        { "S(n):n>0",  "F(0.08)+($bend)[+($a)L($s)][-($a)L($s*0.95)]S(n-1)" },
        { "S(n):n<=0", "F(0.04)[+(35)L(0.95)][-(35)L(0.95)][+(10)L(1.0)]" },
      },
      vars = { a = {40, 60}, s = {0.9, 1.05}, bend = {-6, 6} },
      leaf = { shape = "oval", length = 0.24, width = 0.09 },
      stem_width = 0.010, cup = 0.55,
    },
    branch_cards = { sprig = 1.3 },
  },

  -- WEEPING WILLOW (by the water): a stout trunk, limbs arching up and out, and from them long curtains of thin
  -- hanging shoots with narrow leaves, swaying to the ground.
  weeping_willow = {
    -- FORM in a stand (forest.h ForestSpecies::form; the island's forests): the lower limbs shaded off, a clear bole
    forms = { stand = { axiom_prefix = "F(1.0)", params = { pruneBelow = 0.20 } } },
    axiom = "T(1.0,2)",
    iterations = 12,
    rules = {
      { "T(l,n):n>0",   "F(l)/(97)T(l*0.95,n-1)" },
      { "T(l,n):n<=0",  "[&($s0)B(l*1.5)]/(120)[&($s0)B(l*1.45)]/(120)[&($s0)B(l*1.5)]" },
      { "B(l):l>0.25",  "F(l)/($roll)[&($pitch)B(l*$side)]^(4)B(l*$lead)" },
      { "B(l):l<=0.25", "F(l)[&(70)A(l)][^(20)A(l)]A(l)" },
    },
    vars = { s0 = {30, 40}, roll = {80, 110}, pitch = {32, 44}, side = {0.76, 0.82}, lead = {0.84, 0.88} },
    params = {
      trunkZone = 0.08, trunkZoneTris = 2400,   -- (the trunk, its first limbs, its roots: one surface)
      joints = 1,
      droop = {0.30, 0.45}, light = {0.05, 0.10}, crookChance = {0.10, 0.20}, crookAngle = {10, 20},
      meander = {0.06, 0.10}, lean = {0, 8}, angleJitter = {8, 12},
      flare = 0.6, lobes = {4, 7}, lobeDepth = 0.15, lobeHeight = 1.5, nodeSwell = 0.08, barkNoise = 0.06,
      rootCount = 5, rootSpread = 2.6, collar = 0.5,
      tipRadius = 0.009, pipeExponent = 2.3, radiusScale = 1.5, ringSegments = 10,
      leafSize = {0.50, 0.60}, leavesPerTip = 6, leafThickness = 0.03, leafClump = 1.0, leafPitch = 150, maxLeafCards = 9000,
    },
    bark = "furrowed",
    bark_color = { {0.30, 0.26, 0.21}, {0.36, 0.31, 0.25} },
    leaf_color = { {0.30, 0.42, 0.12}, {0.36, 0.48, 0.14} },
    -- the curtains: long shoots hanging straight down off every leafy twig, narrow leaves all along them
    leaves = { size = 0.10, shoots = 5, shoot_length = 2.0, shoot_angle = 90, lift = -6.0, sag = 0.0, per_shoot = 34,
               angle = 30, fold = 10, curl = 0.1, droop = 0.5, face = 0.3, budget = 60000 },
    card = {
      axiom = "S(6)", iterations = 7, variants = 2,
      rules = {
        { "S(n):n>0",  "F(0.07)[+($a)L($s)][-($a)L($s*0.95)]S(n-1)" },
        { "S(n):n<=0", "F(0.03)L(0.9)" },
      },
      vars = { a = {18, 28}, s = {0.9, 1.05} },
      leaf = { shape = "leaflet", length = 0.14, width = 0.02 },
      stem_width = 0.006, cup = 0.3,
    },
    branch_cards = { sprig = 2.2 },
  },
}

-- DERIVED SPECIES (Glenn, 2026-10-18: the island's beech, fir, aspen and alder, grown till now as an oak, a spruce, a
-- river birch and a multistem): each its nearest kin -- its grammar, ages and forms -- with what makes it itself laid
-- over. `derive`: a copy of the kin; a table in `over` merges into the kin's key by key, a list (a range, the rules)
-- replaces it.
local function copy(t)
  if type(t) ~= "table" then return t end
  local o = {}
  for k, v in pairs(t) do o[k] = copy(v) end
  return o
end
local function derive(kin, over)
  local o = copy(kin)
  for k, v in pairs(over) do
    if type(v) == "table" and type(o[k]) == "table" and v[1] == nil and next(v) ~= nil then o[k] = derive(o[k], v)
    else o[k] = copy(v) end
  end
  return o
end
local R = tree_recipes

-- BEECH: the oak's frame grown straight -- a smooth grey column, hardly a crook, limbs climbing into a dense dome --
-- its leaves oval, wavy-edged, alternate along the shoot; in a stand the longest clear bole of them all
R.beech = derive(R.oak, {
  vars = { pitch = {34, 48}, pitch2 = {26, 40}, crown = {0.80, 1.02} },   -- (straight: its seeds differ in size instead)
  params = { crookChance = {0.02, 0.06}, crookAngle = {6, 12}, meander = {0.03, 0.06}, lobeDepth = {0.02, 0.05},
             flare = {0.30, 0.45}, barkNoise = 0.02, twist = 0, nodeSwell = 0.04 },
  ages = { mature = { params = { crookChance = {0.04, 0.08} } }, old = { params = { crookChance = {0.05, 0.10} } },
           ancient = { params = { crookChance = {0.06, 0.12} } } },
  bark = "smooth",
  bark_color = { {0.40, 0.40, 0.38}, {0.50, 0.50, 0.47} },
  leaf_color = { {0.15, 0.32, 0.08}, {0.23, 0.42, 0.12} },
  leaves = { size = 0.08 },
  card = {
    axiom = "S(4)", iterations = 5,
    rules = {
      { "S(n):n>0",  "F(0.08)+($bend)[+($a)L($s)]F(0.08)-($bend)[-($a)L($s*0.95)]S(n-1)" },
      { "S(n):n<=0", "F(0.04)L(0.9)" },
    },
    vars = { a = {48, 62}, s = {0.9, 1.05}, bend = {-6, 6} },
    leaf = { shape = "oval", length = 0.24, width = 0.14 },
  },
  forms = { stand = { axiom_prefix = "F(4.5)", params = { pruneBelow = 0.5 } } },
  branch_cards = { sprig = 1.15 },   -- (its straighter limbs model more wood than the oak's: the hero held to its 8k)
})

-- SILVER FIR: the spruce's whorls held level, not swept down, the crown narrower and flat-topped old; its needles
-- FLAT in two ranks, dark above with a pale line beneath; the bark smooth grey
R.fir = derive(R.spruce, {
  vars = { pitch = {70, 82} },
  params = { droop = {0.02, 0.06} },
  bark = "smooth",
  bark_color = { {0.34, 0.34, 0.33}, {0.42, 0.42, 0.40} },
  needles = { kind = "flat", length = 0.028, width = 0.0026, forward = 70, side_angle = 70,
              color = { {0.05, 0.14, 0.08}, {0.08, 0.18, 0.10} }, tip_color = {0.15, 0.30, 0.16} },
})

-- ASPEN: the birch's slim leader, its branches climbing (a narrow crown), the bark smooth grey-green; round leaves on
-- long flat stalks -- they tremble
R.aspen = derive(R.birch, {
  vars = { pitch = {26, 38} },
  params = { barkMottle = 0.35, barkFootDark = 0.85, droop = {0.10, 0.18} },
  bark = "smooth",
  bark_color = { {0.60, 0.62, 0.55}, {0.70, 0.72, 0.64} },
  leaf_color = { {0.22, 0.42, 0.12}, {0.32, 0.52, 0.16} },
  leaves = { size = 0.06, droop = 0.3 },
  card = { leaf = { shape = "oval", length = 0.12, width = 0.11 } },
})

-- ALDER: by the water, a dark cone round a straight leader, its limbs held out level; the bark dark and fissured;
-- round leaves, notched at the tip, dark and glossy
R.alder = derive(R.birch, {
  vars = { pitch = {48, 62}, side = {0.60, 0.70} },
  params = { barkMottle = 0.0, barkFootDark = 1.0, droop = {0.06, 0.12}, flare = 0.4, barkNoise = 0.05 },
  bark = "furrowed",
  bark_color = { {0.20, 0.18, 0.16}, {0.28, 0.25, 0.22} },
  leaf_color = { {0.09, 0.22, 0.06}, {0.14, 0.29, 0.08} },
  leaves = { size = 0.075, droop = 0.2 },
  card = { leaf = { shape = "oval", length = 0.13, width = 0.12 } },
  forms = { stand = { axiom_prefix = "F(2.0)", params = { pruneBelow = 0.4 } } },
})

