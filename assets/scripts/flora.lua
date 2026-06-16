-- Procedural flora library (ADR-0021/0023) — generators authored entirely in
-- Lua over the procgen builders. Defines a global `flora` table:
--
--   flora.tree(seed, opts)    -> Mesh   tapered welded trunk + real leaf cards
--   flora.rock(seed, opts)    -> Mesh   sphere + lumps - cuts (port of rock.cpp)
--   flora.grass(seed, opts)   -> Mesh   a tuft of blades
--   flora.flower(seed, opts)  -> Mesh   stem + petals + center
--
-- Hosts (the level loader, tests) load this once so `flora` is available, then
-- run small chunks like `return flora.tree(seed, {species="pine"})`. Pure: no io,
-- deterministic per seed. NOTE: geometry is validated headlessly (counts/taper/
-- determinism); the look needs a macOS render to judge.

flora = {}

local function rand_unit()       -- uniform point on the unit sphere
    local z = math.random() * 2 - 1
    local r = math.sqrt(math.max(0.0, 1 - z * z))
    local t = math.random() * 2 * math.pi
    return { r * math.cos(t), r * math.sin(t), z }
end

local function add(v, w) return { v[1] + w[1], v[2] + w[2], v[3] + w[3] } end
local function scale(v, s) return { v[1] * s, v[2] * s, v[3] * s } end

-- A few tree "species" presets. `rules` are STOCHASTIC (several weighted
-- productions for X), so the seed grows a genuinely different tree each time —
-- that is what gives the forest variety across instances/variants.
local SPECIES = {
    oak  = { axiom = "X", iterations = 3, angle_deg = 32, length = 0.55,
             radius = 0.18, taper = 0.90, leaf_size = 0.20,
             trunk = { 0.34, 0.24, 0.13 }, leaf = { 0.20, 0.44, 0.15 },
             rules = { "F[&+X][&-X]FX", "F[&+X][/&X][\\^X]FX", "FF[&-X][&/+X]X" } },
    pine = { axiom = "X", iterations = 4, angle_deg = 22, length = 0.5,
             radius = 0.16, taper = 0.86, leaf_size = 0.16,
             trunk = { 0.30, 0.20, 0.12 }, leaf = { 0.12, 0.36, 0.18 },
             rules = { "F[&X][^X]FX", "FF[/&X][\\&X]X", "F[&X][/X][\\X]FX" } },
    birch = { axiom = "X", iterations = 4, angle_deg = 28, length = 0.6,
              radius = 0.12, taper = 0.92, leaf_size = 0.18,
              trunk = { 0.80, 0.80, 0.74 }, leaf = { 0.40, 0.55, 0.20 },
              rules = { "F[+X][-X]FX", "FF[+X]X", "F[-X][/X]FX" } },
}

-- Parametric-grammar species (ADR-0030/0032): each defines a *parametric*
-- L-system (successor params are expressions over the predecessor's) plus skin
-- params, grown by flora.param_tree into the real curved generalized-cylinder
-- tree (bark + alpha-cut leaves). Structurally distinct, not just retuned:
-- oak is broad and drooping, pine a narrow whorled conifer, birch slender/weepy.
local PARAM_SPECIES = {
    oak = { iterations = 6, length = 2.2, width = 0.14, angle = 40,
            falloff = 0.76, leader_falloff = 0.88, branches = 2,
            terminal_fraction = 0.34, terminal_forks = 3,
            droop = 0.26, wander = 0.08, angle_jitter = 20, ring_segments = 6,
            tip_radius = 0.02, pipe_exponent = 2.4, radius_scale = 1.6,
            leaves_per_tip = 5, leaf_size = 0.18, leaf_thickness = 0.05,
            bark_color = { 0.32, 0.23, 0.15 }, leaf_color = { 0.20, 0.46, 0.13 } },
    pine = { iterations = 7, length = 1.5, width = 0.13, angle = 68,
             falloff = 0.62, leader_falloff = 0.92, branches = 3,
             terminal_fraction = 0.3, terminal_forks = 3,
             droop = 0.12, wander = 0.05, angle_jitter = 14, ring_segments = 6,
             tip_radius = 0.018, pipe_exponent = 2.3, radius_scale = 1.4,
             leaves_per_tip = 5, leaf_size = 0.12, leaf_thickness = 0.04,
             bark_color = { 0.28, 0.19, 0.12 }, leaf_color = { 0.12, 0.34, 0.16 } },
    birch = { iterations = 6, length = 2.0, width = 0.1, angle = 32,
              falloff = 0.78, leader_falloff = 0.9, branches = 2,
              terminal_fraction = 0.34, terminal_forks = 3,
              droop = 0.36, wander = 0.11, angle_jitter = 22, ring_segments = 6,
              tip_radius = 0.016, pipe_exponent = 2.3, radius_scale = 1.1,
              leaves_per_tip = 5, leaf_size = 0.15, leaf_thickness = 0.045,
              bark_color = { 0.78, 0.76, 0.70 }, leaf_color = { 0.42, 0.55, 0.18 } },
}

-- Grow a tree from a parametric grammar built out of the species params, and
-- return it as a MODEL: a list of {mesh, material} parts (bark opaque + leaf
-- alpha-cut), which the vegetation loader scatters as separate instance groups.
function flora.param_tree(seed, opts)
    opts = opts or {}
    local s = PARAM_SPECIES[opts.species or "oak"] or PARAM_SPECIES.oak
    local function pick(k) if opts[k] ~= nil then return opts[k] else return s[k] end end

    local angle, fall, lead = pick("angle"), pick("falloff"), pick("leader_falloff")
    local branches, roll = pick("branches"), 137.5
    local sys = lsystem.parametric()

    -- Structural growth while the internode is long: lay an internode, spawn
    -- `branches` golden-angle side branches (pitched away + shrunk), continue a
    -- central leader.
    local struct = "F(l,w)"
    for _ = 1, branches do
        struct = struct .. string.format("/(%g)[&(%g)A(l*%g,w*0.6)]", roll, angle, fall)
    end
    struct = struct .. string.format("/(%g)A(l*%g,w*0.75)", roll, lead)

    -- Terminal spray once short (guarded): more forks, wider pitch, faster shrink
    -- => branch ends fork repeatedly into dense short tipped twigs.
    local thresh = pick("length") * (pick("terminal_fraction") or 0.34)
    local tforks = pick("terminal_forks") or 3
    local term = "F(l,w)"
    for _ = 1, tforks do
        term = term .. string.format("/(%g)[&(%g)A(l*0.7,w*0.55)]", roll, angle * 1.3)
    end
    term = term .. string.format("/(%g)A(l*0.7,w*0.6)", roll)

    sys:rule(string.format("A(l,w):l>%g", thresh), struct)
    sys:rule(string.format("A(l,w):l<=%g", thresh), term)
    local modules = sys:expand(
        string.format("A(%g,%g)", pick("length"), pick("width")),
        pick("iterations"), seed)

    local bark, leaves = tree.skin(modules, {
        tip_radius = pick("tip_radius"), pipe_exponent = pick("pipe_exponent"),
        radius_scale = pick("radius_scale"), ring_segments = pick("ring_segments"),
        droop = pick("droop"), wander = pick("wander"),
        angle_jitter = pick("angle_jitter"),
        leaves_per_tip = pick("leaves_per_tip"), leaf_size = pick("leaf_size"),
        leaf_thickness = pick("leaf_thickness"),
        bark_color = pick("bark_color"), leaf_color = pick("leaf_color"),
    }, seed)

    local parts = { { mesh = bark, texture = "bark_" .. (opts.species or "oak"),
                      roughness = 1.0 } }
    if leaves then
        parts[#parts + 1] = { mesh = leaves, texture = "leaf", alpha_test = true,
                              wind = true, roughness = 0.6 }
    end
    return parts
end

-- A majestic THREE-PHASE tree, defined entirely in the grammar (no engine
-- changes): one symbol A with three guarded productions selected by internode
-- length l (ADR-0030 guards + && ranges). Demonstrates that N-phase structure is
-- pure Lua.
--   Phase 1  l > clear      : bare clear trunk (leader only, no branches)
--   Phase 2  term < l <= clear : scaffold crown (limbs + leader)
--   Phase 3  l <= term      : terminal twig sprays (thin => leaf-clad), so the
--                             crown caps with foliage and nothing pokes out bare.
-- Thresholds are fractions of trunk length; trunk vs crown use different falloffs
-- (a tall dominant trunk, a faster-capping crown).
function flora.phased_tree(seed, opts)
    opts = opts or {}
    local len   = opts.length or 2.6
    local clr   = len * (opts.clear_fraction or 0.6)
    local trm   = len * (opts.terminal_fraction or 0.2)
    local trunkFall = opts.trunk_falloff or 0.84
    local crownFall = opts.crown_falloff or 0.82
    local sideFall  = opts.side_falloff or 0.62
    local angle = opts.angle or 42
    local branches = opts.branches or 2
    local roll = 137.5

    local sys = lsystem.parametric()
    -- Phase 1: bare clear trunk.
    sys:rule(string.format("A(l):l>%g", clr),
             string.format("F(l)/(%g)A(l*%g)", roll, trunkFall))
    -- Phase 2: scaffold crown (single-symbol range guard via &&).
    local crown = "F(l)"
    for _ = 1, branches do
        crown = crown .. string.format("/(%g)[&(%g)A(l*%g)]", roll, angle, sideFall)
    end
    crown = crown .. string.format("/(%g)A(l*%g)", roll, crownFall)
    sys:rule(string.format("A(l):l<=%g && l>%g", clr, trm), crown)
    -- Phase 3: terminal sprays — fork into short thin twigs (leaf-clad by
    -- leaf_thickness), capping every end including the leader's tip.
    local ta = angle * 1.4
    sys:rule(string.format("A(l):l<=%g", trm),
             string.format("F(l)/(%g)[&(%g)A(l*0.6)]/(%g)[&(-%g)A(l*0.6)]/(%g)A(l*0.6)",
                           roll, ta, roll, ta, roll))

    local modules = sys:expand(string.format("A(%g)", len),
                               opts.iterations or 12, seed)
    local bark, leaves = tree.skin(modules, {
        tip_radius = opts.tip_radius or 0.02,
        pipe_exponent = opts.pipe_exponent or 2.4,
        radius_scale = opts.radius_scale or 1.7,
        ring_segments = opts.ring_segments or 6,
        droop = opts.droop or 0.12, wander = opts.wander or 0.06,
        leaves_per_tip = opts.leaves_per_tip or 4,
        leaf_size = opts.leaf_size or 0.2,
        leaf_thickness = opts.leaf_thickness or 0.06,
        leaf_clump = opts.leaf_clump or 1.0,
        bark_color = opts.bark_color or { 0.30, 0.22, 0.14 },
        leaf_color = opts.leaf_color or { 0.20, 0.46, 0.13 },
    }, seed)

    local parts = { { mesh = bark, texture = "bark", roughness = 1.0 } }
    if leaves then
        parts[#parts + 1] = { mesh = leaves, texture = "leaf", alpha_test = true,
                              wind = true, roughness = 0.6 }
    end
    return parts
end

-- A tapered, welded trunk/branches with real (oriented) leaf cards instead of
-- blobs. `opts.species` picks a preset; individual fields override it.
function flora.tree(seed, opts)
    opts = opts or {}
    local s = SPECIES[opts.species or "oak"] or SPECIES.oak
    local function pick(k) if opts[k] ~= nil then return opts[k] else return s[k] end end

    local sys = lsystem.create()
    for _, prod in ipairs(opts.rules or s.rules) do sys:rule("X", prod, 1.0) end
    sys:rule("F", "FF")
    local symbols = sys:expand(pick("axiom"), pick("iterations"), seed)
    -- Apices (leftover X) become leaf attachment points; L draws no blob since
    -- leaf_radius = 0, but turtle leaves still report the points.
    local leafy = (symbols:gsub("X", "L"))

    local params = {
        length       = pick("length"),
        radius       = pick("radius"),
        radius_taper = opts.radius_taper or 0.7,
        taper        = pick("taper"),          -- continuous thinning up the trunk
        angle_deg    = pick("angle_deg"),
        leaf_radius  = 0,                       -- branches only (no blobs)
    }

    local branches = lsystem.turtle_mesh_sdf(leafy, params,
                                             opts.smoothness or 0.07, opts.res or 56)
    local trunk_lo = pick("trunk")
    branches = mesh.bake_height_color(branches, trunk_lo, scale(trunk_lo, 1.25))

    -- Real leaves: a small card at each apex, fanned along the branch heading.
    local placements = lsystem.leaves(leafy, params)
    local lsize = pick("leaf_size")
    local leaf_color = pick("leaf")
    local card = mesh.bake_height_color(
        mesh.box({ lsize * 0.5, lsize * 1.6, lsize * 0.04 }), leaf_color, leaf_color)

    local cards = {}
    local step = opts.leaf_step or 1
    for i, lf in ipairs(placements) do
        if (i % step) == 0 then
            local c = mesh.orient(card, lf.direction)   -- card +Y -> heading
            cards[#cards + 1] = mesh.translate(c, lf.position)
        end
    end
    if #cards == 0 then return branches end
    return mesh.merge({ branches, mesh.merge(cards) })
end

-- A rock as an SDF graph: base sphere + smooth-unioned lumps - subtracted cuts,
-- mottled grey. The Lua port of generateRockSdf (rock.cpp).
function flora.rock(seed, opts)
    opts = opts or {}
    math.randomseed(seed)
    local base = opts.radius or 1.0
    local lump_scale = opts.lump_scale or 0.5
    local smooth = opts.smoothness or 0.3
    local res = opts.res or 40

    local parts = { sdf.sphere({ 0, 0, 0 }, base) }
    for _ = 1, (opts.lumps or 6) do
        local d = rand_unit()
        local dist = base * (0.7 + 0.3 * math.random())
        local r = base * lump_scale * (0.4 + 0.6 * math.random())
        parts[#parts + 1] = sdf.sphere(scale(d, dist), r)
    end
    local body = sdf.smooth_union_all(parts, smooth)
    for _ = 1, (opts.cuts or 2) do
        local d = rand_unit()
        local r = base * (0.5 + 0.4 * math.random())
        body = sdf.subtract(body, sdf.sphere(scale(d, base + r * 0.6), r))
    end

    local extent = base * (1.0 + lump_scale) + smooth
    local pad = extent * 0.15 + 2.0 * extent / res
    local m = extent + pad
    local rock = polygonize(body, { min = { -m, -m, -m }, max = { m, m, m } }, res)
    local g = opts.color or { 0.5, 0.5, 0.5 }
    return mesh.bake_height_color(rock, scale(g, 0.8), scale(g, 1.12))
end

-- A clump of grass blades (thin cards), bent and rotated, dark at the base to
-- light at the tip.
-- A dense grass *patch* (not a lone tuft): many blades spread over a disc, so a
-- handful of overlapping instances read as a continuous field. `radius` is the
-- patch size; `blades` the count within it.
function flora.grass(seed, opts)
    opts = opts or {}
    math.randomseed(seed)
    local n = opts.blades or 45
    local h = opts.height or 0.45
    local radius = opts.radius or 1.5
    local lo = opts.color_low or { 0.10, 0.28, 0.06 }
    local hi = opts.color_high or { 0.32, 0.55, 0.16 }
    local blades = {}
    for _ = 1, n do
        local bh = h * (0.6 + math.random() * 0.8)             -- varied height
        local b = mesh.translate(mesh.box({ 0.022, bh, 0.006 }), { 0, bh * 0.5, 0 })
        b = mesh.rotate_z(b, (math.random() - 0.5) * 0.8)      -- bend
        b = mesh.rotate_y(b, math.random() * 2 * math.pi)      -- face anywhere
        -- Uniform-area placement over the disc (sqrt keeps it from clumping center).
        local rr = radius * math.sqrt(math.random())
        local a = math.random() * 2 * math.pi
        b = mesh.translate(b, { rr * math.cos(a), 0, rr * math.sin(a) })
        blades[#blades + 1] = mesh.bake_height_color(b, lo, hi)
    end
    return mesh.merge(blades)
end

-- A small flower: a stem, a ring of petals, and a center.
function flora.flower(seed, opts)
    opts = opts or {}
    math.randomseed(seed)
    local h = opts.height or 0.3
    local petals = opts.petals or 6
    local pcol = opts.color or { 0.85, 0.22, 0.42 }

    local parts = {}
    local stem = mesh.translate(mesh.cylinder(0.012, h), { 0, h * 0.5, 0 })
    parts[#parts + 1] = mesh.bake_height_color(stem, { 0.15, 0.35, 0.10 }, { 0.20, 0.45, 0.14 })
    for i = 1, petals do
        local petal = mesh.translate(mesh.box({ 0.05, 0.012, 0.12 }), { 0, 0, 0.08 })
        petal = mesh.rotate_y(petal, (i / petals) * 2 * math.pi)
        petal = mesh.translate(petal, { 0, h, 0 })
        parts[#parts + 1] = mesh.bake_height_color(petal, pcol, scale(pcol, 1.1))
    end
    local center = mesh.translate(mesh.sphere(0.035), { 0, h, 0 })
    parts[#parts + 1] = mesh.bake_height_color(center, { 0.9, 0.8, 0.1 }, { 0.97, 0.86, 0.2 })
    return mesh.merge(parts)
end

return flora
