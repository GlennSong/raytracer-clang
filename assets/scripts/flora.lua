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
function flora.grass(seed, opts)
    opts = opts or {}
    math.randomseed(seed)
    local n = opts.blades or 6
    local h = opts.height or 0.4
    local lo = opts.color_low or { 0.10, 0.28, 0.06 }
    local hi = opts.color_high or { 0.32, 0.55, 0.16 }
    local blades = {}
    for _ = 1, n do
        local b = mesh.translate(mesh.box({ 0.02, h, 0.006 }), { 0, h * 0.5, 0 })
        b = mesh.rotate_z(b, (math.random() - 0.5) * 0.7)      -- bend
        b = mesh.rotate_y(b, math.random() * 2 * math.pi)      -- face anywhere
        b = mesh.translate(b, { (math.random() - 0.5) * 0.14, 0, (math.random() - 0.5) * 0.14 })
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
