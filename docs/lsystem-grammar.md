# L-system tree grammar — author reference

Trees are defined as **data in the level JSON** (no code). In a level's
`vegetation.species` array, a `"kind": "tree"` entry holds the grammar and the
turtle parameters. Example (`assets/levels/forest.json`):

```json
{
  "kind": "tree",
  "skin": "sdf",            // "sdf" = welded surface; "cylinder" = fast kit-bash
  "smoothness": 0.30,       // SDF blend radius at branch joints (skin=sdf)
  "sdfResolution": 60,      // polygonization grid (higher = finer, slower)
  "axiom": "F",             // starting string
  "rules": { "F": "F[&+F]F[&-F]F" },   // production rules (char -> replacement)
  "iterations": 2,          // how many times the rules are applied
  "length": 1.4,            // world length of one F segment
  "radius": 0.22,           // trunk radius (branches taper from here)
  "radiusTaper": 0.72,      // radius *= taper on each branch push '['
  "angleDeg": 32,           // turn angle for +,-,&,^,/,\
  "material": { "albedo": [0.34, 0.25, 0.15], "roughness": 1.0 }
}
```

## How the grammar works

1. Start from `axiom`.
2. `iterations` times, replace every character that has a rule with its
   replacement string (characters with no rule are copied unchanged).
3. The final string is walked by a 3D "turtle" that draws geometry.

So `axiom "F"`, rule `F -> "FF"`, 2 iterations -> `"FFFF"`. Growth is
exponential in iterations, so keep iterations small (2-3) — and SDF cost grows
with the segment count, so big bushy trees get expensive.

## Turtle symbols

| Symbol | Action |
|--------|--------|
| `F` | draw a branch segment forward (length `length`, current radius), advance |
| `+` / `-` | yaw left / right (rotate about local Z by `angleDeg`) |
| `&` / `^` | pitch down / up (rotate about local X) |
| `/` / `\` | roll (rotate about local Y) |
| `[` | push state (start a branch); radius is multiplied by `radiusTaper` |
| `]` | pop state (return to where the branch started) |

Any other letter is a no-op placeholder you can use in rules to carry structure
(e.g. an `X` that only ever rewrites, never draws).

## Tips

- **Taller trees:** raise `length`, or use a rule with a clear vertical leader
  (several `F`s in a row before branching, e.g. `"FF[&+F][&-F]F"`).
- **Bushier:** more `[...]` branches per rule, or one more `iteration`.
- **3D, not flat:** mix pitch/roll (`&`, `^`, `/`, `\`) so branches leave the
  plane — using only `+`/`-` gives flat, fan-shaped trees.
- **Thin branches look weird (SDF):** the polygonizer can't capture a branch
  thinner than ~1.5 grid cells, so very thin tips are auto-clamped to a minimum
  radius. To get genuinely fine twigs, raise `sdfResolution` (costlier) or use
  `"skin": "cylinder"` (sharp but unwelded).
- **Determinism:** the same parameters always produce the same tree. (Stochastic
  rules — random variety per tree — are a planned extension.)

## Planned extensions (not yet)

- Stochastic rules (a symbol with several weighted replacements) so each tree
  differs.
- A leaf/canopy symbol that places foliage cards or blobs.
- Per-rule parameters (separate trunk vs. branch taper/length).
- Eventually, authoring this from Lua instead of a hand-written string (ADR-0023;
  the `flora` library already grows trees this way).
