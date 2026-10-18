#include "branch_cards.h"

#include "../mesh_builder.h"
#include "foliage_kit.h"   // addKitLeaf, addKitBlossom
#include "material_recipes.h"   // barkTextures
#include "leaf_card.h"     // foliageElementAtlas
#include "real_tree.h"     // renderCardView

#include <algorithm>
#include <cmath>
#include <functional>

namespace engine {

namespace {

// A sprig's own frame, right-handed with the face toward the viewer (renderCardView's side view looks along -z):
// x = right = axis x normal, y = out along the axis, z = the face (TreeMesh::Sprig::normal, skyward)
struct Frame {
    Vec3 base, right, axis, normal;
    Vec3 to(const Vec3& p) const { const Vec3 d = p - base; return Vec3(dot(d, right), dot(d, axis), dot(d, normal)); }
    Vec3 turn(const Vec3& v) const { return Vec3(dot(v, right), dot(v, axis), dot(v, normal)); }
};
// (view k of `views`: turned k / views of a half turn about the axis)
Frame frameOf(const TreeMesh::Sprig& sg, int k = 0, int views = 1) {
    const Vec3 right0 = normalize(cross(sg.axis, sg.normal));
    const double a = PI * k / std::max(1, views);
    const Vec3 right = normalize(right0 * std::cos(a) + sg.normal * std::sin(a));
    return {sg.base, right, sg.axis, normalize(cross(right, sg.axis))};
}

// A small tube for a picture, its bark mapped: u round it MIRRORED (0 .. 1 .. 0: no seam where it wraps), v along it
// (four tiles a metre), the tangent along it (as the bark's normal map wants)
RenderMesh barkTube(const std::vector<Vec3>& pts, const std::vector<double>& radii, int sides, const std::vector<Vec3>& cols) {
    RenderMesh m = MeshBuilder::tube(pts, radii, sides, cols);
    std::vector<double> along(pts.size(), 0.0);
    for (std::size_t i = 1; i < pts.size(); ++i) along[i] = along[i - 1] + (pts[i] - pts[i - 1]).length();
    for (std::size_t j = 0; j < m.vertices.size(); ++j) {
        const std::size_t ring = j / static_cast<std::size_t>(sides), k = j % static_cast<std::size_t>(sides);
        m.vertices[j].u = static_cast<float>(1.0 - std::fabs(2.0 * k / sides - 1.0));
        m.vertices[j].v = static_cast<float>(along[std::min(ring, along.size() - 1)] * 4.0);
    }
    return m;
}

RenderMesh inFrame(const RenderMesh& m, const Frame& f) {
    RenderMesh out = m;
    for (Vertex& v : out.vertices) {
        v.position = f.to(v.position);
        v.normal = f.turn(v.normal);
        v.tangent = f.turn(v.tangent);
    }
    return out;
}

}  // namespace

BranchCardSet bakeBranchCards(const TreeMesh& donor, const LeafCardAtlas& leaves, const BranchCardSpec& spec) {
    BranchCardSet set;
    // the pictures: across the sprigs' sizes (evenly through them sorted by length), each a sprig with leaves
    std::vector<int> leafy;
    for (std::size_t i = 0; i < donor.sprigs.size(); ++i)
        if (!donor.sprigs[i].leaves.indices.empty()) leafy.push_back(static_cast<int>(i));
    if (leafy.empty() || leaves.albedo.pixels.empty()) return set;
    std::sort(leafy.begin(), leafy.end(), [&](int a, int b) { return donor.sprigs[a].length < donor.sprigs[b].length; });
    const int n = std::min(std::max(1, spec.pictures), static_cast<int>(leafy.size()));
    std::vector<int> chosen;
    for (int k = 0; k < n; ++k) {
        // (the quantile's sprig, or the leafiest of its neighbours: a bare stub makes a poor picture)
        const int at = static_cast<int>((k + 0.5) / n * leafy.size());
        int best = leafy[static_cast<std::size_t>(at)];
        for (int d = -2; d <= 2; ++d) {
            const int j = std::clamp(at + d, 0, static_cast<int>(leafy.size()) - 1);
            const int c = leafy[static_cast<std::size_t>(j)];
            if (donor.sprigs[c].leaves.indices.size() > donor.sprigs[best].leaves.indices.size() &&
                std::find(chosen.begin(), chosen.end(), c) == chosen.end())
                best = c;
        }
        if (std::find(chosen.begin(), chosen.end(), best) == chosen.end()) chosen.push_back(best);
    }
    const int cell = std::max(32, spec.cell);
    const int views = std::max(1, spec.views);
    const std::size_t cells = chosen.size() * static_cast<std::size_t>(views);
    const int cols = static_cast<int>(std::ceil(std::sqrt(static_cast<double>(cells))));
    const int rows = static_cast<int>((cells + cols - 1) / cols);
    const int W = cols * cell, H = rows * cell;
    for (TextureData* t : {&set.atlas.albedo, &set.atlas.normal}) {
        t->width = W; t->height = H; t->channels = 4;
        t->pixels.assign(static_cast<std::size_t>(W) * H * 4, 0);
    }
    for (std::size_t i = 3; i < set.atlas.normal.pixels.size(); i += 4) {   // (uncovered: the card's own normal)
        set.atlas.normal.pixels[i - 3] = 128; set.atlas.normal.pixels[i - 2] = 128; set.atlas.normal.pixels[i - 1] = 255;
        set.atlas.normal.pixels[i] = 255;
    }
    // one view of a sprig into atlas cell `at`
    auto bakeView = [&](const TreeMesh::Sprig& sg, int vw, int at) {
        const Frame f = frameOf(sg, vw, views);
        const RenderMesh bark = inFrame(sg.bark, f), leaf = inFrame(sg.leaves, f);
        // the frame: everything the sprig built, a little margin round it, from its base (y = 0) out
        double x0 = 1e9, x1 = -1e9, y0 = 0.0, y1 = -1e9;
        for (const RenderMesh* m : {&bark, &leaf})
            for (const Vertex& v : m->vertices) {
                x0 = std::min(x0, v.position.x); x1 = std::max(x1, v.position.x);
                y0 = std::min(y0, v.position.y); y1 = std::max(y1, v.position.y);
            }
        const double padX = 0.03 * (x1 - x0), padY = 0.03 * (y1 - y0);
        x0 -= padX; x1 += padX; y0 -= padY; y1 += padY;
        const int c = at % cols, r = at / cols;
        const std::size_t off = (static_cast<std::size_t>(r) * cell * W + static_cast<std::size_t>(c) * cell) * 4;
        renderCardView(bark, leaf, leaves.albedo, false, x0, x1, y0, y1, cell, cell, set.colourScale,
                       set.atlas.albedo.pixels.data() + off, W * 4, set.atlas.normal.pixels.data() + off);
        BranchCardSet::View p;
        p.u0 = static_cast<float>(c) / cols; p.u1 = static_cast<float>(c + 1) / cols;
        p.v0 = static_cast<float>(r) / rows; p.v1 = static_cast<float>(r + 1) / rows;
        p.x0 = static_cast<float>(x0); p.x1 = static_cast<float>(x1);
        p.y0 = static_cast<float>(y0); p.y1 = static_cast<float>(y1);
        return p;
    };
    for (std::size_t k = 0; k < chosen.size(); ++k) {
        const TreeMesh::Sprig& sg = donor.sprigs[static_cast<std::size_t>(chosen[k])];
        BranchCardSet::Picture pic;
        pic.length = sg.length; pic.halfWidth = sg.halfWidth;
        for (int vw = 0; vw < views; ++vw) pic.views.push_back(bakeView(sg, vw, static_cast<int>(k) * views + vw));
        set.pictures.push_back(pic);
    }
    return set;
}

int wearBranchCards(TreeMesh& tree, const BranchCardSet& set, const BranchCardSpec& spec, uint32_t seed) {
    if (set.empty() || tree.sprigs.empty()) return 0;
    tree.foliage = 5;
    // the crown: the sprigs' middle, and how far they reach from it (the shade toward its inside, the normals' bend)
    Vec3 mid(0, 0, 0);
    for (const TreeMesh::Sprig& sg : tree.sprigs) mid = mid + sg.base + sg.axis * (0.5 * sg.length);
    mid = mid * (1.0 / tree.sprigs.size());
    double crownR = 0.0;
    for (const TreeMesh::Sprig& sg : tree.sprigs) crownR = std::max(crownR, (sg.base + sg.axis * (0.5 * sg.length) - mid).length());
    crownR = std::max(0.5, crownR);
    const double fold = spec.fold * PI / 180.0;
    const double inv = 1.0 / set.colourScale;
    RenderMesh& m = tree.leaves;
    int laid = 0;
    uint32_t h = seed * 2654435761u + 1u;
    auto rnd = [&]() { h ^= h << 13; h ^= h >> 17; h ^= h << 5; return (h & 0xffffff) / 16777216.0; };
    // one card: the view `p` laid in frame `f`, `s` metres a picture metre along the axis, `sw` across it
    auto layCard = [&](const Frame& f, const BranchCardSet::View& p, double s, double sw) {
        const double spine = std::clamp(0.0, static_cast<double>(p.x0) * 0.9, static_cast<double>(p.x1) * 0.9);
        // a little lean and roll of its own (no two cards on one plane)
        const double roll = (rnd() - 0.5) * 0.35;
        const Vec3 right = normalize(f.right * std::cos(roll) + f.normal * std::sin(roll));
        const Vec3 face = normalize(cross(right, f.axis));
        // the six corners: across at x0, the spine, x1; out at y0 (the base) and y1 (the far end)
        auto at = [&](double x, double y) {
            const double dx = (x - spine) * sw;
            return f.base + f.axis * (y * s) + right * (spine * sw + dx * std::cos(fold)) + face * (std::fabs(dx) * std::sin(fold));
        };
        const double xs[3] = {p.x0, spine, p.x1}, ys[2] = {p.y0, p.y1};
        // each half's own face (lifted by the fold), bent toward out of the crown
        const Vec3 halfFace[2] = {normalize(face * std::cos(fold) + right * std::sin(fold)),
                                  normalize(face * std::cos(fold) - right * std::sin(fold))};
        const uint32_t b = static_cast<uint32_t>(m.vertices.size());
        for (int yi = 0; yi < 2; ++yi)
            for (int xi = 0; xi < 3; ++xi) {
                const Vec3 pos = at(xs[xi], ys[yi]);
                Vec3 out = pos - mid;
                out = out.length() > 1e-6 ? normalize(out) : Vec3(0, 1, 0);
                const Vec3 hf = xi == 0 ? halfFace[0] : xi == 2 ? halfFace[1] : face;
                const Vec3 nrm = normalize(hf * (1.0 - spec.bend) + out * spec.bend);
                const Vec3 tg = normalize(right - nrm * dot(right, nrm));
                const float u = p.u0 + (p.u1 - p.u0) * static_cast<float>((xs[xi] - p.x0) / std::max(1e-6f, p.x1 - p.x0));
                const float v = p.v1 - (p.v1 - p.v0) * static_cast<float>((ys[yi] - p.y0) / std::max(1e-6f, p.y1 - p.y0));
                Vertex vx(pos, nrm, tg, u, v);
                // darker toward the crown's middle
                const double depth = std::clamp((pos - mid).length() / crownR, 0.0, 1.0);
                const double ao = 0.5 + 0.5 * depth;
                vx.color = Vec3(ao * inv, ao * inv, ao * inv);
                m.vertices.push_back(vx);
            }
        // the two halves (the spine their shared edge), each wound to its own face
        MeshBuilder::quadFacing(m, b + 0, b + 1, b + 4, b + 3, halfFace[0]);
        MeshBuilder::quadFacing(m, b + 1, b + 2, b + 5, b + 4, halfFace[1]);
    };
    for (const TreeMesh::Sprig& sg : tree.sprigs) {
        if (sg.length <= 1e-3f) continue;
        // the picture nearest in size and shape (of the two nearest, one at random: neighbours differ)
        int best = 0, second = -1;
        double bestD = 1e9, secondD = 1e9;
        for (std::size_t i = 0; i < set.pictures.size(); ++i) {
            const BranchCardSet::Picture& p = set.pictures[i];
            const double d = std::fabs(std::log(std::max(1e-3f, sg.length) / std::max(1e-3f, p.length))) +
                             0.5 * std::fabs(std::log(std::max(1e-3f, sg.halfWidth) / std::max(1e-3f, p.halfWidth)));
            if (d < bestD) { second = best; secondD = bestD; best = static_cast<int>(i); bestD = d; }
            else if (d < secondD) { second = static_cast<int>(i); secondD = d; }
        }
        if (second >= 0 && secondD < bestD + 0.3 && rnd() < 0.5) best = second;
        const BranchCardSet::Picture& pic = set.pictures[static_cast<std::size_t>(best)];
        // scaled to the sprig along its axis; across it as near as the picture allows (a leaf stays a leaf's shape)
        const double s = sg.length / std::max(1e-3f, pic.length);
        const int views = static_cast<int>(pic.views.size());
        for (int vw = 0; vw < views; ++vw)
            layCard(frameOf(sg, vw, views), pic.views[static_cast<std::size_t>(vw)], s,
                    vw == 0 ? s * std::clamp((sg.halfWidth / std::max(1e-3f, pic.halfWidth)) / s, 0.8, 1.25) : s);
        ++laid;
    }
    return laid;
}


BranchCardSet bakeTwigCards(const TreeRecipe& r, double twigLength, int variants, uint32_t seed) {
    BranchCardSet set;
    const LeafKitSpec& lk = r.leafKit;
    if (!lk.set) return set;
    const bool flowers = lk.blossomSize > 0.0;
    const LeafCardAtlas leafAtlas = foliageElementAtlas(r.card, flowers, seed ^ 0x7e11u);
    if (leafAtlas.albedo.pixels.empty()) return set;
    // (the bark's own maps on the twigs, the leaves' normal map on them, and the back of the picture shaded down:
    // Glenn -- "very flat colored ... some kind of normal map to give them dimension")
    BarkKind barkKind;
    BarkTextures barkMaps;
    if (barkKindFromName(r.bark, barkKind)) barkMaps = barkTextures(barkKind, 1u, 256);
    CardViewExtras extras;
    extras.barkAlbedo = &barkMaps.albedo;
    extras.barkNormal = &barkMaps.normal;
    extras.foliageNormal = &leafAtlas.normal;
    extras.depthShade = 0.35;
    extras.trueNormals = true;
    const double L = std::max(0.08, twigLength);
    set.twigLength = static_cast<float>(L);
    const int nv = std::max(1, variants);
    const int count = 4 * nv;
    const int cols = 4, rows = (count + cols - 1) / cols;
    const int cw = 192, ch = 384;
    const int W = cols * cw, H = rows * ch;
    for (TextureData* t : {&set.atlas.albedo, &set.atlas.normal}) {
        t->width = W; t->height = H; t->channels = 4;
        t->pixels.assign(static_cast<std::size_t>(W) * H * 4, 0);
    }
    for (std::size_t i = 3; i < set.atlas.normal.pixels.size(); i += 4) {
        set.atlas.normal.pixels[i - 3] = 128; set.atlas.normal.pixels[i - 2] = 128; set.atlas.normal.pixels[i - 1] = 255;
        set.atlas.normal.pixels[i] = 255;
    }
    uint32_t h = seed * 2654435761u + 7u;
    auto rnd = [&]() { h ^= h << 13; h ^= h >> 17; h ^= h << 5; return (h & 0xffffff) / 16777216.0; };
    // the bark as the trunk shades: the foliage material lights the card brighter than the bark material does the wood
    const Vec3 bark = (r.barkColor * 0.5 + r.barkColorHi * 0.5) * 0.55;
    // (the leaves' colour: the species' palette -- each variant one of them -- else its leaf colour range)
    int paletteNo = 0;
    auto leafColour = [&]() -> Vec3 {
        if (!r.leafPalettes.empty()) {
            const std::vector<Vec3>& pal = r.leafPalettes[static_cast<std::size_t>(paletteNo) % r.leafPalettes.size()];
            if (!pal.empty()) return pal[static_cast<std::size_t>(rnd() * pal.size()) % pal.size()];
        }
        return r.leafColor + (r.leafColorHi - r.leafColor) * rnd();
    };
    const double leafLen = lk.size;
    // a twig (a polyline from `base` along `dir`, bending a little) with leaves along it: alternate or paired,
    // spreading to the sides in the picture's plane, facing the viewer (+z) with a tilt of their own
    auto twig = [&](RenderMesh& wood, RenderMesh& leaves, const Vec3& base, const Vec3& dir0, double len, double r0,
                    bool leafy) -> std::vector<Vec3> {
        std::vector<Vec3> pts;
        std::vector<double> rad;
        std::vector<Vec3> cols;
        Vec3 p = base, d = normalize(dir0);
        const double bend = (rnd() - 0.5) * 0.25;
        for (int k = 0; k <= 4; ++k) {
            pts.push_back(p);
            rad.push_back(r0 * (1.0 - 0.6 * k / 4.0));
            cols.push_back(bark);
            const Vec3 side = normalize(cross(Vec3(0, 0, 1), d));
            d = normalize(d + side * bend);
            p = p + d * (len / 4.0);
        }
        MeshBuilder::append(wood, barkTube(pts, rad, 6, cols));
        if (!leafy) return pts;
        const int n = std::max(4, static_cast<int>(std::lround(len / (0.32 * leafLen))));
        for (int j = 0; j < n; ++j) {
            const double t = 0.12 + 0.88 * (j + 0.5) / n;
            const int k = std::min(3, static_cast<int>(t * 4.0));
            const double f = t * 4.0 - k;
            const Vec3 at = pts[k] + (pts[k + 1] - pts[k]) * f;
            const Vec3 ax = normalize(pts[k + 1] - pts[k]);
            const Vec3 side = normalize(cross(Vec3(0, 0, 1), ax));
            const double sgn = lk.pairs ? (j % 2 ? 1.0 : -1.0) : ((j % 2) ? 1.0 : -1.0);
            const double ang = (lk.angle + (rnd() - 0.5) * 24.0) * PI / 180.0;
            Vec3 ld = normalize(ax * std::cos(ang) + side * (sgn * std::sin(ang)) + Vec3(0, 0, (rnd() - 0.5) * 0.5));
            const Vec3 nrm = normalize(Vec3(0, 0, 1) + side * (-sgn * 0.25) + Vec3((rnd() - 0.5) * 0.3, (rnd() - 0.5) * 0.3, 0));
            const double sz = leafLen * (0.8 + 0.4 * rnd()) * (0.75 + 0.25 * t);
            const int tile = flowers ? static_cast<int>(rnd() * 2.0) : static_cast<int>(rnd() * 4.0);
            addKitLeaf(leaves, at, ld, nrm, sz, lk.fold, lk.curl * (0.6 + 0.8 * rnd()), 2, std::min(tile, flowers ? 1 : 3),
                       leafColour() * (0.85 + 0.3 * rnd()));
        }
        // the end leaf, on along the twig
        addKitLeaf(leaves, pts.back(), normalize(pts.back() - pts[3]), Vec3(0, 0, 1), leafLen * 0.9, lk.fold, lk.curl, 2,
                   flowers ? 0 : static_cast<int>(rnd() * 4.0) % 4, leafColour());
        if (flowers) {   // the blossom clusters along it (a sakura's umbels) -- a few: a card packed with them reads flat
            const int nc = std::max(1, lk.clusters);
            for (int c = 0; c < nc; ++c) {
                const double t = 0.25 + 0.75 * (c + 0.5) / nc;
                const int k = std::min(3, static_cast<int>(t * 4.0));
                const Vec3 at = pts[k] + (pts[k + 1] - pts[k]) * (t * 4.0 - k);
                const int per = std::max(2, lk.perCluster - 2);
                for (int b = 0; b < per; ++b) {
                    const double phi = 2.0 * PI * b / per + rnd();
                    const Vec3 stalk = normalize(Vec3(std::cos(phi) * 0.8, std::sin(phi) * 0.8, 1.0));
                    const double sz = lk.blossomSize * (0.85 + 0.3 * rnd());
                    addKitBlossom(leaves, at + stalk * (0.9 * sz), stalk, sz, lk.cup, rnd() * 2.0 * PI, 2, 2 + static_cast<int>(rnd() * 2.0) % 2,
                                  leafColour());
                }
            }
        }
        return pts;
    };
    int cell = 0;
    for (int kind = 0; kind < 4; ++kind)
        for (int v = 0; v < nv; ++v, ++cell) {
            const bool leafy = kind < 2, fork = kind == 1 || kind == 3;
            paletteNo = v;
            RenderMesh wood, leaves;
            const double r0 = std::max(0.004, 0.03 * L);
            const std::vector<Vec3> main = twig(wood, leaves, Vec3(0, 0, 0), Vec3((rnd() - 0.5) * 0.1, 1, 0), L, r0, leafy);
            if (fork) {   // a side twig FROM the main one, a little under half way, off to one side of its own direction
                const double sgn = rnd() < 0.5 ? -1.0 : 1.0, at = 0.3 + 0.15 * rnd();
                const double x = at * (main.size() - 1);
                const int k = std::min(static_cast<int>(main.size()) - 2, static_cast<int>(x));
                const Vec3 from = main[k] + (main[k + 1] - main[k]) * (x - k);
                const Vec3 ax = normalize(main[k + 1] - main[k]), side = normalize(cross(Vec3(0, 0, 1), ax));
                const double a = (30.0 + 15.0 * rnd()) * PI / 180.0;
                // (it starts inside the main twig, its radius under the main one's there)
                twig(wood, leaves, from - ax * (0.3 * r0), normalize(ax * std::cos(a) + side * (sgn * std::sin(a))),
                     L * (0.55 + 0.15 * rnd()), r0 * (1.0 - 0.6 * at) * 0.75, leafy);
            }
            double x0 = 1e9, x1 = -1e9, y1 = -1e9;
            for (const RenderMesh* m : {&wood, &leaves})
                for (const Vertex& vx : m->vertices) {
                    x0 = std::min(x0, vx.position.x); x1 = std::max(x1, vx.position.x); y1 = std::max(y1, vx.position.y);
                }
            const double half = std::max(std::fabs(x0), std::fabs(x1)) * 1.04 + 0.01;   // (the twig's foot on the centre line)
            const double y0 = -0.02 * L;
            y1 = y1 * 1.03 + 0.01;
            const int c = cell % cols, rr = cell / cols;
            const std::size_t off = (static_cast<std::size_t>(rr) * ch * W + static_cast<std::size_t>(c) * cw) * 4;
            renderCardView(wood, leaves, leafAtlas.albedo, false, -half, half, y0, y1, cw, ch, set.colourScale,
                           set.atlas.albedo.pixels.data() + off, W * 4, set.atlas.normal.pixels.data() + off, &extras);
            BranchCardSet::Picture pic;
            BranchCardSet::View vw;
            vw.u0 = static_cast<float>(c) / cols; vw.u1 = static_cast<float>(c + 1) / cols;
            vw.v0 = static_cast<float>(rr) / rows; vw.v1 = static_cast<float>(rr + 1) / rows;
            vw.x0 = static_cast<float>(-half); vw.x1 = static_cast<float>(half);
            vw.y0 = static_cast<float>(y0); vw.y1 = static_cast<float>(y1);
            pic.views.push_back(vw);
            pic.length = static_cast<float>(L);
            pic.halfWidth = static_cast<float>(half);
            pic.kind = kind;
            set.pictures.push_back(pic);
        }
    return set;
}

int wearTwigCards(TreeMesh& tree, const BranchCardSet& set, const BranchCardSpec& spec, double fillRadius, double fillSpacing,
                  uint32_t seed) {
    if (set.empty() || set.twigLength <= 0.0f) return 0;
    tree.foliage = 5;
    std::vector<int> byKind[4];
    for (std::size_t i = 0; i < set.pictures.size(); ++i)
        if (set.pictures[i].kind >= 0 && set.pictures[i].kind < 4) byKind[set.pictures[i].kind].push_back(static_cast<int>(i));
    if (byKind[0].empty() || byKind[2].empty()) return 0;
    // the crown's middle and reach (the normals' bend and the shade inside, as the sprig cards)
    Vec3 mid(0, 0, 0);
    double crownR = 0.5;
    int nmid = 0;
    for (const TreeMesh::Twig& t : tree.twigs) { mid = mid + t.b; ++nmid; }
    if (nmid) {
        mid = mid * (1.0 / nmid);
        for (const TreeMesh::Twig& t : tree.twigs) crownR = std::max(crownR, (t.b - mid).length());
    }
    const double fold = spec.fold * PI / 180.0, inv = 1.0 / set.colourScale;
    uint32_t h = seed * 2246822519u + 3u;
    auto rnd = [&]() { h ^= h << 13; h ^= h >> 17; h ^= h << 5; return (h & 0xffffff) / 16777216.0; };
    RenderMesh& m = tree.leaves;
    int laid = 0;
    // one segment: picture `pi` from `a` along `axis` for `len`, its face rolled `roll` from the sky about the axis
    auto lay = [&](int pi, const Vec3& a, const Vec3& axis, double len, double roll) {
        const BranchCardSet::Picture& pic = set.pictures[static_cast<std::size_t>(pi)];
        const BranchCardSet::View& p = pic.views.front();
        const double s = std::clamp(len / set.twigLength, 0.45, 1.8);   // along: to the internode
        const double sw = std::clamp(s, 0.75, 1.25);                     // across: a leaf stays near its size
        Vec3 up = Vec3(0, 1, 0) - axis * dot(Vec3(0, 1, 0), axis);
        if (up.length() < 0.2) up = cross(axis, Vec3(1, 0, 0));
        up = normalize(up);
        const Vec3 side0 = normalize(cross(axis, up));
        // (the picture's face toward the sky -- rolled about the twig -- in two planes, a shallow V)
        for (int pl = 0; pl < 2; ++pl) {
            const double rr = roll + (pl == 0 ? -1.0 : 1.0) * (0.5 * fold + 0.35);
            const Vec3 right = normalize(side0 * std::cos(rr) + up * std::sin(rr));
            const Vec3 face = normalize(cross(right, axis));
            const Vec3 fUp = dot(face, Vec3(0, 1, 0)) < 0.0 ? face * -1.0 : face;
            const uint32_t b = static_cast<uint32_t>(m.vertices.size());
            const double xs[2] = {p.x0, p.x1}, ys[2] = {p.y0, p.y1};
            for (int yi = 0; yi < 2; ++yi)
                for (int xi = 0; xi < 2; ++xi) {
                    const Vec3 pos = a + axis * (ys[yi] * s) + right * (xs[xi] * sw);
                    Vec3 out = pos - mid;
                    out = out.length() > 1e-6 ? normalize(out) : Vec3(0, 1, 0);
                    const Vec3 nrm = normalize(fUp * (1.0 - spec.bend) + out * spec.bend);
                    const Vec3 tg = normalize(right - nrm * dot(right, nrm));
                    const float u = xi ? p.u1 : p.u0, v = yi ? p.v0 : p.v1;
                    Vertex vx(pos, nrm, tg, u, v);
                    const double ao = 0.5 + 0.5 * std::clamp((pos - mid).length() / crownR, 0.0, 1.0);
                    vx.color = Vec3(ao * inv, ao * inv, ao * inv);
                    m.vertices.push_back(vx);
                }
            MeshBuilder::quadFacing(m, b + 0, b + 1, b + 3, b + 2, fUp);
        }
        ++laid;
    };
    auto pick = [&](int kind) { const auto& v = byKind[kind].empty() ? byKind[kind & 2] : byKind[kind];
                                return v[static_cast<std::size_t>(rnd() * v.size()) % v.size()]; };
    // along the fine branching
    for (const TreeMesh::Twig& t : tree.twigs) {
        const Vec3 d = t.b - t.a;
        const double len = d.length();
        if (len < 1e-3) continue;
        const bool y = rnd() < (t.forks >= 2 ? 0.15 : 0.35);
        const int kind = (t.leafy ? 0 : 2) + (y ? 1 : 0);
        lay(pick(kind), t.a, d * (1.0 / len), len, (rnd() - 0.5) * 0.8);
    }
    // FILLING OUT the thin modelled limbs: leafy segments sprouting off them, round them by the golden angle
    if (fillSpacing > 0.0) {
        double roll = rnd() * 2.0 * PI;
        for (const TreeMesh::Limb& l : tree.limbs) {
            if (l.rb > fillRadius || l.rb <= 0.0f) continue;
            const Vec3 d = l.b - l.a;
            const double len = d.length();
            if (len < 1e-3) continue;
            const Vec3 ax = d * (1.0 / len);
            const Vec3 ref = std::fabs(ax.y) < 0.9 ? Vec3(0, 1, 0) : Vec3(1, 0, 0);
            const Vec3 e1 = normalize(cross(ax, ref)), e2 = cross(ax, e1);
            for (double s = fillSpacing * (0.3 + 0.7 * rnd()); s < len; s += fillSpacing * (0.7 + 0.6 * rnd())) {
                roll += 137.5 * PI / 180.0;
                const Vec3 out = e1 * std::cos(roll) + e2 * std::sin(roll);
                const double ang = (40.0 + 25.0 * rnd()) * PI / 180.0;
                Vec3 dir = normalize(ax * std::cos(ang) + out * std::sin(ang) + Vec3(0, 0.25, 0));
                const double r = l.ra + (l.rb - l.ra) * (s / len);
                const Vec3 at = l.a + ax * s + out * (0.8 * r);
                lay(pick(rnd() < 0.4 ? 1 : 0), at, dir, set.twigLength * (0.7 + 0.5 * rnd()), (rnd() - 0.5) * 0.8);
            }
        }
    }
    return laid;
}


int wearNearLeaves(TreeMesh& tree, const TreeRecipe& r, int leaves, int twigTris, double fillRadius, double fillSpacing,
                   uint32_t seed) {
    const LeafKitSpec& lk = r.leafKit;
    if (!lk.set || tree.twigs.empty()) return 0;
    tree.foliage = 2;
    uint32_t h = seed * 2654435761u + 11u;
    auto rnd = [&]() { h ^= h << 13; h ^= h >> 17; h ^= h << 5; return (h & 0xffffff) / 16777216.0; };
    const bool flowers = lk.blossomSize > 0.0;
    // the crown's middle and reach
    Vec3 mid(0, 0, 0);
    for (const TreeMesh::Twig& t : tree.twigs) mid = mid + t.b;
    mid = mid * (1.0 / tree.twigs.size());
    double crownR = 0.5;
    for (const TreeMesh::Twig& t : tree.twigs) crownR = std::max(crownR, (t.b - mid).length());
    // EVERY LEAF ON WOOD THAT IS THERE (Glenn, 2026-10-18: "I see floating leaves" -- shoots and sprouts whose wood
    // the budget had run out on still wore their leaves): the twigs get their bark first, thickest first; the shoots
    // are as many as the budget left gives each leafy twig WITH wood (fewer shoots, never bare-of-wood ones); the
    // sprouts along the thin limbs only while it lasts; and the leaves ride only those -- each piece placed at an anchor
    // on its parent (SpeedTree's anchor points)
    // (the order: the branching first -- the twigs that fork on, thickest first -- then the leafy ends in a shuffled
    // order, so a dense crown past the budget thins EVENLY, not all its ends at once)
    std::vector<std::size_t> order, ends;
    for (std::size_t i = 0; i < tree.twigs.size(); ++i) (tree.twigs[i].leafy ? ends : order).push_back(i);
    std::sort(order.begin(), order.end(), [&](std::size_t a, std::size_t b) { return tree.twigs[a].r > tree.twigs[b].r; });
    for (std::size_t i = ends.size(); i > 1; --i) std::swap(ends[i - 1], ends[static_cast<std::size_t>(rnd() * i) % i]);
    order.insert(order.end(), ends.begin(), ends.end());
    RenderMesh& wood = tree.branches;
    const bool data = wood.tangentIsData;   // (the bark blends: its tangent slot carries UVs)
    const Vec3 barkCol = r.barkColor * 0.5 + r.barkColorHi * 0.5;
    int tris = 0;
    auto addWood = [&](const Vec3& a, const Vec3& b, double r0, double r1) {
        RenderMesh tube = barkTube({a, b}, {r0, r1}, 3, {barkCol, barkCol});
        for (Vertex& v : tube.vertices) {
            v.u *= 0.5f;
            if (data) v.tangent = Vec3(v.u, v.v, 0.0);
        }
        tris += static_cast<int>(tube.indices.size() / 3);
        MeshBuilder::append(wood, tube);
    };
    std::vector<char> hasWood(tree.twigs.size(), 0);
    for (std::size_t i : order) {
        const TreeMesh::Twig& t = tree.twigs[i];
        if (tris + 6 > twigTris) break;
        const double r0 = std::max(0.0025, static_cast<double>(t.r));
        addWood(t.a, t.b, r0, std::max(0.0015, 0.7 * r0));
        hasWood[i] = 1;
    }
    struct Spot { Vec3 a, b; };
    std::vector<Spot> spots;
    double total = 0.0;
    std::size_t leafyWithWood = 0;
    for (std::size_t i = 0; i < tree.twigs.size(); ++i) leafyWithWood += tree.twigs[i].leafy && hasWood[i];
    const int shootsEach = leafyWithWood == 0 ? 0
        : std::clamp(static_cast<int>((twigTris - tris) / 6 / static_cast<int>(leafyWithWood)), 0, lk.shoots);
    for (std::size_t i = 0; i < tree.twigs.size(); ++i) {
        const TreeMesh::Twig& t = tree.twigs[i];
        if (!t.leafy || !hasWood[i]) continue;
        spots.push_back({t.a, t.b});
        total += (t.b - t.a).length();
        const Vec3 d = t.b - t.a;
        const double len = d.length();
        if (len < 1e-4) continue;
        const Vec3 ax = d * (1.0 / len);
        const Vec3 ref = std::fabs(ax.y) < 0.9 ? Vec3(0, 1, 0) : Vec3(1, 0, 0);
        const Vec3 e1 = normalize(cross(ax, ref)), e2 = cross(ax, e1);
        for (int k = 0; k < shootsEach; ++k) {   // (each from a point ON its twig, with its own wood)
            const double roll = k * 2.399 + rnd() * 0.8;
            const Vec3 out = e1 * std::cos(roll) + e2 * std::sin(roll);
            const double ang = (lk.shootAngle + (rnd() - 0.5) * 30.0) * PI / 180.0;
            const Vec3 sd = normalize(ax * std::cos(ang) + out * std::sin(ang) + Vec3(0, lk.lift, 0));
            const Vec3 from = t.a + d * (0.3 + 0.7 * rnd());
            const double sl = lk.shootLength * (0.7 + 0.6 * rnd());
            Vec3 to = from + sd * sl;
            to.y -= lk.sag * sl;
            addWood(from, to, 0.0035, 0.0015);
            spots.push_back({from, to});
            total += sl;
        }
    }
    if (fillSpacing > 0.0)   // (sprouts along the thin modelled limbs: from ON the limb, with wood, while it lasts)
        for (const TreeMesh::Limb& l : tree.limbs) {
            if (l.rb > fillRadius || l.rb <= 0.0f) continue;
            const Vec3 d = l.b - l.a;
            const double len = d.length();
            if (len < 1e-3) continue;
            const Vec3 ax = d * (1.0 / len);
            const Vec3 ref = std::fabs(ax.y) < 0.9 ? Vec3(0, 1, 0) : Vec3(1, 0, 0);
            const Vec3 e1 = normalize(cross(ax, ref)), e2 = cross(ax, e1);
            for (double s = fillSpacing * rnd(); s < len && tris + 6 <= twigTris; s += fillSpacing) {
                const double roll = rnd() * 2.0 * PI;
                const Vec3 out = e1 * std::cos(roll) + e2 * std::sin(roll);
                const Vec3 dir = normalize(ax * 0.6 + out + Vec3(0, 0.3, 0));
                const Vec3 a = l.a + ax * s;
                const Vec3 b = a + dir * (lk.shootLength * (0.7 + 0.6 * rnd()));
                addWood(a, b, 0.0045, 0.0018);
                spots.push_back({a, b});
                total += (b - a).length();
            }
        }
    if (spots.empty() || total <= 0.0) return 0;
    // (the WHOLE tree's leaves as the leaf kit would hang them -- every leafy twig and its shoots -- and as many as the
    // budget gives laid over the wood there is, each larger by the area they stand for: a crown whose budget ran
    // out of wood keeps its leaf area -- the mid's cards have it all -- instead of thinning as one walks up to it)
    double fullLength = 0.0;
    for (const TreeMesh::Twig& t : tree.twigs)
        if (t.leafy) fullLength += (t.b - t.a).length() + lk.shoots * lk.shootLength;
    const double natural = std::max(total, fullLength) / std::max(0.02, 0.45 * lk.size);
    const double count = std::min(static_cast<double>(leaves), natural);
    const double grow = std::clamp(std::sqrt(natural / std::max(1.0, count)), 1.0, 2.5);
    const double spacing = std::max(0.15 * lk.size, total / std::max(1.0, count));
    // (colour: the palette, in patches over the crown, or the leaf colour range)
    auto leafColour = [&](const Vec3& at) -> Vec3 {
        if (!r.leafPalettes.empty()) {
            const std::vector<Vec3>& pal = r.leafPalettes[static_cast<std::size_t>(seed) % r.leafPalettes.size()];
            if (!pal.empty()) {
                const double n = 0.5 + 0.5 * std::sin(at.x * 1.3 + at.z * 0.9 + at.y * 0.7 + seed % 17);
                return pal[std::min(pal.size() - 1, static_cast<std::size_t>(n * pal.size()))];
            }
        }
        return r.leafColor + (r.leafColorHi - r.leafColor) * rnd();
    };
    RenderMesh& m = tree.leaves;
    int laid = 0;
    double carry = rnd() * spacing;
    for (const Spot& sp : spots) {
        const Vec3 d = sp.b - sp.a;
        const double len = d.length();
        if (len < 1e-4) continue;
        const Vec3 ax = d * (1.0 / len);
        const Vec3 ref = std::fabs(ax.y) < 0.9 ? Vec3(0, 1, 0) : Vec3(1, 0, 0);
        const Vec3 e1 = normalize(cross(ax, ref)), e2 = cross(ax, e1);
        int j = 0;
        for (double s = carry; s < len; s += spacing, ++j) {
            const Vec3 at = sp.a + ax * s;
            const double roll = (lk.pairs ? (j / 2) * (PI * 0.5) + (j % 2) * PI : j * 2.399) + (rnd() - 0.5) * 0.6;
            const Vec3 out = e1 * std::cos(roll) + e2 * std::sin(roll);
            const double ang = (lk.angle + (rnd() - 0.5) * 30.0) * PI / 180.0;
            Vec3 ld = normalize(ax * std::cos(ang) + out * std::sin(ang));
            ld = normalize(ld * (1.0 - lk.droop) + Vec3(0, -1, 0) * lk.droop);
            // (its face: toward the sky as the species turns them, and out of the crown)
            Vec3 sky = Vec3(0, 1, 0) - ld * dot(Vec3(0, 1, 0), ld);
            Vec3 away = (at - mid) - ld * dot(at - mid, ld);
            sky = sky.length() > 1e-6 ? normalize(sky) : out;
            away = away.length() > 1e-6 ? normalize(away) : sky;
            const Vec3 face = normalize(sky * lk.face + away * (1.0 - lk.face) + out * 0.3);
            const double depth = std::clamp((at - mid).length() / crownR, 0.0, 1.0);
            const double ao = 0.55 + 0.45 * depth;
            const int tile = flowers ? static_cast<int>(rnd() * 2.0) : static_cast<int>(rnd() * 4.0) % 4;
            addKitLeaf(m, at, ld, face, lk.size * grow * (0.8 + 0.4 * rnd()), lk.fold, lk.curl * (0.6 + 0.8 * rnd()), 2, tile,
                       leafColour(at) * (ao * (0.85 + 0.3 * rnd())));
            ++laid;
        }
        carry = std::fmod(carry + spacing - std::fmod(len, spacing), spacing);
        // (a cluster of cupped blossoms at a twig's end -- about 800 blossoms on the tree: each cup costs a leaf's
        // triangles several times over)
        const double clusterOdds = std::min(0.7, 800.0 / std::max(1.0, static_cast<double>(spots.size()) * std::max(3, lk.perCluster)));
        if (flowers && rnd() < clusterOdds) {
            const int per = std::max(3, lk.perCluster);
            for (int b = 0; b < per; ++b) {
                const double phi = 2.0 * PI * b / per + rnd();
                const Vec3 stalk = normalize(ax * 0.6 + (e1 * std::cos(phi) + e2 * std::sin(phi)) * 0.8 + Vec3(0, 0.3, 0));
                const double sz = lk.blossomSize * std::min(1.6, grow) * (0.85 + 0.3 * rnd());
                addKitBlossom(m, sp.b + stalk * (0.9 * sz), stalk, sz, lk.cup, rnd() * 2.0 * PI, 2, 2 + (b % 2), leafColour(sp.b));
            }
        }
    }
    return laid;
}


BranchCardSet bakeNeedleCards(const TreeRecipe& r, int variants, uint32_t seed) {
    BranchCardSet set;
    const NeedleSpec& nd = r.needles;
    if (!nd.set) return set;
    const bool tuft = nd.kind == "tuft", flat = nd.kind == "flat";
    const int nv = std::max(1, variants), kinds = tuft ? 3 : 2;
    const int cw = 160, ch = 480, cols = nv, rows = kinds;
    const int W = cols * cw, H = rows * ch;
    for (TextureData* t : {&set.atlas.albedo, &set.atlas.normal}) {
        t->width = W; t->height = H; t->channels = 4;
        t->pixels.assign(static_cast<std::size_t>(W) * H * 4, 0);
    }
    for (std::size_t i = 3; i < set.atlas.normal.pixels.size(); i += 4) {
        set.atlas.normal.pixels[i - 3] = 128; set.atlas.normal.pixels[i - 2] = 128; set.atlas.normal.pixels[i - 1] = 255;
        set.atlas.normal.pixels[i] = 255;
    }
    TextureData white;   // (a needle is its vertex colour: a solid "leaf" texture)
    white.width = white.height = 4; white.channels = 4;
    white.pixels.assign(64, 255);
    BarkKind barkKind;
    BarkTextures barkMaps;
    if (barkKindFromName(r.bark, barkKind)) barkMaps = barkTextures(barkKind, 1u, 256);
    CardViewExtras extras;
    extras.barkAlbedo = &barkMaps.albedo;
    extras.barkNormal = &barkMaps.normal;
    extras.depthShade = 0.3;
    extras.trueNormals = true;
    extras.alphaGrow = 1;   // (Glenn, of the spruce: "your needles need more thickness")
    uint32_t h = seed * 2654435761u + 5u;
    auto rnd = [&]() { h ^= h << 13; h ^= h >> 17; h ^= h << 5; return (h & 0xffffff) / 16777216.0; };
    const double L = nd.length, w = nd.width;
    const Vec3 X(1, 0, 0), Y(0, 1, 0), Z(0, 0, 1);
    // one needle: from `p` along `d` (bending a little toward `bendTo`), its face turned to `radial`
    auto needle = [&](RenderMesh& m, const Vec3& p, const Vec3& d, const Vec3& radial, double len, const Vec3& col) {
        Vec3 wv = cross(d, radial);
        if (wv.length() < 1e-6) wv = cross(d, Z);
        wv = normalize(wv);
        const Vec3 mid = p + d * (0.5 * len) + radial * (0.04 * len), tip = p + d * len + radial * (0.02 * len);
        const Vec3 nrm = normalize(radial * 0.8 + Z * 0.4);
        const uint32_t b = static_cast<uint32_t>(m.vertices.size());
        // (a needle is RIDGED down its middle -- a spruce's four-sided: its two halves' normals tilt apart, so in the
        // normal map each needle has a lit side and a shaded side, not one flat tone; Glenn: "Are the needles using a
        // normal map as well?")
        const Vec3 nL = normalize(nrm - wv * 0.7), nR = normalize(nrm + wv * 0.7);
        const Vec3 pts[6] = {p - wv * (0.5 * w), p + wv * (0.5 * w), mid - wv * (0.45 * w), mid + wv * (0.45 * w), tip, mid};
        const Vec3 nrms[6] = {nL, nR, nL, nR, nrm, nrm};
        for (int k = 0; k < 6; ++k) {
            Vertex v(pts[k], nrms[k], wv, 0.5f, 0.5f);
            v.color = col;
            m.vertices.push_back(v);
        }
        // (its midrib the vertex `b + 5`: each half its own triangles, the ridge between them)
        const uint32_t ridge0 = static_cast<uint32_t>(m.vertices.size());
        Vertex r0 = m.vertices[b];
        r0.position = p;
        r0.normal = nrm;
        m.vertices.push_back(r0);
        MeshBuilder::triFacing(m, b, ridge0, b + 5, nrm);
        MeshBuilder::triFacing(m, b, b + 5, b + 2, nrm);
        MeshBuilder::triFacing(m, ridge0, b + 1, b + 3, nrm);
        MeshBuilder::triFacing(m, ridge0, b + 3, b + 5, nrm);
        MeshBuilder::triFacing(m, b + 2, b + 5, b + 4, nrm);
        MeshBuilder::triFacing(m, b + 5, b + 3, b + 4, nrm);
    };
    auto needleColour = [&](double t) {   // (lighter toward the shoot's end: this year's growth)
        const Vec3 c = nd.color + (nd.colorHi - nd.color) * rnd();
        const double k = std::clamp((t - 0.72) / 0.28, 0.0, 1.0);
        return c * (1.0 - k) + nd.tipColor * k;
    };
    // a shoot along +Y, `S` long: its twig, and its needles from `from` (a share of it) to its end
    // a shoot from `o` along `ax` (in the picture's plane), `S` long: its twig, and its needles from `from` (a share of
    // it) to its end -- and, a SPRAY (`subs` pairs), its own side shoots off both sides, as a spruce or fir sprig has
    std::function<void(RenderMesh&, RenderMesh&, const Vec3&, const Vec3&, double, double, int)> shootAt;
    shootAt = [&](RenderMesh& wood, RenderMesh& ndl, const Vec3& o, const Vec3& axis, double S, double from, int subs) {
        const double tr = 0.0015 + 0.004 * S;   // (a shoot is a few millimetres across; its needles make its width)
        const Vec3 Ya = normalize(axis), Xa = normalize(cross(Ya, Z));
        MeshBuilder::append(wood, barkTube({o, o + Ya * S}, {tr, 0.5 * tr}, 6, {nd.twigColor, nd.twigColor}));
        for (int k = 0; k < 2 * subs; ++k) {
            const double tt = 0.2 + 0.6 * (k / 2 + 0.5 * rnd()) / std::max(1, subs);
            const double sgn = k % 2 ? 1.0 : -1.0, a = (nd.sideAngle + (rnd() - 0.5) * 12.0) * PI / 180.0;
            shootAt(wood, ndl, o + Ya * (tt * S), normalize(Ya * std::cos(a) + Xa * (sgn * std::sin(a))), S * (0.45 - 0.2 * tt), 0.0, 0);
        }
        const int n = std::max(6, static_cast<int>(S * (1.0 - from) * nd.density * 100.0));
        for (int i = 0; i < n; ++i) {
            const double t = from + (1.0 - from) * (i + rnd()) / n;
            const Vec3 p = o + Ya * (t * S);
            double phi = flat ? (i % 2 ? 0.0 : PI) + (rnd() - 0.5) * 0.5 : i * 2.39996 + (rnd() - 0.5) * 0.4;
            if (flat && rnd() < 0.25) phi = PI * 0.5 + (rnd() - 0.5) * 0.6;   // (a few above the twig, short)
            const Vec3 radial = Xa * std::cos(phi) + Z * std::sin(phi);
            const double fwd = (nd.forward + (rnd() - 0.5) * 16.0) * PI / 180.0;
            const Vec3 d = normalize(Ya * std::cos(fwd) + radial * std::sin(fwd));
            const double len = L * (0.8 + 0.4 * rnd()) * (flat && std::fabs(std::sin(phi)) > 0.7 ? 0.6 : 1.0) *
                               (0.7 + 0.3 * std::min(1.0, (1.0 - t) * 6.0 + 0.3));
            needle(ndl, p + radial * tr, d, radial, len, needleColour(t));
        }
    };
    // a pine's tuft: bundles of two or three long needles round the twig's last stretch, crowded at its end
    auto tuftShoot = [&](RenderMesh& wood, RenderMesh& ndl, double S) {
        const double tr = 0.003;
        wood = barkTube({Vec3(0, 0, 0), Vec3(0, S, 0)}, {tr, 0.6 * tr}, 6, {nd.twigColor, nd.twigColor});
        const int bundles = std::max(8, static_cast<int>(S * nd.density * 100.0));
        for (int i = 0; i < bundles; ++i) {
            const double t = std::pow((i + rnd()) / bundles, 0.6);
            const Vec3 p(0, t * S, 0);
            const double phi0 = i * 2.39996;
            const int k = rnd() < 0.5 ? 2 : 3;
            for (int j = 0; j < k; ++j) {
                const double phi = phi0 + (j - 0.5 * (k - 1)) * 0.35;
                const Vec3 radial = X * std::cos(phi) + Z * std::sin(phi);
                const double fwd = (nd.forward * (0.6 + 0.6 * t) + (rnd() - 0.5) * 14.0) * PI / 180.0;
                needle(ndl, p + radial * tr, normalize(Y * std::cos(fwd) + radial * std::sin(fwd)), radial,
                       L * (0.85 + 0.3 * rnd()), needleColour(0.5 + 0.5 * t));
            }
        }
    };
    int cell = 0;
    for (int kind = 0; kind < kinds; ++kind)
        for (int v = 0; v < nv; ++v, ++cell) {
            RenderMesh wood, ndl;
            const double S = kind == 0 ? 0.30 : kind == 1 ? 0.16 : 0.14;
            if (kind == 2) tuftShoot(wood, ndl, S);
            else   // (a pine's shoot card lies only on the needled share of its twig; a side shoot is a spray)
                shootAt(wood, ndl, Vec3(0, 0, 0), Y, S, 0.0, kind == 1 && !tuft ? 3 : 0);
            double half = 0.0, y1 = 0.0;
            for (const RenderMesh* m : {&wood, &ndl})
                for (const Vertex& vx : m->vertices) { half = std::max(half, std::fabs(vx.position.x)); y1 = std::max(y1, vx.position.y); }
            half = half * 1.04 + 0.004;
            y1 = y1 * 1.02 + 0.004;
            const double y0 = -0.01 * S;
            const int c = cell % cols, rr = cell / cols;
            const std::size_t off = (static_cast<std::size_t>(rr) * ch * W + static_cast<std::size_t>(c) * cw) * 4;
            renderCardView(wood, ndl, white, false, -half, half, y0, y1, cw, ch, set.colourScale,
                           set.atlas.albedo.pixels.data() + off, W * 4, set.atlas.normal.pixels.data() + off, &extras);
            BranchCardSet::Picture pic;
            BranchCardSet::View vw;
            vw.u0 = static_cast<float>(c) / cols; vw.u1 = static_cast<float>(c + 1) / cols;
            vw.v0 = static_cast<float>(rr) / rows; vw.v1 = static_cast<float>(rr + 1) / rows;
            vw.x0 = static_cast<float>(-half); vw.x1 = static_cast<float>(half);
            vw.y0 = static_cast<float>(y0); vw.y1 = static_cast<float>(y1);
            pic.views.push_back(vw);
            pic.length = static_cast<float>(S);
            pic.halfWidth = static_cast<float>(half);
            pic.kind = kind;
            set.pictures.push_back(pic);
        }
    set.twigLength = 0.30f;
    return set;
}

int wearNeedles(TreeMesh& tree, const TreeRecipe& r, const BranchCardSet& set, double shoots, int twigTris, int maxPlanes,
                uint32_t seed) {
    const NeedleSpec& nd = r.needles;
    if (!nd.set || set.empty() || tree.twigs.empty()) return 0;
    tree.foliage = 5;
    const bool tuft = nd.kind == "tuft", flat = nd.kind == "flat";
    std::vector<int> byKind[3];
    for (std::size_t i = 0; i < set.pictures.size(); ++i)
        if (set.pictures[i].kind >= 0 && set.pictures[i].kind < 3) byKind[set.pictures[i].kind].push_back(static_cast<int>(i));
    if (byKind[0].empty()) return 0;
    uint32_t h = seed * 2246822519u + 9u;
    auto rnd = [&]() { h ^= h << 13; h ^= h >> 17; h ^= h << 5; return (h & 0xffffff) / 16777216.0; };
    Vec3 mid(0, 0, 0);
    for (const TreeMesh::Twig& t : tree.twigs) mid = mid + t.b;
    mid = mid * (1.0 / tree.twigs.size());
    double crownR = 0.5;
    for (const TreeMesh::Twig& t : tree.twigs) crownR = std::max(crownR, (t.b - mid).length());
    const double inv = 1.0 / set.colourScale, bend = 0.45;
    RenderMesh& m = tree.leaves;
    RenderMesh& bark = tree.branches;
    const bool data = bark.tangentIsData;
    const Vec3 barkCol = r.barkColor * 0.5 + r.barkColorHi * 0.5;
    int laid = 0;
    auto pick = [&](int kind) {
        const std::vector<int>& v = byKind[kind].empty() ? byKind[0] : byKind[kind];
        return v[static_cast<std::size_t>(rnd() * v.size()) % v.size()];
    };
    // one plane of picture `pi` from `a` along `axis` for `len`, across `right`
    auto plane = [&](int pi, const Vec3& a, const Vec3& axis, double len, const Vec3& right0) {
        const BranchCardSet::Picture& pic = set.pictures[static_cast<std::size_t>(pi)];
        const BranchCardSet::View& p = pic.views.front();
        const double s = len / std::max(1e-3f, pic.length);
        const double sw = std::clamp(s, 0.85, 1.2);   // (a needle keeps its length across the card)
        const Vec3 right = normalize(right0 - axis * dot(right0, axis));
        Vec3 face = normalize(cross(right, axis));
        if (face.y < 0.0) face = face * -1.0;
        const uint32_t b = static_cast<uint32_t>(m.vertices.size());
        const double xs[2] = {p.x0, p.x1}, ys[2] = {p.y0, p.y1};
        for (int yi = 0; yi < 2; ++yi)
            for (int xi = 0; xi < 2; ++xi) {
                const Vec3 pos = a + axis * (ys[yi] * s) + right * (xs[xi] * sw);
                Vec3 out = pos - mid;
                out = out.length() > 1e-6 ? normalize(out) : Vec3(0, 1, 0);
                const Vec3 nrm = normalize(face * (1.0 - bend) + out * bend);
                Vertex vx(pos, nrm, normalize(right - nrm * dot(right, nrm)), xi ? p.u1 : p.u0, yi ? p.v0 : p.v1);
                const double ao = 0.5 + 0.5 * std::clamp((pos - mid).length() / crownR, 0.0, 1.0);
                vx.color = Vec3(ao * inv, ao * inv, ao * inv);
                m.vertices.push_back(vx);
            }
        MeshBuilder::quadFacing(m, b + 0, b + 1, b + 3, b + 2, face);
    };
    // a shoot's needles: crossed planes (a fir's: one in the branch's plane, opened in a shallow V)
    auto needles = [&](int kind, const Vec3& a, const Vec3& axis, double len, const Vec3& side) {
        const int pi = pick(kind);
        const Vec3 up = normalize(cross(side, axis));
        if (flat) {
            plane(pi, a, axis, len, normalize(side + up * 0.25));
            plane(pi, a, axis, len, normalize(side - up * 0.25));
        } else if (kind == 2) {   // (a tuft: five planes round the twig -- three read as flat fans)
            for (int k = 0; k < 5; ++k) {
                const double rr = k * PI / 5.0 + rnd() * 0.2;
                plane(pi, a, axis, len, side * std::cos(rr) + up * std::sin(rr));
            }
        } else {   // (a brush: three planes round the shoot -- two crossed went edge-on and the shoot vanished)
            const double rr = (rnd() - 0.5) * 0.4;
            for (int k = 0; k < 3; ++k) {
                const double a2 = rr + k * PI / 3.0;
                plane(k == 0 ? pi : pick(kind), a, axis, len, side * std::cos(a2) + up * std::sin(a2));
            }
        }
        ++laid;
    };
    // THE WOOD: the twigs' bark, the thickest first, to the budget (the thinnest wear their cards' own drawn twig)
    {
        std::vector<std::size_t> order(tree.twigs.size());
        for (std::size_t i = 0; i < order.size(); ++i) order[i] = i;
        std::sort(order.begin(), order.end(), [&](std::size_t a, std::size_t b) { return tree.twigs[a].r > tree.twigs[b].r; });
        int tris = 0;
        for (std::size_t i : order) {
            const TreeMesh::Twig& t = tree.twigs[i];
            const double r0 = std::max(0.003, static_cast<double>(t.r));
            const int sides = r0 < 0.008 ? 3 : r0 < 0.02 ? 4 : 5;
            if (tris + 2 * sides > twigTris) break;
            RenderMesh tube = barkTube({t.a, t.b}, {r0, std::max(0.002, 0.75 * r0)}, sides, {barkCol, barkCol});
            for (Vertex& v : tube.vertices) {
                v.u *= 0.5f;
                if (data) v.tangent = Vec3(v.u, v.v, 0.0);
            }
            tris += static_cast<int>(tube.indices.size() / 3);
            MeshBuilder::append(bark, tube);
        }
    }
    // where the needles go: every twig, and (a spruce's) the modelled limbs thin enough -- its whole branches clothed
    struct Shoot { Vec3 a, b; bool tip; };
    std::vector<Shoot> along;
    for (const TreeMesh::Twig& t : tree.twigs) along.push_back({t.a, t.b, t.leafy});
    if (nd.limbRadius > 0.0)
        for (const TreeMesh::Limb& l : tree.limbs)
            if (l.rb <= nd.limbRadius && l.ra <= 1.6 * nd.limbRadius && l.b.y > 0.3) along.push_back({l.a, l.b, false});
    // THE BUDGET (card planes): each shoot's own planes, and its side shoots' as many as fit -- thinned evenly
    const int perShoot = flat ? 2 : 3;
    double planned = 0.0, sides = 0.0;
    for (const Shoot& t : along) {
        const double len = (t.b - t.a).length();
        planned += perShoot;
        sides += 2.0 * perShoot * std::lround(nd.sideShoots * shoots * std::min(1.0, len / 0.3));
    }
    const double keepOwn = std::min(1.0, maxPlanes / std::max(1.0, planned));
    const double keepSide = std::clamp((maxPlanes - planned) / std::max(1.0, sides), 0.0, 1.0);
    for (const Shoot& t : along) {
        if (keepOwn < 1.0 && rnd() >= keepOwn) continue;
        const Vec3 d = t.b - t.a;
        const double len = d.length();
        if (len < 1e-3) continue;
        const Vec3 ax = d * (1.0 / len);
        // the branch's plane: its side, level (a conifer's sprays lie near flat)
        Vec3 side = cross(ax, Vec3(0, 1, 0));
        side = side.length() > 0.1 ? normalize(side) : normalize(cross(ax, Vec3(1, 0, 0)));
        // its own needles, along the share `reach` from its end
        const double from = 1.0 - nd.reach;
        needles(0, t.a + d * from, ax, len * nd.reach, side);
        if (tuft && t.tip) needles(2, t.b - ax * 0.02, normalize(ax + Vec3(0, 0.25, 0)), 0.14 * (0.9 + 0.2 * rnd()), side);
        // its side shoots, from points ON it, in its plane, forward, shorter toward its end
        const int n = static_cast<int>(std::lround(nd.sideShoots * shoots * std::min(1.0, len / 0.3)));
        for (int k = 0; k < 2 * n; ++k) {
            if (keepSide < 1.0 && rnd() >= keepSide) continue;
            const double tt = from + (1.0 - from) * (0.12 + 0.8 * (k / 2 + rnd() * 0.6) / std::max(1, n));
            const double sgn = k % 2 ? 1.0 : -1.0;
            const double a = (nd.sideAngle + (rnd() - 0.5) * 16.0) * PI / 180.0;
            // (in the branch's plane, and fanned up or down out of it: a spray has depth, not one flat comb)
            const Vec3 upOut = normalize(cross(side, ax)) * ((rnd() - 0.5) * 0.9);
            Vec3 sd = normalize(ax * std::cos(a) + side * (sgn * std::sin(a)) + upOut + Vec3(0, 0.08, 0));
            if (rnd() < nd.hang)   // (a pendant shoot: down from the branch, a little forward and out -- the curtain)
                sd = normalize(ax * 0.35 + side * (sgn * 0.3) + Vec3(0, -1, 0));
            const double sl = nd.shootLength * (1.15 - 0.6 * tt) * (0.8 + 0.4 * rnd());
            needles(1, t.a + d * tt, sd, sl, normalize(cross(Vec3(0, 1, 0), sd)));
        }
    }
    return laid;
}

}  // namespace engine
