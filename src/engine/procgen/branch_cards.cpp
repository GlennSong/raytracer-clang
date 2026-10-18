#include "branch_cards.h"

#include "../mesh_builder.h"
#include "foliage_kit.h"   // addKitLeaf, addKitBlossom
#include "leaf_card.h"     // foliageElementAtlas
#include "real_tree.h"     // renderCardView

#include <algorithm>
#include <cmath>

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
                    bool leafy) {
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
        MeshBuilder::append(wood, MeshBuilder::tube(pts, rad, 5, cols));
        if (!leafy) return;
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
        if (flowers) {   // the blossom clusters along it (a sakura's umbels)
            const int nc = std::max(2, lk.clusters + 1);
            for (int c = 0; c < nc; ++c) {
                const double t = 0.25 + 0.75 * (c + 0.5) / nc;
                const int k = std::min(3, static_cast<int>(t * 4.0));
                const Vec3 at = pts[k] + (pts[k + 1] - pts[k]) * (t * 4.0 - k);
                for (int b = 0; b < std::max(3, lk.perCluster); ++b) {
                    const double phi = 2.0 * PI * b / std::max(3, lk.perCluster) + rnd();
                    const Vec3 stalk = normalize(Vec3(std::cos(phi) * 0.8, std::sin(phi) * 0.8, 1.0));
                    const double sz = lk.blossomSize * (0.85 + 0.3 * rnd());
                    addKitBlossom(leaves, at + stalk * (0.9 * sz), stalk, sz, lk.cup, rnd() * 2.0 * PI, 2, 2 + static_cast<int>(rnd() * 2.0) % 2,
                                  leafColour());
                }
            }
        }
    };
    int cell = 0;
    for (int kind = 0; kind < 4; ++kind)
        for (int v = 0; v < nv; ++v, ++cell) {
            const bool leafy = kind < 2, fork = kind == 1 || kind == 3;
            paletteNo = v;
            RenderMesh wood, leaves;
            const double r0 = std::max(0.004, 0.03 * L);
            twig(wood, leaves, Vec3(0, 0, 0), Vec3((rnd() - 0.5) * 0.1, 1, 0), L, r0, leafy);
            if (fork) {   // a side twig off it, a little past half way, to one side
                const double sgn = rnd() < 0.5 ? -1.0 : 1.0, at = 0.35 + 0.2 * rnd();
                const double a = (28.0 + 14.0 * rnd()) * PI / 180.0;
                twig(wood, leaves, Vec3(0, L * at, 0), Vec3(sgn * std::sin(a), std::cos(a), 0), L * (0.55 + 0.15 * rnd()), r0 * 0.7, leafy);
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
                           set.atlas.albedo.pixels.data() + off, W * 4, set.atlas.normal.pixels.data() + off);
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

}  // namespace engine
