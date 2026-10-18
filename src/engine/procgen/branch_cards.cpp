#include "branch_cards.h"

#include "../mesh_builder.h"
#include "real_tree.h"   // renderCardView

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

}  // namespace engine
