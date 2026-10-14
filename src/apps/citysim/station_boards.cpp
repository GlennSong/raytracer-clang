#include "station_boards.h"
#include "../../engine/mesh_builder.h"

#include <algorithm>
#include <cmath>

using engine::Mat4;
using engine::RenderMesh;
using engine::Vec3;
using engine::Vertex;

namespace citysim {

namespace {

const Vec3 kBlue(0.08, 0.20, 0.46);
const Vec3 kPost(0.22, 0.23, 0.25);

void box(RenderMesh& m, Vec3 lo, Vec3 hi, const Vec3& col) {
    const Vec3 c[8] = {{lo.x, lo.y, lo.z}, {hi.x, lo.y, lo.z}, {hi.x, hi.y, lo.z}, {lo.x, hi.y, lo.z},
                       {lo.x, lo.y, hi.z}, {hi.x, lo.y, hi.z}, {hi.x, hi.y, hi.z}, {lo.x, hi.y, hi.z}};
    const int f[6][4] = {{0, 3, 2, 1}, {4, 5, 6, 7}, {0, 4, 7, 3}, {1, 2, 6, 5}, {0, 1, 5, 4}, {3, 7, 6, 2}};
    const Vec3 n[6] = {{0, 0, -1}, {0, 0, 1}, {-1, 0, 0}, {1, 0, 0}, {0, -1, 0}, {0, 1, 0}};
    // (emitQuad winds each face to its normal: the engine's front face is clockwise seen from outside -- this list was
    // counter-clockwise, so the boards' posts and rims were inside out)
    for (int k = 0; k < 6; ++k) engine::MeshBuilder::emitQuad(m, c[f[k][0]], c[f[k][1]], c[f[k][2]], c[f[k][3]], n[k], col);
}

}  // namespace

std::vector<StationBoard> stationBoards(const RailSim& sim) {
    std::vector<StationBoard> out;
    for (const RailLine& L : sim.lines())
        for (const engine::RailStop& st : L.stops) {
            if (st.name.empty() || st.s1 - st.s0 < 2.5 * kBoardFromEnd) continue;
            for (double s : {st.s0 + kBoardFromEnd, st.s1 - kBoardFromEnd}) {
                const Vec3 a = L.at(s - 2.0), b = L.at(s + 2.0), c = L.at(s);
                Vec3 f(b.x - a.x, 0, b.z - a.z);
                const double fl = f.length();
                f = fl > 1e-9 ? f * (1.0 / fl) : Vec3(0, 0, 1);
                const Vec3 left(f.z, 0, -f.x);                       // rail_sim.cpp's car frame
                const Vec3 out_ = left * static_cast<double>(st.side);  // the platform's side
                const Vec3 foot = Vec3(c.x, st.top, c.z) + out_ * (engine::kPlatformGap + kBoardBack);
                const Vec3 z = out_ * -1.0;                            // facing the track
                const Vec3 x = engine::cross(Vec3(0, 1, 0), z);        // along, so +x, +y, +z is right-handed
                StationBoard B;
                B.name = st.name;
                Mat4& m = B.m;
                m.m[0][0] = x.x; m.m[1][0] = x.y; m.m[2][0] = x.z;
                m.m[0][1] = 0;   m.m[1][1] = 1;   m.m[2][1] = 0;
                m.m[0][2] = z.x; m.m[1][2] = z.y; m.m[2][2] = z.z;
                m.m[0][3] = foot.x; m.m[1][3] = foot.y; m.m[2][3] = foot.z;
                m.m[3][0] = m.m[3][1] = m.m[3][2] = 0; m.m[3][3] = 1;
                out.push_back(B);
            }
        }
    return out;
}

RenderMesh stationBoardFrame() {
    RenderMesh m;
    const double hw = kBoardW / 2, y0 = kBoardBottom, y1 = kBoardBottom + kBoardH;
    for (int s = -1; s <= 1; s += 2) {   // the two posts, a little in from the board's ends
        const double px = s * (hw - 0.35);
        box(m, Vec3(px - 0.05, 0, -0.05), Vec3(px + 0.05, y1 + 0.06, 0.05), kPost);
    }
    // the board: a blue slab with a slim dark rim the faces sit just proud of
    box(m, Vec3(-hw - 0.05, y0 - 0.05, -0.04), Vec3(hw + 0.05, y1 + 0.05, 0.04), kPost);
    return m;
}

RenderMesh stationBoardFaces() {
    RenderMesh m;
    const double hw = kBoardW / 2, y0 = kBoardBottom, y1 = kBoardBottom + kBoardH;
    for (int side = -1; side <= 1; side += 2) {   // +1 the front (+z, toward the track), -1 the back
        const double z = side * 0.045;
        const Vec3 n(0, 0, static_cast<double>(side));
        // seen from its own side, left to right is -x..+x on the front and +x..-x on the back
        const double xl = side > 0 ? -hw : hw, xr = -xl;
        engine::MeshBuilder::emitQuadUV(m, Vec3(xl, y1, z), Vec3(xr, y1, z), Vec3(xr, y0, z), Vec3(xl, y0, z), n, Vec3(1, 1, 1),
                                0, 0, 1, 0, 1, 1, 0, 1);
    }
    return m;
}

void stationBoardImage(const engine::Font& font, const std::string& name, engine::TextImage& out) {
    const int h = 120, w = static_cast<int>(std::lround(h * kBoardW / kBoardH));
    const uint8_t blue[4] = {static_cast<uint8_t>(kBlue.x * 255), static_cast<uint8_t>(kBlue.y * 255),
                             static_cast<uint8_t>(kBlue.z * 255), 255};
    const uint8_t white[4] = {245, 245, 240, 255};
    out.resize(w, h, blue);
    // a white rule inside the edge, the way a running-in board is framed
    out.fillRect(6, 6, w - 6, 10, white);
    out.fillRect(6, h - 10, w - 6, h - 6, white);
    float px = 74.0f;
    const float room = static_cast<float>(w) - 60.0f;
    const float tw = font.measure(name, px);
    const float xs = tw > room ? room / tw : 1.0f;   // a long name is narrowed, not shrunk
    const float cap = font.capHeight(px);
    font.draw(out, name, (static_cast<float>(w) - tw * xs) / 2.0f, (static_cast<float>(h) + cap) / 2.0f, px, white, xs);
}

}  // namespace citysim
