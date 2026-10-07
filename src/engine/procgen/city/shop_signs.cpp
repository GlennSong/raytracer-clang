#include "shop_signs.h"
#include "trades.h"

#include <algorithm>
#include <cmath>
#include <map>

namespace engine {

namespace {

// THE WORDS. Hawaiian place and nature words, Japanese ones, and plain American ones: the island is all three.
const char* kHawaiian[] = {"Kona", "Mauka", "Makai", "Hilo", "Lanai", "Hana", "Pali", "Nalu", "Moana", "Lani",
                           "Ulu", "Mele", "Honu", "Pua", "Koa", "Aina", "Ohana", "Lehua", "Manoa", "Kai"};
const char* kJapanese[] = {"Hoshi", "Kumo", "Sora", "Nami", "Tora", "Yama", "Kaze", "Tsuki", "Midori", "Sakura",
                           "Kaede", "Umi", "Hikari", "Mori", "Hinata"};
const char* kEnglish[] = {"Juniper", "Copper", "Saltwater", "Morning", "Harbor", "Golden", "Silver", "Lucky", "Cedar",
                          "Maple", "Blue Door", "Sunset", "Palm", "Driftwood", "Lantern", "Corner", "Old Town",
                          "Little", "Northside", "Bayview"};
const char* kCreature[] = {"Fox", "Anchor", "Owl", "Gecko", "Heron", "Pelican", "Crow", "Stag", "Marlin", "Turtle"};
const char* kNight[] = {"Neon", "Velvet", "Pulse", "Lava", "Echo", "Static", "Mirage", "Volt"};

template <std::size_t N> const char* pick(const char* (&a)[N], uint32_t& h) {
    h ^= h << 13; h ^= h >> 17; h ^= h << 5;
    return a[h % N];
}

}  // namespace

std::string shopName(uint8_t trade, uint32_t seed) {
    uint32_t h = seed * 0x9E3779B9u + 0x7F4A7C15u;
    if (!h) h = 1;
    auto H = [&]() { return std::string(pick(kHawaiian, h)); };
    auto J = [&]() { return std::string(pick(kJapanese, h)); };
    auto E = [&]() { return std::string(pick(kEnglish, h)); };
    h ^= h << 13; h ^= h >> 17; h ^= h << 5;
    const uint32_t form = h % 6u;
    switch (trade) {
        case 0:   // cafe
            switch (form) { case 0: return H() + " Coffee"; case 1: return E() + " Coffee Co."; case 2: return "Kissa " + J();
                            case 3: return "Cafe " + J(); case 4: return E() + " Roasters"; default: return H() + " Espresso"; }
        case 6:   // bakery
            switch (form % 4) { case 0: return E() + " Bakehouse"; case 1: return "Pan " + J(); case 2: return H() + " Bakery";
                                default: return E() + " Crumb"; }
        case 13:  // restaurant
            switch (form) { case 0: return "Ramen " + J(); case 1: return "Izakaya " + J(); case 2: return H() + " Grill";
                            case 3: return E() + " Kitchen"; case 4: return "Poke " + H(); default: return J() + " Sushi"; }
        case 14:  // bar
            switch (form % 5) { case 0: return "The " + E() + " " + pick(kCreature, h); case 1: return H() + " Tavern";
                                case 2: return E() + " Lounge"; case 3: return "Tiki " + H(); default: return E() + " Taproom"; }
        case 15:  // club
            return form % 2 ? std::string(pick(kNight, h)) + (form % 3 ? " Room" : " Club") : "Club " + J();
        case 1:   // grocery
            switch (form % 4) { case 0: return H() + " Market"; case 1: return E() + " Grocery"; case 2: return J() + " Mart";
                                default: return E() + " Market"; }
        case 2:   // boutique
            switch (form % 4) { case 0: return "Studio " + J(); case 1: return E() + " & " + E(); case 2: return H() + " Threads";
                                default: return E() + " Supply"; }
        case 3:   // bookshop
            switch (form % 3) { case 0: return E() + " Books"; case 1: return "Paper " + H(); default: return J() + " Books"; }
        case 4:   // electronics
            switch (form % 3) { case 0: return E() + " Electronics"; case 1: return "Circuit " + H(); default: return J() + " Denki"; }
        case 5:   // pharmacy
            switch (form % 3) { case 0: return H() + " Pharmacy"; case 1: return E() + " Drug"; default: return E() + " Apothecary"; }
        default:
            return E() + " Shop";
    }
}

ShopSignAtlas buildShopSignAtlas(const Font& font, const std::vector<ShopSign>& signs, int pagePx) {
    ShopSignAtlas atlas;
    constexpr int H = 72;   // a board's pixels tall
    const uint8_t board[4] = {18, 18, 22, 255};
    struct Made { int index; int w; TextImage img; };
    std::vector<Made> made;
    atlas.boards.resize(signs.size());
    atlas.smallestCapPx = 1e9f;
    for (std::size_t i = 0; i < signs.size(); ++i) {
        const ShopSign& s = signs[i];
        // the board is as wide as its name and a margin -- the fascia's full width only when the name needs it (a
        // name centred on its own nameplate, and atlas pages that hold dozens, not twenty)
        const int fasciaPx = std::clamp(static_cast<int>(std::lround(H * s.width / std::max(Real(0.05), s.height))), H, pagePx - 16);
        const TradeInfo* tr = tradeById(s.trade);
        const Vec3 c = tr ? tr->fascia : Vec3(0.9, 0.9, 0.9);
        const uint8_t ink[4] = {static_cast<uint8_t>(std::clamp(c.x, 0.0, 1.0) * 255), static_cast<uint8_t>(std::clamp(c.y, 0.0, 1.0) * 255),
                                static_cast<uint8_t>(std::clamp(c.z, 0.0, 1.0) * 255), 255};
        // the lettering: as tall as the board allows (capitals ~55% of it), condensed to 0.75 before it is set
        // smaller, centred, with a margin of a board-height's fifth either side
        float em = H * 0.80f, xs = 1.0f;
        const float budget = static_cast<float>(fasciaPx) - H * 0.4f;
        float tw = font.measure(s.text, em, xs);
        if (tw > budget) { xs = std::max(0.75f, budget / tw); tw = font.measure(s.text, em, xs); }
        if (tw > budget) { em *= budget / tw; tw = font.measure(s.text, em, xs); }
        const int w = std::min(fasciaPx, static_cast<int>(std::ceil(tw + H * 0.6f)));
        TextImage img;
        img.resize(w, H, board);
        const float cap = font.capHeight(em);
        atlas.smallestCapPx = std::min(atlas.smallestCapPx, cap);
        font.draw(img, s.text, 0.5f * (w - tw), 0.5f * (H + cap), em, ink, xs);
        made.push_back({static_cast<int>(i), w, std::move(img)});
    }
    std::stable_sort(made.begin(), made.end(), [](const Made& a, const Made& b) { return a.w > b.w; });
    const int pad = 4, rowH = H + 2 * pad;
    int x = 0, y = 0;
    std::vector<int> usedH;
    auto newPage = [&]() {
        TextImage page;
        page.resize(pagePx, pagePx, board);
        atlas.pages.push_back(std::move(page));
        usedH.push_back(0);
        x = 0; y = 0;
    };
    for (Made& m : made) {
        const int cw = m.w + 2 * pad;
        if (atlas.pages.empty()) newPage();
        if (x + cw > pagePx) { x = 0; y += rowH; }
        if (y + rowH > pagePx) newPage();
        TextImage& page = atlas.pages.back();
        const int ox = x + pad, oy = y + pad;
        for (int yy = -pad; yy < H + pad; ++yy)   // edge-extended so the mips do not bleed a neighbour in
            for (int xx = -pad; xx < m.w + pad; ++xx) {
                const int sx = std::clamp(xx, 0, m.w - 1), sy = std::clamp(yy, 0, H - 1);
                const int dx = ox + xx, dy = oy + yy;
                if (dx < 0 || dy < 0 || dx >= page.w || dy >= page.h) continue;
                const uint8_t* src = &m.img.rgba[(static_cast<std::size_t>(sy) * m.w + sx) * 4];
                uint8_t* dst = &page.rgba[(static_cast<std::size_t>(dy) * page.w + dx) * 4];
                for (int k = 0; k < 4; ++k) dst[k] = src[k];
            }
        ShopSignAtlas::Board& b = atlas.boards[static_cast<std::size_t>(m.index)];
        b.page = static_cast<int>(atlas.pages.size()) - 1;
        b.u0 = static_cast<float>(ox) / pagePx;
        b.u1 = static_cast<float>(ox + m.w) / pagePx;
        b.v0 = static_cast<float>(oy);
        b.v1 = static_cast<float>(oy + H);
        b.widthFrac = static_cast<float>(m.w) / std::max(1, static_cast<int>(std::lround(72.0 * signs[static_cast<std::size_t>(m.index)].width /
                                                                                         std::max(Real(0.05), signs[static_cast<std::size_t>(m.index)].height))));
        usedH.back() = std::max(usedH.back(), y + rowH);
        x += cw;
    }
    for (std::size_t pi = 0; pi < atlas.pages.size(); ++pi) {   // trim each page to its rows; v to texture space
        TextImage& page = atlas.pages[pi];
        const int h = std::max(1, usedH[pi]);
        page.rgba.resize(static_cast<std::size_t>(page.w) * h * 4);
        page.h = h;
    }
    for (ShopSignAtlas::Board& b : atlas.boards) {
        if (atlas.pages.empty()) break;
        const float h = static_cast<float>(atlas.pages[static_cast<std::size_t>(b.page)].h);
        b.v0 /= h; b.v1 /= h;
    }
    if (signs.empty()) atlas.smallestCapPx = 0;
    return atlas;
}

ShopSignMeshes buildShopSignMeshes(const std::vector<ShopSign>& signs, const ShopSignAtlas& atlas, Real cellSize) {
    ShopSignMeshes out;
    std::map<std::tuple<int, int, int>, std::size_t> cellOf;
    for (std::size_t i = 0; i < signs.size() && i < atlas.boards.size(); ++i) {
        const ShopSign& s = signs[i];
        const ShopSignAtlas::Board& b = atlas.boards[i];
        const auto key = std::make_tuple(static_cast<int>(std::floor(s.centre.x / cellSize)),
                                         static_cast<int>(std::floor(s.centre.z / cellSize)), b.page);
        auto it = cellOf.find(key);
        if (it == cellOf.end()) {
            it = cellOf.emplace(key, out.cells.size()).first;
            out.cells.emplace_back();
            out.cells.back().page = b.page;
        }
        RenderMesh& m = out.cells[it->second].mesh;
        const Real hw = s.width * std::min(1.0f, b.widthFrac) * 0.5, hh = s.height * 0.5;
        const uint32_t base = static_cast<uint32_t>(m.vertices.size());
        auto vtx = [&](Vec3 pos, float u, float v) {
            Vertex vx(pos, s.n, u, v);
            vx.tangent = s.right;
            m.vertices.push_back(vx);
        };
        vtx(s.centre - s.right * hw + s.up * hh, b.u0, b.v0);   // TL
        vtx(s.centre + s.right * hw + s.up * hh, b.u1, b.v0);   // TR
        vtx(s.centre + s.right * hw - s.up * hh, b.u1, b.v1);   // BR
        vtx(s.centre - s.right * hw - s.up * hh, b.u0, b.v1);   // BL
        for (uint32_t k : {0u, 2u, 1u, 0u, 3u, 2u}) m.indices.push_back(base + k);
    }
    for (ShopSignMeshes::Cell& c : out.cells) {
        Vec3 lo(1e30, 1e30, 1e30), hi(-1e30, -1e30, -1e30);
        for (const Vertex& v : c.mesh.vertices) {
            lo = Vec3(std::min(lo.x, v.position.x), std::min(lo.y, v.position.y), std::min(lo.z, v.position.z));
            hi = Vec3(std::max(hi.x, v.position.x), std::max(hi.y, v.position.y), std::max(hi.z, v.position.z));
        }
        c.centre = (lo + hi) * 0.5;
        c.radius = (hi - lo).length() * 0.5;
    }
    return out;
}

}  // namespace engine
