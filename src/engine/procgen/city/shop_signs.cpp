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

std::string mallName(uint32_t seed) {
    uint32_t h = seed * 2654435761u + 0x3a11u;
    h ^= h << 13; h ^= h >> 17; h ^= h << 5;
    const char* w = (h % 3u) == 0 ? pick(kHawaiian, h) : (h % 3u) == 1 ? pick(kEnglish, h) : pick(kJapanese, h);
    switch ((h >> 8) % 4u) {
        case 0: return std::string(w) + " Galleria";
        case 1: return std::string(w) + " Center";
        case 2: return std::string("The Shops at ") + w;
        default: return std::string(w) + " Plaza";
    }
}

std::string skyName(uint32_t seed) {
    uint32_t h = seed * 40503u + 0x51a7u;
    h ^= h << 13; h ^= h >> 17; h ^= h << 5;
    const char* w = (h % 2u) ? pick(kJapanese, h) : pick(kHawaiian, h);
    switch ((h >> 8) % 3u) {
        case 0: return std::string(w) + " Sky Dining";
        case 1: return std::string("The ") + w + " Terrace";
        default: return std::string(w) + " Top of the Tower";
    }
}

std::string anchorName(uint32_t seed) {
    uint32_t h = seed * 2246822519u + 0x5a1du;
    h ^= h << 13; h ^= h >> 17; h ^= h << 5;
    static const char* kSurname[] = {"Hollister", "Whitfield", "Ashby", "Kaneshiro", "Maeda", "Halloran", "Pemberton",
                                     "Okamoto", "Delacroix", "Brandt", "Kealoha", "Sutherland"};
    const char* n = kSurname[h % 12u];
    switch ((h >> 8) % 3u) {
        case 0: return std::string(n) + "'s";
        case 1: return std::string(n) + " & Co.";
        default: return std::string(n);
    }
}

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

namespace {

// A glyph mask's soft HALO: the coverage box-blurred `r` px, three passes each way (near enough a gaussian).
std::vector<float> halo(const std::vector<float>& mask, int w, int h, int r) {
    std::vector<float> a = mask, b(mask.size());
    for (int pass = 0; pass < 3; ++pass) {
        for (int y = 0; y < h; ++y) {   // across
            float sum = 0;
            for (int x = -r; x <= r; ++x) sum += a[static_cast<std::size_t>(y) * w + std::clamp(x, 0, w - 1)];
            for (int x = 0; x < w; ++x) {
                b[static_cast<std::size_t>(y) * w + x] = sum / (2 * r + 1);
                sum += a[static_cast<std::size_t>(y) * w + std::min(x + r + 1, w - 1)] - a[static_cast<std::size_t>(y) * w + std::max(x - r, 0)];
            }
        }
        for (int x = 0; x < w; ++x) {   // down
            float sum = 0;
            for (int y = -r; y <= r; ++y) sum += b[static_cast<std::size_t>(std::clamp(y, 0, h - 1)) * w + x];
            for (int y = 0; y < h; ++y) {
                a[static_cast<std::size_t>(y) * w + x] = sum / (2 * r + 1);
                sum += b[static_cast<std::size_t>(std::min(y + r + 1, h - 1)) * w + x] - b[static_cast<std::size_t>(std::max(y - r, 0)) * w + x];
            }
        }
    }
    return a;
}

uint8_t u8(float v) { return static_cast<uint8_t>(std::clamp(v, 0.0f, 1.0f) * 255.0f + 0.5f); }

// Tube light from a coverage mask: by day (`day`) pale glass tinted its colour on the plate; by night (`night`) a hot
// core near white and the colour's halo round it. Both images already sized and filled with the plate.
void lightTubes(const std::vector<float>& mask, int w, int h, Vec3 c, int haloPx, TextImage& day, TextImage& night,
                float haloGain = 1.5f) {
    const std::vector<float> hl = halo(mask, w, h, haloPx);
    for (int i = 0; i < w * h; ++i) {
        const float m = mask[static_cast<std::size_t>(i)], g = std::min(1.0f, hl[static_cast<std::size_t>(i)] * haloGain);
        uint8_t* d = &day.rgba[static_cast<std::size_t>(i) * 4];
        const float tube[3] = {0.45f * static_cast<float>(c.x) + 0.42f, 0.45f * static_cast<float>(c.y) + 0.42f,
                               0.45f * static_cast<float>(c.z) + 0.42f};
        for (int k = 0; k < 3; ++k) d[k] = u8((d[k] / 255.0f) * (1 - m) + tube[k] * m);
        uint8_t* n = &night.rgba[static_cast<std::size_t>(i) * 4];
        const float col[3] = {static_cast<float>(c.x), static_cast<float>(c.y), static_cast<float>(c.z)};
        for (int k = 0; k < 3; ++k) {
            const float core = col[k] * 0.55f + 0.45f;   // the tube itself burns near white
            n[k] = u8(std::max(core * m, col[k] * g * 0.85f));
        }
    }
}

}  // namespace

Vec3 neonColour(uint8_t trade, uint32_t seed) {
    uint32_t h = seed * 0x85EBCA6Bu + 0x1b873593u;
    h ^= h >> 15; h *= 0x2C1B3C6Du; h ^= h >> 12;
    static const Vec3 kBar[] = {Vec3(1.0, 0.18, 0.55), Vec3(1.0, 0.16, 0.10), Vec3(1.0, 0.62, 0.15), Vec3(0.25, 0.85, 1.0),
                                Vec3(1.0, 0.38, 0.72), Vec3(0.35, 1.0, 0.45)};
    static const Vec3 kClub[] = {Vec3(0.62, 0.28, 1.0), Vec3(0.2, 0.9, 1.0), Vec3(1.0, 0.15, 0.62), Vec3(0.3, 0.45, 1.0)};
    return trade == 15 ? kClub[h % 4u] : kBar[h % 6u];
}

ShopSignAtlas buildShopSignAtlas(const Font& font, const std::vector<ShopSign>& signs, int pagePx, const Font* tube) {
    ShopSignAtlas atlas;
    constexpr int H = 72;   // a board's pixels tall
    const uint8_t board[4] = {7, 7, 9, 255};   // near black: by night (emissive = the page) only the lettering glows
    const uint8_t plate[4] = {20, 18, 22, 255};   // a neon sign's dark plate (by day: the tubes read on it)
    const uint8_t black[4] = {0, 0, 0, 255};
    struct Made { int index; int w; TextImage img; TextImage glow; };
    std::vector<Made> made;
    bool anyNeon = false;
    atlas.boards.resize(signs.size());
    atlas.smallestCapPx = 1e9f;
    for (std::size_t i = 0; i < signs.size(); ++i) {
        const ShopSign& s = signs[i];
        // the board is as wide as its name and a margin -- the fascia's full width only when the name needs it (a
        // name centred on its own nameplate, and atlas pages that hold dozens, not twenty)
        const int fasciaPx = std::clamp(static_cast<int>(std::lround(H * s.width / std::max(Real(0.05), s.height))), H, pagePx - 16);
        if (s.style != 0) {
            // NEON: the name in tube, set a little smaller than a board's (a script's loops and tails need the
            // room, and the halo a margin), lit as glass by day and as light by night
            anyNeon = true;
            const Font& f = (s.style == 2 && tube) ? *tube : font;
            // as big as the board holds: a script's ascent-to-descent fills 94% of it, tube capitals 70%
            float em = s.style == 1 ? H * 0.94f / std::max(0.5f, f.ascent(1.0f) + f.descent(1.0f)) : H * 0.70f / std::max(0.3f, f.capHeight(1.0f)),
                  xs = 1.0f;
            const float budget = static_cast<float>(fasciaPx) - H * 0.6f;
            float tw = f.measure(s.text, em, xs);
            if (tw > budget) { em *= budget / tw; tw = f.measure(s.text, em, xs); }
            const int w = std::min(fasciaPx, static_cast<int>(std::ceil(tw + H * 0.8f)));
            TextImage m;
            const uint8_t white[4] = {255, 255, 255, 255};
            m.resize(w, H, black);
            const float cap = f.capHeight(em);
            atlas.smallestCapPx = std::min(atlas.smallestCapPx, cap);
            // a script sits on its x-height more than its capitals: centre the ascent-to-descent span instead
            const float base = s.style == 1 ? 0.5f * (H + f.ascent(em) - f.descent(em)) : 0.5f * (H + cap);
            f.draw(m, s.text, 0.5f * (w - tw), base, em, white, xs);
            std::vector<float> mask(static_cast<std::size_t>(w) * H);
            for (std::size_t k = 0; k < mask.size(); ++k) mask[k] = m.rgba[k * 4] / 255.0f;
            TextImage img, glow;
            img.resize(w, H, plate);
            glow.resize(w, H, black);
            lightTubes(mask, w, H, s.neon, 4, img, glow);
            made.push_back({static_cast<int>(i), w, std::move(img), std::move(glow)});
            continue;
        }
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
        made.push_back({static_cast<int>(i), w, std::move(img), TextImage{}});
    }
    std::stable_sort(made.begin(), made.end(), [](const Made& a, const Made& b) { return a.w > b.w; });
    const int pad = 4, rowH = H + 2 * pad;
    int x = 0, y = 0;
    std::vector<int> usedH;
    auto newPage = [&]() {
        TextImage page;
        page.resize(pagePx, pagePx, board);
        atlas.pages.push_back(std::move(page));
        if (anyNeon) {
            TextImage g;
            g.resize(pagePx, pagePx, black);
            atlas.glow.push_back(std::move(g));
        }
        usedH.push_back(0);
        x = 0; y = 0;
    };
    // a sign's pixels into a page, edge-extended so the mips do not bleed a neighbour in
    auto blit = [&](const TextImage& src, TextImage& page, int ox, int oy, int w) {
        for (int yy = -pad; yy < H + pad; ++yy)
            for (int xx = -pad; xx < w + pad; ++xx) {
                const int sx = std::clamp(xx, 0, w - 1), sy = std::clamp(yy, 0, H - 1);
                const int dx = ox + xx, dy = oy + yy;
                if (dx < 0 || dy < 0 || dx >= page.w || dy >= page.h) continue;
                const uint8_t* s = &src.rgba[(static_cast<std::size_t>(sy) * w + sx) * 4];
                uint8_t* d = &page.rgba[(static_cast<std::size_t>(dy) * page.w + dx) * 4];
                for (int k = 0; k < 4; ++k) d[k] = s[k];
            }
    };
    for (Made& m : made) {
        const int cw = m.w + 2 * pad;
        if (atlas.pages.empty()) newPage();
        if (x + cw > pagePx) { x = 0; y += rowH; }
        if (y + rowH > pagePx) newPage();
        const int ox = x + pad, oy = y + pad;
        blit(m.img, atlas.pages.back(), ox, oy, m.w);
        if (anyNeon) blit(m.glow.rgba.empty() ? m.img : m.glow, atlas.glow.back(), ox, oy, m.w);
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
        const int h = std::max(1, usedH[pi]);
        for (std::vector<TextImage>* set : {&atlas.pages, &atlas.glow}) {
            if (pi >= set->size()) continue;
            TextImage& page = (*set)[pi];
            page.rgba.resize(static_cast<std::size_t>(page.w) * h * 4);
            page.h = h;
        }
    }
    for (ShopSignAtlas::Board& b : atlas.boards) {
        if (atlas.pages.empty()) break;
        const float h = static_cast<float>(atlas.pages[static_cast<std::size_t>(b.page)].h);
        b.v0 /= h; b.v1 /= h;
    }
    if (signs.empty()) atlas.smallestCapPx = 0;
    return atlas;
}

void openSignImages(const Font& tube, TextImage& albedo, TextImage& glow) {
    constexpr int W = 256, H = 112;
    const uint8_t plate[4] = {16, 15, 19, 255}, black[4] = {0, 0, 0, 255}, white[4] = {255, 255, 255, 255};
    // the border: a rounded tube 6 px in from the plate's edge
    std::vector<float> border(static_cast<std::size_t>(W) * H, 0.0f), letters(border.size(), 0.0f);
    const float in = 10, r = 22, t = 3.2f;
    for (int y = 0; y < H; ++y)
        for (int x = 0; x < W; ++x) {
            // distance to a rounded rectangle's outline
            const float qx = std::max(std::abs(x + 0.5f - W * 0.5f) - (W * 0.5f - in - r), 0.0f);
            const float qy = std::max(std::abs(y + 0.5f - H * 0.5f) - (H * 0.5f - in - r), 0.0f);
            const float d = std::abs(std::sqrt(qx * qx + qy * qy) - r);
            border[static_cast<std::size_t>(y) * W + x] = std::clamp(t - d + 0.5f, 0.0f, 1.0f);
        }
    TextImage m;
    m.resize(W, H, black);
    const float em = 64.0f, tw = tube.measure("OPEN", em);
    tube.draw(m, "OPEN", 0.5f * (W - tw), 0.5f * (H + tube.capHeight(em)), em, white);
    for (std::size_t k = 0; k < letters.size(); ++k) letters[k] = m.rgba[k * 4] / 255.0f;
    albedo.resize(W, H, plate);
    glow.resize(W, H, black);
    lightTubes(border, W, H, Vec3(0.2, 0.45, 1.0), 5, albedo, glow, 2.0f);
    TextImage a2 = albedo, g2;
    g2.resize(W, H, black);
    lightTubes(letters, W, H, Vec3(1.0, 0.12, 0.08), 5, a2, g2, 1.6f);
    for (std::size_t k = 0; k < glow.rgba.size(); ++k) glow.rgba[k] = std::max(glow.rgba[k], g2.rgba[k]);
    albedo = std::move(a2);
    // the plate's corners rounded off with the tube (alpha-cut), a few pixels outside it
    for (int y = 0; y < H; ++y)
        for (int x = 0; x < W; ++x) {
            const float qx = std::max(std::abs(x + 0.5f - W * 0.5f) - (W * 0.5f - in - r), 0.0f);
            const float qy = std::max(std::abs(y + 0.5f - H * 0.5f) - (H * 0.5f - in - r), 0.0f);
            if (std::sqrt(qx * qx + qy * qy) > r + t + 5.0f) albedo.rgba[(static_cast<std::size_t>(y) * W + x) * 4 + 3] = 0;
        }
}

RenderMesh openSignMesh() {
    RenderMesh m;
    const Real hw = kOpenSignW * 0.5, hh = kOpenSignH * 0.5;
    const Vec3 n(0, 0, 1);
    auto vtx = [&](Real x, Real y, float u, float v) {
        Vertex vx(Vec3(x, y, 0), n, u, v);
        vx.tangent = Vec3(1, 0, 0);
        m.vertices.push_back(vx);
    };
    vtx(-hw, hh, 0, 0); vtx(hw, hh, 1, 0); vtx(hw, -hh, 1, 1); vtx(-hw, -hh, 0, 1);
    for (uint32_t k : {0u, 2u, 1u, 0u, 3u, 2u}) m.indices.push_back(k);
    return m;
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
        // THE WHOLE FASCIA is the board (the building's own lit fascia band, a pastel lightbox at night, would
        // otherwise frame the name); the name's own image in the middle, and either side plain board -- its edge
        // column of pixels stretched, so the atlas holds only the name's width
        const Real hwAll = s.width * 0.5, hwName = s.width * std::min(1.0f, b.widthFrac) * 0.5, hh = s.height * 0.5;
        auto vtx = [&](Vec3 pos, float u, float v) {
            Vertex vx(pos, s.n, u, v);
            vx.tangent = s.right;
            m.vertices.push_back(vx);
        };
        auto quad = [&](Real x0, Real x1, float u0, float u1) {
            if (x1 - x0 < 1e-3) return;
            const uint32_t base = static_cast<uint32_t>(m.vertices.size());
            vtx(s.centre + s.right * x0 + s.up * hh, u0, b.v0);   // TL
            vtx(s.centre + s.right * x1 + s.up * hh, u1, b.v0);   // TR
            vtx(s.centre + s.right * x1 - s.up * hh, u1, b.v1);   // BR
            vtx(s.centre + s.right * x0 - s.up * hh, u0, b.v1);   // BL
            for (uint32_t k : {0u, 2u, 1u, 0u, 3u, 2u}) m.indices.push_back(base + k);
        };
        const float px = 0.5f / 2048.0f;   // half a texel in: the board's edge column, never its neighbour's
        quad(-hwAll, -hwName, b.u0 + px, b.u0 + px);
        quad(-hwName, hwName, b.u0, b.u1);
        quad(hwName, hwAll, b.u1 - px, b.u1 - px);
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
