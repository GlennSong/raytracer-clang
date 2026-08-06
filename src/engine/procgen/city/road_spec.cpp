#include "road_spec.h"

#include <algorithm>

namespace engine {

namespace {

bool drivable(BandKind k) {
    return k == BandKind::Travel || k == BandKind::Turn ||
           k == BandKind::Parking || k == BandKind::Shoulder;
}

RoadBand band(BandKind k, float w, int8_t dir = 0,
              BandSurface s = BandSurface::Asphalt, uint8_t paint = kPaintNone) {
    RoadBand b;
    b.kind = k; b.width = w; b.dir = dir; b.surface = s; b.paint = paint;
    return b;
}

const char* kindName(BandKind k) {
    switch (k) {
        case BandKind::Travel: return "travel";
        case BandKind::Turn: return "turn";
        case BandKind::Parking: return "parking";
        case BandKind::Shoulder: return "shoulder";
        case BandKind::Median: return "median";
        case BandKind::Bike: return "bike";
        case BandKind::Curb: return "curb";
        case BandKind::Sidewalk: return "sidewalk";
        case BandKind::Barrier: return "barrier";
    }
    return "travel";
}
BandKind kindFromName(const std::string& n) {
    if (n == "turn") return BandKind::Turn;
    if (n == "parking") return BandKind::Parking;
    if (n == "shoulder") return BandKind::Shoulder;
    if (n == "median") return BandKind::Median;
    if (n == "bike") return BandKind::Bike;
    if (n == "curb") return BandKind::Curb;
    if (n == "sidewalk") return BandKind::Sidewalk;
    if (n == "barrier") return BandKind::Barrier;
    return BandKind::Travel;
}
const char* surfaceName(BandSurface s) {
    switch (s) {
        case BandSurface::Asphalt: return "asphalt";
        case BandSurface::Concrete: return "concrete";
        case BandSurface::Dirt: return "dirt";
        case BandSurface::Gravel: return "gravel";
    }
    return "asphalt";
}
BandSurface surfaceFromName(const std::string& n) {
    if (n == "concrete") return BandSurface::Concrete;
    if (n == "dirt") return BandSurface::Dirt;
    if (n == "gravel") return BandSurface::Gravel;
    return BandSurface::Asphalt;
}

}  // namespace

double RoadSpec::totalWidth() const {
    double w = 0;
    for (const RoadBand& b : bands) w += b.width;
    return w;
}

double RoadSpec::carriagewayWidth() const {
    double w = 0;
    for (const RoadBand& b : bands)
        if (drivable(b.kind)) w += b.width;
    return w;
}

int RoadSpec::laneCount() const {
    int n = 0;
    for (const RoadBand& b : bands)
        if (b.kind == BandKind::Travel || b.kind == BandKind::Turn) ++n;
    return n;
}

bool RoadSpec::isOneWay() const {
    int8_t seen = 0;
    for (const RoadBand& b : bands) {
        if (b.dir == 0) continue;
        if (seen == 0) seen = b.dir;
        else if (b.dir != seen) return false;
    }
    return seen != 0;
}

bool RoadSpec::hasSidewalk(int side) const {
    if (bands.empty()) return false;
    // scan inward from the requested end past barriers/curbs.
    if (side < 0) {
        for (const RoadBand& b : bands) {
            if (b.kind == BandKind::Sidewalk) return true;
            if (drivable(b.kind) || b.kind == BandKind::Median) return false;
        }
    } else {
        for (auto it = bands.rbegin(); it != bands.rend(); ++it) {
            if (it->kind == BandKind::Sidewalk) return true;
            if (drivable(it->kind) || it->kind == BandKind::Median) return false;
        }
    }
    return false;
}

void RoadSpec::bandSpan(int i, double& x0, double& x1) const {
    double x = 0;
    for (int k = 0; k < static_cast<int>(bands.size()); ++k) {
        if (k == i) { x0 = x; x1 = x + bands[k].width; return; }
        x += bands[k].width;
    }
    x0 = x1 = x;
}

RoadSpec roadSpecPreset(const std::string& name) {
    RoadSpec s;
    s.name = name;
    const auto SW = [&](float w) { return band(BandKind::Sidewalk, w, 0, BandSurface::Concrete); };
    const auto CB = [&]() { return band(BandKind::Curb, 0.25f, 0, BandSurface::Concrete); };
    if (name == "local") {
        s.bands = { SW(3.0f), CB(), band(BandKind::Travel, 3.5f, -1),
                    band(BandKind::Travel, 3.5f, +1), CB(), SW(3.0f) };
    } else if (name == "arterial") {
        s.bands = { SW(3.5f), CB(),
                    band(BandKind::Travel, 3.5f, -1), band(BandKind::Travel, 3.5f, -1),
                    band(BandKind::Median, 0.6f),
                    band(BandKind::Travel, 3.5f, +1), band(BandKind::Travel, 3.5f, +1),
                    CB(), SW(3.5f) };
    } else if (name == "freeway3") {
        s.designSpeed = 30.0f;
        s.bands = { band(BandKind::Barrier, 0.5f),
                    band(BandKind::Shoulder, 2.0f, -1),
                    band(BandKind::Travel, 3.6f, -1), band(BandKind::Travel, 3.6f, -1),
                    band(BandKind::Travel, 3.6f, -1),
                    band(BandKind::Median, 1.4f),
                    band(BandKind::Travel, 3.6f, +1), band(BandKind::Travel, 3.6f, +1),
                    band(BandKind::Travel, 3.6f, +1),
                    band(BandKind::Shoulder, 2.0f, +1),
                    band(BandKind::Barrier, 0.5f) };
    } else if (name == "ramp1") {
        s.designSpeed = 12.0f;
        s.bands = { band(BandKind::Barrier, 0.4f),
                    band(BandKind::Shoulder, 1.2f, +1),
                    band(BandKind::Travel, 4.0f, +1),
                    band(BandKind::Shoulder, 1.2f, +1),
                    band(BandKind::Barrier, 0.4f) };
    } else if (name == "dirt") {
        s.bands = { band(BandKind::Shoulder, 1.2f, -1, BandSurface::Dirt),
                    band(BandKind::Travel, 3.0f, -1, BandSurface::Dirt),
                    band(BandKind::Travel, 3.0f, +1, BandSurface::Dirt),
                    band(BandKind::Shoulder, 1.2f, +1, BandSurface::Dirt) };
    } else if (name == "alley") {
        s.bands = { band(BandKind::Travel, 3.0f, +1) };
    } else {
        // unknown -> a sane two-lane street, but keep the requested name so a
        // future preset can claim it without silently changing meaning.
        s.bands = { band(BandKind::Travel, 3.5f, -1), band(BandKind::Travel, 3.5f, +1) };
    }
    return s;
}

RoadSpec roadSpecFromLegacy(double width, bool oneWay, double sidewalkWidth,
                            double curbHeight) {
    RoadSpec s;
    s.name = "";                        // custom: synthesized, not a preset
    const bool curbs = curbHeight > 0.0 && sidewalkWidth > 0.0;
    if (curbs) {
        s.bands.push_back(band(BandKind::Sidewalk, static_cast<float>(sidewalkWidth),
                               0, BandSurface::Concrete));
        s.bands.push_back(band(BandKind::Curb, 0.25f, 0, BandSurface::Concrete));
    }
    // Split the legacy carriageway into lanes of ~3.5 m (at least one per
    // direction), preserving its EXACT total width.
    const int dirs = oneWay ? 1 : 2;
    const int lanes = std::max(dirs, static_cast<int>(width / 3.5 + 0.5));
    const float laneW = static_cast<float>(width / lanes);
    for (int i = 0; i < lanes; ++i) {
        const int8_t d = oneWay ? +1 : (i < lanes / 2 ? -1 : +1);
        s.bands.push_back(band(BandKind::Travel, laneW, d));
    }
    if (curbs) {
        s.bands.push_back(band(BandKind::Curb, 0.25f, 0, BandSurface::Concrete));
        s.bands.push_back(band(BandKind::Sidewalk, static_cast<float>(sidewalkWidth),
                               0, BandSurface::Concrete));
    }
    return s;
}

RoadSpec roadSpecStreetParking(double carriageway, int lanesPerDir,
                               double sidewalkWidth, double curbWidth,
                               double parkWidth, double minLane) {
    const int n = std::max(1, lanesPerDir);
    const double park = std::max(0.0, parkWidth);
    // Too narrow to carry parking AND drivable lanes: NO Parking band, and the
    // section falls back to the legacy split (same lanes as before) rather than
    // inventing absurd 5 m ones. Better a road you honestly cannot park on.
    if (park <= 0.0 || carriageway - 2.0 * park < 2.0 * n * minLane)
        return roadSpecFromLegacy(carriageway, false, sidewalkWidth,
                                  /*curbHeight=*/curbWidth > 0.0 ? 1.0 : 0.0);
    // The lanes absorb whatever is left, so carriagewayWidth() == carriageway to
    // the last millimetre (the width-agreement contract; see the header).
    const double laneW = (carriageway - 2.0 * park) / (2.0 * n);
    RoadSpec s;
    s.name = "";                       // custom: synthesized, not a preset
    const bool curbs = curbWidth > 0.0 && sidewalkWidth > 0.0;
    if (curbs) {
        s.bands.push_back(band(BandKind::Sidewalk, static_cast<float>(sidewalkWidth),
                               0, BandSurface::Concrete));
        s.bands.push_back(band(BandKind::Curb, static_cast<float>(curbWidth), 0,
                               BandSurface::Concrete));
    }
    s.bands.push_back(band(BandKind::Parking, static_cast<float>(park), -1));
    for (int i = 0; i < n; ++i)
        s.bands.push_back(band(BandKind::Travel, static_cast<float>(laneW), -1));
    for (int i = 0; i < n; ++i)
        s.bands.push_back(band(BandKind::Travel, static_cast<float>(laneW), +1));
    s.bands.push_back(band(BandKind::Parking, static_cast<float>(park), +1));
    if (curbs) {
        s.bands.push_back(band(BandKind::Curb, static_cast<float>(curbWidth), 0,
                               BandSurface::Concrete));
        s.bands.push_back(band(BandKind::Sidewalk, static_cast<float>(sidewalkWidth),
                               0, BandSurface::Concrete));
    }
    return s;
}

bool roadSpecParkingBand(const RoadSpec& spec, int side, double& centerOffset,
                         double& width) {
    // The carriageway's own span: the mesher sweeps the road centred on THAT,
    // not on the full section (a one-sided sidewalk would otherwise bias every
    // offset). c0 = left edge of the first drivable band, c1 = right edge of the
    // last one.
    double x = 0, c0 = -1, c1 = -1;
    for (const RoadBand& b : spec.bands) {
        if (drivable(b.kind)) {
            if (c0 < 0) c0 = x;
            c1 = x + b.width;
        }
        x += b.width;
    }
    if (c0 < 0) return false;
    const double mid = (c0 + c1) * 0.5;
    const int n = static_cast<int>(spec.bands.size());
    for (int k = 0; k < n; ++k) {
        const int i = side < 0 ? k : n - 1 - k;     // scan inward from that end
        if (spec.bands[i].kind != BandKind::Parking) continue;
        double x0 = 0, x1 = 0;
        spec.bandSpan(i, x0, x1);
        const double c = (x0 + x1) * 0.5;
        if (side < 0 ? c > mid : c < mid) break;    // crossed to the other side
        centerOffset = c - mid;
        width = x1 - x0;
        return true;
    }
    return false;
}

RoadSpec roadSpecFromJson(const nlohmann::json& j) {
    if (j.is_string()) return roadSpecPreset(j.get<std::string>());
    RoadSpec s;
    s.name = j.value("preset", std::string());
    if (!s.name.empty() && !j.contains("bands")) return roadSpecPreset(s.name);
    s.designSpeed = j.value("design_speed", 0.0f);
    if (j.contains("bands") && j["bands"].is_array()) {
        for (const auto& jb : j["bands"]) {
            RoadBand b;
            b.kind = kindFromName(jb.value("kind", std::string("travel")));
            b.width = jb.value("width", 3.6f);
            b.dir = static_cast<int8_t>(jb.value("dir", 0));
            b.surface = surfaceFromName(jb.value("surface", std::string("asphalt")));
            b.paint = static_cast<uint8_t>(jb.value("paint", 0));
            s.bands.push_back(b);
        }
    }
    return s;
}

nlohmann::json roadSpecToJson(const RoadSpec& spec) {
    // A pristine preset serializes as its name (compact, upgrade-friendly).
    if (!spec.name.empty()) {
        const RoadSpec p = roadSpecPreset(spec.name);
        if (p.bands.size() == spec.bands.size()) {
            bool same = p.designSpeed == spec.designSpeed;
            for (std::size_t i = 0; same && i < p.bands.size(); ++i) {
                const RoadBand &a = p.bands[i], &b = spec.bands[i];
                same = a.kind == b.kind && a.width == b.width && a.dir == b.dir &&
                       a.surface == b.surface && a.paint == b.paint;
            }
            if (same) return spec.name;
        }
    }
    nlohmann::json j;
    if (!spec.name.empty()) j["preset"] = spec.name;
    if (spec.designSpeed > 0) j["design_speed"] = spec.designSpeed;
    nlohmann::json bands = nlohmann::json::array();
    for (const RoadBand& b : spec.bands) {
        nlohmann::json jb;
        jb["kind"] = kindName(b.kind);
        jb["width"] = b.width;
        if (b.dir) jb["dir"] = static_cast<int>(b.dir);
        if (b.surface != BandSurface::Asphalt) jb["surface"] = surfaceName(b.surface);
        if (b.paint) jb["paint"] = static_cast<int>(b.paint);
        bands.push_back(jb);
    }
    j["bands"] = std::move(bands);
    return j;
}

}  // namespace engine
