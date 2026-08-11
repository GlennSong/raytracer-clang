#include "profile.h"

#include <cstring>
#include <map>

namespace roadlab {

// --- markings -------------------------------------------------------------

bool Marking::crossableFromLeft() const {
    switch (style) {
        case MarkStyle::None:
        case MarkStyle::Dashed:
        case MarkStyle::WideDashed:
        case MarkStyle::Botts:
            return true;
        case MarkStyle::DashedSolid:   // dashed on the left of the pair
            return true;
        default:
            return false;
    }
}

bool Marking::crossableFromRight() const {
    switch (style) {
        case MarkStyle::None:
        case MarkStyle::Dashed:
        case MarkStyle::WideDashed:
        case MarkStyle::Botts:
            return true;
        case MarkStyle::SolidDashed:   // dashed on the right of the pair
            return true;
        default:
            return false;
    }
}

Marking markSolid(PaintColor c, float w) {
    Marking m;
    m.style = MarkStyle::Solid;
    m.color = c;
    m.width = w;
    return m;
}

Marking markDashed(PaintColor c, float on, float off) {
    Marking m;
    m.style = MarkStyle::Dashed;
    m.color = c;
    m.dashOn = on;
    m.dashOff = off;
    return m;
}

Marking markDouble(PaintColor c) {
    Marking m;
    m.style = MarkStyle::Double;
    m.color = c;
    m.width = 0.12f;
    m.gap = 0.12f;
    return m;
}

Marking markEdge() { return markSolid(PaintColor::White, 0.15f); }
Marking markNone() { return Marking{}; }

// --- strips ---------------------------------------------------------------

bool Strip::drivable() const {
    switch (kind) {
        case StripKind::Travel:
        case StripKind::Turn:
        case StripKind::Bus:
        case StripKind::Parking:
        case StripKind::Shoulder:
        case StripKind::Apron:
            return true;
        default:
            return false;
    }
}

bool Strip::isLane() const {
    return kind == StripKind::Travel || kind == StripKind::Turn || kind == StripKind::Bus;
}

// --- lane sections --------------------------------------------------------

namespace {

struct Span {
    const Strip* strip = nullptr;
    double t0 = 0, t1 = 0;   // t0 < t1
    int id = 0;
};

// Ordered from the most negative t to the most positive: the right stack
// reversed (outermost first) then the left stack outward.
void buildSpans(const LaneSection& sec, double ds, std::vector<Span>& out) {
    out.clear();
    out.reserve(sec.left.size() + sec.right.size());
    double acc = 0.0;
    static thread_local std::vector<Span> rightSpans;
    rightSpans.clear();
    rightSpans.reserve(sec.right.size());
    for (const Strip& s : sec.right) {
        double w = std::max(0.0, s.width.eval(ds));
        rightSpans.push_back({&s, -(acc + w), -acc, s.id});
        acc += w;
    }
    for (size_t i = rightSpans.size(); i-- > 0;) out.push_back(rightSpans[i]);
    acc = 0.0;
    for (const Strip& s : sec.left) {
        double w = std::max(0.0, s.width.eval(ds));
        out.push_back({&s, acc, acc + w, s.id});
        acc += w;
    }
}

}  // namespace

int LaneSection::laneCount() const {
    int n = 0;
    for (const Strip& s : left)
        if (s.isLane()) ++n;
    for (const Strip& s : right)
        if (s.isLane()) ++n;
    return n;
}

double LaneSection::totalWidthAt(double s) const {
    double ds = s - s0, w = 0;
    for (const Strip& st : left) w += std::max(0.0, st.width.eval(ds));
    for (const Strip& st : right) w += std::max(0.0, st.width.eval(ds));
    return w;
}

double LaneSection::carriagewayHalfWidthAt(double s, int side) const {
    // Out to the last drivable strip: the width junction trimming and terrain
    // cut/fill care about, which is not the same as the full right-of-way.
    const std::vector<Strip>& stack = side >= 0 ? left : right;
    double ds = s - s0, acc = 0, best = 0;
    for (const Strip& st : stack) {
        double w = std::max(0.0, st.width.eval(ds));
        acc += w;
        if (st.drivable() || st.kind == StripKind::Curb || st.kind == StripKind::Gutter) best = acc;
    }
    return best;
}

const Strip* LaneSection::strip(int id) const {
    if (id > 0 && size_t(id) <= left.size()) return &left[size_t(id) - 1];
    if (id < 0 && size_t(-id) <= right.size()) return &right[size_t(-id) - 1];
    return nullptr;
}

Strip* LaneSection::strip(int id) {
    return const_cast<Strip*>(static_cast<const LaneSection*>(this)->strip(id));
}

void LaneSection::assignIds() {
    for (size_t i = 0; i < left.size(); ++i) left[i].id = int(i) + 1;
    for (size_t i = 0; i < right.size(); ++i) right[i].id = -(int(i) + 1);
}

// --- cross-section --------------------------------------------------------

void CrossSection::finalize(double roadLength) {
    if (sections.empty()) return;
    std::sort(sections.begin(), sections.end(),
              [](const LaneSection& a, const LaneSection& b) { return a.s0 < b.s0; });
    sections.front().s0 = std::min(sections.front().s0, 0.0);
    for (size_t i = 0; i < sections.size(); ++i) {
        double end = (i + 1 < sections.size()) ? sections[i + 1].s0 : roadLength;
        sections[i].length = std::max(0.0, end - sections[i].s0);
        sections[i].assignIds();
    }
    if (laneOffset.empty()) laneOffset.set(0.0);
    linkSections();
}

int CrossSection::sectionIndexAt(double s) const {
    if (sections.empty()) return -1;
    for (size_t i = sections.size(); i-- > 0;) {
        if (s >= sections[i].s0 - 1e-9) return int(i);
    }
    return 0;
}

const LaneSection& CrossSection::sectionAt(double s) const {
    int i = sectionIndexAt(s);
    return sections[size_t(i < 0 ? 0 : i)];
}

void CrossSection::boundariesAt(double s, std::vector<Boundary>& out) const {
    out.clear();
    if (sections.empty()) return;
    const LaneSection& sec = sectionAt(s);
    double ds = s - sec.s0;
    double off = laneOffset.eval(s);
    // Hot path: the shader calls this per fragment, so the scratch buffer is
    // kept alive rather than reallocated a million times a frame.
    static thread_local std::vector<Span> spans;
    buildSpans(sec, ds, spans);
    if (spans.empty()) return;

    // Outer boundary of every right strip, walking inward toward t = 0.
    for (size_t i = 0; i < spans.size(); ++i) {
        const Span& sp = spans[i];
        if (sp.id >= 0) break;
        Boundary b;
        b.t = sp.t0 + off;
        b.mark = sp.strip->outerMark;
        b.innerKind = sp.strip->kind;
        b.outerKind = i > 0 ? spans[i - 1].strip->kind : sp.strip->kind;
        out.push_back(b);
    }
    // The reference line itself.
    {
        Boundary b;
        b.t = off;
        b.mark = sec.centerMark;
        b.innerKind = sec.right.empty() ? StripKind::Travel : sec.right.front().kind;
        b.outerKind = sec.left.empty() ? StripKind::Travel : sec.left.front().kind;
        out.push_back(b);
    }
    for (size_t i = 0; i < spans.size(); ++i) {
        const Span& sp = spans[i];
        if (sp.id <= 0) continue;
        Boundary b;
        b.t = sp.t1 + off;
        b.mark = sp.strip->outerMark;
        b.innerKind = sp.strip->kind;
        b.outerKind = (i + 1 < spans.size()) ? spans[i + 1].strip->kind : sp.strip->kind;
        out.push_back(b);
    }
    std::sort(out.begin(), out.end(),
              [](const Boundary& a, const Boundary& b) { return a.t < b.t; });
}

bool CrossSection::laneSpanAt(int sectionIdx, int laneId, double s, double& tInner,
                              double& tOuter) const {
    if (sectionIdx < 0 || size_t(sectionIdx) >= sections.size() || laneId == 0) return false;
    const LaneSection& sec = sections[size_t(sectionIdx)];
    double ds = s - sec.s0;
    double off = laneOffset.eval(s);
    const std::vector<Strip>& stack = laneId > 0 ? sec.left : sec.right;
    size_t idx = size_t(laneId > 0 ? laneId - 1 : -laneId - 1);
    if (idx >= stack.size()) return false;
    double acc = 0;
    for (size_t i = 0; i < idx; ++i) acc += std::max(0.0, stack[i].width.eval(ds));
    double w = std::max(0.0, stack[idx].width.eval(ds));
    double sign = laneId > 0 ? 1.0 : -1.0;
    tInner = sign * acc + off;
    tOuter = sign * (acc + w) + off;
    return true;
}

double CrossSection::laneCenterT(int sectionIdx, int laneId, double s) const {
    double a = 0, b = 0;
    if (!laneSpanAt(sectionIdx, laneId, s, a, b)) return laneOffset.eval(s);
    return 0.5 * (a + b);
}

double CrossSection::laneWidthAt(int sectionIdx, int laneId, double s) const {
    double a = 0, b = 0;
    if (!laneSpanAt(sectionIdx, laneId, s, a, b)) return 0.0;
    return std::fabs(b - a);
}

double CrossSection::totalWidthAt(double s) const {
    return sections.empty() ? 0.0 : sectionAt(s).totalWidthAt(s);
}

double CrossSection::leftExtentAt(double s) const {
    if (sections.empty()) return 0.0;
    const LaneSection& sec = sectionAt(s);
    double ds = s - sec.s0, acc = 0;
    for (const Strip& st : sec.left) acc += std::max(0.0, st.width.eval(ds));
    return acc + laneOffset.eval(s);
}

double CrossSection::rightExtentAt(double s) const {
    if (sections.empty()) return 0.0;
    const LaneSection& sec = sectionAt(s);
    double ds = s - sec.s0, acc = 0;
    for (const Strip& st : sec.right) acc += std::max(0.0, st.width.eval(ds));
    return -acc + laneOffset.eval(s);
}

double CrossSection::heightAt(double s, double t) const {
    if (sections.empty()) return 0.0;
    const LaneSection& sec = sectionAt(s);
    double ds = s - sec.s0;
    double local = t - laneOffset.eval(s);
    // Hot path: the shader calls this per fragment, so the scratch buffer is
    // kept alive rather than reallocated a million times a frame.
    static thread_local std::vector<Span> spans;
    buildSpans(sec, ds, spans);
    if (spans.empty()) return 0.0;
    for (size_t i = 0; i < spans.size(); ++i) {
        const Span& sp = spans[i];
        if (local < sp.t0 - 1e-9 || local > sp.t1 + 1e-9) continue;
        if (sp.strip->kind != StripKind::Curb) return sp.strip->height;
        // A kerb is the ramp between its neighbours, not a plateau — which is
        // what makes the sidewalk actually step up out of the gutter.
        double hIn = i > 0 ? spans[i - 1].strip->height : sp.strip->height;
        double hOut = (i + 1 < spans.size()) ? spans[i + 1].strip->height : sp.strip->height;
        double w = sp.t1 - sp.t0;
        double u = w > 1e-6 ? saturate((local - sp.t0) / w) : 0.0;
        return lerp(hIn, hOut, u);
    }
    return local < spans.front().t0 ? spans.front().strip->height : spans.back().strip->height;
}

const Strip* CrossSection::stripAtT(double s, double t, int* laneId) const {
    if (sections.empty()) return nullptr;
    const LaneSection& sec = sectionAt(s);
    double ds = s - sec.s0;
    double local = t - laneOffset.eval(s);
    // Hot path: the shader calls this per fragment, so the scratch buffer is
    // kept alive rather than reallocated a million times a frame.
    static thread_local std::vector<Span> spans;
    buildSpans(sec, ds, spans);
    for (const Span& sp : spans) {
        if (local >= sp.t0 - 1e-9 && local <= sp.t1 + 1e-9) {
            if (laneId) *laneId = sp.id;
            return sp.strip;
        }
    }
    if (laneId) *laneId = 0;
    return nullptr;
}

void CrossSection::linkSections() {
    for (size_t i = 0; i + 1 < sections.size(); ++i) {
        LaneSection& a = sections[i];
        LaneSection& b = sections[i + 1];
        // Match by ordinal within each side first; a lane that has no counterpart
        // (because it tapered out) links to nothing, which is exactly the signal
        // the simulator needs for "you must be out of this lane by its end".
        auto link = [](std::vector<Strip>& sa, std::vector<Strip>& sb, int sign) {
            for (size_t k = 0; k < sa.size(); ++k) {
                if (k < sb.size() && sa[k].kind == sb[k].kind) {
                    sa[k].successor = sign * (int(k) + 1);
                    sb[k].predecessor = sign * (int(k) + 1);
                } else {
                    sa[k].successor = kNoLane;
                }
            }
            for (size_t k = 0; k < sb.size(); ++k) {
                if (k >= sa.size() || sa[k].kind != sb[k].kind) sb[k].predecessor = kNoLane;
            }
        };
        link(a.left, b.left, +1);
        link(a.right, b.right, -1);
    }
}

// --- names ----------------------------------------------------------------

const char* stripKindName(StripKind k) {
    switch (k) {
        case StripKind::Shoulder: return "shoulder";
        case StripKind::Travel: return "travel";
        case StripKind::Turn: return "turn";
        case StripKind::Bus: return "bus";
        case StripKind::Bike: return "bike";
        case StripKind::Parking: return "parking";
        case StripKind::Median: return "median";
        case StripKind::Apron: return "apron";
        case StripKind::Curb: return "curb";
        case StripKind::Gutter: return "gutter";
        case StripKind::Verge: return "verge";
        case StripKind::Sidewalk: return "sidewalk";
        case StripKind::Barrier: return "barrier";
        case StripKind::Slope: return "slope";
    }
    return "?";
}

const char* surfaceName(SurfaceKind s) {
    switch (s) {
        case SurfaceKind::Asphalt: return "asphalt";
        case SurfaceKind::Concrete: return "concrete";
        case SurfaceKind::Gravel: return "gravel";
        case SurfaceKind::Dirt: return "dirt";
        case SurfaceKind::Grass: return "grass";
        case SurfaceKind::Brick: return "brick";
        case SurfaceKind::Steel: return "steel";
    }
    return "?";
}

const char* markStyleName(MarkStyle s) {
    switch (s) {
        case MarkStyle::None: return "none";
        case MarkStyle::Solid: return "solid";
        case MarkStyle::Dashed: return "dashed";
        case MarkStyle::Double: return "double";
        case MarkStyle::SolidDashed: return "solid-dashed";
        case MarkStyle::DashedSolid: return "dashed-solid";
        case MarkStyle::WideDashed: return "wide-dashed";
        case MarkStyle::Botts: return "botts";
        case MarkStyle::Hatch: return "hatch";
        case MarkStyle::Chevron: return "chevron";
    }
    return "?";
}

// --- presets --------------------------------------------------------------

namespace {

Strip mkStrip(StripKind kind, double width, int8_t dir, SurfaceKind surf, Marking outer,
              double height = 0.0, uint32_t access = kAccessCar | kAccessTruck | kAccessBus |
                                                     kAccessEmergency) {
    Strip s;
    s.kind = kind;
    s.width = Poly3::constant(width);
    s.dir = dir;
    s.surface = surf;
    s.outerMark = outer;
    s.height = float(height);
    s.access = access;
    return s;
}

// Kerbside furniture shared by every urban preset: gutter, kerb, footway.
void appendKerb(std::vector<Strip>& stack, double sidewalkWidth, double curbHeight = 0.15) {
    stack.push_back(mkStrip(StripKind::Gutter, 0.30, 0, SurfaceKind::Concrete, markNone(), 0.0,
                            kAccessEmergency));
    stack.push_back(mkStrip(StripKind::Curb, 0.15, 0, SurfaceKind::Concrete, markNone(),
                            curbHeight, 0));
    if (sidewalkWidth > 0.01) {
        stack.push_back(mkStrip(StripKind::Sidewalk, sidewalkWidth, 0, SurfaceKind::Concrete,
                                markNone(), curbHeight, kAccessPed));
    }
}

void appendShoulderAndSlope(std::vector<Strip>& stack, double shoulder, double slope) {
    stack.push_back(mkStrip(StripKind::Shoulder, shoulder, 0, SurfaceKind::Asphalt, markNone(),
                            0.0, kAccessEmergency));
    if (slope > 0.01) {
        stack.push_back(
            mkStrip(StripKind::Slope, slope, 0, SurfaceKind::Grass, markNone(), 0.0, 0));
    }
}

RoadPreset makeStreet2(bool parking) {
    RoadPreset p;
    p.name = parking ? "street2_park" : "street2";
    p.designSpeedKph = 40;
    for (int side = 0; side < 2; ++side) {
        std::vector<Strip>& stack = side == 0 ? p.section.right : p.section.left;
        int8_t dir = side == 0 ? +1 : -1;
        stack.push_back(mkStrip(StripKind::Travel, 3.30, dir, SurfaceKind::Asphalt,
                                parking ? markNone() : markNone()));
        if (parking) {
            stack.push_back(mkStrip(StripKind::Parking, 2.40, dir, SurfaceKind::Asphalt,
                                    markNone(), 0.0, kAccessCar));
        }
        appendKerb(stack, 2.0);
    }
    p.section.centerMark = markDashed(PaintColor::Yellow, 3.0, 9.0);
    return p;
}

RoadPreset makeMultiLane(const std::string& name, int lanesPerDir, double laneWidth,
                         double speed, bool median, double sidewalk, bool twltl) {
    RoadPreset p;
    p.name = name;
    p.designSpeedKph = speed;
    for (int side = 0; side < 2; ++side) {
        std::vector<Strip>& stack = side == 0 ? p.section.right : p.section.left;
        int8_t dir = side == 0 ? +1 : -1;
        if (twltl) {
            Marking m;
            m.style = side == 0 ? MarkStyle::SolidDashed : MarkStyle::SolidDashed;
            m.color = PaintColor::Yellow;
            stack.push_back(mkStrip(StripKind::Turn, 1.80, 0, SurfaceKind::Asphalt, m, 0.0,
                                    kAccessCar | kAccessTruck | kAccessBus | kAccessEmergency));
        } else if (median) {
            stack.push_back(mkStrip(StripKind::Median, 1.35, 0, SurfaceKind::Grass, markNone(),
                                    0.15, 0));
            stack.push_back(
                mkStrip(StripKind::Curb, 0.15, 0, SurfaceKind::Concrete, markNone(), 0.15, 0));
        }
        for (int i = 0; i < lanesPerDir; ++i) {
            bool outermost = (i == lanesPerDir - 1);
            stack.push_back(mkStrip(StripKind::Travel, laneWidth, dir, SurfaceKind::Asphalt,
                                    outermost ? markEdge() : markDashed(PaintColor::White)));
        }
        appendKerb(stack, sidewalk);
    }
    if (twltl) {
        p.section.centerMark = markNone();
    } else if (median) {
        p.section.centerMark = markNone();
    } else {
        p.section.centerMark = markDouble(PaintColor::Yellow);
    }
    return p;
}

RoadPreset makeFreeway(const std::string& name, int lanes) {
    RoadPreset p;
    p.name = name;
    p.designSpeedKph = 110;
    p.allowsPedestrians = false;
    for (int side = 0; side < 2; ++side) {
        std::vector<Strip>& stack = side == 0 ? p.section.right : p.section.left;
        int8_t dir = side == 0 ? +1 : -1;
        stack.push_back(
            mkStrip(StripKind::Barrier, 0.40, 0, SurfaceKind::Concrete, markNone(), 0.85, 0));
        // Inner shoulder; its outer edge is the yellow median edge line.
        stack.push_back(mkStrip(StripKind::Shoulder, 1.20, 0, SurfaceKind::Asphalt,
                                markSolid(PaintColor::Yellow, 0.15f), 0.0, kAccessEmergency));
        for (int i = 0; i < lanes; ++i) {
            bool outermost = (i == lanes - 1);
            stack.push_back(mkStrip(StripKind::Travel, 3.60, dir, SurfaceKind::Asphalt,
                                    outermost ? markEdge() : markDashed(PaintColor::White)));
        }
        appendShoulderAndSlope(stack, 3.00, 5.0);
    }
    p.section.centerMark = markNone();
    return p;
}

RoadPreset makeRamp(const std::string& name, int lanes) {
    RoadPreset p;
    p.name = name;
    p.designSpeedKph = 60;
    p.allowsPedestrians = false;
    // The reference line sits on the ramp's LEFT edge, which is where the yellow
    // line of a one-way carriageway belongs.
    p.section.left.push_back(mkStrip(StripKind::Shoulder, 1.20, 0, SurfaceKind::Asphalt,
                                     markNone(), 0.0, kAccessEmergency));
    p.section.left.push_back(
        mkStrip(StripKind::Barrier, 0.40, 0, SurfaceKind::Concrete, markNone(), 0.85, 0));
    for (int i = 0; i < lanes; ++i) {
        bool outermost = (i == lanes - 1);
        p.section.right.push_back(mkStrip(StripKind::Travel, 4.20, +1, SurfaceKind::Asphalt,
                                          outermost ? markEdge()
                                                    : markDashed(PaintColor::White)));
    }
    p.section.right.push_back(mkStrip(StripKind::Shoulder, 2.40, 0, SurfaceKind::Asphalt,
                                      markNone(), 0.0, kAccessEmergency));
    p.section.right.push_back(
        mkStrip(StripKind::Barrier, 0.40, 0, SurfaceKind::Concrete, markNone(), 0.85, 0));
    p.section.centerMark = markSolid(PaintColor::Yellow, 0.15f);
    return p;
}

RoadPreset makeRoundabout(const std::string& name, int lanes) {
    RoadPreset p;
    p.name = name;
    p.designSpeedKph = 30;
    // Right-hand traffic circulates counter-clockwise, so the island is to the
    // LEFT of the ring's direction of travel (+t).
    p.section.left.push_back(mkStrip(StripKind::Apron, 1.20, 0, SurfaceKind::Concrete,
                                     markNone(), 0.05, kAccessTruck | kAccessEmergency));
    p.section.left.push_back(
        mkStrip(StripKind::Curb, 0.20, 0, SurfaceKind::Concrete, markNone(), 0.14, 0));
    p.section.left.push_back(
        mkStrip(StripKind::Verge, 6.00, 0, SurfaceKind::Grass, markNone(), 0.14, 0));
    for (int i = 0; i < lanes; ++i) {
        bool outermost = (i == lanes - 1);
        p.section.right.push_back(mkStrip(StripKind::Travel, 4.50, +1, SurfaceKind::Asphalt,
                                          outermost ? markNone()
                                                    : markDashed(PaintColor::White, 1.0f, 1.0f)));
    }
    appendKerb(p.section.right, 2.0);
    p.section.centerMark = markSolid(PaintColor::White, 0.15f);
    return p;
}

RoadPreset makeCountry2() {
    RoadPreset p;
    p.name = "country2";
    p.designSpeedKph = 90;
    for (int side = 0; side < 2; ++side) {
        std::vector<Strip>& stack = side == 0 ? p.section.right : p.section.left;
        int8_t dir = side == 0 ? +1 : -1;
        stack.push_back(
            mkStrip(StripKind::Travel, 3.40, dir, SurfaceKind::Asphalt, markEdge()));
        stack.push_back(mkStrip(StripKind::Shoulder, 1.50, 0, SurfaceKind::Gravel, markNone(),
                                0.0, kAccessEmergency));
        stack.push_back(
            mkStrip(StripKind::Slope, 3.00, 0, SurfaceKind::Grass, markNone(), 0.0, 0));
    }
    p.section.centerMark = markDashed(PaintColor::Yellow, 3.0f, 9.0f);
    return p;
}

RoadPreset makeDeck(const std::string& name, bool tunnel) {
    RoadPreset p;
    p.name = name;
    p.designSpeedKph = 80;
    p.allowsPedestrians = false;
    for (int side = 0; side < 2; ++side) {
        std::vector<Strip>& stack = side == 0 ? p.section.right : p.section.left;
        int8_t dir = side == 0 ? +1 : -1;
        stack.push_back(mkStrip(StripKind::Travel, 3.50, dir,
                                tunnel ? SurfaceKind::Concrete : SurfaceKind::Asphalt,
                                markEdge()));
        stack.push_back(mkStrip(StripKind::Shoulder, tunnel ? 0.80 : 1.00, 0,
                                tunnel ? SurfaceKind::Concrete : SurfaceKind::Asphalt,
                                markNone(), 0.0, kAccessEmergency));
        if (tunnel) {
            stack.push_back(
                mkStrip(StripKind::Curb, 0.20, 0, SurfaceKind::Concrete, markNone(), 0.25, 0));
            stack.push_back(mkStrip(StripKind::Sidewalk, 0.90, 0, SurfaceKind::Concrete,
                                    markNone(), 0.25, kAccessEmergency | kAccessPed));
        }
        stack.push_back(mkStrip(StripKind::Barrier, 0.50, 0, SurfaceKind::Concrete, markNone(),
                                tunnel ? 1.20 : 1.05, 0));
    }
    p.section.centerMark = markDouble(PaintColor::Yellow);
    return p;
}

RoadPreset makeOneWay(const std::string& name, int lanes, double width, bool sidewalks) {
    RoadPreset p;
    p.name = name;
    p.designSpeedKph = 40;
    // Everything hangs off the right of the reference line: an honest one-way
    // carriageway whose reference line is its left kerb.
    for (int i = 0; i < lanes; ++i) {
        bool outermost = (i == lanes - 1);
        p.section.right.push_back(mkStrip(StripKind::Travel, width, +1, SurfaceKind::Asphalt,
                                          outermost ? markNone()
                                                    : markDashed(PaintColor::White)));
    }
    if (sidewalks) {
        appendKerb(p.section.right, 1.8);
        appendKerb(p.section.left, 1.8);
    }
    p.section.centerMark = markNone();
    return p;
}

}  // namespace

RoadPreset roadPreset(const std::string& name) {
    if (name == "street2") return makeStreet2(false);
    if (name == "street2_park") return makeStreet2(true);
    if (name == "collector4") return makeMultiLane("collector4", 2, 3.40, 50, false, 2.0, false);
    if (name == "arterial4") return makeMultiLane("arterial4", 2, 3.50, 60, false, 2.5, false);
    if (name == "arterial6_median")
        return makeMultiLane("arterial6_median", 3, 3.50, 70, true, 2.5, false);
    if (name == "boulevard_twltl")
        return makeMultiLane("boulevard_twltl", 2, 3.50, 60, false, 2.5, true);
    if (name == "freeway2") return makeFreeway("freeway2", 2);
    if (name == "freeway3") return makeFreeway("freeway3", 3);
    if (name == "freeway4") return makeFreeway("freeway4", 4);
    if (name == "ramp1") return makeRamp("ramp1", 1);
    if (name == "ramp2") return makeRamp("ramp2", 2);
    if (name == "roundabout1") return makeRoundabout("roundabout1", 1);
    if (name == "roundabout2") return makeRoundabout("roundabout2", 2);
    if (name == "country2") return makeCountry2();
    if (name == "bridge2") return makeDeck("bridge2", false);
    if (name == "tunnel2") return makeDeck("tunnel2", true);
    if (name == "alley1") return makeOneWay("alley1", 1, 4.00, false);
    if (name == "service1") return makeOneWay("service1", 1, 3.60, true);
    if (name == "street1_oneway") return makeOneWay("street1_oneway", 2, 3.30, true);
    return makeStreet2(false);
}

std::vector<std::string> roadPresetNames() {
    return {"street2",   "street2_park",     "street1_oneway",  "alley1",     "service1",
            "collector4", "arterial4",       "arterial6_median", "boulevard_twltl",
            "freeway2",  "freeway3",         "freeway4",        "ramp1",      "ramp2",
            "roundabout1", "roundabout2",    "country2",        "bridge2",    "tunnel2"};
}

// --- transitions ----------------------------------------------------------

double minRadiusForSpeed(double speedKph, double sideFriction) {
    return speedKph * speedKph / (127.0 * std::max(0.02, sideFriction));
}

double designSpeedForRadius(double radius, double sideFriction) {
    return std::sqrt(127.0 * std::max(0.02, sideFriction) * std::max(1.0, radius));
}

double taperLength(double widthChange, double speedKph) {
    double w = std::fabs(widthChange);
    if (w < 1e-3) return 0.0;
    double v = std::max(15.0, speedKph);
    // MUTCD, converted to metric. Above ~72 km/h the taper is proportional to
    // speed; below it, to speed squared. Nobody types these by hand.
    double len = (v >= 72.0) ? (w * v / 1.609344) : (w * v * v / 155.36);
    return std::max(12.0, len);
}

namespace {

int kindGroup(StripKind k) {
    switch (k) {
        case StripKind::Travel:
        case StripKind::Turn:
        case StripKind::Bus:
            return 0;
        case StripKind::Shoulder:
        case StripKind::Gutter:
            return 1;
        case StripKind::Median:
        case StripKind::Barrier:
        case StripKind::Apron:
            return 2;
        case StripKind::Sidewalk:
        case StripKind::Verge:
            return 3;
        case StripKind::Curb:
            return 4;
        case StripKind::Parking:
            return 5;
        case StripKind::Bike:
            return 6;
        case StripKind::Slope:
            return 7;
    }
    return 8;
}

double substitutionCost(StripKind a, StripKind b) {
    if (a == b) return 0.0;
    if (kindGroup(a) == kindGroup(b)) return 1.0;
    return 4.0;
}

struct Pairing {
    int ia = -1;   // index in A, -1 = inserted
    int ib = -1;   // index in B, -1 = deleted
};

// Needleman-Wunsch over the strip-kind sequences. Alignment (rather than
// index-matching) is what lets a 5-lane section become a 3-lane-plus-bike-lane
// section without anyone enumerating the cases.
std::vector<Pairing> alignStrips(const std::vector<Strip>& A, const std::vector<Strip>& B) {
    const size_t n = A.size(), m = B.size();
    const double kGap = 2.0;
    std::vector<double> d((n + 1) * (m + 1), 0.0);
    auto at = [&](size_t i, size_t j) -> double& { return d[i * (m + 1) + j]; };
    for (size_t i = 0; i <= n; ++i) at(i, 0) = double(i) * kGap;
    for (size_t j = 0; j <= m; ++j) at(0, j) = double(j) * kGap;
    for (size_t i = 1; i <= n; ++i) {
        for (size_t j = 1; j <= m; ++j) {
            double sub = at(i - 1, j - 1) + substitutionCost(A[i - 1].kind, B[j - 1].kind);
            double del = at(i - 1, j) + kGap;
            double ins = at(i, j - 1) + kGap;
            at(i, j) = std::min(sub, std::min(del, ins));
        }
    }
    std::vector<Pairing> out;
    size_t i = n, j = m;
    while (i > 0 || j > 0) {
        if (i > 0 && j > 0 &&
            std::fabs(at(i, j) - (at(i - 1, j - 1) + substitutionCost(A[i - 1].kind, B[j - 1].kind))) <
                1e-9) {
            out.push_back({int(i) - 1, int(j) - 1});
            --i;
            --j;
        } else if (i > 0 && std::fabs(at(i, j) - (at(i - 1, j) + kGap)) < 1e-9) {
            out.push_back({int(i) - 1, -1});
            --i;
        } else {
            out.push_back({-1, int(j) - 1});
            --j;
        }
    }
    std::reverse(out.begin(), out.end());
    return out;
}

void blendStack(const std::vector<Strip>& A, double dsA, const std::vector<Strip>& B,
                double length, std::vector<Strip>& out) {
    out.clear();
    for (const Pairing& p : alignStrips(A, B)) {
        Strip s;
        if (p.ia >= 0 && p.ib >= 0) {
            const Strip& a = A[size_t(p.ia)];
            const Strip& b = B[size_t(p.ib)];
            s = b;
            s.width = Poly3::smooth(std::max(0.0, a.width.eval(dsA)), b.width.eval(0.0), length);
        } else if (p.ia >= 0) {
            // Dropped. The paint that closes a lane is a SOLID line, and it comes
            // from the same fact that removes the lane.
            const Strip& a = A[size_t(p.ia)];
            s = a;
            s.width = Poly3::smooth(std::max(0.0, a.width.eval(dsA)), 0.0, length);
            if (s.outerMark.style == MarkStyle::Dashed ||
                s.outerMark.style == MarkStyle::WideDashed) {
                s.outerMark.style = MarkStyle::Solid;
            }
        } else if (p.ib >= 0) {
            const Strip& b = B[size_t(p.ib)];
            s = b;
            s.width = Poly3::smooth(0.0, b.width.eval(0.0), length);
        } else {
            continue;
        }
        out.push_back(s);
    }
}

}  // namespace

double minTransitionLength(const LaneSection& a, const LaneSection& b, double speedKph) {
    double worst = 0.0;
    auto scan = [&](const std::vector<Strip>& sa, const std::vector<Strip>& sb) {
        for (const Pairing& p : alignStrips(sa, sb)) {
            double wa = p.ia >= 0 ? sa[size_t(p.ia)].width.eval(0.0) : 0.0;
            double wb = p.ib >= 0 ? sb[size_t(p.ib)].width.eval(0.0) : 0.0;
            worst = std::max(worst, std::fabs(wb - wa));
        }
    };
    scan(a.left, b.left);
    scan(a.right, b.right);
    return taperLength(worst, speedKph);
}

LaneSection blendSection(const LaneSection& a, const LaneSection& b, double s0, double length) {
    LaneSection out;
    out.s0 = s0;
    out.length = length;
    double dsA = s0 - a.s0;
    blendStack(a.left, dsA, b.left, length, out.left);
    blendStack(a.right, dsA, b.right, length, out.right);
    out.centerMark = b.centerMark;
    out.assignIds();
    return out;
}

void ProfileTimeline::at(double s, const LaneSection& section, double transitionLength) {
    keys_.push_back({s, section, transitionLength});
}

CrossSection ProfileTimeline::build(double roadLength, double designSpeedKph) const {
    CrossSection xs;
    if (keys_.empty()) return xs;
    std::vector<Key> keys = keys_;
    std::sort(keys.begin(), keys.end(), [](const Key& a, const Key& b) { return a.s < b.s; });

    LaneSection current = keys.front().section;
    current.s0 = 0.0;
    current.assignIds();
    xs.sections.push_back(current);

    for (size_t i = 1; i < keys.size(); ++i) {
        const Key& k = keys[i];
        double start = clampd(k.s, xs.sections.back().s0 + 1e-3, roadLength);
        double len = k.transition >= 0.0
                         ? k.transition
                         : minTransitionLength(current, k.section, designSpeedKph);
        double room = std::max(0.0, roadLength - start);
        // A transition that does not fit is not a very short transition, it is a
        // change this road has no room for. Dropping it keeps the lint honest
        // instead of emitting a one-metre 1:0 taper.
        if (room < std::min(len, 5.0)) continue;
        len = std::min(len, room);
        if (len > 0.05) {
            LaneSection tr = blendSection(current, k.section, start, len);
            xs.sections.push_back(tr);
        }
        LaneSection steady = k.section;
        steady.s0 = std::min(roadLength, start + std::max(0.0, len));
        steady.assignIds();
        if (steady.s0 > xs.sections.back().s0 + 1e-3) xs.sections.push_back(steady);
        current = steady;
    }
    xs.finalize(roadLength);
    return xs;
}

// --- cross-section edits --------------------------------------------------

LaneSection sectionWithLanesChanged(const LaneSection& base, int side, int delta,
                                    double laneWidth) {
    LaneSection out = base;
    std::vector<Strip>& stack = side >= 0 ? out.left : out.right;
    // Find the outermost travel lane; lanes are added and dropped there because
    // that is where real roads do it (the inside lane is the through lane).
    int lastLane = -1;
    for (size_t i = 0; i < stack.size(); ++i) {
        if (stack[i].isLane()) lastLane = int(i);
    }
    if (lastLane < 0) return out;
    if (delta > 0) {
        Strip proto = stack[size_t(lastLane)];
        proto.width = Poly3::constant(laneWidth);
        Marking edge = proto.outerMark;
        stack[size_t(lastLane)].outerMark = markDashed(PaintColor::White);
        for (int i = 0; i < delta; ++i) {
            Strip s = proto;
            s.outerMark = (i == delta - 1) ? edge : markDashed(PaintColor::White);
            stack.insert(stack.begin() + lastLane + 1 + i, s);
        }
    } else if (delta < 0) {
        int remove = std::min(-delta, lastLane + 1);
        Marking edge = stack[size_t(lastLane)].outerMark;
        stack.erase(stack.begin() + (lastLane - remove + 1), stack.begin() + lastLane + 1);
        int newLast = -1;
        for (size_t i = 0; i < stack.size(); ++i) {
            if (stack[i].isLane()) newLast = int(i);
        }
        if (newLast >= 0) stack[size_t(newLast)].outerMark = edge;
    }
    out.assignIds();
    return out;
}

LaneSection sectionWithTurnBay(const LaneSection& base, int side, double width) {
    LaneSection out = base;
    std::vector<Strip>& stack = side >= 0 ? out.left : out.right;
    int firstLane = -1;
    for (size_t i = 0; i < stack.size(); ++i) {
        if (stack[i].isLane()) {
            firstLane = int(i);
            break;
        }
    }
    if (firstLane < 0) return out;
    Strip bay = stack[size_t(firstLane)];
    bay.kind = StripKind::Turn;
    bay.width = Poly3::constant(width);
    bay.outerMark = markSolid(PaintColor::White, 0.12f);
    stack.insert(stack.begin() + firstLane, bay);
    out.assignIds();
    return out;
}

LaneSection sectionWithTwltl(const LaneSection& base, double width) {
    LaneSection out = base;
    Marking m;
    m.style = MarkStyle::SolidDashed;
    m.color = PaintColor::Yellow;
    for (int side = 0; side < 2; ++side) {
        std::vector<Strip>& stack = side == 0 ? out.right : out.left;
        Strip bay;
        bay.kind = StripKind::Turn;
        bay.surface = SurfaceKind::Asphalt;
        bay.width = Poly3::constant(width * 0.5);
        bay.dir = 0;
        bay.outerMark = m;
        // Drop any raised median that the TWLTL replaces.
        for (size_t i = 0; i < stack.size();) {
            if (stack[i].kind == StripKind::Median) {
                stack.erase(stack.begin() + long(i));
            } else {
                ++i;
            }
        }
        stack.insert(stack.begin(), bay);
    }
    out.centerMark = markNone();
    out.assignIds();
    return out;
}

}  // namespace roadlab
