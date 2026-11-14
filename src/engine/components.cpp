#include "components.h"

namespace engine {

Mat4 Transform::matrix() const {
    return Mat4::trs(position, orientation, scale);
}

Transform lerp(const Transform& a, const Transform& b, Real t) {
    Transform result;
    result.position = lerp(a.position, b.position, t);
    result.orientation = Quat::slerp(a.orientation, b.orientation, t);
    result.scale = lerp(a.scale, b.scale, t);
    return result;
}

Transform transformFromMatrix(const Mat4& m) {
    Transform t;
    t.position = Vec3(m.m[0][3], m.m[1][3], m.m[2][3]);
    Vec3 cx(m.m[0][0], m.m[1][0], m.m[2][0]);
    Vec3 cy(m.m[0][1], m.m[1][1], m.m[2][1]);
    Vec3 cz(m.m[0][2], m.m[1][2], m.m[2][2]);
    t.scale = Vec3(cx.length(), cy.length(), cz.length());
    if (t.scale.x < 1e-9 || t.scale.y < 1e-9 || t.scale.z < 1e-9) return t;

    Mat4 pure;
    Vec3 nx = cx / t.scale.x, ny = cy / t.scale.y, nz = cz / t.scale.z;
    pure.m[0][0] = nx.x; pure.m[1][0] = nx.y; pure.m[2][0] = nx.z;
    pure.m[0][1] = ny.x; pure.m[1][1] = ny.y; pure.m[2][1] = ny.z;
    pure.m[0][2] = nz.x; pure.m[1][2] = nz.y; pure.m[2][2] = nz.z;
    t.orientation = Quat::fromRotationMatrix(pure);
    return t;
}

uint32_t maxDocumentId(World& world) {
    uint32_t maxId = 0;
    world.each<SourceSpec>([&](Entity, SourceSpec& s) {
        if (s.id > maxId) maxId = s.id;
    });
    return maxId;
}

uint32_t nextDocumentId(World& world) { return maxDocumentId(world) + 1; }

void assignMissingDocumentIds(World& world) {
    uint32_t next = maxDocumentId(world) + 1;
    world.each<SourceSpec>([&](Entity, SourceSpec& s) {
        if (s.id == 0) s.id = next++;
    });
}

Entity findByDocumentId(World& world, uint32_t id) {
    Entity found;
    if (id == 0) return found;
    world.each<SourceSpec>([&](Entity e, SourceSpec& s) {
        if (s.id == id) found = e;
    });
    return found;
}

Mat4 worldMatrix(World& world, Entity e) {
    Transform* t = world.get<Transform>(e);
    if (!t) return Mat4();
    Mat4 local = t->matrix();

    SourceSpec* s = world.get<SourceSpec>(e);
    // Walk up the parent chain, composing parentWorld * local. A depth cap
    // makes a malformed (cyclic) document terminate instead of recursing
    // forever — the editor's reparent guard prevents cycles, but a
    // hand-edited level could still carry one.
    int guard = 0;
    while (s && s->parentId != 0 && guard++ < 64) {
        Entity parent = findByDocumentId(world, s->parentId);
        Transform* pt = parent.valid() ? world.get<Transform>(parent) : nullptr;
        if (!pt || parent == e) break;
        local = pt->matrix() * local;
        s = world.get<SourceSpec>(parent);
        e = parent;
    }
    return local;
}

bool wouldCreateCycle(World& world, Entity child, Entity parent) {
    SourceSpec* childSpec = world.get<SourceSpec>(child);
    if (!childSpec) return false;
    // Walk up from `parent`: if we reach `child`'s id, parenting closes a loop.
    Entity cursor = parent;
    int guard = 0;
    while (cursor.valid() && guard++ < 64) {
        if (cursor == child) return true;
        SourceSpec* s = world.get<SourceSpec>(cursor);
        if (!s || s->parentId == 0) break;
        cursor = findByDocumentId(world, s->parentId);
    }
    return false;
}

}  // namespace engine


#include "ai/nav_graph.h"   // LevelRoadModel::stateSignals
#include "bundle/bundle.h"  // fnv1aStr: LevelRoadModel::runKey
#ifdef RT_ROADS_LANES
#include "procgen/city/polyline.h"           // distToSegment
#include "procgen/city/roads/road_model.h"
#endif
#include <map>

namespace engine {

std::string LevelRoadModel::streetNameNear(const Vec2& p, Real reach) const {
    std::string out;
#ifdef RT_ROADS_LANES
    Real best = reach;
    for (const std::shared_ptr<RoadModel>& m : cities) {
        if (!m) continue;
        Real d = 0;
        const std::string name = engine::streetNameNear(*m, p, best, &d);
        if (!name.empty() && d < best) {
            best = d;
            out = name;
        }
    }
#else
    (void)p;
    (void)reach;
#endif
    return out;
}

#ifdef RT_ROADS_LANES
struct LevelRoadModel::StreetCells {
    static constexpr Real kCell = 25.0;
    struct Seg { int city, street; Vec2 a, b; };
    struct Point { int city, junction; Vec2 at; };
    std::map<std::pair<int, int>, std::vector<Seg>> cells;
    std::map<std::pair<int, int>, std::vector<Point>> junctions;   // each junction's centre and arm mouths
    static std::pair<int, int> cellOf(const Vec2& p) {
        return {static_cast<int>(std::floor(p.x / kCell)), static_cast<int>(std::floor(p.y / kCell))};
    }
};

static std::shared_ptr<const LevelRoadModel::StreetCells> buildCells(const std::vector<std::shared_ptr<RoadModel>>& cities) {
    using Cells = LevelRoadModel::StreetCells;
    std::shared_ptr<Cells> idx = std::make_shared<Cells>();
    for (std::size_t c = 0; c < cities.size(); ++c) {
        if (!cities[c]) continue;
        const RoadModel& m = *cities[c];
        for (std::size_t s = 0; s < m.streets.size(); ++s) {
            const Range& L = m.streets[s].line;
            for (uint32_t k = L.first; L.count >= 2 && k + 1 < L.end(); ++k) {
                const Vec2 a = m.linePoints[k], b = m.linePoints[k + 1];
                const std::pair<int, int> lo = Cells::cellOf(Vec2(std::min(a.x, b.x), std::min(a.y, b.y)));
                const std::pair<int, int> hi = Cells::cellOf(Vec2(std::max(a.x, b.x), std::max(a.y, b.y)));
                for (int x = lo.first; x <= hi.first; ++x)
                    for (int z = lo.second; z <= hi.second; ++z)
                        idx->cells[{x, z}].push_back({static_cast<int>(c), static_cast<int>(s), a, b});
            }
        }
        for (std::size_t j = 0; j < m.junctions.size(); ++j) {
            const ModelJunction& J = m.junctions[j];
            idx->junctions[Cells::cellOf(J.at)].push_back({static_cast<int>(c), static_cast<int>(j), J.at});
            for (std::size_t k = 0; k < J.arms.size(); ++k) {
                Vec2 mouth;
                if (armMouth(m, J, k, mouth)) idx->junctions[Cells::cellOf(mouth)].push_back({static_cast<int>(c), static_cast<int>(j), mouth});
            }
        }
    }
    return idx;
}

LevelRoadModel::Place LevelRoadModel::junctionAt(const Vec2& p, Real reach) const {
    using Cells = LevelRoadModel::StreetCells;
    if (!streetCells_) streetCells_ = buildCells(cities);
    Place out;
    Real best = reach;
    const int r = static_cast<int>(std::ceil(reach / Cells::kCell));
    const std::pair<int, int> c0 = Cells::cellOf(p);
    for (int dx = -r; dx <= r; ++dx)
        for (int dz = -r; dz <= r; ++dz) {
            const std::map<std::pair<int, int>, std::vector<Cells::Point>>::const_iterator it =
                streetCells_->junctions.find({c0.first + dx, c0.second + dz});
            if (it == streetCells_->junctions.end()) continue;
            for (const Cells::Point& q : it->second) {
                const Real d = (q.at - p).length();
                // (ties: the lower city, then the lower junction)
                if (d < best || (d == best && out.valid() && (q.city < out.city || (q.city == out.city && q.junction < out.junction)))) {
                    best = d;
                    out.city = q.city;
                    out.junction = q.junction;
                }
            }
        }
    return out;
}

LevelRoadModel::Place LevelRoadModel::armAt(Place j, const Vec2& p, const Vec2& outward) const {
    j.arm = -1;
    if (!j.valid() || j.junction < 0 || !cities[static_cast<std::size_t>(j.city)]) return j;
    const RoadModel& m = *cities[static_cast<std::size_t>(j.city)];
    const ModelJunction& J = m.junctions[static_cast<std::size_t>(j.junction)];
    const Real len = outward.length();
    if (len < 1e-9) return j;
    const Vec2 out = outward * (1.0 / len);
    const Real cosLimit = std::cos(40.0 * DEG_TO_RAD);
    Real best = 1e30;
    for (std::size_t k = 0; k < J.arms.size(); ++k) {
        const ModelArm& a = J.arms[k];
        Vec2 mouth;
        if (!stated(a.heading) || !armMouth(m, J, k, mouth)) continue;
        const Vec2 h(std::cos(a.heading * DEG_TO_RAD), std::sin(a.heading * DEG_TO_RAD));
        if (dot(h, out) < cosLimit) continue;
        const Real d = (mouth - p).length();
        if (d < best) {
            best = d;
            j.arm = static_cast<int>(k);
            j.street = static_cast<int>(a.street.index);
        }
    }
    return j;
}

LevelRoadModel::Place LevelRoadModel::streetAt(const Vec2& p, Real reach) const {
    using Cells = LevelRoadModel::StreetCells;
    if (!streetCells_) streetCells_ = buildCells(cities);
    Place out;
    Real best = reach;
    const int r = static_cast<int>(std::ceil(reach / Cells::kCell));
    const int cx = static_cast<int>(std::floor(p.x / Cells::kCell)), cz = static_cast<int>(std::floor(p.y / Cells::kCell));
    for (int dx = -r; dx <= r; ++dx)
        for (int dz = -r; dz <= r; ++dz) {
            const std::map<std::pair<int, int>, std::vector<Cells::Seg>>::const_iterator it = streetCells_->cells.find({cx + dx, cz + dz});
            if (it == streetCells_->cells.end()) continue;
            for (const Cells::Seg& s : it->second) {
                const Real d = distToSegment(p, s.a, s.b);
                // (ties: the lower city, then the lower street, as a scan in order would keep)
                if (d < best || (d == best && out.valid() && (s.city < out.city || (s.city == out.city && s.street < out.street)))) {
                    best = d;
                    out.city = s.city;
                    out.street = s.street;
                }
            }
        }
    return out;
}

std::vector<Vec2> LevelRoadModel::junctionPoints() const {
    std::vector<Vec2> out;
    for (const std::shared_ptr<RoadModel>& mp : cities) {
        if (!mp) continue;
        for (const ModelJunction& J : mp->junctions) {
            out.push_back(J.at);
            for (std::size_t k = 0; k < J.arms.size(); ++k) {
                Vec2 mouth;
                if (armMouth(*mp, J, k, mouth) && (mouth - J.at).length() > 0.5) out.push_back(mouth);
            }
        }
    }
    return out;
}

uint64_t LevelRoadModel::runKey(const Place& s) const {
    if (s.city < 0 || s.street < 0 || !cities[static_cast<std::size_t>(s.city)]) return 0;
    const RoadModel& m = *cities[static_cast<std::size_t>(s.city)];
    const std::string name = streetName(m, ModelStreetId(static_cast<uint32_t>(s.street)));
    const std::string key = std::to_string(s.city) + "|" +
                            (name.empty() ? "street|" + m.streets[static_cast<std::size_t>(s.street)].id : "run|" + name);
    return bundle::fnv1aStr(key);
}

bool LevelRoadModel::controlsStated() const {
    for (const std::shared_ptr<RoadModel>& m : cities)
        if (m)
            for (const ModelJunction& J : m->junctions)
                if (!J.control.empty()) return true;
    return false;
}

bool LevelRoadModel::lit(const Place& j) const {
    if (!j.valid() || j.junction < 0 || !cities[static_cast<std::size_t>(j.city)]) return false;
    const ModelJunction& J = cities[static_cast<std::size_t>(j.city)]->junctions[static_cast<std::size_t>(j.junction)];
    const AttachmentKind* pole = attachmentKinds().find(attachmentKinds().use("signal_pole"));
    return !J.control.empty() && pole && !pole->controls.empty() && pole->standsFor(J.control);
}

void LevelRoadModel::stateSignals(NavGraph& nav, int* untied) const {
    if (untied) *untied = 0;
    if (!controlsStated()) return;
    nav.statedSignal.assign(static_cast<std::size_t>(nav.nodeCount()), 0);
    for (int n = 0; n < nav.streetNodeCount(); ++n) {
        if (nav.kindOf(n) != JunctionKind::Intersection) continue;
        const Place j = junctionAt(nav.nodes[static_cast<std::size_t>(n)]);
        if (!j.valid()) {
            if (untied) ++*untied;
            continue;
        }
        nav.statedSignal[static_cast<std::size_t>(n)] = lit(j) ? 1 : 0;
    }
}
#else
LevelRoadModel::Place LevelRoadModel::junctionAt(const Vec2&, Real) const { return Place{}; }
LevelRoadModel::Place LevelRoadModel::armAt(Place j, const Vec2&, const Vec2&) const { return j; }
LevelRoadModel::Place LevelRoadModel::streetAt(const Vec2&, Real) const { return Place{}; }
std::vector<Vec2> LevelRoadModel::junctionPoints() const { return {}; }
uint64_t LevelRoadModel::runKey(const Place&) const { return 0; }
bool LevelRoadModel::controlsStated() const { return false; }
bool LevelRoadModel::lit(const Place&) const { return false; }
void LevelRoadModel::stateSignals(NavGraph&, int* untied) const { if (untied) *untied = 0; }
#endif

}  // namespace engine
