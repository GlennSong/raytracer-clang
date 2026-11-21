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
#include "procgen/city/roads/road_model.h"
#endif

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

const ModelJunction* LevelRoadModel::junctionAt(int c, int junction) const {
    const RoadModel* m = city(c);
    return m && junction >= 0 && static_cast<std::size_t>(junction) < m->junctions.size()
               ? &m->junctions[static_cast<std::size_t>(junction)] : nullptr;
}

std::shared_ptr<RoadModel> LevelRoadModel::builtCity() const {
    return !cities.empty() && cities.front() && cities.front()->built ? cities.front() : nullptr;
}

uint64_t LevelRoadModel::runKey(const Place& s) const {
    if (s.street < 0 || !city(s.city)) return 0;
    const RoadModel& m = *city(s.city);
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
    const ModelJunction* J = junctionAt(j.city, j.junction);
    return J && junctionLit(*J);
}

void LevelRoadModel::stateSignals(NavGraph& nav, int* untied, bool* stated) const {
    if (untied) *untied = 0;
    if (stated) *stated = controlsStated();
    nav.statedSignal.assign(static_cast<std::size_t>(nav.nodeCount()), 0);
    for (int n = 0; n < nav.streetNodeCount(); ++n) {
        if (nav.kindOf(n) != JunctionKind::Intersection) continue;
        // the node's own junction, by id (ADR-0425: the street map carries it)
        Place j;
        j.city = nav.cityOf(n);
        j.junction = nav.junctionOf(n);
        if (j.city < 0 || j.junction < 0 || static_cast<std::size_t>(j.city) >= cities.size()) {
            if (untied) ++*untied;
            continue;
        }
        nav.statedSignal[static_cast<std::size_t>(n)] = lit(j) ? 1 : 0;
    }
}
#else
const ModelJunction* LevelRoadModel::junctionAt(int, int) const { return nullptr; }
std::shared_ptr<RoadModel> LevelRoadModel::builtCity() const { return nullptr; }
std::vector<Vec2> LevelRoadModel::junctionPoints() const { return {}; }
uint64_t LevelRoadModel::runKey(const Place&) const { return 0; }
bool LevelRoadModel::controlsStated() const { return false; }
bool LevelRoadModel::lit(const Place&) const { return false; }
void LevelRoadModel::stateSignals(NavGraph&, int* untied, bool* stated) const {
    if (untied) *untied = 0;
    if (stated) *stated = false;
}
#endif

}  // namespace engine
