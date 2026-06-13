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

