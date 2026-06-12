#include "undo_stack.h"

#include "property_json.h"

namespace engine {

namespace {
// Bounded so a long session can't grow memory without limit; oldest history
// falls off first.
constexpr std::size_t MAX_DEPTH = 256;
}  // namespace

void UndoStack::recordFieldEdit(Entity e, const std::string& component,
                                const char* fieldLabel, nlohmann::json before,
                                nlohmann::json after) {
    if (before == after) return;

    // Coalesce runs of edits to the same field (spinner arrows, drag ticks):
    // one command spanning the first `before` to the latest `after`.
    if (fieldLabel && !undoList.empty()) {
        Command& top = undoList.back();
        if (top.kind == Command::FieldEdit && top.entity == e &&
            top.component == component && top.fieldLabel == fieldLabel) {
            if (top.before == after)
                undoList.pop_back();   // edits cancelled out: drop the no-op
            else
                top.after = std::move(after);
            redoList.clear();
            revision_++;
            return;
        }
    }

    Command cmd;
    cmd.kind = Command::FieldEdit;
    cmd.entity = e;
    cmd.component = component;
    cmd.fieldLabel = fieldLabel ? fieldLabel : "";
    cmd.before = std::move(before);
    cmd.after = std::move(after);
    push(std::move(cmd));
}

void UndoStack::recordTransformEdit(Entity e, const Transform& before,
                                    const Transform& after) {
    Command cmd;
    cmd.kind = Command::TransformEdit;
    cmd.entity = e;
    cmd.transformBefore = before;
    cmd.transformAfter = after;
    push(std::move(cmd));
}

void UndoStack::recordCreate(Entity e) {
    Command cmd;
    cmd.kind = Command::Create;
    cmd.entity = e;
    push(std::move(cmd));
}

void UndoStack::recordDelete(World& world, Entity e) {
    Command cmd;
    cmd.kind = Command::Delete;
    cmd.entity = e;
    cmd.snapshot = capture(world, e);
    push(std::move(cmd));
}

void UndoStack::recordComponentAdd(World& world, Entity e,
                                   const std::string& component) {
    Command cmd;
    cmd.kind = Command::ComponentAdd;
    cmd.entity = e;
    cmd.component = component;
    cmd.after = componentState(world, e, component);
    push(std::move(cmd));
}

void UndoStack::recordComponentRemove(World& world, Entity e,
                                      const std::string& component) {
    Command cmd;
    cmd.kind = Command::ComponentRemove;
    cmd.entity = e;
    cmd.component = component;
    cmd.before = componentState(world, e, component);
    push(std::move(cmd));
}

void UndoStack::push(Command cmd) {
    undoList.push_back(std::move(cmd));
    redoList.clear();
    if (undoList.size() > MAX_DEPTH) undoList.pop_front();
    revision_++;
}

Entity UndoStack::undo(World& world) {
    if (undoList.empty()) return Entity{};
    Command cmd = std::move(undoList.back());
    undoList.pop_back();
    Entity affected = apply(cmd, world, /*forward=*/false);
    redoList.push_back(std::move(cmd));
    revision_++;
    return affected;
}

Entity UndoStack::redo(World& world) {
    if (redoList.empty()) return Entity{};
    Command cmd = std::move(redoList.back());
    redoList.pop_back();
    Entity affected = apply(cmd, world, /*forward=*/true);
    undoList.push_back(std::move(cmd));
    revision_++;
    return affected;
}

void UndoStack::clear() {
    undoList.clear();
    redoList.clear();
}

nlohmann::json UndoStack::componentState(World& world, Entity e,
                                         const std::string& component) {
    ComponentRegistry::Entry* entry = registry.find(component);
    if (!entry || !world.alive(e) || !entry->has(world, e))
        return nlohmann::json();
    nlohmann::json state;
    JsonWriteVisitor writer(state);
    entry->visit(world, e, writer);
    return state;
}

void UndoStack::applyComponentJson(World& world, Entity e,
                                   const std::string& component,
                                   const nlohmann::json& state) {
    ComponentRegistry::Entry* entry = registry.find(component);
    if (!entry || !world.alive(e) || !entry->has(world, e)) return;
    JsonReadVisitor reader(state);
    entry->visit(world, e, reader);
    // Bulk restore: post-edit hooks run with a null label (e.g. the Shape
    // hook rebuilds the mesh whatever changed).
    if (entry->onEdited) entry->onEdited(world, e, nullptr);
    // Editor objects are stationary; keep the interpolation pair in step.
    if (Transform* t = world.get<Transform>(e))
        if (auto* prev = world.get<PrevTransform>(e)) prev->value = *t;
}

Entity UndoStack::apply(Command& cmd, World& world, bool forward) {
    switch (cmd.kind) {
        case Command::FieldEdit:
            applyComponentJson(world, cmd.entity, cmd.component,
                               forward ? cmd.after : cmd.before);
            return cmd.entity;

        case Command::TransformEdit: {
            Transform* t = world.get<Transform>(cmd.entity);
            if (!t) return Entity{};
            *t = forward ? cmd.transformAfter : cmd.transformBefore;
            if (auto* prev = world.get<PrevTransform>(cmd.entity))
                prev->value = *t;
            return cmd.entity;
        }

        case Command::Create:
            if (forward) {   // redo: bring it back
                Entity fresh = recreate(world, cmd.snapshot);
                remap(cmd.entity, fresh);
                cmd.entity = fresh;
                return fresh;
            }
            // undo: snapshot the latest state, then remove it.
            if (!world.alive(cmd.entity)) return Entity{};
            cmd.snapshot = capture(world, cmd.entity);
            world.destroy(cmd.entity);
            return Entity{};

        case Command::Delete:
            if (forward) {   // redo: delete again, with a fresh snapshot
                if (!world.alive(cmd.entity)) return Entity{};
                cmd.snapshot = capture(world, cmd.entity);
                world.destroy(cmd.entity);
                return Entity{};
            }
            {
                Entity fresh = recreate(world, cmd.snapshot);
                remap(cmd.entity, fresh);
                cmd.entity = fresh;
                return fresh;
            }

        case Command::ComponentAdd: {
            ComponentRegistry::Entry* entry = registry.find(cmd.component);
            if (!entry || !world.alive(cmd.entity)) return Entity{};
            if (forward) {
                if (entry->addTo) entry->addTo(world, cmd.entity);
                applyComponentJson(world, cmd.entity, cmd.component, cmd.after);
            } else {
                cmd.after = componentState(world, cmd.entity, cmd.component);
                if (entry->removeFrom) entry->removeFrom(world, cmd.entity);
            }
            return cmd.entity;
        }

        case Command::ComponentRemove: {
            ComponentRegistry::Entry* entry = registry.find(cmd.component);
            if (!entry || !world.alive(cmd.entity)) return Entity{};
            if (forward) {
                cmd.before = componentState(world, cmd.entity, cmd.component);
                if (entry->removeFrom) entry->removeFrom(world, cmd.entity);
            } else {
                if (entry->addTo) entry->addTo(world, cmd.entity);
                applyComponentJson(world, cmd.entity, cmd.component, cmd.before);
            }
            return cmd.entity;
        }
    }
    return Entity{};
}

UndoStack::EntitySnapshot UndoStack::capture(World& world, Entity e) {
    EntitySnapshot snap;
    if (Transform* t = world.get<Transform>(e)) {
        snap.transform = *t;
        snap.hasTransform = true;
    }
    snap.hasPrev = world.has<PrevTransform>(e);
    if (SourceSpec* spec = world.get<SourceSpec>(e)) {
        snap.spec = *spec;
        snap.hasSpec = true;
    }
    if (Renderable* r = world.get<Renderable>(e)) {
        snap.renderable = *r;   // mesh handle reused; uploads outlive entities
        snap.hasRenderable = true;
    }
    if (SceneCamera* cam = world.get<SceneCamera>(e)) {
        snap.camera = *cam;
        snap.hasCamera = true;
    }
    snap.hasSpawn = world.has<PlayerSpawn>(e);
    return snap;
}

Entity UndoStack::recreate(World& world, const EntitySnapshot& snapshot) {
    // An empty snapshot means the entity was already gone when this command
    // captured (it died outside the log) — recreating would leave a ghost.
    if (!snapshot.hasTransform && !snapshot.hasSpec && !snapshot.hasRenderable &&
        !snapshot.hasCamera && !snapshot.hasSpawn)
        return Entity{};
    Entity e = world.create();
    if (snapshot.hasTransform) world.add<Transform>(e, snapshot.transform);
    if (snapshot.hasPrev) world.add<PrevTransform>(e, {snapshot.transform});
    if (snapshot.hasSpec) world.add<SourceSpec>(e, snapshot.spec);
    if (snapshot.hasRenderable) world.add<Renderable>(e, snapshot.renderable);
    if (snapshot.hasCamera) world.add<SceneCamera>(e, snapshot.camera);
    if (snapshot.hasSpawn) world.add<PlayerSpawn>(e);
    return e;
}

void UndoStack::remap(Entity from, Entity to) {
    for (std::deque<Command>* list : {&undoList, &redoList})
        for (Command& cmd : *list)
            if (cmd.entity == from) cmd.entity = to;
}

}  // namespace engine
