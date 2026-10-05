#include "interior_occupants.h"

#include <algorithm>
#include <cmath>

namespace engine {

namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kHalfHeight = 0.9;   // the standing body's origin above its feet
constexpr double kHalfDepth = 0.13;   // its back to its middle, lying down

uint32_t mix(uint32_t h) {
    h ^= h >> 16; h *= 0x7feb352dU;
    h ^= h >> 15; h *= 0x846ca68bU;
    h ^= h >> 16;
    return h;
}

bool hasTag(const FurnitureAsset& a, const char* tag) {
    for (const std::string& t : a.tags)
        if (t == tag) return true;
    return false;
}

// The world yaw a body faces when it looks along `look` (piece space) on this piece.
Real yawOf(const Mat4& xform, const Vec3& look) {
    const Vec3 f = pieceDir(xform, look);
    return std::atan2(f.x, f.z);
}

struct Use {
    uint32_t piece;
    int verb;
    uint32_t order;   // the piece's draw, then the verb's place on it: a piece fills together
};

}  // namespace

std::vector<Occupant> planOccupants(const Interactables& set, const FurnitureLibrary& lib, const OccupantPlan& plan) {
    std::vector<Occupant> out;
    if (plan.people <= 0) return out;
    std::vector<uint32_t> held(set.pieces.size(), 0);
    auto take = [&](uint32_t pi, int vi, Occupant::Pose pose) {
        const InteractPiece& ip = set.pieces[pi];
        const FurnitureAsset* a = lib.find(static_cast<Piece>(ip.piece));
        const FurnVerb& v = a->verbs[static_cast<std::size_t>(vi)];
        Vec3 c(0, 0, 0);
        int n = 0;
        for (std::size_t k = 0; k < a->spots.size(); ++k)
            if (v.spots & (1u << k)) { c = c + a->spots[k].at; ++n; }
        if (n) c = c * (1.0 / n);
        if (pose == Occupant::Pose::Lie)   // the mattress's own spot, not the mean with the edge you sit on
            for (std::size_t k = 0; k < a->spots.size(); ++k)
                if (v.spots & (1u << k)) { c = a->spots[k].at; break; }
        Occupant o;
        o.pose = pose;
        o.piece = pi;
        o.spots = v.spots;
        const Vec3 at = piecePoint(ip.xform, c);
        // The standing body is built about its MIDDLE (y in [-0.9, 0.9], citysim buildPersonMesh); the seated one
        // about its hip on the seat.
        if (pose == Occupant::Pose::Lie) {
            // On the back along the bed, head at the pillow end (-z): the body's up turned to -z, its face to +y; its
            // middle a little toward the pillow from the mattress's own spot (the crown at the pillow), its back on
            // the mattress (half the torso's depth above it).
            o.at = ip.xform * Mat4::translate(c.x, c.y + kHalfDepth, c.z + 0.03) * Mat4::rotateX(-kPi * 0.5);
        } else if (pose == Occupant::Pose::Stand) {
            o.at = Mat4::translate(at.x, at.y + kHalfHeight, at.z) * Mat4::rotateY(yawOf(ip.xform, v.look));
        } else {
            o.at = Mat4::translate(at.x, at.y, at.z) * Mat4::rotateY(yawOf(ip.xform, v.look));
        }
        held[pi] |= v.spots;
        out.push_back(o);
    };
    auto freeNow = [&](uint32_t pi, const FurnVerb& v) {
        return (v.spots & (set.pieces[pi].taken | held[pi])) == 0;
    };

    // THE LECTURER, while anyone is in.
    if (plan.lecturer) {
        for (uint32_t pi = 0; pi < set.pieces.size() && static_cast<int>(out.size()) < plan.people; ++pi) {
            const FurnitureAsset* a = lib.find(static_cast<Piece>(set.pieces[pi].piece));
            if (!a || !hasTag(*a, "lecture")) continue;
            for (int vi = 0; vi < static_cast<int>(a->verbs.size()); ++vi)
                if (a->verbs[static_cast<std::size_t>(vi)].verb == Verb::Stand && freeNow(pi, a->verbs[static_cast<std::size_t>(vi)])) {
                    take(pi, vi, Occupant::Pose::Stand);
                    break;
                }
            if (!out.empty()) break;
        }
    }

    // Every use a body could make of the set, by kind.
    std::vector<Use> beds, seats;
    for (uint32_t pi = 0; pi < set.pieces.size(); ++pi) {
        const FurnitureAsset* a = lib.find(static_cast<Piece>(set.pieces[pi].piece));
        if (!a || a->family == "fixture") continue;   // nobody is drawn sitting on the toilet
        uint32_t pieceDraw = mix(plan.seed * 0x9E3779B9u + pi * 0x85EBCA6Bu);
        // a class on: the lecture hall's rows fill before the classrooms' chairs
        pieceDraw = plan.lecturer && hasTag(*a, "lecture") ? pieceDraw >> 1 : (pieceDraw >> 1) | 0x80000000u;
        for (int vi = 0; vi < static_cast<int>(a->verbs.size()); ++vi) {
            const FurnVerb& v = a->verbs[static_cast<std::size_t>(vi)];
            const Use u{pi, vi, (pieceDraw & 0xFFFFFF00u) | static_cast<uint32_t>(vi & 0xFF)};
            if (v.verb == Verb::Lie && a->family == "sleeping") beds.push_back(u);
            else if (v.verb == Verb::Sit) seats.push_back(u);
            else if (v.verb == Verb::Stand && !hasTag(*a, "lecture")) seats.push_back(u);   // at a lab bench: a place to work
        }
    }
    auto byOrder = [](const Use& x, const Use& y) { return x.order != y.order ? x.order < y.order : x.piece < y.piece; };
    std::sort(beds.begin(), beds.end(), byOrder);
    std::sort(seats.begin(), seats.end(), byOrder);

    if (plan.night)
        for (const Use& u : beds) {
            if (static_cast<int>(out.size()) >= plan.people) break;
            const FurnitureAsset* a = lib.find(static_cast<Piece>(set.pieces[u.piece].piece));
            if (freeNow(u.piece, a->verbs[static_cast<std::size_t>(u.verb)])) take(u.piece, u.verb, Occupant::Pose::Lie);
        }
    // Seats a piece at a time, about one in five left empty -- a lecture hall with every seat taken in order from
    // the front reads as a pattern, not a class.
    for (int pass = 0; pass < 2; ++pass)
        for (const Use& u : seats) {
            if (static_cast<int>(out.size()) >= plan.people) break;
            if (pass == 0 && mix(u.order ^ (plan.seed + 0x51u)) % 5u == 0u) continue;
            const FurnitureAsset* a = lib.find(static_cast<Piece>(set.pieces[u.piece].piece));
            const FurnVerb& v = a->verbs[static_cast<std::size_t>(u.verb)];
            if (freeNow(u.piece, v)) take(u.piece, u.verb, v.verb == Verb::Stand ? Occupant::Pose::Stand : Occupant::Pose::Sit);
        }
    return out;
}

}  // namespace engine
