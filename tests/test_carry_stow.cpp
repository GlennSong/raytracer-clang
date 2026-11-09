// WHERE A THING GOES IN A CAR (ADR-0358): laid flat, beside or on what is there, and refused when it will not fit.

#include "test_framework.h"
#include "../src/engine/carry_stow.h"

using namespace engine;

TEST_CASE(a_door_is_laid_flat_in_a_pickup_bed) {
    const Vec3 lo(-0.75, 0.0, -2.6), hi(0.75, 0.5, -0.7);   // a bed: 1.5 wide, 1.9 long, 0.5 deep
    Vec3 at;
    Quat turn;
    // a door: 1.1 long, 0.9 high, 0.06 thick -- stood up it would stick out; laid on its face it fits
    CHECK(stowSlot(lo, hi, Vec3(0.55, 0.45, 0.03), {}, Vec3(0, 0, -2.6), at, turn));
    const Vec3 h = turnedHalf(turn, Vec3(0.55, 0.45, 0.03));
    CHECK(h.y < 0.05);                                    // flat
    CHECK(at.y - h.y >= lo.y - 1e-6);                     // on the floor...
    CHECK(at.x - h.x >= lo.x - 1e-6 && at.x + h.x <= hi.x + 1e-6);
    CHECK(at.z - h.z >= lo.z - 1e-6 && at.z + h.z <= hi.z + 1e-6);   // ...and inside the bed
    CHECK(at.z < -1.6);                                   // at the tailgate end, where the player stands
}

TEST_CASE(boxes_fill_a_boot_then_stack_then_it_is_full) {
    const Vec3 lo(-0.6, 0.0, -2.2), hi(0.6, 0.5, -1.4);   // a saloon's boot: 1.2 x 0.5 x 0.8
    const Vec3 half(0.25, 0.18, 0.2);                     // a 0.5 x 0.36 x 0.4 box
    std::vector<StowBox> in;
    int put = 0;
    for (int i = 0; i < 20; ++i) {
        Vec3 at;
        Quat turn;
        if (!stowSlot(lo, hi, half, in, Vec3(0, 0, -2.2), at, turn)) break;
        const Vec3 h = turnedHalf(turn, half);
        for (const StowBox& b : in)   // never inside another
            CHECK(std::fabs(at.x - b.centre.x) >= h.x + b.half.x - 0.02 || std::fabs(at.z - b.centre.z) >= h.z + b.half.z - 0.02 ||
                  std::fabs(at.y - b.centre.y) >= h.y + b.half.y - 0.02);
        in.push_back({at, h});
        ++put;
    }
    std::printf("    [stow] %d boxes in a 1.2 x 0.5 x 0.8 boot\n", put);
    CHECK(put >= 4 && put < 20);   // several go in; the boot fills
}

TEST_CASE(a_thing_bigger_than_the_space_is_refused) {
    Vec3 at;
    Quat turn;
    CHECK(!stowSlot(Vec3(-0.5, 0, -1), Vec3(0.5, 0.4, 0), Vec3(0.8, 0.5, 0.6), {}, Vec3(0, 0, 0), at, turn));
}
