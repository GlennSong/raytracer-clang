// The palm palette's brain, pure: which hand anchors it, when it shows and
// hides, where its item slots sit, and which slot the free hand hovers.
// Extracted from HandInteractionSystem so the parts that took on-device
// tuning to get right — the show/keep angle hysteresis and the linger that
// stopped the palette strobing (device sessions three and four) — are
// host-tested with synthetic poses instead of re-verified in a headset per
// change. The system feeds poses in and renders what comes out.
//
// Space-agnostic: positions come back in whatever space the palm poses and
// fingertips are provided in (the sandbox passes world space).

#ifndef ENGINE_XR_PALETTE_H
#define ENGINE_XR_PALETTE_H

#include <algorithm>

#include "../../rt_math.h"
#include "xr_gestures.h"

namespace engine {

class XrPalette {
public:
    struct Config {
        int itemCount = 0;
        Real slotSpacing = 0.09;     // metres between slot centres
        Real lift = 0.09;            // above the palm, along its normal
        Real hoverRadius = 0.055;    // fingertip-to-slot pick distance
        Real showAngle = 0.7;        // rad from +Y to OPEN (tight)
        Real keepAngle = 1.3;        // rad to STAY open (loose) — hysteresis
        Real lingerSeconds = 0.6;    // grace after the palm dips
        int maxPerRow = 5;           // wrap into rows past this (across palm)
        Real rowSpacing = 0.085;     // metres between row centres
    };

    XrPalette() = default;   // itemCount 0: valid, shows nothing
    explicit XrPalette(const Config& config) : config_(config) {}

    struct Inputs {
        XrPalmPose palm[2];        // per hand; invalid = untracked
        bool handBusy[2] = {false, false};   // carrying an object
        bool fingertipValid[2] = {false, false};
        Vec3 fingertip[2];         // index tips, same space as palms
    };

    void update(const Inputs& in, Real now);

    int anchorHand() const { return anchor_; }   // -1 = hidden
    int hoverItem() const { return hover_; }     // -1 = none
    bool shownEdge() const { return shownEdge_; }
    bool hiddenEdge() const { return hiddenEdge_; }
    bool hoverChanged() const { return hoverChanged_; }

    // Rows run along palm.fingers — wrist toward fingertips, i.e. along
    // the forearm (device feedback: across the palm read as sideways) —
    // wrapping onto extra rows across the palm past maxPerRow, each row
    // centred on its own item count.
    Vec3 slotPosition(const XrPalmPose& palm, int slot) const {
        const int rows =
            (config_.itemCount + config_.maxPerRow - 1) / config_.maxPerRow;
        const int row = slot / config_.maxPerRow;
        const int col = slot % config_.maxPerRow;
        const int inRow = std::min(config_.maxPerRow,
                                   config_.itemCount - row * config_.maxPerRow);
        return palm.position + palm.normal * config_.lift +
               palm.fingers *
                   ((col - (inRow - 1) * 0.5) * config_.slotSpacing) +
               palm.across * ((row - (rows - 1) * 0.5) * config_.rowSpacing);
    }

private:
    Config config_;
    int anchor_ = -1;
    int hover_ = -1;
    Real lastUp_ = -1;
    bool shownEdge_ = false;
    bool hiddenEdge_ = false;
    bool hoverChanged_ = false;
};

}  // namespace engine

#endif  // ENGINE_XR_PALETTE_H
