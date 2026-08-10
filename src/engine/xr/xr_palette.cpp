#include "xr_palette.h"

namespace engine {

void XrPalette::update(const Inputs& in, Real now) {
    shownEdge_ = false;
    hiddenEdge_ = false;
    hoverChanged_ = false;

    // Anchor: the first non-busy palm facing up. The CURRENT anchor is held
    // to the loose keepAngle while everyone else must clear the tight
    // showAngle — the hysteresis that stops a palm hovering at the
    // threshold from strobing the palette.
    int anchor = -1;
    for (int h = 0; h < 2; h++) {
        if (in.handBusy[h]) continue;
        const Real angle = (h == anchor_) ? config_.keepAngle
                                          : config_.showAngle;
        if (xrPalmUp(in.palm[h], angle)) {
            anchor = h;
            break;
        }
    }

    // Linger: a palm that dipped a beat ago keeps its palette, as long as
    // the hand itself is still tracked. A real put-away outlasts it.
    if (anchor < 0 && anchor_ >= 0 && now - lastUp_ < config_.lingerSeconds &&
        in.palm[anchor_].valid && !in.handBusy[anchor_]) {
        anchor = anchor_;
    } else if (anchor >= 0) {
        lastUp_ = now;
    }

    if (anchor != anchor_) {
        if (anchor >= 0 && anchor_ < 0) shownEdge_ = true;
        if (anchor < 0 && anchor_ >= 0) hiddenEdge_ = true;
        anchor_ = anchor;
        if (hover_ != -1) hoverChanged_ = true;
        hover_ = -1;
    }
    if (anchor_ < 0) return;

    // Hover: the free hand's index fingertip against the slot centres.
    const int free = 1 - anchor_;
    int hover = -1;
    if (in.fingertipValid[free] && in.palm[anchor_].valid) {
        Real best = config_.hoverRadius;
        for (int i = 0; i < config_.itemCount; i++) {
            const Real d =
                (in.fingertip[free] - slotPosition(in.palm[anchor_], i))
                    .length();
            if (d < best) {
                best = d;
                hover = i;
            }
        }
    }
    if (hover != hover_) {
        hover_ = hover;
        hoverChanged_ = true;
    }
}

}  // namespace engine
