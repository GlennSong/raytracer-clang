#ifndef RAYTRACER_ENGINE_WATCH_SYSTEM_H
#define RAYTRACER_ENGINE_WATCH_SYSTEM_H

#include "../system.h"
#include "../ui/ui_text.h"

namespace engine {

// THE WATCH (ADR-0251; Glenn: "a diegetic piece of UI ... you have a smart watch ... push a button to bring your arm
// closer up to your face"). Hold Q, or down on the D-pad, and the left wrist comes up: a compass that turns as you
// do, the waypoint (set on the map) a gold dot round its edge with how far it is in the middle -- or the time,
// with no waypoint set. Not while both hands are busy (a wall, a ladder, the bars, the water).
//
// First person, the dial is drawn onto the watch's face where PlayerArmsSystem put it (a lit screen: the game UI
// layer, after the tonemap, mapped through the camera onto the face). Third person, the same dial as a panel by the
// character.
class WatchSystem : public System {
public:
    void onStart(FrameContext& ctx) override;
    void update(FrameContext& ctx) override;
    void render(FrameContext& ctx) override;
    void onStop(FrameContext& ctx) override { ui_.release(ctx.renderer); }

    // THE DIAL in its own square of `size` px (0..size, y down): what it shows, from the heading (degrees,
    // clockwise from north = -z), the waypoint's bearing and distance (no waypoint: distance < 0) and the clock
    // (hours; < 0: none). Public for tests.
    struct Face {
        Real headingDeg = 0;
        Real bearingDeg = 0;     // the waypoint's, the same way round
        Real distance = -1;      // m
        Real clockHours = -1;
    };
    static std::string distanceText(Real metres);
    static std::string clockText(Real hours);
    void drawDial(Renderer& r, std::vector<Renderer::UiQuad>& out, float size, const Face& f);

private:
    UiText ui_;
};

}  // namespace engine

#endif
