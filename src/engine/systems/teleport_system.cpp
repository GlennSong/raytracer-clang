#include "teleport_system.h"
#include "../components.h"
#include "../procgen/noise.h"
#include "../procgen/terrain.h"
#include "../procgen/city/city_svg.h"   // SidewalkCrossing: the map's conflict places
#include "../../log.h"
#include "../../renderer/debug_ui_clipboard.h"   // which bridge a Copy went through

#include "physics_system.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <sstream>

#ifdef RT_ENABLE_IMGUI
#include <imgui.h>
#endif

namespace engine {

namespace {
constexpr double kRadToDeg = 57.29577951308232;
constexpr double kEyeAboveGround = 1.4;   // the point-and-teleport stand height
}

TeleportPose TeleportSystem::poseHere(const FrameContext& ctx) {
    const CameraState& c = ctx.view.camera;
    Vec3 fwd = c.target - c.position;
    const Real len = fwd.length();
    if (len > 1e-9) fwd = fwd * (1.0 / len);
    TeleportPose p;
    p.position = c.position;
    p.pitch = std::asin(std::clamp(fwd.y, Real(-1), Real(1))) * kRadToDeg;
    p.yaw = std::atan2(fwd.x, -fwd.z) * kRadToDeg;
    p.hasLook = true;
    return p;
}

bool TeleportSystem::groundAt(FrameContext& ctx, double x, double z, double& y) const {
    if (physics_) {
        // The physics surface, like the T key: deck, rooftop, terrain — whatever
        // is there, from high above.
        Vec3 hit;
        if (physics_->physicsWorld().castRay(Vec3(x, 2000.0, z), Vec3(0.0, -4000.0, 0.0), hit)) {
            y = hit.y;
            return true;
        }
    }
    // No physics (the editor): the analytic terrain.
    bool found = false;
    ctx.world.each<TerrainLodConfig>([&](Entity, TerrainLodConfig& cfg) {
        if (found) return;
        const Noise noise(cfg.seed);
        y = terrainHeight(cfg.params, noise, x, z);
        found = true;
    });
    return found;
}

std::string TeleportSystem::teleport(FrameContext& ctx, const TeleportPose& poseIn, bool playerToo) {
    TeleportPose pose = poseIn;
    if (pose.groundY) {
        double gy = 0.0;
        if (!groundAt(ctx, pose.position.x, pose.position.z, gy))
            return "no ground under (" + std::to_string(pose.position.x) + ", " +
                   std::to_string(pose.position.z) + ")";
        pose.position.y = gy + kEyeAboveGround;
    }
    // The player: the physics character under host control.
    Entity player;
    if (playerToo) {
        ctx.world.each<CharacterController, ControlledBy>(
            [&](Entity e, CharacterController&, ControlledBy&) { if (!player.valid()) player = e; });
    }
    std::string what;
    if (player.valid() && physics_) {
        auto* cc = ctx.world.get<CharacterController>(player);
        if (cc && cc->characterId != INVALID_CHARACTER) {
            physics_->physicsWorld().setCharacterPosition(cc->characterId, pose.position);
            if (auto* t = ctx.world.get<Transform>(player)) {
                t->position = pose.position;
                if (auto* pt = ctx.world.get<PrevTransform>(player)) pt->value = *t;
            }
            camera_.positionLocked = true;   // re-attach: back to playing, from there
            if (pose.hasLook) {
                camera_.pitch = pose.pitch;   // the fly controller keeps degrees
                camera_.yaw = pose.yaw;
            }
            what = "player";
        }
    }
    if (what.empty()) {
        // No player (the editor, or a level without one): the fly camera, via
        // the same staged pose the socket's `camera` verb uses — CameraSystem
        // consumes it next update and detaches.
        auto& s = ctx.settings;
        s.setDouble("flyEyeX", pose.position.x);
        s.setDouble("flyEyeY", pose.position.y);
        s.setDouble("flyEyeZ", pose.position.z);
        if (pose.hasLook) {
            s.setDouble("flyPitch", pose.pitch);
            s.setDouble("flyYaw", pose.yaw);
        } else {
            const TeleportPose here = poseHere(ctx);
            s.setDouble("flyPitch", here.pitch);
            s.setDouble("flyYaw", here.yaw);
        }
        s.setDouble("cameraApply", 1.0);
        what = "camera";
    }
    LOG_INFO << "[teleport] " << what << " -> " << formatTeleportPose(pose.position, pose.pitch, pose.yaw)
             << (pose.groundY ? " (on the ground)" : "");
    return what + " -> " + formatTeleportPose(pose.position, pose.pitch, pose.yaw);
}

void TeleportSystem::loadBookmarks(FrameContext& ctx) {
    bookmarks_.clear();
    // "name|x y z pitch yaw;name|..." — names may not contain '|' or ';'.
    std::stringstream all(ctx.settings.getString("teleport.bookmarks", ""));
    std::string item;
    while (std::getline(all, item, ';')) {
        const std::size_t bar = item.find('|');
        if (bar == std::string::npos) continue;
        TeleportPose p;
        if (parseTeleportPose(item.substr(bar + 1), p) < 3) continue;
        bookmarks_.push_back({item.substr(0, bar), p.position, p.pitch, p.yaw});
    }
}

void TeleportSystem::saveBookmarks(FrameContext& ctx) {
    std::string all;
    for (const TeleportBookmark& b : bookmarks_) {
        if (!all.empty()) all += ";";
        all += b.name + "|" + formatTeleportPose(b.position, b.pitch, b.yaw);
    }
    ctx.settings.setString("teleport.bookmarks", all);
}

void TeleportSystem::onStart(FrameContext& ctx) { loadBookmarks(ctx); }

void TeleportSystem::onStop(FrameContext& ctx) {
    saveBookmarks(ctx);
    ctx.settings.save("settings.json");
}

void TeleportSystem::update(FrameContext& ctx) {
    // Socket one-shots: `teleport <pose>` moves the player (camera when none);
    // `where?` is answered from the view directly by the application.
    auto& s = ctx.settings;
    const std::string req = s.getString("teleport.request", "");
    if (!req.empty()) {
        s.setString("teleport.request", "");
        TeleportPose p;
        if (parseTeleportPose(req, p) >= 2) lastResult_ = teleport(ctx, p, true);
        else lastResult_ = "unreadable pose: " + req;
        s.setString("teleport.result", lastResult_);
    }
}

#ifdef RT_ENABLE_IMGUI
// Every Copy button goes through here so the terminal says whether the click
// registered and what ImGui handed the OS (device: "the buttons don't copy" —
// the log splits "the click never fired" from "the clipboard bridge is dead").
static std::string g_pendingCopy;
static int g_pendingCopyFrames = 0;
static void copyToClipboard(const char* text) {
    ImGui::SetClipboardText(text);
    const char* back = ImGui::GetClipboardText();
    LOG_INFO << "[teleport] clipboard <- \"" << text << "\" via " << debugUiClipboardName()
             << " (read back now: "
             << (back && *back ? (std::string(back) == text ? "same" : "DIFFERENT") : "EMPTY")
             << "; window focus " << (ImGui::GetIO().AppFocusLost ? "LOST" : "held") << ")";
    // The compositor answers LATER: a Wayland set_selection it refuses comes
    // back as a cancel a frame or two on, and only then does a read show the
    // old text again. So read once more after a few frames.
    g_pendingCopy = text;
    g_pendingCopyFrames = 3;
}
#endif

void TeleportSystem::render(FrameContext& ctx) {
#ifdef RT_ENABLE_IMGUI
    if (ImGui::GetCurrentContext() == nullptr) return;
    if (g_pendingCopyFrames > 0 && --g_pendingCopyFrames == 0) {
        const char* back = ImGui::GetClipboardText();
        const bool same = back && g_pendingCopy == back;
        LOG_INFO << "[teleport] clipboard 3 frames later: "
                 << (same ? "same — the compositor kept the copy"
                          : "DIFFERENT — the compositor refused the copy (a Wayland client "
                            "without keyboard focus cannot set the selection)");
    }
    if (!ctx.debugOverlayActive) return;
    ImGui::Begin("Debug");
    // `set teleport.open 1` (socket) unfolds the section once — a probe can
    // shoot the panel; a header cannot be clicked from outside.
    if (ctx.settings.getDouble("teleport.open", 0.0) > 0.5) {
        ctx.settings.setDouble("teleport.open", 0.0);
        ImGui::SetNextItemOpen(true);
    }
    if (ImGui::CollapsingHeader("Teleport")) {
        const TeleportPose here = poseHere(ctx);
        const std::string pose = formatTeleportPose(here.position, here.pitch, here.yaw);
        ImGui::TextDisabled("pose = x y z pitch yaw (metres, degrees; yaw 0 = north/-z)");
        // A READ-ONLY text field, not a label: plain ImGui::Text cannot be
        // selected, so "copy the coordinates out of imgui" needs a field —
        // click, Ctrl+A, Ctrl+C works even where the Copy buttons' OS
        // clipboard bridge does not.
        {
            char hereBuf[128];
            std::snprintf(hereBuf, sizeof(hereBuf), "%s", pose.c_str());
            ImGui::SetNextItemWidth(360.0f);
            ImGui::InputText("##here", hereBuf, sizeof(hereBuf), ImGuiInputTextFlags_ReadOnly);
            ImGui::SameLine();
            ImGui::TextDisabled("(select + Ctrl+C)");
        }
        {
            bool third = ctx.settings.getBool("playerThirdPerson", false);
            if (ImGui::Checkbox("Third person (V on foot)", &third))
                ctx.settings.setDouble("player.setThirdPerson", third ? 1.0 : 0.0);
            ImGui::SameLine();
            ImGui::TextDisabled("persisted in settings.json as playerThirdPerson");
        }
        if (ImGui::Button("Copy pose")) copyToClipboard(pose.c_str());
        ImGui::SameLine();
        if (ImGui::Button("Copy xyz")) {
            char b[96];
            std::snprintf(b, sizeof(b), "%.2f %.2f %.2f", here.position.x, here.position.y, here.position.z);
            copyToClipboard(b);
        }
        ImGui::SameLine();
        if (ImGui::Button("Copy `teleport` cmd")) copyToClipboard(("teleport " + pose).c_str());

        ImGui::Separator();
        ImGui::InputTextWithHint("##paste", "paste: x y z [pitch yaw]  or  x z (on the ground)",
                                 pasteBuf_, sizeof(pasteBuf_));
        ImGui::SameLine();
        if (ImGui::Button("Paste")) {
            const char* clip = ImGui::GetClipboardText();
            if (clip) std::snprintf(pasteBuf_, sizeof(pasteBuf_), "%s", clip);
            LOG_INFO << "[teleport] clipboard -> \"" << (clip ? clip : "") << "\""
                     << (clip && *clip ? "" : " (EMPTY: nothing on the OS clipboard, or the bridge is dead)");
        }
        TeleportPose target;
        const int n = parseTeleportPose(pasteBuf_, target);
        ImGui::BeginDisabled(n < 2);
        if (ImGui::Button("Go (player)")) lastResult_ = teleport(ctx, target, true);
        ImGui::SameLine();
        if (ImGui::Button("Go (camera only)")) lastResult_ = teleport(ctx, target, false);
        ImGui::EndDisabled();
        if (n >= 2) {
            ImGui::SameLine();
            ImGui::TextDisabled(n == 2 ? "x z on the ground" : n >= 5 ? "x y z pitch yaw" : "x y z");
        }
        if (!lastResult_.empty()) ImGui::TextWrapped("%s", lastResult_.c_str());

        // Bookmarks.
        ImGui::Separator();
        ImGui::InputTextWithHint("##bmname", "bookmark name", nameBuf_, sizeof(nameBuf_));
        ImGui::SameLine();
        if (ImGui::Button("Add here")) {
            std::string name = nameBuf_;
            if (name.empty()) name = "spot " + std::to_string(bookmarks_.size() + 1);
            for (char& c : name) if (c == '|' || c == ';') c = ' ';
            bookmarks_.push_back({name, here.position, here.pitch, here.yaw});
            saveBookmarks(ctx);
            nameBuf_[0] = 0;
        }
        for (std::size_t i = 0; i < bookmarks_.size(); ++i) {
            TeleportBookmark& b = bookmarks_[i];
            ImGui::PushID(static_cast<int>(i));
            if (ImGui::SmallButton("Go")) {
                TeleportPose p;
                p.position = b.position; p.pitch = b.pitch; p.yaw = b.yaw; p.hasLook = true;
                lastResult_ = teleport(ctx, p, true);
            }
            ImGui::SameLine();
            if (ImGui::SmallButton("Copy"))
                copyToClipboard(formatTeleportPose(b.position, b.pitch, b.yaw).c_str());
            ImGui::SameLine();
            const bool del = ImGui::SmallButton("x");
            ImGui::SameLine();
            ImGui::Text("%s  (%.0f, %.0f, %.0f)", b.name.c_str(), b.position.x, b.position.y, b.position.z);
            ImGui::PopID();
            if (del) { bookmarks_.erase(bookmarks_.begin() + static_cast<long>(i)); saveBookmarks(ctx); break; }
        }

        // The city map's conflict places (the sidewalk-on-asphalt census).
        const CityMap* map = nullptr;
        ctx.world.each<CityMap>([&](Entity, CityMap& m) { if (!map) map = &m; });
        if (map && map->conflicts && !map->conflicts->empty()) {
            ImGui::Separator();
            if (ImGui::TreeNode("conflicts", "City map conflicts (%zu) — sidewalk band on asphalt",
                                map->conflicts->size())) {
                int idx = 1;
                for (const SidewalkCrossing& c : *map->conflicts) {
                    ImGui::PushID(idx);
                    if (ImGui::SmallButton("Go")) {
                        TeleportPose p;
                        p.position = Vec3(c.pos.x, 0.0, c.pos.y);
                        p.groundY = true;
                        lastResult_ = teleport(ctx, p, true);
                    }
                    ImGui::SameLine();
                    if (ImGui::SmallButton("Copy")) {
                        char b[64];
                        std::snprintf(b, sizeof(b), "%.1f %.1f", c.pos.x, c.pos.y);
                        copyToClipboard(b);
                    }
                    ImGui::SameLine();
                    char row[96];
                    std::snprintf(row, sizeof(row), "%.1f %.1f", c.pos.x, c.pos.y);
                    ImGui::SetNextItemWidth(130.0f);
                    ImGui::InputText("##xz", row, sizeof(row), ImGuiInputTextFlags_ReadOnly);
                    ImGui::SameLine();
                    ImGui::Text("#%d  %.1f m deep  %d m span", idx, c.depth, static_cast<int>(c.spanMetres));
                    ImGui::PopID();
                    if (++idx > 40) { ImGui::TextDisabled("... and more in the SVG"); break; }
                }
                ImGui::TreePop();
            }
        }
    }
    ImGui::End();
#else
    (void)ctx;
#endif
}

}  // namespace engine
