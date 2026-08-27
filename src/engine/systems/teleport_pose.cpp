#include "teleport_pose.h"

#include <cstdio>
#include <cstdlib>
#include <vector>

namespace engine {

int parseTeleportPose(const std::string& text, TeleportPose& out) {
    std::vector<double> nums;
    std::size_t i = 0;
    while (i < text.size() && nums.size() < 5) {
        const char c = text[i];
        if ((c >= '0' && c <= '9') || c == '-' || c == '+' || c == '.') {
            const char* start = text.c_str() + i;
            char* end = nullptr;
            const double v = std::strtod(start, &end);
            if (end != start) {
                nums.push_back(v);
                i += static_cast<std::size_t>(end - start);
                continue;
            }
        }
        ++i;
    }
    if (nums.size() < 2) return 0;
    out = TeleportPose{};
    if (nums.size() == 2) {
        out.position = Vec3(nums[0], 0.0, nums[1]);
        out.groundY = true;
        return 2;
    }
    out.position = Vec3(nums[0], nums[1], nums[2]);
    if (nums.size() >= 4) { out.pitch = nums[3]; out.hasLook = true; }
    if (nums.size() >= 5) out.yaw = nums[4];
    return static_cast<int>(nums.size());
}

std::string formatTeleportPose(const Vec3& p, double pitch, double yaw) {
    char buf[128];
    std::snprintf(buf, sizeof(buf), "%.2f %.2f %.2f %.2f %.2f", p.x, p.y, p.z, pitch, yaw);
    return buf;
}

}  // namespace engine
