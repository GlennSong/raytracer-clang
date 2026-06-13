#ifdef RT_ENABLE_IMGUI

#include "imgui_properties.h"

#include <imgui.h>
#include <algorithm>
#include <cstdio>
#include <cstring>
#include <string>

namespace engine {

namespace {

std::string scalarFormat(const FieldMeta& meta) {
    std::string fmt = "%.3f";
    if (meta.unit[0]) {
        fmt += ' ';
        fmt += meta.unit;
    }
    return fmt;
}

bool bounded(const FieldMeta& meta) {
    return meta.minValue != 0.0 || meta.maxValue != 0.0;
}

}  // namespace

bool ImGuiPropertyVisitor::scalarWidget(const FieldMeta& meta, float& value) {
    const std::string fmt = scalarFormat(meta);
    if (bounded(meta))
        return ImGui::SliderFloat(
            meta.label, &value, static_cast<float>(meta.minValue),
            static_cast<float>(meta.maxValue), fmt.c_str(),
            meta.logScale ? ImGuiSliderFlags_Logarithmic : 0);
    float speed = static_cast<float>(std::max(meta.step, Real(0.001)));
    return ImGui::DragFloat(meta.label, &value, speed, 0.0f, 0.0f, fmt.c_str());
}

void ImGuiPropertyVisitor::trackItem(bool instant, bool changed) {
    if (instant) {
        if (changed) started = finished = true;
        return;
    }
    if (ImGui::IsItemActivated()) started = true;
    if (ImGui::IsItemDeactivatedAfterEdit()) finished = true;
}

void ImGuiPropertyVisitor::field(const FieldMeta& meta, Real& value) {
    if (meta.readOnly) {
        ImGui::LabelText(meta.label, "%.5g", value);
        return;
    }
    float f = static_cast<float>(value);
    if (scalarWidget(meta, f)) {
        value = f;
        noteEdited(meta);
    }
    trackItem(false, false);
}

void ImGuiPropertyVisitor::field(const FieldMeta& meta, float& value) {
    if (meta.readOnly) {
        ImGui::LabelText(meta.label, "%.5g", static_cast<double>(value));
        return;
    }
    float f = value;
    if (scalarWidget(meta, f)) {
        value = f;
        noteEdited(meta);
    }
    trackItem(false, false);
}

void ImGuiPropertyVisitor::field(const FieldMeta& meta, Vec3& value) {
    if (meta.readOnly) {
        ImGui::LabelText(meta.label, "%.3f, %.3f, %.3f", value.x, value.y,
                         value.z);
        return;
    }
    float v[3] = {static_cast<float>(value.x), static_cast<float>(value.y),
                  static_cast<float>(value.z)};
    bool changed;
    if (meta.color) {
        changed = ImGui::ColorEdit3(meta.label, v);
    } else {
        float speed = static_cast<float>(std::max(meta.step, Real(0.001)));
        changed = ImGui::DragFloat3(meta.label, v, speed,
                                    static_cast<float>(meta.minValue),
                                    static_cast<float>(meta.maxValue));
    }
    if (changed) {
        value = Vec3(v[0], v[1], v[2]);
        noteEdited(meta);
    }
    trackItem(false, false);
}

void ImGuiPropertyVisitor::field(const FieldMeta& meta, bool& value) {
    if (meta.readOnly) {
        ImGui::LabelText(meta.label, "%s", value ? "true" : "false");
        return;
    }
    bool b = value;
    bool changed = ImGui::Checkbox(meta.label, &b);
    if (changed) {
        value = b;
        noteEdited(meta);
    }
    trackItem(true, changed);
}

void ImGuiPropertyVisitor::field(const FieldMeta& meta, std::string& value) {
    if (meta.readOnly) {
        ImGui::LabelText(meta.label, "%s", value.c_str());
        return;
    }
    if (!meta.choices.empty()) {
        bool changed = false;
        if (ImGui::BeginCombo(meta.label, value.c_str())) {
            for (const char* option : meta.choices) {
                if (ImGui::Selectable(option, value == option)) {
                    value = option;
                    noteEdited(meta);
                    changed = true;
                }
            }
            ImGui::EndCombo();
        }
        trackItem(true, changed);
        return;
    }
    char buffer[256];
    std::snprintf(buffer, sizeof(buffer), "%s", value.c_str());
    if (ImGui::InputText(meta.label, buffer, sizeof(buffer))) {
        value = buffer;
        noteEdited(meta);
    }
    trackItem(false, false);
}

void ImGuiPropertyVisitor::bitFlag(const FieldMeta& meta, uint32_t& flags,
                                   uint32_t mask) {
    bool set = (flags & mask) != 0;
    bool wasSet = set;
    field(meta, set);
    if (set != wasSet) flags = set ? (flags | mask) : (flags & ~mask);
}

}  // namespace engine

#endif  // RT_ENABLE_IMGUI
