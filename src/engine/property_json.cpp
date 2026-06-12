#include "property_json.h"

#include <nlohmann/json.hpp>

using json = nlohmann::json;

namespace engine {

void JsonWriteVisitor::field(const FieldMeta& meta, Real& value) {
    out[meta.label] = value;
}
void JsonWriteVisitor::field(const FieldMeta& meta, float& value) {
    out[meta.label] = value;
}
void JsonWriteVisitor::field(const FieldMeta& meta, Vec3& value) {
    out[meta.label] = json::array({value.x, value.y, value.z});
}
void JsonWriteVisitor::field(const FieldMeta& meta, bool& value) {
    out[meta.label] = value;
}
void JsonWriteVisitor::field(const FieldMeta& meta, std::string& value) {
    out[meta.label] = value;
}
void JsonWriteVisitor::bitFlag(const FieldMeta& meta, uint32_t& flags,
                               uint32_t mask) {
    out[meta.label] = (flags & mask) != 0;
}

void JsonReadVisitor::field(const FieldMeta& meta, Real& value) {
    if (meta.readOnly || !in.contains(meta.label)) return;
    value = in[meta.label].get<Real>();
}
void JsonReadVisitor::field(const FieldMeta& meta, float& value) {
    if (meta.readOnly || !in.contains(meta.label)) return;
    value = in[meta.label].get<float>();
}
void JsonReadVisitor::field(const FieldMeta& meta, Vec3& value) {
    if (meta.readOnly || !in.contains(meta.label)) return;
    const auto& a = in[meta.label];
    if (a.is_array() && a.size() == 3)
        value = Vec3(a[0].get<Real>(), a[1].get<Real>(), a[2].get<Real>());
}
void JsonReadVisitor::field(const FieldMeta& meta, bool& value) {
    if (meta.readOnly || !in.contains(meta.label)) return;
    value = in[meta.label].get<bool>();
}
void JsonReadVisitor::field(const FieldMeta& meta, std::string& value) {
    if (meta.readOnly || !in.contains(meta.label)) return;
    value = in[meta.label].get<std::string>();
}
void JsonReadVisitor::bitFlag(const FieldMeta& meta, uint32_t& flags,
                              uint32_t mask) {
    if (meta.readOnly || !in.contains(meta.label)) return;
    flags = in[meta.label].get<bool>() ? (flags | mask) : (flags & ~mask);
}

}  // namespace engine
