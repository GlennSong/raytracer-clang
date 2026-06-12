#ifndef RAYTRACER_ENGINE_PROPERTY_JSON_H
#define RAYTRACER_ENGINE_PROPERTY_JSON_H

#include "properties.h"
#include <nlohmann/json.hpp>

namespace engine {

// Property-layer JSON visitors: serialize/deserialize any described component
// without naming its fields — the same walk the inspectors use, so UI and
// persistence cannot drift. Keys are the FieldMeta labels.
class JsonWriteVisitor : public PropertyVisitor {
public:
    explicit JsonWriteVisitor(nlohmann::json& out) : out(out) {}

    void field(const FieldMeta& meta, Real& value) override;
    void field(const FieldMeta& meta, float& value) override;
    void field(const FieldMeta& meta, Vec3& value) override;
    void field(const FieldMeta& meta, bool& value) override;
    void field(const FieldMeta& meta, std::string& value) override;
    void bitFlag(const FieldMeta& meta, uint32_t& flags, uint32_t mask) override;

private:
    nlohmann::json& out;
};

// Reads described fields back; missing keys leave values untouched and
// read-only fields are never written (they're derived/owned elsewhere).
class JsonReadVisitor : public PropertyVisitor {
public:
    explicit JsonReadVisitor(const nlohmann::json& in) : in(in) {}

    void field(const FieldMeta& meta, Real& value) override;
    void field(const FieldMeta& meta, float& value) override;
    void field(const FieldMeta& meta, Vec3& value) override;
    void field(const FieldMeta& meta, bool& value) override;
    void field(const FieldMeta& meta, std::string& value) override;
    void bitFlag(const FieldMeta& meta, uint32_t& flags, uint32_t mask) override;

private:
    const nlohmann::json& in;
};

}  // namespace engine

#endif
