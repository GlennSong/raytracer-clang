#ifndef RAYTRACER_ENGINE_SPARSE_SET_H
#define RAYTRACER_ENGINE_SPARSE_SET_H

#include <vector>
#include <cstdint>
#include <cstddef>

namespace engine {

// Type-erased base so the registry can hold one pool per component type and
// drop a destroyed entity's component without knowing the component type.
class ISparseSet {
public:
    virtual ~ISparseSet() = default;
    virtual void removeIndex(uint32_t entityIndex) = 0;
    virtual bool containsIndex(uint32_t entityIndex) const = 0;
};

// Dense storage of T keyed by entity index: O(1) insert/get/remove (swap-and-
// pop) and cache-friendly dense iteration. Keyed by entity *index* only;
// generation validation is the registry's job (it strips components on destroy,
// so a recycled index starts empty).
template <typename T>
class SparseSet : public ISparseSet {
public:
    static constexpr uint32_t INVALID = 0xFFFFFFFFu;

    T& insert(uint32_t entityIndex, const T& value) {
        if (entityIndex >= sparse.size()) sparse.resize(entityIndex + 1, INVALID);
        if (sparse[entityIndex] != INVALID) {
            denseData[sparse[entityIndex]] = value;        // overwrite existing
            return denseData[sparse[entityIndex]];
        }
        sparse[entityIndex] = static_cast<uint32_t>(denseData.size());
        denseEntity.push_back(entityIndex);
        denseData.push_back(value);
        return denseData.back();
    }

    T* get(uint32_t entityIndex) {
        if (entityIndex >= sparse.size() || sparse[entityIndex] == INVALID) return nullptr;
        return &denseData[sparse[entityIndex]];
    }

    bool containsIndex(uint32_t entityIndex) const override {
        return entityIndex < sparse.size() && sparse[entityIndex] != INVALID;
    }

    void removeIndex(uint32_t entityIndex) override {
        if (entityIndex >= sparse.size() || sparse[entityIndex] == INVALID) return;
        uint32_t dense = sparse[entityIndex];
        uint32_t lastDense = static_cast<uint32_t>(denseData.size() - 1);
        uint32_t lastEntity = denseEntity[lastDense];

        denseData[dense] = denseData[lastDense];           // swap-and-pop
        denseEntity[dense] = lastEntity;
        sparse[lastEntity] = dense;

        denseData.pop_back();
        denseEntity.pop_back();
        sparse[entityIndex] = INVALID;
    }

    std::size_t size() const { return denseData.size(); }
    const std::vector<uint32_t>& entityIndices() const { return denseEntity; }

private:
    std::vector<uint32_t> sparse;       // entityIndex -> dense position (or INVALID)
    std::vector<uint32_t> denseEntity;  // dense position -> entityIndex
    std::vector<T> denseData;           // dense position -> component
};


}  // namespace engine

#endif
