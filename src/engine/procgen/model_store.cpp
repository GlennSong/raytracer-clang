// The model store (#191, model step 6). See model_store.h.
#include "model_store.h"

#include "../bundle/bundle.h"   // fnv1a

#include <algorithm>

namespace engine {

std::shared_ptr<const ProcModel> ModelStore::find(const std::string& name) const {
    const std::lock_guard<std::mutex> lock(mutex_);
    const std::unordered_map<std::string, Entry>::const_iterator it = entries_.find(name);
    return it == entries_.end() ? nullptr : it->second.model;
}

bool ModelStore::entry(const std::string& name, Entry& out) const {
    const std::lock_guard<std::mutex> lock(mutex_);
    const std::unordered_map<std::string, Entry>::const_iterator it = entries_.find(name);
    if (it == entries_.end()) return false;
    out = it->second;
    return true;
}

std::shared_ptr<const ProcModel> ModelStore::put(const std::string& name, uint64_t key, std::vector<ModelInput> inputs, std::shared_ptr<ProcModel> model) {
    if (model == nullptr) return nullptr;
    if (model->name.empty()) model->name = name;
    model->frozen = true;
    const std::lock_guard<std::mutex> lock(mutex_);
    Entry& e = entries_[name];
    e.key = key;
    e.inputs = std::move(inputs);
    // the repo's FNV-1a (bundle.h) over the key and each input's hash, in order
    uint64_t sig = bundle::fnv1a(&key, sizeof key);
    for (const ModelInput& in : e.inputs) sig = bundle::fnv1a(&in.hash, sizeof in.hash, sig);
    e.signature = sig;
    e.model = std::move(model);
    ++builds_;
    return e.model;
}

void ModelStore::forget(const std::string& name) {
    const std::lock_guard<std::mutex> lock(mutex_);
    entries_.erase(name);
}

void ModelStore::clear() {
    const std::lock_guard<std::mutex> lock(mutex_);
    entries_.clear();
}

std::vector<std::string> ModelStore::names() const {
    const std::lock_guard<std::mutex> lock(mutex_);
    std::vector<std::string> out;
    for (const std::pair<const std::string, Entry>& e : entries_) out.push_back(e.first);
    std::sort(out.begin(), out.end());
    return out;
}

std::size_t ModelStore::size() const {
    const std::lock_guard<std::mutex> lock(mutex_);
    return entries_.size();
}

std::size_t ModelStore::builds() const {
    const std::lock_guard<std::mutex> lock(mutex_);
    return builds_;
}

ModelStore& modelStore() {
    static ModelStore store;
    return store;
}

}  // namespace engine
