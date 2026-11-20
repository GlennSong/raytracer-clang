#ifndef RAYTRACER_ENGINE_PROCGEN_MODEL_STORE_H
#define RAYTRACER_ENGINE_PROCGEN_MODEL_STORE_H

// THE MODEL STORE (#191, model step 6): where built models are kept, each under a name, so many things share one.
//
// A model is built once (a recipe is run, or files are read) and kept. Everything that shows a crane refers to the
// one model named "crane": a thousand benches are one model. This is the furniture kit's way (procgen/furniture_kit.h:
// a piece modelled once per variant and kept) for any model.
//
// The store also knows WHAT EACH MODEL WAS BUILT FROM: a key (the recipe's text, its arguments and the toolkit's
// tag, hashed together) and its inputs (each library the recipe required, each file it read, each other model it
// used, with the hash each had). A model is rebuilt when its key or any input has changed, and not otherwise. So an
// edited recipe shows on the next load without restarting, and an unedited one costs a few file hashes.
//
// A model in the store is FROZEN (ProcModel::frozen): it is shared, so nothing may change it. A recipe that wants a
// variation uses it as a piece of a new model.
//
// The store keeps models in memory. It does not write them to disk; a model that is slow to build and wanted across
// runs is a bundle's business (engine/bundle).

#include "proc_model.h"

#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

namespace engine {

// Bumped when a mesh word's output changes, so every kept model is built again (as a lots tag is for the city).
constexpr const char* kModelToolkitTag = "2026-11-19.1";

// One thing a model was built from: a file's path, or "model:<name>" for another model in the store; and its hash
// when the model was built (a file's contents; another model's signature).
struct ModelInput {
    std::string what;
    uint64_t hash = 0;
};

class ModelStore {
public:
    struct Entry {
        uint64_t key = 0;
        std::vector<ModelInput> inputs;
        // The key and every input's hash, hashed together: it changes whenever the model is built from anything
        // different, including something only one of its inputs read. This is the hash a model that USES this one
        // records for it, so a change deep down is seen all the way up.
        uint64_t signature = 0;
        std::shared_ptr<const ProcModel> model;
    };

    // The model kept under `name`, or null.
    std::shared_ptr<const ProcModel> find(const std::string& name) const;
    // What is kept under `name` (its key and inputs too). False when nothing is.
    bool entry(const std::string& name, Entry& out) const;
    // Keep `model` under `name`, in place of whatever was there. It is frozen and, if it had no name, named. Returns
    // it as everything else will see it.
    std::shared_ptr<const ProcModel> put(const std::string& name, uint64_t key, std::vector<ModelInput> inputs, std::shared_ptr<ProcModel> model);
    void forget(const std::string& name);
    void clear();
    std::vector<std::string> names() const;   // sorted
    std::size_t size() const;
    // How many times `put` has been called: a test's way to see that a second ask did not build again.
    std::size_t builds() const;

private:
    mutable std::mutex mutex_;
    std::unordered_map<std::string, Entry> entries_;
    std::size_t builds_ = 0;
};

// The process's one store.
ModelStore& modelStore();

}  // namespace engine

#endif
