#include "asset_root.h"

namespace engine {

namespace {

std::string& rootStorage() {
    static std::string root;
    return root;
}

}  // namespace

void setAssetRoot(std::string root) {
    // Normalise away a trailing separator so assetPath can always join with one.
    while (!root.empty() && root.back() == '/') root.pop_back();
    rootStorage() = std::move(root);
}

const std::string& assetRoot() { return rootStorage(); }

std::string assetPath(const std::string& relative) {
    const std::string& root = rootStorage();
    if (root.empty()) return relative;
    if (relative.empty()) return root;
    // An absolute path is already an answer — prefixing it would produce
    // nonsense like "/bundle/Resources//Users/glenn/scene.json". This is what
    // lets a caller pass a user-supplied path straight through.
    if (relative.front() == '/') return relative;
    return root + "/" + relative;
}

}  // namespace engine
