#ifndef RAYTRACER_ENGINE_ASSET_ROOT_H
#define RAYTRACER_ENGINE_ASSET_ROOT_H

#include <string>

namespace engine {

// Where repo-relative asset paths ("assets/levels/arena.json",
// "shaders/metal/common.metal") resolve from.
//
// EMPTY BY DEFAULT, and that is the important part: every desktop build runs
// from the repo root, so the existing relative paths already work and nothing
// changes for them. This exists for the one case where they cannot — a sandboxed
// application bundle, whose working directory is not the repo and is not
// writable. The visionOS shell sets the root to its bundle resource directory
// before Application::begin, and every path the engine opens then lands inside
// the bundle.
//
// Deliberately a root PREFIX rather than a second search list: the engine
// already has one place that answers "which paths do we search" for scripts
// (script_assets.h), and adding a competing mechanism next to it is how that
// module's own comment says the previous bug happened.

// Set the root. Pass "" to restore the default (working-directory-relative).
// A trailing slash is optional; assetPath normalises it.
void setAssetRoot(std::string root);

// The current root. Empty when unset.
const std::string& assetRoot();

// `relative` prefixed with the asset root, or returned unchanged when no root is
// set or when `relative` is already absolute. Never emits a doubled separator.
std::string assetPath(const std::string& relative);

}  // namespace engine

#endif
