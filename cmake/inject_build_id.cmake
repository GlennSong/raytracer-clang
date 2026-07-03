# Copies web/viewer.html into the build dir with __RT_BUILD__ replaced by a
# short content hash of viewer_web.wasm. The page uses the token to cache-bust
# its ?v= URLs, so browsers cache the ~1.6 MB engine between visits and a new
# deploy (changed wasm bytes) still forces a fresh download.
#
# Invoked at POST_BUILD (root CMakeLists.txt):
#   cmake -DSRC=<web/viewer.html> -DDST=<out/viewer.html> -DWASM=<out/viewer_web.wasm>
#         -P cmake/inject_build_id.cmake
if(NOT EXISTS "${WASM}")
    message(FATAL_ERROR "inject_build_id: wasm not found: ${WASM}")
endif()
file(SHA256 "${WASM}" RT_HASH)
string(SUBSTRING "${RT_HASH}" 0 12 RT_HASH)
file(READ "${SRC}" RT_HTML)
string(REPLACE "__RT_BUILD__" "${RT_HASH}" RT_HTML "${RT_HTML}")
file(WRITE "${DST}" "${RT_HTML}")
