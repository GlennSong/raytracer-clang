#ifndef RAYTRACER_ENGINE_BUNDLE_BUNDLE_H
#define RAYTRACER_ENGINE_BUNDLE_BUNDLE_H

// engine::bundle — one-file, content- and cell-addressed level bundles (ADR-0084).
//
// A bundle is `level.bundle`: a 64-byte header, 64-byte-aligned binary SECTIONS appended in order, then the
// table of contents as JSON (the manifest, with a "sections" array of name/offset/size/fnv). A reader maps
// the file once and hands out views into it, so a whole-level load reads every section and a streaming
// loader touches only the cells it needs — same file, same reader. Sections are named by path convention:
//     <producer>/<global>                  e.g. city/e0/ground, city/e0/roads/nav
//     <producer>/cell/<cx>_<cz>/<name>     e.g. city/e0/cell/12_-3/mesh/asphalt
// The bundle knows nothing about what a section holds; codecs (codecs.h) do, each behind its own magic and
// version. Producers, keys and the bake orchestration live in bake.h.
//
// Compatibility (four layers, see the ADR): the header's format major, endianness marker and Real size are
// checked hard; each section's codec version is checked by its reader; a producer's content key decides
// whether its sections are current; the engine identity and timestamp are recorded, and enforced only on
// request.

#include "engine/bundle/binary_stream.h"
#include <nlohmann/json.hpp>
#include <cstdint>
#include <cstdio>
#include <memory>
#include <string>
#include <vector>

namespace engine {
namespace bundle {

constexpr uint32_t kFormatMajor = 1;
constexpr uint32_t kFormatMinor = 0;
constexpr const char* kBundleFile = "level.bundle";
constexpr const char* kManifestFile = "manifest.json";
constexpr size_t kHeaderBytes = 64;
constexpr size_t kSectionAlign = 64;

// FNV-1a, the repo's content hash (terrain_field.cpp's erosion cache uses the same constants).
constexpr uint64_t kFnvOffset = 1469598103934665603ULL;
uint64_t fnv1a(const void* data, size_t size, uint64_t seed = kFnvOffset);
uint64_t fnv1aStr(const std::string& s, uint64_t seed = kFnvOffset);
// Chains the file's bytes onto `key`; false when the file cannot be read. bytes/mtime are informational.
bool fnv1aFile(const std::string& path, uint64_t& key, uint64_t* bytes = nullptr, int64_t* mtime = nullptr);
std::string hex16(uint64_t v);

// Who built a bundle: the generated build identity (build_info.h) plus the Real width.
struct EngineIdentity { std::string version, buildType, compiler, platform; int realBytes = 0; };
EngineIdentity engineIdentity();
nlohmann::json engineIdentityJson();
std::string isoNowUtc();

struct SectionInfo { std::string name; uint64_t offset = 0, size = 0, fnv = 0; };

// Writes sections as they arrive — to a file or to memory — and finishes with the manifest.
class BundleWriter {
public:
    BundleWriter() = default;
    ~BundleWriter();
    BundleWriter(const BundleWriter&) = delete;
    BundleWriter& operator=(const BundleWriter&) = delete;
    bool openFile(const std::string& path, std::string* err);
    void openMemory();
    bool isOpen() const { return f_ != nullptr || mem_ != nullptr; }
    bool add(const std::string& name, const void* data, size_t size);
    bool add(const std::string& name, const std::vector<uint8_t>& bytes) { return add(name, bytes.data(), bytes.size()); }
    bool addJson(const std::string& name, const nlohmann::json& j);
    // Appends the table of contents and patches the header. `manifest` gains "format" and "sections" in
    // place, so a manifest.json written from it afterwards is byte-for-byte the table of contents.
    bool finish(nlohmann::json& manifest, std::string* err);
    const std::vector<SectionInfo>& sections() const { return sections_; }
    std::shared_ptr<std::vector<uint8_t>> memory() const { return mem_; }   // memory mode, after finish()
    uint64_t bytesWritten() const { return pos_; }
    // The bytes of a section written earlier in THIS bundle (the last of that name): how one producer reads
    // another's products — built a moment ago or copied forward — before the bundle is finished.
    bool readBack(const std::string& name, std::vector<uint8_t>& out) const;

private:
    bool write(const void* p, size_t n);
    bool pad();
    FILE* f_ = nullptr;
    std::string path_;
    std::shared_ptr<std::vector<uint8_t>> mem_;
    uint64_t pos_ = 0;
    std::vector<SectionInfo> sections_;
    bool finished_ = false;
};

class Bundle {
public:
    struct View { const uint8_t* data = nullptr; size_t size = 0; bool empty() const { return data == nullptr; } };
    static std::unique_ptr<Bundle> open(const std::string& path, std::string* err);
    static std::unique_ptr<Bundle> fromMemory(std::shared_ptr<std::vector<uint8_t>> bytes, std::string* err);
    ~Bundle();
    Bundle(const Bundle&) = delete;
    Bundle& operator=(const Bundle&) = delete;

    bool has(const std::string& name) const;
    View section(const std::string& name) const;                    // empty view when absent
    std::vector<std::string> sections(const std::string& prefix) const;   // sorted
    nlohmann::json json(const std::string& name) const;             // null when absent or malformed
    bool checkSection(const std::string& name) const;              // the bytes still hash to the TOC's fnv
    const nlohmann::json& manifest() const { return manifest_; }
    const std::vector<SectionInfo>& sectionTable() const { return sections_; }
    const std::string& path() const { return path_; }
    size_t bytes() const { return size_; }
    bool mapped() const { return map_ != nullptr; }

private:
    Bundle() = default;
    bool parse(std::string* err);
    std::string path_;
    const uint8_t* base_ = nullptr;
    size_t size_ = 0;
    void* map_ = nullptr;
    std::shared_ptr<std::vector<uint8_t>> mem_;
    nlohmann::json manifest_;
    std::vector<SectionInfo> sections_;
};

}  // namespace bundle
}  // namespace engine

#endif
