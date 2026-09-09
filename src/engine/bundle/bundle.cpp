#include "engine/bundle/bundle.h"

#include "engine/build_info.h"
#include "rt_math.h"

#include <algorithm>
#include <chrono>
#include <cstring>
#include <ctime>
#include <filesystem>
#include <fstream>

#ifndef _WIN32
#include <fcntl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>
#endif

namespace engine {
namespace bundle {

namespace {
constexpr char kMagic[4] = {'R', 'T', 'B', 'N'};
constexpr uint32_t kEndianMarker = 0x01020304u;

// Header (64 bytes, little-endian): magic[4] u32 major u32 minor u32 endian u32 realBytes u32 reserved
// u64 tocOffset u64 tocSize u64 sectionCount u64 reserved u64 reserved.
std::vector<uint8_t> headerBytes(uint64_t tocOffset, uint64_t tocSize, uint64_t sectionCount) {
    BinWriter w;
    w.append(kMagic, 4); w.put<uint32_t>(kFormatMajor); w.put<uint32_t>(kFormatMinor); w.put<uint32_t>(kEndianMarker);
    w.put<uint32_t>(static_cast<uint32_t>(sizeof(Real))); w.put<uint32_t>(0);
    w.put<uint64_t>(tocOffset); w.put<uint64_t>(tocSize); w.put<uint64_t>(sectionCount); w.put<uint64_t>(0); w.put<uint64_t>(0);
    w.bytes.resize(kHeaderBytes, 0);
    return w.bytes;
}
}  // namespace

uint64_t fnv1a(const void* data, size_t size, uint64_t seed) {
    const uint8_t* p = static_cast<const uint8_t*>(data); uint64_t h = seed;
    for (size_t i = 0; i < size; ++i) { h ^= p[i]; h *= 1099511628211ULL; }
    return h;
}
uint64_t fnv1aStr(const std::string& s, uint64_t seed) { return fnv1a(s.data(), s.size(), seed); }

bool fnv1aFile(const std::string& path, uint64_t& key, uint64_t* bytes, int64_t* mtime) {
    FILE* f = std::fopen(path.c_str(), "rb"); if (!f) return false;
    std::vector<uint8_t> buf(1 << 16); uint64_t total = 0;
    for (;;) { const size_t n = std::fread(buf.data(), 1, buf.size(), f); if (n == 0) break; key = fnv1a(buf.data(), n, key); total += n; }
    std::fclose(f);
    if (bytes) *bytes = total;
    if (mtime) {
        std::error_code ec; const auto t = std::filesystem::last_write_time(path, ec);
        *mtime = ec ? 0 : static_cast<int64_t>(std::chrono::duration_cast<std::chrono::seconds>(t.time_since_epoch()).count());
    }
    return true;
}

std::string hex16(uint64_t v) { char b[17]; std::snprintf(b, sizeof(b), "%016llx", static_cast<unsigned long long>(v)); return b; }

EngineIdentity engineIdentity() {
    EngineIdentity e; e.version = RT_BUILD_GIT_DESCRIBE; e.buildType = RT_BUILD_TYPE; e.compiler = RT_BUILD_COMPILER; e.platform = RT_BUILD_PLATFORM; e.realBytes = static_cast<int>(sizeof(Real));
    return e;
}
nlohmann::json engineIdentityJson() {
    const EngineIdentity e = engineIdentity();
    return {{"version", e.version}, {"buildType", e.buildType}, {"compiler", e.compiler}, {"platform", e.platform}, {"real", e.realBytes}};
}
std::string isoNowUtc() {
    const std::time_t t = std::time(nullptr); std::tm tm{};
#ifdef _WIN32
    gmtime_s(&tm, &t);
#else
    gmtime_r(&t, &tm);
#endif
    char b[32]; std::strftime(b, sizeof(b), "%Y-%m-%dT%H:%M:%SZ", &tm); return b;
}

// ---- writer -------------------------------------------------------------------------------------------

BundleWriter::~BundleWriter() { if (f_) std::fclose(f_); }

bool BundleWriter::openFile(const std::string& path, std::string* err) {
    f_ = std::fopen(path.c_str(), "wb");
    if (!f_) { if (err) *err = "cannot create " + path; return false; }
    path_ = path;
    const std::vector<uint8_t> h = headerBytes(0, 0, 0);
    return write(h.data(), h.size());
}
void BundleWriter::openMemory() {
    mem_ = std::make_shared<std::vector<uint8_t>>(); const std::vector<uint8_t> h = headerBytes(0, 0, 0); write(h.data(), h.size());
}
bool BundleWriter::write(const void* p, size_t n) {
    if (n == 0) return true;
    if (f_) { if (std::fwrite(p, 1, n, f_) != n) return false; }
    else if (mem_) { const size_t o = mem_->size(); mem_->resize(o + n); std::memcpy(mem_->data() + o, p, n); }
    else return false;
    pos_ += n; return true;
}
bool BundleWriter::pad() {
    const uint64_t r = pos_ % kSectionAlign; if (r == 0) return true;
    static const uint8_t zeros[kSectionAlign] = {0}; return write(zeros, static_cast<size_t>(kSectionAlign - r));
}
bool BundleWriter::add(const std::string& name, const void* data, size_t size) {
    if (finished_ || !isOpen() || name.empty()) return false;
    if (!pad()) return false;
    SectionInfo s; s.name = name; s.offset = pos_; s.size = size; s.fnv = fnv1a(data, size);
    if (!write(data, size)) return false;
    sections_.push_back(std::move(s)); return true;
}
bool BundleWriter::addJson(const std::string& name, const nlohmann::json& j) { const std::string s = j.dump(); return add(name, s.data(), s.size()); }

bool BundleWriter::readBack(const std::string& name, std::vector<uint8_t>& out) const {
    const SectionInfo* s = nullptr;
    for (const SectionInfo& c : sections_) if (c.name == name) s = &c;
    if (!s) return false;
    out.resize(static_cast<size_t>(s->size));
    if (s->size == 0) return true;
    if (mem_) { std::memcpy(out.data(), mem_->data() + s->offset, static_cast<size_t>(s->size)); return true; }
    if (!f_) return false;
    std::fflush(f_);   // the write handle stays open; a second, read handle sees what has been flushed
    std::ifstream in(path_, std::ios::binary); if (!in) return false;
    in.seekg(static_cast<std::streamoff>(s->offset));
    in.read(reinterpret_cast<char*>(out.data()), static_cast<std::streamsize>(s->size));
    return in.gcount() == static_cast<std::streamsize>(s->size);
}

bool BundleWriter::finish(nlohmann::json& manifest, std::string* err) {
    if (finished_ || !isOpen()) { if (err) *err = "bundle writer not open"; return false; }
    if (!pad()) { if (err) *err = "write failed"; return false; }
    manifest["format"] = {{"major", kFormatMajor}, {"minor", kFormatMinor}, {"kind", "rt-level-bundle"}};
    nlohmann::json secs = nlohmann::json::array();
    for (const SectionInfo& s : sections_) secs.push_back({{"name", s.name}, {"offset", s.offset}, {"size", s.size}, {"fnv", hex16(s.fnv)}});
    manifest["sections"] = std::move(secs);
    const uint64_t tocOffset = pos_; const std::string toc = manifest.dump();
    if (!write(toc.data(), toc.size())) { if (err) *err = "write failed"; return false; }
    const std::vector<uint8_t> h = headerBytes(tocOffset, toc.size(), sections_.size());
    if (f_) {
        if (std::fseek(f_, 0, SEEK_SET) != 0 || std::fwrite(h.data(), 1, h.size(), f_) != h.size()) { if (err) *err = "header patch failed"; return false; }
        if (std::fclose(f_) != 0) { f_ = nullptr; if (err) *err = "close failed"; return false; }
        f_ = nullptr;
    } else {
        std::memcpy(mem_->data(), h.data(), h.size());
    }
    finished_ = true; return true;
}

// ---- reader -------------------------------------------------------------------------------------------

Bundle::~Bundle() {
#ifndef _WIN32
    if (map_) munmap(map_, size_);
#endif
}

std::unique_ptr<Bundle> Bundle::open(const std::string& path, std::string* err) {
    std::unique_ptr<Bundle> b(new Bundle()); b->path_ = path;
#ifndef _WIN32
    {
        const int fd = ::open(path.c_str(), O_RDONLY);
        if (fd >= 0) {
            struct stat st{};
            if (fstat(fd, &st) == 0 && st.st_size >= static_cast<off_t>(kHeaderBytes)) {
                void* m = mmap(nullptr, static_cast<size_t>(st.st_size), PROT_READ, MAP_PRIVATE, fd, 0);
                if (m != MAP_FAILED) { b->map_ = m; b->base_ = static_cast<const uint8_t*>(m); b->size_ = static_cast<size_t>(st.st_size); }
            }
            ::close(fd);
        }
    }
#endif
    if (!b->base_) {   // fallback: read the whole file
        FILE* f = std::fopen(path.c_str(), "rb"); if (!f) { if (err) *err = "cannot open " + path; return nullptr; }
        auto mem = std::make_shared<std::vector<uint8_t>>(); std::vector<uint8_t> buf(1 << 20);
        for (;;) { const size_t n = std::fread(buf.data(), 1, buf.size(), f); if (n == 0) break; mem->insert(mem->end(), buf.begin(), buf.begin() + static_cast<std::ptrdiff_t>(n)); }
        std::fclose(f); b->mem_ = mem; b->base_ = mem->data(); b->size_ = mem->size();
    }
    if (!b->parse(err)) return nullptr;
    return b;
}

std::unique_ptr<Bundle> Bundle::fromMemory(std::shared_ptr<std::vector<uint8_t>> bytes, std::string* err) {
    if (!bytes) { if (err) *err = "no bytes"; return nullptr; }
    std::unique_ptr<Bundle> b(new Bundle()); b->path_ = "<memory>"; b->mem_ = std::move(bytes); b->base_ = b->mem_->data(); b->size_ = b->mem_->size();
    if (!b->parse(err)) return nullptr;
    return b;
}

bool Bundle::parse(std::string* err) {
    auto fail = [&](const std::string& why) { if (err) *err = path_ + ": " + why; return false; };
    if (size_ < kHeaderBytes) return fail("too small to be a bundle");
    if (std::memcmp(base_, kMagic, 4) != 0) return fail("not a bundle (magic)");
    uint32_t major = 0, minor = 0, endian = 0, realBytes = 0, reserved = 0; uint64_t tocOffset = 0, tocSize = 0, count = 0;
    BinReader r(base_ + 4, kHeaderBytes - 4);
    r.get(major); r.get(minor); r.get(endian); r.get(realBytes); r.get(reserved); r.get(tocOffset); r.get(tocSize); r.get(count);
    if (!r.ok()) return fail("truncated header");
    if (major != kFormatMajor) return fail("format major " + std::to_string(major) + " (this engine reads " + std::to_string(kFormatMajor) + ")");
    if (endian != kEndianMarker) return fail("written on a machine of the other endianness");
    if (realBytes != sizeof(Real)) return fail("written with Real of " + std::to_string(realBytes) + " bytes (this engine: " + std::to_string(sizeof(Real)) + ")");
    if (tocOffset < kHeaderBytes || tocOffset > size_ || tocSize > size_ - tocOffset) return fail("table of contents out of range");
    try { manifest_ = nlohmann::json::parse(base_ + tocOffset, base_ + tocOffset + tocSize); } catch (const std::exception& ex) { return fail(std::string("bad table of contents: ") + ex.what()); }
    if (!manifest_.contains("sections") || !manifest_["sections"].is_array()) return fail("table of contents has no sections");
    for (const nlohmann::json& s : manifest_["sections"]) {
        SectionInfo si; si.name = s.value("name", std::string()); si.offset = s.value("offset", uint64_t(0)); si.size = s.value("size", uint64_t(0));
        si.fnv = std::strtoull(s.value("fnv", std::string("0")).c_str(), nullptr, 16);
        if (si.offset > size_ || si.size > size_ - si.offset) return fail("section " + si.name + " out of range");
        sections_.push_back(std::move(si));
    }
    std::sort(sections_.begin(), sections_.end(), [](const SectionInfo& a, const SectionInfo& b) { return a.name < b.name; });
    if (count != sections_.size()) return fail("section count disagrees with the table of contents");
    (void)minor; return true;
}

bool Bundle::has(const std::string& name) const { return !section(name).empty(); }

Bundle::View Bundle::section(const std::string& name) const {
    auto it = std::lower_bound(sections_.begin(), sections_.end(), name, [](const SectionInfo& s, const std::string& n) { return s.name < n; });
    if (it == sections_.end() || it->name != name) return {};
    View v; v.data = base_ + it->offset; v.size = static_cast<size_t>(it->size); return v;
}

std::vector<std::string> Bundle::sections(const std::string& prefix) const {
    std::vector<std::string> out;
    for (const SectionInfo& s : sections_) if (s.name.compare(0, prefix.size(), prefix) == 0) out.push_back(s.name);
    return out;
}

nlohmann::json Bundle::json(const std::string& name) const {
    const View v = section(name); if (v.empty()) return nlohmann::json();
    try { return nlohmann::json::parse(v.data, v.data + v.size); } catch (const std::exception&) { return nlohmann::json(); }
}

bool Bundle::checkSection(const std::string& name) const {
    auto it = std::lower_bound(sections_.begin(), sections_.end(), name, [](const SectionInfo& s, const std::string& n) { return s.name < n; });
    if (it == sections_.end() || it->name != name) return false;
    return fnv1a(base_ + it->offset, static_cast<size_t>(it->size)) == it->fnv;
}

}  // namespace bundle
}  // namespace engine
