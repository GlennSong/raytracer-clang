#ifndef RAYTRACER_ENGINE_BUNDLE_BINARY_STREAM_H
#define RAYTRACER_ENGINE_BUNDLE_BINARY_STREAM_H

// Little-endian POD streams for bundle sections (ADR-0084). A writer appends to a byte vector; a reader
// walks a byte view with bounds checks, so a truncated or foreign section reads as ok() == false and never
// as undefined behaviour. Every section starts with a four-character magic and a version so codecs can
// refuse what they do not understand (the per-section layer of the compatibility rule).

#include <cstdint>
#include <cstring>
#include <string>
#include <type_traits>
#include <vector>

namespace engine {
namespace bundle {

class BinWriter {
public:
    std::vector<uint8_t> bytes;

    void magic(const char tag[4], uint32_t version) { append(tag, 4); put<uint32_t>(version); }
    template <class T> void put(const T& v) {
        static_assert(std::is_trivially_copyable<T>::value, "BinWriter::put needs a trivially copyable type");
        const size_t o = bytes.size(); bytes.resize(o + sizeof(T)); std::memcpy(bytes.data() + o, &v, sizeof(T));
    }
    void putStr(const std::string& s) { put<uint32_t>(static_cast<uint32_t>(s.size())); append(s.data(), s.size()); }
    template <class T> void putVec(const std::vector<T>& v) {
        static_assert(std::is_trivially_copyable<T>::value, "BinWriter::putVec needs a trivially copyable element");
        put<uint64_t>(static_cast<uint64_t>(v.size())); append(v.data(), v.size() * sizeof(T));
    }
    void append(const void* p, size_t n) { if (n == 0) return; const size_t o = bytes.size(); bytes.resize(o + n); std::memcpy(bytes.data() + o, p, n); }
};

class BinReader {
public:
    BinReader(const uint8_t* data, size_t size) : d_(data), n_(size) {}
    bool ok() const { return ok_; }
    size_t remaining() const { return n_ - pos_; }
    // Consumes the magic + version; false (and ok() false) when the tag differs.
    bool magic(const char tag[4], uint32_t* version) {
        if (!ok_ || remaining() < 8) return fail();
        if (std::memcmp(d_ + pos_, tag, 4) != 0) return fail();
        pos_ += 4; uint32_t v = 0; if (!get(v)) return false; if (version) *version = v; return true;
    }
    template <class T> bool get(T& v) {
        static_assert(std::is_trivially_copyable<T>::value, "BinReader::get needs a trivially copyable type");
        if (!ok_ || remaining() < sizeof(T)) return fail();
        std::memcpy(&v, d_ + pos_, sizeof(T)); pos_ += sizeof(T); return true;
    }
    bool getStr(std::string& s) {
        uint32_t n = 0; if (!get(n)) return false; if (remaining() < n) return fail();
        s.assign(reinterpret_cast<const char*>(d_ + pos_), n); pos_ += n; return true;
    }
    template <class T> bool getVec(std::vector<T>& v) {
        static_assert(std::is_trivially_copyable<T>::value, "BinReader::getVec needs a trivially copyable element");
        uint64_t n = 0; if (!get(n)) return false; const uint64_t bytesNeeded = n * sizeof(T);
        if (bytesNeeded / sizeof(T) != n || remaining() < bytesNeeded) return fail();
        v.resize(static_cast<size_t>(n)); if (n) std::memcpy(v.data(), d_ + pos_, static_cast<size_t>(bytesNeeded)); pos_ += static_cast<size_t>(bytesNeeded); return true;
    }

private:
    bool fail() { ok_ = false; return false; }
    const uint8_t* d_; size_t n_; size_t pos_ = 0; bool ok_ = true;
};

}  // namespace bundle
}  // namespace engine

#endif
