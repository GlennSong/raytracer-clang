#include "webexport.h"

#include <cstdint>
#include <cstdio>
#include <fstream>
#include <vector>

namespace roadlab {

namespace {

void putFloats(std::vector<char>& out, const float* v, size_t n) {
    const char* p = reinterpret_cast<const char*>(v);
    out.insert(out.end(), p, p + n * sizeof(float));
}

}  // namespace

bool writeWebExport(const Scene& sc, const std::string& prefix, WebExportStats& stats,
                    std::string& error) {
    // 2 m is the mesher's ring spacing (tessellate.h TessParams), and the atlas
    // rows have to line up with the rings the vertices sit on or the row a
    // fragment interpolates is not the row its geometry came from.
    PaintAtlas atlas = bakePaintAtlas(sc.net, 2.0);

    std::vector<char> bin;
    bin.reserve(sc.mesh.verts.size() * kWebVertexFloats * sizeof(float));

    Vec3 lo{1e30, 1e30, 1e30}, hi{-1e30, -1e30, -1e30};
    for (const Vertex& v : sc.mesh.verts) {
        // Roads and junction pads get a real atlas row; everything else — the
        // ground, structures, props — gets -1, which the shader reads as "no
        // paint here" and skips the fetch entirely.
        double row = -1.0;
        if (v.road >= 0 && size_t(v.road) < atlas.profiles.size() &&
            (v.material == MatKind::RoadSurface || v.material == MatKind::JunctionPad))
            row = atlas.profiles[size_t(v.road)].rowCoord(v.s);

        // Pavement carries no vertex colour: roadlab's rasteriser shades asphalt
        // procedurally per pixel (surface.cpp), so RoadSurface and JunctionPad
        // vertices keep Vertex's 0.5 grey default. Exporting that puts white
        // paint on a white road, and a junction pad — which has no atlas row at
        // all, the bake being per-road — would stay a bright grey slab in the
        // middle of an otherwise dark intersection.
        //
        // So the export supplies the albedo the mesh never stored: the same
        // asphalt with its aggregate and crack noise at their means. The grain
        // itself is the surface shader's job, not this file's.
        Vec3 albedo = v.color;
        if (v.material == MatKind::RoadSurface || v.material == MatKind::JunctionPad)
            albedo = {0.082, 0.083, 0.087};

        float f[kWebVertexFloats] = {
            float(v.pos.x),   float(v.pos.y),    float(v.pos.z),
            float(v.normal.x), float(v.normal.y), float(v.normal.z),
            float(v.s),       float(v.t),        float(row),
            float(albedo.x),  float(albedo.y),   float(albedo.z),
        };
        putFloats(bin, f, kWebVertexFloats);

        lo.x = std::min(lo.x, v.pos.x); hi.x = std::max(hi.x, v.pos.x);
        lo.y = std::min(lo.y, v.pos.y); hi.y = std::max(hi.y, v.pos.y);
        lo.z = std::min(lo.z, v.pos.z); hi.z = std::max(hi.z, v.pos.z);
    }
    size_t idxOffset = bin.size();
    {
        const char* p = reinterpret_cast<const char*>(sc.mesh.indices.data());
        bin.insert(bin.end(), p, p + sc.mesh.indices.size() * sizeof(uint32_t));
    }
    size_t profOffset = bin.size();
    {
        // The tiled form, not the logical one: a texture cannot be 19786 tall.
        std::vector<float> img = atlas.profileImage();
        putFloats(bin, img.data(), img.size());
    }
    size_t styleOffset = bin.size();
    putFloats(bin, atlas.styleTex.data(), atlas.styleTex.size());

    std::ofstream binFile(prefix + ".bin", std::ios::binary);
    if (!binFile) {
        error = "cannot write " + prefix + ".bin";
        return false;
    }
    binFile.write(bin.data(), std::streamsize(bin.size()));

    std::ofstream manifest(prefix + ".json");
    if (!manifest) {
        error = "cannot write " + prefix + ".json";
        return false;
    }
    char buf[2048];
    std::snprintf(
        buf, sizeof buf,
        "{\n"
        "  \"scene\": \"%s\",\n"
        "  \"vertexFloats\": %d,\n"
        "  \"vertexCount\": %zu,\n"
        "  \"indexCount\": %zu,\n"
        "  \"vertexOffset\": 0,\n"
        "  \"indexOffset\": %zu,\n"
        "  \"profileOffset\": %zu,\n"
        "  \"profileTexWidth\": %d,\n"
        "  \"profileTexHeight\": %d,\n"
        "  \"profileRows\": %d,\n"
        "  \"rowsPerTile\": %d,\n"
        "  \"styleOffset\": %zu,\n"
        "  \"styleWidth\": %d,\n"
        "  \"styleRows\": %d,\n"
        "  \"maxBounds\": %d,\n"
        "  \"bounds\": { \"lo\": [%.3f, %.3f, %.3f], \"hi\": [%.3f, %.3f, %.3f] }\n"
        "}\n",
        sc.name.c_str(), kWebVertexFloats, sc.mesh.verts.size(), sc.mesh.indices.size(),
        idxOffset, profOffset, atlas.imageWidth(), atlas.imageHeight(), atlas.rows,
        kRowsPerTile, styleOffset, atlas.styleWidth(), atlas.styleRows, RL_MAX_BOUNDS,
        lo.x, lo.y, lo.z, hi.x, hi.y, hi.z);
    manifest << buf;

    // A sampler fixture beside the data.
    //
    // The 12870-case corpus proves rlEvaluateMarkings agrees CPU-to-GPU, but it
    // is HANDED its boundaries. Nothing has ever checked rlFetchBoundaries — the
    // half that reads them out of the two textures, with a linear filter on one
    // axis, a nearest index on the other, and padding slots that must not be
    // confused with unpainted ones. That is the half most likely to be subtly
    // wrong, so it gets its own answers to compare against.
    {
        std::ofstream sam(prefix + ".samples.json");
        if (!sam) {
            error = "cannot write " + prefix + ".samples.json";
            return false;
        }
        sam << "{\n  \"cases\": [\n";
        bool first = true;
        size_t seamCases = 0;
        for (size_t ri = 0; ri < atlas.profiles.size(); ++ri) {
            const PaintProfile& prof = atlas.profiles[ri];
            int n = prof.rows();
            if (n < 2) continue;

            std::vector<double> locals;
            for (int k = 0; k < 12; ++k)
                // Fractional rows on purpose: between rings is where the linear
                // filter actually does something, and where a wrong axis hides.
                locals.push_back((double(n) - 1.0) * double(k) / 11.0);

            // And every place the style row CHANGES, sampled from both sides of
            // the step and dead centre. That is the one spot where the two axes
            // disagree about what they are doing — offsets blending, index
            // snapping — so it is where a fetch that filtered the index, or
            // sampled the wrong texel, stops looking like rounding.
            for (int i = 0; i + 1 < n; ++i) {
                int a = int(prof.profile[size_t(i) * kProfileTexelsPerRow * 4 + 12]);
                int b = int(prof.profile[size_t(i + 1) * kProfileTexelsPerRow * 4 + 12]);
                if (a == b) continue;
                for (double f : {0.25, 0.5, 0.75}) locals.push_back(double(i) + f);
                seamCases += 3;
            }

            for (double local : locals) {
                // Sample at the row the GPU will actually see. The vertex
                // channel is an f32, so rounding here rather than in the reader
                // keeps the comparison about the fetch instead of about a
                // double the shader never had.
                float row = float(double(prof.rowBase) + local);
                RlBoundary got[RL_MAX_BOUNDS];
                int count = atlas.sample(int(ri), double(row), got, RL_MAX_BOUNDS);
                if (!first) sam << ",\n";
                first = false;
                // %.9g round-trips an f32 exactly; %g's default six digits does
                // not, and the residual it leaves looks exactly like a sampler
                // bug — 5e-5 on an offset of -14.4 m.
                std::snprintf(buf, sizeof buf, "    {\"row\": %.9g, \"n\": %d, \"b\": [",
                              double(row), count);
                sam << buf;
                for (int b = 0; b < count; ++b) {
                    std::snprintf(buf, sizeof buf,
                                  "%s[%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g]",
                                  b ? "," : "", double(got[b].t), double(got[b].style),
                                  double(got[b].width), double(got[b].gap),
                                  double(got[b].dashOn), double(got[b].dashOff),
                                  double(got[b].wear), double(got[b].color));
                    sam << buf;
                }
                sam << "]}";
                ++stats.sampleCases;
            }
        }
        sam << "\n  ]\n}\n";
        stats.seamCases = seamCases;
    }

    stats.vertices = sc.mesh.verts.size();
    stats.indices = sc.mesh.indices.size();
    stats.profileRows = atlas.rows;
    stats.styleRows = atlas.styleRows;
    stats.binBytes = bin.size();
    return true;
}

}  // namespace roadlab
