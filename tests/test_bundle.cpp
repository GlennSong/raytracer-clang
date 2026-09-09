// engine::bundle (ADR-0084): the container, the codecs, keys, and a bake with copy-forward — all without
// lanelab or a level of consequence, so the facility stays a general one.
#include "test_framework.h"

#include "../src/engine/bundle/bake.h"
#include "../src/engine/bundle/codecs.h"
#include "../src/engine/mesh_builder.h"

#include <cstdio>
#include <cstring>
#include <filesystem>
#include <thread>
#include <fstream>

using namespace engine;
using namespace engine::bundle;
namespace fs = std::filesystem;

namespace {
std::string tempDir(const char* tag) {
    const fs::path d = fs::temp_directory_path() / (std::string("rt_bundle_") + tag + "_" + std::to_string(static_cast<long long>(std::hash<std::string>()(tag) % 100000)));
    std::error_code ec; fs::remove_all(d, ec); fs::create_directories(d, ec); return d.string();
}
bool sameFloats(const std::vector<float>& a, const std::vector<float>& b) { return a.size() == b.size() && (a.empty() || std::memcmp(a.data(), b.data(), a.size() * sizeof(float)) == 0); }
}  // namespace

TEST_CASE(bundle_round_trips_sections_through_a_file_and_finds_them_by_prefix) {
    const std::string dir = tempDir("file"); const std::string path = dir + "/level.bundle";
    std::vector<uint32_t> big(100000); for (size_t i = 0; i < big.size(); ++i) big[i] = static_cast<uint32_t>(i * 2654435761u);
    {
        BundleWriter w; std::string err; CHECK(w.openFile(path, &err));
        CHECK(w.add("city/e0/ground", "abc", 3));
        CHECK(w.addJson("city/e0/report", nlohmann::json{{"ok", true}, {"n", 3}}));
        CHECK(w.add("city/e0/cell/1_-2/mesh/asphalt", big.data(), big.size() * sizeof(uint32_t)));
        CHECK(w.add("lots/plan", "", 0));
        nlohmann::json manifest{{"kind", "test"}, {"key", "0"}}; CHECK(w.finish(manifest, &err)); CHECK(manifest.contains("sections") && manifest["sections"].size() == 4);
        CHECK(w.sections().size() == 4);
    }
    std::string err; std::unique_ptr<Bundle> b = Bundle::open(path, &err);
    CHECK(b != nullptr); if (!b) { std::printf("    %s\n", err.c_str()); return; }
    CHECK(b->manifest().value("kind", std::string()) == "test");
    CHECK(b->manifest()["format"]["major"].get<uint32_t>() == kFormatMajor);
    const Bundle::View g = b->section("city/e0/ground"); CHECK(g.size == 3 && std::memcmp(g.data, "abc", 3) == 0);
    CHECK(b->json("city/e0/report").value("n", 0) == 3);
    const Bundle::View m = b->section("city/e0/cell/1_-2/mesh/asphalt"); CHECK(m.size == big.size() * sizeof(uint32_t));
    CHECK(m.size == big.size() * sizeof(uint32_t) && std::memcmp(m.data, big.data(), m.size) == 0);
    CHECK(reinterpret_cast<uintptr_t>(m.data) % 64 == reinterpret_cast<uintptr_t>(b->section("city/e0/ground").data) % 64);   // sections stay 64-aligned relative to the base
    CHECK(b->has("lots/plan") && b->section("lots/plan").size == 0 && !b->has("lots/nope"));
    CHECK(b->sections("city/e0/cell/").size() == 1 && b->sections("city/").size() == 3 && b->sections("").size() == 4);
    CHECK(b->checkSection("city/e0/cell/1_-2/mesh/asphalt"));
    CHECK(b->json("missing").is_null());
    std::error_code ec; fs::remove_all(dir, ec);
}

TEST_CASE(bundle_round_trips_through_memory_and_rejects_a_foreign_header) {
    BundleWriter w; w.openMemory(); CHECK(w.add("a/b", "xyz", 3)); std::string err; nlohmann::json manifest = nlohmann::json::object(); CHECK(w.finish(manifest, &err));
    std::shared_ptr<std::vector<uint8_t>> bytes = w.memory(); CHECK(bytes && bytes->size() > kHeaderBytes);
    std::unique_ptr<Bundle> b = Bundle::fromMemory(bytes, &err); CHECK(b != nullptr);
    if (b) { const Bundle::View v = b->section("a/b"); CHECK(v.size == 3 && std::memcmp(v.data, "xyz", 3) == 0); CHECK(!b->mapped()); }
    auto bad = std::make_shared<std::vector<uint8_t>>(*bytes); (*bad)[0] = 'X';
    CHECK(Bundle::fromMemory(bad, &err) == nullptr); CHECK(err.find("magic") != std::string::npos);
    auto newer = std::make_shared<std::vector<uint8_t>>(*bytes); uint32_t major = kFormatMajor + 1; std::memcpy(newer->data() + 4, &major, 4);
    CHECK(Bundle::fromMemory(newer, &err) == nullptr); CHECK(err.find("format major") != std::string::npos);
    auto truncated = std::make_shared<std::vector<uint8_t>>(bytes->begin(), bytes->begin() + 40);
    CHECK(Bundle::fromMemory(truncated, &err) == nullptr);
}

TEST_CASE(bundle_binary_reader_refuses_truncation_and_foreign_magic) {
    BinWriter w; w.magic("TEST", 3); w.putStr("hello"); std::vector<double> d{1.5, 2.5}; w.putVec(d);
    { BinReader r(w.bytes.data(), w.bytes.size()); uint32_t v = 0; std::string s; std::vector<double> e; CHECK(r.magic("TEST", &v) && v == 3 && r.getStr(s) && s == "hello" && r.getVec(e) && e == d && r.ok() && r.remaining() == 0); }
    { BinReader r(w.bytes.data(), w.bytes.size()); uint32_t v = 0; CHECK(!r.magic("NOPE", &v) && !r.ok()); }
    { BinReader r(w.bytes.data(), w.bytes.size() - 8); uint32_t v = 0; std::string s; std::vector<double> e; CHECK(r.magic("TEST", &v) && r.getStr(s)); CHECK(!r.getVec(e) && !r.ok()); }
}

TEST_CASE(packed_mesh_is_exact_under_float32_and_detects_a_varying_vertex_colour) {
    RenderMesh box = MeshBuilder::box(Vec3(2, 1, 3));
    for (Vertex& v : box.vertices) v.color = Vec3(0.2, 0.3, 0.4);
    PackedMesh p = packMesh(box, "asphalt", Vec3(0.1, 0.1, 0.1), PackedMesh::kCollidable, 0.93f, 0.85);
    CHECK(p.colorUniform() && p.collidable() && !p.paint() && p.col.empty() && p.vertexCount() == box.vertices.size() && p.idx == box.indices);
    CHECK_APPROX(p.vertexColor[1], 0.3, 1e-6);
    RenderMesh back = unpackMesh(p); CHECK(back.vertices.size() == box.vertices.size() && back.indices == box.indices);
    for (size_t i = 0; i < box.vertices.size(); ++i) {
        CHECK_APPROX(back.vertices[i].position.x, static_cast<float>(box.vertices[i].position.x), 0.0);
        CHECK_APPROX(back.vertices[i].normal.y, static_cast<float>(box.vertices[i].normal.y), 0.0);
        CHECK_APPROX(back.vertices[i].tangent.z, static_cast<float>(box.vertices[i].tangent.z), 0.0);
        CHECK(back.vertices[i].u == box.vertices[i].u && back.vertices[i].v == box.vertices[i].v);
    }
    PackedMesh again = packMesh(back, "asphalt", Vec3(0.1, 0.1, 0.1), PackedMesh::kCollidable, 0.93f, 0.85);   // pack ∘ unpack is a fixed point
    CHECK(sameFloats(again.pos, p.pos) && sameFloats(again.nrm, p.nrm) && sameFloats(again.tan, p.tan) && sameFloats(again.uv, p.uv) && again.idx == p.idx && again.flags == p.flags);
    MeshCollider mc; colliderFromPacked(p, mc); CHECK(mc.vertices.size() == p.vertexCount() && mc.indices == p.idx && mc.friction == 0.85);
    CHECK_APPROX(mc.vertices[3].x, p.pos[9], 0.0);
    // a pier box appended into a concrete mesh carries the default colour: no longer uniform
    RenderMesh concrete = box; RenderMesh pier = MeshBuilder::box(Vec3(2.6, 4.8, 2.6)); MeshBuilder::append(concrete, pier);
    PackedMesh q = packMesh(concrete, "concrete", Vec3(0.5, 0.5, 0.5), PackedMesh::kCollidable, 0.93f, 0.85);
    CHECK(!q.colorUniform() && q.col.size() == 3 * q.vertexCount());
    RenderMesh qb = unpackMesh(q); CHECK_APPROX(qb.vertices.back().color.x, 1.0, 1e-6); CHECK_APPROX(qb.vertices.front().color.x, 0.2, 1e-6);
    BinWriter w; putPackedMesh(w, q); PackedMesh r; BinReader rd(w.bytes.data(), w.bytes.size()); CHECK(getPackedMesh(rd, r));
    CHECK(r.name == "concrete" && r.flags == q.flags && sameFloats(r.pos, q.pos) && sameFloats(r.col, q.col) && r.idx == q.idx && r.friction == q.friction);
    BinReader bad(w.bytes.data(), w.bytes.size() / 2); PackedMesh r2; CHECK(!getPackedMesh(bad, r2));
}

TEST_CASE(height_grid_road_entity_and_rings_round_trip_field_for_field) {
    HeightGridBlob g; g.x0 = -10; g.y0 = 5; g.res = 2.5; g.nx = 3; g.ny = 2; g.z = {1, 2, 3, 4, 5, 6.25};
    BinWriter w; putHeightGrid(w, g); HeightGridBlob h; BinReader r(w.bytes.data(), w.bytes.size()); CHECK(getHeightGrid(r, h));
    CHECK(h.x0 == g.x0 && h.y0 == g.y0 && h.res == g.res && h.nx == 3 && h.ny == 2 && h.z == g.z);

    RoadEntity e; e.graph.nodes.resize(3);
    e.graph.nodes[0].pos = Vec2(0, 0); e.graph.nodes[1].pos = Vec2(100, 0); e.graph.nodes[1].elev = 8.5; e.graph.nodes[1].elevAbsolute = true; e.graph.nodes[2].pos = Vec2(100, 80); e.graph.nodes[2].tangent = Vec2(0.6, 0.8);
    RoadEdge a; a.a = 0; a.b = 1; a.width = 12; a.klass = RoadClass::Freeway; a.layer = 1; a.provenance = RoadProvenance::CorridorMain; a.oneWay = true; a.walkable = false; a.access = 7; a.baked = true; a.parkOffset = 1.5; a.parkWidth = 2.5;
    RoadEdge b; b.a = 1; b.b = 2; b.klass = RoadClass::Ramp; b.provenance = RoadProvenance::CorridorRamp; b.spec = 2;
    e.graph.edges = {a, b};
    e.look.sidewalk = 2.6; e.look.markings = false; e.look.color = Vec3(0.1, 0.2, 0.3);
    e.plan.freewayPlans = {{Vec2(1, 2), Vec2(3, 4), Vec2(5, 6)}}; CityHub hub; hub.pos = Vec2(9, 9); hub.kind = 2; hub.radial = true; hub.site = 1; e.plan.cityHubs = {hub};
    BinWriter w2; putRoadEntity(w2, e); RoadEntity f; BinReader r2(w2.bytes.data(), w2.bytes.size()); CHECK(getRoadEntity(r2, f));
    CHECK(f.graph.nodes.size() == 3 && f.graph.edges.size() == 2 && f.graph.nodes[1].elev == 8.5 && f.graph.nodes[1].elevAbsolute && f.graph.nodes[2].tangent.y == 0.8);
    CHECK(f.graph.edges[0].klass == RoadClass::Freeway && f.graph.edges[0].oneWay && !f.graph.edges[0].walkable && f.graph.edges[0].access == 7 && f.graph.edges[0].baked && f.graph.edges[0].parkWidth == 2.5 && f.graph.edges[0].provenance == RoadProvenance::CorridorMain);
    CHECK(f.graph.edges[1].klass == RoadClass::Ramp && f.graph.edges[1].spec == 2 && f.graph.edges[1].provenance == RoadProvenance::CorridorRamp);
    CHECK(f.look.sidewalk == 2.6 && !f.look.markings && f.look.crosswalks && f.look.color.z == 0.3);
    CHECK(f.plan.freewayPlans.size() == 1 && f.plan.freewayPlans[0].size() == 3 && f.plan.freewayPlans[0][2].y == 6 && f.plan.cityHubs.size() == 1 && f.plan.cityHubs[0].radial && f.plan.cityHubs[0].site == 1);
    RoadGraph onlyGraph; BinWriter w3; putRoadGraph(w3, e.graph); BinReader r3(w3.bytes.data(), w3.bytes.size()); CHECK(getRoadGraph(r3, onlyGraph) && onlyGraph.edges.size() == 2);

    std::vector<std::vector<Vec2>> rings = {{Vec2(0, 0), Vec2(1, 0), Vec2(1, 1)}, {}};
    BinWriter w4; putRings(w4, rings); std::vector<std::vector<Vec2>> back; BinReader r4(w4.bytes.data(), w4.bytes.size()); CHECK(getRings(r4, back) && back.size() == 2 && back[0].size() == 3 && back[0][2].y == 1 && back[1].empty());
}

TEST_CASE(bundle_keys_are_deterministic_and_order_free) {
    CHECK(fnv1aStr("abc") == fnv1aStr("abc") && fnv1aStr("abc") != fnv1aStr("abd"));
    CHECK(hex16(255) == "00000000000000ff");
    const uint64_t k1 = combinedKey({{"city", 1}, {"lots", 2}}), k2 = combinedKey({{"lots", 2}, {"city", 1}}), k3 = combinedKey({{"city", 1}, {"lots", 3}});
    CHECK(k1 == k2 && k1 != k3);
    setBundleRoot("/tmp/rt_bundle_override"); CHECK(bundleRoot() == "/tmp/rt_bundle_override"); setBundleRoot(""); CHECK(!bundleRoot().empty());
    CHECK(!engineIdentity().version.empty() && engineIdentity().realBytes == static_cast<int>(sizeof(Real)));
    CHECK(isoNowUtc().size() == 20);
}

namespace {
// Two toy producers keyed on different level fields, so a change to one copies the other forward.
struct FieldProducer : BundleProducer {
    std::string nm, field; int* produced;
    FieldProducer(std::string n, std::string f, int* counter) : nm(std::move(n)), field(std::move(f)), produced(counter) {}
    std::string name() const override { return nm; }
    bool applies(const LevelInputs& in) const override { return in.level.contains(field); }
    ProducerIdentity identity(const LevelInputs& in) const override { ProducerIdentity id; id.tag = "test-1"; id.key = fnv1aStr(nm + ":" + id.tag + ":" + in.level[field].dump()); return id; }
    ProducerReport produce(const LevelInputs& in, BundleWriter& out, const ProgressFn* progress) const override {
        ++*produced; const std::string payload = in.level[field].dump(); out.add(nm + "/payload", payload.data(), payload.size()); out.addJson(nm + "/cells", nlohmann::json::array());
        if (progress) { Progress p; p.stage = "work"; p.fraction = 0.5; (*progress)(p); }
        ProducerReport r; r.timings["work"] = 0.01; r.report = {{"payload", payload}}; return r;
    }
};
int g_producedA = 0, g_producedB = 0;
}  // namespace

TEST_CASE(bake_level_builds_once_reports_up_to_date_and_copies_unchanged_producers_forward) {
    registerProducer(std::make_unique<FieldProducer>("tA", "a", &g_producedA));
    registerProducer(std::make_unique<FieldProducer>("tB", "b", &g_producedB));
    const std::string root = tempDir("bake"); const std::string level = root + "/level.json";
    { std::ofstream f(level); f << R"({"version":1,"a":{"x":1},"b":{"y":2},"entities":[]})"; }
    BakeRequest req; req.levelPath = level; req.outRoot = root + "/out"; req.only = {"tA", "tB"};
    std::vector<double> fractions; ProgressFn fn = [&](const Progress& p) { fractions.push_back(p.fraction); return true; };
    BakeReport r1 = bakeLevel(req, &fn);
    CHECK(r1.ok); if (!r1.ok) std::printf("    %s\n", r1.error.c_str());
    CHECK(r1.built.size() == 2 && r1.reused.empty() && g_producedA == 1 && g_producedB == 1 && !r1.upToDate);
    CHECK(fractions.size() == 2 && fractions[0] > 0.2 && fractions[0] < 0.3 && fractions[1] > 0.7 && fractions[1] < 0.8);   // 0.5 of the first, 0.5 of the second, equal weights
    CHECK(fs::exists(r1.dir + "/" + kBundleFile) && fs::exists(r1.dir + "/" + kManifestFile));
    {
        std::string err; std::unique_ptr<Bundle> b = Bundle::open(r1.dir + "/" + kBundleFile, &err); CHECK(b != nullptr);
        if (b) { CHECK(b->has("tA/payload") && b->has("tB/payload")); CHECK(b->manifest()["producers"].size() == 2); CHECK(b->manifest()["engine"]["real"].get<int>() == static_cast<int>(sizeof(Real))); CHECK(!manifestProducer(b->manifest(), "tA").is_null()); }
    }
    BakeReport r2 = bakeLevel(req, nullptr); CHECK(r2.ok && r2.upToDate && r2.reused.size() == 2 && g_producedA == 1);
    { std::ofstream f(level); f << R"({"version":1,"a":{"x":1},"b":{"y":3},"entities":[]})"; }   // only b changed
    BakeReport r3 = bakeLevel(req, nullptr);
    CHECK(r3.ok && r3.dir != r1.dir && r3.built == std::vector<std::string>{"tB"} && r3.reused == std::vector<std::string>{"tA"} && g_producedA == 1 && g_producedB == 2);
    {
        std::string err; std::unique_ptr<Bundle> b = Bundle::open(r3.dir + "/" + kBundleFile, &err); CHECK(b != nullptr);
        if (b) { const Bundle::View v = b->section("tA/payload"); CHECK(v.size == 7 && std::memcmp(v.data, R"({"x":1})", 7) == 0); CHECK(manifestProducer(b->manifest(), "tA").contains("copiedFrom")); }
    }
    req.force = true; BakeReport r4 = bakeLevel(req, nullptr); CHECK(r4.ok && r4.built.size() == 2 && g_producedA == 2);
    req.force = false; req.toMemory = true; BakeReport r5 = bakeLevel(req, nullptr); CHECK(r5.ok && r5.memory && r5.built.size() == 2 && g_producedA == 3);
    { std::string err; std::unique_ptr<Bundle> b = Bundle::fromMemory(r5.memory, &err); CHECK(b && b->has("tB/payload")); }
    // the loader's entry point: a hit after the disk bake, a miss baked in memory under RT_NOCACHE
    setBundleRoot(root + "/out");
    LevelInputs in; std::string err; CHECK(loadLevelInputs(level, in, &err));
    Obtained o = obtainForLevel(in, "tA"); CHECK(o.bundle && o.hit && !o.built && o.bundle->has("tA/payload"));
    CHECK(lastBundleStatus().rfind("hit", 0) == 0);
    setenv("RT_NOCACHE", "1", 1); Obtained n = obtainForLevel(in, "tA"); unsetenv("RT_NOCACHE");
    CHECK(n.bundle && n.built && !n.hit && n.bundle->path() == "<memory>" && g_producedA == 4);
    setenv("RT_BUNDLE_REQUIRE", "1", 1); { std::ofstream f(level); f << R"({"version":1,"a":{"x":9},"b":{"y":3},"entities":[]})"; } LevelInputs in2; CHECK(loadLevelInputs(level, in2, &err));
    Obtained m = obtainForLevel(in2, "tA"); unsetenv("RT_BUNDLE_REQUIRE"); CHECK(!m.bundle && m.status.find("RT_BUNDLE_REQUIRE") != std::string::npos);
    setBundleRoot("");
    std::error_code ec; fs::remove_all(root, ec);
}

TEST_CASE(prune_keeps_the_newest_bundle_per_level_and_whatever_the_caller_resolved_as_current) {
    registerProducer(std::make_unique<FieldProducer>("tA", "a", &g_producedA));
    registerProducer(std::make_unique<FieldProducer>("tB", "b", &g_producedB));
    const std::string root = tempDir("prune"); const std::string level = root + "/level.json"; const std::string out = root + "/out";
    BakeRequest req; req.levelPath = level; req.outRoot = out; req.only = {"tA", "tB"};
    { std::ofstream f(level); f << R"({"version":1,"a":{"x":1},"b":{"y":2},"entities":[]})"; }
    const BakeReport r1 = bakeLevel(req, nullptr); CHECK(r1.ok);
    std::this_thread::sleep_for(std::chrono::milliseconds(1100));   // "created" has second resolution
    { std::ofstream f(level); f << R"({"version":1,"a":{"x":2},"b":{"y":2},"entities":[]})"; }
    const BakeReport r2 = bakeLevel(req, nullptr); CHECK(r2.ok && r2.dir != r1.dir);
    std::error_code ec;
    fs::create_directories(out + "/deadbeefdeadbeef", ec);                                // an aborted write: no manifest
    fs::create_directories(out + "/0123456789abcdef.tmp-42", ec);                         // a bake in flight: never touched
    { std::ofstream f(r1.dir + "/city.glb"); f << "glb"; }                                // an export beside the old bundle
    LevelInputs in; std::string err; CHECK(loadLevelInputs(level, in, &err));
    CHECK(fs::equivalent(bundleDirForLevel(in, out), r2.dir));
    auto entry = [](const PruneReport& rep, const std::string& dir) -> const PruneEntry* { for (const PruneEntry& e : rep.entries) if (fs::equivalent(e.dir, dir)) return &e; return nullptr; };
    // dry run: the older bundle and the manifest-less directory are stale, the newest keeps, the tmp dir is not listed
    const PruneReport dry = pruneBundles(out, {}, false);
    CHECK(dry.entries.size() == 3 && dry.removed == 0 && dry.staleBytes > 0);
    const PruneEntry* e1 = entry(dry, r1.dir); const PruneEntry* e2 = entry(dry, r2.dir); const PruneEntry* ed = entry(dry, out + "/deadbeefdeadbeef");
    CHECK(e1 && e1->stale && !e1->current && e1->extras == std::vector<std::string>{"city.glb"} && e1->level == fs::absolute(level).lexically_normal().string());
    CHECK(e2 && !e2->stale && e2->extras.empty()); CHECK(ed && ed->stale && ed->level == "(no manifest)");
    CHECK(fs::exists(r1.dir + "/" + kBundleFile) && fs::exists(out + "/deadbeefdeadbeef"));
    // a directory the caller resolved as current is kept even when it is not the newest
    const PruneReport kept = pruneBundles(out, {r1.dir}, false);
    const PruneEntry* k1 = entry(kept, r1.dir); CHECK(k1 && !k1->stale && k1->current);
    size_t stale = 0; for (const PruneEntry& e : kept.entries) stale += e.stale; CHECK(stale == 1);
    // apply
    const PruneReport done = pruneBundles(out, {}, true);
    CHECK(done.removed == 2 && !fs::exists(r1.dir) && !fs::exists(out + "/deadbeefdeadbeef") && fs::exists(r2.dir + "/" + kBundleFile) && fs::exists(out + "/0123456789abcdef.tmp-42"));
    fs::remove_all(root, ec);
}
