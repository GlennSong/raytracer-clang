// The control channel (ADR-0078): the viewer's remote-look seam. These gate
// the protocol grammar, the Manual-mode round trip (the AudioBackendMode
// idiom — deterministic, no socket), and one real unix-socket smoke pass so
// the transport itself is exercised on POSIX hosts.
#include "test_framework.h"

#include "../src/engine/control/control_channel.h"
#include "../src/engine/control/control_commands.h"
#include "../src/engine/application.h"
#include "../src/renderer/settings.h"
#include "../src/engine/audio/audio_engine.h"   // PlaySound
#include "../src/engine/systems/camera_system.h"   // CameraPoseRequest
#include "../src/engine/systems/day_night_system.h"   // DayNightRequest, WeatherRequest
#include "../src/engine/systems/player_system.h"   // PersonRequest
#include "../src/engine/systems/teleport_pose.h"   // TeleportRequest, TeleportPanelRequest
#include "../src/engine/systems/render_system.h"   // LampLightsRequest
#include "../src/engine/control/city_requests.h"   // PossessRequest, CityLayerRequest, JitterRequest

#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

#ifndef _WIN32
#include <sys/socket.h>
#include <sys/un.h>
#include <unistd.h>
#include <chrono>
#include <cstdio>
#include <thread>
#endif

using namespace engine;

TEST_CASE(control_parse_lowercases_name_and_keeps_args) {
    ControlCommand c = parseControlCommand("Camera 10 20.5 -3 -88 0");
    CHECK(c.name == "camera");
    CHECK(c.args.size() == 5u);
    CHECK(c.args[0] == "10");
    CHECK(c.args[1] == "20.5");
    CHECK(c.args[2] == "-3");

    // Args keep their case (paths!); only the command name folds.
    c = parseControlCommand("SHOT /tmp/Shot.PNG");
    CHECK(c.name == "shot");
    CHECK(c.args.size() == 1u);
    CHECK(c.args[0] == "/tmp/Shot.PNG");

    // Empty and whitespace-only lines parse to an empty name, not a crash.
    CHECK(parseControlCommand("").name.empty());
    CHECK(parseControlCommand("   ").name.empty());

    // `camera?` is its own command, not `camera` with a stray token.
    CHECK(parseControlCommand("camera?").name == "camera?");

    // `rest` is the line after the name as sent: whitespace inside it kept (publish's JSON).
    c = parseControlCommand("publish  PlaySound {\"clip\": \"a  b\"}");
    CHECK(c.rest == "PlaySound {\"clip\": \"a  b\"}");
    CHECK(parseControlCommand("ping").rest.empty());
}

TEST_CASE(control_manual_mode_round_trips_through_the_handler) {
    ControlChannel ch;
    CHECK(ch.initialize(ControlBackendMode::Manual));
    CHECK(ch.mode() == ControlBackendMode::Manual);
    CHECK(ch.socketPath().empty());

    ch.pushCommand("ping");
    ch.pushCommand("echo hello world");
    int handled = ch.drain([](const std::string& line) {
        ControlCommand c = parseControlCommand(line);
        if (c.name == "ping") return std::string("ok pong");
        std::string out = "ok";
        for (const std::string& a : c.args) out += " " + a;
        return out;
    });
    CHECK(handled == 2);

    std::vector<std::string> replies = ch.takeReplies();
    CHECK(replies.size() == 2u);
    CHECK(replies[0] == "ok pong");
    CHECK(replies[1] == "ok hello world");
    CHECK(ch.takeReplies().empty());   // taken means taken

    // A drain with nothing staged handles nothing.
    CHECK(ch.drain([](const std::string&) { return std::string("x"); }) == 0);
}

TEST_CASE(control_disabled_mode_is_inert) {
    ControlChannel ch;
    CHECK(ch.initialize(ControlBackendMode::Disabled));
    ch.pushCommand("ping");   // staged but never drained in Disabled
    CHECK(ch.drain([](const std::string&) { return std::string("ok"); }) == 0);
}

#ifndef _WIN32
// The real transport, once: connect, send a line, read the reply. The drain
// runs on this (main) thread the way Application's frame pump would; only the
// accept/read happens on the channel's thread — the ADR-0072 staging split.
TEST_CASE(control_socket_round_trips_a_line) {
    ControlChannel ch;
    CHECK(ch.initialize(ControlBackendMode::Socket));
    const std::string path = ch.socketPath();
    CHECK(!path.empty());

    int fd = ::socket(AF_UNIX, SOCK_STREAM, 0);
    CHECK(fd >= 0);
    sockaddr_un addr{};
    addr.sun_family = AF_UNIX;
    std::snprintf(addr.sun_path, sizeof(addr.sun_path), "%s", path.c_str());
    CHECK(::connect(fd, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) == 0);

    const char* line = "ping\n";
    CHECK(::send(fd, line, 5, 0) == 5);

    // The socket thread stages the line; drain until it lands (bounded).
    int handled = 0;
    for (int i = 0; i < 200 && handled == 0; ++i) {
        handled = ch.drain([](const std::string& l) {
            return parseControlCommand(l).name == "ping"
                       ? std::string("ok pong")
                       : std::string("err");
        });
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    CHECK(handled == 1);

    char buf[64] = {};
    ssize_t n = ::read(fd, buf, sizeof(buf) - 1);
    CHECK(n > 0);
    CHECK(std::string(buf, static_cast<size_t>(n)) == "ok pong\n");

    ::close(fd);
    ch.shutdown();
    // The socket file is unlinked on shutdown — no stale endpoints for the
    // MCP shim's discovery glob to trip over.
    CHECK(::access(path.c_str(), F_OK) != 0);
}

// A watching client that hangs up must not take the process with it (SIGPIPE ends it by default), and lines go out
// whole: two posts arrive as two lines.
TEST_CASE(control_socket_posts_whole_lines_and_survive_a_hangup) {
    ControlChannel ch;
    CHECK(ch.initialize(ControlBackendMode::Socket));
    int fd = ::socket(AF_UNIX, SOCK_STREAM, 0);
    sockaddr_un addr{};
    addr.sun_family = AF_UNIX;
    std::snprintf(addr.sun_path, sizeof(addr.sun_path), "%s", ch.socketPath().c_str());
    CHECK(::connect(fd, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) == 0);
    for (int i = 0; i < 200 && ch.client() == 0; ++i) std::this_thread::sleep_for(std::chrono::milliseconds(5));
    const uint64_t client = ch.client();
    CHECK(client != 0);
    CHECK(ch.post(client, "event A {}"));
    CHECK(ch.post(client, "event B {}"));
    std::string got;
    char buf[64];
    for (int i = 0; i < 100 && got.size() < 22; ++i) {
        const ssize_t n = ::recv(fd, buf, sizeof(buf), MSG_DONTWAIT);
        if (n > 0) got.append(buf, static_cast<std::size_t>(n));
        else std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    CHECK(got == "event A {}\nevent B {}\n");
    ::close(fd);
    for (int i = 0; i < 50; ++i) {   // before and after the socket thread notices: either way, no signal
        ch.post(client, "event C {}");
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
    CHECK(true);   // still here
    ch.shutdown();
}
#endif

// The command table (ADR-0253): one handler per verb. A second registration of a name is refused -- the
// if-chain it replaced let `ride` be handled twice, and the second handler never ran.
TEST_CASE(control_commands_refuse_a_second_handler_for_one_name) {
    ControlCommands t;
    CHECK(t.add("ride", "ride bus|off", "the player's seat", [](const ControlCommand&) { return std::string("ok first"); }));
    CHECK(!t.add("ride", "ride <x> <z>", "the agent's bus", [](const ControlCommand&) { return std::string("ok second"); },
                 "a script"));
    CHECK(t.refused() == 1);
    CHECK(t.size() == 1);
    CHECK(t.run(parseControlCommand("ride bus")) == "ok first");
}

TEST_CASE(control_commands_run_pass_args_and_answer_unknown_names) {
    ControlCommands t;
    t.add("echo", "echo <words...>", "says them back", [](const ControlCommand& c) {
        std::string out = "ok";
        for (const std::string& a : c.args) out += " " + a;
        return out;
    });
    CHECK(t.run(parseControlCommand("ECHO a b")) == "ok a b");
    CHECK(t.run(parseControlCommand("nope 1")).rfind("err unknown command: nope", 0) == 0);
}

TEST_CASE(control_commands_list_sorted_and_remove_by_owner) {
    ControlCommands t;
    auto ok = [](const ControlCommand&) { return std::string("ok"); };
    t.add("zeta", "zeta", "last", ok);
    t.add("alpha", "alpha <n>", "first", ok, "scene.lua");
    t.add("mid?", "mid?", "a question", ok, "scene.lua");
    const std::vector<ControlCommandInfo> l = t.list();
    CHECK(l.size() == 3 && l[0].name == "alpha" && l[2].name == "zeta");
    CHECK(l[0].owner == "scene.lua" && l[2].owner == "engine");
    CHECK(t.listingText() == "ok 3 commands: alpha <n>; mid?; zeta;");
    CHECK(t.listingJson().find("\"summary\":\"first\"") != std::string::npos);
    CHECK(t.removeOwnedBy("scene.lua") == 2);
    CHECK(t.size() == 1 && t.has("zeta") && !t.has("alpha"));
    CHECK(t.remove("zeta") && !t.remove("zeta"));
}

// The viewer's own verbs: every one registers once (none refused), each says its usage and what it does
// (that listing is what the MCP server turns into tools), and the front door still answers.
TEST_CASE(control_application_registers_every_verb_once_with_its_words) {
    Application app;
    const ControlCommands& t = app.commands();
    CHECK(t.refused() == 0);
    CHECK(t.size() >= 80);
    for (const ControlCommandInfo& c : t.list()) {
        CHECK(!c.summary.empty());
        CHECK(c.usage.rfind(c.name, 0) == 0);   // the usage line starts with the verb
    }
    // Only verbs that need no renderer: the Application here is not initialized.
    CHECK(app.commands().run(parseControlCommand("ping")) == "ok pong");
    CHECK(app.commands().run(parseControlCommand("commands?")).rfind("ok ", 0) == 0);
    CHECK(app.commands().run(parseControlCommand("bogus")).rfind("err unknown command: bogus", 0) == 0);
    // `ride <x> <z>` goes to the possessed agent with anything after the two numbers, as it did before the merge
    std::string possess;
    EventBus::Subscription sub = app.events().listen<PossessRequest>([&](const PossessRequest& r) { possess = r.command; });
    CHECK(app.commands().run(parseControlCommand("ride 120 -340 0")) == "ok staged (poll possess?)");
    CHECK(possess == "ride 120 -340");
    // ...and the player's ride to the transit system; with none listening, the verb says so
    CHECK(app.commands().run(parseControlCommand("ride bus")).rfind("err ride: nothing here", 0) == 0);
    int ride = -1;
    EventBus::Subscription rides = app.events().listen<RideRequest>([&](const RideRequest& r) { ride = r.mode; });
    CHECK(app.commands().run(parseControlCommand("ride bus")) == "ok ride bus staged");
    CHECK(ride == 1);
}

// Names are stored lowercased, as they arrive; a name that is not one word is refused.
TEST_CASE(control_commands_store_names_as_they_arrive) {
    ControlCommands t;
    CHECK(t.add("PossessStats?", "possessstats?", "a question", [](const ControlCommand&) { return std::string("ok"); }));
    CHECK(t.run(parseControlCommand("possessStats?")) == "ok");
    CHECK(!t.add("two words", "two words", "no", [](const ControlCommand&) { return std::string("ok"); }));
    CHECK(!t.add("", "", "no", [](const ControlCommand&) { return std::string("ok"); }));
    CHECK(t.refused() == 2 && t.size() == 1);
}

// A handler may remove verbs, its own included (a Lua script reloading), while it runs.
TEST_CASE(control_commands_a_handler_can_remove_itself) {
    ControlCommands t;
    std::string tail = " still here";   // captured state that dies with the stored std::function
    t.add("reload_me", "reload_me", "drops its own verbs", [&t, tail](const ControlCommand&) {
        t.removeOwnedBy("scene.lua");
        return "ok" + tail;
    }, "scene.lua");
    CHECK(t.run(parseControlCommand("reload_me")) == "ok still here");
    CHECK(!t.has("reload_me"));
}

// The site's list of verbs (web/engine/commands.html) is built from web/engine/commands.json, which must be the
// table itself. RT_WRITE_COMMANDS_JSON=1 rewrites it; then run tools/commands_page.py.
TEST_CASE(control_site_command_list_matches_the_table) {
    Application app;
    const std::string want = app.commands().listingJson(1) + "\n";
    const std::string path = std::string(RT_SOURCE_DIR) + "/web/engine/commands.json";
    if (std::getenv("RT_WRITE_COMMANDS_JSON")) {
        std::ofstream(path) << want;
        return;
    }
    std::ifstream in(path);
    std::stringstream have;
    have << in.rdbuf();
    if (have.str() != want)
        std::printf("    web/engine/commands.json is out of date: RT_WRITE_COMMANDS_JSON=1 "
                    "RT_TEST_FILTER=control_site ./build-viewer/run_tests, then python3 tools/commands_page.py\n");
    CHECK(have.str() == want);
}

// Watched events (ADR-0253) go out as lines nobody asked for. Manual mode is always client 1 and collects them
// with the replies; Disabled refuses.
TEST_CASE(control_post_reaches_the_client_and_disabled_refuses) {
    ControlChannel manual;
    CHECK(manual.initialize(ControlBackendMode::Manual));
    CHECK(manual.client() == 1);
    CHECK(manual.post(1, "event Hop {}"));
    const std::vector<std::string> replies = manual.takeReplies();
    CHECK(replies.size() == 1 && replies[0] == "event Hop {}");
    ControlChannel off;
    CHECK(off.initialize(ControlBackendMode::Disabled));
    CHECK(off.client() == 0);
    CHECK(!off.post(1, "event Hop {}"));
}

// The bus's named types from the socket: events? lists PlaySound with its fields, publish delivers one, and
// watch needs a connection (this Application has none).
TEST_CASE(control_events_verbs_list_and_publish) {
    Application app;
    const std::string list = app.commands().run(parseControlCommand("events?"));
    CHECK(list.rfind("ok ", 0) == 0);
    CHECK(list.find("PlaySound {") != std::string::npos);
    CHECK(list.find("clip") != std::string::npos);
    std::string clip;
    app.events().subscribe<PlaySound>([&](const PlaySound& s) { clip = s.clip; });
    const std::string r = app.commands().run(parseControlCommand(R"(publish PlaySound {"clip": "door", "volume": 0.5})"));
    CHECK(r == "ok published PlaySound to 1 subscribers");
    CHECK(clip == "door");
    CHECK(app.commands().run(parseControlCommand("publish PlaySound {\"clip\": \"door  slam\"}")).rfind("ok", 0) == 0);
    CHECK(clip == "door  slam");   // the two spaces survive
    CHECK(app.commands().run(parseControlCommand("publish PlaySound {nope")).rfind("err", 0) == 0);
    CHECK(app.commands().run(parseControlCommand("publish Nope")).rfind("err unknown event", 0) == 0);
    CHECK(app.commands().run(parseControlCommand("watch PlaySound")).rfind("err watch:", 0) == 0);
    CHECK(app.commands().run(parseControlCommand("unwatch")) == "ok stopped 0 watches");
}

// Command verbs publish typed requests (ADR-0253): with nobody listening they say so; with a listener it gets the
// parsed request. A question is the engine's to list and a system's to answer: unbound it says who would.
TEST_CASE(control_request_verbs_publish_typed_events) {
    Application app;
    CHECK(app.commands().run(parseControlCommand("teleport 1 2 3")).rfind("err teleport: nothing here", 0) == 0);
    TeleportRequest got;
    int n = 0;
    EventBus::Subscription sub = app.events().listen<TeleportRequest>([&](const TeleportRequest& r) { got = r; ++n; });
    CHECK(app.commands().run(parseControlCommand("teleport 120 -340")) == "ok teleport staged (poll `teleport?`)");
    CHECK(n == 1 && got.pose.groundY && got.pose.position.x == 120 && got.pose.position.z == -340);
    CHECK(app.commands().run(parseControlCommand("teleport nowhere")).rfind("err unreadable pose", 0) == 0);
    CHECK(n == 1);
    sub.reset();
    CHECK(app.events().subscriberCount<TeleportRequest>() == 0);

    CHECK(app.commands().run(parseControlCommand("teleport?")).rfind("err teleport?: nothing answers it here", 0) == 0);
    {
        ControlCommands::Binding b = app.commands().bind("teleport?", "test", [](const ControlCommand&) {
            return std::string("ok here");
        });
        CHECK(static_cast<bool>(b));
        CHECK(app.commands().boundBy("teleport?") == "test");
        CHECK(!app.commands().bind("teleport?", "second", [](const ControlCommand&) { return std::string(); }));
        CHECK(app.commands().run(parseControlCommand("teleport?")) == "ok here");
    }
    CHECK(app.commands().boundBy("teleport?").empty());
    CHECK(app.commands().run(parseControlCommand("teleport?")).rfind("err", 0) == 0);

    CameraPoseRequest pose;
    EventBus::Subscription cam = app.events().listen<CameraPoseRequest>([&](const CameraPoseRequest& r) { pose = r; });
    CHECK(app.commands().run(parseControlCommand("camera 1 2 3 -10 90")) == "ok camera staged");
    CHECK(pose.eye.y == 2 && pose.pitch == -10 && pose.yaw == 90);
    int person = -1;
    EventBus::Subscription per = app.events().listen<PersonRequest>([&](const PersonRequest& r) { person = r.mode; });
    CHECK(app.commands().run(parseControlCommand("person third")) == "ok person staged");
    CHECK(person == 1);
}

// The day/night verbs: each form of `daynight` fills only its field of one DayNightRequest; `weather` a
// WeatherRequest with its snap.
TEST_CASE(control_daynight_and_weather_verbs_publish_one_field_each) {
    Application app;
    std::vector<DayNightRequest> got;
    EventBus::Subscription day = app.events().listen<DayNightRequest>([&](const DayNightRequest& r) { got.push_back(r); });
    CHECK(app.commands().run(parseControlCommand("daynight 18.5")) == "ok time staged");
    CHECK(app.commands().run(parseControlCommand("daynight hold")) == "ok cycle held");
    CHECK(app.commands().run(parseControlCommand("daynight moon auto")) == "ok moon follows the calendar");
    CHECK(app.commands().run(parseControlCommand("daynight 25")).rfind("err usage", 0) == 0);
    CHECK(got.size() == 3);
    if (got.size() == 3) {
        CHECK(got[0].hour == 18.5 && got[0].hold < 0 && !got[0].moonAuto);
        CHECK(got[1].hold == 1 && got[1].hour < 0);
        CHECK(got[2].moonAuto && got[2].moon < 0);
    }
    WeatherRequest w;
    EventBus::Subscription weather = app.events().listen<WeatherRequest>([&](const WeatherRequest& r) { w = r; });
    CHECK(app.commands().run(parseControlCommand("weather storm now")) == "ok weather set");
    CHECK(w.state == "storm" && w.now);
    CHECK(app.commands().run(parseControlCommand("daynight?")).rfind("err daynight?: nothing answers", 0) == 0);
}

// The last Settings one-shots as requests (ADR-0253): overlay layers and lamp lights, clouds, the teleport panel,
// jitter. Each says when nothing takes it, and fills only what it names.
TEST_CASE(control_overlay_clouds_jitter_publish_requests) {
    Application app;
    CHECK(app.commands().run(parseControlCommand("overlay plan on")).rfind("err overlay plan: nothing here", 0) == 0);
    CHECK(app.commands().run(parseControlCommand("overlay lamplights off")).rfind("err overlay lamplights:", 0) == 0);
    std::vector<CityLayerRequest> layers;
    EventBus::Subscription l = app.events().listen<CityLayerRequest>([&](const CityLayerRequest& r) { layers.push_back(r); });
    bool lamps = true;
    EventBus::Subscription lamp = app.events().listen<LampLightsRequest>([&](const LampLightsRequest& r) { lamps = r.on; });
    CHECK(app.commands().run(parseControlCommand("overlay plan on")) == "ok plan");
    CHECK(app.commands().run(parseControlCommand("overlay cones off")) == "ok cones");
    CHECK(app.commands().run(parseControlCommand("overlay lamplights off")) == "ok lamplights");
    CHECK(layers.size() == 2 && layers[0].layer == "plan" && layers[0].on && layers[1].layer == "cones" && !layers[1].on);
    CHECK(!lamps);

    std::vector<CloudsRequest> clouds;
    EventBus::Subscription c = app.events().listen<CloudsRequest>([&](const CloudsRequest& r) { clouds.push_back(r); });
    CHECK(app.commands().run(parseControlCommand("clouds coverage 0.3 wind 12")) == "ok clouds");
    CHECK(app.commands().run(parseControlCommand("clouds off")) == "ok clouds");
    CHECK(app.commands().run(parseControlCommand("clouds coverage")).rfind("err usage", 0) == 0);
    CHECK(app.commands().run(parseControlCommand("clouds fluffy 1")).rfind("err usage", 0) == 0);
    CHECK(clouds.size() == 2);
    if (clouds.size() == 2) {
        CHECK(clouds[0].coverage == 0.3 && clouds[0].wind == 12 && clouds[0].density < 0 && clouds[0].enabled < 0);
        CHECK(clouds[1].enabled == 0 && clouds[1].coverage < 0);
    }

    int panels = 0;
    EventBus::Subscription p = app.events().listen<TeleportPanelRequest>([&](const TeleportPanelRequest&) { ++panels; });
    CHECK(app.commands().run(parseControlCommand("teleport open")) == "ok teleport panel");
    CHECK(panels == 1);

    std::vector<JitterRequest> jit;
    EventBus::Subscription j = app.events().listen<JitterRequest>([&](const JitterRequest& r) { jit.push_back(r); });
    CHECK(app.commands().run(parseControlCommand("jitter")).rfind("ok recording", 0) == 0);
    CHECK(app.commands().run(parseControlCommand("jitter 42 120")).rfind("ok recording", 0) == 0);
    CHECK(app.commands().run(parseControlCommand("jitter bus")).rfind("err usage", 0) == 0);
    CHECK(app.commands().run(parseControlCommand("jitter 1 5")).rfind("err usage", 0) == 0);
    CHECK(jit.size() == 2 && jit[0].agent == -1 && jit[0].frames == 300 && jit[1].agent == 42 && jit[1].frames == 120);
    CHECK(app.commands().run(parseControlCommand("jitter?")).rfind("err jitter?: nothing answers", 0) == 0);
}

// A system's answers (ControlAnswers): the question replies with the latest text while the system holds it, knows
// it was asked, and falls back to the engine's default when the system goes.
TEST_CASE(control_answers_reply_latest_and_release) {
    ControlCommands table;
    table.add("count?", "count?", "How many.", [](const ControlCommand&) { return std::string("err nobody"); });
    {
        ControlAnswers answers;
        answers.bind(&table, "Counter", "count?", "none yet");
        CHECK(answers.secondsSinceAsked() > 1e6);
        CHECK(table.run(parseControlCommand("count?")) == "ok none yet");
        CHECK(answers.secondsSinceAsked() < 5.0);
        answers.set("count?", "7");
        CHECK(table.run(parseControlCommand("count?")) == "ok 7");
        CHECK(answers.get("count?") == "7");
        answers.bind(&table, "Counter", "count?");   // a state entered twice: no second binding
        CHECK(table.run(parseControlCommand("count?")) == "ok 7");
    }
    CHECK(table.run(parseControlCommand("count?")) == "err nobody");
}

// Settings holds preferences only (ADR-0253): a one-shot key works in memory but is never saved, and a key the bus
// retired is dropped when an old file loads.
TEST_CASE(control_settings_save_preferences_only) {
    const char* path = "test_settings_preferences_tmp.json";
    struct Cleanup { const char* p; ~Cleanup() { std::remove(p); } } cleanup{path};
    {
        std::ofstream old(path);
        old << R"({"windowWidth": 1600, "possess.cmd": "release", "teleport.request": "0 0", "traffic.telemetry": "x"})";
    }
    Settings s;
    s.load(path);
    CHECK(s.getDouble("windowWidth", 0) == 1600);
    CHECK(s.getString("teleport.request", "none") == "none");   // a stale request does not fire again
    CHECK(s.getString("possess.cmd", "none") == "none");
    s.setBool("lamps.lights", false);   // retired: a request now
    s.save(path);
    std::ifstream in(path);
    std::stringstream text;
    text << in.rdbuf();
    CHECK(text.str().find("windowWidth") != std::string::npos);
    CHECK(text.str().find("lamps.lights") == std::string::npos);
    CHECK(text.str().find("telemetry") == std::string::npos);
    CHECK(!Settings::persists("clouds.apply") && Settings::persists("clouds.coverage"));
    CHECK(!Settings::persists("playFromHere") && !Settings::persists("citysim.plan"));
}
