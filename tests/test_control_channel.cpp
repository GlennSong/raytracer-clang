// The control channel (ADR-0078): the viewer's remote-look seam. These gate
// the protocol grammar, the Manual-mode round trip (the AudioBackendMode
// idiom — deterministic, no socket), and one real unix-socket smoke pass so
// the transport itself is exercised on POSIX hosts.
#include "test_framework.h"

#include "../src/engine/control/control_channel.h"

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
#endif
