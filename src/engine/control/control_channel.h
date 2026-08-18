#ifndef RAYTRACER_ENGINE_CONTROL_CHANNEL_H
#define RAYTRACER_ENGINE_CONTROL_CHANNEL_H

#include <functional>
#include <memory>
#include <string>
#include <vector>

namespace engine {

// The viewer's remote-look seam (ADR-0078): a local line-protocol command
// channel so an external tool (the MCP server in tools/viewer-mcp.py, or nc)
// can frame cameras, take screenshots, toggle overlays, drive the sim clock,
// and reload the level in a RUNNING viewer — the loop that replaces
// "patch settings.json, relaunch, wait, scrape frame 90".
//
// Sealed the ADR-0012 way: no socket type appears in this header; the .cpp
// owns the POSIX details. Read-and-look only by charter — world EDITS stay
// with the EditorBridge, the one write path into the world.
//
// Thread contract (the ADR-0072 staging pattern): the socket thread ONLY
// buffers received lines under a mutex. All effects happen on the main
// thread, inside drain(), once per frame — the handler runs where Settings,
// Renderer, SimClock and the state stack are legally touchable. Replies are
// routed back to the client from the drain.
enum class ControlBackendMode {
    Disabled,   // inert: every call is a no-op (editor_app, web, tests that
                // don't care) — the RT_ENABLE_* stub idiom without the #ifdef
    Socket,     // unix socket at /tmp/raytracer-viewer-<pid>.sock, one client
                // at a time; RT_CONTROL=0 is the viewer's kill-switch
    Manual,     // no socket, no thread: tests stage lines with pushCommand()
                // and read takeReplies() — deterministic, CI-green (the
                // AudioBackendMode::Manual idiom)
};

// One parsed command line: lowercased name + raw args. Pure, so the protocol
// grammar is unit-testable without a channel.
struct ControlCommand {
    std::string name;
    std::vector<std::string> args;
};
ControlCommand parseControlCommand(const std::string& line);

class ControlChannel {
public:
    ControlChannel();
    ~ControlChannel();
    ControlChannel(const ControlChannel&) = delete;
    ControlChannel& operator=(const ControlChannel&) = delete;

    // Start the chosen backend. Socket mode binds and spawns the accept/read
    // thread; returns false if the socket can't be created (and on _WIN32,
    // where Socket mode is a non-goal). Disabled/Manual always succeed.
    bool initialize(ControlBackendMode mode);
    void shutdown();   // stop the thread, close and unlink the socket

    ControlBackendMode mode() const;
    std::string socketPath() const;   // "" unless Socket mode is live

    // MAIN THREAD, once per frame: pop every staged line, run it through
    // `handler` (one line in, one reply line out), and route the reply back
    // (socket client in Socket mode, takeReplies() buffer in Manual).
    // Returns how many commands were handled.
    using Handler = std::function<std::string(const std::string& line)>;
    int drain(const Handler& handler);

    // Manual mode: stage a line as if a client had sent it / collect replies.
    void pushCommand(const std::string& line);
    std::vector<std::string> takeReplies();

private:
    struct Impl;
    std::unique_ptr<Impl> impl;
};

}  // namespace engine

#endif
