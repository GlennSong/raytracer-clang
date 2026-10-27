#include "control_channel.h"

#include "../../log.h"

#include <algorithm>
#include <atomic>
#include <cctype>
#include <cstdio>
#include <mutex>
#include <sstream>
#include <thread>

#ifndef _WIN32
#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/un.h>
#include <cerrno>
#include <poll.h>
#include <unistd.h>
#endif

namespace engine {
namespace {
#ifndef _WIN32
// A client that hangs up between our checks must not take the viewer with it: a send to a closed socket raises
// SIGPIPE, whose default is to end the process. Linux says so per send; macOS per socket (SO_NOSIGPIPE at accept).
#ifdef MSG_NOSIGNAL
constexpr int kSendFlags = MSG_NOSIGNAL | MSG_DONTWAIT;
#else
constexpr int kSendFlags = MSG_DONTWAIT;
#endif
// Watched events waiting for a client that is not reading are dropped, whole lines, past this much.
constexpr std::size_t kOutboxCap = 256 * 1024;
#endif
}  // namespace

ControlCommand parseControlCommand(const std::string& line) {
    ControlCommand cmd;
    std::istringstream in(line);
    std::string tok;
    while (in >> tok) {
        if (cmd.name.empty()) {
            std::transform(tok.begin(), tok.end(), tok.begin(),
                           [](unsigned char c) { return std::tolower(c); });
            cmd.name = tok;
            const std::streampos after = in.tellg();
            if (after != std::streampos(-1)) {
                const std::size_t from = line.find_first_not_of(" \t", static_cast<std::size_t>(after));
                if (from != std::string::npos) cmd.rest = line.substr(from);
            }
        } else {
            cmd.args.push_back(tok);
        }
    }
    return cmd;
}

struct ControlChannel::Impl {
    ControlBackendMode mode = ControlBackendMode::Disabled;
    std::string path;

    // Staging (ADR-0072): the socket thread appends, drain() swaps out on the
    // main thread. Replies for Manual mode collect here too.
    std::mutex mutex;
    std::vector<std::string> pending;
    std::vector<std::string> manualReplies;
    uint64_t clientId = 0;      // guarded by `mutex`: bumped on every accept, 0 while nobody is connected
    uint64_t nextClientId = 1;
    uint64_t dropped = 0;       // posts a full outbox refused
    // What the client has yet to read, guarded by `mutex`: replies and watched events, whole lines in order. Sent
    // without blocking as the socket takes it, so a slow reader never stalls a frame and never gets half a line.
    std::string outbox;
#ifndef _WIN32
    void flushLocked() {
        while (!outbox.empty() && clientFd >= 0) {
            const ssize_t n = ::send(clientFd, outbox.data(), outbox.size(), kSendFlags);
            if (n > 0) { outbox.erase(0, static_cast<std::size_t>(n)); continue; }
            if (n < 0 && (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR)) return;   // later
            outbox.clear();   // the client is gone (EPIPE, ECONNRESET): the socket thread drops it
            return;
        }
    }
#endif

#ifndef _WIN32
    int listenFd = -1;
    int clientFd = -1;        // guarded by `mutex` (drain writes replies to it)
    std::thread thread;
    std::atomic<bool> running{false};

    void threadMain() {
        // One client at a time, NEWEST WINS: the listen fd stays polled even
        // while a client is connected, so a hung client can't lock out the
        // next one. poll() with a timeout keeps shutdown responsive.
        std::string buffer;
        while (running.load(std::memory_order_relaxed)) {
            struct pollfd pfds[2]{};
            pfds[0].fd = listenFd;
            pfds[0].events = POLLIN;
            int nfds = 1;
            int client;
            {
                std::lock_guard<std::mutex> lock(mutex);
                client = clientFd;
            }
            if (client >= 0) {
                pfds[1].fd = client;
                pfds[1].events = POLLIN;
                nfds = 2;
            }
            if (::poll(pfds, static_cast<nfds_t>(nfds), 200) <= 0)
                continue;   // timeout or EINTR

            if (pfds[0].revents & POLLIN) {
                int fd = ::accept(listenFd, nullptr, nullptr);
                if (fd >= 0) {
                    std::lock_guard<std::mutex> lock(mutex);
                    if (clientFd >= 0) ::close(clientFd);   // newest wins
#ifdef SO_NOSIGPIPE
                    const int one = 1;
                    (void)::setsockopt(fd, SOL_SOCKET, SO_NOSIGPIPE, &one, sizeof(one));
#endif
                    clientFd = fd;
                    clientId = nextClientId++;
                    outbox.clear();   // the last client's unread lines are not this one's
                    buffer.clear();
                    continue;   // re-poll with the new client
                }
            }
            if (nfds < 2 || !(pfds[1].revents & (POLLIN | POLLHUP))) continue;

            char chunk[512];
            ssize_t n = ::read(client, chunk, sizeof(chunk));
            if (n <= 0) {   // EOF or error: drop the client, back to accept
                std::lock_guard<std::mutex> lock(mutex);
                if (clientFd == client) {
                    ::close(clientFd);
                    clientFd = -1;
                    clientId = 0;
                    outbox.clear();
                }
                continue;
            }
            buffer.append(chunk, static_cast<size_t>(n));
            size_t nl;
            while ((nl = buffer.find('\n')) != std::string::npos) {
                std::string line = buffer.substr(0, nl);
                buffer.erase(0, nl + 1);
                if (!line.empty() && line.back() == '\r') line.pop_back();
                if (line.empty()) continue;
                std::lock_guard<std::mutex> lock(mutex);
                pending.push_back(std::move(line));
            }
        }
    }
#endif
};

ControlChannel::ControlChannel() : impl(std::make_unique<Impl>()) {}

ControlChannel::~ControlChannel() { shutdown(); }

bool ControlChannel::initialize(ControlBackendMode mode) {
    shutdown();
    impl->mode = mode;
    if (mode != ControlBackendMode::Socket) return true;
#ifdef _WIN32
    impl->mode = ControlBackendMode::Disabled;   // non-goal (ADR-0078)
    return false;
#else
    impl->path = "/tmp/raytracer-viewer-" + std::to_string(::getpid()) + ".sock";
    ::unlink(impl->path.c_str());   // a stale file from a crashed twin pid

    impl->listenFd = ::socket(AF_UNIX, SOCK_STREAM, 0);
    if (impl->listenFd < 0) return false;
    sockaddr_un addr{};
    addr.sun_family = AF_UNIX;
    if (impl->path.size() >= sizeof(addr.sun_path)) {
        ::close(impl->listenFd);
        impl->listenFd = -1;
        return false;
    }
    std::snprintf(addr.sun_path, sizeof(addr.sun_path), "%s",
                  impl->path.c_str());
    if (::bind(impl->listenFd, reinterpret_cast<sockaddr*>(&addr),
               sizeof(addr)) != 0 ||
        ::listen(impl->listenFd, 1) != 0) {
        ::close(impl->listenFd);
        impl->listenFd = -1;
        return false;
    }
    ::chmod(impl->path.c_str(), 0600);   // user-only, like a terminal

    impl->running = true;
    impl->thread = std::thread([this] { impl->threadMain(); });
    LOG_INFO << "[control] listening at " << impl->path << " (ADR-0078)";
    return true;
#endif
}

void ControlChannel::shutdown() {
#ifndef _WIN32
    if (impl->running) {
        impl->running = false;
        impl->thread.join();
    }
    if (impl->clientFd >= 0) { ::close(impl->clientFd); impl->clientFd = -1; }
    if (impl->listenFd >= 0) { ::close(impl->listenFd); impl->listenFd = -1; }
    if (!impl->path.empty()) { ::unlink(impl->path.c_str()); impl->path.clear(); }
#endif
    impl->mode = ControlBackendMode::Disabled;
    impl->pending.clear();
    impl->manualReplies.clear();
}

ControlBackendMode ControlChannel::mode() const { return impl->mode; }

std::string ControlChannel::socketPath() const { return impl->path; }

int ControlChannel::drain(const Handler& handler) {
    if (impl->mode == ControlBackendMode::Disabled) return 0;
    std::vector<std::string> lines;
    {
        std::lock_guard<std::mutex> lock(impl->mutex);
        lines.swap(impl->pending);
#ifndef _WIN32
        impl->flushLocked();   // what the client could not take last frame
#endif
    }
    for (const std::string& line : lines) {
        std::string reply = handler(line);
        if (impl->mode == ControlBackendMode::Manual) {
            impl->manualReplies.push_back(std::move(reply));
            continue;
        }
#ifndef _WIN32
        reply.push_back('\n');
        std::lock_guard<std::mutex> lock(impl->mutex);
        if (impl->clientFd < 0) continue;
        impl->outbox += reply;   // a reply always goes, if late: the client asked for it
        impl->flushLocked();
#endif
    }
    return static_cast<int>(lines.size());
}

uint64_t ControlChannel::client() const {
    if (impl->mode == ControlBackendMode::Manual) return 1;
    std::lock_guard<std::mutex> lock(impl->mutex);
    return impl->clientId;
}

bool ControlChannel::post(uint64_t client, const std::string& line) {
    if (impl->mode == ControlBackendMode::Disabled || client == 0) return false;
    std::lock_guard<std::mutex> lock(impl->mutex);
    if (impl->mode == ControlBackendMode::Manual) {
        impl->manualReplies.push_back(line);
        return true;
    }
#ifndef _WIN32
    if (impl->clientFd < 0 || impl->clientId != client) return false;
    if (impl->outbox.size() > kOutboxCap) {   // a client that stopped reading: drop the line, never stall
        ++impl->dropped;
        return false;
    }
    impl->outbox += line;
    impl->outbox += '\n';
    impl->flushLocked();
    return true;
#else
    return false;
#endif
}

uint64_t ControlChannel::droppedPosts() const {
    std::lock_guard<std::mutex> lock(impl->mutex);
    return impl->dropped;
}

void ControlChannel::pushCommand(const std::string& line) {
    std::lock_guard<std::mutex> lock(impl->mutex);
    impl->pending.push_back(line);
}

std::vector<std::string> ControlChannel::takeReplies() {
    std::lock_guard<std::mutex> lock(impl->mutex);
    std::vector<std::string> out;
    out.swap(impl->manualReplies);
    return out;
}

}  // namespace engine
